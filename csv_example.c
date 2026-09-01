/******************************************************************************
 * csv_example.c - Cyclic Synchronous Velocity (CiA 402 mode 9) reference for
 *                 EtherCAT servo drives (MDX+ series).
 *
 * Demonstrates a complete CSV bring-up with the open-source SOEM 2.x master:
 *   - Distributed Clocks, Sync0 = 500 us, standard activation
 *   - Required startup SDO parameters (limit objects)
 *   - RxPDO remapped to Controlword + Modes of operation + Target velocity
 *   - Phase-locked real-time cyclic loop (drive DC clock reference)
 *   - Velocity profile: ramp to cruise, hold, ramp to zero (target velocity
 *     is held at 0 until the drive is enabled)
 *
 * Platform: Linux with PREEMPT_RT kernel recommended. Build:
 *   gcc csv_example.c -o csv_example \
 *       -I <SOEM>/include -I <SOEM>/build/include -I <SOEM>/osal \
 *       -I <SOEM>/osal/linux -I <SOEM>/oshw/linux \
 *       -L <SOEM>/build -lsoem -lpthread -lrt
 *
 * Usage:  sudo ./csv_example <network-interface>
 *
 * SAFETY: This program ROTATES THE MOTOR at the configured velocity.
 * Ensure the shaft is free or safely loaded (continuous rotation - not
 * suitable for travel-limited axes without adjusting the profile), keep an
 * emergency stop within reach. Ctrl+C ramps to zero and performs a
 * controlled shutdown. Verify TARGET_VEL units against your drive's
 * velocity scaling before running loaded.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>
#include <time.h>
#include "soem/soem.h"

/* ---- Configuration ------------------------------------------------------ */
#define CYCLE_NS          500000   /* cyclic period and Sync0 cycle (500 us) */
#define NSEC_PER_SEC      1000000000
#define SYNC0_MARGIN_US   2000     /* delay between Sync0 activation and the
                                      SAFE-OP request                        */
#define MODE_CSV          9        /* CiA 402 mode of operation              */

/* Velocity profile (units: counts/s, as used by object 0x60FF; verify the
   scaling in the drive documentation - some drives use user velocity units).
   Example: 100,000 counts/s on a 20-bit encoder (1,048,576 counts/rev)
   is ~0.095 motor rev/s.                                                    */
#define TARGET_VEL        100000   /* cruise velocity (counts/s)             */
#define VEL_RAMP          500      /* velocity increment per cycle           */
#define HOLD_CYCLES       6000     /* cycles at cruise (6000 x 500 us = 3 s) */

static ecx_contextt ctx;
static uint8 IOmap[4096];
static volatile int run = 1;
static void on_sigint(int s){ (void)s; run = 0; }

/* Shared between the cyclic thread and main() */
static volatile uint16_t g_statusword = 0;
static volatile uint16_t g_errcode    = 0;
static volatile int32_t  g_position   = 0;
static volatile int32_t  g_targetvel  = 0;
static volatile int8_t   g_modedisp   = 0;
static volatile int      g_enabled    = 0;
static volatile int      g_inOP       = 0;
static volatile int      g_done       = 0;
static volatile int64_t  g_toff       = 0;

/* Microsecond timestamp since program start (for stage timing output) */
static struct timespec tz;
static int64_t now_us(void){
   struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
   return (int64_t)(t.tv_sec - tz.tv_sec)*1000000 + (t.tv_nsec - tz.tv_nsec)/1000;
}
#define STAGE(...) do{ printf("[%8lld us] ", (long long)now_us()); \
                       printf(__VA_ARGS__); printf("\n"); }while(0)

/* Process data images. pack(1): layout must match the wire format exactly. */
#pragma pack(push,1)
typedef struct {                    /* RxPDO 0x1600 after remap (7 bytes)   */
   uint16_t controlword;            /* 0x6040 */
   int8_t   modes_of_operation;     /* 0x6060 */
   int32_t  target_velocity;        /* 0x60FF, velocity units (counts/s)    */
} out_pdo_t;
typedef struct {                    /* TxPDO 0x1A00 factory default, leading
                                       fields of the 35-byte image          */
   uint16_t error_code;             /* 0x603F */
   uint16_t statusword;             /* 0x6041 */
   int8_t   modes_display;          /* 0x6061 */
   int32_t  position_actual;        /* 0x6064 */
   int32_t  follow_error;           /* 0x60F4 */
} in_pdo_t;
#pragma pack(pop)

static out_pdo_t *out;
static in_pdo_t  *in;

/* ---- Distributed-clock phase lock --------------------------------------- */
static int64 toff = 0;

static void add_time_ns(ec_timet *ts, int64 addtime){
   ec_timet a; a.tv_nsec = addtime % NSEC_PER_SEC;
   a.tv_sec  = (addtime - a.tv_nsec)/NSEC_PER_SEC;
   osal_timespecadd(ts,&a,ts);
}

/* PI controller: trims the loop period so frame transmission stays locked
   to the drive's DC clock. */
static void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime){
   static int64 integral = 0;
   int64 delta = reftime % cycletime;
   if (delta > (cycletime/2)) delta -= cycletime;
   integral += delta;
   *offsettime = -(delta/100) - (integral/20000);
}

static int sdo_w(uint16 slave, uint16 idx, uint8 sub,
                 int size, void *val, const char *what){
   int wkc = ecx_SDOwrite(&ctx, slave, idx, sub, FALSE, size, val, EC_TIMEOUTRXM);
   printf("    %-24s 0x%04X:%02X wkc=%d %s\n", what, idx, sub, wkc, wkc>0?"":"[FAIL]");
   return wkc;
}

/* PO2SO hook: SOEM invokes this during the PRE-OP -> SAFE-OP transition,
   when the mailbox is active and PDO remapping is permitted. */
static int drive_setup(ecx_contextt *c, uint16 slave){
   (void)c; uint8 u8; uint16 u16; uint32 u32;
   STAGE("[PO2SO] configuring slave %u", slave);

   /* Startup parameters. These limit objects default to 0 on the drive and
      must be set for motion to be produced (the position loop's torque and
      velocity outputs are clamped by them). */
   u16=1000;    sdo_w(slave,0x6072,0,2,&u16,"6072 MaxTorque");
   u16=1000;    sdo_w(slave,0x6073,0,2,&u16,"6073 MaxCurrent");
   u16=3000;    sdo_w(slave,0x60E0,0,2,&u16,"60E0 PosTorqueLim");
   u16=3000;    sdo_w(slave,0x60E1,0,2,&u16,"60E1 NegTorqueLim");
   u32=1000000; sdo_w(slave,0x607F,0,4,&u32,"607F MaxVelocity");

   /* RxPDO remap: 0x1600 = Controlword + Modes + Target velocity.
      Sequence per CiA 402: disable assignment, clear, write entries
      (index<<16 | sub<<8 | bitlen), set count, re-assign, enable. */
   u8=0;            sdo_w(slave,0x1C12,0,1,&u8, "1C12 clear");
   u8=0;            sdo_w(slave,0x1600,0,1,&u8, "1600 clear");
   u32=0x60400010;  sdo_w(slave,0x1600,1,4,&u32,"map Controlword");
   u32=0x60600008;  sdo_w(slave,0x1600,2,4,&u32,"map Modes");
   u32=0x60FF0020;  sdo_w(slave,0x1600,3,4,&u32,"map TargetVelocity");
   u8=3;            sdo_w(slave,0x1600,0,1,&u8, "1600 count=3");
   u16=0x1600;      sdo_w(slave,0x1C12,1,2,&u16,"1C12 assign 1600");
   u8=1;            sdo_w(slave,0x1C12,0,1,&u8, "1C12 count=1");
   /* TxPDO 0x1A00 is left at the factory default. */

   { int8_t m=MODE_CSV; sdo_w(slave,0x6060,0,1,&m,"6060 mode=CSV"); }
   return 1;
}

/* ---- Velocity profile (runs inside the cyclic thread) --------------------
   Ramp to TARGET_VEL, hold for HOLD_CYCLES, ramp back to zero. Generated
   cycle-synchronously so the streamed velocity is smooth.                  */
static int32_t vel_step(void){
   static int32_t vel = 0;
   static int     phase = 0;        /* 0: ramp up, 1: hold, 2: ramp down    */
   static int     hold = 0;

   if (phase == 0){
      vel += VEL_RAMP;
      if (vel >= TARGET_VEL){ vel = TARGET_VEL; phase = 1; }
   } else if (phase == 1){
      if (++hold >= HOLD_CYCLES) phase = 2;
   } else {
      vel -= VEL_RAMP;
      if (vel <= 0){ vel = 0; g_done = 1; }
   }
   return vel;
}

/* ---- Real-time cyclic thread --------------------------------------------
   Every cycle: sleep to the phase-corrected boundary, exchange process
   data, update the phase lock, run the CiA 402 state machine, stream the
   velocity setpoint. */
OSAL_THREAD_FUNC_RT ecat_rt_thread(void *arg){
   (void)arg;
   ec_timet ts; osal_get_monotonic_time(&ts);
   ts.tv_nsec = ((ts.tv_nsec / CYCLE_NS) + 1) * CYCLE_NS;
   ecx_send_processdata(&ctx);

   while (run){
      add_time_ns(&ts, CYCLE_NS + toff);
      osal_monotonic_sleep(&ts);
      ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      if (ctx.slavelist[0].hasdc)
         ec_sync(ctx.DCtime, CYCLE_NS, &toff);
      g_toff = toff;

      uint16_t sw = in->statusword;
      g_statusword = sw; g_errcode = in->error_code;
      g_position = in->position_actual; g_modedisp = in->modes_display;

      if (g_inOP){
         if (!g_enabled){
            /* CiA 402 enable sequence, driven by the statusword */
            if      ((sw & 0x004F)==0x0040) out->controlword=0x0006; /* Shutdown     */
            else if ((sw & 0x006F)==0x0021) out->controlword=0x0007; /* Switch on    */
            else if ((sw & 0x006F)==0x0023) out->controlword=0x000F; /* Enable op.   */
            else if ((sw & 0x006F)==0x0027) g_enabled=1;
            else if  (sw & 0x0008)          out->controlword=0x0080; /* Fault reset  */

            /* Hold zero velocity until the drive is enabled */
            out->target_velocity = 0;
         } else {
            out->controlword = 0x000F;
            out->target_velocity = vel_step();
         }
         out->modes_of_operation = MODE_CSV;
         g_targetvel = out->target_velocity;
      }
      ecx_send_processdata(&ctx);
   }
}

int main(int argc, char *argv[]){
   clock_gettime(CLOCK_MONOTONIC, &tz);
   printf("EtherCAT CSV example (500 us DC cycle, SOEM master)\n\n");

   if (argc != 2){ printf("Usage: sudo %s <ifname>\n", argv[0]); return 1; }
   signal(SIGINT, on_sigint);

   if (!ecx_init(&ctx, argv[1])){ printf("ERROR: cannot open %s (root required)\n", argv[1]); return 1; }
   STAGE("NIC open");

   if (ecx_config_init(&ctx) <= 0){ printf("ERROR: no slaves found\n"); ecx_close(&ctx); return 1; }
   STAGE("bus scanned: %d slave(s), '%s' in PRE-OP",
         ctx.slavecount, ctx.slavelist[1].name);

   /* Distributed clocks: measure delays, then activate Sync0 before the
      SAFE-OP transition (DC configuration is validated at that transition). */
   ctx.slavelist[1].PO2SOconfig = drive_setup;
   ecx_configdc(&ctx);
   ecx_dcsync0(&ctx, 1, TRUE, CYCLE_NS, 0);
   STAGE("Sync0 active (%u ns cycle)", (unsigned)CYCLE_NS);
   osal_usleep(SYNC0_MARGIN_US);

   ecx_config_map_group(&ctx, IOmap, 0);
   out = (out_pdo_t *)ctx.slavelist[1].outputs;
   in  = (in_pdo_t  *)ctx.slavelist[1].inputs;
   STAGE("process image mapped: %d out / %d in bytes",
         ctx.slavelist[1].Obytes, ctx.slavelist[1].Ibytes);

   /* Verify SAFE-OP on fresh state; retry acknowledges and re-requests. */
   int reached = 0;
   for (int attempt = 1; attempt <= 3; attempt++){
      ecx_statecheck(&ctx, 1, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
      ecx_readstate(&ctx);
      if (ctx.slavelist[1].state == EC_STATE_SAFE_OP){
         STAGE("SAFE-OP reached (attempt %d)", attempt);
         reached = 1; break;
      }
      STAGE("SAFE-OP attempt %d incomplete: state=0x%02x AL=0x%04x",
            attempt, ctx.slavelist[1].state, ctx.slavelist[1].ALstatuscode);
      ctx.slavelist[1].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
      ecx_writestate(&ctx, 1);
      osal_usleep(100000);
      ctx.slavelist[1].state = EC_STATE_SAFE_OP;
      ecx_writestate(&ctx, 1);
   }
   if (!reached){ printf("ERROR: SAFE-OP not reached\n"); goto shutdown; }

   /* Prime outputs with safe values, start the cyclic thread. */
   out->controlword=0; out->modes_of_operation=MODE_CSV; out->target_velocity=0;
   ecx_send_processdata(&ctx); ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

   OSAL_THREAD_HANDLE rt;
   osal_thread_create_rt(&rt, 128000, (void*)&ecat_rt_thread, NULL);
   STAGE("cyclic thread running");

   /* Request OP immediately; the DC phase lock converges during the
      transition (target velocity is zero throughout the enable). */
   ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
   ecx_writestate(&ctx, 0);
   for (int i=0;i<1000;i++){
      ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, 20000);
      if (ctx.slavelist[0].state == EC_STATE_OPERATIONAL) break;
      osal_usleep(CYCLE_NS/1000);
   }
   if (ctx.slavelist[0].state != EC_STATE_OPERATIONAL){
      STAGE("ERROR: OP not reached (AL=0x%04x)", ctx.slavelist[1].ALstatuscode);
      run=0; goto shutdown;
   }
   STAGE("OPERATIONAL");
   g_inOP = 1;

   for (int i=0;i<4000 && run;i++){
      if (g_enabled) break;
      osal_usleep(CYCLE_NS/1000);
   }
   if (!g_enabled){
      STAGE("ERROR: enable failed sw=0x%04x err=0x%04x", g_statusword, g_errcode);
      run=0; goto shutdown;
   }
   STAGE("ENABLED sw=0x%04x mode=%d", g_statusword, g_modedisp);

   /* Monitor: ramp to cruise, hold, ramp to zero. */
   STAGE("CSV profile: cruise %d counts/s - WATCH THE MOTOR", TARGET_VEL);
   long c=0;
   while (run && !g_done){
      if ((c++ % 200)==0)
         printf("vel_cmd=%9d  pos=%11d  sw=0x%04x err=0x%04x toff=%lld\n",
                g_targetvel, g_position, g_statusword, g_errcode, (long long)g_toff);
      osal_usleep(CYCLE_NS/1000);
   }
   STAGE("profile complete");

shutdown:
   /* Controlled shutdown: zero velocity, disable, leave OP, then stop
      Sync0 - Sync0 must remain active until the drive has left OP. */
   STAGE("shutting down");
   run = 0; osal_usleep(50000);
   if (ctx.slavelist[1].outputs){
      out->target_velocity = 0; out->controlword = 0x0000;
      ecx_send_processdata(&ctx); ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
   }
   ctx.slavelist[0].state = EC_STATE_INIT;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 1, EC_STATE_INIT, EC_TIMEOUTSTATE);
   ecx_dcsync0(&ctx, 1, FALSE, 0, 0);
   ecx_close(&ctx);
   STAGE("done");
   return 0;
}
