/******************************************************************************
 * csp_example.c - Cyclic Synchronous Position (CiA 402 mode 8) reference for
 *                 EtherCAT servo drives (MDX+ series).
 *
 * Demonstrates a complete CSP bring-up with the open-source SOEM 2.x master:
 *   - Distributed Clocks, Sync0 = 1 ms, standard activation
 *   - Required startup SDO parameters (limit objects, interpolation period)
 *   - RxPDO 0x1600: Controlword + Modes + Target position       (7 bytes)
 *   - TxPDO 0x1A00: Statusword + Position + Error + Mode        (9 bytes)
 *   - Phase-locked real-time cyclic loop (drive DC clock reference)
 *   - Bumpless enable (target tracks actual position until enabled), then a
 *     smooth trapezoidal move: out, dwell, return, settle, disable
 *
 * VALIDATED at 1 ms and 4 ms DC cycles on MDX+_EC.
 *
 * Platform: Linux with PREEMPT_RT kernel recommended. Build:
 *   gcc csp_example.c -o csp_example \
 *       -I <SOEM>/include -I <SOEM>/build/include -I <SOEM>/osal \
 *       -I <SOEM>/osal/linux -I <SOEM>/oshw/linux \
 *       -L <SOEM>/build -lsoem -lpthread -lrt -lm
 *
 * Usage:  sudo ./csp_example <network-interface>
 *
 * SAFETY: This program MOVES THE MOTOR (one revolution out and back by
 * default). Ensure free travel, keep an emergency stop within reach.
 * Ctrl+C performs a controlled shutdown. Verify COUNTS_PER_REV against
 * your drive's position scaling before running on a loaded axis.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include "soem/soem.h"

/* ---- Configuration ------------------------------------------------------ */
#define CYCLE_NS          1000000  /* 1 ms.                      */
#define NSEC_PER_SEC      1000000000
#define SYNC0_MARGIN_US   2000
#define MODE_CSP          8

/* Motion profile in physical units (10,000 counts = 1 rev, measured) */
#define COUNTS_PER_REV    10000.0  /* measured: 1.00 rev = 10000 command counts (electronic gearing) */
#define MOVE_REVS         1.0      /* move distance: 1 revolution           */
#define CRUISE_RPS        1.0      /* cruise speed: 1 rev/s                 */
#define ACCEL_RPS2        2.0      /* accel: 2 rev/s^2 -> cruise in 0.5 s   */
#define DWELL_CYCLES      1000     /* pause at far end: 1 s at 1 ms     */
#define CYCLE_S           (CYCLE_NS / 1e9)
#define MOVE_COUNTS       (MOVE_REVS   * COUNTS_PER_REV)
#define CRUISE_CPS        (CRUISE_RPS  * COUNTS_PER_REV)          /* counts/s     */
#define ACCEL_CPS2        (ACCEL_RPS2  * COUNTS_PER_REV)          /* counts/s^2   */

#define WKC_TRIP          8        /* consecutive WKC misses -> safe stop    */

static ecx_contextt ctx;
static uint8 IOmap[4096];
static volatile int run = 1;
static void on_sigint(int s){ (void)s; run = 0; }

/* Shared between the cyclic thread and main() */
static volatile uint16_t g_statusword = 0;
static volatile uint16_t g_errcode    = 0;
static volatile int32_t  g_position   = 0;
static volatile int32_t  g_target     = 0;
static volatile int8_t   g_modedisp   = 0;
static volatile int      g_enabled    = 0;
static volatile int      g_inOP       = 0;
static volatile int      g_done       = 0;
static volatile int64_t  g_toff       = 0;
static volatile int      g_stop       = 0;  /* 0 ok, 1 bus(WKC), 2 fault, 3 bringup */
static int expectedWKC = 0;

static struct timespec tz;
static int64_t now_us(void){
   struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
   return (int64_t)(t.tv_sec - tz.tv_sec)*1000000 + (t.tv_nsec - tz.tv_nsec)/1000;
}
#define STAGE(...) do{ printf("[%8lld us] ", (long long)now_us()); \
                       printf(__VA_ARGS__); printf("\n"); }while(0)

/* Process data images - BOTH directions remapped by this program, so these
   structs are authoritative. pack(1): no padding, matches the wire.        */
#pragma pack(push,1)
typedef struct {                    /* RxPDO 0x1600 (7 bytes)               */
   uint16_t controlword;            /* 0x6040 */
   int8_t   modes_of_operation;     /* 0x6060 */
   int32_t  target_position;        /* 0x607A, command counts               */
} out_pdo_t;
typedef struct {                    /* TxPDO 0x1A00 (9 bytes, our mapping)  */
   uint16_t statusword;             /* 0x6041 */
   int32_t  position_actual;        /* 0x6064, command counts               */
   uint16_t error_code;             /* 0x603F */
   int8_t   modes_display;          /* 0x6061 */
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
      must be set for motion to be produced. */
   u16=1000;    sdo_w(slave,0x6072,0,2,&u16,"6072 MaxTorque");
   u16=1000;    sdo_w(slave,0x6073,0,2,&u16,"6073 MaxCurrent");
   u16=3000;    sdo_w(slave,0x60E0,0,2,&u16,"60E0 PosTorqueLim");
   u16=3000;    sdo_w(slave,0x60E1,0,2,&u16,"60E1 NegTorqueLim");
   u32=1000000; sdo_w(slave,0x607F,0,4,&u32,"607F MaxVelocity");

   /* RxPDO 0x1600: Controlword + Modes + Target position (7 bytes).
      Entry format: (index<<16) | (subindex<<8) | bit length.               */
   u8=0;            sdo_w(slave,0x1C12,0,1,&u8, "1C12 clear");
   u8=0;            sdo_w(slave,0x1600,0,1,&u8, "1600 clear");
   u32=0x60400010;  sdo_w(slave,0x1600,1,4,&u32,"map Controlword");
   u32=0x60600008;  sdo_w(slave,0x1600,2,4,&u32,"map Modes");
   u32=0x607A0020;  sdo_w(slave,0x1600,3,4,&u32,"map TargetPosition");
   u8=3;            sdo_w(slave,0x1600,0,1,&u8, "1600 count=3");
   u16=0x1600;      sdo_w(slave,0x1C12,1,2,&u16,"1C12 assign 1600");
   u8=1;            sdo_w(slave,0x1C12,0,1,&u8, "1C12 count=1");

   /* TxPDO 0x1A00: Statusword + Position actual + Error code + Mode display
      (9 bytes). Remapped so the input layout is OURS on every MDX+ variant
      - factory input PDOs differ between configurations.                   */
   u8=0;            sdo_w(slave,0x1C13,0,1,&u8, "1C13 clear");
   u8=0;            sdo_w(slave,0x1A00,0,1,&u8, "1A00 clear");
   u32=0x60410010;  sdo_w(slave,0x1A00,1,4,&u32,"map Statusword");
   u32=0x60640020;  sdo_w(slave,0x1A00,2,4,&u32,"map PositionActual");
   u32=0x603F0010;  sdo_w(slave,0x1A00,3,4,&u32,"map ErrorCode");
   u32=0x60610008;  sdo_w(slave,0x1A00,4,4,&u32,"map ModesDisplay");
   u8=4;            sdo_w(slave,0x1A00,0,1,&u8, "1A00 count=4");
   u16=0x1A00;      sdo_w(slave,0x1C13,1,2,&u16,"1C13 assign 1A00");
   u8=1;            sdo_w(slave,0x1C13,0,1,&u8, "1C13 count=1");

   /* interpolation time period = 5 x 10^-4 s = 500 us (CSP validates this) */
   /* interpolation time period = 1 x 10^-3 s = 1 ms (matches Sync0) */
   u8 = 1;            sdo_w(slave,0x60C2,1,1,&u8, "60C2:1 interp value");
   { int8_t e = -3;   sdo_w(slave,0x60C2,2,1,&e,  "60C2:2 interp exponent"); }
   { int8_t m=MODE_CSP; sdo_w(slave,0x6060,0,1,&m,"6060 mode=CSP"); }
   return 1;
}

/* ---- Trajectory generator (runs inside the cyclic thread) ----------------
   Trapezoidal profile in counts/cycle: accelerate to CRUISE_CPC, cruise,
   decelerate to stop exactly at the target; dwell; return the same way.   */
/* Smooth trapezoidal profile, floating point: accel/cruise/decel with
   d = v^2/(2a) stopping rule; dwell at far end; return; done. */
static int32_t traj_step(void){
   static double pos = 0.0, vel = 0.0;   /* counts, counts/s */
   static int leg = 0, dwell = 0, settle = 0; /* 0 out,1 dwell,2 back,3 settle */
   double target = (leg == 0) ? MOVE_COUNTS : 0.0;

   if (leg == 1){ if (++dwell >= DWELL_CYCLES) leg = 2; return (int32_t)pos; }
   if (leg == 3){
      /* SETTLE: the motor trails the commanded target (following lag).
         Disabling immediately would strand it short of home and shift the
         start of every subsequent run. Hold the final target until the
         drive reports target-reached (statusword bit 10), 3 s timeout. */
      if ((g_statusword & 0x0400) || (++settle >= 3 * (NSEC_PER_SEC/CYCLE_NS)))
         g_done = 1;
      return (int32_t)pos;
   }

   double dist  = target - pos;
   double dir   = (dist >= 0) ? 1.0 : -1.0;
   double adist = dir * dist;
   double stopd = (vel * vel) / (2.0 * ACCEL_CPS2);

   if (adist <= stopd)          vel -= ACCEL_CPS2 * CYCLE_S;   /* decel  */
   else if (vel < CRUISE_CPS)   vel += ACCEL_CPS2 * CYCLE_S;   /* accel  */
   if (vel < 0.0) vel = 0.0;
   if (vel > CRUISE_CPS) vel = CRUISE_CPS;

   double step = vel * CYCLE_S;
   if (step >= adist){ pos = target; vel = 0.0; leg = (leg==0) ? 1 : 3; }
   else               pos += dir * step;
   return (int32_t)pos;
}

/* ---- Real-time cyclic thread -------------------------------------------- */
OSAL_THREAD_FUNC_RT ecat_rt_thread(void *arg){
   (void)arg;
   static int32_t base = 0;         /* actual position captured at enable   */
   ec_timet ts; osal_get_monotonic_time(&ts);
   ts.tv_nsec = ((ts.tv_nsec / CYCLE_NS) + 1) * CYCLE_NS;
   ecx_send_processdata(&ctx);

   while (run){
      add_time_ns(&ts, CYCLE_NS + toff);
      osal_monotonic_sleep(&ts);
      int wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

      /* WKC supervision: consecutive misses = bus fault -> safe stop */
      static int wkcmiss = 0;
      if (g_inOP){
         if (wkc < expectedWKC){
            if (++wkcmiss >= WKC_TRIP){ g_stop = 1; run = 0; }
         } else wkcmiss = 0;
      }

      if (ctx.slavelist[0].hasdc)
         ec_sync(ctx.DCtime, CYCLE_NS, &toff);
      g_toff = toff;

      uint16_t sw = in->statusword;
      g_statusword = sw; g_errcode = in->error_code;
      g_position = in->position_actual; g_modedisp = in->modes_display;

      if (g_inOP){
         if (!g_enabled){
            /* CiA 402 enable sequence, driven by the statusword */
            if      ((sw & 0x004F)==0x0040) out->controlword=0x0006;
            else if ((sw & 0x006F)==0x0021) out->controlword=0x0007;
            else if ((sw & 0x006F)==0x0023) out->controlword=0x000F;
            else if ((sw & 0x006F)==0x0027){ g_enabled=1; base = in->position_actual; }
            else if  (sw & 0x0008)          out->controlword=0x0080;

            /* CSP CRITICAL: while not enabled, the target must TRACK the
               actual position, so the enable is bumpless. */
            out->target_position = in->position_actual;
         } else if (sw & 0x0008){
            /* drive fault while running: hold at actual, flag, safe stop */
            out->target_position = in->position_actual;
            g_stop = 2; run = 0;
         } else {
            out->controlword = 0x000F;
            out->target_position = base + traj_step();
         }
         out->modes_of_operation = MODE_CSP;
         g_target = out->target_position;
      }
      ecx_send_processdata(&ctx);
   }
}

int main(int argc, char *argv[]){
   clock_gettime(CLOCK_MONOTONIC, &tz);
   printf("AMP EtherCAT CSP example (1 ms DC cycle, SOEM master)\n\n");

   if (argc != 2){ printf("Usage: sudo %s <ifname>\n", argv[0]); return 1; }
   signal(SIGINT, on_sigint);
   mlockall(MCL_CURRENT | MCL_FUTURE);

   if (!ecx_init(&ctx, argv[1])){ printf("ERROR: cannot open %s (root required)\n", argv[1]); return 1; }
   STAGE("NIC open");

   if (ecx_config_init(&ctx) <= 0){ printf("ERROR: no slaves found\n"); ecx_close(&ctx); return 1; }
   STAGE("bus scanned: %d slave(s), '%s' in PRE-OP",
         ctx.slavecount, ctx.slavelist[1].name);
   if (ctx.slavecount != 1){
      printf("ERROR: this single-axis example expects exactly 1 slave, found %d.\n"
             "       Use the two-slave example, or connect only one drive.\n",
             ctx.slavecount);
      ecx_close(&ctx); return 1;
   }

   ctx.slavelist[1].PO2SOconfig = drive_setup;
   ecx_configdc(&ctx);
   ecx_dcsync0(&ctx, 1, TRUE, CYCLE_NS, 0);
   STAGE("Sync0 active (%u ns cycle)", (unsigned)CYCLE_NS);
   osal_usleep(SYNC0_MARGIN_US);

   ecx_config_map_group(&ctx, IOmap, 0);
   out = (out_pdo_t *)ctx.slavelist[1].outputs;
   in  = (in_pdo_t  *)ctx.slavelist[1].inputs;
   STAGE("process image mapped: %d out / %d in bytes (expect %zu / %zu)",
         ctx.slavelist[1].Obytes, ctx.slavelist[1].Ibytes,
         sizeof(out_pdo_t), sizeof(in_pdo_t));
   if (ctx.slavelist[1].Obytes != sizeof(out_pdo_t) ||
       ctx.slavelist[1].Ibytes != sizeof(in_pdo_t)){
      printf("ERROR: mapped sizes do not match the remap - aborting.\n");
      g_stop = 3; goto shutdown;
   }

   /* Verify SAFE-OP on fresh state; retry acknowledges and re-requests. */
   {
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
      if (!reached){ printf("ERROR: SAFE-OP not reached\n"); g_stop = 3; goto shutdown; }
   }

   /* Prime outputs with safe values: target = current actual position. */
   ecx_send_processdata(&ctx); ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
   out->controlword=0; out->modes_of_operation=MODE_CSP;
   out->target_position = in->position_actual;
   ecx_send_processdata(&ctx); ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

   expectedWKC = (ctx.grouplist[0].outputsWKC*2) + ctx.grouplist[0].inputsWKC;
   STAGE("expected WKC: %d", expectedWKC);

   OSAL_THREAD_HANDLE rt;
   osal_thread_create_rt(&rt, 128000, (void*)&ecat_rt_thread, NULL);
   STAGE("cyclic thread running");

   /* Request OP immediately; DC phase lock converges during the transition
      (target tracks actual position throughout the enable). */
   ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
   ecx_writestate(&ctx, 0);
   for (int i=0;i<1000;i++){
      ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, 20000);
      if (ctx.slavelist[0].state == EC_STATE_OPERATIONAL) break;
      osal_usleep(CYCLE_NS/1000);
   }
   if (ctx.slavelist[0].state != EC_STATE_OPERATIONAL){
      STAGE("ERROR: OP not reached (AL=0x%04x)", ctx.slavelist[1].ALstatuscode);
      run=0; g_stop=3; goto shutdown;
   }
   STAGE("OPERATIONAL");
   g_inOP = 1;

   for (int i=0;i<4000 && run;i++){
      if (g_enabled) break;
      osal_usleep(CYCLE_NS/1000);
   }
   if (!g_enabled){
      STAGE("ERROR: enable failed sw=0x%04x err=0x%04x", g_statusword, g_errcode);
      run=0; g_stop=3; goto shutdown;
   }
   STAGE("ENABLED sw=0x%04x mode=%d base=%d", g_statusword, g_modedisp, g_position);

   STAGE("CSP move: +%.1f rev at %.1f rev/s (accel %.1f rev/s^2) - WATCH THE AXIS",
         MOVE_REVS, CRUISE_RPS, ACCEL_RPS2);
   long c=0;
   while (run && !g_done){
      if ((c++ % 100)==0)   /* every 100 ms at 1 ms */
         printf("target=%11d  actual=%11d  sw=0x%04x err=0x%04x toff=%lld\n",
                g_target, g_position, g_statusword, g_errcode, (long long)g_toff);
      osal_usleep(CYCLE_NS/1000);
   }
   if (!g_stop) STAGE("move complete");

    if (g_stop == 1){
      ecx_readstate(&ctx);
      STAGE("!!! STOPPED: WKC loss. slave state=0x%02x AL=0x%04x sw=0x%04x err=0x%04x",
            ctx.slavelist[1].state, ctx.slavelist[1].ALstatuscode,
            g_statusword, g_errcode);
   }
   if (g_stop == 2) STAGE("!!! STOPPED: drive fault sw=0x%04x err=0x%04x",
                          g_statusword, g_errcode);

shutdown:
   STAGE("shutting down");
   run = 0; osal_usleep(50000);
   if (ctx.slavelist[1].outputs){
      out->controlword = 0x0000;
      ecx_send_processdata(&ctx); ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
   }
   ctx.slavelist[0].state = EC_STATE_INIT;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 1, EC_STATE_INIT, EC_TIMEOUTSTATE);
   ecx_dcsync0(&ctx, 1, FALSE, 0, 0);
   ecx_close(&ctx);
   STAGE("done");
   return g_stop;
}
