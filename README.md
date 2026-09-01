# AMP EtherCAT SOEM Samples

Open-source EtherCAT master sample programs for **Applied Motion Products'** servo products, built on [SOEM 2.x](https://github.com/OpenEtherCATsociety/SOEM) and real-time Linux.

Sample programs for the CiA 402 operating modes of these drives. The cyclic
synchronous modes (CSP/CSV/CST) stream setpoints every bus cycle and require a
real-time Linux master (PREEMPT_RT); the profile modes use the drive's internal
trajectory generator and have relaxed timing requirements.

| Mode | Code | File | Real-time required | Status |
|------|------|------|--------------------|--------|
| Cyclic Synchronous Position (CSP) | 8 | `csp_example.c` | yes | ✅ validated on MDX+_EC |
| Cyclic Synchronous Velocity (CSV) | 9 | `csv_example.c` | yes | ✅ validated on MDX+_EC |
| Cyclic Synchronous Torque (CST) | 10 | `cst_example.c` | yes | ✅ validated on MDX+_EC |
| Profile Position (PP) | 1 | `pp_example.c` | no | planned |
| Profile Velocity (PV) | 3 | `pv_example.c` | no | planned |
| Profile Torque (TQ) | 4 | `tq_example.c` | no | planned |
| Homing (HM) | 6 | `hm_example.c` | no | planned |

**Test hardware:** All ✅ status entries validated on the [**MDXT61GNBECA000**](https://www.applied-motion.com/s/product/mdxt61g-ec000/01tJ30000033rKSIAY?name=MDXT61GNBECA000-200W-IP65-MDX-w-17-bit-Absolute-Encoder) — MDX+ EtherCAT integrated servo, 200 W, IP65, 17-bit absolute encoder.

Samples for the planned modes will be added incrementally; the CSP/CSV/CST
samples establish the shared bring-up (DC sync, PDO mapping, CiA 402 state
machine) that all modes build on.


## Requirements

- Linux with the **PREEMPT_RT** kernel (Ubuntu 24.04 + Ubuntu Pro real-time kernel used for validation; see `docs/`)
- **SOEM 2.x** built from source
- Direct Ethernet connection to the drive (Intel-based NIC recommended)
- Root privileges (raw socket access)

## Build

```bash
gcc cst_example.c -o cst_example \
    -I <SOEM>/include -I <SOEM>/build/include \
    -I <SOEM>/osal -I <SOEM>/osal/linux -I <SOEM>/oshw/linux \
    -L <SOEM>/build -lsoem -lpthread -lrt
```

Substitute `csp_example.c` or `csv_example.c` for `cst_example.c` to build the other modes.

## Run

```bash
sudo ./cst_example <network-interface>     # e.g. enp0s31f5
```

## Key implementation notes

- **Startup parameters:** these drives ship with all torque/velocity limit objects at 0 (0x2A47, 0x6072, 0x6073, 0x60E0/0x60E1, 0x607F). Each zero silently clamps motion while the drive reports a healthy status. The samples write the complete chain during PRE-OP.
- **PDO mapping:** the factory RxPDO targets position modes; each sample remaps 0x1600 to Controlword + Modes of operation + the mode-specific target object (Target position for CSP, Target velocity for CSV, Target torque for CST) during the PRE-OP→SAFE-OP hook (remapping is only permitted in PRE-OP).
- **Distributed Clocks:** standard SOEM activation (`ecx_dcsync0`) is sufficient; Sync0 is armed before the SAFE-OP transition. Validated at 500 µs cycle times.
- **Bumpless enable (CSP):** target position must track actual position until the drive is enabled — enabling with a stale target would command an instantaneous jump. CSV and CST are naturally bumpless (target velocity/torque held at 0 during the enable sequence).
- **Shutdown order:** leave OP before deactivating Sync0 — stopping the sync pulses while the drive is in OP trips its sync-loss monitoring.

## ⚠️ Safety

The CSP/CSV/CST samples command motor position/velocity/torque respectively. Ensure the shaft is free or safely loaded, keep an emergency stop within reach, and start with small setpoints. All samples perform a controlled zero-command shutdown on Ctrl+C.


## License / Disclaimer

Sample code provided as-is for evaluation and integration reference. Verify all parameters against your drive's documentation before use on hardware. EtherCAT® is a registered trademark of Beckhoff Automation GmbH.
