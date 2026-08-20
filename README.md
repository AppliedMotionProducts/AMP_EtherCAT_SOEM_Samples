# AMP EtherCAT SOEM Samples

Open-source EtherCAT master sample programs for **Applied Motion Products'** servo products, built on [SOEM 2.x](https://github.com/OpenEtherCATsociety/SOEM) and real-time Linux.

Sample programs for the CiA 402 operating modes of these drives. The cyclic
synchronous modes (CSP/CSV/CST) stream setpoints every bus cycle and require a
real-time Linux master (PREEMPT_RT); the profile modes use the drive's internal
trajectory generator and have relaxed timing requirements.

| Mode | Code | Directory | Real-time required | Status |
|------|------|-----------|--------------------|--------|
| Cyclic Synchronous Position (CSP) | 8 | `csp/` | yes | planned |
| Cyclic Synchronous Velocity (CSV) | 9 | `csv/` | yes | planned |
| Cyclic Synchronous Torque (CST) | 10 | `cst/` | yes | ✅ validated on MDX+_EC |
| Profile Position (PP) | 1 | `pp/` | no | planned |
| Profile Velocity (PV) | 3 | `pv/` | no | planned |
| Profile Torque (TQ) | 4 | `tq/` | no | planned |
| Homing (HM) | 6 | `hm/` | no | planned |

Samples for the planned modes will be added incrementally; the CST sample and
the diagnostic tools establish the shared bring-up (DC sync, PDO mapping,
CiA 402 state machine) that all modes build on.


## Requirements

- Linux with the **PREEMPT_RT** kernel (Ubuntu 24.04 + Ubuntu Pro real-time kernel used for validation; see `docs/`)
- **SOEM 2.x** built from source
- Direct Ethernet connection to the drive (Intel-based NIC recommended)
- Root privileges (raw socket access)

## Build

```bash
gcc cst/jcr_cst_demo.c -o jcr_cst_demo \
    -I <SOEM>/include -I <SOEM>/build/include \
    -I <SOEM>/osal -I <SOEM>/osal/linux -I <SOEM>/oshw/linux \
    -L <SOEM>/build -lsoem -lpthread -lrt
```

## Run

```bash
sudo ./jcr_cst_demo <network-interface>     # e.g. enp0s31f6
```

## Key implementation notes

- **Startup parameters:** these drives ship with all torque/velocity limit objects at 0 (0x2A47, 0x6072, 0x6073, 0x60E0/0x60E1, 0x607F). Each zero silently clamps motion while the drive reports a healthy status. The samples write the complete chain during PRE-OP.
- **PDO mapping:** the factory RxPDO targets position modes; the samples remap 0x1600 to Controlword + Modes of operation + Target torque during the PRE-OP→SAFE-OP hook (remapping is only permitted in PRE-OP).
- **Distributed Clocks:** standard SOEM activation (`ecx_dcsync0`) is sufficient; Sync0 is armed before the SAFE-OP transition. Validated at 500 µs and 4 ms cycle times.
- **Shutdown order:** leave OP before deactivating Sync0 — stopping the sync pulses while the drive is in OP trips its sync-loss monitoring.

## ⚠️ Safety

The CST/CSV samples command **motor torque/velocity**. Ensure the shaft is free or safely loaded, keep an emergency stop within reach, and start with small setpoints. All samples perform a controlled zero-command shutdown on Ctrl+C.

## Validation

The CST sample and tools were validated on JCR joint modules (17:1 harmonic reduction) and MDX+ EtherCAT drives: 5/5 cold-boot first-attempt SAFE-OP transitions (40.2 ± 0.1 ms with full configuration), continuous 1 kHz AL-register monitoring with zero error activity, sustained 500 µs cyclic operation on PREEMPT_RT.

## License / Disclaimer

Sample code provided as-is for evaluation and integration reference. Verify all parameters against your drive's documentation before use on hardware. EtherCAT® is a registered trademark of Beckhoff Automation GmbH.

