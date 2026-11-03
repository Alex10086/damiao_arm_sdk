# Damiao Arm SDK (OpenArm CAN Submodule + USB-CAN Bridge)

C++17 SDK for **Damiao DM-J4310-2EC (24V)** motors and robotic arms, integrating [`enactic/openarm_can`](https://github.com/enactic/openarm_can) as an unmodified Git submodule (`third_party/openarm_can`) and providing a transparent, low-latency **Linux SocketCAN (`vcan0`) <-> Damiao USB-CAN (`/dev/ttyACM0`) Bridge**.

## Phase 1 Architecture: Transparent Driver Adaptation

```
┌────────────────────────────────────────────────────────┐
│            Unmodified OpenArm CAN Submodule            │
│       OpenArm("vcan0", false) -> DM4310 (0x01/0x11)    │
└───────────────────────────┬────────────────────────────┘
                            │ Linux SocketCAN (vcan0)
                            ▼
┌────────────────────────────────────────────────────────┐
│                  SocketCanUsbBridge                    │
│  - Loopback=1, RecvOwnMsgs=0 (zero self-echo loopback) │
│  - Partial write retry, disconnect & auto-reconnect    │
│  - Strict CAN ID / DLC / error-frame filtering         │
└───────────────────────────┬────────────────────────────┘
                            │ Damiao USB-CDC (921600)
                            ▼
                 /dev/ttyACM0 -> J4310 Motor
```

## Build & Run

```bash
# 1. Setup virtual CAN interface vcan0
./scripts/setup_vcan.sh vcan0

# 2. Configure & build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4

# 3. Run unit & bridge edge-case tests (79 checks)
ctest --test-dir build --output-on-failure

# 4. Run Phase 1 OpenArm CAN + Real J4310 hardware acceptance test (embedded bridge)
./build/openarm_single_motor_test --interface vcan0 --serial /dev/ttyACM0 --with-bridge --samples 200 --period-ms 5

# Or run standalone bridge daemon + OpenArm test separately:
./build/damiao_socketcan_bridge --socketcan vcan0 --serial /dev/ttyACM0 &
./build/openarm_single_motor_test --interface vcan0 --samples 500 --period-ms 2
```
