# Damiao J4310 (24V) Linux SocketCAN C++ SDK (V0.1 - Single Motor)

C++17 SDK for Damiao **DM-J4310-2EC (24V)** motors over Linux SocketCAN, designed as the V0.1 single-motor foundation for a 6-axis robotic arm.

## Architecture

```
        Test / Upper Application
                   │
                   ▼
          ┌──────────────────┐
          │   MotorDriver    │
          │                  │
          │  enable()        │
          │  disable()       │
          │  setMIT()        │
          │  setPosVel()     │
          │  setVelocity()   │
          │  getState()      │
          └────────┬─────────┘
                   │
                   ▼
          ┌──────────────────┐
          │   CAN Transport  │
          │                  │
          │  open()          │
          │  send()          │
          │  receive()       │
          │  close()         │
          └────────┬─────────┘
                   │
                   ▼
            Linux SocketCAN
                   │
                   ▼
           Damiao J4310 Motor
```

## V0.1 Modules

| Module | File(s) | Description |
|---|---|---|
| `MotorConfig` / `MotorState` | `include/damiao_arm_sdk/types.hpp` | Model limits (`P_MAX=12.5 rad`, `V_MAX=30.0 rad/s`, `T_MAX=10.0 Nm`), CAN IDs, control mode, fault enum (`0x0`–`0xE`), feedback staleness check |
| `SocketCanTransport` | `include/damiao_arm_sdk/can_transport.hpp`, `src/can_transport.cpp` | Linux `PF_CAN` raw socket `open`/`close`/`send`/`receive` with `poll()` timeout and `CAN_RAW_LOOPBACK` |
| `MotorDriver` & `protocol` | `include/damiao_arm_sdk/motor_driver.hpp`, `src/motor_driver.cpp` | Damiao MIT / POS_VEL / VEL / Enable / Disable / SaveZero / ClearError encoding & 8-byte feedback decoding |
| `Tests` | `tests/test_sdk.cpp` | Protocol codec unit tests, feedback parser tests, exception/staleness tests, and `vcan0` loopback integration tests |
| `Example` | `examples/single_motor_demo.cpp` | Single-motor CLI demo (supports `vcan0` virtual responder or `--real-hw` on `can0`) |

## Build & Run Tests

```bash
chmod +x scripts/setup_vcan.sh
./scripts/setup_vcan.sh vcan0

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# Run unit + vcan0 tests
ctest --test-dir build --output-on-failure

# Run single motor demo on vcan0
./build/single_motor_demo --interface vcan0 --can-id 0x01 --master-id 0x11
```
