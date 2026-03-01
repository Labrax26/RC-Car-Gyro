# RC Car Gyro

A rate-mode gyro stabilizer for RC cars using a WeAct Blackpill F411CEU6 and MPU6050.

## How It Works

The gyro reads the yaw rate (rotation around the vertical axis) from the MPU6050 and uses a PID controller to stabilize the car by correcting the steering servo.

```
Stick input → Desired yaw rate (deg/s)
MPU6050     → Actual yaw rate (deg/s)
Error       → Desired - Actual
PID         → Correction
Servo       → 1000-2000us output
```

The driver's stick directly commands a desired yaw rate. The PID controller steers the servo to achieve and maintain that rate. Any unexpected rotation (bumps, oversteer, twitches) is automatically corrected.

## Hardware

| Component | Model |
|---|---|
| Microcontroller | WeAct Blackpill F411CEU6 |
| IMU | MPU6050 |
| Receiver | ER5A |
| Radio | Radiomaster Pocket |

## Wiring

### MPU6050
| MPU6050 | Blackpill |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | PB7 |
| SCL | PB6 |

### Receiver
| Channel | Pin | Function |
|---|---|---|
| CH1 | PA12 | Steering |
| CH2 | PA11 | - |
| CH3 | PA10 | Throttle |
| CH4 | PA9 | - |
| CH5 | PA8 | - |

### Outputs
| Pin | Function |
|---|---|
| PB3 | Steering servo |
| PB10 | ESC |

### Power
```
BEC 6V → 2 Diodes → Blackpill 5V
BEC 6V → Servo directly
Blackpill 3.3V → MPU6050 VCC
```

## Software

### Libraries
- `Wire.h` — I2C communication
- `Servo.h` — PWM servo output

### MPU6050 Configuration
- Full scale range: ±500 deg/s
- DLPF: 10 Hz
- Calibration: 2000 samples at startup (keep car still!)

### PID
| Term | Value | Notes |
|---|---|---|
| Kp | 2 | Start here, tune up slowly |
| Ki | 0 | If needed add small value once Kp is tuned |
| Kd | 0 | Add only if oscillation needs damping |

Output is constrained to ±500 then offset by +1500 to produce valid servo microseconds (1000-2000us).

### Channel Mapping
```
CH1 (steering stick) → desired yaw rate
0.4 * (PWM - 1500) = desired deg/s
Full right (2000us) = +200 deg/s
Center    (1500us) =    0 deg/s
Full left (1000us) = -200 deg/s
```

## Startup Sequence

1. Interrupts attach — radio becomes readable
2. Servos attach — 1500us sent to ESC and steering
3. Throttle low check — must be at neutral before continuing
4. Gyro init and calibration — keep car still for ~2 seconds
5. ESC arms — ready to drive

## Safety Features

- Throttle low check on startup
- Signal loss detection (constrain PWM to valid range)
- Integrator clamp (±400)
- Servo output clamp (1000-2000us)

## Known Limitations

- Rate mode only — no heading hold
- Long constant radius corners require Ki to hold steady yaw rate
- No speed-based gain scaling
