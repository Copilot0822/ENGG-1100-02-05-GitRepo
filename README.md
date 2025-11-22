# ENGG-1100-02-05 – Arduino Drive & Launch Control

Arduino codebase for an autonomous drive-and-launch system built around an ultrasonic distance sensor, L298N motor drivers, and a custom S-curve motion-profile PID controller (“MotionMagic7”).  

The repository contains two main operating modes:

- **Drive Mode** – for the driving run in the performance test
- **Launch Mode** – for the launching run in the performance test

Both modes share a common support library defined in `classes.h`.

---

## Repository Structure

```text
ENGG-1100-02-05-GitRepo/
├── NewDrivingCode/
│   └── newDrivingCode/
│       ├── newDrivingCode.ino   # Drive mode sketch
│       └── classes.h            # Shared support classes
└── NewShootCode/
    ├── NewShootCode.ino         # Launch mode sketch
    └── classes.h                # Same support classes as above
```

The two `classes.h` files are identical and provide all reusable components (sensors, motor driver, motion controller).

---

## Hardware Overview

The sketches assume an Arduino Uno with:

- **H-bridge motor driver (L298N or similar)**  
  - Drive motor:
    - `ena` (PWM): **D5**
    - `in1`: **D2**
    - `in2`: **D4**
  - Launcher motor (launch mode only):
    - `ena` (PWM): **D3**
    - `in1`: **D7**
    - `in2`: **D8**
- **Ultrasonic distance sensor** (e.g., HC-SR04):
  - Trigger: **D11**
  - Echo: **D10**
- Optional switches (supported by `Switch` class, not used in current `.ino` files). Was deprecated because the switch wiring was deemed un reliable

Pin assignments can be changed by editing the object constructions at the top of each `.ino` file.

---

## Common Support Library (`classes.h`)

All sketches include:

```cpp
#include "classes.h"
```

`classes.h` defines the following reusable classes:

### `Switch`

Simple wrapper for a digital input with optional inversion.

- Constructor: `Switch(int pin, bool invert = false)`
- Methods:
  - `void init()` – sets `pinMode(pin, INPUT_PULLUP)`.
  - `bool State()` – returns logical switch state, automatically handling inversion.

### `UltrasonicSensor`

Abstraction for a basic trigger/echo ultrasonic rangefinder.

- Constructor: `UltrasonicSensor(int trigPin, int echoPin)`
- Methods:
  - `void init()` – configures trigger as `OUTPUT`, echo as `INPUT`.
  - `float Distance()` – triggers the sensor and returns distance in **cm** using pulse timing.

### `L298N`

Helper for driving a DC motor through an L298N-style H-bridge.

- Constructor: `L298N(int enPin, int in1Pin, int in2Pin, bool brake = true)`
  - `brake = true` enables active braking when speed is set to `0`.
- Methods:
  - `void init()` – sets control pins as outputs.
  - `void setSpeed(double speed = 0)`  
    - `speed` in range `[-1.0, 1.0]`
    - `> 0` → forward, `< 0` → reverse  
    - `0` with `brake == true` → active braking (both outputs HIGH, enable LOW)  
    - `0` with `brake == false` → coast (all LOW)

### `MotionMagic7`

Custom 7-segment S-curve motion profile plus PID + feedforward, designed to emulate CTRE Motion Magic on Arduino.

- Constructor: `MotionMagic7(float kP, float kI, float kD, float kF)`
- Configuration:
  - `void setConstraints(float maxVel, float maxAccel, float maxJerk)`
  - Internal integral term clamped by `_iLimit` to prevent windup.
- Control:
  - `void setTarget(float targetPos, float currentPos)`
    - Computes a jerk-limited S-curve trajectory from `currentPos` to `targetPos`.
    - Resets internal PID state.
  - `float update(float currentPos, float dt)`
    - Advances the planned trajectory by `dt`.
    - Computes PID + feedforward output in the range `[-1.0, 1.0]` to follow the planned profile.
  - Monitoring:
    - `float plannedPosition() const`
    - `float plannedVelocity() const`

This class is used as the core motion controller for both drive and launch approaches.

---

## Drive Mode – `NewDrivingCode/newDrivingCode/newDrivingCode.ino`

Autonomous drive routine that moves the robot to a specified distance from an obstacle using ultrasonic feedback and jerk-limited motion control, then performs a return run.

### Key Objects

```cpp
L298N drive(5, 2, 4);        // Drive motor H-bridge
UltrasonicSensor sensor(11, 10);
MotionMagic7 Pid(4, 0.1, 0, 0);  // kP, kI, kD, kF
```

In `setup()`:

- `drive.init();`
- `sensor.init();`
- `Pid.setConstraints(100, 100, 100);`  // max velocity, acceleration, jerk (units are standard si eg. m/s)
- `Serial.begin(9600);`

### Main Parameters

```cpp
const float initDriveT       = 0.25;  // Target distance from wall [m]
const float returnDriveT     = 11.87; // Deprecated return target
const float acceptableError  = 0.05;  // Allowed distance error [m]
```

### State Machine (`DriveStage`)

The `loop()` implements a staged, non-blocking sequence controlled by `DriveStage`:

1. **Stage 0 – Initial Delay**
   - Waits ~1 s to allow the operator to move clear.
   - Increments `DriveStage`.

2. **Stage 1 – PID Initialization**
   - Reads initial distance (converted from cm to m).
   - Records current time (`time = millis()/1000.0f`).
   - Calls `Pid.setTarget(initDriveT, distance)` to generate the S-curve profile.
   - Resets `countOut` and advances to Stage 2.

3. **Stage 2 – Drive to Target Distance**
   - Each loop:
     - Computes `dt` from `millis()`.
     - Calls `Pid.update(distance, dt)` to get the control output.
     - Clamps output to approximately `[-0.8, 0.8]`.
     - Sends output to `drive.setSpeed(output)`.
     - Prints the output and measured distance over Serial.
   - If the absolute distance error stays within `acceptableError` for more than 10 cycles (`countOut > 10`), the drive is considered settled and the stage advances.

4. **Later Stages – Return Run**
   - There is a second movement section “for the return run”.  
   - A closed-loop return to `returnDriveT` using the ultrasonic sensor was originally implemented but later disabled because the ultrasonic sensor was unreliable in the atrium (noise/interference).
   - Instead, Stage 4 drives the robot back at a constant speed:
     ```cpp
     float output = -0.7;  // Fixed reverse drive
     drive.setSpeed(output);
     ```
   - After a condition on `countOut`, the stage advances and the motor is stopped.

5. **Final Stage**
   - Prints `"Done driving!"` once the sequence is complete.

This file is intended to be used for the autonomous drive-only demonstration.

---

## Launch Mode – `NewShootCode/NewShootCode.ino`

Autonomous “launch” routine that drives to a set distance from a wall/target, then fires the launcher.

### Key Objects

```cpp
L298N launch(3, 7, 8);       // Launcher motor
L298N drive(5, 2, 4);        // Drive motor
UltrasonicSensor sensor(11, 10);
MotionMagic7 Pid(4, 0.1, 0, 0);
```

In `setup()`:

- `launch.init();`
- `drive.init();`
- `sensor.init();`
- `Pid.setConstraints(100, 100, 100);`
- `Serial.begin(9600);`

### Main Parameters

```cpp
const int   shootingT               = 0.1;   // Distance to wall before shooting [m]
const float shooterPower            = 1.0f;  // Launcher motor power [-1.0, 1.0]
const float shootingAcceptableError = 0.03;  // Allowed error before firing [m]
```

### State Machine (`ShootStage`)

1. **Stage 0 – Initial Delay**
   - `delay(1000);` to allow the operator to step away.
   - `ShootStage++`.

2. **Stage 1 – PID Initialization**
   - Records the current time.
   - Calls `Pid.setTarget(shootingT, distance)` to generate the S-curve trajectory to the desired shooting distance from the wall.
   - `ShootStage++`.

3. **Stage 2 – Drive to Shooting Distance**
   - Each loop:
     - Reads `distance` from the ultrasonic sensor (m).
     - Computes the PID/S-curve output with `Pid.update(...)`.
     - Clamps the drive output to roughly `[-0.5, 0.5]`.
     - Sends the command to `drive.setSpeed(output)`.
     - Prints the output to the Serial monitor.
   - When `fabs(distance - shootingT) <= shootingAcceptableError` for more than 10 cycles (`countOut > 10`), the drive motor is stopped and `ShootStage` advances.

4. **Stage 3 – Fire Launcher**
   - Calls `launch.setSpeed(shooterPower);`
   - Delays for `500 ms` to allow the shot.
   - Advances to Stage 4.

5. **Stage 4 – Stop Launcher**
   - Calls `launch.setSpeed();` with the default argument (`0`), which stops the launcher (with braking if enabled).
   - Prevents unnecessary heating and power draw.

---



## Tuning

Several constants can be adjusted for different conditions and performance.

- Motion profile and PID:
  - In both sketches:
    ```cpp
    MotionMagic7 Pid(kP, kI, kD, kF);
    Pid.setConstraints(maxVel, maxAccel, maxJerk);
    ```
  - Increase/decrease `kP`, `kI`, `kD`, `kF` for desired response.
  - Adjust `maxVel`, `maxAccel`, `maxJerk` to make motion smoother or more aggressive.

- Distance targets and tolerances:
  - Drive mode:
    ```cpp
    const float initDriveT      // target distance to wall [m]
    const float acceptableError // allowable error band [m]
    ```
  - Launch mode:
    ```cpp
    const int   shootingT
    const float shootingAcceptableError
    ```

- Motor limits:
  - The drive outputs are explicitly clamped (e.g. `[-0.8, 0.8]` or `[-0.5, 0.5]`).  
    These clamp values can be changed to match your power and traction limits.

- Fixed return speed:
  - In drive mode, Stage 4 uses `float output = -0.7;` for the return run.  
    Adjust this magnitude to alter reverse speed.

---

## Serial Monitoring

Both sketches output diagnostic information over the serial port at 9600 baud, including:

- PID output commands to the drive motor.
- Measured distance from the ultrasonic sensor.

Use the Arduino Serial Monitor (or another terminal) to observe behaviour and help with tuning.

---

## Authors

- Benjamin McRae (1379397) – primary author of `classes.h`, `MotionMagic7`, L298N wrapper, and both application sketches.
- Hong Yuet Cheng (1375779) – contributed to the initial design of the `Switch` and `UltrasonicSensor` objects.