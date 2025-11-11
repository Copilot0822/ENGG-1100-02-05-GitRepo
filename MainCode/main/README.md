# Arduino Utility Classes

This repository provides a single header, `classes.h`, that collects several reusable Arduino classes:

- `Switch` — simple digital input with optional inversion and internal pull-up.
- `UltrasonicSensor` — HC-SR04 style ultrasonic distance sensor wrapper.
- `L298N` — DC motor driver helper for an L298N module, including optional braking on zero command.
- `MotionMagic7` — a self-contained, s-curve (jerk-limited) motion-profiled PID-like controller that outputs a motor command in `[-1, 1]` similar in spirit to CTRE Motion Magic.

Place `classes.h` in your Arduino sketch folder and include it from your `.ino` or `.cpp` file.

```cpp
#include "classes.h"

Switch limitSwitch(2, true);
UltrasonicSensor frontSonar(8, 9);
L298N leftMotor(5, 6, 7);
MotionMagic7 motion(0.5f, 0.0f, 0.0f, 0.2f);

void setup() {
    limitSwitch.init();
    frontSonar.init();
    leftMotor.init();

    // motion constraints: maxVel, maxAccel, maxJerk (units/sec, units/sec^2, units/sec^3)
    motion.setConstraints(200.0f, 500.0f, 2000.0f);
    motion.setTarget(1000.0f, 0.0f);  // go to position 1000 from current 0
}

void loop() {
    float dt = 0.02f; // example fixed timestep, 20 ms

    // read sensor
    float distCm = frontSonar.Distance();

    // drive motor from motion profile
    float currentPosition = 0.0f; // replace with your real sensor
    float cmd = motion.update(currentPosition, dt);
    leftMotor.setSpeed(cmd);

    delay(20);
}
```

---

## 1. `Switch`

**Purpose:** Read a digital input as a boolean and handle inverted logic cleanly.

**Key points:**
- Constructor takes the pin and an optional `Invert` flag.
- Call `init()` once in `setup()` to enable the internal pull-up.
- `State()` returns `true` when the switch is considered "active".

**Example:**

```cpp
Switch btn(4, false);

void setup() {
    btn.init();
}

void loop() {
    if (btn.State()) {
        // do something
    }
}
```

---

## 2. `UltrasonicSensor`

**Purpose:** Trigger and read a standard ultrasonic sensor and return the measured distance as a `float` in centimeters.

**Key points:**
- Constructor takes `trigPin` and `echoPin`.
- Call `init()` once in `setup()`.
- `Distance()` performs a trigger+echo cycle and returns a float.
- Internally uses:
  ```cpp
  distance = duration * 0.034f / 2.0f;
  ```
  which is the usual speed-of-sound formula for these modules.

**Example:**

```cpp
UltrasonicSensor sonar(8, 9);

void setup() {
    Serial.begin(9600);
    sonar.init();
}

void loop() {
    float d = sonar.Distance();
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" cm");
    delay(200);
}
```

---

## 3. `L298N`

**Purpose:** Simplify running a DC motor using an L298N module with an enable pin and two direction pins.

**Constructor:**
```cpp
L298N(int enPin, int in1Pin, int in2Pin, bool brake = true);
```

- `enPin` is the PWM-capable enable pin.
- `in1Pin` and `in2Pin` select direction.
- `brake = true` means that a command of `0` will short the motor (both direction pins HIGH) and stop it harder.

**Methods:**
- `init()` sets the pins as outputs.
- `setSpeed(double speed)` takes a value from `-1.0` to `1.0`:
  - `> 0` drives forward, PWM = `speed * 255`.
  - `< 0` drives reverse, PWM = `abs(speed) * 255`.
  - `== 0` brakes or coasts depending on the constructor flag.

**Example:**

```cpp
L298N motor(5, 6, 7);  // ENA=5, IN1=6, IN2=7

void setup() {
    motor.init();
}

void loop() {
    motor.setSpeed(0.5);  // 50% forward
    delay(1000);
    motor.setSpeed(-0.5); // 50% reverse
    delay(1000);
    motor.setSpeed(0.0);  // brake or coast
    delay(1000);
}
```

---

## 4. `MotionMagic7`

**Purpose:** Produce a jerk-limited, 7-segment motion profile and a PID-like tracking output in `[-1, 1]` to drive a motor controller. Useful when you want smooth moves to a position with velocity, acceleration, and jerk limits.

**Setup steps:**
1. Construct with gains:
   ```cpp
   MotionMagic7 motion(kP, kI, kD, kF);
   ```
2. Set motion constraints (units must match your position units):
   ```cpp
   motion.setConstraints(maxVel, maxAccel, maxJerk);
   ```
3. Whenever you want to move to a new position:
   ```cpp
   motion.setTarget(targetPosition, currentPosition);
   ```
4. Every loop, call:
   ```cpp
   float out = motion.update(currentPosition, dtSeconds);
   ```
   and send `out` to your motor driver.

**Notes:**
- The class plans a 7-segment s-curve internally.
- If the move is very short, it auto-scales the profile.
- Output is clamped to `[-1, 1]` so it can be used directly with `analogWrite(...)` scaling, or sent to an L298N wrapper.

**Example integration with L298N:**

```cpp
L298N drive(5, 6, 7);
MotionMagic7 motion(0.4f, 0.0f, 0.0f, 0.1f);

void setup() {
    drive.init();
    motion.setConstraints(300.0f, 800.0f, 2000.0f);
    motion.setTarget(500.0f, 0.0f);  // move to 500
}

void loop() {
    float dt = 0.02f;

    // replace this with a real sensor/encoder value
    float currentPos = 0.0f;

    float cmd = motion.update(currentPos, dt);
    drive.setSpeed(cmd);

    delay(20);
}
```

---

## File structure

- `classes.h` — all class definitions.

Include it like:

```cpp
#include "classes.h"
```

---

## Requirements

- Arduino core (`#include <Arduino.h>`)
- A board that supports `analogWrite` on the chosen enable pin for `L298N`.
- Standard `pulseIn` for the ultrasonic sensor.

---

## License

Add your license here.
