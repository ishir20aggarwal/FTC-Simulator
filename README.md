# FTC Virtual Robot Simulator — Mecanum Demo

## Project Overview

This project is a custom-modified FTC virtual robotics simulator designed for rapid prototyping, testing, and strategy development, with a primary focus on mecanum-drive robots and interactive game mechanics.

The simulator provides a complete virtual FTC field, physics-based motion, ball game elements, scoring logic, and real-time robot control using FTC-style OpModes. It allows teams to simulate robot behavior, test mechanisms, and develop autonomous and TeleOp logic without requiring physical hardware.

**Important Limitation**  
At present, only the *Mecanum Demo OpMode* is fully functional, and it is designed specifically for the *Mecanum Bot*. Other OpModes and robot configurations exist in the project but are not currently supported by the control system.

---

## Mecanum Bot Capabilities

The Mecanum Bot is the core operational robot in this simulator and supports:

- Field-centric mecanum drive with realistic physics  
- Dead-wheel odometry with encoder simulation  
- IMU and gyro simulation  
- Intake mechanism with:
  - Toggle-based control
  - Visual state feedback
  - Internal ball storage (up to three balls)
- Shooter mechanism with:
  - Tunable launch speed and angle
  - Sequential firing control
  - Correct front-of-robot launch positioning
- Real-time visualization of intake location and state

---

## Game and Field System

The simulator implements a complete custom game model:

### Scoring and Hoops

- Balls score when entering the hoop regions  
- Each alliance hopper stores up to nine balls  
- Overflow scoring rule:
  - First nine balls: three points each  
  - Each additional ball: one point

### Hopper and Lever Mechanism

- Each side includes a physical hopper that stores scored balls  
- Field-mounted lever zones trigger a release mechanism:
  - When a robot contacts the lever, all balls in the hopper, including overflow balls, are released back onto the field

### Stuck Ball Recovery System

Some balls can fall into unreachable pockets in the top corners of the field. To prevent gameplay deadlock, the simulator includes a recovery mechanism:

- If a ball enters the top corner pocket and remains nearly motionless for approximately 0.6 seconds, it is automatically counted as scored and routed into the appropriate hopper

This guarantees continuous match flow.

---

## Supported Features

| Feature | Status |
|--------|--------|
Mecanum Demo OpMode | Fully Supported |
Mecanum Bot | Fully Supported |
TeleOp Driving | Fully Supported |
Intake and Shooter Systems | Fully Supported |
Scoring and Overflow Logic | Fully Supported |
Lever Dump Mechanism | Fully Supported |
Autonomous Modes | Not Supported |
Other Robot Types | Not Supported |

---

## Intended Use

This simulator is designed for:

- FTC teams learning mecanum kinematics and odometry  
- Driver practice and match simulation  
- Prototyping intake and shooter logic prior to hardware build  
- Debugging scoring behavior and field interactions  
- Teaching programming concepts with immediate visual feedback

---

## Credits

This project is built upon the **Virtual Robot Controller framework** originally created by:

**Beta8397 — Original Controller Design**

Their work provides the core physics engine, field rendering system, robot architecture, and simulator infrastructure that enable this project. All mecanum control systems, game mechanics, scoring rules, and simulation features described above are custom extensions built on top of that foundation.
