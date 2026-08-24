# Novexa
# 🕷️ Novexa — 18-DOF All-Terrain Hexapod

> **Novexa** is a 3D-printed, six-legged robotic platform designed for adaptive locomotion across varied terrain. It combines inverse kinematics, multiple gait strategies, IMU-based stabilization, obstacle detection, and LiDAR-based environmental mapping.

![Novexa Hexapod](https://github.com/BilalSabugar/Novexa)

---

## 🚀 Overview

Novexa is an experimental **18-degree-of-freedom (DOF) hexapod robot** developed as a platform for studying multi-legged locomotion, gait planning, terrain adaptation, and autonomous robotics.

The robot consists of **six articulated legs**, with **three independently controlled joints per leg**:

* **Coxa** — horizontal/yaw rotation
* **Femur** — vertical/pitch movement
* **Tibia** — forward/pitch extension

This provides a total of:

**6 legs × 3 DOF = 18 DOF**

The mechanical structure is designed entirely in **Autodesk Fusion 360**, with the majority of the robot fabricated using **3D-printed PLA components** for reduced cost and weight.

---

## ✨ Features

* 🦿 **6 articulated legs**
* 🎯 **18 independent degrees of freedom**
* 🧮 Inverse kinematics-based leg control
* 🚶 Multiple locomotion gaits

  * Tripod gait
  * Ripple gait
  * Wave gait
* 🧭 IMU-based orientation feedback
* 📡 Ultrasonic obstacle detection
* 📍 Long-range LiDAR mapping
* 🌡️ Environmental sensing
* ⚖️ PID-based roll and pitch stabilization
* 🔋 Dual-rail power architecture
* 🖨️ Mostly 3D-printed PLA construction
* 🤖 Designed for future autonomous navigation and ROS2 integration

---

## 📐 Mechanical Specifications

| Parameter          | Specification       |
| ------------------ | ------------------- |
| Robot type         | Hexapod             |
| Number of legs     | 6                   |
| Degrees of freedom | 18                  |
| DOF per leg        | 3                   |
| Length             | ~1.0 m              |
| Width              | ~0.8 m              |
| Height             | ~0.4 m              |
| Weight             | ~5.5–6.0 kg         |
| Main material      | PLA                 |
| Tibia grippers     | TPU                 |
| CAD                | Autodesk Fusion 360 |
| Print time         | ~70+ hours          |
| Filament used      | ~2.0–2.5 kg         |
| Infill             | 20–40%, Gyroid      |
| Layer height       | 0.2 mm              |
| Nozzle             | 0.2 mm              |

---

## 🦿 Leg Architecture

Each leg is a 3-link serial manipulator.

```text
          BODY
            │
            ▼
       ┌─────────┐
       │  COXA   │  ← Yaw
       └────┬────┘
            │
            │ L1 = 60 mm
            ▼
       ┌─────────┐
       │  FEMUR  │  ← Pitch
       └────┬────┘
            │
            │ L2 = 90 mm
            ▼
       ┌─────────┐
       │  TIBIA  │  ← Pitch
       └────┬────┘
            │
            │ L3 = 110 mm
            ▼
          FOOT
```

### Link Dimensions

| Link       | Length |
| ---------- | -----: |
| Coxa (L1)  |  60 mm |
| Femur (L2) |  90 mm |
| Tibia (L3) | 110 mm |

The six legs are arranged symmetrically around the body, with the leg positioning designed to provide a large workspace while maintaining a relatively low center of gravity.

---

## 🧮 Kinematics

Novexa uses **inverse kinematics (IK)** to determine the three joint angles required to position each foot at a desired `(x, y, z)` coordinate.

For a target foot position:

```text
P = (x, y, z)
```

the first joint angle is calculated as:

```text
θ1 = atan2(y, x)
```

The horizontal distance from the coxa joint is:

```text
r = √(x² + y²) − L1
```

and the vertical displacement is:

```text
d = z − z0
```

The tibia angle is obtained using the law of cosines:

```text
cos(θ3) =
(r² + d² − L2² − L3²)
────────────────────────
       2L2L3
```

The femur angle is then calculated from the resulting geometry.

This allows the robot to control **foot position rather than directly specifying individual servo angles**, making gait generation and body movement easier to coordinate.

---

## 🚶 Gait Planning

Novexa supports three primary gait strategies.

### 1. Tripod Gait

The six legs are divided into two alternating groups:

```text
Group A → Legs 1, 3, 5
Group B → Legs 2, 4, 6
```

While one tripod is in the swing phase, the other supports the robot.

**Advantages:**

* Fast
* Efficient
* Good static stability
* Suitable for normal walking

---

### 2. Ripple Gait

The legs move sequentially with a phase difference between adjacent legs.

**Advantages:**

* Smoother movement
* Greater stability
* Better suited to uneven terrain

---

### 3. Wave Gait

Only one leg moves at a time while the other five remain in contact with the ground.

**Advantages:**

* Highest stability
* Useful for difficult terrain

**Trade-off:**

* Lower walking speed

---

## ⚡ Electronics Architecture

Novexa uses a **dual-rail power architecture** separating logic electronics from the high-current actuation system.

```text
                    ┌─────────────────┐
                    │     BATTERY     │
                    └────────┬────────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
              ▼                             ▼
       ┌─────────────┐              ┌─────────────┐
       │  LOGIC RAIL │              │ ACTUATION   │
       │    7.4 V    │              │    RAIL     │
       └──────┬──────┘              │   14.8 V    │
              │                     └──────┬──────┘
              ▼                            │
      ┌───────────────┐                    ▼
      │ Arduino Mega  │             ┌─────────────┐
      │   + Sensors   │             │ PCA9685     │
      └───────────────┘             │ Servo Ctrl  │
                                    └──────┬──────┘
                                           │
                              ┌────────────┴────────────┐
                              ▼      ▼      ▼           ▼
                            Servo  Servo  Servo  ... Servo
                              ×18
```

### Power System

| Rail      | Nominal | Terminal | Purpose            |
| --------- | ------: | -------: | ------------------ |
| Logic     |   7.4 V |    8.4 V | MCU + sensors      |
| Actuation |  14.8 V |   16.8 V | Servos / actuators |

The system is designed for approximately **45–60 minutes of operation**, depending on usage.

---

## 🔌 Electronics & Components

| Component               | Quantity | Purpose                   |
| ----------------------- | -------: | ------------------------- |
| Arduino Mega            |        1 | Main controller           |
| PCA9685                 |        1 | PWM control for 18 servos |
| MG995 Servo             |       18 | Leg actuation             |
| MPU6050                 |        1 | 6-axis IMU                |
| HC-SR04                 |        2 | Front obstacle detection  |
| Benewake TF03 100M      |        1 | LiDAR mapping             |
| 7.4 V 2500 mAh Battery  |        1 | Logic power               |
| 14.8 V 5000 mAh Battery |        1 | Actuation power           |

The **PCA9685** is used to control the 18 servo channels required by the six 3-DOF legs.

---

## 🧠 Control Architecture

Novexa's control system is organized into three layers.

```text
┌────────────────────────────────────────┐
│         HIGH-LEVEL AUTONOMOUS          │
│   Obstacle Avoidance / Stabilization   │
└───────────────────┬────────────────────┘
                    │
┌───────────────────▼────────────────────┐
│             MID-LEVEL GAIT              │
│      Gait Generation / Trajectories     │
└───────────────────┬────────────────────┘
                    │
┌───────────────────▼────────────────────┐
│              LOW-LEVEL                 │
│       Servo PWM / Sensor Control        │
└────────────────────────────────────────┘
```

### Low-Level

Responsible for:

* Servo PWM generation
* Sensor communication
* Hardware-level control

### Mid-Level

Responsible for:

* Gait generation
* Phase offsets
* Foot trajectories
* Leg synchronization

### High-Level

Responsible for:

* Obstacle avoidance
* Orientation stabilization
* Autonomous behavior
* Environmental perception

---

## ⚖️ Stabilization

An **MPU6050 IMU** provides orientation feedback to the controller.

A PID controller is used to stabilize the robot's roll and pitch:

```text
Δθ = Kp·e + Ki·∫e dt + Kd·de/dt
```

The stabilization system can compensate for changes in body orientation while walking over uneven terrain.

---

## 📡 Sensor Suite

### MPU6050 IMU

Provides:

* Accelerometer data
* Gyroscope data
* 6-axis motion/orientation information

Used primarily for body orientation and stabilization.

### HC-SR04 Ultrasonic Sensors

Two ultrasonic sensors are used for **front obstacle detection** and nearby distance measurement.

### Benewake TF03 100M LiDAR

Provides long-range distance information for:

* Environmental mapping
* Path planning
* Autonomous navigation

### Environmental Sensors

The platform also incorporates environmental sensing for:

* 🌡️ Temperature
* 💧 Moisture
* 🌫️ CO₂ using NDIR sensing

---

## 🖨️ Fabrication

Novexa was designed as a modular, cost-efficient robotic platform using additive manufacturing.

### Printing Parameters

```text
Material       : PLA
Nozzle         : 0.2 mm
Layer Height   : 0.2 mm
Infill         : 20–40%
Pattern        : Gyroid
Print Time     : 70+ hours
Filament       : ~2.0–2.5 kg
Fasteners      : M2 screws + copper inserts
```

TPU is used for the tibia grippers to provide better contact with the ground.

---

## 🛠️ Design Philosophy

Novexa focuses on combining:

**Mechanical Design**

→ 3D-printed modular structure

**Kinematics**

→ Forward and inverse kinematics

**Locomotion**

→ Multiple gait algorithms

**Perception**

→ IMU + ultrasonic + LiDAR

**Control**

→ PWM + gait engine + PID stabilization

**Autonomy**

→ Obstacle detection + mapping + path planning

This makes Novexa more than a simple servo-driven walking robot; it is intended as a platform for experimenting with **adaptive locomotion and autonomous robotics**.

---

## 🗺️ Roadmap

The current prototype uses an **Arduino Mega** as the primary controller. A future stable version is planned around an **STM32** platform.

Planned development includes:

* [ ] STM32 controller upgrade
* [ ] ROS2 integration
* [ ] Autonomous terrain mapping
* [ ] Neural gait optimization using TinyML
* [ ] Higher-torque and more stable servos
* [ ] Modular environmental sensor pod
* [ ] Solar-assisted charging
* [ ] Detachable leg mechanism
* [ ] Improved autonomous navigation

---

## 📊 Quick Specifications

```text
                    NOVEXA
                       │
        ┌──────────────┴──────────────┐
        │                             │
     6 LEGS                       18 DOF
        │                             │
   3 DOF / LEG                 18 × MG995
        │                             │
        └──────────────┬──────────────┘
                       │
                 ARDUINO MEGA
                       │
                  PCA9685 PWM
                       │
          ┌────────────┼────────────┐
          │            │            │
         IMU       ULTRASONIC     LiDAR
       MPU6050       HC-SR04      TF03
          │            │            │
          └────────────┴────────────┘
                       │
              ADAPTIVE LOCOMOTION
```

---

## 📄 Research Paper

Novexa's design, fabrication, kinematic model, gait strategies, electronics architecture, and future development are documented in the research paper:

**"Fabrication of a 3D Printed Hexapod for All Terrain Adaptive Locomotion"**

Published in the *International Journal of Innovative Research in Technology (IJIRT)*, April 2026.

The paper describes Novexa as an experimental platform for multi-terrain adaptive locomotion and documents the 18-DOF mechanical architecture, fabrication process, inverse kinematics, gait planning, electronics, sensor integration, and future development.

---

## 👥 Authors

**Bilal Sabugar**
**Aditya Jadav**
**Archie Veera**
**Rabinder Henry**

B.Tech — AI & Robotics
**Atlas SkillTech University, Mumbai, India**

---

## 🏫 Acknowledgements

Developed with the resources and laboratory facilities provided by **Atlas SkillTech University**.

---

## 📜 License

License information for this repository should be added here once the project's intended license is finalized.

