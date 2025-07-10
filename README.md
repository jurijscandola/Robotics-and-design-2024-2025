# 🤖 Skipy: The Microwave Queue Manager Social Robot
---

## 🎓 Project Context

This project was developed for the **XIII edition (2025)** of the **"Design and Robotics" interdisciplinary course** at **Politecnico di Milano**. The initiative promoted hands-on, team-based development with a strong focus on **iterative prototyping** and **cross-disciplinary integration**.

## 📌 Summary

**Skipy** is a **social and autonomous robot** developed to efficiently manage microwave queues in a university setting. By dispensing **numbered tickets** and offering a **lottery-based "skip the line" system**, it enhances user experience during peak lunch hours. Its modular architecture ensures scalability, robust navigation, and rich user interaction through lights, movement, and sound.

---

## 🧩 Modular Design & Communication

Skipy is built on a **modular architecture** with five independent units connected via the **I2C communication protocol**, making the system easier to develop, test, and maintain:

- **🗣️ Communication Module**  
  Engages users using **movement, lights, and sounds** to foster an interactive experience.

- **🖨️ Actuator 1 (Ticket Dispenser)**  
  Prints and deploys **numbered or "lucky" tickets** using a custom linear deployment system.

- **🎱 Actuator 2 (Lottery System)**  
  Executes a **lottery** by propelling balls through airflow in a transparent chamber, with **color detection** for ball validation.

- **🧭 Movement & Localization Module**  
  Provides **omnidirectional motion** and **basic obstacle detection**, enabling Skipy to autonomously navigate its environment.

- **🔋 Power Supply Module**  
  Manages energy usage and supports **autonomous recharging** via a custom docking station.

---

## 🧠 Movement & Localization Highlights

Skipy's movement system is engineered for **full planar mobility** using an **omnidirectional three-wheel configuration**.

### 🔧 Key Components

- **DC Motors:**  
  Three JGB37-520 Encoder Hall DC motors controlled via BTS7960 H-bridges for high-torque, precise movement.

- **Omni Wheels:**  
  58mm nylon omni wheels enable **holonomic motion** in any direction.

- **Ultrasonic Sensors (HC-SR04):**  
  Five sensors for **real-time obstacle detection**.

- **Camera Module:**  
  Used for **QR code detection**, enabling **autonomous docking** at the charging station.

- **Microcontrollers:**  
  - **Arduino Uno (ATmega328P):** Core of the movement system.  
  - **Raspberry Pi:** Vision and AprilTag-based localization.  
  - **(Optional) Arduino Mega 2560:** Considered for extended I/O support.

### 📍 Localization & Control Architecture

- **Vision-based Localization:**  
  Uses **AprilTags or QR codes** on the charging station for precise docking.

- **Sensor Fusion:**  
  Combines inputs from **wheel encoders**, **IMU (MPU6050)**, and **visual markers** for robust position tracking (±5 mm accuracy).

- **Control Strategy:**  
  - **State-driven architecture**  
  - **Non-blocking programming**  
  - Supports **manual**, **autonomous**, **tag-following**, and **charging modes**

---

## ⚙️ Performance & Challenges

- **🔝 Max Speed:** 0.5 m/s (safe operation)
- **📏 Obstacle Distance:** Minimum 15 cm
- **🔋 Battery Life:** 3+ hours of continuous usage

### 🧪 Main Challenges Faced

- **Sensor Limitations:** Narrow angle and reflectivity issues with ultrasonic sensors.
- **Motor Sync Issues:** Difficulty in synchronizing wheel speeds during directional transitions.
- **Surface Dependency:** Performance of omni wheels varied with surface texture.
- **Voltage Sag:** Motor load occasionally disrupted sensor reliability.

---

> _Skipy brings order to chaos — one lunch break at a time._
