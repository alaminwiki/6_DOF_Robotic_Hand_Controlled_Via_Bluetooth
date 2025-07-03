# 🚀 **6-DOF Robotic Hand Controlled via Bluetooth | STM32 & PCA9685**

This project demonstrates a **6-degree-of-freedom (6-DOF) robotic hand** controlled by the **STM32 Blue Pill** microcontroller, with precision movement powered by **PCA9685** and wirelessly operated via **Bluetooth** (HC-06 module).

The robotic hand features **10-degree precision control**, providing smooth and accurate motions with real-time feedback. This project is ideal for those interested in robotics, embedded systems, automation, and wireless control systems.

---


## **🔹 Project Highlights:**

- **STM32 Blue Pill** for real-time processing and control
- **PCA9685** for precise control of 6 servo motors using PWM
- **HC-06 Bluetooth module** for seamless wireless control via UART
- **6x Servo Motors** for smooth, articulated movement of the robotic fingers and wrist
- **I²C Communication** to interface with the PCA9685 servo driver
- **UART Communication** for receiving commands from the Bluetooth module
- **10-degree precision** for fine motor control and accurate motion

---

## **🔹 Features & Capabilities:**

- **Precision Control**: 10-degree precision for smooth and accurate servo movements.
- **Wireless Operation**: Real-time control via Bluetooth, enabling remote operation of the robotic hand.
- **Customizable**: Can be adapted for various robotic applications like assistive devices and research.
- **Scalable Design**: Future versions can include more DOF or additional sensors for improved feedback.

---

## **🔹 Key Learnings & Future Improvements:**

- **Motion Optimization**: Working on refining motion algorithms for smoother, more natural movements.
- **Grip Strength and Dexterity**: Future enhancements will focus on improving grip strength and finger dexterity for more complex tasks.
- **Gesture Recognition**: Exploring the possibility of adding AI-based gesture recognition for more intuitive control.

---

## **🔹 Getting Started:**

### **Prerequisites:**

- **STM32 Blue Pill** microcontroller (STM32F103C8T6)
- **PCA9685** Servo Driver (16-channel PWM)
- **HC-06 Bluetooth Module** for wireless communication
- **6x Servo Motors**
- **Wiring/Power Supply** (External 5V for servos)
- **STM32CubeIDE** for programming the STM32

---

### **Installation:**

1. **Configure STM32 with STM32CubeIDE:**
   - Open the project in STM32CubeIDE and configure the I²C and UART peripherals.
   - Load the firmware onto your STM32 Blue Pill via USB.


2. **Servo Calibration:**
   - Power the servos using an external 5V supply.
   - Calibrate the servo motors for accurate movement.

---

### **Usage:**

- Control the robotic hand by sending commands via Bluetooth to the HC-06 module.
- The STM32 will process the data and adjust the servos accordingly to move the fingers and wrist.
- Real-time control allows for smooth and accurate finger articulation for various robotic tasks.

---

## **🔹 Contribution:**

Feel free to fork this repository, contribute, and submit pull requests. Contributions can include code enhancements, adding new features, or improving the motion control algorithms.

---

## **🔹 Keywords:**

Robotics, STM32, PCA9685, Bluetooth, Embedded Systems, Automation, I²C, UART, Servo Control, Gesture Recognition, Precision Control.

---

Let me know if you need further edits or additions to the README! 🚀

Video Of this Project :

https://youtu.be/7FjJbYT7xIs?si=uYSMJaefywGDw0Jh


https://youtu.be/2hPKrFhMjZ0?si=tDs_dVr8lwox4Dy9


https://youtu.be/ZAh5dZRcjxQ?si=tyGtm_TFnVypO4Wn

