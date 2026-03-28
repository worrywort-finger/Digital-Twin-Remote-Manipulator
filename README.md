# Real-Time Cyber-Physical System: Bidirectional Digital Twin

## 📌 Project Overview
This project implements a low-latency, real-time Cyber-Physical System (CPS). It utilizes computer vision to capture human spatial coordinates and broadcasts them over a local UDP network to simultaneously actuate a virtual 3D model in Blender and a physical 2-axis robotic pan-tilt mechanism using an ESP32 microcontroller.

### Key Features
* **Zero-Latency Teleoperation:** Uses asynchronous UDP sockets to prioritize speed over packet verification, ensuring real-time hardware tracking.
* **Kinematic Safety Clutch:** Features a heuristic gesture recognition system (Open Hand = Active, Closed Fist = Paused) to safely disengage the physical hardware without interrupting the software.
* **Concurrent Actuation:** Utilizes the ESP32's 32-bit hardware timers (LEDC/MCPWM) for synchronous, multi-axis motor control without blocking the main execution loop.

---

## ⚙️ System Architecture
1.  **The Perception Layer (`hand_tracker.py`):** Uses OpenCV and Google MediaPipe to track hand landmarks, calculate inverse kinematics, and transmit dual-cast UDP packets.
2.  **The Virtual Receiver (`blender_python_script.txt` & `pan_tilt.blend`):** A non-blocking Python script running inside Blender that manipulates the Euler rotation of the 3D meshes based on the incoming data stream.
3.  **The Physical Receiver (`sketch_mar27a.ino`):** ESP32 C++ firmware that parses the network strings and drives two SG90 servo motors to mirror the human operator.

---

## 🛠️ Hardware Requirements & Wiring
* **Microcontroller:** ESP32 Development Board
* **Actuators:** 2x SG90 Micro Servos (Pan & Tilt)
* **Power Supply:** Isolated 5V power source (e.g., dedicated 5V 2A USB wall charger). *Do not power both servos directly from the ESP32 5V pin, or you will trigger a Brown-Out Detector (BOD) reset.*

### Wiring Schematic
| Component | Wire Color | Connection |
| :--- | :--- | :--- |
| **Pan Servo** | Signal (Yellow) | ESP32 GPIO 18 |
| **Tilt Servo** | Signal (Yellow) | ESP32 GPIO 19 |
| **Both Servos** | VCC (Red) | External 5V Power Supply (+) |
| **Both Servos** | GND (Brown/Black) | External Power Supply (-) **AND** ESP32 GND |

> **⚠️ CRITICAL:** You must establish a Common Ground. Connect a jumper wire from the external power supply's ground to the ESP32's GND pin, or the PWM signal will float and the motors will jitter violently.

---

## 💻 Software Prerequisites
1.  **Python 3.8+** (with `opencv-python version 4.13.0.92` and `mediapipe version 0.10.9` installed).
2.  **Arduino IDE** (Configured with the ESP32 Board Manager).
3.  **ESP32Servo Library** (by Kevin Harrington, installed via Arduino Library Manager).
4.  **Blender 3.0+** (For the virtual twin).

---

### Environment Setup
To ensure this project runs perfectly on your machine without conflicting with your existing Python packages, please follow these steps to set up a virtual environment.

### Step 1: Clone the Repository
First, download the project files to your local machine:
```bash
git clone https://github.com/worrywort-finger/Digital-Twin-Remote-Manipulator.git
cd https://github.com/worrywort-finger/Digital-Twin-Remote-Manipulator.git
```
### Step 2: Create a Virtual Environment

We strongly recommend using a virtual environment (venv) to keep the dependencies isolated. Run the following command in your terminal inside the project folder:

```bash
python -m venv venv
```
### Step 3: Activate the Virtual Environment

You must activate the environment before installing any packages or running the scripts.

    On Windows:
    .\venv\Scripts\activate

    On macOS and Linux:
    source venv/bin/activate

### Step 4: Install Dependencies

With the virtual environment active, install the exact package versions required for the kinematic math and computer vision to function correctly:
```bash
install the exact packages as per the requirements.txt
```


## 🚀 Installation & Execution Protocol

### Step 1: Flash the Physical Twin (ESP32)
1. Open `sketch_mar27a.ino` in the Arduino IDE.
2. Update the network credentials:
```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
```
3. Compile and upload to the ESP32.
4. Open the Serial Monitor (**115200 baud**) and press the physical `EN` button on the board.
5. Note the assigned IP Address printed in the console.

### Step 2: Initialize the Virtual Twin (Blender)

1. Open `pan_tilt.blend`.
2. Navigate to the **Scripting** workspace.
3. Paste the contents of `blender_python_script.txt` into the text editor.
4. Click **Run Script** (The play button). The background listener is now active on port 9000.

### Step 3: Launch the Perception Layer (Python)

1. Open `hand_tracker.py` in your preferred code editor.
2. Update the `ESP32_IP` variable on line 12 with the IP address retrieved in Step 1:
```py
ESP32_IP = "192.168.1.X" # Replace with your ESP32's IP
```
3. Run the script: `python hand_tracker.py`

## 🎮 Operation & Usage

1. Position yourself in front of the webcam.
2. **Engage System:** Hold up an open hand. The HUD will display `STATE: ACTIVE`. Move your hand across the X and Y axes; both the virtual Blender model and the physical servos will track the movement.
3. **Disengage System (Clutch):** Close your hand into a fist. The HUD will display `STATE: PAUSED`. The physical hardware will lock in its current position until the hand is opened again.

## 🔧 Troubleshooting

- **Motors violently jittering or ESP32 looping a reboot:** You are experiencing a hardware brown-out. Ensure you are using an external 5V power supply for the servos and that the common ground is firmly connected.
- **Blender model isn't moving:** Ensure the object names in the Blender Outliner exactly match the names defined in the `blender_python_script.txt` (`Pan_Bracket` and `Tilt_Bracket`).
- **Compilation Error (`Servo.h` not found):** The standard Arduino servo library is incompatible with ESP32 architecture. Ensure you have installed the specific `ESP32Servo` library via the Library Manager.
