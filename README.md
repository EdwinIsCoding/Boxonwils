**Boxonwils - Year 1, Term 3 Robotics Challenge – README**
<br>
*(A complete how-to guide for using the Arduino and Python scripts with the robot)*

---

## Table of Contents

1. [Introduction](#introduction)
2. [Hardware Overview](#hardware-overview)
3. [Software Overview](#software-overview)
4. [Prerequisites](#prerequisites)

   * [Arduino IDE](#arduino-ide)
   * [Python (3.6+)](#python-36)
   * [Libraries & Drivers](#libraries--drivers)
5. [Wiring & Connections](#wiring--connections)

   * [Motoron I²C Controllers](#motoron-i%C2%B2c-controllers)
   * [Reflectance Sensors](#reflectance-sensors)
   * [Ultrasonic (Sonar) Sensors](#ultrasonic-sonar-sensors)
   * [Servos](#servos)
   * [Photoresistor](#photoresistor)
   * [Kill Switch](#kill-switch)
6. [Configuring & Uploading `boxonwils.ino`](#configuring--uploading-boxonwilsino)

   * [Step 1: Clone or Download Repository](#step-1-clone-or-download-repository)
   * [Step 2: Install Arduino Libraries](#step-2-install-arduino-libraries)
   * [Step 3: Adjust I²C Addresses (if needed)](#step-3-adjust-i%C2%B2c-addresses-if-needed)
   * [Step 4: Set Wi-Fi Credentials & Ports](#step-4-set-wi%E2%80%91fi-credentials--ports)
   * [Step 5: Verify Pin-Mapping Matches Your Robot](#step-5-verify-pin%E2%80%91mapping-matches-your-robot)
   * [Step 6: Upload to Arduino Giga](#step-6-upload-to-arduino-giga)
7. [Runtime Behavior & Modes](#runtime-behavior--modes)

   * [1. Kill Switch (Physical & Remote)](#1-kill-switch-physical--remote)
   * [2. Line-Following (Section 1)](#2-section-1)
   * [3. Wall-Following (Section 2)](#3-section-2)
   * [4. More Line-Following with Pits and Ziplines (Section 3)](#4-section-3)
   * [5. Debugging & Print Flags](#5-debugging--print-flags)
8. [Using `parameter_tune_and_kill.py`](#using-parameter_tune_and_killpy)

   * [Step 1: Edit `UDP_IP` & `UDP_PORT`](#step-1-edit-udp_ip--udp_port)
   * [Step 2: Install Dependencies](#step-2-install-dependencies)
   * [Step 3: Run the Script & Available Commands](#step-3-run-the-script--available-commands)
   * [Step 4: Tuning PID, Speeds, & Thresholds](#step-4-tuning-pid-speeds--thresholds)
   * [Step 5: Sending Kill & Mode Commands](#step-5-sending-kill--mode-commands)
9. [Tips & Troubleshooting](#tips--troubleshooting)
10. [License & Credits](#license--credits)

---

## Introduction

Welcome to the **Boxonwils Robotics Challenge** project! This repository contains all the code needed to run an Arduino Giga–based robot capable of line-following, wall-following, and completing challenge tasks in a modular “section” format. The core firmware (`boxonwils.ino`) handles sensor readings, PID loops, motor control, and “section logic.” A companion Python script (`parameter_tune_and_kill.py`) allows remote adjustments of PID constants, speed, thresholds, and even a remote kill switch over UDP.

This README will guide you—step by step—through wiring, configuration, building, uploading, and running everything. The system is mostly plug-and-play; the only adjustments you typically need to make are ensuring I²C motor-driver addresses align with your hardware and that your Wi-Fi credentials match.

---

## Hardware Overview

1. **Arduino Giga (ESP32-based)** – runs the main `boxonwils.ino` firmware.
2. **Two Motoron I²C Motor Controllers (e.g., Motoron 24v15)** – drive four DC motors in pairs (front pair on one controller, rear pair on the other).
3. **Line-Reflectance Sensor Array (11 sensors)** – used for line detection (e.g., QTR-X reflectance array).
4. **Ultrasonic (Sonar) Sensors (HC-SR04 style)** – three directions: two on the left, one front, one right.
5. **Three Hobby Servos** – to manipulate a front claw, a back claw, and an L-bracket for attaching to tasks.
6. **Photoresistor (Light Sensor)** – to detect ambient light (optional for advanced modes, not used by default logic).
7. **Physical Kill Switch (Pushbutton)** – wired to pin 10 on Arduino for manual “stop”/“start.”
8. **Wi-Fi Network** – used for remote UDP commands (tuning or remote kill).

---

## Software Overview

* **`boxonwils.ino` (Arduino Sketch)**
  • Handles setup of I²C motor controllers, sensor inputs, servos, and Wi-Fi.
  • Implements multiple PID routines: line-following (11 reflectance sensors) and wall-following (left-side sonars, alignment).
  • Divides the course into sequential “sections” identified by `lostLineCounter`:
  – Section 1: line segment with 90° bend (line excluded at ends triggers logic)
  – Section 2: left-wall navigation using two left sonars, plus a fixed 90° turn when front obstacle is too close
  – Section 3: line present but need to deploy/retract bracket/claws at specific times to go over a pit and down a zipline at full-speed
  • Responds to UDP commands for: Stop, LINE_PID, WALL_PID, BASE_SPEED, TURN_DURATION, LINE_THRESHOLD, TOGGLE_PRINT_FLAGS.
* **`parameter_tune_and_kill.py` (Python 3 Script)**
  • Opens a UDP socket to send commands to the Arduino (same port `55500`).
  • Provides an interactive CLI to:
  – Send a “Stop” (remote kill) to Arduino
  – Send full or single-parameter PID adjustments for line-following or wall-following
  – Send `BASE_SPEED`, `TURN_DURATION`, `LINE_THRESHOLD`, or toggle print flags
  – Choose from 6 predefined “modes” (combinations of PID + speed presets)
  • Note: Requires editing of `UDP_IP` to match the Arduino’s IP once it’s on the network.

---

## Prerequisites

### Arduino IDE

* Download and install the [Arduino IDE (1.8.19+)](https://www.arduino.cc/).
* Configure it for the **ESP32** (Arduino Giga). In the Boards Manager, install "ESP32 by Espressif Systems."

### Python (3.6+)

* Install Python 3.6 or newer (e.g., [python.org](https://www.python.org/) or via your package manager).
* Verify with:

  ```bash
  python3 --version
  ```

### Libraries & Drivers

1. **Motoron Library** (for I²C motor controllers)
   • In the Arduino IDE, go to **Sketch → Include Library → Manage Libraries…**
   • Search for **Motoron** and install the official Motoron library (e.g., `Motoron by Pololu`).
2. **WiFi.h**, **WiFiUdp.h** (ESP32 core libraries; installed automatically when you install “ESP32 by Espressif”).
3. **Wire.h** (built-in). Note that `Wire1.begin()` is used because the Motoron controllers run on I²C port 1.
4. **Servo.h** (built-in).
5. **Python Dependencies** (no external pip packages are required for `parameter_tune_and_kill.py`; it uses only the standard `socket` module).

---

## Wiring & Connections

> **Tip:** Label each connector clearly, especially the I²C addresses on the Motoron boards, so you know which controller is front vs. back.

> **Warning:** Double-check you do not exceed 5 V on any servo or sensor pins. All signal lines are 5 V, but the Arduino Giga’s I/O is 3.3 V–tolerant. Servos usually work at 5 V logic.

### Motoron I²C Controllers

* **MC1 (Front Motors)** – I²C address `0x10` by default
  • Connect to Arduino **SCL1** and **SDA1** (these map to `Wire1` on Arduino Giga).
  • Power: 10.9-12 V supply shared by four motors (common ground with Arduino).
  • Motors: Channel 1 = Front left, Channel 3 = Front right.
* **MC2 (Back Motors)** – I²C address `0x11` by default
  • Also connect to the same `Wire1` bus—only one pair of SCL1/SDA1 lines, but each Motoron has a unique I²C address.
  • Motors: Channel 1 = Back left, Channel 3 = Back right.

> **Changing Addresses**:
> • If your Motoron boards have different DIP‐switch or solder‐bridge addresses, update these lines in `boxonwils.ino` before uploading:
>
> ```cpp
> MotoronI2C mc1(0x10); // change 0x10 to your front controller’s address  
> MotoronI2C mc2(0x11); // change 0x11 to your back controller’s address  
> ```

### Reflectance Sensors (11 Channels)

* Use an 11-sensor line array (e.g., QTRX-TC11). Each sensor’s **OUT** pin connects to one of these digital‐read pins on Arduino Giga:

  ```
  int sensorPins[11] = {31, 41, 49, 39, 47, 37, 45, 35, 43, 33, 51};
  ```
* Underneath, `readReflectanceSensors()` drives them HIGH for 10 μs, then reads `LOW` transition times to gauge reflectance.
* Make sure the board’s pull-ups (or external pull-ups) are present if required by your sensor array.

### Ultrasonic (Sonar) Sensors

You have four HC-SR04 style sensors wired as:

* **Left 1**: `trigL1 → pin 5`, `echoL1 → pin 4`
* **Left 2**: `trigL2 → pin 9`, `echoL2 → pin 8`
* **Front**: `trigF → pin 38`, `echoF → pin 36`
* **Right**: `trigR → pin 52`, `echoR → pin 50`

> Filtering:
> • Left sensors are filtered via a simple low-pass (α = 0.2), and their average drives the wall-following PID.
> • Front is read raw (no filtering) to detect immediate obstacles.
> • Right is also filtered for potential alignment or future use.

### Servos

Three hobby servos (5 V power, signal to 3.3 V Arduino through logic-level SAFE wiring):

* **Front Claw Servo** – signal to pin 10
* **Back Claw Servo** – signal to pin 11
* **L-Bracket Servo** – signal to pin 12

Servo “start” vs. “final” angles are:

```cpp
const int frontS = 20;  // front-claw start 
const int frontF = 90;  // front-claw deployed

const int backS  = 17;  // back-claw start
const int backF  = 125; // back-claw deployed

const int bracketS = 25;  // L-bracket start
const int bracketF = 65;  // L-bracket deployed
```

### Photoresistor

* **Photoresistor** (optional light sensor) is wired to analog pin `A5`. In the default code, it is read into `photoPin = A5`, but not used in the main logic. You can use it to detect ambient light or as a trigger for specific “dark” modes.

### Kill Switch

* A momentary **pushbutton** is wired between **pin 10** and **GND**, with the internal pull-up (`INPUT_PULLUP`) enabled.
* Pressing the button toggles `killSwitchActivated`. When NOT activated (or after a remote UDP “Stop”), the robot halts motors and servos.

---

## Configuring & Uploading `boxonwils.ino`

### Step 1: Clone or Download Repository

```bash
git clone https://github.com/yourusername/boxonwils-challenge.git
cd boxonwils-challenge
```

*or* download as a ZIP from GitHub and extract.

### Step 2: Install Arduino Libraries

1. Open **Arduino IDE**.
2. Go to **Sketch → Include Library → Manage Libraries…**
3. Search for and install:

* **Motoron** (by Pololu or the version compatible with your controllers)
* **ESP32 Board Support** (“ESP32 by Espressif Systems”) if not already installed.

> **Verify**: After installation, you should see `#include <Motoron.h>` recognized with no errors.

### Step 3: Adjust I²C Addresses (if needed)

Open `boxonwils.ino` and locate these lines near the top:

```cpp
MotoronI2C mc1(0x10); // Front 
MotoronI2C mc2(0x11); // Back
```

Change `0x10`/`0x11` to match the I²C addresses of your front/back Motoron controllers. If both default to `0x10`, adjust one via DIP switches or solder pads so they differ by 1 (e.g., `0x10` and `0x11`).

### Step 4: Set Wi-Fi Credentials & Ports

In `boxonwils.ino`, find:

```cpp
const char ssid[]     = "PhaseSpaceNetwork_2.4G";
const char password[] = "8igMacNet";
IPAddress ip; 
WiFiUDP Udp;
const unsigned int localPort = 55500;
```

* Replace `"PhaseSpaceNetwork_2.4G"` and `"8igMacNet"` with your actual SSID & password.
* If you need a static IP, set `ip = IPAddress(x, x, x, x);` before `WiFi.begin(…)`; otherwise, the robot requests DHCP.
* **Note**: UDP port `55500` must be open/forwarded if you’re on a strict network. Typically a local network is fine.

### Step 5: Verify Pin-Mapping Matches Your Robot

Double-check that the digital and analog pins in `boxonwils.ino` match your wiring, especially:

```cpp
const int killPin = 10;
const int sensorPins[11] = {31,41,49,39,47,37,45,35,43,33,51};
// Sonar pins:
const int trigL1 = 5, echoL1 = 4, trigL2 = 9, echoL2 = 8;
const int trigF  = 38, echoF  = 36;
const int trigR  = 52, echoR  = 50;
// Servo pins in setup():
front.attach(10);
back.attach(11);
bracket.attach(12);
// Photoresistor:
const int photoPin = A5;
```

Modify any pins here if your hardware uses different pins.

### Step 6: Upload to Arduino Giga

1. Plug in your Arduino Giga via USB.
2. In the Arduino IDE, select **Tools → Board → “ESP32 Arduino → Arduino Giga”** (or the correct ESP32 variant).
3. Select the correct **Port** (e.g., `COM5` on Windows, `/dev/ttyACM0` on Linux).
4. Click the **Upload** (→) button.
5. Open **Serial Monitor** (Baud = 9600). You should see:

   ```
   ........
   WiFi connected.
   IP: 192.168.0.113
   ```

   (dots appear while trying to connect; then a final IP address).

---

## Runtime Behavior & Modes

### 1. Kill Switch (Physical & Remote)

* **Physical Kill Switch** (pin 10):
  • At power-up, `killSwitchActivated` is `false`—motors/servos remain OFF until you press the button once.
  • Press button again to **pause** the robot (“OFF”). Press again to **resume** (“ON”).
  • After a remote UDP “Stop” (see below), the first button press **resets** the system (clears `udpKilled`) and sets `killSwitchActivated = true`.
* **Remote Kill (UDP)**:
  • Sending **`Stop`** over UDP sets `udpKilled = true` and `killSwitchActivated = false`.
  • In this state, the only way to resume is to toggle the physical switch once to reset.

### 2. Line-Following (Section 1)

* Uses 11 reflectance sensors to follow a black line on a light surface.
* **Threshold**: `lineThreshold = 300` by default (units = microseconds from the reflectance discharge).
  • You can change via UDP: `"LINE_THRESHOLD,<newValue>"`.
* **PID Constants** (default):

  ```cpp
  float lineKp = 6, lineKi = 0, lineKd = 0.1;
  ```

  • Change via UDP: `"LINE_PID,<Kp>,<Ki>,<Kd>"`.
* **Base Speed**: `baseSpeed = 600` (range 0–800).
  • Adjust via UDP: `"BASE_SPEED,<0–800>"`.
* **Behaviour**:
  • In every `loop()`, if `lostLineCounter < 2`, it calls `followLine()`.
  • Inside `followLine()`:
  – Reads sensors, computes “line error” via weighted sum of reflectance times.
  – Applies PID to compute `correction`.
  – Sets left/right motor speeds via `applyMotorSpeeds(correction)`.
  – If **allSensorsOff()** (line lost), it increments `lostLineCounter`, then does a controlled right-turn until central sensors reacquire line (`centralTwoOfThreeOn()`). That handles 90° intersections.
  – Once line is reacquired, it stops, does a short delay, and resumes following.
* **lostLineCounter** starts at 0. At the first 90° turn, it becomes 1; after the second, it becomes 2—triggering Section 2 logic.

### 3. Wall-Following (Section 2)

* Engaged once `lostLineCounter == 2`. On that one iteration, `retractClaw_Bracket(bracketS, bracket)` retracts the L-bracket for clearance and increments `lostLineCounter` to 3 so you don’t re-enter.
* From then until Section 3 begins, `navWall()` is called when `distL1 < 20 cm`.
  • `distL1` is the filtered reading from left-side sensors (via `readSonarFiltered`).
  • If **front obstacle** closer than 18 cm, executes a fixed 90° right-turn via `turn90("RIGHT", turnDuration)` (default `turnDuration = 2400` ms).
  • Otherwise, it calls `followLeftWall()`:
  – Maintains `targetDistance = 8.0 cm` from the left wall (average of `filteredLeft1` & `filteredLeft2`).
  – `distanceError = targetDistance – averageLeft`.
  – `alignmentError = filteredLeft1 – filteredLeft2`.
  – PID uses these constants (default):

  ```cpp
  float wallKp = 100, wallKi = 0, wallKd = 5, wallKa = 60;
  ```

  – Control output is `pidOutput + alignmentCorrection`, where `alignmentCorrection = wallKa * alignmentError`. 
  – Motor speeds are set as `leftSpeed = baseSpeed + controlOutput`, `rightSpeed = baseSpeed – controlOutput`; both clamped to [–800, 800].
  • You can change wall-following PID via UDP `"WALL_PID,<P>,<I>,<D>,<A>"` (where `<A>` = wall-alignment factor).
  • You can adjust `turnDuration` via `"TURN_DURATION,<milliseconds>"`.

### 4. More Line-Following with Pits and Ziplines (Section 3)

•⁠  ⁠Once ⁠ distL1 > 20 cm ⁠ *and* ⁠ lostLineCounter == 3 ⁠, this indicates the robot has cleared the wall section and come up a ramp.
  • On that frame:
  – ⁠ deployClaw_Bracket(bracketF, bracket) ⁠ moves the L-bracket into position.  
  – Short delay (400 ms).  
  – ⁠ lostLineCounter ⁠ increments to 4 so you don’t re-enter this branch.

•⁠  ⁠Subsequent ⁠ loop() ⁠ calls (with ⁠ lostLineCounter >= 4 ⁠ but ⁠ < 7 ⁠) resume line-following.
  • As the robot follows the line on Section 3, ⁠ lostLineCounter ⁠ eventually reaches 7 or 8 once it reaches the end line or intersection before the pit/zipline.

•⁠  ⁠*Pit/Ziplines Logic*
  * Condition:  
    - ⁠ lostLineCounter == 7 ⁠ *OR*  
    - ⁠ lostLineCounter == 8 && allSensorsOn() ⁠  
    (i.e., a solid black line across the track indicates the pit/zipline entry).

  • On that pass:
  1. ⁠ deployClaw_Bracket(frontF, front) ⁠ → drops front claw (400 ms delay).  
  2. ⁠ deployClaw_Bracket(backF, back) ⁠ → drops back claw (400 ms delay).  
  3. ⁠ fullSend() ⁠ (sets all motors to ⁠ maxSpeed = 800 ⁠) → dash across pit or descend zipline.  
  4. ⁠ delay(3000) ⁠ to allow crossing (3 seconds).  
  5. Retract claws:  
     – ⁠ retractClaw_Bracket(frontS, front) ⁠ (400 ms delay).  
     – ⁠ retractClaw_Bracket(backS, back) ⁠ (400 ms delay).  
  6. ⁠ stopMotors() ⁠ → halts (200 ms delay).  
  7. ⁠ lostLineCounter++ ⁠ → set to 8 or 9, avoiding re-entry.

•⁠  ⁠After this, if ⁠ lostLineCounter >= 9 ⁠, the robot simply stops (⁠ stopMotors() ⁠), having completed all three sections.

### 6. Debugging & Print Flags

* By default, `printFlag = false`. If set to `true`, the firmware prints:
  • Reflectance sensor arrays (raw & binary).
  • Current sonar distances (Left1, Left2, Front, Right).
  • Left/right motor speeds every loop.
* Toggle via UDP: send `"TOGGLE_PRINT_FLAGS"`.
* Example serial output when debugging is on:

  ```
  Reflectance array: 123 456 789 … 
  1 1 0 0 1  … 
  Left 1: 8.2 cm | Left 2: 8.5 cm | Front: 15 cm | Right: 20 cm
  550 - 650
  ```

---

## Using `parameter_tune_and_kill.py`

This Python script gives you a convenient terminal interface to adjust PID constants, base speed, line threshold, turn duration, and send a remote kill. All commands are sent via UDP to the Arduino’s IP.

### Step 1: Edit `UDP_IP` & `UDP_PORT`

Open `parameter_tune_and_kill.py` in a text editor. At top:

```python
UDP_IP = "192.168.0.113"  # change to whatever IP appears in Arduino Serial Monitor
UDP_PORT = 55500
```

1. After uploading `boxonwils.ino` to Arduino, open Serial Monitor and note the IP (e.g., `192.168.0.113`).
2. Replace `UDP_IP` in the script with that exact IP. Keep `UDP_PORT = 55500` (unless you change the port in the Arduino code—then match it here).

### Step 2: Install Dependencies

No extra pip packages are needed. It uses only Python’s built-in `socket` library. Make sure you have Python 3 installed:

```bash
python3 --version
```

### Step 3: Run the Script & Available Commands

In a terminal (macOS/Linux) or command prompt (Windows), navigate to the project folder and run:

```bash
python3 parameter_tune_and_kill.py
```

You’ll see:

```
UDP Command Interface Running
Press:
  'k'  - Send Kill message
  'w'  - Set Wall-Following PID values
  'l'  - Set Line-Following PID values
  'wp', 'wi', 'wd' - Set P/I/D of Wall-Following controller
  'lp', 'li', 'ld' - Set P/I/D of Line-Following controller
  'bs' - Set Base Speed (0–800)
  'td' - Set Turn Duration (in milliseconds)
  'lth' - Set Line Threshold
  'tp' - Toggle Print Flags on Arduino
  '1' to '6' - Set predefined modes with custom PID values
  'q'  - Quit the program
```

At the `Enter command:` prompt, type one of the single keys or strings (e.g., `k`, `l`, `w`, `bs`, `1`, etc.) and press **Enter**. Follow any sub-prompts.

### Step 4: Tuning PID, Speeds, & Thresholds

* **Full PID for Line-Following**:

  ```
  Enter command: l  
  Enter PID values for Line-Following:  
    Enter P: 8  
    Enter I: 0  
    Enter D: 0.2  
  ```

  Sends `"LINE_PID,8,0,0.2"`.
* **Full PID for Wall-Following**:

  ```
  Enter command: w  
  Enter PID values for Wall-Following:  
    Enter P: 120  
    Enter I: 0  
    Enter D: 7  
    Enter A: 50  
  ```

  Sends `"WALL_PID,120,0,7,50"`.
* **Single Parameter**:
  – `wp` → change only **P** for wall: script asks `Enter new P value for WALL-Following …`, then sends `"WALLSETP,<value>"`.
  – `ld` → change **D** for line: sends `"LINESETD,<value>"`.
* **Base Speed**:
  – Type `bs`, then input an integer `0–800`. Sends `"BASE_SPEED,<speed>"`.
* **Turn Duration**:
  – Type `td`, input a millisecond value. Sends `"TURN_DURATION,<ms>"`.
* **Line Threshold**:
  – Type `lth`, input a value (e.g., `350`). Sends `"LINE_THRESHOLD,350"`.
* **Toggle Print Flags**:
  – Type `tp`. Sends `"TOGGLE_PRINT_FLAGS"`.

### Step 5: Sending Kill & Mode Commands

* **Kill** (remote stop): type `k`. Arduino prints

  ```
  🔴 REMOTE KILL RECEIVED — SYSTEM STOPPED 🔴
  ```

  and halts. To reset, press the physical kill button once (it will “Resetting system after remote kill.” and resume).
* **Predefined Modes**: type digits `1`–`6`:

  1. Mode 1: BaseSpeed 400, Line PID = (1, 0, 0.1)
  2. Mode 2: BaseSpeed 400, Line PID = (2, 0, 0)
  3. Mode 3: BaseSpeed 250, Line PID = (10, 0, 0)
  4. Mode 4: BaseSpeed 200, Line PID = (10, 0, 0)
  5. Mode 5: BaseSpeed 300, Line PID = (8, 0, 0)
  6. Mode 6: BaseSpeed 350, Wall PID = (100, 0, 5, 60)

After typing a mode number and pressing **Enter**, the script prints a confirmation, e.g.

```
Mode 3: Base speed set to 250, Line-Following PID set to P=10, I=0, D=0
```

---

## Tips & Troubleshooting

1. **Arduino I²C Anxiety**

   * If one or both Motoron controllers do not.respond, open the Arduino Serial Monitor at 9600 baud and look for errors from `mc1.reinitialize()` or `mc2.reinitialize()`.
   * Use an I²C scanner sketch to confirm addresses are actually `0x10` and `0x11`.
   * Ensure pull-ups (4.7 kΩ typically) are in place on SCL1/SDA1 lines—Motoron boards often have onboard pull-ups, but verify.

2. **Wi-Fi Connectivity**

   * If the robot never prints `WiFi connected.`, double-check SSID/password, and confirm your network is broadcasting on 2.4 GHz (ESP32 doesn’t support 5 GHz).
   * If your router isolates wireless clients, the UDP packets from the Python script may not reach the ESP32; try “AP mode” or a generic 2.4 GHz hotspot.

3. **Reflectance Sensors Not Detecting.Line**

   * Confirm the physical height of the sensor array above the floor (~3–4 mm for QTRX).
   * Use a piece of white paper vs. black tape. Measure raw reflectance values by temporarily adding `Serial.println(reflectanceValues[i]);` in `readReflectanceSensors()`. Adjust `lineThreshold` accordingly.

4. **Sonar Sensors Giving Noisy Readings**

   * Check that the wiring of ECHO pins is correct (pins set to INPUT) and TRIG pins are set to OUTPUT.
   * If `duration` from `pulseIn()` returns 0, code treats it as “no echo” and returns –1 or the last filtered reading.
   * If filtering is too slow (α = 0.2), you can increase α (e.g., to 0.4) or adjust thresholds in code.

5. **Servos Not Moving or Jittering**

   * Ensure the servo power (5 V) has a stable supply and common ground with Arduino.
   * If jitter persists, add a 100 μF or 220 μF capacitor across servo V+ and GND.
   * In code, if `killSwitchActivated == false`, `stopServos()` will `detach()` them; re-attaching happens only when the kill switch is toggled back on.

6. **Section Logic Doesn’t Advance**

   * Print out `lostLineCounter` in `loop()` to verify it’s changing as expected.
   * Check correct sonar wiring. If `distL1` never exceeds 20 cm, Section 2 never ends.
   * If `allSensorsOn()` (all reflectance sensors detect “black” simultaneously) is never triggered, Section 4 won’t start. You can temporarily force `lostLineCounter = 7;` in `loop()` to test the pit/zipline behavior.

7. **UDP Commands Not Being Received**

   * Verify Arduino Serial Monitor shows incoming commands (if you add a debug `Serial.println(command)` inside `parseUDPCommand`).
   * Confirm your PC’s firewall allows outbound UDP to port 55500 to the ESP32’s IP.
   * Use a.simple UDP test tool (e.g., `netcat`) to see if the ESP32 receives packets:

     ```bash
     echo -n "Stop" | nc -u 192.168.0.113 55500
     ```
   * Check `WiFi.localIP()` printed in Serial Monitor is exactly what you’re using in `parameter_tune_and_kill.py`.

---

## License & Credits

* **Firmware & Python Script**
  © 2025 Boxonwils Robotics Team. Licensed under the MIT License (see `LICENSE` file).
* **Motoron Library** by Pololu – follows its own license (BSD-style).
* **ESP32 Arduino Core** by Espressif Systems – follows Apache 2.0.
* **Servo Library** – included with Arduino IDE; MIT license.
* **Thanks** to Dr. Igor Gaponov, Olliver Collier, Alexandros Charitonidis, Dr. Daniel Tozadore, Dr. Yunda Yan, Dr. Pian Yu for the organisation of this Term 3 Robotics Challenge.
