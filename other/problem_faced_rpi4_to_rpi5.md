# Documentation: Challenges and Solutions in Upgrading from Raspberry Pi 4 to Pi 5

## 1. No **pigpiod** Library Support
The pigpiod library, which is typically used to control GPIO pins, is no longer supported on the Raspberry Pi 5. This caused significant challenges in servo control. This library is essential for managing real-time operations, especially in robotics and IoT projects.
### Analysis:
- The pigpiod daemon was removed as the Pi 5 introduced hardware and software changes, replacing certain low-level control mechanisms.
- Attempts to bypass this limitation by assigning higher priority to the Python process managing servo control (using nice or renice) resulted in smoother control but:
<table>
<tr><td><br>Inconsistent behavior:</br></td><td>Inconsistent behavior: Servo movements were jittery under high CPU loads.</td></tr>
<tr><td><br>Hardware damage:</br></td><td>The servo motor eventually overheated and burned out due to irregular signal timings.</td></tr>
</table>

### Solution:
<b>Switch to Arduino Nano for Servo Control.</b> Communication between the Pi 5 and Arduino Nano is handled via Serial UART, ensuring smooth and consistent servo operation.

#### Benefits:
- Servo control is now isolated from Pi 5's load, preventing jitter.
- Arduino Nano offers highly precise timing for PWM signals, eliminating servo overheating.
- Reduced dependency on deprecated libraries.


---

## 2. Using a Virtual Environment for Python 3.9 (Coral) and Camera Library Compatibility
The project was developed in a **Python 3.9** environment due to the use of the **Google Coral** accelerator, which has optimized libraries for this version of Python. However only **Picamera2** library is supported to read the camera module; this library is available only from **Python 3.11** or later, and attempts to use older Python versions like 3.9 failed to run the camera.

### **Solution**
To solve this incompatibility:
1. We used **Picamera2** to handle camera readings. This library relies on the `libcamera` stack and works in a <i>virtual environment</i> with Python 3.11.
2. For the **Google Coral** accelerator, we maintained the Python 3.9 environment, but used a **virtual environment** for both Python versions.
3. We set up the project in such a way that the camera functionality runs in **Python 3.11** (with **Picamera2**) while the Coral models and related tasks run in the **Python 3.9** environment. This solution allows for the use of both technologies without direct dependency issues. Indeed the two different programs run on parallel processes and communicates with the use of ZMQ library's <i>inter process communication</i> capabilities.

#### Additional Benefits:
1. Reduced camera readings latency
2. Reduced workload for the main program
3. Modular and scalable structure

---

## 3. Crashes when Handling Multiple Peripherals
A problem encountered when using **multiple peripherals** (e.g., servo motors, sensors, camera, AI accelerator) with the Raspberry Pi 5 was **frequent crashes** under heavy load, particularly when powering devices directly from the Pi. The servo motor, for instance, was powered through the Raspberry Pi, leading to voltage instability and crashes.

### **Solution**
We moved the servo motor power supply to an **Arduino Nano**. The Nano is powered via the **tension converter** and **stabilizer**, ensuring that it receives consistent power. This setup allows the Pi to focus on its primary processing duties without handling excessive power demands from the motor, thus preventing crashes and system instability.

---

## Summary of Solutions:
1. **No pigpiod library support**: Replaced pigpiod with Arduino Nano to control the servo motor and avoid overload issues.
2. **Python environment incompatibilities**: Used a virtual environment to support Python 3.9 (for Coral) and Python 3.11 (for Picamera2).
3. **Crashes under load**: Shifted the power supply of peripherals directly to the battery, ensuring stable power delivery and preventing crashes.

---

By taking these challenges into account and implementing the corresponding solutions, we were able to effectively upgrade from the Raspberry Pi 4 to the Raspberry Pi 5 while maintaining system stability and ensuring optimal performance for the project.
