# WRO2024-FUTURE-ENGINEERS-VOLTERRA-TEAM-INTERNATIONAL-FINAL


# Hardware Design

## Engineering Factor

We took charge of the entire prototype design, from the compact and efficient integration of hardware, electronics, and mechanics to its aesthetic appeal. Our goal was to create a captivating prototype by infusing it with a charming vintage style.

We were responsible for both the development and assembly of the vehicle. It features a structure made of three **plywood** panels positioned horizontally at various elevations: two larger panels form the base and middle layers, while a smaller panel, reduced in size, sits on the top. These layers were meticulously cut from the plywood board with a jigsaw, then carefully smoothed using sandpaper.

For the wheel system, we primarily used **Meccano** and **Lego** parts. To ensure accurate control of the wheel positioning, we implemented the **Ackermann steering system**. This system provides different turning angles for the inner and outer wheels when cornering. We accomplished this by incorporating a **mechanical differential** and gears to connect the motor to the rear wheels.

## Mobility Management

In our project we have selected the following motors:

<table border="1" cellpadding="10">
  <tr>
    <th>Servomotor</th>
    <th>Encoder Gearmotor</th>
  </tr>
  <tr>
    <td>
    <div align=center>
      <img src="other/media/servomotor_image.jpg" alt="Servomotor" width="200">
      </div>
    </td>
    <td>
    <div align=center>
      <img src="other/media/gearmotor_image.jpg" alt="Encoder Gearmotor" width="200">
      </div>
    </td>
  </tr>
  <tr>
    <td>
      <ul>
        <li><strong>Servo motor</strong></li>
        <li>Weight: 12g</li>
        <li>Dimensions: 23 x 11.5 x 24 mm approx</li>
        <li>Stall torque: 1.6 kgf*cm at 4.8V</li>
        <li>Operating speed: 0.12s/60° at 4.8V at no load</li>
        <li>Stall torque: 2.0 kgf*cm at 6.0V</li>
        <li>Operating speed: 0.10s/60° at 6V at no load</li>
        <li>Rotational range: 180°</li>
        <li>Pulse cycle: ca. 20ms</li>
        <li>Pulse width: 500-2400 µs</li>
      </ul>
    </td>
    <td>
      <ul>
        <li><strong>DC 12V Encoder Gearmotor</strong></li>
        <li>Up to 200 RPM (Revolutions per minute)</li>
        <li>Stall torque extrapolation: 21 kgf*cm</li>
        <li>50:1 integrated gearbox</li>
        <li>No load current: 0.2A at 12V</li>
        <li>Stall current: 5.5A at 12V</li>
      </ul>
    </td>
  </tr>
</table>

To properly implement the DC 12V encoder gear motor and servo motor, a motor driver and mechanical differential were selected. The gear motor drives the mechanical differential, distributing movement to the rear wheels. It was chosen for its torque multiplication, crucial for precise control in small spaces, like a playing field. The motor's reduction ratios and integrated gearbox simplify development and offer strong power, high torque, and durability due to its enclosed design.

<div align=center>
<img src="other/media/differential.gif" alt="banner" width="400px">
</div>

The servo motor steers the front axle, providing efficient, high-torque performance with precise speed control, ideal for dynamic responses. It uses permanent magnets with low rotor inertia, enhancing speed control and energy efficiency.

## Power Management

<table>
  <tr>
    <td><img src="other/media/battery_image.jpg" alt="Battery" /></td>
    <td><b>LiPo 3S battery</b><br>Our vehicle's <b>power source</b> is a high-performance <b>LiPo 3S battery with 1800mAh capacity</b>. It has exceptional high-discharge capacity, with three cells operating at 3.7 volts. The battery weighs 135 grams and measures 34x105x23 mm.</td>
  </tr>
  <tr>
    <td><img src="other/media/connectors_image.jpg" alt="XT-60 connectors" /></td>
    <td><b>XT-60 connectors</b><br>XT-60 connectors are strong, reliable connectors used to connect and disconnect batteries safely. They can handle high currents, up to 60 amps, without overheating. These connectors often work with a voltage stabilizer to provide a steady 5.0V output, making them great for powering devices like a Raspberry Pi through its USB Type-C port.</td>
  </tr>
  <tr>
    <td><img src="other/media/voltage_regulator_image.jpg" alt="Voltage regulator and stabilizer" /></td>
    <td><b>Voltage regulator and stabilizer</b><br>A voltage regulator or DC-DC step-down converter is used to reduce the voltage from a higher level to a stable, lower level, ensuring that sensitive electronics receive the correct power. This type of module efficiently converts excess voltage into a usable form. We employ this piece of hardware in order to provide consistent 5V current to power the Raspberry Pi and Arduino Nano.</td>
  </tr>
  <tr>
    <td><img src="other/media/level_converter.jpg" alt="Level Converter" /></td>
    <td><b>Level Converter</b><br>The level converter facilitates communication between devices with different operating voltages, like Raspberry (3.3V) with Arduino Nano (5.0V) and distance sensor (5.0V).</td>
  </tr>
</table>

**Power Distribution** <br> Sensors and the servo motor receive 5.0V from the **Raspberry** (except the color sensor at 3.3V). The **EDGE TPU** gets 2W of power from the Raspberry's USB3.0 at 5V, with a peak current use of 900mA.

## Sense Management

<table>
  <tr>
    <td><img src="other/media/colour_sensor_image.jpg" alt="Color sensor" /></td>
    <td><b>Color Sensor TCS 34725</b><br>It can detect red, green, blue, and white light components, with an infrared blocking filter for accurate color measurements. It guides the vehicle's direction based on detecting the orange or blue line. Also, we use its data to keep track of the laps. Powered directly at 3.3V from the Raspberry Pi.</td>
  </tr>
  <tr>
    <td><img src="other/media/ultrasound_sensor_image.jpg" alt="Ultrasound sensor" /></td>
    <td><b>Ultrasound HC SR04</b><br>Emits 40kHz pulses and measures flight time in microseconds. Operates at 5.0V. Originally intended for continuous distance measurements, but now only used at the start of the round in order to estimate the position of the vehicle.</td>
  </tr>
  <tr>
    <td><img src="other/media/gearmotor_image2.jpg" alt="Motor Encoder" /></td>
    <td><b>Motor Encoder</b><br>An electromagnetic device powered at 5.0V, used to calculate the distance traveled by measuring wheel revolutions. Not currently in use due to the decision to simplify the prototype.</td>
  </tr>
  <tr>
    <td><img src="other/media/IMU_sensor_image.jpg" alt="IMU" /></td>
    <td><b>WT901C-TTL (Inertial Measurement Unit)</b><br>A nine-axis sensor with the MPU 6050 chip, it measures angular velocity with an angle less than 180° on the X-axis and 90° on the Y-axis. Voltage range: 3.3V to 5.0V.</td>
  </tr>
  <tr>
    <td><img src="other/media/camera_image.jpg" alt="Camera" /></td>
    <td><b>Camera</b><br>A high-definition 5-megapixel camera module with a 175° wide-angle lens, used to detect walls, obstacles, and their colors. Connected to the Raspberry Pi via the CSI connector.</td>
  </tr>
  <tr>
    <td><img src="other/media/arduino_nano_image.jpg" alt="Arduino Nano" /></td>
    <td><b>Arduino Nano</b><br>The Arduino Nano is a small and flexible 5V microcontroller board from the Arduino family, ideal for tasks like controlling PWM (Pulse Width Modulation) signals for motors. It has a 16 MHz ATmega328 chip, providing enough power for many projects. It was originally designed to connect the Raspberry Pi to sensors, but now it takes commands from the Raspberry Pi using UART to control servo motors smoothly and accurately.</td>
  </tr>
</table>

## Theoretical Physics/Engineering/Mechanical Lesson
### Understanding the Ackermann Steering System in Robotics

The Ackermann steering system is a key principle in vehicle dynamics, used in both robotics and automotive engineering to optimize wheel alignment during turns. It ensures that each wheel follows the correct turning radius, reducing tire wear and enhancing stability.

#### Basics of Ackermann Steering

The system's design involves adjusting the angles of the front wheels so that they follow different paths during a turn. The inner wheels turn more sharply than the outer wheels, ensuring all wheels' axes meet at a single turning point. This precise alignment is achieved using a set of steering arms and linkages.

#### Application in Robotics

Ackermann steering is widely used in robotic platforms that mimic car-like movement, called Ackermann robots. It enables these robots to navigate smoothly and handle tight turns, making it essential for applications like autonomous vehicles and delivery robots.

#### Role of Differentials

During a turn, the wheels on the outside need to rotate faster than those on the inside. A differential mechanism solves this by allowing each wheel to turn at different speeds while receiving power from the same source. This prevents skidding and ensures controlled motion through turns.

#### Integration in Robotics

In robotics, combining Ackermann steering with differentials improves precision and maneuverability. The steering controls wheel alignment, while the differential manages torque distribution, allowing for smooth, natural movement essential for advanced navigation tasks.