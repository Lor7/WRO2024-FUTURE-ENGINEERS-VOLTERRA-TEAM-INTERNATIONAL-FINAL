# WRO2024-FUTURE-ENGINEERS-VOLTERRA-TEAM-INTERNATIONAL-FINAL


# Hardware Design

## Engineering Factor

We took charge of the entire prototype design, from the compact and efficient integration of hardware, electronics, and mechanics to its aesthetic appeal. Our goal was to create a captivating prototype by infusing it with a charming vintage style.

We were responsible for both the development and assembly of the vehicle. It features a structure made of three **plywood** panels positioned horizontally at various elevations: two larger panels form the base and middle layers, while a smaller panel, reduced in size, sits on the top. These layers were meticulously cut from the plywood board with a jigsaw, then carefully smoothed using sandpaper.

For the wheel system, we primarily used **Meccano** and **Lego** parts. To ensure accurate control of the wheel positioning, we implemented the **Ackermann steering system**. This system provides different turning angles for the inner and outer wheels when cornering. We accomplished this by incorporating a **mechanical differential** and gears to connect the motor to the rear wheels.



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