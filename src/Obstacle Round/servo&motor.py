from gpiozero import Device
from gpiozero  import Motor, AngularServo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
       
Device.pin_factory = PiGPIOFactory('127.0.0.1')


STEERINGANGLEMID = 0
SERVOOFFSET = -20

motor = Motor(forward = 24, backward = 23, enable = 18, pwm = True, pin_factory = Device.pin_factory)

servo = AngularServo(pin = 17, initial_angle = 0, min_angle = -90, max_angle = 90, 
                    min_pulse_width = 0.0006, max_pulse_width = 0.0024,
                     pin_factory = Device.pin_factory)

def setSteeringAngle(angle):
    if (-55 <= angle <= 55):
        servo.angle = angle + SERVOOFFSET
def setSteeringAngleMid():
    servo.angle = SERVOOFFSET 

setSteeringAngle(0)
motor.stop()
"""
setSteeringAngle(0)
motor.stop()
sleep(2)
motor.forward(0.2)
sleep(2)
motor.forward(0.4)
sleep(2)
motor.forward(0.7)
sleep(2)
motor.forward(1)
sleep(2)
motor.forward(0.6)
sleep(2)
motor.forward(0.3)
sleep(2)
"""
motor.stop()
sleep(3)
for i in range(4):
    setSteeringAngle(-20)
    sleep(2)
    setSteeringAngle(0)
    sleep(2)
    setSteeringAngle(20)
    sleep(2)
sleep(2)



motor.stop()
