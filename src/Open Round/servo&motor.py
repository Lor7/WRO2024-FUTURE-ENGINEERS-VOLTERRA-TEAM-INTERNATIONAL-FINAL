from gpiozero import Device
from gpiozero  import Motor, AngularServo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
       
Device.pin_factory = PiGPIOFactory('127.0.0.1')


STEERINGANGLEMID = 0
SERVOOFFSET = -40

motor = Motor(forward = 23, backward = 24, enable = 12, pwm = True, pin_factory = Device.pin_factory)

servo = AngularServo(pin = 17, initial_angle = 2, min_angle = -90, max_angle = 90, 
                    min_pulse_width = 0.0006, max_pulse_width = 0.0024,
                     pin_factory = Device.pin_factory)

def setSteeringAngle(angle):
    if (-30 <= angle <= 30):
        servo.angle = angle + SERVOOFFSET
def setSteeringAngleMid():
    servo.angle = SERVOOFFSET 

motor.stop()
setSteeringAngle(0)
motor.forward(0.35)
sleep(2)
motor.stop()
