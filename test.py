from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

hub = PrimeHub()

left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)

robot = DriveBase(left_motor, right_motor, wheel_diameter=66, axle_track=148)

robot.use_gyro(True)
robot.settings(800, 400 , 400 , 200)


robot.straight(100)
robot.turn(90)

robot.straight(100)
robot.turn(90)

robot.straight(100)
robot.turn(90)

robot.straight(100)
robot.turn(90)
