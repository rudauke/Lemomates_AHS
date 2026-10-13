# FIRST LEGO LEAGUE - CHALLENGE 
# 2026 / 2027
# ROBOT GAME 
# Authors: LEMOMATES Team
# Version: 2027_001
# Date: 

# Import of used libraries

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, hub_menu, multitask, run_task, wait
from ty_extensions._internal import Unknown

# Definitions of constants and their default values.
# Default values have been determined experimentally.
WHEEL_DIAMETER  = 62.4 # wheel diameter [mm]
AXLE_TRACK      = 150  # distance between the wheels [mm]
STRAIGHT_SPEED  = 500  # normal straight speed for driving [mm/s], max 855
STRAIGHT_ACCEL  = 500  # normal acceleration [mm/s^2], max 8500
STRAIGHT_DECEL  = 1000  # normal deceleration [mm/s^2]
TURN_RATE       = 500  # normal turning rate [deg/s], max 616
TURN_ACCEL      = 300  # normal turning acceleration [deg/s^2]
TURN_DECEL      = 600  # normal turning deceleration [def/s^2]
IMU_ANG_VEL     = 5    # IMU.SETTINGS - ANGULAR VELOCITY THRESHOLD [deg/s]
IMU_ACCEL       = 3000 # IMU.SETTINGS - ACCELERATION THRESHOLD [mm/s^2]


# Initialize hub
hub = PrimeHub(top_side= Axis.Z, front_side= Axis.Y)  # ty: ignore[invalid-argument-type]

# Initialize both drive motors
MotorA  = Motor(Port.A, Direction.COUNTERCLOCKWISE)
MotorB  = Motor(Port.B)
#MotorC  = Motor(Port.C)
#MotorD  = Motor(Port.D)
#MotorE  = Motor(Port.E)
#MotorF  = Motor(Port.F)

#Initialize both arms/attachments motors
armLeftMotor = Motor(Port.C)
armRightMotor = Motor(Port.D)

# Initialize color sensors
colorSensorLeft  = ColorSensor(Port.E)
colorSensorRight = ColorSensor(Port.F)
# Disabling color sensers
#colorSensorLeft.lights.off()
#colorSensorRight.lights.off()
        
# Initialize distance sensor
#distanceSensor = UltrasonicSensor(Port.E)

# Initialize the drive base        
bot = DriveBase(MotorA, MotorB, WHEEL_DIAMETER, AXLE_TRACK)
bot.settings(STRAIGHT_SPEED, (STRAIGHT_ACCEL, STRAIGHT_DECEL), TURN_RATE, (TURN_ACCEL, TURN_DECEL))

# Checks if the device is ready for use, service function
async def readiness():
    # Checks if the device is calibrated and ready for use
    if hub.imu.ready():
        hub.light.on(Color.GREEN)
        # Checks if the device is currently stationary (not moving)
        #if (hub.imu.stationary() == False):
         #   hub.light.on(Color.RED)
    else:
        hub.light.on(Color.RED)
    print("Hub Name -----------: ", hub.system.name())  # ty: ignore[unresolved-attribute]
    print("Hub Ready For Use ? : ", hub.imu.ready())
    print("Battery Voltage     : ", hub.battery.voltage())
    #print("Base Settings       : ", DriveBase.settings())
    print("Battery current     : ", hub.battery.current())
    #print("Charger connected   : ", hub.charger.connected())
    print("Charger current   mA: ", hub.charger.current())
    print("Charger status      : ", hub.charger.status())
    #print("Right arm control   : ", arm_right_motor.control.limits())
    #print("Left arm control    : ", arm_left_motor.control.limits())


async def MyDrive(distance, speed=STRAIGHT_SPEED, then=Stop.COAST, wait=True):
    """
    Moves the robot driving straight forward or backward a specified distance 
    and speed. 
      
    Function's parameters:
        >>> distance <<<
            How far [mm] the robot should go.
            Positive values go forward and negative values go backwards.
            type: float
            values: Any
            default: No default value
            example: distance=200 

        >>> speed <<< 
            How fast the robot should go. 
            Maksymalne prędkości zależą od średnicy wykorzystywanych kół.
            Poniższa wartość maksymalna wyznaczone zostałą eksperymentalnie.
            type: float
            values: <0, 855>.
            default: STRAIGHT_SPEED (przyjęliśmy ok 80% wartości maksymalnej)
            example: speed=700

        >>> then <<< 
            What should happen after robot stops. 
            type: Stop
            values: Stop.HOLD,      
                        Keep controlling the motor to hold it at the commanded 
                        angle.    
                    Stop.BRAKE, 
                        Passively resist small external forces.
                    Stop.COAST, 
                        Let the motor move freely.
                    Stop.COAST_SMART.
                        Let the motor move freely. 
                        For the next relative angle maneuver, 
                        take the last target angle (instead of the current angle) 
                        as the new starting point. 
                        This reduces cumulative errors. 
                        This will apply only if the current angle is less than 
                        twice the configured position tolerance.
            default: Stop.HOLD
            example: then=Stop.HOLD
        
        >>> wait <<< 
            If the robot should wait for this action to stop 
            before doing the next action.
            type: boolean
            values: True, False.
            default: True
            example: wait=True
    """
    speed: int | Unknown = max(-616, min(616, speed))
        
    bot.settings(speed, (STRAIGHT_ACCEL, STRAIGHT_DECEL), TURN_RATE, (TURN_ACCEL, TURN_DECEL))
    await bot.straight(distance, then=then, wait=wait)

    # Restoring initial values
    bot.settings(STRAIGHT_SPEED, (STRAIGHT_ACCEL, STRAIGHT_DECEL), TURN_RATE, (TURN_ACCEL, TURN_DECEL))


async def MyTurn(angle, speed=TURN_RATE, then=Stop.COAST, wait=True):
    """
    Turns the robot to the specified angle.  
    Positive values of angle turn the robot to the right, negative values 
    turn to the left.
        
    Function's parameters:
        >>> angle <<< 
            The angle the robot should turn.
            type: float (Represented by up to seven digits e.g. 123.4567)
            values: Any.
            default: No default value
            example: angle=-90
        >>> then <<<
            What should happen after robot turning.
            type: class Stop
            values: Stop.HOLD,      
                        Keep controlling the motor to hold it at the commanded 
                        angle.    
                    Stop.BRAKE, 
                        Passively resist small external forces.
                    Stop.COAST, 
                        Let the motor move freely.
                    Stop.COAST_SMART.
                        Let the motor move freely. 
                        For the next relative angle maneuver, 
                        take the last target angle (instead of the current angle) 
                        as the new starting point. 
                        This reduces cumulative errors. 
                        This will apply only if the current angle is less than 
                        twice the configured position tolerance.
            default: Stop.BRAKE
            example: then=Stop.HOLD
        >>> wait <<<
            If the robot should wait for this action to stop 
            before doing the next action.
            type: boolean
            values: True, False.
            default: True
            example: wait=False
        >>> speed <<<
            How fast the robot should go. 
            Positive values go forward and negative values go backwards.
            type: float
            values: More than -971, but less than 971.
            default: TURN_RATE
            example: speed=250
    """
    speed = min(abs(speed), 616)

    bot.settings(STRAIGHT_SPEED, (STRAIGHT_ACCEL, STRAIGHT_DECEL), speed, (TURN_ACCEL, TURN_DECEL))
    await bot.turn(angle, then=then, wait=wait)
        
    # Restoring initial values 
    bot.settings(STRAIGHT_SPEED, (STRAIGHT_ACCEL, STRAIGHT_DECEL), TURN_RATE, (TURN_ACCEL, TURN_DECEL))


async def MyCurve(radius, angle, speed=STRAIGHT_SPEED, then=Stop.COAST, wait=True):
    """
    Moves the robot in a curve with a given radius and at a given distance.
        
    Function's parameters:
        >>> radius <<< 
            How far the robot should go, radius of the circel [mm].
            Positive values go forward and negative values go backwards.
            type: float
            values: Any.
            default: No default value
        >>> angle <<< 
            Angle along the circle [deg]
            Positive values go forward and negative values go backwards.
            type: float
            values: Any.
            default: No default value
        >>> speed <<< 
            How fast the robot should go.
            Positive values go forward and negative values go backwards.
            type: float
            values: More than -971, but less than 971.
            default: No default value
        >>> then <<< 
            What should happen after robot stops.
            type: Stop
            values: Stop.HOLD, Stop.BRAKE, Stop.COAST, Stop.COAST_SMART.
            default: Stop.BRAKE
        >>> wait <<< 
            If the robot should wait for this action to stop 
            before doing the next action.
            type: boolean
            values: True, False.
            default: True
    """
    bot.settings(speed, (STRAIGHT_ACCEL, STRAIGHT_DECEL), TURN_RATE, (TURN_ACCEL, TURN_DECEL))    
    await bot.arc(radius, angle, then=then, wait=wait)
    # Restoring initial values 
    bot.settings(STRAIGHT_SPEED, (STRAIGHT_ACCEL, STRAIGHT_DECEL), TURN_RATE, (TURN_ACCEL, TURN_DECEL))

async def RightArmDrive(speed, angle, then=Stop.BRAKE, wait=True):
    await armRightMotor.run_angle(speed, angle, then=then, wait=wait)


async def LeftArmDrive(speed, angle, then=Stop.BRAKE, wait=True):
    await armLeftMotor.run_angle(speed, angle, then=then, wait=wait)


async def WaitForMs(milliseconds):
    """
        Wait for the specified time [ms]
        Method parameters:
        >>> milliseconds <<<
        How long it should wait.
        type: float
        values: Any.
        default: No default value
    """
    await wait(milliseconds)


async def WaitForButton(button):
    """
        Waits for a button to be pressed
        Method parameters:
        >>> button <<<
        Which button that needs to be pressed.
        type: Button
        values: Button.LEFT, Button.RIGHT, Button.BLUETOOTH.
        default: No default value
    """
    while True:
        pressed = hub.buttons.pressed()
        if button in pressed:
           break
        await wait(50)

#async def StopOnLine():
 
async def MonitCurrentData():
    print("Battery voltage              : ", hub.battery.voltage())
    print("Status of the battery charger: ", hub.charger.status())
    print("Get heading angle            : ", hub.imu.heading())


async def Run_1():
    """
    Realizacja misji Robot Game: 
    - Misja 
    """

    await MyDrive(250,300)
    await MyTurn(90,300);
    await MyDrive(265,300)
    await MyTurn(-45,300)
    await MyDrive(80,150)
    await RightArmDrive (200, 62.5)
    await MyDrive (-375,100)



async def Run_2():
    """
    Realizacja misji Robot Game: 

    """

    await MyDrive(-720, 300)
    await MyDrive(70, 300)
    await MyTurn(-55, 300)
    await MyTurn(45, 300)
    await MyDrive(700, 300)

    

async def Run_3():
    """
    Description:
 
    """


async def Run_4():
    """
    Description:
 
    """



async def Run_5():
    """
    Description:
 
    """



async def Run_6():
    """
    Description:
 
    """


async def Run_7():
    """
    Description:
 
    """




async def Run_8():
    """
    Description:
 
    """



async def Run_9():
    """
    Description:
 
    """



async def run_mission(mission_func):
    # wrap missions with hub base settings and gyro handling
    hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
    bot.use_gyro(True)
    try:
        await mission_func()
    finally:
        bot.use_gyro(False)


# Map menu selections to your run functions
run_funcs = {
    "1": Run_1, "2": Run_2, "3": Run_3, 
    "4": Run_4, "5": Run_5, "6": Run_6, 
    "7": Run_7, "8": Run_8, "9": Run_9
}

runs = ["1", "2", "3", "4", "5", "6", "7", "8", "9"]
curr_idx = 0

while True:
    if hub.imu.ready():
        hub.light.on(Color.GREEN)
        
        # Rotate list so the current run appears first in the menu
        current_menu = runs[curr_idx:] + runs[:curr_idx]
        selected = hub_menu(*current_menu)
        
        if selected == ("9"):
            break
            
        if selected in run_funcs:
            # Automatically wrap the selected mission with gyro handling
            run_task(run_mission(run_funcs[selected]))
            
            # Advance to the next run automatically
            curr_idx = (runs.index(selected) + 1) % len(runs)
    else:
        hub.light.on(Color.RED)
    
    wait(10)