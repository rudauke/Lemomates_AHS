# FIRST LEGO LEAGUE - CHALLENGE 
# 2026 / 2027
# ROBOT GAME 
# Authors: LEMOMATES Team
# Version: 2027_001
# Date: 

# Import of used libraries
from typing import Literal

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, hub_menu, multitask, run_task, wait
from ty_extensions._internal import Unknown

# Definitions of constants and their default values.
# Default values have been determined experimentally.
WHEEL_DIAMETER  = 81.5 # wheel diameter [mm]
AXLE_TRACK      = 111  # distance between the wheels [mm]
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
    speed: Literal[-616, 616, 500] | Unknown = max(-616, min(616, speed))
        
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
    - Misja 11. Artefakty Rybackie
    - Misja 12. Operacja Ratunkowa
    """
    hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
    # Enabling gyro
    bot.use_gyro(True)
    
    # Robot wyjezdza z czerwonego obszaru startowego
    # Podniesienie ramienia, Jazda do Operacji Ratunkowej

    await MyDrive(-700,300)
    #await MyTurn(-90, 300)
    await MyCurve(-55, 90, speed=STRAIGHT_SPEED/2, then=Stop.COAST, wait=True)
    await MyCurve(75, 95, speed=STRAIGHT_SPEED/2, then=Stop.COAST, wait=True)
    await MyDrive(750, 300)


    # Disable gyro
    bot.use_gyro(False)  


async def Run_2():
    """
    Realizacja misji Robot Game: 

    """
    hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
    # Enabling gyro 
    bot.use_gyro(True)
    
    await MyDrive(-720, 300)
    await MyDrive(70, 300)
    await MyTurn(-55, 300)
    await MyTurn(45, 300)
    await MyDrive(700, 300)

    # Disable gyro
    bot.use_gyro(False)  
    

async def Run_3():
    """
    Realizacja misji Robot Game: 

    """
    hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
    # Enabling gyro
    bot.use_gyro(True)
    

    # Disable gyro
    bot.use_gyro(False)  

async def Run_4():
    """
    Realizacja misji Robot Game: 

    """
    hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
    # Enabling gyro
    bot.use_gyro(True)
    

    # Disable gyro
    bot.use_gyro(False)  


async def Run_5():

    hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
    # Enabling gyro
    bot.use_gyro(True)
    
    
    # Disable gyro
    bot.use_gyro(False)  


async def Run_6():
    """
    Description:
 
    """
    hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
    # Enabling gyro
    bot.use_gyro(True)

      
    # Disable gyro
    bot.use_gyro(False)  


async def Run_7():
    """
    Realizacja misji Robot Game: 
    
    """
    #hub.imu.settings(IMU_ANG_VEL, IMU_ACCEL)
    # Enabling gyro
    bot.use_gyro(True)

    # Disable gyro
    bot.use_gyro(False)  



async def Run_8():
    hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
    # Enabling gyro
    bot.use_gyro(True)


    # Read the color and reflection
    color =  colorSensorLeft.color()
    reflection =  colorSensorLeft.reflection()
    hsv =  colorSensorLeft.hsv()
    
    # Print the measured color and reflection.
    print(color, "---", reflection, "---", hsv)


    # Disable gyro
    bot.use_gyro(False)  

async def Run_9():
    """
    Description ... 
    """
    hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
    # Enabling gyro 
    bot.use_gyro(True)
    
    """
    print("=====")
    print("Hub Name -----------: ", hub.system.name())
    print("Hub Ready For Use ? : ", hub.imu.ready())
    print("Battery Voltage     : ", hub.battery.voltage())
    print("Base Settings       : ", bot.settings())
    print("Battery current     : ", hub.battery.current())
    print("Charger connected   : ", hub.charger.connected())
    print("Charger current   mA: ", hub.charger.current())
    print("Charger status      : ", hub.charger.status())
    print("-----")
    e_speed = 500
    e_time = 10000
    #MotorA.run_time(e_speed, e_time)
    #MotorB.run_time(e_speed, e_time)
    #MotorC.run_time(e_speed, e_time)
    #MotorD.run_time(e_speed, e_time)
    #MotorE.run_time(e_speed, e_time)
    #MotorF.run_time(e_speed, e_time)
    await MyDrive(10000,500)
    #await WaitForMs(500)
    #await MyTurn(90, 100)
    #await WaitForMs(500)
       
    print("Hub Name -----------: ", hub.system.name())
    print("Hub Ready For Use ? : ", hub.imu.ready())
    print("Battery Voltage     : ", hub.battery.voltage())
    print("Base Settings       : ", bot.settings())
    print("Battery current     : ", hub.battery.current())
    print("Charger connected   : ", hub.charger.connected())
    print("Charger current   mA: ", hub.charger.current())
    print("Charger status      : ", hub.charger.status())
    print("=====")
    
    await MyTurn(90, 616)
    await multitask(RightArmDrive(400,300), MyDrive(250, 950))
    await multitask(MyDrive(110, 200), RightArmDrive(-600, 300))
    await MyCurve(-400, 90, 800) # Radius, Angle, Speed, do tyłu
    """
    # Disable gyro
    bot.use_gyro(False)  


hub.imu.settings(angular_velocity_threshold=IMU_ANG_VEL, acceleration_threshold=IMU_ACCEL)
currRun = 1
while True:
    #Checks if the device is calibrated and ready for use.
    if hub.imu.ready():
        hub.light.on(Color.GREEN)
        
        if currRun == 1:
            selected = hub_menu("1", "2", "3", "4", "5", "6", "7", "8", "9")
        if currRun == 2:
            selected = hub_menu("2", "3", "4", "5", "6", "7", "8", "9", "1")
        if currRun == 3:
            selected = hub_menu("3", "4", "5", "6", "7", "8", "9", "1", "2")
        if currRun == 4:
            selected = hub_menu("4", "5", "6", "7", "8", "9", "1", "2", "3")
        if currRun == 5:
            selected = hub_menu("5", "6", "7", "8", "9", "1", "2", "3", "4")
        if currRun == 6:
            selected = hub_menu("6", "7", "8", "9", "1", "2", "3", "4", "5")
        if currRun == 7:
            selected = hub_menu("7", "8", "9", "1", "2", "3", "4", "5", "6")
        if currRun == 8:
            selected = hub_menu("8", "9", "1", "2", "3", "4", "5", "6", "7")
        if currRun == 9:
            selected = hub_menu("9", "1", "2", "3", "4", "5", "6", "7", "8")

        if selected == "1":
            run_task(Run_1())
            currRun = 2
        if selected == "2":
            run_task(Run_2())    
            currRun = 3
        if selected == "3":
            run_task(Run_3())
            currRun = 4
        if selected == "4":
            run_task(Run_4())
            currRun = 5
        if selected == "5":
            run_task(Run_5())
            currRun = 6
        if selected == "6":
            run_task(Run_6())
            currRun = 9
        if selected == "7":
            run_task(Run_7())
            currRun = 9
        if selected == "8":
            break   
        if selected == "9":
            break        
        
    else:
        hub.light.on(Color.RED)
    
    wait(10)