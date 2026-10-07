from pybricks.hubs import PrimeHub
from pybricks.iodevices import XboxController
from pybricks.parameters import Button, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

# --- HARDWARE SETUP ---
hub = PrimeHub()
left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)

# Arm Motors (Ports C and D)
left_arm = Motor(Port.C)
right_arm = Motor(Port.D)

robot = DriveBase(left_motor, right_motor, wheel_diameter=66, axle_track=148)

# Default hub PID is kept active to prevent turn locking
robot.use_gyro(True)
robot.settings(800, 400, 400, 200)

controller = XboxController()

# --- PROTOTYPING CONFIGURATION ---
BASE_SPEED = 60          
MAX_SPEED = 700          
SPEED_STEP = 40          
RAMP_INTERVAL = 80       

RAMP_DOWN_STEP = 60      
RAMP_DOWN_INTERVAL = 60  

STICK_TURN_ANGLE = 45    
DPAD_TURN_ANGLE = 15     
WALL_SPEED = 250         
DEADZONE = 25            

# --- TRACKING VARIABLES ---
history = []  
ramp_timer = StopWatch()
is_driving = False
start_distance = 0
current_speed = 0
last_direction = 1

def one_wheel_turn(angle, undo=False):
    """Pivots the robot on one stationary wheel. If undo=True, retracts backwards."""
    wheel_degrees = abs(angle) * (148 / 66) * 2 
    
    if undo:
        wheel_degrees = -wheel_degrees
    
    if angle > 0:
        # Turn Right (left wheel moves)
        right_motor.hold() 
        left_motor.run_angle(400, wheel_degrees, wait=True)
    else:
        # Turn Left (right wheel moves)
        left_motor.hold()
        right_motor.run_angle(400, wheel_degrees, wait=True)
        
    robot.stop()

def drive_until_wall(speed):
    """Drives straight into a wall until stalled."""
    robot.drive(speed, 0)
    wait(150)  
    watch = StopWatch()
    while not robot.stalled() and watch.time() < 4000:
        wait(10)
    robot.stop()
    controller.rumble(power=60, duration=150)

while True:
    buttons = controller.buttons.pressed()
    lx, ly = controller.joystick_left()
    
    # Read Triggers safely
    try:
        lt, rt = controller.triggers()
    except Exception:  # noqa: BLE001
        # Button.LT/RT do not exist in Pybricks. Safely default to 0 to prevent crash.
        lt, rt = 0, 0 

    lb_only = Button.LB in buttons and Button.B not in buttons

    # 0. ARM MOTOR CONTROL
    if lt > 5:
        dir_mult = -1 if lb_only else 1
        left_arm.run(lt * 8 * dir_mult)
    else:
        left_arm.hold()

    if rt > 5:
        dir_mult = -1 if lb_only else 1
        right_arm.run(rt * 8 * dir_mult)
    else:
        right_arm.hold()

    # --- STATE CORRUPTION FIX ---
    # Check if a discrete action is triggered. If we are currently driving, 
    # stop the drive and log the distance before executing the action.
    interrupt_pressed = (
        (Button.LB in buttons and Button.B in buttons) or
        Button.UP in buttons or Button.DOWN in buttons or
        Button.LEFT in buttons or Button.RIGHT in buttons or
        (abs(lx) > 70 and abs(ly) < 40)
    )

    if interrupt_pressed and is_driving:
        robot.stop()
        is_driving = False
        driven = robot.distance() - start_distance
        if abs(driven) > 10:
            history.append(("drive", driven, False))

    # 1. UNDO ACTION (LB + B)
    if Button.LB in buttons and Button.B in buttons:
        if history:
            action, val, was_one_wheel = history.pop()
            if action == "turn":
                if was_one_wheel:
                    one_wheel_turn(val, undo=True)
                else:
                    robot.turn(-val)
                    robot.stop()
            elif action == "drive":
                robot.straight(-val)
                robot.stop()
        else:
            robot.turn(180)
            robot.stop()
        
        controller.rumble(power=80, duration=200)
        while Button.B in controller.buttons.pressed() or Button.LB in controller.buttons.pressed():
            wait(10)

    # 2. D-PAD WALL ALIGNMENT
    elif Button.UP in buttons:
        start_dist = robot.distance()
        drive_until_wall(WALL_SPEED)
        history.append(("drive", robot.distance() - start_dist, False))
        while Button.UP in controller.buttons.pressed(): wait(10)
        
    elif Button.DOWN in buttons:
        start_dist = robot.distance()
        drive_until_wall(-WALL_SPEED)
        history.append(("drive", robot.distance() - start_dist, False))
        while Button.DOWN in controller.buttons.pressed(): wait(10)

    # 3. D-PAD INCREMENTAL TURNS
    elif Button.LEFT in buttons:
        if lb_only:
            one_wheel_turn(-DPAD_TURN_ANGLE)
            history.append(("turn", -DPAD_TURN_ANGLE, True))
        else:
            robot.turn(-DPAD_TURN_ANGLE)
            robot.stop()
            history.append(("turn", -DPAD_TURN_ANGLE, False))
        while Button.LEFT in controller.buttons.pressed(): wait(10)
        
    elif Button.RIGHT in buttons:
        if lb_only:
            one_wheel_turn(DPAD_TURN_ANGLE)
            history.append(("turn", DPAD_TURN_ANGLE, True))
        else:
            robot.turn(DPAD_TURN_ANGLE)
            robot.stop()
            history.append(("turn", DPAD_TURN_ANGLE, False))
        while Button.RIGHT in controller.buttons.pressed(): wait(10)

    # 4. JOYSTICK FLICK TURNS
    elif abs(lx) > 70 and abs(ly) < 40:
        angle = STICK_TURN_ANGLE if lx > 0 else -STICK_TURN_ANGLE
        if lb_only:
            one_wheel_turn(angle)
            history.append(("turn", angle, True))
        else:
            robot.turn(angle)
            robot.stop()
            history.append(("turn", angle, False))
            
        while abs(controller.joystick_left()[0]) > 30:  
            wait(10)

    # 5. GYRO DRIVE WITH RAMP-UP
    elif abs(ly) > DEADZONE:
        last_direction = 1 if ly > 0 else -1
        
        if not is_driving:
            is_driving = True
            start_distance = robot.distance()
            current_speed = BASE_SPEED
            ramp_timer.reset()
            robot.drive(last_direction * current_speed, 0)
            
        elif ramp_timer.time() > RAMP_INTERVAL:
            if current_speed < MAX_SPEED:
                current_speed = min(MAX_SPEED, current_speed + SPEED_STEP)
                robot.drive(last_direction * current_speed, 0)
            ramp_timer.reset()

    # 6. DELICATE RAMP-DOWN LOGIC
    else:
        if is_driving and ramp_timer.time() > RAMP_DOWN_INTERVAL:
            current_speed -= RAMP_DOWN_STEP
            
            if current_speed <= 0:
                robot.stop()
                is_driving = False
                
                driven = robot.distance() - start_distance
                if abs(driven) > 10:
                    history.append(("drive", driven, False))
            else:
                robot.drive(last_direction * current_speed, 0)
                ramp_timer.reset()

    wait(10)