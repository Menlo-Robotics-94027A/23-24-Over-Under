# ----------------------------------------------------------------------------- #
#                                                                               #                                                                          
#    Project:        Sharkbot 2.0                                               #
#    Module:         main.py                                                    #
#    Author:         VEX                                                        #
#    Created:        Sat Nov 04 2023                                            #
#    Description:    This example will use Controller button events to          # 
#                    control the V5 Clawbot arm and claw                        #
#                                                                               #                                                                          
#    Configuration:  V5 Sharkbot                                                #
#                    Controller                                                 #
#                    Catapult Motor in Port 19                                  #
#                    Catapult Motor in Port 20                                  #
#                    Left Front Motor in Port 11                                #
#                    Left Back Motor in Port 12                                 #
#                    Right Front Motor in Port 13                               #
#                    Right Back Motor in Port 14                                #
#                    Wing Pneumatic 1 in Port A                                 #
#                    Wing Pneumatic 2 in Port B                                 #
#                                                                               #                                                                          
# ----------------------------------------------------------------------------- #

#region VEXcode Generated Robot Configuration
from vex import *
import time

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
Cata_motor_a = Motor(Ports.PORT19, GearSetting.RATIO_36_1, False)
Cata_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_36_1, True)
Cata = MotorGroup(Cata_motor_a, Cata_motor_b)
Cata.set_velocity(75, PERCENT)
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
digital_out_a = DigitalOut(brain.three_wire_port.a)
digital_out_b = DigitalOut(brain.three_wire_port.b)
Climber_motor_a = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
Climber_motor_b = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
Climber = MotorGroup(Climber_motor_a, Climber_motor_b)
Climber.set_velocity(100, PERCENT)

# wait for rotation sensor to fully initialize
wait(30, MSEC)


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")



# define variables used for controlling motors based on controller inputs
controller_1_right_shoulder_control_motors_stopped = True
controller_1_left_shoulder_control_motors_stopped = True
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False
program = 0

# CREATE GLOBAL VARIABLES
# Which autonomous program to run
which_auton = 0
# Set if need to reverse auton values for opp side of field
flip = 1 # Default, don't need to change values at all

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, controller_1_right_shoulder_control_motors_stopped, remote_control_code_enabled, controller_1_left_shoulder_control_motors_stopped
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis1
            # right = axis3 - axis1
            drivetrain_left_side_speed = controller_1.axis3.position() - controller_1.axis1.position()
            drivetrain_right_side_speed = controller_1.axis3.position() + controller_1.axis1.position()
            
            # check if the value is inside of the deadband range
            if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    # stop the left drive motor
                    left_drive_smart.stop()
                    # tell the code that the left motor has been stopped
                    drivetrain_l_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                drivetrain_l_needs_to_be_stopped_controller_1 = True
            # check if the value is inside of the deadband range
            if drivetrain_right_side_speed < 5 and drivetrain_right_side_speed > -5:
                # check if the right motor has already been stopped
                if drivetrain_r_needs_to_be_stopped_controller_1:
                    # stop the right drive motor
                    right_drive_smart.stop()
                    # tell the code that the right motor has been stopped
                    drivetrain_r_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                drivetrain_r_needs_to_be_stopped_controller_1 = True
            
            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_l_needs_to_be_stopped_controller_1:
                left_drive_smart.set_velocity(drivetrain_left_side_speed, PERCENT)
                left_drive_smart.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_r_needs_to_be_stopped_controller_1:
                right_drive_smart.set_velocity(drivetrain_right_side_speed, PERCENT)
                right_drive_smart.spin(FORWARD)
            # check the buttonR1/buttonR2 status
            # to control Cata
            if controller_1.buttonR1.pressing():
                Cata.spin(FORWARD)
                controller_1_right_shoulder_control_motors_stopped = False
            elif controller_1.buttonR2.pressing():
                Cata.spin(REVERSE)
                controller_1_right_shoulder_control_motors_stopped = False
            elif not controller_1_right_shoulder_control_motors_stopped:
                Cata.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_right_shoulder_control_motors_stopped = True
            
            # to control Climber
            if controller_1.buttonL1.pressing():
                Climber.spin(FORWARD)
                controller_1_left_shoulder_control_motors_stopped = False
            elif controller_1.buttonL2.pressing():
                Climber.spin(REVERSE)
                controller_1_left_shoulder_control_motors_stopped = False
            elif not controller_1_left_shoulder_control_motors_stopped:
                Climber.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_left_shoulder_control_motors_stopped = True
        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

# endregion VEXcode Generated Robot Configuration
def autonomous_opp():
    pneumatic_off()
    # Get pre-load under other alliance's goal
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)
    drivetrain.drive_for(FORWARD, 19)
    drivetrain.turn_for(LEFT, 35, DEGREES)
    drivetrain.drive_for(FORWARD, 16)

    drivetrain.set_drive_velocity(20, PERCENT)
    drivetrain.set_turn_velocity(20, PERCENT)
    # Remove match-load
    drivetrain.drive_for(REVERSE, 17)
    drivetrain.turn_for(RIGHT, 40, DEGREES)
    drivetrain.drive_for(REVERSE, 4)
    # drivetrain.drive_for(REVERSE, 4)
    digital_out_b.set(True)
    drivetrain.drive_for(REVERSE, 6)
    drivetrain.turn_for(RIGHT, 55, DEGREES)

    # Touch colored bar
    # Adjust so driving straight
    drivetrain.turn_for(LEFT, 20, DEGREES)
    drivetrain.drive_for(FORWARD, 6)
    drivetrain.turn_for(RIGHT, 20, DEGREES)
    drivetrain.drive_for(FORWARD, 18)
    # Cata.spin_for(FORWARD, 100, DEGREES)
    
    # sleep(7, SECONDS)
    # drivetrain.set_drive_velocity(40, PERCENT)
    # drivetrain.set_turn_velocity(100, PERCENT)
    # drivetrain.drive_for(REVERSE, 4)
    # #pneumatic_on()
    # digital_out_a.set(False)
    # drivetrain.set_drive_velocity(100, PERCENT)
    # drivetrain.drive_for(REVERSE, 4)
    # drivetrain.turn_for(LEFT, 120, DEGREES)
    # drivetrain.drive_for(REVERSE, 4)
    # ######################################
    # Starting Position 1 (opposite goal) #
    # ######################################
    # Launch pre-load
    # Remove tri-ball from corner
    # Touch elevation bar

    # ######################################
    #   Starting Position 2 (goal side)   #
    # ######################################
    # Push pre=load into goal
    # Maybe push tri-ball in neutral zone into the goal
    # Touch elevation bar

    # ######################################
    #          Skills Autonomous          #
    # ######################################
    # Launch 44 triballs
    # Drive to goal on other side of field
    # Push triballs in

def autonomous_same():
    pass


def pneumatic_on():
    digital_out_a.set(True)
    digital_out_b.set(True)

def pneumatic_off():
    digital_out_a.set(False)
    digital_out_b.set(False)

def startup_brain():
    red_1 = [30, 46, 100, 50]
    red_2 = [30, 120 + 46, 100, 50]
    blue_1 = [20 + 130 + 170 + 20, 46, 100, 50]
    blue_2 = [20 + 130 + 170 + 20, 120 + 46, 100, 50]
    skills = [20 + 130 + 170 + 20, 170, 100, 50]
    brain.screen.set_font(FontType.MONO30)
    brain.screen.set_fill_color(Color.WHITE)
    brain.screen.draw_rectangle(0, 0, 480, 272)
    brain.screen.set_pen_color(Color.BLUE)
    #brain.screen.draw_rectangle(10, 10, 460, 74, Color.BLACK)
    brain.screen.set_cursor(1,9)
    brain.screen.print("94027A SharkBot")

    # Replace with photo of field
    brain.screen.draw_image_from_file('field.bmp', 155, 46)

    brain.screen.set_pen_color(Color.BLACK)

    # Create buttons
    brain.screen.set_pen_width(0)
    brain.screen.set_fill_color(Color.RED)
    brain.screen.draw_rectangle(red_1[0], red_1[1], red_1[2], red_1[3])
    brain.screen.set_cursor(3,3)
    brain.screen.print("Red 1")
    brain.screen.draw_rectangle(red_2[0], red_2[1], red_2[2], red_2[3])
    brain.screen.set_cursor(7,3)
    brain.screen.print("Red 2")

    brain.screen.set_fill_color(Color.BLUE)
    brain.screen.draw_rectangle(blue_1[0], blue_1[1], blue_1[2], blue_1[3])
    brain.screen.set_cursor(3,24)
    brain.screen.print("Blue 1")
    brain.screen.draw_rectangle(blue_2[0], blue_2[1], blue_2[2], blue_2[3])
    brain.screen.set_cursor(7,24)
    brain.screen.print("Blue 2")

    # Create Skills button
    brain.screen.set_fill_color(Color.YELLOW)
    brain.screen.draw_rectangle(skills[0], skills[1], skills[2], skills[3])
    brain.screen.set_cursor(11,24)
    brain.screen.print("Skills")

    while brain.screen.pressing() == False:
        wait(5, MSEC)

    #auton = 0
    if brain.screen.pressing():
        x = brain.screen.x_position()
        y = brain.screen.y_position()

        if x > red_1[0] and x < red_1[0] + red_1[2]:
            if y > red_1[1] and y < red_1[1] + red_1[3]:
                confirmation_brain(1)
                return 1
            if y > red_2[1] and y < red_2[1] + red_2[3]:
                confirmation_brain(2)
                return 2
        elif x > blue_1[0] and x < blue_1[0] + blue_1[2]:
            if y > blue_1[1] and y < blue_1[1] + blue_1[3]:
                confirmation_brain(3)
                return 3
            if y > blue_2[1] and y < blue_2[1] + blue_2[3]:
                confirmation_brain(4)
                return 4
            if y > skills[1] and y < skills[1] + skills[3]:
                confirmation_brain(5)
                return 5
        else:
            startup_brain()
    # brain.screen.print("Done")

def confirmation_brain(auton):
    global which_auton
    while brain.screen.pressing():
        wait(5, MSEC)
    brain.screen.set_fill_color(Color.WHITE)
    brain.screen.set_font(FontType.MONO20)
    brain.screen.draw_rectangle(0, 0, 480, 272)
    # brain.screen.draw_image_from_file('field.bmp', 155, 46)
    brain.screen.set_cursor(1, 3)

    if auton == 1:
        brain.screen.print("You selected: red alliance by the blue goal")
    elif auton == 2:
        brain.screen.print("You selected: red alliance by the red goal")
    elif auton == 3:
        brain.screen.print("You selected: blue alliance by the blue goal")
    elif auton == 4:
        brain.screen.print("You selected: blue alliance by the red goal")
    elif auton == 5:
        brain.screen.print("You selected: skills")

    yes = [135, 60, 200, 60]
    no = [135, 140, 200, 60]
    brain.screen.set_fill_color(Color.GREEN)
    brain.screen.draw_rectangle(yes[0], yes[1], yes[2], yes[3])
    brain.screen.set_cursor(5,16)
    brain.screen.print("Confirm selection")

    brain.screen.set_fill_color(Color.RED)
    brain.screen.draw_rectangle(no[0], no[1], no[2], no[3])
    brain.screen.set_cursor(9,16)
    brain.screen.print("Change selection")

    while brain.screen.pressing() == False:
        wait(5, MSEC)

    if brain.screen.pressing():
        x = brain.screen.x_position()
        y = brain.screen.y_position()

        if y > yes[1] and y < yes[1] + yes[3]:
            which_auton = auton
            match_brain()
        elif y > no[1] and y < no[1] + no[3]:
            startup_brain()
    

def match_brain():
    brain.screen.set_fill_color(Color.WHITE)
    brain.screen.draw_rectangle(0, 0, 480, 272)
    brain.screen.draw_image_from_file('shark.bmp', 30, -3)

# Set pneumatic buttons
controller_1.buttonUp.pressed(pneumatic_on)
controller_1.buttonDown.pressed(pneumatic_off)

pneumatic_off()
startup_brain()