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

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
Cata_motor_a = Motor(Ports.PORT19, GearSetting.RATIO_36_1, False)
Cata_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_36_1, True)
Cata = MotorGroup(Cata_motor_a, Cata_motor_b)
Cata.set_velocity(80, PERCENT)
left_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_motor_b = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
digital_out_a = DigitalOut(brain.three_wire_port.a)
digital_out_b = DigitalOut(brain.three_wire_port.b)


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
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, controller_1_right_shoulder_control_motors_stopped, remote_control_code_enabled
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
        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

#endregion VEXcode Generated Robot Configuration
def autonomous():
    #######################################
    # Starting Position 1 (opposite goal) #
    #######################################
    # Launch pre-load
    # Remove tri-ball from corner
    # Touch elevation bar

    #######################################
    #   Starting Position 2 (goal side)   #
    #######################################
    # Push pre=load into goal
    # Maybe push tri-ball in neutral zone into the goal
    # Touch elevation bar

    #######################################
    #          Skills Autonomous          #
    #######################################
    # Launch 44 triballs
    # Drive to goal on other side of field
    # Push triballs in

    pass


def pneumatic_on():
    digital_out_a.set(True)
    digital_out_b.set(True)

def pneumatic_off():
    digital_out_a.set(False)
    digital_out_b.set(False)

controller_1.buttonUp.pressed(pneumatic_on)
controller_1.buttonDown.pressed(pneumatic_off)

