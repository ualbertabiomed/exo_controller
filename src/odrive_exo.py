#!/usr/bin/env python3
import time
import odrive
from odrive.enums import *
import odrive.utils
import math
# from math_conversion import *
import fibre
import fibre.libfibre
import rospy
from std_msgs.msg import String, Float32

class ODrive_ROS:
    """
    ODrive ROS Node
    """

    # Declare variables
    odrv_ctrl = None
    TERMpub = None
    TERMsub = None

    def __init__(self):

        # Initialize ROS Node
        rospy.init_node("odrive_node", anonymous=True)

        # Initialize ODrive Control Object
        self.odrv_ctrl = ODrive_ctrl()

        # Initialize Publishers and Subscribers
        self.TERMsub = rospy.Subscriber("term_channel", String, self.term_callback)
        self.TERMpub = rospy.Publisher("odrive_info", String, queue_size = 10)

        # Try to connect to ODrive
        self.odrv_ctrl.connect()

    def term_callback(self, data):
        cmd_data = data.data.strip()
        rospy.loginfo(cmd_data)
        cmd_length = len(cmd_data)
        response = -1

        cmd = cmd_data[0]
        args = cmd_data[1:].strip()

        print("Got command: {} with data: {}".format(cmd, args))

        if (cmd == "p"):
            response = self.set_motor_position(args)
        elif (cmd == "c"):
            response = self.start_calibration(args)
        elif (cmd == "l"):
            response = self.set_limit(args)
        elif (cmd == "f"):
            response = self.print_motor_config(args)
        elif (cmd == "e"):
            response = self.print_error(args)
        else:
            rospy.loginfo("Undefined error")

    def user_loop(self):
        """
        For user control via menu
        """
        while (True):
            key = input("1. Calibrate\n2. Position Control Mode\n3. Velocity Control Mode\n4. Torque Control Mode\n5. Set Limits\n6. Dump Configs + Errors\nChoose a command (Press a number): ")
            print("\n")

            if key == '1':
            # Calibration -----
                key = input("1. Auto (Don't do this while motor is connect to arm)\n2. User\nChoose a calibration type: ")
                if key == '1':
                    # Don't do this while connected to arm
                    # self.odrv_ctrl.auto_calibrate()
                    pass
                elif key == '2':
                    # self.odrv_ctrl.user_calibrate()
                    pass
                else:
                    pass
            elif key == '2':
            # Position Control Mode ----- 
                key = ''
                while key != "q":
                    try:
                        print("Current Position [turns]: " + str(self.odrv_ctrl.get_position()))
                        print("\n")
                    except:
                        print("Cannot get position estimate")
                        print("\n")

                    key = input("Enter a position or press q to quit: ")

                    if key != 'q': 
                        try: 
                            self.odrv_ctrl.set_position(int(key))
                            time.sleep(1)
                        except: 
                            print("Cannot set position")
            elif key == '3':
            # Velocity Control Mode -----
                key = ''
                while key != 'q':
                    try:
                        print("Current Position [turns]: " + str(self.odrv_ctrl.get_position()))
                        print("Current Velocity [turns/s]: " + str(self.odrv_ctrl.get_velocity()))
                        print("\n")
                    except:
                        print("Cannot get position or velocity estimate")
                        print("\n")

                    key = input("Enter a velocity or press q to quit: ")

                    if key != 'q':
                        try: 
                            self.odrv_ctrl.set_velocity(int(key))
                            time.sleep(1)
                        except:
                            print("Cannot set velocity")
            elif key == '4':
            # Torque Control Mode -----
                key = ''
                while key != 'q':
                    try:
                        print("Current Position [turns]: " + str(self.odrv_ctrl.get_position()))
                        print("\n")
                    except:
                        print("Cannot get position estimate")
                        print("\n")

                    key = input("Enter a torque or press q to quit: ")

                    if key != 'q': 
                        try: 
                            self.odrv_ctrl.set_torque(int(key))
                            time.sleep(1)
                        except:
                            print("Cannot set torque")
            elif key == '5':
            # Set Limits
                print("Current Limit: " + str(self.odrv_ctrl.get_current_limit()) + "\nVelocity Limit: " + str(self.odrv_ctrl.get_velocity_limit())),
                key = input("\n1. Current Limit\n2. Velocity Limit\n3. Abort\nWhich limit would you like to change: ")
                if key == '1':
                    # Current
                    key = input("Enter a new Current Limit: ")
                    self.odrv_ctrl.set_current_limit(int(key))
                    print("New Current Limit: " + str(self.odrv_ctrl.get_current_limit()))
                    print("\n")
                elif key == '2':
                    # Velocity
                    key = input("Enter a new Velocity Limit: ")
                    self.odrv_ctrl.set_velocity_limit(int(key))
                    print("New Velocity Limit: " + str(self.odrv_ctrl.get_velocity_limit()))
                    print("\n")
                else:
                # Aborts with any other key
                    pass
            elif key == '6':
            # Dump Configs
                key = input("Reset Errors? [y/n]: ")
                if (key == "y"):
                    self.odrv_ctrl.dump_errors(True)
                else:
                    self.odrv_ctrl.dump_errors()
                print("\n")
                self.odrv_ctrl.dump_motor_config()
                print("\n")
                self.odrv_ctrl.dump_encoder_config()
                print("\n")

    def exo_loop(self):
        """
        For use in full system
        """
        pass

class ODrive_ctrl:
    """
    ODrive control commands - This class should get moved to the "include" folder
    """

    # Declare ODrive Related Variables
    odrv = None
    axis0 = None # Motor 0
    axis1 = None # Motor 1
    encoder_cpr = 8192
    encoder_kv = 192

    # Connection -----

    def connect(self):
        print("Finding an ODrive...")
        self.odrv = odrive.find_any()
        if self.odrv:
            self.axis0 = self.odrv.axis0
            self.axis1 = self.odrv.axis1
            print("Found an ODrive!")
            self.set_requested_state(self.axis0, AXIS_STATE_IDLE)
        else:
            print("Couldn't find an ODrive...")

    def reboot_odrive(self):
        print("Rebooting and Reconnecting")
        try:
            self.odrv.reboot()
        except:
            print("Expected connection loss due to reboot")
        # except fibre.ChannelBrokenException:
        #     print("Expected connection loss due to reboot")
        self.connect()

    def save_configuration(self):
        try:
            self.odrv.save_configuration()
            time.sleep(10)
            self.reboot_odrive()
            time.sleep(10)
        except fibre.libfibre.ObjectLostError:
            self.reboot_odrive()
            time.sleep(10) 

    # Calibration -----

    def auto_calibrate(self):
        '''
        Motor Calibration
        https://docs.odriverobotics.com/
        '''

        # Reset Configuration
        try:
            print("Erasing configuration...")
            self.odrv.erase_configuration()
            time.sleep(10)
            self.reboot_odrive()
            time.sleep(10)
        except fibre.libfibre.ObjectLostError:
            self.reboot_odrive()
            time.sleep(10) 

        # ----- Set the limit -----
        # self.axis0.motor.config.current_lim [A]
        # self.axis0.controller.config.vel_limit [turn/s]

        # TODO Figure out proper limits given motor specs
        self.axis0.motor.config.calibration_current = 10
        self.axis0.motor.config.current_lim = 10
        self.axis0.motor.config.resistance_calib_max_voltage = 3
        self.axis0.controller.config.vel_limit = 20000

        # ----- Set hardware parameters -----
        self.odrv.config.enable_brake_resistor = True
        self.odrv.config.dc_max_negative_current = -10 # [Amps]

        self.save_configuration();

        print("<axis>.requested_state: AXIS_STATE_FULL_CALIBRATION_SEQUENCE")
        self.set_requested_state(self.axis0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        time.sleep(10)

        # Check if motor is calibrated. Important for next step.
        print("<axis>.motor.is_calibrated: " + str(self.axis0.motor.is_calibrated))

        '''
        Encoder Calibration
        https://docs.odriverobotics.com/encoders
        '''

        # ----- Encoder with index signal -----

        # Startup Sequence Notes
        # self.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        # self.axis0.encoder.config.calib_range = 0.05
        # self.axis0.motor.config.calibration_current = 10.0
        # self.axis0.motor.config.resistance_calib_max_voltage = 12.0
        # self.axis0.controller.config.vel_limit = 5

        print("<axis>.encoder.config.cpr: " + str(self.axis0.encoder.config.cpr))
        print("<axis>.encoder.config.mode: " + str(self.axis0.encoder.config.mode))

        self.axis0.encoder.config.use_index = True

        print("<axis>.requested_state: AXIS_STATE_ENCODER_INDEX_SEARCH")
        self.set_requested_state(self.axis0, AXIS_STATE_ENCODER_INDEX_SEARCH)
        time.sleep(10)

        print("<axis>.requested_state: AXIS_STATE_ENCODER_OFFSET_CALIBRATION")
        self.set_requested_state(self.axis0, AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
        time.sleep(10)

        # This should be zero
        print("<axis0>.error: " + str(self.axis0.error))
        # This should print a number like -326 or 1364
        print("<axis0>.encoder.config.phase_offset: " + str(self.axis0.encoder.config.phase_offset))
        # This should print 1 or -1
        print("<axis0>.encoder.config.direction: " + str(self.axis0.encoder.config.direction))

        self.axis0.encoder.config.pre_calibrated = True

        # If you would like to search for the index at startup
        self.axis0.config.startup_encoder_index_search = False
        # If you are looking to start your machine as quickly as possible on bootup
        self.axis0.motor.config.pre_calibrated = True

        self.save_configuration()

        print("<axis>.requested_state: AXIS_STATE_FULL_CALIBRATION_SEQUENCE")
        self.set_requested_state(self.axis0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        time.sleep(10)

        print("<axis>.encoder.index_found: " + str(self.axis0.encoder.index_found))
        print("<axis>.encoder.is_ready: " + str(self.axis0.encoder.is_ready))
        print("<axis>.encoder.config.precalibrated: " + str(self.axis0.encoder.config.pre_calibrated))

        print("Auto-Calibration finished")
        print("\n")

    def user_calibrate(self):
        """
        This calibration is completely user driven, only sets values and then
        allows the user to freely move arm until the index is found
        - MUST do the above full calibration sequence at least once before this
        will work. i.e. run calibrate() with motor disconnected from arm, power
        down odrive, connect arm and restart odrive, run user_calibrate and
        manually rotate motor through the index position (marked with red line
        on motor)
        """

        # These 3 values are the ones we had to change from default, they will
        # change with different power supplies:
        #   Garage Power Supply: 3,3,5
        #   Turnigy ReaktorPro: 10, 10, 3 (Defaults)
        #   https://docs.google.com/spreadsheets/d/12vzz7XVEK6YNIOqH0jAz51F5VUpc-lJEs3mmkWP1H4Y/edit#gid=0 --> Motor limits

        # ----- Set the limit -----
        # self.axis0.motor.config.current_lim [A]
        # self.axis0.controller.config.vel_limit [turn/s]
        # self.axis0.motor.config.calibration_current [A]

        # TODO Figure out proper limits given motor specs
        self.axis0.motor.config.calibration_current = 10
        self.axis0.motor.config.current_lim = 10
        self.axis0.motor.config.resistance_calib_max_voltage = 3
        self.axis0.controller.config.vel_limit = 20000

        # ----- Set hardware parameters -----
        self.odrv.config.enable_brake_resistor = True
        self.odrv.config.dc_max_negative_current = -10
        self.odrv.config.max_regen_current = 10

        self.save_configuration()
                
        print("Delaying 10 seconds for manual calibration movement")
        time.sleep(10)

        print("<axis>.encoder.index_found: " + str(self.axis0.encoder.index_found))
        print("<axis>.encoder.is_ready: " + str(self.axis0.encoder.is_ready))
        print("<axis>.encoder.config.precalibrated: " + str(self.axis0.encoder.config.pre_calibrated))

        print("User-Calibration finished")
        print("\n")

    # Getters -----

    def get_position(self):
        try:
            return self.axis0.encoder.pos_estimate
        except:
            print("Cannot get position")

    def get_velocity(self):
        try:
            return self.axis0.encoder.vel_estimate
        except:
            print("Cannot get velocity")

    def get_current_limit(self):
        try:
            return self.axis0.motor.config.current_lim
        except:
            print("Cannot get current limit")

    def get_velocity_limit(self):
        try:
            return self.axis0.controller.config.vel_limit
        except:
            print("Cannot get velocity limit")

    # Setters -----

    def set_control_mode(self, axis, mode):
        """
        https://docs.odriverobotics.com/commands.html
        https://docs.odriverobotics.com/api/odrive.controller.controlmode

        CONTROL_MODE_VOLTAGE_CONTROL = 0 (this one is not normally used)
        CONTROL_MODE_TORQUE_CONTROL = 1
        CONTROL_MODE_VELOCITY_CONTROL = 2
        CONTROL_MODE_POSITION_CONTROL = 3
        """
        if mode in range(4):
            axis.controller.config.control_mode = mode
        else:
            print("Error: Invalid control mode")

    def set_requested_state(self, axis, state):
        """
        https://docs.odriverobotics.com/api/odrive.axis.axisstate

        AXIS_STATE_UNDEFINED = 0
        AXIS_STATE_IDLE = 1
        AXIS_STATE_STARTUP_SEQUENCE = 2
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
        AXIS_STATE_MOTOR_CALIBRATION = 4
        AXIS_STATE_SENSORLESS_CONTROL = 5
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8
        AXIS_STATE_LOCKIN_SPIN = 9
        AXIS_STATE_ENCODER_DIR_FIND = 10
        """
        if state in range(11):
            axis.requested_state = state
        else:
            print("Error: Invalid requested state")

    def set_position(self, position):
        try:
            self.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
            self.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            # <axis>.controller.input_pos = <turn>
            self.axis0.controller.input_pos = position
            # print("Set position to: " + str(position) + " encoder units")
            print("Set position to: " + str(position) + " turns")
        except:
            print("Cannot set position")

    
    def set_velocity(self, velocity):
        try:
            self.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
            self.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            # <axis>.controller.input_vel = <turn/s>
            self.axis0.controller.input_vel = velocity
            # print("Set Velocity to: " + str(velocity) + "counts/s (Non-Ramped)")
            print("Set velocity to: " + str(velocity) + " turn/s")
        except:
            print("Cannot set velocity")

    def set_torque(self, torque):
        try:
            self.set_requested_state(self.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL)
            self.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            # <axis>.controller.input_torque = <torque in Nm>
            self.axis0.controller.input_torque = torque
            print("Set torque to: " + str(torque) + " torque in Nm")
        except:
            print("Cannot set torque")

    def set_current_limit(self, new_current_lim):
        self.axis0.motor.config.current_lim = new_current_lim
        print("Set global current limit = " + str(new_current_lim))

    def set_velocity_limit(self, new_velocity_lim):
        self.axis0.controller.config.vel_limit = new_velocity_lim
        print("Set global velocity limit = " + str(new_velocity_lim))

    # Config and Error Dumps -----

    def dump_errors(self, clear=False):
        odrive.utils.dump_errors(self.odrv, clear)

    def dump_motor_config(self):
        print(self.axis0.motor.config)

    def dump_encoder_config(self):
        print(self.axis0.encoder.config)

if __name__ == "__main__":
    odrv_node = ODrive_ROS()
    while (True):
        odrv_node.user_loop()
        rospy.spin()