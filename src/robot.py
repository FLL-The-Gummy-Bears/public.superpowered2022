"""
MIT License

Copyright (c) 2023 FLL-The-Gummy-Bears

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""

""" Module containing code necessary for robot navigation"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, GyroSensor, ColorSensor)
from pybricks.media.ev3dev import ImageFile, SoundFile, Font
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.iodevices import Ev3devSensor
from pybricks.tools import wait, DataLog, StopWatch
from math import fabs

#this is the robot class that has all the navigation blocks and sensor checking

class Robot:

    def __init__(self, name = "MyRobot"):
        """ Constructor
        Args:
            name (str) : Name of robot 
        
        Returns:
            Nothing
        """
        self.name = name
        self.wheel_diameter = 81.6
        self.axle_track = 107

        self.ev3 = EV3Brick()

        small_font = Font(size=12)
        large_font = Font(size=14)

        self.ev3.screen.set_font(small_font)

        # create robot timer and datalog for global recording
        # datalog will record distance, gyro angle, left color reflection, right color reflection, large motor speed (left/right), medium motor speed (left/right)

        self.timer=StopWatch()
        self.RecordOn= False
        self.datalog = DataLog('time', 'distance', 'gyroangle', 'lcolor','rcolor', "llspd",'rlspd',"lmspd","rmspd", name="log_robot", timestamp= False, extension = 'csv', append=True)

        failedsensor = 0
        #assuming all sensors/motors are OK
        while True:
            try: 
                self.gyro_port = Port.S2
                self.gyro = GyroSensor(port = self.gyro_port)
            except:
                self.gyro = None
                self.ev3.screen.print("PORT 2 GYRO NOT FOUND!!!")
                failedsensor += 1
            try:
                self.lcolor = ColorSensor(port = Port.S1)
            except:
                self.lcolor = None
                self.ev3.screen.print("PORT 1 COLOR NOT FOUND!!!")
                failedsensor += 1
            try:
                self.rcolor = ColorSensor(port = Port.S3)
            except:
                self.rcolor = None
                self.ev3.screen.print("PORT 3 COLOR NOT FOUND!!!")
                failedsensor += 1
            try:
                self.llmotor = Motor(port = Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
            except:
                self.llmotor = None
                self.ev3.screen.print("PORT B MOTOR NOT FOUND!!!")
                failedsensor += 1
            try:
                self.rlmotor = Motor(port = Port.C, positive_direction = Direction.COUNTERCLOCKWISE)
            except:
                self.rlmotor = None
                self.ev3.screen.print("PORT C MOTOR NOT FOUND!!!")
                failedsensor += 1
            try:
                self.lmmotor = Motor(port = Port.A, positive_direction = Direction.COUNTERCLOCKWISE)
            except:
                self.lmmotor = None
                self.ev3.screen.print("PORT A MOTOR NOT FOUND!!!")
                failedsensor += 1
            try:
                self.rmmotor = Motor(port = Port.D, positive_direction = Direction.COUNTERCLOCKWISE)
            except:
                self.rmmotor = None
                self.ev3.screen.print("PORT D MOTOR NOT FOUND!!!")
                failedsensor += 1
            if failedsensor == 0:
                break
            else:
                failedsensor = 0
                self.ev3.screen.print("Please check cable connection + replug!")
                wait(2000)

        self.driver = DriveBase(self.llmotor, self.rlmotor, self.wheel_diameter, self.axle_track)

    def datalog(self, data_logger):
        """ Write Data Log File for Robot Navigations (time,distance, gyro angle, left color reflection, right color reflection, 
            large motor speed (left/right), medium motor speed (left/right) )
        Args:
            data_logger (binary): True - Turn on data logging, False: Turn off data logging
        """
        if self.RecordOn is True: data_logger=True

        if data_logger is True:
            self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                             self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())
        else:
            pass

    def linesquaring(self, align_color, align_speed, colormode=False,interval=2, data_logger=False, timer_reset=False):
        """ Uses color or rli to align on colored lines
        Args:
            align_color (str) : "white", "black"
            align_speed (int) : speed
            colormode(binary) : True - use color mode, False - use reflection mode
            interval : time interval for loop, default 10, millisecond
        """
        if self.RecordOn is True: data_logger=True

        if timer_reset is True: 
            self.timer.reset()

        if align_color == "black":
            target_reflection = 15
        elif align_color == "white" :
            target_reflection = 90
        else:
            raise RuntimeError("Invalid alignment color, input 'black' or 'white' only")
                
        leftwheelbrake=False
        rightwheelbrake=False
    
        if colormode is True:
            pass
        else:
            while True:
            
                if align_color == "black": 
                
                    # Move forward
                    left_reflection = self.lcolor.reflection()
                    right_reflection = self.rcolor.reflection()

                    if data_logger is True:
                        self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                        self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())
                    # Stop left motor after detecting black 
                    if left_reflection <= target_reflection:
                        leftwheelbrake=True
                        self.llmotor.hold()
                    # Stop right motor after detecting black
                    if right_reflection <= target_reflection :
                        rightwheelbrake=True
                        self.rlmotor.hold()
                    # If both sensors detect black, stop the robot

                else: # Align white
                    
                    # Move forward
                    left_reflection = self.lcolor.reflection()
                    right_reflection = self.rcolor.reflection()

                    if data_logger is True:
                        self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                        self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())

                    # Stop left motor after detecting white
                    if left_reflection >= target_reflection:
                        leftwheelbrake=True
                        self.llmotor.hold()
                    # Stop right motor after detecting white
                    if right_reflection >= target_reflection :
                        rightwheelbrake=True
                        self.rlmotor.hold()
                    # If both sensors detect white, stop the robot
                
                if rightwheelbrake and leftwheelbrake:
                    break

                self.llmotor.run(align_speed)
                self.rlmotor.run(align_speed)
                if interval == 0:
                    pass
                else:
                    wait(interval)

            self.llmotor.hold()
            self.rlmotor.hold()

            current_time = self.timer.time()
            while self.timer.time()<=current_time+ 200:
                if data_logger is True:
                    self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                    self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())

            self.llmotor.stop()
            self.rlmotor.stop()

    def check_turns(self, turn_angle, turn_rate, spinturn):

        # check turn direction(left/right) based on user input of speed (+/-) and angle (+/-)
        # if sign(speed) * sign(angle) > 0 --> left turn (angle_change > 0)
        # if sign(speed) * sign(angle) < 0 --> right turn (angle_change < 0)

        if turn_angle * turn_rate > 0:
            turn_direction=1
            angle_change=fabs(turn_angle)
        else: 
            turn_direction=-1
            angle_change=-fabs(turn_angle)

        # check turn type (spin/pivot) to determine motor rotation direcitons for each large motoer
        # left pivot turn, brake the left motor while turnning the right motor forward
        # right pivot turn, , brake the right motor while turnning the left motor forward
        # left spin turn, left motor backward and right motor forward
        # right spin turn, left motor forward and right motor backward

        if angle_change > 0 and not spinturn: 
            direction_llmotor=0
            direction_rlmotor=1
        elif angle_change <= 0 and not spinturn: 
            direction_llmotor=1
            direction_rlmotor=0
        elif angle_change > 0 and spinturn: 
            direction_llmotor=-1
            direction_rlmotor=1
        else: 
            direction_llmotor=1
            direction_rlmotor=-1            

        return turn_direction, angle_change, direction_llmotor, direction_rlmotor

    def gyro_turn(self, turn_angle, turn_rate, spinturn=True, interval=0, wait_time=200, adaptive_precision = True, data_logger=False, timer_reset= False ):
        """ Use gyro sensor for turning
        Args:
            turn_angle (int) : angle in deg
            turn_rate (int) : turn rate of the robot (deg/s)
            spinturn(bool): flag for indicating whether to do a spin turn or pivot turn. Default: true
            data_logger (boolean) : default False, if True will turn on debugger mode to record the data of gyro
            timer (StopWatch object): default None, use global timer for debug purpose
            interval : time interval for loop, default 10, milliseconds
        """
        if self.RecordOn is True: data_logger=True

        # setup turn time protection incase of getting stuck
        max_time = max(fabs(turn_angle/turn_rate)*3000,2000)

        turn_starting_time=self.timer.time()

        # Reset Robot timer
        if timer_reset is True: 
            self.timer.reset()

        if data_logger is True:
            self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                             self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())
            
        turn_speed = fabs(turn_rate)

        turn_direction, angle_change, direction_llmotor, direction_rlmotor = self.check_turns(turn_angle, turn_rate, spinturn)

        # adaptive precision based on input power, need tests to find best calibrations
        if adaptive_precision is True:
            precision = int(turn_speed/30)
            if precision > 12: 
                precision = 12
        else:
            precision = 1.0;

        angle_ini = self.gyro.angle()
        angle_target = angle_ini + angle_change

        gyro_angle = self.gyro.angle()

        while True:

            #if reached max stall time, robot breaks the loop and stops turning
            if self.timer.time() >= max_time + turn_starting_time:
                break
            #robot turns based on direction of each motor
            self.llmotor.run(direction_llmotor * turn_speed)
            self.rlmotor.run(direction_rlmotor * turn_speed)

            # data logger output            
            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())

            if (gyro_angle - angle_target) * turn_direction  >= -precision  :
                break

            if interval == 0:
                pass
            else:
                wait(interval)
            gyro_angle = self.gyro.angle()        
            
        #hold motor for stop
        self.llmotor.hold()
        self.rlmotor.hold()

        current_time = self.timer.time()
        while self.timer.time() <=current_time + wait_time:
            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())               

        #complete stop
        self.llmotor.stop()
        self.rlmotor.stop()

        angle_final = self.gyro.angle()
        error = angle_final - angle_target

        if data_logger is True:
            self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                             self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())   
        return error

    def gyroturn_correction(self, turn_angle, turn_rate, spinturn=True, interval=10, debug_mode=False):
        """ correcting gyro sensor angle
        Args:
            turn_angle (int) : angle in mm
            turn_rate (int) : turn rate of the robot  (deg/s)
            spinturn(bool): flag for indicating whether to do a spin turn or pivot turn. Default: true
            interval : time interval for loop, default 10, milliseconds
        """

        # check turn direction(left/right) based on user input of speed (+/-) and angle (+/-)
     
        turn_speed = fabs(turn_rate)
        turn_direction, angle_change, direction_llmotor, direction_rlmotor = self.check_turns(turn_angle, turn_rate, spinturn)

        # coarse turn with high speed
        angle_ini = self.gyro.angle()
        angle_target = angle_ini + angle_change

        error1 = self.gyro_turn(turn_angle=angle_change, turn_rate = turn_speed, spinturn=spinturn,interval=interval)
        angle_end1=self.gyro.angle()

        # fine correction turn with lower speed
        error2 = self.gyro_turn(turn_angle=-error1, turn_rate=30, spinturn=True,interval=interval)
        angle_end2=self.gyro.angle()

        error = angle_end2 - angle_target

        # in debug mode print out angle and error information
        if debug_mode is True:
            print ("starting angle:",angle_ini, "target angle: ", angle_target, "angle1:", angle_end1, "angle2:", angle_end2)
            print ( "error1:", error1, "error2:", error2, "error:", error)

        return error

    def wall_align(self, motor_speed, align_time):
        """ Both motors run for a certain time to align on the wall
        
        Args:
            motor_speed (int) : speed
            align_time (int) : amount of time the motors will run in miliseconds
        
        """ 
        self.llmotor.run_time(motor_speed, align_time, wait=False)
        self.rlmotor.run_time(motor_speed, align_time, wait=True)

    def debug_mode(self, option_num=1):
    
        """ Choose 1 out of 3 choices of debug modes.
    
        Args: 
        option_num (int) :  1. To pass 2. Wait 0.2 seconds 3. Wait for brick button press
        """
    
        # Wait until any Brick Button is pressed.
        if option_num == 1:
            pass

        elif option_num == 2:
            wait(200)

        elif option_num == 3:
            while True:
                if Button.CENTER in self.ev3.buttons.pressed():
                    break
                
    def complete_stop (self, stop_option=1):
        """robot complete stop
        Choose 1 out of 3 choices of complete stop modes.
    
        Args: 
        option_num (int) :  1. use hold 2. use brake 3. use stop 4. hold then stop to avoid out of order
        """
        self.driver.stop()
        if stop_option == 1:
            self.llmotor.hold()
            self.rlmotor.hold()

        elif stop_option == 2:
            self.llmotor.brake()
            self.rlmotor.brake()

        elif stop_option == 3:
            self.llmotor.stop()
            self.rlmotor.stop()

        elif stop_option == 4:
            self.llmotor.hold()
            self.rlmotor.hold()
            wait(20)
            self.llmotor.stop()
            self.rlmotor.stop()
        
    def drive_straight(self,speed,distance,acc=759, stop=True):
        """Drive straight without PID
        
        Args:
            speed (int) : speed in mm/s
            distance (int) : distance in mm, Positive = forward, Negative = backwards
            acc (int) : acceleartion of the robot, Default value = 759
            stop (binary): complete stop at end of driving
        """
        self.driver.settings(straight_speed=speed,straight_acceleration=acc)
        self.driver.straight(distance=distance)

        if stop is True:
            self.complete_stop()
    
    def check_drive_direction(self, speed, drive_distance):
        # Update driving direction based on user input of speed (+/-) and distance (+/-): 
        # if sign(speed) * sign (drive_distance) > 0 --> forward driving with speed of abs(speed)
        # if sign(speed) * sign (drive_distance) <= 0 --> backward driving with speed of -abs(speed)

        if speed * drive_distance > 0: # forward driving
            drive_direction = 1
        else:                          # backward driving
            drive_direction = -1
        
        drive_speed = drive_direction * fabs(speed)
        return drive_speed, drive_direction

    def pid_drive(self, speed, drive_distance, straight_acc=400, turn_acc=10, kp=2.0, ki=0, kd=0, feedforward=0, interval=0, data_logger=False, timer_reset= False, stop=True): 
        """ PID(Proportional | Integral | Derivative) drive - go straight 
        Args:
            speed (int) : speed
            drive_distance (int) : distance in mm
            straight_acc (int): linear acceleration, unused
            turn_acc (int): turn acceleration, unused
            kp (int) : proportional gain
            ki (int) : integral gain
            kd (int) : derivative gain
            positive number for feedforward is going right and negative is going left
            data_logger (boolean) : default False, if True will turn on debugger mode to record the data of gyro and distance
            # To-do: Test the values for the Ki and Kd. 
            # Ki should be somewhere from 0.1-0.25 (Not completely sure)
            # Kd changed slightly will make the robot become very crazy, Kd value should be less than 1 (Not completely sure)
            interval : time interval for loop, default 10, milliseconds
        """
        if self.RecordOn is True: data_logger=True

        distance_ini = self.driver.distance()
        target_angle = self.gyro.angle()

        Kp = kp
        Ki = ki
        Kd = kd

        error_last = 0
        error_derivative = 0
        error_intergral = 0.0

        if timer_reset is True:
            self.timer.reset()

        # Update driving direction based on user input of speed (+/-) and distance (+/-): 
        drive_speed, drive_direction = self.check_drive_direction(speed , drive_distance)

        # Drive linear acc and turn acc settings (not used, need to double check)
    #    self.driver.settings(straight_acceleration=straight_acc) #turn_acceleration=turn_acc

        while True:

            distance_travelled_abs = fabs(self.driver.distance() - distance_ini)
            current_angle = self.gyro.angle()
            error = current_angle - target_angle

            error_derivative = error - error_last
            error_last = error
            error_intergral = error_intergral + error

            steering_correction = error * Kp + error_intergral * Ki + error_derivative * Kd+feedforward
                        
            if distance_travelled_abs >= fabs(drive_distance):
                break
            else:
                    self.driver.drive(drive_speed, steering_correction)

            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())

            if interval == 0:
                pass
            else:
                wait(interval)

        if stop is True:
            self.complete_stop()

    def followline(self, drive_speed, drive_time):
        """ Follow between the black and white black line for fixed amount of time
            Args:
                drive_speed (int) : speed, user specified speed(millimeters per second) for line following
                drive_time (int) : time (milliseconds), user specified time for line following
        """
            
        # Calculate the light threshold. Choose values based on your measurements.
        BLACK = 11
        WHITE = 96
        threshold = (BLACK + WHITE) / 2

        # Set the gain of the proportional line controller. This means that for every
        # percentage point of light deviating from the threshold, we set the turn
        # rate of the drivebase to 2.0 degrees per second.
        KP = 2.0

        # Start following the line endlessly.
        timer1 = StopWatch()
        while True:
            # Calculate the deviation from the threshold.
            error = self.rcolor.reflection() - threshold

            # Calculate the turn rate.
            turn_rate = KP * error

            # Set the drive base speed and turn rate.
            self.driver.drive(drive_speed, turn_rate)

            # You can wait for a short time or do other things in this loop.
            wait(10)
            
            if timer1.time() > drive_time:
                break

    def gyro_calib(self):
        """ Gyro sensor calibration via mode switching ("GYRO-CAL" --> "GYRO-ANG")
        """
        print("robot: gyro_calib")
        self.ev3.light.on(Color.RED)
        wait(200)
        gyro = Ev3devSensor(self.gyro_port)

        for i in range(3):
            gyro.read("GYRO-CAL")
            wait(200)
            angle = int(gyro.read("GYRO-ANG")[0]) 
            if angle == 0:
                print("gyro calib done!", i)
                break
        wait(200)
        self.gyro.reset_angle(0)
        self.ev3.speaker.beep()
        self.ev3.light.on(Color.GREEN)

    def robot_warmup(self, warmup_time=8000):
        """This function warms up robot's motors so it doesn't have problem loading program

        Args:
            warmup_time: how long motors should warm up (milliseconds)
        """
        self.rmmotor.run_time(speed=200,time=warmup_time,then=Stop.HOLD, wait=True)
        self.rmmotor.stop()
        self.lmmotor.run_time(speed=200,time=warmup_time,then=Stop.HOLD, wait=True)
        self.lmmotor.stop()

        self.rlmotor.run_time(speed=200,time=warmup_time,then=Stop.HOLD, wait=True)
        self.rlmotor.stop()
        self.llmotor.run_time(speed=200,time=warmup_time,then=Stop.HOLD, wait=True)
        self.llmotor.stop()

        self.rmmotor.reset_angle(0)
        self.lmmotor.reset_angle(0)
        self.rlmotor.reset_angle(0)
        self.llmotor.reset_angle(0)

        self.ev3.speaker.beep()

    def check_sensor_reading (self, waitinterval=2):
        """This function check sensor readings at specified port

        Args:
            waitinterval: wait interval between each check (milliseconds)
        """
        mysensor=Ev3devSensor(self.gyro_port)
        mysensor_path = '/sys/class/lego-sensor/sensor' + str(mysensor.sensor_index)
        modes_path = mysensor_path + '/modes'

        with open(modes_path, 'r') as m:
            # Read the contents.
            contents = m.read()
            # Strip the newline symbol, and split at every space symbol.
            modes= contents.strip().split(' ')
        
        print(modes)
        index=0
        try:
            while True:
                index +=1
                for mode in modes:
                    val=mysensor.read(mode)
                    if index %25 == 0:
                        print("loop-", index, "mode:", mode, "readings:", val)

                    wait(waitinterval)
        except:
            print("caught error")

    def drive_ramp(self,speed,distance,acc=759, ramp_up_factor = 0.1, ramp_down_factor = 0.1, interval = 0, stop=True):
        """Moves straight without PID
        
        Args:
            speed (int) : speed in mm/s
            distance (int) : distance in mm, Positive = forward, Negative = backwards
            acc (int) : acceleartion of the robot, Default value = 759
            ramp_up_factor (int) : distance factor of ramp up
            ramp_down_factor (int) : distance factor of ramp down
        """
        if self.RecordOn is True: data_logger=True

        distance_ini = self.driver.distance()

        # Update driving direction based on user input of speed (+/-) and distance (+/-): 
        driver_speed, drive_direction = self.check_drive_direction(speed , distance)

        # set min/max speed
        min_speed = 30
        max_speed = fabs(speed)
        distance_abs = fabs(distance)  # use absolute distance as distance measure

        # calculate ramp up/ramp down distance
        ramp_up_distance = ramp_up_factor*distance_abs
        ramp_down_distance = ramp_down_factor*distance_abs
        # calculate the slopes for ramp up and ramp down
        slope_ramp_up = (max_speed - min_speed)/(ramp_up_distance)
        slope_ramp_down = (min_speed - max_speed)/(ramp_down_distance)

        while True:

            # create speed ramp up and ramp down function based on distance travelled
            distance_travelled_abs = fabs(self.driver.distance() - distance_ini)
            speed_func_up = min_speed + slope_ramp_up * (distance_travelled_abs -  0)
            speed_func_down = max_speed + slope_ramp_down * (distance_travelled_abs -(distance_abs - ramp_down_distance))

            # apply speed ramp up for starting and speed ramp down when approaching, otherwise constant speed
            if distance_travelled_abs <= ramp_up_distance:
                drive_speed = speed_func_up
            elif distance_travelled_abs >= distance_abs - ramp_down_distance:
                drive_speed = speed_func_down
            else:
                drive_speed = max_speed 

            # driving with the speed function
            self.driver.drive(speed = drive_direction * drive_speed, turn_rate=0)

            # break loop when approaching distance target
            if distance_travelled_abs >= fabs(distance):
                break

            if interval == 0:
                pass
            else:
                wait(interval)

        if stop is True:
            self.complete_stop()
    
    def turn(self, turn_rate, turn_angle, turn_acceleration=500):
        """Standard driverbase turn (no Gyro)
        
        Args:
            turn_rate (int) : turn speed in deg/s
            turn_angle (deg) : turn angle in deg, Positive = left, Negative = right
            turn_acceleration (int) : turn acceleartion of the robot, Default value = 200
            turn_rate, turn_acceleration
        """
        turn_angle_reverse = -turn_angle # convert to right turn negative, left turn positive
        self.driver.settings(turn_rate=turn_rate,turn_acceleration=turn_acceleration)
        self.driver.turn(turn_angle_reverse)
        self.complete_stop()
    
    def turn_correction(self, turn_rate , turn_angle, turn_acceleration=500, debug_mode = False):
        """Standard driverbase turn with correction, direction wise--> right : positive, left: negative, 
        only appy gyro twice to update the tune error
        
        Args: 
            turn_rate (int) : turn speed in deg/s
            turn_angle (deg) : turn angle in deg, Positive = Left, Negative = right
            turn_acceleration (int) : turn acceleration of the robot, Default value = 200
            turn_rate, turn_acceleration
        
        """
        start_angle = self.gyro.angle() 
        target_angle = start_angle + turn_angle # right turn is positive, left turn is negative

        self.turn(turn_rate = turn_rate, turn_angle= turn_angle, turn_acceleration= turn_acceleration)
        wait(200)
        angle_1 = self.gyro.angle()
        error1 = angle_1 - target_angle

        self.turn(turn_rate= 30, turn_angle= -error1, turn_acceleration= turn_acceleration)
        # mixed with gyro turn for better accuracy
    #    self.gyro_turn(angle_deg=-error1, speed=30, spinturn=True,interval=10)
        wait(200)

        angle_2 = self.gyro.angle()
        error2 = angle_2 - target_angle
        
        if debug_mode is True:
            print("start angle: ", start_angle, "target_angle", target_angle, "angles", angle_1, angle_2, "errors:", error1, error2)        

    def hybrid_turn_correction(self, turn_rate , turn_angle, turn_acceleration=500, debug_mode = False):
        """Standard driverbase turn with 2nd turn based on gyro (Spin Turn), direction wise--> right : positive, left: negative
        
        Args: 
            turn_rate (int) : turn speed in deg/s
            turn_angle (deg) : turn angle in deg, Positive = Left, Negative = right
            turn_acceleration (int) : turn acceleration of the robot, Default value = 200
            turn_rate, turn_acceleration
        
        """
        start_angle = self.gyro.angle() 
        target_angle = start_angle + turn_angle # right turn is positive, left turn is negative

        # First turn is standard turn
        self.turn(turn_rate = turn_rate, turn_angle= turn_angle, turn_acceleration= turn_acceleration)
        wait(200)
        angle_1 = self.gyro.angle()
        error1 = angle_1 - target_angle

        # Second turn is gyro turn with slow speed 30
        self.gyro_turn(turn_angle=-error1, turn_rate=50, spinturn=True, interval = 10)
        wait(200)

        angle_2 = self.gyro.angle()
        error2 = angle_2 - target_angle
        
        if debug_mode is True:
            print("start angle: ", start_angle, "target_angle", target_angle, "angles", angle_1, angle_2, "errors:", error1, error2)        

    def drive_time(self, speed, drive_time, turn_rate=0, straight_acceleration=700, turn_acceleration=500, interval=0, data_logger=False, timer_reset= False, stop=True):
        """Standard driverbase drive for time (no Gyro)
        
        Args:
            speed (int) : drive speed in mm/s
            turn_rate (int): turn speed in deg/s
            drive_time(int): drive time in milisec
            straight_acceleration (int) : straight_acceleration of the robot, Default value = 759
            turn_acceleration (int) : turn acceleartion of the robot, Default value = 100
            interval (int): wait interval for the loop, milisec
        """
        if self.RecordOn is True: data_logger=True

        self.driver.settings(straight_acceleration=straight_acceleration, turn_acceleration=turn_acceleration)
        if timer_reset is True:
            self.timer.reset()

        starting_time=self.timer.time()
        
        while self.timer.time() <= drive_time + starting_time:
            self.driver.drive(speed, turn_rate)
            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())
            if interval == 0:
                pass
            else:
                wait(interval)

        if stop is True:
            self.complete_stop()

    def drive_distance (self, speed, turn_rate, drive_distance, straight_acceleration=700, turn_acceleration=500, interval=0, data_logger=False, timer_reset= False, stop=True):
        """Standard driverbase drive for distance (no Gyro)
        
        Args:
            speed (int) : drive speed in mm/s
            turn_rate (int): turn speed in deg/s
            drive_distance(int): drive distance in mm
            straight_acceleration (int) : straight_acceleration of the robot, Default value = 759
            turn_acceleration (int) : turn acceleartion of the robot, Default value = 100
            interval (int): wait interval for the loop, milisec
        """
        if self.RecordOn is True: data_logger=True

        self.driver.settings(straight_acceleration=straight_acceleration, turn_acceleration=turn_acceleration)
        distance_ini=self.driver.distance()

        if timer_reset is True:
            self.timer.reset()

        starting_time=self.timer.time()
        
        while fabs(self.driver.distance()-distance_ini) < fabs(drive_distance):
            self.driver.drive(speed, turn_rate)
            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())
            if interval == 0:
                pass
            else:
                wait(interval)

        if stop is True:
            self.complete_stop()
        
    def drive_straight_pastwarning(self, speed, max_distance, distance_after_warning, data_logger=False, stop = True):
        """ 
        
        """
        if self.RecordOn is True: data_logger=True

        initial_distance = self.driver.distance()
        max_distance_abs=fabs(max_distance)
        distance_after_warning_abs = fabs(distance_after_warning)

        while True:
            self.driver.drive(speed, 0)
            distance_travelled_abs = fabs(self.driver.distance() - initial_distance)

            left_reflection = self.lcolor.reflection()
            right_reflection = self.rcolor.reflection()
            print("left reflection: ", left_reflection, "right reflection: ", right_reflection)

            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())
                
            # update max distance once color line detected
            if left_reflection >= 90 or right_reflection >= 90: 
                max_distance_abs = distance_after_warning_abs + distance_travelled_abs
                print ("white line detected, max distance updated to: ", max_distance_abs)

            if distance_travelled_abs >= max_distance_abs:
                print ("max distance reached!")
                break

        if stop is True:    
            self.complete_stop()

    def pid_rampdrive(self, speed, drive_distance, straight_acc=400, ramp_up_factor = 0.1, ramp_down_factor = 0.1, turn_acc=10, kp=2.0, ki=0, kd=0, feedforward=0, interval=0, data_logger=False, timer_reset= False, stop=True): #works for both negative speed/distance
        """ PID(Proportional | Integral | Derivative) drive - go straight with ramp-up/ramp-down power when starting/approaching 
        Args:
            speed (int) : speed
            drive_distance (int) : distance in mm
            straight_acc (int): linear acceleration, unused
            turn_acc (int): turn acceleration, unused
            kp (int) : proportional gain
            ki (int) : integral gain
            kd (int) : derivative gain
            positive number for feedforward is going right and negative is going left
            data_logger (boolean) : default False, if True will turn on debugger mode to record the data of gyro and distance
            # To-do: Test the values for the Ki and Kd. 
            # Ki should be somewhere from 0.1-0.25 (Not completely sure)
            # Kd changed slightly will make the robot become very crazy, Kd value should be less than 1 (Not completely sure)
            interval : time interval for loop, default 10, milliseconds
        """
        if self.RecordOn is True: data_logger=True

        distance_ini = self.driver.distance()
        target_angle = self.gyro.angle()

        Kp = kp
        Ki = ki
        Kd = kd

        error_last = 0
        error_derivative = 0
        error_intergral = 0.0

        if timer_reset is True:
            self.timer.reset()

        # Update driving direction based on user input of speed (+/-) and distance (+/-): 
        driver_speed, speed_direction = self.check_drive_direction(speed , drive_distance)

        # Drive linear acc and turn acc settings (not used, need to double check)
        #self.driver.settings(straight_acceleration=straight_acc) #turn_acceleration=turn_acc

        min_speed = 30
        max_speed = fabs(speed)
        distance_abs = fabs(drive_distance)  # use absolute distance as distance measure
 
        while True:
            # create speed ramp up and ramp down function based on distance travelled
            distance_travelled_abs = fabs(self.driver.distance() - distance_ini)

            drive_speed = self.ramp_function (distance_travelled_abs/distance_abs, min = min_speed, max = max_speed, ramp_up =ramp_up_factor, ramp_down = ramp_down_factor)

            current_angle = self.gyro.angle()
            error = current_angle - target_angle

            error_derivative = error - error_last
            error_last = error
            error_intergral = error_intergral + error

            steeringcorrection = error * Kp + error_intergral * Ki + error_derivative* Kd + feedforward
                        
            if distance_travelled_abs >= distance_abs:
                break
            else:
                self.driver.drive(speed_direction * drive_speed, steeringcorrection)

            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())

            if interval == 0:
                pass
            else:
                wait(interval)

        if stop is True:
            self.complete_stop()

    def gyro_ramp_turn(self, turn_angle, turn_rate, spinturn=True, interval=0, wait_time=200, adaptive_precision = True, ramp_up_factor = 0.1, ramp_down_factor = 0.3, data_logger=False, timer_reset= False ):
        """ Use gyro sensor for turning, ramps speed down
        Args:
            turn_angle (int) : angle in deg
            turn_rate (int) : turn rate of the robot (deg/s)
            spinturn(bool): flag for indicating whether to do a spin turn or pivot turn. Default: true
            data_logger (boolean) : default False, if True will turn on debugger mode to record the data of gyro
            timer (StopWatch object): default None, use global timer for debug purpose
            interval : time interval for loop, default 10, milliseconds
            ramp_up_factor (int) : angle factor of ramp up
            ramp_down_factor (int) : angle factor of ramp down
        """
        if self.RecordOn is True: data_logger=True

        # setup turn time protection incase of getting stuck
        max_time = max(fabs(turn_angle/turn_rate)*3000,2000)

        if timer_reset is True:
            self.timer.reset()

        starting_time=self.timer.time()

        # check turn direction(left/right) based on user input of speed (+/-) and angle (+/-)
        turn_direction, angle_change, direction_llmotor, direction_rlmotor = self.check_turns(turn_angle, turn_rate, spinturn)

        turn_speed_abs = fabs(turn_rate)
        angle_abs = fabs(turn_angle)

        angle_ini = self.gyro.angle()

        # set up min/max turn rates
        min_turn_rate = 50
        max_turn_rate = turn_speed_abs

        # adaptive precision based on input power, need tests to find best calibrations
        if adaptive_precision is True:
            precision = int(turn_speed_abs/30)
            if precision > 10: precision =10
        else:
            precision = 1.0;

        angle_target = angle_ini + angle_change
        gyro_angle = self.gyro.angle()

        while (gyro_angle - angle_target) * turn_direction <= -precision:

            # stall protection time
            if self.timer.time() >= max_time + starting_time:
                break

            # calculate ramp speed based on current position
            angle_turned_abs = fabs(self.gyro.angle() - angle_ini)
            turn_speed = self.ramp_function (angle_turned_abs/angle_abs, min = min_turn_rate, max = max_turn_rate, ramp_up =ramp_up_factor, ramp_down = ramp_down_factor)

            self.llmotor.run(direction_llmotor * turn_speed)
            self.rlmotor.run(direction_rlmotor * turn_speed)

            #if you want to write a data log
            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())

            if interval == 0:
                pass
            else:
                wait(interval)
            gyro_angle = self.gyro.angle()        

        # hold motor for stop        
        self.llmotor.hold()
        self.rlmotor.hold()

        # data-log during wait time
        current_time = self.timer.time()
        while self.timer.time()<=current_time+wait_time:
            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())

        self.llmotor.stop()
        self.rlmotor.stop()

        angle_final = self.gyro.angle()
        error = angle_final - angle_target

        if data_logger is True:
            self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                            self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())
        return error

    def gyro_ramp_turn_correction(self, turn_angle, turn_rate, spinturn=True, ramping=True, interval=10, ramp_up_factor = 0.2, ramp_down_factor = 0.5, debug_mode=False):
        """ correcting gyro sensor angle
        Args:
            turn_angle (int) : angle in mm
            turn_rate (int) : turn rate of the robot  (deg/s)
            spinturn(bool): flag for indicating whether to do a spin turn or pivot turn. Default: true
            ramping(bool): flag for indicating whether to ramp the power of the turn or not. Default: true
            interval : time interval for loop  (milliseconds) Default: 10
            ramp_up_factor(float): when to stop ramping the power. Default: 0.2
            ramp_down_factor(float): when to start ramping the power down. Default: 0.5
        """

        if self.RecordOn is True: data_logger=True

        turn_speed = fabs(turn_rate)

        # check turn direction(left/right) based on user input of speed (+/-) and angle (+/-)
        turn_direction, angle_change, direction_llmotor, direction_rlmotor = self.check_turns(turn_angle, turn_rate, spinturn)

        # coarse turn with high speed
        angle_ini = self.gyro.angle()
        angle_target = angle_ini + angle_change

        if ramping:
            error1 = self.gyro_ramp_turn(turn_angle=angle_change, turn_rate = turn_speed, spinturn=spinturn,interval=interval, ramp_up_factor=ramp_up_factor, ramp_down_factor=ramp_down_factor)
        else:
            error1 = self.gyro_turn(turn_angle=angle_change, turn_rate = turn_speed, spinturn=spinturn,interval=interval)
        
        angle_end1=self.gyro.angle()

        # fine correction turn with lower speed
        error2 = self.gyro_turn(turn_angle=-error1, turn_rate=30, spinturn=True,interval=interval)
        angle_end2=self.gyro.angle()

        error = angle_end2 - angle_target

        # in debug mode print out angle and error information
        if debug_mode is True:
            print ("starting angle:",angle_ini, "target angle: ", angle_target, "angle1:", angle_end1, "angle2:", angle_end2)
            print ( "error1:", error1, "error2:", error2, "error:", error)

        return error

    def driver_angle(self, target_angle, speed, distance, kp=2.0, data_logger = False, timer_reset= False, stop=False):
        """ Uses PID smooth drive to get the robot facing a certain angle
        Args:
            target_angle (int): the angle the robot will drive to
            speed (int): the speed of the robot
            distance (int): the distance the robot will travel
            kp (int): the correction factor coefficient
            data_logger (bool): whether data log is turned on
            timer_reset (bool): whether the timer resets at the beginning
            stop (bool): whether the robot complete stops after traveling the target distance
        """
        if self.RecordOn is True: data_logger=True

        if data_logger is True:
            self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                            self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())
                
        # reset timer
        if timer_reset is None: 
            self.timer.reset()
        
        initial_distance = self.driver.distance()

        while True:
            current_angle = self.gyro.angle()

            # calculate error and steering corrections
            error = current_angle - target_angle
            correction = kp * error

            self.driver.drive(speed, correction)

            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())
            if abs(self.driver.distance()-initial_distance)>=distance:
                break

        if stop is True:
            self.complete_stop(stop_option=3) # stop option set to 3 to avoid crazy driving at the end of program due to hold

    def drive_angle_pastwarning(self, speed, target_angle, max_distance, distance_after_warning, kp=2, reflection_white=91, reflection_black=15, stop=True, data_logger=True):
        """Detects the line at an angle then moves the robot further at that angle for a distance
        Args:
            speed (int): the speed of the robot
            target_angle (int): the angle the robot drives to the line and will continue to drive
            max_distance (int): the maximun distance the robot drives before breaking
            distance_after_warning (int): the distance the robot will drive after the line
            kp (int): the correction factor coefficient
            reflection_white (int): reflection threshold value for white color
            reflection_black (int): reflection threshold value for black color
            stop (bool): whether the robot will complete stop after the target distance
        
        """
        if self.RecordOn is True: data_logger=True

        initial_distance = self.driver.distance()
        max_distance_abs=fabs(max_distance)
        distance_after_warning_abs = fabs(distance_after_warning)

        # initialize line dection variables for right and left color sensors
        is_leftwarning_detected=False
        is_rightwarning_detected=False
        distance_travelled_at_warning_left=0
        distance_travelled_at_warning_right=0

        # create first reflection measurement for both color sensors
        left_reflection_last = self.lcolor.reflection()
        right_reflection_last = self.rcolor.reflection()
        
        reflection_mid = 0.5 *(reflection_white + reflection_black)
        while True:

            # using propotion control to follow target angle during driving
            current_angle = self.gyro.angle()
            error = current_angle - target_angle
            correction = kp * error

            self.driver.drive(speed, correction)

            # check distance travelled
            distance_travelled_abs = fabs(self.driver.distance() - initial_distance)

            # update color sensor reflection measurements
            left_reflection = self.lcolor.reflection()
            right_reflection = self.rcolor.reflection()

            print (left_reflection, right_reflection, distance_travelled_abs, max_distance_abs)

            # check left color sensor line detection and update distance_travelled_at_warning_left
            if left_reflection >= reflection_white:
                is_leftwarning_detected=True
                distance_travelled_at_warning_left=distance_travelled_abs

            # check right color sensor line detection and update distance_travelled_at_warning_right
            if right_reflection >= reflection_white:
                is_rightwarning_detected=True
                distance_travelled_at_warning_right=distance_travelled_abs

            # update max distance travelled at warning for both sensors
            distance_travelled_at_warning=max(distance_travelled_at_warning_left, distance_travelled_at_warning_right)

            # create data logging
            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())

            # update max_distance_abs once both sensors detect the line    
            if is_leftwarning_detected is True and is_rightwarning_detected is True: 
#                self.ev3.speaker.beep()
                max_distance_abs = distance_after_warning_abs + distance_travelled_at_warning

#            if max_distance_abs <=120: max_distance_abs =120

            # update last reflection values for both color sensors
            left_reflection_last=left_reflection
            right_reflection_last=right_reflection

            # distance protections if no above allowed max distance travelled.
            if distance_travelled_abs >= max_distance_abs:
                break
            
        if stop is True:
            self.complete_stop()

    def realign_correction(self, distance_after_detection, speed, max_distance=200, color_reflec_black=13, color_reflec_white=90, kp = 2.0, target_angle=None, stop=True, data_logger=False):
        """Align black then smoothly corrects to be vertical to the black line
        Args:
            distance_after (int): the distance the robot travels after aligning
            speed (int): the speed of the correction
            color_reflec_black (int): the threshold value of the reflection (black) the robot will detect (<)
            color_reflec_white (int): the threshold value of the reflection (white) the robot will detect (>)
            target_angle (int, deg): the value of target drive angle to follow (default None means keep current robot angle)
            kp (int): the correction factor coefficient
        """
        if self.RecordOn is True: data_logger=True

        # setting up of variables for the number of line detection by each color sensor and the related distance travelled at detection
        rb_breaking = 0
        lb_breaking = 0

        rb_distance = 0
        lb_distance = 0

        color_mid = 0.5 * (color_reflec_white + color_reflec_black)

        # setting up last reflect measurements for both color sensors
        rreflec_last = self.rcolor.reflection()
        lreflec_last = self.lcolor.reflection()

        # if target angle is not specified, keep current robot angle as driving angle
        if target_angle is None:
            target_angle = self.gyro.angle()

        starting_distance=self.driver.distance()
        print (starting_distance, max_distance)

        current_angle = self.gyro.angle()

        while True:

            # using propotion control to follow target angle during driving
            current_angle = self.gyro.angle()
            error = current_angle - target_angle
            correction = kp * error
            self.driver.drive(speed=speed, turn_rate=correction)

            # measure the color reflection for both left/right sensors
            rreflec = self.rcolor.reflection()
            lreflec = self.lcolor.reflection()

            print(rreflec, lreflec)
            # update detection distance for the first detection by right color sensor
            if rreflec <= color_reflec_black :
            #if rreflec <= color_mid and rreflec_last >= color_mid:
                if rb_breaking == 0:
                    self.ev3.speaker.beep()
                    rb_distance = self.driver.distance()
                    print("right detection distance:", rb_distance)
                rb_breaking = rb_breaking+1

            # update detection distance for the first detection by left color sensor
            if lreflec <= color_reflec_black :
            #if lreflec <= color_mid and lreflec_last >=color_mid:
                if lb_breaking == 0:
                    self.ev3.speaker.beep()
                    lb_distance = self.driver.distance()
                    print("Left detection distance:", lb_distance)
                lb_breaking = lb_breaking+1

            distance_travelled = abs(self.driver.distance()-starting_distance)
           # print(self.rcolor.reflection(),self.lcolor.reflection(), lb_breaking, rb_breaking, distance_travelled)   

            # update last reflection values for both left and right color sensors
            rreflec_last = rreflec
            lreflec_last = lreflec
            
            if data_logger is True:
                self.datalog.log(self.timer.time(), self.driver.distance(),self.gyro.angle(), self.lcolor.reflection(),self.rcolor.reflection(), \
                                self.llmotor.speed(), self.rlmotor.speed(), self.lmmotor.speed(), self.rmmotor.speed())

            # break the loop if both color sensors detect line
            if lb_breaking >= 1 and rb_breaking >= 1:
                break 
            
            # loop break if beyond max distance
            if distance_travelled >= max_distance:
                break

        # use geometry to calculate the inclined angle
        dcolor = rb_distance-lb_distance
        length_between_colorsensors = 110

        # calculate the inclined angles based on right triangles
        error_angle = dcolor/length_between_colorsensors * (180/3.1415)

        # correct the angle error when continuing drive to make sure vertical to the color line
        target_angle = current_angle + error_angle

        # correct the tilted drive angle during this drive (min power 150)
        if speed <=100:
            speed = 100
        self.driver_angle(target_angle= target_angle, speed=speed, distance = distance_after_detection, kp=kp, data_logger=data_logger)
        
        if stop is True:
            self.complete_stop() # stop option set to 3 to avoid crazy driving at the end of program due to hold

        print("dcolor:", dcolor, "error angle:", error_angle)

    def ramp_function (self, x, min = 30, max = 100, ramp_up =0.1, ramp_down = 0.1):
        """ Define Ramp functions between [0,1], where the output(power or speed) will ramp up linearly from min to max during ramp up
            then ramp down linearly from max to min during ramp down
        Args:
            x: percentage of distance, turn angle
            min: minimum power
            max: maximum power
            ramp_up: ramp up factor from beginning
            ramp_down: ramp down factor before ending
        """
        # the power is set to max after the ramp up and before the ramp down
        if x >=ramp_up and x <= 1-ramp_down:
            power=max
        # the power is ramped up during the ramp up part using the equation
        elif x>=0 and x<=ramp_up:
            power = (max-min)/(ramp_up)*x+min
        # the power is ramped down during the ramp down part uding the eqtuion
        elif x>=1-ramp_down and x<=1:
            power = (max-min)/(-ramp_down)*(x-(1-ramp_down))+max
        else:
        # setting the power to min before the drive/turn and after
            power = min
        
        # returns the power function 
        return power
