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

#!/usr/bin/env pybricks-micropython
import sys, os
from pybricks.tools import wait, StopWatch, DataLog
from utils import basic_test
#from threading import Thread
from pybricks.experimental import run_parallel
from pybricks.parameters import Button, Color, Direction, Port, Stop

sys.path.append("../src")

from robot import Robot

#split from orginal test_navigation
#only contains basic navigation

def basic_tests():
#    test_gyro_turn()
    test_gyro_turncorrection()
    test_wall_align()
    test_linesquaring_black()
    test_pid_drive()
    test_linesquaring_white()

@basic_test(text="PLACE WITH BACK FACING WALL")
def test_wall_align():
    """ Test the wall align function 
    Expectation: The robot should be perfectly
    straight against the wall.
    """

    robot = Robot("Robot")
    robot.wall_align(motor_speed = -800, align_time = 5000)

@basic_test(text="PLACE COLOR SENSORS BEHIND BLACK LINE AT ANGLE")
def test_linesquaring_black():
    """ Test the linesquaring function 
    (align vertically on white/black lines) 
    Expectation: The robot should be perfectly
    straight on the line with both of its color
    sensors parked on the same line.
    """

    robot = Robot("Robot")
    robot.linesquaring(align_color = "black", align_speed = 200, debug = False)

@basic_test(text="PLACE COLOR SENSORS AT 30° ANGLE BEHIND BLACK LINE")
def test_linesquaring_black2 ():
    """ Test the linesquaring function 
    (align vertically on white/black lines) 
    Expectation: The robot should be perfectly
    straight on the line with both of its color
    sensors parked on the same line.
    """

    robot = Robot("robot")
    robot.linesquaring(align_color = "black", align_speed = 200)

@basic_test(text="PLACE COLOR SENSORS BEHIND WHITE LINE AT ANGLE")
def test_linesquaring_white():
    """ Test the linesquaring function 
    (align vertically on white/black lines) 
    Expectation: The robot should be perfectly
    straight on the line with both of its color
    sensors parked on the same line.
    """

    robot = Robot("Robot")
    robot.linesquaring(align_color = "white", align_speed = 200, debug = False)

@basic_test(text="PLACE ROBOT IN LEFT LAUNCH AREA, FACING WIND TURBINE")
def test_pid_drive():
    """ Test the PID straight drive function
    Expectation: The robot should drive straight
    and arrive at its destination with no heading
    differences from when it started.
    """

    robot = Robot("Robot")
    robot.pid_drive(speed = 700, drive_distance = 500)

@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH TURN AREA")
def test_gyro_turn():
    """ Uses the gyro sensor to turn the robot
    tests the accuracy of the different speeds for the robot
    we are expecting to see that the lower the power, the more accurate the robot is
    """
    robot = Robot("Dr.Turner")
    robot.gyro_calib()
    wait(200)
    robot.gyro.reset_angle(0)
    turn_angles=[15, 30, 60, 90, 120, 150, 180]
    speeds=[50, 100, 200, 400, 800]
    for angle in turn_angles:
        for speed in speeds:
            theta1=robot.gyro.angle()
            error = robot.gyro_turn(turn_angle=-angle, turn_rate=speed, spinturn=True)
            wait(200)
            theta2=robot.gyro.angle()
            print ("angle:", angle, "speed:", speed, "error:", error, theta1, theta2, "change:", theta2-theta1)

@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH TURN AREA")
def test_gyro_turncorrection():
    """ Uses the gyro sensor to turn the robot
    tests the accuracy of the different speeds for the robot
    we are expecting to see that the lower the power, the more accurate the robot is
    """
    robot = Robot("Dr.Turner")
    robot.gyro_calib()
    wait(400)
    turn_angles=[15, 30, 60,90, 120, 150,180]
    speeds=[50,100,200,400,800]
    for angle_deg in turn_angles:
        for speed in speeds:
            error = robot.gyroturn_correction(turn_angle=angle_deg, turn_rate=speed, spinturn=True)
            print (angle_deg, speed, error)
            wait(400)

@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH TURN AREA")
def test_gyro_turn3():
    """Continuous turning, when the brick button is pressed, the robot does another turn
    sequence: spin-left, spin-right, pivot-left, pivot-right
    """
    robot = Robot("Dr.Continuous_Turner")
    for i in range(10):
        robot.gyro_turn(turn_angle=90, turn_rate=100, spinturn=False)
        robot.debug_mode(3)
        robot.gyro_turn(turn_angle=-90, turn_rate=100, spinturn=False)
        robot.debug_mode(3)
        robot.gyro_turn(turn_angle=90, turn_rate=100, spinturn=True)
        robot.debug_mode(3)
        robot.gyro_turn(turn_angle=-90, turn_rate=100, spinturn=True)

def test_basic_navigation():
    """
    Go forward, turn, go forward, touch the energy storage, go back, turn, color align, go forward and touch the oil rig
    """
    robot = Robot("Dr.Basic_Navigation")
    robot.debug_mode(3)
    robot.pid_drive(200, 240) #PID DRIVE HAS ISSUES!
    robot.debug_mode(3)
    robot.gyro_turn(turn_angle =-20, turn_rate =100, spinturn=True) #the program can't handle negative speed
    robot.debug_mode(3)
    robot.pid_drive(200, 400)
    robot.debug_mode(3)
    robot.gyro_turn(turn_angle = 30, turn_rate = 100, spinturn=True)
    robot.debug_mode(3)
    robot.pid_drive(200, 190)
    robot.debug_mode(3)
    robot.pid_drive(200, -200)
    robot.debug_mode(3)
    robot.gyro_turn(turn_angle = 90, turn_rate = 100, spinturn=True)
    robot.debug_mode(3)
    robot.pid_drive(200, -100)
    robot.debug_mode(3)
    robot.gyro_turn(turn_angle = -90, turn_rate = 100, spinturn=True)
    robot.debug_mode(3)
    robot.pid_drive(200, 150)
    robot.debug_mode(3)
    robot.gyro_turn(turn_angle = 90, turn_rate = 100, spinturn=True)
    robot.debug_mode(3)
    robot.color_align_reflection(align_color="black", align_speed=50)
    robot.debug_mode(3)
    robot.pid_drive(200, 80)

@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH SPACE TO MOVE STRAIGHT")
def test_drive_straight():
    """
    Testing the basic move straight without PID
    """
    robot = Robot("Dr.NoPID")
    robot.drive_straight(speed=190,distance=100)
    robot.drive_straight(speed=190,distance=-100)

@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH SPACE TO TURN")
def test_gyro_turn_debugger():
    """
    Testing the gyro turn debug mode
    """
    robot = Robot("Robot")
    robot.gyro_turn(turn_angle =-90, turn_rate =50, spinturn=True, data_logger = True)

@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH SPACE TO LINE FOLLOWING")
def test_line_following():
    """
    Testing the line following function
    """
    robot = Robot("Robot")
    robot.debug_mode(3)
    robot.followline(speed=200, time = 1000)

@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH SPACE TO MOVE STRAIGHT")
def test_checksensor(interval=2, option=1):
    """
    Testing the check sensor reading function
    """
    robot = Robot("Robot")
    if option == 1:
        robot.check_sensor_reading(waitinterval=interval)
    if option == 2:
        robot.gyro_calib()
        robot.check_sensor_reading(waitinterval=interval)

@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH SPACE TO MOVE")
def test_ramp_drive():
    robot = Robot ("nothing")
    robot.drive_ramp(speed= 400, distance=200, acc=700, ramp_up_factor=0.1, ramp_down_factor=0.2)

@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH SPACE TO MOVE")
def test_robot_turn_corrections(turn_option=1):
    """ Uses turn corrections (gyroturn_correction-1, turn_correction-2, hybrid_turn_correction-3)
    to turn the robot, tests the accuracy of the different speeds & angles for the robot
    log the speed, angle , error and time for data analysis
    """

    robot = Robot("Dr.Turner")
    robot.gyro_calib()
    wait(200)

    datafile = DataLog("speed", "turn_ang", "angle_err", "time", name='expt-test-turncorrs', timestamp=False, extension='csv', append=True)
    datafile.log("Turn Option:",turn_option)

    speeds=[50, 100, 150, 200, 300, 400]
    turn_angles=[15, 30, 60, 90, 120, 150, 180, -15, -30, -60, -90, -120, -150, -180]
    timer1=StopWatch()

    for speed in speeds:
        for angle_deg in turn_angles:
            wait(200)
            timer1.reset()
            angle_ini = robot.gyro.angle()
            angle_target = angle_ini + angle_deg

            if turn_option == 1: # gyro turn correction
                robot.gyroturn_correction(turn_rate = speed, turn_angle=angle_deg)
            if turn_option == 2: # standard turn correction
                robot.turn_correction(turn_rate = speed, turn_angle=angle_deg)
            if turn_option == 3: # hybrid turn correction
                robot.hybrid_turn_correction(turn_rate = speed, turn_angle=angle_deg)
            if turn_option == 4:
                robot.gyro_ramp_turn_correction(turn_angle=angle_deg, turn_rate=speed)

            turn_time =timer1.time()
            wait(200)
            angle_end=robot.gyro.angle()
            datafile.log(speed, angle_deg, angle_end-angle_target, turn_time)
            print(speed, angle_deg, angle_end-angle_target, turn_time)

#@basic_test(text="PLACE ROBOT IN OPEN SPACE WITH ENOUGH SPACE TO MOVE")