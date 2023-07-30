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
#from threading import Thread
from pybricks.experimental import run_parallel
from pybricks.parameters import Button, Color, Direction, Port, Stop

sys.path.append("../src")

from robot import Robot

#create a datalog test function for datalogging and robot training

def test_robot_turns (turn_option=1):
    """ Uses turn (gyro_turn-1, turn-2)
    to turn the robot, tests the accuracy of the different speeds & angles for the robot
    log the speed, angle , error and time for data analysis
    """

    robot = Robot("Dr.Turner")
    robot.debug_mode(option_num=3)
    robot.gyro_calib()
    datafile = DataLog("speed", "turn_ang", "angle_err", "time", name='expt-test-turns', timestamp=False, extension='csv', append=True)
    datafile.log("Turn Option:",turn_option)
    wait(200)

    speeds=[50, 100, 150, 200, 300, 400]
    turn_angles1=[15, 30, 60, 90, 120, 150, 180, -15, -30, -60, -90, -120, -150,-180]
    turn_angles2=[-15, -30, -60, -90, -120, -150,-180]
    turn_angles = turn_angles1

    timer1=StopWatch()

    for speed in speeds:
        for angle_deg in turn_angles:
            wait(200)
            timer1.reset()
            robot.gyro.reset_angle(0)
            time_ini=timer1.time()
            angle_ini = robot.gyro.angle()
            angle_target = angle_ini + angle_deg
            if turn_option == 1:
                robot.gyro_turn(turn_angle=angle_deg, turn_rate=speed)
            if turn_option == 2:
                robot.turn(turn_angle=angle_deg, turn_rate=speed, turn_acceleration=400)
            if turn_option == 3:
                robot.gyro_ramp_turn(turn_angle=angle_deg, turn_rate=speed)
            time_end=timer1.time()
            wait(200)
            angle_end=robot.gyro.angle()

            datafile.log(speed, angle_deg, angle_end-angle_target, time_end-time_ini)

            print(speed, angle_deg, angle_end-angle_target, time_end-time_ini)

def test_robot_drive_straight(drive_option=1):

    robot = Robot("robot")
    robot.debug_mode(option_num=3)
    robot.gyro_calib()
    wait(200)
    datafile = DataLog("speed", "distance_tgt", "gyro-err", "dist-err", "time", name='expt-test-drive', timestamp=False, extension='csv', append=True)
    datafile.log("Turn Option:",drive_option)

    speeds=[50, 100, 150, 200, 300, 400, 600, 800]
    distances=[100,-100, 200, -200, 500, -500]

    timer1=StopWatch()

    for speed in speeds:
        for distance in distances:
            wait(200)
            time_ini=timer1.time()
            distance0= robot.driver.distance()
            angle_ini = robot.gyro.angle()
            if drive_option ==1:
                robot.drive_straight(speed=speed, distance=distance, acc=200)
            elif drive_option ==2:
                robot.pid_drive(speed = speed, drive_distance=distance)
            elif drive_option == 3:
                robot.pid_rampdrive(speed=speed, drive_distance=distance)
            else:
                pass

            travel_time = timer1.time() - time_ini
            wait(400)
            distance_reported = robot.driver.distance() - distance0
            gyroerror = robot.gyro.angle() - angle_ini
            print(speed, distance,  gyroerror, distance_reported- distance, travel_time)
            datafile.log(speed, distance,  gyroerror, distance_reported- distance, travel_time)

def test_motorsDC_datalog ():

    # create robot object
    bella = Robot("bella")

    # create timers 
    timer1=StopWatch()
    timer2=StopWatch()

    # print time on ev3 screen and reset timer (you can also pause/resume it)
    bella.ev3.screen.print("current time:", timer2.time())
    timer2.reset()

    # run the left medium motor at given dc power (percentage) 

    Mymotor=bella.lmmotor

    dc_power_pct=100
    Mymotor.dc(dc_power_pct)

    # create data log file for data recording
    datalog_file = DataLog("time", "speed", name="test_motor",timestamp=False, extension="csv")
    # record testing dc power
    datalog_file.log("dc power: ", dc_power_pct)

    # recording data (time & speed) for 1000 msecs
    while timer1.time()<1000:
         datalog_file.log(timer1.time(), Mymotor.speed())

    # stop motor
    Mymotor.stop()

def test_drivebase_datalog ():

    robot1 = Robot("boxy")

    # set up robot accelerations for straight drive and turns
    robot1.driver.settings(straight_acceleration=700, turn_acceleration=600)

    # create timer
    timer1 =StopWatch()
    timer1.reset()
    
    drive_time = 1000
    drive_speed = 500
    turn_rate = 200

    while timer1.time() < drive_time:
        robot1.driver.drive(drive_speed, turn_rate)

    robot1.complete_stop()

def test_drivetime_datalog ():
    robot = Robot("robot")
    num = 2
    if num == 1:
        robot.drive_distance(speed=300,turn_rate=0,drive_distance=300)
    if num == 2:
        robot.drive_time(speed=300,turn_rate=0,drive_time=500)

def test_smoothdrive_solarfarm():
    robot = Robot("Mr.Smith")
    timer1=StopWatch()

    def robotwait():
        robot.complete_stop()
        robot.debug_mode(option_num=3)

    datalog_file=DataLog("time","distance","driver_angle", "gyro_angle", name="log-smoothdrive", timestamp=False, extension="csv", append=False)

    drive_speed=120
    timer1.reset()
    robot.driver.reset()
    robot.gyro.reset_angle(0)

    distance=robot.driver.distance()
    driver_angle=robot.driver.angle()
    gyro_angle=robot.gyro.angle()

    # First 1 sec
    while True:

        if robot.driver.distance() <= 40:
            turn_rate=0
            drive_speed = 200
        elif robot.driver.distance() <= 190:
            turn_rate=-55
            drive_speed = 90
        elif robot.driver.distance() <= 240:
            turn_rate=-60
        elif robot.driver.distance() <= 500:
            drive_speed = 90        
            turn_rate=-20
        else:
            break

        robot.driver.drive(drive_speed, turn_rate=turn_rate)
        datalog_file.log(timer1.time(), robot.driver.distance(),robot.driver.angle(), robot.gyro.angle())
        # insert your data log recording here

        if robot.driver.distance() >= 240:
            robot.rmmotor.run_until_stalled(speed=400, duty_limit=40)

    robotwait()

def test_os_mvfiles():

    power="20"

    file1="test1.csv"
    file2="test%s.csv" % power
    cmd='mv %s %s' %(file1, file2)
    os.system(cmd)

def test_driverecord():
    robot = Robot("Mr.Smith")
    timer1=StopWatch()
    robot.debug_mode(3)
    datalog_file=DataLog("time","distance","speed", "driver_angle", "gyro_angle", name="log-driverecord", timestamp=False, extension="csv", append=False)
    robot.ev3.speaker.beep()
    timer1.reset()
    robot.gyro.reset_angle(0)
    robot.driver.reset()

    time_last=timer1.time()
    dis_last=robot.driver.distance()

    while True:
        time=timer1.time()
        dis = robot.driver.distance()
        speed = (dis-dis_last)/(time-time_last + 0.001)

        driver_angle=robot.driver.angle()
        gyro_angle=robot.gyro.angle()

        datalog_file.log(time, dis, speed, driver_angle, gyro_angle)

        # update last time, distance
        time_last=time
        dis_last=dis

        if Button.LEFT in robot.ev3.buttons.pressed ():
            break
    
    robot.ev3.speaker.beep()




def test_turn_acc():
    robot = Robot("Mr.Turn")
    timer=StopWatch()
    datalog=DataLog("angle", 'rate', "acc", "turn_time", name="log-turnacc", timestamp=False, extension="csv", append=True)
    angle=90
    rate=400

    acc_list=[100,200,400,600, 800]

    for acc in acc_list:
        current_time=timer.time()
        robot.turn(turn_angle=angle, turn_rate=rate, turn_acceleration=acc)
        turn_time=timer.time()-current_time
        datalog.log(angle, rate, acc, turn_time)
        print(turn_time)

def test_motorcontrol ():
    robot = Robot("Mr.Turn")

    motor=robot.lmmotor
    motor_list=[robot.lmmotor, robot.rmmotor, robot.llmotor, robot.rlmotor]

    for motor in motor_list:
        scale=motor.control.scale
#        motor.control.limits(speed=100, acceleration=100, actuation=100)

        control_limit=motor.control.limits()
        control_pid=motor.control.pid()
        control_tgt_tol=motor.control.target_tolerances()
        control_stl_tol=motor.control.stall_tolerances()

        print(motor)
        print("motor control scale:", scale)
        print("motor control limits (speed, acc, atu):", control_limit)
        print("motor control pid (kp, ki, kd, int_rg, int_rt, ff):", control_pid)
        print("motor target tolerances (speed, position):", control_tgt_tol)
        print("motor stall tolerances (speed, time): " , control_stl_tol)

    driver_setting=robot.driver.settings()
    print("driver settings (staigt speed/acc, turn rate/acc):", driver_setting)
    print(robot.driver.heading_control.scale)
    print(robot.driver.heading_control.limits())
    print(robot.driver.heading_control.pid())

    print(robot.driver.distance_control.scale)
    print(robot.driver.distance_control.limits())
    print(robot.driver.distance_control.pid())

def test_driverecord2():
    robot = Robot()
    timer=StopWatch()
    datalog_file=DataLog("time+","speed4","speed5", "speed6", "speed10","angle4", "angle5","angle6", "angle10", name="log-curvefit", timestamp=False, extension="csv", append=False)

    robot.debug_mode(3)

    robot.ev3.speaker.beep()

    robot.driver.reset()
    robot.gyro.reset_angle(0)
    timer.reset()

    while True:
        x = timer.time()+1000
        x2 = x * x
        x3 = x2 * x
        x4 = x2 * x2
        x5 = x2 * x3

        speed10 = 2.53 + -0.0175*x + 3.65E-05*x2 + -3.61E-08*x3 + 2.02E-11*x4 + -6.95E-15*x5 + 1.52E-18*x5*x + -2.12E-22*x2*x5 + 1.82E-26*x3*x5 + -8.83E-31*x4*x5 + 1.84E-35*x5*x5
        angle10 = 361 + -1.45 *x + 2.42E-03*x2 + -2.2E-06*x3 + 1.2E-09*x4 + -4.11E-13*x5 + 9.04E-17*x5*x + -1.27E-20*x5*x2 + 1.11E-24*x5*x3 + -5.47E-29*x5*x4 + 1.16E-33*x5*x5

        speed6=-3.08 + 7.06E-03*x  -5.78E-06*x2 + 2.33E-09*x3  -4.94E-13*x2*x2 + 5.25E-17*x2*x3  -2.21E-21*x3*x3
        angle6=60.7  -0.0876*x + 2.05E-05*x2 + 1.64E-08*x3  -8.3E-12*x2*x2 + 1.3E-15*x2*x3  -6.81E-20*x3*x3

        speed5=0.12-2.87E-05*x + 1.16E-07*x2 -5.56E-11*x3 + 1.05E-14*x4 -6.9E-19*x5
        angle5=159 -0.306*x + 2.02E-04*x2 -5.72E-08*x3 + 7.23E-12*x4 -3.37E-16*x5

        speed4 = -0.19 + 5.33E-04*x -2.44E-07*x2 + 4.86E-11*x2*x -3.37E-15 * x2 *x2
        angle4 = 7.66 + -0.0312*x + 2.61E-05*x2 -6.26E-09*x2*x + 4.47E-13*x2*x2

        datalog_file.log(x,speed4, speed5, speed6, speed10, angle4, angle5, angle6, angle10)

        speed= speed4 * 1000
        turn_rate=-angle4

        if abs(speed) >=500:
            speed=0


        if timer.time()<=800:
            turn_rate=0


        robot.driver.drive(speed , turn_rate)
        if timer.time()>=6000:
            break
        wait(10)
    robot.complete_stop()

def test_drivecircle():
    robot=Robot("Test")
    robot.driver.reset()
    robot.gyro.reset_angle(0)
    datalog_file=DataLog("time","distance","driver_angle", "gyro_angle", name="log-driverecord", timestamp=False, extension="csv", append=False)
    timer=StopWatch()
    speed=200
    kp=2.0

    while True:
        target_angle = 0
        gyro_angle= robot.gyro.angle()
        steering_correction=kp * (gyro_angle - target_angle)
        robot.driver.drive(speed,steering_correction)
        datalog_file.log(timer.time(), robot.driver.distance(),robot.driver.angle(),robot.gyro.angle())
        if robot.driver.distance() >=400:
            break
    
    robot.ev3.speaker.beep()
    while True:
        target_angle = 45
        gyro_angle= robot.gyro.angle()
        steering_correction=kp * (gyro_angle - target_angle)
        robot.driver.drive(speed,steering_correction)
        datalog_file.log(timer.time(), robot.driver.distance(),robot.driver.angle(),robot.gyro.angle())

        if robot.driver.distance() >=800:
            break
    robot.ev3.speaker.beep()

    while True:
        target_angle = 90
        gyro_angle= robot.gyro.angle()
        steering_correction=kp * (gyro_angle - target_angle)
        robot.driver.drive(speed,steering_correction)
        datalog_file.log(timer.time(), robot.driver.distance(),robot.driver.angle(),robot.gyro.angle())

        if robot.driver.distance() >=1000:
            break
    
    robot.ev3.speaker.beep()

    while True:
        target_angle = 135
        gyro_angle= robot.gyro.angle()
        steering_correction=kp * (gyro_angle - target_angle)
        robot.driver.drive(speed,steering_correction)
        datalog_file.log(timer.time(), robot.driver.distance(),robot.driver.angle(),robot.gyro.angle())

        if robot.driver.distance() >=1200:
            break
    
    robot.ev3.speaker.beep()

    while True:
        target_angle = 210
        gyro_angle= robot.gyro.angle()
        steering_correction=kp * (gyro_angle - target_angle)
        robot.driver.drive(speed,steering_correction)
        datalog_file.log(timer.time(), robot.driver.distance(),robot.driver.angle(),robot.gyro.angle())

        if robot.driver.distance() >=1500:
            break
    
    robot.ev3.speaker.beep()

    while True:
        target_angle = 240
        gyro_angle= robot.gyro.angle()
        steering_correction=kp * (gyro_angle - target_angle)
        robot.driver.drive(speed,steering_correction)
        datalog_file.log(timer.time(), robot.driver.distance(),robot.driver.angle(),robot.gyro.angle())

        if robot.driver.distance() >=2300:
            break
    
    robot.ev3.speaker.beep()

    robot.complete_stop()


    """
        if timer.time()<=200:
          robot.driver.drive(150,0)
        elif timer.time()<=200+600:
          robot.driver.drive(300,0)
        elif timer.time()<=200+600+700:
          robot.driver.drive(250, -12)
        elif timer.time()<=200+600+700+1000:
          robot.driver.drive(300, -20)
        else:
            break
    robot.complete_stop()
    """
def test_drivecircle2():
    robot=Robot("Test")
    robot.driver.reset()
    robot.gyro.reset_angle(0)
    datalog_file=DataLog("time","distance","driver_angle", "gyro_angle", name="log-driverecord", timestamp=False, extension="csv", append=False)
    timer=StopWatch()
    speed=400
    kp=3.0

    while True:
        x = robot.driver.distance()
        target_angle = 8.78 + -0.0942 * x + 2.14E-04 *x*x + -2.29E-08*x*x*x+ -1.52E-11*x*x*x*x
        if x <=300:
            target_angle=0
        if target_angle > 240:
            target_angle = 240
        gyro_angle= robot.gyro.angle()
        steering_correction=kp * (gyro_angle - target_angle)
        robot.driver.drive(speed,steering_correction)
        datalog_file.log(timer.time(), robot.driver.distance(),robot.driver.angle(),robot.gyro.angle())
        if robot.driver.distance() >=2300:
            break
    robot.ev3.speaker.beep()

    robot.complete_stop()

def test_driveangle_PID ():

    robot=Robot("Test")
    robot.driver.reset()
    robot.gyro.reset_angle(0)

    speed=200
    kp=3.0
    target_angle =0.0

    while True:
        x =robot.driver.distance()
        target_angle = 1.44 + -0.0763 *x + 3.39E-04*x*x + 1.48E-05 *x*x*x + -5.84E-08*x*x*x*x + 6.07E-11*x*x*x*x*x
        if x >=200:
            target_angle = target_angle +10
        gyro_angle= robot.gyro.angle()
        steering_correction=kp * (gyro_angle - target_angle)
        robot.driver.drive(speed,steering_correction)
        if robot.driver.distance() >=600:
            break

    robot.ev3.speaker.beep()
    robot.complete_stop()

if __name__=="__main__":

    #Quick functionality tests

    test_option=8

    if test_option ==1:
       test_motorsDC_datalog()
    elif test_option ==2:
         test_smoothdrive_solarfarm()
    elif test_option ==3:
        test_driveangle_PID ()
    elif test_option ==4:
        test_drivebase_datalog()
    elif test_option ==5:
        test_robot_drive_straight (drive_option=2)
    elif test_option ==6:
        test_robot_drive_straight ()
    elif test_option ==7: 
        test_drivecircle2 ()
    else:
        test_driverecord()  