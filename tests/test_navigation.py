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
from threading import Thread
from pybricks.experimental import run_parallel
from pybricks.parameters import Button, Color, Direction, Port, Stop

sys.path.append("../src")

from robot import Robot

#test function for complex navigation tests

def test_robot_turns (turn_option=1):
    """ Uses turn (gyro_turn-1, turn-2)
    to turn the robot, tests the accuracy of the different speeds & angles for the robot
    log the speed, angle , error and time for data analysis
    """

    robot = Robot("Dr.Turner")
    robot.debug_mode(option_num=2)
    robot.gyro_calib()

    datafile = DataLog("speed", "turn_ang", "angle_err", "time", name='expt-test-turns', timestamp=False, extension='csv', append=True)
    datafile.log("Turn Option:",turn_option)
    wait(200)

    speeds2=[50, 100, 150, 200, 300, 400]
    speeds=[200]
    turn_angles1=[15, 30, 60, 90, 120, 150, 180, -15, -30, -60, -90, -120, -150,-180]
    turn_angles2=[-15, -30, -60, -90, -120, -150,-180]
    turn_angles =[-90]

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
                robot.datalog.log("#speed", speed, "angle_tgt", angle_target )
                robot.gyro_turn(turn_angle=angle_deg, turn_rate=speed, data_logger=True, timer_reset= True)

            if turn_option == 2:
                robot.turn(turn_angle=angle_deg, turn_rate=speed, turn_acceleration=400)
            if turn_option == 3:
                robot.gyro_ramp_turn(turn_angle=angle_deg, turn_rate=speed)
            time_end=timer1.time()
            wait(200)
            angle_end=robot.gyro.angle()

            datafile.log(speed, angle_deg, angle_end-angle_target, time_end-time_ini)

            print(speed, angle_deg, angle_end-angle_target, time_end-time_ini)

def test_robot_drive_straight (drive_option):

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

def test_pid_straight_acc(acc=500):
    
    robot = Robot("robot")
    robot.gyro_calib()
    robot.pid_drive(speed=400,straight_acc=acc,drive_distance=300)

def test_pid_rampdrive():
    robot = Robot("robot")
    robot.pid_rampdrive(speed=-200,drive_distance=500,ramp_up_factor=0.1, ramp_down_factor=0.1)
    robot.debug_mode(option_num=2)
    robot.pid_rampdrive(speed=-500,drive_distance=-500, ramp_up_factor=0.1, ramp_down_factor=0.1)

def test_drive_straight_pastwarning():
    robot = Robot("robot")
    robot.drive_straight_pastwarning(speed=200, max_distance=500, distance_after_warning=50)

def test_feedforward():
    """
    positive number for feedforward is going right and negative is going left
    """
    robot = Robot("robot")
    robot.debug_mode(option_num=3)
    robot.debug_mode(option_num=1)
    robot.pid_rampdrive(speed=200,drive_distance=620,kp=2, feedforward=-0.9)

def test_run_parallel():

    """ Using pybrick experimental feature to runs functions in parallel and waits for all to complete.
    make sure use 'from pybricks.experimental import run_parallel'
    """
    robot = Robot("robot")
    robot.debug_mode(option_num=3)

    def run_motor():
        robot.rmmotor.run_until_stalled(speed=200, duty_limit=80)
        robot.rmmotor.stop()

    def run_motor2():
        robot.lmmotor.run_until_stalled(speed=200, duty_limit=80)
        robot.lmmotor.stop()

    def run_motor2time():
        robot.lmmotor.run_time(speed=-600, time=700)
        robot.lmmotor.stop()

    def rightarm_detach_waterunits ():
        # swing the right arm slowly outside water reservior to detach the two WUs
        #robot.rmmotor.run_angle (speed=20, rotation_angle=60)

        robot.rmmotor.run_until_stalled(speed=100, duty_limit=80)
        robot.rmmotor.stop()
    
    def robot_pullback ():
        # drive back to make sure water units latched on the bar
        robot.pid_rampdrive(speed=-150, drive_distance=200)
    
    run_parallel (rightarm_detach_waterunits, robot_pullback)

def test_piston_rackpinion():
    robot = Robot("robot")
    robot.debug_mode(option_num=3)
    # push energy unit attachment to the targets then release back
    robot.rmmotor.run_time(speed=-800,time=3100)
    robot.rmmotor.brake()

    robot.rmmotor.run_time(speed=800,time=3100)
    robot.rmmotor.brake()

#    run_parallel(run_motor, run_motor2time)

def test_drive_distance(speed = 200, distance=300):
    robot = Robot("robot")
    robot.debug_mode(option_num=3)
    robot.pid_rampdrive(speed=speed,drive_distance=distance)

    distance_reported = robot.driver.distance()
    print(speed, distance, distance_reported)

def test_open_grabber(opt=1):
    robot = Robot("robot")
    robot.rmmotor.run_until_stalled(speed=400, duty_limit=40) # Done
    robot.debug_mode(option_num=opt)
    robot.rmmotor.run_angle(speed=-200,rotation_angle=200) # Done

def test_gyro_lag():
    robot = Robot("robot")
    robot.debug_mode(3)
    wait(500)
    robot.gyro_turn(turn_angle=90, turn_rate=200, data_logger=True, interval=5, wait_time=200)

def test_motors_datalog ():

    # create robot object
    robot = Robot("robot")

    # create timers 
    timer1=StopWatch()
    timer2=StopWatch()

    # print time on ev3 screen and reset timer (you can also pause/resume it)
    robot.ev3.screen.print("current time:", timer2.time())
    timer2.reset()

    # run the left medium motor at given dc power (percentage) 
    my_motor=robot.rlmotor
    dc_power_pct=20
    my_motor.dc(dc_power_pct)

    # create data log file for data recording
    datalog_file = DataLog("time", "speed", name="test_motor",timestamp=False, extension="csv")
    # record testing dc power
    datalog_file.log("dc power: ", dc_power_pct)
    datalog_file2= DataLog("dc_power", "speed", name="test_DC_speed",timestamp=False, extension="csv")

    # recording data (time & speed) for 1000 msecs
   
    DC_list = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]
    for DCpower in DC_list:
        sum=0
        number_point=0
        my_motor.dc(DCpower)
        timer1.reset()
        while timer1.time()<5000:
            if timer1.time()<=500:
                sum=sum+my_motor.speed()
                number_point=number_point+1
                datalog_file.log(timer1.time(), DCpower, my_motor.speed())
        Average = sum/number_point
        robot.ev3.screen.print(Average)
        print(DCpower, Average)
        datalog_file2.log(DCpower, Average)
        wait(500)

    # stop motor
    my_motor.stop()

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

def test_datalog_drivetime ():
    robot = Robot("robot")
    num = 2
    if num == 1:
        robot.drive_distance(speed=300,turn_rate=0,drive_distance=300)
    if num == 2:
        robot.drive_time(speed=300,turn_rate=0,drive_time=500)

def test_smooth_drive():
    robot = Robot("Mr.Smith")
    timer1=StopWatch()
    datalog_file=DataLog("time","distance","driver_angle", "gyro_angle", name="log-smoothdrive", timestamp=False, extensition="csv", append=False)

    drive_speed=200

    distance=robot.driver.distance()
    driver_angle=robot.driver.angle()
    gyro_angle=robot.gyro.angle()

    timer1.reset()

    # First 1 sec
    while True:
        robot.driver.drive(drive_speed, turn_rate=0)
        # insert your data log recording here

        if timer1.time() >= 1000:
            break
    
    # 2nd 1 sec
    while True:
        robot.driver.drive(drive_speed, turn_rate=30)
        
        # insert your data log recording here


        if timer1.time() >=2000:
            break
    
    # 3rd 1 sec
    while True:

        robot.driver.drive(drive_speed, turn_rate=-60)

        # insert your data log recording here

        if timer1.time() >=3000:
            break

    robot.complete_stop()

def test_os_mvfiles():

    power="20"

    file1="test1.csv"
    file2="test%s.csv" % power

    cmd='mv %s %s' %(file1, file2)

    os.system(cmd)
    
def test_L1_smooth_drive():
    robot = Robot("Ocean")   
    robot.debug_mode(3)
    timer=StopWatch()
    robot.gyro_calib() 
    
    # no cart, dino on top of robot,  using gyro-turn and pid-drive
    # move forward to powerplant and use trigger to release energy unit
    robot.pid_rampdrive(speed=350, drive_distance=797, ramp_down_factor=0.1, ramp_up_factor=0.05)
    wait(100)
    robot.debug_mode(option_num=1)

    # sligntly move back to dis-engage the trigger attachment
    robot.gyro_ramp_turn(turn_angle=2, turn_rate=50)
    robot.pid_rampdrive(speed=-200, drive_distance=80)
    robot.debug_mode(option_num=2)
    #random space for the pleasure of Bella's eyes on next line
    #mia completely supports this decision ^_^
    
    #drive towards water resovior
    def drive_water_resovior():
        robot.driver_angle(target_angle=-58, speed=200, distance=630, kp=3)

    #open arm to capture water units
    def open_arm():
        robot.rmmotor.run_until_stalled(speed=400, duty_limit=50) # Done
        wait(100)
        robot.rmmotor.run_angle(speed=-200,rotation_angle=200) # Done

     # run water resovior and capture units in parallel
    run_parallel (drive_water_resovior, open_arm)
    
    #move towards solar farm and captures units next to smart grid
    #robot.driver_angle(target_angle=-90, speed=100, distance=150)
    robot.drive_angle_pastwarning(speed=100, target_angle=-90, max_distance=150, distance_after_warning=38, stop=False)

    #turn towards other two energy units on left
    robot.driver_angle(target_angle=-45, speed=100, distance=100)

    #robot navigating to last two units
    def navigate_2_units():
        robot.driver_angle(target_angle=-15, speed=100, distance=160)

    #robot closing arm to capture two units
    def close_arm():
        wait(1000)
        robot.rmmotor.run_until_stalled(speed=400, duty_limit=50) # Done
    
    #navigate to and capture 2 energy units
    run_parallel (navigate_2_units, close_arm)

    robot.driver_angle(target_angle=0, speed=-100, distance=20)

    #go hommeeee
    robot.driver_angle(target_angle=70, speed=500, distance=940)
      
    robot.complete_stop()                        #The robot completely stops
    print(timer.time())

def test_toyfactory_navigation():
    robot = Robot("robot")
    robot.gyro.reset_angle(0)
    robot.driver.reset()
    distance_1=robot.driver.distance()
    distance=2450
    kp = 3
    while True:
        current_angle = robot.gyro.angle()
        x = robot.driver.distance()
        target_angle=0.105 + -0.0188*x + 1.39E-04*x*x + -3.81E-07*x*x*x + 2.16E-10*x*x*x*x + -3.69E-14*x*x*x*x*x
        error = current_angle - target_angle
        correction = error*kp
        robot.driver.drive(100,correction)
        if abs(robot.driver.distance()-distance_1)>=distance:
            break
    robot.complete_stop()

def test_drive_angle_pastwarning():
    robot=Robot("minion")
    robot.drive_angle_pastwarning(speed=100, target_angle=-90, max_distance=150, distance_after_warning=50)

def test_drive_dam():
    robot = Robot("robot")
    # START 3M INSIDE FROM LEFT HOME

    def curve_hydro():
        robot.driver_angle(target_angle=-62, speed=150, distance=210)
        robot.complete_stop()
    
    def close_rightarm():
            # close the right arm
            robot.rmmotor.run_until_stalled(speed=-80, duty_limit=None)
            #robot.rmmotor.stop()
    
    run_parallel(curve_hydro, close_rightarm)
    # move forward with jig to align on model
    # capture water unit 3 and swing it to right side to avoid collision (slow swing)
    robot.rmmotor.run_until_stalled(speed=60, duty_limit=80)
    robot.pid_rampdrive(speed=150, drive_distance=250)
    robot.debug_mode(option_num=2)
    #print(robot.gyro.angle())
    def leftarm_control():
        # raise left arm to release water unit, this part can be threaded
        robot.lmmotor.run_until_stalled(speed=300, duty_limit=60)
        wait(100)
        robot.lmmotor.run_until_stalled(speed=-300, duty_limit=60)
        wait(100)
        robot.lmmotor.stop()

    def rightarm_control():
        # turn right arm to place water units in position
            robot.gyro_ramp_turn(turn_angle=3, turn_rate=50)
            robot.rmmotor.run_until_stalled(speed=-150, duty_limit=80)
            robot.debug_mode(option_num=1)
            
            #realign robot according to jig
            robot.gyro_turn(turn_angle=2,turn_rate=50)
            robot.rmmotor.stop()

    # run the two actions in parallel
    run_parallel(leftarm_control, rightarm_control)
    #print(robot.gyro.angle())

    # drive back towards home to de-latch the water units
    robot.pid_rampdrive(speed=-280, drive_distance=310)
                      
def test_launch3 (option = 1):
    """ Navigates to the hydroelectric dam mission and hang two water units on the bars, leave one in water reservoir
    use liftarm to lift the bar and release/capture energy units
    Starting point: Left Launch area, 2M from Left boundary, align on the wall
    Note the robot may keep turning which suspected to be gyro-calib issue (need gyro calibration blocks)
    """
    robot=Robot("robot")
    robot.debug_mode(option_num=3)
    robot.gyro_calib()
    timer=robot.timer

    if option == 1: # clear water unit 3 in case it's not captured in L1

        def prepare_align():
            # drive slightly forward
            robot.pid_rampdrive(speed=100, drive_distance=40)

            # turn to facing hydro-dam
            robot.gyroturn_correction(turn_rate=120, turn_angle =-50)
            robot.debug_mode(option_num=1)

        def close_rightarm():
            # close the right arm
            robot.rmmotor.run_until_stalled(speed=-80, duty_limit=None)
            #robot.rmmotor.stop()
        def move_close_dam():
            # move close to hydro-dam before the water unit to allow folk capture
            robot.pid_rampdrive(speed=150,drive_distance=165)
        
        # right arm close is after the turn
        prepare_align ()
        run_parallel(move_close_dam, close_rightarm)

        # capture water unit 3 and swing it to right side to avoid collision (slow swing)
        robot.rmmotor.run_until_stalled(speed=60, duty_limit=80)

        # move forward with jig to align on model
        robot.pid_rampdrive(speed=150, drive_distance=250)
        robot.debug_mode(option_num=2)
        #print(robot.gyro.angle())

        def leftarm_control():
            # raise left arm to release water unit, this part can be threaded
            robot.lmmotor.run_until_stalled(speed=300, duty_limit=60)
            wait(100)
            robot.lmmotor.run_until_stalled(speed=-300, duty_limit=60)
            wait(100)
            robot.lmmotor.stop()

        def rightarm_control():
            # turn right arm to place water units in position
            robot.rmmotor.run_until_stalled(speed=-150, duty_limit=80)
            robot.debug_mode(option_num=1)
        
            # realign robot according to jig
            robot.gyro_turn(turn_angle=2.5,turn_rate=50)
            robot.rmmotor.stop()

        # run the two actions in parallel
        run_parallel(leftarm_control, rightarm_control)
        #print(robot.gyro.angle())
        timer.reset()

        # drive back towards home to de-latch the water units
        robot.driver_angle(target_angle=-55, speed=-150, distance=130)

        robot.ev3.speaker.beep()
    
        # swing the right arm slowly outside water reservior to detach the two WUs
        # robot.rmmotor.run_angle (speed=20, rotation_angle=80, then=Stop.HOLD, wait=False)
        # drive back to make sure water units latched on the bar
        # robot.pid_rampdrive(speed=-150, drive_distance=50)
        robot.debug_mode(option_num=2)

        def return_lefthome ():
        # move back to make sure completely inside launch area
            robot.driver_angle(target_angle=-90, speed=-350, distance=340, kp=3.0)
            robot.complete_stop()

        # close right arm, this part can be parallel with drive straight 
        def close_right_arm():
            robot.rmmotor.run_until_stalled(speed=-150, duty_limit=80)

        run_parallel(return_lefthome, close_right_arm)
           #print(robot.gyro.angle())

        # wait sufficient long to put energy units/innovation project into front part to be delievered into Power to X
        #having jig which helps reset robot alignment
        robot.debug_mode(3)
        robot.gyro.reset_angle(-90)

        # drive toward the Power-X circle
        robot.driver_angle(target_angle=-90, speed=300, distance=130)
        robot.driver_angle(target_angle=-38, speed = 450, distance=640)
        robot.driver_angle(target_angle=-40, speed=-200, distance=160)

        # turn towards right home and return
        def go_right_home():
            robot.driver_angle(target_angle=-100, speed=800, distance=200)
            robot.driver_angle(target_angle=-115, speed=800, distance=400)
            robot.driver_angle(target_angle=-120, speed=800, distance=500)
        def close_rightarm():
            # close the right arm
            robot.rmmotor.run_until_stalled(speed=-80, duty_limit=None)
            #robot.rmmotor.stop()
        run_parallel(close_rightarm,go_right_home)

        robot.lmmotor.stop()
        robot.rmmotor.stop()
        robot.complete_stop(stop_option=3)

    else:
        pass

def gyro_beeping():
    robot = Robot("someone")
    rbreaking = 0
    lbreaking = 0

    while True:
        rreflec = robot.rcolor.reflection()
        lreflec = robot.lcolor.reflection()

        print(robot.rcolor.reflection(),robot.lcolor.reflection())

        robot.driver.drive(speed=100, turn_rate=0)

        if rreflec < 10:
            if rbreaking == 0:
                robot.ev3.speaker.beep()
                rdistance = robot.driver.distance()
            rbreaking = rbreaking+1
            
        if lreflec < 10:
            if lbreaking == 0:
                robot.ev3.speaker.beep()
                ldistance = robot.driver.distance()
            lbreaking = lbreaking+1

        print(lbreaking,rbreaking)   

        if robot.driver.distance() >= 500:
            break
        if lbreaking >= 1 and rbreaking >= 1:
            break 
    dcolor = rdistance-ldistance
    length = 110
    error_angle = dcolor/length * (180/3.1415)
    print(error_angle)
    robot.gyro.reset_angle
    currenangle = robot.gyro.angle()
    target_angle = currenangle+error_angle
    robot.driver_angle(target_angle= target_angle, speed=50, distance = 200, kp=2.0)
    # robot.gyro_turn(turn_angle=error_angle, turn_rate=50)
    robot.complete_stop()

def test_l1():
    robot = Robot("something")
    timer = timer.time()
    while True:
        x = timer
        target_agl = -66.2 + 0.0584*x + 5.84E-05*x*x + -3.88E-08*x*x*x + 5.85E-12*x*x*x*x
        robot.driver.drive(turn_rate=target_agl,speed=100)

def test_l4_smooth():
    robot = Robot('hi')
    def realign_robot():
        robot.realign_correction(distance_after=100,speed=100,color_reflec=15)
        # reset angle to east (90)
        robot.gyro.reset_angle(90)

        # move forward to prepare the missions
        # robot.pid_rampdrive(speed=150,drive_distance=142)

    def prepare_rightarm():
        # return right liftarm to poision for hybrid car
        robot.rmmotor.run_until_stalled(speed = 400, duty_limit=30)
        robot.rmmotor.stop()

    # run two tasks in parallel
    run_parallel(realign_robot, prepare_rightarm)
    robot.debug_mode(option_num=2)

    # turn to best position for both hybrid car and smart grid, best angle for boxy
    # red is 30, for silver and blue is 31 (drive distance need adjust to > 80)
    robot.gyro_ramp_turn_correction(turn_angle=32 -robot.gyro.angle(),turn_rate=100)
    robot.debug_mode(option_num=1)

    # drive forward to engage both the right lift arm and left pull trigger 
    robot.pid_rampdrive(speed=100,drive_distance=85)
    robot.debug_mode(option_num=1)

    #quickly raise right liftarm to complete hybrid car
    robot.rmmotor.run_until_stalled(speed = -800, duty_limit=None)
    robot.debug_mode(option_num=1)
    robot.rmmotor.stop()


    # slightly turn to use trigger complete smart grid
    robot.gyro_ramp_turn(turn_angle= 50 -robot.gyro.angle(),turn_rate=200)
    wait(200)
    # return to original position
    robot.gyro_ramp_turn(turn_angle= 30 -robot.gyro.angle(),turn_rate=100)

    # move back and prepare home left
    robot.pid_rampdrive(speed=-150,drive_distance=125)
    robot.debug_mode(2)
    robot.gyro_ramp_turn_correction(turn_angle= 90 -robot.gyro.angle(),turn_rate=200)

    #go straight home on left side
    robot.pid_rampdrive(speed=800,drive_distance=710)
    wait(200)

    # turn to left home and drive back (best angle for red is 145, blue and silver is 140)
    robot.gyro_ramp_turn_correction(turn_angle= 145 -robot.gyro.angle(),turn_rate=300)
    robot.pid_drive(speed=800,drive_distance=700)

    robot.complete_stop(stop_option=3)

def ramp_test():
    robot = Robot('Ramper')
    robot.debug_mode(option_num=3)
    power_push=300
    robot.pid_rampdrive(speed=200,drive_distance=115, ramp_down_factor=0.2)
#    robot.drive_straight(speed=250,distance=120)
    # pushing power can not be too high (prefer 350~450)
    for i in range(2):
        wait(250)
        robot.drive_straight(speed=power_push,distance=-60)
        wait(450)
        robot.drive_straight(speed=power_push,distance=68)
    wait(200)
    robot.drive_straight(speed=power_push,distance=-60)

def l4_navigation():
    robot = Robot('bella')
    robot.debug_mode(option_num=3)
#    robot.realign_correction(distance_after_dection=142, speed=100, max_distance=250)
    robot.rmmotor.run_angle(speed=-1200,rotation_angle=2300)
    robot.rmmotor.run_angle(speed=1200,rotation_angle=2300)


def test_windturbine():
    robot = Robot('123456789101112131415161718192021222324252627282930')
    robot.drive_straight(speed=700,distance=100)
    for i in range(2):
        wait(200)
        robot.drive_straight(speed=700,distance=-60)
        wait(200)
        robot.drive_straight(speed=700,distance=70)
    wait(500)
    
if __name__=="__main__":

    #Quick functionality tests

    test_option=11

    if test_option ==1:
       test_pid_straight_acc(acc=700)
    elif test_option ==2:
         test_pid_rampdrive()
    elif test_option ==3:
        test_robot_turns (turn_option=3)
    elif test_option ==4:
        test_drive_dam()
    elif test_option ==5:
        test_robot_drive_straight (drive_option=2)
    elif test_option ==6:
        test_drive_straight_pastwarning()
    elif test_option ==7:
        test_piston_rackpinion()
    elif test_option ==8:
        test_drive_distance(speed = 800, distance=300)
    elif test_option ==9:
        test_L1_smooth_drive()
    elif test_option ==10:
        ramp_test()    
    else:
        l4_navigation()