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
from robot import Robot
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import ImageFile, SoundFile, Font
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.experimental import run_parallel

# mission code calibrated for boxy-sunny
robot = Robot("Bunny")

def ms_powerplant_waterunit ():
    """ Navigates to the power plant mission, then move close to water units next to water reservoir
    Starting point: Corner of right home, robot back to 2boundary line, use jig to help alignment
    """
    robot.debug_mode(option_num=2)
    robot.timer.reset()
    robot.gyro_calib() 
    
    # no cart, dino on top of robot,  using gyro-turn and pid-drive
    # move forward to powerplant and use trigger to release energy unit
    robot.pid_rampdrive(speed=350, drive_distance=797, ramp_up_factor=0.05, ramp_down_factor=0.125)
    wait(100)
    robot.debug_mode(option_num=1)

    # sligntly move back to dis-engage the trigger attachment
    robot.gyro_ramp_turn(turn_angle=1.5, turn_rate=50)
    robot.pid_rampdrive(speed=-200, drive_distance=70)
    robot.debug_mode(option_num=2)
    
    #drive towards water resovior
    def drive_water_resovior():
        robot.driver_angle(target_angle=-58, speed=200, distance=640, kp=3.0, stop=True)

    #open arm to capture water units
    def open_arm():
        robot.rmmotor.run_until_stalled(speed=400, duty_limit=50) # Done
        wait(100)
        robot.rmmotor.run_angle(speed=-200,rotation_angle=190) # Done

     # run water resovior and capture units in parallel
    run_parallel (drive_water_resovior, open_arm)

def ms_waterunit_solarfarm_home ():
    """ Navigates to the water units to collect units 1&2 and then to solar farm to collect units 3, 2 &1, 
    then return to left home and collect water unit 3
    Starting point: near water resevior and close to solar farm  
    """
    robot.debug_mode(option_num=1)
    #move towards solar farm and captures units next to smart grid
    robot.driver_angle(target_angle=-80, speed=80, distance=60, kp=3.0, stop=False)
    robot.drive_angle_pastwarning(speed=120, target_angle=-91, max_distance=150, distance_after_warning=25, reflection_white=93,stop=True, kp=3.0)
    robot.debug_mode(1)

    #turn towards other two energy units on left, angle can be slightly adjusted to avoid collide with 2 EUs
    robot.driver_angle(target_angle=-35, speed=100, distance=65, stop=False, kp=2.0)

    #robot navigating to last two units
    def navigate_2_units():
        robot.driver_angle(target_angle=-14, speed=100, distance=62,kp=2.0, stop=False)
        robot.driver_angle(target_angle=-6, speed=100, distance=105,kp=2.0, stop=True)

    #robot closing arm to capture two units
    def close_arm():
        wait(1200)
        robot.rmmotor.run_until_stalled(speed=400, duty_limit=50) # Done
    
    #navigate to and capture 2 energy units
    run_parallel (navigate_2_units, close_arm)
    
    robot.driver_angle(target_angle=0, speed=-150, distance=40, stop=False)
    #go hommeeee
    robot.driver_angle(target_angle=40, speed=250, distance=100,kp=3.0, stop=False)
    robot.driver_angle(target_angle=60, speed=500, distance=220,kp=3.0, stop=False)
    robot.driver_angle(target_angle=45, speed=800, distance=350,kp=3.0, stop=True)

    robot.rmmotor.run_angle(speed=-400,rotation_angle=100)
    robot.rmmotor.stop()
    robot.complete_stop(stop_option=3)                        #The robot completely stops

def ms_energystorage_oilrig_truck ():
    """ This mission completes three tasks : energy storage, oil rig, 
    and oil truck missions with the help of combined attachment.
    Starting point: wheel should be on the inner edge of the thick black line nearest to left home
    """
    robot.debug_mode(option_num=2)
    robot.timer.reset()
    robot.gyro_calib()

     # drive forward to latch oil truck , use large kp here to fight for lean towards right
     # should use feedforward here
    robot.pid_rampdrive(speed=200,drive_distance=620,kp=3.0, feedforward=-0.2, interval=0)
    wait(100)

#    robot.gyro_turn(turn_angle=-2.0,turn_rate=30)
#    robot.pid_rampdrive(speed=-50,drive_distance=5)
#    wait(100)

    # push energy unit attachment to the targets then release back
    rotation_option=0
    runtime=3400
    speed=1000
    if rotation_option==0:
        robot.rmmotor.run_time(speed=-speed,time=runtime)
    else:
        robot.rmmotor.run_angle(speed=-speed,rotation_angle=2400)

    robot.rmmotor.brake()

    def retract_motor():
        if rotation_option==0:
            robot.rmmotor.run_time(speed=speed,time=runtime)
        else:
            robot.rmmotor.run_angle(speed=speed,rotation_angle=2400)

    # drive back and completely return home
    def move_back():
        wait(700)
        robot.pid_rampdrive(speed=-300,drive_distance=475,kp=2,ramp_up_factor=0.3)

    # run parallel to save time
    run_parallel(retract_motor,move_back)

    # drive back and completely return home
    robot.gyro_turn(turn_angle=110,turn_rate=1000)
    wait(100)

    robot.rmmotor.stop()
    # use stop option 3 to avoid robot go weired
    robot.complete_stop(stop_option=3)
    print(robot.timer.time())

def ms_hydroelectric_dam_lefthome ():
    """ Navigates to the hydroelectric dam mission and hang two water units on the bars, leave one in water reservoir
    use liftarm to lift the bar and release/capture energy units
    Starting point: Left Launch area, 2M from Left boundary, align on the wall
    Note the robot may keep turning which suspected to be gyro-calib issue (need gyro calibration blocks)
    """
    robot.debug_mode(option_num=2)
    robot.timer.reset()
    robot.gyro_calib()

    def prepare_align():
        # drive slightly forward
        robot.pid_rampdrive(speed=100, drive_distance=37)

        # turn to facing hydro-dam
        robot.gyro_ramp_turn_correction(turn_rate=150, turn_angle =-50)
        robot.debug_mode(option_num=1)

    def close_rightarm():
        # close the right arm
        robot.rmmotor.run_until_stalled(speed=-80, duty_limit=None)

    def move_close_dam():
        # move close to hydro-dam before the water unit to allow folk capture
        robot.pid_rampdrive(speed=150,drive_distance=160)
    
    # right arm close is after the turn
    prepare_align ()
    run_parallel(move_close_dam, close_rightarm)

    # capture water unit 3 and swing it to right side to avoid collision (slow swing)
#    robot.lmmotor.stop()
    robot.rmmotor.run_until_stalled(speed=60, duty_limit=60)
#    robot.rmmotor.run_time(speed = 100, time = 600)

    # move forward with jig to align on model
    robot.driver_angle(target_angle= -48, speed=150, distance=240, stop=True)
    robot.lmmotor.stop()

    robot.debug_mode(option_num=1)

    def leftarm_control():
        # raise left arm to release water unit, this part can be threaded
        robot.lmmotor.run_until_stalled(speed=300, duty_limit=60)
        wait(100)
        robot.lmmotor.run_until_stalled(speed=-300, duty_limit=60)
        wait(100)
        robot.lmmotor.stop()

    def rightarm_control():
        # swing right arm to place water units in position
        robot.rmmotor.run_until_stalled(speed=-300, duty_limit=60)
        robot.debug_mode(option_num=1)
    
        # realign robot on mission model with small angle adjust
        robot.gyro_turn(turn_angle=1.5,turn_rate=50)
        robot.rmmotor.stop()

    # run the two actions (release energy units and hang up water units) in parallel
    run_parallel(leftarm_control, rightarm_control)

    # drive back towards home to de-latch the two water units on the bars
    robot.driver_angle(target_angle=-55, speed=-250, distance=130)
    robot.debug_mode(option_num=1)

    def return_lefthome ():
    # move back to make sure completely inside launch area
        robot.driver_angle(target_angle=-90, speed=-350, distance=340, kp=3.0)
        robot.complete_stop()

    # close right arm, this part can be parallel with drive straight 
    def close_right_arm():
        robot.rmmotor.run_until_stalled(speed=-150, duty_limit=80)

    run_parallel(return_lefthome, close_right_arm)
        #print(robot.gyro.angle())

def ms_innovation_righthome ():

    # wait sufficient long to put energy units/innovation project into front part to be delievered into Power to X
    #having jig which helps reset robot alignment
    robot.debug_mode(3)
    robot.gyro.reset_angle(-90)

    # drive toward the Power-X circle
    robot.driver_angle(target_angle=-90, speed=400, distance=140)
    robot.driver_angle(target_angle=-43, speed = 600, distance=640, kp=3, stop=True)

    # move back to leave EU and innovation project in PowerX
    robot.driver_angle(target_angle=-43, speed=-300, distance=100)

    # turn towards right home and return
    def go_right_home():
        robot.driver_angle(target_angle=-100, speed=800, distance=220, kp=3)
        robot.driver_angle(target_angle=-115, speed=800, distance=380, kp=3)
        robot.driver_angle(target_angle=-100, speed=800, distance=400, kp=3)

    def close_rightarm():
        # close the right arm to avoid collide with powerplant
        robot.rmmotor.run_until_stalled(speed=-80, duty_limit=None)
        #robot.rmmotor.stop()

    run_parallel(close_rightarm,go_right_home)

    robot.lmmotor.stop()
    robot.rmmotor.stop()
    robot.complete_stop(stop_option=3)
        
def ms_watchtv_windturbine (debug_mode=False):

    """Starting to right launching area, complete missions of watchTv, windturbine, solarbattery and toyfactory, 
    then leave the 3 energy units on solar battery field, and prepare to next missions (hybrid car & smart grid)
    Starting point: right boundary of right launching area, make sure the hybrid car lift arm is on the black 
    boundary line
    """ 
    robot.debug_mode(option_num=2)
    robot.timer.reset()
    robot.gyro_calib()

    # Watch television 
    def watchtv ():
        robot.pid_rampdrive(speed=200,drive_distance=383,ramp_down_factor=0.30)

    def raiseup_rightarm():    
        # raise up hybrid car liftarm to avoid collisions
        robot.rmmotor.run_until_stalled(speed=-200, duty_limit=50)
        robot.rmmotor.stop()
    
    # run two tasks in parallel
    run_parallel(watchtv,raiseup_rightarm)
    robot.debug_mode(option_num=2)

    #Navigate to wind turbine: drive back, turn left, 
    robot.pid_rampdrive(speed=-150,drive_distance=78)
    robot.debug_mode(option_num=2)
    robot.gyro_ramp_turn_correction(turn_angle=45 - robot.gyro.angle(),turn_rate=170)
    robot.debug_mode(option_num=2)

    # move forward, then turn right facing turbine (-45 deg)
    robot.pid_rampdrive(speed=300,drive_distance=474)
    robot.debug_mode(option_num=2)
    robot.gyro_ramp_turn_correction(turn_angle= -45 -robot.gyro.angle(),turn_rate=150)

    # Wind turbine, push three time to dump the 3 energy units
    robot.debug_mode(option_num=1)
    robot.pid_rampdrive(speed=200,drive_distance=140) # 140mm
    # pushing power can not be too high (prefer 350~450)
    for i in range(2):
        wait(225)
        robot.drive_straight(speed=400,distance=-60)
        wait(900)
        robot.drive_straight(speed=500,distance=68)
    wait(100)

def ms_solarbattery_toyfactory (debug_mode=False):
    """Starting after windturbine, navigate backwards to complete mission of solarbattery and toyfactory, 
    then leave the 3 energy units on solar battery field, and prepare to next missions (hybrid car & smart grid)
    Starting point: Facing windturbine, 50 mm away from color line
    """ 

    # Solar battery and toy factory, move back to collect solar unit in battery field
    robot.pid_rampdrive(speed=-200,drive_distance=280)

    # move slightly forward and prepare toy factory mission
    robot.pid_rampdrive(speed=200,drive_distance=150)
    wait(200)
    #turn to toy factory to leave the 3 windturbine energy units on field using global angle 135
    robot.gyro_ramp_turn_correction(turn_angle = 135 -robot.gyro.angle(), turn_rate = 350)  

    if debug_mode is True:
        print("current angle:", robot.gyro.angle())

    # move close to solar battery field, slightly move back and use left motor to lift cart.
    robot.drive_time(speed=150,drive_time=700)
    wait(200)
    robot.pid_rampdrive(speed=-100,drive_distance=30)
    wait(150)

    # lift 
    # cart to keep energy units on battery field
    robot.lmmotor.run_time(speed=-600, time=800)
    robot.lmmotor.stop()

    # position adjustment to face toyfactory before move back
#    robot.gyro_ramp_turn_correction(turn_angle = 135 -robot.gyro.angle(), turn_rate = 150)
    # move backward slightly to allow space for turn to next mission
    robot.pid_rampdrive(speed=-120, drive_distance=124)

    # turn to color marking line for next mission
    robot.gyro_ramp_turn_correction(turn_angle= 89.5 -robot.gyro.angle(),turn_rate=150)

def ms_hybridcar_smartgrid_home (solution=2):
    """After toy factory, position to finish hybrid car and smart grid mission, then return to left home
    Starting point: color marking lines next to hybrid car
    """ 
    robot.debug_mode(option_num=1)
    #After toy factory, navigate to smart grid/hybrid car
    # move close to color marking line and do line squaring there.
    distance_1 = robot.driver.distance()

    realign_option=0

    def realign_robot():
        # realign robot on color lines then go forward (or option 1 realign based on color line detection when driving through)
        # three options: double color line alignment or realign_correction

        if realign_option == 0:
            # double line alignment to robot position and re-set angle
            robot.pid_rampdrive(speed=200,drive_distance=140)
            robot.linesquaring(align_color="white",align_speed=100)
            robot.linesquaring(align_color="black",align_speed=60)
    #       wait(200)

            # reset angle to east (90)
            robot.gyro.reset_angle(90)
            distance_2 = robot.driver.distance()
            print("Distance:", distance_2 - distance_1)
        
        # move forward from color lines to prepare the missions (142~145mm)
            robot.pid_rampdrive(speed=150,drive_distance=111)

        elif realign_option == 1:
            # smooth drive with automatic color line dection and position adjustment
            robot.pid_rampdrive(speed=200,drive_distance=140)
            robot.realign_correction(distance_after_detection=110, speed=80, max_distance=200,stop=True, kp=2.0)
            wait(200)
            robot.gyro.reset_angle(90)           

        else:
            pass

    def prepare_rightarm():
        # return right liftarm to poision for hybrid car
        wait(700)
        robot.rmmotor.run_until_stalled(speed = 500, duty_limit=40)
        robot.rmmotor.stop()

    # run two tasks in parallel
    run_parallel(realign_robot, prepare_rightarm)
    robot.debug_mode(option_num=2)

    # parking robot between hybrid car and smart grid
    parking_option=1

    if parking_option ==1: # sunny

        # robot turn 45 deg to parallel to hybrid car 
#        robot.gyro_ramp_turn_correction(turn_angle=44 -robot.gyro.angle(),turn_rate=100)
        robot.gyro_ramp_turn(turn_angle=45 -robot.gyro.angle(),turn_rate=50)
        wait(100)
        robot.pid_rampdrive(speed=100,drive_distance=70)
        robot.debug_mode(option_num=2)

        # Adjust robot position and quickly raise right liftarm to complete hybrid car
        robot.gyro_ramp_turn(turn_angle=41 - robot.gyro.angle(),turn_rate=50)
        robot.debug_mode(option_num=1)
        robot.rmmotor.run_until_stalled(speed = -500, duty_limit=None)
        robot.rmmotor.run_until_stalled(speed = 300, duty_limit=None)
    
        # double security turns
        robot.gyro_ramp_turn(turn_angle=38 -robot.gyro.angle(),turn_rate=50)
        robot.debug_mode(option_num=1)
        robot.rmmotor.run_until_stalled(speed = -500, duty_limit=None)

        # slightly turn right and move forward to latch smart grid
        robot.gyro_ramp_turn(turn_angle=30 -robot.gyro.angle(),turn_rate=50)
        robot.pid_rampdrive(speed=50,drive_distance=20)
        robot.debug_mode(option_num=1)

        # slightly turn to use trigger pull the smart grid
        robot.gyro_ramp_turn(turn_angle= 50 -robot.gyro.angle(),turn_rate=200)
        robot.debug_mode(option_num=1)
        wait(100)

        # return to vertical position
        robot.gyro_ramp_turn(turn_angle= 25 -robot.gyro.angle(),turn_rate=100)
        robot.debug_mode(option_num=1)
        
    elif parking_option == 2: # boxy-blue
        # robot turn 45 deg to parallel to hybrid car 
#        robot.gyro_ramp_turn_correction(turn_angle=44 -robot.gyro.angle(),turn_rate=100)
        robot.gyro_ramp_turn(turn_angle=45 -robot.gyro.angle(),turn_rate=50)
        wait(100)
        robot.pid_rampdrive(speed=100,drive_distance=80)

        # Adjust robot position and quickly raise right liftarm to complete hybrid car
        robot.gyro_ramp_turn(turn_angle=33 -robot.gyro.angle(),turn_rate=50)
        robot.debug_mode(option_num=1)
        robot.rmmotor.run_until_stalled(speed = -500, duty_limit=None)

        # slightly turn right and move forward to latch smart grid
        robot.gyro_ramp_turn(turn_angle=30 -robot.gyro.angle(),turn_rate=50)
        robot.pid_rampdrive(speed=50,drive_distance=20)
        robot.debug_mode(option_num=1)

        robot.pid_rampdrive(speed=-50,drive_distance=55)
        robot.debug_mode(option_num=3)
        # slightly turn to use trigger pull the smart grid
        #robot.gyro_ramp_turn(turn_angle= 50 -robot.gyro.angle(),turn_rate=200)
        robot.debug_mode(option_num=1)
        wait(100)

        # return to vertical position
        robot.gyro_ramp_turn(turn_angle= 25 -robot.gyro.angle(),turn_rate=100)
        robot.debug_mode(option_num=1)
    else:
        # robot turn 45 deg to parallel to hybrid car 
        # robot.gyro_ramp_turn_correction(turn_angle=44 -robot.gyro.angle(),turn_rate=100)
        robot.gyro_ramp_turn(turn_angle=45 -robot.gyro.angle(),turn_rate=50)
        wait(100)
        robot.pid_rampdrive(speed=100,drive_distance=80)

        # Adjust robot position and quickly raise right liftarm to complete hybrid car
        robot.gyro_ramp_turn(turn_angle=33 -robot.gyro.angle(),turn_rate=50)
        robot.debug_mode(option_num=1)
        robot.rmmotor.run_until_stalled(speed = -500, duty_limit=None)

        # slightly turn right and move forward to latch smart grid
        robot.gyro_ramp_turn(turn_angle=30 -robot.gyro.angle(),turn_rate=50)
        robot.pid_rampdrive(speed=50,drive_distance=20)
        robot.debug_mode(option_num=1)

        # slightly turn to use trigger pull the smart grid
        robot.gyro_ramp_turn(turn_angle= 50 -robot.gyro.angle(),turn_rate=200)
        robot.debug_mode(option_num=1)
        wait(100)

        # return to vertical position
        robot.gyro_ramp_turn(turn_angle= 25 -robot.gyro.angle(),turn_rate=100)
        robot.debug_mode(option_num=1)        

    # move back , turn to west (90 deg) and prepare go home left
    robot.pid_rampdrive(speed=-150,drive_distance=115)
    robot.gyro_ramp_turn_correction(turn_angle= 90 -robot.gyro.angle(),turn_rate=200)

    robot.debug_mode(option_num=1)

    # return home with curved no-stop drive 
    robot.driver_angle(target_angle=93, speed=500, distance=150, kp=3)
    robot.driver_angle(target_angle=75, speed=600, distance=150, kp=3)
    robot.driver_angle(target_angle=83, speed=700, distance=250, kp=3)
    robot.driver_angle(target_angle=143, speed=800, distance=870, kp=3, stop=True)

    robot.complete_stop(stop_option=1)
    robot.rmmotor.stop()
    print("timer:", robot.timer.time())

def ms_deliver_oiltruck1():
    """After Launch 4 if we have sufficient time (> 12 sec), this mission will carry oil truck from left to right home, 
    then park it in gas station.
    """ 
    robot.debug_mode(option_num=3)
    robot.ev3.speaker.beep()

    robot.pid_rampdrive(speed=800, drive_distance=1825)
    robot.complete_stop(stop_option=3)

def ms_deliver_oiltruck2():
    # reposition robot by hand to facing towards parking
    robot.debug_mode(option_num=3)
    robot.ev3.speaker.beep()
    robot.pid_rampdrive(speed=500,drive_distance=900)
    robot.complete_stop(stop_option=3)
    
def launch_powerplant():
    robot.ev3.screen.print(" Mission PowerPlant ")
    ms_powerplant_waterunit()
    ms_waterunit_solarfarm_home ()
    print("time:", robot.timer.time())

def launch_hydrounits ():
    robot.ev3.screen.print(" Mission Hydro-Electric Dam ")
    ms_hydroelectric_dam_lefthome()
    ms_innovation_righthome()
    print("time:", robot.timer.time())

def launch_smartcar ():
    robot.ev3.screen.print(" TV & Smart Car ")
    ms_watchtv_windturbine()
    ms_solarbattery_toyfactory ()
    ms_hybridcar_smartgrid_home(solution=2)
    ms_deliver_oiltruck1()
    ms_deliver_oiltruck2()

    print("time:", robot.timer.time())

def launch_energyoil ():
    robot.ev3.screen.print(" Energy Oil")
    ms_energystorage_oilrig_truck()
    print("time:", robot.timer.time())

def deliver_truck_gasstation():
    robot.ev3.screen.print("Truck to Gas Station")
    ms_deliver_oiltruck2()

def displayMenu(missionList):
    """Display list of Mission on ev3 screen and use up/down botton to navigate the menu list
        Args: 
            missionList: list of missions to be launched, e,g ["innovationproject", "hybrid car", "TV"]
            the name of each item has to be exactly the keys of the mission dictionary.
    """
    selectedMission_ID = 0
    num_missions = len(missionList)
    repaint = True

    # missionList_keys= ["Gyro Calibration", "Power Plant","Hydro Units","Smart Car", "Energy Oil"]
    mission_dictionary = { 
        "Gyro Calibration": (lambda: robot.gyro_calib()),
        "Robot Warmup": (lambda: robot.robot_warmup(warmup_time=800)),
        "Power Plant": (lambda: launch_powerplant()),
        "Hydro Units": (lambda: launch_hydrounits()),
        "Smart Car": (lambda: launch_smartcar()),
        "Energy Oil": (lambda: launch_energyoil()),
        "Gas Station": (lambda: deliver_truck_gasstation())

    }

    while True:
        # display mission list and hightlight the selected one

        small_font = Font(size=12)       # 10 pixel height for text on screen
        large_font = Font(size=14)       # 10 pixel height for text on screen

        robot.ev3.screen.set_font(small_font)   #Choose a preset font for writing next texts

        if repaint:
            robot.ev3.screen.clear()
            robot.ev3.screen.print("Super Powered Missions:")
            robot.ev3.screen.print("")
            for i in range(num_missions):
                if i == selectedMission_ID:
                    prefix = ">>> "
                else:
                    prefix = "     "
                robot.ev3.screen.print(prefix + "L" + str(i) + ". " + missionList[i] )
                repaint = False  

        # button up/down to select, loop over if reach the end
        
        button_pressed = robot.ev3.buttons.pressed()

        up_pressed = Button.UP in button_pressed
        down_pressed = Button.DOWN in button_pressed
        center_pressed = Button.CENTER in button_pressed

        if center_pressed:
            robot.ev3.screen.print("Mission selected, preparing  ...")
            robot.ev3.light.on(Color.RED)

            # find mission key and run the corresponding launch from mission dictionary
            mission_key = missionList[selectedMission_ID]
            print(mission_key)
            mission_dictionary[mission_key]()

            # update to next mission
            selectedMission_ID += 1
            wait(50)
            robot.ev3.screen.print("Mission finished, updated to next one or use UP/DOWN to select ...")

            robot.ev3.light.on(Color.GREEN)
            repaint = True

        elif up_pressed:
            selectedMission_ID = selectedMission_ID - 1
            if selectedMission_ID < 0:
                selectedMission_ID = num_missions -1
            repaint = True

        elif down_pressed:
            selectedMission_ID = selectedMission_ID + 1
            if selectedMission_ID >= num_missions:
                selectedMission_ID = 0
            repaint = True
        wait(3)
    return selectedMission_ID

if __name__=="__main__":

    selection = 0

    timer1 = StopWatch()
    timer1.reset()

    if selection is 0:
        # use the exact keys from mission_dictionary to define mission list
        missionList= ["Robot Warmup", "Power Plant", "Energy Oil", "Hydro Units", "Smart Car" ]
        displayMenu(missionList)
    if selection is 1:
        # use the exact keys from mission_dictionary to define mission list
        missionList= ["Robot Warmup", "Power Plant", "Energy Oil", "Hydro Units", "Gas Station"]
        displayMenu(missionList)
    if selection is 2:
        # use the exact keys from mission_dictionary to define mission list
        missionList= ["Power Plant", "Energy Oil", "Hydro Units", "Smart Car"]
        displayMenu(missionList)
    if selection is 3:
        # use the exact keys from mission_dictionary to define mission list
        missionList= ["Gas Station"]
        displayMenu(missionList)
    if selection is 4:
        timer1.reset()
        launch_hydrounits()
    if selection is 5:
        timer1.reset()
        launch_smartcar()

    print("mission time:", timer1.time()/1000)