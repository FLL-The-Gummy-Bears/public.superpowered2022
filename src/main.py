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
import missions as missions

#create dictionary that maps keyword to each launch, then use it to define Launch order
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
        "Gyro Calibration": (lambda: missions.robot.gyro_calib()),
        "Robot Warmup": (lambda: missions.robot.robot_warmup(warmup_time=500)),
        "Power Plant": (lambda: missions.launch_powerplant()),
        "Hydro Units": (lambda: missions.launch_hydrounits()),
        "Smart Car": (lambda: missions.launch_smartcar()),
        "Energy Oil": (lambda: missions.launch_energyoil())
    }

    while True:
        # display mission list and hightlight the selected one

        small_font = Font(size=12)       # 10 pixel height for text on screen
        large_font = Font(size=14)       # 10 pixel height for text on screen

        missions.robot.ev3.screen.set_font(small_font)   #Choose a preset font for writing next texts

        if repaint:
            missions.robot.ev3.screen.clear()
            missions.robot.ev3.screen.print("Super Powered Missions:")
            missions.robot.ev3.screen.print("")
            for i in range(num_missions):
                if i == selectedMission_ID:
                    prefix = ">>> "
                else:
                    prefix = "     "
                missions.robot.ev3.screen.print(prefix + "L" + str(i) + ". " + missionList[i] )
                repaint = False  

        # button up/down to select, loop over if reach the end
        
        button_pressed = missions.robot.ev3.buttons.pressed()

        up_pressed = Button.UP in button_pressed
        down_pressed = Button.DOWN in button_pressed
        center_pressed = Button.CENTER in button_pressed

        if center_pressed:
            missions.robot.ev3.screen.print("Mission selected, preparing  ...")
            missions.robot.ev3.light.on(Color.RED)

            # find mission key and run the corresponding launch from mission dictionary
            mission_key = missionList[selectedMission_ID]
            print(mission_key)
            mission_dictionary[mission_key]()

            # update to next mission
            selectedMission_ID += 1
            wait(5)
            missions.robot.ev3.screen.print("Mission finished, updated to next one or use UP/DOWN to select ...")

            missions.robot.ev3.light.on(Color.GREEN)
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
        else:
            pass
        wait(2)
    return selectedMission_ID

def main():

    selection = 0

    timer1 = StopWatch()
    timer1.reset()

    if selection is 0:
        # use the exact keys from mission_dictionary to define mission list
        missionList= ["Robot Warmup","Power Plant", "Energy Oil", "Hydro Units", "Smart Car" ]
        displayMenu(missionList)
    if selection is 1:
        # use the exact keys from mission_dictionary to define mission list
        missionList= ["Robot Warmup","Power Plant", "Hydro Units", "Smart Car", "Energy Oil"]
        displayMenu(missionList)
    if selection is 2:
        #power plant
        timer1.reset()
        missions.launch_powerplant()
    if selection is 3:
        timer1.reset()
        missions.launch_energyoil()
    if selection is 4:
        timer1.reset()
        missions.launch_hydrounits()
    if selection is 5:
        timer1.reset()
        missions.launch_smartcar()

    print("mission time:", timer1.time()/1000)


if __name__=="__main__":
    main()




