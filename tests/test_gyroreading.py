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

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, GyroSensor, ColorSensor)
from pybricks.media.ev3dev import ImageFile, SoundFile, Font
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.iodevices import Ev3devSensor
from pybricks.tools import wait, DataLog, StopWatch

#test function to check gyro reading

def check_sensor_reading (waitinterval=2):

    mysensor=Ev3devSensor(Port.S2)
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

if __name__=="__main__":
    check_sensor_reading(waitinterval=10)
