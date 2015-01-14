#!/usr/bin/python
# -*- coding: utf-8 -*-
__author__ = 'Martijn van Leeuwen'
# Copyright 2015 Martijn van Leeuwen, VOC Vanleeuwen Opensource Consultancy, The Netherlands

# Control inteface for the IWO-538
# As there are 6 dc servo's
# U need at least 3 L293D's to controle them
#
# The Raspberry Pi has not enough GPIO ports by default.
# So you need to solder some extra points on your pi.
# To get enough, or use an IO extender.
#
# Current To Do:
#
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.



import RPi.GPIO as io
io.setmode(io.BCM)
import sys, tty, termios, time
from termcolor import colored, cprint

#Setup the motors
# Use Default Pin 0 if it's not in use.
motor1_in1_pin = 17
motor1_in2_pin = 4
motor2_in1_pin = 24
motor2_in2_pin = 25
motor3_in1_pin = 22
motor3_in2_pin = 27
motor4_in1_pin = 0
motor4_in1_pin = 0
motor5_in2_pin = 0
motor5_in2_pin = 0

#Setup the pins.
io.setup(motor1_in1_pin, io.OUT)
io.setup(motor1_in2_pin, io.OUT)
io.setup(motor2_in1_pin, io.OUT)
io.setup(motor2_in2_pin, io.OUT)
io.setup(motor3_in1_pin, io.OUT)
io.setup(motor3_in2_pin, io.OUT)
io.setup(motor4_in1_pin, io.OUT)
io.setup(motor4_in2_pin, io.OUT)
io.setup(motor5_in1_pin, io.OUT)
io.setup(motor5_in2_pin, io.OUT)

# LED setup
io.setup(18, io.OUT)
io.output(18, False)
io.setup(23, io.OUT)
io.output(23, False)

#Define input
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Define motor settings
def motor1_forward():
    io.output(motor1_in1_pin, True)
    io.output(motor1_in2_pin, False)

def motor1_reverse():
    io.output(motor1_in1_pin, False)
    io.output(motor1_in2_pin, True)

def motor1_stop():
    io.output(motor1_in1_pin, False)
    io.output(motor1_in2_pin, False)
 
def motor2_forward():
    io.output(motor2_in1_pin, True)
    io.output(motor2_in2_pin, False)

def motor2_reverse():
    io.output(motor2_in1_pin, False)
    io.output(motor2_in2_pin, True)

def motor2_stop():
    io.output(motor2_in1_pin, False)
    io.output(motor2_in2_pin, False)

def motor3_forward():
    io.output(motor3_in1_pin, True)
    io.output(motor3_in2_pin, False)

def motor3_reverse():
    io.output(motor3_in1_pin, False)
    io.output(motor3_in2_pin, True)

def motor3_stop():
    io.output(motor3_in1_pin, False)
    io.output(motor3_in2_pin, False)

def motor4_forward():
  io.output(motor4_in1_pin, True)
  io.output(motor4_in2_pin, False)

def motor4_reverse():
  io.output(motor4_in1_pin, False)
  io.output(motor4_in2_pin, True)

def motor4_stop():
  io.output(motor4_in1_pin, False)
  io.output(motor4_in2_pin, False)

def motor5_forward():
  io.output(motor5_in1_pin, True)
  io.output(motor5_in2_pin, False)

def motor5_reverse():
  io.output(motor5_in1_pin, False)
  io.output(motor5_in2_pin, True)

def motor5_stop():
  io.output(motor5_in1_pin, False)
  io.output(motor5_in2_pin, False)

def motors_stop():
  io.output(motor1_in1_pin, False)
  io.output(motor1_in2_pin, False)
  io.output(motor2_in1_pin, False)
  io.output(motor2_in2_pin, False)
  io.output(motor3_in1_pin, False)
  io.output(motor3_in2_pin, False)
  io.output(motor4_in1_pin, False)
  io.output(motor4_in2_pin, False)
  io.output(motor5_in1_pin, False)
  io.output(motor5_in2_pin, False)

def toggleLights():
    global lightStatus
    if(lightStatus == False):
        io.output(18, True)
        io.output(23, True)
        lightStatus = True
    else:
        io.output(18, False)
        io.output(23, False)
        lightStatus = False

def togglearm(direction):
  global arm1Status

  if(direction == "forward"):
    if(arm1Status == "center"):
      motor2_forward()
      arm1Status = "forward"
    elif(arm1Status == "backward"):
      motor2_stop()
      arm1Status = "center"

  if(direction == "backward"):
    if(arm1Status == "center"):
      motor2_reverse()
      arm1Status = "backward"
    elif(arm1Status == "forward"):
      motor2_stop()
      arm1Status = "center"

def togglearm2(direction):
  global arm2Status

  if(direction == "forward"):
    if(arm2Status == "center"):
      motor3_forward()
      arm2Status = "forward"
    elif(arm2Status == "backward"):
      motor3_stop()
      arm2Status = "center"

  if(direction == "backward"):
    if(arm2Status == "center"):
      motor3_reverse()
      arm2Status = "backward"
    elif(arm2Status == "forward"):
      motor3_stop()
      arm2Status = "center"

def toggleSteering(direction):
    global wheelStatus

    if(direction == "right"):
        if(wheelStatus == "centre"):
            motor1_forward()
            wheelStatus = "right"
        elif(wheelStatus == "left"):
            motor1_stop()
            wheelStatus = "centre"

    if(direction == "left"):
        if(wheelStatus == "centre"):
            motor1_reverse()
            wheelStatus = "left"
        elif(wheelStatus == "right"):
            motor1_stop()
            wheelStatus = "centre"

# Main 
motors_stop()

lightStatus = False
wheelStatus = "centre"
arm1Status = "center"
arm2Status = "center"
arm3Status = "center"
clawStatus = "center"

bold = "\033[1m"
reset = "\033[0;0m"
cprint("OWI-535 Robot Arm Conrole", 'blue')
print " " + bold + "w/s:" + reset + " arm back/forth"
print " " + bold + "a/d:" + reset + " steer left/right"
print " " + bold + "z/c:" + reset + " arm2 up/down"
print " " + bold + "r/f:" + reset + " arm3 up/down"
print " " + bold + "q/e:" + reset + " claw open/close"
print " " + bold + "l  :" + reset + " lights on/off"
print " " + bold + "h  :" + reset + " all stop"
print " " + bold + "x  :" + reset + " exit app."
print("p: General Info")

while True:
  char = getch()
  if(char == "w"):
    togglearm("forward")

  if(char == "s"):
    togglearm("backward")

  if(char == "d"):
    toggleSteering("left")

  if(char == "a"):
    toggleSteering("right")

  if(char == "z"):
    togglearm2("forward")

  if(char == "c"):
    togglearm2("backward")

  if(char == "l"):
    toggleLights()

  if(char == "x"):
    cprint("Program Ended", 'red')
    break
  char = ""

io.cleanup()
