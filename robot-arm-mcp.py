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
# Current ToDo:
# Implement a MCP23008 or MCP23017.
#
# As there is not enough power to supply more then 2 motors at the same time, its best to use one motor at a time
# To prevent using multiple motors by default, a failsafe is build in.
# This failsave can be overridden by setting the FAILSAVE variable to False.
#

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from Adafruit_I2C import Adafruit_I2C
import smbus
import RPi.GPIO as io
io.setmode(io.BCM)
import sys, tty, termios, time
from termcolor import colored, cprint

VERSION = "0.2.0"

# Settings for the MCP230017
MCP23017_IODIRA = 0x00
MCP23017_IODIRB = 0x01
MCP23017_GPIOA  = 0x12
MCP23017_GPIOB  = 0x13
MCP23017_GPPUA  = 0x0C
MCP23017_GPPUB  = 0x0D
MCP23017_OLATA  = 0x14
MCP23017_OLATB  = 0x15
MCP23008_GPIOA  = 0x09
MCP23008_GPPUA  = 0x06
MCP23008_OLATA  = 0x0A

class Adafruit_MCP230XX(object):
    OUTPUT = 0
    INPUT = 1

    def __init__(self, address, num_gpios, busnum=-1):
        assert num_gpios >= 0 and num_gpios <= 16, "Number of GPIOs must be between 0 and 16"
        self.i2c = Adafruit_I2C(address=address, busnum=busnum)
        self.address = address
        self.num_gpios = num_gpios

        # set defaults
        if num_gpios <= 8:
            self.i2c.write8(MCP23017_IODIRA, 0xFF)  # all inputs on port A
            self.direction = self.i2c.readU8(MCP23017_IODIRA)
            self.i2c.write8(MCP23008_GPPUA, 0x00)
        elif num_gpios > 8 and num_gpios <= 16:
            self.i2c.write8(MCP23017_IODIRA, 0xFF)  # all inputs on port A
            self.i2c.write8(MCP23017_IODIRB, 0xFF)  # all inputs on port B
            self.direction = self.i2c.readU8(MCP23017_IODIRA)
            self.direction |= self.i2c.readU8(MCP23017_IODIRB) << 8
            self.i2c.write8(MCP23017_GPPUA, 0x00)
            self.i2c.write8(MCP23017_GPPUB, 0x00)

    def _changebit(self, bitmap, bit, value):
        assert value == 1 or value == 0, "Value is %s must be 1 or 0" % value
        if value == 0:
            return bitmap & ~(1 << bit)
        elif value == 1:
            return bitmap | (1 << bit)

    def _readandchangepin(self, port, pin, value, currvalue = None):
        assert pin >= 0 and pin < self.num_gpios, "Pin number %s is invalid, only 0-%s are valid" % (pin, self.num_gpios)
        #assert self.direction & (1 << pin) == 0, "Pin %s not set to output" % pin
        if not currvalue:
             currvalue = self.i2c.readU8(port)
        newvalue = self._changebit(currvalue, pin, value)
        self.i2c.write8(port, newvalue)
        return newvalue


    def pullup(self, pin, value):
        if self.num_gpios <= 8:
            return self._readandchangepin(MCP23008_GPPUA, pin, value)
        if self.num_gpios <= 16:
            lvalue = self._readandchangepin(MCP23017_GPPUA, pin, value)
            if (pin < 8):
                return
            else:
                return self._readandchangepin(MCP23017_GPPUB, pin-8, value) << 8

    # Set pin to either input or output mode
    def config(self, pin, mode):
        if self.num_gpios <= 8:
            self.direction = self._readandchangepin(MCP23017_IODIRA, pin, mode)
        if self.num_gpios <= 16:
            if (pin < 8):
                self.direction = self._readandchangepin(MCP23017_IODIRA, pin, mode)
            else:
                self.direction |= self._readandchangepin(MCP23017_IODIRB, pin-8, mode) << 8

        return self.direction

    def output(self, pin, value):
        # assert self.direction & (1 << pin) == 0, "Pin %s not set to output" % pin
        if self.num_gpios <= 8:
            self.outputvalue = self._readandchangepin(MCP23008_GPIOA, pin, value, self.i2c.readU8(MCP23008_OLATA))
        if self.num_gpios <= 16:
            if (pin < 8):
                self.outputvalue = self._readandchangepin(MCP23017_GPIOA, pin, value, self.i2c.readU8(MCP23017_OLATA))
            else:
                self.outputvalue = self._readandchangepin(MCP23017_GPIOB, pin-8, value, self.i2c.readU8(MCP23017_OLATB)) << 8

        return self.outputvalue


        self.outputvalue = self._readandchangepin(MCP23017_IODIRA, pin, value, self.outputvalue)
        return self.outputvalue

    def input(self, pin):
        assert pin >= 0 and pin < self.num_gpios, "Pin number %s is invalid, only 0-%s are valid" % (pin, self.num_gpios)
        assert self.direction & (1 << pin) != 0, "Pin %s not set to input" % pin
        if self.num_gpios <= 8:
            value = self.i2c.readU8(MCP23008_GPIOA)
        elif self.num_gpios > 8 and self.num_gpios <= 16:
            value = self.i2c.readU8(MCP23017_GPIOA)
            value |= self.i2c.readU8(MCP23017_GPIOB) << 8
        return value & (1 << pin)

    def readU8(self):
        result = self.i2c.readU8(MCP23008_OLATA)
        return(result)

    def readS8(self):
        result = self.i2c.readU8(MCP23008_OLATA)
        if (result > 127): result -= 256
        return result

    def readU16(self):
        assert self.num_gpios >= 16, "16bits required"
        lo = self.i2c.readU8(MCP23017_OLATA)
        hi = self.i2c.readU8(MCP23017_OLATB)
        return((hi << 8) | lo)

    def readS16(self):
        assert self.num_gpios >= 16, "16bits required"
        lo = self.i2c.readU8(MCP23017_OLATA)
        hi = self.i2c.readU8(MCP23017_OLATB)
        if (hi > 127): hi -= 256
        return((hi << 8) | lo)

    def write8(self, value):
        self.i2c.write8(MCP23008_OLATA, value)

    def write16(self, value):
        assert self.num_gpios >= 16, "16bits required"
        self.i2c.write8(MCP23017_OLATA, value & 0xFF)
        self.i2c.write8(MCP23017_OLATB, (value >> 8) & 0xFF)

# RPi.GPIO compatible interface for MCP23017 and MCP23008

class MCP230XX_GPIO(object):
    OUT = 0
    IN = 1
    BCM = 0
    BOARD = 0
    def __init__(self, busnum, address, num_gpios):
        self.chip = Adafruit_MCP230XX(address, num_gpios, busnum)
    def setmode(self, mode):
        # do nothing
        pass
    def setup(self, pin, mode):
        self.chip.config(pin, mode)
    def input(self, pin):
        return self.chip.input(pin)
    def output(self, pin, value):
        self.chip.output(pin, value)
    def pullup(self, pin, value):
        self.chip.pullup(pin, value)

# Setup the MC23017
mcp = Adafruit_MCP230XX(address = 0x20, num_gpios = 16) # MCP23017

#Setup the motors
motor1_in1_pin = 7
motor1_in2_pin = 6
motor2_in1_pin = 5
motor2_in2_pin = 4
motor3_in1_pin = 3
motor3_in2_pin = 2
motor4_in1_pin = 8
motor4_in2_pin = 9
motor5_in1_pin = 10
motor5_in2_pin = 11

led1_pin = 0
led2_pin = 15

FAILSAVE = True # Set to False to allow the use of more then one motor at the same time.
# Warning: This might burn out your Pi and or MCP as you will need to make sure there is 
#           enough power available to handle this.
#           Common errors include the total freeze of the Raspberry PI.

# Setup Pins
mcp.config(motor1_in1_pin, mcp.OUTPUT)
mcp.config(motor1_in2_pin, mcp.OUTPUT)
mcp.config(motor2_in1_pin, mcp.OUTPUT)
mcp.config(motor2_in2_pin, mcp.OUTPUT)
mcp.config(motor3_in1_pin, mcp.OUTPUT)
mcp.config(motor3_in2_pin, mcp.OUTPUT)
mcp.config(motor4_in1_pin, mcp.OUTPUT)
mcp.config(motor4_in2_pin, mcp.OUTPUT)
mcp.config(motor5_in1_pin, mcp.OUTPUT)
mcp.config(motor5_in2_pin, mcp.OUTPUT)

# LED setup
mcp.config(led1_pin, mcp.OUTPUT)
mcp.config(led2_pin, mcp.OUTPUT)
mcp.output(led1_pin, False)
mcp.output(led2_pin, False)

MotorActive = False

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
    mcp.output(motor1_in1_pin, True)
    mcp.output(motor1_in2_pin, False)

def motor1_reverse():
    mcp.output(motor1_in1_pin, False)
    mcp.output(motor1_in2_pin, True)

def motor1_stop():
    mcp.output(motor1_in1_pin, False)
    mcp.output(motor1_in2_pin, False)

def motor2_forward():
    mcp.output(motor2_in1_pin, True)
    mcp.output(motor2_in2_pin, False)

def motor2_reverse():
    mcp.output(motor2_in1_pin, False)
    mcp.output(motor2_in2_pin, True)

def motor2_stop():
    mcp.output(motor2_in1_pin, False)
    mcp.output(motor2_in2_pin, False)

def motor3_forward():
    mcp.output(motor3_in1_pin, True)
    mcp.output(motor3_in2_pin, False)

def motor3_reverse():
    mcp.output(motor3_in1_pin, False)
    mcp.output(motor3_in2_pin, True)

def motor3_stop():
    mcp.output(motor3_in1_pin, False)
    mcp.output(motor3_in2_pin, False)

def motor4_forward():
    mcp.output(motor4_in1_pin, True)
    mcp.output(motor4_in2_pin, False)

def motor4_reverse():
    mcp.output(motor4_in1_pin, False)
    mcp.output(motor4_in2_pin, True)

def motor4_stop():
    mcp.output(motor4_in1_pin, False)
    mcp.output(motor4_in2_pin, False)

def motor5_forward():
    mcp.output(motor5_in1_pin, True)
    mcp.output(motor5_in2_pin, False)

def motor5_reverse():
    mcp.output(motor5_in1_pin, False)
    mcp.output(motor5_in2_pin, True)

def motor5_stop():
    mcp.output(motor5_in1_pin, False)
    mcp.output(motor5_in2_pin, False)

def motors_stop():
  global MotorActive
  mcp.output(motor1_in1_pin, False)
  mcp.output(motor1_in2_pin, False)
  mcp.output(motor2_in1_pin, False)
  mcp.output(motor2_in2_pin, False)
  mcp.output(motor3_in1_pin, False)
  mcp.output(motor3_in2_pin, False)
  mcp.output(motor4_in1_pin, False)
  mcp.output(motor4_in2_pin, False)
  mcp.output(motor5_in1_pin, False)
  mcp.output(motor5_in2_pin, False)
  MotorActive = False

def toggleLights():
    global lightStatus
    if(lightStatus == False):
      mcp.output(led1_pin, True)
      mcp.output(led2_pin, True)
      lightStatus = True
    else:
      mcp.output(led1_pin, False)
      mcp.output(led2_pin, False)
      lightStatus = False

def togglearm(direction):
  global arm1Status

  if(direction == "forward"):
    if(arm1Status == "center"):
      motor2_forward()
      arm1Status = "forward"
      MotorActive = True
    elif(arm1Status == "backward"):
      motor2_stop()
      arm1Status = "center"
      MotorActive = False

  if(direction == "backward"):
    if(arm1Status == "center"):
      motor2_reverse()
      arm1Status = "backward"
      MotorActive = True
    elif(arm1Status == "forward"):
      motor2_stop()
      arm1Status = "center"
      MotorActive = False

def togglearm2(direction):
  global arm2Status

  if(direction == "forward"):
    if(arm2Status == "center"):
      motor3_forward()
      arm2Status = "forward"
      MotorActive = True
    elif(arm2Status == "backward"):
      motor3_stop()
      arm2Status = "center"
      MotorActive = False

  if(direction == "backward"):
    if(arm2Status == "center"):
      motor3_reverse()
      arm2Status = "backward"
      MotorActive = True
    elif(arm2Status == "forward"):
      motor3_stop()
      arm2Status = "center"
      MotorActive = False

def togglearm3(direction):
  global arm3Status

  if(direction == "forward"):
    if(arm3Status == "center"):
      motor4_forward()
      arm3Status = "forward"
      MotorActive = True
    elif(arm3Status == "backward"):
      motor4_stop()
      arm3Status = "center"
      MotorActive = False

  if(direction == "backward"):
    if(arm3Status == "center"):
      motor4_reverse()
      arm3Status = "backward"
      MotorActive = True
    elif(arm3Status == "forward"):
      motor4_stop()
      arm3Status = "center"
      MotorActive = False

def togglearm4(direction):
  global clawStatus

  if(direction == "forward"):
    if(clawStatus == "center"):
      motor5_forward()
      clawStatus = "forward"
      MotorActive = True
    elif(clawStatus == "backward"):
      motor5_stop()
      clawStatus = "center"
      MotorActive = False

  if(direction == "backward"):
    if(clawStatus == "center"):
      moor5_reverse()
      clawStatus = "backward"
      MotorActive = True
    elif(clawStatus == "forward"):
      motor5_stop()
      clawStatus = "center"
      MotorActive = False

def toggleSteering(direction):
    global wheelStatus

    if(direction == "right"):
        if(wheelStatus == "center"):
            motor1_reverse()
            wheelStatus = "right"
            MotorActive = True
        elif(wheelStatus == "left"):
            motor1_stop()
            wheelStatus = "center"
            MotorActive = False

    if(direction == "left"):
        if(wheelStatus == "center"):
            motor1_forward()
            wheelStatus = "left"
            MotorActive = True
        elif(wheelStatus == "right"):
            motor1_stop()
            wheelStatus = "center"
            MotorActive = False

# Main
if __name__ == '__main__':
#  mcp.output(motor1_in1_pin, False)
#  mcp.output(motor1_in2_pin, False)
#  mcp.output(motor2_in1_pin, False)
#  mcp.output(motor2_in2_pin, False)
#  mcp.output(motor3_in1_pin, False)
#  mcp.output(motor3_in2_pin, False)
#  mcp.output(motor4_in1_pin, False)
#  mcp.output(motor4_in2_pin, False)
#  mcp.output(motor5_in1_pin, False)
#  mcp.output(motor5_in2_pin, False)
  motors_stop()
  lightStatus = False
  wheelStatus = "center"
  arm1Status = "center"
  arm2Status = "center"
  arm3Status = "center"
  arm4Status = "center"
  clawStatus = "center"

  bold = "\033[1m"
  reset = "\033[0;0m"
  cprint("OWI-535 Robot Arm Conrole", 'blue')
  cprint("Found an MP23017 on address 0x20", 'blue')
  print(" " + bold + "Version:" + reset + " : %s ", str(VERSION))
  print " " + bold + "w/s:" + reset + " arm back/forth"
  print " " + bold + "a/d:" + reset + " steer left/right"
  print " " + bold + "z/c:" + reset + " steer left/right"
  print " " + bold + "r/f:" + reset + " steer left/right"
  print " " + bold + "q/e:" + reset + " claw open/close"
  print " " + bold + "l  :" + reset + " lights"
  print " " + bold + "h  :" + reset + " all stop"
  print " " + bold + "x  :" + reset + " exit app."

  while True:
    char = getch()
    if(char == "w"):
      togglearm("forward")

    if(char == "s"):
      togglearm("backward")

    if(char == "a"):
      toggleSteering("left")

    if(char == "d"):
      toggleSteering("right")

    if(char == "z"):
      togglearm2("forward")

    if(char == "c"):
      togglearm2("backward")

    if(char == "r"):
      togglearm3("forward")

    if(char == "f"):
      togglearm3("backward")

    if(char == "q"):
      togglearm4("forward")

    if(char == "e"):
      togglearm4("backward")

    if(char == "l"):
      toggleLights()

    if(char == "h"):
      motors_stop()

    if(char == "x"):
      if (MotorActive == True):
        motors_stop()
      if (lightStatus == True):
        toggleLights()
      cprint("Program Ended", 'red')
      break
    char = ""

  mcp.cleanup()
