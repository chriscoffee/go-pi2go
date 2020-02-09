#!/usr/bin/python
#
# pi2go2.py
#
# Python Module to externalise all Pi2Go2 specific hardware
#
# Copyright 4tronix
#
# This code is in the public domain and may be freely copied and used
# No warranty is provided or implied
#
#======================================================================


#======================================================================
# General Functions
# (Both versions)
#
# init(brightness). Initialises GPIO pins, switches motors and LEDs Off. If brightness is 0, no LEDs are initialised
# cleanup(). Sets all motors and LEDs off and sets GPIO to standard values
# version(). Returns 3 for Pi2Go2. Invalid until after init() has been called
#======================================================================


#======================================================================
# Motor Functions
#
# stop(): Stops both motors
# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# turnreverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
#======================================================================


#======================================================================
# Wheel Sensor Functions
#
# stepForward(speed, steps): moves the unit forwards specified number of steps, then stops
# stepReverse(speed, steps): Moves backward specified number of counts, then stops
# stepSpinL(speed, steps): Spins left specified number of counts, then stops
# stepSpinR(speed, steps): Spins right specified number of counts, then stops
#======================================================================


#======================================================================
# RGB LED Functions
#
# setColor(color): Sets all LEDs to color - requires show()
# setPixel(ID, color): Sets pixel ID to color - requires show()
# show(): Updates the LEDs with state of LED array
# clear(): Clears all LEDs to off - requires show()
# rainbow(): Sets the LEDs to rainbow colors - requires show()
# fromRGB(red, green, blue): Creates a color value from R, G and B values
# toRGB(color): Converts a color value to separate R, G and B
# wheel(pos): Generates rainbow colors across 0-255 positions
#======================================================================


#======================================================================
# IR Sensor Functions
#
# irLeft(): Returns state of Left IR Obstacle sensor
# irRight(): Returns state of Right IR Obstacle sensor
# irAll(): Returns true if any of the Obstacle sensors are triggered
# irLeftLine(): Returns state of Left IR Line sensor
# irRightLine(): Returns state of Right IR Line sensor
#======================================================================


#======================================================================
# UltraSonic Functions
#
# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
#======================================================================


#======================================================================
# Analog and Light Sensor Functions
#
# getLight(Sensor). Returns the value 0..1023 for the selected sensor, 0 <= Sensor <= 3
# getLightFL(). Returns the value 0..1023 for Front-Left light sensor
# getLightFR(). Returns the value 0..1023 for Front-Right light sensor
# getLightBL(). Returns the value 0..1023 for Back-Left light sensor
# getLightBR(). Returns the value 0..1023 for Back-Right light sensor
# getBattery(). Returns the voltage of the battery pack (>7.2V is good, less is bad)
#======================================================================


#======================================================================
# Switch Functions
#
# getSwitch(). Returns the value of the tact switch: True==pressed
#======================================================================


# Import all necessary libraries
import RPi.GPIO as GPIO, sys, time, os
import spidev
from rpi_ws281x import *

# Define Type of Pi2Go
PGNone = 0
PGFull = 1
PGLite = 2
PG2 = 3
PGType = PGNone # Set to None until we find out which during init()

# Pins 26, 19 Left Motor
# Pins 36, 40 Right Motor
L1 = 26
L2 = 19
R1 = 21
R2 = 16
# Variables to track movements to prevent sudden forward/reverse current surges. -1 reverse, 0 stop, +1 forward
lDir = 0
rDir = 0

# Define RGB LEDs
leds = None
_brightness = 40
numPixels = 10

# Define obstacle sensors and line sensors
irFL = 22      # Front Left obstacle sensor
irFR = 17      # Front Right obstacle sensor
lineLeft = 23  # Left Line sensor
lineRight = 27 # Right Line sensor

# Define Sonar Pin (same pin for both Ping and Echo)
sonar0 = 20    # front sonar
sonar1 = 4     # left side sonar

# Define pin for switch
switch = 15

# Define Wheel sensor pins
lSense = 5
rSense = 6

# Global variables for wheel sensor counting
countL = 0
countR = 0
targetL = -1
targetR = -1

# Global SPI variable for MCP3008
spi = 0

#======================================================================
# General Functions
#
# init(). Initialises GPIO pins, switches motors and LEDs Off, etc
def init(brightness):
    global p, q, a, b, pwm, pcfADC, PGType
    global irFL, irFR, irMID, lineLeft, lineRight
    global spi, leds

    GPIO.setwarnings(False)

    PGType = PG2

    #use physical pin numbering
    GPIO.setmode(GPIO.BCM)
    
    # Initialise LEDs
    if (leds == None and brightness>0):
        _brightness = brightness
        leds = Adafruit_NeoPixel(numPixels, 18, 800000, 5, False, _brightness)
        leds.begin()

    #set up digital line detectors as inputs
    GPIO.setup(lineRight, GPIO.IN) # Right line sensor
    GPIO.setup(lineLeft, GPIO.IN) # Left line sensor

    #Set up IR obstacle sensors as inputs
    GPIO.setup(irFL, GPIO.IN) # Left obstacle sensor
    GPIO.setup(irFR, GPIO.IN) # Right obstacle sensor

    #use pwm for motor outputs
    GPIO.setup(L1, GPIO.OUT)
    p = GPIO.PWM(L1, 20)
    p.start(0)

    GPIO.setup(L2, GPIO.OUT)
    q = GPIO.PWM(L2, 20)
    q.start(0)

    GPIO.setup(R1, GPIO.OUT)
    a = GPIO.PWM(R1, 20)
    a.start(0)

    GPIO.setup(R2, GPIO.OUT)
    b = GPIO.PWM(R2, 20)
    b.start(0)

    #set switch as input with pullup
    GPIO.setup(switch, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # initialise wheel counters
    GPIO.setup(lSense, GPIO.IN) # Left wheel sensor
    GPIO.setup(rSense, GPIO.IN) # Right wheel sensor
    GPIO.add_event_detect(lSense, GPIO.RISING, callback=lCounter)
    GPIO.add_event_detect(rSense, GPIO.RISING, callback=rCounter)

    # Initialise analog SPI MCP3008
    spi = spidev.SpiDev()
    spi.open(0,0)
    spi.max_speed_hz = 40000


# cleanup(). Sets all motors and LEDs off and sets GPIO to standard values
def cleanup():
    global running
    running = False
    stop()
    if (leds != None):
        clear()
        show()
    time.sleep(0.1)
    GPIO.cleanup()


# version(). Returns 3 for Pi2Go2. Invalid until after init() has been called
def version():
    return PGType

# End of General Functions
#======================================================================


#======================================================================
# Motor Functions
#
# stop(): Stops both motors - coasts slowly to a stop
def stop():
    global lDir, rDir
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(0)
    lDir = 0
    rDir = 0

# brake(): Stops both motors - regenrative braking to stop quickly
def brake():
    global lDir, rDir
    p.ChangeDutyCycle(100)
    q.ChangeDutyCycle(100)
    a.ChangeDutyCycle(100)
    b.ChangeDutyCycle(100)
    lDir = 0
    rDir = 0

# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
def forward(speed):
    global lDir, rDir
    if (lDir == -1 or rDir == -1):
        brake()
        time.sleep(0.2)
    p.ChangeDutyCycle(speed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(speed)
    b.ChangeDutyCycle(0)
    p.ChangeFrequency(max(speed/2, 10))
    a.ChangeFrequency(max(speed/2, 10))
    lDir = 1
    rDir = 1

# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
def reverse(speed):
    global lDir, rDir
    if (lDir == 1 or rDir == 1):
        brake()
        time.sleep(0.2)
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(speed)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(speed)
    q.ChangeFrequency(max(speed/2, 10))
    b.ChangeFrequency(max(speed/2, 10))
    lDir = -1
    rDir = -1

# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinLeft(speed):
    global lDir, rDir
    if (lDir == 1 or rDir == -1):
        brake()
        time.sleep(0.2)
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(speed)
    a.ChangeDutyCycle(speed)
    b.ChangeDutyCycle(0)
    q.ChangeFrequency(min(speed+5, 20))
    a.ChangeFrequency(min(speed+5, 20))
    lDir = -1
    rDir = 1

# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinRight(speed):
    global lDir, rDir
    if (lDir == -1 or rDir == 1):
        brake()
        time.sleep(0.2)
    p.ChangeDutyCycle(speed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(speed)
    p.ChangeFrequency(min(speed+5, 20))
    b.ChangeFrequency(min(speed+5, 20))
    lDir = 1
    rDir = -1

# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnForward(leftSpeed, rightSpeed):
    global lDir, rDir
    if (lDir == -1 or rDir == -1):
        brake()
        time.sleep(0.2)
    p.ChangeDutyCycle(leftSpeed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(rightSpeed)
    b.ChangeDutyCycle(0)
    p.ChangeFrequency(min(leftSpeed+5, 20))
    a.ChangeFrequency(min(rightSpeed+5, 20))
    lDir = 1
    rDir = 1

# turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnReverse(leftSpeed, rightSpeed):
    global lDir, rDir
    if (lDir == 1 or rDir == 1):
        brake()
        time.sleep(0.2)
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(leftSpeed)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(rightSpeed)
    q.ChangeFrequency(min(leftSpeed+5, 20))
    b.ChangeFrequency(min(rightSpeed+5, 20))
    lDir = -1
    rDir = -1

# End of Motor Functions
#======================================================================


#======================================================================
# Wheel Sensor Functions

def stopL():
    p.ChangeDutyCycle(100)
    q.ChangeDutyCycle(100)

def stopR():
    a.ChangeDutyCycle(100)
    b.ChangeDutyCycle(100)

def lCounter(pin):
    global countL, targetL
    countL += 1
    if countL == targetL:
        stopL()
        targetL = -1

def rCounter(pin):
    global countR, targetR
    countR += 1
    if countR == targetR:
        stopR()
        targetR = -1

# stepForward(speed, steps): Moves forward specified number of counts, then stops
def stepForward(speed, counts):
    global countL, countR, targetL, targetR
    countL = 0
    countR = 0
    targetL = counts-(speed/20)
    targetR = counts-(speed/20)
    forward(speed)
    while (targetL != -1) or (targetR != -1):
        time.sleep(0.002)

# stepReverse(speed, steps): Moves backward specified number of counts, then stops
def stepReverse(speed, counts):
    global countL, countR, targetL, targetR
    countL = 0
    countR = 0
    targetL = counts-(speed/20)
    targetR = counts-(speed/20)
    reverse(speed)
    while (targetL != -1) or (targetR != -1):
        time.sleep(0.002)

# stepSpinL(speed, steps): Spins left specified number of counts, then stops
def stepSpinL(speed, counts):
    global countL, countR, targetL, targetR
    countL = 0
    countR = 0
    targetL = counts-(speed/20)
    targetR = counts-(speed/20)
    spinLeft(speed)
    while (targetL != -1) or (targetR != -1):
        time.sleep(0.002)

# stepSpinR(speed, steps): Spins right specified number of counts, then stops
def stepSpinR(speed, counts):
    global countL, countR, targetL, targetR
    countL = 0
    countR = 0
    targetL = counts-(speed/20)
    targetR = counts-(speed/20)
    spinRight(speed)
    while (targetL != -1) or (targetR != -1):
        time.sleep(0.002)


# End of Wheel Sensor Functions
#======================================================================


#======================================================================
# IR Sensor Functions
#
# irLeft(): Returns state of Left IR Obstacle sensor
def irLeft():
    if GPIO.input(irFL)==0:
        return True
    else:
        return False

# irRight(): Returns state of Right IR Obstacle sensor
def irRight():
    if GPIO.input(irFR)==0:
        return True
    else:
        return False

# irAll(): Returns true if either of the Obstacle sensors are triggered
def irAll():
    if GPIO.input(irFL)==0 or GPIO.input(irFR)==0:
        return True
    else:
        return False

# irLeftLine(): Returns state of Left IR Line sensor
def irLeftLine():
    if GPIO.input(lineLeft)==0:
        return True
    else:
        return False

# irRightLine(): Returns state of Right IR Line sensor
def irRightLine():
    if GPIO.input(lineRight)==0:
        return True
    else:
        return False

# End of IR Sensor Functions
#======================================================================


#======================================================================
# UltraSonic Functions
#
# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
#
def getDistance(sonar = sonar0): # default to front sensor
    GPIO.setup(sonar, GPIO.OUT)
    # Send 10us pulse to trigger
    GPIO.output(sonar, True)
    time.sleep(0.00001)
    GPIO.output(sonar, False)
    start = time.time()
    count=time.time()
    GPIO.setup(sonar,GPIO.IN)
    while GPIO.input(sonar)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonar)==1 and time.time()-count<0.1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop-start
    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    return distance

# End of UltraSonic Functions
#======================================================================


#======================================================================
# Light Sensor Analog Functions
#
# read3008(sensor). Returns the value 0..1023 for the selected sensor, 0 <= Sensor <= 4
# Sensor 4 = battery raw voltage value (unconverted)
def readADC(sensor):
    global spi
    adc = spi.xfer2([1,(8+sensor)<<4, 0])
    data = (((adc[1]&3) << 8) + adc[2]) / 1023.0
    return data

# getLightFL(). Returns the value 0..1023 for Front-Left light sensor
def getLightFL():
    value  = readADC(1)
    return value

# getLightFR(). Returns the value 0..1023 for Front-Right light sensor
def getLightFR():
    value  = readADC(0)
    return value

# getLightRL(). Returns the value 0..1023 for Back-Left light sensor
def getLightRL():
    value  = readADC(3)
    return value

# getLightRR(). Returns the value 0..1023 for Back-Right light sensor
def getLightRR():
    value  = readADC(2)
    return value

# getBattery(). Returns the voltage of the battery pack as a float
def getBattery():
    raw = readADC(4)
    return (raw*12.15)

# End of Light Sensor Functions
#======================================================================


#======================================================================
# Switch Functions
#
# getSwitch(). Returns the value of the tact switch: True==pressed
def getSwitch():
    val = GPIO.input(switch)
    return (val == 0)
#
# End of switch functions
#======================================================================

#======================================================================
# RGB LED Functions
#
def setColor(color):
    for i in range(numPixels):
        setPixel(i, color)

def setPixel(ID, color):
    if (ID <= numPixels):
        leds.setPixelColor(ID, color)

def show():
    leds.show()

def clear():
    for i in range(numPixels):
        setPixel(i, 0)

def rainbow():
    i = 0
    for x in range(numPixels):
        setPixel(x, int(wheel(x * 256 / numPixels)))

def fromRGB(red, green, blue):
        return ((int(red)<<16) + (int(green)<<8) + blue)

def toRGB(color):
        return (((color & 0xff0000) >> 16), ((color & 0x00ff00) >> 8), (color & 0x0000ff))

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return fromRGB(255 - pos * 3, pos * 3, 0) # Red -> Green
    elif pos < 170:
        pos -= 85
        return fromRGB(0, 255 - pos * 3, pos * 3) # Green -> Blue
    else:
        pos -= 170
        return fromRGB(pos * 3, 0, 255 - pos * 3) # Blue -> Red

#
# End of RGB LED Functions
#==============================================================
