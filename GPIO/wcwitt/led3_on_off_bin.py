#!/usr/bin/python3

# *****************************************************************************
# Raspberry Pi Python program to light up LEDs in a binary counting pattern.
# *****************************************************************************
# Wolf Witt, 2025-12-05
# *****************************************************************************


# GPIO Control Library
# --------------------
# https://pypi.org/project/rpi-lgpio/
# https://rpi-lgpio.readthedocs.io/en/latest/
# https://rpi-lgpio.readthedocs.io/en/latest/api.html


# *** Import Libraries ***

import RPi.GPIO as GPIO
import time


# *** Set Up Pin Assignments ***

GPIO.setmode ( GPIO.BOARD )

ledPinList = ( 11, 13, 15 )  # GPIO 17, 27, 22


# *** Set Up Pin Behavior Modes *** 

for ledPin in ledPinList :
    GPIO.setup  ( ledPin, GPIO.OUT )


# *** Drive Pins ***

progressCount = 0
while True :
    for ledIndex in range ( 0, len(ledPinList) ) :
        ledState = GPIO.HIGH if progressCount & 0x01 << ledIndex else GPIO.LOW
        GPIO.output ( ledPinList[ledIndex], ledState )
    progressCount += 1
    time.sleep ( 1 )
