#!/usr/bin/python3

# *****************************************************************************
# Raspberry Pi Python program to cycle through (i.e. light up) a set of
# LEDs in sequence.
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

GPIO.setmode ( GPIO.BOARD )  # use header pin numbers, not GPIO numbers

ledPinList = ( 11, 13, 15 )  # GPIO 17, 27, 22


# *** Set Up Pin Behavior Modes And Set To Initial States *** 

for ledPin in ledPinList :
    GPIO.setup  ( ledPin, GPIO.OUT  )  # set pin as output
    GPIO.output ( ledPin, GPIO.LOW  )  # initialize pin to low (off, 0V)


# *** Drive Pins ***

while True :
    for ledPin in ledPinList :
        GPIO.output ( ledPin, GPIO.HIGH )  # drive pin to high (on,  3.3V)
        time.sleep ( 1 )  # wait one second
        GPIO.output ( ledPin, GPIO.LOW  )  # drive pin to low  (off, 0V  )
