#!/usr/bin/python3

# *****************************************************************************
# Raspberry Pi Python program to toggle one LED on/off indefinitely.
# *****************************************************************************
# Wolf Witt, 2025-12-03
# *****************************************************************************

# GPIO Control Library Reference
# ------------------------------
# https://pypi.org/project/rpi-lgpio/
# https://rpi-lgpio.readthedocs.io/en/latest/
# https://rpi-lgpio.readthedocs.io/en/latest/api.html


# *** Import Libraries ***

import RPi.GPIO as GPIO
import time


# *** Set Up Pin Assignments ***

GPIO.setmode ( GPIO.BOARD )  # use header pin numbers, not GPIO numbers

ledPin = 36  # GPIO 16


# *** Set Up Pin Behavior Modes *** 

GPIO.setup ( ledPin, GPIO.OUT )  # set pin as output


# *** Drive Pins ***

while True:
    GPIO.output ( ledPin, GPIO.LOW  )  # drive pin low  (off, 0V)
    time.sleep ( 3 )  # wait three seconds
    GPIO.output ( ledPin, GPIO.HIGH )  # drive pin high (on, 3.3V)
    time.sleep ( 3 )  # wait three seconds