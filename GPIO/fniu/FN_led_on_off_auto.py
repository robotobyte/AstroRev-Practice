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

#red is 1.8V
# yellow is 2.1V
# gren is 2.2 V

# *** Import Libraries ***

import RPi.GPIO as GPIO
import time


# *** Set Up Pin Assignments ***

GPIO.setmode ( GPIO.BOARD )  # use header pin numbers, not GPIO numbers


ledpins = (15, 16, 18) # gpio 22, 23, 24


# *** Set Up Pin Behavior Modes *** 

for pin in ledpins:
 GPIO.setup (pin, GPIO.OUT)
 GPIO.output (pin, GPIO.LOW)


while True:
 for pin in ledpins:
  GPIO.outpt(pin, GPIO.HIGH)
  time.sleep(1)
  GPIO.outpt(pin, GPIO.LOW)
