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

ledPinRed = 15  # GPIO 22
ledPinYellow = 16 #GPIO 23
ledPinGreen = 18 #GPIO 24


# *** Set Up Pin Behavior Modes *** 

GPIO.setup ( ledPinRed, GPIO.OUT )  # set pin as output
GPIO.setup ( ledPinYellow, GPIO.OUT )
GPIO.setup ( ledPinGreen, GPIO.OUT )
# *** Drive Pins ***
GPIO.output ( ledPinRed, GPIO.LOW  )  # drive pin low  (off, 0V)
GPIO.output ( ledPinYellow, GPIO.LOW  )  # drive pin low  (off, 0V)
GPIO.output ( ledPinGreen, GPIO.LOW  )  # drive pin low  (off, 0V)

while True:
    # green 1 second
    GPIO.output ( ledPinGreen, GPIO.HIGH  )  
    time.sleep ( 1 )  # wait 1 seconds
    GPIO.output ( ledPinGreen, GPIO.LOW  )  
    #yellow 1 and then yellow and green 1
    GPIO.output ( ledPinYellow, GPIO.HIGH )  
    time.sleep ( 1 )  # wait 1 seconds
    GPIO.output ( ledPinGreen, GPIO.HIGH  )  
    time.sleep ( 1 )  # wait 1 seconds
    GPIO.output ( ledPinGreen, GPIO.LOW  )  
    GPIO.output ( ledPinYellow, GPIO.LOW  )  
  #red 1 and then red and green 1
    GPIO.output ( ledPinRed, GPIO.HIGH  )  
    time.sleep ( 1 )  # wait 1 seconds
    GPIO.output ( ledPinGreen, GPIO.HIGH  )
    time.sleep ( 1 )  # wait 1 seconds
    GPIO.output ( ledPinGreen, GPIO.LOW  )
    GPIO.output ( ledPinYellow, GPIO.HIGH  )
    time.sleep ( 1 )  # wait 1 seconds
    GPIO.output ( ledPinGreen, GPIO.HIGH  )
    time.sleep ( 1 )  # wait 1 seconds
  #close all
    GPIO.output ( ledPinRed, GPIO.LOW  )  # drive pin low  (off, 0V)
    GPIO.output ( ledPinYellow, GPIO.LOW  )  # drive pin low  (off, 0V)
    GPIO.output ( ledPinGreen, GPIO.LOW  )  # drive pin low  (off, 0V)
