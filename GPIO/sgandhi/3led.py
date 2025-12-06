import RPi.GPIO as GPIO
import time



GPIO.setmode ( GPIO.BOARD )  # use header pin numbers, not GPIO numbers

ledPin1 = 36 
ledPin2 = 37
ledPin3 = 38



GPIO.setup ( ledPin1, GPIO.OUT ) 
GPIO.setup(ledPin2, GPIO.OUT)
GPIO.setup(ledPin3, GPIO.OUT)


while True:
    GPIO.output ( ledPin1, GPIO.LOW  )  
    time.sleep ( 3 )  # wait three seconds
    GPIO.output ( ledPin1, GPIO.HIGH )  
    time.sleep ( 3 )  # wait three seconds
    GPIO.output ( ledPin2, GPIO.LOW  )  
    time.sleep ( 3 )  # wait three seconds
    GPIO.output ( ledPin2, GPIO.HIGH )  
    time.sleep ( 3 )  # wait three seconds
    GPIO.output ( ledPin3, GPIO.LOW  )  
    time.sleep ( 3 )  # wait three seconds
    GPIO.output ( ledPin3, GPIO.HIGH )  
    time.sleep ( 3 )  # wait three seconds
