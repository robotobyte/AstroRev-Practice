import RPi.GPIO as GPIO
import time

LED1 = 17
LED2 = 27
LED3 = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)
GPIO.setup(LED3, GPIO.OUT)

try:
    while True:
        # Step 1
        GPIO.output(LED1, 1)
        GPIO.output(LED2, 0)
        GPIO.output(LED3, 0)
        time.sleep(1)

        # Step 2
        GPIO.output(LED1, 0)
        GPIO.output(LED2, 1)
        GPIO.output(LED3, 0)
        time.sleep(1)

        # Step 3
        GPIO.output(LED1, 1)
        GPIO.output(LED2, 1)
        GPIO.output(LED3, 0)
        time.sleep(1)

        # Step 4
        GPIO.output(LED1, 0)
        GPIO.output(LED2, 0)
        GPIO.output(LED3, 1)
        time.sleep(1)

        # Step 5 
        GPIO.output(LED1, 1)
        GPIO.output(LED2, 0)
        GPIO.output(LED3, 1)
        time.sleep(1)

        # Step 6 
        GPIO.output(LED1, 0)
        GPIO.output(LED2, 1)
        GPIO.output(LED3, 1)
        time.sleep(1)

        # Step 7
        GPIO.output(LED1, 1)
        GPIO.output(LED2, 1)
        GPIO.output(LED3, 1)
        time.sleep(1)

        # Step 8
        GPIO.output(LED1, 0)
        GPIO.output(LED2, 0)
        GPIO.output(LED3, 0)
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()

#red LED: 1.8V, 300 ohms
#yellow LED: 2.1V, 240 ohms
#green LED: 2.2V, 220 ohms

