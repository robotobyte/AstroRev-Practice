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

        GPIO.output(LED1, 1)
        GPIO.output(LED2, 0)
        GPIO.output(LED3, 1)
        time.sleep(1)

        GPIO.output(LED1, 0)
        GPIO.output(LED2, 1)
        GPIO.output(LED3, 1)
        time.sleep(1)


        # Step 5 (all on)
        GPIO.output(LED1, 1)
        GPIO.output(LED2, 1)
        GPIO.output(LED3, 1)
        time.sleep(1)

        # Step 6 (all off)
        GPIO.output(LED1, 0)
        GPIO.output(LED2, 0)
        GPIO.output(LED3, 0)
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()

#red LED is 1.8V, yellow LED 2.1, green LED is 2.2
#red resistor: 300 ohms
#yellow resistor: 240 ohms
#green resistor: 220 ohms
