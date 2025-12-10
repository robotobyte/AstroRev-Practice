import RPi.GPIO as GPIO
import time

LED_PINS = [17, 27, 22]

GPIO.setmode(GPIO.BCM)
for pin in LED_PINS:
    GPIO.setup(pin, GPIO.OUT)

try:
    while True:
        for pin in LED_PINS:
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(pin, GPIO.LOW)
except KeyboardInterrupt:
    GPIO.cleanup()
