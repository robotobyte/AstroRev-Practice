import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

i2c = board.I2C()  # uses board.SCL and board.SDA

acc_gyr_sensor = LSM6DSOX(i2c)

while True:

    print ( "Acceleromater: X:%7.3f, Y: %7.3f, Z: %7.3f m/s^2"     % (acc_gyr_sensor.acceleration) )
    print ( "Gyroscope    : X:%7.3f, Y: %7.3f, Z: %7.3f radians/s" % (acc_gyr_sensor.gyro)         )

    print('')

    time.sleep(1.0)

