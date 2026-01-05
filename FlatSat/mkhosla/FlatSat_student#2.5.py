"""
The Python code you will write for this module should read
acceleration data from the IMU. When a reading comes in that surpasses
an acceleration threshold (indicating a shake), your Pi should pause,
trigger the camera to take a picture, then save the image with a
descriptive filename. You may use GitHub to upload your images automatically,
but for this activity it is not required.

The provided functions are only for reference, you do not need to use them. 
You will need to complete the take_photo() function and configure the VARIABLES section
"""

#AUTHOR: 
#DATE:

#import libraries
import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS # of accelerometer and gyroscope
from adafruit_lis3mdl import LIS3MDL # of magnetometer
from git import Repo
from picamera2 import Picamera2, Preiew # added preview feature
import math # for acceleration vector math

#VARIABLES
THRESHOLD = 0.05      #Any desired value from the accelerometer (taking into account g)
REPO_PATH = "home/astrorev/Documents/AstroRev-Practice"     #Your github repo path: ex. /home/pi/FlatSatChallenge
FOLDER_PATH = "FlatSat/mkhosla"   #Your image folder path in your GitHub repo: ex. /Images
EARTH_G = 9.81
TIME_OF_NO_MOTION = 1

#imu and camera initialization
i2c = board.I2C()
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)
picam2 = Picamera2()


def git_push():
    """
    This function is complete. Stages, commits, and pushes new images to your GitHub repo.
    """
    try:
        repo = Repo(REPO_PATH)
        origin = repo.remote('origin')
        print('added remote')
        origin.pull()
        print('pulled changes')
        repo.git.add(REPO_PATH + FOLDER_PATH)
        repo.index.commit('New Photo')
        print('made the commit')
        origin.push()
        print('pushed changes')
    except:
        print('Couldn\'t upload to git')


def img_gen(name):
    """
    This function is complete. Generates a new image name.

    Parameters:
        name (str): your name ex. MasonM
    """
    t = time.strftime("_%H%M%S")
    imgname = (f'{REPO_PATH}/{FOLDER_PATH}/{name}{t}.jpg')
    return imgname


def take_photo():
    """
    Takes a photo when the FlatSat is shaken.
    
    """
    
    time_start = time.monotonic() # capture timestamp in second (returns floating pt number); time of last detected motion
    picture_taken = False

    preview_config = picam2.creat_preview_configuration() # sets up standard preview
    picam2.configure(preview_config)
    picam2.start()
    
    while True:
        accelx, accely, accelz = accel_gyro.acceleration # this is an API call
        accel_magnitude = abs(math.sqrt((accelx**2) + (accely**2) + (accelz**2)) - EARTH_G) # setting true value of acceleration
        print ("Acceleration (m/s^2):", accel_magnitude)

        time_current = time.monotonic() # this value is ALWAYS increasing
        
        if accel_magnitude >= THRESHOLD:
            
            print ("acceleration detected")
            time_start = time_current # resetting value of timer based on motion detected
            picture_taken = False

        else: # so when acceleration is below threshold 
            time_elapsed = time_current - time_start

            if time_elapsed >= TIME_OF_NO_MOTION and not picture_taken: # chatgpt told me to the "and not"; also I arbitrarily set timer_delay = 1
                picam2.capture_file(img_gen(MahikaK))
                picture_taken = True
                picam2.stop_preview() # stopping preview and camera when done
                picam2.stop()
                git_push()

       

                    

    


def main():
    take_photo()


if __name__ == '__main__':
    main()
