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
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
#from git import Repo
from picamera2 import Picamera2
import math

#VARIABLES
THRESHOLD = 0.5      #Any desired value from the accelerometer
REPO_PATH = "/home/astrorev/Documents/AstroRev-Practice"     #Your github repo path: ex. /home/pi/FlatSatChallenge
FOLDER_PATH = "FlatSat/agupta"   #Your image folder path in your GitHub repo: ex. /Images
EARTHG = 9.81

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
    # ----- STATE CONSTANTS -----
    STILL = 0
    MOVING = 1
    INIT_TIMER = 2
    STOPPED = 3
    TAKE_PIC = 4
    POISON = 5

    state = STILL
    stop_start_time = 0
    STABLE_TIME = 2.0   # seconds device must stay still before photo

    name = "Ayaan"

    picam2.configure(picam2.create_still_configuration())
    picam2.start()

    while True:
        ax, ay, az = accel_gyro.acceleration

        # total acceleration magnitude
        motion = ((ax**2 + ay**2 + az**2) ** 0.5)- EARTHG > THRESHOLD

        # ----------------- STILL -----------------
        if state == STILL:
            
            if motion:
                state = MOVING

        # ----------------- MOVING ----------------
        elif state == MOVING:
            if not motion:
                state = INIT_TIMER

        # ------------ INITIALIZE TIMER ----------
        elif state == INIT_TIMER:
            stop_start_time = time.monotonic()
            state = STOPPED

        # ---------------- STOPPED ----------------
        elif state == STOPPED:
            if motion:
                state = MOVING
            else:
                if time.monotonic() - stop_start_time >= STABLE_TIME:
                    state = TAKE_PIC

        # --------------- TAKE PICTURE ------------
        elif state == TAKE_PIC:
            filename = img_gen(name)
            picam2.capture_file(filename)
            print("Saved:", filename)

            try:
                pass
                #git_push()
            except:
                pass

            # After taking picture, check if motion returned
            if motion:
                state = POISON
            else:
                state = STILL

        # --------------- POISON PICTURE ----------
        elif state == POISON:
            # image was bad because motion resumed
            print("Motion detected. Discarding photo.")

            # do nothing to file, simply wait for stillness
            if not motion:
                state = STILL
        print(state)

        #CHECKS IF READINGS ARE ABOVE THRESHOLD
            #PAUSE
            #name = ""     #First Name, Last Initial  ex. MasonM
            #TAKE PHOTO
            #PUSH PHOTO TO GITHUB
        
        #PAUSE


def main():
    take_photo()


if __name__ == '__main__':
    main()
#0 is still
#1 is moving
#2 is starting timer
#3 is stopped again
#4 is taking photo
#5 is poison
