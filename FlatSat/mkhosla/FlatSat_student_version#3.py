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

#AUTHOR: Mahika Khosla
#DATE: Jan 6, 2026

#import libraries
import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
from git import Repo
from picamera2 import Picamera2
# for case code structure
import sys
from enum import Enum 

#VARIABLES
THRESHOLD = 0.5      #Any desired value from the accelerometer
REPO_PATH = "home/astrorev/Documents/AstroRev-Practice" #Your github repo path: ex. /home/pi/FlatSatChallenge
FOLDER_PATH = "FlatSat/mkhosla" #Your image folder path in your GitHub repo: ex. /Images
EARTH_G = 9.8 # in m/s^2
WAIT_TIME = 3 # in seconds

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


def take_photo(): # the big question is how do I embed action into the states?
    """
    This function is NOT complete. Takes a photo when the FlatSat is shaken.
    Replace psuedocode with your own code.
    """
    # Define state symbols
    class State(Enum):
        STILL = 0
        IN_MOTION = 1
        TIMER_INITIALIZE = 2
        MOTION_STOPPED = 3
        PICTURE_TIME = 4
        PICTURE_IN_GIT = 5

    # Initializing state variable
    currentState = State.STILL

    # setting up the camera
    preview_config = picam2.creat_preview_configuration() # sets up standard preview
    picam2.configure(preview_config)
    picam2.start()

    # Initialize all input/event variables (not sure how to deal w/ these)
    acc_above_thresh = False
           
    while True:
        accelx, accely, accelz = accel_gyro.acceleration
        accel_magnitude = abs(math.sqrt((accelx**2) + (accely**2) + (accelz**2)) - EARTH_G) # setting true value of acceleration
        acc_above_thresh = accel_magnitude > THRESHOLD
        
        timer_current = time.monotonic()

        # Perform current-to-next state transitions
        match currentState:
            case State.STILL:
                if acc_above_thresh:
                    nextState = State.IN_MOTION
                else:
                    nextState = State.STILL
            case State.IN_MOTION:
                if acc_above_thresh:
                    nextState = State.IN_MOTION
                else:
                    nextState = State.TIMER_INITIALIZE
            case State.TIMER_INITIALIZE:
                # moore machine example
                timer_start = timer_current # reading the timer once
                if acc_above_thresh:
                    nextState = State.IN_MOTION
                else:
                    nextState = State.MOTION_STOPPED
            case State.MOTION_STOPPED: # here we are waiting for time to expire
                if acc_above_thresh:
                    nextState = State.IN_MOTION
                else:
                    if timer_current - timer_start < WAIT_TIME:
                        nextState = State.MOTION_STOPPED
                    else:
                        nextState = State.PICTURE_TIME
            case State.PICTURE_TIME:
                if not acc_above_thresh:
                    # mealy machine example - actions within arcs
                    picam2.capture_file(img_gen(MahikaK))
                    picam2.stop_preview() # stopping preview and camera when done
                    picam2.stop()
                    nextState = State.PICTURE_IN_GIT
                else:
                    nextState = State.IN_MOTION
            case State.PICTURE_IN_GIT:
                git_push() # let's see if this line works
                nextState = State.STILL # unconditional

        print (f"{currentState.name} -> {nextState.name}")
        
        # Update state for next iteration
        currentState = nextState

def main():
    take_photo()


if __name__ == '__main__':
    main()
