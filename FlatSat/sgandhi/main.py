import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
from git import Repo
from picamera2 import Picamera2


THRESHOLD = 0.5
REPO_PATH = "home/astrorev/Documents/AstroRev-Practice"
FOLDER_PATH = "flatsat/agupta"
EARTHG = 9.81

i2c = board.I2C()
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)
picam2 = Picamera2()


def git_push():

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

    t = time.strftime("_%H%M%S")
    imgname = (f'{REPO_PATH}/{FOLDER_PATH}/{name}{t}.jpg')
    return imgname


def take_photo():

    STILL = 0
    MOVING = 1
    INIT_TIMER = 2
    STOPPED = 3
    TAKE_PIC = 4
    POISON = 5

    state = STILL
    stop_start_time = 0
    STABLE_TIME = 2.0

    name = "Sidn"

    picam2.configure(picam2.create_still_configuration())
    picam2.start()

    while True:
        ax, ay, az = accel_gyro.acceleration


        motion = ((ax ** 2 + ay ** 2 + az ** 2) ** 0.5) - EARTHG > THRESHOLD


        if state == STILL:
            if motion:
                state = MOVING


        elif state == MOVING:
            if not motion:
                state = INIT_TIMER


        elif state == INIT_TIMER:
            stop_start_time = time.monotonic()
            state = STOPPED


        elif state == STOPPED:
            if motion:
                state = MOVING
            else:
                if time.monotonic() - stop_start_time >= STABLE_TIME:
                    state = TAKE_PIC


        elif state == TAKE_PIC:
            filename = img_gen(name)
            picam2.capture_file(filename)
            print("Saved:", filename)

            try:
                git_push()
            except:
                pass


            if motion:
                state = POISON
            else:
                state = STILL


        elif state == POISON:

            print("Motion detected. Discarding photo.")


            if not motion:
                state = STILL


   


def main():
    take_photo()


if __name__ == '__main__':
    main()
