import time
import math
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6
from adafruit_lis3mdl import LIS3MDL
from git import Repo
from picamera2 import Picamera2
from enum import Enum

# Constants
MOTION_THRESHOLD = 0.5
REPO_PATH = "home/astrorev/Documents/AstroRev-Practice"
IMAGE_FOLDER = "FlatSat/vkhosla"
GRAVITY_ACCEL = 9.8
WAIT_AFTER_MOTION = 3  # seconds

# Initialize sensors and camera
i2c_bus = board.I2C()
imu_sensor = LSM6(i2c_bus)
mag_sensor = LIS3MDL(i2c_bus)
camera = Picamera2()

def push_to_github():
    """
    Automates the process of pushing new images to your repository.
    """
    try:
        repo = Repo(REPO_PATH)
        origin = repo.remote('origin')
        origin.pull()
        repo.git.add(REPO_PATH + '/' + IMAGE_FOLDER)
        repo.index.commit("Captured new image")
        origin.push()
        print("Images uploaded successfully.")
    except Exception as error:
        print(f"Failed to push images: {error}")

def create_image_filename(username):
    """
    Generate a filename incorporating username and current timestamp.
    """
    timestamp = time.strftime("_%H%M%S")
    filename = f"{REPO_PATH}/{IMAGE_FOLDER}/{username}{timestamp}.jpg"
    return filename

def detect_and_capture():
    """
    Monitors accelerometer readings and takes a photo when motion stops.
    """
    class State(Enum):
        IDLE = 0
        MOVING = 1
        START_TIMER = 2
        WAITING = 3
        SNAPSHOT = 4
        UPLOAD = 5

    current_state = State.IDLE
    next_state = current_state

    # Camera configuration
    cam_config = camera.create_preview_configuration()
    camera.configure(cam_config)
    camera.start()

    motion_detected = False
    timer_start_time = None

    while True:
        ax, ay, az = imu_sensor.acceleration
        total_accel = abs(math.sqrt(ax**2 + ay**2 + az**2) - GRAVITY_ACCEL)
        motion_detected = total_accel > MOTION_THRESHOLD
        current_time = time.monotonic()

        if current_state == State.IDLE:
            if motion_detected:
                next_state = State.MOVING
            else:
                next_state = State.IDLE

        elif current_state == State.MOVING:
            if motion_detected:
                next_state = State.MOVING
            else:
                next_state = State.START_TIMER
                timer_start_time = current_time

        elif current_state == State.START_TIMER:
            if motion_detected:
                next_state = State.MOVING
            else:
                if current_time - timer_start_time >= WAIT_AFTER_MOTION:
                    next_state = State.SNAPSHOT
                else:
                    next_state = State.START_TIMER

        elif current_state == State.SNAPSHOT:
            if not motion_detected:
                filename = create_image_filename("Vihaan")  # Replace with your name
                camera.capture_file(filename)
                camera.stop_preview()
                camera.stop()
                next_state = State.UPLOAD
            else:
                next_state = State.MOVING

        elif current_state == State.UPLOAD:
            push_to_github()
            camera.start()
            next_state = State.IDLE

        print(f"{current_state.name} -> {next_state.name}")
        current_state = next_state
        time.sleep(0.1)

def main():
    detect_and_capture()

if __name__ == "__main__":
    main()
