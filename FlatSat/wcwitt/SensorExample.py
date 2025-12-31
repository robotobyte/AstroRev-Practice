# *****************************************************************************
# Sensor Processing Code Example for FlatSat Project, 2025
# - Get input from gyroscope and accelerometer
# - Calibrate Earth g
# - Track cumulative rotation
# - Track velocity (as derived from acceleration)
# - Detect stillness and take some action in response
# - Optionally provide observability through prints and/or LEDs
# -----------------------------------------------------------------------------
# December 2025
# *****************************************************************************

# Import Libraries
# ----------------

from enum import Enum
import math
import numpy as np
import time
import board
import RPi.GPIO as GPIO
from adafruit_lsm6ds import AccelRange, GyroRange, Rate
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

# Set Configuartion Flags
# -----------------------

# Run configuration settings...
CFG_RUN_TIME_LIMIT                    = -1.0  # max runtime for sensor loop (in seconds; negative = run forever)
CFG_SENSOR_SAMPLE_INTERVAL            =  1.0  # time in seconds; should usually be zero except for debug
CFG_ALLOW_STATE_UPDATE_ONLY_ON_SAMPLE = False
CFG_CALIB_WAIT                        =  3.0  # time to wait
CFG_STILL_WAIT                        =  1.0  # when still, time to wait before action (in seconds)
CFG_PAUSE_WAIT                        =  1.0  # after action, time to wait before next potential action

# Observability settings via print to terminal...
OBS_PRINT_ANY          = True   # enable/disable printing of any observability messages
OBS_PRINT_STATE        = True   # enable/disable printing of control state and sample counts
OBS_PRINT_STATE_CHANGE = True   # enable/disable print on state change regardless of PRINT_STATE or PRINT_INTERVAL
OBS_PRINT_RAW          = True   # enable/disable printing of raw sensor values
OBS_PRINT_INST         = True   # enable/disable printing of instantaneous values
OBS_PRINT_CUMUL        = True   # enable/disable printing of cumululative value
OBS_PRINT_INTERVAL     = 1.0    # print interval time in seconds (negative = print always)

# Obesrvability via GPIO-connected LEDs...
OBS_LED = True

# Define and setup GPIO pins...
# ------------------------------

GPIO.setmode ( GPIO.BCM )  # can't use BOARD mode; BCM forced by other library (?)

# GPIO for output LEDs...

# Create enumerated type for LED instances...
class LedInstance(Enum) :
  ROT_NOW      = 0
  ROT_CUMUL    = 1
  ACC_NOW      = 2
  VEL_CUMUL    = 3
  STATE_READY  = 4
  STATE_ACTION = 5

# Assign observability LEDs to pins using GPIO pin numbers...
led_pin = dict()
led_pin [ LedInstance.ROT_NOW      ] =  6  # header pin 31
led_pin [ LedInstance.ROT_CUMUL    ] = 13  # header pin 33
led_pin [ LedInstance.ACC_NOW      ] = 19  # header pin 35
led_pin [ LedInstance.VEL_CUMUL    ] = 26  # header pin 37
led_pin [ LedInstance.STATE_READY  ] = 20  # header pin 38
led_pin [ LedInstance.STATE_ACTION ] = 21  # header pin 40

# Set up all LED pins as outputs...
for led_instance in LedInstance :
  GPIO.setup ( led_pin[led_instance], GPIO.OUT )

# Allocate data structure for whether LEDs should be lit...
led_on = dict()

# GPIO for input buttons...

# Create enumerated type of button instances...
class ButtonInstance(Enum) :
  RESET        = 0
  FORCE_CALIB  = 1
  FORCE_ACTION = 2

# Assign input buttons to pins using GPIO pin numbers...
button_pin = dict()
button_pin [ ButtonInstance.RESET        ] = 18  # header pin 12
button_pin [ ButtonInstance.FORCE_CALIB  ] = 23  # header pin 16
button_pin [ ButtonInstance.FORCE_ACTION ] = 24  # header pin 18

# Set up button pins as inputs with pull-up resistor...
for button_instance in ButtonInstance :
  GPIO.setup ( button_pin[button_instance], GPIO.IN, GPIO.PUD_UP )

# Connect Devices
# ---------------

# Set up for use of I2C interface (via RPi board.SCL and .SDA)...
i2c = board.I2C()

# Connect to LSM6DSOX acceleromter/gyro unit at default I2C address
# (default it 0x6a; alternate is 0x6b)...
acc_gyr = LSM6DSOX(i2c)

# Increase sample rate from default of 104Hz to better match
# run rate of this code on a RPi 4, which gets about 400 to
# 500 main loop iterations per second...
acc_gyr.accelerometer_data_rate = Rate.RATE_833_HZ
acc_gyr.gyro_data_rate          = Rate.RATE_833_HZ
# For reference, see also bottom of this file and this link:
# https://docs.circuitpython.org/projects/lsm6dsox/en/latest/examples.html#rate-test

# Define Constants
# ----------------

ACC_EARTH_G     = 9.8    # Earth's acceleration of gravity, 9.8 m/s^2
ACC_REL_SLOP    = 0.05   # Accept 5% of measurement inaccuracy (applied to comparison with g)
ACC_NOISE_LEVEL = 0.2    # Zero out values less than or equal to this value, assuming they're noise

GYR_NOISE_LEVEL = 0.01   # Zero out values less than or equal to this value, assuming they're noise

VEL_ZERO_THRESH = 0.001  # Zero out values less than or equal to this value, assuming they're rounding or accumulation errors

TWO_PI = 2 * math.pi     # 2*pi precalculated for convenience

SWITCH_DEBOUNCE_TIME = 200

# Define Enumerated Type for Control State Machine
# ------------------------------------------------

class ControlState(Enum):
    CALIBRATE_NOT_STABLE = 0  # device not yet stable for calibration
    CALIBRATE_STABLE     = 1  # device stable, waiting for timer before calibrating
    CALIBRATING          = 2  # calibrating
    READY                = 4  # ready for motion (then then take an action when the motion ceases)
    MOVING               = 5  # motion detected
    STILL                = 6  # no motion detected, waitin for timer before action
    ACTION               = 7  # take action (e.g. take photo)
    PAUSE                = 8  # take post-action pause

# Define Helper Functions
# -----------------------

def rot_is_zero ( np_rot ) :
  all_zero = True
  for rot_axis in np_rot.ravel() :
  # all_zero = all_zero and math.fabs(rot_axis) <= GYR_NOISE_LEVEL
    all_zero = all_zero and ( math.fabs(rot_axis) == 0.0 )
  return all_zero

def acc_is_only_g ( np_acc ) :
  acc_magnitude = np.linalg.norm(np_acc)
  acc_rel_earth_g = math.fabs ( acc_magnitude - ACC_EARTH_G ) / ACC_EARTH_G
  return acc_rel_earth_g <= ACC_REL_SLOP

def vel_is_zero ( np_vel ) :
  acc_magnitude = np.linalg.norm(np_vel)
  return acc_magnitude == 0

# Define Event Handler Functions
# ------------------------------

def handleReset () :
  pendingReset = True

def handleCalib () :
  pendingCalib = True

def handleAction () :
  pendingAction = True

# Attach event handlers to input buttons...
#GPIO.add_event_detect( button_pin [ RESET       ], GPIO.FALLING, handleReset,  SWITCH_DEBOUNCE_TIME )
#GPIO.add_event_detect( button_pin [ FORCE_CALIB ], GPIO.FALLING, handleCalib,  SWITCH_DEBOUNCE_TIME )
#GPIO.add_event_detect( button_pin [ FORCE_ACTION], GPIO.FALLING, handleAction, SWITCH_DEBOUNCE_TIME )
# FIXME: Need to debug "Failed to add edge detection" error.

# Allocate Numpy Matrices
# -----------------------

np_acc_filtered = np.zeros ( (3,1) )
np_gyr_filtered = np.zeros ( (3,1) )

np_rot_cumul = np.zeros( (3,1) )
np_vel_cumul = np.zeros( (3,1) )

np_rot_cumul_cos = np.zeros ( (3,1) )
np_rot_cumul_sin = np.zeros ( (3,1) )

np_g_calib = np.zeros( (3,1) )

# Initialize Loop Variables
# -------------------------

# Initialize time trackers...
time_start       = time.monotonic()  # system time in seconds with fractional part (float)
time_last_sample = time_start - CFG_SENSOR_SAMPLE_INTERVAL
time_last_print  = time_start

# Initialize state control variables...
control_state_current = ControlState.CALIBRATE_NOT_STABLE
sample_sensor_allowed_by_fsm = True

# Initialize sensor sample and observability print counters...
sample_count_total       = 0
sample_count_since_print = 0
print_count              = 0

# Initialize pending-event flags...
pendingReset  = False
pendingCalib  = False
pendingAction = False

try :
  while True :

    # Capture current time for current iteration of the loop...
    time_current = time.monotonic()

    # Determine whether to sample sensor(s) given configured sampling
    # inverval (interval should be zero except for debug)...
    sample_sensor_allowed_by_time = time_current - time_last_sample > CFG_SENSOR_SAMPLE_INTERVAL

    # --- Obtain Values From Sensors ---

    # Sample sensors and update calculations...
    if sample_sensor_allowed_by_time and sample_sensor_allowed_by_fsm :

      # Read sensors...
      acc_value_samples = acc_gyr.acceleration  # three-axis accelerometer; radian/sec
      gyr_value_samples = acc_gyr.gyro          # three-axis gyrscope; meter/sec^2
      sample_count_total       += 1
      sample_count_since_print += 1

      # Compute elapsed time since last sample...
      time_delta = time_current - time_last_sample

      # Convert tuples from sensors into column-major matrices...
      np_acc_raw = np.array ( object=acc_value_samples, order="F" )
      np_gyr_raw = np.array ( object=gyr_value_samples, order="F" )

      # --- Filter Raw Sensor Values As Needed ---

      # Zero out (i.e. discard) low-level noise from accelerometer...
      for axis_index, acc_value in np.ndenumerate(np_acc_raw) :
        np_acc_filtered[axis_index] = 0 if math.fabs(acc_value) <= ACC_NOISE_LEVEL else acc_value

      # Zero out low-level noise from gyro...
      for axis_index, gyr_value in np.ndenumerate(np_gyr_raw) :
        np_gyr_filtered[axis_index] = 0 if math.fabs(gyr_value) <= GYR_NOISE_LEVEL else gyr_value

      # --- Calculate Rotation Amounts ---

      # Calculate incremental rotation amount since last sample...
      np_rot_delta = np_gyr_filtered * time_delta
      # Update cumulative rotation amount since calibration...
    # np_rot_cumul = np_rot_cumul + np_rot_delta
      np_rot_cumul = np.add ( np_rot_cumul, np_rot_delta )

      # Subtract out full rotations; just keep incremental amount from most recent full circle...
      for axis_index, rot_cumul in np.ndenumerate(np_rot_cumul) :
        # Do the modulo operation only when needed to avoid compounding rounding errors...
        np_rot_cumul[axis_index] = rot_cumul % TWO_PI if math.fabs(rot_cumul) > TWO_PI else rot_cumul

      # --- Prepare For Rotating Linear Motion Data ---

      # Calculate cos and sin values of per-axis rotation angles,
      # to rotate back to calibration point...
      for axis_index, rot_value in np.ndenumerate(np_rot_cumul) :
        np_rot_cumul_cos[axis_index] = math.cos(-rot_value)
        np_rot_cumul_sin[axis_index] = math.sin(-rot_value)

      # Create 3D transformation/rotation matrices, one per axis...
      np_rot3d_x = np.array (
        [ [                  1,                      0,                        0 ], 
          [                  0,  np_rot_cumul_cos[0][0], -np_rot_cumul_sin[0][0] ],
          [                  0,  np_rot_cumul_sin[0][0],  np_rot_cumul_cos[0][0] ]
        ]
      )
      np_rot3d_y = np.array (
        [ [  np_rot_cumul_cos[1][0],                  0,  np_rot_cumul_sin[1][0] ],
          [                       0,                  1,                       0 ],
          [ -np_rot_cumul_sin[1][0],                  0,  np_rot_cumul_cos[1][0] ]
        ]
      )
      np_rot3d_z = np.array (
        [ [  np_rot_cumul_cos[2][0], -np_rot_cumul_sin[2][0],                  0 ],
          [  np_rot_cumul_sin[2][0],  np_rot_cumul_cos[2][0],                  0 ],
          [                       0,                       0,                  1 ]
        ]
      )
      # Compute combined (all-axis) 3D rotation matrix...
      np_rot3d_xyz = np_rot3d_x @ np_rot3d_y @ np_rot3d_z

      # --- Rotate Linear Measurements Relative To Calibration Point ---

      # Rotate acceleration vector, so it's relative to device orientation
      # at calibration time...
      np_acc_relcal = np_rot3d_xyz @ np_acc_filtered

      # Subtract g (gravity acceleration) from current (relative to
      # calibration orientation) acceleration...
    # np_acc_no_g = np_acc_relcal - np_g_calib
      np_acc_no_g = np.subtract ( np_acc_relcal, np_g_calib )

      # Calculate current velocity...
      np_vel_delta = np_acc_no_g * time_delta               # change in velocity since last sample
    # np_vel_cumul = np_vel_cumul + np_vel_delta            # accumulated velocity
      np_vel_cumul = np.add ( np_vel_cumul, np_vel_delta )  # accumulated velocity

      # Filter out small junk values (e.g. due to float rounding or 
      # representation errors) from cumulative velocity...
      for axis_index, vel_value in np.ndenumerate(np_vel_cumul) :
        np_vel_cumul[axis_index] = 0 if math.fabs(vel_value) <= VEL_ZERO_THRESH else vel_value

      # Update time state for next loop iteration...
      time_last_sample = time_current

    # --- Track Control State And Perform State Actions ---

    # Perform core finite state machine (FSM) function...
    match control_state_current :
      case ControlState.CALIBRATE_NOT_STABLE :
        if acc_is_only_g(np_acc_filtered) and rot_is_zero(np_gyr_filtered) :
          time_calib_start = time_current
          control_state_next = ControlState.CALIBRATE_STABLE
        else :
          control_state_next = ControlState.CALIBRATE_NOT_STABLE
      case ControlState.CALIBRATE_STABLE :
        if acc_is_only_g(np_acc_filtered) and rot_is_zero(np_gyr_filtered) :
          if time_current - time_calib_start >= CFG_CALIB_WAIT :
            control_state_next = ControlState.CALIBRATING
        else :
          control_state_next = ControlState.CALIBRATE_NOT_STABLE        
      case ControlState.CALIBRATING :
        np_rot_cumul.fill(0)
        np_vel_cumul.fill(0)
      # np_g_calib = np_acc_filtered  # <-- Assignment doesn't copy! It gives a new reference to the right-hand side.
        np_g_calib = np.copy( np_acc_filtered )
        control_state_next = ControlState.READY
      case ControlState.READY :
        if acc_is_only_g(np_acc_filtered) and rot_is_zero(np_gyr_filtered) :
          control_state_next = ControlState.READY
        else :
          control_state_next = ControlState.MOVING
      case ControlState.MOVING :
        if acc_is_only_g(np_acc_filtered) and rot_is_zero(np_gyr_filtered) :
          time_still_start = time_current
          control_state_next = ControlState.STILL
        else :
          control_state_next = ControlState.MOVING
        pass
      case ControlState.STILL :
        if acc_is_only_g(np_acc_filtered) and rot_is_zero(np_gyr_filtered) :
          if time_current - time_still_start >= CFG_STILL_WAIT :
            control_state_next = ControlState.ACTION
          else :
            control_state_next = ControlState.STILL
        else :
          control_state_next = ControlState.MOVING
        pass
      case ControlState.ACTION :
        # Need to perform the actual action here (e.g. trigger camera)...
        # CAUTION: If action has long latency, critical sensor samples
        #          may be dropped.
        time_pause_start = time_current
        control_state_next = ControlState.PAUSE
      case ControlState.PAUSE :
        if time_current - time_pause_start >= CFG_PAUSE_WAIT :
          control_state_next = ControlState.READY
        else :
          control_state_next = ControlState.PAUSE

    # Override next state based on external events (e.g. reset button)...
    if pendingReset :
      control_state_next = ControlState.CALIBRATE_NOT_STABLE
      pendingReset = False
    if pendingCalib :
      control_state_next = ControlState.CALIBRATING
      pendingCalib = False
    if pendingAction :
      control_state_next = ControlState.ACTION
      pendingAction = False

    # If calibrating is next, assume device is still, so don't update
    # sensor values...
    sample_sensor_allowed_by_fsm = control_state_next != ControlState.CALIBRATING

    # --- Provide Observability If So Configured ---

    # Observability via print statements...
    if OBS_PRINT_ANY and ( time_current - time_last_print >= OBS_PRINT_INTERVAL or \
       OBS_PRINT_STATE_CHANGE and control_state_next != control_state_current ) :

      print_count += 1

      if OBS_PRINT_STATE or OBS_PRINT_STATE_CHANGE and control_state_next != control_state_current :
        print ( "Status:" )
        print ( "- Control state        : ", end="" )
        if ( control_state_next == control_state_current ) :
          print ( "%s"  % control_state_current.name )
        else:
          print ( "%s -> %s"  % (control_state_current.name,control_state_next.name) )

      if OBS_PRINT_STATE :
        print ( "- Observability print  : %5d"       % print_count  )
        print ( "- Total sensor samples : %5d (+%d)" % (sample_count_total,sample_count_since_print) )

      if OBS_PRINT_RAW :
        print ( "Raw sensor out:" )
        print ( "- Accelerometer: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"    % acc_value_samples )
        print ( "- Gyroscope    : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian/s" % gyr_value_samples )

      if OBS_PRINT_INST :
        print ( "Instantaneous rotation rate:" )
        print ( "- Filtered     : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian/s" % tuple(np_gyr_filtered.ravel())  )
        print ( "- Is zero      : %s"                                         % rot_is_zero(np_gyr_filtered)    )
        print ( "Instantaneous acceleration:" )
        print ( "- Filtered     : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"     % tuple(np_acc_filtered.ravel())  )
        print ( "- Magnitude    :   %+8.4f"                                    % np.linalg.norm(np_acc_filtered) )
        print ( "- Is only g    : %s"                                          % acc_is_only_g(np_acc_filtered)  )

      if OBS_PRINT_CUMUL :
        print ( "Sensor derived:" )
        print ( "- Rotation:" )
        print ( "  - Cumulative rotation : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian" % tuple(np_rot_cumul.ravel())  )
        print ( "  - Rotation is zero    : %s" % rot_is_zero(np_rot_cumul) )
        print ( "  - Acceleration rel-cal: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"  % tuple(np_acc_relcal.ravel()) )
        print ( "  - Acceleration no-g   : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"  % tuple(np_acc_no_g.ravel())   )
        print ( "- Velocity: " )
        print ( "  - Velocity delta      : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s"    % tuple(np_vel_delta.ravel())  )
        print ( "  - Velocity cumulative : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s"    % tuple(np_vel_cumul.ravel())  )
        print ( "  - Velocity is zero    : %s" % vel_is_zero(np_vel_cumul) )

      print ( "" )
      time_last_print = time_current
      sample_count_since_print = 0

    # Observability via LEDs...
    if OBS_LED :

      led_on [ LedInstance.ROT_NOW   ] = not rot_is_zero(np_gyr_filtered)
      led_on [ LedInstance.ROT_CUMUL ] = not rot_is_zero(np_rot_cumul)
      led_on [ LedInstance.ACC_NOW   ] = not acc_is_only_g(np_acc_filtered)
      led_on [ LedInstance.VEL_CUMUL ] = not vel_is_zero(np_vel_cumul)

      led_on [ LedInstance.STATE_READY  ] = control_state_current == ControlState.READY
      led_on [ LedInstance.STATE_ACTION ] = control_state_current == ControlState.ACTION or control_state_current == ControlState.PAUSE

      for led_instance in LedInstance :
        GPIO.output ( led_pin[led_instance], GPIO.HIGH if led_on[led_instance] else GPIO.LOW )

    # --- Prepare For Next Loop Iteration ---

    # Move to new control state for next loop iteration...
    control_state_current = control_state_next

    # Exit while loop if configured time limit is exceeded...
    if CFG_RUN_TIME_LIMIT > 0 and time_current - time_start >= CFG_RUN_TIME_LIMIT :
      break

except KeyboardInterrupt:
  pass  # just continue on with finally block

finally:
  for led_instance in LedInstance :
    GPIO.output ( led_pin[led_instance], GPIO.LOW )
  print ( "" )
  print ( "Final Info" )
  print ( "----------" )
  print ( "Total sensor samples      : %6d" %  sample_count_total                                )
  print ( "Average samples per second: %6d" % (sample_count_total/(time.monotonic()-time_start)) )
  print ( "Accelerometer:" )
  print ( "- Data Range: %s" % acc_gyr.accelerometer_range     )
  print ( "- Data Rate : %s" % acc_gyr.accelerometer_data_rate )
  print ( "Gyro:" )
  print ( "- Data Range: %s" % acc_gyr.gyro_range     )
  print ( "- Data Rate : %s" % acc_gyr.gyro_data_rate )

# *****************************************************************************
# END OF CODE
# *****************************************************************************

# -----------------------------------------------------------------------------
# Sensor Range and Rate Reference:
# (from /usr/local/lib/python3.13/dist-packages/adafruit_lsm6ds)
# -----------------------------------------------------------------------------
# Default sensing range settings:
# - self.accelerometer_range = AccelRange.RANGE_4G
# - self.gyro_range          = GyroRange.RANGE_250_DPS
# Range options for accelerometer:
# - ("RANGE_2G", 0, 2, 0.061),
# - ("RANGE_4G", 2, 4, 0.122),  <<< default
# - ("RANGE_8G", 3, 8, 0.244),
# - ("RANGE_16G", 1, 16, 0.488),
# Range options for gyroscope:
# - ("RANGE_125_DPS", 125, 125, 4.375),
# - ("RANGE_250_DPS", 0, 250, 8.75),  <<< default
# - ("RANGE_500_DPS", 1, 500, 17.50),
# - ("RANGE_1000_DPS", 2, 1000, 35.0),
# - ("RANGE_2000_DPS", 3, 2000, 70.0),
# -----------------------------------------------------------------------------
# Detault data rate settings:
# - self.accelerometer_data_rate = Rate.RATE_104_HZ
# - self.gyro_data_rate          = Rate.RATE_104_HZ
# Rate options for accelerometer and gyroscope:
# - ("RATE_SHUTDOWN", 0, 0, None),
# - ("RATE_12_5_HZ", 1, 12.5, None),
# - ("RATE_26_HZ", 2, 26.0, None),
# - ("RATE_52_HZ", 3, 52.0, None),
# - ("RATE_104_HZ", 4, 104.0, None),  <<< default
# - ("RATE_208_HZ", 5, 208.0, None),
# - ("RATE_416_HZ", 6, 416.0, None),
# - ("RATE_833_HZ", 7, 833.0, None),
# - ("RATE_1_66K_HZ", 8, 1666.0, None),
# - ("RATE_3_33K_HZ", 9, 3332.0, None),
# - ("RATE_6_66K_HZ", 10, 6664.0, None),
# - ("RATE_1_6_HZ", 11, 1.6, None),
# -----------------------------------------------------------------------------