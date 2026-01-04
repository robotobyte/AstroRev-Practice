# *****************************************************************************
# Sensor Processing Code Example for FlatSat Project, 2025
# - Get input from gyroscope and accelerometer
# - Calibrate Earth g
# - Track cumulative rotation
# - Track velocity (as derived from acceleration)
# - Detect stillness and take some action in response
# - Optionally provide observability through prints and/or LEDs
# -----------------------------------------------------------------------------
# December 2025, January 2026
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

# Define Classes (for production code, these should be in separate files/libraries)
# ---------------------------------------------------------------------------------

# Reference on treating class/object members are protected or private:
# https://stackoverflow.com/questions/797771/python-protected-attributes

class ValueOverTime :

  # FIXME: Using lists for data structures. Should use arrays instead!?!

  def __init__ ( self, history_depth, value_element_count=1 ) :

    self.history_depth       = history_depth
    self.value_element_count = value_element_count

    self.reset_history ()
    self.reset_bias    ()

  def reset_history ( self ) :

    self.history_occupancy  = 0
    self.tail_ptr           = self.history_depth
    self.tail_ptr_last      = self.tail_ptr - 1

    self.value_list         = [ [0]*self.value_element_count for i in range(self.history_depth) ]
    self.delta_list         = [ [0]*self.value_element_count for i in range(self.history_depth) ]

    self.value_sum          = [0] * self.value_element_count
    self.delta_sum          = [0] * self.value_element_count
    self.reset_max ()

  def reset_max ( self ) :

    self.delta_max          = [0] * self.value_element_count

  def reset_bias ( self ) :

    self.value_bias         = [0] * self.value_element_count

  def add_value ( self, value ) :

    self.tail_ptr_last = self.tail_ptr
    self.tail_ptr += 1
    if self.tail_ptr >= self.history_depth :
      self.tail_ptr = 0

    value_last = [0] * self.value_element_count
    if self.history_occupancy >= 1 :
      for j in range ( 0, self.value_element_count ) :
        value_last[j] = self.value_list[self.tail_ptr_last][j]
    # at start-up, assume value_last of 0

    if self.history_occupancy >= self.history_depth :
      for j in range ( 0, self.value_element_count ) :
        self.value_sum[j] -= self.value_list[self.tail_ptr][j]
        self.delta_sum[j] -= self.delta_list[self.tail_ptr][j]

    if self.value_element_count == 1 :
      self.value_list[self.tail_ptr][0] = value - self.value_bias[0]
      self.delta_list[self.tail_ptr][0] = value - value_last[0]
    else :
      for j in range ( 0, self.value_element_count ) :
         self.value_list[self.tail_ptr][j] = value[j]
         self.delta_list[self.tail_ptr][j] = value[j] - value_last[j]

    for j in range ( 0, self.value_element_count ) :
      self.value_sum[j] += self.value_list[self.tail_ptr][j]
      self.delta_sum[j] += self.delta_list[self.tail_ptr][j]

    if self.history_occupancy < self.history_depth :
      self.history_occupancy += 1
    if self.history_occupancy > 1 :
      for j in range ( 0, self.value_element_count ) :
        self.delta_max[j] = max ( self.delta_max[j], math.fabs(self.delta_list[self.tail_ptr][j]) )

  def get_value_n ( self, ptr=0 ) :
    value = [0] * self.value_element_count
    for j in range ( 0, self.value_element_count ) :
      value[j] = self.value_list[self.tail_ptr][j] - self.value_bias[j]
    if self.value_element_count == 1 :
      return value[0]
    else :
      return value

  def get_value_0  ( self ) :  # get last value inserted, i.e. value at tail
    return self.get_value_n ( self.tail_ptr )
  def get_value_m1 ( self ) :  # get second to last value inserted (tail-1)
    return self.get_value_n ( self.tail_ptr_last )

  def get_value_midpoint_0m1 (self ) :
    value = [0] * self.value_element_count
    for j in range ( 0, self.value_element_count ) :
      value[j] = ( self.value_list[self.tail_ptr][j] + self.value_list[self.tail_ptr_last][j] ) / 2 - self.value_bias[j]
    if self.value_element_count == 1 :
      return value[0]
    else :
      return value

  def get_value_history_average ( self ) :
    average = [0] * self.value_element_count
    for j in range ( 0, self.value_element_count ) :
      average[j] = self.value_sum[j] / self.history_occupancy - self.value_bias[j]
    if self.value_element_count == 1 :
      return average[0]
    else :
      return average

  def get_delta_history_average ( self ) :

    average = [0] * self.value_element_count
    for j in range ( 0, self.value_element_count ) :
      average[j] = self.delta_sum[j] / self.history_occupancy
    if self.value_element_count == 1 :
      return average[0]
    else :
      return average

  def get_delta_all_max ( self ) :

    if self.value_element_count == 1 :
      return self.delta_max[0]
    else :
      return self.delta_max

  def set_bias ( self, bias ) :

    if self.value_element_count == 1 :
      self.value_bias[0] = bias
    else :
      for j in range ( 0, self.value_element_count ) :
        if type(bias) is list :
          self.value_bias[j] = bias[j]
        else :
          self.value_bias[j] = bias

# Set Configuartion Values and Flags
# ----------------------------------

# Run configuration settings...
CFG_RUN_TIME_LIMIT                    =  -1.0   # max runtime for sensor loop (in seconds; negative = run forever)
CFG_SENSOR_SAMPLE_INTERVAL            =   0.0   # time in seconds; should usually be zero except for debug
CFG_ALLOW_STATE_UPDATE_ONLY_ON_SAMPLE = False   # constrain state machine transitions to occur only when sensors are sampled

CFG_STARTUP_WAIT                      =   1.0   # time to wait after code start-up before looking for stability
CFG_CALIB_STABLE_WAIT                 =   3.0   # time to wait after sensing stability, before calibration
CFG_CALIB_NOT_STABLE_TIMEOUT          =  10.0   # max time to wait for stability before timeout/error
CFG_CALIB_ATTEMPT_LIMIT               =  10     # max attempts through NOT_STABLE/STABLE loop before timeout/error
CFG_STILL_WAIT                        =   1.0   # when still, time to wait before action (in seconds)
CFG_PAUSE_WAIT                        =   1.0   # after action, time to wait before next potential action

CFG_SENSOR_SAMPLE_DEPTH               =  128    # depth of sensor sample history array (for bias, noise assessment, etc.)
CFG_INTERPOLATE_LAST_SAMPLE_PAIR      = True    # enable/disable whether to use only last sample or average of last two for integrations

CFG_ACC_VALUE_NOISE_LEVEL_INIT        = 0.01    # Zero out values less than or equal to this value, assuming they're noise; applied per axis
CFG_ACC_DELTA_NOISE_LEVEL_INIT        = 0.01    # Absolute allowance for noise on top of measured sensor value jiggle
CFG_ACC_NOISE_LEVEL_REL_MARGIN        = 0.03    # Additional relative noise margin on top of absolute value
CFG_ACC_EARTH_G_REL_MARGIN            = 0.05    # Accept 5% of relative measurement inaccuracy (applied to comparison with g)

CFG_GYR_VALUE_NOISE_LEVEL_INIT        = 0.025   # Zero out values less than or equal to this value, assuming they're noise; applied per axis
CFG_GYR_DELTA_NOISE_LEVEL_INIT        = 0.01    # Absolute allowance for noise on top of measured sensor value jiggle
CFG_GYR_NOISE_LEVEL_REL_MARGIN        = 0.03    # Additional relative noise margin on top of absolute value

CFG_VEL_ZERO_THRESH                   = 0.1     # Zero out values less than or equal to this value, assuming they're rounding or accumulation errors

CFG_SWITCH_DEBOUNCE_TIME              = 200     # Time allowance for mechanical switch debounce

# Observability settings via print to terminal...
OBS_PRINT_ANY          = True   # enable/disable printing of any observability messages
OBS_PRINT_STATE        = True   # enable/disable printing of control state and sample counts
OBS_PRINT_STATE_CHANGE = True   # enable/disable print on state change regardless of PRINT_STATE or PRINT_INTERVAL
OBS_PRINT_RAW          = True   # enable/disable printing of raw sensor values
OBS_PRINT_ADJUSTED     = True   # enable/disable printing of possibly adjusted (e.g. interpolated) sensor values
OBS_PRINT_OVER_TIME    = True   # enable/disable printing of sample history derived values
OBS_PRINT_FILTERED     = True   # enable/disable printing of instantaneous values
OBS_PRINT_CUMUL        = True   # enable/disable printing of cumululative value
OBS_PRINT_INTERVAL     = 0.1    # print interval time in seconds (negative = print always)

# Obesrvability via GPIO-connected LEDs...
OBS_LED = True

# Define Exception Classes
# ------------------------

class CalibrationFailure ( BaseException ) :
  pass

# Define and Set Up GPIO Pins...
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
sensor_acc_gyr = LSM6DSOX(i2c)

# Increase sample rate from default of 104Hz to better match
# run rate of this code on a RPi 4, which gets about 400 to
# 500 main loop iterations per second...
sensor_acc_gyr.accelerometer_data_rate = Rate.RATE_833_HZ
sensor_acc_gyr.gyro_data_rate          = Rate.RATE_833_HZ
# For reference, see also bottom of this file and this link:
# https://docs.circuitpython.org/projects/lsm6dsox/en/latest/examples.html#rate-test

# Define Useful Constants
# -----------------------

ACC_EARTH_G = 9.80665      # Earth's acceleration of gravity, 9.8 m/s^2 (https://en.wikipedia.org/wiki/Gravity_of_Earth)
TWO_PI      = 2 * math.pi  # 2*pi precalculated for convenience

# Define Enumerated Type for Control State Machine
# ------------------------------------------------

class ControlState(Enum):
  STARTUP_WAIT         =  0  # wait time after power-up/start-up to start looking for stability
  CALIBRATE_NOT_STABLE =  1  # device not yet stable for calibration
  CALIBRATE_PREPARE    =  2  # prepare for calibration (assuming sufficient stability time in CALIBRATE_STABLE)
  CALIBRATE_STABLE     =  3  # device stable, waiting for timer before calibrating
  CALIBRATING          =  4  # calibrating
  CALIBRATION_ERROR    =  5  # calibration failed (too many NOT_STABLE/STABLE loops or wait-for-stable timeout)
  READY                =  6  # ready for motion (to then take an action when the motion ceases)
  MOVING               =  7  # motion detected
  STILL                =  8  # no motion detected, waiting for timer before action
  ACTION               =  9  # take action (e.g. take photo)
  PAUSE                = 10  # take post-action pause (why? why not! take a break... smell the roses...)

# Define Helper Functions
# -----------------------

def list_elements_are_zero ( element_list, zero_margin=0 ) :
  all_zero = True
  if type(zero_margin) is not list :
    zero_margin = [zero_margin] * len(element_list)
  for element, margin in zip ( element_list, zero_margin ) :
    all_zero = all_zero and ( math.fabs(element) <= margin )
  return all_zero

def np_elements_are_zero ( np_v3, zero_margin=0 ) :
  all_zero = True
  if type(zero_margin) is not list :
    zero_margin = [zero_margin] * np_v3.size
  for element, margin in zip ( np_v3.ravel(), zero_margin ) :
    all_zero = all_zero and ( math.fabs(element) <= margin )
  return all_zero

def np_magnitude_is_zero ( np_v3, zero_margin=0 ) :
  magnitude = np.linalg.norm(np_v3)
  return magnitude <= zero_margin

def acc_is_only_g ( np_acc, g_rel_margin=CFG_ACC_EARTH_G_REL_MARGIN ) :
  acc_magnitude = np.linalg.norm(np_acc)
  acc_rel_earth_g = math.fabs ( acc_magnitude - ACC_EARTH_G ) / ACC_EARTH_G
  return acc_rel_earth_g <= g_rel_margin

def acc_offset_from_g ( acc_values ) :
  g_measured = 0
  for axis_value in acc_values :
    g_measured += axis_value ** 2
  g_measured = math.sqrt(g_measured)
  g_to_mag_delta_rel = ACC_EARTH_G / g_measured
  offset = [0] * 3
  for axis_index in range ( 0, 3 ) :
    offset[axis_index] = acc_values[axis_index] * ( 1 - g_to_mag_delta_rel )
  return offset

def motion_detected ( np_acc, np_vel, np_gyr, gyr_noise_level ):
  no_acc = acc_is_only_g ( np_acc )
  no_vel = np_magnitude_is_zero ( np_vel )
  no_rot = np_elements_are_zero ( np_gyr, gyr_noise_level )
  return not ( no_acc and no_vel and no_rot )

# Define Event Handler Functions
# ------------------------------

def handleReset () :
  pendingReset = True

def handleCalib () :
  pendingCalib = True

def handleAction () :
  pendingAction = True

# Attach event handlers to input buttons...
#GPIO.add_event_detect( button_pin [ RESET       ], GPIO.FALLING, handleReset,  CFG_SWITCH_DEBOUNCE_TIME )
#GPIO.add_event_detect( button_pin [ FORCE_CALIB ], GPIO.FALLING, handleCalib,  CFG_SWITCH_DEBOUNCE_TIME )
#GPIO.add_event_detect( button_pin [ FORCE_ACTION], GPIO.FALLING, handleAction, CFG_SWITCH_DEBOUNCE_TIME )
# FIXME: Need to debug "Failed to add edge detection" error.
# FIXME: Library's debounce mechanisms is not good; see https://raspberrypi.stackexchange.com/questions/76667/debouncing-buttons-with-rpi-gpio-too-many-events-detected

# Sensor Sample Filter Objects and Parameters
# -------------------------------------------

acc_values_over_time = ValueOverTime ( CFG_SENSOR_SAMPLE_DEPTH, 3 )
gyr_values_over_time = ValueOverTime ( CFG_SENSOR_SAMPLE_DEPTH, 3 )

acc_value_noise_level = [ CFG_ACC_VALUE_NOISE_LEVEL_INIT ] * 3
acc_delta_noise_level = [ CFG_ACC_DELTA_NOISE_LEVEL_INIT ] * 3
gyr_value_noise_level = [ CFG_GYR_VALUE_NOISE_LEVEL_INIT ] * 3
gyr_delta_noise_level = [ CFG_GYR_DELTA_NOISE_LEVEL_INIT ] * 3

# Allocate Numpy Matrices
# -----------------------

np_acc_filtered2 = np.zeros ( (3,1) )
np_gyr_filtered2 = np.zeros ( (3,1) )

np_acc_last = np.zeros ( (3,1) )
np_gyr_last = np.zeros ( (3,1) )

np_rot_cumul = np.zeros( (3,1) )
np_vel_cumul = np.zeros( (3,1) )

np_rot_cumul_cos = np.zeros ( (3,1) )
np_rot_cumul_sin = np.zeros ( (3,1) )

np_g_calib = np.zeros( (3,1) )

np_acc_no_g_filtered = np.zeros( (3,1) )

# Initialize Loop Variables
# -------------------------

# Initialize time trackers...
time_start       = time.monotonic()  # system time in seconds with fractional part (float)
time_last_sample = time_start - CFG_SENSOR_SAMPLE_INTERVAL
time_last_print  = time_start - OBS_PRINT_INTERVAL

# Initialize state control and tracking variables...
control_state_current        = ControlState.STARTUP_WAIT  # start-up state for state machine
sample_sensor_allowed_by_fsm = True                       # flag whether state machine wants sensor to be sampled
sample_sensor_last           = False                      # history of whether sensors were sampled in last loop iteration
interpolate_last_sample_pair = False                      # setting for how to feed sensor values into further calculations
# This interpolate_last_sample_pair controls how to use sensor samples
# through calibration. For the main processing (e.g. motion tracking) loop,
# interpolate_last_sample_pair will be updated based on the setting of
# CFG_INTERPOLATE_LAST_SAMPLE_PAIR. If False, only the last sensor sample
# is used for incremental integration (i.e. change over time given the
# time delta since the last sensor sample), and if True, the average of
# the last two sensor samples is used.

# Initialize sensor sample and observability print counters...
sample_count_total       = 0
sample_count_since_print = 0
print_count              = 0

# Initialize pending-event flags...
pendingReset  = False
pendingCalib  = False
pendingAction = False

# Run Main Loop
# -------------

try :
  while True :

    # Capture current time for current iteration of the loop...
    time_current = time.monotonic()

    # Determine whether to sample sensor(s) given configured sampling
    # inverval (this interval should be zero except for debug)...
    sample_sensor_allowed_by_time = time_current - time_last_sample > CFG_SENSOR_SAMPLE_INTERVAL

    # --- Obtain Values From Sensors ---

    # Sample sensors and update calculations...
    sample_sensor_current = sample_sensor_allowed_by_time and sample_sensor_allowed_by_fsm
    if sample_sensor_current :

      # Read sensors...
      acc_values_sampled = sensor_acc_gyr.acceleration  # three-axis accelerometer; radian/sec
      gyr_values_sampled = sensor_acc_gyr.gyro          # three-axis gyroscope; meter/sec^2
      sample_count_total       += 1
      sample_count_since_print += 1

      # Compute elapsed time since last sample...
      time_delta = time_current - time_last_sample

      # Add new sensor values to history tracker...
      acc_values_over_time.add_value ( acc_values_sampled )
      gyr_values_over_time.add_value ( gyr_values_sampled )

      # For subsequent calculations (e.g. integrating accelerating into
      # velocity), choose whetehr to use just the most recent sensor
      # samples or interpolate between the last two sensor samples...
      if interpolate_last_sample_pair :
        acc_values_adjusted = acc_values_over_time.get_value_midpoint_0m1()
        gyr_values_adjusted = gyr_values_over_time.get_value_midpoint_0m1()
      else :
        acc_values_adjusted = acc_values_over_time.get_value_0()
        gyr_values_adjusted = gyr_values_over_time.get_value_0()

      # Convert tuples from sensors into column matrices...
      np_acc_filtered1 = np.array ( [ acc_values_adjusted ] ).T
      np_gyr_filtered1 = np.array ( [ gyr_values_adjusted ] ).T

      # --- Filter Raw Sensor Values As Needed ---

      # Zero out (i.e. discard) low-level noise from accelerometer...
      for (axis_index,acc_value), noise_level in zip ( np.ndenumerate(np_acc_filtered1), acc_value_noise_level ) :
        np_acc_filtered2[axis_index] = 0 if math.fabs(acc_value) <= noise_level else acc_value

      # Zero out low-level noise from gyro...
      for (axis_index,gyr_value), noise_level in zip ( np.ndenumerate(np_gyr_filtered1), gyr_value_noise_level ) :
        np_gyr_filtered2[axis_index] = 0 if math.fabs(gyr_value) <= noise_level else gyr_value

      # --- Calculate Rotation Amounts ---

      # Calculate incremental rotation amount since last sample...
      np_rot_delta = np_gyr_filtered2 * time_delta
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
        [ [                       1,                      0,                        0 ],
          [                       0,  np_rot_cumul_cos[0][0], -np_rot_cumul_sin[0][0] ],
          [                       0,  np_rot_cumul_sin[0][0],  np_rot_cumul_cos[0][0] ]
        ]
      )
      np_rot3d_y = np.array (
        [ [  np_rot_cumul_cos[1][0],                       0,  np_rot_cumul_sin[1][0] ],
          [                       0,                       1,                       0 ],
          [ -np_rot_cumul_sin[1][0],                       0,  np_rot_cumul_cos[1][0] ]
        ]
      )
      np_rot3d_z = np.array (
        [ [  np_rot_cumul_cos[2][0], -np_rot_cumul_sin[2][0],                       0 ],
          [  np_rot_cumul_sin[2][0],  np_rot_cumul_cos[2][0],                       0 ],
          [                       0,                       0,                       1 ]
        ]
      )
      # Compute combined (all-axis) 3D rotation matrix...
      np_rot3d_xyz = np_rot3d_x @ np_rot3d_y @ np_rot3d_z

      # --- Rotate Linear Measurements Relative To Calibration Point ---

      # Rotate acceleration vector, so it's relative to device orientation
      # at calibration time...
      np_acc_relcal = np_rot3d_xyz @ np_acc_filtered2

      # Subtract g (gravity acceleration) from current (relative to
      # calibration orientation) acceleration...
    # np_acc_no_g = np_acc_relcal - np_g_calib
      np_acc_no_g = np.subtract ( np_acc_relcal, np_g_calib )

      # Zero out low-level noise from net acceleration...
      for (axis_index,acc_value), noise_level in zip ( np.ndenumerate(np_acc_no_g), acc_value_noise_level ) :
        np_acc_no_g_filtered[axis_index] = 0 if math.fabs(acc_value) <= noise_level else acc_value

      # Calculate current velocity...
      np_vel_delta = np_acc_no_g_filtered * time_delta               # change in velocity since last sample
    # np_vel_cumul = np_vel_cumul + np_vel_delta            # accumulated velocity
      np_vel_cumul = np.add ( np_vel_cumul, np_vel_delta )  # accumulated velocity

      # Filter out small junk values (e.g. due to float rounding or 
      # representation errors) from cumulative velocity...
      for axis_index, vel_value in np.ndenumerate(np_vel_cumul) :
        np_vel_cumul[axis_index] = 0 if math.fabs(vel_value) <= CFG_VEL_ZERO_THRESH else vel_value

      # Update time state for next loop iteration...
      sample_sensor_last = sample_sensor_current
      time_last_sample = time_current

    # --- Track Control State And Perform State Actions ---

    # Perform core finite state machine (FSM) function...
    match control_state_current :

      case ControlState.STARTUP_WAIT :
        calib_attempt_count = 0
        if time_current - time_start >= CFG_STARTUP_WAIT :
          acc_values_over_time.reset_max ()
          gyr_values_over_time.reset_max ()
          time_calib_not_stable_start = time_current
          control_state_next = ControlState.CALIBRATE_NOT_STABLE
        else :
          control_state_next = ControlState.STARTUP_WAIT

      case ControlState.CALIBRATE_NOT_STABLE :
        if acc_is_only_g(np_acc_filtered2) and np_elements_are_zero(np_gyr_filtered2,gyr_value_noise_level) and \
           list_elements_are_zero(acc_values_over_time.get_delta_history_average(),acc_delta_noise_level) and \
           list_elements_are_zero(gyr_values_over_time.get_delta_history_average(),gyr_delta_noise_level) :
          calib_attempt_count += 1
          control_state_next = ControlState.CALIBRATE_PREPARE
        elif time_current - time_calib_not_stable_start >= CFG_CALIB_NOT_STABLE_TIMEOUT :
          control_state_next = ControlState.CALIBRATION_ERROR
        else :
          control_state_next = ControlState.CALIBRATE_NOT_STABLE

      case ControlState.CALIBRATE_PREPARE :
        acc_values_over_time.reset_max ()
        gyr_values_over_time.reset_max ()
        time_calib_stable_start = time_current
        control_state_next = ControlState.CALIBRATE_STABLE

      case ControlState.CALIBRATE_STABLE :
        if calib_attempt_count > CFG_CALIB_ATTEMPT_LIMIT :
          control_state_next = ControlState.CALIBRATION_ERROR
        elif acc_is_only_g(np_acc_filtered2) and np_elements_are_zero(np_gyr_filtered2,gyr_value_noise_level) and \
             list_elements_are_zero(acc_values_over_time.get_delta_history_average(),acc_delta_noise_level) and \
             list_elements_are_zero(gyr_values_over_time.get_delta_history_average(),gyr_delta_noise_level) :
          if time_current - time_calib_stable_start >= CFG_CALIB_STABLE_WAIT :
            control_state_next = ControlState.CALIBRATING
          else:
            control_state_next = ControlState.CALIBRATE_STABLE
        else :
          time_calib_not_stable_start = time_current
          control_state_next = ControlState.CALIBRATE_NOT_STABLE

      case ControlState.CALIBRATING :
        np_rot_cumul.fill ( 0 )
        np_vel_cumul.fill ( 0 )
        acc_values_over_time.set_bias ( acc_offset_from_g ( acc_values_over_time.get_value_history_average() ) )
        acc_value_noise_level = [ av * (1+CFG_ACC_NOISE_LEVEL_REL_MARGIN) for av in acc_values_over_time.get_delta_all_max() ]
        gyr_values_over_time.set_bias ( gyr_values_over_time.get_value_history_average() )
        gyr_value_noise_level = [ gv * (1+CFG_GYR_NOISE_LEVEL_REL_MARGIN) for gv in gyr_values_over_time.get_delta_all_max() ]
        np_g_calib = np.array ( [ acc_values_over_time.get_value_history_average() ] ).T
        interpolate_last_sample_pair = CFG_INTERPOLATE_LAST_SAMPLE_PAIR
        control_state_next = ControlState.READY

      case ControlState.CALIBRATION_ERROR :
        raise CalibrationFailure ( "Calibration Failed" )

      case ControlState.READY :
        if motion_detected ( np_acc_filtered2, np_vel_cumul, np_gyr_filtered2, gyr_value_noise_level ) :
          control_state_next = ControlState.MOVING
        else :
          control_state_next = ControlState.READY

      case ControlState.MOVING :
        if motion_detected ( np_acc_filtered2, np_vel_cumul, np_gyr_filtered2, gyr_value_noise_level ) :
          control_state_next = ControlState.MOVING
        else :
          time_still_start = time_current
          control_state_next = ControlState.STILL

      case ControlState.STILL :
        if motion_detected ( np_acc_filtered2, np_vel_cumul, np_gyr_filtered2, gyr_value_noise_level ) :
          control_state_next = ControlState.MOVING
        elif time_current - time_still_start >= CFG_STILL_WAIT :
          control_state_next = ControlState.ACTION
        else :
          control_state_next = ControlState.STILL

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
      control_state_next = ControlState.STARTUP_WAIT
      pendingReset = False
    if pendingCalib :
      control_state_next = ControlState.CALIBRATING
      pendingCalib = False
    if pendingAction :
      control_state_next = ControlState.ACTION
      pendingAction = False

    # If calibrating is next, assume device is still, so don't update
    # sensor values...
    sample_sensor_allowed_by_fsm = control_state_next != ControlState.CALIBRATING and \
                                   control_state_next != ControlState.CALIBRATION_ERROR

    # --- Provide Observability If So Configured ---

    # Observability via print statements...
    obs_print_now = OBS_PRINT_ANY and ( \
                      time_current - time_last_print >= OBS_PRINT_INTERVAL and sample_sensor_current or \
                      OBS_PRINT_STATE_CHANGE and control_state_next != control_state_current \
                    )
    if obs_print_now :

      print_count += 1

      if OBS_PRINT_STATE or OBS_PRINT_STATE_CHANGE and control_state_next != control_state_current :
        print ( "Status:" )
        print ( "- Control state        : ", end="" )
        if ( control_state_next == control_state_current ) :
          print ( "%s"  % control_state_current.name )
        else:
          print ( "%s -> %s"  % (control_state_current.name,control_state_next.name) )

      if OBS_PRINT_STATE :
        print ( "- Observability print  : %5d"       % print_count )
        print ( "- Total sensor samples : %5d (+%d)" % (sample_count_total,sample_count_since_print) )

      if OBS_PRINT_RAW :
        print ( "Raw sensor readings:" )
        print ( "- Accelerometer: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"    % acc_values_sampled )
        print ( "- Gyroscope    : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian/s" % gyr_values_sampled )
      if OBS_PRINT_ADJUSTED:
        print ( "Adjusted (possibly interpolated) sensor readings:" )
        print ( "- Accelerometer: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"    % tuple(acc_values_adjusted) )
        print ( "- Gyroscope    : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian/s" % tuple(gyr_values_adjusted) )

      if OBS_PRINT_OVER_TIME :
        print ( "Raw readings over time:" )
        print ( "- Acc value avg: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"    % tuple(acc_values_over_time.get_value_history_average()) )
        print ( "- Acc delta avg: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"    % tuple(acc_values_over_time.get_delta_history_average()) )
        print ( "- Acc delta max: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"    % tuple(acc_values_over_time.get_delta_all_max        ()) )
        print ( "- Gyr value avg: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"    % tuple(gyr_values_over_time.get_value_history_average()) )
        print ( "- Gyr delta avg: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"    % tuple(gyr_values_over_time.get_delta_history_average()) )
        print ( "- Gyr delta max: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"    % tuple(gyr_values_over_time.get_delta_all_max        ()) )

      if OBS_PRINT_FILTERED :
        print ( "Instantaneous acceleration:" )
        print ( "- Filtered 1   : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"     % tuple(np_acc_filtered1.ravel())                             )
        print ( "- Filtered 2   : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"     % tuple(np_acc_filtered2.ravel())                             )
        print ( "- Magnitude    :   %+8.4f"                                    % np.linalg.norm(np_acc_filtered2)                            )
        print ( "- Is only g    : %s"                                          % acc_is_only_g(np_acc_filtered2)                             )
        print ( "Instantaneous rotation rate:" )
        print ( "- Filtered 1   : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian/s" % tuple(np_gyr_filtered1.ravel())                              )
        print ( "- Filtered 2   : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian/s" % tuple(np_gyr_filtered2.ravel())                              )
        print ( "- Is zero      : %s"                                         % np_elements_are_zero(np_gyr_filtered2,gyr_value_noise_level) )

      if OBS_PRINT_CUMUL :
        print ( "Sensor derived:" )
        print ( "- Rotation:" )
        print ( "  - Cumulative rotation         : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian" % tuple(np_rot_cumul.ravel())         )
        print ( "  - Rotation is zero            : %s" % np_elements_are_zero(np_rot_cumul,gyr_value_noise_level)                  )
        print ( "- Adjusted Acceleration:" )
        print ( "  - Acceleration rel calib point: X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"  % tuple(np_acc_relcal.ravel())        )
        print ( "  - Acceleration no-g           : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"  % tuple(np_acc_no_g.ravel())          )
        print ( "  - Acceleration no-g filtered  : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2"  % tuple(np_acc_no_g_filtered.ravel()) )
        print ( "  - Acceleration no-g is zero   : %s" % np_elements_are_zero(np_acc_no_g_filtered)                                )
        print ( "- Velocity: " )
        print ( "  - Velocity delta              : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s"    % tuple(np_vel_delta.ravel())         )
        print ( "  - Velocity cumulative         : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s"    % tuple(np_vel_cumul.ravel())         )
        print ( "  - Velocity is zero            : %s" % np_magnitude_is_zero(np_vel_cumul)                                        )

      print ( "" )
      time_last_print = time_current
      sample_count_since_print = 0

    # Observability via LEDs...
    if OBS_LED :

      # Compute states for sensor indicator LEDs...
      led_on [ LedInstance.ROT_NOW   ] = not np_elements_are_zero(np_gyr_filtered2,gyr_value_noise_level)
      led_on [ LedInstance.ROT_CUMUL ] = not np_elements_are_zero(np_rot_cumul)
      led_on [ LedInstance.ACC_NOW   ] = not acc_is_only_g(np_acc_filtered2)
      led_on [ LedInstance.VEL_CUMUL ] = not np_magnitude_is_zero(np_vel_cumul)

      # Compute states for state machine indicator LEDs...
      led_on [ LedInstance.STATE_READY  ] = control_state_current == ControlState.READY
      led_on [ LedInstance.STATE_ACTION ] = control_state_current == ControlState.ACTION or control_state_current == ControlState.PAUSE

      # Drive GPIO pins according to computed states...
      for led_instance in LedInstance :
        GPIO.output ( led_pin[led_instance], GPIO.HIGH if led_on[led_instance] else GPIO.LOW )

    # --- Prepare For Next Loop Iteration ---

    # Move to new control state for next loop iteration...
    control_state_current = control_state_next

    # Exit while loop if configured time limit is exceeded...
    if CFG_RUN_TIME_LIMIT > 0 and time_current - time_start >= CFG_RUN_TIME_LIMIT :
      break

# Handle Exceptions
# -----------------

except KeyboardInterrupt:
  pass  # just continue on with finally block

except CalibrationFailure:
  pass  # just continue on with finally block

# Finish Up
# ---------

finally:
  for led_instance in LedInstance :
    GPIO.output ( led_pin[led_instance], GPIO.LOW )
  print ( "" )
  print ( "Final Info" )
  print ( "----------" )
  print ( "Total sensor samples      : %6d" %  sample_count_total                                )
  print ( "Average samples per second: %6d" % (sample_count_total/(time.monotonic()-time_start)) )
  print ( "Accelerometer:" )
  print ( "- Data Range Index: %s" % sensor_acc_gyr.accelerometer_range     )
  print ( "- Data Rate  Index: %s" % sensor_acc_gyr.accelerometer_data_rate )
  print ( "- Value Bias      : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2" % tuple(acc_values_over_time.value_bias) )
  print ( "- Noise Level     : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  m/s^2" % tuple(acc_value_noise_level)           )
  print ( "Gyro:" )
  print ( "- Data Range Index: %s" % sensor_acc_gyr.gyro_range     )
  print ( "- Data Rate  Index: %s" % sensor_acc_gyr.gyro_data_rate )
  print ( "- Value Bias      : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian/s" % tuple(gyr_values_over_time.value_bias) )
  print ( "- Noise Level     : X:%+8.4f,  Y: %+8.4f,  Z: %+8.4f  radian/s" % tuple(gyr_value_noise_level)           )

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