# Maze Solver Robot (Pico W) - hardware + geometry constants.
# Everything here is MEASURED on the real robot, not taken from datasheets.

# ---------- maze geometry ----------
GRID = 4                    # 4x4 maze
CELL_MM = 250               # cell interior, mm
WALL_MM = 30                # wall thickness, mm
PITCH_MM = CELL_MM + WALL_MM  # 280 mm centre-to-centre = one cell drive

START = (3, 0)              # top-left  (row 3 = top, col 0 = left)
GOAL = (0, 3)               # bottom-right

# ---------- drivetrain calibration ----------
WHEEL_DIA_MM = 44.0
WHEEL_CIRC_MM = 138.23
CPR = 5887                  # encoder counts per wheel revolution (x4 quadrature, measured)
MM_PER_COUNT = 0.02348
COUNTS_PER_CELL = 11925     # PITCH_MM / MM_PER_COUNT, verified on the floor

# A 90 deg spin turns each wheel through a quarter of the wheelbase circle.
# Set WHEELBASE_MM and COUNTS_PER_90 follows; until then it stays 0 and
# Motion.turn_*() refuses to move rather than driving an unknown distance.
WHEELBASE_MM = 0.0          # TODO: measure wheel centre to wheel centre
_QUARTER_ARC_MM = 3.141592653589793 * WHEELBASE_MM / 4.0
COUNTS_PER_90 = int(_QUARTER_ARC_MM / MM_PER_COUNT) if WHEELBASE_MM else 0

# ---------- GPIO map ----------
PIN_PWMA = 0                # left motor
PIN_AIN1 = 1
PIN_AIN2 = 2
PIN_BIN1 = 3
PIN_BIN2 = 4
PIN_PWMB = 5                # right motor
PIN_STBY = 6

PIN_ENCL_A = 7
PIN_ENCL_B = 8
PIN_ENCR_A = 9
PIN_ENCR_B = 10

PIN_SONAR_L_TRIG = 11
PIN_SONAR_L_ECHO = 12
PIN_SONAR_F_TRIG = 13
PIN_SONAR_F_ECHO = 14
PIN_SONAR_R_TRIG = 15
PIN_SONAR_R_ECHO = 16

PIN_MPU_INT = 17
PIN_I2C0_SDA = 18
PIN_I2C0_SCL = 19

# Set True for a motor whose wiring makes a positive command drive it backwards.
# Confirm with the wheels lifted before any run - and note the upstream firmware
# has these the other way round (LEFT False / RIGHT True). Ours is from the
# build log of THIS chassis; theirs is from the project notes.
LEFT_MOTOR_INVERTED = True
RIGHT_MOTOR_INVERTED = False

# ---------- drive limits ----------
# Duty is clamped to MAX_PWM in both directions, at the motor, so no bug above
# this line can command full voltage into a stalled gearbox.
MAX_PWM = 0.45
BASE_PWM = 0.30             # forward cruise
TURN_PWM = 0.30             # spin turns
PWM_FREQ = 20000            # above hearing, easy on the H-bridge

# Differential PID that keeps a forward move from curving.
STRAIGHT_KP = 0.8
STRAIGHT_KI = 0.0
STRAIGHT_KD = 0.05
STRAIGHT_I_LIMIT = 1.0
CONTROL_PERIOD_S = 0.02

# No motion primitive may run longer than this. A stalled wheel must not mean a
# motor driven at duty until the battery dies.
COMMAND_TIMEOUT_S = 4.0

# ---------- sensing ----------
WALL_THRESHOLD_MM = 150     # sonar reading below this = wall on that side
SENSOR_SAMPLES = 3          # median of N pings, so one dropped echo is not a wall
ULTRASONIC_TIMEOUT_US = 30000       # ~5 m, past the useful range of an HC-SR04
SPEED_OF_SOUND_MM_PER_US = 0.343
NO_ECHO_MM = 4000           # an unreadable sensor reads as open space, not None

# ---------- run mode ----------
# True  = the run loop steps the solver on a timer, motors untouched. Lets the
#         whole stack run on a bare Pico, and is how the dashboard was tested.
# False = every step is a real turn + a real 280 mm drive.
# Leave this True until COUNTS_PER_90 is calibrated: turns raise until then.
SIMULATE = True

# ---------- wifi ----------
WIFI_SSID = "CHANGE_ME"
WIFI_PASS = "CHANGE_ME"
