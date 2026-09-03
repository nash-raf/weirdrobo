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

WHEELBASE_MM = 0.0          # TODO: measure, needed for the 90 deg turn
COUNTS_PER_90 = 0           # TODO: derive from WHEELBASE_MM once measured

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

LEFT_MOTOR_INVERTED = True  # left motor is wired backwards, flipped in software

# ---------- sensing ----------
WALL_THRESHOLD_MM = 150     # sonar reading below this = wall on that side

# ---------- wifi ----------
WIFI_SSID = "CHANGE_ME"
WIFI_PASS = "CHANGE_ME"
