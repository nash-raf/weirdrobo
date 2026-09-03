# Motion primitives: FORWARD_1_CELL, TURN_LEFT_90, TURN_RIGHT_90, STOP.
#
# Adapted from firmware/motion.py on samiulislam07/autonomous-maze-solving-
# robot @ Samiul, with one deliberate change: every primitive is a coroutine.
# That project's version blocks in sleep_ms(); on this one the web server shares
# the single core with the robot loop, so a blocking move would freeze the
# dashboard for the whole 28 cm.
#
# Nothing above this module knows about PWM or encoder ticks - the navigator
# asks for a cell or a quarter turn and gets one, or is told the move failed.
#
# Two safety rules are built in rather than bolted on:
#   * every primitive is bounded by COMMAND_TIMEOUT_S, so a stalled wheel ends
#     the move instead of driving at duty until the battery dies;
#   * the caller is told when a move ran out of time, so a run fails with a
#     traceable message rather than quietly drifting out of the maze.

import uasyncio as asyncio

import config
from hardware import elapsed_ms, now_ms


class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self._integral = 0.0
        self._previous_error = None

    def reset(self):
        self._integral = 0.0
        self._previous_error = None

    def update(self, error, dt):
        if dt <= 0:
            raise ValueError("dt must be positive")
        self._integral += error * dt
        if self.integral_limit is not None:
            limit = abs(self.integral_limit)
            if self._integral > limit:
                self._integral = limit
            elif self._integral < -limit:
                self._integral = -limit
        derivative = 0.0
        if self._previous_error is not None:
            derivative = (error - self._previous_error) / dt
        self._previous_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


class MotionTimeout(Exception):
    """A primitive hit COMMAND_TIMEOUT_S before reaching its encoder target."""


class TurnsNotCalibrated(Exception):
    """config.COUNTS_PER_90 is 0 - measure WHEELBASE_MM first."""


class Motion:
    """Closed-loop cell moves and quarter turns against encoder targets."""

    def __init__(self, robot):
        self.robot = robot
        self.straight = PID(config.STRAIGHT_KP, config.STRAIGHT_KI, config.STRAIGHT_KD,
                            integral_limit=config.STRAIGHT_I_LIMIT)
        self.last_error = None

    # -- primitives -----------------------------------------------------
    async def forward_one_cell(self):
        """Drive one cell pitch (280 mm), holding both wheels in step."""
        await self._run_to_target(config.COUNTS_PER_CELL, config.COUNTS_PER_CELL,
                                  config.BASE_PWM, straight=True)

    async def turn_left_90(self):
        self._require_turn_calibration()
        await self._run_to_target(-config.COUNTS_PER_90, config.COUNTS_PER_90,
                                  config.TURN_PWM, straight=False)

    async def turn_right_90(self):
        self._require_turn_calibration()
        await self._run_to_target(config.COUNTS_PER_90, -config.COUNTS_PER_90,
                                  config.TURN_PWM, straight=False)

    async def turn_around(self):
        await self.turn_left_90()
        await asyncio.sleep_ms(120)
        await self.turn_left_90()

    def stop(self):
        self.robot.stop()

    def emergency_stop(self):
        self.robot.emergency_stop()

    async def execute(self, action):
        """Dispatch by the action names from maze.action_for_turn()."""
        if action == "forward":
            await self.forward_one_cell()
        elif action == "turn_left":
            await self.turn_left_90()
        elif action == "turn_right":
            await self.turn_right_90()
        elif action == "turn_around":
            await self.turn_around()
        elif action == "stop":
            self.stop()
        else:
            raise ValueError("Unknown action: " + str(action))

    def _require_turn_calibration(self):
        if not config.COUNTS_PER_90:
            raise TurnsNotCalibrated(
                "config.COUNTS_PER_90 is 0 - measure WHEELBASE_MM before turning")

    # -- the loop -------------------------------------------------------
    async def _run_to_target(self, left_target, right_target, speed, straight):
        """Drive until both wheels have turned through their target tick count.

        `straight` adds the differential correction that keeps a forward move
        from curving; a spin turn deliberately runs open loop per wheel, since
        the two wheels are meant to disagree.
        """
        self.robot.reset_encoders()
        self.straight.reset()
        deadline_ms = int(config.COMMAND_TIMEOUT_S * 1000)
        started = now_ms()
        dt = config.CONTROL_PERIOD_S

        left_sign = 1 if left_target >= 0 else -1
        right_sign = 1 if right_target >= 0 else -1
        left_goal = abs(left_target)
        right_goal = abs(right_target)

        while True:
            left_ticks, right_ticks = self.robot.encoder_counts()
            left_done = abs(left_ticks) >= left_goal
            right_done = abs(right_ticks) >= right_goal
            if left_done and right_done:
                break

            if elapsed_ms(started) > deadline_ms:
                self.robot.stop()
                self.last_error = ("motion timed out after "
                                   + str(config.COMMAND_TIMEOUT_S) + "s at left="
                                   + str(left_ticks) + " right=" + str(right_ticks))
                raise MotionTimeout(self.last_error)

            correction = 0.0
            if straight:
                correction = self.straight.update(abs(left_ticks) - abs(right_ticks), dt)

            left_duty = 0.0 if left_done else left_sign * (speed - correction)
            right_duty = 0.0 if right_done else right_sign * (speed + correction)
            self.robot.drive(left_duty, right_duty)
            await asyncio.sleep_ms(int(dt * 1000))

        self.robot.stop()
        await asyncio.sleep_ms(80)      # let the chassis settle before sensing
