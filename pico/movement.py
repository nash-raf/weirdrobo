# One-cell drive and 90 degree turn. All coroutines - they must yield often
# so the web server keeps answering while the robot moves.
import uasyncio as asyncio
import config

BASE_DUTY = 0.55
KP_STRAIGHT = 0.0004        # per count of left/right imbalance
RAMP_DOWN_COUNTS = 2000     # slow down over the last stretch
MIN_DUTY = 0.30


class Movement:
    def __init__(self, drive, encoders):
        self.drive = drive
        self.enc = encoders

    async def forward_cell(self, counts=config.COUNTS_PER_CELL):
        """Drive straight one cell (~280 mm), encoder-balanced, ramped stop."""
        self.enc.reset()
        while True:
            l, r = self.enc.counts()
            done = (abs(l) + abs(r)) / 2
            if done >= counts:
                break
            duty = BASE_DUTY
            remaining = counts - done
            if remaining < RAMP_DOWN_COUNTS:
                duty = MIN_DUTY + (BASE_DUTY - MIN_DUTY) * (remaining / RAMP_DOWN_COUNTS)
            correction = (abs(l) - abs(r)) * KP_STRAIGHT
            self.drive.set(duty - correction, duty + correction)
            await asyncio.sleep_ms(5)
        self.drive.stop()
        await asyncio.sleep_ms(120)     # let it settle before sensing

    async def turn(self, direction):
        """direction: 'left' or 'right'. Spin in place 90 degrees.

        NOT CALIBRATED YET - needs WHEELBASE_MM measured, then
        COUNTS_PER_90 = (pi * WHEELBASE_MM / 4) / MM_PER_COUNT.
        """
        target = config.COUNTS_PER_90
        if not target:
            # Not calibrated: refuse rather than drive an unknown distance.
            self.drive.stop()
            await asyncio.sleep_ms(10)
            return False
        self.enc.reset()
        sign = -1 if direction == "left" else 1
        while self.enc.average() < target:
            self.drive.set(BASE_DUTY * sign, -BASE_DUTY * sign)
            await asyncio.sleep_ms(5)
        self.drive.stop()
        await asyncio.sleep_ms(120)
        return True

    async def face(self, current_heading, target_heading):
        """Turn from current_heading to target_heading, shortest way."""
        from maze import RIGHT_OF, LEFT_OF, BACK_OF
        if current_heading == target_heading:
            return True
        if RIGHT_OF[current_heading] == target_heading:
            return await self.turn("right")
        if LEFT_OF[current_heading] == target_heading:
            return await self.turn("left")
        if BACK_OF[current_heading] == target_heading:
            ok = await self.turn("right")
            return ok and await self.turn("right")
        return False
