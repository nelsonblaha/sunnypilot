"""Stop-and-wait hold (nelsonblaha/sunnypilot fork, branch stop-and-wait).

Once openpilot longitudinal has brought the car to a standstill with no lead car, keep it
stopped until the driver asks to go:
  - gas pedal, or
  - RES/+ on the cruise stalk. Toyota TSS2 exposes no RES/+ button event, so this is
    detected as a rise in the PCM set speed, or the PCM leaving its own standstill state.

It never adds acceleration and never makes the car stop. It only keeps a stopped car stopped.
A stop behind a lead car is not held, so stop-and-go traffic behaves as before.

Kill switch: create /data/stop_and_wait_disabled on the device and reboot.
"""
import os

DISABLE_FILE = "/data/stop_and_wait_disabled"
MOVING_SPEED = 1.0        # m/s (~2.2 mph). Above this the car is driving again: drop any hold, re-arm for the next stop.
SET_SPEED_RISE = 0.2      # m/s. PCM set speed moves in whole km/h (0.28 m/s), so any RES/+ step clears this.
LEAD_CLEAR_FRAMES = 10    # planner frames (20 Hz => 0.5 s) with no lead before a stop can be held; ignores lead flicker.


class StandstillHold:
  def __init__(self, enabled=None, log=None):
    if enabled is None:
      enabled = not os.path.exists(DISABLE_FILE)
    self.enabled = enabled
    self.log = log
    self.holding = False
    self.released = False          # the driver released this stop; do not re-arm until the car has moved
    self.min_set_speed = 0.0
    self.prev_pcm_standstill = False
    self.no_lead_frames = 0

  def _event(self, state, **kwargs):
    if self.log is not None:
      try:
        self.log("stop_and_wait", state=state, **kwargs)
      except Exception:
        pass

  def update(self, sm, active):
    """Returns True while the hold is keeping the car stopped. `active` = openpilot longitudinal is engaged."""
    if not self.enabled:
      return False

    cs = sm['carState']
    set_speed = cs.cruiseState.speed
    pcm_standstill = cs.cruiseState.standstill
    pcm_resumed = self.prev_pcm_standstill and not pcm_standstill
    self.prev_pcm_standstill = pcm_standstill

    if sm['radarState'].leadOne.present:
      self.no_lead_frames = 0
    else:
      self.no_lead_frames += 1

    # Driving again: nothing to hold, and the next stop may be held.
    if cs.vEgo > MOVING_SPEED:
      if self.holding:
        self._event("release", reason="moving")
      self.holding = False
      self.released = False
      return False

    # Gas at (or near) a stop means "go", whether or not a hold is active yet.
    if cs.gasPressed:
      if self.holding:
        self._event("release", reason="gas")
      self.holding = False
      self.released = True
      return False

    if self.holding:
      reason = None
      if set_speed > self.min_set_speed + SET_SPEED_RISE:
        reason = "set_speed_up"
      elif pcm_resumed:
        reason = "pcm_resume"
      elif not active:
        reason = "disengaged"

      if reason is None:
        self.min_set_speed = min(self.min_set_speed, set_speed)
        return True

      self._event("release", reason=reason)
      self.holding = False
      # A disengage (brake, cancel) is not a request to go: re-engaging at the same stop holds again.
      self.released = reason != "disengaged"
      return False

    if active and cs.standstill and not self.released and self.no_lead_frames >= LEAD_CLEAR_FRAMES:
      self.holding = True
      self.min_set_speed = set_speed
      self._event("hold", set_speed=float(set_speed))
      return True

    return False
