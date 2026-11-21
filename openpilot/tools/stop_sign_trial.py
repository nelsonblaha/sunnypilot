"""Read a parked comma's local rlog and replay a candidate stop-sign prompt.

Run from the openpilot environment with one or more local rlog.zst paths.
Times are monotonic seconds. Sign reports are candidates for video review.
The Toyota sign code supplies neither distance nor lane attribution.
"""
import argparse
import json
from pathlib import Path
import time


class Trial:
  def __init__(self):
    self.sign = None
    self.prev = None
    self.hold_start = None
    self.resume = None
    self.sign_frames = 0
    self.rsa_frames = 0
    self.states = 0
    self.stops = 0
    self.events = []

  def sign_report(self, t, code):
    self.rsa_frames += 1
    self.sign_frames += code in (97, 113)
    self.sign = t if code in (97, 113) else None

  def car_state(self, t, stopped, drive, gas, button):
    self.states += 1
    prev = self.prev
    self.prev = (t, stopped, gas)
    if not drive or prev is None or not 0 < t - prev[0] <= 0.1:
      if self.hold_start is not None:
        self.events.append(dict(event="incomplete_hold", t=t))
      self.hold_start = self.resume = None
      if not drive:
        self.prev = None
      return
    if stopped and not prev[1]:
      self.stops += 1
      # Require a recent actual RSA2 frame. Never infer a sign from slowing down.
      if self.sign is not None and 0 <= t - self.sign <= 1.0:
        self.hold_start = t
        self.events.append(dict(event="would_prompt_at_standstill", t=t))
      return
    if self.hold_start is None:
      return
    if self.resume is None and (button or (gas and not prev[2])):
      self.resume = t
      self.events.append(dict(event="fresh_input", input="button" if button else "accelerator", t=t))
    if not stopped:
      order = ("no_recorded_input" if self.resume is None else
               "same_sample" if self.resume == t else "input_before_motion")
      self.events.append(dict(event="motion", ordering=order, t=t))
      self.hold_start = self.resume = None

  def result(self):
    return dict(rsa2_frames=self.rsa_frames, stop_code_frames=self.sign_frames,
                car_state_samples=self.states, standstills=self.stops, events=self.events,
                incomplete_at_end=self.hold_start is not None)


def parked():
  params = Path("/data/params/d")
  try:
    if (params / "IsOffroad").read_bytes() != b"1":
      return False
    try:
      return (params / "IsOnroad").read_bytes() == b"0"
    except FileNotFoundError:
      return True  # Offroad devices may clear the complementary flag.
  except OSError:
    return False


def scan(path, is_parked=parked):
  from openpilot.cereal import log
  if not is_parked():
    raise ValueError("parked comma required")
  import zstandard
  trial = Trial()
  build = None
  data = bytearray()
  with path.open("rb") as f, zstandard.ZstdDecompressor().stream_reader(f) as stream:
    while chunk := stream.read(2 * 1024 * 1024):
      if not is_parked() or len(data) + len(chunk) > 160 * 1024 * 1024:
        raise ValueError("interrupted or oversized replay segment")
      data.extend(chunk)
  for n, event in enumerate(log.Event.read_multiple_bytes(data)):
    if n % 512 == 0 and not is_parked():
      raise ValueError("parked replay interrupted")
    if not event.valid:
      continue
    t = event.logMonoTime * 1e-9
    kind = event.which()
    if kind == "initData":
      build = event.initData.gitCommit
    elif kind == "can":
      for msg in event.can:
        if msg.src == 2 and msg.address == 1162 and len(msg.dat) == 8:
          trial.sign_report(t, msg.dat[0])
    elif kind == "carState":
      cs = event.carState
      button = any(b.pressed and str(b.type) in ("accelCruise", "resumeCruise") for b in cs.buttonEvents)
      trial.car_state(t, cs.standstill, str(cs.gearShifter) == "drive", cs.gasPressed, button)
  if not is_parked():
    raise ValueError("parked replay interrupted")
  return dict(build=build, **trial.result())


def scan_batch(first, end, completed, directory=Path("/data/media/0/realdata")):
  """Scan available full logs in a bounded batch, preferring recent drives."""
  import capnp
  import zstandard
  if not parked():
    return {"parked": False}
  paths = []
  for path in directory.glob("*--*--*/rlog.zst"):
    parts = path.parent.name.split("--")
    if len(parts) != 3 or len(parts[0]) != 8 or path.parent.name in completed:
      continue
    try:
      route, segment = int(parts[0], 16), int(parts[2])
    except ValueError:
      continue
    if first <= route < end:
      paths.append((-route, segment, path))
  results = {}
  started = time.monotonic()
  for _, _, path in sorted(paths):
    if results and (len(results) >= 6 or time.monotonic() - started >= 8):
      break
    if not parked():
      return {"parked": False}
    try:
      results[path.parent.name] = scan(path)
    except (OSError, RuntimeError, ValueError, capnp.KjException, zstandard.ZstdError) as err:
      if not parked():
        return {"parked": False}
      results[path.parent.name] = {"error": type(err).__name__}
  return {"parked": parked(), "reports": results, "exhausted": len(results) == len(paths)}


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("rlogs", nargs="+", type=Path)
  for path in parser.parse_args().rlogs:
    print(json.dumps(dict(segment=path.parent.name, **scan(path))), flush=True)
