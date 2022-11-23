import os

from cereal import car
from common.params import Params
from system.hardware import PC, EON, TICI
from selfdrive.manager.process import PythonProcess, NativeProcess, DaemonProcess

WEBCAM = os.getenv("USE_WEBCAM") is not None

dp_dm = Params().get_bool('dp_dm')

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return dp_dm and params.get_bool("IsDriverViewEnabled")  # type: ignore

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return CP.notCar  # type: ignore

def logging(started, params, CP: car.CarParams) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

procs = [
  # due to qualcomm kernel bugs SIGKILLing camerad sometimes causes page table corruption
  NativeProcess("camerad", "system/camerad", ["./camerad"], unkillable=True, callback=driverview),
  NativeProcess("clocksd", "system/clocksd", ["./clocksd"]),
  # NativeProcess("logcatd", "system/logcatd", ["./logcatd"]),
  # NativeProcess("proclogd", "system/proclogd", ["./proclogd"]),
  # PythonProcess("logmessaged", "system.logmessaged", offroad=True),
  # PythonProcess("timezoned", "system.timezoned", enabled=not PC, offroad=True),

  # DaemonProcess("manage_athenad", "selfdrive.athena.manage_athenad", "AthenadPid"),
  # NativeProcess("dmonitoringmodeld", "selfdrive/modeld", ["./dmonitoringmodeld"], enabled=dp_dm, callback=driverview),
  # NativeProcess("encoderd", "selfdrive/loggerd", ["./encoderd"]),
  # NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"], onroad=False, callback=logging),
  NativeProcess("modeld", "selfdrive/modeld", ["./modeld"]),
  NativeProcess("sensord", "selfdrive/sensord", ["./sensord"], enabled=not PC, offroad=EON, sigkill=EON),
  NativeProcess("ubloxd", "selfdrive/locationd", ["./ubloxd"], enabled=(not PC or WEBCAM)),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], offroad=True, watchdog_max_dt=(5 if (TICI or EON) else None)),
  NativeProcess("soundd", "selfdrive/ui/soundd", ["./soundd"], offroad=True),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"]),
  NativeProcess("boardd", "selfdrive/boardd", ["./boardd"], enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd"),
  PythonProcess("torqued", "selfdrive.locationd.torqued"),
  PythonProcess("controlsd", "selfdrive.controls.controlsd"),
  # PythonProcess("deleter", "selfdrive.loggerd.deleter", offroad=True),
  # PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", enabled=(not PC or WEBCAM), callback=driverview),
  # PythonProcess("laikad", "selfdrive.locationd.laikad"),
  # PythonProcess("rawgpsd", "selfdrive.sensord.rawgps.rawgpsd", enabled=TICI),
  PythonProcess("navd", "selfdrive.navd.navd"),
  PythonProcess("pandad", "selfdrive.boardd.pandad", offroad=True),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd"),
  PythonProcess("pigeond", "selfdrive.sensord.pigeond", enabled=TICI),
  PythonProcess("plannerd", "selfdrive.controls.plannerd"),
  PythonProcess("radard", "selfdrive.controls.radard"),
  PythonProcess("thermald", "selfdrive.thermald.thermald", offroad=True),
  # PythonProcess("tombstoned", "selfdrive.tombstoned", enabled=not PC, offroad=True),
  PythonProcess("updated", "selfdrive.updated", enabled=not PC, onroad=False, offroad=True),
  # PythonProcess("uploader", "selfdrive.loggerd.uploader", offroad=True),
  # PythonProcess("statsd", "selfdrive.statsd", offroad=True),

  # NativeProcess("bridge", "cereal/messaging", ["./bridge"], onroad=False, callback=notcar),
  # PythonProcess("webjoystick", "tools.joystick.web", onroad=False, callback=notcar),

  # EON only
  PythonProcess("rtshield", "selfdrive.rtshield", enabled=EON),
  PythonProcess("shutdownd", "system.hardware.eon.shutdownd", enabled=EON),
  PythonProcess("androidd", "system.hardware.eon.androidd", enabled=EON, offroad=True),

  # dp
  # PythonProcess("dpmonitoringd", "selfdrive.dragonpilot.dpmonitoringd", enabled=not dp_dm),
  PythonProcess("mapd", "selfdrive.mapd.mapd"),
  PythonProcess("systemd", "selfdrive.dragonpilot.systemd", offroad=True),
  PythonProcess("gpxd", "selfdrive.dragonpilot.gpxd"),
  PythonProcess("otisserv", "selfdrive.dragonpilot.otisserv", offroad=True),
  # PythonProcess("loggerd", "selfdrive.dragonpilot.loggerd"),
]

managed_processes = {p.name: p for p in procs}
