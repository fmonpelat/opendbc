from opendbc.can import CANParser
from opendbc.car import Bus, CanBusBase, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.gwm.values import DBC

# The front radar is not on any bus the harness can see: it talks to the camera on a private CAN.
# The camera publishes the fused ACC target (distance, absolute speed, status) in 0x2B8 on the camera bus.
LEAD_MSG = "UNDEFINED_INTERESTING_SIGNALS"  # 0x2B8, 10 Hz
LEAD_MSG_ADDR = 0x2B8
LEAD_STATE_MOVING = 1
LEAD_STATE_STOPPED = 3
VALID_LEAD_STATES = (LEAD_STATE_MOVING, LEAD_STATE_STOPPED)  # 2 is an unconfirmed candidate, 0 is no target


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.updated_messages = set()
    self.trigger_msg = LEAD_MSG_ADDR
    self.lead_present = False

    if CP.radarUnavailable:
      self.rcp = None
      self.rcp_speed = None
    else:
      can_bus = CanBusBase(CP, None)
      self.rcp = CANParser(DBC[CP.carFingerprint][Bus.pt], [(LEAD_MSG, 10)], can_bus.offset + 2)
      # vRel must be relative to the same vEgo radard uses (carState.vEgo), so use the same wheel speeds and factor
      self.rcp_speed = CANParser(DBC[CP.carFingerprint][Bus.pt], [("WHEEL_SPEEDS", 50)], can_bus.offset)

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    self.rcp_speed.update(can_strings)
    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None
    self.updated_messages.clear()

    ret = structs.RadarData()
    if not self.rcp.can_valid or not self.rcp_speed.can_valid:
      ret.errors.canError = True
      return ret

    msg = self.rcp.vl[LEAD_MSG]
    if msg["LEAD_STATE"] in VALID_LEAD_STATES:
      if not self.lead_present:
        # a new target gets a new track id, so radard never merges two different cars
        self.track_id += 1
        self.lead_present = True

      ws = self.rcp_speed.vl["WHEEL_SPEEDS"]
      v_ego = (ws["FRONT_LEFT_WHEEL_SPEED"] + ws["FRONT_RIGHT_WHEEL_SPEED"] +
               ws["REAR_LEFT_WHEEL_SPEED"] + ws["REAR_RIGHT_WHEEL_SPEED"]) / 4 * CV.KPH_TO_MS * self.CP.wheelSpeedFactor

      pt = structs.RadarData.RadarPoint()
      pt.trackId = self.track_id
      pt.dRel = msg["LEAD_DISTANCE"]
      pt.yRel = 0.0  # no lateral position available; the ACC target is in our lane
      pt.vRel = msg["LEAD_SPEED"] - v_ego  # LEAD_SPEED is the target's absolute speed (0 when LEAD_STATE is stopped)
      ret.points = [pt]
    else:
      self.lead_present = False

    return ret
