from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 2
  ACCEL_MAX = 2
  ACCEL_MIN = -3.5

  def __init__(self, CP: CarParams):
    self.STEER_ERROR_MAX = 80
    self.STEER_MAX = 253
    self.STEER_DELTA_UP = 4
    self.STEER_DELTA_DOWN = 6
    if CP.carFingerprint == CAR.GWM_HAVAL_JOLION_HEV:
      # The stock camera LKA commands up to +-300, but the EPS stops applying any openpilot command above ~253 (routes
      # 000001ea-ed, 000001fe), so the cap stays at 253. Rates follow the stock LKA, measured per 50 Hz frame: up p99 5-9
      # (max 11), down p99 10 (max 15). Up is 7, not 8: at 253 it is the highest that keeps the up jerk within the ISO 11270
      # limit of test_lateral_limits (2.81 m/s^3)
      self.STEER_DELTA_UP = 7
      self.STEER_DELTA_DOWN = 10

class GwmSafetyFlags(IntFlag):
  LONG_CONTROL = 1
  JOLION = 2  # ACC main also follows the camera mode (0x2AB ADAS_MODE_ON)


@dataclass
class GWMCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.gwm]))


@dataclass
class GWMPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'gwm_haval_h6_mk3_generated',
  })


class CAR(Platforms):
  GWM_HAVAL_H6 = GWMPlatformConfig(
    [GWMCarDocs("Haval H6 2019-26")],
    CarSpecs(wheelbase=2.738, steerRatio=17.416),
  )
  GWM_HAVAL_JOLION_HEV = GWMPlatformConfig(
    [GWMCarDocs("Haval Jolion HEV 2026")],
    CarSpecs(wheelbase=2.700, steerRatio=18.6),  # learned by paramsd on the car: 18.59 +- 0.10
  )


GREATWALLMOTORS_VERSION_REQUEST_MULTI = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)
GREATWALLMOTORS_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

GREATWALLMOTORS_RX_OFFSET = 0x6a

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for bus, obd_multiplexing in [(1, True), (1, False), (0, False)] for request in [
    Request(
      [GREATWALLMOTORS_VERSION_REQUEST_MULTI],
      [GREATWALLMOTORS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      rx_offset=GREATWALLMOTORS_RX_OFFSET,
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
    Request(
      [GREATWALLMOTORS_VERSION_REQUEST_MULTI],
      [GREATWALLMOTORS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
  ]],
  fw_version_regex=br"\xf1\x87[0-9A-Z]{15}\xf1\x89[0-9A-Z]{15}",
)

DBC = CAR.create_dbc_map()
