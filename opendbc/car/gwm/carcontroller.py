import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_meas_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.values import CAR, CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState

MAX_USER_TORQUE = 100  # 1.0 Nm


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.eps_counter_last = None
    self.CAN = gwmcan.CanBus(CP)
    self.accel = 0.0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # Increment counter so cancel is prioritized even without openpilot longitudinal
    if CC.cruiseControl.cancel:
      counter = (CS.steer_and_ap_stalk_msg['COUNTER'] + 1) % 16
      can_sends.append(gwmcan.create_buttons_command(
        self.packer,
        self.CAN,
        counter,
        CS.steer_and_ap_stalk_msg,
        cancel_command=True,
      ))

    if self.frame % 2 == 0: # 50 Hz
      # Steer command
      new_torque = int(round(actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
      # Prevent sending the same 'apply_torque = 1' torque repeatedly, as it can cause EPS faults.
      if abs(apply_torque) == 1:
        apply_torque = apply_torque * 2
      if not lat_active:
        apply_torque = 0
      can_sends.append(gwmcan.create_steer_command(
        self.packer,
        self.CAN,
        camera_stock_values=CS.camera_stock_values,
        steer=apply_torque,
        steer_req=lat_active,
      ))
      self.apply_torque_last = apply_torque

    # EPS feedback to the camera with a simulated driver torque, to satisfy the camera's steer nudge requests.
    # Jolion: mirror each EPS frame once (new COUNTER). The EPS stops sending 0x162 for ~120 ms every ~655 s; on a fixed
    # 50 Hz tick we kept resending its last frame with the same COUNTER and, with lane assist enabled, the camera raised
    # LSS fault (stock, the camera only sees the silence and tolerates it)
    if self.CP.carFingerprint == CAR.GWM_HAVAL_JOLION_HEV:
      send_wheel_touch = CS.eps_stock_values["COUNTER"] != self.eps_counter_last
    else:
      send_wheel_touch = self.frame % 2 == 0
    if send_wheel_touch:
      self.eps_counter_last = CS.eps_stock_values["COUNTER"]
      ea_simulated_torque = float(np.clip(self.apply_torque_last * 2, -self.params.STEER_MAX, self.params.STEER_MAX))
      if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
        ea_simulated_torque = CS.out.steeringTorque
      can_sends.append(gwmcan.create_wheel_touch(
        self.packer,
        self.CAN,
        eps_stock_values=CS.eps_stock_values,
        ea_simulated_torque=ea_simulated_torque,
      ))

      # Longitudinal control
      if self.CP.openpilotLongitudinalControl:
        standstill = actuators.longControlState == LongCtrlState.stopping
        self.accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        if self.accel < 0:
          accel = - abs(self.accel / CarControllerParams.ACCEL_MIN)
        else:
          accel = self.accel / CarControllerParams.ACCEL_MAX
        can_sends.append(gwmcan.create_longitudinal_command(
          self.packer,
          self.CAN,
          longitudinal_stock_values=CS.longitudinal_stock_values,
          accel=accel,
          active=CC.longActive,
          standstill=standstill,
        ))

    if self.frame % 5 == 0: # 20 Hz
      # HUD updates
      can_sends.append(gwmcan.create_hud_command(
        self.packer,
        self.CAN,
        hud_stock_values=CS.hud_stock_values,
        steer_required=CC.latActive,
        report_steering=self.CP.carFingerprint == CAR.GWM_HAVAL_JOLION_HEV,
      ))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
