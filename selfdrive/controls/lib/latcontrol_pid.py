import math

from cereal import log
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.pid import PIDController
from selfdrive.swaglog import cloudlog


class LatControlPID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             k_f=CP.lateralTuning.pid.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    
    self.kf = CP.lateralTuning.pid.kf # Just storing to detect a change

    self.last_angle_is_negative = False
    self.lateralTuneDivided = CP.lateralTuneDivided # storing to detect change

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, VM, params, last_actuators, desired_curvature, desired_curvature_rate, llk):
    # if self.CP is not CS.CP:
    #   self.CP = CS.CP # This should not happen.
    # CP param removed
    
    # k_f is immutable, and PI is too abstract for using a CP reference  
    if self.CP.lateralTuning.pid.kf != self.kf:
      self.pid.update_params(k_f=self.CP.lateralTuning.pid.kf)
      self.kf =self.CP.lateralTuning.pid.kf
    
    # TODO: JJS: Ensure that changes to CP are reflected in PI controller
    # TODO: JJS: Find a way for pid to read from something mutable directly, rather than comparing every time

    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)
    pid_log.usingNegativeTune = False

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
    error = angle_steers_des - CS.steeringAngleDeg

    # This is messy. Normally not divided and we are using pid
    # Fingers crossed error's sign matches (normally) the likely torque sign
    # If lateralTuneDivided changes to false when it was True
    # We need to switch from pidNegative to pid
    # I'm sure there is a better way
    # TODO: JJS: If this works, need to abstract at the latcontrol level as well as controlsd

    LR_SPLIT_PT = 0.1 # Degrees to offset the zero pt for L/R split.  PID should use the negative turn close to zero
    # TODO: Abstract the split point?

    if (self.lateralTuneDivided != self.CP.lateralTuneDivided):
      if not self.CP.lateralTuneDivided: # Split tune was disabled live - update to positive tune
        self.pid.update_params(k_f=self.CP.lateralTuning.pid.kf,
                              k_p=(self.CP.lateralTuning.pid.kpBP, self.CP.lateralTuning.pid.kpV),
                              k_i=(self.CP.lateralTuning.pid.kiBP, self.CP.lateralTuning.pid.kiV)
                              )
        self.kf = self.CP.lateralTuning.pid.kf
        cloudlog.warning("PID switched to positive tune, kf: {self.kf}")
      self.lateralTuneDivided = self.CP.lateralTuneDivided

    

    if self.lateralTuneDivided:
      # Cannot use error; error will be negative any time the wheel moves to the left
      # We are only concerned with instances where the car needs the wheel right of center
      # To prevent nasty flip-flopping between tunes, LR_SPLIT_PT should apply a non-zero split point.
      # The sign of the split point biases the opposite sign around zero
      # TODO: JJS: look at raw values for LateralPIDState.steeringAngleDesiredDeg
      # When the wheel is close to center there will probably be an eyeball range
      angle_is_negative = (angle_steers_des < LR_SPLIT_PT)
      pid_log.usingNegativeTune = angle_is_negative
      if self.last_angle_is_negative != angle_is_negative: # Angle has changed sign
        if angle_is_negative:
          self.pid.update_params(k_f=self.CP.lateralTuningNegative.pid.kf,
                                k_p=(self.CP.lateralTuningNegative.pid.kpBP, self.CP.lateralTuningNegative.pid.kpV),
                                k_i=(self.CP.lateralTuningNegative.pid.kiBP, self.CP.lateralTuningNegative.pid.kiV)
                                )
          self.kf = self.CP.lateralTuningNegative.pid.kf
          cloudlog.warning("PID switched to negative tune, kf: {self.kf}")
        else:
          self.pid.update_params(k_f=self.CP.lateralTuning.pid.kf,
                                k_p=(self.CP.lateralTuning.pid.kpBP, self.CP.lateralTuning.pid.kpV),
                                k_i=(self.CP.lateralTuning.pid.kiBP, self.CP.lateralTuning.pid.kiV)
                                )
          self.kf = self.CP.lateralTuning.pid.kf
          cloudlog.warning("PID switched to positive tune, kf: {self.kf}")

        self.last_angle_is_negative = angle_is_negative


    pid_log.steeringAngleDesiredDeg = angle_steers_des
    pid_log.angleError = error
    
    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      # TODO: JJS: feedforward function may need to be splitable as well
      # offset does not contribute to resistive torque
      steer_feedforward = self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)

      output_steer = self.pid.update(error, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_steer) < 1e-3, CS)

    return output_steer, angle_steers_des, pid_log
