from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.subaru import subarucan
from selfdrive.car.subaru.values import DBC, PREGLOBAL_CARS, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.es_distance_cnt = -1
    self.es_lkas_cnt = -1
    self.cruise_button_prev = 0
    self.steer_rate_limited = False
    
    self.cruise_buttons_cnt = -1
    self.speed_check_cnt = -1
    
    self.dn_button_press = False
    self.dn_button_press_cnt = -1
    
    self.up_button_press = False
    self.up_button_press_cnt = -1

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, c, CS, frame, actuators, pcm_cancel_cmd, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart):

    can_sends = []

    # *** steering ***
    if (frame % self.p.STEER_STEP) == 0:

      apply_steer = int(round(actuators.steer * self.p.STEER_MAX))

      # limits due to driver torque

      new_steer = int(round(apply_steer))
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
      self.steer_rate_limited = new_steer != apply_steer

      if not c.latActive:
        apply_steer = 0

      if self.CP.carFingerprint in PREGLOBAL_CARS:
        can_sends.append(subarucan.create_preglobal_steering_control(self.packer, apply_steer, frame, self.p.STEER_STEP))
      else:
        can_sends.append(subarucan.create_steering_control(self.packer, apply_steer, frame, self.p.STEER_STEP))

      self.apply_steer_last = apply_steer

    # *** speed control button test - check once per 2 seconds - press for 5 frames if required ***
    
    cspeed_dn_cmd = False
    cspeed_up_cmd = False
    
    new_cspeed = 0   # will have no effect
    #new_cspeed = 60   # will move ACC to 60 km/h every 2 seconds
    
    if ((CS.out.cruiseState.enabled != 0) and (CS.cruise_state == 0) and (new_cspeed != 0) and (new_cspeed < 140) and (c.enabled)):
      if (self.speed_check_cnt > 200):
        self.speed_check_cnt = -1
 
        
        #new_cspeed = 50 #CS.out.cruiseState.speed * CV.MS_TO_KPH
      
        #Check down first - but to a miminum of 30 kph
        if ((new_cspeed > 29) and (new_cspeed < CS.out.cruiseState.speed * CV.MS_TO_KPH)):
          self.dn_button_press = True  #press the down button
        
        else:
          #Check up more than 20 kph, but to a max of 131 kph
          if ((new_cspeed < 131) and (new_cspeed > (CS.out.cruiseState.speed * CV.MS_TO_KPH))):
            self.up_button_press = True  #press the down button
        
        
      else:
        self.speed_check_cnt += 1

    #press the down button if required for 5 frames
    if self.dn_button_press:
      if (self.dn_button_press_cnt < 5):
        cspeed_dn_cmd = True
        self.dn_button_press_cnt += 1
      else:
        self.dn_button_press = False
        self.dn_button_press_cnt = -1
         
       #press the up button if required for 5 frames
    if self.up_button_press:
      if (self.up_button_press_cnt < 5):
        cspeed_up_cmd = True
        self.up_button_press_cnt += 1
      else:
        self.up_button_press = False
        self.up_button_press_cnt = -1

    # *** alerts and pcm cancel ***

    if self.CP.carFingerprint in PREGLOBAL_CARS:
      if self.es_distance_cnt != CS.es_distance_msg["Counter"]:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          cruise_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          cruise_button = 1
        else:
          cruise_button = CS.cruise_button

        # unstick previous mocked button press
        if cruise_button == 1 and self.cruise_button_prev == 1:
          cruise_button = 0
        self.cruise_button_prev = cruise_button

        can_sends.append(subarucan.create_preglobal_es_distance(self.packer, cruise_button, CS.es_distance_msg))
        self.es_distance_cnt = CS.es_distance_msg["Counter"]

    else:
      if self.es_distance_cnt != CS.es_distance_msg["Counter"]:
        can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg, pcm_cancel_cmd, cspeed_dn_cmd, cspeed_up_cmd))
        self.es_distance_cnt = CS.es_distance_msg["Counter"]

      if self.es_lkas_cnt != CS.es_lkas_msg["Counter"]:
        can_sends.append(subarucan.create_es_lkas(self.packer, CS.es_lkas_msg, c.enabled, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart))
        self.es_lkas_cnt = CS.es_lkas_msg["Counter"]
        
      if self.cruise_buttons_cnt != CS.sw_cruise_buttons_msg["Counter"]:
        can_sends.append(subarucan.create_cruise_buttons(self.packer, CS.sw_cruise_buttons_msg, cspeed_dn_cmd, cspeed_up_cmd))
        self.cruise_buttons_cnt = CS.sw_cruise_buttons_msg["Counter"]

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.p.STEER_MAX

    return new_actuators, can_sends
