from selfdrive.services import service_list
from collections import deque
import selfdrive.messaging as messaging
import cereal
import math
from common.realtime import sec_since_boot

cereal_SensorEventData_acceleration = 0
cereal_SensorEventData_magnetic = 1
cereal_SensorEventData_orientation = 2
cereal_SensorEventData_gyro = 3
cereal_SensorEventData_pressure = 4
cereal_SensorEventData_magneticUncalibrated = 5
cereal_SensorEventData_gyroUncalibrated = 6
cereal_SensorEventData_proximity = 7
cereal_SensorEventData_light = 8

NS2S = 1e-9
S2NS = 1e9
EPSILON = 0.0000001

class GYROController:
  def __init__(self):
    self.sensorEvents = messaging.sub_sock('sensorEvents', conflate=True)
    self.roll = 0.
    self.pitch = 0.01
    self.yaw = 9.8
    self.roll_list = deque([0., 0., 0., 0., 0.])
    self.pitch_list = deque([0., 0., 0., 0., 0.])
    self.yaw_list = deque([0., 0., 0., 0., 0.])
    self.mag_roll = 0.
    self.mag_pitch = 0.01
    self.mag_yaw = 9.8
    self.mag_roll_list = deque([0., 0., 0., 0., 0.])
    self.mag_pitch_list = deque([0., 0., 0., 0., 0.])
    self.mag_yaw_list = deque([0., 0., 0., 0., 0.])
    self.gyro_roll = 0.
    self.gyro_pitch = 0.01
    self.gyro_yaw = 9.8
    self.gyro_roll_list = deque([0., 0., 0., 0., 0.])
    self.gyro_pitch_list = deque([0., 0., 0., 0., 0.])
    self.gyro_yaw_list = deque([0., 0., 0., 0., 0.])
    self.gyro_t_ns = 0
    self.last_gyro_t_ns = 0

  def list_add(self,my_list,my_element):
    avg = 0.
    my_list.popleft()
    my_list.append(my_element)
    for i in my_list:
        avg = avg + i
    avg = int( avg * 1000 / 5 )/1000.
    return avg

  def vector_to_rad(self,p,r,y):
    pitch = 0
    roll = 0
    yaw = 0
    if (r*r + y*y) > 0:
      pitch = math.atan (p/math.sqrt(r*r + y*y)) 
    if (p*p+y*y) > 0:
      roll = math.atan (r/math.sqrt(p*p + y*y)) 
    if (p*p+r*r) > 0:
      yaw =  math.atan (y/math.sqrt(p*p + r*r)) 
    return pitch,roll,yaw

  def update_gyro_data(self,p,r,y):
    self.last_gyro_t_ns = self.gyro_t_ns
    self.gyro_t_ns = int(sec_since_boot() * S2NS)
    roll = float(r)
    pitch = float(p)
    yaw = float(y)
    sinTethaOverTwo = 0.
    if self.last_gyro_t_ns > 0:
      dT = float((self.gyro_t_ns - self.last_gyro_t_ns) * NS2S)
      omegaMagnitude = math.sqrt(r*r + p*p + y*y)
      roll = r
      pitch = p
      yaw = y
      if omegaMagnitude > EPSILON:
        roll /= omegaMagnitude
        pitch /= omegaMagnitude
        yaw /= omegaMagnitude
      tethaOverTwo = omegaMagnitude * dT / 2.
      sinTethaOverTwo = math.sin(tethaOverTwo)
      #cosTethaOverTwo = math.cos(tethaOverTwo)
    delta_roll = roll * sinTethaOverTwo
    delta_pitch = pitch * sinTethaOverTwo
    delta_yaw = yaw * sinTethaOverTwo
    return delta_pitch, delta_roll, delta_yaw
    

  def update(self,v,a,str_angl):
    se_list = None
    min_str_angl = 1.0
    str_ratio = 12.45 #car official steer ratio
    wb =2.964 #wheelbase in meters
    #compute radius of curvature
    r = 0.
    d = 1.
    lat_a = 0.
    if abs(str_angl) > min_str_angl:
      d = str_angl / abs(str_angl)
      r = wb / math.sqrt(2-2*math.cos(2*str_angl/str_ratio))
      lat_a = d * v * v /  r
    se_list = messaging.recv_sock(self.sensorEvents,wait=False)
    if se_list is not None:
        for se in se_list.sensorEvents:
            if se.which == cereal_SensorEventData_acceleration:
                self.yaw = self.list_add(self.yaw_list,se.acceleration.v[0])
                self.roll = self.list_add(self.roll_list,se.acceleration.v[1]) #+ lat_a
                self.pitch = self.list_add(self.pitch_list,se.acceleration.v[2]) #- a
            if se.which == cereal_SensorEventData_magnetic:
                self.mag_yaw = self.list_add(self.mag_yaw_list,se.magnetic.v[0])
                self.mag_roll = self.list_add(self.mag_roll_list,se.magnetic.v[1])
                self.mag_pitch = self.list_add(self.mag_pitch_list,se.magnetic.v[2])
            if se.which == cereal_SensorEventData_gyro:
                dp,dr,dy = self.update_gyro_data(se.gyro.v[2],se.gyro.v[1],se.gyro.v[0])
                self.gyro_yaw += dy 
                self.gyro_roll += dr 
                self.gyro_pitch += dp 
    return self.vector_to_rad(self.pitch, self.roll, self.yaw),self.vector_to_rad(self.mag_pitch, self.mag_roll, self.mag_yaw),(self.gyro_pitch, self.gyro_roll, self.gyro_yaw)
