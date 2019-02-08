from selfdrive.services import service_list
from collections import deque
import selfdrive.messaging as messaging
import zmq
import cereal
import math

cereal_SensorEventData_acceleration = 0
cereal_SensorEventData_magnetic = 1
cereal_SensorEventData_orientation = 2
cereal_SensorEventData_gyro = 3
cereal_SensorEventData_pressure = 4
cereal_SensorEventData_magneticUncalibrated = 5
cereal_SensorEventData_gyroUncalibrated = 6
cereal_SensorEventData_proximity = 7
cereal_SensorEventData_light = 8

class GYROController:
  def __init__(self):
    context = zmq.Context()
    self.poller = zmq.Poller()
    self.sensorEvents = messaging.sub_sock(context, service_list['sensorEvents'].port, conflate=True, poller=self.poller)
    self.roll = 0.
    self.pitch = 0.1
    self.yaw = 9.8
    self.roll_list = deque([0., 0., 0., 0., 0.])
    self.pitch_list = deque([0., 0., 0., 0., 0.])
    self.yaw_list = deque([0., 0., 0., 0., 0.])

  def list_add(self,my_list,my_element):
    avg = 0.
    my_list.popleft()
    my_list.append(my_element)
    for i in my_list:
        avg = avg + i
    avg = int( avg * 1000 / 5 )/1000.
    return avg

  def vector_to_rad(self,p,r,y):
    aP = p #* 3.9
    aR = r #* 3.9
    aY = y #* 3.9
    pitch = math.atan (aP/math.sqrt(aR*aR + aY*aY)) #/math.pi
    roll = math.atan (aR/math.sqrt(aP*aP + aY*aY)) #/math.pi
    yaw =  math.atan (aY/math.sqrt(aP*aP + aR*aR)) #/math.pi
    return pitch,roll,yaw
      

  def update(self):
    se_list = None
    for socket, _ in self.poller.poll(0):
        if socket is self.sensorEvents:
          se_list = messaging.recv_sock(socket)
    if se_list is not None:
        for se in se_list.sensorEvents:
            if se.which == cereal_SensorEventData_acceleration:
                self.yaw = self.list_add(self.yaw_list,se.acceleration.v[0])
                self.roll = self.list_add(self.roll_list,se.acceleration.v[1])
                self.pitch = self.list_add(self.pitch_list,se.acceleration.v[2])
    #return self.pitch, self.roll, self.yaw
    return self.vector_to_rad(self.pitch, self.roll, self.yaw)
