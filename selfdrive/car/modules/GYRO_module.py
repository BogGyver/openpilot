from selfdrive.services import service_list
from collections import deque
import selfdrive.messaging as messaging
import zmq
import cereal

cereal_SensorEventData_acceleration = 0
cereal_SensorEventData_magnetic = 1
cereal_SensorEventData_orientation = 2
cereal_SensorEventData_gyro = 3
cereal_SensorEventData_pressure = 4
cereal_SensorEventData_magneticUncalibrated = 5
cereal_SensorEventData_gyroUncalibrated = 6
cereal_SensorEventData_proximity = 7
cereal_SensorEventData_light = 8

class GYRO_module:
  def __init__(self):
    context = zmq.Context()
    self.poller = zmq.Poller()
    self.sensorEvents = messaging.sub_sock(context, service_list['sensorEvents'].port, conflate=True, poller=self.poller)
    self.roll = 0.
    self.pitch = 0.
    self.roll_list = deque([0., 0., 0., 0., 0.])
    self.pitch_list = deque([0., 0., 0., 0., 0.])

  def list_add(self,my_list,my_element):
      avg = 0.
      my_list.popleft()
      my_list.append(my_element)
      for i in my_list:
          avg = avg + i
      avg = int( avg * 1000 / 5 )/1000.
      return avg

  def update(self):
    se_list = None
    for socket, _ in self.poller.poll(0):
        if socket is self.sensorEvents:
          se_list = messaging.recv_sock(socket)
    if se_list is not None:
        for se in se_list.sensorEvents:
            if se.which == cereal_SensorEventData_acceleration:
                self.roll = self.list_add(self.roll_list,se.acceleration.v[1])
                self.pitch = self.list_add(self.pitch_list,se.acceleration.v[2])
                #print "   Roll=[",self.roll,"]  Pitch=[",self.pitch,"]"
    return self.roll, self.pitch
