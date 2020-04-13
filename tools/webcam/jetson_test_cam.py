#!/usr/bin/env python3
import numpy as np
import time
import sys
import pygame
from pygame.locals import *
from selfdrive.car.tesla.readconfig import CarSettings

if __name__ == "__main__":
  import cv2
  strm_template = ("v4l2src device=/dev/v4l/by-id/%s  ! video/x-raw,width=%d,height=%d,framerate=%d/1,format=YUY2 !"
                             " nvvidconv ! video/x-raw(memory:NVMM),format=I420 !"
                             " nvvidconv ! video/x-raw,format=BGRx !"
                             " videoconvert ! video/x-raw,format=BGR !"
                             " videoscale ! video/x-raw,width=%d,height=%d !"
                             " appsink ")
  cs = CarSettings()
  #cs.roadCameraID
  #cs.driverCameraID
  if len(sys.argv) != 2:
    cam = str(sys.argv)
  else:
    cam = sys.argv[1]
  if cam == "road" or cam == "driver":
    print ("Processing camera [%s]\n" % cam)
    pygame.init()
    pygame.display.set_caption("OpenPilot Jetson camera stream on Pygame")
    if cam == "road":
      strm = strm_template % (cs.roadCameraID, 800, 600, 20, 1164,874)
      screen = pygame.display.set_mode([1164,874])
    else:
      strm = strm_template % (cs.driverCameraID, 640, 480, 10, 1152,864)
      screen = pygame.display.set_mode([1152,864])
    print("Capturing with stream [%s}\n" % strm)
    cap = cv2.VideoCapture(strm)
    i = 0
    try:
      while True:
        ret, frame = cap.read()
        if ret:
          if i == 0:
            print(frame.shape+'\r')
          screen.fill([0,0,0])
          frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
          frame = frame.swapaxes(0,1)
          frame = pygame.surfarray.make_surface(frame)
          screen.blit(frame, (0,0))
          pygame.display.update()

          for event in pygame.event.get():
            if event.type == KEYDOWN:
              sys.exit(0)
          i = (i + 1) % 20
    except (KeyboardInterrupt, SystemExit):
      pygame.quit()
      cv2.destroyAllWindows()
  else:
    print ("\nUnknown arguments %s\n" % str(sys.argv))
    print ("Usage: %s road|driver\n" % sys.argv[0])
