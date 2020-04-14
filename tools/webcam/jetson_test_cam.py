#!/usr/bin/env python3
import numpy as np
import time
import sys
from selfdrive.car.tesla.readconfig import CarSettings
from PIL import Image

if __name__ == "__main__":
  import cv2
  strm_template = ("v4l2src device=/dev/v4l/by-id/%s  ! video/x-raw,width=%d,height=%d,framerate=%d/1,format=YUY2 !"
                             " nvvidconv ! video/x-raw(memory:NVMM),format=I420 !"
                             " nvvidconv ! video/x-raw,format=BGRx !"
                             " videoconvert ! video/x-raw,format=BGR !"
                             " videoscale ! video/x-raw,width=%d,height=%d ! %s"
                             " videobox autocrop=true ! video/x-raw,width=%d,height=%d !"
                             " appsink ")
  cs = CarSettings()
  if len(sys.argv) != 2:
    cam = str(sys.argv)
  else:
    cam = sys.argv[1]
  if cam == "road" or cam == "driver":
    print ("Processing camera [%s]\n" % cam)
    if cam == "road":
      flip_command = ""
      if cs.roadCameraFlip == 1:
        flip_command = "videoflip method=rotate-180 ! "
      strm = strm_template % (cs.roadCameraID, 800, 600, 20, round(1164*cs.roadCameraFx*1.5),round(874*cs.roadCameraFx*1.5),flip_command,1164,874)
    else:
      flip_command = ""
      if cs.driverCameraFlip == 1:
        flip_command = "videoflip method=180 ! "
      strm = strm_template % (cs.driverCameraID, 640, 480, 10, round(1152*cs.driverCameraFx),round(864*cs.driverCameraFx),flip_command,1152,864)
      dx = round(1152/4)
      dy = round(864/4)
    print("Capturing with stream [%s}\n" % strm)
    cap = cv2.VideoCapture(strm)
    i = 0
    try:
      while True:
        ret, frame = cap.read()
        if cam == "road":
          img = frame
        else:
          img = frame[:,-864//2:,:]
        if ret:
          if i == 0:
            print(img.shape,end='\r')
          cv2.imshow('preview',img)
          cv2.waitKey(1)
    except (KeyboardInterrupt, SystemExit):
      cv2.destroyAllWindows()
  else:
    print ("\nUnknown arguments %s\n" % str(sys.argv))
    print ("When using over ssh make sure you start seesion with 'ssh -Y user@jetson'\n")
    print ("Usage: %s road|driver\n" % sys.argv[0])
