#!/usr/bin/env python3
# Created by Raf 5/2019

from cereal import tinkla
from tinkla_interface import TinklaClient
import time
from selfdrive.car.tesla.readconfig import CarSettings

class TinklaTestClient():

    def __init__(self):
        #self.start_server()
        self.tinklaClient = TinklaClient()
        openPilotId = "test_openpilotId"
        source = "tinkladTestClient"
        userHandle = "test_user_handle"

        info = tinkla.Interface.UserInfo.new_message(
            openPilotId=openPilotId,
            userHandle=userHandle,
            gitRemote="test_github.com/something",
            gitBranch="test_gitbranch",
            gitHash="test_123456"
        )
        start_time = time.time()
        self.tinklaClient.setUserInfo(info)
        elapsed_time_us = (time.time() - start_time) * 1000 * 1000
        print("Info Time Elapsed = %d" % (elapsed_time_us))

        event = tinkla.Interface.Event.new_message(
            openPilotId=openPilotId,
            source=source,
            category=self.tinklaClient.eventCategoryKeys.userAction,
            name="pull_stalk",
        )
        event.value.textValue="up"
        start_time = time.time()
        self.tinklaClient.logUserEvent(event)
        elapsed_time_us = (time.time() - start_time) * 1000 * 1000
        print("Event Time Elapsed = %d" % (elapsed_time_us))

        carsettings = CarSettings("./bb_openpilot_config.cfg")
        carsettings.userHandle = userHandle
        print("userHandle = '%s'" % (userHandle))

        print("attemptToSendPendingMessages")
        self.tinklaClient.attemptToSendPendingMessages()

        print("send crash log")
        self.tinklaClient.logCrashStackTraceEvent(openPilotId=openPilotId)

        print("send user info 2")
        info = tinkla.Interface.UserInfo.new_message(
            openPilotId=openPilotId,
            userHandle=userHandle + "2",
            gitRemote="test_github.com/something",
            gitBranch="test_gitbranch",
            gitHash="test_123456"
        )
        self.tinklaClient.setUserInfo(info)

        #print("send can error")
        #self.tinklaClient.logCANErrorEvent(source=source, canMessage=1, additionalInformation="test can error logging", openPilotId=openPilotId)
        #time.sleep(1)
        #self.tinklaClient.logCANErrorEvent(source=source, canMessage=2, additionalInformation="test can error logging", openPilotId=openPilotId)

        #print("send process comm error")
        #self.tinklaClient.logProcessCommErrorEvent(source=source, processName="processNameWouldBeHere1", count=10, eventType="Not Alive", openPilotId=openPilotId)
        #time.sleep(1)
        #self.tinklaClient.logProcessCommErrorEvent(source=source, processName="processNameWouldBeHere2", count=10, eventType="Not Alive", openPilotId=openPilotId)

if __name__ == "__main__":
    TinklaTestClient()
