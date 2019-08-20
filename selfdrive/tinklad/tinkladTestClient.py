#!/usr/bin/env python2.7

from cereal import tinkla
from tinkla_interface import TinklaClient
import time
from selfdrive.car.tesla.readconfig import CarSettings

class TinklaTestClient():

    def __init__(self):
        #self.start_server()
        self.tinklaClient = TinklaClient()

        info = tinkla.Interface.UserInfo.new_message(
            openPilotId="test_openpilotId",
            userHandle="test_user_handle",
            gitRemote="test_github.com/something",
            gitBranch="test_gitbranch",
            gitHash="test_123456"
        )
        start_time = time.time()
        self.tinklaClient.setUserInfo(info)
        elapsed_time_us = (time.time() - start_time) * 1000 * 1000
        print("Info Time Elapsed = %d" % (elapsed_time_us))

        event = tinkla.Interface.Event.new_message(
            openPilotId="test_openpilotId",
            source="unittest",
            category=self.tinklaClient.eventCategoryKeys.userAction,
            name="pull_stalk",
        )
        event.value.textValue="up"
        start_time = time.time()
        self.tinklaClient.logUserEvent(event)
        elapsed_time_us = (time.time() - start_time) * 1000 * 1000
        print("Event Time Elapsed = %d" % (elapsed_time_us))

        carsettings = CarSettings("./bb_openpilot_config.cfg")
        userHandle = carsettings.userHandle
        print("userHandle = '%s'" % (userHandle))

        print("attemptToSendPendingMessages")
        self.tinklaClient.attemptToSendPendingMessages()

        print("send crash log")
        self.tinklaClient.logCrashStackTraceEvent(dongleId="test_openpilotId")

        print("send can error")
        self.tinklaClient.logCANErrorEvent(source="tinkladTestClient", canMessage=123, additionalInformation="test can error logging", dongleId="test_openpilotId")

if __name__ == "__main__":
    TinklaTestClient()
    