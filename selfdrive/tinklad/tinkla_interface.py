#!/usr/bin/env python3
# Created by Raf 5/2019

from cereal import tinkla
import os
import zmq
import datetime
from selfdrive.tinklad import tinklad
import time

## For helpers:
import traceback
from selfdrive.car.tesla.readconfig import CarSettings
from common.params import Params

LOG_PREFIX = "tinklad client: "

tinklaClient = None

def now_iso8601():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat()+"+0000"

class TinklaClient():
    sock = None
    pid = None
    lastCanErrorTimestamp = 0
    lastProcessErrorTimestamp = 0

    eventCategoryKeys = tinklad.TinklaInterfaceEventCategoryKeys()
    messageTypeKeys = tinklad.TinklaInterfaceMessageKeys()
    actions = tinklad.TinklaInterfaceActions()

    # Configurable:
    # Note: If throttling, events are dropped
    shouldThrottleCanErrorEvents = True
    shouldThrottleProcessCommErrorEvents = True
    # Setting to every 30min for now, because we're getting a bunch of plan, pathPlan issues. 
    # Should change to around every 1min in the future when this is resolved
    throttlingPeriodInSeconds = (60*30) # One event every `throttlingPeriodInSeconds`
    
    def start_client(self):
        if os.getpid() == self.pid:
            return

        try:
            self.zctx = zmq.Context()
            self.sock = self.zctx.socket(zmq.PUSH)
            self.sock.connect("ipc:///tmp/tinklad")
            self.pid = os.getpid()
        except zmq.ZMQError:
            print("Unable to connect to tinklad")
            self.sock = None

    def setUserInfo(self, info):
        self.start_client()
        if self.sock is None:
            return

        info.timestamp = now_iso8601()
        message = tinkla.Interface.new_message()
        message.version = tinkla.interfaceVersion
        message.message.userInfo = info
        message.message.userInfo.version = tinkla.interfaceVersion

        try:
            self.sock.send(message.to_bytes(), zmq.NOBLOCK)
        except zmq.error.Again:
            # drop :/
            pass
    
    def logUserEvent(self, event):
        self.start_client()
        if self.sock is None:
            return

        event.timestamp = now_iso8601()
        message = tinkla.Interface.new_message()
        message.version = tinkla.interfaceVersion
        message.message.event = event
        message.message.event.version = tinkla.interfaceVersion

        try:
            self.sock.send(message.to_bytes(), zmq.NOBLOCK)
        except zmq.error.Again:
            # drop :/
            pass

    def attemptToSendPendingMessages(self):
        self.start_client()
        if self.sock is None:
            return

        message = tinkla.Interface.new_message()
        message.version = tinkla.interfaceVersion
        message.message.action = self.actions.attemptToSendPendingMessages

        try:
            self.sock.send(message.to_bytes(), zmq.NOBLOCK)
        except zmq.error.Again:
            # drop :/
            pass

    ## Helpers:

    def logCrashStackTraceEvent(self, openPilotId = None):
        if openPilotId is None:
            openPilotId = self.openPilotId
        event = tinkla.Interface.Event.new_message(
            openPilotId=openPilotId,
            source="n/a",
            category=self.eventCategoryKeys.crash,
            name="crash",
        )
        trace = traceback.format_exc().replace('"', '`').replace("'", '`')
        userInfo = "User Handle: %s \nOpenPilotId: %s" % (self.userHandle, self.openPilotId)
        gitInfo = "Git Remote: %s\nBranch: %s\nCommit: %s" % (self.gitRemote, self.gitBranch, self.gitHash)
        event.value.textValue="%s\n%s\n%s" % (userInfo, gitInfo, trace)
        self.logUserEvent(event)

    def logCANErrorEvent(self, source, canMessage, additionalInformation, openPilotId = None):
        if not self.carSettings.shouldLogCanErrors:
            return
        if self.shouldThrottleCanErrorEvents:
            now = time.time()
            if now - self.lastCanErrorTimestamp < self.throttlingPeriodInSeconds:
                return
            self.lastCanErrorTimestamp = now
            
        if openPilotId is None:
            openPilotId = self.openPilotId
        event = tinkla.Interface.Event.new_message(
            openPilotId=openPilotId,
            source=source,
            category=self.eventCategoryKeys.canError,
            name="CAN Error",
        )
        canInfo = "Can Message: {0}".format(hex(canMessage))
        userInfo = "User Handle: %s \nOpenPilotId: %s" % (self.userHandle, self.openPilotId)
        gitInfo = "Git Remote: %s\nBranch: %s\nCommit: %s" % (self.gitRemote, self.gitBranch, self.gitHash)
        event.value.textValue="%s\n%s\n%s\n%s" % (userInfo, gitInfo, canInfo, additionalInformation)
        self.logUserEvent(event)

    def logProcessCommErrorEvent(self, source, processName, count, eventType, openPilotId = None):
        if not self.carSettings.shouldLogProcessCommErrors:
            return
        if self.shouldThrottleProcessCommErrorEvents:
            now = time.time()
            if now - self.lastProcessErrorTimestamp < self.throttlingPeriodInSeconds:
                return
            self.lastProcessErrorTimestamp = now

        if openPilotId is None:
            openPilotId = self.openPilotId
        event = tinkla.Interface.Event.new_message(
            openPilotId=openPilotId,
            source=processName,
            category=self.eventCategoryKeys.processCommError,
            name="Process Comm Error",
        )
        additionalInformation = "Process: '%s'  \nType: '%s' \nCount: '%d' \nSource: '%s'" % (processName, eventType, count, source)
        userInfo = "User Handle: %s \nOpenPilotId: %s" % (self.userHandle, self.openPilotId)
        gitInfo = "Git Remote: %s\nBranch: %s\nCommit: %s" % (self.gitRemote, self.gitBranch, self.gitHash)
        event.value.textValue="%s\n%s\n%s" % (userInfo, gitInfo, additionalInformation)
        self.logUserEvent(event)

    def print_msg(self, message):
        print(message)

    def __init__(self):
        try:
            params = Params()
        except OSError:
            params = Params(db="./params")
        try:
            self.carSettings = CarSettings()
        except IOError:
            self.carSettings = CarSettings(optional_config_file_path="./bb_openpilot.cfg")
        self.openPilotId = params.get("DongleId")
        self.userHandle = self.carSettings.userHandle
        self.gitRemote = params.get("GitRemote")
        self.gitBranch = params.get("GitBranch")
        self.gitHash = params.get("GitCommit")

        self.start_client()
        tinklaClient = self
