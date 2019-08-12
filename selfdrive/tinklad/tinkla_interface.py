#!/usr/bin/env python2.7

from cereal import tinkla
import os
import zmq
import datetime
import tinklad

tinklaClient = None

def now_iso8601():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat()+"+0000"

class TinklaClient():
    sock = None
    pid = None

    eventCategoryKeys = tinklad.TinklaInterfaceEventCategoryKeys()
    messageTypeKeys = tinklad.TinklaInterfaceMessageKeys()
    actions = tinklad.TinklaInterfaceActions()
    
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


    def print_msg(self, message):
        print(message)

    def __init__(self):
        self.start_client()
