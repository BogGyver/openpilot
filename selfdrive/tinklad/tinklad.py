#!/usr/bin/env python2.7

import zmq
import cereal
from pqueue import Queue
from airtable_publisher import Publisher
import requests
import time

LOG_PREFIX = "tinklad: "

# This needs to match tinkla.capnp message keys
class TinklaInterfaceMessageKeys():
    userInfo = 'userInfo'
    event = 'event'
    action = 'action'

# This needs to match tinkla.capnp event category keys
class TinklaInterfaceEventCategoryKeys():
    general = 'general'
    userAction = 'userAction'
    openPilotAction = 'openPilotAction'
    crash = 'crash'
    other = 'other'

class TinklaInterfaceActions():
    attemptToSendPendingMessages = 'attemptToSendPendingMessages'

class Cache():
    def push(self, event):
        self.task_queue._put(event)

    def pop(self):
        try:
            return self.task_queue._get()
        except Exception as error: #TODO: VERSIONING # pylint: disable=broad-except 
            print(LOG_PREFIX + "pop(): Error retrieving element from task queue (old format?) (%s)" % (error))
            self.task_queue._destroy()
            self.__open()
            return None

    def task_done(self):
        return self.task_queue.task_done()

    def count(self):
        return self.task_queue._qsize()

    def __open(self):
        try:
            self.task_queue = Queue("/data/tinklad-cache", tempdir="/data/local/tmp")
        except OSError:
            self.task_queue = Queue("./tinklad-cache", tempdir="./")

    def __init__(self):
        self.__open()


class TinklaServer(): 

    last_attempt_time = 0

    def attemptToSendPendingMessages(self):
        # Throttle to once per minute
        now = time.time()
        if now - self.last_attempt_time < 60:
            return
        self.last_attempt_time = now

        if self.cache.count() == 0 and self.publisher.pending_info_dict == None:
            return
        if not self.isOnline():
            return
        print(LOG_PREFIX + "Attempting to send pending messages")
        self.publish_pending_events()

    def setUserInfo(self, info, **kwargs):
        print(LOG_PREFIX + "Sending info to publisher: %s" % (info.to_dict()))
        self.info = info
        try:
            self.publisher.send_info(info)
        except Exception as error: # pylint: disable=broad-except 
            print(LOG_PREFIX + "Error attempting to publish user info (%s)" % (error))

    def logUserEvent(self, event, **kwargs):
        self.cache.push(event)
        self.publish_pending_events()

    def publish_pending_events(self):
        if self.cache.count() > 0:
            print(LOG_PREFIX + 'Cache has %d elements, attempting to publish...' %(self.cache.count()))

        while self.cache.count() > 0:
            event = self.cache.pop()
            if event.version != cereal.tinkla.interfaceVersion:
                print(LOG_PREFIX + "Unsupported event version: %0.2f (supported version: %0.2f)" % (event.version, cereal.tinkla.interfaceVersion))
                return
            try:
                print(LOG_PREFIX + "Sending event to publisher: %s" % (event.to_dict()))
                self.publisher.send_event(event)
                self.cache.task_done()
            except AssertionError as error:
                self.cache.push(event)
                print(LOG_PREFIX + "Error attempting to publish, will retry later (%s)" % (error))
                return
            except Exception as error: # pylint: disable=broad-except 
                self.cache.push(event)
                print(LOG_PREFIX + "Error attempting to publish, will retry later (%s)" % (error))
                return

    def isOnline(self):
        url='https://api.airtable.com/v0/appht7GB4aJS2A0LD'
        timeout=5
        try:
            _ = requests.get(url, timeout=timeout)
            return True
        except requests.ConnectionError:
            return False
        return False

    def __init__(self):
        self.publisher = Publisher()
        # set persitent cache for bad network / offline
        self.cache = Cache()
        self.publish_pending_events()
        messageKeys = TinklaInterfaceMessageKeys()
        actions = TinklaInterfaceActions()

        # Start server:
        ctx = zmq.Context()
        sock = ctx.socket(zmq.PULL)
        sock.bind("ipc:///tmp/tinklad")
        context = zmq.Context()
        
        while True:
            data = ''.join(sock.recv_multipart())
            tinklaInterface = cereal.tinkla.Interface.from_bytes(data)
            if tinklaInterface.version != cereal.tinkla.interfaceVersion:
                print(LOG_PREFIX + "Unsupported message version: %0.2f (supported version: %0.2f)" % (tinklaInterface.version, cereal.tinkla.interfaceVersion))
                continue
            messageType = tinklaInterface.message.which()
            if messageType == messageKeys.userInfo:
                info = tinklaInterface.message.userInfo
                self.setUserInfo(info)
            elif messageType == messageKeys.event:
                event = tinklaInterface.message.event
                self.logUserEvent(event)
            elif messageType == messageKeys.action:
                if tinklaInterface.message.action == actions.attemptToSendPendingMessages:
                    self.attemptToSendPendingMessages()
                else:
                    print(LOG_PREFIX + "Unsupported action: %s" % tinklaInterface.message.action)


def main(gctx=None):
    print("Starting tinklad service ...")
    TinklaServer()


if __name__ == "__main__":
  main()
  