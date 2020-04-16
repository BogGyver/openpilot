#!/usr/bin/env python3
# Created by Raf 5/2019

import zmq
import cereal
from selfdrive.tinklad.pqueue import Queue
from selfdrive.tinklad.airtable_publisher import Publisher
import requests
import time
import os
import asyncio

LOG_PREFIX = "tinklad: "

# This needs to match tinkla.capnp message keys
class TinklaInterfaceMessageKeys:
    userInfo = 'userInfo'
    event = 'event'
    action = 'action'

# This needs to match tinkla.capnp event category keys
class TinklaInterfaceEventCategoryKeys:
    general = 'general'
    userAction = 'userAction'
    openPilotAction = 'openPilotAction'
    crash = 'crash'
    canError = 'canError'
    processCommError = 'processCommError'
    other = 'other'

class TinklaInterfaceActions:
    attemptToSendPendingMessages = 'attemptToSendPendingMessages'

class Cache():
    name = ""

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
        cacheDirectoryName = "tinklad-cache" + "/" + self.name
        cachePath = None
        tempdir = None
        if os.path.exists("/data"):
            cachePath = "/data/" + cacheDirectoryName 
            tempdir="/data/local/tmp"
        else: # Development environment:
            cachePath = "./" + cacheDirectoryName
            tempdir="./"
        if not os.path.exists(cachePath):
            os.makedirs(cachePath)
        self.task_queue = Queue(cachePath, tempdir=tempdir)

    def __init__(self, name):
        self.name = name
        self.__open()


class TinklaServer(): 

    last_attempt_time = 0

    async def attemptToSendPendingMessages(self):
        # Throttle to once per minute
        now = time.time()
        if now - self.last_attempt_time < 60:
            return
        self.last_attempt_time = now

        if self.eventCache.count() == 0 and self.userInfoCache.count() == 0:
            return
        if not self.isOnline():
            return
        print(LOG_PREFIX + "Attempting to send pending messages")
        await self.publish_pending_userinfo()
        await self.publish_pending_events()

    async def setUserInfo(self, info, **kwargs):
        print(LOG_PREFIX + "Pushing user info to cache")
        self.userInfoCache.push(info)
        await self.publish_pending_userinfo()

    async def publish_pending_userinfo(self):
        if self.userInfoCache.count() == 0:
            return

        print(LOG_PREFIX + 'User Info Cache has %d elements, attempting to publish...' %(self.userInfoCache.count()))
        newest_record = self.userInfoCache.pop()
        while self.userInfoCache.count() > 0:
            record = self.userInfoCache.pop()
            if record.timestamp > newest_record.timestamp:
                newest_record = record 

        info = newest_record
        print(LOG_PREFIX + "Sending info to publisher: %s" % (info.to_dict()))
        self.info = info
        try:
            await self.publisher.send_info(info)
            self.userInfoCache.task_done()
        except Exception as error: # pylint: disable=broad-except 
            self.userInfoCache.push(info)
            print(LOG_PREFIX + "Error attempting to publish user info (%s) (Cache has %d elements)" % (error, self.userInfoCache.count()))

    async def logUserEvent(self, event, **kwargs):
        self.eventCache.push(event)
        await self.publish_pending_events()

    async def publish_pending_events(self):
        if self.eventCache.count() > 0:
            print(LOG_PREFIX + 'Cache has %d elements, attempting to publish...' %(self.eventCache.count()))

        while self.eventCache.count() > 0:
            event = self.eventCache.pop()
            if event.version != cereal.tinkla.interfaceVersion:
                print(LOG_PREFIX + "Unsupported event version: %0.2f (supported version: %0.2f)" % (event.version, cereal.tinkla.interfaceVersion))
                return
            try:
                print(LOG_PREFIX + "Sending event to publisher: %s" % (event.to_dict()))
                await self.publisher.send_event(event)
                self.eventCache.task_done()
            except AssertionError as error:
                self.eventCache.push(event)
                print(LOG_PREFIX + "Error attempting to publish, will retry later (%s)" % (error))
                return
            except Exception as error: # pylint: disable=broad-except 
                self.eventCache.push(event)
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

    async def messageLoop(self, sock):
        messageKeys = TinklaInterfaceMessageKeys()
        actions = TinklaInterfaceActions()

        while True:
            data = b''.join(sock.recv_multipart())
            #print(LOG_PREFIX + "Received Data: " + repr(data) + "'")
            tinklaInterface = cereal.tinkla.Interface.from_bytes(data)
            if tinklaInterface.version != cereal.tinkla.interfaceVersion:
                print(LOG_PREFIX + "Unsupported message version: %0.2f (supported version: %0.2f)" % (tinklaInterface.version, cereal.tinkla.interfaceVersion))
                continue
            messageType = tinklaInterface.message.which()
            #if messageType != messageKeys.action:
            print(LOG_PREFIX + "> Received message. Type: '%s'" % messageType)
            if messageType == messageKeys.userInfo:
                info = tinklaInterface.message.userInfo
                await self.setUserInfo(info)
            elif messageType == messageKeys.event:
                event = tinklaInterface.message.event
                await self.logUserEvent(event)
            elif messageType == messageKeys.action:
                if tinklaInterface.message.action == actions.attemptToSendPendingMessages:
                    await self.attemptToSendPendingMessages()
                else:
                    print(LOG_PREFIX + "Unsupported action: %s" % tinklaInterface.message.action)

    def __init__(self):
        loop = asyncio.get_event_loop()
        self.publisher = Publisher()
        # set persitent cache for bad network / offline
        self.eventCache = Cache("events")
        self.userInfoCache = Cache("user_info")
        tasks = [
            self.publish_pending_userinfo(),
            self.publish_pending_events()
        ]
        loop.run_until_complete(asyncio.wait(tasks))

        # Start server:
        ctx = zmq.Context()
        sock = ctx.socket(zmq.PULL)
        sock.bind("ipc:///tmp/tinklad")
        context = zmq.Context()

        loop.run_until_complete(self.messageLoop(sock=sock))


def main(gctx=None):
    print("Starting tinklad service ...")
    TinklaServer()


if __name__ == "__main__":
    main()
