#!/usr/bin/env python2.7

import os
import zmq
import calendar
import time
import capnp
import cereal
from pqueue import Queue
from airtable_publisher import Publisher

LOG_PREFIX = "tinklad: "

class Cache():
    def push(self, event):
        self.task_queue._put(event)

    def pop(self):
        try:
            return self.task_queue._get()
        except Exception as error: 
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
            self.task_queue = Queue("/data/tinklad-cache")
        except:
            self.task_queue = Queue("./tinklad-cache")

    def __init__(self):
        self.__open()


class TinklaServer(): 

    def setUserInfo(self, info, **kwargs):
        print(LOG_PREFIX + "Sending info to publisher: %s" % (info.to_dict()))
        self.info = info
        self.publisher.send_info(info)

    def logUserEvent(self, event, **kwargs):
        self.cache.push(event)
        self.publish_pending_events()

    def publish_pending_events(self):
        if self.cache.count() > 0:
            print(LOG_PREFIX + 'Cache has %d elements, attempting to publish...' %(self.cache.count()))

        while self.cache.count() > 0:
            event = self.cache.pop()
            try:
                print(LOG_PREFIX + "Sending event to publisher: %s" % (event.to_dict()))
                self.publisher.send_event(event)
                self.cache.task_done()
            except AssertionError as error:
                self.cache.push(event)
                print(LOG_PREFIX + "Error attempting to publish, will retry later (%s)" % (error))
                return
            except Exception as error:
                self.cache.push(event)
                print(LOG_PREFIX + "Error attempting to publish, will retry later (%s)" % (error))
                return

    def __init__(self):
        self.publisher = Publisher()
        # set persitent cache for bad network / offline
        self.cache = Cache()
        self.publish_pending_events()

        # Start server:
        ctx = zmq.Context()
        sock = ctx.socket(zmq.PULL)
        sock.bind("ipc:///tmp/tinklad")
        context = zmq.Context()
        while True:
            bytes = ''.join(sock.recv_multipart())
            tinklaInterface = cereal.tinkla.Interface.from_bytes(bytes)
            messageType = tinklaInterface.message.which()
            if messageType == 'userInfo':
                info = tinklaInterface.message.userInfo
                self.setUserInfo(info)
            elif messageType == 'userEvent':
                event = tinklaInterface.message.userEvent
                self.logUserEvent(event)



def main(gctx=None):
    print("Starting tinklad service ...")
    TinklaServer()


if __name__ == "__main__":
  main()
  