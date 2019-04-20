#!/usr/bin/env python2.7
import json
import os
import random
import time
import threading
import traceback
import zmq
import Queue
from jsonrpc import JSONRPCResponseManager, dispatcher
from websocket import create_connection, WebSocketTimeoutException

import selfdrive.crash as crash
import selfdrive.messaging as messaging
from common.params import Params
from selfdrive.services import service_list
from selfdrive.swaglog import cloudlog
from selfdrive.version import version, dirty

ATHENA_HOST = os.getenv('ATHENA_HOST', 'wss://athena.comma.ai')
HANDLER_THREADS = os.getenv('HANDLER_THREADS', 4)

dispatcher["echo"] = lambda s: s
payload_queue = Queue.Queue()
response_queue = Queue.Queue()

def handle_long_poll(ws):
  end_event = threading.Event()

  threads = [
    threading.Thread(target=ws_recv, args=(ws, end_event)),
    threading.Thread(target=ws_send, args=(ws, end_event))
  ] + [
    threading.Thread(target=jsonrpc_handler, args=(end_event,))
    for x in xrange(HANDLER_THREADS)
  ]

  map(lambda thread: thread.start(), threads)
  try:
    while not end_event.is_set():
      time.sleep(0.1)
  except (KeyboardInterrupt, SystemExit):
    end_event.set()
    raise
  finally:
    for i, thread in enumerate(threads):
      thread.join()

def jsonrpc_handler(end_event):
  while not end_event.is_set():
    try:
      data = payload_queue.get(timeout=1)
      response = JSONRPCResponseManager.handle(data, dispatcher)
      response_queue.put_nowait(response)
    except Queue.Empty:
      pass
    except Exception as e:
      cloudlog.exception("athena jsonrpc handler failed")
      traceback.print_exc()
      response_queue.put_nowait(json.dumps({"error": str(e)}))

# security: user should be able to request any message from their car
# TODO: add service to, for example, start visiond and take a picture
@dispatcher.add_method
def getMessage(service=None, timeout=1000):
  context = zmq.Context()
  if service is None or service not in service_list:
    raise Exception("invalid service")
  socket = messaging.sub_sock(context, service_list[service].port)
  socket.setsockopt(zmq.RCVTIMEO, timeout)
  ret = messaging.recv_one(socket)
  return ret.to_dict()

def ws_recv(ws, end_event):
  while not end_event.is_set():
    try:
      data = ws.recv()
      payload_queue.put_nowait(data)
    except WebSocketTimeoutException:
      pass
    except Exception:
      traceback.print_exc()
      end_event.set()

def ws_send(ws, end_event):
  while not end_event.is_set():
    try:
      response = response_queue.get(timeout=1)
      ws.send(response.json)
    except Queue.Empty:
      pass
    except Exception:
      traceback.print_exc()
      end_event.set()

def backoff(retries):
  return random.randrange(0, min(128, int(2 ** retries)))

def main(gctx=None):
  params = Params()
  dongle_id = params.get("DongleId")
  access_token = params.get("AccessToken")
  ws_uri = ATHENA_HOST + "/ws/" + dongle_id

  crash.bind_user(id=dongle_id)
  crash.bind_extra(version=version, dirty=dirty, is_eon=True)
  crash.install()

  conn_retries = 0
  while 1:
    try:
      print("connecting to %s" % ws_uri)
      ws = create_connection(ws_uri,
                             cookie="jwt=" + access_token,
                             enable_multithread=True)
      ws.settimeout(1)
      conn_retries = 0
      handle_long_poll(ws)
    except (KeyboardInterrupt, SystemExit):
      break
    except Exception:
      conn_retries += 1
      traceback.print_exc()

    time.sleep(backoff(conn_retries))

if __name__ == "__main__":
  main()
