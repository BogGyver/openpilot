"""Install exception handler for process crash."""
import os
import sys, traceback
import threading
import capnp
from selfdrive.tinklad.tinkla_interface import TinklaClient
from cereal import tinkla
from selfdrive.version import version, dirty
from selfdrive.car.tesla.readconfig import CarSettings
from common.params import Params

from selfdrive.swaglog import cloudlog

if os.getenv("NOLOG") or os.getenv("NOCRASH"):
  def capture_exception(*exc_info):
    pass
  def bind_user(**kwargs):
    pass
  def bind_extra(**kwargs):
    pass
  def install():
    pass
else:
  from raven import Client
  from raven.transport.http import HTTPTransport
  client = Client('https://1994756b5e6f41cf939a4c65de45f4f2:cefebaf3a8aa40d182609785f7189bd7@app.getsentry.com/77924',
                  install_sys_hook=False, transport=HTTPTransport, release=version, tags={'dirty': dirty})

  def sendCrashInfoToTinklad():
    carSettings = CarSettings()
    params = Params()
    tinklaClient = TinklaClient()
    dongleId = params.get("DongleId")
    userHandle = carSettings.userHandle
    event = tinkla.Interface.Event.new_message(
        openPilotId=dongleId,
        source="n/a",
        category=tinklaClient.eventCategoryKeys.crash,
        name="crash",
    )
    trace = traceback.format_exc().replace('"', '`').replace("'", '`')
    gitRemote = params.get("GitRemote")
    gitBranch = params.get("GitBranch")
    gitHash = params.get("GitCommit")
    userInfo = "User Handle: %s OpenPilotId: %s" % (userHandle, dongleId)
    gitInfo = "Git Remote: %s\nBranch: %s\nCommit: %s" % (gitRemote, gitBranch, gitHash)
    event.value.textValue="%s\n%s\n%s" % (userInfo, gitInfo, trace)
    tinklaClient.logUserEvent(event)

  def capture_exception(*args, **kwargs):
    exc_info = sys.exc_info()
    sendCrashInfoToTinklad()
    if not exc_info[0] is capnp.lib.capnp.KjException:
      client.captureException(*args, **kwargs)
    cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  def bind_user(**kwargs):
    client.user_context(kwargs)

  def bind_extra(**kwargs):
    client.extra_context(kwargs)

  def install():
    # installs a sys.excepthook
    __excepthook__ = sys.excepthook
    def handle_exception(*exc_info):
      if exc_info[0] not in (KeyboardInterrupt, SystemExit):
        capture_exception(exc_info=exc_info)
      __excepthook__(*exc_info)
    sys.excepthook = handle_exception

    """
    Workaround for `sys.excepthook` thread bug from:
    http://bugs.python.org/issue1230540
    Call once from the main thread before creating any threads.
    Source: https://stackoverflow.com/a/31622038
    """
    init_original = threading.Thread.__init__

    def init(self, *args, **kwargs):
      init_original(self, *args, **kwargs)
      run_original = self.run

      def run_with_except_hook(*args2, **kwargs2):
        try:
          run_original(*args2, **kwargs2)
        except Exception:
          sys.excepthook(*sys.exc_info())

      self.run = run_with_except_hook

    threading.Thread.__init__ = init
