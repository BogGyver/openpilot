#include <cassert>

#include <kaitai/kaitaistream.h>

#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "common/params.h"

#include "system/ubloxd/ublox_msg.h"

ExitHandler do_exit;
using namespace ublox;

int main() {
  LOGW("starting ubloxd");
  AlignedBuffer aligned_buf;
  UbloxMsgParser parser;

  PubMaster pm({"ubloxGnss", "gpsLocationExternal"});
  SubMaster sm({"gpsLocationTesla"});

  auto teslaAccuracy = 1000.0;

  std::unique_ptr<Context> context(Context::create());
  std::unique_ptr<SubSocket> subscriber(SubSocket::create(context.get(), "ubloxRaw"));
  assert(subscriber != NULL);
  subscriber->setTimeout(100);

  bool use_tesla_gps = true;
  if (use_tesla_gps) {
    LOGE("Using Tesla GPS when Comma GPS not accurate");
  } else {
    LOGE("Using Comma GPS only");
  }

  while (!do_exit) {
    if (use_tesla_gps) {
      sm.update(0);
      if (sm.updated("gpsLocationTesla")) {
        teslaAccuracy = sm["gpsLocationTesla"].getGpsLocationTesla().getAccuracy();
      }
    }
    std::unique_ptr<Message> msg(subscriber->receive());
    if (!msg) {
      if (errno == EINTR) {
        do_exit = true;
      }
      continue;
    }

    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(msg.get()));
    cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();
    auto ubloxRaw = event.getUbloxRaw();
    float log_time = 1e-9 * event.getLogMonoTime();

    const uint8_t *data = ubloxRaw.begin();
    size_t len = ubloxRaw.size();
    size_t bytes_consumed = 0;

    while(bytes_consumed < len && !do_exit) {
      size_t bytes_consumed_this_time = 0U;
      if(parser.add_data(log_time, data + bytes_consumed, (uint32_t)(len - bytes_consumed), bytes_consumed_this_time)) {

        try {
          auto ublox_msg = parser.gen_msg();
          if (ublox_msg.second.size() > 0) {
            if ((ublox_msg.first == "gpsLocationExternal") && (use_tesla_gps)) {
              //extract cereal from flat array
              capnp::FlatArrayMessageReader reader(ublox_msg.second);
              auto commaMessage = (reader.getRoot<cereal::Event>()).getGpsLocationExternal();
              auto commaAccuracy = commaMessage.getAccuracy();
              if (commaAccuracy > teslaAccuracy) {
                //tesla is better, send tesla
                MessageBuilder msg_builder;
                auto gpsLoc = msg_builder.initEvent().initGpsLocationExternal();
                auto teslaMessage = sm["gpsLocationTesla"].getGpsLocationTesla();
                gpsLoc.setSource(cereal::GpsLocationData::SensorSource::UBLOX);
                gpsLoc.setFlags(teslaMessage.getFlags());
                gpsLoc.setLatitude(teslaMessage.getLatitude());
                gpsLoc.setLongitude(teslaMessage.getLongitude());
                gpsLoc.setAltitude(teslaMessage.getAltitude());
                gpsLoc.setSpeed(teslaMessage.getSpeed());
                gpsLoc.setBearingDeg(teslaMessage.getBearingDeg());
                gpsLoc.setAccuracy(teslaMessage.getAccuracy());
                gpsLoc.setUnixTimestampMillis(commaMessage.getUnixTimestampMillis());
                gpsLoc.setVNED(teslaMessage.getVNED());
                gpsLoc.setVerticalAccuracy(teslaMessage.getVerticalAccuracy());
                gpsLoc.setSpeedAccuracy(teslaMessage.getSpeedAccuracy());
                gpsLoc.setBearingAccuracyDeg(teslaMessage.getBearingAccuracyDeg());
                pm.send(ublox_msg.first.c_str(), msg_builder);
              } else {
                auto bytes = ublox_msg.second.asBytes();
                pm.send(ublox_msg.first.c_str(), bytes.begin(), bytes.size());
              }
            } else {
              auto bytes = ublox_msg.second.asBytes();
              pm.send(ublox_msg.first.c_str(), bytes.begin(), bytes.size());
            }
          }
        } catch (const std::exception& e) {
          LOGE("Error parsing ublox message %s", e.what());
        }

        parser.reset();
      }
      bytes_consumed += bytes_consumed_this_time;
    }
  }

  return 0;
}