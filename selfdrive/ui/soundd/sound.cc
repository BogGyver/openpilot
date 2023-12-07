#include "selfdrive/ui/soundd/sound.h"

#include <cmath>

#include <QAudio>
#include <QAudioDeviceInfo>
#include <QDebug>

#include "cereal/messaging/messaging.h"
#include "common/util.h"
#include "common/params.h"

// TODO: detect when we can't play sounds
// TODO: detect when we can't display the UI

Sound::Sound(QObject *parent) : sm({"controlsState", "microphone"}) {
  qInfo() << "default audio device: " << QAudioDeviceInfo::defaultOutputDevice().deviceName();

  for (auto &[alert, fn, loops, volume] : sound_list) {
    QSoundEffect *s = new QSoundEffect(this);
    QObject::connect(s, &QSoundEffect::statusChanged, [=]() {
      assert(s->status() != QSoundEffect::Error);
    });
    s->setSource(QUrl::fromLocalFile("../../assets/sounds/" + fn));
    s->setVolume(volume);
    sounds[alert] = {s, loops};
  }
  disable_start_stop_sounds = Params().tinkla_get_bool_param("TinklaDisableStartStopSounds");
  disable_prompt_sounds = Params().tinkla_get_bool_param("TinklaDisablePromptSounds");

  QTimer *timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &Sound::update);
  timer->start(1000 / UI_FREQ);
}

void Sound::update() {
  const bool started_prev = sm["deviceState"].getDeviceState().getStarted();
  sm.update(0);
  const bool started = sm["deviceState"].getDeviceState().getStarted();
  if (started && !started_prev) {
    started_frame = sm.frame;
    disable_start_stop_sounds = Params().tinkla_get_bool_param("TinklaDisableStartStopSounds");
    disable_prompt_sounds = Params().tinkla_get_bool_param("TinklaDisablePromptSounds");
  }

  // scale volume using ambient noise level
  if (sm.updated("microphone")) {
    float volume = util::map_val(sm["microphone"].getMicrophone().getFilteredSoundPressureWeightedDb(), 30.f, 60.f, 0.f, 1.f);
    volume = QAudio::convertVolume(volume, QAudio::LogarithmicVolumeScale, QAudio::LinearVolumeScale);
    // set volume on changes
    if (std::exchange(current_volume, std::nearbyint(volume * 10)) != current_volume) {
      Hardware::set_volume(volume);
    }
  }

  setAlert(Alert::get(sm, 0));
}

void Sound::setAlert(const Alert &alert) {
  if (!current_alert.equal(alert)) {
    current_alert = alert;
    // stop sounds
    for (auto &[s, loops] : sounds) {
      // Only stop repeating sounds
      if (s->loopsRemaining() > 1 || s->loopsRemaining() == QSoundEffect::Infinite) {
        s->stop();
      }
    }

    // play sound
    if (alert.sound != AudibleAlert::NONE) {
      if ((disable_start_stop_sounds) && ((alert.sound == AudibleAlert::ENGAGE) || (alert.sound == AudibleAlert::DISENGAGE) || (alert.sound == AudibleAlert::REFUSE))) {
        return;
      }
      if ((disable_prompt_sounds) && ((alert.sound == AudibleAlert::PROMPT) || (alert.sound == AudibleAlert::PROMPT_REPEAT) || (alert.sound == AudibleAlert::PROMPT_DISTRACTED))) {
        return;
      }
      auto &[s, loops] = sounds[alert.sound];
      s->setLoopCount(loops);
      s->play();
    }
  }
}
