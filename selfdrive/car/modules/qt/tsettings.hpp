#pragma once

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif
#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/qt/widgets/offroad_alerts.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/home.h"

class TinklaTogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TinklaTogglesPanel(SettingsWindow *parent = nullptr);
};

class TeslaTogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TeslaTogglesPanel(SettingsWindow *parent = nullptr);
};

class ToyotaTogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit ToyotaTogglesPanel(SettingsWindow *parent = nullptr);
};
