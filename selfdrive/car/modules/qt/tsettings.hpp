#pragma once

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>



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
