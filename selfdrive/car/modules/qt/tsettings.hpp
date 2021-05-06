#pragma once

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>



class TinklaTogglesPanel : public QWidget {
  Q_OBJECT
public:
  explicit TinklaTogglesPanel(QWidget *parent = nullptr);
};

class TeslaTogglesPanel : public QWidget {
  Q_OBJECT
public:
  explicit TeslaTogglesPanel(QWidget *parent = nullptr);
};

class ToyotaTogglesPanel : public QWidget {
  Q_OBJECT
public:
  explicit ToyotaTogglesPanel(QWidget *parent = nullptr);
};
