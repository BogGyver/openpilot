#pragma once

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "common/params.h"
#include "selfdrive/ui/qt/widgets/toggle.hpp"
#include "selfdrive/ui/qt/widgets/controls.hpp"

class TinklaParamControl : public ToggleControl {
  Q_OBJECT

public:
  TinklaParamControl(const QString &param, const QString &title, const QString &desc, const QString &icon, QWidget *parent = nullptr) : ToggleControl(title, desc, icon, parent) {
    if (!Params().tinkla_get_bool_param(param.toStdString().c_str())) { 
        toggle.togglePosition();
    }
    QObject::connect(this, &ToggleControl::toggleFlipped, [=](int state) {
      Params().tinkla_set_bool_param(param.toStdString().c_str(),state);
    });
  }

}; 