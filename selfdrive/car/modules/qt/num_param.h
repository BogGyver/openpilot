#pragma once

#include <QPushButton>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"

class NumParamControl : public ButtonControl {
  Q_OBJECT

public:
  NumParamControl(QString theLabel, QString theDescription, QString theWindowInfo, QString theParam, float theDefaultValue);

private:
  QString param_name;
  QString window_info;
  QLabel param_label;
  float default_value;
  float value;

  void refresh();
};
