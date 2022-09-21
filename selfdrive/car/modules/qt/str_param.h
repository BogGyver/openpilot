#pragma once

#include <QPushButton>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"


class StrParamControl : public ButtonControl {
  Q_OBJECT

public:
  StrParamControl(QString theLabel, QString theDescription, QString theWindowTitle, QString theWindowInfo, QString theParam, QString theDefaultValue, QString icon);

private:
  QString param_name;
  QString window_info;
  QString window_title;
  QLabel param_label;
  QString description_txt;
  QString label_txt;

  Params params;

  QString default_value;
  QString value;

  void refresh();
};
