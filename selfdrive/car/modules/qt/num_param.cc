#include "selfdrive/car/modules/qt/num_param.h"
#include "selfdrive/car/modules/qt/tinklaparams.h"
#include "selfdrive/ui/qt/api.h"
#include "selfdrive/ui/qt/widgets/input.h"

NumParamControl::NumParamControl(QString theLabel, QString theDescription, QString theWindowInfo, QString theParam, float theDefaultValue) : ButtonControl(theLabel, "", theDescription) {
  param_label.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  param_label.setStyleSheet("color: #aaaaaa");
  hlayout->insertWidget(1, &param_label);
  param_name = theParam;
  default_value = theDefaultValue;
  window_info = theWindowInfo;

  QObject::connect(this, &ButtonControl::clicked, [=]() {
    try {
      QString txtValue = InputDialog::getText(window_info, this);
      QTextStream floatTextStream(&txtValue);
      floatTextStream >> value;
      tinkla_set_float_param(param_name.toStdString(),value);
    } catch(std::exception e) {
      //restore to last value
    }
    refresh();
  });

  refresh();
}

void NumParamControl::refresh() {
  value = tinkla_get_float_param(param_name.toStdString(),default_value);
  param_label.setText(QString::number(value));
  setText("Change");
  setEnabled(true);
}