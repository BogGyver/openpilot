#include "selfdrive/car/modules/qt/num_param.h"
#include "selfdrive/car/modules/qt/tinklaparams.h"
#include "selfdrive/ui/qt/api.h"
#include "selfdrive/ui/qt/widgets/input.h"

NumParamControl::NumParamControl(QString theLabel, QString theDescription, 
              QString theWindowTitle, QString theWindowInfo, QString theUom,
              QString theParam, float theDefaultValue,
              float minVal, float maxVal, float stepVal, QString icon) : ButtonControl(theLabel, "", theDescription) {
  param_label.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  param_label.setStyleSheet("color: #aaaaaa");
  QPixmap pix(icon);
  QLabel *icon_label = new QLabel();
  icon_label->setPixmap(pix.scaledToWidth(80, Qt::SmoothTransformation));
  icon_label->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  hlayout->insertWidget(0,icon_label);
  hlayout->insertWidget(2, &param_label);
  param_name = theParam;
  default_value = theDefaultValue;
  window_title = theWindowTitle;
  window_info = theWindowInfo;
  description_txt = theDescription;
  label_txt = theLabel;
  uom = theUom;
  min_val = minVal;
  max_val = maxVal;
  step_val = stepVal;
  window_info = window_info + "\n" + 
            "Min: " + QString::number(min_val)+
            " | Max: " + QString::number(max_val) +
            " | Step: " + QString::number(step_val);

  QObject::connect(this, &ButtonControl::clicked, [=]() {
    try {
      QString txtValue = InputDialog::getNumber(window_title, this,
                  window_info, false, 1, QString::number(value));
      if (txtValue != "") {
        QTextStream floatTextStream(&txtValue);
        floatTextStream >> value;
        value = step_val * (int)(value/step_val);
        if (value < min_val) 
          value = min_val;
        if (value > max_val)
          value = max_val;
        tinkla_set_float_param(param_name.toStdString(),value);
      }
    } catch(std::exception e) {
      //restore to last value
    }
    refresh();
  });

  refresh();
}


void NumParamControl::refresh() {
  value = tinkla_get_float_param(param_name.toStdString(),default_value);
  param_label.setText(QString::number(value)+uom);
  setText("Change");
  setEnabled(true);
}