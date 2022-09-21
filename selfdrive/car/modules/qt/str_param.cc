#include "selfdrive/car/modules/qt/str_param.h"
#include "selfdrive/ui/qt/api.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/car/modules/qt/tinklaparams.h"


StrParamControl::StrParamControl(QString theLabel, QString theDescription, 
              QString theWindowTitle, QString theWindowInfo, QString theUom,
              QString theParam, QString theDefaultValue,
              QString icon) : ButtonControl(theLabel, "", theDescription) {
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

  QObject::connect(this, &ButtonControl::clicked, [=]() {
    try {
      QString txtValue = InputDialog::getNumber(window_title, this,
                  window_info, false, 1, value);
      tinkla_set_str_param(param_name.toStdString(),txtValue.toStdString);
    } catch(std::exception e) {
      //restore to last value
    }
    refresh();
  });

  refresh();
}


void StrParamControl::refresh() {
  bool locked = params.getBool((param_name + "Lock").toStdString());
  value = QString::fromStdString(tinkla_get_str_param(param_name.toStdString(),default_value));
  param_label.setText(value);
  setText("Change");
  setEnabled(!locked);
}