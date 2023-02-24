#include "selfdrive/car/modules/qt/str_param.h"
#include "selfdrive/ui/qt/api.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/car/modules/qt/tinklaparams.h"



StrParamControl::StrParamControl(QString theLabel, QString theDescription, 
              QString theWindowTitle, QString theWindowInfo,
              QString theParam, QString theDefaultValueDisplay, QString theDefaultValue,
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
  default_value_display = theDefaultValueDisplay;
  window_title = theWindowTitle;
  window_info = theWindowInfo;
  description_txt = theDescription;
  label_txt = theLabel;

  QObject::connect(this, &ButtonControl::clicked, [=]() {
    QString cur = QString::fromStdString(tinkla_get_str_param(param_name.toStdString(),default_value.toStdString()));
    if (cur == default_value) {
      cur = default_value_display;
    }
    QStringList options = window_info.split(",");
    QString selection = MultiOptionDialog::getSelection(window_title, options, cur, this);
    if (!selection.isEmpty()) {
      if (selection == theDefaultValueDisplay) {
        selection = default_value;
      }
      tinkla_set_str_param(param_name.toStdString(),selection.toStdString());
    }
    refresh();
  });

  refresh();
}
 

void StrParamControl::refresh() {
  bool locked = params.getBool((param_name + "Lock").toStdString());
  value = QString::fromStdString(tinkla_get_str_param(param_name.toStdString(),default_value.toStdString()));
  param_label.setText(value);
  setText("Change");
  setEnabled(!locked);
}