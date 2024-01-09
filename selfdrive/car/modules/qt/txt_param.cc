#include "selfdrive/car/modules/qt/txt_param.h"

#include "selfdrive/car/modules/qt/tinklaparams.h"
#include "selfdrive/ui/qt/api.h"
#include "selfdrive/ui/qt/widgets/input.h"

TextParamControl::TextParamControl(QString theLabel, QString theDescription, 
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
    if (text() == tr("ADD")) {
      QString theText = InputDialog::getText(theWindowTitle, this);
      if (theText.length() > 0) {
        tinkla_set_str_param(param_name.toStdString(),theText.toStdString());
        refresh();
      }
    } else {
      tinkla_set_str_param(param_name.toStdString(),default_value.toStdString());
      refresh();
    }
  });

  refresh();
}

void TextParamControl::refresh() {
  QString param = QString::fromStdString(tinkla_get_str_param(param_name.toStdString(),default_value.toStdString()));
  if (param.length()) {
    setValue(param);
    setText(tr("REMOVE"));
  } else {
    setValue("");
    setText(tr("ADD"));
  }
  setEnabled(true);
}