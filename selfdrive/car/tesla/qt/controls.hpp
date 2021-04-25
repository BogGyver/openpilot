#include "selfdrive/car/tesla/qt/tinklaparams.hpp"



class TinklaParamControl : public ToggleControl {
  Q_OBJECT

public:
  TinklaParamControl(const QString &param, const QString &title, const QString &desc, const QString &icon, QWidget *parent = nullptr) : ToggleControl(title, desc, icon, parent) {
    if (!tinkla_get_bool_param(param.toStdString().c_str())) { 
        toggle.togglePosition();
    }
    QObject::connect(this, &ToggleControl::toggleFlipped, [=](int state) {
      tinkla_set_bool_param(param.toStdString().c_str(),state);
    });
  }
};