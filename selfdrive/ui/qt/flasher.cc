#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QScrollBar>
#include <QVBoxLayout>
#include <QWidget>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

void set_text_1(QLabel *label) {
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "Welcome to the Tesla EPAS flasher\n");
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "This app will patch your EPAS firmware so you can use OpenPilot on preAP Tesla Model S.\n");
  label->setText(label->text() + "Please hit Backup to start the process or Cancel to reboot and return to OpenPilot without changing your firmware.\n");
  label->setText(label->text() + "\n");
}

int main(int argc, char *argv[]) {
  initApp();
  QApplication a(argc, argv);
  QWidget window;
  setMainWindow(&window);

  QGridLayout *main_layout = new QGridLayout(&window);
  main_layout->setMargin(50);

  QLabel *label = new QLabel(argv[1]);
  label->setWordWrap(true);
  label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  ScrollView *scroll = new ScrollView(label);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  main_layout->addWidget(scroll, 0, 0, Qt::AlignTop);

  // Scroll to the bottom
  QObject::connect(scroll->verticalScrollBar(), &QAbstractSlider::rangeChanged, [=]() {
    scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
  });

  QPushButton *btn = new QPushButton();
  QPushButton *btn2 = new QPushButton();
  QPushButton *btn3 = new QPushButton();
  QPushButton *btn4 = new QPushButton();
#ifdef __aarch64__
  btn->setText("Reboot");
  QObject::connect(btn, &QPushButton::clicked, [=]() {
    Hardware::reboot();
  });
  btn2->setText("Flash");
  QObject::connect(btn2, &QPushButton::clicked, [=]() {
    //kill what we have to again (just in case)
    //flash EPAS
  });
  btn3->setText("Backup");
  QObject::connect(btn3, &QPushButton::clicked, [=]() {
    //kill what we have to 
    //backup EPAS
  });
  btn4->setText("Cancel");
  QObject::connect(btn4, &QPushButton::clicked, [=]() {
    Hardware::reboot();
  });
#else
  btn->setText("Exit");
  QObject::connect(btn, &QPushButton::clicked, &a, &QApplication::quit);
#endif
  main_layout->addWidget(btn, 0, 3, Qt::AlignRight | Qt::AlignBottom);
  main_layout->addWidget(btn2, 0, 2, Qt::AlignRight | Qt::AlignBottom);
  main_layout->addWidget(btn3, 0, 1, Qt::AlignRight | Qt::AlignBottom);
  main_layout->addWidget(btn4, 0, 0, Qt::AlignRight | Qt::AlignBottom);
  window.setStyleSheet(R"(
    * {
      outline: none;
      color: white;
      background-color: black;
      font-size: 60px;
    }
    QPushButton {
      padding: 50px;
      padding-right: 100px;
      padding-left: 100px;
      border: 2px solid white;
      border-radius: 20px;
      margin-right: 40px;
    }
  )");
  set_text_1(label);
  return a.exec();
}
