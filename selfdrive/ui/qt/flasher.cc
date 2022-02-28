#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QScrollBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QProcess>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

QProcess *process;
QLabel *label;
int stage = 0;
QPushButton *btn;
QPushButton *btn2;
QPushButton *btn3;
QPushButton *btn4;

void set_text_1() {
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "Welcome to the Tesla EPAS flasher\n");
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "This app will patch your EPAS firmware so you can use OpenPilot on preAP Tesla Model S.\n");
  label->setText(label->text() + "Please press the brake pedal then hit Flash to start the process or Restore to restore original firmware.\n");
  label->setText(label->text() + "NOTE: KEEP BRAKE PEDAL PRESSED UNTIL PATCH PROCESS IS COMPLETE\n");
}

void set_text_2() {
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "FLASH PROCESS STARTED\n");
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "NOTE: KEEP BRAKE PEDAL PRESSED UNTIL FLASH PROCESS IS COMPLETE\n");
  label->setText(label->text() + "\n");
}

void set_text_3() {
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "FLASH PROCESS ENDED\n");
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "Hit Reboot to return to OpenPilot.\n");
  label->setText(label->text() + "\n");
}

void set_text_4() {
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "RESTORE PROCESS STARTED\n");
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "NOTE: KEEP BRAKE PEDAL PRESSED UNTIL RESTORE PROCESS IS COMPLETE\n");
  label->setText(label->text() + "\n");
}

void set_text_5() {
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "RESTORE PROCESS ENDED\n");
  label->setText(label->text() + "=================================\n");
  label->setText(label->text() + "Hit Reboot to return to OpenPilot.\n");
  label->setText(label->text() + "\n");
}

void run_script(QString script) {
  process->start(script);
}

void readStdOut() {
  label->setText(label->text() + process->readAllStandardOutput());
}

void readErrorOut() {
  label->setText(label->text() + process->readAllStandardError());
}

void onFinished(int) {
  if (stage == 1) {
    set_text_3();
    btn->setEnabled(true);
    btn2->setEnabled(false);
    btn3->setEnabled(false);
    btn4->setEnabled(false);
    btn->repaint(); 
    btn2->repaint(); 
    btn3->repaint(); 
    btn4->repaint();
    stage = 0;
    return;
  }
  if (stage == 2) {
    set_text_5();
    btn->setEnabled(true);
    btn2->setEnabled(false);
    btn3->setEnabled(false);
    btn4->setEnabled(false);
    btn->repaint(); 
    btn2->repaint(); 
    btn3->repaint(); 
    btn4->repaint();
    stage = 0;
  }

}

int main(int argc, char *argv[]) {
  initApp();
  QApplication a(argc, argv);
  QWidget window;
  setMainWindow(&window);

  process = new QProcess();  // create on the heap, so it doesn't go out of scope
  QObject::connect(process, &QProcess::readyReadStandardOutput, [=](){ 
    label->setText(label->text() + process->readAllStandardOutput()); 
  });

  QObject::connect(process, static_cast<void(QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), [=](int exitCode, QProcess::ExitStatus exitStatus){ 
    onFinished(1); 
  });
  

  QGridLayout *main_layout = new QGridLayout(&window);
  main_layout->setMargin(50);

  label = new QLabel(argv[1]);
  label->setWordWrap(true);
  label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  ScrollView *scroll = new ScrollView(label);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  main_layout->addWidget(scroll, 0, 0, 1, 4);

  // Scroll to the bottom
  QObject::connect(scroll->verticalScrollBar(), &QAbstractSlider::rangeChanged, [=]() {
    scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
  });

  btn = new QPushButton();
  btn2 = new QPushButton();
  btn3 = new QPushButton();
  btn4 = new QPushButton();
  btn->setText("Reboot");
  QObject::connect(btn, &QPushButton::clicked, [=]() {
    Hardware::reboot();
  });

  btn2->setText("Flash");
  QObject::connect(btn2, &QPushButton::clicked, [=]() {
    set_text_4();
    btn->setEnabled(false);
    btn2->setEnabled(false);
    btn3->setEnabled(false);
    btn4->setEnabled(false);
    btn->repaint(); 
    btn2->repaint(); 
    btn3->repaint(); 
    btn4->repaint(); 
    //kill what we have to again (just in case)
    //flash EPAS
    stage = 2;
    run_script("PYTHONPATH=/data/openpilot ./patch.py");
  });

  btn3->setText("Restore");
  QObject::connect(btn3, &QPushButton::clicked, [=]() {
    set_text_2();
    btn->setEnabled(false);
    btn2->setEnabled(false);
    btn3->setEnabled(false);
    btn4->setEnabled(false);
    btn->repaint(); 
    btn2->repaint(); 
    btn3->repaint(); 
    btn4->repaint();
    //kill what we have to
    //baclup EPAS
    stage = 1;
    run_script("PYTHONPATH=~/openpilot ./patch.py --restore");
  });
  
  btn4->setText("Cancel");
  QObject::connect(btn4, &QPushButton::clicked, [=]() {
    Hardware::reboot();
  });
  main_layout->addWidget(btn, 0, 3, Qt::AlignRight | Qt::AlignBottom);
  main_layout->addWidget(btn2, 0, 2, Qt::AlignRight | Qt::AlignBottom);
  main_layout->addWidget(btn3, 0, 1, Qt::AlignRight | Qt::AlignBottom);
  main_layout->addWidget(btn4, 0, 0, Qt::AlignRight | Qt::AlignBottom);
  window.setStyleSheet(R"(
    * {
      outline: none;
      padding-bottom: 100px;
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
  set_text_1();
  btn->setEnabled(false);
  btn2->setEnabled(true);
  btn3->setEnabled(true);
  btn4->setEnabled(true);
  btn->repaint(); 
  btn2->repaint(); 
  btn3->repaint(); 
  btn4->repaint();
  return a.exec();
}
