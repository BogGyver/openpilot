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


#define MAXPATHLEN 200 

QProcess *process;
QLabel *label;
int stage = 0;
QPushButton *btn;
QPushButton *btn2;
QPushButton *btn3;
QPushButton *btn4;

void onFinished(int a) {
  if (stage == -1) {
    stage = 0;
    return;
  }
  if (a == 0) {
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
  if (a == 1) {
    btn->setEnabled(true);
    btn2->setEnabled(true);
    btn3->setEnabled(true);
    btn4->setEnabled(false);
    btn->repaint(); 
    btn2->repaint(); 
    btn3->repaint(); 
    btn4->repaint();
    stage = 0;
  }
}


void run_script(QString script) {
  process = new QProcess();  // create on the heap, so it doesn't go out of scope
  QObject::connect(process, &QProcess::readyReadStandardOutput, [=](){ 
    label->setText(label->text() + process->readAllStandardOutput()); 
  });
  QObject::connect(process, &QProcess::readyReadStandardError, [=](){ 
    label->setText(label->text() + process->readAllStandardError()); 
  });

  QObject::connect(process, static_cast<void(QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), [=](int exitCode, QProcess::ExitStatus exitStatus){ 
    label->setText(label->text() + process->readAll());
    onFinished(exitCode); 
  });

  QObject::connect(process, &QProcess::errorOccurred, [=](QProcess::ProcessError error) 
  { 
    label->setText(label->text() + process->readAll());
    onFinished(0); 
  });
  process->start(script);
}

void reformatLabel(QString textToAdd) {
  QString labelText = label->text() + textToAdd;
  bool notDone = true;
  int start = 0;
  while (notDone) {
    int ind_n = labelText.indexOf("\n",start);
    int ind_r = labelText.indexOf("\r",start);
    if ((ind_n == -1) && (ind_n == -1)) {
      notDone = false;
    } else {
      if (ind_n == -1) ind_n = labelText.size() + 5;
      if (ind_r == -1) ind_r = labelText.size() + 5;
      int pos = std::min(ind_n,ind_r);
      int index = 1;
      if (std::abs(ind_n - ind_r) == 1 ) //consecutive
        index = 2;
      labelText.insert(pos-1,QChar(254));
      start = pos+index;
    }
  }
  QStringList ltlist = labelText.split(QChar(254),QString::SkipEmptyParts);
  QString newLabelText = "";
  QString lastRtext = "";
  int counter = 0;
  for ( const auto& line : ltlist  )
  {
      counter++;
      int ind_n = line.indexOf("\n",0);
      int ind_r = line.indexOf("\r",0);
      if ((ind_n == -1) && (ind_r > 0)) {
        //we only have \r, so just save in case is the last line
        lastRtext = line;
      } else {
        //reset lastRtext
        if ((line.size() == 0) && (counter == ltlist.size())) {
          //this is the last element and most likely empty so we do nothing
        } else {
          newLabelText = newLabelText + line;
          lastRtext = "";
        }
      }
  }
  label->setText(newLabelText+lastRtext);
}

void readStdOut() {
  //label->setText(label->text() + process->readAllStandardOutput());
  reformatLabel(process->readAllStandardOutput());
}

void readErrorOut() {
  //label->setText(label->text() + process->readAllStandardError());
  reformatLabel(process->readAllStandardError());
}

int main(int argc, char *argv[]) {
  int length;
  char fullpath[MAXPATHLEN];
  length = readlink("/proc/self/exe", fullpath, sizeof(fullpath));
  if (length < 0) {
      perror("resolving symlink /proc/self/exe.");
      exit(1);
  }
  if (length >= sizeof(fullpath)) {
      fprintf(stderr, "Path too long.\n");
      exit(1);
  }
  fullpath[length] = '\0';
  QString basePath(fullpath);
  int j = basePath.lastIndexOf('/') + 1;
  basePath = basePath.left(j);
  if (argc != 6) {
    fprintf(stderr, "flasher requires 5 arguments\n");
    fprintf(stderr, "Usage: flasher Button1 script1.sh Button2 script2.sh splash_script.sh\n");
    exit(1);
  }
  QString button1_label(argv[1]);
  QString button1_script(argv[2]);
  QString button2_label(argv[3]);
  QString button2_script(argv[4]);
  QString splash_script(argv[5]);
  initApp();
  QApplication a(argc, argv);
  QWidget window;
  setMainWindow(&window);

  process = new QProcess();  // create on the heap, so it doesn't go out of scope
  QObject::connect(process, &QProcess::readyReadStandardOutput, [=](){ 
    label->setText(label->text() + process->readAllStandardOutput()); 
  });
  QObject::connect(process, &QProcess::readyReadStandardError, [=](){ 
    label->setText(label->text() + process->readAllStandardError()); 
  });

  QObject::connect(process, static_cast<void(QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), [=](int exitCode, QProcess::ExitStatus exitStatus){ 
    label->setText(label->text() + process->readAll());
    onFinished(exitCode); 
  });

  QObject::connect(process, &QProcess::errorOccurred, [=](QProcess::ProcessError error) 
  { 
    label->setText(label->text() + process->readAll());
    onFinished(0); 
  });
  

  QGridLayout *main_layout = new QGridLayout(&window);
  main_layout->setMargin(50);

  label = new QLabel("");
  label->setWordWrap(false);
  label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  ScrollView *scroll = new ScrollView(label);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  main_layout->addWidget(scroll, 0, 0, 1, 4);

  // Scroll to the bottom
  QObject::connect(scroll->verticalScrollBar(), &QAbstractSlider::rangeChanged, [=]() {
    scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
  });

  // Scroll to the left
  QObject::connect(scroll->horizontalScrollBar(), &QAbstractSlider::rangeChanged, [=]() {
    scroll->horizontalScrollBar()->setValue(scroll->horizontalScrollBar()->minimum());
  });

  btn = new QPushButton();
  btn2 = new QPushButton();
  btn3 = new QPushButton();
  btn4 = new QPushButton();
  btn->setText("Reboot");
  QObject::connect(btn, &QPushButton::clicked, [=]() {
    Hardware::reboot();
  });

  btn2->setText(button2_label);
  QObject::connect(btn2, &QPushButton::clicked, [=]() {
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
    run_script(basePath+button2_script);
  });

  btn3->setText(button1_label);
  QObject::connect(btn3, &QPushButton::clicked, [=]() {
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
    run_script(basePath+button1_script);
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
      font-size: 40px;
    }
    QPushButton {
      padding: 50px;
      padding-right: 100px;
      padding-left: 100px;
      border: 2px solid white;
      border-radius: 20px;
      margin-right: 40px;
      font-size: 60px;
    }
  )");

  btn->setEnabled(false);
  btn2->setEnabled(true);
  btn3->setEnabled(true);
  btn4->setEnabled(true);
  btn->repaint(); 
  btn2->repaint(); 
  btn3->repaint(); 
  btn4->repaint();
  stage = -1;
  run_script(basePath+splash_script);
  return a.exec();
}
