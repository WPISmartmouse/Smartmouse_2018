#pragma once

#include <qt5/QtWidgets/QWidget>
#include <qt5/QtCore/QtCore>

class WorldWidget : public QWidget {
 Q_OBJECT
 public:
  WorldWidget();

 protected:
  void paintEvent(QPaintEvent *event);
};

