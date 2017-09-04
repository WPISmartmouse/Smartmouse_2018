#pragma once

#include <QtWidgets>
#include <QtGui/QPaintEvent>

class MazeWidget : public QWidget {
 Q_OBJECT

 public:
  MazeWidget();

  void paintEvent(QPaintEvent *event);
  const QString getTabName();
  static const int PADDING_PX;
};

