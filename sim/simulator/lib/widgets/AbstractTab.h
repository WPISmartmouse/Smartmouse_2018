#pragma once

#include <QtWidgets/QWidget>

class AbstractTab : public QWidget {

  virtual const QString getTabName() = 0;

};
