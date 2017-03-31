#pragma once

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>

#endif


namespace gazebo {
  class GAZEBO_VISIBLE RegenerateWidget : public GUIPlugin {
  Q_OBJECT

  public:
    static constexpr unsigned int WIDTH = 300;
    static constexpr unsigned int HEIGHT = 80;

    RegenerateWidget();

    virtual ~RegenerateWidget();

  protected slots:

    void OnRandomButton();

    void OnBrowseFile();

    void OnInsertFromFile();

  private:
    transport::NodePtr node;
    transport::PublisherPtr regenPub;
    QLineEdit *textEdit;
    std::string maze_filename;
  };
}
