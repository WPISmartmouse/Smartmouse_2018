#include <limits>

#include <sim/simulator/lib/widgets/PIDWidget.h>
#include <sim/simulator/lib/common/TopicNames.h>
#include <msgs/msgs.h>

PIDWidget::PIDWidget() : num_points_(20) {
  left_setpoint_ = new QLineSeries();
  left_actual_= new QLineSeries();
  right_setpoint_ = new QLineSeries();
  right_actual_ = new QLineSeries();

  chart_ = new QChart();
  chart_->legend()->hide();
  chart_->addSeries(left_setpoint_);
  chart_->addSeries(left_actual_);
  chart_->addSeries(right_setpoint_);
  chart_->addSeries(right_actual_);

  x_axis_ = new QValueAxis;
  y_axis_ = new QValueAxis;

  chart_->setAxisX(x_axis_);
  chart_->setAxisY(y_axis_);
  chart_->setAnimationOptions(QChart::NoAnimation);

  setChart(chart_);

  this->node_.Subscribe(TopicNames::kPID, &PIDWidget::PIDCallback, this);
}

const QString PIDWidget::getTabName() {
  return QString("PID");
}

void PIDWidget::PIDCallback(const smartmouse::msgs::PIDDebug &msg) {
  double t = smartmouse::msgs::ConvertSec(msg.stamp());
  pid_data_.push_back({msg.left_mps_setpoint(), msg.left_mps_actual(), msg.right_mps_setpoint(), msg.right_mps_actual()});

  left_setpoint_->clear();
  left_actual_->clear();
  right_setpoint_->clear();
  right_actual_->clear();
  int i = 0;
  double y_min = std::numeric_limits<double>::max();
  double y_max = 0;
  for (auto data : pid_data_) {
    double l_set = data.at(0);
    double l_actual = data.at(1);
    double r_set = data.at(2);
    double r_actual = data.at(3);

    y_min = std::min({y_min, l_set, l_actual, r_set, r_actual});
    y_max = std::max({y_max, l_set, l_actual, r_set, r_actual});

    left_setpoint_->append((float)i/pid_data_.size(), l_set);
    left_actual_->append((float)i/pid_data_.size(), l_actual);
    right_setpoint_->append((float)i/pid_data_.size(), r_set);
    right_actual_->append((float)i/pid_data_.size(), r_actual);
    ++i;
  }

  x_axis_->setRange(0, pid_data_.size());
  y_axis_->setRange(y_min, y_max);

  if (pid_data_.size() > num_points_) {
    pid_data_.pop_front();
  }
}
