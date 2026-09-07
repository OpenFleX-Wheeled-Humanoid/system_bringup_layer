#include "swerve_base_panel/swerve_base_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QFontDatabase>
#include <QFontMetrics>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace swerve_base_panel
{
namespace
{
QString arrow(int codepoint)
{
  return QString(QChar(codepoint));
}

QString speedText(const char * name, double value, const char * unit)
{
  return QString("%1: %2 %3").arg(name).arg(value, 0, 'f', 2).arg(unit);
}
}  // namespace

SwerveBasePanel::SwerveBasePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setupUi();
}

double SwerveBasePanel::updateSCurve(
  SCurveState & state, double target_velocity, double acceleration_time,
  double smoothness, double dt)
{
  const double max_velocity = std::max(0.0, state.max_velocity);
  target_velocity = std::clamp(target_velocity, -max_velocity, max_velocity);
  const double max_acceleration = max_velocity / std::max(0.1, acceleration_time);
  const double jerk_time = std::max(0.01, acceleration_time * smoothness);
  const double max_jerk = max_acceleration / jerk_time;
  const double velocity_error = target_velocity - state.current_velocity;
  const double desired_acceleration = std::clamp(
    velocity_error / dt, -max_acceleration, max_acceleration);
  const double max_jerk_change = max_jerk * dt;
  const double acceleration_change = std::clamp(
    desired_acceleration - state.current_acceleration,
    -max_jerk_change, max_jerk_change);
  state.current_acceleration = std::clamp(
    state.current_acceleration + acceleration_change, -max_acceleration, max_acceleration);
  state.current_velocity = std::clamp(
    state.current_velocity + state.current_acceleration * dt, -max_velocity, max_velocity);
  return state.current_velocity;
}

SwerveBasePanel::~SwerveBasePanel()
{
  if (command_timer_ != nullptr) {
    command_timer_->stop();
  }
  if (keyboard_control_enabled_) {
    releaseKeyboard();
  }
  publishStop();
}

void SwerveBasePanel::onInitialize()
{
  setupRos();

  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, this, &SwerveBasePanel::onRosSpinTimer);
  ros_spin_timer_->start(50);

  command_timer_ = new QTimer(this);
  connect(command_timer_, &QTimer::timeout, this, &SwerveBasePanel::onCommandTimer);

  setSpeedPercent(speed_percent_);
  updateStatus(QStringLiteral("Idle"), "#F3F4F6");
}

void SwerveBasePanel::setupUi()
{
  setStyleSheet(
    R"(
    QWidget {
      background: #FAFAFA;
      color: #1F2937;
      font-size: 12px;
    }
    QGroupBox {
      border: 1px solid #D1D5DB;
      border-radius: 6px;
      margin-top: 8px;
      padding-top: 8px;
      font-weight: bold;
    }
    QGroupBox::title {
      subcontrol-origin: margin;
      left: 8px;
      padding: 0 4px;
    }
    QPushButton {
      background: #FFFFFF;
      border: 1px solid #C8CED8;
      border-radius: 6px;
      padding: 7px 10px;
      min-height: 30px;
      font-size: 15px;
      font-weight: 700;
    }
    QPushButton:hover { background: #F3F4F6; border-color: #9CA3AF; }
    QPushButton:pressed { background: #E5E7EB; }
    QPushButton#MotionButton,
    QPushButton#RotateButton {
      min-height: 38px;
      font-size: 22px;
      padding: 7px 12px;
    }
    QPushButton#EmergencyStop {
      background: #DC2626;
      color: #FFFFFF;
      border-color: #991B1B;
      min-height: 34px;
      font-size: 15px;
    }
    QPushButton#EmergencyStop:hover { background: #B91C1C; }
    QPushButton#KeyboardToggle {
      min-height: 34px;
      max-height: 40px;
      padding: 6px 12px;
      font-size: 15px;
    }
    QPushButton#KeyboardToggle:checked {
      background: #2563EB;
      color: #FFFFFF;
      border-color: #1D4ED8;
    }
    QSlider::groove:horizontal {
      border: none;
      height: 6px;
      background: #E5E7EB;
      border-radius: 3px;
    }
    QSlider::handle:horizontal {
      background: #2563EB;
      width: 16px;
      margin: -5px 0;
      border-radius: 8px;
      border: 1px solid #1D4ED8;
    }
    QSlider::sub-page:horizontal { background: #60A5FA; border-radius: 3px; }
    QLabel#StatusLabel {
      border: 1px solid #D1D5DB;
      border-radius: 5px;
      padding: 6px;
    }
  )");

  auto * root = new QVBoxLayout();
  root->setContentsMargins(8, 6, 8, 6);
  root->setSpacing(8);

  auto * title_row = new QHBoxLayout();
  auto * title = new QLabel(QStringLiteral("Base Control"));
  title->setStyleSheet("font-size:15px;font-weight:bold;color:#111827;");
  topic_label_ = new QLabel(QStringLiteral("Topic: /cmd_vel"));
  topic_label_->setStyleSheet("font-size:11px;color:#6B7280;");
  title_row->addWidget(title);
  title_row->addStretch();
  title_row->addWidget(topic_label_);
  root->addLayout(title_row);

  auto * line = new QFrame();
  line->setFrameShape(QFrame::HLine);
  line->setStyleSheet("color:#E5E7EB;");
  root->addWidget(line);

  auto * speed_group = new QGroupBox(QStringLiteral("Speed"));
  auto * speed_layout = new QVBoxLayout(speed_group);

  auto * speed_button_row = new QHBoxLayout();
  speed_down_button_ = new QPushButton(QStringLiteral("减速  （C）"));
  speed_up_button_ = new QPushButton(QStringLiteral("加速  （Z）"));
  speed_button_row->addWidget(speed_down_button_);
  speed_button_row->addWidget(speed_up_button_);
  speed_layout->addLayout(speed_button_row);

  auto * speed_slider_row = new QHBoxLayout();
  speed_slider_ = new QSlider(Qt::Horizontal);
  speed_slider_->setRange(kMinSpeedPercent, kMaxSpeedPercent);
  speed_slider_->setValue(speed_percent_);
  speed_slider_->setTickInterval(25);
  speed_slider_->setTickPosition(QSlider::TicksBelow);
  speed_slider_row->addWidget(speed_slider_, 1);

  auto * speed_value_layout = new QVBoxLayout();
  linear_speed_label_ = new QLabel();
  angular_speed_label_ = new QLabel();
  const QFont mono_font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  linear_speed_label_->setFont(mono_font);
  angular_speed_label_->setFont(mono_font);
  QFontMetrics fm(mono_font);
  const int speed_label_width = fm.horizontalAdvance("Angular: 0.00 rad/s") + 14;
  linear_speed_label_->setMinimumWidth(speed_label_width);
  angular_speed_label_->setMinimumWidth(speed_label_width);
  speed_value_layout->addWidget(linear_speed_label_);
  speed_value_layout->addWidget(angular_speed_label_);
  speed_slider_row->addLayout(speed_value_layout);
  speed_layout->addLayout(speed_slider_row);
  root->addWidget(speed_group);

  auto * move_group = new QGroupBox(QStringLiteral("Move"));
  auto * move_grid = new QGridLayout(move_group);
  move_grid->setSpacing(6);

  forward_button_ = new QPushButton(arrow(0x2B06) + QStringLiteral("  （W）"));
  backward_button_ = new QPushButton(arrow(0x2B07) + QStringLiteral("  （S）"));
  left_button_ = new QPushButton(arrow(0x2B05) + QStringLiteral("  （A）"));
  right_button_ = new QPushButton(arrow(0x27A1) + QStringLiteral("  （D）"));
  forward_button_->setObjectName("MotionButton");
  backward_button_->setObjectName("MotionButton");
  left_button_->setObjectName("MotionButton");
  right_button_->setObjectName("MotionButton");
  emergency_stop_button_ = new QPushButton(QStringLiteral("STOP"));
  emergency_stop_button_->setObjectName("EmergencyStop");

  const QSizePolicy button_policy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  forward_button_->setSizePolicy(button_policy);
  backward_button_->setSizePolicy(button_policy);
  left_button_->setSizePolicy(button_policy);
  right_button_->setSizePolicy(button_policy);
  emergency_stop_button_->setSizePolicy(button_policy);

  move_grid->addWidget(forward_button_, 0, 1);
  move_grid->addWidget(left_button_, 1, 0);
  move_grid->addWidget(emergency_stop_button_, 1, 1);
  move_grid->addWidget(right_button_, 1, 2);
  move_grid->addWidget(backward_button_, 2, 1);
  root->addWidget(move_group);

  auto * rotate_group = new QGroupBox(QStringLiteral("Rotate"));
  auto * rotate_layout = new QHBoxLayout(rotate_group);
  rotate_left_button_ = new QPushButton(arrow(0x21BA) + QStringLiteral("  （Q）"));
  rotate_right_button_ = new QPushButton(arrow(0x21BB) + QStringLiteral("  （E）"));
  rotate_left_button_->setObjectName("RotateButton");
  rotate_right_button_->setObjectName("RotateButton");
  rotate_layout->addWidget(rotate_left_button_);
  rotate_layout->addWidget(rotate_right_button_);
  root->addWidget(rotate_group);

  keyboard_toggle_button_ = new QPushButton(QStringLiteral("键盘控制：关"));
  keyboard_toggle_button_->setObjectName("KeyboardToggle");
  keyboard_toggle_button_->setCheckable(true);
  keyboard_toggle_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  root->addWidget(keyboard_toggle_button_);

  status_label_ = new QLabel();
  status_label_->setObjectName("StatusLabel");
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);
  root->addStretch();

  setLayout(root);

  connect(
    speed_down_button_, &QPushButton::clicked, this,
    &SwerveBasePanel::onDecreaseSpeedClicked);
  connect(speed_up_button_, &QPushButton::clicked, this, &SwerveBasePanel::onIncreaseSpeedClicked);
  connect(speed_slider_, &QSlider::valueChanged, this, &SwerveBasePanel::onSpeedSliderChanged);

  connectMotionButton(forward_button_, &SwerveBasePanel::onForwardPressed);
  connectMotionButton(backward_button_, &SwerveBasePanel::onBackwardPressed);
  connectMotionButton(left_button_, &SwerveBasePanel::onLeftPressed);
  connectMotionButton(right_button_, &SwerveBasePanel::onRightPressed);
  connectMotionButton(rotate_left_button_, &SwerveBasePanel::onRotateLeftPressed);
  connectMotionButton(rotate_right_button_, &SwerveBasePanel::onRotateRightPressed);
  connect(
    emergency_stop_button_, &QPushButton::clicked, this,
    &SwerveBasePanel::onEmergencyStopClicked);
  connect(
    keyboard_toggle_button_,
    &QPushButton::toggled,
    this,
    &SwerveBasePanel::onKeyboardControlToggled);

  updateSpeedLabels();
}

void SwerveBasePanel::setupRos()
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  node_ = std::make_shared<rclcpp::Node>(
    "swerve_base_panel",
    rclcpp::NodeOptions().use_global_arguments(true));

  node_->declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
  node_->declare_parameter<int>("default_speed_percent", kDefaultSpeedPercent);

  cmd_vel_topic_ = node_->get_parameter("cmd_vel_topic").as_string();
  if (!config_loaded_) {
    speed_percent_ = static_cast<int>(clamp(
        node_->get_parameter("default_speed_percent").as_int(),
        kMinSpeedPercent,
        kMaxSpeedPercent));
    applySpeedPercent();
  }

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic_,
    rclcpp::SystemDefaultsQoS());

  topic_label_->setText(QStringLiteral("Topic: ") + QString::fromStdString(cmd_vel_topic_));
  RCLCPP_INFO(node_->get_logger(), "Swerve base panel publishing to: %s", cmd_vel_topic_.c_str());
}

void SwerveBasePanel::connectMotionButton(
  QPushButton * button,
  void (SwerveBasePanel::* pressed_slot)())
{
  if (button == nullptr) {
    return;
  }

  button->setAutoRepeat(false);
  button->setFocusPolicy(Qt::NoFocus);
  button->setMouseTracking(true);
  button->installEventFilter(this);

  connect(button, &QPushButton::pressed, this, pressed_slot);
  connect(
    button, &QPushButton::released, this, [this]() {
      stopActiveCommand(QStringLiteral("Stopped"));
    });
}

bool SwerveBasePanel::eventFilter(QObject * watched, QEvent * event)
{
  if (watched == active_motion_button_ &&
    (event->type() == QEvent::Leave ||
    event->type() == QEvent::Hide ||
    event->type() == QEvent::FocusOut))
  {
    stopActiveCommand(QStringLiteral("Stopped"));
  }

  return rviz_common::Panel::eventFilter(watched, event);
}

void SwerveBasePanel::keyPressEvent(QKeyEvent * event)
{
  if (event != nullptr && keyboard_control_enabled_ && !event->isAutoRepeat() &&
    handleKeyboardPress(event->key()))
  {
    event->accept();
    return;
  }

  rviz_common::Panel::keyPressEvent(event);
}

void SwerveBasePanel::keyReleaseEvent(QKeyEvent * event)
{
  if (event != nullptr && keyboard_control_enabled_ && !event->isAutoRepeat() &&
    handleKeyboardRelease(event->key()))
  {
    event->accept();
    return;
  }

  rviz_common::Panel::keyReleaseEvent(event);
}

void SwerveBasePanel::onDecreaseSpeedClicked()
{
  setSpeedScale(0.9);
  updateStatus(QStringLiteral("Speed updated"), "#ECFDF5");
}

void SwerveBasePanel::onIncreaseSpeedClicked()
{
  setSpeedScale(1.1);
  updateStatus(QStringLiteral("Speed updated"), "#ECFDF5");
}

void SwerveBasePanel::onSpeedSliderChanged(int value)
{
  setSpeedPercent(value);
  updateStatus(QStringLiteral("Speed updated"), "#ECFDF5");
}

void SwerveBasePanel::onForwardPressed()
{
  startContinuousCommand(forward_button_, linear_speed_, 0.0, 0.0, QStringLiteral("W"));
}

void SwerveBasePanel::onBackwardPressed()
{
  startContinuousCommand(backward_button_, -linear_speed_, 0.0, 0.0, QStringLiteral("S"));
}

void SwerveBasePanel::onLeftPressed()
{
  startContinuousCommand(left_button_, 0.0, linear_speed_, 0.0, QStringLiteral("A"));
}

void SwerveBasePanel::onRightPressed()
{
  startContinuousCommand(right_button_, 0.0, -linear_speed_, 0.0, QStringLiteral("D"));
}

void SwerveBasePanel::onRotateLeftPressed()
{
  startContinuousCommand(rotate_left_button_, 0.0, 0.0, angular_speed_, QStringLiteral("Q"));
}

void SwerveBasePanel::onRotateRightPressed()
{
  startContinuousCommand(rotate_right_button_, 0.0, 0.0, -angular_speed_, QStringLiteral("E"));
}

void SwerveBasePanel::onEmergencyStopClicked()
{
  clearKeyboardMotion();
  resetKeyboardSmoothing();
  stopActiveCommand(QStringLiteral("Emergency stop"));
  publishStop();
  updateStatus(QStringLiteral("Emergency stop"), "#FEE2E2");
}

void SwerveBasePanel::onKeyboardControlToggled(bool checked)
{
  setKeyboardControlEnabled(checked);
}

void SwerveBasePanel::startContinuousCommand(
  QPushButton * source_button,
  double linear_x,
  double linear_y,
  double angular_z,
  const QString & label)
{
  clearKeyboardMotion();
  resetKeyboardSmoothing();

  if (active_motion_button_ != nullptr && active_motion_button_ != source_button) {
    active_motion_button_->setDown(false);
  }

  active_motion_button_ = source_button;
  if (active_motion_button_ != nullptr) {
    active_motion_button_->setDown(true);
  }
  active_linear_x_ = linear_x;
  active_linear_y_ = linear_y;
  active_angular_z_ = angular_z;
  command_active_ = true;

  publishTwist(active_linear_x_, active_linear_y_, active_angular_z_);
  if (command_timer_ != nullptr) {
    command_timer_->start(kPublishIntervalMs);
  }

  updateStatus(QStringLiteral("%1 active").arg(label), "#DBEAFE");
}

void SwerveBasePanel::stopActiveCommand(const QString & reason)
{
  const bool was_active = command_active_ || active_motion_button_ != nullptr;
  if (active_motion_button_ != nullptr) {
    active_motion_button_->setDown(false);
  }
  command_active_ = false;
  active_motion_button_ = nullptr;

  if (command_timer_ != nullptr) {
    command_timer_->stop();
  }

  if (was_active) {
    publishStop();
    updateStatus(reason.isEmpty() ? QStringLiteral("Stopped") : reason, "#F3F4F6");
  }
}

void SwerveBasePanel::onCommandTimer()
{
  if (command_active_) {
    publishTwist(active_linear_x_, active_linear_y_, active_angular_z_);
    return;
  }
  if (keyboard_command_active_) {
    updateKeyboardCommand();
  }
}

void SwerveBasePanel::onRosSpinTimer()
{
  if (node_ != nullptr) {
    rclcpp::spin_some(node_);
  }
}

void SwerveBasePanel::publishTwist(double linear_x, double linear_y, double angular_z)
{
  if (cmd_vel_pub_ == nullptr || !rclcpp::ok()) {
    return;
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_x;
  twist.linear.y = linear_y;
  twist.angular.z = angular_z;
  cmd_vel_pub_->publish(twist);
}

void SwerveBasePanel::publishStop()
{
  publishTwist(0.0, 0.0, 0.0);
}

bool SwerveBasePanel::handleKeyboardPress(int key)
{
  if (key == Qt::Key_Z) {
    onIncreaseSpeedClicked();
    return true;
  }
  if (key == Qt::Key_C) {
    onDecreaseSpeedClicked();
    return true;
  }
  if (key == Qt::Key_Space) {
    clearKeyboardMotion();
    resetKeyboardSmoothing();
    stopActiveCommand(QStringLiteral("Emergency stop"));
    publishStop();
    updateStatus(QStringLiteral("Emergency stop"), "#FEE2E2");
    return true;
  }

  // A held mouse button owns /cmd_vel until its release. Keyboard speed keys
  // above still update the displayed scale, but motion keys cannot take over.
  if (command_active_) {
    return true;
  }

  if (!isKeyboardMotionKey(key)) {
    return false;
  }
  pressed_keyboard_keys_.insert(key);
  keyboard_command_active_ = true;
  if (command_timer_ != nullptr && !command_timer_->isActive()) {
    command_timer_->start(kPublishIntervalMs);
  }
  updateKeyboardCommand();
  return true;
}

bool SwerveBasePanel::handleKeyboardRelease(int key)
{
  if (isKeyboardMotionKey(key)) {
    pressed_keyboard_keys_.erase(key);
    return true;
  }

  return key == Qt::Key_Z || key == Qt::Key_C || key == Qt::Key_Space;
}

bool SwerveBasePanel::isKeyboardMotionKey(int key) const
{
  return key == Qt::Key_W || key == Qt::Key_Up ||
         key == Qt::Key_S || key == Qt::Key_Down ||
         key == Qt::Key_A || key == Qt::Key_Left ||
         key == Qt::Key_D || key == Qt::Key_Right ||
         key == Qt::Key_Q || key == Qt::Key_E;
}

void SwerveBasePanel::updateKeyboardCommand()
{
  const auto is_pressed = [this](int key) {
      return pressed_keyboard_keys_.find(key) != pressed_keyboard_keys_.end();
    };
  const int forward = is_pressed(Qt::Key_W) || is_pressed(Qt::Key_Up);
  const int backward = is_pressed(Qt::Key_S) || is_pressed(Qt::Key_Down);
  const int left = is_pressed(Qt::Key_A) || is_pressed(Qt::Key_Left);
  const int right = is_pressed(Qt::Key_D) || is_pressed(Qt::Key_Right);
  const int rotate_left = is_pressed(Qt::Key_Q);
  const int rotate_right = is_pressed(Qt::Key_E);

  const double target_vx = static_cast<double>(forward - backward) * linear_speed_;
  const double target_vy = static_cast<double>(left - right) * linear_speed_;
  const double target_wz = static_cast<double>(rotate_left - rotate_right) * angular_speed_;
  const double dt = static_cast<double>(kPublishIntervalMs) / 1000.0;
  const double smooth_vx = updateSCurve(
    keyboard_vx_, target_vx, kKeyboardAccelerationTime, kKeyboardSmoothness, dt);
  const double smooth_vy = updateSCurve(
    keyboard_vy_, target_vy, kKeyboardAccelerationTime, kKeyboardSmoothness, dt);
  const double smooth_wz = updateSCurve(
    keyboard_wz_, target_wz, kKeyboardAccelerationTime, kKeyboardSmoothness, dt);
  publishTwist(smooth_vx, smooth_vy, smooth_wz);

  const bool moving_target = forward || backward || left || right || rotate_left || rotate_right;
  keyboard_command_active_ = moving_target || !keyboardSmoothingAtRest();
  if (!keyboard_command_active_ && command_timer_ != nullptr) {
    command_timer_->stop();
    publishStop();
    updateStatus(QStringLiteral("Stopped"), "#F3F4F6");
  } else if (moving_target) {
    updateStatus(QStringLiteral("Keyboard control active"), "#DBEAFE");
  }
}

void SwerveBasePanel::resetKeyboardSmoothing()
{
  for (auto * state : {&keyboard_vx_, &keyboard_vy_, &keyboard_wz_}) {
    state->current_velocity = 0.0;
    state->current_acceleration = 0.0;
  }
}

void SwerveBasePanel::clearKeyboardMotion()
{
  pressed_keyboard_keys_.clear();
  keyboard_command_active_ = false;
  if (command_timer_ != nullptr && !command_active_) {
    command_timer_->stop();
  }
}

bool SwerveBasePanel::keyboardSmoothingAtRest() const
{
  constexpr double epsilon = 1e-4;
  return std::abs(keyboard_vx_.current_velocity) < epsilon &&
         std::abs(keyboard_vy_.current_velocity) < epsilon &&
         std::abs(keyboard_wz_.current_velocity) < epsilon &&
         std::abs(keyboard_vx_.current_acceleration) < epsilon &&
         std::abs(keyboard_vy_.current_acceleration) < epsilon &&
         std::abs(keyboard_wz_.current_acceleration) < epsilon;
}

void SwerveBasePanel::setKeyboardControlEnabled(bool enabled)
{
  if (keyboard_control_enabled_ == enabled) {
    return;
  }

  keyboard_control_enabled_ = enabled;
  if (keyboard_toggle_button_ != nullptr) {
    const bool old_block = keyboard_toggle_button_->blockSignals(true);
    keyboard_toggle_button_->setChecked(enabled);
    keyboard_toggle_button_->setText(
      enabled ? QStringLiteral("键盘控制：开") : QStringLiteral("键盘控制：关"));
    keyboard_toggle_button_->blockSignals(old_block);
  }

  if (enabled) {
    setFocus(Qt::OtherFocusReason);
    grabKeyboard();
    updateStatus(QStringLiteral("Keyboard control ON"), "#DBEAFE");
  } else {
    clearKeyboardMotion();
    resetKeyboardSmoothing();
    stopActiveCommand(QStringLiteral("Stopped"));
    publishStop();
    releaseKeyboard();
    updateStatus(QStringLiteral("Keyboard control OFF"), "#F3F4F6");
  }
}

void SwerveBasePanel::setSpeedScale(double scale)
{
  const int next_percent =
    static_cast<int>(std::round(static_cast<double>(speed_percent_) * scale));
  setSpeedPercent(next_percent);
}

void SwerveBasePanel::setSpeedPercent(int percent)
{
  speed_percent_ = static_cast<int>(clamp(percent, kMinSpeedPercent, kMaxSpeedPercent));

  if (speed_slider_ != nullptr && speed_slider_->value() != speed_percent_) {
    const bool old_block = speed_slider_->blockSignals(true);
    speed_slider_->setValue(speed_percent_);
    speed_slider_->blockSignals(old_block);
  }

  applySpeedPercent();
  refreshActiveCommandVelocity();
  updateSpeedLabels();
}

void SwerveBasePanel::applySpeedPercent()
{
  const double scale = static_cast<double>(speed_percent_) / 100.0;
  linear_speed_ = clamp(kDefaultLinearSpeed * scale, kMinLinearSpeed, kMaxLinearSpeed);
  angular_speed_ = clamp(kDefaultAngularSpeed * scale, kMinAngularSpeed, kMaxAngularSpeed);
  keyboard_vx_.max_velocity = linear_speed_;
  keyboard_vy_.max_velocity = linear_speed_;
  keyboard_wz_.max_velocity = angular_speed_;
}

void SwerveBasePanel::refreshActiveCommandVelocity()
{
  if (!command_active_) {
    return;
  }

  if (active_motion_button_ == forward_button_) {
    active_linear_x_ = linear_speed_;
    active_linear_y_ = 0.0;
    active_angular_z_ = 0.0;
  } else if (active_motion_button_ == backward_button_) {
    active_linear_x_ = -linear_speed_;
    active_linear_y_ = 0.0;
    active_angular_z_ = 0.0;
  } else if (active_motion_button_ == left_button_) {
    active_linear_x_ = 0.0;
    active_linear_y_ = linear_speed_;
    active_angular_z_ = 0.0;
  } else if (active_motion_button_ == right_button_) {
    active_linear_x_ = 0.0;
    active_linear_y_ = -linear_speed_;
    active_angular_z_ = 0.0;
  } else if (active_motion_button_ == rotate_left_button_) {
    active_linear_x_ = 0.0;
    active_linear_y_ = 0.0;
    active_angular_z_ = angular_speed_;
  } else if (active_motion_button_ == rotate_right_button_) {
    active_linear_x_ = 0.0;
    active_linear_y_ = 0.0;
    active_angular_z_ = -angular_speed_;
  }

  publishTwist(active_linear_x_, active_linear_y_, active_angular_z_);
}

void SwerveBasePanel::updateSpeedLabels()
{
  if (linear_speed_label_ != nullptr) {
    linear_speed_label_->setText(speedText("Linear", linear_speed_, "m/s"));
  }
  if (angular_speed_label_ != nullptr) {
    angular_speed_label_->setText(speedText("Angular", angular_speed_, "rad/s"));
  }
}

void SwerveBasePanel::updateStatus(const QString & text, const QString & color)
{
  if (status_label_ == nullptr) {
    return;
  }

  status_label_->setText(QStringLiteral("Status: ") + text);
  status_label_->setStyleSheet(
    QString(
      "QLabel#StatusLabel { background:%1; border:1px solid #D1D5DB; "
      "border-radius:5px; padding:6px; color:#1F2937; }")
    .arg(color));
}

double SwerveBasePanel::clamp(double value, double lower, double upper) const
{
  return std::max(lower, std::min(upper, value));
}

void SwerveBasePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("speed_scale_percent", speed_percent_);
  config.mapSetValue("linear_speed_mmps", static_cast<int>(linear_speed_ * 1000.0 + 0.5));
  config.mapSetValue("angular_speed_mradps", static_cast<int>(angular_speed_ * 1000.0 + 0.5));
  config.mapSetValue("cmd_vel_topic", QString::fromStdString(cmd_vel_topic_));
}

void SwerveBasePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  const std::string previous_topic = cmd_vel_topic_;
  int int_value = 0;
  if (config.mapGetInt("speed_scale_percent", &int_value)) {
    speed_percent_ = static_cast<int>(clamp(int_value, kMinSpeedPercent, kMaxSpeedPercent));
  } else if (config.mapGetInt("linear_speed_mmps", &int_value)) {
    const double speed_mps = static_cast<double>(int_value) / 1000.0;
    speed_percent_ = static_cast<int>(std::round((speed_mps / kDefaultLinearSpeed) * 100.0));
    speed_percent_ = static_cast<int>(clamp(speed_percent_, kMinSpeedPercent, kMaxSpeedPercent));
  }

  QString topic;
  if (config.mapGetString("cmd_vel_topic", &topic) && !topic.trimmed().isEmpty()) {
    cmd_vel_topic_ = topic.trimmed().toStdString();
  }
  config_loaded_ = true;

  if (node_ != nullptr && cmd_vel_topic_ != previous_topic) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::SystemDefaultsQoS());
  }

  setSpeedPercent(speed_percent_);
  if (topic_label_ != nullptr) {
    topic_label_->setText(QStringLiteral("Topic: ") + QString::fromStdString(cmd_vel_topic_));
  }
}

}  // namespace swerve_base_panel

PLUGINLIB_EXPORT_CLASS(swerve_base_panel::SwerveBasePanel, rviz_common::Panel)
