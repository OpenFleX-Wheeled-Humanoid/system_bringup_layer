#pragma once

#include <set>
#include <string>

#include <QEvent>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QWidget>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

class QKeyEvent;

namespace swerve_base_panel
{

class SwerveBasePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SwerveBasePanel(QWidget * parent = nullptr);
  ~SwerveBasePanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;
  bool eventFilter(QObject * watched, QEvent * event) override;

protected:
  void keyPressEvent(QKeyEvent * event) override;
  void keyReleaseEvent(QKeyEvent * event) override;

private Q_SLOTS:
  void onDecreaseSpeedClicked();
  void onIncreaseSpeedClicked();
  void onSpeedSliderChanged(int value);
  void onForwardPressed();
  void onBackwardPressed();
  void onLeftPressed();
  void onRightPressed();
  void onRotateLeftPressed();
  void onRotateRightPressed();
  void onEmergencyStopClicked();
  void onKeyboardControlToggled(bool checked);
  void onCommandTimer();
  void onRosSpinTimer();

private:
  void setupUi();
  void setupRos();
  void connectMotionButton(QPushButton * button, void (SwerveBasePanel::* pressed_slot)());
  void startContinuousCommand(
    QPushButton * source_button,
    double linear_x,
    double linear_y,
    double angular_z,
    const QString & label);
  void stopActiveCommand(const QString & reason = QString());
  void publishTwist(double linear_x, double linear_y, double angular_z);
  void publishStop();
  bool handleKeyboardPress(int key);
  bool handleKeyboardRelease(int key);
  bool isKeyboardMotionKey(int key) const;
  void setKeyboardControlEnabled(bool enabled);
  void updateKeyboardCommand();
  void resetKeyboardSmoothing();
  void clearKeyboardMotion();
  bool keyboardSmoothingAtRest() const;
  void setSpeedScale(double scale);
  void setSpeedPercent(int percent);
  void applySpeedPercent();
  void refreshActiveCommandVelocity();
  void updateSpeedLabels();
  void updateStatus(const QString & text, const QString & color);
  double clamp(double value, double lower, double upper) const;

  static constexpr double kDefaultLinearSpeed = 0.20;
  static constexpr double kDefaultAngularSpeed = 0.20;
  static constexpr double kMinLinearSpeed = 0.05;
  static constexpr double kMaxLinearSpeed = 1.00;
  static constexpr double kMinAngularSpeed = 0.05;
  static constexpr double kMaxAngularSpeed = 1.00;
  static constexpr int kMinSpeedPercent = 25;
  static constexpr int kMaxSpeedPercent = 500;
  static constexpr int kDefaultSpeedPercent = 100;
  static constexpr int kPublishIntervalMs = 50;
  static constexpr double kKeyboardAccelerationTime = 0.5;
  static constexpr double kKeyboardSmoothness = 0.3;

  struct SCurveState
  {
    double max_velocity{0.0};
    double current_velocity{0.0};
    double current_acceleration{0.0};
  };

  static double updateSCurve(
    SCurveState & state, double target_velocity, double acceleration_time,
    double smoothness, double dt);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  QTimer * ros_spin_timer_{nullptr};
  QTimer * command_timer_{nullptr};

  QPushButton * speed_down_button_{nullptr};
  QPushButton * speed_up_button_{nullptr};
  QSlider * speed_slider_{nullptr};
  QPushButton * forward_button_{nullptr};
  QPushButton * backward_button_{nullptr};
  QPushButton * left_button_{nullptr};
  QPushButton * right_button_{nullptr};
  QPushButton * rotate_left_button_{nullptr};
  QPushButton * rotate_right_button_{nullptr};
  QPushButton * emergency_stop_button_{nullptr};
  QPushButton * keyboard_toggle_button_{nullptr};
  QPushButton * active_motion_button_{nullptr};

  QLabel * topic_label_{nullptr};
  QLabel * linear_speed_label_{nullptr};
  QLabel * angular_speed_label_{nullptr};
  QLabel * status_label_{nullptr};

  std::string cmd_vel_topic_{"/cmd_vel"};
  int speed_percent_{kDefaultSpeedPercent};
  double linear_speed_{kDefaultLinearSpeed};
  double angular_speed_{kDefaultAngularSpeed};
  bool config_loaded_{false};

  bool command_active_{false};
  double active_linear_x_{0.0};
  double active_linear_y_{0.0};
  double active_angular_z_{0.0};
  bool keyboard_control_enabled_{false};
  bool keyboard_command_active_{false};
  std::set<int> pressed_keyboard_keys_;
  SCurveState keyboard_vx_;
  SCurveState keyboard_vy_;
  SCurveState keyboard_wz_;
};

}  // namespace swerve_base_panel
