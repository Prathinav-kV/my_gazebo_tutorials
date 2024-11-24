/**
 * @file walker_class.cpp
 * @brief Implementation of the walker bot using the State Design Pattern.
 * The bot alternates between moving and turning states based on sensor data.
 *
 * @author your name
 * @version 0.1
 * @date 2023-11-26
 *
 * @copyright Copyright (c) 2024 Prathinav Karnala Venkata
 *
 */
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Forward declaration of WalkerClass
class WalkerClass;

// Abstract base class for Walker States
class WalkerState {
 public:
  virtual ~WalkerState() = default;

  /**
   * @brief Handle state-specific logic.
   *
   * @param walker Reference to the WalkerClass instance.
   */
  virtual void handle(WalkerClass &walker) = 0;  // NOLINT(runtime/references)
};

// MovingState class for when the robot is moving forward
class MovingState : public WalkerState {
 public:
  void handle(WalkerClass &walker) override;
};

// TurningState class for when the robot is turning
class TurningState : public WalkerState {
 public:
  explicit TurningState(bool turn_clockwise)
      : turn_clockwise_(turn_clockwise) {}

  void handle(WalkerClass &walker) override;

 private:
  bool turn_clockwise_;
};

// WalkerClass manages the robot and its state
class WalkerClass : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new WalkerClass object and initialize state and ROS
   * components.
   */
  WalkerClass() : Node("walker"), turn_clockwise_(true) {
    // Subscribe to laser scan topic
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&WalkerClass::scanCallback, this, std::placeholders::_1));

    // Publisher for robot velocity
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Initial state
    current_state_ = std::make_shared<MovingState>();
  }

  /**
   * @brief Set the current state of the walker.
   *
   * @param state The new state to transition to.
   */
  void setState(std::shared_ptr<WalkerState> state) { current_state_ = state; }

  /**
   * @brief Run the current state's logic.
   */
  void run() {
    if (current_state_) {
      current_state_->handle(*this);
    }
  }

  /**
   * @brief Publish velocity commands to the robot.
   *
   * @param linear Linear velocity.
   * @param angular Angular velocity.
   */
  void move(double linear, double angular) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_pub_->publish(msg);
  }

  /**
   * @brief Check if there is an obstacle in front of the robot.
   *
   * @return true if an obstacle is detected.
   * @return false otherwise.
   */
  bool obstacleDetected() const { return obstacle_detected_; }

  /**
   * @brief Toggle the turn direction between clockwise and counterclockwise.
   *
   * @return true if the turn should be clockwise.
   */
  bool toggleTurnDirection() {
    turn_clockwise_ = !turn_clockwise_;
    return turn_clockwise_;
  }

 private:
  // Callback for laser scan data
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Check for obstacles in a range
    obstacle_detected_ = false;
    for (size_t i = 0; i < 30; ++i) {
      if (msg->ranges[i] < 0.8 ||
          msg->ranges[msg->ranges.size() - 1 - i] < 0.8) {
        obstacle_detected_ = true;
        break;
      }
    }

    // Log if an obstacle is detected
    if (obstacle_detected_) {
      RCLCPP_INFO(this->get_logger(), "Obstacle Encountered!");
    }

    // Run the current state logic
    run();
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  std::shared_ptr<WalkerState> current_state_;
  bool obstacle_detected_{false};
  bool turn_clockwise_;
};

// Implementation of MovingState
void MovingState::handle(WalkerClass &walker) {
  if (walker.obstacleDetected()) {
    // Transition to TurningState
    walker.setState(
        std::make_shared<TurningState>(walker.toggleTurnDirection()));
  } else {
    // Move forward
    walker.move(0.2, 0.0);
  }
}

// Implementation of TurningState
void TurningState::handle(WalkerClass &walker) {
  if (!walker.obstacleDetected()) {
    // Transition back to MovingState
    walker.setState(std::make_shared<MovingState>());
  } else {
    // Log turning direction
    if (turn_clockwise_) {
      RCLCPP_INFO(walker.get_logger(), "Turning Clockwise!");
    } else {
      RCLCPP_INFO(walker.get_logger(), "Turning Anti-Clockwise!");
    }

    // Turn in the specified direction
    walker.move(0.0, turn_clockwise_ ? -0.5 : 0.5);
  }
}

// Main function
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto walker = std::make_shared<WalkerClass>();
  rclcpp::spin(walker);
  rclcpp::shutdown();
  return 0;
}
