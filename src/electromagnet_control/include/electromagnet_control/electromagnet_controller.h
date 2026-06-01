#ifndef ELECTROMAGNET_CONTROL_ELECTROMAGNET_CONTROLLER_H_
#define ELECTROMAGNET_CONTROL_ELECTROMAGNET_CONTROLLER_H_

#include <ros/ros.h>
#include <mavros_msgs/CommandLong.h>

#include <string>
#include <vector>

namespace electromagnet_control
{

class ElectromagnetController
{
public:
  struct InitOptions
  {
    InitOptions()
        : enable_default(false),
          actuator_slot_default(3),
          actuator_slots_default{3, 4, 2, 1},
          actuator_set_index_default(0),
          release_value_default(-1.0),
          hold_value_default(1.0),
          hold_low_default(true),
          release_resend_period_default(0.05),
          error_throttle_sec_default(2.0),
          max_fail_count_default(30),
          disable_on_fail_default(false)
    {
    }

    bool enable_default;
    int actuator_slot_default;
    std::vector<int> actuator_slots_default;
    int actuator_set_index_default;
    double release_value_default;
    double hold_value_default;
    bool hold_low_default;
    double release_resend_period_default;
    double error_throttle_sec_default;
    int max_fail_count_default;
    bool disable_on_fail_default;
  };

  static constexpr uint16_t kMavCmdDoSetActuator = 187;

  bool init(ros::NodeHandle &nh, const InitOptions &options = InitOptions());

  bool enabled() const { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }

  int actuatorSlot() const { return actuator_slot_; }
  int actuatorSetIndex() const { return actuator_set_index_; }
  double releaseValue() const { return release_value_; }
  double holdValue() const { return hold_value_; }
  bool holdLow() const { return hold_low_; }
  double releaseResendPeriod() const { return release_resend_period_; }
  double errorThrottleSec() const { return error_throttle_sec_; }
  int maxFailCount() const { return max_fail_count_; }
  bool disableOnFail() const { return disable_on_fail_; }
  const std::vector<int> &actuatorSlots() const { return actuator_slots_; }

  std::string slotsCsv() const;

  void resetRuntimeState();
  void setLastErrorLogTime(const ros::Time &t) { last_error_log_time_ = t; }

  int getReleaseSlot(size_t release_order = 0);

  bool sendValue(int actuator_slot, double value, const char *tag);
  bool sendHold(int actuator_slot, const char *tag = "hold");
  bool sendRelease(int actuator_slot, const char *tag = "release");

  void streamOutputs(const ros::Time &now, const std::vector<int> &released_slots);

private:
  static double clampCommandValue(double value);

  bool shouldLogError(const ros::Time &now);
  void onCommandFailure(const char *tag, int actuator_slot, int result = -1);
  bool isReleasedSlot(int slot, const std::vector<int> &released_slots) const;

private:
  ros::ServiceClient actuator_cmd_client_;

  bool enabled_ = false;
  int actuator_slot_ = 3;
  std::vector<int> actuator_slots_;
  int actuator_set_index_ = 0;
  double release_value_ = -1.0;
  double hold_value_ = 1.0;
  bool hold_low_ = true;
  double release_resend_period_ = 0.05;
  double error_throttle_sec_ = 2.0;
  int max_fail_count_ = 30;
  bool disable_on_fail_ = false;
  int release_order_index_ = 0;

  int command_fail_count_ = 0;
  ros::Time last_release_cmd_time_;
  ros::Time last_stream_cmd_time_;
  ros::Time last_error_log_time_;
  bool slot_overflow_warned_ = false;
};

}  // namespace electromagnet_control

#endif
