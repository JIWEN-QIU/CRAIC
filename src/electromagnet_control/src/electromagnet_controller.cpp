#include "electromagnet_control/electromagnet_controller.h"

#include <algorithm>
#include <limits>
#include <sstream>

namespace electromagnet_control
{

double ElectromagnetController::clampCommandValue(const double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

bool ElectromagnetController::init(ros::NodeHandle &nh, const InitOptions &options)
{
  nh.param("magnet/enable", enabled_, options.enable_default);
  nh.param("magnet/actuator_slot", actuator_slot_, options.actuator_slot_default);
  nh.param("magnet/actuator_slots", actuator_slots_, options.actuator_slots_default);
  if (actuator_slots_.empty()) actuator_slots_.push_back(actuator_slot_);

  nh.param("magnet/actuator_set_index", actuator_set_index_,
           options.actuator_set_index_default);
  nh.param("magnet/release_value", release_value_, options.release_value_default);
  nh.param("magnet/hold_value", hold_value_, options.hold_value_default);
  nh.param("magnet/hold_low", hold_low_, options.hold_low_default);
  nh.param("magnet/release_resend_period", release_resend_period_,
           options.release_resend_period_default);
  nh.param("magnet/error_throttle_sec", error_throttle_sec_,
           options.error_throttle_sec_default);
  nh.param("magnet/max_fail_count", max_fail_count_, options.max_fail_count_default);
  nh.param("magnet/disable_on_fail", disable_on_fail_,
           options.disable_on_fail_default);
  nh.param("release_order_index", release_order_index_, 0);

  release_value_ = clampCommandValue(release_value_);
  hold_value_ = clampCommandValue(hold_value_);
  if (release_resend_period_ < 0.02) release_resend_period_ = 0.02;
  if (error_throttle_sec_ < 0.2) error_throttle_sec_ = 0.2;
  if (max_fail_count_ < 1) max_fail_count_ = 1;
  if (release_order_index_ < 0) release_order_index_ = 0;

  if (!actuator_slots_.empty())
  {
    release_order_index_ =
        std::min<int>(release_order_index_, actuator_slots_.size() - 1);
    std::rotate(actuator_slots_.begin(),
                actuator_slots_.begin() + release_order_index_,
                actuator_slots_.end());
  }

  for (const int slot : actuator_slots_)
  {
    if (slot < 1 || slot > 6)
    {
      ROS_ERROR("[ElectromagnetController]: Invalid slot %d in magnet/actuator_slots. "
                "Disable magnet control.",
                slot);
      enabled_ = false;
      break;
    }
  }

  actuator_cmd_client_ =
      nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  resetRuntimeState();
  return true;
}

std::string ElectromagnetController::slotsCsv() const
{
  std::ostringstream oss;
  for (size_t i = 0; i < actuator_slots_.size(); ++i)
  {
    if (i > 0) oss << ",";
    oss << actuator_slots_[i];
  }
  return oss.str();
}

void ElectromagnetController::resetRuntimeState()
{
  command_fail_count_ = 0;
  last_release_cmd_time_ = ros::Time(0);
  last_stream_cmd_time_ = ros::Time(0);
  last_error_log_time_ = ros::Time(0);
  slot_overflow_warned_ = false;
}

int ElectromagnetController::getReleaseSlot(size_t release_order)
{
  if (actuator_slots_.empty()) return actuator_slot_;
  if (release_order < actuator_slots_.size()) return actuator_slots_[release_order];

  if (!slot_overflow_warned_)
  {
    ROS_WARN("[ElectromagnetController]: release count (%zu) exceeds actuator_slots size (%zu). "
             "Remaining releases will use the last slot %d.",
             release_order + 1, actuator_slots_.size(), actuator_slots_.back());
    slot_overflow_warned_ = true;
  }
  return actuator_slots_.back();
}

bool ElectromagnetController::shouldLogError(const ros::Time &now)
{
  if (!last_error_log_time_.isZero() &&
      (now - last_error_log_time_).toSec() < error_throttle_sec_)
  {
    return false;
  }
  last_error_log_time_ = now;
  return true;
}

void ElectromagnetController::onCommandFailure(const char *tag, int actuator_slot,
                                               int result)
{
  ++command_fail_count_;
  const ros::Time now = ros::Time::now();
  if (shouldLogError(now))
  {
    if (result >= 0)
    {
      ROS_WARN("[ElectromagnetController]: DO_SET_ACTUATOR failed (%s, slot=%d, result=%d), "
               "fail_count=%d.",
               tag, actuator_slot, result, command_fail_count_);
    }
    else
    {
      ROS_WARN("[ElectromagnetController]: DO_SET_ACTUATOR failed (%s, slot=%d), fail_count=%d.",
               tag, actuator_slot, command_fail_count_);
    }
  }

  if (disable_on_fail_ && command_fail_count_ >= max_fail_count_)
  {
    enabled_ = false;
    ROS_WARN("[ElectromagnetController]: Disable magnet control after %d consecutive failures.",
             command_fail_count_);
  }
}

bool ElectromagnetController::sendValue(int actuator_slot, double value, const char *tag)
{
  if (!enabled_) return false;
  if (!actuator_cmd_client_.exists())
  {
    onCommandFailure("service_not_ready", actuator_slot);
    return false;
  }

  mavros_msgs::CommandLong srv;
  srv.request.broadcast = false;
  srv.request.command = kMavCmdDoSetActuator;
  srv.request.confirmation = 0;

  const float nan = std::numeric_limits<float>::quiet_NaN();
  srv.request.param1 = nan;
  srv.request.param2 = nan;
  srv.request.param3 = nan;
  srv.request.param4 = nan;
  srv.request.param5 = nan;
  srv.request.param6 = nan;

  const float cmd_value = static_cast<float>(clampCommandValue(value));
  switch (actuator_slot)
  {
    case 1:
      srv.request.param1 = cmd_value;
      break;
    case 2:
      srv.request.param2 = cmd_value;
      break;
    case 3:
      srv.request.param3 = cmd_value;
      break;
    case 4:
      srv.request.param4 = cmd_value;
      break;
    case 5:
      srv.request.param5 = cmd_value;
      break;
    case 6:
      srv.request.param6 = cmd_value;
      break;
    default:
      ROS_ERROR("[ElectromagnetController]: Invalid actuator slot %d.", actuator_slot);
      return false;
  }

  srv.request.param7 = static_cast<float>(actuator_set_index_);

  if (!actuator_cmd_client_.call(srv))
  {
    onCommandFailure(tag, actuator_slot);
    return false;
  }

  if (!srv.response.success)
  {
    onCommandFailure(tag, actuator_slot, static_cast<int>(srv.response.result));
    return false;
  }

  command_fail_count_ = 0;
  last_release_cmd_time_ = ros::Time::now();
  return true;
}

bool ElectromagnetController::sendHold(int actuator_slot, const char *tag)
{
  return sendValue(actuator_slot, hold_value_, tag);
}

bool ElectromagnetController::sendRelease(int actuator_slot, const char *tag)
{
  return sendValue(actuator_slot, release_value_, tag);
}

bool ElectromagnetController::isReleasedSlot(
    const int slot, const std::vector<int> &released_slots) const
{
  return std::find(released_slots.begin(), released_slots.end(), slot) !=
         released_slots.end();
}

void ElectromagnetController::streamOutputs(
    const ros::Time &now, const std::vector<int> &released_slots)
{
  if (!enabled_ || actuator_slots_.empty()) return;
  if ((now - last_stream_cmd_time_).toSec() < release_resend_period_) return;

  for (const int slot : actuator_slots_)
  {
    const bool released = isReleasedSlot(slot, released_slots);
    if (released && !hold_low_) continue;

    const double value = released ? release_value_ : hold_value_;
    const char *tag = released ? "stream_release" : "stream_hold";
    (void)sendValue(slot, value, tag);
  }

  last_stream_cmd_time_ = now;
}

}  // namespace electromagnet_control
