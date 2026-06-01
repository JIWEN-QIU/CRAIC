#include "electromagnet_control/electromagnet_controller.h"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <sstream>
#include <vector>

class ElectromagnetServiceNode
{
public:
  explicit ElectromagnetServiceNode(ros::NodeHandle &nh) : nh_(nh)
  {
    electromagnet_control::ElectromagnetController::InitOptions options;
    options.enable_default = true;
    controller_.init(nh_, options);

    nh_.param("magnet/release_sequence_slots", release_sequence_slots_,
              controller_.actuatorSlots());
    if (release_sequence_slots_.empty())
    {
      release_sequence_slots_ = controller_.actuatorSlots();
    }

    for (const int slot : release_sequence_slots_)
    {
      if (slot < 1 || slot > 6)
      {
        ROS_ERROR("[ElectromagnetService]: Invalid slot %d in magnet/release_sequence_slots. "
                  "Disable magnet control.",
                  slot);
        controller_.setEnabled(false);
        break;
      }
      if (std::find(controller_.actuatorSlots().begin(),
                    controller_.actuatorSlots().end(), slot) ==
          controller_.actuatorSlots().end())
      {
        ROS_WARN("[ElectromagnetService]: release_sequence slot %d is not listed in "
                 "magnet/actuator_slots=[%s]. It will not be streamed by hold/release timer.",
                 slot, controller_.slotsCsv().c_str());
      }
    }

    release_next_srv_ =
        nh_.advertiseService("release_next", &ElectromagnetServiceNode::releaseNextCb, this);
    hold_all_srv_ =
        nh_.advertiseService("hold_all", &ElectromagnetServiceNode::holdAllCb, this);
    timer_ = nh_.createTimer(ros::Duration(0.02), &ElectromagnetServiceNode::timerCb, this);

    ROS_INFO("[ElectromagnetService]: Ready. enabled=%s slots=[%s] sequence=[%s].",
             controller_.enabled() ? "true" : "false",
             controller_.slotsCsv().c_str(),
             vectorCsv(release_sequence_slots_).c_str());
  }

private:
  static std::string vectorCsv(const std::vector<int> &values)
  {
    std::ostringstream oss;
    for (size_t i = 0; i < values.size(); ++i)
    {
      if (i > 0) oss << ",";
      oss << values[i];
    }
    return oss.str();
  }

  bool releaseNextCb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
  {
    if (!controller_.enabled())
    {
      res.success = false;
      res.message = "magnet control disabled";
      return true;
    }
    if (next_release_index_ >= release_sequence_slots_.size())
    {
      res.success = false;
      res.message = "no release slots remaining";
      ROS_WARN("[ElectromagnetService]: Reject release_next: no release slots remaining.");
      return true;
    }

    const int slot = release_sequence_slots_[next_release_index_];
    if (!controller_.sendRelease(slot, "service_release_next"))
    {
      res.success = false;
      res.message = "release command failed";
      return true;
    }

    if (std::find(released_slots_.begin(), released_slots_.end(), slot) ==
        released_slots_.end())
    {
      released_slots_.push_back(slot);
    }
    ++next_release_index_;

    std::ostringstream oss;
    oss << "released slot " << slot << " (" << next_release_index_ << "/"
        << release_sequence_slots_.size() << ")";
    res.success = true;
    res.message = oss.str();
    ROS_WARN("[ElectromagnetService]: %s.", res.message.c_str());
    return true;
  }

  bool holdAllCb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
  {
    bool ok = controller_.enabled();
    for (const int slot : controller_.actuatorSlots())
    {
      ok = controller_.sendHold(slot, "service_hold_all") && ok;
    }

    if (ok)
    {
      released_slots_.clear();
      next_release_index_ = 0;
      res.success = true;
      res.message = "all configured slots held; release sequence reset";
      ROS_WARN("[ElectromagnetService]: %s.", res.message.c_str());
    }
    else
    {
      res.success = false;
      res.message = "failed to hold all slots";
    }
    return true;
  }

  void timerCb(const ros::TimerEvent &)
  {
    controller_.streamOutputs(ros::Time::now(), released_slots_);
  }

private:
  ros::NodeHandle nh_;
  electromagnet_control::ElectromagnetController controller_;
  ros::ServiceServer release_next_srv_;
  ros::ServiceServer hold_all_srv_;
  ros::Timer timer_;
  std::vector<int> release_sequence_slots_;
  std::vector<int> released_slots_;
  size_t next_release_index_ = 0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "electromagnet_service_node");
  ros::NodeHandle nh("~");
  ElectromagnetServiceNode node(nh);
  ros::spin();
  return 0;
}
