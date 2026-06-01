#include "electromagnet_control/electromagnet_controller.h"

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <algorithm>
#include <vector>

class ElectromagnetTestNode
{
public:
  explicit ElectromagnetTestNode(ros::NodeHandle &nh) : nh_(nh)
  {
    electromagnet_control::ElectromagnetController::InitOptions options;
    options.enable_default = true;
    controller_.init(nh_, options);

    nh_.param("test_slot", test_slot_, controller_.getReleaseSlot(0));
    hold_sub_ = nh_.subscribe("hold", 1, &ElectromagnetTestNode::holdCallback, this);
    release_sub_ = nh_.subscribe("release", 1, &ElectromagnetTestNode::releaseCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.02), &ElectromagnetTestNode::timerCallback, this);

    ROS_INFO("[ElectromagnetTest]: Ready. slot=%d, enabled=%s, slots=[%s].",
             test_slot_, controller_.enabled() ? "true" : "false",
             controller_.slotsCsv().c_str());
    ROS_INFO("[ElectromagnetTest]: Publish std_msgs/Empty to '~hold' or '~release'.");
  }

private:
  void holdCallback(const std_msgs::Empty::ConstPtr &)
  {
    if (controller_.sendHold(test_slot_, "test_hold"))
    {
      released_slots_.erase(
          std::remove(released_slots_.begin(), released_slots_.end(), test_slot_),
          released_slots_.end());
      ROS_INFO("[ElectromagnetTest]: Hold sent on slot %d.", test_slot_);
    }
  }

  void releaseCallback(const std_msgs::Empty::ConstPtr &)
  {
    if (controller_.sendRelease(test_slot_, "test_release"))
    {
      if (std::find(released_slots_.begin(), released_slots_.end(), test_slot_) ==
          released_slots_.end())
      {
        released_slots_.push_back(test_slot_);
      }
      ROS_WARN("[ElectromagnetTest]: Release sent on slot %d.", test_slot_);
    }
  }

  void timerCallback(const ros::TimerEvent &)
  {
    controller_.streamOutputs(ros::Time::now(), released_slots_);
  }

private:
  ros::NodeHandle nh_;
  electromagnet_control::ElectromagnetController controller_;
  ros::Subscriber hold_sub_;
  ros::Subscriber release_sub_;
  ros::Timer timer_;
  int test_slot_ = 3;
  std::vector<int> released_slots_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "electromagnet_test_node");
  ros::NodeHandle nh("~");
  ElectromagnetTestNode node(nh);
  ros::spin();
  return 0;
}
