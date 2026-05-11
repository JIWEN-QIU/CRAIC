#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t
{
public:
	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};

	struct Battery
	{
		int series;           // 电池串联数
		double cell_full_voltage;    // 单电芯满电电压
		double cell_cutoff_voltage;  // 单电芯截止电压
	};

	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	AutoTakeoffLand takeoff_land;
	Battery battery;

	double ctrl_freq_max;
	int manual_ctrl_mode;
	double max_manual_vel;
	double max_manual_vel_z;
	double max_manual_yaw_rate;
	double manual_ctrl_lookahead_time;

	Parameter_t();
	void config_from_ros_handle(const ros::NodeHandle &nh);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

#endif
