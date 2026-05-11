#include "PX4CtrlParam.hpp"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);

	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "manual_ctrl_mode", manual_ctrl_mode);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "max_manual_vel_z", max_manual_vel_z);
	read_essential_param(nh, "max_manual_yaw_rate", max_manual_yaw_rate);
	read_essential_param(nh, "manual_ctrl_lookahead_time", manual_ctrl_lookahead_time);

	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
	read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
	read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

	read_essential_param(nh, "battery/series", battery.series);
	read_essential_param(nh, "battery/cell_full_voltage", battery.cell_full_voltage);
	read_essential_param(nh, "battery/cell_cutoff_voltage", battery.cell_cutoff_voltage);

	if (battery.series <= 0)
	{
		ROS_ERROR("\"battery/series\" should be positive. Reset to 1.");
		battery.series = 1;
	}
	if (battery.cell_full_voltage <= battery.cell_cutoff_voltage)
	{
		ROS_ERROR("\"battery/cell_full_voltage\" should be greater than \"battery/cell_cutoff_voltage\". Reset to defaults (4.2, 3.3).");
		battery.cell_full_voltage = 4.2;
		battery.cell_cutoff_voltage = 3.3;
	}

	if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
	{
		takeoff_land.enable_auto_arm = false;
		ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	}
	if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
	{
		takeoff_land.no_RC = false;
		ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}
	if (manual_ctrl_mode != 0 && manual_ctrl_mode != 1)
	{
		ROS_ERROR("\"manual_ctrl_mode\" should be 0 or 1. Reset to 0.");
		manual_ctrl_mode = 0;
	}
	if (manual_ctrl_lookahead_time < 0.0)
	{
		ROS_ERROR("\"manual_ctrl_lookahead_time\" should be non-negative. Reset to 0.0.");
		manual_ctrl_lookahead_time = 0.0;
	}
	if (max_manual_vel_z < 0.0)
	{
		ROS_ERROR("\"max_manual_vel_z\" should be non-negative. Reset to 0.0.");
		max_manual_vel_z = 0.0;
	}
	if (max_manual_yaw_rate < 0.0)
	{
		ROS_ERROR("\"max_manual_yaw_rate\" should be non-negative. Reset to 0.0.");
		max_manual_yaw_rate = 0.0;
	}
};
