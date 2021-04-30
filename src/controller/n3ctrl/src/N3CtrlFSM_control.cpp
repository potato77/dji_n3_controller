#include "N3CtrlFSM.h"

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

void N3CtrlFSM::process_control(const ros::Time& now_time)
{
	Controller_Output_t u;
	SO3_Controller_Output_t u_so3;

	//ROS_WARN("[n3ctrl] state = %d",state); //zxzxzxzx
	//result: state always equals 2
	//ROS_WARN("[n3ctrl] js_ctrl_mode = %d",js_ctrl_mode); //zxzxzxzx
	
	// 手柄直接控制，是那么也不做
	if (state == DIRECT_CTRL)
	{
		// process_raw_control(u, u_so3);
		return;
	}	
	// 手柄控制
	else if (state == JS_CTRL)
	{
		// js_ctrl_mode = feedback
		if (js_ctrl_mode==JS_CTRL_MODE_RAW)
		{
			ROS_WARN("[n3ctrl] js_ctrl_mode = JS_CTRL_MODE_RAW");//zxzxzxzx
			process_raw_control(u, u_so3);
		}
		else
		{
			// This function is called when it is running normally. zxzxzxzx
			// js_ctrl_mode = feedback
			// 执行手柄定点控制，（核心！！）
			process_js_control(u, u_so3);
		}
	}
	else if (state == JS_NO_CTRL)
	{
		ROS_WARN("[n3ctrl] state = JS_NO_CTRL");//zxzxzxzx
		process_no_control(u, u_so3);
	}
	else if (state == JS_RESET_POS_CTRL)
	{
		process_break_control(u, u_so3);
	}
	else if (state == CMD_HOVER)
	{
		// CMD控制下悬停控制，（核心！！）
		// 后期将该状态机改为接受定点控制
		process_hover_control(u, u_so3);
	}
	else if (state == CMD_CTRL)
	{
		//ROS_WARN("[n3ctrl] state = CMD_CTRL");//zxzxzxzx
		//  CMD控制下轨迹追踪控制，（核心！！）
		process_cmd_control(u, u_so3);
	}
	else if (state == CMD_NO_CTRL)
	{
		process_no_control(u, u_so3);
	}
	else if (state == CMD_RESET_POS_CTRL)
	{
		process_break_control(u, u_so3);
	}
	else
	{
		ROS_ASSERT(false);
	}

	if (idling_state != NOIDLING)
	{
		double idling_lasting_time = (now_time - idling_start_time).toSec();
		process_idling_control(u, u_so3, idling_lasting_time);
	}

	// 发布u_so3控制指令，无实际作用
	// fsm.controller.ctrl_so3_thrust_pub =nh.advertise<geometry_msgs::WrenchStamped>("ctrl_so3/thrust", 10);
	controller.publish_so3_ctrl(u_so3, now_time);
	// 对u.yaw做了处理，没太搞懂
	align_with_imu(u);
	
	// 发布控制指令，至dji_sdk（核心！！）
	//	fsm.controller.ctrl_pub = nh.advertise<sensor_msgs::Joy>("ctrl", 10);
	controller.publish_ctrl(u, now_time, odom_data.msg.header.stamp);

	// 这一整段是配置油门，但都没使用上，可忽略
	if (idling_state == NOIDLING)
	{
		// 这基本是大部分情况了
		if ((state==JS_CTRL && axis_states[2]==FIX && js_ctrl_mode==JS_CTRL_MODE_FEEDBACK) // JS_CTRL hovering case
			|| (state==CMD_HOVER) // CMD_HOVER case
			|| (state==CMD_CTRL   // CMD_CTRL verticle stationary case
				&& idling_state==NOIDLING // NOT in idling mode
				&& std::fabs(cmd_data.v(2)) < param.hover.vert_velo_limit_for_update
				&& cmd_data.p(2) > param.hover.vert_height_limit_for_update
				&& odom_data.p(2) > param.hover.vert_height_limit_for_update))
		{
#if 0			
			// 默认没启用
			hov_thr_kf.update(imu_data.a(2));

			// This line may not take effect according to param.hov.use_hov_percent_kf
			param.config_full_thrust(hov_thr_kf.get_hov_thr());

			hov_thr_kf.process(u.thrust);
			hov_thr_kf.publish_thr();
#else			
			// 更新悬停油门
			hov_thr_kf.simple_update(imu_data.q, u.thrust, imu_data.a);
			// This line may not take effect according to param.hov.use_hov_percent_kf
			// 设置油门上限，但没使用
			param.config_full_thrust(hov_thr_kf.get_hov_thr());
			hov_thr_kf.publish_thr();
#endif			
		}
		
	}
}

void N3CtrlFSM::process_no_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	u.roll = 0.0;
	u.pitch = 0.0;
	u.yaw = get_yaw_from_odom();
	u.thrust = 0.0;
	u.mode = Controller_Output_t::VERT_VELO;

	u_so3.Rdes = rotz(get_yaw_from_odom());
	u_so3.Fdes = Vector3d(0, 0, param.mass * param.gra);
}

void N3CtrlFSM::process_raw_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	double des_yaw = yaw_add( get_yaw_from_odom(), -(rc_data.yaw * param.rc.yaw_scale));
	
	u.roll  =  (rc_data.roll * param.rc.attitude_scale);
	u.pitch =  (rc_data.pitch * param.rc.attitude_scale);
	u.yaw   = des_yaw;
	u.thrust = rc_data.thr * param.rc.vert_velo_scale;
	u.mode = Controller_Output_t::VERT_VELO;
	u_so3.Rdes = ypr_to_R(Vector3d(u.yaw, u.pitch, u.roll));
	u_so3.Fdes = Vector3d(0, 0, param.mass * (param.gra + u.thrust * 0.5));
}

void N3CtrlFSM::process_idling_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3, double idling_lasting_time)
{
	u.roll = 0.0;
	u.pitch = 0.0;
	u.yaw = get_yaw_from_odom();
	if (idling_lasting_time < param.idling.landing_timeout)
	{
		u.thrust = param.mass * param.gra / param.full_thrust * param.idling.landing_thrust_percent;
	}
	else
	{
		u.thrust = param.idling.lowest_thrust;
	}
	u.mode = Controller_Output_t::VERT_THRU;
}


void N3CtrlFSM::process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	// 将期望位置设定为悬停点
	// des.p = hover_pose.head<3>();
	// des.v = Vector3d::Zero();
	// des.yaw = hover_pose(3);

	if(point_data.get_cmd)
	{
		//改为获取指定目标点
		des.p = point_data.p;
		des.v = Vector3d::Zero();
		des.yaw = point_data.yaw;
	}else
	{
		//改为获取指定目标点
		des.p = point_data.p;
		des.v = Vector3d::Zero();
		des.yaw = point_data.yaw;
	}

	des.a = Vector3d::Zero();

	// 后续改为参数，可以选择不同的控制器
	if(1)
	{
		controller.update(des, odom_data, u, u_so3);
	}else
	{
		controller.pos_controller(des, odom_data, u);
	}
	

	

	publish_desire(des);
}

void N3CtrlFSM::process_break_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = odom_data.p;
	des.v = Vector3d::Zero();
	des.yaw = get_yaw_from_odom();
	des.a = Vector3d::Zero();

	controller.update(des, odom_data, u, u_so3);

	publish_desire(des);
}

void N3CtrlFSM::process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	// cmd_data 是回调函数得到的
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.yaw = cmd_data.yaw;
	des.a = cmd_data.a;

	// 核心代码
	controller.update(des, odom_data, u, u_so3);

	publish_desire(des);	
}

// 手柄定点控制
void N3CtrlFSM::process_js_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	Vector3d des_v;
	double des_dyaw;
	
	get_des_from_js(des_v, des_dyaw);
	des.a = Vector3d::Zero();

	int axis_id;
	for(axis_id=0; axis_id<3; ++axis_id)
	{
		switch(axis_states[axis_id]) // xy axis
		{
			// 定点
			case FIX:
				// hover_pose通过set_hov_with_odom()已设置过
				des.p(axis_id) = hover_pose(axis_id);
				des.v(axis_id) = 0.0;
				break;
			// 定点 + 速度
			case MOVE:
				des.p(axis_id) = odom_data.p(axis_id);
				des.v(axis_id) = des_v(axis_id);
				// ROS_WARN("[n3ctrl] state = MOVE");//zxzxzxzx
				break;
			// 定点
			case BREAK:
				des.p(axis_id) = odom_data.p(axis_id);
				des.v(axis_id) = 0.0;
				break;
			default:
				ROS_ASSERT(false);
		}
	}

	axis_id = 3;
	switch(axis_states[axis_id]) // xy axis
	{
		case FIX:
			des.yaw = hover_pose(axis_id);
			break;
		case MOVE:
			// 当前yaw+期望yaw（偏差）
			des.yaw = yaw_add(get_yaw_from_odom(), des_dyaw);
			break;
		case BREAK:
			ROS_ASSERT(false);
			break;
		default:
			ROS_ASSERT(false);
	}

	// 计算控制指令
	controller.update(des, odom_data, u, u_so3); 

	// 发布 fsm.des_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("desire_pose", 10);
	// 只是用于rviz显示或者debug
	publish_desire(des);
}
