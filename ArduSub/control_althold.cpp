/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Sub.h"


/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Sub::althold_init(bool ignore_checks)
{
	if(!ap.depth_sensor_present) { // can't hold depth without a depth sensor, exit immediately.
		gcs_send_text(MAV_SEVERITY_WARNING, "Depth hold requires external pressure sensor.");
		return false;
	}

	// 配置中的PILOT_VELZ_MAX，最大速度
    // initialize vertical speeds and leash lengths
    // sets the maximum speed up and down returned by position controller
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    gcs_send_text_fmt(MAV_SEVERITY_DEBUG, "alt check set speed z: up %f down %f",
            pos_control.get_speed_up(),
            pos_control.get_speed_down());

    // 配置中的PILOT_ACCEL_Z，加速度
    pos_control.set_accel_z(g.pilot_accel_z);
    gcs_send_text_fmt(MAV_SEVERITY_DEBUG, "alt check set sccel z: %f",
                pos_control.get_accel_z());


    // ?? 这里不何不直接调用relax_alt_hold_controllers?


    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    gcs_send_text_fmt(MAV_SEVERITY_DEBUG, "alt check set alt target: %f",
                pos_control.get_alt_target());

    gcs_send_text_fmt(MAV_SEVERITY_DEBUG, "inertial_nav.get_velocity_z: %f",
            inertial_nav.get_velocity_z());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());
    gcs_send_text_fmt(MAV_SEVERITY_DEBUG, "check set_desired_velocity_z: %f",
                pos_control.get_desired_velocity().z);

    // stop takeoff if running
    takeoff_stop();

    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

/*
 * TODO:
 * 1. 默认运行频率400HZ,可以降低一些
 * 2. 水面使用ap.at_surface，而不是现在的inertial_nav.get_altitude() < g.surface_depth
 *
 */
// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{
	uint32_t tnow = AP_HAL::millis();

	/*
	 * 每次都要初始化一次，可能是考虑到在运行过程中，还是可以通过地面站来调整参数
	 * TODO: 参数怎么调，需要测试，默认值(都是50)过于激烈。另外传感器的精度也影响效果吧
	 * 加速度好像是固定值，速度是一个范围
	 */
    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    /*
     * arm 居然是马达的状态，而不是系统的。想想也是，系统通电后就开始工作了
     */
    // 未解锁 或 马达没转
    if(!motors.armed() || !motors.get_interlock()) {

        // 不用解锁，万达就开始转？
        // 未解锁状态进入定深模式时，马达会低速转动，用以警示
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);

        // 未解锁时，暂时禁用自稳
        // Multicopters do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // 未解锁前，所以状态值都是读当前值，比如深度、方向
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-motors.get_throttle_hover());
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }


    // 马达完全放开
	motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

	/*
	 * simple模式，让方向控制更简单
	 * 但对于水下，看不到设备，可以加一个一键回中功能，即自动旋转到初始方向，避免绕线
	 */

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // 默认的控制，没有为roll设置按键
    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // 让之前设置的最大速度生效：没有等比缩放，而是直接截断，保持油门的线性。z速度限制较小时，超过的油门被截断
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    /*
     * 定深下的姿态保持，查代码发现pos_control中引用attitude_control很少，传入的参数也很少，可能能力非常有限
     * TODO: 能自动定向？
     * TODO: 转弯时，能自动调整roll吗？如果有点倾斜，转变的效果可能会更好，降低侧翻的风险。四轴不知有没有，可以测试一下
     */
	// call attitude controller
	if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
	    // 正在转向
	    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
		last_pilot_heading = ahrs.yaw_sensor;
		last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

	} else { // hold current heading

		// this check is required to prevent bounce back after very fast yaw maneuvers
		// the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
		if(tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
			target_yaw_rate = 0; // Stop rotation on yaw axis

	        // 转向操作后，再等250ms，然后才会固定下来自动保持
			// call attitude controller with target yaw rate = 0 to decelerate on yaw axis
			attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
			last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

		} else { // call attitude controller holding absolute absolute bearing
			// 没有转向，或者转向结束超过250ms
		    attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true, get_smoothing_gain());
		}
	}

	// 难道是快到水面时，自动调整速度？
	// adjust climb rate using rangefinder
	if (rangefinder_alt_ok()) {
		// if rangefinder is ok, use surface tracking
		target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
	}

	// 到达水底的处理
	// call z axis position controller
	if(ap.at_bottom) {
		// 如果到达底部了，则重新设置目标深度为底部上面10cm
	    pos_control.relax_alt_hold_controllers(0.0); // clear velocity and position targets, and integrator
		pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
	} else {
		// ? 这里表面相关的逻辑，是不是应该使用 ap.at_surface ?
	    if(inertial_nav.get_altitude() < g.surface_depth) { // pilot allowed to move up or down freely
			// 绝大部分都走这个分支
	        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
		} else if(target_climb_rate < 0) { // pilot allowed to move only down freely
			if(pos_control.get_vel_target_z() > 0) {
				// 已经到达水面，油门向下，但实际还在向上走。此时从当前状态重新设定参数
			    pos_control.relax_alt_hold_controllers(0); // reset target velocity and acceleration
			}
			pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
		} else if(pos_control.get_alt_target() > g.surface_depth) { // hold depth at surface level.
			// 如果已经在水面了，则保持在水面深度
		    pos_control.set_alt_target(g.surface_depth);
		}
	}

	/*
	 * 曾经考虑过浮上水面之后，如果再次下潜，就重新自动设置当前方向为新的方向，不过好像没什么必要
	 * 而且如果手工控制转向之后就会自动重新标定方向
	 */

	pos_control.update_z_controller();

	// TODO: 没有看到roll控制的输出，估计没有

	// 前进和后退动作不处理，直接传递给电机

    //control_in is range 0-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
