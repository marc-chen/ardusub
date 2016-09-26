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

    // initialize vertical speeds and leash lengths
    // sets the maximum speed up and down returned by position controller
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    gcs_send_text_fmt(MAV_SEVERITY_DEBUG, "alt check set speed z: up %f down %f",
            pos_control.get_speed_up(),
            pos_control.get_speed_down());

    pos_control.set_accel_z(g.pilot_accel_z);
    gcs_send_text_fmt(MAV_SEVERITY_DEBUG, "alt check set sccel z: %f",
                pos_control.get_accel_z());

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

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{
	uint32_t tnow = AP_HAL::millis();

	// 这里为何一直初始化？为了保险起见？
    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    if(!motors.armed() || !motors.get_interlock()) {

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

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // 没有等比缩放，而是直接截断，保持油门的线性。z速度限制较小时，超过的油门被截断
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

	// call attitude controller
	if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
		attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
		last_pilot_heading = ahrs.yaw_sensor;
		last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

	} else { // hold current heading

	    // 定深模式下，方向好像是自动保持的

		// this check is required to prevent bounce back after very fast yaw maneuvers
		// the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
		if(tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
			target_yaw_rate = 0; // Stop rotation on yaw axis

	        // 调整方向之后会持续250ms影响方向，然后才会固定下来自动保持

			// call attitude controller with target yaw rate = 0 to decelerate on yaw axis
			attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
			last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

		} else { // call attitude controller holding absolute absolute bearing
			attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true, get_smoothing_gain());
		}
	}

	// 难道是快到水面时，自动调整速度？
	// adjust climb rate using rangefinder
	if (rangefinder_alt_ok()) {
		// if rangefinder is ok, use surface tracking
		target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
	}

	// BUG修正：这里应该是ap.at_surface，而不是ap.at_bottom，难怪在快接近水面时马达总是在转
	// call z axis position controller
	if(ap.at_surface) {
		pos_control.relax_alt_hold_controllers(0.0); // clear velocity and position targets, and integrator
		pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
	} else {
		if(inertial_nav.get_altitude() < g.surface_depth) { // pilot allowed to move up or down freely
			pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
		} else if(target_climb_rate < 0) { // pilot allowed to move only down freely
			if(pos_control.get_vel_target_z() > 0) {
				pos_control.relax_alt_hold_controllers(0); // reset target velocity and acceleration
			}
			pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
		} else if(pos_control.get_alt_target() > g.surface_depth) { // hold depth at surface level.
			pos_control.set_alt_target(g.surface_depth);
		}
	}

	/*
	 * 曾经考虑过浮上水面之后，如果再次下潜，就重新自动设置当前方向为新的方向，不过好像没什么必要
	 * 而且如果手工控制转向之后就会自动重新标定方向
	 */

	pos_control.update_z_controller();

	// 前进和后退动作不处理，直接传递给电机

    //control_in is range 0-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
