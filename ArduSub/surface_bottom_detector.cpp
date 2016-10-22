/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
// Code by Jacob Walser: jwalser90@gmail.com

#include "Sub.h"

// counter to verify contact with bottom
static uint32_t bottom_detector_count = 0;
static uint32_t surface_detector_count = 0;
static float current_depth = 0;

// checks if we have have hit bottom or surface and updates the ap.at_bottom and ap.at_surface flags
// called at MAIN_LOOP_RATE
// ToDo: doesn't need to be called this fast
void Sub::update_surface_and_bottom_detector()
{
	if(!motors.armed()) { // only update when armed
		set_surfaced(false);
		set_bottomed(false);
		return;
	}

	Vector3f velocity;
	ahrs.get_velocity_NED(velocity);

	/*
	 * 用速度来判断是否在z轴方向稳定
	 * 看来定深的最大速度必须超过5cm/s，否则容易引起误判
	 */
	// check that we are not moving up or down
	bool vel_stationary = velocity.z > -0.05 && velocity.z < 0.05;

	// 有深度传感器时，直接读深度来判断是否在水面
	if (ap.depth_sensor_present) { // we can use the external pressure sensor for a very accurate and current measure of our z axis position
		current_depth = barometer.get_altitude(); // m


		if(ap.at_surface) {
		    // 已经在水面时，阈值放宽5cm，避免抖动。有可能马上就进入非水面状态，太敏感了
			set_surfaced(current_depth > g.surface_depth/100.0 - 0.05); // add a 5cm buffer so it doesn't trigger too often
		} else {
		    // 小于阈值（10cm）即认为是水面了
		    /*
			set_surfaced(current_depth > g.surface_depth/100.0); // If we are above surface depth, we are surfaced
			*/
		    // 水面检测，增加延时判断，避免拉动
		    if (current_depth > g.surface_depth/100.0) {
	            if( surface_detector_count < ((float)SURFACE_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
	                surface_detector_count++;
	            } else {
	                set_surfaced(true);
	            }
		    } else {
                surface_detector_count = 0;
		        set_surfaced(false);
		    }
		}


		// 传感器对于底部判断，没有帮助
		// 到达水底的条件：油门向下最大持续BOTTOM_DETECTOR_TRIGGER_SEC秒且速度基本不变
		if(motors.limit.throttle_lower && vel_stationary) {
			// bottom criteria met - increment the counter and check if we've triggered
			if( bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
				bottom_detector_count++;
			} else {
				set_bottomed(true);
			}

		} else {
            bottom_detector_count = 0;
            // TODO: 离开 bottom 太快了 ?
		    set_bottomed(false);
		}

	// with no external baro, the only thing we have to go by is a vertical velocity estimate
	} else if (vel_stationary) {

	    // 没有深度传感器，只能根据相对状态来判断，有可能误判

	    /*
	     * 在水面的判定条件：
	     *   上下速度不超过5cm/s
	     *   油门向上最大
	     *   这种状态累计超过1秒（BUG？改为持续1秒是不是更合理）
	     */
		if(motors.limit.throttle_upper) {

			// surface criteria met, increment counter and see if we've triggered
			if( surface_detector_count < ((float)SURFACE_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
				surface_detector_count++;
			} else {
				set_surfaced(true);
			}

		} else if(motors.limit.throttle_lower) {
			// bottom criteria met, increment counter and see if we've triggered
			if( bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
				bottom_detector_count++;
			} else {
				set_bottomed(true);
			}

		} else { // we're not at the limits of throttle, so reset both detectors
			set_surfaced(false);
			set_bottomed(false);
		}

	} else { // we're moving up or down, so reset both detectors
		set_surfaced(false);
		set_bottomed(false);
	}
}

void Sub::set_surfaced(bool at_surface) {

	if(ap.at_surface == at_surface) { // do nothing if state unchanged
		return;
	}

	ap.at_surface = at_surface;

	if(!ap.at_surface) {
	    Log_Write_Event(DATA_SURFACED);
		gcs_send_text(MAV_SEVERITY_INFO, "Off Surface");
	} else {
		surface_detector_count = 0;
		Log_Write_Event(DATA_NOT_SURFACED);
		gcs_send_text(MAV_SEVERITY_INFO, "Surfaced");
	}
}

void Sub::set_bottomed(bool at_bottom) {

	if(ap.at_bottom == at_bottom) { // do nothing if state unchanged
		return;
	}

	ap.at_bottom = at_bottom;

	if(!ap.at_bottom) {
		Log_Write_Event(DATA_BOTTOMED);
		gcs_send_text(MAV_SEVERITY_INFO, "Off Bottom");
	} else {
		bottom_detector_count = 0;
		Log_Write_Event(DATA_NOT_BOTTOMED);
		gcs_send_text(MAV_SEVERITY_INFO, "Bottomed");
	}
}
