// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"
#include "version.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED

// This is the help function
int8_t Sub::main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Commands:\n"
                         "  logs\n"
                         "  setup\n"
                         "  test\n"
                         "  reboot\n"
                         "\n");
    return(0);
}

// Command/function table for the top-level menu.
const struct Menu::command main_menu_commands[] = {
//   command		function called
//   =======        ===============
    {"logs",                MENU_FUNC(process_logs)},
    {"setup",               MENU_FUNC(setup_mode)},
    {"test",                MENU_FUNC(test_mode)},
    {"reboot",              MENU_FUNC(reboot_board)},
    {"help",                MENU_FUNC(main_menu_help)},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

int8_t Sub::reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
void Sub::run_cli(AP_HAL::UARTDriver *port)
{
    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    // disable main_loop failsafe
    failsafe_disable();

    // cut the engines
    if(motors.armed()) {
        motors.armed(false);
        motors.output();
    }

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED

static void mavlink_delay_cb_static()
{
    sub.mavlink_delay_cb();
}


static void failsafe_check_static()
{
    sub.failsafe_check();
}

void Sub::init_ardupilot()
{
    if (!hal.gpio->usb_connected()) {
        /*
         * http://www.zhihu.com/question/21240031
         * ZigBee就是一种便宜的，低功耗的近距离无线组网通讯技术。
         * XBee模块是美国DIGI的Zigbee模块，XBEE只是型号，是一种远距离低功耗的数传模块，频段有2.4G，915M，868M三种同时可兼容802.15.4协议。
         * 可以理解成XBee模块是基于Zigbee的一种具体产品，Zigbee是一种技术。
         */
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }

    // USB，或者数传
    // initialise serial port
    serial_manager.init_console();

    // cliSerial 是 hal->console 的别名
    cliSerial->printf("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %u\n",
                      (unsigned)hal.util->available_memory());

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    // printf
    report_version();

    // load parameters from EEPROM
    load_parameters();

    BoardConfig.init();

    // initialise serial port
    serial_manager.init();

    // init EPM cargo gripper
#if EPM_ENABLED == ENABLED
    epm.init();
#endif

    // initialise notify system
    // disable external leds if epm is enabled because of pin conflict on the APM
    notify.init(true);

    // initialise battery monitor
    battery.init();

    // Init RSSI
    rssi.init();
    
    barometer.init();

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    ap.usb_connected = true;
    check_usb_mux();

    // setup telem slots with serial ports
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        gcs[i].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, i);
    }

#if FRSKY_TELEM_ENABLED == ENABLED
    // setup frsky
    frsky_telemetry.init(serial_manager);
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    GCS_MAVLINK::set_dataflash(&DataFlash);

    // update motor interlock state
    update_using_interlock();
    
    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs

    // initialise which outputs Servo and Relay events can use
    ServoRelayEvents.set_channel_mask(~motors.get_motor_mask());

    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.init(&DataFlash, serial_manager);

    if(g.compass_enabled)
        init_compass();

#if OPTFLOW == ENABLED
    // make optflow available to AHRS
    ahrs.set_optflow(&optflow);
#endif

    // init Location class
	Location_Class::set_ahrs(&ahrs);
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
	Location_Class::set_terrain(&terrain);
	wp_nav.set_terrain(&terrain);
#endif
	wp_nav.set_avoidance(&avoid);

    pos_control.set_dt(MAIN_LOOP_SECONDS);

    // init the optical flow sensor
    init_optflow();

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(&DataFlash, serial_manager);
#endif

#if PRECISION_LANDING == ENABLED
    // initialise precision landing
    init_precland();
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if CLI_ENABLED == ENABLED
    if (g.cli_enabled) {
        const char *msg = "\nPress ENTER 3 times to start interactive setup\n";
        cliSerial->println(msg);
        if (gcs[1].initialised && (gcs[1].get_uart() != NULL)) {
            gcs[1].get_uart()->println(msg);
        }
        if (num_gcs > 2 && gcs[2].initialised && (gcs[2].get_uart() != NULL)) {
            gcs[2].get_uart()->println(msg);
        }
    }
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_DISABLED
    while (barometer.get_last_update() == 0) {
        // the barometer begins updating when we get the first
        // HIL_STATE message
        gcs_send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
        delay(1000);
    }

    // set INS to HIL mode
    ins.set_hil_mode();
#endif

    if(barometer.num_instances() > 1) { // We have an external MS58XX pressure sensor connected

		barometer.set_primary_baro(1); // Set the primary baro to external MS58XX !!Changes and saves parameter value!!


    	ap.depth_sensor_present = true;
		for(int i = 1; i < barometer.num_instances(); i++) {
			barometer.set_type(i, BARO_TYPE_WATER); // Altitude (depth) is calculated differently underwater
			// TODO:文档都说是10，这里的参数却是40，BUG？
			barometer.set_precision_multiplier(i, 40); // The MS58XX values reported need to be multiplied by 10 to match units everywhere else
		}


		EKF.set_baro_alt_noise(0.1f); // Depth readings are very accurate and up-to-date
		EKF2.set_baro_alt_noise(0.1f);

	} else { //We only have onboard baro

		// No external underwater depth sensor detected
		barometer.set_primary_baro(0); // Set the primary baro to default board baro !!Changes and saves parameter value!!

		ap.depth_sensor_present = false;
		for(int i = 1; i < barometer.num_instances(); i++) {
			barometer.set_type(i, BARO_TYPE_AIR); // Default fcu air baro
			barometer.set_precision_multiplier(i, 1); // Use default values
		}
		EKF.set_baro_alt_noise(10.0f); // Readings won't correspond with rest of INS
		EKF2.set_baro_alt_noise(10.0f);


	}

    leak_detector.init();

	// read Baro pressure at ground
	//-----------------------------
	init_barometer(true);

    // cope with MS5607 in place of MS5611 on fake pixhawks
	if(barometer.get_pressure(0) < 60000) {
		barometer.set_precision_multiplier(0, 2);
		init_barometer(true); // recalibrate with correct scalar
	}

	// backwards compatibility
	if(attitude_control.get_accel_yaw_max() < 110000.0f) {
		attitude_control.save_accel_yaw_max(110000.0f);
	}

	last_pilot_heading = ahrs.yaw_sensor;

    // initialise rangefinder
#if RANGEFINDER_ENABLED == ENABLED
    init_rangefinder();
#endif

    // initialise AP_RPM library
    rpm_sensor.init();

    // initialise mission library
    mission.init();

    // initialise the flight mode and aux switch
    // ---------------------------
    reset_control_switch();
    init_aux_switches();

    startup_INS_ground();

    // set landed flags
    set_land_complete(false);

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    failsafe_enable();

    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    ins.set_dataflash(&DataFlash);

    // init vehicle capabilties
    init_capabilities();

    cliSerial->print("\nReady to FLY ");

    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Sub::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// calibrate gyros - returns true if succesfully calibrated
bool Sub::calibrate_gyros()
{
    // gyro offset calibration
    sub.ins.init_gyro();

    // reset ahrs gyro bias
    if (sub.ins.gyro_calibrated_ok_all()) {
        sub.ahrs.reset_gyro_drift();
        return true;
    }

    return false;
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Sub::position_ok()
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_position_ok() || optflow_position_ok());
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Sub::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors.armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Sub::optflow_position_ok()
{
#if OPTFLOW != ENABLED
    return false;
#else
    // return immediately if optflow is not enabled or EKF not used
    if (!optflow.enabled() || !ahrs.have_inertial_nav()) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal relative position
    if (!motors.armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    } else {
        return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    }
#endif
}

// update_auto_armed - update status of auto_armed flag
void Sub::update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors.armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
    }else{
        // arm checks
        // if motors are armed and throttle is above zero auto_armed should be true
        if(motors.armed()) {
            set_auto_armed(true);
        }
    }
}

void Sub::check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == ap.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    ap.usb_connected = usb_check;
}

// frsky_telemetry_send - sends telemetry data using frsky telemetry
//  should be called at 5Hz by scheduler
#if FRSKY_TELEM_ENABLED == ENABLED
void Sub::frsky_telemetry_send(void)
{
    frsky_telemetry.send_frames((uint8_t)control_mode);
}
#endif

/*
  should we log a message type now?
 */
bool Sub::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = motors.armed() || DataFlash.log_while_disarmed();
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        start_logging();
    }
    return ret;
#else
    return false;
#endif
}
