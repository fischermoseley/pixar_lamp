def print_odrive_config(odrv, axis):
    print(f"max current: {axis.motor.config.current_lim}")
    print(f"regen current: {odrv.config.dc_max_negative_current}")
    print(f"calibration current: {axis.motor.config.calibration_current}")
    print(f"max velocity: {axis.controller.config.vel_limit}")
    print(f"pole pairs: {axis.motor.config.pole_pairs}")
    print(f"torque constant: {axis.motor.config.torque_constant}")
    print(f"motor type: {axis.motor.config.motor_type}")
    print(f"encoder cpr: {axis.encoder.config.cpr}")

def configure_odrive(axis):
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    sleep(5)
    assert(axis.motor.is_calibrated)

    axis.encoder.config.use_index = True
    sleep(5)
    assert(axis.error == 0)

    axis.encoder.config.pre_calibrated = True
    axis.config.startup_encoder_index_search = True
    axis.motor.config.pre_calibrated = True