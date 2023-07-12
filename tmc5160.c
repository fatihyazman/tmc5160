/****************************************************************************/
/* File Name    : tmc5160.h                                                 */
/* Description  : TMC5160 stepper motor driver implementation file          */
/* Author       : Hasan KAYNAR                                              */
/* Initial Date : 27.12.2022                                                */
/* Updated Date : 29.03.2023                                                */
/* Version      : 1.3                                                       */
/****************************************************************************/

/*--------------------------------------------------------------------------*/
/* Includes                                                                 */
/*--------------------------------------------------------------------------*/
#include "tmc5160.h"
#include "string.h"

/*--------------------------------------------------------------------------*/
/* TMC5160 module structure                                                 */
/*--------------------------------------------------------------------------*/
static tmc5160_module_struct_t tmc5160[TMC5160_MOTOR_COUNT];

/**
 * \brief                       Default register access
 * \note                        Register access permissions
 *                              0x00: none (reserved)
 *                              0x01: read
 *                              0x02: write
 *                              0x03: read/write
 *                              0x13: read/write, seperate functions/values for reading or writing
 *                              0x23: read/write, flag register (write to clear)
 *                              0x42: write, has hardware presets on reset
 */
static const uint8_t tmc5160_default_reg_access[TMC5160_REG_COUNT] =
{
/*   0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F   */
	0x03, 0x23, 0x01, 0x02, 0x13, 0x02, 0x02, 0x01, 0x03, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00, 0x00,     /* 0x00 - 0x0F */
	0x02, 0x02, 0x01, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     /* 0x10 - 0x1F */
	0x03, 0x03, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x02, 0x02, 0x02, 0x03, 0x00, 0x00,     /* 0x20 - 0x2F */
	0x00, 0x00, 0x00, 0x02, 0x03, 0x23, 0x01, 0x00, 0x03, 0x03, 0x02, 0x23, 0x01, 0x02, 0x00, 0x00,     /* 0x30 - 0x3F */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     /* 0x40 - 0x4F */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     /* 0x50 - 0x5F */
	0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x01, 0x01, 0x03, 0x02, 0x02, 0x01,     /* 0x60 - 0x6F */
	0x42, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     /* 0x70 - 0x7F */
};

/**
 * \brief                       Init the TMC5160
 * \param[in] comm_mode         Communication mode.
 * \param[in] channel           Motor channel information.
 * \param[in] reset_state       An int32_t array with 128 elements. This holds the values to be used for a reset.
 * \param[in] write_wrapper     Pass in this parameter the address of the wrapped function with the microcontroller specific spi transmit function.
 * \param[in] read_wrapper      Pass in this parameter the address of the wrapped function with the microcontroller specific spi receive function.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 1
results_enum_t tmc5160_init(tmc5160_comm_mode_enum_t comm_mode, tmc5160_channels_enum_t channel, tmc5160_spi_write_fn tmc5160_spi_write_wrapper, tmc5160_spi_read_fn tmc5160_spi_read_wrapper)
{
	results_enum_t return_value = RESULT_SUCCESS;

	memset(&tmc5160[(uint8_t)channel], 0, sizeof(tmc5160[(uint8_t)channel]));

	/* General Configuration Register Address Loading */
	tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.addr          = 0x00;      /* RW   */
	tmc5160[(uint8_t)channel].gconf_regs.gstat_reg.addr          = 0x01;      /* R+WC */
	tmc5160[(uint8_t)channel].gconf_regs.io_input_reg.addr       = 0x04;      /* R    */
	tmc5160[(uint8_t)channel].gconf_regs.xcomp_reg.addr          = 0x05;      /* W    */
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.addr = 0x06;      /* W    */
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.addr  = 0x07;      /* R    */
	tmc5160[(uint8_t)channel].gconf_regs.factory_conf_reg.addr   = 0x08;      /* RW   */
	tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.addr     = 0x09;      /* W    */
	tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.addr       = 0x0A;      /* W    */
	tmc5160[(uint8_t)channel].gconf_regs.global_scaler_reg.addr  = 0x0B;      /* W    */
	tmc5160[(uint8_t)channel].gconf_regs.offset_read_reg.addr    = 0x0C;      /* R    */

	/* Velocity Dependent Driver Feature Control Register Address Loading */
	tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.addr  = 0x10;      /* W    */
	tmc5160[(uint8_t)channel].vel_regs.tpower_down_reg.addr = 0x11;      /* W    */
	tmc5160[(uint8_t)channel].vel_regs.tstep_reg.addr       = 0x12;      /* R    */
	tmc5160[(uint8_t)channel].vel_regs.tpwmthrs_reg.addr    = 0x13;      /* W    */
	tmc5160[(uint8_t)channel].vel_regs.tcoolthrs_reg.addr   = 0x14;      /* W    */
	tmc5160[(uint8_t)channel].vel_regs.thigh_reg.addr       = 0x15;      /* W    */

	/* Ramp Generator Motion Control Register Address Loading */
	tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_mode_reg.addr  = 0x20;      /* RW   */
	tmc5160[(uint8_t)channel].ramp_gen_regs.xactual_reg.addr    = 0x21;      /* RW   */
	tmc5160[(uint8_t)channel].ramp_gen_regs.vactual_reg.addr    = 0x22;      /* R    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.vstart_reg.addr     = 0x23;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.a1_reg.addr         = 0x24;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.v1_reg.addr         = 0x25;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.amax_reg.addr       = 0x26;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.vmax_reg.addr       = 0x27;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.dmax_reg.addr       = 0x28;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.d1_reg.addr         = 0x2A;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.vstop_reg.addr      = 0x2B;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.tzero_wait_reg.addr = 0x2C;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.xtarget_reg.addr    = 0x2D;      /* RW   */
	tmc5160[(uint8_t)channel].ramp_gen_regs.vdc_min_reg.addr    = 0x33;      /* W    */
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr    = 0x34;      /* RW   */
	tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.addr  = 0x35;      /* R+WC */
	tmc5160[(uint8_t)channel].ramp_gen_regs.xlatch_reg.addr     = 0x36;      /* R    */

	/* Encoder Register Address Loading */
	tmc5160[(uint8_t)channel].enc_regs.enc_mode_reg.addr      = 0x38;      /* RW   */
	tmc5160[(uint8_t)channel].enc_regs.xenc_reg.addr          = 0x39;      /* RW   */
	tmc5160[(uint8_t)channel].enc_regs.enc_const_reg.addr     = 0x3A;      /* W    */
	tmc5160[(uint8_t)channel].enc_regs.enc_status_reg.addr    = 0x3B;      /* R+WC */
	tmc5160[(uint8_t)channel].enc_regs.enc_latch_reg.addr     = 0x3C;      /* R    */
	tmc5160[(uint8_t)channel].enc_regs.enc_deviation_reg.addr = 0x3D;      /* W    */

	/* Microstepping Control Register Address Loading */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslut0_reg.addr     = 0x60;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslut1_reg.addr     = 0x61;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslut2_reg.addr     = 0x62;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslut3_reg.addr     = 0x63;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslut4_reg.addr     = 0x64;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslut5_reg.addr     = 0x65;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslut6_reg.addr     = 0x66;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslut7_reg.addr     = 0x67;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslutsel_reg.addr   = 0x68;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mslutstart_reg.addr = 0x69;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mscnt_reg.addr      = 0x6A;      /* R    */
	tmc5160[(uint8_t)channel].motor_drv_regs.mstep_control.mscuract_reg.addr   = 0x6B;      /* R    */

	/* Driver Register Address Loading */
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr  = 0x6C;      /* RW   */
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.addr  = 0x6D;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.dc_ctrl_reg.addr    = 0x6E;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr = 0x6F;      /* R    */
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr   = 0x70;      /* W    */
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_scale_reg.addr  = 0x71;      /* R    */
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_auto_reg.addr   = 0x72;      /* R    */
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.lost_steps_reg.addr = 0x73;      /* R    */

	tmc5160[(uint8_t)channel].state             = TMC5160_CONFIG_READY;
	tmc5160[(uint8_t)channel].spi_write_wrapper = tmc5160_spi_write_wrapper;
	tmc5160[(uint8_t)channel].spi_read_wrapper  = tmc5160_spi_read_wrapper;

	for (uint8_t i = 0; i < TMC5160_REG_COUNT; ++i)
	{
		tmc5160[(uint8_t)channel].reg_access[i] = tmc5160_default_reg_access[i];
	}

	switch (comm_mode)
	{
	case TMC5160_COMM_DEFAULT:

	    /*
	     *
	     * ToDo:
	     *
	     */

		break;

	case TMC5160_COMM_SPI:

	    /*
	     *
	     * ToDo:
	     *
	     */

		break;

	case TMC5160_COMM_UART:
		/* This part will be left blank */
		break;
	}

	return_value = tmc5160_reset(channel);
	tmc5160_factory_default_settings(channel);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Determines the status of the drive.
 * \param[in] channel           Motor channel information.
 * \param[in] driver_state      TMC5160_DRIVER_ENABLE:  The driver becomes enable.
 *                              TMC5160_DRIVER_DISABLE: The driver becomes disable.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 2
bool tmc5160_driver_state(tmc5160_channels_enum_t channel, tmc5160_driver_state_enum_t driver_state)
{
	if (driver_state == TMC5160_DRIVER_ENABLE)
	{
		return TMC5160_LOGIC_LOW;
	}
	else
	{
		return TMC5160_LOGIC_HIGH;
	}
}

/**
 * \brief                       Reset the TMC5160
 * \param[in] channel           Motor channel information.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 3
results_enum_t tmc5160_reset(tmc5160_channels_enum_t channel)
{
	results_enum_t return_value = RESULT_SUCCESS;

	if (tmc5160[(uint8_t)channel].state != TMC5160_CONFIG_READY)
	{
		return_value = RESULT_NOT_FOR_ME;
	}

	/* Reset the dirty bits and wipe the shadow registers */
	for (uint8_t i = 0; i < TMC5160_REG_COUNT; ++i)
	{
		tmc5160[(uint8_t)channel].reg_access[i] &= ~TMC5160_ACCESS_DIRTY;
	}

	tmc5160[(uint8_t)channel].state = TMC5160_CONFIG_RESET;

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Restore the TMC5160 to the state stored in the shadow registers. This can be used to recover the IC configuration after a VM power loss.
 * \param[in] channel           Motor channel information.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 4
results_enum_t tmc5160_restore(tmc5160_channels_enum_t channel)
{
	results_enum_t return_value = RESULT_SUCCESS;

	if (tmc5160[(uint8_t)channel].state != TMC5160_CONFIG_READY)
	{
		return_value = RESULT_NOT_FOR_ME;
	}

	tmc5160[(uint8_t)channel].state = TMC5160_CONFIG_RESTORE;

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Write function with spi.
 * \param[in] channel           Motor channel information.
 * \param[in] addr              Register address to write.
 * \param[in] value             Value to write.
 * \param[in] timeout           Timeout period set for spi.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 5
results_enum_t tmc5160_spi_write_reg(tmc5160_channels_enum_t channel, uint8_t addr, int32_t value, uint32_t timeout)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t data[TMC5160_DATA_SIZE];

	data[0] = addr | TMC5160_WRITE_BIT;
	data[1] = (value >> 24) & 0xFF;
	data[2] = (value >> 16) & 0xFF;
	data[3] = (value >> 8) & 0xFF;
	data[4] = value & 0xFF;

	return_value = tmc5160[(uint8_t)channel].spi_write_wrapper(channel, data, TMC5160_DATA_SIZE, timeout);

	/* Write to the mark the register dirty */
	addr = TMC5160_ADDRESS(addr);
	tmc5160[(uint8_t)channel].reg_access[addr] |= TMC5160_ACCESS_DIRTY;

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Read function with spi.
 * \param[in] channel           Motor channel information.
 * \param[in] addr              Register address to be read.
 * \param[in] timeout           Timeout period set for spi.
 * \retval                      Value read from register.
 */
int32_t tmc5160_spi_read_reg(tmc5160_channels_enum_t channel, uint8_t addr, uint32_t timeout)
{
	uint8_t data[TMC5160_DATA_SIZE] = { 0 };

	/* Remove write access bit first */
	addr = TMC5160_ADDRESS(addr);
	data[0] = addr;
	tmc5160[(uint8_t)channel].spi_read_wrapper(channel, data, TMC5160_DATA_SIZE, timeout);

	return ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | (data[3] << 8) | data[4];
}

/**
 * \brief                       Call this function in a timer callback function that will be called every 1ms.
 * \param[in] channel           Motor channel information.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 7
results_enum_t tmc5160_timer_callback(tmc5160_channels_enum_t channel)
{
	results_enum_t return_value = RESULT_SUCCESS;

	static uint16_t tick = 0;

	if (tmc5160[(uint8_t)channel].state != TMC5160_CONFIG_READY)
	{
		return_value = RESULT_NOT_FOR_ME;
	}

	if ((tick += TMC5160_TIMER_PERIOD) > TMC5160_WORK_TIME)
	{
		/* ToDo: */
//		int32_t xactual = tmc5160_get_ramp_xactual(channel);
		tick = 0;
	}

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Loop function that needs to be called in the main loop.
 * \param[in] channel           Motor channel information.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 8
results_enum_t tmc5160_loop(tmc5160_channels_enum_t channel)
{
	results_enum_t return_value = RESULT_SUCCESS;

	/* Calculate velocity v = dx/dt */
	/* ToDo: */

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set the pin connected to the button as an external interrupt and call this function there.
 * \param[in] channel           Motor channel information.
 * \param[in] enable            0: Normal operation.
 *                              1: Emergency stop: ENCA_DCIN stops the sequencer when tied high (no steps become executed by the sequencer, motor goes to standstill state).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 9
results_enum_t tmc5160_emergency_stop_callback(tmc5160_channels_enum_t channel, bool enable)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.stop_enable = enable;
	value = tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Factory reset.
 * \param[in] channel           Motor channel information.
 * \retval                      The success or error value of the function.
 */
void tmc5160_factory_default_settings(tmc5160_channels_enum_t channel)
{


	tmc5160_set_gconf_multistep_filt(channel, true);
	tmc5160_set_gconf_pwm_mode(channel, true);

	tmc5160_set_shortconf_s2vs_level(channel, 6);
	tmc5160_set_shortconf_s2g_level(channel, 6);
	tmc5160_set_shortconf_short_filter(channel, 1);
	tmc5160_set_shortconf_short_delay(channel, 0);

	tmc5160_set_drv_bbmtime(channel, 0);
	tmc5160_set_drv_bbmclks(channel, 4);
	tmc5160_set_drv_otselect(channel, 0);  /* 150 C */

	tmc5160_set_drv_strength(channel, 2);
	tmc5160_set_drv_filt_isense(channel, 0);

	tmc5160_set_global_scaler(channel, 0x62);
	tmc5160_set_ihold(channel, 2);
	tmc5160_set_irun(channel, 17);
	tmc5160_set_ihold_delay(channel, 0);

	tmc5160_set_tpower_down(channel, 10);
	tmc5160_set_enc_res(channel, 0x00010000);

	tmc5160_set_chopper_toff(channel, 3);
	tmc5160_set_chopper_hstrt_sine_offset(channel, 5);
	tmc5160_set_chopper_hend_fd_time(channel, 2);
	tmc5160_set_chopper_blank_time(channel, 2);
	tmc5160_set_chopper_passive_fd_time(channel, 4);
	tmc5160_set_chopper_microstep_res(channel, MICROSTEP_256);

	tmc5160_set_sw_mode_en_softstop(channel, 1);

	tmc5160_set_pwm_ofs_amplitude(channel, 0x1E);
	tmc5160_set_pwm_gradient(channel, 0);
	tmc5160_set_pwm_frequency(channel, 1);  /* ~35kHz */

	tmc5160_set_pwm_autoscale(channel, true);
	tmc5160_set_pwm_autograd(channel, true);
	tmc5160_set_pwm_freewheeling_mode(channel, 1);
	tmc5160_set_pwm_regulation(channel, 4);
	tmc5160_set_pwm_limit(channel, 0xC);

	tmc5160_set_ramp_vstart(channel, 100);
	tmc5160_set_ramp_vstop(channel, 10000);
	tmc5160_set_ramp_vmax(channel, 100000);
	tmc5160_set_ramp_amax(channel, 1000);
	tmc5160_set_ramp_dmax(channel, 2000);
	tmc5160_set_ramp_a1(channel, 900);
	tmc5160_set_ramp_d1(channel, 1000);
	tmc5160_set_ramp_type(channel, TMC5160_MODE_POSITION);


}

/**
 * \brief                       Set move to.
 * \param[in] channel           Motor channel information.
 * \param[in] position          Target location parameter.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 11
results_enum_t tmc5160_set_move_to(tmc5160_channels_enum_t channel, int32_t position)
{
	results_enum_t return_value = RESULT_SUCCESS;

	tmc5160_set_ramp_type(channel, TMC5160_MODE_POSITION);
	tmc5160_set_ramp_xtarget(channel, position);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set move by.
 * \param[in] channel           Motor channel information.
 * \param[in] ticks             Encloses target location in tick parameter.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 12
results_enum_t tmc5160_set_move_by(tmc5160_channels_enum_t channel, int32_t *ticks)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.xactual_reg.addr;
	*ticks += tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);

	tmc5160_set_move_to(channel, *ticks);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set otp programming.
 * \param[in] channel           Motor channel information.
 * \param[in] otp_prog          Set write access programs OTP memory (one bit at a time), read access refreshes read data from OTP after a write.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 13
results_enum_t tmc5160_set_otp_prog(tmc5160_channels_enum_t channel, uint16_t otp_prog)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.otp_write_pins.reserved = otp_prog;
	return_value = tmc5160_spi_write_reg(channel, addr, otp_prog, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Selection of OTP bit to be programmed to the selected byte location.
 * \param[in] channel           Motor channel information.
 * \param[in] otp_bit           n = 0...7: programs bit n to a logic 1.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 14
results_enum_t tmc5160_set_otp_prog_bit(tmc5160_channels_enum_t channel, uint8_t otp_bit)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.otp_write_pins.otp_bit = otp_bit;
	reserved = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.otp_write_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, reserved, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set otp byte.
 * \param[in] channel           Motor channel information.
 * \param[in] otp_byte          Set to 00
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 15
results_enum_t tmc5160_set_otp_prog_byte(tmc5160_channels_enum_t channel, uint8_t otp_byte)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.otp_write_pins.otp_byte = otp_byte;
	reserved = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.otp_write_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, reserved, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set otp magic.
 * \param[in] channel           Motor channel information.
 * \param[in] otp_magic         Set to 0xBD to enable programming. A programming time of minimum 10ms per bit is recommended (check by reading OTP_READ).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 16
results_enum_t tmc5160_set_otp_prog_magic(tmc5160_channels_enum_t channel, uint8_t otp_magic)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.otp_write_pins.otp_magic = otp_magic;
	reserved = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_write_reg.otp_write_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, reserved, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set target position.
 * \param[in] channel           Motor channel information.
 * \param[in] xtarget           The maximum possible displacement is +/-((2^31)-1).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 17
results_enum_t tmc5160_set_ramp_xtarget(tmc5160_channels_enum_t channel, int32_t xtarget)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.xtarget_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.xtarget_reg.xtarget = xtarget;
	return_value = tmc5160_spi_write_reg(channel, addr, xtarget, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set actual position.
 * \param[in] channel           Motor channel information.
 * \param[in] xactual           (-2^31) ... (+2^31-1) Hint: This value normally should only be modified, when homing the drive.
 *                              In positioning mode, modifying the register content will start a motion.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 18
results_enum_t tmc5160_set_ramp_xactual(tmc5160_channels_enum_t channel, int32_t xactual)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.xactual_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.xactual_reg.xactual = xactual;
	return_value = tmc5160_spi_write_reg(channel, addr, xactual, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set maximum speed.
 * \param[in] channel           Motor channel information.
 * \param[in] vmax              Motion ramp target velocity (for positioning ensure VMAX ≥ VSTART) (unsigned).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 19
results_enum_t tmc5160_set_ramp_vmax(tmc5160_channels_enum_t channel, uint32_t vmax)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.vmax_reg.addr;

	if (vmax > TMC5160_MAX_VELOCITY)
	{
		tmc5160[(uint8_t)channel].ramp_gen_regs.vmax_reg.vmax = TMC5160_MAX_VELOCITY;
		return_value = tmc5160_spi_write_reg(channel, addr, TMC5160_MAX_VELOCITY, TMC5160_SPI_TIMEOUT);
	}
	else
	{
		tmc5160[(uint8_t)channel].ramp_gen_regs.vmax_reg.vmax = vmax;
		return_value = tmc5160_spi_write_reg(channel, addr, vmax, TMC5160_SPI_TIMEOUT);
	}

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       This is the acceleration and deceleration value for velocity mode.
 * \param[in] channel           Motor channel information.
 * \param[in] amax              Second acceleration between V1 and VMAX.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 20
results_enum_t tmc5160_set_ramp_amax(tmc5160_channels_enum_t channel, uint16_t amax)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.amax_reg.addr;

	if (amax > TMC5160_MAX_ACCELERATION)
	{
		tmc5160[(uint8_t)channel].ramp_gen_regs.amax_reg.amax = TMC5160_MAX_ACCELERATION;
		return_value = tmc5160_spi_write_reg(channel, addr, TMC5160_MAX_ACCELERATION, TMC5160_SPI_TIMEOUT);
	}
	else
	{
		tmc5160[(uint8_t)channel].ramp_gen_regs.amax_reg.amax = amax;
		return_value = tmc5160_spi_write_reg(channel, addr, amax, TMC5160_SPI_TIMEOUT);
	}

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set acceleration a1.
 * \param[in] channel           Motor channel information.
 * \param[in] a1                First acceleration between VSTART and V1.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 21
results_enum_t tmc5160_set_ramp_a1(tmc5160_channels_enum_t channel, uint16_t a1)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.a1_reg.addr;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.a1_reg.a1 = a1;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       First acceleration-deceleration phase threshold velocity.
 * \param[in] channel           Motor channel information.
 * \param[in] v1                0: Disables A1 and D1 phase, use AMAX, DMAX only.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 22
results_enum_t tmc5160_set_ramp_v1(tmc5160_channels_enum_t channel, uint32_t v1)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.v1_reg.addr;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.v1_reg.v1 = v1;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set maximum deceleration.
 * \param[in] channel           Motor channel information.
 * \param[in] dmax              Deceleration between VMAX and V1.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 23
results_enum_t tmc5160_set_ramp_dmax(tmc5160_channels_enum_t channel, uint16_t dmax)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.dmax_reg.addr;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.dmax_reg.dmax = dmax;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set deceleration d1.
 * \param[in] channel           Motor channel information.
 * \param[in] d1                Deceleration between V1 and VSTOP.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 24
results_enum_t tmc5160_set_ramp_d1(tmc5160_channels_enum_t channel, uint16_t d1)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.d1_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.d1_reg.d1 = d1;
	return_value = tmc5160_spi_write_reg(channel, addr, d1, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set velocity start.
 * \param[in] channel           Motor channel information.
 * \param[in] vstart            For universal use, set VSTOP ≥ VSTART. This is not required if the motion distance is sufficient to ensure deceleration from VSTART to VSTOP.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 25
results_enum_t tmc5160_set_ramp_vstart(tmc5160_channels_enum_t channel, uint32_t vstart)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	if (tmc5160[(uint8_t)channel].ramp_gen_regs.vstart_reg.vstart > tmc5160[(uint8_t)channel].ramp_gen_regs.vstop_reg.vstop)
	{
		return 0;
	}

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.vstart_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.vstart_reg.vstart = vstart;
	return_value = tmc5160_spi_write_reg(channel, addr, vstart, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set velocity stop.
 * \param[in] channel           Motor channel information.
 * \param[in] vstop             Set VSTOP ≥ VSTART to allow positioning for short distances. Do not set 0 in positioning mode, minimum 10 recommend, set >100 for faster ramp termination!
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 26
results_enum_t tmc5160_set_ramp_vstop(tmc5160_channels_enum_t channel, uint32_t vstop)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	if (tmc5160[(uint8_t)channel].ramp_gen_regs.vstart_reg.vstart > tmc5160[(uint8_t)channel].ramp_gen_regs.vstop_reg.vstop)
	{
		return 0;
	}

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.vstop_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.vstop_reg.vstop = vstop;
	return_value = tmc5160_spi_write_reg(channel, addr, vstop, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set ramp mode type.
 * \param[in] channel           Motor channel information.
 * \param[in] ramp_mode         0: Positioning mode (using all A, D and V parameters)
 *                              1: Velocity mode to positive VMAX (using AMAX acceleration)
 *                              2: Velocity mode to negative VMAX (using AMAX acceleration)
 *                              3: Hold mode (velocity remains unchanged, unless stop event occurs)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 27
results_enum_t tmc5160_set_ramp_type(tmc5160_channels_enum_t channel, tmc5160_ramp_type_enum_t ramp_mode)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_mode_reg.ramp_mode = (uint8_t)ramp_mode;
	return_value = tmc5160_spi_write_reg(channel, addr, (uint8_t)ramp_mode, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set SW_MODE register.
 * \param[in] channel           Motor channel information.
 * \param[in] sw_mode           Loads value into Reference Switch & StallGuard2 register.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 28
results_enum_t tmc5160_set_sw_mode(tmc5160_channels_enum_t channel, uint16_t sw_mode)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved = sw_mode;
	return_value = tmc5160_spi_write_reg(channel, addr, sw_mode, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set automatic left stop.
 * \param[in] channel           Motor channel information.
 * \param[in] stop_en_left      1: Enables automatic motor stop during active left reference switch input. The motor restarts in case the stop switch becomes released.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 29
results_enum_t tmc5160_set_sw_mode_left_stop(tmc5160_channels_enum_t channel, bool stop_en_left)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.stop_en_left = stop_en_left;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set automatic right stop.
 * \param[in] channel           Motor channel information.
 * \param[in] stop_en_right     1: Enables automatic motor stop during active right reference switch input. The motor restarts in case the stop switch becomes released.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 30
results_enum_t tmc5160_set_sw_mode_right_stop(tmc5160_channels_enum_t channel, bool stop_en_right)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.stop_en_right = stop_en_right;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Sets the active polarity of the left reference switch input.
 * \param[in] channel           Motor channel information.
 * \param[in] pol_stop_left     0: non-inverted, high active: a high level on REFL stops the motor.
 *                              1: inverted, low active: a low level on REFL stops the motor.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 31
results_enum_t tmc5160_set_sw_mode_pol_stop_left(tmc5160_channels_enum_t channel, bool pol_stop_left)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.pol_stop_left = pol_stop_left;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Sets the active polarity of the right reference switch input.
 * \param[in] channel           Motor channel information.
 * \param[in] pol_stop_right    0: non-inverted, high active: a high level on REFR stops the motor.
 *                              1: inverted, low active: a low level on REFR stops the motor.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 32
results_enum_t tmc5160_set_sw_mode_pol_stop_right(tmc5160_channels_enum_t channel, bool pol_stop_right)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.pol_stop_right = pol_stop_right;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set swap REFL and REFR.
 * \param[in] channel           Motor channel information.
 * \param[in] swap_left_right   1: Swap the left and the right reference switch input REFL and REFR.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 33
results_enum_t tmc5160_set_sw_mode_swap_left_right(tmc5160_channels_enum_t channel, bool swap_left_right)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.swap_left_right = swap_left_right;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set latch left active.
 * \param[in] channel           Motor channel information.
 * \param[in] active_left       1: Activates latching of the position to XLATCH upon an active going edge on the left reference switch input REFL.
 *                              Hint: Activate latch_l_active to detect any spurious stop event by reading status_latch_l.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 34
results_enum_t tmc5160_set_sw_mode_latch_active_left(tmc5160_channels_enum_t channel, bool latch_active_left)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.latch_active_left = latch_active_left;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set latch left inactive.
 * \param[in] channel           Motor channel information.
 * \param[in] inactive_left     1: Activates latching of the position to XLATCH upon an inactive going edge on the left reference switch input REFL. The active level is defined by pol_stop_l.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 35
results_enum_t tmc5160_set_sw_mode_latch_inactive_left(tmc5160_channels_enum_t channel, bool latch_inactive_left)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.latch_inactive_left = latch_inactive_left;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set latch right active.
 * \param[in] channel           Motor channel information.
 * \param[in] active_right      1: Activates latching of the position to XLATCH upon an active going edge on the right reference switch input REFR.
 *                              Hint: Activate latch_r_active to detect any spurious stop event by reading status_latch_r.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 36
results_enum_t tmc5160_set_sw_mode_latch_active_right(tmc5160_channels_enum_t channel, bool latch_active_right)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.latch_active_right = latch_active_right;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set latch right inactive.
 * \param[in] channel           Motor channel information.
 * \param[in] inactive_right    1: Activates latching of the position to XLATCH upon an inactive going edge on the right reference switch input REFR. The active level is defined by pol_stop_r.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 37
results_enum_t tmc5160_set_sw_mode_latch_inactive_right(tmc5160_channels_enum_t channel, bool latch_inactive_right)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.latch_inactive_right = latch_inactive_right;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set enable latch encoder.
 * \param[in] channel           Motor channel information.
 * \param[in] en_latch_enc      1: Latch encoder position to ENC_LATCH upon reference switch event.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 38
results_enum_t tmc5160_set_sw_mode_en_latch_enc(tmc5160_channels_enum_t channel, bool en_latch_enc)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.en_latch_enc = en_latch_enc;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set stallGuard2 stop.
 * \param[in] channel           Motor channel information.
 * \param[in] sg_stop           1: Enable stop by StallGuard2 (also available in DcStep mode). Disable to release motor after stop event. Program TCOOLTHRS for velocity threshold.
 *                              Hint: Do not enable during motor spin-up, wait until the motor velocity exceeds a certain value, where StallGuard2 delivers a stable result. This velocity threshold should be programmed using TCOOLTHRS.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 39
results_enum_t tmc5160_set_sw_mode_sg_stop(tmc5160_channels_enum_t channel, bool sg_stop)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.sg_stop = sg_stop;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set soft stop or hard stop.
 * \param[in] channel           Motor channel information.
 * \param[in] en_softstop       0: Hard stop.
 *                              1: Soft stop.
 *
 *                              The soft stop mode always uses the deceleration ramp settings DMAX, V1, D1, VSTOP and TZEROWAIT for stopping the motor. A stop occurs when the velocity sign matches the reference switch position (REFL for negative velocities, REFR for positive velocities) and the respective switch stop function is enabled.
 *                              A hard stop also uses TZEROWAIT before the motor becomes released.
 *
 *                              Attention: Do not use soft stop in combination with StallGuard2. Use soft stop for StealthChop operation at high velocity. In this case, hard stop must be avoided, as it could result in severe overcurrent.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 40
results_enum_t tmc5160_set_sw_mode_en_softstop(tmc5160_channels_enum_t channel, bool en_softstop)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint16_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.en_softstop = en_softstop;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set waiting time after ramp down.
 * \param[in] channel           Motor channel information.
 * \param[in] tzero_wait        Defines the waiting time after ramping down to zero velocity before next movement or direction inversion can start. Time range is about 0 to 2 seconds. This setting avoids excess acceleration e.g. from VSTOP to -VSTART.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 41
results_enum_t tmc5160_set_wait_time_ramp_down(tmc5160_channels_enum_t channel, uint16_t tzero_wait)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.tzero_wait_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.tzero_wait_reg.tzero_wait = tzero_wait;
	return_value = tmc5160_spi_write_reg(channel, addr, tzero_wait, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set speed threshold for high speed mode.
 * \param[in] channel           Motor channel information.
 * \param[in] value             This velocity setting allows velocity dependent switching into a different chopper mode and fullstepping to maximize torque. (unsigned). The stall detection feature becomes switched off for 2-3 electrical periods whenever passing THIGH threshold to compensate for the effect of switching modes.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 42
results_enum_t tmc5160_set_se_hspeed_thresh(tmc5160_channels_enum_t channel, int32_t value)
{
	results_enum_t return_value = RESULT_SUCCESS;
	int32_t temp;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.thigh_reg.addr;
	temp = MIN(0xFFFF, (1 << 24) / ((value) ? value : 1));
	return_value = tmc5160_spi_write_reg(channel, addr, temp, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set minimum speed for switching to dcStep.
 * \param[in] channel           Motor channel information.
 * \param[in] value             Automatic commutation DcStep becomes enabled above velocity VDCMIN (unsigned) (only when using internal ramp generator, not for STEP/DIR interface – in STEP/DIR mode, DcStep becomes enabled by the external signal DCEN).
 *                              In this mode, the actual position is determined by the sensor-less motor commutation and becomes fed back to XACTUAL. In case the motor becomes heavily loaded, VDCMIN also is used as the minimum step velocity. Activate stop on stall (sg_stop) to detect step loss.
 *                              0: Disable, DcStep off.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 43
results_enum_t tmc5160_set_min_speed_dcstep(tmc5160_channels_enum_t channel, int32_t value)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.vdc_min_reg.addr;
	tmc5160[(uint8_t)channel].ramp_gen_regs.vdc_min_reg.vdc_min = value;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set analog i scale.
 * \param[in] channel           Motor channel information.
 * \param[in] recalibrate       1: Zero crossing recalibration during driver disable (via DRV_ENN or via TOFF setting).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 44
results_enum_t tmc5160_set_gconf_recalibrate(tmc5160_channels_enum_t channel, bool recalibrate)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.recalibrate = recalibrate;
	value = tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set pwm mode.
 * \param[in] channel           Motor channel information.
 * \param[in] en_pwm_mode       1: StealthChop voltage PWM mode enabled(depending on velocity thresholds). Switch from off to
 *                              on state while in stand-still and at IHOLD=nominal IRUN current, only.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 45
results_enum_t tmc5160_set_gconf_pwm_mode(tmc5160_channels_enum_t channel, bool en_pwm_mode)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.en_pwm_mode = en_pwm_mode;
	value = tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set multistep filter.
 * \param[in] channel           Motor channel information.
 * \param[in] multistep_filt    1: Enable step input filtering for StealthChop optimization with external step source (default=1).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 46
results_enum_t tmc5160_set_gconf_multistep_filt(tmc5160_channels_enum_t channel, bool multistep_filt)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.multistep_filt = multistep_filt;
	value = tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set short to VS detector level for lowside FETs.
 * \param[in] channel           Motor channel information.
 * \param[in] s2vs_level        Checks for voltage drop in LS MOSFET and sense resistor. 4 (highest sensitivity) ... 15 (lowest sensitivity).
 *                              Hint: Settings from 1 to 3 will trigger during normal operation due to voltage drop on sense resistor.
 *                              (Reset Default: OTP 6 or 12).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 47
results_enum_t tmc5160_set_shortconf_s2vs_level(tmc5160_channels_enum_t channel, uint8_t s2vs_level)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.s2vs_level = s2vs_level;
	value = tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set short to GND detector level for highside FETs.
 * \param[in] channel           Motor channel information.
 * \param[in] s2g_level         Checks for voltage drop on high side MOSFET 2 (highest sensitivity) ... 15 (lowest sensitivity).
 *                              Attention: Settings below 6 not recommended at >52V operation – false detection might result.
 *                              (Reset Default: OTP 6 or 12)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 48
results_enum_t tmc5160_set_shortconf_s2g_level(tmc5160_channels_enum_t channel, uint8_t s2g_level)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.s2g_level = s2g_level;
	value = tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set spike filtering bandwidth for short detection.
 * \param[in] channel           Motor channel information.
 * \param[in] short_filter      00 (lowest, 100ns)
 *                              01 (1μs)
 *                              10 (2μs)
 *                              11 (3μs)
 *                              Hint: A good PCB layout will allow using setting 0. Increase value, if erroneous short detection occurs.
 *                              (Reset Default = %01).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 49
results_enum_t tmc5160_set_shortconf_short_filter(tmc5160_channels_enum_t channel, uint8_t short_filter)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.short_filter = short_filter;
	value = tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set detection delay.
 * \param[in] channel           Motor channel information.
 * \param[in] short_delay       0: 750ns:  normal
 *                              1: 1500ns: high
 *                              The short detection delay shall cover the bridge switching time. 0 will work for most applications.
 *                              (Reset Default = 0)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 50
results_enum_t tmc5160_set_shortconf_short_delay(tmc5160_channels_enum_t channel, bool short_delay)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.short_delay = short_delay;
	value = tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set break-before make delay.
 * \param[in] channel           Motor channel information.
 * \param[in] bbmtime           0: shortest (100ns) ... 16: (200ns) ... 24: longest (375ns) > 24 not recommended, use BBMCLKS instead.
 *                              Hint: Choose the lowest setting safely covering the switching event to avoid bridge cross-conduction. Add roughly 30% of reserve.
 *                              (Reset Default = 0).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 51
results_enum_t tmc5160_set_drv_bbmtime(tmc5160_channels_enum_t channel, uint8_t bbmtime)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.bbmtime = bbmtime;
	value = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set digital BBM time in clock cycles (typ. 83ns). The longer setting rules (BBMTIME vs. BBMCLKS).
 * \param[in] channel           Motor channel information.
 * \param[in] bbmclks           0 ... 15
 *                              (Reset Default: OTP 4 or 2)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 52
results_enum_t tmc5160_set_drv_bbmclks(tmc5160_channels_enum_t channel, uint8_t bbmclks)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.bbmclks = bbmclks;
	value = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set selection of over temperature level for bridge disable, switch on after cool down to 120°C / OTPW level.
 * \param[in] channel           Motor channel information.
 * \param[in] otselect          00: 150 C
 *                              01: 143 C
 *                              10: 136 C (not recommended when VSA > 24V)
 *                              11: 120 C (not recommended, no hysteresis)
 *                              Hint: Adapt overtemperature threshold as required to protect the MOSFETs or other components on the PCB.
 *                              (Reset Default = %00)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 53
results_enum_t tmc5160_set_drv_otselect(tmc5160_channels_enum_t channel, uint8_t otselect)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.otselect = otselect;
	value = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set selection of gate driver current. Adapts the gate driver current to the gate charge of the external MOSFETs.
 * \param[in] channel           Motor channel information.
 * \param[in] drv_strength      00: weak
 *                              01: weak+TC (medium above OTPW level)
 *                              10: medium
 *                              11: strong
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 54
results_enum_t tmc5160_set_drv_strength(tmc5160_channels_enum_t channel, uint8_t drvstrength)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.drvstrength = drvstrength;
	value = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set filter time constant of sense amplifier to suppress ringing and coupling from second coil operation.
 * \param[in] channel           Motor channel information.
 * \param[in] filt_isense       00: low  -100ns
 *                              01:      -200ns
 *                              10:      -300ns
 *                              11: high -400ns
 *                              Hint: Increase setting if motor chopper noise occurs due to cross-coupling of both coils.
 *                              (Reset Default = %00)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 55
results_enum_t tmc5160_set_drv_filt_isense(tmc5160_channels_enum_t channel, uint8_t filt_isense)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.filt_isense = filt_isense;
	value = tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       TPOWERDOWN sets the delay time after stand still (stst) of the motor to motor current power down. Time range is about 0 to 4 seconds.
 * \param[in] channel           Motor channel information.
 * \param[in] tpower_down       Attention: A minimum setting of 2 is required to allow automatic tuning of StealthChop PWM_OFS_AUTO.
 *                              Reset Default = 10
 *                              0 ... ((2^8)-1) * 2^18 tCLK.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 56
results_enum_t tmc5160_set_tpower_down(tmc5160_channels_enum_t channel, uint8_t tpower_down)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.tpower_down_reg.addr;
	tmc5160[(uint8_t)channel].vel_regs.tpower_down_reg.tpower_down = tpower_down;
	return_value = tmc5160_spi_write_reg(channel, addr, tpower_down, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Chopper and driver configuration.
 * \param[in] channel           Motor channel information.
 * \param[in] value             (Reset default = 0x10410150)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 57
results_enum_t tmc5160_set_chopper_config(tmc5160_channels_enum_t channel, uint32_t value)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved = value;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set chopper off time.
 * \param[in] channel           Motor channel information.
 * \param[in] toff              Off time setting controls duration of slow decay phase.
 *                              Nclk = 24 + 32 * TOFF
 *                              0000:            Driver disable, all bridges off
 *                              0001:            1 - use only with TBL >= 2
 *                              0010: ... 1111:  2 ... 15
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 58
results_enum_t tmc5160_set_chopper_toff(tmc5160_channels_enum_t channel, uint8_t toff)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.toff = toff;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set chopper hysteresis start / sine wave offset.
 * \param[in] channel           Motor channel information.
 * \param[in] hstrt_sine_offset 000  ...  111:  Add 1, 2, ..., 8 to hysteresis low value HEND.
 *                              0000 ... 1111:  Offset is -3, -2, -1, 0, 1, ..., 12
 *                              This is the sine wave offset and 1/512 of the value becomes added to the absolute value of each sine wave entry.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 59
results_enum_t tmc5160_set_chopper_hstrt_sine_offset(tmc5160_channels_enum_t channel, uint8_t hstrt_sine_offset)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;

	if (tmc5160_get_chopper_mode(channel))
	{
		tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.hstrt = hstrt_sine_offset;
		value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
		return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);
	}
	else
	{
		tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.hend = hstrt_sine_offset;
		value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
		return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);
	}

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set chopper hysteresis end / fast decay time.
 * \param[in] channel           Motor channel information.
 * \param[in] hend_fd_time      Fast decay time settings (MSB: fd3):
 *                              0000 ... 1111:
 *                              Fast decay time setting TFD with  NCLK= 32*TFD (%0000: slow decay only)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 60
results_enum_t tmc5160_set_chopper_hend_fd_time(tmc5160_channels_enum_t channel, uint8_t hend_fd_time)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;

	if (tmc5160_get_chopper_mode(channel))
	{
		tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.hend = hend_fd_time;
		value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
		return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);
	}
	else
	{
		tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.fd3 = 1;
		tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.hstrt = hend_fd_time;
	}

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set fast decay comparator.
 * \param[in] channel           Motor channel information.
 * \param[in] disfdcc           chm = 1:
 *                              disfdcc = 1 disables current comparator usage for termination of the fast decay cycle.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 61
results_enum_t tmc5160_set_chopper_fast_decay(tmc5160_channels_enum_t channel, bool disfdcc)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.disfdcc = disfdcc;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set constant toff mode.
 * \param[in] channel           Motor channel information.
 * \param[in] chm               0: Standard mode (SpreadCycle).
 *                              1: Constant off time with fast decay time. Fast decay time is also terminated when the
 *                              negative nominal current is reached. Fast decay is after on time.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 62
results_enum_t tmc5160_set_chopper_mode(tmc5160_channels_enum_t channel, bool chm)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.chm = chm;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set chopper blank time.
 * \param[in] channel           Motor channel information.
 * \param[in] tbl               Set comparator blank time to 16, 24, 36 or 54 clocks.
 *                              0: 16 clock
 *                              1: 24 clock
 *                              2: 36 clock
 *                              3: 54 clock
 *                              Hint: %01 or %10 is recommended for most applications (Reset Default: OTP %01 or %10)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 63
results_enum_t tmc5160_set_chopper_blank_time(tmc5160_channels_enum_t channel, uint8_t tbl)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.tbl = tbl;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set high speed full step selection.
 * \param[in] channel           Motor channel information.
 * \param[in] vhighfs           This bit enables switching to fullstep, when VHIGH is exceeded. Switching takes place only at 45° position. The
 *                              fullstep target current uses the current value from the microstep table at the 45° position.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 64
results_enum_t tmc5160_set_chopper_high_vel_fs(tmc5160_channels_enum_t channel, bool vhighfs)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.vhighfs = vhighfs;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set high velocity chopper mode.
 * \param[in] channel           Motor channel information.
 * \param[in] vhighchm          This bit enables switching to chm=1 and fd=0, when VHIGH is exceeded. This way, a higher velocity can be achieved.
 *                              Can be combined with vhighfs=1. If set, the TOFF setting automatically becomes doubled during high velocity operation
 *                              in order to avoid doubling of the chopper frequency.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 65
results_enum_t tmc5160_set_chopper_high_vel(tmc5160_channels_enum_t channel, bool vhighchm)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.vhighchm = vhighchm;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       TPFD allows dampening of motor mid-range resonances.
 * \param[in] channel           Motor channel information.
 * \param[in] tpfd              Passive fast decay time setting controls duration of the fast decay phase inserted after bridge polarity change
 *                              NCLK = 128 * TPFD
 *                              0000: Disable
 *                              0001: 1
 *                              ...
 *                              1111: 15
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 66
results_enum_t tmc5160_set_chopper_passive_fd_time(tmc5160_channels_enum_t channel, uint8_t tpfd)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.tpfd = tpfd;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set microstep resolution.
 * \param[in] channel           Motor channel information.
 * \param[in] microstep         0: 256
 *                              1: 128
 *                              2: 64
 *                              3: 32
 *                              4: 16
 *                              5: 8
 *                              6: 4
 *                              7: 2
 *                              8: fullstep
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 67
results_enum_t tmc5160_set_chopper_microstep_res(tmc5160_channels_enum_t channel, microstep_res_enum_t microstep)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.mres = microstep;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set enable double edge step pulses.
 * \param[in] channel           Motor channel information.
 * \param[in] dedge             1: Enable step impulse at each step edge to reduce step frequency requirement.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 68
results_enum_t tmc5160_set_chopper_double_edge(tmc5160_channels_enum_t channel, bool dedge)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.dedge = dedge;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set short to supply protection.
 * \param[in] channel           Motor channel information.
 * \param[in] diss2g            0: Short to GND protection is on.
 *                              1: Short to GND protection is disabled.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 69
results_enum_t tmc5160_set_chopper_gnd_protection(tmc5160_channels_enum_t channel, bool diss2g)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.diss2g = diss2g;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set short to GND protection.
 * \param[in] channel           Motor channel information.
 * \param[in] diss2vs           0: Short to VS protection is on.
 *                              1: Short to VS protection is disabled.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 70
results_enum_t tmc5160_set_chopper_supply_protection(tmc5160_channels_enum_t channel, bool diss2vs)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.diss2vs = diss2vs;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set smart energy current minimum.
 * \param[in] channel           Motor channel information.
 * \param[in] seimin            0: 1/2 of current setting(IRUN).
 *                              1: 1/4 of current setting(IRUN).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 71
results_enum_t tmc5160_set_cool_se_current_min(tmc5160_channels_enum_t channel, bool seimin)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.seimin = seimin;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set smart energy current down step.
 * \param[in] channel           Motor channel information.
 * \param[in] sedn              00: For each 32 StallGuard2 values decrease by one.
 *                              01: For each 8 StallGuard2 values decrease by one.
 *                              10: For each 2 StallGuard2 values decrease by one.
 *                              11: For each StallGuard2 value decrease by one.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 72
results_enum_t tmc5160_set_cool_se_current_down_step(tmc5160_channels_enum_t channel, uint8_t sedn)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.sedn = sedn;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set smart energy hysteresis.
 * \param[in] channel           Motor channel information.
 * \param[in] semax             If the StallGuard2 result is equal to or above (SEMIN+SEMAX+1)*32, the motor current becomes decreased to save energy.
 *                              0000: 0, 0001: 1, 0010: 2, 0011: 3, 0100: 4, 0101: 5, 0110: 6, 0111: 7,
 *                              1000: 8, 1001: 9, 1010: 10, 1011: 11, 1100: 12, 1101: 13, 1110: 14, 1111: 15
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 73
results_enum_t tmc5160_set_cool_se_hysteresis(tmc5160_channels_enum_t channel, uint8_t semax)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.semax = semax;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set smart energy current up step.
 * \param[in] channel           Motor channel information.
 * \param[in] seup              Current increment steps per measured StallGuard2 value.
 *                              00: 1
 *                              01: 2
 *                              10: 4
 *                              11: 8
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 74
results_enum_t tmc5160_set_cool_se_current_up_step(tmc5160_channels_enum_t channel, uint8_t seup)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.seup = seup;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set smart energy hysteresis start.
 * \param[in] channel           Motor channel information.
 * \param[in] semin             If the StallGuard2 result falls below SEMIN*32, the motor current becomes increased to reduce motor load angle.
 *                              0000: smart current control CoolStep off.
 *                              0001 ... %1111: 1 ... 15.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 75
results_enum_t tmc5160_set_cool_se_hysteresis_start(tmc5160_channels_enum_t channel, uint8_t semin)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.semin = semin;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set stallGuard2 filter.
 * \param[in] channel           Motor channel information.
 * \param[in] sfilt             0: Standard mode, high time resolution for StallGuard2.
 *                              1: Filtered mode, StallGuard2 signal updated for each four fullsteps (resp. six fullsteps for 3 phase motor) only to compensate for motor pole tolerances.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 76
results_enum_t tmc5160_set_cool_sg_filter(tmc5160_channels_enum_t channel, bool sfilt)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.sfilt = sfilt;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set stallGuard2 threshold.
 * \param[in] channel           Motor channel information.
 * \param[in] sgt               This signed value controls StallGuard2 level for stall output and sets the optimum measurement range for readout. A lower value gives a higher sensitivity. Zero is the starting value working with most motors.
 *                              -64 to +63 : A higher value makes StallGuard2 less sensitive and requires more torque to indicate a stall.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 77
results_enum_t tmc5160_set_cool_sg_thresh(tmc5160_channels_enum_t channel, int8_t sgt)
{
	results_enum_t return_value = RESULT_SUCCESS;
	int32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.sgt = sgt;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set smart energy stall velocity.
 * \param[in] channel           Motor channel information.
 * \param[in] sg_stop           1: Enable stop by StallGuard2 (also available in DcStep mode). Disable to release motor after stop event. Program TCOOLTHRS for velocity threshold.
 *                              Do not enable during motor spin-up, wait until the motor velocity exceeds a certain value, where StallGuard2 delivers a stable result. This velocity threshold should be programmed using TCOOLTHRS.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 78
results_enum_t tmc5160_set_se_velocity_stall(tmc5160_channels_enum_t channel, bool sg_stop)
{
	results_enum_t return_value[2] = { RESULT_SUCCESS, RESULT_SUCCESS };
	uint16_t value;
	uint8_t tcool_addr, sw_mode_addr;

	tcool_addr = tmc5160[(uint8_t)channel].vel_regs.tcoolthrs_reg.addr;
	sw_mode_addr = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.addr;

	tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.sg_stop = sg_stop;
	value = tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
	return_value[0] = tmc5160_spi_write_reg(channel, sw_mode_addr, value, TMC5160_SPI_TIMEOUT);

	value = MIN(0xFFFF, (1 << 24) / ((value) ? value : 1));
	return_value[1] = tmc5160_spi_write_reg(channel, tcool_addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value[0]))
	{
		result_error_callback(return_value[0], TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}
	else if (RESULT_IS_ERROR(return_value[1]))
	{
		result_error_callback(return_value[1], TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value[0] | return_value[1];  /* ToDo: Duzenleme yapilacak. */
}

/**
 * \brief                       Set smart energy threshold speed.
 * \param[in] channel           Motor channel information.
 * \param[in] value             This is the lower threshold velocity for switching on smart energy CoolStep and StallGuard feature. (unsigned).
 *                              See tmc5160 data sheet for important parameter information.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 79
results_enum_t tmc5160_set_se_speed_thresh(tmc5160_channels_enum_t channel, int32_t value)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.tcoolthrs_reg.addr;
	value = MIN(0xFFFF, (1 << 24) / ((value) ? value : 1));
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set pwm threshold speed.
 * \param[in] channel           Motor channel information.
 * \param[in] value             This is the upper velocity for StealthChop voltage PWM mode.
 *                              See tmc5160 data sheet for important parameter information.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 80
results_enum_t tmc5160_set_pwm_thresh_speed(tmc5160_channels_enum_t channel, int32_t value)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.tpwmthrs_reg.addr;
	value = MIN(0xFFFFF, (1 << 24) / ((value) ? value : 1));
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set voltage PWM mode StealthChop.
 * \param[in] channel           Motor channel information.
 * \param[in] value             Loads value into PWMCONF register.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 81
results_enum_t tmc5160_set_pwm_config(tmc5160_channels_enum_t channel, uint32_t value)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved = value;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set pwm gradient.
 * \param[in] channel           Motor channel information.
 * \param[in] pwm_grad          Velocity dependent gradient for PWM amplitude:
 *                              PWM_GRAD * 256 / TSTEP
 *                              This value is added to PWM_OFS to compensate for the velocity-dependent motor back-EMF.
 *                              See tmc5160 data sheet for important parameter information.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 82
results_enum_t tmc5160_set_pwm_gradient(tmc5160_channels_enum_t channel, uint8_t pwm_grad)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_grad = pwm_grad;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set offset amplitude.
 * \param[in] channel           Motor channel information.
 * \param[in] pwm_ofs           User defined PWM amplitude offset (0-255) related to full motor current (CS_ACTUAL=31) in stand still.
 *                              (Reset default = 30).
 *                              PWM_OFS = 0 will disable scaling down motor current below a motor specific lower measurement threshold. This setting should only be used under certain conditions, i.e.,
 *                              when the power supply voltage can vary up and down by a factor of two or more. It prevents the motor going out of regulation, but it also prevents power down below the regulation limit.
 *                              PWM_OFS > 0 allows automatic scaling to low PWM duty cycles even below the lower regulation threshold. This allows low (standstill) current settings based on the actual (hold) current scale (register IHOLD_IRUN).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 83
results_enum_t tmc5160_set_pwm_ofs_amplitude(tmc5160_channels_enum_t channel, uint8_t pwm_ofs)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_ofs = pwm_ofs;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set pwm frequency.
 * \param[in] channel           Motor channel information.
 * \param[in] pwm_freq          00: fPWM=2/1024 fCLK (Reset default).
 *                              01: fPWM=2/683 fCLK.
 *                              10: fPWM=2/512 fCLK.
 *                              11: fPWM=2/410 fCLK.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 84
results_enum_t tmc5160_set_pwm_frequency(tmc5160_channels_enum_t channel, uint8_t pwm_freq)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_freq = pwm_freq;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set pwm autoscale.
 * \param[in] channel           Motor channel information.
 * \param[in] pwm_autoscale     0: User defined feed forward PWM amplitude. The current settings IRUN and IHOLD are not enforced by regulation, but scale the PWM amplitude, only!
 *                                 The resulting PWM amplitude (limited to 0…255) is:
 *                                 PWM_OFS * ((CS_ACTUAL+1) / 32) + PWM_GRAD * 256 / TSTEP.
 *                              1: Enable automatic current control (Reset default)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 85
results_enum_t tmc5160_set_pwm_autoscale(tmc5160_channels_enum_t channel, bool pwm_autoscale)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_autoscale = pwm_autoscale;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set pwm automatic gradient adaptation.
 * \param[in] channel           Motor channel information.
 * \param[in] pwm_autograd      0: Fixed value for PWM_GRAD (PWM_AUTO_GRAD = PWM_GRAD).
 *                              1: Automatic tuning (only with pwm_autoscale=1)
 *                              (Reset default)
 *                              See tmc5160 data sheet for important parameter information.
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 86
results_enum_t tmc5160_set_pwm_autograd(tmc5160_channels_enum_t channel, bool pwm_autograd)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_autograd = pwm_autograd;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set freewheeling mode.
 * \param[in] channel           Motor channel information.
 * \param[in] freewheel         Stand still option when motor current setting is zero (I_HOLD = 0).
 *                              00: Normal operation
 *                              01: Freewheeling
 *                              10: Coil shorted using LS drivers
 *                              11: Coil shorted using HS drivers
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 87
results_enum_t tmc5160_set_pwm_freewheeling_mode(tmc5160_channels_enum_t channel, uint8_t freewheel)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.freewheel = freewheel;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set regulation loop gradient.
 * \param[in] channel           Motor channel information.
 * \param[in] pwm_regulation    User defined maximum PWM amplitude change per half wave when using pwm_autoscale = 1. (1 ... 15):
 *                              1:  0.5 increments (slowest regulation)
 *                              2:  1 increments
 *                              3:  1.5 increments
 *                              4:  2 increments (Reset default)
 *                              ...
 *                              8:  4 increments
 *                              ...
 *                              15: 7.5 increments (fastest regulation).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 88
results_enum_t tmc5160_set_pwm_regulation(tmc5160_channels_enum_t channel, uint8_t pwm_regulation)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_regulation = pwm_regulation;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set PWM automatic scale amplitude limit when switching on.
 * \param[in] channel           Motor channel information.
 * \param[in] pwm_lim           Limit for PWM_SCALE_AUTO when switching back from SpreadCycle to StealthChop.
 *                              This value defines the upper limit for bits 7 to 4 of the automatic current control when switching back.
 *                              It can be set to reduce the current jerk during mode change back to StealthChop.
 *                              It does not limit PWM_GRAD or PWM_GRAD_AUTO offset.
 *                              (Default = 12).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 89
results_enum_t tmc5160_set_pwm_limit(tmc5160_channels_enum_t channel, uint8_t pwm_lim)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.addr;
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_lim = pwm_lim;
	value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}


/**
 * \brief                       Set encoder position.
 * \param[in] channel           Motor channel information.
 * \param[in] xenc              Actual encoder position (signed).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 90
results_enum_t tmc5160_set_enc_pos(tmc5160_channels_enum_t channel, int32_t xenc)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].enc_regs.xenc_reg.addr;
	tmc5160[(uint8_t)channel].enc_regs.xenc_reg.xenc = xenc;
	return_value = tmc5160_spi_write_reg(channel, addr, xenc, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set encoder resolution.
 * \param[in] channel           Motor channel information.
 * \param[in] enc_const         Accumulation constant (signed)
 *                              16 bit integer part, 16 bit fractional part.
 *                              X_ENC accumulates
 *                              +/- ENC_CONST / (2^16*X_ENC) (binary) or +/-ENC_CONST / (10^4*X_ENC) (decimal)
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 91
results_enum_t tmc5160_set_enc_res(tmc5160_channels_enum_t channel, int32_t enc_const)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].enc_regs.enc_const_reg.addr;
	tmc5160[(uint8_t)channel].enc_regs.enc_const_reg.enc_const = enc_const;
	return_value = tmc5160_spi_write_reg(channel, addr, enc_const, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Global scaling of Motor current. This value is multiplied to the current scaling to adapt a drive to a certain motor type.
 *                              This value should be chosen before tuning other settings because it also influences chopper hysteresis.
 * \param[in] channel           Motor channel information.
 * \param[in] global_scaler     0:            Full Scale (or write 256)
 *                              1  ... 31:    Not allowed for operation
 *                              32 ... 255:   32/256 ... 255/256 of maximum current
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 92
results_enum_t tmc5160_set_global_scaler(tmc5160_channels_enum_t channel, uint8_t global_scaler)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.global_scaler_reg.addr;
	tmc5160[(uint8_t)channel].gconf_regs.global_scaler_reg.global_scaler = global_scaler;
	return_value = tmc5160_spi_write_reg(channel, addr, global_scaler, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Standstill current. In combination with StealthChop mode, setting IHOLD=0 allows to choose
 *                              freewheeling or coil short circuit for motor stand still.
 * \param[in] channel           Motor channel information.
 * \param[in] ihold             (0=1/32 ... 31=32/32).
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 93
results_enum_t tmc5160_set_ihold(tmc5160_channels_enum_t channel, uint8_t ihold)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.addr;
	tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.ihold_irun_pins.ihold = ihold;
	value = tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.ihold_irun_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Set motor run current.
 * \param[in] channel           Motor channel information.
 * \param[in] irun              (0=1/32 ... 31=32/32).
 * \retval                      The success or error value of the function.
 * \note                        Choose sense resistors in a way, that normal IRUN is 16 to 31 for best microstep performance.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 94
results_enum_t tmc5160_set_irun(tmc5160_channels_enum_t channel, uint8_t irun)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.addr;
	tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.ihold_irun_pins.irun = irun;
	value = tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.ihold_irun_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Controls the number of clock cycles for motor power down after a motion as soon as standstill is detected
 *                              (stst=1) and TPOWERDOWN has expired. The smooth transition avoids a motor jerk upon power down.
 * \param[in] channel           Motor channel information.
 * \param[in] ihold_delay       0:          instant power down
 *                              1 ... 15:   Delay per current reduction step in multiple of 2^18 clocks
 * \retval                      The success or error value of the function.
 */
#undef FUNCTION_CODE
#define FUNCTION_CODE 95
results_enum_t tmc5160_set_ihold_delay(tmc5160_channels_enum_t channel, uint8_t ihold_delay)
{
	results_enum_t return_value = RESULT_SUCCESS;
	uint32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.addr;
	tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.ihold_irun_pins.ihold_delay = ihold_delay;
	value = tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.ihold_irun_pins.reserved;
	return_value = tmc5160_spi_write_reg(channel, addr, value, TMC5160_SPI_TIMEOUT);

	if (RESULT_IS_ERROR(return_value))
	{
		result_error_callback(return_value, TMC5160_LIBRARY_CODE, FUNCTION_CODE);
	}

	return return_value;
}

/**
 * \brief                       Get otp memory read.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_otp_read(tmc5160_channels_enum_t channel)
{
	uint8_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.otp_read_pins.reserved = reserved;

	return reserved;
}

/**
 * \brief                       Get reset default for FCLKTRIM
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 *                              0:  lowest frequency setting
 *                              31: highest frequency setting
 */
uint8_t tmc5160_get_otp_read_fclktrim(tmc5160_channels_enum_t channel)
{
	uint8_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.otp_read_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.otp_read_pins.otp_fclktrim;
}

/**
 * \brief                       Get reset default for short-detection Levels:
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 *                              0: S2G_LEVEL = S2VS_LEVEL = 6
 *                              1: S2G_LEVEL = S2VS_LEVEL = 12
 */
bool tmc5160_get_otp_read_s2_level(tmc5160_channels_enum_t channel)
{
	uint8_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.otp_read_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.otp_read_pins.otp_s2_level;
}

/**
 * \brief                       Get reset default for DRVCONF.BBMCLKS
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 *                              0: BBMCLKS=4
 *                              1: BBMCLKS=2
 */
bool tmc5160_get_otp_read_bbm(tmc5160_channels_enum_t channel)
{
	uint8_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.otp_read_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.otp_read_pins.otp_bbm;
}

/**
 * \brief                       Get reset default for TBL.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 *                              0: TBL=2 (-3us)
 *                              1: TBL=1 (-2us)
 */
bool tmc5160_get_otp_read_tbl(tmc5160_channels_enum_t channel)
{
	uint8_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.otp_read_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].gconf_regs.otp_prog_read_reg.otp_read_pins.otp_tbl;
}

/**
 * \brief                       Get target position.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_ramp_xtarget(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.xtarget_reg.xtarget;
}

/**
 * \brief                       Get actual position.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_ramp_xactual(tmc5160_channels_enum_t channel)
{
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.xactual_reg.addr;

	return tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
}

/**
 * \brief                       Get actual speed.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_ramp_vactual(tmc5160_channels_enum_t channel)
{
	int32_t vactual;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.vactual_reg.addr;
	vactual = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].ramp_gen_regs.vactual_reg.vactual = vactual;

	return vactual;
}

/**
 * \brief                       Get maximum speed.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint32_t tmc5160_get_ramp_vmax(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.vmax_reg.vmax;
}

/**
 * \brief                       Get maximum acceleration.
 * \param[in] channel           Motor channel information.
 * \retval                      This is the acceleration and deceleration value for velocity mode.
 */
uint16_t tmc5160_get_ramp_amax(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.amax_reg.amax;
}

/**
 * \brief                       Get acceleration a1.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint16_t tmc5160_get_ramp_a1(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.a1_reg.a1;
}

/**
 * \brief                       Get velocity v1.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint32_t tmc5160_get_ramp_v1(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.v1_reg.v1;
}

/**
 * \brief                       Get maximum deceleration.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint16_t tmc5160_get_ramp_dmax(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.dmax_reg.dmax;
}

/**
 * \brief                       Get deceleration d1.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint16_t tmc5160_get_ramp_d1(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.d1_reg.d1;
}

/**
 * \brief                       Get velocity start.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint32_t tmc5160_get_ramp_vstart(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.vstart_reg.vstart;
}

/**
 * \brief                       Get velocity stop.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint32_t tmc5160_get_ramp_vstop(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.vstop_reg.vstop;
}

/**
 * \brief                       Get ramp mode type.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_ramp_type(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_mode_reg.ramp_mode;
}

/**
 * \brief                       Get position reached flag.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_rs_pos_flag(tmc5160_channels_enum_t channel)
{
	uint16_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.ramp_stat_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.ramp_stat_pins.reached_pos;
}

/**
 * \brief                       Get right end stop.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_rs_right_endstop(tmc5160_channels_enum_t channel)
{
	uint16_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.ramp_stat_pins.reserved = reserved;

	return !tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.ramp_stat_pins.status_stop_right;
}

/**
 * \brief                       Get left end stop.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_rs_left_endstop(tmc5160_channels_enum_t channel)
{
	uint16_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.ramp_stat_pins.reserved = reserved;

	return !tmc5160[(uint8_t)channel].ramp_gen_regs.ramp_stat_reg.ramp_stat_pins.status_stop_left;
}

/**
 * \brief                       Get SW_MODE register.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_sw_mode(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.reserved;
}

/**
 * \brief                       Get automatic right stop.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_sw_mode_right_stop(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.stop_en_right;
}

/**
 * \brief                       Get automatic left stop.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_sw_mode_left_stop(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.stop_en_left;
}

/**
 * \brief                       Get waiting time after ramp down.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint16_t tmc5160_get_wait_time_ramp_down(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.tzero_wait_reg.tzero_wait;
}

/**
 * \brief                       Get speed threshold for high speed mode.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_se_hspeed_thresh(tmc5160_channels_enum_t channel)
{
	int32_t buffer;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.thigh_reg.addr;
	buffer = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);

	return MIN(0xFFFF, (1 << 24) / ((buffer) ? buffer : 1));
}

/**
 * \brief                       Get minimum speed for switching to dcStep.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint32_t tmc5160_get_min_speed_dcstep(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].ramp_gen_regs.vdc_min_reg.vdc_min;
}

/**
 * \brief                       Get analog i scale.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_gconf_recalibrate(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.recalibrate;
}

/**
 * \brief                       Get pwm mode.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_gconf_pwm_mode(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.en_pwm_mode;
}

/**
 * \brief                       Get multistep filter.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_gconf_multistep_filt(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.gconf_reg.gconf_flag.multistep_filt;
}

/**
 * \brief                       Get short to VS detector level for lowside FETs.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_shortconf_s2vs_level(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.s2vs_level;
}

/**
 * \brief                       Get short to GND detector level for highside FETs.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_shortconf_s2g_level(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.s2g_level;
}

/**
 * \brief                       Get spike filtering bandwidth for short detection.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_shortconf_short_filter(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.short_filter;
}

/**
 * \brief                       Get short detection delay.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_shortconf_short_delay(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.short_conf_reg.conf_pins.short_delay;
}

/**
 * \brief                       Get break-before make delay.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_drv_bbmtime(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.bbmtime;
}

/**
 * \brief                       Get digital BBM time in clock cycles.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_drv_bbmclks(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.bbmclks;
}

/**
 * \brief                       Get selection of over temperature level.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_drv_otselect(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.otselect;
}

/**
 * \brief                       Get selection of gate driver current.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_drv_strength(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.drvstrength;
}

/**
 * \brief                       Get filter time constant of sense amplifier to suppress ringing and coupling from second coil operation.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_drv_filt_isense(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.drv_conf_reg.conf_pins.filt_isense;
}

/**
 * \brief                       Get power down time.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_tpower_down(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].vel_regs.tpower_down_reg.tpower_down;
}

/**
 * \brief                       Get chopper config register.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint32_t tmc5160_get_chopper_config(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.reserved;
}

/**
 * \brief                       Get chopper off time.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_chopper_toff(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.toff;
}

/**
 * \brief                       Get chopper hysteresis start / sine wave offset.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_chopper_hstrt_sine_offset(tmc5160_channels_enum_t channel)
{
	uint8_t offset;

	if (tmc5160_get_chopper_mode(channel))
	{
		return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.hstrt;
	}
	else
	{
		offset = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.hend;

		if (tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.fd3)
		{
			offset |= 1 << 3;
		}

		return offset;
	}
}

/**
 * \brief                       Get chopper hysteresis end / fast decay time.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_chopper_hend_fd_time(tmc5160_channels_enum_t channel)
{
	uint8_t value;

	if (tmc5160_get_chopper_mode(channel))
	{
		return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.hend;
	}
	else
	{
		value = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.hstrt;

		if (tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.fd3)
		{
			value |= 1 << 3;
		}

		return value;
	}
}

/**
 * \brief                       Get fast decay comparator.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_chopper_fast_decay(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.disfdcc;
}

/**
 * \brief                       Get constant toff mode.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_chopper_mode(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.chm;
}

/**
 * \brief                       Get chopper blank time.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_chopper_blank_time(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.tbl;
}

/**
 * \brief                       Get high speed full step mode.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_chopper_high_vel_fs(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.vhighfs;
}

/**
 * \brief                       Get high speed chopper mode.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_chopper_high_vel(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.vhighchm;
}

/**
 * \brief                       Get chopper passive fast decay time
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_chopper_passive_fd_time(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.tpfd;
}

/**
 * \brief                       Get microstep resolution.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_chopper_microstep_res(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.mres;
}

/**
 * \brief                       Get double edge step pulses.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_chopper_double_edge(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.dedge;
}

/**
 * \brief                       Get short to GND protection.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_chopper_gnd_protection(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.diss2g;
}

/**
 * \brief                       Get short to supply protection.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_chopper_supply_protection(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.chop_conf_reg.chop_conf_pins.diss2vs;
}

/**
 * \brief                       Get smart energy current minimum.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_cool_se_current_min(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.seimin;
}

/**
 * \brief                       Get smart energy current down step.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_cool_se_current_down_step(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.sedn;
}

/**
 * \brief                       Get smart energy hysteresis.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_cool_se_hysteresis(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.semax;
}

/**
 * \brief                       Get smart energy current up step.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_cool_se_current_up_step(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.seup;
}

/**
 * \brief                       Get smart energy hysteresis start.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_cool_se_hysteresis_start(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.semin;
}

/**
 * \brief                       Get stallGuard2 filter.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_cool_sg_filter(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.sfilt;
}

/**
 * \brief                       Get stallGuard2 threshold.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_cool_sg_thresh(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.cool_conf_reg.cool_conf_pins.sgt;
}

/**
 * \brief                       Get smart energy stall velocity.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_se_velocity_stall(tmc5160_channels_enum_t channel)
{
	int32_t value;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.tcoolthrs_reg.addr;

	if (tmc5160[(uint8_t)channel].ramp_gen_regs.sw_mode_reg.sw_mode_pins.sg_stop)
	{
		value = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
		return MIN(0xFFFF, (1 << 24) / ((value) ? value : 1));
	}
	else
	{
		value = 0;
		return value;
	}
}

/**
 * \brief                       Get smart energy threshold speed.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_se_speed_thresh(tmc5160_channels_enum_t channel)
{
	int32_t buffer;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.tcoolthrs_reg.addr;
	buffer = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);

	return MIN(0xFFFF, (1 << 24) / ((buffer) ? buffer : 1));
}

/**
 * \brief                       Get stallguard2 value and driver error flags.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint32_t tmc5160_get_drv_status(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return reserved;
}

/**
 * \brief                       Get stallguard result(load value).
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint16_t tmc5160_get_drv_status_sg_result(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.sg_result;
}

/**
 * \brief                       Get short to supply indicator phase A.
 *                              1: Short to supply detected on phase A or B. The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the DRV_ENN input. Sense resistor voltage drop is included in the measurement!
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_s2vsa(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.s2vsa;
}

/**
 * \brief                       Get short to supply indicator phase B.
 *                              1: Short to supply detected on phase A or B. The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the DRV_ENN input. Sense resistor voltage drop is included in the measurement!
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_s2vsb(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.s2vsb;
}

/**
 * \brief                       Get StealthChop indicator.
 *                              1: Driver operates in StealthChop mode.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_stealth(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.stealth;
}

/**
 * \brief                       Get full step active indicator.
 *                              1: Indicates that the driver has switched to fullstep as defined by chopper mode settings and velocity thresholds.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_fsactive(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.fsactive;
}

/**
 * \brief                       Get actual motor current / smart energy current.
 *                              Actual current control scaling, for monitoring smart energy current scaling controlled via settings in register COOLCONF, or for monitoring the function of the automatic current scaling.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_drv_status_cs_actual(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.cs_actual;
}

/**
 * \brief                       Get stallguard2 status.
 *                              1: Motor stall detected (SG_RESULT=0) or DcStep stall in DcStep mode.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_stallguard(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.sg;
}

/**
 * \brief                       Get overtemperature flag.
 *                              1: Overtemperature limit has been reached. Drivers become disabled until otpw is also cleared due to cooling down of the IC. The overtemperature flag is common for both bridges.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_ot(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.ot;
}

/**
 * \brief                       Get overtemperature pre-warning flag.
 *                              1: Overtemperature pre-warning threshold is exceeded. The overtemperature pre-warning flag is common for both bridges.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_otpw(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.otpw;
}

/**
 * \brief                       Get short to ground indicator phase A.
 *                              1: Short to GND detected on phase A or B. The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the DRV_ENN input.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_s2ga(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.s2ga;
}

/**
 * \brief                       Get short to ground indicator phase B.
 *                              1: Short to GND detected on phase A or B. The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the DRV_ENN input.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_s2gb(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.s2gb;
}

/**
 * \brief                       Get open load indicator phase A.
 *                              1: Open load detected on phase A or B.
 *                              Hint: This is just an informative flag. The driver takes no action upon it. False detection may occur in fast motion and standstill. Check during slow motion, only.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_ola(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.ola;
}

/**
 * \brief                       Get open load indicator phase B.
 *                              1: Open load detected on phase A or B.
 *                              Hint: This is just an informative flag. The driver takes no action upon it. False detection may occur in fast motion and standstill. Check during slow motion, only.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_olb(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.olb;
}

/**
 * \brief                       Get standstill indicator.
 *                              This flag indicates motor stand still in each operation mode. This occurs 2^20 clocks after the last step pulse.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_drv_status_stst(tmc5160_channels_enum_t channel)
{
	uint32_t reserved;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.addr;
	reserved = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);
	tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.reserved = reserved;

	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.drv_status_reg.drv_status_pins.stst;
}

/**
 * \brief                       Set pwm threshold speed.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_pwm_thresh_speed(tmc5160_channels_enum_t channel)
{
	int32_t buffer;
	uint8_t addr;

	addr = tmc5160[(uint8_t)channel].vel_regs.tpwmthrs_reg.addr;
	buffer = tmc5160_spi_read_reg(channel, addr, TMC5160_SPI_TIMEOUT);

	return MIN(0xFFFFF, (1 << 24) / ((buffer) ? buffer : 1));
}

/**
 * \brief                       Get voltage PWM mode StealthChop.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint32_t tmc5160_get_pwm_config(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.reserved;
}

/**
 * \brief                       Get pwm gradient.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_pwm_gradient(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_grad;
}

/**
 * \brief                       Get offset amplitude.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_pwm_ofs_amplitude(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_ofs;
}

/**
 * \brief                       Get pwm frequency.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_pwm_frequency(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_freq;
}

/**
 * \brief                       Get pwm autoscale.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_pwm_autoscale(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_autoscale;
}

/**
 * \brief                       Get pwm automatic gradient adaptation.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
bool tmc5160_get_pwm_autograd(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_autograd;
}

/**
 * \brief                       Get freewheeling mode.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_pwm_freewheeling_mode(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.freewheel;
}

/**
 * \brief                       Get regulation loop gradient.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_pwm_regulation(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_regulation;
}

/**
 * \brief                       Get PWM automatic scale amplitude limit when switching on.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_pwm_limit(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].motor_drv_regs.drv_control.pwm_conf_reg.pwm_conf_pins.pwm_lim;
}

/**
 * \brief                       Get encoder position.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint32_t tmc5160_get_enc_pos(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].enc_regs.xenc_reg.xenc;
}

/**
 * \brief                       Get encoder resolution.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
int32_t tmc5160_get_enc_res(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].enc_regs.enc_const_reg.enc_const;
}

/**
 * \brief                       Global scaling of Motor current. This value is multiplied to the current scaling to adapt a drive to a certain motor type.
 *                              This value should be chosen before tuning other settings because it also influences chopper hysteresis.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_global_scaler(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].gconf_regs.global_scaler_reg.global_scaler;
}

/**
 * \brief                       Get standstill current.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_ihold(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.ihold_irun_pins.ihold;
}

/**
 * \brief                       Get motor run current.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_irun(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.ihold_irun_pins.irun;
}

/**
 * \brief                       Get ihold_delay.
 * \param[in] channel           Motor channel information.
 * \retval                      Value read from register.
 */
uint8_t tmc5160_get_ihold_delay(tmc5160_channels_enum_t channel)
{
	return tmc5160[(uint8_t)channel].vel_regs.ihold_irun_reg.ihold_irun_pins.ihold_delay;
}
