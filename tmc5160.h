/****************************************************************************/
/* File Name    : tmc5160.h                                                 */
/* Description  : TMC5160 stepper motor driver header file                  */
/* Author       : Hasan KAYNAR                                              */
/* Initial Date : 27.12.2022                                                */
/* Updated Date : 29.03.2023                                                */
/* Version      : 1.3                                                       */
/****************************************************************************/

#ifndef _TMC5160_H_
#define _TMC5160_H_

/*--------------------------------------------------------------------------*/
/* Includes                                                                 */
/*--------------------------------------------------------------------------*/
#include "main.h"

/*--------------------------------------------------------------------------*/
/* TMC5160 Constants                                                        */
/*--------------------------------------------------------------------------*/
#define TMC5160_WRITE_BIT        0x80         /* Collect with this macro to do a write operation */
#define TMC5160_ADDRESS_MASK     0x7F         /* Mask to remove write access */
#define TMC5160_REG_COUNT        128          /* Total number of registers */
#define TMC5160_DATA_SIZE        5U           /* Number of data to be read and written */
#ifndef TMC5160_MOTOR_COUNT
#define TMC5160_MOTOR_COUNT      3U           /* Enter how many drives you will use (Each driver represents 1 motor). */
#endif
#define TMC5160_TIMER_PERIOD     1U           /* TMC5160 library timer callback period mS  */
#define TMC5160_SPI_TIMEOUT      1000U        /* 1000mS for timeout parameter */
#define TMC5160_WORK_TIME        5000U        /* Period of work to be done periodically */
#define TMC5160_LIBRARY_CODE     16           /* The library code defined in the document "16 - Definitions for tmc5160.xlsx" */
#define TMC5160_BIG_ENDIAN       0            /* Big endian is used for arm processors */
#define TMC5160_LITTLE_ENDIAN    1            /* For processor architectures that use little endian, set it to 1 */
#define TMC5160_LOGIC_HIGH       1            /* Means 1 as a logic level */
#define TMC5160_LOGIC_LOW        0            /* Means 0 as a logic level */

/*--------------------------------------------------------------------------*/
/* TMC5160 Register Access - Permission Combinations                        */
/*--------------------------------------------------------------------------*/
#define TMC5160_ACCESS_NONE         0x00      /* No access permission (Reserved). */
#define TMC5160_ACCESS_READ         0x01      /* Read permission */
#define TMC5160_ACCESS_WRITE        0x02      /* Write permission */
#define TMC5160_ACCESS_RW           0x03      /* Read and write (TMC5160_ACCESS_READ | TMC5160_ACCESS_WRITE) */
#define TMC5160_ACCESS_DIRTY        0x08      /* Register has been written since reset -> shadow register is valid for restore */
#define TMC5160_ACCESS_RW_SEPARATE  0x13      /* Read and write, with separate values/functions                  (TMC5160_ACCESS_RW    | TMC5160_ACCESS_RW_SPECIAL) */
#define TMC5160_ACCESS_R_FLAGS      0x21      /* Read, has flags (read to clear)                                 (TMC5160_ACCESS_READ  | TMC5160_ACCESS_FLAGS)      */
#define TMC5160_ACCESS_RW_FLAGS     0x23      /* Read and write, has flags (read or write to clear)              (TMC5160_ACCESS_RW    | TMC5160_ACCESS_FLAGS)      */
#define TMC5160_ACCESS_W_PRESET     0x42      /* Write, has hardware preset - skipped in reset routine           (TMC5160_ACCESS_WRITE | TMC5160_ACCESS_HW_PRESET)  */
#define TMC5160_ACCESS_RW_PRESET    0x43      /* Read and write, has hardware presets - skipped in reset routine (TMC5160_ACCESS_RW    | TMC5160_ACCESS_HW_PRESET)  */

/*--------------------------------------------------------------------------*/
/* TMC5160 Register Access - Special Register Bits                          */
/*--------------------------------------------------------------------------*/
#define TMC5160_ACCESS_RW_SPECIAL   0x10      /* Read and write are independent - different values and/or different functions */
#define TMC5160_ACCESS_FLAGS        0x20      /* Register has read or write to clear flags. */
#define TMC5160_ACCESS_HW_PRESET    0x40      /* Register has hardware presets (e.g. Factory calibrations) - do not write a default value */

/*--------------------------------------------------------------------------*/
/* TMC5160 Helper Macros                                                    */
/*--------------------------------------------------------------------------*/
#define TMC5160_ADDRESS(x)          ((x) & (TMC5160_ADDRESS_MASK))                                                                     /* Macro to remove write bit for shadow register array access */
#define TMC5160_IS_READABLE(x)      ((x) & TMC5160_ACCESS_READ)
#define TMC5160_IS_WRITABLE(x)      ((x) & TMC5160_ACCESS_WRITE)
#define TMC5160_IS_DIRTY(x)         ((x) & TMC5160_ACCESS_DIRTY)
#define TMC5160_IS_PRESET(x)        ((x) & TMC5160_ACCESS_HW_PRESET)
#define TMC5160_IS_RESETTABLE(x)    (((x) & (TMC5160_ACCESS_W_PRESET)) == TMC5160_ACCESS_WRITE)                                        /* Write bit set, Hardware preset bit not set */
#define TMC5160_IS_RESTORABLE(x)    (((x) & TMC5160_ACCESS_WRITE) && (!(x & TMC5160_ACCESS_HW_PRESET) || (x & TMC5160_ACCESS_DIRTY)))  /* Write bit set, if it's a hardware preset register, it needs to be dirty */

/*--------------------------------------------------------------------------*/
/* TMC5160 Maximum Limit Values                                             */
/*--------------------------------------------------------------------------*/
#define TMC5160_MAX_VELOCITY        8388096U
#define TMC5160_MAX_ACCELERATION    65535U

/*
 * \brief                       Use this enumeration to enable or disable the driver
 */
typedef enum tmc5160_driver_state_enum
{
    TMC5160_DRIVER_ENABLE, TMC5160_DRIVER_DISABLE
} tmc5160_driver_state_enum_t;

/*
 * \brief                       Error enumeration
 */
typedef enum tmc5160_error_enum
{
    RESULT_ERR_TMC5160_NONE = 0x00,
    RESULT_ERR_TMC5160_GENERIC = 0x01,
    RESULT_ERR_TMC5160_FUNCTION = 0x02,
    RESULT_ERR_TMC5160_MOTOR = 0x08,
    RESULT_ERR_TMC5160_VALUE = 0x10,
    RESULT_ERR_TMC5160_CHIP = 0x40
} tmc5160_error_enum_t;

/*
 * \brief                       Communication mode
 */
typedef enum tmc5160_comm_mode_enum
{
    TMC5160_COMM_DEFAULT, TMC5160_COMM_SPI, TMC5160_COMM_UART
} tmc5160_comm_mode_enum_t;

/*
 * \brief                       Configuration state enumeration
 */
typedef enum tmc5160_config_state_enum
{
    TMC5160_CONFIG_READY, TMC5160_CONFIG_RESET, TMC5160_CONFIG_RESTORE
} tmc5160_config_state_enum_t;

#ifndef TMC5160_MOTOR_COUNT
/**
 * \brief                       TMC5160 motor channels.
 */
typedef enum tmc5160_channels_enum
{
    TMC5160_CHANNEL_1, TMC5160_CHANNEL_2, TMC5160_CHANNEL_3, TMC5160_CHANNEL_4
} tmc5160_channels_enum_t;
#endif
/**
 * \brief                       TMC5160 microstep resolution.
 */
typedef enum microstep_res_enum
{
    MICROSTEP_256,
    MICROSTEP_128,
    MICROSTEP_64,
    MICROSTEP_32,
    MICROSTEP_16,
    MICROSTEP_8,
    MICROSTEP_4,
    MICROSTEP_2,
    FULLSTEP
} microstep_res_enum_t;

/**
 * \brief                       TMC5160 ramp type.
 */
typedef enum tmc5160_ramp_type_enum
{
    TMC5160_MODE_POSITION,
    TMC5160_MODE_VELPOS,
    TMC5160_MODE_VELNEG,
    TMC5160_MODE_HOLD
} tmc5160_ramp_type_enum_t;

/*
 * \brief                       General Configuration Registers.
 */
typedef struct tmc5160_gconf_regs_struct
{
    struct
    {
        uint8_t addr; /* 0x00 - RW */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint32_t recalibrate : 1;
				uint32_t faststandstill : 1;
				uint32_t en_pwm_mode : 1;
				uint32_t multistep_filt : 1;
				uint32_t shaft : 1;
				uint32_t diag0_err : 1;
				uint32_t diag0_otpw : 1;
				uint32_t diag0_stall : 1;
				uint32_t diag1_stall : 1;
				uint32_t diag1_index : 1;
				uint32_t diag1_onstate : 1;
				uint32_t diag1_steps_skipped : 1;
				uint32_t diag0_int_pushpull : 1;
				uint32_t diag1_poscomp_pushpull : 1;
				uint32_t small_hysteresis : 1;
				uint32_t stop_enable : 1;
				uint32_t direct_mode : 1;
				uint32_t test_mode : 1;
				uint32_t _bit18 : 1;
				uint32_t _bit19 : 1;
				uint32_t _bit20 : 1;
				uint32_t _bit21 : 1;
				uint32_t _bit22 : 1;
				uint32_t _bit23 : 1;
				uint32_t _bit24 : 1;
				uint32_t _bit25 : 1;
				uint32_t _bit26 : 1;
				uint32_t _bit27 : 1;
				uint32_t _bit28 : 1;
				uint32_t _bit29 : 1;
				uint32_t _bit30 : 1;
				uint32_t _bit31 : 1;
#else
                uint32_t _bit31 :1;
                uint32_t _bit30 :1;
                uint32_t _bit29 :1;
                uint32_t _bit28 :1;
                uint32_t _bit27 :1;
                uint32_t _bit26 :1;
                uint32_t _bit25 :1;
                uint32_t _bit24 :1;
                uint32_t _bit23 :1;
                uint32_t _bit22 :1;
                uint32_t _bit21 :1;
                uint32_t _bit20 :1;
                uint32_t _bit19 :1;
                uint32_t _bit18 :1;
                uint32_t test_mode :1;
                uint32_t direct_mode :1;
                uint32_t stop_enable :1;
                uint32_t small_hysteresis :1;
                uint32_t diag1_poscomp_pushpull :1;
                uint32_t diag0_int_pushpull :1;
                uint32_t diag1_steps_skipped :1;
                uint32_t diag1_onstate :1;
                uint32_t diag1_index :1;
                uint32_t diag1_stall :1;
                uint32_t diag0_stall :1;
                uint32_t diag0_otpw :1;
                uint32_t diag0_err :1;
                uint32_t shaft :1;
                uint32_t multistep_filt :1;
                uint32_t en_pwm_mode :1;
                uint32_t faststandstill :1;
                uint32_t recalibrate :1;
#endif
            };
            uint32_t reserved;
        } gconf_flag; /* Global configuration flag */
    } gconf_reg; /* Global configuration register */

    struct
    {
        uint8_t addr; /* 0x01 - R+WC */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint8_t reset : 1;     /* Indicates that the IC has been reset. All registers have been cleared to reset values. */
				uint8_t drv_err : 1;   /* Indicates, that the driver has been shut down due to overtemperature or short circuit detection. Read DRV_STATUS for details. The flag can only be cleared when the temperature is below the limit again. */
				uint8_t uv_cp : 1;     /* Indicates an undervoltage on the charge pump. The driver is disabled during undervoltage. This flag is latched for information. */
				uint8_t _bit3 : 1;
				uint8_t _bit4 : 1;
				uint8_t _bit5 : 1;
				uint8_t _bit6 : 1;
				uint8_t _bit7 : 1;
#else
                uint8_t _bit7 :1;
                uint8_t _bit6 :1;
                uint8_t _bit5 :1;
                uint8_t _bit4 :1;
                uint8_t _bit3 :1;
                uint8_t uv_cp :1; /* Indicates an undervoltage on the charge pump. The driver is disabled during undervoltage. This flag is latched for information. */
                uint8_t drv_err :1; /* Indicates, that the driver has been shut down due to overtemperature or short circuit detection. Read DRV_STATUS for details. The flag can only be cleared when the temperature is below the limit again. */
                uint8_t reset :1; /* Indicates that the IC has been reset. All registers have been cleared to reset values. */
#endif
            };
            uint8_t reserved;
        } gstat_flag; /* Re-Write with ‘1’ bit to clear respective flags */
    } gstat_reg; /* Global status register */

    struct
    {
        uint8_t addr; /* 0x04 - R */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint16_t ref_left_step : 1;
				uint16_t ref_right_dir : 1;
				uint16_t encb_dcen_cfg4 : 1;
				uint16_t enca_dcin_cfg5 : 1;
				uint16_t drv_enn : 1;
				uint16_t encn_dco_cfg6 : 1;
				uint16_t sd_mode : 1;         /* 1 = External step and dir source */
				uint16_t swcomp_in : 1;       /* Shows voltage difference of SWN and SWP. Bring DIAG outputs to high level with pushpull disabled to test the comparator. */
				uint16_t _bit8 : 1;
				uint16_t _bit9 : 1;
				uint16_t _bit10 : 1;
				uint16_t _bit11 : 1;
				uint16_t _bit12 : 1;
				uint16_t _bit13 : 1;
				uint16_t _bit14 : 1;
				uint16_t _bit15 : 1;
#else
                uint16_t _bit15 :1;
                uint16_t _bit14 :1;
                uint16_t _bit13 :1;
                uint16_t _bit12 :1;
                uint16_t _bit11 :1;
                uint16_t _bit10 :1;
                uint16_t _bit9 :1;
                uint16_t _bit8 :1;
                uint16_t swcomp_in :1; /* Shows voltage difference of SWN and SWP. Bring DIAG outputs to high level with pushpull disabled to test the comparator. */
                uint16_t sd_mode :1; /* 1 = External step and dir source */
                uint16_t encn_dco_cfg6 :1;
                uint16_t drv_enn :1;
                uint16_t enca_dcin_cfg5 :1;
                uint16_t encb_dcen_cfg4 :1;
                uint16_t ref_right_dir :1;
                uint16_t ref_left_step :1;
#endif
            };
            uint16_t reserved;
        } input_pins; /* Reads the state of all input pins available*/
    } io_input_reg;

    struct
    {
        uint8_t addr; /* 0x05 - W */
        uint32_t xcomp; /* XACTUAL = X_COMPARE: Output signal PP (position pulse) becomes high. It returns to a low state if the positions mismatch. */
    } xcomp_reg; /* Position comparison register for motion controller position strobe. The Position pulse is available on output SWP_DIAG1. */

    struct
    {
        uint8_t addr; /* 0x06 - W */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint16_t otp_bit : 3;
				uint16_t _bit3 : 1;
				uint16_t otp_byte : 2;
				uint16_t _bit6 : 1;
				uint16_t _bit7 : 1;
				uint16_t otp_magic : 8;
#else
                uint16_t otp_magic :8;
                uint16_t _bit7 :1;
                uint16_t _bit6 :1;
                uint16_t otp_byte :2;
                uint16_t _bit3 :1;
                uint16_t otp_bit :3;
#endif
            };
            uint16_t reserved;
        } otp_write_pins;
    } otp_prog_write_reg;

    struct
    {
        uint8_t addr; /* 0x07 - R */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint8_t otp_fclktrim : 5;   /* Reset default for FCLKTRIM:                0: lowest frequency setting,   31: highest frequency setting. */
				uint8_t otp_s2_level : 1;   /* Reset default for short-detection Levels:  0: S2G_LEVEL = 6,              1: S2G_LEVEL = 12              */
				uint8_t otp_bbm : 1;        /* Reset default for DRVCONF.BBMCLKS:         0: BBMCLKS = 4,                1: BBMCLKS = 2                 */
				uint8_t otp_tbl : 1;        /* Reset default for TBL:                     0: TBL = %10 (~3us),           1: TBL = %01 (~2us)            */
#else
                uint8_t otp_tbl :1; /* Reset default for TBL:                     0: TBL = %10 (~3us),           1: TBL = %01 (~2us)            */
                uint8_t otp_bbm :1; /* Reset default for DRVCONF.BBMCLKS:         0: BBMCLKS = 4,                1: BBMCLKS = 2                 */
                uint8_t otp_s2_level :1; /* Reset default for short-detection Levels:  0: S2G_LEVEL = 6,              1: S2G_LEVEL = 12              */
                uint8_t otp_fclktrim :5; /* Reset default for FCLKTRIM:                0: lowest frequency setting,   31: highest frequency setting. */
#endif
            };
            uint8_t reserved;
        } otp_read_pins;
    } otp_prog_read_reg;

    struct
    {
        uint8_t addr; /* 0x08 - RW */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint8_t factory_conf : 5;
				uint8_t _bit5 : 1;
				uint8_t _bit6 : 1;
				uint8_t _bit7 : 1;
#else
                uint8_t _bit7 :1;
                uint8_t _bit6 :1;
                uint8_t _bit5 :1;
                uint8_t factory_conf :5;
#endif
            };
            uint8_t reserved;
        };
    } factory_conf_reg;

    /* SHORT_CONF Register */
    struct
    {
        uint8_t addr; /* 0x09 - W */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint32_t s2vs_level : 4;
				uint32_t _bit4 : 1;
				uint32_t _bit5 : 1;
				uint32_t _bit6 : 1;
				uint32_t _bit7 : 1;
				uint32_t s2g_level : 4;
				uint32_t _bit12 : 1;
				uint32_t _bit13 : 1;
				uint32_t _bit14 : 1;
				uint32_t _bit15 : 1;
				uint32_t short_filter : 2;
				uint32_t short_delay : 1;
				uint32_t _bit19 : 1;
				uint32_t _bit20 : 1;
				uint32_t _bit21 : 1;
				uint32_t _bit22 : 1;
				uint32_t _bit23 : 1;
				uint32_t _bit24 : 1;
				uint32_t _bit25 : 1;
				uint32_t _bit26 : 1;
				uint32_t _bit27 : 1;
				uint32_t _bit28 : 1;
				uint32_t _bit29 : 1;
				uint32_t _bit30 : 1;
				uint32_t _bit31 : 1;
#else
                uint32_t _bit31 :1;
                uint32_t _bit30 :1;
                uint32_t _bit29 :1;
                uint32_t _bit28 :1;
                uint32_t _bit27 :1;
                uint32_t _bit26 :1;
                uint32_t _bit25 :1;
                uint32_t _bit24 :1;
                uint32_t _bit23 :1;
                uint32_t _bit22 :1;
                uint32_t _bit21 :1;
                uint32_t _bit20 :1;
                uint32_t _bit19 :1;
                uint32_t short_delay :1;
                uint32_t short_filter :2;
                uint32_t _bit15 :1;
                uint32_t _bit14 :1;
                uint32_t _bit13 :1;
                uint32_t _bit12 :1;
                uint32_t s2g_level :4;
                uint32_t _bit7 :1;
                uint32_t _bit6 :1;
                uint32_t _bit5 :1;
                uint32_t _bit4 :1;
                uint32_t s2vs_level :4;
#endif
            };
            uint32_t reserved;
        } conf_pins;
    } short_conf_reg;

    struct
    {
        uint8_t addr; /* 0x0A - W */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint32_t bbmtime : 5;
				uint32_t _bit5 : 1;
				uint32_t _bit6 : 1;
				uint32_t _bit7 : 1;
				uint32_t bbmclks : 4;
				uint32_t _bit12 : 1;
				uint32_t _bit13 : 1;
				uint32_t _bit14 : 1;
				uint32_t _bit15 : 1;
				uint32_t otselect : 2;
				uint32_t drvstrength : 2;
				uint32_t filt_isense : 2;
				uint32_t _bit22 : 1;
				uint32_t _bit23 : 1;
				uint32_t _bit24 : 1;
				uint32_t _bit25 : 1;
				uint32_t _bit26 : 1;
				uint32_t _bit27 : 1;
				uint32_t _bit28 : 1;
				uint32_t _bit29 : 1;
				uint32_t _bit30 : 1;
				uint32_t _bit31 : 1;
#else
                uint32_t _bit31 :1;
                uint32_t _bit30 :1;
                uint32_t _bit29 :1;
                uint32_t _bit28 :1;
                uint32_t _bit27 :1;
                uint32_t _bit26 :1;
                uint32_t _bit25 :1;
                uint32_t _bit24 :1;
                uint32_t _bit23 :1;
                uint32_t _bit22 :1;
                uint32_t filt_isense :2;
                uint32_t drvstrength :2;
                uint32_t otselect :2;
                uint32_t _bit15 :1;
                uint32_t _bit14 :1;
                uint32_t _bit13 :1;
                uint32_t _bit12 :1;
                uint32_t bbmclks :4;
                uint32_t _bit7 :1;
                uint32_t _bit6 :1;
                uint32_t _bit5 :1;
                uint32_t bbmtime :5;
#endif
            };
            uint32_t reserved;
        } conf_pins;
    } drv_conf_reg;

    struct
    {
        uint8_t addr; /* 0x0B - W */
        uint8_t global_scaler;
    } global_scaler_reg;

    struct
    {
        uint8_t addr; /* 0x0C - R */
        union
        {
            struct
            {
#if BIG_ENDIAN
				int16_t cal_result_phaseA : 8;    /* Offset calibration result phase A */
				int16_t cal_result_phaseB : 8;    /* Offset calibration result phase B */
#else
                int16_t cal_result_phaseB :8; /* Offset calibration result phase B */
                int16_t cal_result_phaseA :8; /* Offset calibration result phase A */
#endif
            };
            int16_t reserved;
        };
    } offset_read_reg;
} tmc5160_gconf_regs_struct_t;

/**
 * \brief                       Velocity Dependent Driver Feature Control Register Set
 */
typedef struct tmc5160_vel_regs_struct
{
    struct
    {
        uint8_t addr; /* 0x10 - Write */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint32_t ihold : 5;
				uint32_t _bit5 : 1;
				uint32_t _bit6 : 1;
				uint32_t _bit7 : 1;
				uint32_t irun : 5;
				uint32_t _bit13 : 1;
				uint32_t _bit14 : 1;
				uint32_t _bit15 : 1;
				uint32_t ihold_delay : 4;
				uint32_t _bit20 : 1;
				uint32_t _bit21 : 1;
				uint32_t _bit22 : 1;
				uint32_t _bit23 : 1;
				uint32_t _bit24 : 1;
				uint32_t _bit25 : 1;
				uint32_t _bit26 : 1;
				uint32_t _bit27 : 1;
				uint32_t _bit28 : 1;
				uint32_t _bit29 : 1;
				uint32_t _bit30 : 1;
				uint32_t _bit31 : 1;
#else
                uint32_t _bit31 :1;
                uint32_t _bit30 :1;
                uint32_t _bit29 :1;
                uint32_t _bit28 :1;
                uint32_t _bit27 :1;
                uint32_t _bit26 :1;
                uint32_t _bit25 :1;
                uint32_t _bit24 :1;
                uint32_t _bit23 :1;
                uint32_t _bit22 :1;
                uint32_t _bit21 :1;
                uint32_t _bit20 :1;
                uint32_t ihold_delay :4;
                uint32_t _bit15 :1;
                uint32_t _bit14 :1;
                uint32_t _bit13 :1;
                uint32_t irun :5;
                uint32_t _bit7 :1;
                uint32_t _bit6 :1;
                uint32_t _bit5 :1;
                uint32_t ihold :5;
#endif
            };
            uint32_t reserved;
        } ihold_irun_pins;
    } ihold_irun_reg;

    struct
    {
        uint8_t addr; /* 0x11 - Write */
        uint8_t tpower_down;
    } tpower_down_reg;

    struct
    {
        uint8_t addr; /* 0x12 - Read */
        uint32_t tstep;
    } tstep_reg;

    struct
    {
        uint8_t addr; /* 0x13 - Write */
        uint32_t tpwmthrs;
    } tpwmthrs_reg;

    struct
    {
        uint8_t addr; /* 0x14 - Write */
        uint32_t tcoolthrs;
    } tcoolthrs_reg;

    struct
    {
        uint8_t addr; /* 0x15 - Write */
        uint32_t thigh;
    } thigh_reg;
} tmc5160_vel_regs_struct_t;

/**
 * \brief                       Ramp Generator Motion Control Register Set
 */
typedef struct tmc5160_ramp_gen_regs_struct
{
    struct
    {
        uint8_t addr; /* 0x20 - RW */
        uint8_t ramp_mode;
    } ramp_mode_reg;

    struct
    {
        uint8_t addr; /* 0x21 - RW */
        int32_t xactual;
    } xactual_reg;

    struct
    {
        uint8_t addr; /* 0x22 - Read */
        int32_t vactual;
    } vactual_reg;

    struct
    {
        uint8_t addr; /* 0x23 - Write */
        uint32_t vstart;
    } vstart_reg;

    struct
    {
        uint8_t addr; /* 0x24 - Write */
        uint16_t a1;
    } a1_reg;

    struct
    {
        uint8_t addr; /* 0x25 - Write */
        uint32_t v1;
    } v1_reg;

    struct
    {
        uint8_t addr; /* 0x26 - Write */
        uint16_t amax;
    } amax_reg;

    struct
    {
        uint8_t addr; /* 0x27 - Write */
        uint32_t vmax;
    } vmax_reg;

    struct
    {
        uint8_t addr; /* 0x28 - Write */
        uint16_t dmax;
    } dmax_reg;

    struct
    {
        uint8_t addr; /* 0x2A - Write */
        uint16_t d1;
    } d1_reg;

    struct
    {
        uint8_t addr; /* 0x2B - Write */
        uint32_t vstop;
    } vstop_reg;

    struct
    {
        uint8_t addr; /* 0x2C - Write */
        uint16_t tzero_wait;
    } tzero_wait_reg;

    struct
    {
        uint8_t addr; /* 0x2D - RW */
        int32_t xtarget;
    } xtarget_reg;

    struct
    {
        uint8_t addr; /* 0x33 - Write */
        uint32_t vdc_min;
    } vdc_min_reg;

    struct
    {
        uint8_t addr; /* 0x34 - RW */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint16_t stop_en_left : 1;
				uint16_t stop_en_right : 1;
				uint16_t pol_stop_left : 1;
				uint16_t pol_stop_right : 1;
				uint16_t swap_left_right : 1;
				uint16_t latch_active_left : 1;
				uint16_t latch_inactive_left : 1;
				uint16_t latch_active_right : 1;
				uint16_t latch_inactive_right : 1;
				uint16_t en_latch_enc : 1;
				uint16_t sg_stop : 1;
				uint16_t en_softstop : 1;
				uint16_t _bit12 : 1;
				uint16_t _bit13 : 1;
				uint16_t _bit14 : 1;
				uint16_t _bit15 : 1;
#else
                uint16_t _bit15 :1;
                uint16_t _bit14 :1;
                uint16_t _bit13 :1;
                uint16_t _bit12 :1;
                uint16_t en_softstop :1;
                uint16_t sg_stop :1;
                uint16_t en_latch_enc :1;
                uint16_t latch_inactive_right :1;
                uint16_t latch_active_right :1;
                uint16_t latch_inactive_left :1;
                uint16_t latch_active_left :1;
                uint16_t swap_left_right :1;
                uint16_t pol_stop_right :1;
                uint16_t pol_stop_left :1;
                uint16_t stop_en_right :1;
                uint16_t stop_en_left :1;
#endif
            };
            uint16_t reserved;
        } sw_mode_pins;
    } sw_mode_reg; /* Reference Switch & StallGuard2 Event Configuration Register */

    struct
    {
        uint8_t addr; /* 0x35 - R+WC */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint16_t status_stop_left : 1;     /* R    */
				uint16_t status_stop_right : 1;    /* R    */
				uint16_t status_latch_left : 1;    /* R+WC */
				uint16_t status_latch_right : 1;   /* R+WC */
				uint16_t event_stop_left : 1;      /* R    */
				uint16_t event_stop_right : 1;     /* R    */
				uint16_t event_stop_sg : 1;        /* R+WC */
				uint16_t event_pos_reached : 1;    /* R+WC */
				uint16_t reached_vel : 1;          /* R    */
				uint16_t reached_pos : 1;          /* R    */
				uint16_t vzero : 1;                /* R    */
				uint16_t tzero_wait_active : 1;    /* R    */
				uint16_t second_move : 1;          /* R+WC */
				uint16_t status_sg : 1;            /* R    */
				uint16_t _bit14 : 1;
				uint16_t _bit15 : 1;
#else
                uint16_t _bit15 :1;
                uint16_t _bit14 :1;
                uint16_t status_sg :1; /* R    */
                uint16_t second_move :1; /* R+WC */
                uint16_t tzero_wait_active :1; /* R    */
                uint16_t vzero :1; /* R    */
                uint16_t reached_pos :1; /* R    */
                uint16_t reached_vel :1; /* R    */
                uint16_t event_pos_reached :1; /* R+WC */
                uint16_t event_stop_sg :1; /* R+WC */
                uint16_t event_stop_right :1; /* R    */
                uint16_t event_stop_left :1; /* R    */
                uint16_t status_latch_right :1; /* R+WC */
                uint16_t status_latch_left :1; /* R+WC */
                uint16_t status_stop_right :1; /* R    */
                uint16_t status_stop_left :1; /* R    */
#endif
            };
            uint16_t reserved;
        } ramp_stat_pins;
    } ramp_stat_reg;

    struct
    {
        uint8_t addr; /* 0x36 - Read */
        uint32_t xlatch;
    } xlatch_reg;
} tmc5160_ramp_gen_regs_struct_t;

/**
 * \brief                       Encoder Registers
 * \note                        The encoder interface is not available in Step&Direction mode, as the
 *                              encoder pins serve a different function in that mode.
 */
typedef struct tmc5160_enc_regs_struct
{
    struct
    {
        uint8_t addr; /* 0x38 - RW */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint16_t pol_A : 1;
				uint16_t pol_B : 1;
				uint16_t pol_N : 1;
				uint16_t ignore_AB : 1;
				uint16_t clr_cont : 1;
				uint16_t clr_once : 1;
				uint16_t edge_pos : 1;
				uint16_t edge_neg : 1;
				uint16_t clr_enc_x : 1;
				uint16_t latch_x_act : 1;
				uint16_t enc_sel_dec : 1;
				uint16_t _bit_11 : 1;
				uint16_t _bit_12 : 1;
				uint16_t _bit_13 : 1;
				uint16_t _bit_14 : 1;
				uint16_t _bit_15 : 1;
#else
                uint16_t _bit_15 :1;
                uint16_t _bit_14 :1;
                uint16_t _bit_13 :1;
                uint16_t _bit_12 :1;
                uint16_t _bit_11 :1;
                uint16_t enc_sel_dec :1;
                uint16_t latch_x_act :1;
                uint16_t clr_enc_x :1;
                uint16_t edge_neg :1;
                uint16_t edge_pos :1;
                uint16_t clr_once :1;
                uint16_t clr_cont :1;
                uint16_t ignore_AB :1;
                uint16_t pol_N :1;
                uint16_t pol_B :1;
                uint16_t pol_A :1;
#endif
            };
            uint16_t reserved;
        } enc_mode_pins;
    } enc_mode_reg;

    struct
    {
        uint8_t addr; /* 0x39 - RW */
        int32_t xenc;
    } xenc_reg;

    struct
    {
        uint8_t addr; /* 0x3A - W */
        int32_t enc_const;
    } enc_const_reg;

    struct
    {
        uint8_t addr; /* 0x3B - R+WC */
        uint8_t enc_status;
    } enc_status_reg;

    struct
    {
        uint8_t addr; /* 0x3C - R */
        uint32_t enc_latch;
    } enc_latch_reg;

    struct
    {
        uint8_t addr; /* 0x3D - W */
        uint32_t enc_deviation;
    } enc_deviation_reg;
} tmc5160_enc_regs_struct;

/**
 * \brief                       Microstepping Control Registers
 */
typedef struct mstep_control_regs_struct
{
    struct
    {
        uint8_t addr; /* 0x60 - Write */
        uint32_t mslut0;
    } mslut0_reg;

    struct
    {
        uint8_t addr; /* 0x61 - Write */
        uint32_t mslut1;
    } mslut1_reg;

    struct
    {
        uint8_t addr; /* 0x62 - Write */
        uint32_t mslut2;
    } mslut2_reg;

    struct
    {
        uint8_t addr; /* 0x63 - Write */
        uint32_t mslut3;
    } mslut3_reg;

    struct
    {
        uint8_t addr; /* 0x64 - Write */
        uint32_t mslut4;
    } mslut4_reg;

    struct
    {
        uint8_t addr; /* 0x65 - Write */
        uint32_t mslut5;
    } mslut5_reg;

    struct
    {
        uint8_t addr; /* 0x66 - Write */
        uint32_t mslut6;
    } mslut6_reg;

    struct
    {
        uint8_t addr; /* 0x67 - Write */
        uint32_t mslut7;
    } mslut7_reg;

    struct
    {
        uint8_t addr; /* 0x68 - Write */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint32_t width_sel_0 : 2;
				uint32_t width_sel_1 : 2;
				uint32_t width_sel_2 : 2;
				uint32_t width_sel_3 : 2;
				uint32_t lut_seg_start1 : 8;
				uint32_t lut_seg_start2 : 8;
				uint32_t lut_seg_start3 : 8;
#else
                uint32_t lut_seg_start3 :8;
                uint32_t lut_seg_start2 :8;
                uint32_t lut_seg_start1 :8;
                uint32_t width_sel_3 :2;
                uint32_t width_sel_2 :2;
                uint32_t width_sel_1 :2;
                uint32_t width_sel_0 :2;
#endif
            };
            uint32_t reserved;
        } mslutsel_pins;
    } mslutsel_reg;

    struct
    {
        uint8_t addr; /* 0x69 - Write */
        uint16_t mslutstart;
    } mslutstart_reg;

    struct
    {
        uint8_t addr; /* 0x6A - Read  */
        uint16_t mscnt;
    } mscnt_reg;

    struct
    {
        uint8_t addr; /* 0x6B - Read  */
        union
        {
            struct
            {
#if BIG_ENDIAN
				int32_t cur_b : 9;
				int32_t _bit9 : 1;
				int32_t _bit10 : 1;
				int32_t _bit11 : 1;
				int32_t _bit12 : 1;
				int32_t _bit13 : 1;
				int32_t _bit14 : 1;
				int32_t _bit15 : 1;
				int32_t cur_a : 9;
				int32_t _bit25 : 1;
				int32_t _bit26 : 1;
				int32_t _bit27 : 1;
				int32_t _bit28 : 1;
				int32_t _bit29 : 1;
				int32_t _bit30 : 1;
				int32_t _bit31 : 1;
#else
                int32_t _bit31 :1;
                int32_t _bit30 :1;
                int32_t _bit29 :1;
                int32_t _bit28 :1;
                int32_t _bit27 :1;
                int32_t _bit26 :1;
                int32_t _bit25 :1;
                int32_t cur_a :9;
                int32_t _bit15 :1;
                int32_t _bit14 :1;
                int32_t _bit13 :1;
                int32_t _bit12 :1;
                int32_t _bit11 :1;
                int32_t _bit10 :1;
                int32_t _bit9 :1;
                int32_t cur_b :9;
#endif
            };
            int32_t reserved;
        } mscuract_pins;
    } mscuract_reg;
} mstep_control_regs_struct_t;

/**
 * \brief                       Driver Control Registers
 */
typedef struct drv_regs_struct
{
    struct
    {
        uint8_t addr; /* 0x6C - RW */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint32_t toff : 4;
				uint32_t hstrt : 3;
				uint32_t hend : 4;
				uint32_t fd3 : 1;
				uint32_t disfdcc : 1;
				uint32_t _bit13 : 1;
				uint32_t chm : 1;
				uint32_t tbl : 2;
				uint32_t _bit17 : 1;
				uint32_t vhighfs : 1;
				uint32_t vhighchm : 1;
				uint32_t tpfd : 4;
				uint32_t mres : 4;
				uint32_t intpol : 1;
				uint32_t dedge : 1;
				uint32_t diss2g : 1;
				uint32_t diss2vs : 1;
#else
                uint32_t diss2vs :1;
                uint32_t diss2g :1;
                uint32_t dedge :1;
                uint32_t intpol :1;
                uint32_t mres :4;
                uint32_t tpfd :4;
                uint32_t vhighchm :1;
                uint32_t vhighfs :1;
                uint32_t _bit17 :1;
                uint32_t tbl :2;
                uint32_t chm :1;
                uint32_t _bit13 :1;
                uint32_t disfdcc :1;
                uint32_t fd3 :1;
                uint32_t hend :4;
                uint32_t hstrt :3;
                uint32_t toff :4;
#endif
            };
            uint32_t reserved;
        } chop_conf_pins;
    } chop_conf_reg;

    struct
    {
        uint8_t addr; /* 0x6D - W */
        union
        {
            struct
            {
#if BIG_ENDIAN
				int32_t semin : 4;
				int32_t _bit4 : 1;
				int32_t seup : 2;
				int32_t _bit7 : 1;
				int32_t semax : 4;
				int32_t _bit12 : 1;
				int32_t sedn : 2;
				int32_t seimin : 1;
				int32_t sgt : 7;
				int32_t _bit23 : 1;
				int32_t sfilt : 1;
				int32_t _bit25 : 1;
				int32_t _bit26 : 1;
				int32_t _bit27 : 1;
				int32_t _bit28 : 1;
				int32_t _bit29 : 1;
				int32_t _bit30 : 1;
				int32_t _bit31 : 1;
#else
                int32_t _bit31 :1;
                int32_t _bit30 :1;
                int32_t _bit29 :1;
                int32_t _bit28 :1;
                int32_t _bit27 :1;
                int32_t _bit26 :1;
                int32_t _bit25 :1;
                int32_t sfilt :1;
                int32_t _bit23 :1;
                int32_t sgt :7;
                int32_t seimin :1;
                int32_t sedn :2;
                int32_t _bit12 :1;
                int32_t semax :4;
                int32_t _bit7 :1;
                int32_t seup :2;
                int32_t _bit4 :1;
                int32_t semin :4;
#endif
            };
            int32_t reserved;
        } cool_conf_pins;
    } cool_conf_reg;

    struct
    {
        uint8_t addr; /* 0x6E - W */
        uint32_t dc_ctrl;
    } dc_ctrl_reg;

    struct
    {
        uint8_t addr; /* 0x6F - R */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint32_t sg_result : 10;
				uint32_t _bit10 : 1;
				uint32_t _bit11 : 1;
				uint32_t s2vsa : 1;
				uint32_t s2vsb : 1;
				uint32_t stealth : 1;
				uint32_t fsactive : 1;
				uint32_t cs_actual : 5;
				uint32_t _bit21 : 1;
				uint32_t _bit22 : 1;
				uint32_t _bit23 : 1;
				uint32_t sg : 1;
				uint32_t ot : 1;
				uint32_t otpw : 1;
				uint32_t s2ga : 1;
				uint32_t s2gb : 1;
				uint32_t ola : 1;
				uint32_t olb : 1;
				uint32_t stst : 1;
#else
                uint32_t stst :1;
                uint32_t olb :1;
                uint32_t ola :1;
                uint32_t s2gb :1;
                uint32_t s2ga :1;
                uint32_t otpw :1;
                uint32_t ot :1;
                uint32_t sg :1;
                uint32_t _bit23 :1;
                uint32_t _bit22 :1;
                uint32_t _bit21 :1;
                uint32_t cs_actual :5;
                uint32_t fsactive :1;
                uint32_t stealth :1;
                uint32_t s2vsb :1;
                uint32_t s2vsa :1;
                uint32_t _bit11 :1;
                uint32_t _bit10 :1;
                uint32_t sg_result :10;
#endif
            };
            uint32_t reserved;
        } drv_status_pins;
    } drv_status_reg; /* DRV_STATUS - STALLGUARD2 Value And Driver Error Flags - Read */

    struct
    {
        uint8_t addr; /* 0x70 - W */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint32_t pwm_ofs : 8;
				uint32_t pwm_grad : 8;
				uint32_t pwm_freq : 2;
				uint32_t pwm_autoscale : 1;
				uint32_t pwm_autograd : 1;
				uint32_t freewheel : 2;
				uint32_t _bit22 : 1;
				uint32_t _bit23 : 1;
				uint32_t pwm_regulation : 4;
				uint32_t pwm_lim : 4;
#else
                uint32_t pwm_lim :4;
                uint32_t pwm_regulation :4;
                uint32_t _bit23 :1;
                uint32_t _bit22 :1;
                uint32_t freewheel :2;
                uint32_t pwm_autograd :1;
                uint32_t pwm_autoscale :1;
                uint32_t pwm_freq :2;
                uint32_t pwm_grad :8;
                uint32_t pwm_ofs :8;
#endif
            };
            uint32_t reserved;
        } pwm_conf_pins;
    } pwm_conf_reg; /* Voltage PWM Mode StealthChop */

    struct
    {
        uint8_t addr; /* 0x71 - R */
        union
        {
            struct
            {
#if BIG_ENDIAN
				int32_t pwm_scale_sum : 8;    /* 8 bitlik unsigned olacak tur donusum uygula */
				int32_t _bit8 : 1;
				int32_t _bit9 : 1;
				int32_t _bit10 : 1;
				int32_t _bit11 : 1;
				int32_t _bit12 : 1;
				int32_t _bit13 : 1;
				int32_t _bit14 : 1;
				int32_t _bit15 : 1;
				int32_t pwm_scale_auto : 9;
				int32_t _bit25 : 1;
				int32_t _bit26 : 1;
				int32_t _bit27 : 1;
				int32_t _bit28 : 1;
				int32_t _bit29 : 1;
				int32_t _bit30 : 1;
				int32_t _bit31 : 1;
#else
                int32_t _bit31 :1;
                int32_t _bit30 :1;
                int32_t _bit29 :1;
                int32_t _bit28 :1;
                int32_t _bit27 :1;
                int32_t _bit26 :1;
                int32_t _bit25 :1;
                int32_t pwm_scale_auto :9;
                int32_t _bit15 :1;
                int32_t _bit14 :1;
                int32_t _bit13 :1;
                int32_t _bit12 :1;
                int32_t _bit11 :1;
                int32_t _bit10 :1;
                int32_t _bit9 :1;
                int32_t _bit8 :1;
                int32_t pwm_scale_sum :8; /* 8 bitlik unsigned olacak tur donusum uygula */
#endif
            };
            int32_t reserved;
        } pwm_scale_value;
    } pwm_scale_reg;

    struct
    {
        uint8_t addr; /* 0x72 - R */
        union
        {
            struct
            {
#if BIG_ENDIAN
				uint32_t pwm_ofs_auto : 8;
				uint32_t _bit8 : 1;
				uint32_t _bit9 : 1;
				uint32_t _bit10 : 1;
				uint32_t _bit11 : 1;
				uint32_t _bit12 : 1;
				uint32_t _bit13 : 1;
				uint32_t _bit14 : 1;
				uint32_t _bit15 : 1;
				uint32_t pwm_grad_auto : 8;
				uint32_t _bit24 : 1;
				uint32_t _bit25 : 1;
				uint32_t _bit26 : 1;
				uint32_t _bit27 : 1;
				uint32_t _bit28 : 1;
				uint32_t _bit29 : 1;
				uint32_t _bit30 : 1;
				uint32_t _bit31 : 1;
#else
                uint32_t _bit31 :1;
                uint32_t _bit30 :1;
                uint32_t _bit29 :1;
                uint32_t _bit28 :1;
                uint32_t _bit27 :1;
                uint32_t _bit26 :1;
                uint32_t _bit25 :1;
                uint32_t _bit24 :1;
                uint32_t pwm_grad_auto :8;
                uint32_t _bit15 :1;
                uint32_t _bit14 :1;
                uint32_t _bit13 :1;
                uint32_t _bit12 :1;
                uint32_t _bit11 :1;
                uint32_t _bit10 :1;
                uint32_t _bit9 :1;
                uint32_t _bit8 :1;
                uint32_t pwm_ofs_auto :8;
#endif
            };
            uint32_t reserved;
        } pwm_auto_value;
    } pwm_auto_reg;

    struct
    {
        uint8_t addr; /* 0x73 - R */
        uint32_t lost_steps;
    } lost_steps_reg;
} drv_regs_struct_t;

/**
 * \brief                       This structure contains microstep and driver control registers.
 */
typedef struct tmc5160_motor_drv_regs_struct
{
    mstep_control_regs_struct_t mstep_control;
    drv_regs_struct_t drv_control;
} tmc5160_motor_drv_regs_struct_t;

/**
 * \brief                       Extern function that need to be wrapped with SPI.
 * \param[in] data              Send the data you want to write over SPI.
 * \param[in] length            Length of data packet.
 * \param[in] timeout           Timeout parameter.
 * \retval                      The success or error value of the function.
 */
typedef results_enum_t (*tmc5160_spi_write_fn)(tmc5160_channels_enum_t channel,
                                               uint8_t *data, uint16_t data_len,
                                               uint32_t timeout);

/**
 * \brief                       Extern function that need to be wrapped with SPI.
 * \param[in] data              Send an empty data string over SPI and receive back over data.
 * \param[in] length            Length of data packet.
 * \param[in] timeout           Timeout parameter.
 * \retval                      The success or error value of the function.
 */
typedef results_enum_t (*tmc5160_spi_read_fn)(tmc5160_channels_enum_t channel,
                                              uint8_t *data, uint16_t data_len,
                                              uint32_t timeout);

/**
 * \brief                       This structure; it contains registers, related flags, and global variables related to tmc5160.
 */
typedef struct tmc5160_module_struct
{
    tmc5160_gconf_regs_struct_t gconf_regs;
    tmc5160_vel_regs_struct_t vel_regs;
    tmc5160_ramp_gen_regs_struct_t ramp_gen_regs;
    tmc5160_enc_regs_struct enc_regs;
    tmc5160_motor_drv_regs_struct_t motor_drv_regs;
    tmc5160_config_state_enum_t state;
    uint8_t config_index;
    tmc5160_spi_write_fn spi_write_wrapper;
    tmc5160_spi_read_fn spi_read_wrapper;
    uint8_t reg_access[TMC5160_REG_COUNT];
} tmc5160_module_struct_t;

/*--------------------------------------------------------------------------*/
/* Function declaration                                                     */
/*--------------------------------------------------------------------------*/
results_enum_t tmc5160_init(tmc5160_comm_mode_enum_t comm_mode,
                            tmc5160_channels_enum_t channel,
                            tmc5160_spi_write_fn tmc5160_spi_write_wrapper,
                            tmc5160_spi_read_fn tmc5160_spi_read_wrapper);
bool tmc5160_driver_state(tmc5160_channels_enum_t channel,
                          tmc5160_driver_state_enum_t driver_state);
results_enum_t tmc5160_reset(tmc5160_channels_enum_t channel);
results_enum_t tmc5160_restore(tmc5160_channels_enum_t channel);
results_enum_t tmc5160_timer_callback(tmc5160_channels_enum_t channel);
results_enum_t tmc5160_loop(tmc5160_channels_enum_t channel);
results_enum_t tmc5160_emergency_stop_callback(tmc5160_channels_enum_t channel,
                                               bool enable);
void tmc5160_factory_default_settings(tmc5160_channels_enum_t channel);
results_enum_t tmc5160_set_move_to(tmc5160_channels_enum_t channel,
                                   int32_t position);
results_enum_t tmc5160_set_move_by(tmc5160_channels_enum_t channel,
                                   int32_t *ticks);

/*--------------------------------------------------------------------------*/
/* Setter functions                                                         */
/*--------------------------------------------------------------------------*/
results_enum_t tmc5160_spi_write_reg(tmc5160_channels_enum_t channel,
                                     uint8_t addr, int32_t value,
                                     uint32_t timeout);
results_enum_t tmc5160_set_otp_prog(tmc5160_channels_enum_t channel,
                                    uint16_t otp_prog);
results_enum_t tmc5160_set_otp_prog_bit(tmc5160_channels_enum_t channel,
                                        uint8_t otp_bit);
results_enum_t tmc5160_set_otp_prog_byte(tmc5160_channels_enum_t channel,
                                         uint8_t otp_byte);
results_enum_t tmc5160_set_otp_prog_magic(tmc5160_channels_enum_t channel,
                                          uint8_t otp_magic);
results_enum_t tmc5160_set_ramp_xtarget(tmc5160_channels_enum_t channel,
                                        int32_t value);
results_enum_t tmc5160_set_ramp_xactual(tmc5160_channels_enum_t channel,
                                        int32_t value);
results_enum_t tmc5160_set_ramp_vmax(tmc5160_channels_enum_t channel,
                                     uint32_t vmax);
results_enum_t tmc5160_set_ramp_amax(tmc5160_channels_enum_t channel,
                                     uint16_t amax);
results_enum_t tmc5160_set_ramp_a1(tmc5160_channels_enum_t channel,
                                   uint16_t a1);
results_enum_t tmc5160_set_ramp_v1(tmc5160_channels_enum_t channel,
                                   uint32_t v1);
results_enum_t tmc5160_set_ramp_dmax(tmc5160_channels_enum_t channel,
                                     uint16_t dmax);
results_enum_t tmc5160_set_ramp_d1(tmc5160_channels_enum_t channel,
                                   uint16_t d1);
results_enum_t tmc5160_set_ramp_vstart(tmc5160_channels_enum_t channel,
                                       uint32_t vstart);
results_enum_t tmc5160_set_ramp_vstop(tmc5160_channels_enum_t channel,
                                      uint32_t vstop);
results_enum_t tmc5160_set_ramp_type(tmc5160_channels_enum_t channel,
                                     tmc5160_ramp_type_enum_t ramp_mode);
results_enum_t tmc5160_set_sw_mode(tmc5160_channels_enum_t channel,
                                   uint16_t sw_mode);
results_enum_t tmc5160_set_sw_mode_left_stop(tmc5160_channels_enum_t channel,
                                             bool soft_en_left);
results_enum_t tmc5160_set_sw_mode_right_stop(tmc5160_channels_enum_t channel,
                                              bool soft_en_right);
results_enum_t tmc5160_set_sw_mode_pol_stop_left(
        tmc5160_channels_enum_t channel, bool pol_stop_left);
results_enum_t tmc5160_set_sw_mode_pol_stop_right(
        tmc5160_channels_enum_t channel, bool pol_stop_right);
results_enum_t tmc5160_set_sw_mode_swap_left_right(
        tmc5160_channels_enum_t channel, bool swap_left_right);
results_enum_t tmc5160_set_sw_mode_latch_active_left(
        tmc5160_channels_enum_t channel, bool latch_active_left);
results_enum_t tmc5160_set_sw_mode_latch_inactive_left(
        tmc5160_channels_enum_t channel, bool latch_inactive_left);
results_enum_t tmc5160_set_sw_mode_latch_active_right(
        tmc5160_channels_enum_t channel, bool latch_active_right);
results_enum_t tmc5160_set_sw_mode_latch_inactive_right(
        tmc5160_channels_enum_t channel, bool latch_inactive_right);
results_enum_t tmc5160_set_sw_mode_en_latch_enc(tmc5160_channels_enum_t channel,
                                                bool en_latch_enc);
results_enum_t tmc5160_set_sw_mode_sg_stop(tmc5160_channels_enum_t channel,
                                           bool sg_stop);
results_enum_t tmc5160_set_sw_mode_en_softstop(tmc5160_channels_enum_t channel,
                                               bool en_softstop);
results_enum_t tmc5160_set_wait_time_ramp_down(tmc5160_channels_enum_t channel,
                                               uint16_t tzero_wait);
results_enum_t tmc5160_set_se_hspeed_thresh(tmc5160_channels_enum_t channel,
                                            int32_t value);
results_enum_t tmc5160_set_min_speed_dcstep(tmc5160_channels_enum_t channel,
                                            int32_t value);
results_enum_t tmc5160_set_gconf_recalibrate(tmc5160_channels_enum_t channel,
                                             bool recalibrate);
results_enum_t tmc5160_set_gconf_pwm_mode(tmc5160_channels_enum_t channel,
                                          bool en_pwm_mode);
results_enum_t tmc5160_set_gconf_multistep_filt(tmc5160_channels_enum_t channel,
                                                bool multistep_filt);
results_enum_t tmc5160_set_shortconf_s2vs_level(tmc5160_channels_enum_t channel,
                                                uint8_t s2vs_level);
results_enum_t tmc5160_set_shortconf_s2g_level(tmc5160_channels_enum_t channel,
                                               uint8_t s2g_level);
results_enum_t tmc5160_set_shortconf_short_filter(
        tmc5160_channels_enum_t channel, uint8_t short_filter);
results_enum_t tmc5160_set_shortconf_short_delay(
        tmc5160_channels_enum_t channel, bool short_delay);
results_enum_t tmc5160_set_drv_bbmtime(tmc5160_channels_enum_t channel,
                                       uint8_t bbmtime);
results_enum_t tmc5160_set_drv_bbmclks(tmc5160_channels_enum_t channel,
                                       uint8_t bbmclks);
results_enum_t tmc5160_set_drv_otselect(tmc5160_channels_enum_t channel,
                                        uint8_t otselect);
results_enum_t tmc5160_set_drv_strength(tmc5160_channels_enum_t channel,
                                        uint8_t drvstrength);
results_enum_t tmc5160_set_drv_filt_isense(tmc5160_channels_enum_t channel,
                                           uint8_t filt_isense);
results_enum_t tmc5160_set_tpower_down(tmc5160_channels_enum_t channel,
                                       uint8_t tpower_down);
results_enum_t tmc5160_set_chopper_config(tmc5160_channels_enum_t channel,
                                          uint32_t value);
results_enum_t tmc5160_set_chopper_toff(tmc5160_channels_enum_t channel,
                                        uint8_t toff);
results_enum_t tmc5160_set_chopper_hstrt_sine_offset(
        tmc5160_channels_enum_t channel, uint8_t hstrt_sine_offset);
results_enum_t tmc5160_set_chopper_hend_fd_time(tmc5160_channels_enum_t channel,
                                                uint8_t hend_fd_time);
results_enum_t tmc5160_set_chopper_fast_decay(tmc5160_channels_enum_t channel,
                                              bool disfdcc);
results_enum_t tmc5160_set_chopper_mode(tmc5160_channels_enum_t channel,
                                        bool chm);
results_enum_t tmc5160_set_chopper_blank_time(tmc5160_channels_enum_t channel,
                                              uint8_t tbl);
results_enum_t tmc5160_set_chopper_high_vel_fs(tmc5160_channels_enum_t channel,
                                               bool vhighfs);
results_enum_t tmc5160_set_chopper_high_vel(tmc5160_channels_enum_t channel,
                                            bool vhighchm);
results_enum_t tmc5160_set_chopper_passive_fd_time(
        tmc5160_channels_enum_t channel, uint8_t tpfd);
results_enum_t tmc5160_set_chopper_microstep_res(
        tmc5160_channels_enum_t channel, microstep_res_enum_t microstep);
results_enum_t tmc5160_set_chopper_double_edge(tmc5160_channels_enum_t channel,
                                               bool dedge);
results_enum_t tmc5160_set_chopper_gnd_protection(
        tmc5160_channels_enum_t channel, bool diss2g);
results_enum_t tmc5160_set_chopper_supply_protection(
        tmc5160_channels_enum_t channel, bool diss2vs);
results_enum_t tmc5160_set_cool_se_current_min(tmc5160_channels_enum_t channel,
                                               bool seimin);
results_enum_t tmc5160_set_cool_se_current_down_step(
        tmc5160_channels_enum_t channel, uint8_t sedn);
results_enum_t tmc5160_set_cool_se_hysteresis(tmc5160_channels_enum_t channel,
                                              uint8_t semax);
results_enum_t tmc5160_set_cool_se_current_up_step(
        tmc5160_channels_enum_t channel, uint8_t seup);
results_enum_t tmc5160_set_cool_se_hysteresis_start(
        tmc5160_channels_enum_t channel, uint8_t semin);
results_enum_t tmc5160_set_cool_sg_filter(tmc5160_channels_enum_t channel,
                                          bool sfilt);
results_enum_t tmc5160_set_cool_sg_thresh(tmc5160_channels_enum_t channel,
                                          int8_t sgt);
results_enum_t tmc5160_set_se_velocity_stall(tmc5160_channels_enum_t channel,
                                             bool sg_stop);
results_enum_t tmc5160_set_se_speed_thresh(tmc5160_channels_enum_t channel,
                                           int32_t value);
results_enum_t tmc5160_set_pwm_thresh_speed(tmc5160_channels_enum_t channel,
                                            int32_t value);
results_enum_t tmc5160_set_pwm_config(tmc5160_channels_enum_t channel,
                                      uint32_t value);
results_enum_t tmc5160_set_pwm_gradient(tmc5160_channels_enum_t channel,
                                        uint8_t pwm_grad);
results_enum_t tmc5160_set_pwm_ofs_amplitude(tmc5160_channels_enum_t channel,
                                             uint8_t pwm_ofs);
results_enum_t tmc5160_set_pwm_frequency(tmc5160_channels_enum_t channel,
                                         uint8_t pwm_freq);
results_enum_t tmc5160_set_pwm_autoscale(tmc5160_channels_enum_t channel,
                                         bool pwm_autoscale);
results_enum_t tmc5160_set_pwm_autograd(tmc5160_channels_enum_t channel,
                                        bool pwm_autograd);
results_enum_t tmc5160_set_pwm_freewheeling_mode(
        tmc5160_channels_enum_t channel, uint8_t freewheel);
results_enum_t tmc5160_set_pwm_regulation(tmc5160_channels_enum_t channel,
                                          uint8_t pwm_regulation);
results_enum_t tmc5160_set_pwm_limit(tmc5160_channels_enum_t channel,
                                     uint8_t pwm_lim);
results_enum_t tmc5160_set_enc_pos(tmc5160_channels_enum_t channel,
                                   int32_t xenc);
results_enum_t tmc5160_set_enc_res(tmc5160_channels_enum_t channel,
                                   int32_t enc_const);
results_enum_t tmc5160_set_global_scaler(tmc5160_channels_enum_t channel,
                                         uint8_t global_scaler);
results_enum_t tmc5160_set_ihold(tmc5160_channels_enum_t channel,
                                 uint8_t ihold);
results_enum_t tmc5160_set_irun(tmc5160_channels_enum_t channel, uint8_t irun);
results_enum_t tmc5160_set_ihold_delay(tmc5160_channels_enum_t channel,
                                       uint8_t ihold_delay);

/*--------------------------------------------------------------------------*/
/* Getter functions                                                         */
/*--------------------------------------------------------------------------*/
int32_t tmc5160_spi_read_reg(tmc5160_channels_enum_t channel, uint8_t addr,
                             uint32_t timeout);
uint8_t tmc5160_get_otp_read(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_otp_read_fclktrim(tmc5160_channels_enum_t channel);
bool tmc5160_get_otp_read_s2_level(tmc5160_channels_enum_t channel);
bool tmc5160_get_otp_read_bbm(tmc5160_channels_enum_t channel);
bool tmc5160_get_otp_read_tbl(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_ramp_xtarget(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_ramp_xactual(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_ramp_vactual(tmc5160_channels_enum_t channel);
uint32_t tmc5160_get_ramp_vmax(tmc5160_channels_enum_t channel);
uint16_t tmc5160_get_ramp_amax(tmc5160_channels_enum_t channel);
uint16_t tmc5160_get_ramp_a1(tmc5160_channels_enum_t channel);
uint32_t tmc5160_get_ramp_v1(tmc5160_channels_enum_t channel);
uint16_t tmc5160_get_ramp_dmax(tmc5160_channels_enum_t channel);
uint16_t tmc5160_get_ramp_d1(tmc5160_channels_enum_t channel);
uint32_t tmc5160_get_ramp_vstart(tmc5160_channels_enum_t channel);
uint32_t tmc5160_get_ramp_vstop(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_ramp_type(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_rs_pos_flag(tmc5160_channels_enum_t channel);
bool tmc5160_get_rs_right_endstop(tmc5160_channels_enum_t channel);
bool tmc5160_get_rs_left_endstop(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_sw_mode(tmc5160_channels_enum_t channel);
bool tmc5160_get_sw_mode_right_stop(tmc5160_channels_enum_t channel);
bool tmc5160_get_sw_mode_left_stop(tmc5160_channels_enum_t channel);
uint16_t tmc5160_get_wait_time_ramp_down(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_se_hspeed_thresh(tmc5160_channels_enum_t channel);
uint32_t tmc5160_get_min_speed_dcstep(tmc5160_channels_enum_t channel);
bool tmc5160_get_gconf_recalibrate(tmc5160_channels_enum_t channel);
bool tmc5160_get_gconf_pwm_mode(tmc5160_channels_enum_t channel);
bool tmc5160_get_gconf_multistep_filt(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_shortconf_s2vs_level(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_shortconf_s2g_level(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_shortconf_short_filter(tmc5160_channels_enum_t channel);
bool tmc5160_get_shortconf_short_delay(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_drv_bbmtime(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_drv_bbmclks(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_drv_otselect(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_drv_strength(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_drv_filt_isense(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_tpower_down(tmc5160_channels_enum_t channel);
uint32_t tmc5160_get_chopper_config(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_chopper_toff(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_chopper_hstrt_sine_offset(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_chopper_hend_fd_time(tmc5160_channels_enum_t channel);
bool tmc5160_get_chopper_fast_decay(tmc5160_channels_enum_t channel);
bool tmc5160_get_chopper_mode(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_chopper_blank_time(tmc5160_channels_enum_t channel);
bool tmc5160_get_chopper_high_vel_fs(tmc5160_channels_enum_t channel);
bool tmc5160_get_chopper_high_vel(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_chopper_passive_fd_time(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_chopper_microstep_res(tmc5160_channels_enum_t channel);
bool tmc5160_get_chopper_double_edge(tmc5160_channels_enum_t channel);
bool tmc5160_get_chopper_gnd_protection(tmc5160_channels_enum_t channel);
bool tmc5160_get_chopper_supply_protection(tmc5160_channels_enum_t channel);
bool tmc5160_get_cool_se_current_min(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_cool_se_current_down_step(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_cool_se_hysteresis(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_cool_se_current_up_step(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_cool_se_hysteresis_start(tmc5160_channels_enum_t channel);
bool tmc5160_get_cool_sg_filter(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_cool_sg_thresh(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_se_velocity_stall(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_se_speed_thresh(tmc5160_channels_enum_t channel);
uint32_t tmc5160_get_drv_status(tmc5160_channels_enum_t channel);
uint16_t tmc5160_get_drv_status_sg_result(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_s2vsa(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_s2vsb(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_stealth(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_fsactive(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_drv_status_cs_actual(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_stallguard(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_ot(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_otpw(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_s2ga(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_s2gb(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_ola(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_olb(tmc5160_channels_enum_t channel);
bool tmc5160_get_drv_status_stst(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_pwm_thresh_speed(tmc5160_channels_enum_t channel);
uint32_t tmc5160_get_pwm_config(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_pwm_gradient(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_pwm_ofs_amplitude(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_pwm_frequency(tmc5160_channels_enum_t channel);
bool tmc5160_get_pwm_autoscale(tmc5160_channels_enum_t channel);
bool tmc5160_get_pwm_autograd(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_pwm_freewheeling_mode(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_pwm_regulation(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_pwm_limit(tmc5160_channels_enum_t channel);
uint32_t tmc5160_get_enc_pos(tmc5160_channels_enum_t channel);
int32_t tmc5160_get_enc_res(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_global_scaler(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_ihold(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_irun(tmc5160_channels_enum_t channel);
uint8_t tmc5160_get_ihold_delay(tmc5160_channels_enum_t channel);

#endif /* _TMC5160_H_ */
