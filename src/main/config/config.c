/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"

#include "blackbox/blackbox_io.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/ledstrip.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "config/config_profile.h"
#include "config/config_master.h"

#define BRUSHED_MOTORS_PWM_RATE 16000
#ifdef STM32F4
#define BRUSHLESS_MOTORS_PWM_RATE 2000
#else
#define BRUSHLESS_MOTORS_PWM_RATE 400
#endif

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, escAndServoConfig_t *escAndServoConfigToUse, pidProfile_t *pidProfileToUse);

#if !defined(FLASH_SIZE)
#error "Flash size not defined for target. (specify in KB)"
#endif


#ifndef FLASH_PAGE_SIZE
    #ifdef STM32F303xC
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #ifdef STM32F10X_MD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #if defined(STM32F40_41xxx)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
    #endif

    #if defined (STM32F411xE)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
    #endif

#endif

#if !defined(FLASH_SIZE) && !defined(FLASH_PAGE_COUNT)
    #ifdef STM32F10X_MD
        #define FLASH_PAGE_COUNT 128
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_COUNT 128
    #endif
#endif

#if defined(FLASH_SIZE)
#if defined(STM32F40_41xxx)
    #define FLASH_PAGE_COUNT 4 // just to make calculations work
#elif defined (STM32F411xE)
    #define FLASH_PAGE_COUNT 4 // just to make calculations work
#else
    #define FLASH_PAGE_COUNT ((FLASH_SIZE * 0x400) / FLASH_PAGE_SIZE)
#endif
#endif

#if !defined(FLASH_PAGE_SIZE)
#error "Flash page size not defined for target."
#endif

#if !defined(FLASH_PAGE_COUNT)
#error "Flash page count not defined for target."
#endif

#if FLASH_SIZE <= 128
    #define FLASH_TO_RESERVE_FOR_CONFIG 0x800
#else
    #define FLASH_TO_RESERVE_FOR_CONFIG 0x1000
#endif

#if !defined(CONFIG_START_FLASH_ADDRESS)
    // use the last flash pages for storage
    #define CONFIG_START_FLASH_ADDRESS (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))
#endif

master_t masterConfig;                 // master config struct with data independent from profiles
profile_t *currentProfile;
static uint32_t activeFeaturesLatch = 0;

static uint8_t currentControlRateProfileIndex = 0;
controlRateConfig_t *currentControlRateProfile;

static const uint8_t EEPROM_CONF_VERSION = 124;

static void resetAccelerometerTrims(flightDynamicsTrims_t *accelerometerTrims)
{
    accelerometerTrims->values.pitch = 0;
    accelerometerTrims->values.roll = 0;
    accelerometerTrims->values.yaw = 0;
}

void resetPidProfile(pidProfile_t *pidProfile)
{

#if defined(STM32F411xE) || defined(STM32F40_41xxx)
    pidProfile->pidController = 2; //default is LUX
#else
    pidProfile->pidController = 1; //default is MWRW
#endif

    pidProfile->P8[ROLL] = 55;
    pidProfile->I8[ROLL] = 80;
    pidProfile->D8[ROLL] = 18;
    pidProfile->P8[PITCH] = 65;
    pidProfile->I8[PITCH] = 100;
    pidProfile->D8[PITCH] = 22;
    pidProfile->P8[YAW] = 110;
    pidProfile->I8[YAW] = 180;
    pidProfile->D8[YAW] = 0;
    pidProfile->P8[PIDALT] = 50;
    pidProfile->I8[PIDALT] = 0;
    pidProfile->D8[PIDALT] = 0;
    pidProfile->P8[PIDPOS] = 15; // POSHOLD_P * 100;
    pidProfile->I8[PIDPOS] = 0; // POSHOLD_I * 100;
    pidProfile->D8[PIDPOS] = 0;
    pidProfile->P8[PIDPOSR] = 34; // POSHOLD_RATE_P * 10;
    pidProfile->I8[PIDPOSR] = 14; // POSHOLD_RATE_I * 100;
    pidProfile->D8[PIDPOSR] = 53; // POSHOLD_RATE_D * 1000;
    pidProfile->P8[PIDNAVR] = 25; // NAV_P * 10;
    pidProfile->I8[PIDNAVR] = 33; // NAV_I * 100;
    pidProfile->D8[PIDNAVR] = 83; // NAV_D * 1000;
    pidProfile->P8[PIDLEVEL] = 55;
    pidProfile->I8[PIDLEVEL] = 55;
    pidProfile->D8[PIDLEVEL] = 100;
    pidProfile->P8[PIDMAG] = 40;
    pidProfile->P8[PIDVEL] = 120;
    pidProfile->I8[PIDVEL] = 45;
    pidProfile->D8[PIDVEL] = 1;

    pidProfile->gyro_lpf_hz = 70;    // filtering ON by default
#ifdef STM32F4
    pidProfile->dterm_lpf_hz = 70;   // filtering ON by default
    pidProfile->yaw_pterm_cut_hz = 30;
    pidProfile->P_f[ROLL] = 5.000f;     // new PID for raceflight. test carefully
    pidProfile->I_f[ROLL] = 1.000f;
    pidProfile->D_f[ROLL] = 0.110f;
    pidProfile->P_f[PITCH] = 6.500f;
    pidProfile->I_f[PITCH] = 1.500f;
    pidProfile->D_f[PITCH] = 0.140f;
    pidProfile->P_f[YAW] = 9.300f;
    pidProfile->I_f[YAW] = 1.750f;
    pidProfile->D_f[YAW] = 0.000f;
    pidProfile->A_level = 3.000f;
    pidProfile->H_level = 3.000f;
    pidProfile->H_sensitivity = 100;
#else
    pidProfile->dterm_lpf_hz = 40;   // filtering ON by default
    pidProfile->yaw_pterm_cut_hz = 30;
    pidProfile->P_f[ROLL] = 1.1f;     // new PID with preliminary defaults test carefully
    pidProfile->I_f[ROLL] = 0.4f;
    pidProfile->D_f[ROLL] = 0.01f;
    pidProfile->P_f[PITCH] = 1.5f;
    pidProfile->I_f[PITCH] = 0.4f;
    pidProfile->D_f[PITCH] = 0.01f;
    pidProfile->P_f[YAW] = 4.0f;
    pidProfile->I_f[YAW] = 0.4f;
    pidProfile->D_f[YAW] = 0.00f;
    pidProfile->A_level = 3.000f;
    pidProfile->H_level = 3.000f;
    pidProfile->H_sensitivity = 100;
#endif

#ifdef GTUNE
    pidProfile->gtune_lolimP[ROLL] = 10;          // [0..200] Lower limit of ROLL P during G tune.
    pidProfile->gtune_lolimP[PITCH] = 10;         // [0..200] Lower limit of PITCH P during G tune.
    pidProfile->gtune_lolimP[YAW] = 10;           // [0..200] Lower limit of YAW P during G tune.
    pidProfile->gtune_hilimP[ROLL] = 100;         // [0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_hilimP[PITCH] = 100;        // [0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_hilimP[YAW] = 100;          // [0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
    pidProfile->gtune_pwr = 0;                    // [0..10] Strength of adjustment
    pidProfile->gtune_settle_time = 450;          // [200..1000] Settle time in ms
    pidProfile->gtune_average_cycles = 16;        // [8..128] Number of looptime cycles used for gyro average calculation
#endif
}

#ifdef GPS
void resetGpsProfile(gpsProfile_t *gpsProfile)
{
    gpsProfile->gps_wp_radius = 200;
    gpsProfile->gps_lpf = 20;
    gpsProfile->nav_slew_rate = 30;
    gpsProfile->nav_controls_heading = 1;
    gpsProfile->nav_speed_min = 100;
    gpsProfile->nav_speed_max = 300;
    gpsProfile->ap_mode = 40;
}
#endif

#ifdef BARO
void resetBarometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->baro_sample_count = 21;
    barometerConfig->baro_noise_lpf = 0.6f;
    barometerConfig->baro_cf_vel = 0.985f;
    barometerConfig->baro_cf_alt = 0.965f;
}
#endif

void resetSensorAlignment(sensorAlignmentConfig_t *sensorAlignmentConfig)
{
    sensorAlignmentConfig->gyro_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->acc_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->mag_align = ALIGN_DEFAULT;
}

void resetEscAndServoConfig(escAndServoConfig_t *escAndServoConfig)
{
    escAndServoConfig->minthrottle = 1150;
    escAndServoConfig->maxthrottle = 1850;
    escAndServoConfig->mincommand = 1000;
    escAndServoConfig->servoCenterPulse = 1500;
}

void resetFlight3DConfig(flight3DConfig_t *flight3DConfig)
{
    flight3DConfig->deadband3d_low = 1406;
    flight3DConfig->deadband3d_high = 1514;
    flight3DConfig->neutral3d = 1460;
    flight3DConfig->deadband3d_throttle = 50;
}

#ifdef TELEMETRY
void resetTelemetryConfig(telemetryConfig_t *telemetryConfig)
{
    telemetryConfig->telemetry_inversion = 0;
    telemetryConfig->telemetry_switch = 0;
    telemetryConfig->gpsNoFixLatitude = 0;
    telemetryConfig->gpsNoFixLongitude = 0;
    telemetryConfig->frsky_coordinate_format = FRSKY_FORMAT_DMS;
    telemetryConfig->frsky_unit = FRSKY_UNIT_METRICS;
    telemetryConfig->frsky_vfas_precision = 0;
    telemetryConfig->hottAlarmSoundInterval = 5;
}
#endif

void resetBatteryConfig(batteryConfig_t *batteryConfig)
{
    batteryConfig->vbatscale = VBAT_SCALE_DEFAULT;
    batteryConfig->vbatresdivval = VBAT_RESDIVVAL_DEFAULT;
    batteryConfig->vbatresdivmultiplier = VBAT_RESDIVMULTIPLIER_DEFAULT;
    batteryConfig->vbatmaxcellvoltage = 43;
    batteryConfig->vbatmincellvoltage = 33;
    batteryConfig->vbatwarningcellvoltage = 35;
    batteryConfig->currentMeterOffset = 0;
    batteryConfig->currentMeterScale = 400; // for Allegro ACS758LCB-100U (40mV/A)
    batteryConfig->batteryCapacity = 0;
    batteryConfig->currentMeterType = CURRENT_SENSOR_ADC;
}

#ifdef SWAP_SERIAL_PORT_0_AND_1_DEFAULTS
#define FIRST_PORT_INDEX 1
#define SECOND_PORT_INDEX 0
#else
#define FIRST_PORT_INDEX 0
#define SECOND_PORT_INDEX 1
#endif

void resetSerialConfig(serialConfig_t *serialConfig)
{
    uint8_t index;
    memset(serialConfig, 0, sizeof(serialConfig_t));

    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialConfig->portConfigs[index].identifier = serialPortIdentifiers[index];
        serialConfig->portConfigs[index].msp_baudrateIndex = BAUD_115200;
        serialConfig->portConfigs[index].gps_baudrateIndex = BAUD_57600;
        serialConfig->portConfigs[index].telemetry_baudrateIndex = BAUD_AUTO;
        serialConfig->portConfigs[index].blackbox_baudrateIndex = BAUD_115200;
    }

    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;

#if defined(USE_VCP)
    // This allows MSP connection via USART & VCP so the board can be reconfigured.
    serialConfig->portConfigs[1].functionMask = FUNCTION_MSP;
#endif

    serialConfig->reboot_character = 'R';
}

static void resetControlRateConfig(controlRateConfig_t *controlRateConfig) {
    controlRateConfig->rcRate8 = 100;
    controlRateConfig->rcExpo8 = 70;
    controlRateConfig->thrMid8 = 50;
    controlRateConfig->thrExpo8 = 0;
    controlRateConfig->dynThrPID = 0;
    controlRateConfig->rcYawExpo8 = 20;
    controlRateConfig->tpa_breakpoint = 1500;

    controlRateConfig->AcroPlusFactor = 20;

    for (uint8_t axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++) {
        if (axis == 2) {
            controlRateConfig->rates[axis] = 35;
        } else {
            controlRateConfig->rates[axis] = 20;
        }
    }

}

void resetRcControlsConfig(rcControlsConfig_t *rcControlsConfig) {
    rcControlsConfig->deadband = 0;
    rcControlsConfig->yaw_deadband = 0;
    rcControlsConfig->alt_hold_deadband = 40;
    rcControlsConfig->alt_hold_fast_change = 1;
}

void resetMixerConfig(mixerConfig_t *mixerConfig) {
    mixerConfig->yaw_motor_direction = 1;
    mixerConfig->yaw_jump_prevention_limit = 200;
#ifdef USE_SERVOS
    mixerConfig->tri_unarmed_servo = 1;
    mixerConfig->servo_lowpass_freq = 400;
    mixerConfig->servo_lowpass_enable = 0;
#endif
}

uint8_t getCurrentProfile(void)
{
    return masterConfig.current_profile_index;
}

static void setProfile(uint8_t profileIndex)
{
    currentProfile = &masterConfig.profile[profileIndex];
}

uint8_t getCurrentControlRateProfile(void)
{
    return currentControlRateProfileIndex;
}

controlRateConfig_t *getControlRateConfig(uint8_t profileIndex) {
    return &masterConfig.controlRateProfiles[profileIndex];
}

static void setControlRateProfile(uint8_t profileIndex)
{
    currentControlRateProfileIndex = profileIndex;
    currentControlRateProfile = &masterConfig.controlRateProfiles[profileIndex];
}

uint16_t getCurrentMinthrottle(void)
{
    return masterConfig.escAndServoConfig.minthrottle;
}

// Default settings
static void resetConf(void)
{
    int i;

    // Clear all configuration
    memset(&masterConfig, 0, sizeof(master_t));

    setProfile(0);
    setControlRateProfile(0);

    masterConfig.beeper_off.flags = BEEPER_OFF_FLAGS_MIN;
    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.mixerMode = MIXER_QUADX;
    featureClearAll();

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    // only enable the VBAT feature by default if the board has a voltage divider otherwise
    // the user may see incorrect readings and unexpected issues with pin mappings may occur.
    featureSet(FEATURE_VBAT);
#endif

    featureSet(FEATURE_FAILSAFE);
    featureSet(FEATURE_USE_PWM_RATE);
    featureSet(FEATURE_SBUS_INVERTER);

    // global settings
    masterConfig.current_profile_index = 0;     // default profile
    masterConfig.dcm_kp = 2500;                // 1.0 * 10000
    masterConfig.dcm_ki = 0;                    // 0.003 * 10000

    resetAccelerometerTrims(&masterConfig.accZero);

    resetSensorAlignment(&masterConfig.sensorAlignmentConfig);

    masterConfig.boardAlignment.rollDegrees = 0;
    masterConfig.boardAlignment.pitchDegrees = 0;
    masterConfig.boardAlignment.yawDegrees = 0;
    masterConfig.acc_hardware = ACC_DEFAULT;     // default/autodetect
    masterConfig.max_angle_inclination = 700;    // 70 degrees
    masterConfig.yaw_control_direction = 1;
    masterConfig.gyroConfig.gyroMovementCalibrationThreshold = 16;

    // xxx_hardware: 0:default/autodetect, 1: disable
    masterConfig.mag_hardware = 1;

    masterConfig.baro_hardware = 1;

    resetBatteryConfig(&masterConfig.batteryConfig);

#ifdef TELEMETRY
    resetTelemetryConfig(&masterConfig.telemetryConfig);
#endif

#ifdef CONFIG_SERIALRX_PROVIDER
    masterConfig.rxConfig.serialrx_provider = CONFIG_SERIALRX_PROVIDER;
#else
    masterConfig.rxConfig.serialrx_provider = SERIALRX_SPEKTRUM1024;
#endif
	masterConfig.rf_loop_ctrl = 2;                 // High DLPF, H4

#if defined(CC3D) || defined(MOTOLAB) || defined(LUX_RACE)
    masterConfig.acc_hardware = 1;     // default/autodetect
#endif
        
#ifdef STM32F4
    masterConfig.rxConfig.max_aux_channel = 99;
#elif defined(STM32F303xC)
    masterConfig.rxConfig.max_aux_channel = 6;
#else
    masterConfig.rxConfig.max_aux_channel = 4;
#endif
    
    masterConfig.rxConfig.spektrum_sat_bind = 0;
    masterConfig.rxConfig.midrc = 1500;
    masterConfig.rxConfig.mincheck = 1100;
    masterConfig.rxConfig.maxcheck = 1900;
    masterConfig.rxConfig.rx_min_usec = 885;          // any of first 4 channels below this value will trigger rx loss detection
    masterConfig.rxConfig.rx_max_usec = 2115;         // any of first 4 channels above this value will trigger rx loss detection

    for (i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &masterConfig.rxConfig.failsafe_channel_configurations[i];
        channelFailsafeConfiguration->mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        channelFailsafeConfiguration->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(masterConfig.rxConfig.rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(masterConfig.rxConfig.midrc);
    }

    masterConfig.rxConfig.rssi_channel = 0;
    masterConfig.rxConfig.rssi_scale = RSSI_SCALE_DEFAULT;
    masterConfig.rxConfig.rssi_ppm_invert = 0;
    masterConfig.rxConfig.rcSmoothing = 1;
    masterConfig.rxConfig.fpvCamAngleDegrees = 0;

    resetAllRxChannelRangeConfigurations(masterConfig.rxConfig.channelRanges);

    masterConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;

    masterConfig.retarded_arm = 0;  // TODO - Cleanup retarded arm support
    masterConfig.disarm_kill_switch = 1;
    masterConfig.auto_disarm_delay = 5;
    masterConfig.small_angle = 25;

    resetMixerConfig(&masterConfig.mixerConfig);

    masterConfig.airplaneConfig.fixedwing_althold_dir = 1;

    // Motor/ESC/Servo
    resetEscAndServoConfig(&masterConfig.escAndServoConfig);
    resetFlight3DConfig(&masterConfig.flight3DConfig);

#ifdef BRUSHED_MOTORS
    masterConfig.motor_pwm_protocol = MOTOR_PWM_PROTOCOL_BRUSHED;
    masterConfig.motor_pwm_rate = BRUSHED_MOTORS_PWM_RATE;
#else
    masterConfig.motor_pwm_protocol = MOTOR_PWM_PROTOCOL_STD;
    masterConfig.motor_pwm_rate = BRUSHLESS_MOTORS_PWM_RATE;
#endif
    masterConfig.servo_pwm_rate = 50;

#ifdef CC3D
    masterConfig.use_buzzer_p6 = 0;
#endif

#ifdef GPS
    // gps/nav stuff
    masterConfig.gpsConfig.provider = GPS_NMEA;
    masterConfig.gpsConfig.sbasMode = SBAS_AUTO;
    masterConfig.gpsConfig.autoConfig = GPS_AUTOCONFIG_ON;
    masterConfig.gpsConfig.autoBaud = GPS_AUTOBAUD_OFF;
#endif

    resetSerialConfig(&masterConfig.serialConfig);

    masterConfig.emf_avoidance = 0;

    resetPidProfile(&currentProfile->pidProfile);

    resetControlRateConfig(&masterConfig.controlRateProfiles[0]);

    // for (i = 0; i < CHECKBOXITEMS; i++)
    //     cfg.activate[i] = 0;

    resetRollAndPitchTrims(&currentProfile->accelerometerTrims);

    currentProfile->mag_declination = 0;
    currentProfile->acc_lpf_hz = 20;
    currentProfile->accz_lpf_cutoff = 5.0f;
    currentProfile->accDeadband.xy = 40;
    currentProfile->accDeadband.z = 40;
    currentProfile->acc_unarmedcal = 1;

#ifdef BARO
    resetBarometerConfig(&currentProfile->barometerConfig);
#endif

    // Radio
    parseRcChannels("AETR1234", &masterConfig.rxConfig);

    resetRcControlsConfig(&currentProfile->rcControlsConfig);

    currentProfile->throttle_correction_value = 0;      // could 10 with althold or 40 for fpv
    currentProfile->throttle_correction_angle = 800;    // could be 80.0 deg with atlhold or 45.0 for fpv

    // Failsafe Variables
    masterConfig.failsafeConfig.failsafe_delay = 10;              // 1sec
    masterConfig.failsafeConfig.failsafe_off_delay = 10;          // 1sec
    masterConfig.failsafeConfig.failsafe_throttle = 1000;         // default throttle off.
    masterConfig.failsafeConfig.failsafe_kill_switch = 0;         // default failsafe switch action is identical to rc link loss
    masterConfig.failsafeConfig.failsafe_throttle_low_delay = 100; // default throttle low delay for "just disarm" on failsafe condition
    masterConfig.failsafeConfig.failsafe_procedure = 0;           // default full failsafe procedure is 0: auto-landing

#ifdef USE_SERVOS
    // servos
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        currentProfile->servoConf[i].min = DEFAULT_SERVO_MIN;
        currentProfile->servoConf[i].max = DEFAULT_SERVO_MAX;
        currentProfile->servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
        currentProfile->servoConf[i].rate = 100;
        currentProfile->servoConf[i].angleAtMin = DEFAULT_SERVO_MIN_ANGLE;
        currentProfile->servoConf[i].angleAtMax = DEFAULT_SERVO_MAX_ANGLE;
        currentProfile->servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }

    // gimbal
    currentProfile->gimbalConfig.mode = GIMBAL_MODE_NORMAL;
#endif

#ifdef GPS
    resetGpsProfile(&currentProfile->gpsProfile);
#endif

    // custom mixer. clear by defaults.
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        masterConfig.customMotorMixer[i].throttle = 0.0f;

#ifdef LED_STRIP
    applyDefaultColors(masterConfig.colors, CONFIGURABLE_COLOR_COUNT);
    applyDefaultLedStripConfig(masterConfig.ledConfigs);
#endif

#ifdef BLACKBOX
	featureSet(FEATURE_BLACKBOX);
#ifdef CONFIG_BLACKBOX_DEVICE
	masterConfig.blackbox_device = CONFIG_BLACKBOX_DEVICE;
#else
    masterConfig.blackbox_device = BLACKBOX_DEVICE_SERIAL;
#endif
    masterConfig.blackbox_rate_num = 1;
    masterConfig.blackbox_rate_denom = 1;
#endif

#ifdef CONFIG_FEATURE_RX_SERIAL
    featureSet(FEATURE_RX_SERIAL);
#endif
#ifdef CONFIG_FEATURE_ONESHOT125
    featureSet(FEATURE_ONESHOT);
    masterConfig.motor_pwm_protocol = MOTOR_PWM_PROTOCOL_125;
    masterConfig.motor_pwm_rate = BRUSHLESS_MOTORS_PWM_RATE;
#endif

#ifdef CONFIG_MSP_PORT
    masterConfig.serialConfig.portConfigs[CONFIG_MSP_PORT].functionMask = FUNCTION_MSP; 
    masterConfig.serialConfig.portConfigs[CONFIG_MSP_PORT].msp_baudrateIndex = BAUD_9600;
#endif
#ifdef CONFIG_RX_SERIAL_PORT
    masterConfig.serialConfig.portConfigs[CONFIG_RX_SERIAL_PORT].functionMask = FUNCTION_RX_SERIAL;
#endif

    // alternative defaults settings for ALIENFLIGHTF1 and ALIENFLIGHTF3 targets
#ifdef ALIENFLIGHT
    featureSet(FEATURE_MOTOR_STOP);
    masterConfig.rxConfig.spektrum_sat_bind = 5;
    masterConfig.escAndServoConfig.minthrottle = 1000;
    masterConfig.escAndServoConfig.maxthrottle = 2000;
    masterConfig.motor_pwm_protocol = MOTOR_PWM_PROTOCOL_BRUSHED;
    masterConfig.motor_pwm_rate = 32000;
    currentProfile->pidProfile.pidController = 2;
#ifdef ALIENFLIGHTF3
    masterConfig.batteryConfig.vbatscale = 20;
    masterConfig.mag_hardware = MAG_NONE;            // disabled by default
    currentProfile->pidProfile.P_f[ROLL] = 1.0f;
    currentProfile->pidProfile.I_f[ROLL] = 0.4f;
    currentProfile->pidProfile.D_f[ROLL] = 0.01f;
    currentProfile->pidProfile.P_f[PITCH] = 1.0f;
    currentProfile->pidProfile.I_f[PITCH] = 0.4f;
    currentProfile->pidProfile.D_f[PITCH] = 0.01f;
    currentProfile->pidProfile.P_f[YAW] = 4.0f;
    currentProfile->pidProfile.I_f[YAW] = 0.4f;
    currentProfile->pidProfile.D_f[YAW] = 0.00f;
#endif
#ifdef ALIENFLIGHTF4
    currentProfile->pidProfile.P_f[ROLL] = 5.000f;
    currentProfile->pidProfile.I_f[ROLL] = 1.000f;
    currentProfile->pidProfile.D_f[ROLL] = 0.080f;
    currentProfile->pidProfile.P_f[PITCH] = 5.000f;
    currentProfile->pidProfile.I_f[PITCH] = 1.000f;
    currentProfile->pidProfile.D_f[PITCH] = 0.080f;
    currentProfile->pidProfile.P_f[YAW] = 8.400f;
    currentProfile->pidProfile.I_f[YAW] = 1.200f;
    currentProfile->pidProfile.D_f[YAW] = 0.000f;
#endif
    masterConfig.failsafeConfig.failsafe_delay = 2;
    masterConfig.failsafeConfig.failsafe_off_delay = 0;
    masterConfig.mixerConfig.yaw_jump_prevention_limit = YAW_JUMP_PREVENTION_LIMIT_HIGH;
    currentControlRateProfile->rcRate8 = 100;
    currentControlRateProfile->rates[FD_PITCH] = 20;
    currentControlRateProfile->rates[FD_ROLL] = 20;
    currentControlRateProfile->rates[FD_YAW] = 30;
    parseRcChannels("TAER1234", &masterConfig.rxConfig);

    //  { 1.0f, -0.414178f,  1.0f, -1.0f },          // REAR_R
    masterConfig.customMotorMixer[0].throttle = 1.0f;
    masterConfig.customMotorMixer[0].roll = -0.414178f;
    masterConfig.customMotorMixer[0].pitch = 1.0f;
    masterConfig.customMotorMixer[0].yaw = -1.0f;

    //  { 1.0f, -0.414178f, -1.0f,  1.0f },          // FRONT_R
    masterConfig.customMotorMixer[1].throttle = 1.0f;
    masterConfig.customMotorMixer[1].roll = -0.414178f;
    masterConfig.customMotorMixer[1].pitch = -1.0f;
    masterConfig.customMotorMixer[1].yaw = 1.0f;

    //  { 1.0f,  0.414178f,  1.0f,  1.0f },          // REAR_L
    masterConfig.customMotorMixer[2].throttle = 1.0f;
    masterConfig.customMotorMixer[2].roll = 0.414178f;
    masterConfig.customMotorMixer[2].pitch = 1.0f;
    masterConfig.customMotorMixer[2].yaw = 1.0f;

    //  { 1.0f,  0.414178f, -1.0f, -1.0f },          // FRONT_L
    masterConfig.customMotorMixer[3].throttle = 1.0f;
    masterConfig.customMotorMixer[3].roll = 0.414178f;
    masterConfig.customMotorMixer[3].pitch = -1.0f;
    masterConfig.customMotorMixer[3].yaw = -1.0f;

    //  { 1.0f, -1.0f, -0.414178f, -1.0f },          // MIDFRONT_R
    masterConfig.customMotorMixer[4].throttle = 1.0f;
    masterConfig.customMotorMixer[4].roll = -1.0f;
    masterConfig.customMotorMixer[4].pitch = -0.414178f;
    masterConfig.customMotorMixer[4].yaw = -1.0f;

    //  { 1.0f,  1.0f, -0.414178f,  1.0f },          // MIDFRONT_L
    masterConfig.customMotorMixer[5].throttle = 1.0f;
    masterConfig.customMotorMixer[5].roll = 1.0f;
    masterConfig.customMotorMixer[5].pitch = -0.414178f;
    masterConfig.customMotorMixer[5].yaw = 1.0f;

    //  { 1.0f, -1.0f,  0.414178f,  1.0f },          // MIDREAR_R
    masterConfig.customMotorMixer[6].throttle = 1.0f;
    masterConfig.customMotorMixer[6].roll = -1.0f;
    masterConfig.customMotorMixer[6].pitch = 0.414178f;
    masterConfig.customMotorMixer[6].yaw = 1.0f;

    //  { 1.0f,  1.0f,  0.414178f, -1.0f },          // MIDREAR_L
    masterConfig.customMotorMixer[7].throttle = 1.0f;
    masterConfig.customMotorMixer[7].roll = 1.0f;
    masterConfig.customMotorMixer[7].pitch = 0.414178f;
    masterConfig.customMotorMixer[7].yaw = -1.0f;
#endif

    // copy first profile into remaining profile
    for (i = 1; i < MAX_PROFILE_COUNT; i++) {
        memcpy(&masterConfig.profile[i], currentProfile, sizeof(profile_t));
    }

    // copy first control rate config into remaining profile
    for (i = 1; i < MAX_CONTROL_RATE_PROFILE_COUNT; i++) {
        memcpy(&masterConfig.controlRateProfiles[i], currentControlRateProfile, sizeof(controlRateConfig_t));
    }

    for (i = 1; i < MAX_PROFILE_COUNT; i++) {
        masterConfig.profile[i].defaultRateProfileIndex = i % MAX_CONTROL_RATE_PROFILE_COUNT;
    }
}

static uint8_t calculateChecksum(const uint8_t *data, uint32_t length)
{
    uint8_t checksum = 0;
    const uint8_t *byteOffset;

    for (byteOffset = data; byteOffset < (data + length); byteOffset++)
        checksum ^= *byteOffset;
    return checksum;
}

static bool isEEPROMContentValid(void)
{
    const master_t *temp = (const master_t *) CONFIG_START_FLASH_ADDRESS;
    uint8_t checksum = 0;

    // check version number
    if (EEPROM_CONF_VERSION != temp->version)
        return false;

    // check size and magic numbers
    if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return false;

    // verify integrity of temporary copy
    checksum = calculateChecksum((const uint8_t *) temp, sizeof(master_t));
    if (checksum != 0)
        return false;

    // looks good, let's roll!
    return true;
}

void activateControlRateConfig(void)
{
    generatePitchRollCurve(currentControlRateProfile);
    generateYawCurve(currentControlRateProfile);
    generateThrottleCurve(currentControlRateProfile, &masterConfig.escAndServoConfig);
}

void activateConfig(void)
{
    static imuRuntimeConfig_t imuRuntimeConfig;

    activateControlRateConfig();

    resetAdjustmentStates();

    useRcControlsConfig(
        currentProfile->modeActivationConditions,
        &masterConfig.escAndServoConfig,
        &currentProfile->pidProfile
    );

    useGyroConfig(&masterConfig.gyroConfig, currentProfile->pidProfile.gyro_lpf_hz);

#ifdef TELEMETRY
    telemetryUseConfig(&masterConfig.telemetryConfig);
#endif
    currentProfile->pidProfile.pidController = constrain(currentProfile->pidProfile.pidController, 1, 2); // This should prevent UNUSED values. CF 1.11 support
    pidSetController(currentProfile->pidProfile.pidController);

#ifdef GPS
    gpsUseProfile(&currentProfile->gpsProfile);
    gpsUsePIDs(&currentProfile->pidProfile);
#endif

    useFailsafeConfig(&masterConfig.failsafeConfig);
    setAccelerationTrims(&masterConfig.accZero);

    mixerUseConfigs(
#ifdef USE_SERVOS
        currentProfile->servoConf,
        &currentProfile->gimbalConfig,
#endif
        &masterConfig.flight3DConfig,
        &masterConfig.escAndServoConfig,
        &masterConfig.mixerConfig,
        &masterConfig.airplaneConfig,
        &masterConfig.rxConfig
    );

    imuRuntimeConfig.dcm_kp = masterConfig.dcm_kp / 10000.0f;
    imuRuntimeConfig.dcm_ki = masterConfig.dcm_ki / 10000.0f;
    imuRuntimeConfig.acc_cut_hz = currentProfile->acc_lpf_hz;
    imuRuntimeConfig.acc_unarmedcal = currentProfile->acc_unarmedcal;
    imuRuntimeConfig.small_angle = masterConfig.small_angle;

    imuConfigure(
        &imuRuntimeConfig,
        &currentProfile->pidProfile,
        &currentProfile->accDeadband,
        currentProfile->accz_lpf_cutoff,
        currentProfile->throttle_correction_angle
    );

#if defined(BARO) || defined(SONAR)
    configureAltitudeHold(
        &currentProfile->pidProfile,
        &currentProfile->barometerConfig,
        &currentProfile->rcControlsConfig,
        &masterConfig.escAndServoConfig
    );
#endif

#ifdef BARO
    useBarometerConfig(&currentProfile->barometerConfig);
#endif
}

void validateAndFixConfig(void)
{
    if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_PPM) || featureConfigured(FEATURE_RX_SERIAL) || featureConfigured(FEATURE_RX_MSP))) {
        featureSet(FEATURE_RX_PARALLEL_PWM); // Consider changing the default to PPM
    }

    if (featureConfigured(FEATURE_RX_PPM)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
    }

    if (featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_SERIAL);
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_SERIAL)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
#if defined(STM32F10X)
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
            featureClear(FEATURE_CURRENT_METER);
        }
#endif

#if defined(STM32F10X) || defined(CHEBUZZ) || defined(STM32F3DISCOVERY)
        // led strip needs the same ports
        featureClear(FEATURE_LED_STRIP);
#endif

        // software serial needs free PWM ports
        featureClear(FEATURE_SOFTSERIAL);
    }


#if defined(LED_STRIP) && (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (featureConfigured(FEATURE_SOFTSERIAL) && (
            0
#ifdef USE_SOFTSERIAL1
            || (LED_STRIP_TIMER == SOFTSERIAL_1_TIMER)
#endif
#ifdef USE_SOFTSERIAL2
            || (LED_STRIP_TIMER == SOFTSERIAL_2_TIMER)
#endif
    )) {
        // led strip needs the same timer as softserial
        featureClear(FEATURE_LED_STRIP);
    }
#endif

#if defined(NAZE) && defined(SONAR)
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM) && featureConfigured(FEATURE_SONAR) && featureConfigured(FEATURE_CURRENT_METER) && masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(OLIMEXINO) && defined(SONAR)
    if (feature(FEATURE_SONAR) && feature(FEATURE_CURRENT_METER) && masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(CC3D) && defined(DISPLAY) && defined(USE_USART3)
    if (doesConfigurationUsePort(SERIAL_PORT_USART3) && feature(FEATURE_DISPLAY)) {
        featureClear(FEATURE_DISPLAY);
    }
#endif

#ifdef STM32F303xC
    // hardware supports serial port inversion, make users life easier for those that want to connect SBus RX's
    masterConfig.telemetryConfig.telemetry_inversion = 1;
#endif

#if defined(CC3D) && defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        featureClear(FEATURE_SONAR);
    }
#endif

#if defined(COLIBRI_RACE) || defined(LUX_RACE)
    masterConfig.serialConfig.portConfigs[0].functionMask = FUNCTION_MSP;
    if(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_MSP);
        featureSet(FEATURE_RX_PPM);
    }
    if(featureConfigured(FEATURE_RX_SERIAL)) {
        masterConfig.serialConfig.portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
        //masterConfig.rxConfig.serialrx_provider = SERIALRX_SBUS;
    }
#endif

    useRxConfig(&masterConfig.rxConfig);

    serialConfig_t *serialConfig = &masterConfig.serialConfig;

    if (!isSerialConfigValid(serialConfig)) {
        resetSerialConfig(serialConfig);
    }
}

void initEEPROM(void)
{
}

void readEEPROM(void)
{
    // Sanity check
    if (!isEEPROMContentValid())
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);

    suspendRxSignal();

    // Read flash
    memcpy(&masterConfig, (char *) CONFIG_START_FLASH_ADDRESS, sizeof(master_t));

    if (masterConfig.current_profile_index > MAX_PROFILE_COUNT - 1) // sanity check
        masterConfig.current_profile_index = 0;

    setProfile(masterConfig.current_profile_index);

    if (currentProfile->defaultRateProfileIndex > MAX_CONTROL_RATE_PROFILE_COUNT - 1) // sanity check
        currentProfile->defaultRateProfileIndex = 0;

    setControlRateProfile(currentProfile->defaultRateProfileIndex);

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
}

void readEEPROMAndNotify(void)
{
    // re-read written data
    readEEPROM();
    beeperConfirmationBeeps(1);
}

void writeEEPROM(void)
{
    // Generate compile time error if the config does not fit in the reserved area of flash.
    BUILD_BUG_ON(sizeof(master_t) > FLASH_TO_RESERVE_FOR_CONFIG);

    FLASH_Status status = 0;
    uint32_t wordOffset;
    int8_t attemptsRemaining = 3;

    suspendRxSignal();

    // prepare checksum/version constants
    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.size = sizeof(master_t);
    masterConfig.magic_be = 0xBE;
    masterConfig.magic_ef = 0xEF;
    masterConfig.chk = 0; // erase checksum before recalculating
    masterConfig.chk = calculateChecksum((const uint8_t *) &masterConfig, sizeof(master_t));

    // write it
    FLASH_Unlock();
    while (attemptsRemaining--) {
#ifdef STM32F40_41xxx
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif
#ifdef STM32F411xE
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif
#ifdef STM32F303
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#endif
#ifdef STM32F10X
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#endif
        for (wordOffset = 0; wordOffset < sizeof(master_t); wordOffset += 4) {
            if (wordOffset % FLASH_PAGE_SIZE == 0) {
#if defined(STM32F40_41xxx)
                //status = FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3); //0x080E0000 to 0x08100000
                status = FLASH_EraseSector(FLASH_Sector_8, VoltageRange_3); //0x08080000 to 0x080A0000
#elif defined (STM32F411xE)
                status = FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3); //0x08060000 to 0x08080000
#else
                status = FLASH_ErasePage(CONFIG_START_FLASH_ADDRESS + wordOffset);
#endif
                if (status != FLASH_COMPLETE) {
                    break;
                }
            }

            status = FLASH_ProgramWord(CONFIG_START_FLASH_ADDRESS + wordOffset,
                    *(uint32_t *) ((char *) &masterConfig + wordOffset));
            if (status != FLASH_COMPLETE) {
                break;
            }
        }
        if (status == FLASH_COMPLETE) {
            break;
        }
    }
    FLASH_Lock();

    // Flash write failed - just die now
    if (status != FLASH_COMPLETE || !isEEPROMContentValid()) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
    }

    resumeRxSignal();
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
        return;
    }

    resetEEPROM();
}

void resetEEPROM(void)
{
    resetConf();
    writeEEPROM();
}

void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROMAndNotify();
}

void changeProfile(uint8_t profileIndex)
{
    masterConfig.current_profile_index = profileIndex;
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(profileIndex + 1);
}

void changeControlRateProfile(uint8_t profileIndex)
{
    if (profileIndex > MAX_CONTROL_RATE_PROFILE_COUNT) {
        profileIndex = MAX_CONTROL_RATE_PROFILE_COUNT - 1;
    }
    setControlRateProfile(profileIndex);
    activateControlRateConfig();
}

void handleOneshotFeatureChangeOnRestart(void)
{
    // Shutdown PWM on all motors prior to soft restart
    StopPwmAllMotors();
    delay(50);
    // Apply additional delay when OneShot125 feature changed from on to off state
    delay(ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS);
}

void latchActiveFeatures()
{
    activeFeaturesLatch = masterConfig.enabledFeatures;
}

bool featureConfigured(uint32_t mask)
{
    return masterConfig.enabledFeatures & mask;
}

bool feature(uint32_t mask)
{
    return activeFeaturesLatch & mask;
}

void featureSet(uint32_t mask)
{
    masterConfig.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    masterConfig.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    masterConfig.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return masterConfig.enabledFeatures;
}
