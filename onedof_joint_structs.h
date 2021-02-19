#ifndef ACTUATOR_STRUCTS
#define ACTUATOR_STRUCTS

/*****************************************************************************************
 * INCLUDES
 ****************************************************************************************/

#include <iostream>
//#include "actuator_controller_interface.h"
#include <stdbool.h>
#include <map>

using namespace std;

/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/

// esmacat control modes
#define EXIT        0
#define STOP        1
#define CURRENT     2
#define TORQUE      3
#define NULLTORQUE  4
#define GRAVITY     5
#define FREEZE      6
#define IMPEDANCE   7
#define HOMING      8
#define POSITION    9
#define WEIGHT      10
#define IMPEDANCE_EXT 11
#define TRIGGER     12

#define PASSIVE     101
#define ADAPTIVE    102
#define ANTI_G      103
#define TRANSPARENT 104
#define RESISTIVE   105
#define CHALLENGING 106

#define HOMING_DONE 108
#define POSITION_DONE 109

/** enum for control mode of the robot */
enum robot_control_mode_t{
    quit                 = 0,
    standby              = 1,
    current_control      = 2,
    torque_control       = 3,
    zerotorque_control   = 4,
    gravity_control      = 5,
    freeze_control       = 6,
    impedance_control    = 7,
    homing_control          = 8,
    homing_done             = 108,
    go_position_control     = 9,
    go_position_done        = 109,
    weight_comp_control     = 10,
    impedance_ext_control   = 11,
    speed_control           = 12,
    passive_control         = PASSIVE,
    adaptive_control        = ADAPTIVE,
    anti_g_control          = ANTI_G,
    transparent_control     = TRANSPARENT,
    resistive_control       = RESISTIVE,
    challenging_control     = CHALLENGING,
};

/** Labels for control mode of the robot */
const string robot_mode_labels[] = {
    "EXIT",
    "STOP",
    "CURRENT",
    "TORQUE",
    "NULLTORQUE",
    "GRAVITY",
    "FREEZE",
    "IMPEDANCE",
    "HOMING",
    "POSITION",
    "WEIGHT",
    "IMPEDANCE_EXT",
    "TRIGGER",
    "ADAPTIVE",
    "PASSIVE",
    "RESISTIVE",
    "CHALLENGING",
};

/** Labels for robot control mode */
static map< robot_control_mode_t, const char * > robot_control_labels = {
    {quit,               "Quit"},
    {standby,            "STAND-BY"},
    {current_control,    "CURRENT"},
    {torque_control,     "TORQUE"},
    {zerotorque_control, "ZERO-TORQUE"},
    {gravity_control,    "GRAVITY"},
    {freeze_control,     "FREEZE"},
    {impedance_control,   "IMPEDANCE"},
    {homing_control,            "HOMING"},
    {homing_done,               "HOMING_DONE"},
    {go_position_control,       "GO-POSITION"},
    {go_position_done,          "GO-POSITION_DONE"},
    {weight_comp_control,       "WEIGHT_COMPENSATION"},
    {impedance_ext_control,     "IMPEDANCE_EXT"},
    {speed_control,             "SPEED_CONTROL"},

    {passive_control,           "PASSIVE_CONTROL"},
    {adaptive_control,          "ADAPTIVE_CONTROL"},
    {anti_g_control,            "ANTI_G_CONTROL"},
    {transparent_control,       "TRANSPARENT"},
    {resistive_control,         "RESISTIVE"},
    {challenging_control,       "CHALLENGING"},
};

/** enum for control mode of actuator controller */
enum class actuator_control_mode_t { direct_escon_control, torque_control =  2};

/** Labels for control mode of actuator controller */
const string control_mode_labels[] = {
  "DIRECT",
  "",
  "TORQUE",
};

/*****************************************************************************************
 * STRUCTS
 ****************************************************************************************/

/** @brief Structure that holds all the readings of the actuator for each joint **/
struct joint_values_t
{
    /** Index of the joint within the Harmony */
    int joint_index = -1;
    /** Error in ESCON (1), No errors in ESCON (0); IN_MD_FAULT on ESCON datasheet */
    int escon_fault = 0;
    /** Loadcell reading for the joint in milli-Nm */
    double loadcell_reading_mNm = 0.0;
    /** Filtered Loadcell reading for the joint in milli-Nm */
    float filtered_load_mNm = 0.0;
    /** Raw reading with sign applied from the absolute encoder in counts per turn */
    int16_t signed_raw_absolute_encoder_reading_cpt = 0;
    /** Unfiltered reading from the absolute encoder in counts per turn */
    int16_t unfiltered_absolute_encoder_reading_cpt = 0;
    /** Filtered reading from the absolute encoder in counts per turn */
    float filtered_absolute_encoder_reading_cpt = 0.0;
    /** Filtered reading from the absolute encoder in degrees */
    float filtered_absolute_encoder_reading_degrees =0.0;
    /** Filtered reading from the absolute encoder in radians */
    float filtered_absolute_encoder_reading_radians =0.0;
    /** Unfiltered reading from the incremental encoder in counts per turn */
    int32_t unfiltered_incremental_encoder_reading_cpt =0;
    /** Reading (with offset applied) from the incremental encoder in counts per turn */
    int incremental_encoder_reading_cpt =0;

//    /** Acceleration in counts per sec^2 from incremental encoder */
//    float acceleration_counts_per_sec_square = 0.0;
    /** Reading (with offset applied) from the incremental encoder in degrees*/
    float incremental_encoder_reading_degrees =0.0;
    /** Reading (with offset applied) from the incremental encoder in counts radians */
    double incremental_encoder_reading_radians =0.0;

    float incremental_encoder_speed_radians_sec = 0.0;

    /** Velocity reading in counts/sec from incremental encoder*/
    float incremental_encoder_speed_radians_sec_driver = 0.0;

    /** Length reading the linear actuator in mm */
    float linear_actuator_length_mm =0.0;
    /** Default initialization of all the readings (loadcell readings, absolute encoder readings
     * and incremental encoder) for the joint to 0; Joint index is set to -1
     */

    float torque_setpoint_mNm = 0.0;

    bool encoder_calibrated = false;
};

/** @brief Structure that holds the configuration of the actuator for each joint **/
struct sea_joint_configuration_t
{
    /** Index of the joint within the Harmony */
    int joint_index = 0;
    /** Torque constant of the motor in milli-Nm per mA */
    float torque_constant_mNm_per_A = 0.0;
    /** Gear ratio of the motor (output shaft to motor shaft) */
    uint16_t gear_ratio = 0;
    /** Gear ratio of the motor (output shaft to motor shaft) */
    uint16_t transmission_ratio = 0;
//    /** motor_rotor_inertia_g_per_cm2 */
//    float motor_rotor_inertia_g_per_cm2 = 0.0;
//    /** motor_rotor_inertia_g_per_cm2 */
//    float gearhead_rotor_inertia_g_per_cm2 = 0.0;
    /** Gear power efficiency of the motor (output shaft to motor shaft) */
    float gear_power_efficiency = 0.0;
    /** The setpoint for the ESCON is normalized, this is the conversion factor to
     * convert the setpoint back to mA*/
    /** Sign for the current setpoint for the ESCON */
    int desired_torque_sign = 1;
    /** TRUE = Incremental encoder readings are used;
     * FALSE = Incremental encoder readings are not used */
    bool use_incremental_encoder = 1;
    /** Default initialization of the configuration paramters for the joint actuator */
    /** Hard stop position for the joint in positive direction in degrees */
    float hard_stop_upper_limit_degrees = 180.0;
    /** Hard stop position for the joint in negative direction in degrees */
    float hard_stop_lower_limit_degrees = -180.0;

    /** Offset to be applied to the loadcell reading in mV */
    float loadcell_offset_mV = 0.0;
    /** Factor that translates the loadcell reading in mV to load reading in milli-Nm */
    float loadcell_calibration_mV_to_mNm = 0.0;
    /** Sign of the readings with positive load */
    int loadcell_sign = 1;

    /** Offset for the linear actuator reading in mV */
    float linear_actuator_offset_mV = 0.0;
    /** Calibration factor to convert the linear actuator reading from mV to mm */
    float linear_actuator_calibration_mV_to_mm = 0.0;

    /** The setpoint for the ESCON is normalized, this is the conversion factor to
     * convert the setpoint back to mA*/
    float current_conversion_factor_A_to_setpoint = 0.0;

    /** Offset of the absolute encoder in counts */
    int16_t absolute_encoder_offset_counts = 0;
    /** Offset of the incremental encoder in counts */
    /** Sign of the readings from the absolute encoder with positive position */
    int absolute_encoder_sign = 1;
    /** Resolution of the incremental encoder in counts per turn */
    int incremental_encoder_resolution_cpt = 0;
    /** Offset of the incremental encoder in counts */
    int32_t incremental_encoder_offset_counts = 0;
    /** Sign of the readings from the incremental encoder with positive position */
    int incremental_encoder_sign = 1;

    /** Incremental encoder offset from endstop to zero position */
    float calibration_offset_rad = 0.0;
    /** Incremental encoder homing direction */
    int calibration_sign = 0.0;
    /** Calibration threshold torque */
    float calibration_threshold_torque = 0.0;

    float escon_analog_output0_voltage_V_to_current_A_offset = 0.0;
    float escon_analog_output0_voltage_V_to_current_A_slope = 0.0;
    float escon_analog_output1_velocity_V_to_current_rpm_offset = 0.0;
    float escon_analog_output1_velocity_V_to_current_rpm_slope = 0.0;

};

/** Holds the configuration parameters for the torque, position and
impedance control loops*/
struct sea_controller_configuration_t
{
    /** Proportional gain for the torque control loop */
    float torque_control_p_gain = 0.0;
    /** Integral gain for the torque control loop */
    float torque_control_i_gain = 0.0;
    /** Derivative gain for the torque control loop */
    float torque_control_d_gain = 0.0;
    /** Max allowable torque change in milli-Nm per interval */
    uint16_t max_torque_change_mNm_per_ms = 0;
    /** Proportional gain for the position control loop */
    float position_control_p_gain = 0.0;
    /** Integral gain for the position control loop */
    float position_control_i_gain = 0.0;
    /** Derivative gain for the position control loop */
    float position_control_d_gain = 0.0;
    /** Proportional gain for the impedance control loop */
    double impedance_control_k_gain_mNm_per_rad = 0.0;
    /** Derivative gain for the impedance control loop */
    double impedance_control_d_gain_mNm_per_rad_per_sec =0.0;
    /** Maximum error in radians (used for limiting the measured error)
     * allowed in the impedance control loop */
    float impedance_control_max_error_radians = 0.0;

    double impedance_control_setpoint_rad = 0.0;
    /** Maximum allowed torque in milli-Nm used to limit the torque
     * applied to compensate for friction */
    float friction_comp_max_torque_mNm = 0.0;
    float friction_torque_threshold_rad_per_s = 0.0;

    double friction_comp_vibration_frequency = 0.0;
    double friction_comp_vibration_amplitude = 0.0;
    float friction_comp_viscous_coeff = 0.0;

    float soft_to_hard_stop_offset_deg = 0.0;
    float soft_stop_max_torque_mNm = 0.0;
    /** Maximum allowable velocity under impedance control in rads/sec */
    float max_velocity_threshold_rad_per_sec = 0.0;
    /** Maximum allowable loadcell reading in milli-Nm; used to disable
     * motor */

    float max_torque_control_input_mNm = 0.0;

    float min_torque_control_input_mNm = 0.0;

    float velocity_low_pass_filter_weight_for_current_measure = 1.0;

    float loadcell_low_pass_filter_weight_for_current_measure = 1.0;

    float gain_inertia_dynamics_compensation = 0.0;

    float max_integrated_torque_error_mNm = 0.0;

    float max_allowable_redundancy_error_for_motor_current_mA = 0.0;

    float max_allowable_redundancy_error_for_motor_velocity_rad_per_sec = 0.0;

    /** Sets the control mode in which the controller is used: <br>
     * 2 = torque control performed on the slave <br>
     * anything else = current control through ESCON performed on slave*/
    actuator_control_mode_t control_mode;

    /** Initializes all the parameters of the torque, position
    and impedance control loops to 0 */
};

/** Holds all values pertaining to the torque control loop */
struct torque_control_terms_t
{

    /** Torque setpoint in milli-Nm that is provided as an input to torque control loop */
    float torque_setpoint_mNm;
    /** Difference between the torque setpoint and the measured torque in milli-Nm */
    float torque_error_mNm;

    /** Torque output by the Proportional control in the PID */
    float torque_feedback_p_mNm;
    /** Torque output by the Integral control in the PID */
    float torque_feedback_i_mNm;
    /** Torque output by the Derivative control in the PID */
    float torque_feedback_d_mNm;

    /** Feedforward torque in milli-Nm that is applied in addition to the PID output */
    float torque_feedforward_demanded_mNm = 0;
    /** Torque in milli-Nm output by the PID */
    float torque_feedback_demanded_mNm;
    /** Torque demanded in milli-Nm (PID+FF) */
    float torque_total_demanded_mNm;

    /** Initialize all values of the struct to 0 */
    torque_control_terms_t()
        {
        torque_setpoint_mNm=0;
        torque_error_mNm=0;
        torque_feedback_p_mNm=0;
        torque_feedback_i_mNm=0;
        torque_feedback_d_mNm=0;
        torque_feedforward_demanded_mNm =0;
        torque_feedback_demanded_mNm=0;
        torque_total_demanded_mNm=0;
    }
};

/** Holds all values pertaining to the impedance control loop */
struct impedance_control_terms_t
{

    /** Difference between the position setpoint and the measured
     * setpoint in radians */
    float error_radians = 0.0;
    /** Output torque of the PD impedance control in milli-Nm */
    float impedance_control_torque_mNm = 0.0;
    /** Torque applied due to gravity in milli-Nm */
    float gravity_torque_mNm = 0.0;
    /** Torque required to compensate for friction in milli-Nm */
    float friction_comp_torque_mNm = 0.0;
    /** Torque used to implement soft-stops */
    float soft_stop_torque_mNm = 0.0;
    /** Torque setpoint in milli-Nm that is provided as an output of impedance control */
    float torque_setpoint_mNm = 0.0;

    /** Position setpoint for impedance control in radians */
    float impedance_control_setpoint_rad = 0.0;
    /** Proportional gain for the impedance control loop */
    float impedance_control_k_mNm_rad = 0.0;
    /** Derivative gain for the impedance control loop */
    float impedance_control_d_mNm_rad_per_sec    = 0.0;

    bool impedance_control_first_run = true;
    // initialize all values to 0
};

/** Holds the commanded impedance control parameters */
struct impedance_control_command_t{
    /** Proportional gain for the impedance control loop */
    float impedance_control_k_gain_mNm_per_rad;
    /** Derivative gain for the impedance control loop */
    float impedance_control_d_gain_mNm_per_rad_per_sec;
    /** Position setpoint for impedance control in radians */
    float impedance_control_setpoint_rad;
    /** Feedforward torque for impedance controller */
    float impedance_control_feedforward_torque_mNm;


    /** Negative and positive soft-stops */
    float soft_stop_lower_limit_rad;
    float soft_stop_upper_limit_rad;

    impedance_control_command_t()
    {
        impedance_control_k_gain_mNm_per_rad = 0;
        impedance_control_d_gain_mNm_per_rad_per_sec = 0;
        impedance_control_setpoint_rad = -1;
        impedance_control_feedforward_torque_mNm = 0;
        soft_stop_lower_limit_rad = -180;
        soft_stop_upper_limit_rad = 180;
    }
};

/** Holds the weight compensation parameters */
struct arm_weight_compensation_config_t{

    /** Human weight */
    float human_weight_kg;
    /** Percentage of weight assistance for human arm */
    float weight_assistance;
    /** Human height */
    float human_height_m;

    /** Forearm length */
    float forearm_length_m;
    /** Upperarm length */
    float upperarm_length_m;

    /** Initializes all the parameters to 0 */
    arm_weight_compensation_config_t()
    {
        human_weight_kg = 70;
        human_height_m = 1.80;
        weight_assistance = 1.0;
        forearm_length_m = 0;
        upperarm_length_m = 0;
    }
};

#endif // ACTUATOR_STRUCTS
