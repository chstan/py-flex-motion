// Signed integer type.
typedef long i32;
typedef double f64;
typedef float f32;
typedef short i16;
typedef char i8;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;
typedef unsigned long tBoolean;

// Axis Configuration Attributes
typedef enum
{
   TnimcAxisConfigurationParameterEnable = 0,
} TnimcAxisConfigurationParameter;

// Move Constraint Attributes
typedef enum
{
   TnimcMoveConstraintVelocity = 0,
   TnimcMoveConstraintAcceleration,
   TnimcMoveConstraintDeceleration,
   TnimcMoveConstraintAccelerationJerk,
   TnimcMoveConstraintDecelerationJerk,
} TnimcMoveConstraint;

// Motion IO Attributes
typedef enum
{
   TnimcMotionIOParameterForwardLimitEnable = 0,
   TnimcMotionIOParameterReverseLimitEnable,
   TnimcMotionIOParameterForwardSoftwareLimitEnable,
   TnimcMotionIOParameterReverseSoftwareLimitEnable,
   TnimcMotionIOParameterHomeInputEnable,
   TnimcMotionIOParameterForwardLimitPolarity,
   TnimcMotionIOParameterReverseLimitPolarity,
   TnimcMotionIOParameterHomeInputPolarity,
   TnimcMotionIOParameterForwardSoftwareLimitPosition,
   TnimcMotionIOParameterReverseSoftwareLimitPosition,
   TnimcMotionIOParameterInhibitInEnable,
   TnimcMotionIOParameterInhibitInPolarity,
   TnimcMotionIOParameterInPositionEnable,
   TnimcMotionIOParameterInPositionPolarity,
} TnimcMotionIOParameter;

// Motion IO Status
typedef enum
{
   TnimcMotionIOExecutionForwardLimitStatus = 0,
   TnimcMotionIOExecutionReverseLimitStatus,
   TnimcMotionIOExecutionForwardSoftwareLimitStatus,
   TnimcMotionIOExecutionReverseSoftwareLimitStatus,
   TnimcMotionIOExecutionHomeInputStatus,
   TnimcMotionIOExecutionInhibitInStatus,
   TnimcMotionIOExecutionInPositionStatus,
} TnimcMotionIOExecution;

// Camming Attributes
typedef enum
{
   TnimcCammingParameterMasterCycle = 0,
   TnimcCammingParameterMasterOffset,
   TnimcCammingParameterSlaveOffset,
} TnimcCammingParameter;


/* PID
**    This structure is passed to flex_load_pid_parameters() function
**    to configure all the control gains at the same time.
*/
typedef struct
{
   u16 kp;
   u16 ki;
   u16 ilim;
   u16 kd;
   u16 td;
   u16 kv;
   u16 aff;
   u16 vff;
} PID;

/* REGISTRY
**   This structure is passed to flex_read_registry() function in order to
**   return info about the object registry entry.
*/
typedef struct
{
   u16 device;
   u16 type;
   u32 pstart;
   u32 size;
} REGSTRY, REGISTRY;


/* NIMC_CAMMING_ENABLE_DATA
**    This structure is used to enable/disable camming operation for
**    multiple axis at the same time.
*/
#pragma pack(push, 1)
typedef struct
{
   i32 axisIndex;
   u8 enable;
   f64 position;
} NIMC_CAMMING_ENABLE_DATA;
#pragma pack(pop)


/* NIMC_DATA
**    This structure is used for various configurtion functions to
**    load parameter with different data types.
*/
#pragma pack(push, 1)
typedef struct
{
   i32 longData;
   u8 boolData;
   f64 doubleData;
} NIMC_DATA;
#pragma pack(pop)


// <BEGINFUNCTIONS>

// The Initialization function
i32 flex_initialize_controller (u8 boardID, i8 *settingsName);

///////////////////////////////////////////////////////////////////////////////////////
// Axis & Resource Configuration Functions
///////////////////////////////////////////////////////////////////////////////////////

//Configure Axis Resources
i32 flex_config_axis(u8 boardID, u8 axis, u8 primaryFeedback, u8 secondaryFeedback,
                          u8 primaryOutput, u8 secondaryOutput);

//Configure Move Complete Status Criteria
i32 flex_config_mc_criteria(u8 boardID, u8 axis, u16 criteria, u16 deadband, u8 delay,
                                 u8 minPulse);

//Configure Stepper Output
i32 flex_configure_stepper_output(u8 boardID, u8 axisOrStepperOutput, u16 outputMode, u16 polarity, u16 driveMode);

//Configure Vector Space
i32 flex_config_vect_spc(u8 boardID, u8 vectorSpace, u8 xAxis, u8 yAxis, u8 zAxis);

//Enable Axes
i32 flex_enable_axis(u8 boardID, u8 reserved, u8 PIDRate, u16 axisMap);

//Load Advanced Control Parameter
i32 flex_load_advanced_control_parameter(u8 boardID, u8 axis, u16 parameterType, u32 value, u8 inputVector);

// Load Counts/Steps per Revolution
i32 flex_load_counts_steps_rev(u8 boardID, u8 axis, u16 unitType, u32 countsOrSteps);

//Load All PID Parameters
i32 flex_load_pid_parameters(u8 boardID, u8 axis, PID *PIDValues, u8 inputVector);

// Load Single PID Parameter
i32 flex_load_single_pid_parameter(u8 boardID, u8 axis, u16 parameterType, u16 PIDValue, u8 inputVector);

//Set Stepper Loop Mode
i32 flex_set_stepper_loop_mode(u8 boardID, u8 axis, u16 loopMode);

//Load Velocity Filter Parameter
i32 flex_load_velocity_filter_parameter(u8 boardID, u8 axis, u16 filterDistance, u16 filterTime, u8 inputVector);

//Load Commutation Mode
i32 flex_load_commutation_parameter(u8 boardID, u8 axis, u16 attribute, f64 value);

// Load Axis Configuration Parameter
i32 flex_load_axis_configuration_parameter(i32 boardID, i32 axisID, TnimcAxisConfigurationParameter attribute, NIMC_DATA* data);

///////////////////////////////////////////////////////////////////////////////////////
// Trajectory Functions
///////////////////////////////////////////////////////////////////////////////////////

// Load Acceleraton
i32 flex_load_acceleration(u8 boardID, u8 axisOrVectorSpace, u16 accelerationType, u32 acceleration, u8 inputVector);

//Load Following Error
i32 flex_load_follow_err(u8 boardID, u8 axis, u16 followingError, u8 inputVector);

//Load Velocity in RPM
i32 flex_load_rpm(u8 boardID, u8 axisOrVectorSpace, f64 RPM, u8 inputVector);

// Load Acceleration in RPS/sec
i32 flex_load_rpsps(u8 boardID, u8 axisOrVectorSpace, u16 accelerationType, f64 RPSPS, u8 inputVector);

//Load Target Position
i32 flex_load_target_pos(u8 boardID, u8 axis, i32 targetPosition, u8 inputVector);

//Load Velocity
i32 flex_load_velocity(u8 boardID, u8 axisOrVectorSpace, i32 velocity, u8 inputVector);

//Load Vector Space Position
i32 flex_load_vs_pos (u8 boardID, u8 vectorSpace, i32 xPosition, i32 yPosition, i32 zPosition, u8 inputVector);

//Load Move Constraints
i32 flex_load_move_constraint (i32 boardID, i32 axisID, TnimcMoveConstraint attribute, NIMC_DATA* value);

//Read per Axis Status
i32 flex_read_axis_status(u8 boardID, u8 axis, u8 returnVector);
//Read per Axis Status
i32 flex_read_axis_status_rtn(u8 boardID, u8 axis, u16 *axisStatus);

//Read Blend Status
i32 flex_read_blend_status(u8 boardID, u8 axisOrVectorSpace, u8 returnVector);
//Read Blend Status
i32 flex_read_blend_status_rtn(u8 boardID, u8 axisOrVectorSpace, u16 *blendStatus);

//Read Following Error
i32 flex_read_follow_err(u8 boardID, u8 axisOrVectorSpace, u8 returnVector);
//Read Following Error
i32 flex_read_follow_err_rtn(u8 boardID, u8 axisOrVectorSpace, i16 *followingError);

//Read Motion Complete Status
i32 flex_read_mcs_rtn(u8 boardID, u16 *moveCompleteStatus);

//Read Position
i32 flex_read_pos(u8 boardID, u8 axis, u8 returnVector);
//Read Position
i32 flex_read_pos_rtn(u8 boardID, u8 axis, i32 *position);

//Read Velocity in RPM
//The data is returned in "IEEE f64" format and has to be converted appropriately.
i32 flex_read_rpm(u8 boardID, u8 axisOrVectorSpace,u8 returnVector);
//Read Velocity in RPM
//The data is returned in "IEEE f64" format and is converted in the DLL.
i32 flex_read_rpm_rtn(u8 boardID, u8 axisOrVectorSpace, f64 *RPM);

// Read Trajectory Status
i32 flex_read_trajectory_status(u8 boardID, u8 axisOrVectorSpace, u16 statusType, u8 returnVector);
// Read Trajectory Status
i32 flex_read_trajectory_status_rtn(u8 boardID, u8 axisOrVectorSpace, u16 statusType, u16 *status);

//Read Velocity
i32 flex_read_velocity(u8 boardID, u8 axisOrVectorSpace, u8 returnVector);
//Read Velocity
i32 flex_read_velocity_rtn(u8 boardID, u8 axisOrVectorSpace, i32 *velocity);

//Read Vector Space Position
i32 flex_read_vs_pos (u8 boardID, u8 vectorSpace, u8 returnVector);
//Read Vector Space Position
i32 flex_read_vs_pos_rtn (u8 boardID, u8 vectorSpace, i32 *xPosition, i32 *yPosition, i32 *zPosition);

//Reset Position
i32 flex_reset_pos (u8 boardID, u8 axis, i32 position1, i32 position2, u8 inputVector);

//Set Operation Mode
i32 flex_set_op_mode (u8 boardID, u8 axisOrVectorSpace, u16 operationMode);

// Check Move Complete Status
i32 flex_check_move_complete_status (u8 boardID, u8 axisOrVectorSpace,
                                          u16 axisOrVSMap, u16* moveComplete);

// Check Blend Complete Status
i32 flex_check_blend_complete_status (u8 boardID, u8 axisOrVectorSpace,
                                           u16 axisOrVSMap, u16* blendComplete);

// Wait for Move Complete
i32 flex_wait_for_move_complete (u8 boardID, u8 axisOrVectorSpace, u16 axisOrVSMap,
                                      u32 timeout, i32 pollInterval, u16* moveComplete);

// Wait for Blend Complete
i32 flex_wait_for_blend_complete (u8 boardID, u8 axisOrVectorSpace, u16 axisOrVSMap,
                                       u32 timeout, i32 pollInterval, u16* blendComplete);

//////// Arcs  Functions

//Load Circular Arc
i32 flex_load_circular_arc (u8 boardID, u8 vectorSpace, u32 radius, f64 startAngle,
                     f64 travelAngle, u8 inputVector);

//Load Helical Arc
i32 flex_load_helical_arc (u8 boardID, u8 vectorSpace, u32 radius, f64 startAngle,
               f64 travelAngle, i32 linearTravel, u8 inputVector);

//Load Spherical Arc
i32 flex_load_spherical_arc (u8 boardID, u8 vectorSpace, u32 radius, f64 planePitch,
            f64 planeYaw, f64 startAngle, f64 travelAngle, u8 inputVector);


//////// Gearing Functions

//Configure Gear Master
i32 flex_config_gear_master(u8 boardID, u8 axis, u8 masterAxisOrEncoderOrADC);

//Enable Gearing
i32 flex_enable_gearing(u8 boardID, u16 gearMap);

//Enable Gearing Single Axis
i32 flex_enable_gearing_single_axis (u8 boardID, u8 axis, u16 enable);

//Load Gear Ratio
i32 flex_load_gear_ratio(u8 boardID, u8 axis, u16 absoluteOrRelative,
                           i16 ratioNumerator, u16 ratioDenominator, u8 inputVector);


//////// Electronic Camming Functions

//Configure Camming Master
i32 flex_configure_camming_master(i32 boardID, i32 axisID, i32 masterResource, f64 masterCycle);

//Load Camming Parameter
i32 flex_load_camming_parameter(i32 boardID, i32 axisID, TnimcCammingParameter attribute, NIMC_DATA* data);

//Enable Camming (Multiaxes)
i32 flex_enable_camming(i32 boardID, u32 arraySize, NIMC_CAMMING_ENABLE_DATA* dataArray);

//Enable Camming Single Axis
i32 flex_enable_camming_single_axis(i32 boardID, i32 axisID, u16 enable, f64 position);

//////// Advanced Trajectory Functions

//Acquire Trajectory Data
i32 flex_acquire_trajectory_data(u8 boardID, u16 axisMap, u16 numberOfSamples, u16 timePeriod);

//Load Base Velocity
i32 flex_load_base_vel(u8 boardID, u8 axis, u16 baseVelocity, u8 inputVector);

//Load Blend Factor
i32 flex_load_blend_fact(u8 boardID, u8 axisOrVectorSpace, i16 blendFactor, u8 inputVector);

//Load Position Modulus
i32 flex_load_pos_modulus(u8 boardID, u8 axis, u32 positionModulus, u8 inputVector);

//Load Run/Stop Threshold
i32 flex_load_run_stop_threshold(u8 boardID, u8 axis, u16 runStopThreshold, u8 inputVector);

//Load Velocity Threshold
i32 flex_load_velocity_threshold(u8 boardID, u8 axis, u32 threshold, u8 inputVector);

//Load Velocity Threshold in RPM
i32 flex_load_rpm_thresh(u8 boardID, u8 axis, f64 threshold, u8 inputVector);

//Load S-Curve Time
i32 flex_load_scurve_time(u8 boardID, u8 axisOrVectorSpace, u16 sCurveTime, u8 inputVector);

//Load Torque Limit
i32 flex_load_torque_lim(u8 boardID, u8 axis, i16 primaryPositiveLimit, i16 primaryNegativeLimit,
                                     i16 secondaryPositiveLimit, i16 secondaryNegativeLimit, u8 inputVector);

//Load Torque Offset
i32 flex_load_torque_offset(u8 boardID, u8 axis, i16 primaryOffset, i16 secondaryOffset, u8 inputVector);

//Load Velocity Override
i32 flex_load_velocity_override(u8 boardID, u8 axisOrVectorSpace, f32 overridePercentage, u8 inputVector);

//Read DAC
i32 flex_read_dac(u8 boardID, u8 axisOrDAC, u8 returnVector);
//Read DAC
i32 flex_read_dac_rtn(u8 boardID, u8 axisOrDAC, i16 *DACValue);

//Read DAC Limit Status
i32 flex_read_dac_output_limit_status(u8 boardID, u8 returnVector);

//Read DAC Limit Status
i32 flex_read_dac_output_limit_status_rtn(u8 boardID, u16 *positiveStatus, u16 *negativeStatus);

//Read Steps Generated
i32 flex_read_steps_gen(u8 boardID, u8 axisOrStepperOutput, u8 returnVector);
//Read Steps Generated
i32 flex_read_steps_gen_rtn(u8 boardID, u8 axisOrStepperOutput, i32 *steps);

//Read Target Position
i32 flex_read_target_pos(u8 boardID, u8 axis, u8 returnVector);
//Read Target Position
i32 flex_read_target_pos_rtn(u8 boardID, u8 axis, i32 *targetPosition);

//Read Trajectory Data
i32 flex_read_trajectory_data(u8 boardID, u8 returnVector);
//Read Trajectory Data
//returnData points to an array of size <= 12 depending upon the
//number of axes data is being acquired for. For 6 axes returnData
//points to an array of size 12.
i32 flex_read_trajectory_data_rtn(u8 boardID, i32 *returnData);


///////////////////////////////////////////////////////////////////////////////////////
// Buffered Functions
///////////////////////////////////////////////////////////////////////////////////////

// Configure Buffer
i32 flex_configure_buffer (u8 boardID, u8 buffer, u8 resource, u16 bufferType, i32 bufferSize,
            u32 totalPoints, u16 oldDataStop, f64 requestedInterval, f64 *actualInterval);

// Write Buffer
i32 flex_write_buffer (u8 boardID, u8 buffer, u32 numberOfPoints,
            u16 regenerationMode, i32 *data, u8 inputVector);

// Read Buffer
i32 flex_read_buffer (u8 boardID, u8 buffer, u32 numberOfPoints, u8 returnVector);
i32 flex_read_buffer_rtn (u8 boardID, u8 buffer, u32 numberOfPoints, i32* data);

// Check Buffer
i32 flex_check_buffer (u8 boardID, u8 buffer, u8 returnVector);
i32 flex_check_buffer_rtn (u8 boardID, u8 buffer, u32 *backlog, u16 *bufferState,
            u32 *pointsDone);

// Clear Buffer
i32 flex_clear_buffer (u8 boardID, u8 buffer);

///////////////////////////////////////////////////////////////////////////////////////
// Start & Stop Motion Functions
///////////////////////////////////////////////////////////////////////////////////////

//Blend Motion
i32 flex_blend(u8 boardID, u8 axisOrVectorSpace, u16 axisOrVSMap);

//Start Motion
i32 flex_start(u8 boardID, u8 axisOrVectorSpace, u16 axisOrVSMap);

//Stop Motion
i32 flex_stop_motion(u8 boardID, u8 axisOrVectorSpace, u16 stopType, u16 axisOrVSMap);


///////////////////////////////////////////////////////////////////////////////////////
// Motion IO Functions
///////////////////////////////////////////////////////////////////////////////////////

//Enable Home Inputs
i32 flex_enable_home_inputs(u8 boardID, u16 homeMap);

//Enable Limits
i32 flex_enable_axis_limit(u8 boardID, u16 limitType, u16 forwardLimitMap, u16 reverseLimitMap);

//Load Software Limit Positions
i32 flex_load_sw_lim_pos(u8 boardID, u8 axis, i32 forwardLimit, i32 reverseLimit, u8 inputVector);

//Read Home Input Status
i32 flex_read_home_input_status(u8 boardID, u8 returnVector);

//Read Home Input Status
i32 flex_read_home_input_status_rtn(u8 boardID, u16 *homeStatus);

//Read Limit Status
i32 flex_read_axis_limit_status(u8 boardID, u16 limitType, u8 returnVector);

//Read Limit Status
i32 flex_read_axis_limit_status_rtn(u8 boardID, u16 limitType, u16 *forwardLimitStatus, u16 *reverseLimitStatus);

//Set Home Input Polarity
i32 flex_set_home_polarity(u8 boardID, u16 homePolarityMap);

//Set Inhibit MOMO
i32 flex_set_inhibit_output_momo(u8 boardID, u16 mustOn, u16 mustOff);

//Set Limit Input Polarity
i32 flex_set_limit_input_polarity(u8 boardID, u16 forwardPolarityMap, u16 reversePolarityMap);

//Configure Inhibit Output
i32 flex_config_inhibit_output(u8 boardID, u8 axis, u16 enableInhibit, u16 polarity, u16 driveMode);

//Configure Breakpoint
i32 flex_configure_breakpoint(u8 boardID, u8 axisOrEncoder, u16 enableMode, u16 actionOnBreakpoint, u16 operation);

//Configure Breakpoint Output
i32 flex_configure_breakpoint_output(u8 boardID, u8 axisOrEncoder, u16 polarity, u16 driveMode);

//Enable Breakpoint
i32 flex_enable_breakpoint(u8 boardID, u8 axisOrEncoder, u16 enable);

//Load Breakpoint Modulus
i32 flex_load_bp_modulus(u8 boardID, u8 axisOrEncoder, u32 breakpointModulus, u8 inputVector);

//Load Breakpoint Position
i32 flex_load_pos_bp(u8 boardID, u8 axisOrEncoder, i32 breakpointPosition, u8 inputVector);

//Read Breakpoint Status
i32 flex_read_breakpoint_status(u8 boardID, u8 axisOrEncoder, u16 breakpointType, u8 returnVector);
//Read Breakpoint Status
i32 flex_read_breakpoint_status_rtn(u8 boardID, u8 axisOrEncoder, u16 breakpointType, u16 *breakpointStatus);

//Set Breakpoint Output MOMO
i32 flex_set_breakpoint_output_momo(u8 boardID, u8 axisOrEncoder, u16 mustOn, u16 mustOff, u8 inputVector);

//Enable High-Speed Capture
i32 flex_enable_hs_capture(u8 boardID, u8 axisOrEncoder, u16 enable);

//Read Captured Position
i32 flex_read_cap_pos(u8 boardID, u8 axisOrEncoder, u8 returnVector);
//Read Captured Position
i32 flex_read_cap_pos_rtn(u8 boardID, u8 axisOrEncoder, i32 *capturedPosition);

//Read High-Speed Capture Status
i32 flex_read_hs_cap_status(u8 boardID, u8 axisOrEncoder, u8 returnVector);
//Read High-Speed Capture Status
i32 flex_read_hs_cap_status_rtn(u8 boardID, u8 axisOrEncoder, u16 *highSpeedCaptureStatus);

//Configure High-Speed Capture (replaces flex_set_hs_cap_pol)
i32 flex_configure_hs_capture(u8 boardID, u8 axisOrEncoder, u16 captureMode, u16 operation);

//Configure Drive Signal
i32 flex_configure_drive_signal(u8 boardID, u8 axis, u16 port, u16 pin, u16 mode, u16 polarity);

//Read Drive Signal Status
i32 flex_read_drive_signal_status(u8 boardID, u8 axis, u8 returnVector);
//Read Drive Signal Status
i32 flex_read_drive_signal_status_rtn(u8 boardID, u8 axis, u16 *driveStatus);

// Load Motion IO Parameter
i32 flex_load_motion_io_parameter(i32 boardID, i32 axisID, TnimcMotionIOParameter attribute, NIMC_DATA* data);

// Read Motion IO Execution Data
i32 flex_read_motion_io_execution_data(i32 boardID, i32 axisID, TnimcMotionIOExecution attribute, NIMC_DATA* data);

///////////////////////////////////////////////////////////////////////////////////////
// Find Home & Index Functions
///////////////////////////////////////////////////////////////////////////////////////

//Find Reference
i32 flex_find_reference(u8 boardID, u8 axisOrVectorSpace, u16 axisOrVSMap, u8 findType);

//Wait Reference
i32 flex_wait_reference(u8 boardID, u8 axisOrVectorSpace, u16 axisOrVSMap, u32 timeout,
                             u32 pollingInterval, u16 *found);

//Read Reference Status
i32 flex_read_reference_status(u8 boardID, u8 axisOrVectorSpace, u16 axisOrVSMap, u16 attribute, u8 returnVector);
//Read Reference Status
i32 flex_read_reference_status_rtn(u8 boardID, u8 axisOrVectorSpace, u16 axisOrVSMap, u16 attribute, u16 *value);

//Check Reference
i32 flex_check_reference(u8 boardID, u8 axisOrVectorSpace, u16 axisOrVSMap, u16 *found, u16 *finding);

//Load Reference Parameter
i32 flex_load_reference_parameter(u8 boardID, u8 axis, u8 findType, u16 attribute, f64 value);

//Get Reference Parameter
i32 flex_get_reference_parameter(u8 boardID, u8 axis, u8 findType, u16 attribute, f64* value);

//Find Home
i32 flex_find_home(u8 boardID, u8 axis, u16 directionMap);

//Find Index
i32 flex_find_index(u8 boardID, u8 axis, u16 direction, i16 offset);

///////////////////////////////////////////////////////////////////////////////////////
// Analog & Digital IO Functions
///////////////////////////////////////////////////////////////////////////////////////

// Configure PWM Output
i32 flex_configure_pwm_output (u8 boardID, u8 PWMOutput, u16 enable, u16 clock);

// Enable ADCs
i32 flex_enable_adcs (u8 boardID, u8 reserved, u16 ADCMap);

// Enable Encoders
i32 flex_enable_encoders (u8 boardID, u16 encoderMap);

// Load DAC
i32 flex_load_dac (u8 boardID, u8 DAC, i16 outputValue, u8 inputVector);

// Load PWM Duty Cycle
i32 flex_load_pwm_duty (u8 boardID, u8 PWMOutput, u16 dutycycle, u8 inputVector);

// Read Encoder Position
i32 flex_read_encoder (u8 boardID, u8 axisOrEncoder, u8 returnVector);
// Read Encoder Position
i32 flex_read_encoder_rtn (u8 boardID, u8 axisOrEncoder, i32 *encoderCounts);

// Read I/O Port
i32 flex_read_port (u8 boardID, u8 port, u8 returnVector);
// Read I/O Port
i32 flex_read_port_rtn (u8 boardID, u8 port, u16 *portData);

// Reset Encoder Position
i32 flex_reset_encoder (u8 boardID, u8 encoder, i32 position, u8 inputVector);

// Select Signal (RTSI)
i32 flex_select_signal (u8 boardID, u16 destination, u16 source);

// Read ADC value
i32 flex_read_adc16(u8 boardID, u8 ADC, u8 returnVector);

// Read ADC value
i32 flex_read_adc16_rtn(u8 boardID, u8 ADC, i32 *ADCValue);

// Set ADC Range
i32 flex_set_adc_range (u8 boardID, u8 ADC, u16 range);

// Configure Encoder Filter
i32 flex_configure_encoder_filter (u8 boardID, u8 axisOrEncoder, u16 frequency);

//Configure Encoder Polarity
i32 flex_configure_encoder_polarity(u8 boardID, u16 indexPolarity, u16 phaseAPolarity, u16 phaseBPolarity);

// Set I/O Port Direction
i32 flex_set_port_direction (u8 boardID, u8 port, u16 directionMap);

// Set I/O Port MOMO
i32 flex_set_port(u8 boardID, u8 port, u8 mustOn, u8 mustOff, u8 inputVector);

// Set I/O Port Polarity
i32 flex_set_port_pol (u8 boardID, u8 port, u16 portPolarityMap);

///////////////////////////////////////////////////////////////////////////////////////
// Error & Utility Functions
///////////////////////////////////////////////////////////////////////////////////////

// Get Error Description
i32 flex_get_error_description(u16 descriptionType, i32 errorCode, u16 commandID, u16 resourceID,
                                    i8 *charArray, u32 *sizeOfArray);

// Get Motion Board Information
i32 flex_get_motion_board_info (u8 boardID, u32 informationType, u32 *informationValue);

// Get Motion Board Information
i32 flex_get_motion_board_info_string(u8 boardID, u32 informationType, i8 *charArray, u32 *sizeOfArray);

// Get Motion Board Name
i32 flex_get_motion_board_name (u8 boardID, i8 *charArray, u32 *sizeOfArray);

//Read Error Message
i32 flex_read_error_msg_rtn(u8 boardID, u16 *commandID, u16 *resourceID, i32 *errorCode);

//Read Error Message Detail
i32 flex_read_error_msg_detail_rtn(u8 boardID, u16 *commandID, u16 *resourceID, i32 *errorCode, u16 *lineNumber, u16 *fileNumber);

// Get Last Error
i32 flex_get_last_error(u8 boardID, u16 *commandID, u16 *resourceID, i32 *errorCode);

// Get u32
i32 flex_getu32(u8 boardID, u8 resource, u16 attribute, u32 *value);

// Set u32
i32 flex_setu32(u8 boardID, u8 resource, u16 attribute, u32 value);

// Get f64
i32 flex_getf64(u8 boardID, u8 resource, u16 attribute, f64 *value);

// Set f64
i32 flex_setf64(u8 boardID, u8 resource, u16 attribute, f64 value);

///////////////////////////////////////////////////////////////////////////////////////
// On-board Programming Functions
///////////////////////////////////////////////////////////////////////////////////////

//Begin Program Storage
i32 flex_begin_store(u8 boardID, u8 program);

//End Program Storage
i32 flex_end_store(u8 boardID, u8 program);

//Insert Label
i32 flex_insert_program_label(u8 boardID, u16 labelNumber);

//Jump to Label on Condition
i32 flex_jump_on_event (u8 boardID, u8 resource, u16 condition, u16 mustOn,
                                       u16 mustOff, u16 matchType, u16 labelNumber);

//Load Program Delay
i32 flex_load_delay(u8 boardID, u32 delayTime);

//Pause/Resume Program
i32 flex_pause_prog(u8 boardID, u8 program);

//Read Program Status
i32 flex_read_program_status (u8 boardID, u8 program, u8 returnVector);
//Read Program Status
i32 flex_read_program_status_rtn (u8 boardID, u8 program, u16 *programStatus);

//Run Program
i32 flex_run_prog(u8 boardID, u8 program);

//Set User Status MOMO
i32 flex_set_status_momo(u8 boardID, u8 mustOn, u8 mustOff);

//Stop Program
i32 flex_stop_prog(u8 boardID, u8 program);

//Wait on Condition
i32 flex_wait_on_event (u8 boardID, u8 resource, u16 waitType, u16 condition, u16 mustOn,
                                u16 mustOff, u16 matchType, u16 timeOut, u8 returnVector);

//Load Program Time Slice
i32 flex_load_program_time_slice (u8 boardID, u8 program, u16 timeSlice, u8 inputVector);

//////// Object Management Functions

//Load Memory Object Description
//Maximum size of description that can be entered is 32 chars
i32 flex_load_description(u8 boardID, u8 object, i8 *description);

// Object Memory Management
i32 flex_object_mem_manage(u8 boardID, u8 object, u16 operation);

//Read Memory Object Description
//description points to an array of size 32
i32 flex_read_description_rtn(u8 boardID, u8 object, i8 * description);

//Read Object Registry
i32 flex_read_registry_rtn(u8 boardID, u8 index, REGISTRY *registryRecord);


//////// Data Operations Functions

//Add Variables
i32 flex_add_vars(u8 boardID, u8 variable1, u8 variable2, u8 returnVector);

//AND Variables
i32 flex_and_vars(u8 boardID, u8 variable1, u8 variable2, u8 returnVector);

//Divide Variables
i32 flex_div_vars(u8 boardID, u8 variable1, u8 variable2, u8 returnVector);

//Load Constant to Variable
i32 flex_load_var(u8 boardID, i32 value, u8 variable1);

//Logical Shift Variable
i32 flex_lshift_var(u8 boardID, u8 variable1, i8 logicalShift, u8 returnVector);

//Multiply Variables
i32 flex_mult_vars(u8 boardID, u8 variable1, u8 variable2, u8 returnVector);

//Invert Variable
i32 flex_not_var(u8 boardID, u8 variable1, u8 returnVector);

//OR Variables
i32 flex_or_vars(u8 boardID, u8 variable1, u8 variable2, u8 returnVector);

//Read Variable
i32 flex_read_var(u8 boardID, u8 variable1, u8 returnVector);
//Read Variable
i32 flex_read_var_rtn(u8 boardID, u8 variable1, i32 *value);

//Subtract Variables
i32 flex_sub_vars(u8 boardID, u8 variable1, u8 variable2, u8 returnVector);

//Exclusive OR Variables
i32 flex_xor_vars(u8 boardID, u8 variable1, u8 variable2, u8 returnVector);


///////////////////////////////////////////////////////////////////////////////////////
// Advanced Functions
///////////////////////////////////////////////////////////////////////////////////////

//Clear Power Up Status
i32 flex_clear_pu_status(u8 boardID);

//Communicate
i32 flex_communicate(u8 boardID, u8 mode, u8 wordCount, u8 *resource,
              u16 *command, u16 *data, u8 vector);

//Flush Return Data Buffer
i32 flex_flush_rdb(u8 boardID);

//Enable Auto Start
i32 flex_enable_auto_start(u8 boardID, u8 enableOrDisable, u8 programToExecute);

//Enable Shut Down
i32 flex_enable_shutdown(u8 boardID);

//Enable 1394 Watchdog
i32 flex_enable_1394_watchdog(u8 boardID, u16 enableOrDisable);

//Read Communication Status
i32 flex_read_csr_rtn(u8 boardID, u16 *csr);

//Reset Default Parameters
i32 flex_reset_defaults (u8 boardID);

//Save Default Parameters
i32 flex_save_defaults (u8 boardID);

//Set Interrupt Event Mask
i32 flex_set_irq_mask(u8 boardID, u16 mask);

//Read Board Temperature
i32 flex_read_board_temperature(u8 boardID, f64* temperature);

//Read Return Data Buffer
i32 flex_read_rdb(u8 boardID, u16 *number, u16 *wordCount, u8 *resource,
                    u16 *command, u16 *commandData);

