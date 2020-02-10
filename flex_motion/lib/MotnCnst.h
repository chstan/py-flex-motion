///////////////////////////////////////////////////////////////////////////////
//
//  Title     : MotnCnst.h
//  Project   : NI-Motion Constants
//  Author    : Motion Control
//  Platforms : All
//  Copyright : National Instruments 2005.  All Rights Reserved.
//  Purpose   : Global constants for NI-Motion driver.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ___motncnst_h___
#define ___motncnst_h___

#include "nimcBasicTypes.h"

///////////////////////////////////////////////////////////////////////////////
// TYPE CONSTANTS
//   These constant values are used across multiple API.
///////////////////////////////////////////////////////////////////////////////
// Boolean
#define  NIMC_FALSE              0
#define  NIMC_TRUE               1

// Polarity
#define NON_INVERTING            0
#define INVERTING                1
#define NIMC_ACTIVE_HIGH         0
#define NIMC_ACTIVE_LOW          1

// Output Drive Mode
#define NIMC_OPEN_COLLECTOR      0
#define NIMC_TOTEM_POLE          1

// Maximum Device per System
#define  NIMC_MAXBOARDS          32             // Obsolete NI-Motion 7.2

// Maximum String Length
#define NIMC_MAX_FILENAME_LEN    256            // Obsolete NI-Motion 7.2


///////////////////////////////////////////////////////////////////////////////
// MOTION BOARD INFORMATION
//   These constant values are used to represent various board information. Refer
//   to GetMotionBoardInfo API. 
///////////////////////////////////////////////////////////////////////////////
// Board Attributes
#define NIMC_BOARD_FAMILY                                1100     
#define NIMC_BOARD_TYPE                                  1120
#define NIMC_BUS_TYPE                                    1130
#define NIMC_NOT_APPLICABLE                              1500     
#define NIMC_NUM_AXES                                    1510
#define NIMC_NUM_LICENSED_AXES                           1520     
#define NIMC_BOOT_VERSION                                3010
#define NIMC_FIRMWARE_VERSION                            3020
#define NIMC_DSP_VERSION                                 3030
#define NIMC_FPGA_VERSION                                3040
#define NIMC_FPGA1_VERSION                               3040
#define NIMC_FPGA2_VERSION                               3050
#define NIMC_CONTROLLER_SERIAL_NUMBER                    2040
#define NIMC_CONTROLLER_LOCAL_OR_REMOTE                  2050
#define NIMC_DRIVER_VERSION                              2060
#define NIMC_NUMBER_OF_COORDINATE_SPACES                 2070
#define NIMC_LICENSE_TYPE                                2080
#define NIMC_CLOSED_LOOP_CAPABLE                         1150     // Obsolete NI-Motion 7.2
#define NIMC_FLEXMOTION_BOARD_CLASS                      2030     // Obsolete NI-Motion 7.2

// Board Type
#define PCI_7330                 63
#define PXI_7330                 64
#define PCI_7334                 32
#define PXI_7334                 25
#define PCI_7340                 61
#define PXI_7340                 62
#define PCI_7342                 37
#define PXI_7342                 36
#define PCI_7344                 28
#define PXI_7344                 27
#define PCI_7350                 34
#define PXI_7350                 35
#define PCI_7390                 71
#define SOFTMOTION               99
#define FW_7344                  31             // Obsolete NI-Motion 7.2
#define ENET_7344                41             // Obsolete NI-Motion 7.2
#define SER_7340                 51             // Obsolete NI-Motion 7.2

// Board Family
#define NIMC_NI_MOTION           0  
#define NIMC_FLEX_MOTION         0              // Obsolete NI-Motion 7.2
#define NIMC_VALUE_MOTION        1              // Obsolete NI-Motion 7.2

// Bus Type
#define NIMC_ISA_BUS             0              // Obsolete NI-Motion 7.2
#define NIMC_PCI_BUS             1
#define NIMC_PXI_BUS             2
#define NIMC_UNKNOWN_BUS         3
#define NIMC_1394_BUS            4
#define NIMC_ENET_BUS            5              // Obsolete NI-Motion 7.2 
#define NIMC_SERIAL_BUS          6              // Obsolete NI-Motion 7.2
#define NIMC_VIRTUAL_BUS         7
#define NIMC_CAN_BUS             8

// Number of Axes
#define TWO_AXIS                 2              // Obsolete NI-Motion 7.2
#define THREE_AXIS               3              // Obsolete NI-Motion 7.2
#define FOUR_AXIS                4              // Obsolete NI-Motion 7.2
#define SIX_AXIS                 6              // Obsolete NI-Motion 7.2
#define EIGHT_AXIS               8              // Obsolete NI-Motion 7.2

// Motor System Type
#define SERVO                    0              // Obsolete NI-Motion 7.2
#define STEPPER                  1              // Obsolete NI-Motion 7.2
#define SERVO_STEPPER            2              // Obsolete NI-Motion 7.2

// Stepper Mode Type
#define OPEN_LOOP                0              // Obsolete NI-Motion 7.2
#define CLOSED_LOOP              1              // Obsolete NI-Motion 7.2

// Operating System
#define NIMC_UNKNOWN_OS          0xFFFF         // Obsolete NI-Motion 7.2
#define NIMC_WIN95               0              // Obsolete NI-Motion 7.2
#define NIMC_WIN98               2              // Obsolete NI-Motion 7.2
#define NIMC_WINNT               3              // Obsolete NI-Motion 7.2
#define NIMC_WIN2000             4              // Obsolete NI-Motion 7.2
#define NIMC_PHARLAP             5              // Obsolete NI-Motion 7.2
#define NIMC_LINUX               6              // Obsolete NI-Motion 7.2

// License Type
#define NIMC_NOTACTIVATED        0
#define NIMC_NI_7300             4000
#define NIMC_CANOPEN             3000
#define NIMC_NI_7400             2000
#define NIMC_ORMEC               1000

// Board Class
#define NIMC_FLEX_7344           1              // Obsolete NI-Motion 7.2
#define NIMC_FLEX_7334           2              // Obsolete NI-Motion 7.2
#define NIMC_FLEX_7348           3              // Obsolete NI-Motion 7.2
#define NIMC_FLEX_7342           4              // Obsolete NI-Motion 7.2
#define NIMC_NI_MOTION_7344      1              // Obsolete NI-Motion 7.2
#define NIMC_NI_MOTION_7334      2              // Obsolete NI-Motion 7.2
#define NIMC_NI_MOTION_7350      3              // Obsolete NI-Motion 7.2
#define NIMC_NI_MOTION_7342      4              // Obsolete NI-Motion 7.2
#define NIMC_NI_MOTION_7340      5              // Obsolete NI-Motion 7.2
#define NIMC_NI_MOTION_7330      6              // Obsolete NI-Motion 7.2
#define NIMC_NI_MOTION_7390      7              // Obsolete NI-Motion 7.2

// Board Location
#define NIMC_LOCAL_CONTROLLER    0
#define NIMC_REMOTE_CONTROLLER   1


///////////////////////////////////////////////////////////////////////////////
// SOFTMOTION AXIS INFORMATION
//    These constant values are used to represent various NI SoftMotion axis
//    information. These values are reserved for internal used only
///////////////////////////////////////////////////////////////////////////////
// Axis Type
#define NIMC_NI_7451                                     2001
#define NIMC_NI_7431                                     2100
#define NIMC_ORMEC_SMS_DRIVE                             1001
#define NIMC_CANOPEN_COPLEY_ACCELNET_MODULE              3001
#define NIMC_CANOPEN_COPLEY_XENUS                        3002
#define NIMC_CANOPEN_COPLEY_ACCELNET_PANEL               3004
#define NIMC_CANOPEN_COPLEY_STEPNET_MODULE               3005
#define NIMC_CANOPEN_COPLEY_STEPNET_PANEL                3006
#define NIMC_CANOPEN_COPLEY_STEPNET_PANEL2               3007
#define NIMC_CANOPEN_COPLEY_STEPNET_MICRO_MODULE         3008
#define NIMC_CANOPEN_COPLEY_XENUS_RESOLVER               3009
#define NIMC_CANOPEN_COPLEY_XENUS2                       3010
#define NIMC_CANOPEN_COPLEY_ACCELNET_PANEL2              3011
#define NIMC_CANOPEN_COPLEY_XENUS3                       3012
#define NIMC_CANOPEN_COPLEY_XENUS_RESOLVER2              3013

// Axis Presence Status
#define NIMC_DEVICE_ADDED        0
#define NIMC_DEVICE_FOUND        1
#define NIMC_DEVICE_ALL          2


///////////////////////////////////////////////////////////////////////////////
// AXIS CONFIGURATION ATTRIBUTES
//    These constant values are used during axis configuration. These attributes
//    indicates the axis system setup and resources used for the axis. These
//    attributes are used by multiple APIs.
///////////////////////////////////////////////////////////////////////////////
// Control Loop Update Rate
#define NIMC_PID_RATE_62_5       0
#define NIMC_PID_RATE_125        1
#define NIMC_PID_RATE_188        2
#define NIMC_PID_RATE_250        3
#define NIMC_PID_RATE_313        4
#define NIMC_PID_RATE_375        5
#define NIMC_PID_RATE_438        6
#define NIMC_PID_RATE_500        7

// Axis Configuration Attributes
typedef enum
{
   TnimcAxisConfigurationParameterEnable = 0,
} TnimcAxisConfigurationParameter;


///////////////////////////////////////////////////////////////////////////////
// STEPPER ATTRIBUTES
//    These constant values are used to configure the stepper output.
///////////////////////////////////////////////////////////////////////////////
// Stepper Operation Mode
#define NIMC_OPEN_LOOP           0
#define NIMC_CLOSED_LOOP         1
#define NIMC_P_COMMAND           2

// Stepper Pulse Mode
#define NIMC_CLOCKWISE_COUNTERCLOCKWISE 0
#define NIMC_STEP_AND_DIRECTION         1

// Units Per Revolution Attributes
#define NIMC_COUNTS              0
#define NIMC_STEPS               1


///////////////////////////////////////////////////////////////////////////////
// SERVO TUNING ATTRIBUTES
//    This constant values are used to configure various tuning parameter on 
//    servo axes. Refer to LoadAdvancedControlParameter and LoadPIDParameters APIs.
///////////////////////////////////////////////////////////////////////////////
// Servo Operation Mode
#define NIMC_EXTERNAL_COMMUTATION                        0
#define NIMC_ONBOARD_COMMUTATION                         1

// PID Attributes
#define NIMC_KP                  0
#define NIMC_KI                  1
#define NIMC_IL                  2
#define NIMC_KD                  3
#define NIMC_TD                  4
#define NIMC_KV                  5
#define NIMC_AFF                 6
#define NIMC_VFF                 7

// Advanced Control Attribute Types
#define NIMC_STATIC_FRICTION_MODE                        0
#define NIMC_STATIC_FRICTION_MAX_DEADZONE                3
#define NIMC_STATIC_FRICTION_MIN_DEADZONE                4
#define NIMC_STATIC_FRICTION_ITERM_OFFSET_FWD            5
#define NIMC_STATIC_FRICTION_ITERM_OFFSET_REV            6
#define NIMC_PID_RATE_MULTIPLIER                         7
#define NIMC_NOTCH_FILTER_FREQUENCY                      8
#define NIMC_NOTCH_FILTER_BANDWIDTH                      9
#define NIMC_NOTCH_FILTER_ENABLE                         10
#define NIMC_LOWPASS_FILTER_CUTOFF_FREQUENCY             11
#define NIMC_LOWPASS_FILTER_ENABLE                       12
#define NIMC_SECONDARY_PID_MODE                          13
#define NIMC_CONTROL_LOOP_RATE                           14

// Secondary PID Mode
#define NIMC_PID_DISABLED                                0        
#define NIMC_PID_CHANGE_IN_FEEDBACK                      1        
#define NIMC_PID_ACCELERATING                            2        
#define NIMC_PID_MOVING                                  3        
#define NIMC_PID_MOVING_REVERSE                          4        

// PID Rate Multiplier
#define NIMC_MAX_PID_RATE_MULTIPLIER                     100

// Static Friction Mode
#define NIMC_STICTION_OFF        0
#define NIMC_STICTION_ZERO_DAC   1
#define NIMC_STICTION_KILL       2

// External Commutation Attributes
#define NIMC_COMM_INITIALIZATION_TYPE                    0
#define NIMC_COMM_FIND_ZERO_VOLT                         1
#define NIMC_COMM_FIND_ZERO_TIME                         2
#define NIMC_COMM_HALL_SENSOR_OFFSET                     3        // Obsolete NI-Motion 7.2
#define NIMC_COMM_DIRECT_SET_PHASE                       4
#define NIMC_COMM_ELECTRICAL_CYCLE_COUNTS                5
#define NIMC_COMM_HALL_SENSOR_TYPE                       6
#define NIMC_COMM_INITIALIZE_ON_BOOT                     7        // Obsolete NI-Motion 7.2
#define NIMC_COMM_MODE                                   8

// External Commutation Mode
#define NIMC_HALL_SENSOR         0
#define NIMC_SHAKE_AND_WAKE      1
#define NIMC_DIRECT_SET          2

// Hall Sensor Type
#define NIMC_HALL_SENSOR_TYPE_1  0
#define NIMC_HALL_SENSOR_TYPE_2  1

// PID Loop Rate Attribute
#define MAX_PIDLOOPRATE          16000          // Obsolete NI-Motion 7.2


///////////////////////////////////////////////////////////////////////////////
// TRAJECTORY ATTRIBUTES
//    These constant values are used to configure parameters for the motion 
//    control profile generation engine. These attributes are used by multiple APIs.
///////////////////////////////////////////////////////////////////////////////
// Acceleration Attributes
#define NIMC_BOTH                0
#define NIMC_ACCELERATION        1
#define NIMC_DECELERATION        2
#define NIMC_ACCEL NIMC_ACCELERATION                              // Obsolete NI-Motion 7.2
#define NIMC_DECEL NIMC_DECELERATION                              // Obsolete NI-Motion 7.2

// Operation Modes
#define NIMC_ABSOLUTE_POSITION   0
#define NIMC_RELATIVE_POSITION   1
#define NIMC_VELOCITY            2
#define NIMC_RELATIVE_TO_CAPTURE 3
#define NIMC_MODULUS_POSITION    4
#define NIMC_ABSOLUTE_CONTOURING 5
#define NIMC_RELATIVE_CONTOURING 6

// Stop Control Modes
#define NIMC_DECEL_STOP          0
#define NIMC_HALT_STOP           1
#define NIMC_KILL_STOP           2

// Trajectory Status Attributes
#define NIMC_RUN_STOP_STATUS                             0
#define NIMC_MOTOR_OFF_STATUS                            1
#define NIMC_VELOCITY_THRESHOLD_STATUS                   2
#define NIMC_MOVE_COMPLETE_STATUS                        3

// Axis Status Bitmap
#define NIMC_RUN_STOP_BIT                                0x0001         
#define NIMC_PROFILE_COMPLETE_BIT                        0x0002         
#define NIMC_AXIS_OFF_BIT                                0x0004         
#define NIMC_FOLLOWING_ERROR_BIT                         0x0008         
#define NIMC_LIMIT_SWITCH_BIT                            0x0010         
#define NIMC_HOME_SWITCH_BIT                             0x0020         
#define NIMC_SW_LIMIT_BIT                                0x0040         
#define NIMC_AXIS_COMM_WATCHDOG_BIT                      0x0080         
#define NIMC_VELOCITY_THRESHOLD_BIT                      0x0100         
#define NIMC_POS_BREAKPOINT_BIT                          0x0200         
#define NIMC_HOME_FOUND_BIT                              0x0400         
#define NIMC_INDEX_FOUND_BIT                             0x0800         
#define NIMC_HIGH_SPEED_CAPTURE_BIT                      0x1000         
#define NIMC_DIRECTION_BIT                               0x2000         
#define NIMC_BLEND_STATUS_BIT                            0x4000         
#define NIMC_MOVE_COMPLETE_BIT                           0x8000         

// Velocity Filter Parameters
#define MAX_VELOCITY_FILTER_CONSTANT                     1000     // Obsolete NI-Motion 7.2        
#define MAX_VELOCITY_UPDATE_INTERVAL                     2500     // Obsolete NI-Motion 7.2

// Move Constraint Attributes
typedef enum
{
   TnimcMoveConstraintVelocity = 0,
   TnimcMoveConstraintAcceleration,
   TnimcMoveConstraintDeceleration,
   TnimcMoveConstraintAccelerationJerk,
   TnimcMoveConstraintDecelerationJerk,
} TnimcMoveConstraint;


///////////////////////////////////////////////////////////////////////////////
// BUFFER OPERATION ATTRIBUTES
//    These constant values are used to configure onboard memory buffer for
//    various operations
///////////////////////////////////////////////////////////////////////////////
// Buffered Operations Mode
#define NIMC_GENERAL_PURPOSE_INPUT                       0
#define NIMC_GENERAL_PURPOSE_OUTPUT                      1
#define NIMC_POSITION_DATA                               2
#define NIMC_BREAKPOINT_DATA                             3
#define NIMC_HS_CAPTURE_READBACK                         4
#define NIMC_CAMMING_POSITION                            5

// Buffer Status
#define NIMC_BUFFER_NOT_EXIST       0
#define NIMC_BUFFER_READY           1
#define NIMC_BUFFER_ACTIVE          2
#define NIMC_BUFFER_DONE            3
#define NIMC_BUFFER_OLDDATASTOP     4

// Write Regeneration Mode
#define NIMC_REGENERATION_NO_CHANGE                      0
#define NIMC_REGENERATION_LAST_WRITE                     1

// Contouring Attributes
#define NIMC_MIN_CONTOURING_INTERVAL                     10       // Obsolete NI-Motion 7.2
#define NIMC_MAX_CONTOURING_INTERVAL                     90.5     // Obsolete NI-Motion 7.2
#define NIMC_MAX_SPLINE_POINTS                           181      // Obsolete NI-Motion 7.2


///////////////////////////////////////////////////////////////////////////////
// ONBOARD PROGRAMMING ATTRIBUTES
//    These constant values are used during onboard programming. The attributes 
//    related to program control and configuration. These attributes are used
//    multiple APIs. 
///////////////////////////////////////////////////////////////////////////////
// Object Memory Control
#define NIMC_OBJECT_SAVE         0
#define NIMC_OBJECT_DELETE       1
#define NIMC_OBJECT_FREE         2

// Object Type
#define NIMC_OBJECT_TYPE_PROGRAM 1
#define NIMC_OBJECT_TYPE_BUFFER  2

// Program Wait and Program Jump Conditions
#define NIMC_CONDITION_LESS_THAN                         0  
#define NIMC_CONDITION_EQUAL                             1  
#define NIMC_CONDITION_LESS_THAN_OR_EQUAL                2  
#define NIMC_CONDITION_GREATER_THAN                      3  
#define NIMC_CONDITION_NOT_EQUAL                         4  
#define NIMC_CONDITION_GREATER_THAN_OR_EQUAL             5  
#define NIMC_CONDITION_TRUE                              6  
#define NIMC_CONDITION_HOME_FOUND                        7  
#define NIMC_CONDITION_INDEX_FOUND                       8  
#define NIMC_CONDITION_HIGH_SPEED_CAPTURE                9  
#define NIMC_CONDITION_POSITION_BREAKPOINT               10 
#define NIMC_CONDITION_ANTICIPATION_TIME_BREAKPOINT      11 
#define NIMC_CONDITION_VELOCITY_THRESHOLD                12 
#define NIMC_CONDITION_MOVE_COMPLETE                     13 
#define NIMC_CONDITION_PROFILE_COMPLETE                  14 
#define NIMC_CONDITION_BLEND_COMPLETE                    15 
#define NIMC_CONDITION_MOTOR_OFF                         16 
#define NIMC_CONDITION_HOME_INPUT_ACTIVE                 17 
#define NIMC_CONDITION_LIMIT_INPUT_ACTIVE                18 
#define NIMC_CONDITION_SOFTWARE_LIMIT_ACTIVE             19 
#define NIMC_CONDITION_PROGRAM_COMPLETE                  20 
#define NIMC_CONDITION_IO_PORT_MATCH                     21 
#define NIMC_CONDITION_CENTER_FOUND                      22 
#define NIMC_CONDITION_FORWARD_LIMIT_FOUND               23 
#define NIMC_CONDITION_REVERSE_LIMIT_FOUND               24 

// Program Wait and Program Jump Condition Matching Type
#define NIMC_MATCH_ALL           0
#define NIMC_MATCH_ANY           1

// Program Wait Condition Chaining
#define NIMC_WAIT                0
#define NIMC_WAIT_OR             1

// Program Status
#define NIMC_PROGRAM_DONE        0
#define NIMC_PROGRAM_PLAYING     1
#define NIMC_PROGRAM_PAUSED      2
#define NIMC_PROGRAM_STORING     3

// Program Download Status
#define NIMC_PROGRAM_DLOAD_STOP                          0        // Obsolete NI-Motion 7.2
#define NIMC_PROGRAM_DLOAD_START                         1        // Obsolete NI-Motion 7.2
#define NIMC_PROGRAM_DLOAD_PROGRESS                      2        // Obsolete NI-Motion 7.2

// Program Timeslice
#define NIMC_MAX_TIMESLICE       20             // Obsolete NI-Motion 7.2


///////////////////////////////////////////////////////////////////////////////
// POSITION COMPARE AND INPUT CAPTURE ATTRIBUTES
//    These constant values are used to configure position compare and input
//    capture behavior. These attributes are used by multiple APIs, refer to
//    position compare (breakpoint) and input capture (high-speed capture)
//    related APIs.
///////////////////////////////////////////////////////////////////////////////
// Position Compare and Input Capture Buffer Operation
#define NIMC_OPERATION_SINGLE                            0
#define NIMC_OPERATION_BUFFERED                          1

// Position Compare Mode
#define NIMC_BREAKPOINT_OFF                              0
#define NIMC_ABSOLUTE_BREAKPOINT                         1
#define NIMC_RELATIVE_BREAKPOINT                         2
#define NIMC_MODULO_BREAKPOINT                           3
#define NIMC_PERIODIC_BREAKPOINT                         4

// Position Compare Output Action
#define NIMC_NO_CHANGE                                   0
#define NIMC_RESET_BREAKPOINT                            1
#define NIMC_SET_BREAKPOINT                              2
#define NIMC_TOGGLE_BREAKPOINT                           3
#define NIMC_PULSE_BREAKPOINT                            4

// Position Compare Type
#define NIMC_POSITION_BREAKPOINT                         0
#define NIMC_ANTICIPATION_TIME_BREAKPOINT                1        // Obsolete NI-Motion 7.2

// Input Capture Mode
#define NIMC_HS_NON_INVERTING_LEVEL                      0
#define NIMC_HS_INVERTING_LEVEL                          1
#define NIMC_HS_LOW_TO_HIGH_EDGE                         2
#define NIMC_HS_HIGH_TO_LOW_EDGE                         3
#define NIMC_HS_NON_INVERTING_DI                         4
#define NIMC_HS_INVERTING_DI                             5


///////////////////////////////////////////////////////////////////////////////
// ANALOG IO ATTRIBUTES
//    These constant values are used to configure the analog input and analog
//    output resources. 
///////////////////////////////////////////////////////////////////////////////
// ADC Input Range
#define NIMC_ADC_UNIPOLAR_5      0
#define NIMC_ADC_BIPOLAR_5       1
#define NIMC_ADC_UNIPOLAR_10     2
#define NIMC_ADC_BIPOLAR_10      3


///////////////////////////////////////////////////////////////////////////////
// MOTION IO ATTRIBUTES
//    These constant values are used to configure and control the motion IO
//    line such as limits and inhibits.
///////////////////////////////////////////////////////////////////////////////
// Limit Input Attribute
#define  NIMC_LIMIT_INPUTS       0
#define  NIMC_SOFTWARE_LIMITS    1

// Home Input Attribute
#define  NIMC_HOME_INPUTS        2              // Obsolete NI-Motion 7.2

// Input Capture Lines
#define NIMC_HSC_TRIGGER1        0
#define NIMC_HSC_TRIGGER2        1
#define NIMC_HSC_TRIGGER3        2
#define NIMC_HSC_TRIGGER4        3
#define NIMC_HSC_TRIGGER5        4
#define NIMC_HSC_TRIGGER6        5
#define NIMC_HSC_TRIGGER7        6
#define NIMC_HSC_TRIGGER8        7

// Encoder Filter Frequency
#define NIMC_ENCODER_FILTER_25_6MHz                      0
#define NIMC_ENCODER_FILTER_12_8MHz                      1
#define NIMC_ENCODER_FILTER_6_4MHz                       2
#define NIMC_ENCODER_FILTER_3_2MHz                       3
#define NIMC_ENCODER_FILTER_1_6MHz                       4
#define NIMC_ENCODER_FILTER_800KHz                       5
#define NIMC_ENCODER_FILTER_400KHz                       6
#define NIMC_ENCODER_FILTER_200KHz                       7
#define NIMC_ENCODER_FILTER_100KHz                       8
#define NIMC_ENCODER_FILTER_50KHz                        9
#define NIMC_ENCODER_FILTER_25KHz                        10
#define NIMC_ENCODER_FILTER_DISABLED                     11

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


///////////////////////////////////////////////////////////////////////////////
// DIGITAL IO ATTRIBUTES
//    These constant values are used to configure and control the digital IO
//    lines.
///////////////////////////////////////////////////////////////////////////////
// Digital IO Lines
#define NIMC_DIO_BIT0            0
#define NIMC_DIO_BIT1            1
#define NIMC_DIO_BIT2            2
#define NIMC_DIO_BIT3            3
#define NIMC_DIO_BIT4            4
#define NIMC_DIO_BIT5            5
#define NIMC_DIO_BIT6            6
#define NIMC_DIO_BIT7            7

// RTSI IO Lines
#define NIMC_RTSI_BIT0           0              // Obsolete NI-Motion 7.2
#define NIMC_RTSI_BIT1           1              // Obsolete NI-Motion 7.2
#define NIMC_RTSI_BIT2           2              // Obsolete NI-Motion 7.2
#define NIMC_RTSI_BIT3           3              // Obsolete NI-Motion 7.2
#define NIMC_RTSI_BIT4           4              // Obsolete NI-Motion 7.2
#define NIMC_RTSI_BIT5           5              // Obsolete NI-Motion 7.2
#define NIMC_RTSI_BIT6           6              // Obsolete NI-Motion 7.2
#define NIMC_RTSI_BIT7           7              // Obsolete NI-Motion 7.2

// IO Line Direction
#define NIMC_OUTPUT              0
#define NIMC_INPUT               1

// PWM Pulse Generation Mode
#define NIMC_PWM_FREQ_SCALE_512                          0
#define NIMC_PWM_FREQ_SCALE_1K                           1
#define NIMC_PWM_FREQ_SCALE_2K                           2
#define NIMC_PWM_FREQ_SCALE_4K                           3
#define NIMC_PWM_FREQ_SCALE_8K                           4
#define NIMC_PWM_FREQ_SCALE_16K                          5
#define NIMC_PWM_FREQ_SCALE_33K                          6
#define NIMC_PWM_FREQ_EXT_CLK_256                        7
#define NIMC_PWM_FREQ_SCALE_65K                          8
#define NIMC_PWM_FREQ_SCALE_131K                         9
#define NIMC_PWM_FREQ_SCALE_262K                         10
#define NIMC_PWM_FREQ_SCALE_524K                         11
#define NIMC_PWM_FREQ_SCALE_1048K                        12
#define NIMC_PWM_FREQ_SCALE_2097K                        13
#define NIMC_PWM_FREQ_SCALE_4194K                        14
#define NIMC_PWM_FREQ_EXT_CLK_33K                        15

///////////////////////////////////////////////////////////////////////////////
// IO ROUTING ATTRIBUTES
//    These constant values are used to configure IO lines routing. Refer to
//    SelectSignal API.
///////////////////////////////////////////////////////////////////////////////
// Source/Destination Lines
#define NIMC_RTSI0               0
#define NIMC_RTSI1               1
#define NIMC_RTSI2               2
#define NIMC_RTSI3               3
#define NIMC_RTSI4               4
#define NIMC_RTSI5               5
#define NIMC_RTSI6               6
#define NIMC_RTSI7               7

// Source Only Lines
#define NIMC_HS_CAPTURE1         8
#define NIMC_HS_CAPTURE2         9
#define NIMC_HS_CAPTURE3         10
#define NIMC_HS_CAPTURE4         11
#define NIMC_HS_CAPTURE5         12
#define NIMC_HS_CAPTURE6         13
#define NIMC_HS_CAPTURE7         14
#define NIMC_HS_CAPTURE8         15

// Destination Only Lines
#define NIMC_TRIGGER_INPUT       8
#define NIMC_BREAKPOINT1         9
#define NIMC_BREAKPOINT2         10
#define NIMC_BREAKPOINT3         11
#define NIMC_BREAKPOINT4         12
#define NIMC_BREAKPOINT5         15
#define NIMC_BREAKPOINT6         16
#define NIMC_BREAKPOINT7         49
#define NIMC_BREAKPOINT8         50
#define NIMC_PHASE_A1            17
#define NIMC_PHASE_A2            18
#define NIMC_PHASE_A3            19
#define NIMC_PHASE_A4            20
#define NIMC_PHASE_A5            21
#define NIMC_PHASE_A6            22
#define NIMC_PHASE_A7            51
#define NIMC_PHASE_A8            52
#define NIMC_PHASE_B1            23
#define NIMC_PHASE_B2            24
#define NIMC_PHASE_B3            25
#define NIMC_PHASE_B4            26
#define NIMC_PHASE_B5            27
#define NIMC_PHASE_B6            28
#define NIMC_PHASE_B7            53
#define NIMC_PHASE_B8            54
#define NIMC_INDEX1              29
#define NIMC_INDEX2              30
#define NIMC_INDEX3              31
#define NIMC_INDEX4              32
#define NIMC_INDEX5              33
#define NIMC_INDEX6              34
#define NIMC_INDEX7              55
#define NIMC_INDEX8              56
#define NIMC_RTSI_SOFTWARE_PORT  13
#define NIMC_DONT_DRIVE          14
#define NIMC_PXI_STAR_TRIGGER    60

// Drive Signal Attributes
#define NIMC_DRIVE_SIGNAL_CLEAR_ALL                      0
#define NIMC_IN_POSITION_MODE                            1
#define NIMC_DRIVE_FAULT_MODE                            2


///////////////////////////////////////////////////////////////////////////////
// FIND REFERENCE ATTRIBUTES
//    These constant values are used to configure find reference move. Refer to
//    ReferenceMove related APIs.
//////////////////////////////////////////////////////////////////////////////
// Reference Move Type
#define NIMC_FIND_HOME_REFERENCE                         0x00
#define NIMC_FIND_INDEX_REFERENCE                        0x01
#define NIMC_FIND_CENTER_REFERENCE                       0x02
#define NIMC_FIND_FORWARD_LIMIT_REFERENCE                0x03
#define NIMC_FIND_REVERSE_LIMIT_REFERENCE                0x04
#define NIMC_FIND_SEQUENCE_REFERENCE                     0x05
#define NIMC_MAX_FIND_TYPES                              5

// Reference Attributes
#define NIMC_INITIAL_SEARCH_DIRECTION                    0x0
#define NIMC_FINAL_APPROACH_DIRECTION                    0x1
#define NIMC_EDGE_TO_STOP_ON                             0x2
#define NIMC_SMART_ENABLE                                0x3
#define NIMC_ENABLE_RESET_POSITION                       0x4
#define NIMC_OFFSET_POSITION                             0x5
#define NIMC_PRIMARY_RESET_POSITION                      0x6
#define NIMC_SECONDARY_RESET_POSITION                    0x7
#define NIMC_APPROACH_VELOCITY_PERCENT                   0x8
#define NIMC_SEQUENCE_SEARCH_ORDER                       0x9
#define NIMC_ENABLE_SEARCH_DISTANCE                      0xA
#define NIMC_SEARCH_DISTANCE                             0xB
#define NIMC_PHASEA_REFERENCE_STATE                      0xC
#define NIMC_PHASEB_REFERENCE_STATE                      0xD

// Reference Status
#define NIMC_HOME_FOUND                                  0x0
#define NIMC_INDEX_FOUND                                 0x1
#define NIMC_CENTER_FOUND                                0x2
#define NIMC_FORWARD_LIMIT_FOUND                         0x3
#define NIMC_REVERSE_LIMIT_FOUND                         0x4
#define NIMC_REFERENCE_FOUND                             0x5
#define NIMC_CURRENT_SEQUENCE_PHASE                      0x6
#define NIMC_FINDING_REFERENCE                           0x7

// Reference Search Direction
#define NIMC_FORWARD_DIRECTION   0
#define NIMC_REVERSE_DIRECTION   1

// Reference Approach Direction
#define NIMC_INTO_LIMIT          0
#define NIMC_AWAY_FROM_LIMIT     1


///////////////////////////////////////////////////////////////////////////////
// GEARING AND CAMMING ATTRIBUTES
//    These constant values are used to configure gearing and camming 
//    operations.
///////////////////////////////////////////////////////////////////////////////
// Gearing Operation Mode
#define NIMC_ABSOLUTE_GEARING    0  
#define NIMC_RELATIVE_GEARING    1  

// Camming Attributes
typedef enum
{
   TnimcCammingParameterMasterCycle = 0,
   TnimcCammingParameterMasterOffset,
   TnimcCammingParameterSlaveOffset,
} TnimcCammingParameter;


///////////////////////////////////////////////////////////////////////////////
// ADVANCED AND UTILITY ATTRIBUTES
//    These constant values are used to configure advanced feature and control
//    utility attributes.
///////////////////////////////////////////////////////////////////////////////
// Board Communication Status
#define NIMC_READY_TO_RECEIVE                            0x01    
#define NIMC_DATA_IN_RDB                                 0x02     
#define NIMC_PACKET_ERROR                                0x10     
#define NIMC_E_STOP                                      0x10     
#define NIMC_POWER_UP_RESET                              0x20     
#define NIMC_MODAL_ERROR_MSG                             0x40     
#define NIMC_HARDWARE_FAIL                               0x80     

// Error Retrieval Mode
#define NIMC_ERROR_ONLY                                  0
#define NIMC_FUNCTION_NAME_ONLY                          1
#define NIMC_RESOURCE_NAME_ONLY                          2
#define NIMC_COMBINED_DESCRIPTION                        3

// Attributes To Write
#define NIMC_BP_WINDOW                                   0x0200      
#define NIMC_PULL_IN_WINDOW                              0x0400      
#define NIMC_PULL_IN_TRIES                               0x0401      
#define NIMC_STOP_TYPE_ON_SWITCH                         0x0403 
#define NIMC_STEP_DUTY_CYCLE                             0x0600      
#define NIMC_DECEL_STOP_JERK                             0x0500      
#define NIMC_HOST_THROTTLE                               0x04D2   // Obsolete NI-Motion 7.2
#define NIMC_RANGE_CHECK                                 0x1A85   // Obsolete NI-Motion 7.2
#define NIMC_BOUNCY_LIMIT_DETECTION                      0x0402   // Obsolete NI-Motion 7.2
#define NIMC_FE_HALT_COMPENSATION                        0x0900   // Obsolete NI-Motion 7.2
#define NIMC_HOST_LOOP_TIME                              0x0800   // Obsolete NI-Motion 7.2
#define NIMC_BP_PULSE_WIDTH                              0x0201   // Obsolete NI-Motion 7.2
#define NIMC_STICTION_MODE                               0x0406   // Obsolete NI-Motion 7.2
#define NIMC_STICTION_MAX_DEADBAND                       0x0407   // Obsolete NI-Motion 7.2  
#define NIMC_STICTION_MIN_DEADBAND                       0x0408   // Obsolete NI-Motion 7.2
#define NIMC_STICTION_ITERM_OFFSET_FWD                   0x0409   // Obsolete NI-Motion 7.2
#define NIMC_STICTION_ITERM_OFFSET_REV                   0x040A   // Obsolete NI-Motion 7.2

// Attributes To Read
#define NIMC_PROGRAM_AUTOSTART                           0x0300      
#define NIMC_GEARING_ENABLED_STATUS                      0x0301
#define NIMC_BUS_TIMEOUT                                 0x0100   // Obsolete NI-Motion 7.2

// Stepper Output Duty Cycle
#define NIMC_STEP_DUTY_CYCLE_25                          25
#define NIMC_STEP_DUTY_CYCLE_50                          50

// Pull In Window Attribute
#define NIMC_MAX_PULL_IN_WINDOW  32767          // Obsolete NI-Motion 7.2
#define NIMC_MAX_PULL_IN_TRIES   32767          // Obsolete NI-Motion 7.2

// Decel Stop Jerk Constants
#define  NIMC_MAX_DECEL_STOP_JERK                        1        // Obsolete NI-Motion 7.2

// Communication Attribute
#define NIMC_SEND_COMMAND        0              // Obsolete NI-Motion 7.2 
#define NIMC_SEND_AND_READ       1              // Obsolete NI-Motion 7.2
#define NIMC_READ_RDB            2              // Obsolete NI-Motion 7.2

// 1394 Communication Watchdog Parameter
#define NIMC_ENABLE_1394_WATCHDOG                        1        // Obsolete NI-Motion 7.2
#define NIMC_DISABLE_1394_WATCHDOG                       0        // Obsolete NI-Motion 7.2

// Performance Options Attributes
#define NIMC_EXTENDED_WATCHDOG                           1
#define NIMC_EXTENDED_ARC_INTERVAL                       2


///////////////////////////////////////////////////////////////////////////////
// RESOURCE IDS
//    These constant values are used to indicate valid resource ID to be used 
//    with NI-Motion APIs.
///////////////////////////////////////////////////////////////////////////////
// Multi-Resource Controls
#define NIMC_AXIS_CTRL                                   0x00
#define NIMC_VECTOR_SPACE_CTRL                           0x10
#define NIMC_ENCODER_CTRL                                0x20
#define NIMC_DAC_CTRL                                    0x30
#define NIMC_STEP_OUTPUT_CTRL                            0x40
#define NIMC_ADC_CTRL                                    0x50
#define NIMC_ALTERNATE_EX_CTRL                           0x60
#define NIMC_IO_PORT_CTRL                                0x00
#define NIMC_PWM_CTRL                                    0x00
#define NIMC_PROGRAM_CTRL                                0x00
#define NIMC_SECONDARY_ENCODER_CTRL                      0x70
#define NIMC_SECONDARY_DAC_CTRL                          0x80
#define NIMC_SECONDARY_ADC_CTRL                          0x90
#define NIMC_ALTERNATE_CTRL                              0xA0
#define NIMC_AXIS_EX_CTRL                                0xB0
#define NIMC_ENCODER_EX_CTRL                             0xC0
#define NIMC_DAC_EX_CTRL                                 0xD0
#define NIMC_STEP_OUTPUT_EX_CTRL                         0xE0
#define NIMC_ADC_EX_CTRL                                 0xF0

// Axis Resource IDs
#define NIMC_NOAXIS              0x00
#define NIMC_AXIS1               0x01
#define NIMC_AXIS2               0x02
#define NIMC_AXIS3               0x03
#define NIMC_AXIS4               0x04
#define NIMC_AXIS5               0x05
#define NIMC_AXIS6               0x06
#define NIMC_AXIS7               0x07
#define NIMC_AXIS8               0x08
#define NIMC_AXIS9               0x09
#define NIMC_AXIS10              0x0A
#define NIMC_AXIS11              0x0B
#define NIMC_AXIS12              0x0C
#define NIMC_AXIS13              0x0D
#define NIMC_AXIS14              0x0E
#define NIMC_AXIS15              0x0F
#define NIMC_AXIS16              0xB1
#define NIMC_AXIS17              0xB2
#define NIMC_AXIS18              0xB3
#define NIMC_AXIS19              0xB4
#define NIMC_AXIS20              0xB5
#define NIMC_AXIS21              0xB6
#define NIMC_AXIS22              0xB7
#define NIMC_AXIS23              0xB8
#define NIMC_AXIS24              0xB9
#define NIMC_AXIS25              0xBA
#define NIMC_AXIS26              0xBB
#define NIMC_AXIS27              0xBC
#define NIMC_AXIS28              0xBD
#define NIMC_AXIS29              0xBE
#define NIMC_AXIS30              0xBF

// Coordinate (Vector Space) Resource IDs
#define NIMC_VECTOR_SPACE1       0x11
#define NIMC_VECTOR_SPACE2       0x12
#define NIMC_VECTOR_SPACE3       0x13
#define NIMC_VECTOR_SPACE4       0x14
#define NIMC_VECTOR_SPACE5       0x15
#define NIMC_VECTOR_SPACE6       0x16
#define NIMC_VECTOR_SPACE7       0x17
#define NIMC_VECTOR_SPACE8       0x18
#define NIMC_VECTOR_SPACE9       0x19
#define NIMC_VECTOR_SPACE10      0x1A
#define NIMC_VECTOR_SPACE11      0x1B
#define NIMC_VECTOR_SPACE12      0x1C
#define NIMC_VECTOR_SPACE13      0x1D
#define NIMC_VECTOR_SPACE14      0x1E
#define NIMC_VECTOR_SPACE15      0x1F

// Encoder Resource IDs
#define NIMC_ENCODER1            0x21
#define NIMC_ENCODER2            0x22
#define NIMC_ENCODER3            0x23
#define NIMC_ENCODER4            0x24
#define NIMC_ENCODER5            0x25
#define NIMC_ENCODER6            0x26
#define NIMC_ENCODER7            0x27
#define NIMC_ENCODER8            0x28
#define NIMC_ENCODER9            0x29
#define NIMC_ENCODER10           0x2A
#define NIMC_ENCODER11           0x2B
#define NIMC_ENCODER12           0x2C
#define NIMC_ENCODER13           0x2D
#define NIMC_ENCODER14           0x2E
#define NIMC_ENCODER15           0x2F
#define NIMC_ENCODER16           0xC1
#define NIMC_ENCODER17           0xC2
#define NIMC_ENCODER18           0xC3
#define NIMC_ENCODER19           0xC4
#define NIMC_ENCODER20           0xC5
#define NIMC_ENCODER21           0xC6
#define NIMC_ENCODER22           0xC7
#define NIMC_ENCODER23           0xC8
#define NIMC_ENCODER24           0xC9
#define NIMC_ENCODER25           0xCA
#define NIMC_ENCODER26           0xCB
#define NIMC_ENCODER27           0xCC
#define NIMC_ENCODER28           0xCD
#define NIMC_ENCODER29           0xCE
#define NIMC_ENCODER30           0xCF

// Secondary Encoder Resource IDs
#define NIMC_SECONDARY_ENCODER1                                0x71
#define NIMC_SECONDARY_ENCODER2                                0x72
#define NIMC_SECONDARY_ENCODER3                                0x73
#define NIMC_SECONDARY_ENCODER4                                0x74
#define NIMC_SECONDARY_ENCODER5                                0x75
#define NIMC_SECONDARY_ENCODER6                                0x76
#define NIMC_SECONDARY_ENCODER7                                0x77
#define NIMC_SECONDARY_ENCODER8                                0x78
#define NIMC_SECONDARY_ENCODER9                                0x79
#define NIMC_SECONDARY_ENCODER10                               0x7A
#define NIMC_SECONDARY_ENCODER11                               0x7B
#define NIMC_SECONDARY_ENCODER12                               0x7C
#define NIMC_SECONDARY_ENCODER13                               0x7D
#define NIMC_SECONDARY_ENCODER14                               0x7E
#define NIMC_SECONDARY_ENCODER15                               0x7F

// DAC Output Resource IDs
#define NIMC_DAC1                0x31
#define NIMC_DAC2                0x32
#define NIMC_DAC3                0x33
#define NIMC_DAC4                0x34
#define NIMC_DAC5                0x35
#define NIMC_DAC6                0x36
#define NIMC_DAC7                0x37
#define NIMC_DAC8                0x38
#define NIMC_DAC9                0x39
#define NIMC_DAC10               0x3A
#define NIMC_DAC11               0x3B
#define NIMC_DAC12               0x3C
#define NIMC_DAC13               0x3D
#define NIMC_DAC14               0x3E
#define NIMC_DAC15               0x3F
#define NIMC_DAC16               0xD1
#define NIMC_DAC17               0xD2
#define NIMC_DAC18               0xD3
#define NIMC_DAC19               0xD4
#define NIMC_DAC20               0xD5
#define NIMC_DAC21               0xD6
#define NIMC_DAC22               0xD7
#define NIMC_DAC23               0xD8
#define NIMC_DAC24               0xD9
#define NIMC_DAC25               0xDA
#define NIMC_DAC26               0xDB
#define NIMC_DAC27               0xDC
#define NIMC_DAC28               0xDD
#define NIMC_DAC29               0xDE
#define NIMC_DAC30               0xDF

// Secondary DAC Output Resource IDs
#define NIMC_SECONDARY_DAC1      0x91
#define NIMC_SECONDARY_DAC2      0x92
#define NIMC_SECONDARY_DAC3      0x93
#define NIMC_SECONDARY_DAC4      0x94
#define NIMC_SECONDARY_DAC5      0x95
#define NIMC_SECONDARY_DAC6      0x96
#define NIMC_SECONDARY_DAC7      0x97
#define NIMC_SECONDARY_DAC8      0x98
#define NIMC_SECONDARY_DAC9      0x99
#define NIMC_SECONDARY_DAC10     0x9A
#define NIMC_SECONDARY_DAC11     0x9B
#define NIMC_SECONDARY_DAC12     0x9C
#define NIMC_SECONDARY_DAC13     0x9D
#define NIMC_SECONDARY_DAC14     0x9E
#define NIMC_SECONDARY_DAC15     0x9F

// Stepper Output Resource IDs
#define NIMC_STEP_OUTPUT1        0x41
#define NIMC_STEP_OUTPUT2        0x42
#define NIMC_STEP_OUTPUT3        0x43
#define NIMC_STEP_OUTPUT4        0x44
#define NIMC_STEP_OUTPUT5        0x45
#define NIMC_STEP_OUTPUT6        0x46
#define NIMC_STEP_OUTPUT7        0x47
#define NIMC_STEP_OUTPUT8        0x48
#define NIMC_STEP_OUTPUT9        0x49
#define NIMC_STEP_OUTPUT10       0x4A
#define NIMC_STEP_OUTPUT11       0x4B
#define NIMC_STEP_OUTPUT12       0x4C
#define NIMC_STEP_OUTPUT13       0x4D
#define NIMC_STEP_OUTPUT14       0x4E
#define NIMC_STEP_OUTPUT15       0x4F
#define NIMC_STEP_OUTPUT16       0xE1
#define NIMC_STEP_OUTPUT17       0xE2
#define NIMC_STEP_OUTPUT18       0xE3
#define NIMC_STEP_OUTPUT19       0xE4
#define NIMC_STEP_OUTPUT20       0xE5
#define NIMC_STEP_OUTPUT21       0xE6
#define NIMC_STEP_OUTPUT22       0xE7
#define NIMC_STEP_OUTPUT23       0xE8
#define NIMC_STEP_OUTPUT24       0xE9
#define NIMC_STEP_OUTPUT25       0xEA
#define NIMC_STEP_OUTPUT26       0xEB
#define NIMC_STEP_OUTPUT27       0xEC
#define NIMC_STEP_OUTPUT28       0xED
#define NIMC_STEP_OUTPUT29       0xEE
#define NIMC_STEP_OUTPUT30       0xEF

// ADC Input Resource IDs
#define NIMC_ADC1                0x51
#define NIMC_ADC2                0x52
#define NIMC_ADC3                0x53
#define NIMC_ADC4                0x54
#define NIMC_ADC5                0x55
#define NIMC_ADC6                0x56
#define NIMC_ADC7                0x57
#define NIMC_ADC8                0x58
#define NIMC_ADC9                0x59
#define NIMC_ADC10               0x5A
#define NIMC_ADC11               0x5B
#define NIMC_ADC12               0x5C
#define NIMC_ADC13               0x5D
#define NIMC_ADC14               0x5E
#define NIMC_ADC15               0x5F
#define NIMC_ADC16               0xF1
#define NIMC_ADC17               0xF2
#define NIMC_ADC18               0xF3
#define NIMC_ADC19               0xF4
#define NIMC_ADC20               0xF5
#define NIMC_ADC21               0xF6
#define NIMC_ADC22               0xF7
#define NIMC_ADC23               0xF8
#define NIMC_ADC24               0xF9
#define NIMC_ADC25               0xFA
#define NIMC_ADC26               0xFB
#define NIMC_ADC27               0xFC
#define NIMC_ADC28               0xFD
#define NIMC_ADC29               0xFE
#define NIMC_ADC30               0xFF

// Secondary ADC Input Resource IDs
#define NIMC_SECONDARY_ADC1      0x81
#define NIMC_SECONDARY_ADC2      0x82
#define NIMC_SECONDARY_ADC3      0x83
#define NIMC_SECONDARY_ADC4      0x84
#define NIMC_SECONDARY_ADC5      0x85
#define NIMC_SECONDARY_ADC6      0x86
#define NIMC_SECONDARY_ADC7      0x87
#define NIMC_SECONDARY_ADC8      0x88
#define NIMC_SECONDARY_ADC9      0x89
#define NIMC_SECONDARY_ADC10     0x8A
#define NIMC_SECONDARY_ADC11     0x8B
#define NIMC_SECONDARY_ADC12     0x8C
#define NIMC_SECONDARY_ADC13     0x8D
#define NIMC_SECONDARY_ADC14     0x8E
#define NIMC_SECONDARY_ADC15     0x8F

// Bidirectional Digital IO Port Resource IDs
#define NIMC_IO_PORT1            0x01
#define NIMC_IO_PORT2            0x02
#define NIMC_IO_PORT3            0x03
#define NIMC_IO_PORT4            0x04
#define NIMC_IO_PORT5            0x05
#define NIMC_IO_PORT6            0x06
#define NIMC_IO_PORT7            0x07
#define NIMC_IO_PORT8            0x08
#define NIMC_IO_PORT9            0x09
#define NIMC_IO_PORT10           0x0A
#define NIMC_IO_PORT11           0x0B
#define NIMC_IO_PORT12           0x0C
#define NIMC_IO_PORT13           0x0D
#define NIMC_IO_PORT14           0x0E
#define NIMC_IO_PORT15           0x0F
#define NIMC_RTSI_PORT           0x09
#define NIMC_HSC_PORT            0x0A
#define NIMC_IO_PORT16           0x11
#define NIMC_IO_PORT17           0x12
#define NIMC_IO_PORT18           0x13
#define NIMC_IO_PORT19           0x14
#define NIMC_IO_PORT20           0x15
#define NIMC_IO_PORT21           0x16
#define NIMC_IO_PORT22           0x17
#define NIMC_IO_PORT23           0x18
#define NIMC_IO_PORT24           0x19
#define NIMC_IO_PORT25           0x1A
#define NIMC_IO_PORT26           0x1B
#define NIMC_IO_PORT27           0x1C
#define NIMC_IO_PORT28           0x1D
#define NIMC_IO_PORT29           0x1E
#define NIMC_IO_PORT30           0x1F

// Digital Input Port Resource IDs
#define NIMC_DIGITAL_INPUT_PORT                          0
#define NIMC_DIGITAL_INPUT_PORT1                         (1  | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT2                         (2  | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT3                         (3  | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT4                         (4  | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT5                         (5  | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT6                         (6  | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT7                         (7  | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT8                         (8  | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT9                         (9  | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT10                        (10 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT11                        (11 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT12                        (12 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT13                        (13 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT14                        (14 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT15                        (15 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT16                        (16 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT17                        (17 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT18                        (18 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT19                        (19 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT20                        (20 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT21                        (21 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT22                        (22 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT23                        (23 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT24                        (24 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT25                        (25 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT26                        (26 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT27                        (27 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT28                        (28 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT29                        (29 | NIMC_DIGITAL_INPUT_PORT)
#define NIMC_DIGITAL_INPUT_PORT30                        (30 | NIMC_DIGITAL_INPUT_PORT)

// Digital Output Port Resource IDs
#define NIMC_DIGITAL_OUTPUT_PORT                         0x80
#define NIMC_DIGITAL_OUTPUT_PORT1                        (1  | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT2                        (2  | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT3                        (3  | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT4                        (4  | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT5                        (5  | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT6                        (6  | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT7                        (7  | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT8                        (8  | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT9                        (9  | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT10                       (10 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT11                       (11 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT12                       (12 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT13                       (13 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT14                       (14 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT15                       (15 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT16                       (16 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT17                       (17 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT18                       (18 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT19                       (19 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT20                       (20 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT21                       (21 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT22                       (22 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT23                       (23 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT24                       (24 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT25                       (25 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT26                       (26 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT27                       (27 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT28                       (28 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT29                       (29 | NIMC_DIGITAL_OUTPUT_PORT)
#define NIMC_DIGITAL_OUTPUT_PORT30                       (30 | NIMC_DIGITAL_OUTPUT_PORT)

// Secondary Input Capture Resource IDs
#define NIMC_SECOND_HS_CAPTURE                           0xA0
#define NIMC_SECOND_HS_CAPTURE_EX                        0x60
#define NIMC_SECOND_HS_CAPTURE1                          (0x01 | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE2                          (0x02 | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE3                          (0x03 | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE4                          (0x04 | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE5                          (0x05 | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE6                          (0x06 | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE7                          (0x07 | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE8                          (0x08 | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE9                          (0x09 | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE10                         (0x0A | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE11                         (0x0B | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE12                         (0x0C | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE13                         (0x0D | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE14                         (0x0E | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE15                         (0x0F | NIMC_SECOND_HS_CAPTURE)
#define NIMC_SECOND_HS_CAPTURE16                         (0x01 | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE17                         (0x02 | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE18                         (0x03 | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE19                         (0x04 | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE20                         (0x05 | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE21                         (0x06 | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE22                         (0x07 | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE23                         (0x08 | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE24                         (0x09 | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE25                         (0x0A | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE26                         (0x0B | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE27                         (0x0C | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE28                         (0x0D | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE29                         (0x0E | NIMC_SECOND_HS_CAPTURE_EX)
#define NIMC_SECOND_HS_CAPTURE30                         (0x0F | NIMC_SECOND_HS_CAPTURE_EX)

// Secondary Position Compare Resource IDs
#define NIMC_SECOND_BREAKPOINT                           0xA0
#define NIMC_SECOND_BREAKPOINT_EX                        0x60
#define NIMC_SECOND_BREAKPOINT1                          (0x01 | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT2                          (0x02 | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT3                          (0x03 | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT4                          (0x04 | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT5                          (0x05 | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT6                          (0x06 | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT7                          (0x07 | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT8                          (0x08 | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT9                          (0x09 | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT10                         (0x0A | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT11                         (0x0B | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT12                         (0x0C | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT13                         (0x0D | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT14                         (0x0E | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT15                         (0x0F | NIMC_SECOND_BREAKPOINT)
#define NIMC_SECOND_BREAKPOINT16                         (0x01 | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT17                         (0x02 | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT18                         (0x03 | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT19                         (0x04 | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT20                         (0x05 | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT21                         (0x06 | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT22                         (0x07 | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT23                         (0x08 | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT24                         (0x09 | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT25                         (0x0A | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT26                         (0x0B | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT27                         (0x0C | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT28                         (0x0D | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT29                         (0x0E | NIMC_SECOND_BREAKPOINT_EX)
#define NIMC_SECOND_BREAKPOINT30                         (0x0F | NIMC_SECOND_BREAKPOINT_EX)

// Secondary PID Control Resource IDs
#define NIMC_SECOND_PID1                                 (0x01 | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID2                                 (0x02 | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID3                                 (0x03 | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID4                                 (0x04 | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID5                                 (0x05 | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID6                                 (0x06 | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID7                                 (0x07 | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID8                                 (0x08 | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID9                                 (0x09 | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID10                                (0x0A | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID11                                (0x0B | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID12                                (0x0C | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID13                                (0x0D | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID14                                (0x0E | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID15                                (0x0F | NIMC_ALTERNATE_CTRL)
#define NIMC_SECOND_PID16                                (0x01 | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID17                                (0x02 | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID18                                (0x03 | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID19                                (0x04 | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID20                                (0x05 | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID21                                (0x06 | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID22                                (0x07 | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID23                                (0x08 | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID24                                (0x09 | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID25                                (0x0A | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID26                                (0x0B | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID27                                (0x0C | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID28                                (0x0D | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID29                                (0x0E | NIMC_ALTERNATE_EX_CTRL)
#define NIMC_SECOND_PID30                                (0x0F | NIMC_ALTERNATE_EX_CTRL)

// PWM Output Resource IDs
#define NIMC_PWM1                0x01
#define NIMC_PWM2                0x02

#endif // ___motncnst_h___
