Attribute VB_Name = "MOTNERR"
'=============================================================================================
' MotnErr.bas  -  Motion Control Error Codes
'
' Always use symbolic names and not explicit values when referring to
' specific error codes in your program.
'
' These error codes are also documented in Appendix A of the 
' Software Reference Manual and in the Function Reference Online Help
'
' Copyright 2000
' National Instruments Corporation.
' All rights reserved.
'
' Last Updated: 1/8/2007
'
'=============================================================================================

Global Const NIMC_noError                                                     = 0      ' No error.
Global Const NIMC_readyToReceiveTimeoutError                                  = -70001 ' Ready to Receive Timeout. The controller is still not ready to receive 
                                                                                       ' commands after the specified timeout period. This error may occur if the 
                                                                                       ' controller is busy processing previous commands. If this error persists, 
                                                                                       ' even when the controller should not be busy, contact National Instruments.
Global Const NIMC_currentPacketError                                          = -70002 ' Either this function is not supported by this type of controller, or the 
                                                                                       ' controller received an incomplete command packet and cannot execute the 
                                                                                       ' function.
Global Const NIMC_noReturnDataBufferError                                     = -70003 ' No data in the Return Data Buffer.  The kernel driver returns an error if 
                                                                                       ' it runs out of time waiting for the controller to return data to the 
                                                                                       ' Return Data Buffer. This error can also be returned if the power-up state 
                                                                                       ' of the controller has not been cleared.
Global Const NIMC_halfReturnDataBufferError                                   = -70004 ' Partial readback packet. The data returned by the controller is 
                                                                                       ' incomplete. The kernel driver timed out after getting partial data.
Global Const NIMC_boardFailureError                                           = -70005 ' Most likely, your controller is not installed or configured properly.  If 
                                                                                       ' this error persists when you know your controller is installed and 
                                                                                       ' configured properly, it indicates an internal hardware failure.
Global Const NIMC_badResourceIDOrAxisError                                    = -70006 ' An invalid axis number or other resource ID (vector space, encoder, I/O 
                                                                                       ' port, and so on) was used. If you are using the NI SoftMotion Controller, 
                                                                                       ' this error indicates that the resource used in the operation is not 
                                                                                       ' configured.
Global Const NIMC_CIPBitError                                                 = -70007 ' A previous function is currently being executed, so the controller cannot 
                                                                                       ' accept this function until the previous function has completed.  If this 
                                                                                       ' problem persists, try putting a delay between the offending commands.
Global Const NIMC_previousPacketError                                         = -70008 ' The function called previous to this one is not supported by this type of 
                                                                                       ' controller.
Global Const NIMC_packetErrBitNotClearedError                                 = -70009 ' Packet error bit not cleared by terminator (hardware error).
Global Const NIMC_badCommandError                                             = -70010 ' Command not recognized. Invalid command sent to the controller.
Global Const NIMC_badReturnDataBufferPacketError                              = -70011 ' Corrupt readback data. The data returned by the motion controller is 
                                                                                       ' corrupt.
Global Const NIMC_badBoardIDError                                             = -70012 ' Illegal board ID. You must use the board ID assigned to your controller in 
                                                                                       ' Measurement & Automation Explorer.
Global Const NIMC_packetLengthError                                           = -70013 ' Command packet length is incorrect.  Your NI motion controller firmware 
                                                                                       ' may be out of date.
Global Const NIMC_closedLoopOnlyError                                         = -70014 ' This command is valid only on closed-loop axes (closed-loop stepper and 
                                                                                       ' servo).
Global Const NIMC_returnDataBufferFlushError                                  = -70015 ' Unable to flush the Return Data Buffer.
Global Const NIMC_servoOnlyError                                              = -70016 ' This command is valid only on servo axes.
Global Const NIMC_stepperOnlyError                                            = -70017 ' This command is valid only on stepper axes.
Global Const NIMC_closedLoopStepperOnlyError                                  = -70018 ' This command is valid only on closed-loop stepper axes.
Global Const NIMC_noBoardConfigInfoError                                      = -70019 ' Controller configuration information is missing or corrupt.  A motion 
                                                                                       ' controller has not been configured at this Board ID.
Global Const NIMC_countsNotConfiguredError                                    = -70020 ' The steps per revolution and/or counts per revolution are not loaded for 
                                                                                       ' this axis.
Global Const NIMC_systemResetError                                            = -70021 ' System reset did not occur in maximum time allowed.
Global Const NIMC_functionSupportError                                        = -70022 ' This command is not supported by this controller or operating system.
Global Const NIMC_parameterValueError                                         = -70023 ' One of the parameters passed into the function has an illegal value.
Global Const NIMC_motionOnlyError                                             = -70024 ' Motion command sent to an Encoder board.
Global Const NIMC_returnDataBufferNotEmptyError                               = -70025 ' The Return Data Buffer is not empty. Commands that expect data returned 
                                                                                       ' from the controller cannot be sent until the Return Data Buffer is 
                                                                                       ' cleared.
Global Const NIMC_modalErrorsReadError                                        = -70026 ' The Motion Error Handler.flx VI discovered modal error(s) in the modal 
                                                                                       ' error stack.  These error(s) can be viewed in the Modal Error(s) Out 
                                                                                       ' Indicator/terminal of this VI.
Global Const NIMC_processTimeoutError                                         = -70027 ' Under Windows NT, a function call made to the motion controller timed out 
                                                                                       ' waiting for driver access. For ethernet controllers, a function call made 
                                                                                       ' to the motion controller timed out waiting for return data.  This can 
                                                                                       ' happen due to a slow or lost connection with the ethernet box product.
Global Const NIMC_insufficientSizeError                                       = -70028 ' The resource is not large enough to supported the specified operation.
Global Const NIMC_reserved4Error                                              = -70029 ' Reserved for obsolete product.
Global Const NIMC_reserved5Error                                              = -70030 ' Reserved for obsolete product.
Global Const NIMC_reserved6Error                                              = -70031 ' Reserved for obsolete product.
Global Const NIMC_reserved7Error                                              = -70032 ' Reserved for obsolete product.
Global Const NIMC_badPointerError                                             = -70033 ' A NULL pointer has been passed into a function inappropriately.  If the 
                                                                                       ' API function does not take a pointer, then this indicates an unexpected 
                                                                                       ' error has occurred internal to the driver.  Please contact National 
                                                                                       ' Instruments with the name of the function or VI that returned this error.
Global Const NIMC_wrongReturnDataError                                        = -70034 ' Incorrect data has been returned by the controller.  This data does not 
                                                                                       ' correspond to the expected data for the command sent to the controller.
Global Const NIMC_watchdogTimeoutError                                        = -70035 ' A fatal error has occurred on the controller. You must reset the 
                                                                                       ' controller by power cycling your computer. Contact National Instruments 
                                                                                       ' technical support if this problem persists.
Global Const NIMC_invalidRatioError                                           = -70036 ' A specified ratio is invalid.
Global Const NIMC_irrelevantAttributeError                                    = -70037 ' The specified attribute is not relevant.
Global Const NIMC_internalSoftwareError                                       = -70038 ' An unexpected error has occurred internal to the driver.  Please contact 
                                                                                       ' National Instruments with the name of the function or VI that returned 
                                                                                       ' this error.
Global Const NIMC_1394WatchdogEnableError                                     = -70039 ' The communication watchdog on the 1394 motherboard could not be started.
Global Const NIMC_reservedOnBoardProgramError                                 = -70040 ' The current device is reserved for onboard program while download process 
                                                                                       ' is in progress.  To release the device, use End Program Storage after the 
                                                                                       ' download process is completed.
Global Const NIMC_boardIDInUseError                                           = -70041 ' The boardID is already in use by another controller.  If trying to change 
                                                                                       ' a controllers boardID, select a boardID not in use.
Global Const NIMC_RemoteConnectionFailureError                                = -70042 ' Unable to establish a connection with the remote system.  Use MAX to 
                                                                                       ' verify that your remote board mappings are valid.
Global Const NIMC_calibrationOutOfRangeError                                  = -70043 ' The calibration target is out of range. We can't reach the calibration 
                                                                                       ' target even with full scale caldac or minimum caldac.
Global Const NIMC_calibrationStepError                                        = -70044 ' For external calibration users must do provide the password to the board, 
                                                                                       ' perform adjustment and set the new onboard reference in sequence.
Global Const NIMC_axesNotKilledError                                          = -70045 ' During the calibration process (both internal and external), please make 
                                                                                       ' sure that all axes are in a killed state using the Stop Motion function.
Global Const NIMC_invalidCalibrationDataError                                 = -70046 ' Calibration data were not successfully read from the flash EEPROM. 
Global Const NIMC_modeNotSupportedError                                       = -70047 ' This mode is not supported by this controller.
Global Const NIMC_invalidBreakpointWindowError                                = -70048 ' The breakpoint window is not valid.
Global Const NIMC_downloadChecksumError                                       = -70049 ' There was an error during checksum on a file being downloaded to the NI 
                                                                                       ' motion controller.
Global Const NIMC_reserved50Error                                             = -70050 ' Reserved.
Global Const NIMC_firmwareDownloadError                                       = -70051 ' Firmware download failed.  Reset the controller and try downloading again.
Global Const NIMC_FPGAProgramError                                            = -70052 ' Internal Error. The FPGA failed to program. Reset the controller and try 
                                                                                       ' again. If the problem persists, contact National Instruments technical 
                                                                                       ' support.
Global Const NIMC_DSPInitializationError                                      = -70053 ' Internal Error.  The DSP failed to initialize.  Reset the controller and 
                                                                                       ' try again. If the problem persists, contact National Instruments technical 
                                                                                       ' support.
Global Const NIMC_corrupt68331FirmwareError                                   = -70054 ' Corrupt onboard microprocessor firmware detected.  Download new firmware.
Global Const NIMC_corruptDSPFirmwareError                                     = -70055 ' Corrupt DSP firmware detected.  Download new DSP firmware.
Global Const NIMC_corruptFPGAFirmwareError                                    = -70056 ' Corrupt FPGA firmware detected.  Download new FPGA firmware.
Global Const NIMC_interruptConfigurationError                                 = -70057 ' Internal Error.  Host interrupt configuration failed and interrupt support 
                                                                                       ' is disabled.
Global Const NIMC_IOInitializationError                                       = -70058 ' Internal Error.  The I/O structure on the controller failed to initialize. 
                                                                                       '  Reset the controller and try again. If the problem persists, contact 
                                                                                       ' National Instruments technical support.
Global Const NIMC_flashromCopyError                                           = -70059 ' Error copying to the FLASH ROM.
Global Const NIMC_corruptObjectSectorError                                    = -70060 ' The objects stored in FLASH ROM are corrupt.
Global Const NIMC_bufferInUseError                                            = -70061 ' The specified buffer is in use.
Global Const NIMC_oldDataStopError                                            = -70062 ' The Read Buffer or Write Buffer function was unable to complete 
                                                                                       ' sucessfully because old data was encountered in the course of reading from 
                                                                                       ' or writing to the buffer.
Global Const NIMC_bufferConfigurationError                                    = -70063 ' The buffer has not been configured properly.  Buffer type cannot be 
                                                                                       ' changed.  The buffer might exist in ROM, but has not yet been configured 
                                                                                       ' since the last power cycle.
Global Const NIMC_illegalBufferOperation                                      = -70064 ' This operation is invalid at this time.  You cannot write to a ROM buffer; 
                                                                                       ' or you cannot read or write to the buffer at this time.
Global Const NIMC_illegalContouringError                                      = -70065 ' This operation cannot be executed while a contouring operation is in 
                                                                                       ' progress.
Global Const NIMC_virtualBoardError                                           = -70066 ' The board ID passed is currently being used for a virtual NI-Motion 
                                                                                       ' controller.  No commands can be sent to a virtual board.
Global Const NIMC_maxBreakpointFrequencyError                                 = -70067 ' A buffered breakpoint operation exceeded the maximum frequency allowed.
Global Const NIMC_maxHSCaptureFrequencyError                                  = -70068 ' A buffered high-speed capture operation exceeded the maximum frequency 
                                                                                       ' allowed.
Global Const NIMC_invalidHallSensorStateError                                 = -70069 ' Invalid Hall sensor state. Please verify the connection to the Hall 
                                                                                       ' sensors of ALL enabled onboard commutation axes.
Global Const NIMC_commutationModeError                                        = -70070 ' Commutation mode and commutation parameters cannot be changed while the 
                                                                                       ' axis is enabled.
Global Const NIMC_DIOReservedForHallSensors                                   = -70071 ' Direction of the DIO lines reserved for Hall sensor input cannot be set to 
                                                                                       ' output.
Global Const NIMC_boardInPowerUpResetStateError                               = -70072 ' The NI motion controller is in power up reset state.  Please execute the 
                                                                                       ' Clear Power Up Status function or VI before sending any commands to the 
                                                                                       ' controller.
Global Const NIMC_boardInShutDownStateError                                   = -70073 ' The controller cannot accept this function, as it has been shut down.
Global Const NIMC_shutDownFailedError                                         = -70074 ' The controller failed to shut down.  This could be because it failed to 
                                                                                       ' disable the DACs, the encoders, or the ADCs, or because it could not reset 
                                                                                       ' the I/O back to user defaults. 
Global Const NIMC_hostFIFOBufferFullError                                     = -70075 ' Communication FIFO buffer between the host computer and the controller is 
                                                                                       ' full.
Global Const NIMC_noHostDataError                                             = -70076 ' Communications error.  The controller did not receive any data in the 
                                                                                       ' command packet from the host computer.
Global Const NIMC_corruptHostDataError                                        = -70077 ' Communications error.  The controller received corrupt data in the packet 
                                                                                       ' from the host computer.
Global Const NIMC_invalidFunctionDataError                                    = -70078 ' Invalid function data has been passed.  This is usually a parameter out of 
                                                                                       ' range, or an illegal combination of parameter values.
Global Const NIMC_autoStartFailedError                                        = -70079 ' The controller could not run the onboard program on auto start.  When you 
                                                                                       ' enable auto start, make sure that you specify a valid program number and 
                                                                                       ' that the program is saved in FLASH ROM.
Global Const NIMC_returnDataBufferFullError                                   = -70080 ' The Return Data Buffer on the controller is full.
Global Const NIMC_reserved81Error                                             = -70081 ' Reserved - Never used for DSP messaging error.
Global Const NIMC_reserved82Error                                             = -70082 ' Reserved - Never used for position wrap-around error.
Global Const NIMC_DSPXmitBufferFullError                                      = -70083 ' Internal error. The transmit buffer of the DSP is full. Messages from DSP 
                                                                                       ' to the onboard microprocessor are being delayed or lost.
Global Const NIMC_DSPInvalidCommandError                                      = -70084 ' Internal error.  The DSP received an illegal command.
Global Const NIMC_DSPInvalidDeviceError                                       = -70085 ' Internal error.  The DSP received a command with an invalid Device ID.
Global Const NIMC_invalidFeedbackResetPositionError                           = -70086 ' Invalid reset position is applied to the feedback device. If the same 
                                                                                       ' feedback device is configured as both primary and secondary feedback, it 
                                                                                       ' must be reset to the same position.
Global Const NIMC_blendNotCompleteError                                       = -70087 ' Blend operation must be completed before another blend operation can be 
                                                                                       ' started. Use Check Blend Complete Status to read the current blend 
                                                                                       ' operation status.
Global Const NIMC_invalidFeedbackDeviceError                                  = -70088 ' The current operation cannot be performed on the axis feedback device. 
                                                                                       ' Configuring phase and index settings can be applied  only to axes with 
                                                                                       ' encoder feedback. If you are using the NI SoftMotion Controller, this 
                                                                                       ' error indicates the feedback device is not supported by this axis.
Global Const NIMC_axisFindingReferenceError                                   = -70089 ' The current operation cannot be completed because the axis is performing a 
                                                                                       ' find reference move. Changing find reference parameters and starting other 
                                                                                       ' moves requires the axis to finish the find reference move first.
Global Const NIMC_onboardProgramSupportError                                  = -70090 ' This function cannot be executed as part of an onboard program. Make sure 
                                                                                       ' a program is not currently storing or run End Program Storage to complete 
                                                                                       ' the current storing program before calling this function.
Global Const NIMC_availableForUse91                                           = -70091 ' None
Global Const NIMC_DSPXmitDataError                                            = -70092 ' Internal error.  The data returned by the DSP is incomplete or corrupt.
Global Const NIMC_DSPCommunicationsError                                      = -70093 ' Internal error. A command from the onboard microprocessor to the DSP was 
                                                                                       ' corrupt and ignored.
Global Const NIMC_DSPMessageBufferEmptyError                                  = -70094 ' Internal error.  This is an internal message to indicate that the there is 
                                                                                       ' no more data in the internal message buffer.
Global Const NIMC_DSPCommunicationsTimeoutError                               = -70095 ' Internal error.  There was an internal timeout while sending commands to 
                                                                                       ' the DSP.
Global Const NIMC_passwordError                                               = -70096 ' This function is password protected. Please enter the correct password to 
                                                                                       ' access it.
Global Const NIMC_mustOnMustOffConflictError                                  = -70097 ' There is a conflict between the mustOn and mustOff values set for this 
                                                                                       ' function.
Global Const NIMC_reserved98Error                                             = -70098 ' Was BAD_IO_DIR_ERROR
Global Const NIMC_reserved99Error                                             = -70099 ' Was OUTCOMP_ERROR - Counter/timer breakpoint is set for an invalid I/O bit.
Global Const NIMC_IOEventCounterError                                         = -70100 ' Problem with the I/O Event Counter.
Global Const NIMC_reserved101Error                                            = -70101 ' Was PAI_TM_ERROR - Problem with the I/O timer gate.
Global Const NIMC_wrongIODirectionError                                       = -70102 ' The I/O bit configuration does not agree with its port's direction setting.
Global Const NIMC_wrongIOConfigurationError                                   = -70103 ' I/O bit configuration is not possible for that pin.
Global Const NIMC_outOfEventsError                                            = -70104 ' Internal error.  The number of events pending have reached the maximum 
                                                                                       ' allowed.
Global Const NIMC_IOReservedError                                             = -70105 ' I/O has been reserved.  This error occurs when I/O has been reserved for 
                                                                                       ' another use such as Hall effect sensors or drive signals.  You must 
                                                                                       ' unreserve the I/O before using it.  This error might also be generated if 
                                                                                       ' you attempt to reserve a high-speed capture (HSC) line for a drive signal 
                                                                                       ' when HSC is enabled.
Global Const NIMC_outputDeviceNotAssignedError                                = -70106 ' No DAC or stepper output is assigned to this axis.
Global Const NIMC_splineUnderflowError                                        = -70107 ' The motion controller was too busy to send points to the spline engine and 
                                                                                       ' an underflow occurred.  Increase the Coarse Arc Points Interval Period in 
                                                                                       ' MAX if performing an arc move, or increase the requested interval between 
                                                                                       ' points in your buffer if performing a contour move.
Global Const NIMC_PIDUpdateRateError                                          = -70108 ' PID rate specified is too fast for the number of axes and/or encoders 
                                                                                       ' enabled.
Global Const NIMC_feedbackDeviceNotAssignedError                              = -70109 ' No primary feedback device (encoder or ADC) is assigned to a servo or 
                                                                                       ' closed-loop stepper axis.
Global Const NIMC_reserved110Error                                            = -70110 ' None - was duplicate - same as 106 -NIMC_outputDeviceNotAssignedError
Global Const NIMC_axisConfigurationSwitchError                                = -70111 ' A resource is currently configured for another axis that is enabled.  
                                                                                       ' Disable the axis using that resource. If you are using the NI SoftMotion 
                                                                                       ' Controller, you can only assign a resource to an axis with the same index.
Global Const NIMC_axisConfigurationClLoopError                                = -70112 ' An enabled closed-loop axis must have a resource mapped as its primary 
                                                                                       ' feedback.  Disable the axis before removing the resource.
Global Const NIMC_noMoreRAMError                                              = -70113 ' No RAM available for object (program or buffer) storage.
Global Const NIMC_reserved114Error                                            = -70114 ' Was NIMC_nestedProgramError.
Global Const NIMC_jumpToInvalidLabelError                                     = -70115 ' A Jump to Label on Condition function in a program had an invalid label.
Global Const NIMC_invalidConditionCodeError                                   = -70116 ' Condition selected is invalid.
Global Const NIMC_homeLimitNotEnabledError                                    = -70117 ' Find Reference function cannot execute because the Home and/or Limit 
                                                                                       ' inputs are not enabled.  Either enable Limits or use SmartEnable.
Global Const NIMC_findHomeError                                               = -70118 ' Find home was not successful because the motor stopped before the home 
                                                                                       ' switch was found.
Global Const NIMC_limitSwitchActiveError                                      = -70119 ' The desired move cannot be completed because the limit input is active in 
                                                                                       ' the direction of travel.
Global Const NIMC_softwareUpdateRequiredError                                 = -70120 ' This NI Motion controller is not compatible with this version of the 
                                                                                       ' NI-Motion Software.  Please upgrade your NI-Motion Software.
Global Const NIMC_positionRangeError                                          = -70121 ' Absolute target position loaded would cause the move length to be out of 
                                                                                       ' the +/-31 bit range allowed for a single move segment.
Global Const NIMC_encoderDisabledError                                        = -70122 ' The encoder is disabled.  The encoder must be enabled to read it.
Global Const NIMC_moduloBreakpointError                                       = -70123 ' The breakpoint value or the breakpoint window loaded exceeds the modulo 
                                                                                       ' range.
Global Const NIMC_findIndexError                                              = -70124 ' Find Index sequence did not find the index successfully.
Global Const NIMC_wrongModeError                                              = -70125 ' The function was not executed because it was attempted at an illegal time.
Global Const NIMC_axisConfigurationError                                      = -70126 ' An axis cannot change feedback while moving or change output while 
                                                                                       ' enabled.  Stop and/or disable the axis and then configure it.
Global Const NIMC_pointsTableFullError                                        = -70127 ' The points table for cubic splining is full.
Global Const NIMC_available128Error                                           = -70128 ' Was used for TABLE_ALLOC_ERROR but NIMC_noMoreRAMError should be used 
                                                                                       ' instead.
Global Const NIMC_axisDisabledError                                           = -70129 ' A disabled axis has been commanded to move.  Enable the axis before 
                                                                                       ' executing a move on it.
Global Const NIMC_memoryRangeError                                            = -70130 ' An invalid memory location is being addressed on the controller.
Global Const NIMC_inPositionUpdateError                                       = -70131 ' Internal error.  The axis position could not be read for in-position 
                                                                                       ' verification.
Global Const NIMC_targetPositionUpdateError                                   = -70132 ' Internal error.  The DSP was too busy to update the target position.
Global Const NIMC_pointRequestMissingError                                    = -70133 ' Internal error.  The internal points request buffer is missing a request.
Global Const NIMC_internalSamplesMissingError                                 = -70134 ' Internal error.  The internal samples buffer is missing samples.
Global Const NIMC_reserved135Error                                            = -70135 ' Was NIMC_axisEnabledError
Global Const NIMC_eventTimeoutError                                           = -70136 ' A wait operation timed out or a read function timed out.
Global Const NIMC_objectReferenceError                                        = -70137 ' An attempt was made to reference a nonexistent program object or buffer 
                                                                                       ' object.  Or, the object number is already in use by an object of a 
                                                                                       ' different type.  Choose a different object number, or free/delete the 
                                                                                       ' object currently owning that object number.
Global Const NIMC_outOfMemoryError                                            = -70138 ' There is not enough FLASH ROM space to save this object, which is either a 
                                                                                       ' program or buffer. If you are using the NI SoftMotion Controller, this 
                                                                                       ' error indicates that the targeted device does not have enough memory to 
                                                                                       ' perform the requested operation.
Global Const NIMC_registryFullError                                           = -70139 ' Object registry is full.  The number of programs and buffers has reached 
                                                                                       ' the limit.  Free some programs or buffers from RAM or ROM using the Object 
                                                                                       ' Memory Management function.
Global Const NIMC_noMoreProgramPlayerError                                    = -70140 ' All program players (maximum 10) are in use storing/playing programs.
Global Const NIMC_programOverruleError                                        = -70141 ' A Start, Blend, Find Home, or Find Index function being executed from an 
                                                                                       ' onboard program has been overruled by a Stop Motion function from the host 
                                                                                       ' computer. The program is left in the PAUSED state.  Execute the 
                                                                                       ' Pause/Resume Program function to continue.
Global Const NIMC_followingErrorOverruleError                                 = -70142 ' A Start, Blend, Find Home, or Find Index function being executed from an 
                                                                                       ' onboard program has been overruled due to a following error condition.  
                                                                                       ' The program is left in the PAUSED state.  Execute the Pause/Resume Program 
                                                                                       ' function to continue.
Global Const NIMC_reserved143Error                                            = -70143 ' None - Was used for I/O interrupt stuck on condition.
Global Const NIMC_illegalVariableError                                        = -70144 ' An illegal general-purpose variable is being used.
Global Const NIMC_illegalVectorSpaceError                                     = -70145 ' The vector space being used does not have enough axes assigned to it. If 
                                                                                       ' you are using the NI SoftMotion Controller, this error indicates that the 
                                                                                       ' vector space has not been configured.
Global Const NIMC_noMoreSamplesError                                          = -70146 ' There are no samples to read.  Execute Acquire Trajectory Data before 
                                                                                       ' trying to read samples.
Global Const NIMC_slaveAxisKilledError                                        = -70147 ' Gearing cannot be enabled because the slave axis is in a killed state.  
                                                                                       ' Issue a halt stop with the Stop Motion function on the slave axis to 
                                                                                       ' energize it.
Global Const NIMC_ADCDisabledError                                            = -70148 ' The ADC is disabled.  The ADC channel must be enabled to read it.
Global Const NIMC_operationModeError                                          = -70149 ' Axes that are a part of a vector space are either in velocity mode or have 
                                                                                       ' different operation modes.
Global Const NIMC_followingErrorOnFindHomeError                               = -70150 ' Find Home sequence did not find home successfully because the axis tripped 
                                                                                       ' on following error.
Global Const NIMC_invalidVelocityError                                        = -70151 ' The vector velocity is not valid.  The resulting angular velocity is out 
                                                                                       ' of range.  Change the vector velocity for the arc move.
Global Const NIMC_invalidAccelerationError                                    = -70152 ' The vector acceleration is not valid.  The resulting angular acceleration 
                                                                                       ' is out of range.  Change the vector acceleration for the arc move.
Global Const NIMC_samplesBufferFullError                                      = -70153 ' Internal error.  The internal samples buffer is full. 
Global Const NIMC_illegalVectorError                                          = -70154 ' The input or return vector being used is invalid.
Global Const NIMC_QSPIFailedError                                             = -70155 ' Internal error.  The internal QSPI serial bus failed and ADC values cannot 
                                                                                       ' be read.
Global Const NIMC_reserved156Error                                            = -70156 ' None - Duplicate - same as Error 27
Global Const NIMC_pointsBufferFullError                                       = -70157 ' Internal error.  The internal point request buffer is full. 
Global Const NIMC_axisInitializationError                                     = -70158 ' Internal Error.  The internal axis data structures failed to initialize.  
                                                                                       ' Reset the controller and try again. If the problem persists, contact 
                                                                                       ' National Instruments technical support.
Global Const NIMC_encoderInitializationError                                  = -70159 ' Internal Error.  The internal encoder data structures failed to 
                                                                                       ' initialize.  Reset the controller and try again. If the problem persists, 
                                                                                       ' contact National Instruments technical support.
Global Const NIMC_stepChannelInitializationError                              = -70160 ' Internal Error.  The internal stepper output data structures failed to 
                                                                                       ' initialize.  Reset the controller and try again. If the problem persists, 
                                                                                       ' contact National Instruments technical support.
Global Const NIMC_blendFactorConflictError                                    = -70161 ' Axes, which are part of a vector space, have different blend factors.  
                                                                                       ' Make sure that all the axes in the vector space have the same blend 
                                                                                       ' factor.
Global Const NIMC_torqueOffsetError                                           = -70162 ' The torque offset is outside of the torque limit range.
Global Const NIMC_invalidLimitRangeError                                      = -70163 ' The negative (lower) limit is greater than or equal to the positive 
                                                                                       ' (upper) limit.
Global Const NIMC_ADCConfigurationError                                       = -70164 ' ADCs cannot be enabled or disabled while axes are enabled with analog 
                                                                                       ' feedback.  Also, ADC ranges cannot be changed on ADC channels being used 
                                                                                       ' for analog feedback while axes are enabled.
Global Const NIMC_findReferenceError                                          = -70165 ' Find Reference was not successful because the motor stopped before the 
                                                                                       ' reference was found.
Global Const NIMC_followingErrorOnFindReference                               = -70166 ' Find Reference sequence did not find reference successfully because the 
                                                                                       ' axis tripped on following error.
Global Const NIMC_initializationInProgress                                    = -70167 ' The controller is currently initializing.
Global Const NIMC_invalidMotionIDError                                        = -70168 ' An invalid Motion ID was passed in a function.
Global Const NIMC_invalidPointerError                                         = -70169 ' A NULL or invalid pointer was passed as a parameter to this function.
Global Const NIMC_interfaceNotSupportedError                                  = -70170 ' The interface you are requesting is not supported.
Global Const NIMC_breakpointBufferFullError                                   = -70171 ' Internal Error.  The internal breakpoint buffer is full. 
Global Const NIMC_hsCaptureBufferFullError                                    = -70172 ' Internal Error.  The internal High-Speed Capture buffer is full. 
Global Const NIMC_internalBreakpointMissingError                              = -70173 ' Internal error.  The internal breakpoint buffer is missing data.
Global Const NIMC_internalHSCaptureMissingError                               = -70174 ' Internal error.  The internal high-speed capture buffer is missing data.
Global Const NIMC_arcPointsBufferFullError                                    = -70175 ' Internal Error.  The internal arc points buffer is full. 
Global Const NIMC_timeGuaranteeError                                          = -70176 ' The controller could not guarantee that all onboard programs are running 
                                                                                       ' in a timely fashion.
Global Const NIMC_invalidTimeSliceError                                       = -70177 ' You have entered an invalid value for the onboard program time slice.
Global Const NIMC_onlyInAProgramError                                         = -70178 ' This function can only be executed from an Onboard Program.
Global Const NIMC_invalidMasterAxisError                                      = -70179 ' The gearing master for the specified axis is invalid.
Global Const NIMC_invalidMasterEnabledError                                   = -70180 ' The gear master for an axis may not be set to 'None' while gearing is 
                                                                                       ' enabled for the axis.
Global Const NIMC_remoteBoardMismatchError                                    = -70181 ' The serial number of the motion controller with the given board ID does 
                                                                                       ' not match what is expected.
Global Const NIMC_deviceNotActivatedError                                     = -70182 ' The device or resource has not been activated, or the license file is 
                                                                                       ' invalid.
Global Const NIMC_dataTransmissionError                                       = -70183 ' Data transmission to the targeted device failed. If you are using the NI 
                                                                                       ' SoftMotion Controller, this error indicates that the device in use is not 
                                                                                       ' supported by the NI SoftMotion Controller.
Global Const NIMC_deviceTypeNotSupported                                      = -70184 ' The device in use is not supported by the NI SoftMotion Controller.
Global Const NIMC_serializationFailedError                                    = -70185 ' Failed to store user defaults for the target device.
Global Const NIMC_homeSwitchActiveError                                       = -70186 ' The find reference move was not successful because the home switch is 
                                                                                       ' active.
Global Const NIMC_incorrectBufferSizeError                                    = -70187 ' The specified buffer size is incorrect for the requested operation.
Global Const NIMC_invalidBufferHandleSpecifiedError                           = -70188 ' Internal error. The specified buffer handle is invalid.
Global Const NIMC_blendNotAllowedInThisModeError                              = -70189 ' Blending is not valid in this position mode.
Global Const NIMC_invalidLoopRateError                                        = -70190 ' The specified host loop rate is invalid for the device or the current 
                                                                                       ' control loop rate.
Global Const NIMC_axisCommunicationWatchdogError                              = -70191 ' Communication with the device has timed out.
Global Const NIMC_communicationInterfaceNotFoundError                         = -70192 ' Some of the NI SoftMotion Controller components are missing, incompatible, 
                                                                                       ' or not properly started.  Make sure your interface card is properly 
                                                                                       ' installed, and the device drivers are running.  Otherwise, try 
                                                                                       ' reinstalling or repairing the NI SoftMotion Controller.
Global Const NIMC_axisAlreadyAddedError                                       = -70194 ' The NI SoftMotion Controller axis has already been added to the controller.
Global Const NIMC_invalidAxisScaleError                                       = -70195 ' The axis scale is not correct for loading the move constraint in this 
                                                                                       ' unit. Use the Load Counts/Steps per Revolution function to load the 
                                                                                       ' correct axis scale.
Global Const NIMC_axisNotPresentError                                         = -70196 ' The selected axis does not exist in the NI SoftMotion Controller.
Global Const NIMC_startBlockedDueToFollowingError                             = -70197 ' Start motion is blocked because of following error. Verify the axis 
                                                                                       ' feedback and control, and then execute a halt stop before starting the 
                                                                                       ' move.
Global Const NIMC_configurationFileNotFoundError                              = -70198 ' The configuration file for the selected NI SoftMotion Controller is 
                                                                                       ' missing.
Global Const NIMC_gearingMasterNotActiveError                                 = -70199 ' Gearing cannot be enabled because the master is not active.
Global Const NIMC_controllerNotInPowerUpResetStateError                       = -70200 ' This operation requires the controller to be in a power up reset state. 
                                                                                       ' Please reset the controller.
Global Const NIMC_vectorSpaceCannotBeConfiguredError                          = -70201 ' The vector space cannot be configured because one of the axes in the 
                                                                                       ' vector space is moving.
Global Const NIMC_failedToAddDeviceError                                      = -70202 ' Failed to add the drive to the controller. Please verify the drive ID is 
                                                                                       ' configured properly.
Global Const NIMC_noMoreBufferError                                           = -70203 ' Failed to create a buffer on the controller. Please clear one or more 
                                                                                       ' buffers.
Global Const NIMC_IOResourceNotConfiguredError                                = -70204 ' You cannot enable the specified feature because the I/O resource necessary 
                                                                                       ' for the feature is not yet configured or mapped.
Global Const NIMC_invalidSerialNumberError                                    = -70205 ' The controller does not have a valid serial number. Please contact 
                                                                                       ' National Instruments to resolve this problem.
Global Const NIMC_invalidLicensingInformationError                            = -70206 ' Unable to verify licensing information from this controller. Power cycle 
                                                                                       ' the system and contact National Instruments if the problem persists.
Global Const NIMC_inhibitInputActiveError                                     = -70207 ' The move cannot be completed because the inhibit input of the commanded 
                                                                                       ' axis is active.
Global Const NIMC_IOResourceConfigurationError                                = -70208 ' You cannot change the configuration or mapping for this I/O resource 
                                                                                       ' because an enabled feature is currently using it. Please disable the 
                                                                                       ' feature before reconfiguring the I/O resource.
Global Const NIMC_driveReadyNotActiveError                                    = -70209 ' The move cannot be started because the drive ready input of the commanded 
                                                                                       ' axis is not active.
Global Const NIMC_clientConnectionRefusedError                                = -70210 ' An unexpected error has occurred internal to the driver.  The driver has 
                                                                                       ' refused to accept the client connection.  Please contact National 
                                                                                       ' Instruments technical support with the name of the function or VI that 
                                                                                       ' returned this error.
Global Const NIMC_dataOutOfRangeError                                         = -70211 ' The data parameter passed to the function does not contain a valid value.  
                                                                                       ' Check the VI or function reference help for valid values.
Global Const NIMC_invalidIndexError                                           = -70212 ' The index parameter passed to the function is invalid.  Check the VI or 
                                                                                       ' function reference help for valid values.
Global Const NIMC_controllerInDisabledStateError                              = -70213 ' The controller cannot perform the requested operation because it is 
                                                                                       ' currently disabled. Please reenable the controller in MAX.
Global Const NIMC_attributeIsReadOnlyError                                    = -70214 ' The selected attribute is read-only.
Global Const NIMC_invalidSettingsNameSpecifiedError                           = -70215 ' An invalid initialization settings name was specified.


Global Const  NIMC_badDeviceOrAxisError =  NIMC_badResourceIDOrAxisError
