#ifndef ___MotnErr_h___
#define ___MotnErr_h___
#define  NIMCDM_unrecoverableStallDetected                  -77107 // NI SoftMotion detected an unrecoverable stall on the stepper motor and stopped the move in progress.  Unrecoverable stalls are typically caused by too high of a commanded velocity for the motor and its load. Try decreasing the velocity and executing the move again.
#define  NIMCDM_targetVersionConflictError                  -77106 // The version of the LabVIEW NI SoftMotion Module installed on the host differs from the version installed on the RT target. Please use Measurement & Automation Explorer to update the LabVIEW NI SoftMotion Module on the RT target.
#define  NIMCDM_driveNotEnabledError                        -77105 // The drive connected to the axis you are trying to use is disabled or the commanded operation cannot execute because the drive is disabled.  Use the Power command or enable the Enable Drive on Transition to Active Mode checkbox on the Axis Setup page of the Axis Configuration dialog box to enable the drive.
#define  NIMCDM_updatingEnabledRepetitiveTableWarning        77004 // A repetitive-mode table was updated while in active mode.
#define  NIMC_resourceIsActiveWarning                        70002 // The required resources are actively completing the requested commands, but the requested commands not yet complete.
#define  NIMCDM_bufferCanNotWriteError                      -77104 // The specified buffer cannot be written to because it is full and has no more iterations available for rewriting.
#define  NIMCCFG_writeToReadOnlyError                       -78017 // The attribute you are attempting to write data to is read only.
#define  NIMCDM_solidworksAssemblyFileMismatchError         -77103 // The SolidWorks assembly that is currently open in SolidWorks is not the assembly that is configured in this project.
#define  NIMCDM_solidworksFailedToRetrieveMotionStudyError  -77101 // The motion study that is configured in this project could not be obtained or no longer exists.  Please select Change Motion Study from the SolidWorks assembly shortcut menu to configure a different motion study.
#define  NIMCDM_solidworksApplicationNotOpenError           -77100 // The SolidWorks application is not open.  The SolidWorks assembly associated with this project must be open to perform a SolidWorks simulation.
#define  NIMCDM_solidworksSensorNotPresentError             -77102 // One or more of the sensors you have mapped in the Map Sensors dialog box does not exist in the associated SolidWorks assembly.  To fix these mappings, select Map Sensors from the SolidWorks assembly shortcut menu.
#define  NIMCDM_NISoftMotionModuleBusyError                 -77099 // The requested operation cannot be completed because NI SoftMotion is busy. Try reducing the NI Scan Engine scan period.
#define  NIMCDM_invalidConfigurationError                   -77098 // Error initializing configuration. Please verify your axis configuration settings by right-clicking the axis in the LabVIEW Project window and selecting Properties.
#define  NIMCDM_scanModeTransitionWarning                    77003 // The NI Scan Engine has switched from Configuration mode to Active mode. Any system state information is lost. You may need to initialize the system again.
#define  NIMCDM_moveInProgressError                         -77097 // The commanded operation cannot be completed because a move is in progress.
#define  NIMCDM_solidworksAssemblyNotOpenError              -77096 // The SolidWorks assembly that has been deployed from the project is not open.
#define  NIMCDM_solidworksMotionStudyTypeNotSupportedError  -77095 // The motion study in your SolidWorks assembly is of an unsupported type.  Please activate the SolidWorks Motion add-in and choose "Motion Analysis" as the motion study type.
#define  NIMCDM_invalidLimitStateError                      -77094 // Both limit inputs are currently active. Verify that your limits are properly connected and configured.
#define  NIMCDM_ECamAlreadyEnabledError                     -77093 // Electronic camming is already enabled. Disable camming before you modify camming parameters.
#define  NIMC_noAxesInCoordinateError                       -70234 // There are no axes in the specified coordinate.
#define  NIMC_resourceNotFound                              -70233 // The specified resource was not found.
#define  NIMC_unableToConfirmDriveDisabledError             -70232 // The controller was unable to confirm that the drive is disabled. Please check your setup and try again.
#define  NIMC_invalidPreemptError                           -70231 // An invalid attempt to preempt the current operation was made.
#define  NIMCDM_invalidPropertyMethodError                  -77092 // An Axis Interface property or method can only be used with an unbound axis.
#define  NIMCDM_operationFailedDeviceFaultError             -77091 // The requested operation cannot be completed due to an existing hardware fault. Clear the fault condition and execute the operation again.
#define  NIMCDM_invalidOperationAtRunModeError              -77073 // The requested operation is not possible when the NI Scan Engine is in Active mode. Switch the NI Scan Engine to Configuration mode before executing the operation again.
#define  NIMCDM_busOperationTimeoutError                    -77072 // Communication to the hardware device exceeds the timeout limit.
#define  NIMCDM_busOperationPendingError                    -77071 // Communication request to the hardware device cannot be completed due to another operation in progress.
#define  NIMCDM_executeCalledInConfigModeWarning             77002 // The NI SoftMotion scan cycle has been executed when the NI Scan Engine is in Configuration mode. Execution data is not valid until NI SoftMotion and the hardware device successfully switch to Active mode.
#define  NIMCDM_busOperationPendingWarning                   77001 // Communication to the hardware device is in progress but is not completed. 
#define  NIMCDM_scanEngineFaultModeError                    -77090 // The requested operation is not possible when the NI Scan Engine is in Fault mode.
#define  NIMCDM_internalSoftwareError                       -77086 // An internal software error has occurred. Please contact National Instruments for assistance.
#define  NIMCDM_solidworksFeatureDataInvalidError           -77083 // The SolidWorks feature data is invalid.
#define  NIMCDM_solidworksCalculationFailedError            -77085 // An error occurred while the SolidWorks motion study was calculating.
#define  NIMCDM_incompatibleSolidworksVersionError          -77089 // The version of SolidWorks you have installed is not compatible with this version of the NI SoftMotion Module. Refer to the NI SoftMotion Module readme for supported SolidWorks versions.
#define  NIMCDM_solidworksDriveTypeNotSupportedError        -77084 // A motor in SolidWorks is of a non-supported drive type.
#define  NIMCDM_nullPointerError                            -77088 // An error occurred while accessing a null pointer.
#define  NIMCDM_osNotRecognized                             -77081 // The operating system is either not supported or the OS name is not recognized. Refer to the readme for supported operating systems.
#define  NIMCDM_incompatibeWithScanEngineError              -77082 // You have installed NI SoftMotion with NI Scan Engine Support. When using NI SoftMotion with the NI Scan Engine, only the NI Scan Engine can create, destroy, or execute a Trajectory Generator instance or motion resource such as axes or tables. Applications written with the NI SoftMotion Development Module will not work in this mode. Refer to the NI SoftMotion readme for information about how to upgrade your NI SoftMotion Development Module applications to work with the NI Scan Engine.
#define  NIMC_softmotionLicenseError                        -70230 // The evaluation period for NI SoftMotion has expired or you are using an unlicensed NI SoftMotion feature. You must purchase a license to use this feature of NI SoftMotion. Go to ni.com to learn about the features available in NI SoftMotion and to purchase a license for NI SoftMotion.
#define  NIMCDM_deployedDataInvalidError                    -77080 // An error occurred while reading deployed data.
#define  NIMCDM_SolidWorksAxisTypeNotSupportedError         -77079 // This SolidWorks motor type is not supported by NI SoftMotion. Select a valid motor type and synchronize your assembly again.
#define  NIMCDM_solidworksInterfaceError                    -77078 // An internal error was returned through the SolidWorks interface.
#define  NIMC_communicationTimeoutError                     -70229 // A function call made to the motion controller timed out waiting for a response to an executed command.
#define  NIMCDM_splineTypeCannotBeChangedSinceAxisActiveError -77074 // Spline type cannot be changed when the axis is active.
#define  NIMCDM_invalidIndexSpecifiedError                  -77075 // The specified index is not valid for the given resource.
#define  NIMCDM_driveReadyNotActiveError                    -77076 // The move cannot be started because the drive ready signal is not active.
#define  NIMCDM_scanEngineModeActiveError                   -77077 // The requested operation is not possible when the NI Scan Engine is in Active mode.
#define  NIMC_unableToConfirmDriveEnabledError              -70228 // The controller was unable to confirm that the drive is enabled.  The drive may have enabled but exceeded maximum position error and disabled again.  Please check your setup and try again.
#define  NIMC_driveNotEnabledError                          -70227 // The drive connected to the axis you are trying to use is disabled or the commanded operation cannot execute because the drive is disabled.  Use the Power command or enable the Enable Drive on Transition to Active Mode checkbox on the Axis Setup page of the Axis Configuration dialog box to enable the drive.
#define  NIMC_invalidHandleError                            -70226 // The handle provided is invalid.
#define  NIMC_moveStoppedBecauseLimitOrHomeActiveError      -70225 // The move stopped because an axis encountered a limit, home switch, or software limit.
#define  NIMC_faultStateError                               -70217 // A required resource is in a fault state.
#define  NIMC_operationAbortedWarning                        70001 // The requested operation has been aborted by a direct user action.
#define  NIMC_interfaceNotInitializedError                  -70224 // A required interface has not been properly initialized.  Please call the appropriate initialization method.
#define  NIMC_cammingDisabledIndirectlyError                -70223 // Camming has been disabled by an indirect user action or an external event.
#define  NIMC_gearingDisabledIndirectlyError                -70222 // Gearing has been disabled by an indirect user action or an external event.
#define  NIMC_positionCaptureDisabledIndirectlyError        -70221 // Position Capture has been disabled by an indirect user action or an external event.
#define  NIMC_positionCompareDisabledIndirectlyError        -70220 // Position Compare has been disabled by an indirect user action or an external event.
#define  NIMC_moveStoppedBecauseMaxPosErrorExceededError    -70219 // The move stopped because an axis exceeded the programmed position error limit. Verify the feedback resource connections and/or tune your motor.
#define  NIMC_moveStoppedBecauseDriveDisabledError          -70218 // The move stopped because the drive was disabled.
#define  NIMC_faultNotClearedWarning                         70003 // The fault condition could not be cleared or a fault occurred immediately after clearing faults.  Please ensure that the root cause of the fault has been resolved and try again.
#define  NIMC_resourceIsBusyError                           -70216 // A required resource is busy and unable to accept further commands at this time.
#define  NIMCDM_startPendingError                           -77070 // A Start operation is pending. Please add a delay between consecutive preemptive Start Motion calls.
#define  NIMC_invalidSettingsNameSpecifiedError             -70215 // An invalid initialization settings name was specified.
#define  NIMC_FPGAProgramError                              -70052 // Internal Error. The FPGA failed to program. Reset the controller and try again. If the problem persists, contact National Instruments technical support.
#define  NIMC_controllerInDisabledStateError                -70213 // The controller cannot perform the requested operation because it is currently disabled. Please reenable the controller in MAX.
#define  NIMCDM_communicationDeviceConfigurationError       -77069 // The communication device is not configured correctly. This error can be caused by an unsupported device or by an invalid communication parameter such as baud rate or port number. Check the communication settings in MAX to configure these parameters.
#define  NIMCDM_indexFoundError                             -77023 // Find Index sequence did not find the index successfully.
#define  NIMCDM_invalidSoftwareLimitPositionLoadedError     -77068 // The negative (lower) limit is greater than or equal to the positive (upper) limit.
#define  NIMCDM_addedListCannotBeClearedAsDevicePresentError -77066 // Internal software error.  The persisted list of added axes could not be cleared.
#define  NIMCDM_feedbackSupportError                        -77065 // The selected feedback device is not supported by the controller or the drive, or it is not available for the current axis.
#define  NIMCDM_timeSourceTimedOutError                     -77061 // Internal software error. The timing source being used by NI SoftMotion has timed out
#define  NIMCDM_timeSourceNotConfiguredError                -77060 // Internal software error. The timing source being used by NI SoftMotion has timed out
#define  NIMCDM_timeSourceNotSupportedError                 -77059 // Internal software error. The timing source being used by NI SoftMotion has timed out
#define  NIMCDM_notchParametersNotConfiguredCorrectlyError  -77058 // The notch filter parameters specified are not correct.
#define  NIMCDM_LabVIEWCommInterfaceNotFoundError           -77056 // Internal software error. This may be due to a bad installation. Reinstall or repair the LabVIEW NI SoftMotion Module software.
#define  NIMCDM_feedbackMappedToAxisError                   -77053 // The selected feedback device cannot be used because it is used by a different active axis.
#define  NIMCDM_viLibraryNotFoundError                      -77049 // Internal software error. This may be due to a bad installation.  Reinstall or repair the LabVIEW NI SoftMotion Module software.
#define  NIMCDM_watchdogNotSupportedError                   -77048 // Software watchdog functionality is not supported on this target.
#define  NIMCDM_watchdogAlreadyRunningError                 -77047 // The watchdog service has already been started.
#define  NIMCDM_invalidPhaseSpecifiedError                  -77045 // The specified find reference phase sequence is invalid.
#define  NIMCDM_bufferUpdateInvalidError                    -77044 // The requested buffer interval is not valid.
#define  NIMCDM_contouredUpdateMissedError                  -77042 // The contoured buffer could not be updated in time.  Please reduce the host loop rate.
#define  NIMCDM_noMoreDataError                             -77041 // The buffer ran out of data sooner than expected.
#define  NIMCDM_failedToCreateInternalInterfacesError       -77039 // Internal software error. This may be due to a bad installation.  Reinstall or repair the LabVIEW NI SoftMotion Module software.
#define  NIMCDM_invalidDataError                            -77037 // Data being set is invalid.
#define  NIMCDM_motionServiceStartedError                   -77036 // The motion service has already been started
#define  NIMCDM_findReferenceError                          -77026 // The find reference move cannot complete because of one of the following reasons: the axis is stopped, the axis unexpectedly encountered a limit, or due to an incorrect home switch signal transition.
#define  NIMCDM_axisNotActiveError                          -77021 // You cannot perform this operation while the axis is disabled or killed.
#define  NIMCDM_updateFailedError                           -77014 // The axis could not be updated in the specified period.  Please change your control loop or host loop update rates
#define  NIMCDM_homeSwitchActiveError                       -77024 // The find reference move was not successful because the home switch is active.
#define  NIMCDM_limitSwitchActiveError                      -77025 // The limit switch is active. The axis (axes) has reached its end of travel.
#define  NIMCDM_axisCommunicationWatchdogError              -77055 // Communication with the device has timed out. Verify your connections, then switch the NI Scan Engine to Configuration mode and back to Active mode to restore communication with the device.
#define  NIMCDM_coordinateCannotBeConfiguredAsAxisIsMovingError -77067 // The coordinate cannot be configured because one of the axes in the coordinate is moving.
#define  NIMCDM_masterNotActiveError                        -77064 // Gearing cannot be enabled because the master axis is not active.
#define  NIMCDM_blendNotCompleteError                       -77063 // Blend operation must be completed before another blend operation can be started.
#define  NIMCDM_configurationFileNotFoundError              -77062 // The configuration file for NI SoftMotion is missing.
#define  NIMCDM_axisCannotBeAddedAsControllerNotResetError  -77057 // Unused
#define  NIMCDM_axisDisabledError                           -77054 // This operation is not supported on a disabled axis. Enable the axis and try again.
#define  NIMCDM_startBlockedDueToFollowingErrorError        -77052 // Start motion is blocked because of following error. Verify the axis feedback and control, and then execute a halt stop before starting the move.
#define  NIMCDM_incorrectArraySizeError                     -77051 // The specified array size is too small.
#define  NIMCDM_invalidLoopRateError                        -77050 // The specified host loop rate is invalid for the device or the current control loop rate.
#define  NIMCDM_controllerEStopStateActiveError             -77046 // The emergency stop (E-stop) signal was activated. Determine the cause of the E-stop condition and correct the problem, then switch to Configuration state and back to Active state before continuing execution.
#define  NIMCDM_contouringSamplingIntervalError             -77043 // The sampling interval is invalid.
#define  NIMCDM_deviceAlreadyAddedError                     -77040 // The specified device is already added.
#define  NIMCDM_failedToAddDeviceError                      -77038 // Failed to add the drive to the controller. Please verify the drive ID is configured properly.
#define  NIMCDM_blendNotAllowedInThisModeError              -77035 // Blending is not allowed when in velocity mode or when configured for contouring. This error can also be returned if you call the blend function when the blend complete status is FALSE.
#define  NIMCDM_bufferEmptyError                            -77034 // The specified buffer does not contain any data.
#define  NIMCDM_bufferInUseError                            -77033 // The specified buffer is already in use for another operation.
#define  NIMCDM_bufferNotUpdatedError                       -77032 // Old data was encountered in the contour table. Verify that you are updating the contour points fast enough or increase the contour interval. If you have written all the points to the contour table make sure you set last update in the Update Points method to TRUE.
#define  NIMCDM_insufficientSizeError                       -77031 // The specified buffer is of insufficient size to store the data sent.
#define  NIMCDM_incorrectBufferSizeError                    -77030 // The size of the buffer or array is incorrect.
#define  NIMCDM_bufferNotEmptyError                         -77029 // The requested buffer is not empty.
#define  NIMCDM_invalidBufferHandleSpecifiedError           -77028 // The buffer handle specified for the contouring move is invalid.
#define  NIMCDM_noMoreBuffersError                          -77027 // Create buffer failed as the total number of buffers supported have all been used.
#define  NIMCDM_axisTrippedOnFollowingError                 -77022 // The axis exceeded the programmed following error limits. Verify your feedback resource and/or tune your motor.
#define  NIMCDM_limitsNotEnabledError                       -77020 // Find Reference function cannot execute because the Limit inputs are not enabled.  Enable the Limit inputs and try again.
#define  NIMCDM_noEncoderError                              -77019 // No encoder is assigned to a servo or closed-loop stepper axis.
#define  NIMCDM_coordinateSpaceNotConfigured                -77018 // The specified coordinate space is not configured.
#define  NIMCDM_referenceMoveInProgressError                -77017 // The current operation cannot be completed because the axis is performing a find reference move. Changing find reference parameters and starting other moves requires the axis to finish the find reference move first.
#define  NIMCDM_functionNotSupportedError                   -77016 // This command is not supported by this controller or operating system.
#define  NIMCDM_specifiedDeviceNotFoundError                -77015 // The specified device was not found.
#define  NIMCDM_invalidHandleSpecifiedError                 -77013 // The resource (axis, coordinate, or blend) handle provided is not valid.
#define  NIMCDM_dataOutOfRangeError                         -77012 // Parameter specified exceeds the valid data range.
#define  NIMCDM_homeSwitchNotEnabledError                   -77011 // Find Reference function cannot execute because the Home input is not enabled.  Enable the Home input and try again.
#define  NIMCDM_serializationFailedError                    -77010 // Failed to store user defaults for the target device.
#define  NIMCDM_motionServiceNotStartedError                -77009 // A function in NI SoftMotion was called without creating or starting the controller service. Verify that the NI Scan Engine switched to Active mode and that there are no faults on the axes.
#define  NIMCDM_deviceNotPresentError                       -77008 // The specified device was not detected.
#define  NIMC_invalidIndexError                             -70212 // The index parameter passed to the function is invalid.  Check the VI or function reference help for valid values.
#define  NIMC_dataOutOfRangeError                           -70211 // The data parameter passed to the function does not contain a valid value.  Check the VI or function reference help for valid values.
#define  NIMC_clientConnectionRefusedError                  -70210 // An unexpected error has occurred internal to the driver.  The driver has refused to accept the client connection.  Please contact National Instruments technical support with the name of the function or VI that returned this error.
#define  NIMC_attributeIsReadOnlyError                      -70214 // The selected attribute is read-only.
#define  NIMCDM_noDevicesFoundError                         -77007 // No supported devices were detected.
#define  NIMCDM_communicationInterfaceNotFoundError         -77006 // Some of the NI SoftMotion Module components are missing, incompatible, or not properly started.  Verify your hardware connections and that the device drivers are running. Otherwise, try reinstalling or repairing the LabVIEW NI SoftMotion Module software. 
#define  NIMCDM_resourceNotConfiguredError                  -77005 // The specified resource handle is either invalid or does not exist, or you are using an NI SoftMotion trajectory generator resource handle with NI Scan Engine support installed.
#define  NIMCDM_invalidAttributeError                       -77004 // Attribute selected is not valid for current operation.
#define  NIMCDM_deviceTypeNotSupportedError                 -77003 // The device in use is not supported by the NI SoftMotion Module.
#define  NIMCDM_dataTransmissionError                       -77002 // Data was not successfully transmitted to the device connected to the NI SoftMotion Module.  This may be caused if NI SoftMotion cannot update the device(s) at the specified host loop rate.  Reduce the host loop rate and try again.
#define  NIMCDM_outOfMemoryError                            -77001 // The targeted device does not have enough memory to perform the requested operation.
#define  NIMC_driveReadyNotActiveError                      -70209 // The move cannot be started because the drive ready input of the commanded axis is not active.
#define  NIMC_IOResourceConfigurationError                  -70208 // You cannot change the configuration or mapping for this I/O resource because an enabled feature is currently using it. Please disable the feature before reconfiguring the I/O resource.
#define  NIMC_inhibitInputActiveError                       -70207 // The move cannot be completed because the inhibit input of the commanded axis is active.  Make sure all unused axes are disabled, the inhibit inputs are set to the correct active state, and that there are no errors on the drive causing the inhibit input to be active.
#define  NIMC_invalidLicensingInformationError              -70206 // Unable to verify licensing information from this controller. Power cycle the system and contact National Instruments if the problem persists.
#define  NIMC_invalidSerialNumberError                      -70205 // The controller does not have a valid serial number. Please contact National Instruments to resolve this problem.
#define  NIMC_IOResourceNotConfiguredError                  -70204 // You cannot enable the specified feature because the I/O resource necessary for the feature is not yet configured or mapped.
#define  NIMC_noMoreBufferError                             -70203 // Failed to create a buffer on the controller. Please clear one or more buffers.
#define  NIMC_failedToAddDeviceError                        -70202 // Failed to add the drive to the controller. Please verify the drive ID is configured properly.
#define  NIMC_vectorSpaceCannotBeConfiguredError            -70201 // The vector space cannot be configured because one of the axes in the vector space is moving.
#define  NIMC_controllerNotInPowerUpResetStateError         -70200 // This operation requires the controller to be in a power up reset state. Please reset the controller.
#define  NIMC_gearingMasterNotActiveError                   -70199 // Gearing cannot be enabled because the master is not active.
#define  NIMCCFG_axisAlreadyExistsError                     -78016 // The axis number of the axis being added already exists.
#define  NIMC_configurationFileNotFoundError                -70198 // Internal Error.
#define  NIMC_startBlockedDueToFollowingError               -70197 // Start motion is blocked because of following error. Verify the axis feedback and control, and then execute a halt stop before starting the move.
#define  NIMC_axisNotPresentError                           -70196 // Internal Error
#define  NIMC_invalidAxisScaleError                         -70195 // The axis scale is not correct for loading the move constraint in this unit. Use the Load Counts/Steps per Revolution function to load the correct axis scale.
#define  NIMC_axisAlreadyAddedError                         -70194 // Internal Error.
#define  NIMC_communicationInterfaceNotFoundError           -70192 // Internal Error.
#define  NIMC_axisCommunicationWatchdogError                -70191 // Communication with the device has timed out.
#define  NIMC_invalidLoopRateError                          -70190 // The specified host loop rate is invalid for the device or the current control loop rate.
#define  NIMC_blendNotAllowedInThisModeError                -70189 // Blending is not valid in this position mode.
#define  NIMC_invalidBufferHandleSpecifiedError             -70188 // Internal error. The specified buffer handle is invalid.
#define  NIMC_incorrectBufferSizeError                      -70187 // The specified buffer size is incorrect for the requested operation.
#define  NIMC_homeSwitchActiveError                         -70186 // The find reference move was not successful because the home switch is active.
#define  NIMC_serializationFailedError                      -70185 // Failed to store user defaults for the target device.
#define  NIMC_deviceTypeNotSupported                        -70184 // Internal Error.
#define  NIMC_dataTransmissionError                         -70183 // Data transmission to the targeted device failed. 
#define  NIMCCFG_copySettingsFailedError                    -78015 // Failed to copy the initialization settings.
#define  NIMCDM_noError                                          0 // No error.
#define  NIMC_deviceNotActivatedError                       -70182 // The device or resource has not been activated, or the license file is invalid.
#define  NIMCCFG_initializationFailedError                  -78014 // The Motion interface to mxs could not be initialized.  This is usually due to an internal error.  Please ensure that Measurment & Automation Explorer is installed correctly.
#define  NIMC_remoteBoardMismatchError                      -70181 // The serial number of the motion controller with the given board ID does not match what is expected.
#define  NIMC_invalidMasterEnabledError                     -70180 // The gear master for an axis may not be set to 'None' while gearing is enabled for the axis. Disable gearing before changing the gear master.
#define  NIMC_invalidMasterAxisError                        -70179 // The gearing master for the specified axis is invalid.  Please configure a valid gear master then enable gearing for this axis.
#define  NIMC_invalidFeedbackResetPositionError             -70086 // Invalid reset position is applied to the feedback device. If the same feedback device is configured as both primary and secondary feedback, it must be reset to the same position.
#define  NIMC_onlyInAProgramError                           -70178 // This function can only be executed from an Onboard Program.
#define  NIMC_invalidTimeSliceError                         -70177 // You have entered an invalid value for the onboard program time slice.  All onboard program time slices cannot add up to be greater than 20 ms.
#define  NIMC_timeGuaranteeError                            -70176 // The controller could not guarantee that all onboard programs are running in a timely fashion. If you are performing arcs try selecting Extended Arc Points Interval from the Performance Options tab in MAX.  Otherwise try selecting the Extended Watchdog Interval.
#define  NIMC_arcPointsBufferFullError                      -70175 // Internal Error.  The internal arc points buffer is full. 
#define  NIMC_modeNotSupportedError                         -70047 // This mode is not supported by this controller.
#define  NIMC_softwareUpdateRequiredError                   -70120 // This NI Motion controller is not compatible with this version of the NI-Motion Software.  Please upgrade your NI-Motion Software.
#define  NIMC_noError                                            0 // No error.
#define  NIMC_internalBreakpointMissingError                -70173 // Internal error.  The internal breakpoint buffer is missing data.
#define  NIMC_internalHSCaptureMissingError                 -70174 // Internal error.  The internal high-speed capture buffer is missing data.
#define  NIMC_hsCaptureBufferFullError                      -70172 // Internal Error.  The internal High-Speed Capture buffer is full. 
#define  NIMC_breakpointBufferFullError                     -70171 // Internal Error.  The internal breakpoint buffer is full. 
#define  NIMC_interfaceNotSupportedError                    -70170 // The interface you are requesting is not supported.
#define  NIMC_invalidPointerError                           -70169 // A NULL or invalid pointer was passed as a parameter to this function.
#define  NIMC_invalidMotionIDError                          -70168 // An invalid Motion ID was passed in a function.
#define  NIMC_initializationInProgress                      -70167 // The controller is currently initializing.
#define  NIMCCFG_exportFailedError                          -78013 // The initalization settings file could not be exported. This error is usually due to an internal error.  Ensure the initialization settings file being exported are present in the configuration database.
#define  NIMCCFG_importFailedError                          -78012 // The initialization settings file could not be imported.  Check the path and name of the file you are trying to import.
#define  NIMC_followingErrorOnFindReference                 -70166 // The Find Reference sequence did not successfully find the reference position because the axis position error limit was exceeded.
#define  NIMC_findReferenceError                            -70165 // Find Reference was not successful because the motor stopped before the reference position was located.
#define  NIMCCFG_commitFailedError                          -78011 // Changes could not be persisted to the initialization settings.
#define  NIMCCFG_objectInUseError                           -78010 // The object being deleted is in use (being referenced by other objects).
#define  NIMCCFG_deleteFailedError                          -78009 // Failed to delete the initialization setting specified.  Ensure the setting is not currently being used by a motion device.
#define  NIMCCFG_createFailedError                          -78008 // Failed to create a new initialization setting.
#define  NIMCCFG_invalidSettingsNameSpecifiedError          -78007 // An invalid initialization settings name was specified.
#define  NIMCCFG_insufficientSizeError                      -78006 // The array passed in is of insufficient size to return the data required.
#define  NIMCCFG_outOfRangeError                            -78005 // The value being saved to the database is out of range.
#define  NIMCCFG_invalidResourceSpecifedError               -78004 // The attribute accessed is on an invalid axis or resource.
#define  NIMCCFG_irrelevantAttributeError                   -78003 // The attribute used is invalid.
#define  NIMCCFG_setFailedError                             -78002 // Failed to set an attribute in the initialization settings.
#define  NIMCCFG_getFailedError                             -78001 // Failed to get an attribute value from the initialization settings.
#define  NIMC_ADCConfigurationError                         -70164 // ADCs cannot be enabled or disabled while axes are enabled with analog feedback.  Also, ADC ranges cannot be changed on ADC channels being used for analog feedback while axes are enabled.
#define  NIMC_invalidLimitRangeError                        -70163 // The negative (lower) limit is greater than or equal to the positive (upper) limit.
#define  NIMC_torqueOffsetError                             -70162 // The torque offset is outside of the torque limit range.
#define  NIMC_blendFactorConflictError                      -70161 // Axes, which are part of a vector space, have different blend factors.  Make sure that all the axes in the vector space have the same blend factor.
#define  NIMC_reserved98Error                               -70098 // Was BAD_IO_DIR_ERROR
#define  NIMC_splineUnderflowError                          -70107 // The motion controller was too busy to send points to the spline engine and an underflow occurred.  Increase the Coarse Arc Points Interval Period in MAX if performing an arc move, or increase the requested interval between points in your buffer if performing a contour move.
#define  NIMC_IOReservedError                               -70105 // I/O has been reserved.  This error occurs when I/O has been reserved for another use such as Hall effect sensors or drive signals.  You must unreserve the I/O before using it.  This error might also be generated if you attempt to reserve a high-speed capture (HSC) line for a drive signal when HSC is enabled.
#define  NIMC_invalidBreakpointWindowError                  -70048 // The breakpoint window is not valid.
#define  NIMC_downloadChecksumError                         -70049 // There was an error during checksum on a file being downloaded to the NI motion controller.
#define  NIMC_invalidCalibrationDataError                   -70046 // Calibration data were not successfully read from the flash EEPROM. 
#define  NIMC_axesNotKilledError                            -70045 // During the calibration process (both internal and external), please make sure that all axes are in a killed state using the Stop Motion function.
#define  NIMC_calibrationStepError                          -70044 // For external calibration users must do provide the password to the board, perform adjustment and set the new onboard reference in sequence.
#define  NIMC_calibrationOutOfRangeError                    -70043 // The calibration target is out of range. We can't reach the calibration target even with full scale caldac or minimum caldac.
#define  NIMC_RemoteConnectionFailureError                  -70042 // Unable to establish a connection with the remote system.  Use MAX to verify that your remote board mappings are valid.
#define  NIMC_boardIDInUseError                             -70041 // The boardID is already in use by another controller.  If trying to change a controllers boardID, select a boardID not in use.
#define  NIMC_reservedOnBoardProgramError                   -70040 // The current device is reserved for onboard program while download process is in progress.  To release the device, use End Program Storage after the download process is completed.
#define  NIMC_1394WatchdogEnableError                       -70039 // The communication watchdog on the 1394 motherboard could not be started.
#define  NIMC_moduloBreakpointError                         -70123 // The breakpoint value or the breakpoint window loaded exceeds the modulo range.
#define  NIMC_axisConfigurationClLoopError                  -70112 // An enabled closed-loop axis must have a resource mapped as its primary feedback.  Disable the axis before removing the resource.
#define  NIMC_availableForUse91                             -70091 // None
#define  NIMC_axisConfigurationSwitchError                  -70111 // A resource is currently configured for another axis that is enabled.  Disable the axis using that resource.
#define  NIMC_blendNotCompleteError                         -70087 // Blend operation must be completed before another blend operation can be started. Use Check Blend Complete Status to read the current blend operation status.
#define  NIMC_invalidFeedbackDeviceError                    -70088 // The current operation cannot be performed on the axis feedback device. Configuring phase and index settings can be applied  only to axes with encoder feedback.
#define  NIMC_axisFindingReferenceError                     -70089 // The current operation cannot be completed because the axis is performing a find reference move. Changing find reference parameters and starting other moves requires the axis to finish the find reference move first.
#define  NIMC_onboardProgramSupportError                    -70090 // This function cannot be executed as part of an onboard program. Make sure a program is not currently storing or run End Program Storage to complete the current storing program before calling this function.
#define  NIMC_autoStartFailedError                          -70079 // The controller could not run the onboard program on auto start.  When you enable auto start, make sure that you specify a valid program number and that the program is saved in FLASH ROM.
#define  NIMC_shutDownFailedError                           -70074 // The controller failed to shut down.  This could be because it failed to disable the DACs, the encoders, or the ADCs, or because it could not reset the I/O back to user defaults. 
#define  NIMC_boardInShutDownStateError                     -70073 // The controller cannot accept this function, as it has been shut down.
#define  NIMC_boardInPowerUpResetStateError                 -70072 // The NI motion controller is in power up reset state.  Please execute the Clear Power Up Status function or VI before sending any commands to the controller.
#define  NIMC_DIOReservedForHallSensors                     -70071 // Direction of the DIO lines reserved for Hall sensor input cannot be set to output.
#define  NIMC_commutationModeError                          -70070 // Commutation mode and commutation parameters cannot be changed while the axis is enabled.
#define  NIMC_invalidHallSensorStateError                   -70069 // Invalid Hall sensor state. Please verify the connection to the Hall sensors of ALL enabled onboard commutation axes.
#define  NIMC_maxHSCaptureFrequencyError                    -70068 // A buffered high-speed capture operation exceeded the maximum frequency allowed.
#define  NIMC_maxBreakpointFrequencyError                   -70067 // A buffered breakpoint operation exceeded the maximum frequency allowed.
#define  NIMC_virtualBoardError                             -70066 // The board ID passed is currently being used for a virtual NI-Motion controller.  No commands can be sent to a virtual board.
#define  NIMC_illegalContouringError                        -70065 // This operation cannot be executed while a contouring operation is in progress.
#define  NIMC_illegalBufferOperation                        -70064 // This operation is invalid at this time.  You cannot write to a ROM buffer; or you cannot read or write to the buffer at this time.
#define  NIMC_bufferConfigurationError                      -70063 // The buffer has not been configured properly.  Buffer type cannot be changed.  The buffer might exist in ROM, but has not yet been configured since the last power cycle.
#define  NIMC_oldDataStopError                              -70062 // The Read Buffer or Write Buffer function was unable to complete successfully because old data was encountered in the course of reading from or writing to the buffer.
#define  NIMC_bufferInUseError                              -70061 // The specified buffer is in use.
#define  NIMC_interruptConfigurationError                   -70057 // Internal Error.  Host interrupt configuration failed and interrupt support is disabled.
#define  NIMC_firmwareDownloadError                         -70051 // Firmware download failed.  Reset the controller and try downloading again.
#define  NIMC_stepChannelInitializationError                -70160 // Internal Error.  The internal stepper output data structures failed to initialize.  Reset the controller and try again. If the problem persists, contact National Instruments technical support.
#define  NIMC_encoderInitializationError                    -70159 // Internal Error.  The internal encoder data structures failed to initialize.  Reset the controller and try again. If the problem persists, contact National Instruments technical support.
#define  NIMC_axisInitializationError                       -70158 // Internal Error.  The internal axis data structures failed to initialize.  Reset the controller and try again. If the problem persists, contact National Instruments technical support.
#define  NIMC_pointsBufferFullError                         -70157 // Internal error.  The internal point request buffer is full. 
#define  NIMC_reserved110Error                              -70110 // None - was duplicate - same as 106 -NIMC_outputDeviceNotAssignedError
#define  NIMC_reserved156Error                              -70156 // None - Duplicate - same as Error 27
#define  NIMC_QSPIFailedError                               -70155 // Internal error.  The internal QSPI serial bus failed and ADC values cannot be read.
#define  NIMC_samplesBufferFullError                        -70153 // Internal error.  The internal samples buffer is full. 
#define  NIMC_reserved143Error                              -70143 // None - Was used for I/O interrupt stuck on condition.
#define  NIMC_internalSamplesMissingError                   -70134 // Internal error.  The internal samples buffer is missing samples.
#define  NIMC_pointRequestMissingError                      -70133 // Internal error.  The internal points request buffer is missing a request.
#define  NIMC_targetPositionUpdateError                     -70132 // Internal error.  The DSP was too busy to update the target position.
#define  NIMC_inPositionUpdateError                         -70131 // Internal error.  The axis position could not be read for in-position verification.
#define  NIMC_findHomeError                                 -70118 // Find home was not successful because the motor stopped before the home switch was found.
#define  NIMC_outOfEventsError                              -70104 // Internal error.  The number of events pending have reached the maximum allowed.
#define  NIMC_DSPCommunicationsTimeoutError                 -70095 // Internal error.  There was an internal timeout while sending commands to the DSP.
#define  NIMC_DSPCommunicationsError                        -70093 // Internal error. A command from the onboard microprocessor to the DSP was corrupt and ignored.
#define  NIMC_DSPMessageBufferEmptyError                    -70094 // Internal error.  This is an internal message to indicate that the there is no more data in the internal message buffer.
#define  NIMC_DSPXmitDataError                              -70092 // Internal error.  The data returned by the DSP is incomplete or corrupt.
#define  NIMC_DSPInvalidDeviceError                         -70085 // Internal error.  The DSP received a command with an invalid Device ID.
#define  NIMC_DSPInvalidCommandError                        -70084 // Internal error.  The DSP received an illegal command.
#define  NIMC_DSPXmitBufferFullError                        -70083 // Internal error. The transmit buffer of the DSP is full. Messages from DSP to the onboard microprocessor are being delayed or lost.
#define  NIMC_reserved82Error                               -70082 // Reserved - Never used for position wrap-around error.
#define  NIMC_reserved81Error                               -70081 // Reserved - Never used for DSP messaging error.
#define  NIMC_corruptHostDataError                          -70077 // Communications error.  The controller received corrupt data in the packet from the host computer.
#define  NIMC_noHostDataError                               -70076 // Communications error.  The controller did not receive any data in the command packet from the host computer.
#define  NIMC_hostFIFOBufferFullError                       -70075 // Communication FIFO buffer between the host computer and the controller is full.
#define  NIMC_corruptObjectSectorError                      -70060 // The objects stored in FLASH ROM are corrupt.
#define  NIMC_flashromCopyError                             -70059 // Error copying to the FLASH ROM.
#define  NIMC_IOInitializationError                         -70058 // Internal Error.  The I/O structure on the controller failed to initialize.  Reset the controller and try again. If the problem persists, contact National Instruments technical support.
#define  NIMC_corruptFPGAFirmwareError                      -70056 // Corrupt FPGA firmware detected.  Download new FPGA firmware.
#define  NIMC_corruptDSPFirmwareError                       -70055 // Corrupt DSP firmware detected.  Download new DSP firmware.
#define  NIMC_corrupt68331FirmwareError                     -70054 // Corrupt onboard microprocessor firmware detected.  Download new firmware.
#define  NIMC_DSPInitializationError                        -70053 // Internal Error.  The DSP failed to initialize.  Reset the controller and try again. If the problem persists, contact National Instruments technical support.
#define  NIMC_illegalVectorError                            -70154 // The input or return vector being used is invalid.
#define  NIMC_invalidAccelerationError                      -70152 // The vector acceleration is not valid.  The resulting angular acceleration is out of range.  Change the vector acceleration for the arc move.
#define  NIMC_invalidVelocityError                          -70151 // The vector velocity is not valid.  The resulting angular velocity is out of range.  Change the vector velocity for the arc move.
#define  NIMC_followingErrorOnFindHomeError                 -70150 // Find Home sequence did not find home successfully because the axis tripped on following error.
#define  NIMC_operationModeError                            -70149 // Axes that are a part of a vector space are either in velocity mode or have different operation modes.
#define  NIMC_ADCDisabledError                              -70148 // The ADC is disabled.  The ADC channel must be enabled to read it.
#define  NIMC_slaveAxisKilledError                          -70147 // Gearing cannot be enabled because the slave axis is in a killed state.  Issue a halt stop with the Stop Motion function on the slave axis to energize it.
#define  NIMC_noMoreSamplesError                            -70146 // There are no samples to read.  Execute Acquire Trajectory Data before trying to read samples.
#define  NIMC_illegalVectorSpaceError                       -70145 // The vector space being used does not have enough axes assigned to it.
#define  NIMC_illegalVariableError                          -70144 // An illegal general-purpose variable is being used.
#define  NIMC_followingErrorOverruleError                   -70142 // A Start, Blend, Find Home, or Find Index function being executed from an onboard program has been overruled due to a following error condition.  The program is left in the PAUSED state.  Execute the Pause/Resume Program function to continue.
#define  NIMC_programOverruleError                          -70141 // A Start, Blend, Find Home, or Find Index function being executed from an onboard program has been overruled by a Stop Motion function from the host computer. The program is left in the PAUSED state.  Execute the Pause/Resume Program function to continue.
#define  NIMC_registryFullError                             -70139 // Object registry is full.  The number of programs and buffers has reached the limit.  Free some programs or buffers from RAM or ROM using the Object Memory Management function.
#define  NIMC_outOfMemoryError                              -70138 // There is not enough memory to perform the requested operation.  If you are using an NI 73xx controller, this error may indicate that there is not enough FLASH ROM space to save a program or buffer. If you are using NI SoftMotion, this error indicates that the targeted device does not have enough memory to perform the requested operation.
#define  NIMC_noMoreProgramPlayerError                      -70140 // All program players (maximum 10) are in use storing/playing programs.
#define  NIMC_objectReferenceError                          -70137 // An attempt was made to reference a nonexistent program object or buffer object.  Or, the object number is already in use by an object of a different type.  Choose a different object number, or free/delete the object currently owning that object number.
#define  NIMC_eventTimeoutError                             -70136 // A wait operation timed out or a read function timed out.
#define  NIMC_reserved135Error                              -70135 // Was NIMC_axisEnabledError
#define  NIMC_memoryRangeError                              -70130 // An invalid memory location is being addressed on the controller.
#define  NIMC_axisDisabledError                             -70129 // A disabled axis has been commanded to move.  Enable the axis before executing a move on it.
#define  NIMC_available128Error                             -70128 // Was used for TABLE_ALLOC_ERROR but NIMC_noMoreRAMError should be used instead.
#define  NIMC_pointsTableFullError                          -70127 // The points table for cubic splining is full.
#define  NIMC_axisConfigurationError                        -70126 // An axis cannot change feedback while moving or change output while enabled.  Stop and/or disable the axis and then configure it.
#define  NIMC_wrongModeError                                -70125 // The function was not executed because it was attempted at an illegal time.
#define  NIMC_findIndexError                                -70124 // Find Index sequence did not find the index successfully.
#define  NIMC_encoderDisabledError                          -70122 // The encoder is disabled.  The encoder must be enabled to read it.
#define  NIMC_positionRangeError                            -70121 // Absolute target position loaded would cause the move length to be out of the +/-31 bit range allowed for a single move segment.
#define  NIMC_limitSwitchActiveError                        -70119 // The desired move cannot be completed because the limit input is active in the direction of travel.
#define  NIMC_homeLimitNotEnabledError                      -70117 // Find Reference function cannot execute because the Home and/or Limit inputs are not enabled.  Either enable Limits or use SmartEnable.
#define  NIMC_invalidConditionCodeError                     -70116 // Condition selected is invalid.
#define  NIMC_jumpToInvalidLabelError                       -70115 // A Jump to Label on Condition function in a program had an invalid label.
#define  NIMC_reserved114Error                              -70114 // Was NIMC_nestedProgramError.
#define  NIMC_noMoreRAMError                                -70113 // No RAM available for object (program or buffer) storage.
#define  NIMC_feedbackDeviceNotAssignedError                -70109 // No primary feedback device (encoder or ADC) is assigned to a servo or closed-loop stepper axis.
#define  NIMC_PIDUpdateRateError                            -70108 // PID rate specified is too fast for the number of axes and/or encoders enabled.
#define  NIMC_outputDeviceNotAssignedError                  -70106 // No DAC or stepper output is assigned to this axis.
#define  NIMC_wrongIOConfigurationError                     -70103 // I/O bit configuration is not possible for that pin.
#define  NIMC_wrongIODirectionError                         -70102 // The I/O bit configuration does not agree with its port's direction setting.
#define  NIMC_reserved101Error                              -70101 // Was PAI_TM_ERROR - Problem with the I/O timer gate.
#define  NIMC_IOEventCounterError                           -70100 // Problem with the I/O Event Counter.
#define  NIMC_reserved99Error                               -70099 // Was OUTCOMP_ERROR - Counter/timer breakpoint is set for an invalid I/O bit.
#define  NIMC_mustOnMustOffConflictError                    -70097 // There is a conflict between the mustOn and mustOff values set for this function.
#define  NIMC_passwordError                                 -70096 // This function is password protected. Please enter the correct password to access it.
#define  NIMC_returnDataBufferFullError                     -70080 // The Return Data Buffer on the controller is full.
#define  NIMC_invalidFunctionDataError                      -70078 // Invalid function data has been passed.  This is usually a parameter out of range, or an illegal combination of parameter values.
#define  NIMC_invalidRatioError                             -70036 // A specified ratio is invalid.
#define  NIMC_modalErrorsReadError                          -70026 // The Motion Error Handler.flx VI discovered modal error(s) in the modal error stack.  These error(s) can be viewed in the Modal Error(s) Out Indicator/terminal of this VI.
#define  NIMC_parameterValueError                           -70023 // One of the parameters passed into the function has an illegal value.
#define  NIMC_insufficientSizeError                         -70028 // The resource is not large enough to perform the specified operation.
#define  NIMC_internalSoftwareError                         -70038 // An unexpected error has occurred internal to the driver.  Please contact National Instruments with the name of the function or VI that returned this error.
#define  NIMC_irrelevantAttributeError                      -70037 // The specified attribute is not relevant.
#define  NIMC_reserved50Error                               -70050 // Reserved.
#define  NIMC_functionSupportError                          -70022 // This command is not supported by this controller or operating system.
#define  NIMC_watchdogTimeoutError                          -70035 // A fatal error has occurred on the controller. You must reset the controller by power cycling your computer. Contact National Instruments technical support if this problem persists.
#define  NIMC_wrongReturnDataError                          -70034 // Incorrect data has been returned by the controller.  This data does not correspond to the expected data for the command sent to the controller.
#define  NIMC_badPointerError                               -70033 // A NULL pointer has been passed into a function inappropriately.  If you are using LabVIEW verify that you have a valid resource selected.  If the API function does not take a pointer then this error indicates that an unexpected error has occurred in the software.
#define  NIMC_reserved7Error                                -70032 // Reserved for obsolete product.
#define  NIMC_reserved6Error                                -70031 // Reserved for obsolete product.
#define  NIMC_reserved5Error                                -70030 // Reserved for obsolete product.
#define  NIMC_reserved4Error                                -70029 // Reserved for obsolete product.
#define  NIMC_motionOnlyError                               -70024 // Motion command sent to an Encoder board.
#define  NIMC_systemResetError                              -70021 // System reset did not occur in maximum time allowed.
#define  NIMC_countsNotConfiguredError                      -70020 // The steps per revolution and/or counts per revolution are not loaded for this axis.
#define  NIMC_closedLoopStepperOnlyError                    -70018 // This command is valid only on closed-loop stepper axes.
#define  NIMC_stepperOnlyError                              -70017 // This command is valid only on stepper axes.
#define  NIMC_servoOnlyError                                -70016 // This command is valid only on servo axes.
#define  NIMC_returnDataBufferFlushError                    -70015 // Unable to flush the Return Data Buffer.
#define  NIMC_closedLoopOnlyError                           -70014 // This command is valid only on closed-loop axes (closed-loop stepper and servo).
#define  NIMC_badCommandError                               -70010 // Command not recognized. Invalid command sent to the controller.
#define  NIMC_processTimeoutError                           -70027 // A function call made to the motion controller timed out waiting for access to a shared resource.
#define  NIMC_returnDataBufferNotEmptyError                 -70025 // The Return Data Buffer is not empty. Commands that expect data returned from the controller cannot be sent until the Return Data Buffer is cleared.
#define  NIMC_noBoardConfigInfoError                        -70019 // Controller configuration information is missing or corrupt.  A motion controller has not been configured at this Board ID.
#define  NIMC_packetLengthError                             -70013 // Command packet length is incorrect.  Your NI motion controller firmware may be out of date.
#define  NIMC_badBoardIDError                               -70012 // Illegal board ID. You must use the board ID assigned to your controller in Measurement & Automation Explorer.
#define  NIMC_badReturnDataBufferPacketError                -70011 // Corrupt readback data. The data returned by the motion controller is corrupt.
#define  NIMC_badResourceIDOrAxisError                      -70006 // An invalid axis number or other resource ID (vector space, encoder, I/O port, and so on) was used.
#define  NIMC_packetErrBitNotClearedError                   -70009 // Packet error bit not cleared by terminator (hardware error).
#define  NIMC_previousPacketError                           -70008 // The function called previous to this one is not supported by this type of controller.
#define  NIMC_CIPBitError                                   -70007 // A previous function is currently being executed, so the controller cannot accept this function until the previous function has completed.  If this problem persists, try putting a delay between the offending commands.
#define  NIMC_boardFailureError                             -70005 // Most likely, your controller is not installed or configured properly.  If this error persists when you know your controller is installed and configured properly, it indicates an internal hardware failure.
#define  NIMC_halfReturnDataBufferError                     -70004 // Partial readback packet. The data returned by the controller is incomplete. The kernel driver timed out after getting partial data.
#define  NIMC_noReturnDataBufferError                       -70003 // No data in the Return Data Buffer.  The kernel driver returns an error if it runs out of time waiting for the controller to return data to the Return Data Buffer. This error can also be returned if the power-up state of the controller has not been cleared.
#define  NIMC_currentPacketError                            -70002 // Either this function is not supported by this type of controller, or the controller received an incomplete command packet and cannot execute the function.
#define  NIMC_readyToReceiveTimeoutError                    -70001 // Ready to Receive Timeout. The controller is still not ready to receive commands after the specified timeout period. This error may occur if the controller is busy processing previous commands. If this error persists, even when the controller should not be busy, contact National Instruments.

#endif //___MotnErr_h___
