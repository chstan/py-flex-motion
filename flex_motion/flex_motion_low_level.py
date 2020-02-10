import functools
from functools import wraps
from typing import Union, List, Any

from _flex_motion_cffi import ffi, lib

from .flex_config import *


def axis_list_to_bitmap(axes: List[int]):
    """
    List of numeric axes, like [0, 2, 3] to the equivalent NI bitmap.
    NI uses 1 indexing and we use 0, so this is equivalent to
    2 ** 1 + 2 ** 3 + 2 ** 4
    :param axes:
    :return:
    """

    return sum(2 ** (x + 1) for x in axes)


def axis_bool_to_bitmap(axis_bools: List[bool]):
    return axis_list_to_bitmap([i for i, attr in enumerate(axis_bools) if attr])


def axis_bool_attr_to_bitmap(axis_attrs: List[Any]):
    return axis_list_to_bitmap([i for i, attr in enumerate(axis_attrs) if bool(attr.value)])


def wrap_stdcall_raise(c_wrapper_func):
    @wraps(c_wrapper_func)
    def wrapper(*args, **kwargs):
        return_value = c_wrapper_func(*args, **kwargs)

        if isinstance(return_value, int):
            return_code = return_value
            return_value = None
        else:
            return_code, return_value = return_value

        if return_code != 0:
            raise Exception(f'FFI Error Code {return_code} in {c_wrapper_func.__name__}')

        return return_value

    return wrapper


@wrap_stdcall_raise
def read_csr_rtn(board_id: int) -> int:
    with ffi.new('unsigned short *') as p_csr:
        status = lib.flex_read_csr_rtn(board_id, p_csr)
        value = p_csr[0]

    return status, value


@wrap_stdcall_raise
def read_csr_rtn(board_id: int) -> int:
    with ffi.new('unsigned short *') as p_csr:
        status = lib.flex_read_csr_rtn(board_id, p_csr)
        value = p_csr[0]

    return status, value


@wrap_stdcall_raise
def load_acceleration(board_id: int, axis: int, acceleration_type: AccelerationAttrs,
                      acceleration: int, data_source=Source.Local.value):
    return lib.flex_load_acceleration(board_id, axis, acceleration_type.value, acceleration, data_source)


@wrap_stdcall_raise
def load_rpsps(board_id: int, axis: int, acceleration_type: AccelerationAttrs,
               rpsps: float, data_source=Source.Local.value):
    return lib.flex_load_rpsps(board_id, axis, acceleration_type.value, rpsps, data_source)


@wrap_stdcall_raise
def load_scurve_time(board_id: int, axis: int, scurve_time: int, data_source=Source.Local.value):
    return lib.flex_load_scurve_time(board_id, axis, scurve_time, data_source)


@wrap_stdcall_raise
def load_velocity(board_id: int, axis: int, velocity: int, data_source=Source.Local.value):
    return lib.flex_load_velocity(board_id, axis, velocity, data_source)


@wrap_stdcall_raise
def load_target_pos(board_id: int, axis: int, position: int, data_source=Source.Local.value):
    return lib.flex_load_target_pos(board_id, axis, position, data_source)


@wrap_stdcall_raise
def set_op_mode(board_id: int, axis: int, op_mode: OperatingMode, data_source=Source.Local.value):
    return lib.flex_set_op_mode(board_id, axis, op_mode.value, data_source)


@wrap_stdcall_raise
def load_follow_err(board_id: int, axis: int, following_error_trip_point: int, data_source=Source.Local.value):
    return lib.flex_load_follow_err(board_id, axis, following_error_trip_point, data_source)


@wrap_stdcall_raise
def load_rpm(board_id: int, axis_or_vector_space: int, rpm: float, data_source=Source.Local.value):
    return lib.flex_load_rpm(board_id, axis_or_vector_space, rpm, data_source)


@wrap_stdcall_raise
def load_rpm_thresh(board_id: int, axis: int, rpm_thresh: float, data_source=Source.Local.value):
    return lib.flex_load_rpm_thresh(board_id, axis, rpm_thresh, data_source)


@wrap_stdcall_raise
def load_vel_thresh(board_id: int, axis: int, vel_thresh: int, data_source=Source.Local.value):
    return lib.flex_load_vel_thresh(board_id, axis, vel_thresh, data_source)


@wrap_stdcall_raise
def load_velocity_override(board_id: int, which_axis_or_vector_space: int,
                           override_percentage: float, data_source=Source.Local.value):
    return lib.flex_load_velocity_override(board_id, which_axis_or_vector_space, override_percentage, data_source)


@wrap_stdcall_raise
def start(board_id: int, resource: ResourceControl, axes: Union[int, List[int]]):
    if not isinstance(axes, int):
        axes = functools.reduce(lambda a1, a2: a1 | a2, axes)

    return lib.flex_start(board_id, resource.value, axes)


@wrap_stdcall_raise
def read_pos_rtn(board_id: int, axis: int) -> int:
    with ffi.new('int *') as p_position:
        status = lib.flex_read_pos_rtn(board_id, axis, p_position)
        value = p_position[0]
    return status, value


@wrap_stdcall_raise
def read_target_pos_rtn(board_id: int, axis: int) -> int:
    with ffi.new('int *') as p_target_position:
        status = lib.flex_read_target_pos_rtn(board_id, axis, p_target_position)
        value = p_target_position[0]
    return status, value


@wrap_stdcall_raise
def read_dac_rtn(board_id: int, which_axis_or_dac: int) -> int:
    with ffi.new('short *') as p_dac_output:
        status = lib.flex_read_dac_rtn(board_id, which_axis_or_dac, p_dac_output)
        value = p_dac_output[0]
    return status, value


@wrap_stdcall_raise
def read_dac_limit_status_rtn(board_id: int) -> List[DACLimitStatus]:
    p_positive_status = ffi.new('unsigned char *')
    p_negative_status = ffi.new('unsigned char *')
    status = lib.flex_read_dac_rtn(board_id, p_positive_status, p_negative_status)
    value_high, value_low = p_positive_status[0], p_negative_status[0]
    return status, DACLimitStatus.from_flags(value_high, value_low)


@wrap_stdcall_raise
def read_velocity_rtn(board_id: int, which_axis_or_vector_space: int) -> int:
    with ffi.new('int *') as p_velocity:
        status = lib.flex_read_velocity_rtn(board_id, which_axis_or_vector_space, p_velocity)
        value = p_velocity[0]
    return status, value


@wrap_stdcall_raise
def read_rpm_rtn(board_id: int, axis: int) -> float:
    with ffi.new('double *') as p_rpm:
        status = lib.flex_read_rpm_rtn(board_id, axis, p_rpm)
        value = p_rpm[0]
    return status, value


@wrap_stdcall_raise
def read_axis_status_rtn(board_id: int, axis: int) -> AxisStatus:
    with ffi.new('unsigned short *') as p_axis_status:
        status = lib.flex_read_axis_status_rtn(board_id, axis, p_axis_status)
        value = p_axis_status[0]
    return status, AxisStatus.from_flags(value)


@wrap_stdcall_raise
def read_blend_status_rtn(board_id: int, axis_or_vector_space: AxisOrVectorSpace) -> List[BlendStatus]:
    with ffi.new('unsigned short *') as p_blend_status:
        status = lib.flex_read_blend_status_rtn(board_id, axis_or_vector_space.value, p_blend_status)
        value = p_blend_status[0]
    return status, BlendStatus.from_flags(value, axis_or_vector_space)


@wrap_stdcall_raise
def stop_motion(board_id: int, axis_or_vector_space: int, stop_type: StopModes, axes_or_vector_spaces=0):
    if not isinstance(axes_or_vector_spaces, int):
        axes_or_vector_spaces = functools.reduce(lambda axis1, axis2: axis1 | axis2, axes_or_vector_spaces)

    return lib.flex_stop_motion(board_id, axis_or_vector_space, stop_type.value, axes_or_vector_spaces)


@wrap_stdcall_raise
def read_follow_error_rtn(board_id: int, which_axis_or_vector_space: int) -> int:
    with ffi.new('short *') as p_following_error:
        status = lib.flex_read_follow_error_rtn(board_id, which_axis_or_vector_space, p_following_error)
        value = p_following_error[0]
    return status, value


@wrap_stdcall_raise
def read_mcs_status_rtn(board_id: int) -> List[MotionStatus]:
    with ffi.new('unsigned short *') as p_motion_complete:
        status = lib.flex_read_mcs_status_rtn(board_id, p_motion_complete)
        value = p_motion_complete[0]
    return status, MotionStatus.from_flags(value)


@wrap_stdcall_raise
def read_error_msg_rtn(board_id: int):
    p_command_id = ffi.new('unsigned short *')
    p_resource_id = ffi.new('unsigned short *')
    p_error_code = ffi.new('int *')
    status = lib.flex_read_error_msg_rtn(board_id, p_command_id, p_resource_id, p_error_code)
    return status, (p_command_id[0], p_resource_id[0], p_error_code[0],)


@wrap_stdcall_raise
def read_error_msg_detail_rtn(board_id: int):
    p_command_id = ffi.new('unsigned short *')
    p_resource_id = ffi.new('unsigned short *')
    p_error_code = ffi.new('int *')
    p_line_number = ffi.new('unsigned short *')
    p_file_number = ffi.new('unsigned short *')
    status = lib.flex_read_error_msg_rtn(board_id, p_command_id, p_resource_id, p_error_code, p_line_number, p_file_number)
    return status, (p_command_id[0], p_resource_id[0], p_error_code[0], p_line_number[0], p_file_number[0])


@wrap_stdcall_raise
def clear_pu_status(board_id: int):
    return lib.flex_clear_pu_status(board_id)


@wrap_stdcall_raise
def config_axis(board_id: int, axis: int, feedback_and_output: AxisFeedbackAndOutput):
    return lib.flex_config_axis(board_id, axis, *feedback_and_output.to_config_fields())


@wrap_stdcall_raise
def enable_axes(board_id: int, axis_list: List[int], pid_rate: PIDRate = PIDRate.US_500):
    """
    Defaults to the slowest PID rate in order to try to avoid errors due to encoders or servos.
    :param board_id:
    :param axis_list:
    :param pid_rate:
    :return:
    """
    return lib.flex_enable_axes(board_id, 0, pid_rate.value, axis_list_to_bitmap(axis_list))


@wrap_stdcall_raise
def configure_inhibits(board_id, axis_inhibit_list: List[AxisInhibit], axis_polarity_list: List[Polarity]):
    return lib.flex_configure_inhibits(
        board_id,
        axis_bool_attr_to_bitmap(axis_inhibit_list),
        axis_bool_attr_to_bitmap(axis_polarity_list),
    )


@wrap_stdcall_raise
def set_limit_polarity(board_id: int, forward_polarity_map: List[Polarity], reverse_polarity_map: List[Polarity]):
    return lib.flex_set_limit_polarity(
        board_id, axis_bool_attr_to_bitmap(forward_polarity_map),
        axis_bool_attr_to_bitmap(reverse_polarity_map)
    )


@wrap_stdcall_raise
def set_home_polarity(board_id: int, home_polarity_map: List[Polarity]):
    return lib.flex_set_home_polarity(board_id, axis_bool_attr_to_bitmap(home_polarity_map))


@wrap_stdcall_raise
def enable_limits(board_id: int, limit_type: LimitType, forward_limit_map: List[bool], reverse_limit_map: List[bool]):
    return lib.flex_enable_limits(
        board_id, limit_type.value,
        axis_bool_to_bitmap(forward_limit_map), axis_bool_to_bitmap(reverse_limit_map)
    )


@wrap_stdcall_raise
def load_sw_lim_pos(board_id: int, axis: int, forward_limit: int, reverse_limit: int, data_source=Source.Local.value):
    return lib.flex_load_sw_lim_pos(board_id, axis, forward_limit, reverse_limit, data_source)


@wrap_stdcall_raise
def enable_home_inputs(board_id: int, home_map: List[bool]):
    return lib.flex_enable_home_inputs(board_id, axis_bool_to_bitmap(home_map))


@wrap_stdcall_raise
def config_step_mode_pol(board_id: int, axis_or_stepper: int, mode: StepperModeKind = StepperModeKind.StepAndDirection,
                         polarity: Polarity = Polarity.Inverting):
    return lib.flex_config_step_mode_pol(
        board_id, axis_or_stepper, mode.value + 4 * polarity.value)


@wrap_stdcall_raise
def load_counts_rev(board_id: int, axis: int, counts_per_rev: int):
    return lib.flex_load_counts_rev(board_id, axis, counts_per_rev)


@wrap_stdcall_raise
def load_steps_rev(board_id: int, axis: int, steps_per_rev: int):
    return lib.flex_load_steps_rev(board_id, axis, steps_per_rev)


@wrap_stdcall_raise
def load_pid_parameters(board_id: int, axis: int, pid_values: PID, data_source=Source.Local.value):
    p_pid_values = pid_values.into_c()
    return lib.flex_load_pid_parameters(board_id, axis, p_pid_values, data_source)


@wrap_stdcall_raise
def load_single_pid_parameter(board_id: int, axis: int, pid_which: PIDWhich, value: int, data_source=Source.Local.value):
    return lib.flex_load_single_pid_parameter(board_id, axis, pid_which.value, value, data_source)


@wrap_stdcall_raise
def set_stepper_loop_mode(board_id: int, axis: int, loop_mode: LoopMode = LoopMode.OpenLoop):
    return lib.flex_load_pid_parameters(board_id, axis, loop_mode.value)


@wrap_stdcall_raise
def find_home(board_id: int, axis: int, direction: HomeDirection):
    return lib.flex_find_home(board_id, axis, direction.to_flags())


@wrap_stdcall_raise
def find_index(board_id: int, axis: int, direction: Direction = Direction.Forward, offset: int = 0):
    return lib.flex_find_index(board_id, axis, direction.value, offset)


@wrap_stdcall_raise
def reset_encoder(board_id: int, encoder: int, position: int, data_source=Source.Local.value):
    return lib.flex_reset_encoder(board_id, encoder, position, data_source)


@wrap_stdcall_raise
def reset_pos(board_id: int, axis: int, position_primary: int, position_secondary: int, data_source=Source.Local.value):
    return lib.flex_reset_pos(board_id, axis, position_primary, position_secondary, data_source)


@wrap_stdcall_raise
def load_vel_tc_rs(board_id: int, axis: int, filter_time: int, run_stop_threshold: int, data_source=Source.Local.value):
    return lib.flex_load_vel_tc_rs(board_id, axis, filter_time, run_stop_threshold, data_source)


@wrap_stdcall_raise
def load_vs_ps(board_id: int, vector_space: int, xyz: XYZ, data_source=Source.Local.value):
    return lib.flex_load_vs_ps(board_id, vector_space, xyz.x, xyz.y, xyz.z, data_source)

