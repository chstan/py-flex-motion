from typing import Type, List, Tuple, Optional

import arpes_daq.motors_server.flex_motion_low_level as ll
from arpes_daq.motors_server.flex_config import BoardConfig, BoardStatus, AxisConfig, AccelerationAttrs, PID, AxisKind, \
    LimitType


class Axis:
    board: Type['Board']
    axis_index: int
    config: AxisConfig

    def __init__(self, board: Type['Board'], axis_index: int, config=AxisConfig):
        self.board = board
        self.axis_index = axis_index
        self.config = config

    @property
    def pid(self) -> Optional[PID]:
        return self.config.pid

    @pid.setter
    def pid(self, new_pid):
        if new_pid is None:
            raise ValueError('You cannot provide no PID settings.')

        if self.is_stepper:
            raise ValueError(f'Stepper axes have no PID settings. Axis {self.axis_index} is misconfigured.')

        self.config.pid = new_pid
        ll.load_pid_parameters(self.board.board_id, self.axis_index, self.config.pid)

    @property
    def software_limit(self) -> Tuple[int, int]:
        raw_reverse, raw_forward = self.config.software_limit
        return (-2**30 if raw_reverse is None else raw_reverse,
                2**30 - 1 if raw_forward is None else raw_forward)

    @software_limit.setter
    def software_limit(self, values):
        self.config.software_limit = values
        ll.load_sw_lim_pos(self.board.board_id, self.axis_index, *self.software_limit)

    def configure_as_resource(self):
        ll.config_axis(self.board.board_id, self.axis_index, self.config.feedback_and_output)

    @property
    def is_stepper(self):
        return self.config.kind == AxisKind.Stepper

    @property
    def is_closed_loop(self):
        return self.config.feedback_and_output.is_closed_loop

    def startup(self):
        """
        Performs roughly these steps
        5. Set Stepper Loop Mode (stepper axes only)
        """
        if self.is_stepper:
            ll.config_step_mode_pol(self.board.board_id, self.axis_index,
                                    self.config.step_mode_kind, self.config.step_polarity)

        if self.config.feedback_and_output.primary_encoder is not None:
            ll.load_counts_rev(self.board.board_id, self.axis_index, self.config.counts_per_rev)

        if self.is_stepper:
            ll.load_steps_rev(self.board.board_id, self.axis_index, self.config.steps_per_rev)
            ll.set_stepper_loop_mode(self.board.board_id, self.axis_index, self.config.step_loop_mode)

        if not self.is_stepper and self.config.pid is not None:
            self.pid = self.config.pid  # applies pid settings to board via setter

        ll.set_op_mode(self.board.board_id, self.axis_index, self.config.operating_mode)
        ll.load_follow_err(self.board.board_id, self.axis_index, self.config.following_error)
        ll.load_velocity(self.board.board_id, self.axis_index, self.config.velocity)
        ll.load_acceleration(self.board.board_id, self.axis_index, AccelerationAttrs.Acceleration,
                             self.config.acceleration)
        ll.load_acceleration(self.board.board_id, self.axis_index, AccelerationAttrs.Deceleration,
                             self.config.deceleration)


class Board:
    board_id: int
    config: BoardConfig
    axes: List[Axis]

    def __init__(self, board_id, config):
        self.board_id = board_id
        self.config = config
        self.axes = [Axis(board=self, axis_index=i, config=axis_config)
                     for i, axis_config in enumerate(self.config.axes)]

    @property
    def startup(self):
        ll.clear_pu_status(self.board_id)
        status = ll.read_csr_rtn(self.board_id)
        if status & BoardStatus.PowerUpReset.value:
            raise ValueError(f'The board (board_id={self.board_id}) is in reset condition. Initialize in MAX first.')

        for axis in self.axes:
            axis.configure_as_resource()

        ll.enable_axes(self.board_id, list(range(len(self.axes))), self.config.pid_rate)

        for axis in self.axes:
            ll.load_sw_lim_pos(self.board_id, axis.axis_index, *axis.software_limit)

        ll.configure_inhibits(
            self.board_id,
            [a.config.inhibit_on_startup for a in self.axes],
            [a.config.inhibit_polarity for a in self.axes],
        )
        ll.set_limit_polarity(
            self.board_id,
            [a.config.forward_limit_polarity for a in self.axes],
            [a.config.reverse_limit_polarity for a in self.axes],
        )
        ll.set_home_polarity(self.board_id, [a.config.home_polarity for a in self.axes])

        def not_none(x): return x is not None
        ll.enable_limits(
            self.board_id, LimitType.Hardware,
            [not_none(a.config.hardware_limit[0]) for a in self.axes],
            [not_none(a.config.hardware_limit[1]) for a in self.axes]
        )
        ll.enable_limits(
            self.board_id, LimitType.Software,
            [not_none(a.config.software_limit[0]) for a in self.axes],
            [not_none(a.config.software_limit[1]) for a in self.axes]
        )
        ll.enable_home_inputs(self.board_id, [a.config.home_enabled for a in self.axes])

        for axis in self.axes:
            axis.startup()

        if self.config.establish_position_on_startup:
            self.establish_position()

    def establish_position(self):
        raise NotImplementedError()