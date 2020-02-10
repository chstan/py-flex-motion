from dataclasses import field, dataclass, InitVar, fields
from enum import Enum, Flag
from pathlib import Path
from typing import List, Optional, Dict, Tuple
import toml

from _flex_motion_cffi import ffi, lib

__all__ = (
    'XYZ',
    'Polarity',
    'SwitchPolarity',
    'Direction',

    'PID',
    'PIDRate',
    'PIDWhich',

    'Source', # "returnVector"

    'AccelerationAttrs',
    'OperatingMode',
    'ResourceControl',
    'StopModes',

    'StepperModeKind',
    'StepSize',
    'AxisKind',

    'DACLimitStatus',
    'AxisStatus',
    'AxisOrVectorSpace',
    'AxisFeedbackAndOutput',
    'AxisInhibit',
    'BlendStatus',
    'MotionStatus',
    'LimitType',
    'HomeDirection',
    'LoopMode',
    'BoardStatus',
    'NIMCCammingEnableData',
    'NIMCData',

    'ENCODER_CONTROL',
    'ENCODER_OFFSET', 'ADC_OFFSET', 'DAC_OFFSET', 'STEPPER_OFFSET',

    'AxisConfig',
    'BoardConfig',
)

ENCODER_OFFSET = 0x21
ADC_OFFSET = 0x51
DAC_OFFSET = 0x31
STEPPER_OFFSET = 0x41 # use 4, and 5 on two stepper boards

ENCODER_CONTROL = ENCODER_OFFSET - 1


@dataclass
class PID:
    kp: int
    ki: int
    ilim: int
    kd: int
    td: int
    kv: int
    aff: int
    vff: int

    def from_c(self, p_struct):
        _direct_map_fields(p_struct, self, ['kp', 'ki', 'ilim', 'kd', 'td', 'kv', 'aff', 'vff'])

    def into_c(self):
        p_struct = ffi.new('struct PID *')
        _direct_map_fields(self, p_struct, ['kp', 'ki', 'ilim', 'kd', 'td', 'kv', 'aff', 'vff'])
        return p_struct


@dataclass
class XYZ:
    x: int = 0
    y: int = 0
    z: int = 0


class AxisOrVectorSpace(Enum):
    MultipleAxes = 0x00
    MultipleVectorSpaces = 0x10

    Axis = 0x00
    VectorSpace = 0x10


class PIDRate(Enum):
    US_63 = 0
    US_125 = 1
    US_188 = 2
    US_250 = 3
    US_312 = 4
    US_375 = 5
    US_438 = 6
    US_500 = 7


@dataclass
class AxisFeedbackAndOutput:
    primary_encoder: int = None
    secondary_encoder: int = None
    primary_adc: int = None
    secondary_adc: int = None

    primary_output_stepper: int = None
    secondary_output_stepper: int = None
    primary_output_dac: int = None
    secondary_output_dac: int = None

    @property
    def is_closed_loop(self) -> bool:
        return bool(self.primary_adc is not None or self.primary_encoder is not None)

    def to_config_fields(self):
        assert self.primary_encoder is None or self.primary_adc is None
        assert self.secondary_encoder is None or self.secondary_adc is None
        assert self.primary_output_stepper is None or self.primary_output_dac is None
        assert self.secondary_output_stepper is None or self.secondary_output_dac is None

        primary_feedback = 0
        secondary_feedback = 0
        primary_output = 0
        secondary_output = 0

        if self.primary_encoder is not None:
            primary_feedback = ENCODER_OFFSET + self.primary_encoder
        if self.primary_adc is not None:
            primary_feedback = ADC_OFFSET + self.primary_adc
        if self.secondary_encoder is not None:
            secondary_feedback = ENCODER_OFFSET + self.secondary_encoder
        if self.secondary_adc is not None:
            secondary_feedback = ADC_OFFSET + self.secondary_adc

        if self.primary_output_stepper is not None:
            primary_output = STEPPER_OFFSET + self.primary_output_stepper
        if self.primary_output_dac is not None:
            primary_output = DAC_OFFSET + self.primary_output_dac
        if self.secondary_output_stepper is not None:
            secondary_output = STEPPER_OFFSET + self.secondary_output_stepper
        if self.secondary_output_dac is not None:
            secondary_output = DAC_OFFSET + self.secondary_output_dac

        return primary_feedback, secondary_feedback, primary_output, secondary_output


class Direction(Enum):
    Reverse = 1
    Forward = 0


class BlendStatus(Enum):
    BlendComplete = 1
    BlendPending = 0

    @classmethod
    def from_flags(cls, flags: int, axis_or_vector_space: AxisOrVectorSpace):
        bool_arr = [bool(flags & (2 ** (i + 1))) for i in range(6)]
        parsed = [cls.BlendComplete if b else cls.BlendPending for b in bool_arr]

        return parsed[:3] if axis_or_vector_space.value == AxisOrVectorSpace.VectorSpace.value else parsed


@dataclass
class AxisStatus:
    is_running: bool
    is_profile_complete: bool
    is_motor_on: bool
    has_following_error: bool
    is_limit_active: bool
    is_home_active: bool
    is_software_limit_active: bool
    is_velocity_above_treshold: bool
    has_breakpoint_occurred: bool
    has_home_been_found: bool
    has_index_been_found: bool
    has_high_speed_capture_occurred: bool
    direction: Direction
    is_blend_complete: bool
    is_move_complete: bool

    @classmethod
    def from_flags(cls, flags: int):
        return AxisStatus(
            is_running=bool(flags & (2 ** 0)),
            is_profile_complete=bool(flags & (2 ** 1)),
            is_motor_on=not bool(flags & (2 ** 2)),
            has_following_error=bool(flags & (2 ** 3)),
            is_limit_active=bool(flags & (2 ** 4)),
            is_home_active=bool(flags & (2 ** 5)),
            is_software_limit_active=bool(flags & (2 ** 6)),
            is_velocity_above_treshold=bool(flags & (2 ** 8)),
            has_breakpoint_occurred=bool(flags & (2 ** 9)),
            has_home_been_found=bool(flags & (2 ** 10)),
            has_index_been_found=bool(flags & (2 ** 11)),
            has_high_speed_capture_occurred=bool(flags & (2 ** 12)),
            direction=Direction.Reverse if (flags & (2 ** 13)) else Direction.Forward,
            is_blend_complete=bool(flags & (2 ** 14)),
            is_move_complete=bool(flags & (2 ** 15)),
        )

@dataclass
class HomeDirection:
    final: Direction = Direction.Forward
    search: Direction = Direction.Forward
    stop_edge: Direction = Direction.Forward

    def to_flags(self):
        return self.final.value + self.search.value * 2 + self.stop_edge.value * 4


class LimitType(Enum):
    Hardware = 0
    Software = 1


class LoopMode(Enum):
    OpenLoop = 0
    ClosedLoop = 1


class SwitchPolarity(Enum):
    ActiveLow = 1
    ActiveHigh = 0
    Inverting = 1
    NonInverting = 0


class AxisKind(Enum):
    Stepper = 0
    Servo = 1


class AxisInhibit(Enum):
    Enabled = 1
    Disabled = 0


class StepperModeKind(Enum):
    StepAndDirection = 1 # default mode
    CWAndCCW = 0


class Polarity(Enum):
    Inverting = 1 # default mode
    NonInverting = 0


class StepSize(Enum):
    FullStep = 0
    HalfStep = 0
    MicroStep = 0


def _coerce_enum(cls, instance):
    if isinstance(instance, cls):
        return instance

    try:
        return cls[instance]
    except KeyError:
        return cls(instance)


def _patchup(instance):
    """
    Looks for enum fields and coerces them to the appropriate real type.
    This allows us in TOML configuration to pass either string or integer values
    for enums and have typed enums on the dataclass body.
    :param instance:
    :return:
    """
    for f in fields(instance):
        try:
            field_type = f.type
        except TypeError:
            field_type = None

        if isinstance(field_type, type) and issubclass(field_type, Enum):
            value = getattr(instance, f.name)
            setattr(instance, f.name, _coerce_enum(field_type, value))




class PIDWhich(Enum):
    KP = 0
    KI = 1
    IL = 2
    KD = 3
    TD = 4
    KV = 5
    AFF = 6
    VFF = 7


class MotionStatus(Enum):
    Complete = 1
    Moving = 0

    @classmethod
    def from_flags(cls, flags: int):
        bool_arr = [bool(flags & (2 ** (i + 1))) for i in range(6)]
        return [cls.Complete if b else cls.Moving for b in bool_arr]


class DACLimitStatus(Enum):
    PositiveLimit = 1
    NotLimited = 0
    NegativeLimit = -1

    @classmethod
    def from_flags(cls, flags_high: int, flags_low: int):
        bool_high = [bool(flags_high & (2 ** (i + 1))) for i in range(6)]
        bool_low = [bool(flags_low & (2 ** (i + 1))) for i in range(6)]

        result = []
        for i in range(6):
            if bool_high[i]:
                result.append(cls.PositiveLimit)
            elif bool_low[i]:
                result.append(cls.NegativeLimit)
            else:
                result.append(cls.NotLimited)
        return result


class BoardStatus(Flag):
    Ready = 0x01
    DataInRDB = 0x02
    PacketErrorOrEmergencyStop = 0x10
    PowerUpReset = 0x20
    ModalErrorMessage = 0x40
    HardwareFailure = 0x80


class ResourceControl(Enum):
    Axis = 0x00
    VectorSpace = 0x10
    Encoder = 0x20
    DAC = 0x30
    StepOutput = 0x40
    ADC = 0x50
    AlternateEx = 0x60
    IOPort = 0x00
    PWM = 0x00
    Program = 0x00
    SecondaryEncoder = 0x70
    SecondaryDAC = 0x80
    SecondaryADC = 0x90
    Alternate = 0xA0
    AxisEx = 0xB0
    EncoderEx = 0xC0
    DACEx = 0xD0
    StepOutputEx = 0xE0
    ADCEx = 0xF0


class Source(Enum):
    """
    See NI Flex Motion "Input Vectors". This controls whether
    data is provided through the function call or from a stored
    value on the board.
    """
    Local = 0xFF


class AccelerationAttrs(Enum):
    Both = 0
    Acceleration = 1
    Deceleration = 2


class OperatingMode(Enum):
    Absolute = 0
    Relative = 1
    Velocity = 2
    RelativeToCapture = 3
    ModulusPosition = 4
    AbsoluteContouring = 5
    RelativeContouring = 6


class StopModes(Enum):
    Decelerate = 0
    Halt = 1
    Kill = 2



@dataclass
class Registry:
    device: int
    type: int
    pstart: int
    size: int

    def from_c(self, p_struct):
        _direct_map_fields(p_struct, self, ['device', 'type', 'pstart', 'size'])

    def to_c(self):
        p_struct = ffi.new('struct REGISTRY *')
        _direct_map_fields(self, p_struct, ['device', 'type', 'pstart', 'size'])
        return p_struct


@dataclass
class NIMCCammingEnableData:
    axis_index: int
    enable: bool
    position: float

    def from_c(self, p_struct):
        self.axis_index = p_struct.axisIndex
        self.enable = p_struct.enable > 0
        self.position = p_struct.position

    def to_c(self):
        p_struct = ffi.new('struct NIMC_CAMMING_ENABLE_DATA *')
        p_struct.axisIndex = self.axis_index
        p_struct.enable = 1 if self.enable else 0
        p_struct.position = self.position
        return p_struct


@dataclass
class NIMCData:
    long_data: int
    bool_data: bool
    float_data: float

    def from_c(self, p_struct):
        self.long_data = p_struct.longData
        self.bool_data = p_struct.boolData > 0
        self.float_data = p_struct.doubleData

    def to_c(self):
        p_struct = ffi.new('struct NIMC_DATA *')
        p_struct.longData = self.long_data
        p_struct.boolData = 1 if self.bool_data else 0
        p_struct.doubleData = self.float_data
        return p_struct


def _direct_map_fields(src, dst, fields):
    for field in fields:
        setattr(dst, field, getattr(src, field))


@dataclass
class AxisConfig:
    acceleration: int
    deceleration: int = None
    velocity: Optional[int] = None

    operating_mode: OperatingMode = OperatingMode.Absolute
    following_error: int = 32767

    enabled: bool = True
    kind: AxisKind = AxisKind.Stepper

    hardware_limit: Tuple[bool, bool] = (False, False,)
    software_limit: Tuple[int, int] = (None, None,)
    forward_limit_polarity: Polarity = Polarity.Inverting
    reverse_limit_polarity: Polarity = Polarity.Inverting

    counts_per_rev: int = 2000
    steps_per_rev: int = 2000

    inhibit_on_startup: AxisInhibit = AxisInhibit.Enabled
    inhibit_polarity: Polarity = Polarity.Inverting

    step_mode: StepSize = StepSize.FullStep
    step_polarity: Polarity = Polarity.Inverting
    step_mode_kind: StepperModeKind = StepperModeKind.StepAndDirection
    step_loop_mode: LoopMode = LoopMode.OpenLoop

    home_enabled: bool = False
    home_polarity: Polarity = Polarity.Inverting

    feedback: InitVar[Dict[str, int]] = None
    output: InitVar[Dict[str, int]] = None
    feedback_and_output: AxisFeedbackAndOutput = None

    pid_constants: InitVar[Dict[str, int]] = None
    pid: PID = None

    def __post_init__(self, feedback, output, pid_constants):
        feedback, output = feedback or {}, output or {}

        if self.deceleration is None:
            self.deceleration = self.acceleration

        _patchup(self)

        if pid_constants is not None:
            self.pid = PID(**pid_constants)

        self.feedback_and_output = AxisFeedbackAndOutput()
        if self.kind == AxisKind.Stepper:
            self.feedback_and_output.primary_encoder = feedback.get('primary')
            self.feedback_and_output.secondary_encoder = feedback.get('secondary')
            self.feedback_and_output.primary_output_stepper = output.get('primary')
            self.feedback_and_output.secondary_output_stepper = output.get('secondary')
        else:
            self.feedback_and_output.primary_adc = feedback.get('primary')
            self.feedback_and_output.secondary_adc = feedback.get('secondary')
            self.feedback_and_output.primary_output_dac = output.get('primary')
            self.feedback_and_output.secondary_output_dac = output.get('secondary')


@dataclass
class VectorSpaceConfig:
    pass


@dataclass
class BoardConfig:
    name: str
    board_id: int
    limit_input_polarity: SwitchPolarity
    home_input_polarity: SwitchPolarity

    enable_limits: bool = True
    enable_home_inputs: bool = False
    pid_rate: PIDRate = 'US_500'

    establish_position_on_startup: bool = False

    axes: List[AxisConfig] = field(default_factory=list)
    vector_spaces: List[VectorSpaceConfig] = field(default_factory=list)

    def __post_init__(self):
        _patchup(self)


def read_motor_configuration(path: Path):
    with open(str(path.absolute())) as f:
        raw_config = toml.load(f)

    flex_config = raw_config.get('flex_motion')

    flex_boards = {}
    for board_k, board in flex_config.items():
        if 'name' not in board:
            board['name'] = board_k

        board = BoardConfig(**board)
        board.axes = [AxisConfig(**axis) for axis in board.axes]
        flex_boards[board_k] = board

    return flex_boards
