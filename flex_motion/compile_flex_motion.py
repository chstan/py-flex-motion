from pathlib import Path
from cffi import FFI

flex_motion_ffi = FFI()
LIB_PATH = Path(__file__).parent.absolute() / 'lib'
INC_PATH = LIB_PATH

with open(str(INC_PATH / 'flexmotn_flat.h')) as f:
    flex_motion_ffi.cdef(f.read())

flex_motion_ffi.set_source('_flex_motion_cffi', f'#include "flexmotn_flat.h"',
                                       libraries=[str(INC_PATH / 'FlexMS32')],
                                       include_dirs=[str(INC_PATH)])

if __name__ == '__main__':
    flex_motion_ffi.compile(verbose=True)