# PyFlexMotion

PyFlexMotion provides Python wrapper layers around 
the National Instruments package Flex Motion.

API layers are provided at both low (direct API access) and 
high-level (Axis/Board/Object Oriented) abstractions which 
can be mixed.

All configuration is structured and can be specified with 
versioned `*.toml` motor configuration files.

Have a look at the example configurations in `flex_motion/example_configurations`.

## Installing

`pip install flex_motion` should work, but I recommend `git clone` 
followed by `pip install -e .` until I work out how to compile for other platforms.

## Getting Started

```python
from pathlib import Path
from flex_motion import Board

# build dictionary of hardware motion control boards
boards = Board.from_config(Path('path/to/my/config'))

# configure the board, attached trajectories, and axes
boards['A'].start()

# application code..
```

## System Requirements

Because of library requirements, you will need to install PyFlexMotion
into a 32-bit Python environment. Because of NI platform constraints,
only 32 **and** 64-bit installations of Windows will work.

PyFlexMotion uses CFFI as the interface layer to Flex Motion and ships
with the NI library files and the compiled C interface for the x86-64
CPU architecture. If you need to run on a different architecture, clone 
the repository and rebuild the CFFI bindings manually.   

### Getting 32-bit Python

If you are using Conda or Miniconda, you can ensure a 32-bit Python by
setting the `CONDA_FORCE_32BIT` flag.

Here's how this looks on `bash` and PowerShell.

```bash
$> set CONDA_FORCE_32BIT=1
$> conda create -n my_environment python=3.7

$> conda activate my_environment
```

In particular, you need to ensure that `CONDA_FORCE_32BIT` is set also when 
you *activate* the environment.

### Rebuilding the CFFI bindings

Navigate into the `flex_motion` package and run `compile_flex_motion.py`
in the interpreter you created. That should do it!

## General Advice

1. NEVER write fresh motion software on an assembled system, test and prototype your software 
   with motors detached so that you can avoid unnecessary hardware damage.
2. Double check electrical connections before working on motion software.
3. Double check your configuration before running software.
4. Try basic motions in NI Motion Assistant first, to work out any quirks. You can also get much 
   of the needed configuration from inside MAX or NI Motion Assistant.