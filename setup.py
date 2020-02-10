"""
Package setup.py
"""

from pathlib import Path
from setuptools import setup

about = {}
with open(str(Path(__file__).parent.absolute() / 'flex_motion' / 'version.py')) as fp:
    exec(fp.read(), about)

VERSION = about['VERSION']


def read_content(filepath):
    with open(filepath) as fobj:
        return fobj.read()


classifiers = [
    "Development Status :: 3 - Alpha",
    "Intended Audience :: Science/Research",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: GNU General Public License v3 or later (GPLv3+)",
    "Programming Language :: Python",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.7",
    "Programming Language :: Python :: Implementation :: CPython",
    "Natural Language :: English",
    'Operating System :: Microsoft :: Windows :: Windows 7',
    'Operating System :: Microsoft :: Windows :: Windows 8',
    'Operating System :: Microsoft :: Windows :: Windows 10',
    'Topic :: Scientific/Engineering',
    'Topic :: Software Development :: Libraries :: Python Modules',
]


long_description = (read_content("README.md"))

requirements = [
    'setuptools',
    'cffi',
    'toml',
]

extras_require = {
    'reST': ['Sphinx'],
}

setup(name='flex_motion',
      version=VERSION,
      description="Python bindings around National Instruments's Flex Motion",
      long_description=long_description,
      python_requires='>=3.7.0',
      author='Conrad Stansbury',
      author_email='chstansbury@gmail.com',
      url='#',
      classifiers=classifiers,
      packages=['flex_motion'],
      data_files=[],
      install_requires=requirements,
      include_package_data=True,
      extras_require=extras_require,
)
