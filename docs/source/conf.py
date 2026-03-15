project = 'ME 405 Term Project - Mecha15'
author = 'Max Schecter, Gabe Haarberg'
release = '1.0'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.apidoc',
    'sphinx.ext.viewcode',
    'myst_parser',
    'sphinx_autodoc_typehints',
]

# Mock MicroPython-only imports so autodoc can run on a desktop
autodoc_mock_imports = [
    'pyb', 'micropython', 'ulab', 'ujson',
    'utime', 'machine',
    'serial',          # pyserial (desktop/host.py)
    'matplotlib',      # desktop/plot.py
]

# MicroPython adds ticks_us/ticks_ms/ticks_diff to the built-in `time` module.
# Patch them here so encoder.py and task_competition.py can be imported by autodoc.
try:
    import utime as _time
except ImportError:
    import time as _time
if not hasattr(_time, 'ticks_us'):
    _time.ticks_us   = lambda: 0
if not hasattr(_time, 'ticks_ms'):
    _time.ticks_ms   = lambda: 0
if not hasattr(_time, 'ticks_diff'):
    _time.ticks_diff = lambda a, b: a - b

# MicroPython's gc has mem_free(); CPython's does not.
import gc as _gc
if not hasattr(_gc, 'mem_free'):
    _gc.mem_free = lambda: 0

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

html_theme = 'furo'
html_static_path = ['_static']
html_title = project

autosummary_generate = True
autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}

# Source file extensions
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# Point autodoc at the project root so it can find all modules
import sys, os
sys.path.insert(0, os.path.abspath('../..'))          # project root (main.py, task_*.py, etc.)
sys.path.insert(0, os.path.abspath('../../drivers'))  # drivers/ package
sys.path.insert(0, os.path.abspath('../../desktop'))  # desktop/ tools

# Run apidoc during sphinx-build (replaces separate Makefile apidoc step)
_project_root = os.path.abspath('../..')
apidoc_modules = [
    {
        'path': _project_root,
        'destination': 'api/generated',
        'exclude_patterns': [
            os.path.join(_project_root, '.venv'),
            os.path.join(_project_root, 'docs'),
            os.path.join(_project_root, '__pycache__'),
            os.path.join(_project_root, 'main.py'),
        ],
        'separate_modules': True,
        'module_first': True,
    },
]
