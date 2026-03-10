# API Reference

Auto-generated from source docstrings.

## Main

`main.py` is the entry-point script. It initializes all hardware drivers, creates
task instances, wires up inter-task `Share`/`Queue` objects, registers tasks with
the `cotask` scheduler, and runs the main scheduler loop. Because it executes
top-level hardware code at import time it is not auto-documented here; see
[Software Architecture](software.md) for a full description and code excerpts.

## Controller

```{eval-rst}
.. automodule:: controller
   :members:
   :undoc-members:
   :show-inheritance:
```

## Constants

```{eval-rst}
.. automodule:: constants
   :members:
   :undoc-members:
```

## Tasks

### Motor Task

```{eval-rst}
.. automodule:: task_motor
   :members:
   :undoc-members:
   :show-inheritance:
```

### Line Follow Task

```{eval-rst}
.. automodule:: task_line_follow
   :members:
   :undoc-members:
   :show-inheritance:
```

### Reflectance Task

```{eval-rst}
.. automodule:: task_reflectance
   :members:
   :undoc-members:
   :show-inheritance:
```

### IMU Task

```{eval-rst}
.. automodule:: task_imu
   :members:
   :undoc-members:
   :show-inheritance:
```

### Observer Task

```{eval-rst}
.. automodule:: task_observer
   :members:
   :undoc-members:
   :show-inheritance:
```

### Competition Task

```{eval-rst}
.. automodule:: task_competition
   :members:
   :undoc-members:
   :show-inheritance:
```

### User Interface Task

```{eval-rst}
.. automodule:: task_user
   :members:
   :undoc-members:
   :show-inheritance:
```

## Scheduling & Shared Memory

### cotask

```{eval-rst}
.. automodule:: cotask
   :members:
   :undoc-members:
   :show-inheritance:
```

### task_share

```{eval-rst}
.. automodule:: task_share
   :members:
   :undoc-members:
   :show-inheritance:
```

## Drivers

### Motor Driver

```{eval-rst}
.. automodule:: motor
   :members:
   :undoc-members:
   :show-inheritance:
```

### Encoder Driver

```{eval-rst}
.. automodule:: encoder
   :members:
   :undoc-members:
   :show-inheritance:
```

### Reflectance Sensor Driver

```{eval-rst}
.. automodule:: reflectance
   :members:
   :undoc-members:
   :show-inheritance:
```

### IMU Driver

```{eval-rst}
.. automodule:: imu
   :members:
   :undoc-members:
   :show-inheritance:
```

## Desktop Tools

### Host (Serial Data Collector)

```{eval-rst}
.. automodule:: host
   :members:
   :undoc-members:
   :show-inheritance:
```

### Plot

```{eval-rst}
.. automodule:: plot
   :members:
   :undoc-members:
   :show-inheritance:
```
