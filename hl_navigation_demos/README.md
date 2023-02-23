# Demonstrate the use of the tools with different ways to perform the same demonstration

## Demonstration

- Two Thymios starts at ((-0.5, 0.1), 0.0) and ((0.5, -0.1), 0.0) respectively.
- There is an obstacles of size 0.1 at (0, 0) 
- There is a wall around the area a square of size 3, centered at (0, 0)
- The Thymios have to travel between (1, 0) and (-1, 0) with a tolerance of 0.1 at 0.12 m/s


### Different implementations

### Naked C++ exec

#### Limitations
- no collisions
- perfect sensing
- not real-time
	
#### Depends on
- hl_navigation


### Naked Python exec

#### Limitations
- no collisions
- perfect sensing
- not real-time
	
#### Depends on
- hl_navigation
- hl_navigation_py

### C++ simulation

#### Limitations
- not real-time
	
#### Depends on
- hl_navigation
- hl_navigation_sim

### Python simulation

#### Limitations
- not real-time
	
#### Depends on
- hl_navigation
- hl_navigation_sim
- hl_navigation_sim_py (TBD)

### PyEnki simulation

#### Limitations
- perfect sensing
	
#### Depends on
- hl_navigation
- hl_navigation_py

### CoppeliaSim internal simulation

#### Limitations
- perfect sensing (but could be extended)
	
#### Depends on
- hl_navigation
- hl_navigation_coppeliasim


### CoppeliaSim ROS2 simulation

#### Limitations
- perfect sensing (but could be extended)
	
#### Depends on
- hl_navigation
- hl_navigation_ros


### Real robot ROS2 simulation

#### Limitations
- perfect sensing (but could be extended)
	
#### Depends on
- hl_navigation
- hl_navigation_ros
