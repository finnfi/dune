# Finns note regarding the LSTS assignment in TTK22: 

The task is implemented in src/Tutorial/PlanVisit. 

The development and testing was done in the VirtualMachine provided in the lectures. 

To task is tested on the lauc-xplore-1, where it is activated when the vehicle is spawned (e.g.: ./dune -c lauv-xplore-1 -p Simulation). Waypoints can be added in the ini file of the vehicle. 

Compilation is done as mentioned in the lecture slides. 

DUNE: Unified Navigation Environment
======================================

DUNE: Unified Navigation Environment is a runtime environment for unmanned systems on-board software. It is used to write generic embedded software at the heart of the system, e.g. code or control, navigation, communication, sensor and actuator access, etc. It provides an operating-system and architecture independent platform abstraction layer, written in C++, enhancing portability among different CPU architectures and operating systems.

[![Build Status](https://travis-ci.org/LSTS/dune.svg?branch=master)](https://travis-ci.org/LSTS/dune)
[![Build status](https://ci.appveyor.com/api/projects/status/tdcdgyf408u4y0ng?svg=true)](https://ci.appveyor.com/project/zepinto/dune) 

