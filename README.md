# DroneSnapper-public
This repository contains Julia flight simulator developed under TU Delft honours project of developing navigation algorithms for the DroneSnapper drone interceptor.

**Licensed under Mozilla Public License 2.0** see LICENSE file for the full license text

### Julia v1
This folder contains the first version of the flight simulator. Particular files are responsible for:
1. DsSim2 --- sets up and runs simulation as well saves its output, **main file to run**
2. SimCore --- this is a module that provides single function - it contains an actual simulation loop as well as means to set it up computationally and some saving features
3. coefLoader --- loads airplane aerodynamic data from external files and provides interpolation functions
4. dataTypes --- single source of Julia struct(s)
5. forcesCalculator --- provides functions calculating forces and moments acting on the airplane mid-flight
6. mathhelpers --- provides 4th order Runge-Kutta integration function
7. utils_print --- plotting and command line printing functions
8. *Autopilot --- currently not available as contained proprietary information, release planned
