# Track simulation
# Goal of the project:
Create a basic track simulation. Must be accurate within 5 seconds of actual lap times, should be 

# Current setup:
Main lap sim setup: read in the dxf file drawn by hand in oneshape, generate into one spline(track generator), read by track sim
track.m: trace the track
Includes brake simulation for generating brake biases across the track. 
All values are in imperical system (speed: ft/s, acceleration: ft/s^2)

# Work to be done:
Move away from dxf file, replace it with gps data from competition (gps_comp.csv)

MF13 lap time average: 151s
