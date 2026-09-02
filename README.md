# Track simulation
# Goal of the project
Create a basic track simulation, simulate the car going along the endurance track at competition and provide an optimal lap time.

# What is endurance?
Endurance is a part of the fsae competition. It is ten laps around a track with one driver change in the middle. Points are assigned based on time to complete the ten laps and how many cones were hit (not important for the sim, but fun to know). In 2026 competition, MF13 achieved time of 1509.953 seconds, which is about 151 seconds per lap.

# Current setup
[Insert diagram here]
LapSimMain.m:
TrackImperical.dxf:
TrackSpline.mat: 
calculateBrakeBias.m:
debug.m:
fsaetrack.xlsx:
fsaetrack_fixed.xlsx:
gps_comp.csv:
spline_points1.xlsx:
track.m:
track.xlsx:
trackGenerator.m:
vehicleParams.m:

Main track sim setup: read in the dxf file drawn by hand in oneshape, generate into one spline(track generator), read by track sim
track.m: trace the track
Includes brake simulation for reviewing brake biases across the track. 
All values are in imperical system (speed: ft/s, acceleration: ft/s^2)

# Work to be done
Move away from dxf file, replace it with gps data from competition (gps_comp.csv)

MF13 lap time average: 151s
