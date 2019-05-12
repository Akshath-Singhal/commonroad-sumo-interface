# SUMO - CommonRoad Interface

This interface couples the framework for motion planning of automated vehicles based on [CommonRoad_io](https://pypi.org/project/commonroad-io/) and the traffic simulator [SUMO](https://sumo.dlr.de).

# Prerequisites
The package is written in Python 3.6 and tested on Linux.
Install required packages:
```console
cd sumo-interface
pip install -r requirements.txt
sudo apt-get install ffmpeg
```

## Install SUMO

Clone a customized version of SUMO for smooth lane changes from https://gitlab.lrz.de/ge36pob/sumo and check out branch `smooth-lane-change`.
For installation we recommend building with:
```
cd sumo
mkdir build && cd build
cmake ..
make -j8
```
More options on the installation can be found here: https://sumo.dlr.de/wiki/Installing/Linux_Build .

## Configure SUMO and local environment
1. copy the file `pathConfig_DEFAULT.py` and rename it to `pathConfig.py`. Add local paths, if required.

# Integrating the SUMO-interface into CommonRoad
- All scenarios are stored in `/scenarios`. All files for each scenario must be named consistently with the same scenario name as the folder.
- the default config file with all options for the simulation can be found in `/config/default.py`.

## Convert maps
Before using the interface, a map needs to be available in CommonRoad xml and SUMO's .net.xml format. Currently only the conversion from .net.xml to commonroad format is implemented. For an example see `example_scripts/create_scenario_files.py`.

## Create .rou.xml files
.rou.xml files specify all vehicles for the simulation. An introduction can be found here: https://sumo.dlr.de/wiki/Tutorials/quick_start#Traffic_demand.
To use the interface for planning with ego vehicles, the vehicle ID in the .rou.xml file needs to start with the prefix `egoVehicle`.
A simple script to create .rou.xml files using https://sumo.dlr.de/wiki/Tools/Trip is implemented in the function
`ScenarioWrapper.recreate_route_file()`. See `example_scripts/create_scenario_files.py` for an example.

## Configure SUMO and sumo-interface
To use smooth lane changes, add
~~~~
<processing>
	<time-to-teleport value="-1"/>
	<lanechange-output value="output_lanechange.txt"/>
	<lanechange.duration value="10"/>
	<lanechange.smooth-change value="1"/>
</processing>
~~~~
to the .sumo.cfg file of the scenario (activated by default for scenarios generated by the class `ScenarioWrapper`).

Further options for the interface can be found in the sumo-interface config file `sumo-interface/config/default.py`.

## Minimal example for using the interface:
The file `example_scripts/minimal_example.py` gives an example how the interface can be integrated into an existing trajectory planner. For testing, the scenarios `cross_example_map` and `traci_tls` are recommended.
Plug in a trajectory planner and run:
```python
from sumo2cr.interface.sumo_simulation import SumoSimulation
from sumo2cr.visualization.video import create_video
from scenarios.a9.scenario_config import Conf as conf

sumo_sim = SumoSimulation()
sumo_sim.initialize(conf)

output_folder = "/path/to/output_folder"

for t in range(conf.simulation_steps):
    ego_vehicles = sumo_sim.ego_vehicles
    commonroad_scenario = sumo_sim.commonroad_scenario_at_time_step(sumo_sim.current_time_step)

    # plan trajectories for all ego vehicles
    for id, ego_vehicle in ego_vehicles.items():
        current_state = ego_vehicle.current_state
        ego_trajectory =  # plug in trajectory planner here
        ego_vehicle.set_planned_trajectory(ego_trajectory)

    sumo_sim.simulate_step()

create_video(sumo_sim, conf.video_start, conf.video_end, output_folder)
```
By default, the videos are created in `\videos`.
