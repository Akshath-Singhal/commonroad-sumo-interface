"""
Converts net.xml to Commonroad xml files and creates rou files for simulation.
"""

import matplotlib.pyplot as plt
from typing import List
from commonroad.visualization.draw_dispatch_cr import draw_object
from sumo2cr.maps.sumo_scenario import ScenarioWrapper
from commonroad.common.util import Interval
from commonroad.scenario.trajectory import State
import os

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["ZIM Projekt ZF4086007BZ8"]
__version__ = "1.0.0"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

scenario_folder = os.path.join(os.path.dirname(__file__),'../scenarios')

net_file = os.path.join(scenario_folder, 'a9/a9.net.xml')
sumo_cfg_file = os.path.join(scenario_folder, 'a9/a9.sumo.cfg')

# parameters
dt = 0.1
convert_map = True
n_vehicles_max:int = 30
veh_per_second = 50
n_ego_vehicles:int = 1
ego_ids:List[int] = []
initial_states:List[State] = []
ego_start_time:int=10
departure_time_ego = 3
departure_interval_vehicles = Interval(0,20)

if convert_map:
    # generate commonroad map + rou file + cfg file
    scenario = ScenarioWrapper.full_conversion_from_net(net_file, dt, n_vehicles_max, n_ego_vehicles, ego_ids, ego_start_time, departure_time_ego, departure_interval_vehicles,veh_per_second)
else:
    # generate rou file
    scenario = ScenarioWrapper.recreate_route_file(sumo_cfg_file, dt, n_vehicles_max, n_ego_vehicles, ego_ids, ego_start_time, departure_time_ego, departure_interval_vehicles,veh_per_second)

plt.figure(figsize=(25, 25))
draw_object(scenario.lanelet_network)

plt.autoscale()
plt.axis('equal')
plt.xlim([290,380])
plt.ylim([195,250])
plt.show()