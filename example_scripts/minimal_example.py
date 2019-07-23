"""
Minimal example, plugging in of trajectory planner required.
"""
from sumo2cr.interface.sumo_simulation import SumoSimulation
from sumo2cr.visualization.video import create_video
from scenarios.a9.scenario_config import Conf as conf

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["ZIM Projekt ZF4086007BZ8"]
__version__ = "1.0.0"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

sumo_sim = SumoSimulation()
sumo_sim.initialize(conf)

output_folder = "/path/to/output_folder"

for t in range(conf.simulation_steps):
    ego_vehicles = sumo_sim.ego_vehicles
    commonroad_scenario = sumo_sim.commonroad_scenario_at_time_step(sumo_sim.current_time_step)

    # plan trajectories for all ego vehicles
    for id, ego_vehicle in ego_vehicles.items():
        current_state = ego_vehicle.current_state
        ego_trajectory =  # plug in a trajectory planner here
        ego_vehicle.set_planned_trajectory(ego_trajectory)

    sumo_sim.simulate_step()

sumo_sim.stop()

create_video(sumo_sim, conf.video_start, conf.video_end, output_folder)

