"""
Minimal example, plugging in of trajectory planner required.
"""
from sumo2cr.interface.sumo_simulation import SumoSimulation
from sumo2cr.visualization.video import create_video
from scenarios.a9.scenario_config import Conf as conf
from copy import deepcopy

sumo_sim = SumoSimulation()
sumo_sim.initialize(conf)

output_folder = "/home/klischat/GIT_REPOS/sumo-interface/sumo2cr/tests"

for t in range(conf.simulation_steps):
    print('time step :' +str(t))
    ego_vehicles = sumo_sim.ego_vehicles
    commonroad_scenario = sumo_sim.commonroad_scenario_at_time_step(sumo_sim.current_time_step)
    print(sumo_sim.current_time_step)


    # plan trajectories for all ego vehicles
    for id, ego_vehicle in ego_vehicles.items():
        current_state = ego_vehicle.current_state

        # own implementation for testing - ego vehicle just stays in the initial position
        next_state = deepcopy(current_state)
        next_state.time_step = 1

        ego_vehicle.set_planned_trajectory([next_state])

    sumo_sim.simulate_step()

sumo_sim.stop()
create_video(sumo_sim, conf.video_start, conf.video_end, output_folder)


