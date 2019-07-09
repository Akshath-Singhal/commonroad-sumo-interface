# sumo id prefix
EGO_ID_START = 'egoVehicle'
# CommonRoad id prefixes
ID_DICT={'laneExt': 1, 'laneSpecial': 2, 'obstacleVehicle': 3, 'egoVehicle': 4}


class SumoCommonRoadConfig():
    verbose = False
    # simulation
    dt = 0.1  # length of simulation step of the interface
    delta_steps = 2  # number of sub-steps simulated in SUMO during every dt
    presimulation_steps = 10  # number of time steps before simulation with ego vehicle starts (not plotted)
    simulation_steps = 100  # number of simulated (and synchronized) time steps
    with_sumo_gui = False

    # plotting
    video_start = 1  # start of
    video_end = 100
    scenario_name = '<scenario_name>'
    # autoscale plot limits; plot_x1,plot_x2, plot_y1, plot_y2 only works if plot_auto is False
    plot_auto = True
    center_ego_vehicle = True  # video is centered at ego vehicle
    # axis limits of plot
    plot_x1 = 450
    plot_x2 = 550
    plot_y1 = 65
    plot_y2 = 1100
    figsize_x = 15
    figsize_y= 15
    window_width = 150
    window_height = 200

    # ego vehicle
    ego_veh_width = 2.0
    ego_veh_length = 5.0

    #ego vehicle sync parameters
    lanelet_check_time_window = int(2/dt)  # Time window to detect the lanelet change in seconds
    protection_margin = 2.0  # The absolute margin allowed between the planner position and ego position in SUMO
    consistency_window = 4  # Variable can be used  to force the consistency to certain number of steps
    lane_change_sync = False  # Used to limit the sync mechanism only to movexy
    lane_change_tol = 0.00  # tolerance for detecting start of lane change