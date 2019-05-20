from config.default import SumoCommonRoadConfig

class Conf(SumoCommonRoadConfig):
    verbose = True
    #simulation
    dt = 0.1 # length of simulation step
    delta_steps = 1 # number of substeps simulated in SUMO during every dt
    command_line_input = False  # if true, ignore the simulation_step and ask in command line
    simulation_steps = 70
    with_sumo_gui = False

    # plotting
    video_start = 10
    video_end = simulation_steps
    scenario_name = 'a9'
    plot_auto = False  #plot_x1,plot_x2, plot_y1, plot_y2 only works if plot_auto is False
    focus_ego_vehicle = True
    plot_x1 = 50     #plot_x1 and plot_x2 defines the range of plotted scnario in x axis
    plot_x2 = 300
    plot_y1 = -10 #plot_y1 and plot_y2 defines the range of plotted scnario in y axis
    plot_y2 = 600
    figsize_x = 70/5#15
    figsize_y= 40/5#20
    window_width = 70 #70
    window_height = 19

    # ego vehicle
    presimulation_steps = 120
    ego_veh_width = 1.674
    ego_veh_length = 4.298

    #ego vehicle sync parameters
    lanelet_check_time_window = int(2/dt) #Time window to detect the lanelet change in seconds
    protection_margin = 2.0  # The absolute margin allowed between the planner position and ego position in SUMO
    consistency_window = 4  # Variable can be used  to force the consistency to certain number of steps
    lane_change_sync = False  # Used to limit the sync mechanism only to movexy
    lane_change_tol = 0.00  # tolerance for detecting start of lane change