from sumo_config.default import SumoCommonRoadConfig


class Conf(SumoCommonRoadConfig):
    #simulation
    dt = 0.1 # length of simulation step
    delta_steps = 1 # number of substeps simulated in SUMO during every dt
    command_line_input = False  # if true, ignore the simulation_step and ask in command line
    simulation_steps = 40

    # plotting
    video_start = 1
    video_end = simulation_steps
    scenario_name = 'tri_junction'
    with_sumo_gui = False
    plot_auto = False  #plot_x1,plot_x2, plot_y1, plot_y2 only works if plot_auto is False
    center_ego_vehicle = False
    plot_x1 = 300     #plot_x1 and plot_x2 defines the range of plotted scnario in x axis
    plot_x2 = 360
    plot_y1 = 190 #plot_y1 and plot_y2 defines the range of plotted scnario in y axis
    plot_y2 = 235
    figsize_x = 10
    figsize_y= 10
    window_width = 70
    window_height = 70

    # ego vehicle
    presimulation_steps = 105
    ego_veh_width = 1.8
    ego_veh_length = 4.5
