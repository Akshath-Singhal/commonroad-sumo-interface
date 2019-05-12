from commonroad.visualization.draw_dispatch_cr import draw_object
from matplotlib import pyplot as plt, animation as manimation
from config.plot_params import draw_params_obstacle, draw_params_ego
import warnings
import os
import time
from sumo2cr.interface.sumo_simulation import SumoSimulation

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["ZIM Projekt ZF4086007BZ8"]
__version__ = "1.0.0"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

def create_video(sumo_sim: SumoSimulation, video_start:int, video_end:int, out_folder:str):
    """
    Create video from simulated scenario.

    :param sumo_sim: the sumo simulation instance
    :param video_start: initial time step of the video
    :param video_stop: final step of the video
    :param out_folder: folder for video output
    :return:
    """
    if video_start < 1 or video_end > sumo_sim.current_time_step:
        warnings.warn('<create_video>: Requesting time interval [{},{}] that has not been simulated yet. Choose from interval [1,{}]'
                      .format(video_start,video_end,sumo_sim.current_time_step),stacklevel=2)
        return

    if not os.path.isdir(out_folder):
        raise ValueError('out_folder {} does not exist.'.format(out_folder))

    timestamp = time.strftime("%Y-%m-%d-%H%M%S")
    video_file_name = os.path.join(out_folder, str(sumo_sim.scenarios.scenario_name) +'_' + timestamp + '.mp4')
    plot_dir = os.path.join(out_folder,timestamp)
    os.makedirs(plot_dir)

    frames_per_second = 1/(sumo_sim.dt)

    FFMpegWriter = manimation.writers['ffmpeg']
    writer = FFMpegWriter(fps=frames_per_second)

    fig = plt.figure(figsize=(sumo_sim.conf.figsize_x, sumo_sim.conf.figsize_y))
    plt.tight_layout()
    plt.ioff()

    with writer.saving(fig, video_file_name, 100):
        for t in range (video_start, video_end):
            if sumo_sim.conf.verbose is True:
                _printProgressBar(t + 1 - video_start, video_end - video_start + 1, prefix ='Writing video:', suffix ='Complete')
            plt.cla()
            draw_plot(sumo_sim, t)
            writer.grab_frame()
            plt.savefig(plot_dir + '/' + str(t) + '.svg')


def draw_plot(sumo_sim: SumoSimulation, time_step:int=None, _draw_params_ego=None, _draw_params_obstacle=None):
    """
    create a plot of the current simulation
    :param sumo_sim: instance of simulation
    :param scenario_name: name of the scenario where simulation happen

    :return:
    """
    if time_step is None:
        time_step = sumo_sim.current_time_step

    # add time interval to params
    if _draw_params_ego is None:
        _draw_params_ego = draw_params_ego
        _draw_params_ego.update({'time_begin': 0, 'time_end': sumo_sim.conf.video_end - sumo_sim.conf.video_start})
    if _draw_params_obstacle is None:
        _draw_params_obstacle = draw_params_obstacle
        _draw_params_obstacle.update({'time_begin': 0, 'time_end': sumo_sim.conf.video_end - sumo_sim.conf.video_start})

    ego_veh = sumo_sim.get_ego_obstacles(time_step)
    scenario = sumo_sim.commonroad_scenario_at_time_step(time_step)

    # set plot limits
    if (sumo_sim.conf.plot_auto == False):
        if sumo_sim.conf.focus_ego_vehicle is True and len(ego_veh) > 0:
            pos = ego_veh[0].initial_state.position
            ori = sumo_sim.ego_vehicles[ego_veh[0].obstacle_id].initial_state.orientation# - np.deg2rad(90)
            # ori=0

            plot_limits = [-sumo_sim.conf.window_width / 2 + pos[0], sumo_sim.conf.window_width / 2 + pos[0],
                           -sumo_sim.conf.window_height / 2 + pos[1], sumo_sim.conf.window_height / 2 + pos[1]]
            for ego_veh_i in ego_veh:
                ego_veh_i.translate_rotate(-pos,-ori)
                ego_veh_i.translate_rotate(pos, 0)

            scenario.translate_rotate(-pos,-ori)
            scenario.translate_rotate(pos, 0)
        else:
            plot_limits = [sumo_sim.conf.plot_x1, sumo_sim.conf.plot_x2,
                           sumo_sim.conf.plot_y1, sumo_sim.conf.plot_y2]
    else:
        plot_limits = None

    # draw scenario and ego vehicles
    draw_object(scenario, draw_params=_draw_params_obstacle, plot_limits=plot_limits)
    draw_object(ego_veh, draw_params=_draw_params_ego)

    # add title and timestep to movie
    plt.title("t = {:.2f} s".format(float(time_step) * sumo_sim.dt),
              fontdict=None, loc='center', pad=None)  # **kwargs)
    plt.gca().set_aspect('equal')
    plt.xticks([])
    plt.yticks([])
    if plot_limits is None:
        plt.autoscale()
    else:
        plt.xlim(plot_limits[0], plot_limits[1])
        plt.ylim(plot_limits[2], plot_limits[3])

    # only for paper
    plt.draw()

# Print iterations progress
def _printProgressBar (iteration, total, prefix ='', suffix ='', decimals = 1, length = 100, fill ='█'):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
    """

    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print('\r%s |%s| %s%% %s' % (prefix, bar, percent, suffix), end='')

    # Print New Line on Complete
    if iteration == total:
        print()