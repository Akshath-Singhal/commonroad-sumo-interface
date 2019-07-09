import os, sys, warnings
import matplotlib.pyplot as plt
from commonroad.visualization.draw_dispatch_cr import draw_object
from typing import List
import pathlib
import xml.etree.ElementTree as et
from xml.dom import minidom
from sumo2cr.interface.util import NetError
from sumo2cr.maps.util import get_scenario_name_from_netfile, convert_net_to_cr, generate_rou_file
from pathConfig import DEFAULT_CFG_FILE
from commonroad.scenario.trajectory import State
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.common.util import Interval
from commonroad.common.file_reader import CommonRoadFileReader

from sumo_config.default import SumoCommonRoadConfig as conf

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["ZIM Projekt ZF4086007BZ8"]
__version__ = "1.0.0"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


class ScenarioWrapper:
    def __init__(self):
        self.scenario_name: str = ''
        self.net_file: str = ''
        self.cr_map_file: str = ''
        self.sumo_cfg_file = None
        self.ego_start_time: int = 0
        self.sumo_net = None
        self.lanelet_network: LaneletNetwork = None

    def initialize(self,scenario_name:str,sumo_cfg_file:str,cr_map_file:str,ego_start_time:int=None):
        self.scenario_name = scenario_name
        self.sumo_cfg_file = sumo_cfg_file
        self.net_file = self._get_net_file(self.sumo_cfg_file)
        self.cr_map_file = cr_map_file
        self.ego_start_time = ego_start_time
        self.lanelet_network = CommonRoadFileReader(self.cr_map_file).open_lanelet_network()
        # self.ids_sumo2cr, self.ids_cr2sumo = self.initialize_id_dicts(params.ID_DICT)
        # self.sumo_net = sumolib_net.readNet(self.net_file, withInternal=True)

    @classmethod
    def init_from_scenario(cls, scenario_name, ego_start_time=None) -> 'ScenarioWrapper':
        obj = cls()
        sumo_cfg_file = os.path.join(os.path.dirname(__file__), '../../scenarios/',scenario_name, scenario_name + '.sumo.cfg')
        cr_map_file = os.path.join(os.path.dirname(__file__), '../../scenarios/',scenario_name, scenario_name + '.cr.xml')
        if not os.path.exists(cr_map_file):
            warnings.warn('<ScenarioWrapper/init_from_scenario> CR file {} does not exist. Converting from .net.xml file.'
                          .format(cr_map_file),stacklevel=2)
            net_file = obj._get_net_file(sumo_cfg_file)
            convert_net_to_cr(net_file)
        obj.initialize(scenario_name,sumo_cfg_file,cr_map_file,ego_start_time)
        return obj

    @classmethod
    def recreate_route_file(cls, sumo_cfg_file, dt:float, n_vehicles_max:int, n_ego_vehicles:int, ego_ids:List[int]=[], ego_start_time:int=0,
                            departure_time_ego:int=0, departure_interval_vehicles:Interval=Interval(0,conf.simulation_steps),veh_per_second=None) -> 'ScenarioWrapper':
        """
        Creates new .rou.xml file. Assumes .cr.xml, .net.xml and .sumo.cfg file have already been created in scenario folder.
        :param sumo_cfg_file:
        :param n_vehicles_max:
        :param n_ego_vehicles:
        :param ego_ids:
        :param ego_start_time:
        :param departure_interval_vehicles:
        :return:
        """
        sumo_scenario = cls()
        out_folder = os.path.dirname(sumo_cfg_file)
        net_file = sumo_scenario._get_net_file(sumo_cfg_file)
        scenario_name = get_scenario_name_from_netfile(net_file)
        generate_rou_file(net_file, dt, n_vehicles_max, departure_interval_vehicles, n_ego_vehicles, departure_time_ego, ego_ids, veh_per_second, out_folder)
        cr_map_file = os.path.join(os.path.dirname(__file__), '../../scenarios/', scenario_name, scenario_name + '.cr.xml')

        sumo_scenario.initialize(scenario_name, sumo_cfg_file, cr_map_file, ego_start_time)
        return sumo_scenario

    @classmethod
    def full_conversion_from_net(cls, net_file: str, dt:float, n_vehicles_max:int, n_ego_vehicles:int, ego_ids:List[int]=[],
                                 ego_start_time:int=0, departure_time_ego:int=0,
                                 departure_interval_vehicles:Interval=Interval(0,conf.simulation_steps), veh_per_second=None,scenario_folder:str=None) -> 'ScenarioWrapper':
        """
        Convert net file to CommonRoad xml and generate specify ego vehicle either by using generated vehicles and/or by initial states.
        :param net_file: path of .net.xml file
        :param dt: length of time step
        :param n_vehicles_max: total number of ego vehicles that are generated
        :param ego_id: if specified, vehicle with given id from .rou file is marked as ego_vehicle
        :param initial_state: List of initial states for ego vehicle
        :param scenario_folder: folder that contains all scenario folders. If not given, use folder from net_file
        :return:
        """
        assert len(ego_ids) <= n_ego_vehicles, "total number of given ego_vehicles must be <= n_ego_vehicles, but {}not<={}"\
            .format(len(ego_ids),n_ego_vehicles)
        assert n_ego_vehicles <= n_vehicles_max

        if scenario_folder is None:
            scenario_folder = os.path.join(os.path.dirname(__file__), '../../scenarios')

        sumo_scenario = cls()
        scenario_name = get_scenario_name_from_netfile(net_file)
        out_folder = os.path.join(scenario_folder,scenario_name)
        pathlib.Path(out_folder).mkdir(parents=False, exist_ok=True)

        # create files
        cr_map_file = convert_net_to_cr(net_file, out_folder)
        generate_rou_file(net_file, dt, n_vehicles_max, departure_interval_vehicles, n_ego_vehicles, departure_time_ego, ego_ids, veh_per_second, out_folder)
        sumo_cfg_file = sumo_scenario.generate_cfg_file(scenario_name,out_folder)

        sumo_scenario.initialize(scenario_name,sumo_cfg_file,cr_map_file,ego_start_time)
        return sumo_scenario

    def generate_cfg_file(self, scenario_name, output_folder:str):
        sumo_cfg_file = os.path.join(output_folder, scenario_name + '.sumo.cfg')
        tree = et.parse(os.path.join(os.path.dirname(__file__), '../../',DEFAULT_CFG_FILE),)
        tree.findall('*/net-file')[0].attrib['value'] = scenario_name + '.net.xml'
        tree.findall('*/route-files')[0].attrib['value'] = scenario_name + '.rou.xml'
        for elem in tree.iter():
            if (elem.text):
                elem.text = elem.text.strip()
            if (elem.tail):
                elem.tail = elem.tail.strip()
        rough_string = et.tostring(tree.getroot(), encoding='utf-8')
        reparsed = minidom.parseString(rough_string)
        text = reparsed.toprettyxml(indent="\t",newl="\n")
        file = open(sumo_cfg_file, "w")
        file.write(text)

        return sumo_cfg_file

    def _get_net_file(self, sumo_cfg_file):
        """
        :param sumo_cfg_file: SUMO sumo_config file (.sumocfg)
        :return: net-file specified in the sumo_config file
        """
        if not os.path.isfile(sumo_cfg_file):
            raise ValueError("File not found: {}. Maybe scenario name is incorrect.".format(sumo_cfg_file))
        tree = et.parse(sumo_cfg_file)
        file_directory = os.path.dirname(sumo_cfg_file)
        # find net-file
        all_net_files = tree.findall('*/net-file')
        if len(all_net_files) != 1:
            raise NetError(len(all_net_files))
        return os.path.join(file_directory, all_net_files[0].attrib['value'])

    def print_lanelet_net(self, with_lane_id=True, with_succ_pred=False, with_adj=False, with_speed=False):
        plt.figure(figsize=(25, 25))
        plt.gca().set_aspect('equal')
        draw_object(self.lanelet_network)
        k = len(self.lanelet_network.lanelets)
        # add annotations
        for l in self.lanelet_network.lanelets:
            # assure that text for two different lanelets starting on same position is placed differently
            print(l.lanelet_id)
            k = k - 1
            info = ''
            if with_lane_id:
                id = 'id: ' + str(l.lanelet_id)
                plt.text(l.center_vertices[0,0], l.center_vertices[0,1], id, zorder=100, size=8,
                         color='r',
                         verticalalignment='top')
            if with_succ_pred:
                info = info + '\nsucc: ' + str(l.successor) + ' pred: ' + str(l.predecessor)
            if with_adj:
                info = info + ' \nadj_l: ' + str(l.adj_left) + '; adj_l_same_dir: ' + str(l.adj_left_same_direction)
                info = info + ' \nadj_r: ' + str(l.adj_right) + '; adj_r_same_dir: ' + str(l.adj_right_same_direction)
            if with_speed:
                info = info + '\nspeed limit: ' + str(l.speed_limit)
            plt.plot(l.center_vertices[0, 0], l.center_vertices[0, 1], 'x')
            plt.text(l.center_vertices[0, 0], l.center_vertices[0, 1], info, zorder=100, size=8,
                     verticalalignment='top')
        plt.show()