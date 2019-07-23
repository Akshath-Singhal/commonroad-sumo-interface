import os
import subprocess
from typing import List
from xml.dom import minidom
from xml.etree import ElementTree as et

import sumolib
import numpy as np
import warnings

from commonroad.common.util import Interval
from lxml import etree
from commonroad.common.file_writer import CommonRoadFileWriter
from opendrive2lanelet.network import Network
from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from sumo_config.default import EGO_ID_START

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["ZIM Projekt ZF4086007BZ8"]
__version__ = "1.0.0"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"



def convert_net_to_cr(net_file:str, out_folder:str=None,verbose=False) -> str:
    """
    Converts .net file to CommonRoad xml using netconvert and OpenDRIVE 2 Lanelet Converter.

    :param net_file: path of .net.xml file
    :param out_folder: path of output folder for CommonRoad scenario.

    :return: commonroad map file
    """
    assert isinstance(net_file,str)

    if out_folder is None:
        out_folder = os.path.dirname(net_file)

    # filenames
    scenario_name = get_scenario_name_from_netfile(net_file)
    opendrive_file = os.path.join(out_folder, scenario_name + '.xodr')
    cr_map_file = os.path.join(out_folder, scenario_name + '.cr.xml')

    # convert to OpenDRIVE file using netconvert
    out = subprocess.check_output(['netconvert', '-s', net_file, '--opendrive-output', opendrive_file, '--junctions.scurve-stretch','1.0'])
    if verbose:
        print('converted to OpenDrive (.xodr)')
    # convert to commonroad using opendrive2lanelet
    # import, parse and convert OpenDRIVE file
    with open(opendrive_file, "r") as fi:
        open_drive = parse_opendrive(etree.parse(fi).getroot())

    road_network = Network()
    road_network.load_opendrive(open_drive)
    scenario = road_network.export_commonroad_scenario()
    if verbose:
        print('converted to Commonroad (.cr.xml)')
    # write CommonRoad scenario to file
    commonroad_writer = CommonRoadFileWriter(scenario, planning_problem_set=None,
                                             source="Converted from SUMO net using netconvert and OpenDRIVE 2 Lanelet Converter",
                                             tags='',author='',affiliation='')
    with open(cr_map_file, "w") as fh:
        commonroad_writer.write_scenario_to_file_io(file_io=fh)

    return cr_map_file

def find_ego_ids_by_departure_time(rou_file: str, n_ego_vehicles: int, departure_time_ego: int, ego_ids: list) -> list:
    """
    Returns ids of vehicles from route file which match desired departure time as close as possible.

    :param rou_file: path of route file
    :param n_ego_vehicles:  number of ego vehicles
    :param departure_time_ego: desired departure time ego vehicle
    :param ego_ids: if desired ids of ego_vehicle known, specify here

    """
    vehicles = sumolib.output.parse_fast(rou_file, 'vehicle', ['id', 'depart'])
    dep_times = []
    veh_ids = []
    for veh in vehicles:
        veh_ids.append(veh[0])
        dep_times.append(int(float(veh[1])))

    if n_ego_vehicles > len(veh_ids):
        warnings.warn('only {} vehicles in route file instead of {}'.format(len(veh_ids), n_ego_vehicles),stacklevel=1)
        n_ego_vehicles = len(veh_ids)

    # check if specified ids exist
    for id in ego_ids:
        if id not in veh_ids:
            warnings.warn('<generate_rou_file> id {} not in route file!'.format_map(id))
            del id

    # assign vehicles as ego by finding closest departure time
    dep_times = np.array(dep_times)
    veh_ids = np.array(veh_ids)
    greater_start_time = np.where(dep_times >= departure_time_ego)[0]
    for index in greater_start_time:
        if len(ego_ids) == n_ego_vehicles:
            break
        else:
            ego_ids.append(veh_ids[index])

    if len(ego_ids) < n_ego_vehicles:
        n_ids_missing = n_ego_vehicles - len(ego_ids)
        ego_ids.extend((veh_ids[greater_start_time[0] - n_ids_missing:greater_start_time[0]]).tolist())

    return ego_ids


def get_scenario_name_from_netfile(filepath:str) -> str:
    """
    Returns the scenario name specified in the net file.

    :param filepath: the path of the net file

    """
    scenario_name:str = (os.path.splitext(os.path.basename(filepath))[0]).split('.')[0]
    return scenario_name

def generate_rou_file(net_file:str, dt:float, n_vehicles_max:int, departure_time: Interval, n_ego_vehicles:int, departure_time_ego:int, ego_ids:List[int]=None, veh_per_second:float=None, out_folder:str=None) -> str:
    """
    Creates route & trips files using randomTrips generator.

    :param net_file: path of .net.xml file
    :param dt: length of the time step
    :param n_vehicles_max: max. number of vehicles in route file
    :param departure_time: Interval of departure times for vehicles
    :param n_ego_vehicles: number of ego vehicles
    :param departure_time_ego: desired departure time ego vehicle
    :param ego_ids: if desired ids of ego_vehicle known, specify here
    :param veh_per_second: number of vehicle departures per second
    :param out_folder: output folder of route file (same as net_file if None)

    :return: path of route file
    """
    if out_folder is None:
        out_folder = os.path.dirname(net_file)

    # vehicles per second
    if veh_per_second is not None:
        period = 1 / (veh_per_second * dt)
    else:
        period = 1

    # filenames
    scenario_name = get_scenario_name_from_netfile(net_file)
    rou_file = os.path.join(out_folder, scenario_name + '.rou.xml')
    trip_file = os.path.join(out_folder, scenario_name + '.trips.xml')

    # create route file
    step_per_departure = ((departure_time.end - departure_time.start) / n_vehicles_max)
    try:
        subprocess.check_output(['python', os.path.join(os.environ['SUMO_HOME'],'tools/randomTrips.py'), '-n', net_file,
                                    '-o', trip_file, '-r', rou_file, '-b', str(departure_time.start),
                                    '-e', str(departure_time.end), '-p', str(step_per_departure),'--allow-fringe',
                                     '--trip-attributes=departLane=\"free\" departSpeed=\"random\" departPos=\"base\"',
                                     '--period',str(period)
                                     ])
    except subprocess.CalledProcessError as e:
        raise RuntimeError("Command '{}' return with error (code {}): {}".format(e.cmd, e.returncode, e.output))

    # get ego ids and add EGO_ID_START prefix
    ego_ids = find_ego_ids_by_departure_time(rou_file, n_ego_vehicles, departure_time_ego, ego_ids)
    write_ego_ids_to_rou_file(rou_file, ego_ids)

    return rou_file


def write_ego_ids_to_rou_file(rou_file:str, ego_ids:List[int]) -> None:
    """
    Writes ids of ego vehicles to the route file.

    :param rou_file: the route file
    :param ego_ids: a list of ego vehicle ids

    """
    tree = et.parse(rou_file)
    vehicles = tree.findall('vehicle')
    ego_str = {}
    for ego_id in ego_ids:
        ego_str.update({str(ego_id): EGO_ID_START + str(ego_id)})

    for veh in vehicles:
        if veh.attrib['id'] in ego_str:
            veh.attrib['id'] = ego_str[veh.attrib['id']]

    for elem in tree.iter():
        if (elem.text):
            elem.text = elem.text.strip()
        if (elem.tail):
            elem.tail = elem.tail.strip()
    rough_string = et.tostring(tree.getroot(), encoding='utf-8')
    reparsed = minidom.parseString(rough_string)
    text = reparsed.toprettyxml(indent="\t", newl="\n")
    file = open(rou_file, "w")
    file.write(text)