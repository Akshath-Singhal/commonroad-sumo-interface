import os
import xml.etree.ElementTree as et

import warnings

import sumo_config.default
from sumo_config import plot_params
from typing import List, Tuple

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["ZIM Projekt ZF4086007BZ8"]
__version__ = "1.0.0"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

def get_route_files(config_file) -> List[str]:
    """
    Returns net-file and route-files specified in the config file.

    :param config_file: SUMO config file (.sumocfg)

    """
    if not os.path.isfile(config_file):
        raise FileNotFoundError(config_file)
    tree = et.parse(config_file)
    file_directory = os.path.dirname(config_file)

    # find route-files
    all_route_files = tree.findall('*/route-files')
    route_files = []
    if len(all_route_files) < 1:
        raise RouteError()
    for item in all_route_files:
        attributes = item.attrib['value'].split(',')
        for route in attributes:
            route_files.append(os.path.join(file_directory, route))
    return route_files


def initialize_id_dicts(id_convention: dict) -> Tuple[dict, dict]:
    """
    Creates empty nested dict structure for sumo2cr and cr2sumo dicts from id_convention and returns them.
    
    :param id_convention: dict with mapping from object type to id start number
    
    """
    sumo2cr = {}
    cr2sumo = {}
    for k in id_convention:
        sumo2cr[k] = {}
        cr2sumo[k] = {}
    sumo2cr['all_ids'] = {}
    cr2sumo['all_ids'] = {}
    return sumo2cr, cr2sumo

def generate_cr_id(type:str, sumo_id:int, ids_sumo2cr:dict) -> int:
    """ 
    Generates a new commonroad ID without adding it to any ID dictionary.
    
    :param type: one of the keys in params.id_convention; the type defines the first digit of the cr_id
    :param sumo_id: id in sumo simulation
    :param ids_sumo2cr: dictionary of ids in sumo2cr

    """
    if type not in sumo_config.default.ID_DICT:
        raise ValueError(
            '{0} is not a valid type of id_convention. Only allowed: {1}'.format(type, sumo_config.default.ID_DICT.keys()))
    if sumo_id in ids_sumo2cr[type]:
        warnings.warn('For this sumo_id there is already a commonroad id. No cr ID is generated')
        return ids_sumo2cr[type][sumo_id]
    elif sumo_id in ids_sumo2cr['all_ids']:
        raise ValueError(
            'Two sumo objects of different types seem to have same sumo ID {0}. ID must be unique'.format(sumo_id))
        # If this case happens for auto-generated nets, this code will have to be changed. Conversion of SUMO id to
        # cr id would have to incorporate type.

    cr_id = int(str(sumo_config.default.ID_DICT[type]) + str(len(ids_sumo2cr[type])))
    return cr_id


def cr2sumo(cr_id:int, ids_cr2sumo:dict) -> int:
    """ 
    Gets CommonRoad ID and returns corresponding SUMO ID.
    
    :param ids_cr2sumo: dictionary of ids in cr2sumo
    
    """

    if type(cr_id) == list:
        print("id: " + str(cr_id) + ": " + str(ids_cr2sumo['all_ids']))
        print("\n")

    if cr_id is None:
        return None
    elif cr_id in ids_cr2sumo['all_ids']:
        return ids_cr2sumo['all_ids'][cr_id]
    else:
        raise ValueError('Commonroad id {0} does not exist.'.format(cr_id))


def sumo2cr(sumo_id:int, ids_sumo2cr:dict) -> int:
    """ 
    Returns corresponding CommonRoad ID according to sumo id. 
    
    :param sumo_id: sumo id
    :param ids_sumo2cr: dictionary of ids in sumo2cr.

    """
    if sumo_id is None:
        return None
    elif sumo_id in ids_sumo2cr['all_ids']:
        return ids_sumo2cr['all_ids'][sumo_id]
    else:
        if sumo_id == "":
            warnings.warn('Tried to convert id <empty string>. \
                Check if your net file is complete (e. g. having internal-links,...)')
        raise ValueError('Sumo id \'%s\' does not exist.' % sumo_id)


class NetError(Exception):
    """ 
    Exception raised if there is no net-file or multiple net-files. 

    """
    def __init__(self, len):
        self.len = len

    def __str__(self):
        if self.len == 0:
            return repr('There is no net-file.')
        else:
            return repr('There are more than one net-files.')


class RouteError(Exception):
    """ 
    Exception raised if there is no route-file. 

    """
    def __str__(self):
        return repr('There is no route-file.')



class OutOfLanelet(Exception):
    """ 
    Exception raised if the position of the ego vehicle is outside all lanelets.
    
    """
    def __init__(self, position):
        self.position = position

    def __str__(self):
        return repr('The position %s is not part of any lanelet.' % self.position)
