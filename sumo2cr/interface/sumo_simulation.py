import sys, os

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from collections import defaultdict
import logging
import math
import sumo_config.default
import traci
import numpy as np
import copy
from commonroad.scenario.trajectory import State
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.geometry.shape import Rectangle
from sumo2cr.interface.ego_vehicle import EgoVehicle
from sumo2cr.maps.sumo_scenario import ScenarioWrapper
from sumo2cr.interface.util import *
from pathConfig import *
from sumo_config.plot_params import *
from sumo_config.default import SumoCommonRoadConfig
from sumo2cr.interface.util import initialize_id_dicts
from typing import Dict, List, Union, Type, Tuple

__author__ = "Moritz Klischat, Mostafa Eissa"
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
    sys.exit("please declare environment variable 'SUMO_HOME' to point at the path of your SUMO installation.")

logging.basicConfig(stream=sys.stderr, level=logging.CRITICAL)


class SumoSimulation:
    """
    Class for interfacing between the SUMO simulation and CommonRoad.
    """
    def __init__(self):
        """Init empty object"""
        self.dt = None
        self.planning_problem_set = None
        self.obstacle_states: Dict[int,Dict[int,State]] = defaultdict(lambda: dict()) # obstacle_states[time_steps[cr_id]
        self.simulationdomain = traci._simulation.SimulationDomain()
        self.vehicledomain = traci._vehicle.VehicleDomain()
        self.routedomain = traci._route.RouteDomain()
        self._current_time_step = 0
        self.ids_sumo2cr, self.ids_cr2sumo = initialize_id_dicts(sumo_config.default.ID_DICT)
        self.obstacle_shapes: Dict[int,Rectangle] = {}
        self.scenarios = ScenarioWrapper()
        self.ego_vehicles: Dict[int, EgoVehicle] = dict()
        self.conf = SumoCommonRoadConfig
        self._silent = False
        # enables dummy synchronization of ego vehicles without planner for testing
        self._dummy_ego_simulation = False
        # ego sync parameters
        self._lc_duration_max = 10
        self._lc_counter = 0  # Count how many steps are counter for the lane change
        self._lc_inaction = 0  # Flag indicates that SUMO is performing a lane change for the ego
        self.lateral_position_buffer = dict()  # stores lateral position [ego_vehicle_id,float]

    def initialize(self, conf: Type[SumoCommonRoadConfig] = None) -> None:
        """
        Reads scenario files, starts traci simulation, initializes vehicles, conducts pre-simulation.

        :param conf: configuration object. If None, use default configuration.
        
        """

        if conf is not None:
            self.conf = conf

        self.scenarios = ScenarioWrapper.init_from_scenario(self.conf.scenario_name)

        self.dt = self.conf.dt
        self.dt_sumo = self.conf.dt / self.conf.delta_steps
        self.delta_steps = self.conf.delta_steps

        if self.conf.with_sumo_gui:
            warnings.warn('Continuous lane change currently not implemented.')
            traci.start([SUMO_GUI_BINARY, "-c", self.scenarios.sumo_cfg_file, "--step-length", str(self.dt_sumo)])
        else:
            traci.start([SUMO_BINARY, "-c", self.scenarios.sumo_cfg_file, "--step-length", str(self.dt_sumo)])

        # simulate until ego_time_start
        self.__presimulation_silent(self.conf.presimulation_steps)

        # initializes vehicle positions (and ego vehicles, if defined in .rou file)
        if self.planning_problem_set is not None:
            if len(self.ego_vehicles) > 0:
                warnings.warn('<SumoSimulation/init_ego_vehicles> Ego vehicles are defined through .rou file and planning problem. Is that intended?')
            self.init_ego_vehicles_from_planning_problem(self.planning_problem_set)

    def init_ego_vehicles_from_planning_problem(self, planning_problem_set: PlanningProblemSet) -> None:
        """
        Initializes the ego vehicles according to planning problem set.

        :param planning_problem_set: The planning problem set which defines the ego vehicles. 
        """

        # retrieve arbitrary route id for initialization (will not be used by interface)
        generic_route_id = self.routedomain.getIDList()[0]

        width = self.conf.ego_veh_width
        length = self.conf.ego_veh_length

        # create ego vehicles if planning problem is given
        if planning_problem_set is not None:
            for planning_problem in planning_problem_set.planning_problem_dict.values():
                cr_id = self.cr_scenario.generate_object_id()
                sumo_id = self._create_sumo_id()
                self.vehicledomain.add(sumo_id,generic_route_id)
                self.ids_sumo2cr[sumo_config.default.EGO_ID_START][sumo_id] = cr_id
                self.ids_sumo2cr['all_ids'][sumo_id] = cr_id
                self.ids_cr2sumo[sumo_config.default.EGO_ID_START][cr_id] = sumo_id
                self.ids_cr2sumo['all_ids'][cr_id] = sumo_id
                self._add_ego_vehicle(
                    EgoVehicle(cr_id, planning_problem.initial_state, self.conf.delta_steps, width, length, planning_problem))

    def _add_ego_vehicle(self, ego_vehicle: EgoVehicle):
        """
        Adds a ego vehicle to the current ego vehicle set.

        :param ego_vehicle: the ego vehicle to be added.
        """
        self._ego_vehicles[ego_vehicle.id] = ego_vehicle

    @property
    def ego_vehicles(self) -> Dict[int, EgoVehicle]:
        """
        Returns the ego vehicles of the current simulation.

        """
        return self._ego_vehicles

    @ego_vehicles.setter
    def ego_vehicles(self, ego_vehicles: Dict[int, EgoVehicle]):
        """
        Sets the ego vehicles of the current simulation.

        :param ego_behicles: ego vehicles used to set up the current simulation.
        """
        if hasattr(self, '_ego_vehicles'):
            raise ValueError('Cannot set ego_vehicles, use SumoSimlation.forward_ego_vehicles() or ._add_ego_vehicle()')
        else:
            self._ego_vehicles = ego_vehicles

    @property
    def current_time_step(self) -> int:
        """
        :return: current time step of interface
        """
        return self._current_time_step

    @current_time_step.setter
    def current_time_step(self,current_time_step):
        """
        Time step should not be set manually.
        """
        raise(ValueError('Time step should not be set manually'))

    def print_lanelet_net(self, with_lane_id=True, with_succ_pred=False, with_adj=False, with_speed=False):
        """ 
        Plots commonroad road network without vehicles or obstacles.

        :param with_lane_id: if set to true, the lane id will also be printed.
        :param with_succ_pred: if set to true, the successor and predecessor lanelets will be printed.
        :param with_adj: if set to true, adjacant lanelets will be printed.
        :param with_speed: if set to true, the speed limit will be printed.
        """
        self.scenarios.print_lanelet_net(with_lane_id=with_lane_id, with_succ_pred=with_succ_pred,
                                         with_adj=with_adj, with_speed=with_speed)

    def commonroad_scenario_at_time_step(self, time_step:int, add_ego=False, start_0=True) -> Scenario:
        """ 
        Creates and returns a commonroad scenario at the given time_step. Initial time_step=0 for all obstacles.
        
        :param time_step: the scenario will be created according this time step.
        :param add_ego: whether to add ego vehicles to the scenario.
        :param start_0: if set to true, initial time step of vehicles is 0, otherwise, the current time step

        
        """
        self.cr_scenario = Scenario(self.dt, '1')
        self.cr_scenario.lanelet_network = self.scenarios.lanelet_network
        self.cr_scenario.add_objects(self._get_cr_obstacles(time_step, add_ego=add_ego, start_0=start_0))
        return self.cr_scenario

    def simulate_step(self) -> None:
        """ 
        Executes next simulation step (consisting of delta_steps sub-steps with dt_sumo=dt/delta_steps) in SUMO
        
        """

        # simulate sumo scenario for delta_steps time steps
        for i in range(self.delta_steps):
            # send ego vehicles to SUMO
            if not self._dummy_ego_simulation and len(self.ego_vehicles) > 0:
                self._send_ego_vehicles(self.ego_vehicles, i)

            # execute SUMO simulation step
            traci.simulationStep()            
            for ego_veh in list(self.ego_vehicles.values()):
                ego_veh._current_time_step += 1

        # get updated obstacles from sumo
        self._current_time_step += 1
        self._fetch_sumo_vehicles()

    def __presimulation_silent(self, presimulation_steps:int):
        """
        Simulate SUMO without synchronization of interface. Used before starting interface simulation.
        
        :param presimulation_steps: the steps of simulation which are executed before checking the existence of ego vehicles and configured simulation step.
        
        """
        assert self.current_time_step == 0
        assert presimulation_steps >= 0, \
            'ego_time_start={} must be >0'.format(self.conf.presimulation_steps)

        if presimulation_steps == 0:
            return

        self._silent = True
        for i in range(presimulation_steps * self.delta_steps):
            traci.simulationStep()

        self._fetch_sumo_vehicles()
        self._silent = False

        # when ego vehicles has not entered simulation: continue simulation
        j = 0
        while len(self.ego_vehicles)==0 and j<=self.conf.simulation_steps*self.conf.delta_steps:
            j += 1
            traci.simulationStep()

            self._fetch_sumo_vehicles()

        if len(self.ego_vehicles)==0:
            warnings.warn('No ego vehicles entered simulation.',stacklevel=1)
        elif j>1:
            warnings.warn('Simulated {} more steps until ego vehicle entered simulation.'.format((j-1)/self.conf.delta_steps), stacklevel=1)

    def _fetch_sumo_vehicles(self):
        """
        Gets and stores all vehicle states from SUMO. Initializes ego vehicles when they enter simulation.
        
        """
        step = self.current_time_step
        vehicle_ids = self.vehicledomain.getIDList()

        if vehicle_ids:
            for veh_id in vehicle_ids:
                state = self._get_current_state_from_sumo(veh_id)
                if veh_id not in self.ids_sumo2cr['all_ids']:
                    # initializes new vehicle
                    if veh_id.startswith(sumo_config.default.EGO_ID_START) and not self._silent:
                        # new ego vehicle
                        cr_id = self._create_cr_id(sumo_config.default.EGO_ID_START, veh_id)
                        if self._dummy_ego_simulation:
                            state.time_step = step -1
                        else:
                            state.time_step = step
                        self._add_ego_vehicle(
                            EgoVehicle(cr_id, state, self.conf.delta_steps, self.conf.ego_veh_width, self.conf.ego_veh_length))
                    elif not self._silent:
                        # new obstacle vehicle
                        cr_id = self._create_cr_id('obstacleVehicle', veh_id)
                        self.obstacle_shapes[cr_id] = Rectangle(self.vehicledomain.getLength(veh_id),
                                                                self.vehicledomain.getWidth(veh_id))
                        self.obstacle_states[step][self.ids_sumo2cr['obstacleVehicle'][veh_id]] = state
                elif veh_id in self.ids_sumo2cr['obstacleVehicle']:
                    # get obstacle vehicle state
                    self.obstacle_states[step][self.ids_sumo2cr['obstacleVehicle'][veh_id]] = state
                elif not self._silent and not veh_id in self.ids_sumo2cr['egoVehicle']:
                    raise NotImplemented

                """For testing wit _dummy_ego_simulation"""
                if not self._silent and self._dummy_ego_simulation and veh_id in self.ids_sumo2cr['egoVehicle']:
                    ego_veh = self.ego_vehicles[self.ids_sumo2cr['egoVehicle'][veh_id]]
                    ori = state.orientation
                    state_list = []
                    for t in range(0,self.conf.delta_steps):
                        state_tmp = copy.deepcopy(state)
                        state_tmp.position = state.position + (t+1) * state.velocity * self.dt * np.array([np.cos(ori), np.sin(ori)])
                        state_tmp.time_step = t +1
                        state_list.append(state_tmp)

                    ego_veh.set_planned_trajectory(state_list)

    def _get_current_state_from_sumo(self, veh_id: str) -> State:
        """
        Gets the current state from sumo.

        :param veh_id: the id of the vehicle, whose state will be returned from SUMO.

        :return: the state of the given vehicle 
        """
        state = State(position=np.array(self.vehicledomain.getPosition(veh_id)),
                      orientation=math.radians(-self.vehicledomain.getAngle(veh_id) + 90),
                      velocity=self.vehicledomain.getSpeed(veh_id),
                      acceleration=self.vehicledomain.getAcceleration(veh_id),
                      time_step=0)
        return state

    def _get_cr_obstacles(self, time_step: int, add_ego:bool=False, start_0:bool=False) -> List[DynamicObstacle]:
        """
        Gets all vehicles in commonroad format from recorded simulation.

        :param time_step: time step of scenario
        :param add_ego: if True, add ego vehicles as well
        :param start_0: if True, initial time step of vehicles is 0, otherwise, the current time step

        """
        vehicle_dict: Dict[int,State] = self.obstacle_states[time_step]
        obstacles = []
        for veh_id, state in vehicle_dict.items():
            if start_0:
                state.time_step = 0
            else:
                state.time_step = time_step

            dynamic_obstacle = DynamicObstacle(obstacle_id=veh_id, obstacle_type=ObstacleType.CAR,
                                               initial_state=state, obstacle_shape=self.obstacle_shapes[veh_id])
            obstacles.append(dynamic_obstacle)

        if add_ego:
            obstacles.extend(self.get_ego_obstacles(time_step))

        return obstacles

    def get_ego_obstacles(self, time_step: Union[int, None] = None) -> List[DynamicObstacle]:
        """
        Get list of ego vehicles converted to Dynamic obstacles
        :param time_step: initial time step, if None, get complete driven trajectory
        :return:
        """
        obstacles = []
        for veh_id, ego_veh in self.ego_vehicles.items():
            obs = ego_veh.get_dynamic_obstacle(time_step)
            if obs is not None:
                obstacles.append(obs)

        return obstacles

    def _send_ego_vehicles(self, ego_vehicles: Dict[int, EgoVehicle], delta_step:int=0) -> None:
        """
        Sends the infomation of ego vehicles to SUMO.

        :param ego_vehicles: list of dictionaries.
            For each ego_vehicle, write tuple (cr_ego_id, cr_position, cr_lanelet_id, cr_orientation, cr_lanelet_id)
            cr_lanelet_id can be omitted but this is not recommended, if the lanelet is known for sure.
        :param delta_step: which time step of the planned trajectory should be sent

        """
        for id_cr, ego_vehicle in ego_vehicles.items():
            assert ego_vehicle.current_time_step == self.current_time_step,\
                'Trajectory of ego vehicle has not been updated. Still at time_step {}, while simulation step {} should be simulated.'.format(ego_vehicle.current_time_step,self.current_time_step+1)

            # Check if there is a lanelet change in the configured time window
            lc_future_status, lc_duration = self.check_lanelets_future_change(ego_vehicle.current_state,ego_vehicle.get_planned_trajectory)
            # If there is a lanelet change, check whether the change is just started
            lc_status = self.check_lc_start(id_cr, lc_future_status)
            # Calculate the sync mechanism based on the future information
            sync_mechanism = self.check_sync_mechanism(lc_status, id_cr, ego_vehicle.current_state)
            # Execute MoveXY or SUMO lane change according to the sync mechanism
            planned_state = ego_vehicle.get_planned_state(delta_step)
            self.forward_info2sumo(planned_state, sync_mechanism,  lc_duration, id_cr)

    def _get_ego_ids(self) -> Dict[int, str]:
        """
        Returns a dictionary with all current ego vehicle ids and corresponding sumo ids
        """
        return self.ids_cr2sumo[sumo_config.default.EGO_ID_START]

    def _create_sumo_id(self) -> int:
        """
        Generates a new unused id for SUMO
        :return:
        """
        id_list = self.vehicledomain.getIDList()
        id = int(len(id_list))
        i = 0
        while i<1000:
            if id not in id_list:
                return id
            else:
                id += 1
                i+=1

    def _create_cr_id(self, type:str, sumo_id) -> int:
        """
        Generates a new cr ID and adds it to ID dictionaries

        :param type: one of the keys in params.id_convention; the type defines the first digit of the cr_id
        :param sumo_id: id in sumo simulation

        :return: cr_id as int
        """
        cr_id = generate_cr_id(type, sumo_id, self.ids_sumo2cr)
        self.ids_sumo2cr[type][sumo_id] = cr_id
        self.ids_sumo2cr['all_ids'][sumo_id] = cr_id
        self.ids_cr2sumo[type][cr_id] = sumo_id
        self.ids_cr2sumo['all_ids'][cr_id] = sumo_id

        return cr_id

    @property
    def _silent(self):
        """Ego vehicle is not synced in this mode."""
        return self.__silent

    @_silent.setter
    def _silent(self, silent):
        assert self.current_time_step==0
        self.__silent = silent

    def stop(self):
        """ Exits SUMO Simulation """
        traci.close()
        sys.stdout.flush()

    # Ego sync functions
    def check_lanelets_future_change(self, current_state:State, planned_traj:List[State]) -> Tuple[str, int]:
        """
        Checks the lanelet changes of the ego vehicle in the future time_window.

        :param lanelet_network: object of the lanelet network
        :param time_window: the time of the window to check the lanelet change
        :param traj_index: index of the planner output corresponding to the current time step

        :return: lc_status, lc_duration: lc_status is the status of the lanelet change in the next time_window; lc_duration is the unit of time steps (using sumo dt)

        """
        lc_duration_max = min(self.conf.lanelet_check_time_window,len(planned_traj))
        lanelet_network = self.scenarios.lanelet_network
        lc_status = 'NO_LC'
        lc_duration = 0

        # find current lanelets
        current_position=current_state.position
        current_lanelets_ids=lanelet_network.find_lanelet_by_position([current_position])[0]
        current_lanelets=[lanelet_network.find_lanelet_by_id(id) for id in current_lanelets_ids]

        # check for lane change
        for current_lanelet in current_lanelets:
            for t in range(lc_duration_max):
                    future_lanelet_ids=lanelet_network.find_lanelet_by_position([planned_traj[t].position])[0]
                    if current_lanelet.adj_right in future_lanelet_ids:
                        lc_status = 'RIGHT_LC'
                        lc_duration = 2 * t * self.conf.delta_steps
                        break
                    elif current_lanelet.adj_left in future_lanelet_ids:
                        lc_status = 'LEFT_LC'
                        lc_duration = 2 * t * self.conf.delta_steps
                        break
                    else:
                        pass

        if self.conf.verbose==True:
            print('current lanelets: '+str(current_lanelets))
            print('lc_status='+lc_status)
            print('lc_duration=' + str(lc_duration))
        return lc_status,lc_duration

    def check_lc_start(self,ego_id:str,lc_future_status:str) ->str:
        """
        This function checks if a lane change is started according to the change in the lateral position and the lanelet
        change prediction. Note that checking the change of lateral position only is sensitive to the tiny changes, also
        at the boundaries of the lanes the lateral position sign is changed because it will be calculated relative to
        the new lane. So we check the future lanelet change to avoid these issues.

        :param ego_id: id of the ego vehicle
        :param lc_future_status: status of the future lanelet changes 

        :return: lc_status: the status whether the ego vehicle starts a lane change or no
        """
        lateral_position = self.vehicledomain.getLateralLanePosition(cr2sumo(ego_id, self.ids_cr2sumo))

        if lc_future_status == 'NO_LC' or not id in self.lateral_position_buffer:
            lc_status = 'NO_LC'
        elif lc_future_status=='RIGHT_LC' and self.lateral_position_buffer[id] > self.conf.lane_change_tol + lateral_position:
            lc_status='RIGHT_LC_STARTED'
        elif lc_future_status=='LEFT_LC' and self.lateral_position_buffer[id] < -self.conf.lane_change_tol + lateral_position:
            lc_status='LEFT_LC_STARTED'
        else:
            lc_status='NO_LC'

        if self.conf.verbose==True:
            print('LC current status: '+ lc_status)

        self.lateral_position_buffer[id] = lateral_position

        return lc_status

    def _consistency_protection(self,ego_id:str, current_state:State) -> str:
        """
        Checks the L2 distance between SUMO position and the planner position and returns CONSISTENCY_ERROR if it is
        above the configured margin.

        :param ego_id: id of the ego vehicle (string)
        :param current_state: the current state read from the commonroad motion planner

        :return: retval: the status whether there is a consistency error between sumo and planner positions or not
        """
        cons_error = 'CONSISTENCY_NO_ERROR'

        pos_sumo = self.vehicledomain.getPosition(cr2sumo(ego_id, self.ids_cr2sumo))
        pos_cr = current_state.position
        dist_error = np.linalg.norm(pos_cr - pos_sumo)
        if dist_error > self.conf.protection_margin:
            cons_error='CONSISTENCY_ERROR'

        if self.conf.verbose == True:
            print('SUMO X: '+str(pos_sumo[0])+' **** SUMO Y: '+str(pos_sumo[1]))
            print('TRAJ X: ' + str(pos_cr[0]) + ' **** TRAJ Y: ' + str(pos_cr[1]))
            print('Error Value: '+str(dist_error))
            print('Error Status: '+cons_error)

        if self._lc_inaction == 0:
            cons_error = 'CONSISTENCY_NO_ERROR'
        return cons_error

    def check_sync_mechanism(self, lc_status:str, ego_id:int, current_state:State) -> str:
        """
        Defines the sync mechanism type that should be executed according to the ego vehicle motion.

        :param lc_status: status of the lanelet change in the next time_window
        :param ego_id: id of the ego vehicle (string)
        :param current_state: the current state read from the commonroad motion planner
        
        :return: retval: the sync mechanism that should be followed while communicating from the interface to sumo
        """
        if self.conf.lane_change_sync == True:
            #Check the error between SUMO and CR positions
            cons_error = self._consistency_protection(ego_id, current_state)
            if cons_error=='CONSISTENCY_NO_ERROR': #CONSISTENCY_NO_ERROR means error below the configured margin
                if self._lc_inaction==0:
                    if lc_status == 'RIGHT_LC_STARTED':
                        self._lc_inaction = 1
                        retval='SYNC_SUMO_R_LC'
                    elif lc_status == 'LEFT_LC_STARTED':
                        self._lc_inaction = 1
                        retval='SYNC_SUMO_L_LC'
                    else:
                        retval = 'SYNC_MOVE_XY'
                else: #There is a lane change currently in action so do nothing and just increament the counter
                    self._lc_counter+=1
                    if self._lc_counter>=self._lc_duration_max:
                        self._lc_counter=0
                        self._lc_inaction=0
                    retval='SYNC_DO_NOTHING'
            else: #There is a consistency error so force the sync mechanism to moveToXY to return back to zero error
                retval = 'SYNC_MOVE_XY'
                self._lc_counter = 0
                self._lc_inaction = 0
        else:
            retval = 'SYNC_MOVE_XY'
        if self.conf.verbose == True:
            print('Sync Mechanism is: ' + retval)
            print('Lane change performed since ' + str(self._lc_counter))
        return retval

    def forward_info2sumo(self, planned_state:State, sync_mechanism:str, lc_duration:int, ego_id:str):
        """
        Forwards the information to sumo (either initiate moveToXY or changeLane) according to the sync mechanism.
        
        :param planned_state: the planned state from commonroad motion planner
        :param sync_mechanism: the sync mechanism that should be followed while communicating from the interface to sumo
        :param lc_duration: lane change duration, expressed in number of time steps
        :param ego_id: id of the ego vehicle
        """
        id_sumo = cr2sumo(ego_id, self.ids_cr2sumo)

        if sync_mechanism=='SYNC_MOVE_XY':
            position = planned_state.position
            sumo_angle = 90 - math.degrees(planned_state.orientation)

            self.vehicledomain.moveToXY(vehID=id_sumo, edgeID='dummy', lane=-1, x=position[0], y=position[1],
                                        angle=sumo_angle, keepRoute=2)

            self.vehicledomain.setSpeedMode(id_sumo, 0)
            self.vehicledomain.setSpeed(id_sumo, planned_state.velocity)

        elif sync_mechanism=='SYNC_SUMO_R_LC':
            # A lane change (right lane change) is just started, so we will initiate lane change request by traci
            # self.vehicledomain.setLaneChangeDuration(cr2sumo(ego_id, self.ids_cr2sumo), lc_duration)
            self.vehicledomain.setLaneChangeMode(id_sumo, 512)
            targetlane = self.vehicledomain.getLaneIndex(id_sumo) - 1
            self.vehicledomain.changeLane(id_sumo, targetlane, 0.1)
        elif sync_mechanism=='SYNC_SUMO_L_LC':
            # A lane change (left lane change) is just started, so we will initiate lane change request by traci
            # self.vehicledomain.setLaneChangeDuration(cr2sumo(ego_id, self.ids_cr2sumo), lc_duration)
            self.vehicledomain.setLaneChangeMode(id_sumo, 512)
            targetlane = self.vehicledomain.getLaneIndex(id_sumo) + 1
            self.vehicledomain.changeLane(id_sumo, targetlane, 0.1)
        elif sync_mechanism=='SYNC_DO_NOTHING':
            pass
        else:
            pass
