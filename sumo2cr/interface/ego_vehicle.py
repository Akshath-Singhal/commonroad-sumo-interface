import warnings
from typing import Dict, List, Union
import numpy as np
import copy
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import State, Trajectory

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["ZIM Projekt ZF4086007BZ8"]
__version__ = "1.0.0"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

class EgoVehicle:
    """
    Interface object for ego vehicle.
    Intended use: Get current state with EgoVehicle.current_state() and set planned trajectory with
    EgoVehicle.set_planned_trajectory(planned_trajectory).
    """
    def __init__(self, id, initial_state:State, delta_steps: int, width:float=2.0, length:float=5.0, planning_problem:PlanningProblem=None):
        self.id = id
        self._width = width
        self._length = length
        self._initial_state = initial_state
        self._state_dict: Dict[State] = dict()  # collects driven states
        self.shape = Rectangle(self.length,self.width,center=np.array([0,0]),orientation=0.0)
        self._driven_trajectory = None  # returns trajectory object of driven
        self._planned_trajectories: Dict[List[State]] = {}  # collects trajectories from planner for every time step
        self._current_time_step = initial_state.time_step
        self.delta_steps = delta_steps

        if planning_problem is not None:
            assert self.initial_state == planning_problem.initial_state, '<sumo_simulation/ego_vehicle> planning problem\' initial_state must equal initial_state'
            self.planning_problem = planning_problem

    def set_planned_trajectory(self, planned_state_list:List[State]) -> None:
        """
        Set planned trajectory beginning with current time step.
        :param planned_state_list:
        :param initial_time_step. If None, add at current time step
        :return:
        """

        assert len(planned_state_list) >= self.delta_steps,\
            'planned_trajectory must contain at least {} states, but contains {}. (See delta_steps in config file)'\
                .format(self.delta_steps,len(planned_state_list))
        assert 1 == planned_state_list[0].time_step,\
            'planned_trajectory must start at next time_step ({}) but starts at time_step {}'\
                .format(1, planned_state_list[0].time_step)
        self._planned_trajectories[self.current_time_step] = planned_state_list
        self.add_state(planned_state_list[0])

    @property
    def get_planned_trajectory(self) -> List[State]:
        """Get planned trajectory from current time step"""
        return self._planned_trajectories[self.current_time_step]

    def get_dynamic_obstacle(self,time_step: Union[int,None]=None) -> DynamicObstacle:
        """
        If time step is false, add complete driven trajectory.
        If time step is int: start from given step and add planned trajectory
        :param time_step: initial time step of vehicle
        :return: DynamicObstacle
        """
        if time_step is False:
            return DynamicObstacle(self.id,obstacle_type=ObstacleType.CAR,
                                    obstacle_shape=Rectangle(self.length,self.width,center=np.array([0,0]),orientation=0.0),
                                    initial_state=self.initial_state, prediction=self.driven_trajectory)
        elif isinstance(time_step,int):
            if time_step in self._state_dict:
                if time_step in self._planned_trajectories:
                    prediction = TrajectoryPrediction(Trajectory(self._planned_trajectories[time_step][0].time_step,
                                                                 self._planned_trajectories[time_step]),
                                         self.shape)
                else:
                    prediction = None
                return DynamicObstacle(self.id, obstacle_type=ObstacleType.CAR,
                                obstacle_shape=Rectangle(self.length, self.width, center=np.array([0, 0]), orientation=0.0),
                                initial_state=self.get_state_at_timestep(time_step), prediction=prediction)
        else:
            raise ValueError('time needs to be None or integer')

    def get_planned_state(self, delta_step: int=0):
        planned_state:State = copy.deepcopy(self._planned_trajectories[self.current_time_step][0])
        if self.delta_steps > 1:
            # linear interpolation
            for state in planned_state.attributes:
                curr_state = getattr(self.current_state, state)
                next_state = getattr(planned_state, state)
                setattr(planned_state, state, curr_state + (delta_step+1)/self.delta_steps * (next_state-curr_state))

        return planned_state

    @property
    def current_state(self) -> State:
        if self.current_time_step == self.initial_state.time_step:
            return self.initial_state
        else:
            return self._state_dict[self.current_time_step]

    def get_state_at_timestep(self,time_step: int):
        if time_step == self.initial_state.time_step:
            return self.initial_state
        else:
            state = self._state_dict[time_step]
            state.time_step = 0
            return state

    @current_state.setter
    def current_state(self,current_state):
        raise PermissionError('current_state cannot be set manually, use set_planned_trajectory()')

    @property
    def current_time_step(self):
        return self._current_time_step

    @current_time_step.setter
    def current_time_step(self, current_time_step):
        raise PermissionError('current_state cannot be set manually, use set_planned_trajectory()')

    @property
    def goal(self):
        return self.planning_problem.goal

    def add_state(self,state):
        self._state_dict[self._current_time_step+1] = state

    @property
    def driven_trajectory(self) -> TrajectoryPrediction:
        """
        Returns trajectory prediction object for driven trajectory (mainly for plotting)
        :return:
        """
        state_dict_tmp ={}
        for t, state in self._state_dict.items():
            state_dict_tmp[t] = state
            state_dict_tmp[t].time_step = t

        sorted_list = sorted(state_dict_tmp.keys())
        state_list = [state_dict_tmp[key] for key in sorted_list]
        return TrajectoryPrediction(Trajectory(self.initial_state.time_step+1, state_list),self.shape)

    @driven_trajectory.setter
    def driven_trajectory(self, _):
        if hasattr(self, '_driven_trajectory'):
            warnings.warn('driven_trajectory of vehicle cannot be changed')
            return

    @property
    def width(self):
        return self._width

    @width.setter
    def width(self,width):
        if hasattr(self, '_width'):
            warnings.warn('width of vehicle cannot be changed')
            return

    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, length):
        if hasattr(self, '_length'):
            warnings.warn('length of vehicle cannot be changed')
            return

    @property
    def initial_state(self):
        return self._initial_state

    @initial_state.setter
    def initial_state(self, _):
        if hasattr(self, '_initial_state'):
            warnings.warn('initial_state of vehicle cannot be changed')
            return