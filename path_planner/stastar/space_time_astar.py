import networkx as nx
from typing import Tuple, List, Dict,Set
from map_module.map import Map
from path_planner.path_planner import PathPlanner
from .state import State
from heapq import heappush, heappop
import numpy as np


class STAstarPlanner(PathPlanner):

    def __init__(self, map_input: Map):
        super().__init__(map_input)
        self.map = map_input

    def h(self, start: int, goal: int) -> int:
        start = self.map.id2pos(start)
        goal = self.map.id2pos(goal)
        return np.abs(goal[0]-start[0])+np.abs(goal[1]-start[1])


    def plan_path(self, start: int, goal: int, dynamic_obstacles: Dict[int, list],max_iter: int = 1000) -> List[int]:
        # Prepare dynamic obstacles
        dynamic_obstacles = dict((k, np.array(list(v))) for k, v in dynamic_obstacles.items())
        # Assume dynamic obstacles are agents with same radius, distance needs to be 2*radius
        def occupying(pos: tuple, time: int) -> bool:
            occupying = np.array([obstacle==pos for obstacle in dynamic_obstacles.setdefault(time, [])])
            return occupying.any()

        # Initialize the start state
        s = State(start, 0, 0, self.h(start, goal))

        open_set = [s]
        closed_set = set()

        # Keep track of parent nodes for reconstruction
        came_from = dict()

        iter_ = 0
        while open_set and iter_ < max_iter:
            iter_ += 1
            current_state = open_set[0]  # Smallest element in min-heap
            if current_state.equal_to(goal):
                print('STA*: Path found after {0} iterations'.format(iter_))
                return self.reconstruct_path(came_from, current_state)

            closed_set.add(heappop(open_set))
            epoch = current_state.time + 1
            for neighbour in self.map.G.neighbors(current_state.id):
                neighbour_state = State(neighbour, epoch, current_state.g_score + 1, self.h(neighbour, goal))
                # Check if visited
                if neighbour_state in closed_set:
                    continue

                # Avoid obstacles
                if occupying(neighbour, epoch):
                    continue

                # Add to open set
                if neighbour_state not in open_set: 
                    came_from[neighbour_state] = current_state
                    heappush(open_set, neighbour_state)
        return []

    def plan_paths(self, starts: Dict[int, int], goals: Dict[int, int]) -> Dict[int, List[int]]:
        robot_paths = dict()
        if len(starts) != len(goals):
            print('Warning: Starts and Goals do not match!')
            return robot_paths
        dynamic_obstacles = dict()
        for robot_id in starts:
            start_id = starts[robot_id]
            goal_id = goals[robot_id]
            path = self.plan_path(start_id, goal_id,dynamic_obstacles,1000)
            robot_paths[robot_id] = path
            for time,pos in zip(range(len(path)),path):
                time_obstacles = dynamic_obstacles.setdefault(time,[])
                time_obstacles.append(pos)
                dynamic_obstacles[time] = time_obstacles
        return robot_paths
    
    '''
    Reconstruct path from A* search result
    '''
    def reconstruct_path(self, came_from: Dict[State, State], current: State) -> np.ndarray:
        total_path = [current.id]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current.id)
        return total_path[::-1]
    