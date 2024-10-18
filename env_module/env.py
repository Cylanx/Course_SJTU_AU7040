import os
from tqdm import tqdm
import random
from map_module.map import Map
from env_module.robot import Robot
from env_module.board import Board
from path_planner import AstarPlanner,STAstarPlanner
from action_executor import ActionExecutor
from logger_module.Logger import ConfigurableLogger
from task_assignment.KM import KM

class Env:
    def __init__(self, map_path, slover='astar', seed=0):
        self.demo_map = Map(map_path)
        self.board = Board(self.demo_map, 1000)
        self.seed = seed
        self.solvers = {
            'astar': AstarPlanner,
            #'your_Alg': Your/Own/Plannner/Class,
        }
        # =========================== TODO: 实现任意one-shot MAPF算法，实现多机器人无碰完成任务 ===========================
        # self.planner = self.slovers['your_Alg'](self.demo_map)

        # 示例：A* planner
        self.planner = self.solvers[slover](self.demo_map)

    # 随机生成任务点和小车起始点
    def generate_random_tasks(self, num_tasks, seed=0):
        normal_nodes = self.demo_map.nodes_by_type('normal')
        if len(normal_nodes) < num_tasks * 2:
            raise ValueError("Nodes are not enough for unique task and robot starts")
        random.seed(seed)
        chosen_nodes = random.sample(normal_nodes, num_tasks * 2)
        part_size = num_tasks
        robot_start_points = chosen_nodes[:part_size] 
        task_goal_points = chosen_nodes[part_size:]
        tasks = []
        for goal in task_goal_points:
            tasks.append({
                'goal': goal
            })
        robots = {
            i: Robot(i, start, 1, self.demo_map)
            for i, start in enumerate(robot_start_points)
        }
        return tasks, robots
    
    def calculate_total_path_length(self, paths):
        total_length = 0
        for path in paths.values():
            total_length += len(path) - 1
        return total_length
    
    
    def generate_task_assignments(self, robots, tasks, robot_start_id_dict, robot_goal_id_dict):

        # ============================== TODO: 任务分配类KM实现 ======================================
        km = KM(self.demo_map, robots, tasks) 
        # =====================================================================================

        row, col = km.compute()
        task_assignments = {}

        for robot_id, task_idx in zip(row, col):
            if task_idx <= len(tasks):
                task = tasks[task_idx]
                task_assignments[robots[robot_id].id()] = {
                    'robot_id': robots[robot_id].id(),
                    'task_id': task_idx,
                    'start_id':robots[robot_id].node_id(),
                    'goal_id':task['goal']
                }
        for robot_id, assignment in task_assignments.items():
            start_node_id = assignment['start_id']
            goal_node_id = assignment['goal_id']
            robot_start_id_dict[robot_id] = start_node_id
            robot_goal_id_dict[robot_id] = goal_node_id

        for robot in robots.values(): 
            robot_id = robot.id()  
            goal_id = task_assignments.get(robot_id)['start_id']  
            if goal_id:  
                robot.set_goal_id(goal_id)  



    def run(self, log_dir):
        logger = ConfigurableLogger('./logger_module/logging.conf', log_dir)

        fig_save_dir = os.path.join(log_dir, 'figs')
        if not os.path.exists(fig_save_dir):
            os.makedirs(fig_save_dir)
        robot_start_id_dict, robot_goal_id_dict = dict(), dict()
        tasks = list()
        num_tasks = 10
        robot_num = 10

        robots = dict()
        tasks, robots = self.generate_random_tasks(num_tasks, seed=self.seed)

        # =============================== 任务分配算法调用, 编写好之后取消注释 ===========================
        self.generate_task_assignments(robots, tasks, robot_start_id_dict, robot_goal_id_dict) 
        # ==============================================================================


        # =============================== 任务分配示例, 编写好KM之后将其注释 ===========================
        # 顺序分配 
        # for i, task in enumerate(tasks):
        #     goal_id = task['goal']
        #     print("goal_id: ", goal_id)
        #     robot_goal_id_dict[i] = goal_id
        #     robot_start_id_dict[i] = robots[i].node_id()
        # ==============================================================================
  
        executor = ActionExecutor(robots)

        # 路径规划planner调用 
        robot_paths = self.planner.plan_paths(robot_start_id_dict, robot_goal_id_dict)
        print("robot_paths: ", robot_paths)

        # print("\n matched list")
        # for robot_id, assignment in task_assignments.items():
        #     print(f"Robot {robot_id} taskID: {assignment['task_id']}, StartID: {assignment['start_id']},goalID: {assignment['goal_id']}")

        SoC = self.calculate_total_path_length(robot_paths)
        print("********************** The Sum of Cost are :", SoC,'******************************')
        self.board.show_robots(robots, robot_paths, filename='{}/{}'.format(fig_save_dir, 0), plt_show=False)
        if len(robot_paths) != robot_num:
            raise ValueError("Not all robots have a planned path")
        for robot_id in robots:
            robots[robot_id].set_path(robot_paths[robot_id])


        # ===============================  任务执行 ===========================
        moving_robots_num = 0
        max_steps = 10000
        for timestep in tqdm(range(max_steps)):
            moving_robots_num = len([robot for robot in robots.values() if len(robot.path()) > 0])
            if moving_robots_num == 0:
                print("All robots have reached their goals!")
                break
            logger.info('Timestep {}: '.format(timestep) + str(executor))
            executor.execute_actions()

            # 注意：这里是可视化部分，可能运行较慢。可以直接注释掉/减少可视化元素以提高运行速度
            self.board.show_robots(robots, robot_paths,
                                   filename='{}/{}'.format(fig_save_dir, timestep + 1), plt_show=False)
