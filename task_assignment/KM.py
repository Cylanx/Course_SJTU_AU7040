import numpy as np
from map_module.map import Map


# class KM:
#     def __init__(self,map_path,robots,tasks):
#         map_path = 'random-32-32-10.json'
#         self.demo_map = Map(map_path)
#         self.matrix = None
#         self.max_weight = 0
#         self.row, self.col = 0, 0  
#         self.size = 0   # 方阵大小
#         self.lx = None  # 左侧权值
#         self.ly = None  # 右侧权值
#         self.match = None   # 匹配结果
#         self.slack = None   # 边权和顶标最小的差值
#         self.visx = None    # 左侧是否加入增广路
#         self.visy = None    # 右侧是否加入增广路
#         self.robots = robots
#         self.tasks = tasks

#     def cal_cost_matrix(self):
#         pass

#     def compute(self):
#         cost_matrix = self.cal_cost_matrix()
#         sorted_rows, sorted_cols = [], []
#         # TODO: 根据代价矩阵计算最小匹配
#         return sorted_rows, sorted_cols

class KM:
    def __init__(self,map_path,robots,tasks):
        map_path = 'random-32-32-10.json'
        self.demo_map = Map(map_path)  
        print(self.demo_map)
        self.matrix = None
        self.max_weight = 0
        self.row, self.col = 0, 0  # 源数据行列
        self.size = 0   # 方阵大小
        self.lx = None  # 左侧权值
        self.ly = None  # 右侧权值
        self.match = None   # 匹配结果
        self.slack = None   # 边权和顶标最小的差值
        self.visx = None    # 左侧是否加入增广路
        self.visy = None    # 右侧是否加入增广路
        self.robots = robots
        self.tasks = tasks
        print(self.tasks)

    def reset_slack(self):
        self.slack.fill(self.max_weight + 1)

    def reset_vis(self):
        self.visx.fill(False)
        self.visy.fill(False)

    def find_path(self, x):
        self.visx[x] = True
        for y in range(self.size):
            if self.visy[y]:
                continue
            tmp_delta = self.lx[x] + self.ly[y] - self.matrix[x][y]
            if tmp_delta == 0:
                self.visy[y] = True
                if self.match[y] == -1 or self.find_path(self.match[y]):
                    self.match[y] = x
                    return True
            elif self.slack[y] > tmp_delta:
                self.slack[y] = tmp_delta

        return False

    def km_cal(self):
        for x in range(self.size):
            self.reset_slack()
            while True:
                self.reset_vis()
                if self.find_path(x):
                    break
                else:  # update slack
                    delta = self.slack[~self.visy].min()
                    self.lx[self.visx] -= delta
                    self.ly[self.visy] += delta
                    self.slack[~self.visy] -= delta

    def cal_cost_matrix(self):
        self.matrix = np.zeros((len(self.robots),len(self.tasks)))
        self.row, self.col = self.matrix.shape  # 源数据行列
        self.size = max(self.row, self.col)
        for i in range(len(self.robots)):
            for j in range(len(self.tasks)):
                start = self.robots[i].pos()
                end = self.demo_map.id2pos(self.tasks[j]['goal'])
                cost = np.abs(end[0]-start[0])+np.abs(end[1]-start[1])
                self.matrix[i,j] = 64-cost

    def compute(self):
        self.cal_cost_matrix()
        print(self.matrix)
        self.max_weight = self.matrix.sum()
        self.lx = self.matrix.max(1)
        self.ly = np.array([0] * self.size, dtype=int)
        self.match = np.array([-1] * self.size, dtype=int)
        self.slack = np.array([0] * self.size, dtype=int)
        self.visx = np.array([False] * self.size, dtype=bool)
        self.visy = np.array([False] * self.size, dtype=bool)

        self.km_cal()

        match = [i[0] for i in sorted(enumerate(self.match), key=lambda x: x[1])]
        sorted_rows, sorted_cols = [], []
        for i in range(self.row):
            sorted_rows.append(i)
            sorted_cols.append(match[i] if match[i] < self.col else -1) # 没有对应的值给-1
        return sorted_rows, sorted_cols