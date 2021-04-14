
from Original_APF import APF, Vector2d
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Circle
import random
import time
import numpy as np

def check_vec_angle(v1: Vector2d, v2: Vector2d):
    v1_v2 = v1.deltaX * v2.deltaX + v1.deltaY * v2.deltaY
    angle = math.acos(v1_v2 / (v1.length * v2.length)) * 180 / math.pi
    return angle


class APF_Improved(APF):
    def __init__(self, start: (), goal: (), obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = is_plot
        self.delta_t = 0.01

    def repulsion(self):

        rep = Vector2d(0, 0)
        for obstacle in self.obstacles:
            # obstacle = Vector2d(0, 0)
            obs_to_rob = self.current_pos - obstacle
            rob_to_goal = self.goal - self.current_pos
            if (obs_to_rob.length > (self.rr)):
                pass
            else:
                rep_1 = Vector2d(obs_to_rob.direction[0], obs_to_rob.direction[1]) * self.k_rep * (
                        1.0 / obs_to_rob.length - 1.0 / self.rr) / (obs_to_rob.length ** 2) * (rob_to_goal.length ** 2)
                rep_2 = Vector2d(rob_to_goal.direction[0], rob_to_goal.direction[1]) * self.k_rep * ((1.0 / obs_to_rob.length - 1.0 / self.rr) ** 2) * rob_to_goal.length
                rep +=(rep_1+rep_2)
        return rep


if __name__ == '__main__':

    k_att, k_rep = 0.5, 1.0
    rr = 5
    step_size, max_iters, goal_threashold = 0.2, 500, 0.2
    step_size_ = 0.2



    start, goal = (0, 0), (1.4, 42.00)
    is_plot = True
    if is_plot:
        img = plt.imread("bpop.png")
        fig, ax = plt.subplots()
        fig.set_size_inches(15, 10)
        plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)
        ax.imshow(img, extent=[-41.22, 60.28, -4.00, 46.75], alpha=0.96)
        plt.xlim([-41.22, 60.28])
        plt.ylim([-4.00, 46.75])
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.grid(alpha=0.2)
        plt.plot(start[0], start[1], '*r')
        plt.plot(goal[0], goal[1], '*r')

        #axes2 = fig.add_axes([0.07, 0.55, 0.35, 0.3])


        # fig = plt.figure(figsize=(16, 16))
        # subplot = fig.add_subplot(111)
        # subplot.set_xlabel('X-distance: m')
        # subplot.set_ylabel('Y-distance: m')
        # subplot.plot(start[0], start[1], '*r')
        # subplot.plot(goal[0], goal[1], '*r')

        corc = 40
        sine_degt1 = math.sin(math.radians(corc))
        cos_degt1 = math.cos(math.radians(corc))
        z1 = 10 * sine_degt1
        p1 = 10 * cos_degt1
        Pio = plt.quiver(2750, 550, z1, p1, scale=110, color='white', pivot='middle')







    obs = [[-4, 8], [2, 5], [3, 2.5], [6, 20.00], [6, 30.00], [10.0, 12.60], [1.1, 24.00], [1.4, 14.00]]

    print('obstacles: {0}'.format(obs))
    for i in range(0):
        obs.append([random.uniform(2, goal[1] - 1), random.uniform(2, goal[1] - 1)])

    if is_plot:
        for OB in obs:
            circle = Circle(xy=(OB[0], OB[1]), radius=(rr/2), alpha=0.3)
            ax.add_patch(circle)
            plt.plot(OB[0], OB[1], 'xk')
    t1 = time.time()
    #for i in range(1000):

    # path plan
    if is_plot:
        apf = APF_Improved(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    else:
        apf = APF_Improved(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    apf.path_plan()
    if apf.is_path_plan_success:
        path = apf.path
        path_ = []
        i = int(step_size_ / step_size)
        while (i < len(path)):
            path_.append(path[i])
            i += int(step_size_ / step_size)

        if path_[-1] != path[-1]:
            path_.append(path[-1])
        print('planed path points:{}'.format(path_))
        print('path plan success')


        if is_plot:
            px, py = [K[0] for K in path_], [K[1] for K in path_]
            print('pox ',px)
            print('length of pox : ', len(px))
            new_px =[]
            new_py =[]
            for i in range(len(px)):
                new_px.append(round(px[i],3))
                new_py.append(round(py[i], 3))

            print('New Px : ', new_px)
            print('New Py : ', new_py)

            DS_px = []
            DS_py = []
            for i in range(len(new_px)):
                if i%4 == 0:
                    DS_px.append(round(new_px[i], 3))
                    DS_py.append(round(new_py[i], 3))
            print('Length of DSx : ', len(DS_px))
            print('Length of DSy : ', len(DS_py))
            print('DS px : ',DS_px )
            print('DS py : ', DS_py)

            print('poy',py)
            print('length of poy : ', len(py))

            ax.plot(px, py, '^k')


            #plt.plot(DS_px, DS_py, 'o', color='red')
            ows_Info = plt.text(37.50, 23.00,
                                f'Path Planning Success \nThe Way Points for the route: ',
                                fontsize=10, color='black')

            ows_Info2 = plt.text(37.50, 20.00,
                                f'Downs SAp \nThe Way Points for the route: {DS_px}',
                                fontsize=10, color='black')

            plt.show()

            # axes2.plot(DS_px, DS_py, 'r')
            # axes2.set_xlabel('X Axis')
            # axes2.set_ylabel('Y Axis')
            # axes2.set_title('Square function')
            # plt.show()


    else:
        print('path plan failed')
    t2 = time.time()
    print('Time:{},time2:{}'.format(t2-t1,(t2-t1)/1000))
