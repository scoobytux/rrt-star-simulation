'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh
'''
import math
import matplotlib.pyplot as plt
import numpy as np

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import *
from Robot_sight_lib import *
from Robot_map_lib import map_display
from Robot_csv_lib import read_map_csv
from Program_config import *
from Robot_control_panel import *



config = Config()

def motion(x, u, dt):
    '''
    motion model
    '''

    #x[2] += u[1] * dt
    #x[0] += u[0] * math.cos(x[2]) * dt
    #x[1] += u[0] * math.sin(x[2]) * dt
    #x[3] = u[0]
    #x[4] = u[1]
    x[0] += u[0] 
    x[1] += u[1] 
    return x


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)
  
def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")
                  
def plot_AH_paths(AH_paths, goal):
    for path in AH_paths:
        AH_sp = path[0]       # start point
        AH_nextps = path[1]   # all next points
        blind_ps = path[2]    # all blind points
        goal_appear = path[3] # goad appear
        #print (path)
        # draw AH path segment
        for AH_point in AH_nextps:
            plt.plot((AH_sp[0],AH_point[0]), (AH_sp[1], AH_point[1]), "--or")
        
        # draw blind points segment
        for bp in blind_ps:
            plt.plot((AH_sp[0],bp[0]), (AH_sp[1], bp[1]), "--y")
            
        if goal_appear:
            plt.plot((AH_sp[0],goal[0]), (AH_sp[1], goal[1]), "-r")
            
def saw_goal(center, radius, t_sight, goal):
    return inside_local_true_sight(goal, center, radius, t_sight)

def reached_goal(center, goal, config):
    return point_dist(center, goal) <= config.robot_radius
    
def check_goal(center, goal, config, radius, t_sight):
    s_goal = False
    r_goal = point_dist(center, goal) <= config.robot_radius
    if not r_goal:
        s_goal = saw_goal(center, radius, t_sight, goal)
    return r_goal, s_goal
    
def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")

    menu_result = menu()
    run_times = menu_result[0]
    mapname = menu_result[1]
    start = 0 
    goal = 17

    ob = read_map_csv(mapname) 
    ob = np.array(ob)
    
    
    print ("OB", ob)
    print ("OB1", ob[:,2])
    print ("OB2", ob[0])
          
          
    # draw map obstacles 
    map_display(plt, mapname, ob)
    #start point
    plot_point(plt, ob[:,start], ls="*m")
    #goal
    plot_point(plt, ob[:,goal], ls="*r")
    
    plt.plot((ob[0][0],ob[0][-1]), (ob[1][0],ob[1][-1]), "-b")
    #plot_arrow(x[0], x[1], x[2])
    plt.axis("equal") # make sure ox oy axises are same resolution open local points
     
    print("Done")

    plt.show()

if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    #main(robot_type=RobotType.circle)
