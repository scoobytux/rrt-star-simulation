'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
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

def motion(current_position, next_pt):
    '''
    motion model
    '''
    current_position[0] = approximately_num(next_pt[0])
    current_position[1] = approximately_num(next_pt[1])
    return current_position
  
def plot_robot(plt, x, y, yaw, config):  # pragma: no cover
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
    # run  python .\Paper_6_visibility_graph_global_sight.py -n 6 -g 40_30

    print(__file__ + " start!!")

    config.robot_type = robot_type
    robotvision = config.robot_vision
    # set same window size to easy capture pictures
    plt.figure(figsize=(7,7))
    menu_result = menu()
    run_times = menu_result[0]
    mapname = menu_result[1]
    start = menu_result[2] 
    goal = menu_result[3]
    
    # current position of robot
    cpos = np.array([start[0], start[1]])

    # read map 
    ob = read_map_csv(mapname) # obstacles
    
    # traversal sight to draw visible visited places
    traversal_sight = []

    # active open points [global]
    ao_gobal = [] 

    r_goal = True
    s_goal = True

    no_way_togoal = False
    
    visible_graph = graph_intiailze()
    visited_path = []

    # for display information
    run_count = 0
    
    print ("\n____Robot is reaching to goal: {0} from start {1}".format(goal, start))
    
    while True:
        run_count += 1
        center = [cpos[0], cpos[1] ]
        
        print ("\n_____Run times:{0}, at {1}".format(run_count, center))
        # clean old data
        next_pt = []
        
        # scan to get sights at local
        closed_sights, open_sights = scan_around(center, robotvision, ob, goal)
        
        # check if the robot saw or reach the goal
        r_goal, s_goal = check_goal(center, goal, config, robotvision, closed_sights)
        
        if not s_goal and not r_goal:
            # get local open points
            open_local_pts = []
            if len(open_sights) > 0:
                open_sights = np.array(open_sights)
                open_local_pts = open_sights[:, 2]    # open_local_pts
                print ("open_local_pts,", open_local_pts)
                for i in range( len(open_local_pts)):
                    open_local_pts[i][0] = approximately_num(open_local_pts[i][0])
                    open_local_pts[i][1] = approximately_num(open_local_pts[i][1])

            # check whether open local points are active 
            if len(open_local_pts) : # new local found
                if len(traversal_sight) == 0:
                    # ranks new local open points
                    ao_local_pts = open_local_pts
                    ranks_new = np.array([ranking(center, pt, goal) for pt in open_local_pts])
                    # active open points at local
                    ao_local = np.concatenate((ao_local_pts, ranks_new), axis=1)
                    # add local to global
                    ao_gobal = np.array(ao_local)
                else:
                    open_local_pts_status = [inside_global_true_sight(pt, robotvision, traversal_sight) for pt in open_local_pts]
                    ao_local_pts = open_local_pts[np.logical_not(open_local_pts_status)]
                    print ("ao_local_pts,", ao_local_pts)
                    if len(ao_local_pts) > 0:
                        ranks_new = np.array([ranking(center, pt, goal) for pt in ao_local_pts])
                        # active open points at local
                        ao_local = np.concatenate((ao_local_pts, ranks_new), axis=1)
                        # add local to global
                        ao_gobal = np.concatenate((ao_gobal, ao_local), axis=0)
                    else:
                        ao_local_pts = []
                        #print ("No new open point at this local")
                
                graph_insert(visible_graph, center, ao_local_pts)
            
            else:   # there is no direction 
                print ("there is no direction reaching the goal")
                
            # pick next point to make a move
            picked_idx, next_pt = pick_next(ao_gobal)
            
            # find the shortest skeleton path from current position (center) to next point
            skeleton_path = BFS_skeleton_path(visible_graph, tuple(center), tuple(next_pt))

            # remove picked point from active global open point
            if picked_idx != -1:
                ao_gobal= np.delete(ao_gobal, picked_idx, axis=0)
            else:
                print ("No way to reach the goal!")
                no_way_togoal = True
 
        else:
            next_pt = goal
            # find the shortest path from center to next point
            skeleton_path = [center, goal]
            
        # record the path
        traversal_sight.append([center, closed_sights, open_sights])
        if print_traversal_sight:
            print ("traversal_sight:", traversal_sight)
        
        asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sight, robotvision)
        #asp = remove_validation(asp)
        visited_path.append(asp)
        
        #make a move from current position
        if not no_way_togoal:
            cpos = motion(cpos, next_pt)  # simulate robot
        
        if show_animation:

            # clear plot
            plt.cla()

            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            # draw map obstacles 
            map_display(plt, mapname, ob)

            # show_traversal_sight
            if show_traversal_sight:
                i = 0
                for local in traversal_sight:
                    lcenter = local[0]  # center of robot at local
                    lc_sight = local[1] # closed sight at local
                    lo_sight = local[2] # open sight at local
                    plot_vision(plt, lcenter[0], lcenter[1], robotvision, lc_sight, lo_sight)
                    plt.text(lcenter[0] + 1, lcenter[1] + 1, "C({0})".format(i))
                    i = i + 1
            if 1:
                i = 0
                for local in traversal_sight:
                    lcenter = local[0]  # center of robot at local
                    plt.text(lcenter[0] -2, lcenter[1] + 1, "C({0})".format(i))
                    i = i + 1
            
            if show_robot:
                # plot robot 
                plot_robot(plt, center[0], center[1], 0, config)
            
            if 1: #show_goal:
                # plot goal
                plot_goal(plt, goal, r_goal, s_goal)            
            
            if show_start:
                # plot start
                plot_start(plt, start)
            
            # plot robot's vision at local (center)
            plot_vision(plt, center[0], center[1], robotvision, closed_sights, open_sights)
            
            if show_active_openpt and len(ao_gobal) > 0:
                plot_points(plt, ao_gobal, ls_aopt)
           
            if show_visible_graph:
                plot_visible_graph(plt, visible_graph, ls_vg)
                
            if show_visited_path:
                plot_paths(plt, visited_path, ls_vp, ls_goingp)
                
            if show_sketelon_path:
                plot_lines(plt, skeleton_path, ls_sp)
                
            if show_approximately_shortest_path:
                plot_lines(plt, asp, ls_asp)
                
            if show_critical_line_segments:
                plot_critical_line_segments(plt, critical_ls, ls_cls)            

            # display next point if existing
            if show_next_point:
                if len(next_pt) > 0:
                    plot_point(plt, next_pt, ls_nextpt)
                    #plt.text(next_pt[0] + 2, next_pt[1] + 1, "next point")
                    
            # to set equal make sure x y axises are same resolution 
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
        
        # Run n times for debugging
        if run_times == run_count:
            break
        
        # check reaching goal
        if r_goal:
            print("Goal!!")
            break
        if no_way_togoal:
            break
    print ("visited_path:", visited_path)            
    print("Done")

    plt.show()

if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    #main(robot_type=RobotType.circle)