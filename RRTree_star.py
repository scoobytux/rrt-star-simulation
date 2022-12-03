import numpy as np

import pickle
import sys

from Tree import Tree, Node
from RRTree import RRTree
from RRT_draw_lib import Plot_RRT
from Robot_math_lib import *

from RRT_user_input import menu_RRT
from Program_config import *
from Obstacles import Obstacles

sys.setrecursionlimit(12000)

class RRTree_star(RRTree):

    """ RRTree class from Tree class """
    def __init__(self, root, step_size = 5, radius=5, random_area=(0, 100), sample_size = 100):
        super().__init__(root, step_size, radius, random_area, sample_size)

    ''' add node to RRTree , return new_node and its neighbour_node(s)'''
    def add_node_RRTstar(self, accepted_coordinate):
        # find all neighbours of node_coordinate
        neighbour_nodes = self.neighbour_nodes(accepted_coordinate, self.radius)
        # pick nearest neighbour as new_node's parent
        nearest_neighbour_node = self.neighbours_smallest_cost(accepted_coordinate, neighbour_nodes)


        if nearest_neighbour_node is not None: # found a neighbour to link to 
            # allocate new node then link to RRTree
            new_node = Node(accepted_coordinate)
            new_node.add_neighbours(neighbours=neighbour_nodes)
            self.add_node(new_node=new_node)
            self.add_edge(parent_node=nearest_neighbour_node, node=new_node)
            return new_node, neighbour_nodes, nearest_neighbour_node
        return None, None, None        

    def build(self,  goal_coordinate, plotter: Plot_RRT=None, obstacles=None, show_animation=False):
        first_saw_goal = False

        for i in range(1, self.sampling_size):
            
            while True:
                # generate random coordinate in sampling area = [min, max]
                rand_coordinate = self.random_coordinate()
                
                # orient to goal sometime :))
                if i %100 == 0 and not self.reach_goal: # bias to goal sometime
                    rand_coordinate = np.array(goal_coordinate)

                # bring closer random coordinate to tree 
                accepted_coordinate = self.bring_closer(rand_coordinate=rand_coordinate)

                if not obstacles.check_point_collision(accepted_coordinate, obstacles.obstacles_line_segments):
                    break

            
            # if tree first saw given goal , instead of adding new random , add goal
            if not first_saw_goal:
                nn_goal = self.saw_goal(goal_coordinate)    # return nearst neighbour node of goal
                if nn_goal is not None: # existing a node nearby goal
                    first_saw_goal = True
                    self.reach_goal = True
                    accepted_coordinate = goal_coordinate
            
            # add and link node to tree
            new_node, neighbour_nodes, nearest_neighbour_node  = self.add_node_RRTstar(accepted_coordinate)

            ''' rewire for RRT* '''
            self.rewire(node=new_node, neighbour_nodes=neighbour_nodes)

            if self.reach_goal:
                goal_node = self.get_node_by_coords(goal_coordinate)
                self.path_to_goal = self.path_to_root(goal_node)
                self.total_goal_cost = goal_node.cost

            ''' for display '''
            if i == self.sampling_size - 1:
                plotter.build_tree_animation(num_iter= i, Tree= self, obstacles=obstacles,  goal_coords=goal_coordinate, \
                    start_coords=self.root.coords, rand_coordinate= rand_coordinate, rand_node=new_node, 
                    neighbour_nodes=neighbour_nodes, nearest_neighbour_node=nearest_neighbour_node, color_tree=TreeColor.by_cost)
        

if __name__ == '__main__':
    ''' initial parameters '''
    try:
        with open('rrt_star.pickle', 'rb') as f: 
            RRT_star = pickle.load(f)
    except:
        read_tree = False
        print("Create and save the tree ...")
    else:
        read_tree = True

    # get user input
    menu_result = menu_RRT()
    # get start_cooridinate and goal_coordinate
    start_cooridinate = menu_result.sx, menu_result.sy
    goal_coordinate = menu_result.gx, menu_result.gy

    step_size = menu_result.step_size
    radius = menu_result.radius
    sample_size = menu_result.ss
    map_name = menu_result.m
    world_name = None

    ''' Running '''
    # set same window size to capture pictures
    plotter = Plot_RRT(title="Rapidly-exploring Random Tree Star (RRT*)")
    plotter.set_equal()

    obstacles = Obstacles()
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles.read(world_name, map_name)
    obstacles.line_segments()

    # find working space boundary
    x_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0])
    x_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1])
    y_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0])
    y_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1])
    random_area = ([x_min, y_min], [x_max, y_max])

    if not read_tree:
        ''' build tree '''
        start_node = Node(start_cooridinate, cost=0)            # initial root node, cost to root = 0
        RRT_star = RRTree_star(root=start_node, step_size=step_size, radius=radius, 
                        random_area=random_area, sample_size=sample_size)
        RRT_star.build(goal_coordinate=goal_coordinate, plotter=plotter, obstacles=obstacles, show_animation=True)

        with open('rrt_star.pickle', 'wb') as f:
            pickle.dump(RRT_star, f)
    else:
        plotter.show_tree(num_iter=sample_size, Tree=RRT_star, obstacles=obstacles,  goal_coords=goal_coordinate, \
                    start_coords=start_cooridinate, color_tree=TreeColor.by_cost)

    plotter.show()
