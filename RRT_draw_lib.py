from Obstacles import Obstacles
from Plot_base_lib import Plot_base
from Program_config import *
from Queue_class import Priority_queue
from Robot_class import Robot
from Tree import Node, Tree
import numpy as np

class Plot_RRT(Plot_base):
    def __init__(self, size=(7,7), title="Robotic self"):
        super().__init__(size, title)

    ''' plot connection between 2 nodes'''
    def connection(self, nodeA, nodeB, ls="-b", lw=1):
        self.line_segment( (nodeA.coords, nodeB.coords), ls=ls, lw=lw)
    
    ''' plot edges from node to its children '''
    def tree_edges(self, node, ls=ls_tree_edge):
        for node_children in node.children:
            self.line_segment( (node.coords, node_children.coords), ls, lw=0.2)

    ''' plot edges from node to its children '''
    def tree_neighbour_edges(self, node, ls=ls_tree_edge):
        children_node = node.children
        neighbour_nodes = node.neighbours
        for n_node in neighbour_nodes:
            if n_node in children_node:
                lw = 0.7
                ls = "-b"
            else:
                lw = 0.2
                ls = ":"
            self.line_segment( (node.coords, n_node.coords), ls=ls, lw=lw)

    ''' plot a tree's node '''
    def tree_node(self, node:Node, ls_active=ls_tree_node_active, ls_inactive= ls_tree_node_inactive,\
            ls_visited = ls_tree_node_visited):
        
        if node.visited:
            self.point(node.coords, ls=ls_visited)
        elif node.active:
            self.point(node.coords, ls=ls_active)
        else:
            self.point(node.coords, ls=ls_inactive)

    ''' plot tree (all edges and vertices) from given node as tree's root '''

    def tree(self, tree, node_en=True, edge_en=True, neighbour_en = False, color_mode=TreeColor.no):
        nodes_coords  = []
        nodes_lmc = []
        nodes_cost = []
        MAX_RANGE = max(tree.sampling_area[0][1],tree.sampling_area[1][1])*1.5
        for node in tree.all_nodes():   # get all nodes
            nodes_coords.append(node.coords)
            if node.lmc == float("inf"):
                nodes_lmc.append(MAX_RANGE)    
            else:
                nodes_lmc.append(node.lmc)
            if node.cost == float("inf"):
                nodes_cost.append(MAX_RANGE)   
            else:
                nodes_cost.append(node.cost)

            # draw edge
            if neighbour_en: # plot all edges between nodes and their neighbours
                self.tree_neighbour_edges(node)
            elif edge_en:
                self.tree_edges(node)      # plot all edges between nodes and their children            

            if node_en and color_mode==TreeColor.no:
                self.tree_node(node)   # plot nodes

        nodes_coords = np.array(nodes_coords)
        if node_en:
            if color_mode == TreeColor.by_lmc:
                self.point_colors(nodes_coords, nodes_lmc, colormap="Dark2")
            elif color_mode == TreeColor.by_cost:
                self.point_colors(nodes_coords, nodes_cost, colormap="Dark2")
            

    ''' plot path_coords that is sequence of nodes coordinate'''
    def path_coords(self, path, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for i in range(len(path)-1):
            nodeA = path[i]
            nodeB = path[i+1]
            self.point(nodeA, ls_node)
            self.line_segment( (nodeA, nodeB), ls_edge, lw=lw)

    ''' plot paths which contains many paths_coords'''
    def paths_coords(self, paths, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for path in paths:
            self.path_coords(path, ls_node = ls_node, ls_edge=ls_edge, lw=lw)

    ''' plot path that is sequence of nodes'''
    def path(self, path, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for i in range(len(path)-1):
            nodeA = path[i]
            nodeB = path[i+1]
            self.tree_node(nodeA, ls_node)
            self.connection(nodeA, nodeB, ls_edge, lw=lw)

    ''' plot paths which contains many paths'''
    def paths(self, paths, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for path in paths:
            self.path(path, ls_node = ls_node, ls_edge=ls_edge, lw=lw)

    ''' plot path that is sequence of nodes'''
    def path_cost(self, path, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for i in range(len(path)-1):
            nodeA = path[i]
            nodeB = path[i+1]
            self.tree_node(nodeA, ls_node)
            self.connection(nodeA=nodeA, nodeB=nodeB, ls=ls_edge, lw=lw)
            self.text(nodeA.coords, "{0:.2f}".format(nodeA.cost))   # cost
            self.text((nodeA.coords[0],nodeA.coords[1]-1), "{0:.2f}".format(nodeA.lmc))   # cost



    def paths_cost(self, paths, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for path in paths:
            self.path_cost(path, ls_node = ls_node, ls_edge=ls_edge, lw=lw)

    ''' plot all info ( number of iteration, cost, path, tree) '''
    def build_tree_animation(self, num_iter, Tree, obstacles,  goal_coords, start_coords, rand_coordinate=None, rand_node=None,\
            neighbour_nodes=[], nearest_neighbour_node= None, color_tree=TreeColor.no):
        
        # clear plot
        self.clear()

        ''' draw map obstacles/world '''            
        # prepare title
        status_title = self.prepare_title(num_iter, Tree.total_goal_cost)

        # plot map
        if obstacles is None:
            obstacles = Obstacles() # replace with empty obstacle

        if show_map:
            self.show_map(world_name=None, obstacles=obstacles, plot_title=status_title)
        
        # draw current tree
        self.tree(Tree, color_mode=color_tree, neighbour_en=True)

        # draw goal
        self.goal(goal_coords, Tree.reach_goal, None)

        # draw start 
        self.start(start_coords)

        ''' animation new node '''
        if rand_coordinate is not None:
            self.point(rand_coordinate, ls=ls_rand_coordinates)
            self.line_segment((rand_coordinate, rand_node.coords), ls=ls_ls_to_nn)
            
        if rand_node is not None:
            self.vision_area(rand_node.coords, Tree.radius)
            self.point(rand_node.coords, ls=ls_random_node)
        if nearest_neighbour_node is not None:
            self.point(nearest_neighbour_node.coords, ls=ls_nearest_n_node)
        for neighbour_node in neighbour_nodes:
            self.point(neighbour_node.coords, ls=ls_neighbour_node)
        
        # path from root to goal
        self.path(Tree.path_to_goal, ls_edge=ls_ahead_path, ls_node=ls_path_node, lw=lw_path)
        self.pause(0.001)
    
    def show_tree(self, num_iter, Tree, obstacles,  goal_coords, start_coords, color_tree=TreeColor.no):
        
        # clear plot
        self.clear()

        ''' draw map obstacles/world '''            
        # prepare title
        status_title = self.prepare_title(num_iter, Tree.total_goal_cost)

        # plot map
        if obstacles is None:
            obstacles = Obstacles() # replace with empty obstacle

        if show_map:
            self.show_map(world_name=None, obstacles=obstacles, plot_title=status_title)
        
        # draw current tree
        self.tree(Tree, color_mode=color_tree, neighbour_en=True)

        # draw goal
        self.goal(goal_coords, Tree.reach_goal, None)

        # draw start 
        self.start(start_coords)

        ''' animation new node '''
        # if rand_coordinate is not None:
        #     self.point(rand_coordinate, ls=ls_rand_coordinates)
        #     self.line_segment((rand_coordinate, rand_node.coords), ls=ls_ls_to_nn)
            
        # if rand_node is not None:
        #     self.vision_area(rand_node.coords, Tree.radius)
        #     self.point(rand_node.coords, ls=ls_random_node)
        # if nearest_neighbour_node is not None:
        #     self.point(nearest_neighbour_node.coords, ls=ls_nearest_n_node)
        # for neighbour_node in neighbour_nodes:
        #     self.point(neighbour_node.coords, ls=ls_neighbour_node)
        
        # path from root to goal
        self.path(Tree.path_to_goal, ls_edge=ls_ahead_path, ls_node=ls_path_node, lw=lw_path)
        self.pause(0.001)
    
    def RRTX_animation(self, Tree=Tree, obstacles=Obstacles, robot=Robot, obstacle_nodes=[],\
                    discovered_obstacle_nodes = [], all_children= [], rrt_queue=Priority_queue,\
                    sql_nodes= [], easy_experiment=False):
        
        # clear plot
        self.clear()

        ''' draw map obstacles/world '''            
        # prepare title
        #status_title = self.prepare_title(num_iter, Tree.total_goal_cost)
        status_title = "range {0}, path cost {1:0.2f}".format(robot.vision_range, robot.cost)
        if robot.reach_goal:
            status_title += ", reached goal."
        # plot map
        if show_map:
            self.show_map(world_name=None, obstacles=obstacles, plot_title=status_title)
        
        # draw tree
        self.tree(Tree, color_mode=TreeColor.by_lmc)
        
        # draw goal
        self.goal(robot.goal, robot.reach_goal, None)

        # draw start 
        self.start(robot.start)
        self.robot(robot=robot)
        self.vision_area(robot.coordinate, robot.vision_range)
        self.point_text(robot.coordinate, "1r", "bot")

        self.path(path=robot.path_look_ahead_to_goal, ls_edge=ls_ahead_path, ls_node=ls_path_node, lw=lw_path)
        self.paths_coords(paths=robot.visited_paths,ls_edge=ls_visited_path, ls_node=ls_path_node, lw=lw_path)
        
        # debug 
        debug = False
        if debug:
            for pt in obstacle_nodes:
                self.point(pt.coords, "ok")
                #self.point_text(pt.coords, "ok", "o")
            for pt in discovered_obstacle_nodes:
                self.point(pt.coords, "og")
                #self.point_text(pt.coords, "og", '_')
            for pt in all_children:
                self.point(pt.coords, ".b")
                #self.point_text(pt.coords, "^b", 'c')

            for pt in sql_nodes:
                #self.point(pt.coords, "^b")
                self.point_text(pt.coords, ".m", '_')
            
            queue_node = rrt_queue.get_all_values()
            for pt in queue_node:
                #self.point(pt.coords, "Pr")
                self.point_text(pt.coords, "1r", 'q')
        if not easy_experiment:
            self.pause(0.001)

