#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, PoseArray, Quaternion, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
import utils
from PIL import Image as im
import matplotlib.pyplot as plt
from scipy import ndimage, misc
import tf

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_gt_topic = "/odom"
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = utils.LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_gt_topic, Odometry, self.odom_cb)

        self.p_init         = None # Initial Position (map)
        self.p_goal         = None # Goal Position (map)

        self.occupancy      = None # Occupancy Grid: work space of type numpy.ndarray(height, width)
        self.p_occ          = None # Occupancy grid position (translation wrt map origin)
    	self.occ_heading    = None # Occupancy grid angle (heading wrt map)
        self.height         = None # Occupancy grid height
        self.width          = None # Occupancy grid width
    	self.resolution     = None # Occupancy grid resolution (meters/cell)

        self.node_init      = None # Initial Node (grid)
        self.node_goal      = None # Goal Node (grid)
        self.nodes          = []   # Nodes: sampled from (row, col) pairs in occupancy grid
        self.graph          = {}   # Graph: a neighbor lookup table of type {start: [dest1, dest2, ...]}

        self.running_search = False

    def map_cb(self, msg):
        ''' Build an occupancy matrix, and define a transform to/from the map frame.
        '''
        self.height, self.width = msg.info.height, msg.info.width
        self.resolution = msg.info.resolution

        data = np.array(msg.data)
        self.height, self.width = msg.info.height, msg.info.width
        self.resolution = msg.info.resolution
        self.grid = np.reshape(data, (self.height, self.width))
        self.grid= np.where(self.grid < 0.0, 100, self.grid)
        self.grid = np.fliplr(self.grid)
        self.occupancy = ndimage.grey_dilation(self.grid, size=(12, 12)).astype(self.grid.dtype)
        #rotate using the orientation message

        #plt.imshow(self.occupancy)
        #plt.show()
    	(self.p_occ, self.occ_heading) = utils.pose_from_msg(msg.info.origin)

        print "Map CB"

    def odom_cb(self, msg):
        if not self.running_search:
            (self.p_init, _) = utils.pose_from_msg(msg.pose.pose)
            print "Odom CB"

    def goal_cb(self, msg):
        # if not self.resolution or not self.occ_heading:
        #     return

        (self.p_goal, _) = utils.pose_from_msg(msg.pose)
        self.running_search = True
        self.plan_path(self.p_init, self.p_goal, self.occupancy)
        print "Goal CB"

    def plan_path(self, start_point, end_point, map):
        print "PLAN PATH"
        #y, x
        self.node_init = ((-int((start_point[1] - self.p_occ[1])/self.resolution)), (self.width+int((start_point[0] - self.p_occ[0])/self.resolution)))
        self.node_goal = ((-int((end_point[1] - self.p_occ[1])/self.resolution)), (self.width+int((end_point[0] - self.p_occ[0])/self.resolution)))

        # self.node_goal = #self.grid_from_map(self.p_goal)

        # 1. Randomly sample discrete points from the free occupancy grid
        # Pinit and Pgoal are NOT included in nodes. Connect them to the graph using 'self.k_nearest()'
        self.nodes = self.sample_free_nodes(self.occupancy)
        self.nodes.append(self.node_init)
        self.nodes.append(self.node_goal)
        self.graph = { n: [] for n in self.nodes }
        print "\tSampled Free Nodes:", len(self.nodes), "nodes total"

        # 2. Run a simplified probablistic roadmap method: k-(s)PRM
        prm_start = time.time()
        self.simple_prm(k=10)
        prm_end = time.time()
        print "    PRM:", sum(len(ends) for ends in self.graph.values()), "edges total"
        print "    Time Elapsed:", prm_end - prm_start, "sec"

        # 3. Select a path from the PRM using A*

        # Run A*!
        self.astar(self.node_init, self.node_goal, self.graph)
        print("A star done")

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()

    #===========================================================================
    # PROBABILISTIC ROAD MAP (k-nearest, simplified)
    #===========================================================================

    def sample_free_nodes(self, grid):
        ''' Returns a list of row-column pairs in a configuration space that are unoccupied.
        '''
        (C_rows, C_cols) = np.where(grid == 0)
        N = len(C_rows)
        C_i = np.arange(0, N, 1)
        #N/50 chooses the number of nodes
        chosen_i = np.random.choice(C_i, N/50, replace=False)
        rows = [int(r) for r in C_rows[chosen_i]]
        cols = [int(c) for c in C_cols[chosen_i]]
        return zip(rows, cols)

    def simple_prm(self, k):
        '''
        Builds a k-nearest simplified probabilistic roadmap (k-sPRM).
        Saves valid paths in 'self.graph'.
        # From https://arxiv.org/pdf/1105.1186.pdf
        # Algorithm 2: k-sPRM
        '''
        for v in self.nodes:
            dest_nodes = self.k_nearest(v, k)
            for u in dest_nodes:
                if u in self.graph[v]:
                    continue
                if self.collision_free(v, u):
                    self.graph[v].append(u)
                    self.graph[u].append(v)

        return

    def k_nearest(self, node, k):
        '''
        Returns a list of the k nodes closest to a given node.
        '''
        v = node
        d2 = lambda u: (u[0] - v[0])**2 + (u[1] - v[1])**2
        nearest_nodes = sorted(self.nodes, key=d2)
        return nearest_nodes[1:k+1]

    def naive_collision_free(self, n1, n2):
        '''
        Returns True if all occupancy grid cells in LOS between n1 and n2
        are unoccupied, else returns False.
        Dev Note: If 'self.occupancy' refers to a work space (i.e. whether a cell
        is occupied), then this method might return True for a path that would
        collide the car.
        However, if 'self.occupancy' refers to a configuration space (i.e.
        whether a position is physically possible), then this method is sufficient.
        '''
        # For parametric line p = (n2-n1)*t + n1
        for r in xrange(n1[0], n2[0]+1):
            try:
                t = float(r - n1[0]) / float(n2[0] - n1[0])
                c = int( (n2[1] - n1[1])*t + n1[1] )
            except:
                c = n2[1]

            if self.occupancy[r, c] != 0:
                return False

        return True

    def collision_free(self, n1, n2):
        return self.naive_collision_free(n1, n2)
       

    def astar(self, start_point, end_point, grid_map):
        open_set = {start_point}
        visited = {}
        g = {}
        f = {}

        for v in grid_map:
            g[v], f[v] = np.inf, np.inf

        g[start_point] = 0
        f[start_point] = self.euclid_distance(start_point, end_point)

        #loop through nodes in the open_set
        while open_set:
            scores = {}
            for coord in open_set:
                scores[coord] = f[coord]
            current_node = min(scores, key=scores.get) # element of open_set with lowest f score

            if current_node == end_point: # found end point!
                self.rebuilt_path(visited, current_node)
                break
            open_set.remove(current_node)

            self.astar_checker(current_node, g, f, visited, open_set, grid_map, end_point)


    def rebuilt_path(self, visited, current):
        optimal_path = [current]
        while current in visited.keys():
            current = visited[current]
            optimal_path.append(current)

        print("OPTIMAL PATH:", optimal_path)

        for coord in optimal_path[::-1]:
            point = Point()
            #p = self.map_from_grid(coord[0],coord[1])
            #point.x = p[0]
            #point.y = p[1]

            point.x = self.resolution * (coord[1]-self.width) + self.p_occ[0]
            point.y = -self.resolution * coord[0] + self.p_occ[1]
            self.trajectory.addPoint(point)

    def astar_checker(self, current_node, g, f, visited, open_set, grid_map, end_point):

            # go through the neighbors and add likely ones to the open_set
            for neighbor in grid_map[current_node]:
                proposed_g = g[current_node] + self.euclid_distance(current_node, neighbor)

                if proposed_g < g[neighbor]:
                    visited[neighbor] = current_node
                    g[neighbor] = proposed_g
                    f[neighbor] = g[neighbor] + self.euclid_distance(neighbor, end_point)

                    if neighbor not in open_set:
                        open_set.add(neighbor)

    def euclid_distance(self, x, y):
        # return np.sqrt((x[0]-x[1])**2 + (y[0]-y[1])**2)
        return np.sqrt((y[0]-x[0])**2 + (y[1]-x[1])**2)

    #===========================================================================
    # HELPER METHODS
    #===========================================================================

    def transform(self, dPos, dTheta, scale):
        ''' Returns a function that will transform a given position between two frames. '''
        cT, sT = np.cos(dTheta), np.sin(dTheta)
        R = np.array([[cT, -sT], [sT, cT]])
        return lambda pos: scale * (np.matmul(R,pos) + dPos)

    def grid_from_map(self, p_map):
        ''' Returns (r, c) in the occupancy grid frame. '''
        p_grid = self.transform(self.p_occ, self.occ_heading, 1/self.resolution)(p_map)
        return int(p_grid[0]), int(p_grid[1])

    def map_from_grid(self, r, c):
        ''' Returns [x, y] in the map frame. '''
        p_grid = np.array([r, c])
        p_map = self.transform(-self.p_occ/self.resolution, -self.occ_heading, self.resolution)(p_grid)
        return p_map



if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
