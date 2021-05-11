import numpy as np 
import math 
import plotting

# Import the bicycle model for prediction
from bicycle_model import State

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self,xs,cost=0):
        self.x    = xs[0]
        self.y    = xs[1]
        self.yaw  = xs[2] 
        self.v    = xs[3]
        self.parent = None
        self.cost = cost # only used in RRT*
        self.is_root = False

### Kinodynamic RRT Star on an occupancy grid
# The inputs to the rrt start class are the goal, and start 
# time_forward: how far in the future we want to look 
# throttle_speed: speed we want to use when generating paths: I'm going to start with the minimum speed for the
# hardware platform which is 0.3 m/s
# goal_sample_rate
# search_radius 
# max number of iterations

class KinodynamicRRTStar:
    def __init__(self, x_start, x_goal, time_forward,goal_sample_rate, throttle_speed, number_of_motion_primitives, iter_max,grid,min_speed=0.1):
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)

        # Define the parameters for rrt

        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.time_forward = time_forward
        self.throttle_speed = throttle_speed
        self.number_of_motion_primitives = number_of_motion_primitives
        self.min_speed = 0.1

        # optimality stuff for rrt_star
        self.search_radius = 1.0
        self.step_len = 0.30

        # Initialize the list of vertices with the start point
        self.list_of_vertices = [self.s_start]

        # The path begins as an empty list
        self.path = []

        # create the plotting utility, this also has information about the obstcles 
        # and free space
        self.plotting = plotting.Plotting(grid,x_start,x_goal)

        # get the obstacle boundaries and the free space
        self.x_obs = self.plotting.x_obs
        self.y_obs = self.plotting.y_obs
        self.x_free = self.plotting.x_free
        self.y_free = self.plotting.y_free

         # get the origin and resolution of the occupancy grid
        self.origin= self.plotting.origin
        self.res = self.plotting.res
    

        # This is the occupancy map 
        self.grid = self.plotting.grid

    def planning(self):
        # Iter-Max looks the number of samples described in sertac karaman's paper
        for k in range(self.iter_max):
            
            # generate a random sample in the workspace
            # in this case only from the freespace points
            node_rand = self.generate_random_node(self.goal_sample_rate)

            # find the nearest neighbor in the tree (probably euclidean)
            node_near = self.nearest_neighbor(self.list_of_vertices, node_rand)

            # This is like a steering function, this is where the kinodynamic simulation occurs
            node_new, trace_trajectory = self.new_state(node_near, node_rand)

            # print the sample number 
            if k % 10 == 0:
                print("Sample Number: ",k,"Number of vertices",len(self.list_of_vertices))


            # get a new node and if the ray connecting the new node with near node 
            # is collision free then we can add the vertex to our list of nodes
            if node_new and not self.is_collision(node_near, node_new):

                
                # find the closest neighbors this is a list
                #neighbor_indices = self.find_nearest_neighbors(node_new)
                # append the new node
                self.list_of_vertices.append(node_new)
            
            
                # Plotting 
                
                #self.plotting.animation(self.list_of_vertices, self.path, "rrt*, N = " + str(self.iter_max),True)



    """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
    def generate_random_node(self,goal_sample_rate):

        if np.random.random() > goal_sample_rate:
            x = np.random.choice(self.x_free)
            y = np.random.choice(self.y_free)
            return Node((x,y,0,0))
        return self.s_goal


    """
        nearest node in terms of euclidean distance
    """
    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([np.linalg.norm([nd.x - n.x, nd.y - n.y])
                                        for nd in node_list]))]



    """
        function that generates a new node based on a random sample

        Node start is the nearest node in the tree
        Node goal is the random node that we just randomly sampled

        This is where I think the kinodynamic planning will come into effect

    """
    def new_state(self, node_start, node_goal):

        # get the distance between these two and the angle

        node_state = [node_start.x,node_start.y,node_start.yaw,node_start.v] 
        
        # the starting state will be the nearest node
        s = State(node_state)

        # compute points in the future defined the prediction horizon and the throttle and motions primitives
        primitives = s.simulate_motion_primitives(throttle=self.throttle_speed,num_primitives=self.number_of_motion_primitives,T=self.time_forward,dt=0.05)
        
        # get all the points from motion primitives
        pts = np.asarray([[trace[-1][0], trace[-1][1]] for trace in primitives]).reshape((-1,2))
        node_end = np.asarray([node_goal.x,node_goal.y]).reshape((-1,2))

        # compute the distances from each of the motion primitives
        dist = np.linalg.norm(pts-node_end,axis=-1)

        # get the point closest to the randomly sampled point
        index = dist.argmin()
        nearest_trajectory = pts[index]

        # create the new node by selecting the last point in the trajectory
        # each trajectory technically has a speed but I think I just want to assume that each point has 
        # a speed equally to the minimum speed, not sure how this will work
        node_new = Node(xs=[nearest_trajectory[-1][:3],self.min_speed])
        
        # make the parent of this node the nearest node
        node_new.parent = node_start

        # return the nearest trajectory
        return node_new, nearest_trajectory


    """function that checks for collisions in the newly sampled point,
       In this case we can use the trajectory that was used in the euler simulation"""
    def is_collision(self,start_node,end_node):

        bloat =5 
        x_index = int(round((end_node.x  - self.origin[0])/(self.res)))
        y_index = int(round((end_node.y  - self.origin[1])/(self.res)))
        
        # bloat each index and look 0.1 m (3*0.4) in either direction for an obstacle or freespace 
        vals = self.grid[x_index-bloat:x_index+bloat+1,y_index-bloat:y_index+bloat+1].flatten()

        # in the occupancy grid a value of 100 means occupied, -1 is unknown
        if(100 in vals or -1 in vals):
            return True
        return False


    """
    This function returns the nearest neighbors of new node. The search radius is the parameter that ensures 
    asymptotic optimality, it has a minimum distance between the step len and this other thing that I'm still wondering
    about tbh. This function can be sped up (vectorized implementation)

    """
    def find_nearest_neighbors(self, node_new):

        
        # n is the number of vertices
        n = len(self.list_of_vertices) + 1
        
        # this is the optimality criterion
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        # yeah this really should be vectorized compute the distance to every node from this one
        # this is an ordered list of each vertex which is a node
        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.list_of_vertices]

        # return only those nodes that are within the search radius and that don't result in an intersection with 
        # an obstacle
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.is_collision(node_new, self.list_of_vertices[ind])]

        # return the list of obstacles

        return dist_table_index


if __name__ == "__main__":
    x_start = (0,0)
    x_goal = (1.422220,1.244794)
    grid = 'porto_grid.npy'
    step_length = 0.30 
    goal_sample_rate = 0.10
    search_radius = 1.00
    n_samples = 100

    rrt_star = RrtStar(x_start, x_goal, step_length,goal_sample_rate, search_radius, n_samples,grid)
    rrt_star.planning()
    rrt_star.plot_final()
