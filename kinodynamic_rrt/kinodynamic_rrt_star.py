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
    def __init__(self, x_start, x_goal, time_forward,goal_sample_rate, throttle_speed, number_of_motion_primitives, iter_max,grid,min_speed=0.1,rrt_star=False):
        self.rrt_star  = rrt_star
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)
        if(self.rrt_star):
            self.s_start.cost = 0

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

        # This prevents duplicate nodes from being added:
        self.list_of_xys = []

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

            # find the nearest neighbor in the tree (euclidean distance for now)
            node_near = self.nearest_neighbor(self.list_of_vertices, node_rand)

            # This is like a steering function, this is where the kinodynamic simulation occurs
            node_new, trace_trajectory = self.new_state(node_near, node_rand)

            # print the sample number 
            if k % 10 == 0:
                print("Sample Number: ",k,"Number of vertices",len(self.list_of_vertices))


            ## check if this point is already in the tree
            point = (node_new.x,node_new.y)
            add = not point in self.list_of_xys


            # get a new node and if the ray connecting the new node with near node 
            # is collision free then we can add the vertex to our list of nodes
            # Select the next line of code depending on the fidelity of the collision checking
            # The first checks the whole trajectory, the second checks the whole trace.
            #if node_new and not self.is_collision(node_near, node_new) and add:
            if node_new and not self.is_collision_trajectory(trace_trajectory) and add:
                # if the path is not in collision 
                self.list_of_vertices.append(node_new)
                self.list_of_xys.append(point)
                
                # get the neigbors within a particular radius of the sample

                neighbor_indices  = self.find_nearest_neighbors(node_new)

                # if there are nodes within that radius consider if we should re-wire the tree
                if neighbor_indices:
                    # choose the parent node from the list of neighbors
                    # and rewire the tree
                    self.rewire(node_new,neighbor_indices)
                    
            

            # get the index of the minimum cost vertex within a step length of the goal 
            #index = self.search_goal_parent()
            #self.path = self.extract_path([])
            # Plotting 
            #self.plotting.animation(self.list_of_vertices, self.path, "Kinodynamic rrt*, N = " + str(self.iter_max),True)
            
            #print(self.list_of_vertices)
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
        nearest_trajectory = primitives[index]

        # get the distance here too
        if(self.rrt_star):
            new_dist = dist[index]

        #print(nearest_trajectory)

        #print(pts[index],pts)

        # use the first three elements of the state with a minimum speed of 0.3 
        # so that the forward projection moves faster
        new_state =list(nearest_trajectory[-1][:3])+[self.min_speed]

        #print(new_state)
        # create the new node by selecting the last point in the trajectory
        # each trajectory technically has a speed but I think I just want to assume that each point has 
        # a speed equally to the minimum speed, not sure how this will work
        node_new = Node(xs=new_state)

        if(self.rrt_star):
            node_new.cost = node_start.cost + new_dist 
        
        # make the parent of this node the nearest node
        node_new.parent = node_start

        # return the nearest trajectory
        return node_new, nearest_trajectory


    """function that checks for collisions in the newly sampled point,
       In this case we can use the trajectory that was used in the euler simulation.
       This currently naive because it just checks if the end point is in collision, which is fine 
       if the step size is small but not correct if not. It's fast though haha
       
       """
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


    """function that checks for collisions in the newly sampled point,
       In this case we are checking the whole trace so this is more correct but it's probably slower
    """
    def is_collision_trajectory(self,trajectory):
        bloat=2 
        for pt in trajectory:
            x_index = int(round((pt[0]  - self.origin[0])/(self.res)))
            y_index = int(round((pt[1]  - self.origin[1])/(self.res)))
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


    """ starting from the vertex that has been identified as closest the goal work backwards.
    """
    def extract_path(self, node_end):
        
        # last node in the path is the goal node
        path = [[self.s_goal.x, self.s_goal.y]]
        node = node_end
        
        # from this node ask for parents
        while node.parent is not None:
           path.append([node.x, node.y])
           node = node.parent
           #print(node.x,node.y)
        path.append([node.x, node.y])

        return path






    """
        This is another function that can benefit from a vectorized implementation
        It computes the distance to each vertex within the tree and either returns the vertex
        that is within a step radius of the goal and has a minimum cost or the index of the 
        newest node.

    """
    def search_goal_parent(self,all_paths=False):
        # create a list of distances to the goal
        # random sampling allows you to add a node to the tree more than once (need to figure out how to prevent that)
        # did that I can now change this to be simpler
        # well we will only consider 
        candidates =[]
        check_for_redundant = []
        for n in self.list_of_vertices:
            dist = math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y)
            if(dist <= self.step_len and n not in candidates):
                if((n.x,n.y,n.yaw) in check_for_redundant):
                    continue
                else:
                    candidates.append(n)
                    check_for_redundant.append((n.x,n.y,n.yaw))


        if(len(candidates)==0):
             print("No path found yet.")
        else:
             print("Path Found: Nodes within one step:",len(candidates))

        # get the node that is the closest to the goal when we simulate one more step
        dists = []
        for node in candidates:
            # generate another state
            node_new, _ = self.new_state(node, self.s_goal)

            # get the dist
            dist = math.hypot(node_new.x - self.s_goal.x, node_new.y - self.s_goal.y)
            dists.append(dist)
        print(dists)
        
        
        path =[]
        if(len(candidates)>0 and not all_paths):
            index = np.asarray(dists).argmin()
            node = candidates[index]
            path = self.extract_path(node)
        
        return path,candidates


    """
        Function that plots final solution obtained
    """
    def plot_final(self):
        
        # get the index of the closest point to the goal
        self.path,_ = self.search_goal_parent()


        # get the path
        # self.path = self.extract_path([])

        # get the index of the minimum cost vertex within a step length of the goal 
        self.plotting.animation(self.list_of_vertices, self.path, "Porto Final Solution, N={}".format(self.iter_max),True,True)

    """ 
        Function that plots each of the paths
    """
    """
        Function that plots final solution obtained
    """
    def plot_final_all(self):
        
        # get the index of the closest point to the goal
        self.path,candidates = self.search_goal_parent(all_paths=True)
        paths = []
        for node in candidates:
            path = self.extract_path(node)
            paths.append(path)
        # get the path
        # self.path = self.extract_path([])

        # get the index of the minimum cost vertex within a step length of the goal 
        self.plotting.animation_all(self.list_of_vertices, paths, "All paths Porto Final Solution, N={}".format(self.iter_max),True,True)



if __name__ == "__main__":
    # These are x,y,yaw,speed
    x_start = (-0.006356, 0.030406, 0.322701, 0.1)
    x_goal = (1.077466, 0.921832,0.750663, 0.1)
    grid = 'porto_grid.npy'
    time_forward = 0.3
    n_samples = 10000
    goal_sample_rate = 0.10
    throttle_speed = 0.3
    number_of_motion_primitives = 5

    # RRT*
    kinodynamic_rrt = KinodynamicRRTStar(x_start, x_goal, time_forward,goal_sample_rate, throttle_speed, number_of_motion_primitives, n_samples,grid,min_speed=0.1,rrt_star=True)
    kinodynamic_rrt.planning()
    kinodynamic_rrt.plot_final()
    #kinodynamic_rrt.plot_final_all()

