import numpy as np 
import math 

import plotting

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self,point,cost=0):
        self.x = point[0]
        self.y = point[1]
        self.parent = None
        self.cost = cost # only used in RRT*
        self.is_root = False




### RRT Star Class for occupancy grid
# The inputs to the rrt start class are the goal, and start 
# step_len:
# goal_sample_rate
# search_radius 
# max number of iterations
class RrtStar:
    def __init__(self, x_start, x_goal, step_len,goal_sample_rate, search_radius, iter_max,grid):
        # The goal and the start node are nodes within our empty graph
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)

        # Define the parameters for rrt
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max

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
            #node_rand = self.generate_random_node(self.goal_sample_rate)

            node_rand = self.generate_random_node_bias(self.goal_sample_rate)

            # find the nearest neighbor in the tree (probably euclidean)
            node_near = self.nearest_neighbor(self.list_of_vertices, node_rand)

            # This is like a steering function, we take a step in the direction and
            # angle of a newly sampled point (so for f1tenth it might be using the ackermann dynamics)
            # who knows 
            node_new = self.new_state(node_near, node_rand)

            # print the sample number 
            if k % 10 == 0:
                print("Sample Number: ",k,"Number of vertices",len(self.list_of_vertices))

            # get a new node and if the ray connecting the new node with near node 
            # is collision free then we can add the vertex to our list of nodes
            if node_new and not self.is_collision(node_near, node_new):

                # find the closest neighbors this is a list
                neighbor_indices = self.find_nearest_neighbors(node_new)

                # append the new node
                self.list_of_vertices.append(node_new)
                
                # Go through the list of neighbors
                if neighbor_indices:
                    # choose the parent node from the list of neighbors, the parent will be the node
                    # with the lowest cost
                    self.choose_parent(node_new, neighbor_indices)

                    # rewire the tree, this ensure's the optimality of the search
                    self.rewire(node_new, neighbor_indices)
            

            # get the index of the minimum cost vertex within a step length of the goal 
            index = self.search_goal_parent()
            self.path = self.extract_path(self.list_of_vertices[index])

            # Plotting 
            #self.plotting.animation(self.list_of_vertices, self.path, "rrt*, N = " + str(self.iter_max),True)





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


    """function that checks for collisions in the newly sampled point"""
    def is_collision(self,start_node,end_node):

        bloat = 5
        # get the steps in the right direction
        x_len = end_node.x - start_node.x 
        y_len = end_node.y - start_node.y 

        x_index = int(round((start_node.x  - self.origin[0])/(self.res)))
        y_index = int(round((start_node.y  - self.origin[1])/(self.res)))
        
        # bloat each index and look 0.1 m (3*0.4) in either direction for an obstacle or freespace 
        vals = self.grid[x_index-bloat:x_index+bloat+1,y_index-bloat:y_index+bloat+1].flatten()

        # in the occupancy grid a value of 100 means occupied, -1 is unknown
        if(100 in vals or -1 in vals):
            return True
        return False
            


    """
        Return the node with the minimum cost (in this case it's a euclidean distance based)
        cost, this function doesn't return anything other than connect the graph
    """
    def choose_parent(self, node_new, neighbor_indices):
        cost = [self.get_new_cost(self.list_of_vertices[i], node_new) for i in neighbor_indices]

        # get the parent with the minimum cost
        cost_min_index = neighbor_indices[int(np.argmin(cost))]

        # the new node's parent is the node with the minimum cost
        node_new.parent = self.list_of_vertices[cost_min_index]
        node_new.cost = cost[int(np.argmin(cost))]


    """
        check if path is feasible
    """
    def path_found(self):
        if self.s_goal in self.path and self.s_start in self.path:
            print("Path Found")
        else:
            print("Still Searching")
        

    
    """
        This is another function that can benefit from a vectorized implementation
        It computes the distance to each vertex within the tree and either returns the vertex
        that is within a step radius of the goal and has a minimum cost or the index of the 
        newest node.

    """
    def search_goal_parent(self):

        # create a list of distances to the goal
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.list_of_vertices]

        # get all the nodes that are within a step length of the goal
        # return all the indices that satisify the criteria
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]
        
        # with this list of nodes that are within a step length of the goal, compute the distances to each of the nodes,
        # and also ensure that connecting this node with the goal doesn't result in a collision 
        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.list_of_vertices[i]) for i in node_index
                         if not self.is_collision(self.list_of_vertices[i], self.s_goal)]
            # cost_list = [dist_list[i] + self.list_of_vertices[i].cost for i in node_index
            #              if not self.is_collision(self.list_of_vertices[i], self.s_goal)]
            if(len(cost_list)>0):
                return node_index[int(np.argmin(cost_list))]
        

        ## TBD: need to grok this a little more
        return len(self.list_of_vertices) - 1



    """
        function that generates a new node based on a random sample

        Node start is the nearest node in the tree
        Node goal is the random node that we just randomly sampled

    """
    def new_state(self, node_start, node_goal):

        # get the distance between these two and the angle

        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        # instead of using this new point we take a step in that direction with a max distance 
        # step_len
        dist = min(self.step_len, dist)

        # create the new node
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        
        # make the parent of this node the nearest node
        node_new.parent = node_start

        return node_new


    """
        Rewire the tree. Performing this operation is what gives rrt_star asymptotic optimality. Basically if adding this node to it's neighbors 
        decreases the cost of those paths then we should do so by rewiring the tree
    """
    def rewire(self, node_new, neighbor_indices):

        # iterate through the list of neighbor nodes
        for i in neighbor_indices:
            node_neighbor = self.list_of_vertices[i]
            
            # if the cost at the current vertex is greater when adding the new node
            # then we should rewire the tree by making this new node it's parent, thereby decreasing this path's length
            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
               node_neighbor.parent = node_new

            # if node_neighbor.cost > self.get_new_cost(node_new, node_neighbor):
            #     node_neighbor.parent = node_new



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
            return Node((x,y))
        return self.s_goal


    """
    This method should randomly sample the free space, and returns a viable point

    Args:
    Returns:
    (x, y) (float float): a tuple representing the sampled point

    """
    def generate_random_node_bias(self,goal_sample_rate):


        #x_index = int(round((self.s_goal.x- self.origin[0])/(self.res)))
        #y_index = int(round((self.s_goal.y - self.origin[1])/(self.res)))
        #print(self.s_start.x,self.s_start.y)
        x_ind = np.where((self.x_free>=self.s_start.x-2.01) & (self.x_free<=self.s_start.x+2.01))[0]
        y_ind = np.where((self.y_free>=self.s_start.y-2.01) &(self.y_free<=self.s_start.y+2.01))[0]
        #print(x_ind,y_ind)
        
        self.x_free_bias = self.x_free[x_ind]
        self.y_free_bias = self.y_free[y_ind]
    
        if np.random.random() > goal_sample_rate:
            x = np.random.choice(self.x_free_bias)
            y = np.random.choice(self.y_free_bias)
            return Node((x,y))
        return self.s_goal

    """
        Function that returns new between two nodes, the cost of the starting node and the end node
    """
    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)
        
        # the new cost is the cost of this node plus the new distance
        #return node_start.cost +dist 
        return self.cost(node_start) + dist

    """
        nearest node in terms of euclidean distance
    """
    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([np.linalg.norm([nd.x - n.x, nd.y - n.y])
                                        for nd in node_list]))]

    """
        Function that returns the euclidean distance and angle and angle betwen two nodes
    """
    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


    """
        computes the euclidean distance of the path of a node
        This can be sped up by storing the cost at each node

    """
    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:

            # Let's see 
            #cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            cost += node.cost
            node = node.parent

        return cost

    
    """
        Function that plots final solution obtained
    """
    def plot_final(self):
        self.plotting.animation(self.list_of_vertices, self.path, "Porto Final Solution, N={}".format(self.iter_max),True,True)



    
if __name__ == "__main__":
    x_start = (0,0)
    x_goal = (1.422220,1.244794)
    grid = 'porto_grid.npy'
    step_length = 0.30 
    goal_sample_rate = 0.10
    search_radius = 1.00
    n_samples = 2000

    rrt_star = RrtStar(x_start, x_goal, step_length,goal_sample_rate, search_radius, n_samples,grid)
    rrt_star.planning()
    rrt_star.plot_final()
