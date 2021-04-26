"""
2D rrt star
@author: Patrick Musau
based on the implementation of huiming zhou
"""

import sys
import math 
import numpy as np 

"""Import some data structures and files needed for rrt star:
 - env defines the search space used in this example: 5 circles, 4 rectangles, and search space boundaries as well as size of search space x: (0,50) y: (0,30)
 - plotting utilities

"""
import env, plotting, utils, queue 

# RRT star operates on graphs let's define a graph node
# n is a 2D point
class Node:
    def __init__(self,n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None



# RRT Star class
# The inputs to the rrt start class are the goal, and start 
# step_len:
# goal_sample_rate
# search_radius 
# max number of iterations
class RrtStar:
    def __init__(self, x_start, x_goal, step_len,goal_sample_rate, search_radius, iter_max,x_width=50,y_width=30):

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

        # create object that defines the planning workspace
        self.env = env.Env(x_width,y_width)

        # Instantiate the Plotting Utilities
        self.plotting = plotting.Plotting(x_start, x_goal,x_width,y_width)

        # Instantiate the other relevant collision checking and distance utilities
        self.utils = utils.Utils(x_width,y_width)

        # Define the width and height of the workspace
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range

        # definitions of obstacles and workspace boundaries
        #self.obs_circle = self.env.obs_circle()
        #self.obs_rectangle = self.env.obs_rectangle()
        #self.obs_boundary = self.env.obs_boundary()


    def planning(self):
        # Iter-Max looks the number of samples described in sertac karaman's paper
        for k in range(self.iter_max):
            
            # need to generate a random sample in the workspace
            node_rand = self.generate_random_node(self.goal_sample_rate)

            # find the nearest neighbor in the tree (probably euclidean)
            node_near = self.nearest_neighbor(self.list_of_vertices, node_rand)

            # This is like a steering function, we take a step in the direction and
            # angle of a newly sampled point (so for f1tenth it might be using the ackermann dynamics)
            # who knows 
            node_new = self.new_state(node_near, node_rand)

            # print the sample number 
            if k % 500 == 0:
                print("Sample Number: ",k)

            # get a new node and if the ray connecting the new node with near node 
            # is collision free then we can add the vertex to our list of nodes
            if node_new and not self.utils.is_collision(node_near, node_new):

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
            self.plotting.animation(self.list_of_vertices, self.path, "rrt*, N = " + str(self.iter_max),True)



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
                            not self.utils.is_collision(node_new, self.list_of_vertices[ind])]

        # return the list of obstacles

        return dist_table_index


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
                         if not self.utils.is_collision(self.list_of_vertices[i], self.s_goal)]
            return node_index[int(np.argmin(cost_list))]
        

        ## TBD: need to grok this a little more
        return len(self.list_of_vertices) - 1



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
        path.append([node.x, node.y])

        return path

    """
        Function that returns a random node from a uniform distribution 
        We don't want boundary points so we use a delta
        Occasinally we want it to return the goal point
    """
    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal


    """
        Function that returns new between two nodes, the cost of the starting node and the end node
    """
    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist


    """
        Function that returns the euclidean distance and angle betwen two nodes
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
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    """
        nearest node in terms of euclidean distance
    """
    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]


    
   
        




def main():
    x_start = (18, 8)  # Starting node
    x_goal = (37, 18)
    x_width = 50 
    y_width = 30
    #plotting.Plotting(x_start, x_goal,x_width,y_width).plot_grid("Test")


    # call order
    # x_start, x_goal, step length, search radius, number of samples
    rrt_star = RrtStar(x_start, x_goal, 10, 0.10, 20, 100,x_width,y_width)
    rrt_star.planning()


if __name__ == '__main__':
    main()