from time import sleep
from cmap import *
from gui import *
from utils import *

#don't need to import np because utils does it

from math import degrees, tan, atan, sin, cos
import random

MAX_NODES = 20000

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, a nodes is an object that has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

#done
def step_from_to(node0, node1, limit=300):
    ############################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    if (get_dist(node0, node1) < limit):
        return node1
    
    #get angle, will be proportionally the same for triangle > limit
    #may need to correct rads to degs
    
    #Goes off the screen but math looks right...
    #angle = degrees(atan(tan((node1.y - node0.y)/(node1.x - node0.x))))%360
    #x_coord = node0.x + (limit * cos(angle))
    #y_coord = node0.y + (limit * sin(angle))

    angle = np.arctan2(node1.y, node1.x)
    y_coord = np.sin(angle)*limit
    x_coord = np.cos(angle)*limit
    #problem is here!!!!

    new_node = Node((x_coord, y_coord))

    return new_node
    
################################################################################

#done
def node_generator(cmap):
    #print('node_generator()')
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    
    ############################################################################
    #loop if we don't get a node in the boundries
    
    while True:
    #get random coords and set them as x, y on rand_node
    #try np.random.uniform() to generate values?
        rand_x = np.random.uniform(0, cmap.width)
        rand_y = np.random.uniform(0, cmap.height)
        #rand_x = random.randint(0, cmap.width)
        #rand_y = random.randint(0, cmap.height)
    
        rand_node = Node((rand_x, rand_y))

        #make sure the node's coords are in the boundries
        if (cmap.is_inbound(rand_node) is True):
            if (cmap.is_inside_obstacles(rand_node) is not True):
               break;
   # print('generated a node')    
    return rand_node

################################################################################

def RRT(cmap, start):
   # print('RRT')
    cmap.add_node(start)

    map_width, map_height = cmap.get_size()

    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        
        #creates new node, calls generate_node()
        rand_node = cmap.get_random_valid_node()
        
        #init shortest distance set it to a high value so it can find the next shortest distance easier
        #using a index to iterate through the node list
        #and a temp index to keep the index of the shortest distance node
        current_node_list = cmap.get_nodes()
        limit = 300
        #holds the index of the nearest node in our list
        temp_index = 0
        i = 0

        #loop finds closest node in our tree to the new random node (shortest route)
        while(i < cmap.get_num_nodes()):
            # temp is the distance between rand and current node
            temp = get_dist(rand_node, current_node_list[i])
            #check if we have new limit, save index if yes
            if(temp < limit):
                limit = temp
                temp_index = i

            i = i+1
        ########################################################################
        #save the nearest node to the rand node
        nearest_node = current_node_list[temp_index]
        rand_node = step_from_to(nearest_node, rand_node)

        #create temp tuple of the nodes we want to look at
        temp_line_seg = nearest_node, rand_node
        #obstacle detected?
        if(cmap.is_collision_with_obstacles(temp_line_seg) is not True):
            cmap.add_node(rand_node)
            sleep(0.01)
            cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RRTThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global grid, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/map1.json", node_generator)
    visualizer = Visualizer(cmap)
    robot = RRTThread()
    robot.start()
    visualizer.start()
    stopevent.set()
