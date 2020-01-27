import cozmo

from cmap import *
from gui import *
from utils import *

#added libraries
import random
from time import sleep

MAX_NODES = 20000

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, all nodes are Node object, each of which has its own
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

    #check if node1 is within the limit
    if (get_dist(node0, node1) < limit):
        return node1

    #if greater than limit, create new node with closer coords
    angle = np.arctan2(node1.y, node1.x)
    y_coord = np.sin(angle)*limit
    x_coord = np.cos(angle)*limit

    new_node = Node((x_coord, y_coord))

    return new_node
    ############################################################################

#done
def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    pass
    ############################################################################
    #generate a node with rand coords. If it fails, try again
    while True:
    #try np.random.uniform() to generate values?
        #rand_x = np.random.uniform(0, cmap.width)
        #rand_y = np.random.uniform(0, cmap.height)
        rand_x = random.randint(0, cmap.width)
        rand_y = random.randint(0, cmap.height)
    
        rand_node = Node((rand_x, rand_y))

        #make sure the node's coords are in the boundries
        if (cmap.is_inbound(rand_node) is True):
            if (cmap.is_inside_obstacles(rand_node) is not True):
               break;

    return rand_node

#done
def RRT(cmap, start):
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
        #
        ########################################################################
        rand_node = cmap.get_random_valid_node()
        current_node_list = cmap.get_nodes()

        i = 0
        index = 0
        limit = 300
        #go through all nodes to find closest one
        while(i < cmap.get_num_nodes()):
            # temp is the distance between rand and current node
            temp = get_dist(rand_node, current_node_list[i])
            #check if we have new limit, save index if yes
            if(temp < limit):
                limit = temp
                temp_index = i
            i = i+1

        #create a step from rand_node to nearest_node
        nearest_node = current_node_list[index]
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


#initial vars are (robot: robot.cozmo.Robot)
async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
    #if see cube, exception_hander. if goal, RRT, if obstacle, replan
    '''
        1) identify a target cube
            if cube not detected
                move to center of arena
            while(cube not detected)
                rotate
        2) use RRT to find a path to a specific face on the cube
        3) follow the path found by RRT
        4) replan to avoid any obstacle cubes that are added during navigation
    '''
    #the cushion around cube and cozmo
    cube_radius = 60
    cozmo_radius = 100
    center_coords = 325, 225
    #bools to change states
    found_goal = False
    goal = None
    go_to_center = 0

    #temp vars
    cozmo_pose = None
    object_pose = None

    #
    while (True):
        #update coords (cozmo thinks he is a point)
        #cozmo_pose = Node(tuple(robot.pose.x, robot.pose.y))

        #1
        #for all object(s) cozmo detects, determine if it is the goal or an obstacle
        for obj in robot.world.visible_objects:
            #update relevent info of this new object
            cozmo_pose = Node(tuple(robot.pose.x, robot.pose.y))
            object_pose = Node(tuple(obj.pose.x, obj.pose.y))

            #update object pose with cozmo's pose
            object_pose = Node(cozmo_pose.x + object_pose.x, cozmo_pose.y + object_pose.y)
            #get angle between cozmo and object
            angle = object_pose.rotation.angle_z.radians

            #if obstacle
            if (robot.light_cubes[cozmo.objects.LightCube1Id].object_id != obj.object_id):
               #do we have the obstacle already?
                if (not cmap.is_inside_obstacles(object_pose)):
                    cmap.reset()
                    #need a list for the node points for the obstacle, needs to be in order
                    obstacle_nodes = []
                    obstacle_nodes.append(get_global_node(angle, object_pose, Node(tuple(cube_radius, cube_radius))))
                    obstacle_nodes.append(get_global_node(angle, object_pose, Node(tuple(cube_radius, -cube_radius))))
                    obstacle_nodes.append(get_global_node(angle, object_pose, Node(tuple(-cube_radius, -cube_radius))))
                    obstacle_nodes.append(get_global_node(angle, object_pose, Node(tuple(-cube_radius, cube_radius))))
                    
                    cmap.add_obstacle(obstacle_nodes)
                    RRT(cmap, cozmo_pose)
            #if goal
            else:
                local_pose = Node(tuple(0, -cozmo_radius))
                goal_pose = get_global_node(angle, object_pose, local_pose)
                #remove old goal and update new goal
                cmap.clear_goals()
                cmap.add_goal(goal_pose)
                #keep track
                found_goal = True
                goal = object_pose

        ###########################################################################
        #2
        #no goal, spin around 3 times
        if found_goal is not True:
            #rotate and iterate flag_center
            await robot.drive_wheels(10, -10)
            go_to_center + 1

        #if flag_center triggered, drive to center
        if go_to_center >=3:#dont use gotopose.
            #orient towards center
            #drive to center
            await robot.drive_wheels(calc_goto_coords(cozmo_pose, center_coords))
        
        ###########################################################################
        #3
        #found the goal
        if found_goal is True:
            #have the goal, is the cmap solved?
            if cmap.is_solved():
                #move to nodes
                #4
                #traverse list
                #get path of nodes. Drive through the nodes

                center_list = cmap.get_goals
                center = center_list[0]
                if cozmo_pose == center:
                    found_goal = True

            else:
                RRT(cmap, cozmo_pose)

#does the big math, translates matrix
def calc_goto_coords(cozmo_coords, coords):
    try:
        cube_coords = coords

        cube_matrix = np.matrix([[cube_coords.position.x],
                                 [cube_coords.position.y],
                                 [1]])
          
        cos = math.cos(cozmo_coords.rotation.angle_z.radians)
        sin = math.sin(cozmo_coords.rotation.angle_z.radians)
        #translation matrix with cozmo's angles in radians
        translation_matrix = np.matrix([[cos, -sin, cozmo_coords.position.x],
                                        [sin, cos, cozmo_coords.position.y],
                                        [0, 0, 1]])

        inverse_trans_matrix = translation_matrix.I

        #calculate coords to travel
        new_x = (inverse_trans_matrix * cube_matrix).A[0][0]
        new_y = (inverse_trans_matrix * cube_matrix).A[1][0]
        go_to_x = new_x
        go_to_y = new_y

        return Node(tuple(new_x, new_y))

    except:
        return "something wrong with calculating coordinates"

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
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
    global cmap, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
