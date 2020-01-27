#!/usr/bin/env python3

#Sean Nishi and Stanley Chan
#Lab 5 Spring 2019

#useful resources
#https://github.com/duongnganh/GT-CS3630-Robotics/blob/master/Lab6/


import cv2
import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time

from ar_markers.hamming.detect import detect_markers

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *

# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

#marker size in inches
marker_size = 3.5

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"


async def image_processing(robot):

    global camK, marker_size

    event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera image to opencv format
    opencv_image = np.asarray(event.image)
    
    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)
    
    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        print("ID =", marker.id);
        print(marker.contours);
        #time.sleep(10000)
    #cv2.imshow("Markers", opencv_image)

    return markers

#calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):
    
    marker2d_list = []
    
    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
        R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        #print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0])
        
        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        # print('x =', x, 'y =', y,'theta =', yaw)
        
        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x,y,math.degrees(yaw)))

    return marker2d_list


#compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    if cvt_inch:
        last_x, last_y = last_x / 25.4, last_y / 25.4
        curr_x, curr_y = curr_x / 25.4, curr_y / 25.4

    return [[last_x, last_y, last_h],[curr_x, curr_y, curr_h]]

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)

#---------------------------------------------------------------------
#play the unhappy animation
#def temper_tantrum(robot):
#    await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabUnhappy).wait_for_completed()

#plays unhappy animation and resets the ParticleFilter
async def cozmo_picked_up(pf, robot, grid):
    print("PUT ME DOWN\n")
    #temper_tantrum()
    await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabUnhappy).wait_for_completed()
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    #return ParticleFilter(grid)


async def run(robot: cozmo.robot.Robot):
    global last_pose
    global grid, gui

    # start streaming
    robot.camera.image_stream_enabled = True

    #start particle filter
    pf = ParticleFilter(grid)

    ############################################################################
    ######################### YOUR CODE HERE####################################
    markers = []
    at_goal = False
    cozmo_displaced = False

    #set initial head angle
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    while True:
        #check if robot is picked up
        if robot.is_picked_up:
            await cozmo_picked_up(pf, robot, grid,)
            pf = ParticleFilter(grid)
            at_goal = False
            continue

        #while we arent at the goal, update the map
        if at_goal is False:
            #get the current robot pose
            current_pose = robot.pose

            #obtain odometry info
            odom = compute_odometry(current_pose)

            #obtain list of seen markers and their poses
            markers = await image_processing(robot)

            #convert the marker measurements
            #returns [x, y, h]
            odom_measurements = cvt_2Dmarker_measurements(markers)
        
            #update the particle filter using the obtained info
            updated_pf = pf.update(odom, odom_measurements)
            #update the particle GUI for debugging
            gui.show_robot(Particle(goal[0], goal[1], goal[2]))
            gui.show_particles(pf.particles)
            gui.show_mean(updated_pf[0], updated_pf[1], updated_pf[2], updated_pf[3])
            gui.updated.set()

       
        #did we converge? (have confidence we know where we are)
        #if not converge(pf.particles):
        converged = compute_mean_pose(pf.particles)

        if not converged[3] and not at_goal:
            print('not converged and not at goal')
            #if the robot is picked up, play animation
            if robot.is_picked_up:
                cozmo_displaced = True
                await cozmo_picked_up(pf, robot, grid)
                #await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabUnhappy, in_parallel = True).wait_for_completed()
                pf = ParticleFilter(grid)

            else:
                #robot's next actions?
                #update the pose
                last_pose = current_pose

                if robot.is_picked_up:
                    await cozmo_picked_up(pf, robot, grid)
                    pf = ParticleFilter(grid)
                    continue

                #if we didnt detect any markers but we got measurements
                if len(markers) != 0 and odom_measurements[0][0] > 2:
                    await robot.drive_straight(cozmo.util.distance_inches(1), cozmo.util.speed_mmps(25.4), False).wait_for_completed()

                else:
                    await robot.turn_in_place(cozmo.util.degrees(-30)).wait_for_completed()
        #else we have converged
        else:
            print("converged\n")
            if robot.is_picked_up:
                at_goal = False
                await cozmo_picked_up(pf, robot, grid)
                pf = ParticleFilter(grid)
                continue

            #are we at the goal?
            if not at_goal:
                #returns x, y, h, c
                print('not at goal')
                mean_pose = compute_mean_pose(pf.particles)
                                
                dist_to_goal_inches = (goal[0] * 25/25.4, goal[1] * 25/25.4, goal[2])
                d_x = dist_to_goal_inches[0] - mean_pose[0]
                d_y = dist_to_goal_inches[1] - mean_pose[1]
                #need to convert from rad to deg
                angle = math.degrees(math.atan2(d_y, d_x))
                distance = math.sqrt(d_x**2 + d_y**2) * 25.4
                turn_angle = diff_heading_deg(angle, mean_pose[2])

                #if the robot is picked up, reset pf
                if robot.is_picked_up:
                    await cozmo_picked_up(pf, robot, grid)
                    pf = ParticleFilter(grid)
                    continue
                #else rotate the robot to face the goal
                else:
                    await robot.turn_in_place(cozmo.util.degrees(turn_angle)).wait_for_completed()

                #this is in inches
                distance_travelled = 0
                cozmo_displaced_while_moving = False
                
                #cozmo facing the goal, need to move to it
                while distance_travelled < distance:
                    print(distance)
                    print(distance_travelled)
                    #if cozmo is interrupted when moving, need to stop
                    if robot.is_picked_up:
                        await cozmo_picked_up(pf, robot, grid)
                        pf = ParticleFilter(grid)
                        cozmo_displaced_while_moving = True
                        break

                    await robot.drive_straight(cozmo.util.distance_inches(min(1, distance - distance_travelled)), cozmo.util.speed_mmps(25.4), False).wait_for_completed()
                    distance_travelled = distance_travelled + 25.4
 
                #make sure we dont do anything if we detect cozmo being displaced
                if cozmo_displaced_while_moving:
                    await cozmo_picked_up(pf, robot, grid)
                    pf = ParticleFilter(grid)
                    continue

                if robot.is_picked_up:
                    await cozmo_picked_up(pf, robot, grid)
                    pf = ParticleFilter(grid)
                    continue
                else:
                    await robot.turn_in_place(cozmo.util.degrees(-angle)).wait_for_completed()
                                
                if robot.is_picked_up:
                    await cozmo_picked_up(pf, robot, grid)
                    pf = ParticleFilter(grid)
                    continue
                else:
                    await robot.play_anim_trigger(cozmo.anim.Triggers.AcknowledgeObject).wait_for_completed()
                    print('MADE IT TO THE GOAL! SETTING AT_GOAL TO TRUE')
                    at_goal = True

            #if we are at the goal but are picked up....
            #if we have converged and are at the goal
            else:
                print("AT GOAL\n ")
                await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabUnhappy).wait_for_completed()

                while at_goal is True and not robot.is_picked_up:
                    await robot.drive_straight(cozmo.util.distance_inches(0), cozmo.util.speed_mmps(40), False).wait_for_completed()
                    #if cozmo is picked up while at the goal...
                    if robot.is_picked_up:
                        await cozmo_picked_up(pf, robot, grid)
                        pf = ParticleFilter(grid)
                        at_goal = False
                        break
                    
                    #robot.stop_all_motors()
                

    ############################################################################


class CozmoThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    gui.start()
