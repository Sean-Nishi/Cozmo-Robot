from grid import *
from particle import Particle
from utils import *
from setting import *

#imported libraries
import math
import numpy as np
import random

alpha1 = alpha2 = 0.001
alpha3 = alpha4 = 0.005

# ------------------------------------------------------------------------
def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- noisy odometry measurements, a pair of robot pose, i.e. last time
                step pose and current time step pose
                u = [_xt-1, _xt]
                _xt-1 = [x, y, heading]
                _xt = [x', y', heading']

        Returns: the list of particle representing belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    ###############################################################################
    
    #adding noise to odom measurements
    new_odom = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)

    rot1 = math.atan2(new_odom[1][1] - new_odom[0][1], new_odom[1][0] - new_odom[0][0]) - new_odom[0][2]
    rotTrans = np.sqrt((new_odom[0][0] - new_odom[1][0])**2+(new_odom[0][1] - new_odom[1][1])**2)
    rot2 = new_odom[1][2] - new_odom[0][2] - rot1

    for p in particles:
        
                    
        #gaussian stuff. this is the hat stuff
        #mu = 0, sigma is the expression
        h_rot1 = rot1 - np.random.normal(0, abs(alpha1*rot1 + alpha2*rotTrans))
        h_rotTrans = rotTrans - np.random.normal(0, abs(alpha3*rotTrans + alpha4*(rot1 + rot2)))
        h_rot2 = rot2 - np.random.normal(0, abs(alpha1*rot2 + alpha2*rotTrans))
        
        #move the particle with the big math
        #updates position and heading of each particle
        #print("Before coords: ", p.x, p.y, p.h)
        p.move(h_rot1, h_rotTrans, h_rot2)
        #print("After moving coords: ", p.x, p.y, p.h)

    return particles
    #return [move_a_particle(p, odom) for p in particles]

def move_a_particle(particle, odom):
    #convert distance moved in robot's frame
    #add_odom_noise() returns 2 poses
    new_odom = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
    
    #interpret to robot frame
    dx, dy = rotate_point(new_odom[0][0], new_odom[0][1], particle.h)
    #make new particle with updated coords
    return Particle(particle.x + dx, particle.y + dy, particle.h + new_odom[1][2])

##################################################################################
    '''
    for p in particles:
        #adding noise to odom measurements
        odom = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        
        #old work before realizing we can use rotate_point() from utils.py
        #big math
        rot1 = math.atan2(odom[1][1] - odom[0][1], odom[1][0] - odom[0][0]) - odom[0][2]
        rot2 = odom[1][2] - odom[0][2] - rot1
        rotTrans = np.sqrt((odom[0][0] - odom[1][0])**2+(odom[0][1] - odom[1][1])**2)
        
        #cleaner rotation
        #dx, dy = rotate_point(x_noise, y_noise, particle.h)

        #new_part = Particle(p.x + dx, p.y + dy, p.h + h_noise)

        #gaussian stuff. this is the hat stuff
        #mu = 0, sigma is the expression
        h_rot1 = rot1 - np.random.normal(0, abs(alpha1*rot1 + alpha2*rotTrans))
        h_rot2 = rot2 - np.random.normal(0, abs(alpha1*rot2 + alpha2*rotTrans))
        h_rotTrans = rotTrans - np.random.normal(0, abs(alpha3*rotTrans + alpha4*(rot1 + rot2)))

        #move the particle with the big math
        #updates position and heading of each particle
        p.move(h_rot1, h_rotTrans, h_rot2)

    return particles
    '''

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- a list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
        grid -- grid world map containing the marker information. 
                see grid.py and CozGrid for definition

        Returns: the list of particle representing belief p(x_{t} | u_{t})
                after measurement update
    """
    return particles

    '''

    ##############################################################
    #holds the particle p and seen object m
    matched = [p, m]

    #for each seen object, find which particle is near the robot
    for m in mm_list:
        #convert seen objects in mm_list to robot frame

        #compare distance of m to each particle
        for p in particles:
            if matched is None:
                matched = [p, m]
            elif(p.x - m.x, p.y - m.y):
                matched = [p, m]
                
    '''
    #individual weights as a list
    #get_particle_weights
    particle_weights = [get_particle_weight(p, measured_marker_list, grid) for p in particles]

    #normalize and resample
    total_weights = sum(particle_weights)
    #get the sample_indicies
    sample_indicies = np.random.choice(len(particles), PARTICLE_COUNT, p = [i / total_weights for i in particle_weights])
    #resample w/ weights
    sample_particles = [particles[j] for j in sample_indicies]

    #adding randomness to prevent crit error
    #rand_stuff = [x, y, h, count]
    rand_stuff = compute_mean_pose(sample_particles)

    #need to add int conversion for create_random().
    #Need to add 1 so at least one particle is placed
    if rand_stuff[3]:
        rand_particles_count = int(PARTICLE_COUNT / 500) + 1
    else:
        rand_particles_count = int(PARTICLE_COUNT / 100) + 1

    sample_particles = sample_particles[rand_particles_count:]
    #create the new random particles
    rand_particles = Particle.create_random(rand_particles_count, grid)

    #returning all the particles (including new ones)
    return sample_particles + rand_particles
#####################################################################################
#added functions
#function that will get the weight of an individual particle
def get_particle_weights(particle, measured_marker_list, grid):
    #cases where particle cant exist
    if not grid.is_free(particle.x, particle.y):
        return 0
    if not grid.is_in(particle.x, particle.y):
        return 0

    #create predicted_markers list using helper function
    predicted_markers = particle.read_markers(grid)

    #if no visible markers but we predict one
    if len(measured_marker_list) == 0:
        #if we expected to see markers at this particle but there weren't any visible
        #markers then our error is high
        if len(predicted_markers) > 0:
            return 0.1 #weight is 0.1
        return 1 #weight is 1

    #P(measurements match predicted markers)
    measured_particle_prob = []

    #matching p to mm_list
    for mm_x, mm_y, mm_h, in measured_marker_list:
        marker_prob_list = []
        #calculate if the measured matches predicted marker
        for g_x, g_y, g_h in predicted_markers:
            p_x = gauss_pdf(0, MARKER_TRANS_SIGMA, mm_x - g_x)
            p_y = gauss_pdf(0, MARKER_TRANS_SIGMA, mm_y - g_y)
            p_h = gauss_pdf(0, MARKER_ROT_SIGMA, mm_h - g_h)
            #add the new data set to the marker_prob_list
            marker_prob_list.append(P_AorB([p_x, p_y, p_h]))
        measured_particle_prob.append(P_AorB(marker_prob_list))
    #returns weight of individual particle
    return P_AandB(measured_particle_prob)

#This functions computes P(x|mean, sigma)
def gauss_pdf(mean, sigma, x):
    return (1/(sigma * np.sqrt(2 * math.pi)) * math.exp(-(x - mean)**2 / (2 * sigma**2)))

#does P(a and b) = P(a)P(b) for all particles
def P_AandB(probability_list):
    p = 1
    for i in probability_list:
        p = p * i
    return p

#does P(a or b) = P(a) + P(b) - P(a and b) for all particles
def P_AorB(marker_prob_list):
    p = 0
    for i in marker_prob_list:
        p = p + i - (p * i)
    return p
