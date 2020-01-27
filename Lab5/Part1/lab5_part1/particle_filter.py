from grid import *
from particle import Particle
from utils import *
from setting import *
from time import sleep
import numpy as np

alpha1 = alpha2 = 0.001
alpha3 = alpha4 = 0.005

def motion_update(particles, odom):
    """ Particle filter motion update
        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*
        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []
    #adding noise to odom measurements
    new_odom = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
    
    for p in particles:
        rot1 = math.atan2(new_odom[1][1] - new_odom[0][1], new_odom[1][0] - new_odom[0][0])# - new_odom[0][2]
        rotTrans = np.sqrt((new_odom[0][0] - new_odom[1][0])**2+(new_odom[0][1] - new_odom[1][1])**2)
        rot2 = new_odom[1][2] - new_odom[0][2] - rot1
             
        #gaussian stuff. this is the hat stuff
        #mu = 0, sigma is the expression
        h_rot1 = rot1 - np.random.normal(0, abs(alpha1*rot1 + alpha2*rotTrans))
        h_rotTrans = rotTrans - np.random.normal(0, abs(alpha3*rotTrans + alpha4*(rot1 + rot2)))
        h_rot2 = rot2 - np.random.normal(0, abs(alpha1*rot2 + alpha2*rotTrans))
        
        #move the particle with the big math
        #updates position and heading of each particle
        #print("Before coords: ", p.x, p.y, p.h)
        p.move(h_rot1, h_rotTrans, h_rot2)
        #motion_particles.append(p)
        #print("After moving coords: ", p.x, p.y, p.h)

    #return motion_particles
    return particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update
        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one
        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles
        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    weight = []
    count = 0

    #to be used below the first if...else... statement
    #temp lists for particles and weighted particles
    particle_list = []
    weighted_list = []
    new_particle_list = []

    #robot doesnt see any markers
    if len(measured_marker_list) == 0:
        s = 1
        for p in particles:
            weight.append((p, 1/len(particles)))
    #robot sees some markers
    else:
        for p in particles:
            visible_markers = p.read_markers(grid)

            #out of bounds check for particles
            if (p.x < 0) or (p.x >= grid.width) or (p.y < 0) or (p.y >= grid.height):
                weight.append((p, 0))
                continue
            if (p.x, p.y) in grid.occupied:
                weight.append((p, 0))
                continue

            match = []
            diff = int(math.fabs(len(measured_marker_list)-len(visible_markers)))

            for mm in measured_marker_list:
                if len(visible_markers) == 0:
                    break
                mmx, mmy, mmh = add_marker_measurement_noise(mm, MARKER_TRANS_SIGMA, MARKER_ROT_SIGMA)

                # find min_particle, the closest marker out of visible_markersarticle
                #initial settings before determining which is the
                #closest marker in visible_markers
                min_particle = visible_markers[0]
                min_dist = grid_distance(mmx, mmy, min_particle[0], min_particle[1])


                for v_p in visible_markers:
                    #visible_x, visible_y, and visible_h
                    v_px, v_py, v_ph = v_p[0], v_p[1], v_p[2]
                    #get distance
                    dist = grid_distance(mmx, mmy, v_px, v_py)
                    #check if it is lower
                    if dist < min_dist:
                        min_dist = dist
                        min_particle = v_p

                # storing [min_particle, mm] for later
                match.append((min_particle, mm))
                #remove the match from the visible markers
                visible_markers.remove(min_particle)

            # use match[] to calculate weight of p
            prob = 1

            maxc1 = 0
            maxc2 = (45 ** 2) / (2*(MARKER_ROT_SIGMA ** 2))

            #find the weights of each matching particle
            #min_particle -> i, mm -> j
            for i, j in match:
                dist_between_markers = grid_distance(i[0], i[1], j[0], j[1])
                angle_between_markers = diff_heading_deg(i[2], j[2])
                const1 = (dist_between_markers ** 2) / (2*(MARKER_TRANS_SIGMA ** 2))
                const2 = (angle_between_markers ** 2) / (2*(MARKER_ROT_SIGMA ** 2))
                maxc1 = max(maxc1, const1)
                prob *= np.exp(-const1-const2)

            for _ in range(diff):
                prob *= np.exp(-maxc1-maxc2)

            weight.append((p, prob))

        #normalize the weights
        s = 0
        weight.sort(key=lambda x: x[1])
        delete = int(PARTICLE_COUNT/100)
        weight = weight[delete:]
        for i, j in weight:
            if j == 0:
                count+=1
            else:
                s += j
        weight = weight[count:]
        count += delete

    #-------------------------------------------------------------------------
    #create new particles with the corresponding weights
    #
    for i, j in weight:
        new_particle = Particle(i.x, i.y, i.h)
        weighted_list.append(j/s)
        particle_list.append(new_particle)

    #if the particle_list isn't empty generate a random sample
    if particle_list != []:
        new_particle_list = np.random.choice(particle_list, len(particle_list), True, weighted_list)

    #create the new particles
    measured_particles = Particle.create_random(count, grid)[:]

    #add gaussian distribution/noise to results
    #already used new_particle as a var, now using new_parti
    for p in new_particle_list:
        noisy_h = add_gaussian_noise(p.h, ODOM_HEAD_SIGMA)
        noisy_x = add_gaussian_noise(p.x, ODOM_TRANS_SIGMA)
        noisy_y = add_gaussian_noise(p.y, ODOM_TRANS_SIGMA)
        new_parti = Particle(noisy_x, noisy_y, noisy_h)
        measured_particles.append(new_parti)

    #return the new list
    return measured_particles