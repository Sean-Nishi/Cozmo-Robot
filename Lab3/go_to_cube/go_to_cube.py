#Lab 3 for Mobile Robotics
#Sean Nishi and Stanley Chan
#go_to_cubes.py

'''
Sources provided by prof
State Machine
https://pypi.org/project/python-statemachine/

Writing to Cozmo's screen, uses anki's vector code
https://www.kinvert.com/anki-vector-text-on-screen/

'''

#imported libraries
import asyncio
import cv2
import sys
import time
import numpy as np
import cozmo
import math
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from statemachine import StateMachine, State

#

#state machine
class CozmoStates(StateMachine):
    #Machine states, starts looking left
    look_left = State('Look left for cube', initial = True)
    look_right = State('Look right for the cube')
    turn_left = State('Turning left')
    turn_right = State('Turning right')
    drive_straight = State('Drive Straight')
    near_cube = State('In front of cube')

    ###############################################################################
    #transition functions
    cube_not_found = look_left.to.itself() | look_right.to.itself() | turn_left.to(look_left) | turn_right.to(look_right) | drive_straight.to(look_left) | near_cube.to(look_left)

    found_cube_left = look_left.to(turn_left) | look_right.to(turn_right) | turn_left.to.itself() | turn_right.to(turn_left) | drive_straight.to(turn_left) | near_cube.to.itself()

    found_cube_right = look_left.to(turn_right) | look_right.to(turn_right) | turn_left.to(turn_right) | turn_right.to.itself() | drive_straight.to(turn_right) | near_cube.to.itself()

    found_cube_ahead = look_left.to(drive_straight) | look_right.to(drive_straight) | turn_left.to(drive_straight) | turn_right.to(drive_straight) | drive_straight.to.itself() | near_cube.to.itself()

    saw_cube_left = drive_straight.to(look_left)

    saw_cube_right = drive_straight.to(look_right)

    #################################################################################
    #on_... functions. These execute when the state changes, mostly set motor_speed
    def on_enter_look_left(self):
        self.set_motor_speed(-5, 5)

    def on_enter_look_right(self):
        self.set_motor_speed(5, -5)

    def on_enter_turn_left(self):
        self.set_motor_speed(-1, 2)

    def on_enter_turn_right(self):
        self.set_motor_speed(2, -1)

    def on_enter_drive_straight(self):
        self.set_motor_speed(5, 5)

    #def on_cube_not_found(self):
        #print('cube_not_found is called in state machine')

    #when we are near cube1, we automatically set cube_number to 2
    def on_enter_near_cube(self):
        self.cube_number = 2

    #####################################################################################
    #vars to keep track of all the info. Will be propagated at runtime
    l_motor_speed = 0
    r_motor_speed = 0
    max_speed = 15
    #how close we need to be -> 70mm
    proximity_distance = 70
    spin = 1
    mid_screen = 38

    #which cube are we looking for?
    cube_number = 1
    #unique cubes in array
    #cube1_properties = CustomObjectMarkers.Diamonds2
    #cube2_properties = CustomObjectMarkers.Diamonds3
    cube_arr = {1: CustomObjectMarkers.Diamonds2, 2: CustomObjectMarkers.Diamonds3}

    #is the current cube detected?
    cube = False
    #cube coordinates both x and y
    cube_coords = False
    #pose of the cube on the screen
    cube_pose = False
    #bool to keep track of cube
    see_cube = False
    start_cube2 = False

    #cozmo's coordinates, will be x, y values only.
    #set from set_cozmos_coords()
    cozmo_coords = False

    different_vector = False
    #Where does Cozmo need to go? x and y coords, probably should make this an array later
    go_to_x = False
    go_to_y = False

    #string to keep track of the last state
    current_state = 'Look left for cube'
    last_state = ""

    ####################################################################################
    #setters and getters
    def set_cozmo_coords(self, coords):
        self.cozmo_coords = coords

    def set_motor_speed(self, left, right):
        self.l_motor_speed = left
        self.r_motor_speed = right

    def get_last_state(self):
        return self.last_state

    def get_current_state(self):
        return self.current_state

    ######################################################################################
    #Cozmo sees a cube
    def handle_object_appeared(self, evt, **kw):
        if isinstance(evt.obj, CustomObject):
            print("Cozmo started seeing %s" % str(evt.obj.object_type))
            self.cube = evt.obj
            self.cube_coords = evt.obj.pose
            self.see_cube = True
            self.cube_pose = evt.obj.pose

            if (self.cube_coords is not False) and (self.cozmo_coords is not False):
                self.different_vector = self.cube_coords.position - self.cozmo_coords.position

            self.calc_goto_coords()
        return

    #Cozmo can't see cube
    def handle_object_disappeared(self, evt, **kw):
        if isinstance(evt.obj, CustomObject):
            print("Cozmo stopped seeing %s" % str(evt.obj.object_type))
            self.see_cube = False

        return

    #does the big math, translates matrix
    def calc_goto_coords(self):
        try:
            self.cube_coords = self.cube.pose.position


            cube_matrix = np.matrix([[self.cube_coords.position.x],
                                     [self.cube_coords.position.y],
                                     [1]])
            
            cos = math.cos(self.cozmo_coords.rotation.angle_z.radians)
            sin = math.sin(self.cozmo_coords.rotation.angle_z.radians)

            #translation matrix with cozmo's angles in radians
            translation_matrix = np.matrix([[cos, -sin, self.cozmo_coords.position.x],
                                            [sin, cos, self.cozmo_coords.position.y],
                                            [0, 0, 1]])

            inverse_trans_matrix = translation_matrix.I

            #calculate coords to travel
            new_x = (inverse_trans_matrix * cube_matrix).A[0][0]
            new_y = (inverse_trans_matrix * cube_matrix).A[1][0]

            self.go_to_x = new_x
            self.go_to_y = new_y

            return new_x, new_y

        except:
            return "something wrong with calculating coordinates"
    

    #calculated how far away the cube is from Cozmo, hypotenous of x, y coords
    def calc_dist_from_cozmo(self):
        x_squared = math.pow(self.go_to_x, 2)
        y_squared = math.pow(self.go_to_y, 2)

        return math.sqrt(x_squared + y_squared)

##########################################################################################
#main function, need to be async because of the say_text(), really dumb
async def main(robot: cozmo.robot.Robot):
    #start with the state machine, immediatly set Cozmo's coordinates
    state_machine = CozmoStates()
    state_machine.set_cozmo_coords(robot.pose.position)

    #event handlers for the 2 cubes
    robot.add_event_handler(cozmo.objects.EvtObjectAppeared, CozmoStates.handle_object_appeared)
    robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, CozmoStates.handle_object_disappeared)

    # define a unique cube (44mm x 44mm x 44mm) (approximately the same size as a light cube)
    # with a 30mm x 30mm Diamonds2 image on every face
    await robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                           state_machine.cube_arr[state_machine.cube_number],
                                           44,
                                           30, 30, True)

    #cozmo looks for cube1 first
    state_machine.cube_number = 2
    await robot.say_text('b', in_parallel = True).wait_for_completed()

    while True:
        #refresh cozmo's coords every .1 seconds
        print('\n##########new loop##########\n')

        #get new camera image
        event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout = 30)

        state_machine.set_cozmo_coords(robot.pose.position)

        #when we are in part 1
        print('check cube #')
        if state_machine.cube_number is 1:
            print('cube = 1')
            #did we find the cube? initially false
            if state_machine.cube_coords is False:
                print('cube1 not found')
                state_machine.cube_not_found()
                print('do we get here?')
            else:
                print('cube1 found')
                state_machine.calc_goto_coords()

                #if we are still far away from the cube
                if (state_machine.calc_dist_from_cozmo() > state_machine.max_distance):
                    #go left or right?
                    if state_machine.go_to_y > 25:
                        state_machine.go_left()
                    elif state_machine.go_to_y < -25:
                        state_machine.go_right()
                    else:
                        state_machine.go_straight()

                else:
                    robot.stop_all_motors()
                    state_machine.stopped_at_cube1()

        #Finished part 1, starting part 2
        if state_machine.cube_number is 2:
            if state_machine.start_cube2 is False:
                print('\n\nLOOKING FOR CUBE2')
                #need to refresh properties of the cube in the state machine
                robot.stop_all_motors()
                await robot.say_text('Look for cube 2', in_parallel = True).wait_for_completed()
                state_machine.set_cozmo_coords(robot.pose.position)
                state_machine.calc_goto_coords()
                state_machine.calc_dist_from_cozmo()
                state_machine.cube = False
                state_machine.cube_coords = False
                state_machine.start_cube2 = True

                #give new properties of cube to state_machine
                await robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                           state_machine.cube_arr[state_machine.cube_number],
                                           44,
                                           30, 30, True)


            #if we dont see the cube or can't get the coords, turn
            if (state_machine.cube_coords is False) or (state_machine.see_cube is False):
                #handling cozmo's movement correction
                print('cube2 not found')
                if state_machine.get_current_state() is 'drive_straight':
                    if state_machine.go_to_y < state_machine.midpoint:
                        state_machine.saw_cube_left()
                    else:
                        state_machine.saw_cube_right()
                
                #if cozmo sees the cube, where do we go? (left, right, or straight)
            else:
                print('cube2 found')
                state_machine.calc_goto_coords()
                                 #is cozmo close to the cube?
                if state_machine.calc_dist_from_cozmo() > state_machine.proximity_distance:
                    if state_machine.go_to_y > 25:
                        state_machine.found_cube_left()
                    elif state_machine.go_to_y < -25:
                        state_machine.found_cube_right()
                    else:
                        state_machine.drive_straight()

                #cozmo is close to the cube
                else:
                    state_machine.set_motor_speed(0, 0)

            #update states
            last_state = state_machine.get_last_state()
            current_state = state_machine.get_current_state()

            #if last_state is not current_state:
                #print('CHANGING STATES!')
                #print('LAST STATE: ' + last_state)
                #print('CURRENT STATE: ' + current_state)
                #state_machine.last_state = current_state

                #face_image = Image.new('RGBA', (128, 64), (0, 0, 0, 255))
                #dc = ImageDraw.Draw(face_image)
                #screen_text = cozmo.oled_face.convert_image_to_screen_data(faceImage)
                #robot.display_oled_face_image(screen_text, 600000000, in_parallel = True)

            await robot.drive_wheels(state_machine.l_motor_speed, state_machine.r_motor_speed)

        time.sleep(0.1)       

cozmo.run_program(main, use_viewer=True)
