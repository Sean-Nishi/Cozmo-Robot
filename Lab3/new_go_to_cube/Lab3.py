#Lab 3 for Mobile Robotics 1
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
import numpy as np
import cozmo
import time
import math

#state machine and object marker libs
from statemachine import StateMachine, State
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

#from anki's vector code
from PIL import ImageDraw, ImageFont, Image

#state machine
class CozmoStates(StateMachine):
    #######################################################################
    #Machine states, start looking left
    look_left = State('Look left for cube', initial=True)
    look_right = State('Look right for cube')
    turn_left = State('Turning left')
    turn_right = State('Turning right')
    drive_straight = State('Drive straight')
    near_cube = State('In front of cube')

    ####################################################################
    #transition functions
    cube_not_found = look_left.to.itself() | look_right.to.itself() | turn_left.to(look_left) | turn_right.to(look_right) | drive_straight.to(look_left) | near_cube.to(look_left)

    found_cube_left = look_left.to(turn_left) | look_right.to(turn_right) | turn_left.to.itself() | turn_right.to(turn_left) | drive_straight.to(turn_left) | near_cube.to.itself()

    found_cube_right = look_left.to(turn_right) | look_right.to(turn_right) | turn_left.to(turn_right) | turn_right.to.itself() | drive_straight.to(turn_right) | near_cube.to.itself()

    found_cube_ahead = look_left.to(drive_straight) | look_right.to(drive_straight) | turn_left.to(drive_straight) | turn_right.to(drive_straight) | drive_straight.to.itself() | near_cube.to.itself()

    saw_cube_left = drive_straight.to(look_left)

    saw_cube_right = drive_straight.to(look_right)

    at_cube = look_left.to(near_cube) | look_right.to(near_cube) | turn_left.to(near_cube) | turn_right.to(near_cube) | drive_straight.to(near_cube) | near_cube.to(near_cube)

    #####################################################################
    #on_... functions. These execute when a state is reached
    def on_enter_look_left(self):
        self.set_motor_speed(-1, 1)

    def on_enter_look_right(self):
        self.set_motor_speed(1, -1)

    def on_enter_turn_left(self):
        self.set_motor_speed(-1, 1.5)

    def on_enter_turn_right(self):
        self.set_motor_speed(1.5, -1)

    def on_enter_drive_straight(self):
        self.set_motor_speed(2, 2)

    #when we are near cube1, we automatically set cube_number to 2
    def on_enter_near_cube(self):
        #set the motor speed to 0
        self.set_motor_speed(0, 0)
        self.current_cube = 2

    ####################################################################################
    #vars to keep track of all the info. Will be propagated at runtime
    cozmo_coords = False
    l_motor_speed = 0
    r_motor_speed = 0
    max_speed = 10

    #how close we need to be -> 70mm
    proximity_distance = 70
    spin = 1
    mid_screen = 40

    #initially 1, will be changed to 2
    current_cube = 1
    #is the current cube detected?
    cube = False
    #what are the coords of the detected cube?
    cube_coords = False
    #pose of the cube?
    cube_pose = False
    #bool to keep track of cube
    see_cube = False 

    #cube array with the markers for the cubes we are detecting
    cube_arr = { 1: CustomObjectMarkers.Diamonds2, 2: CustomObjectMarkers.Diamonds3 }
    
    diff_vector = False    

    #where do we need to go?
    goto_x = False
    goto_y = False

    #string to keep track of last state
    last_state = ''

    ##################################################################################
    #handlers
    def handle_object_appeared(self, evt, **kw):
        if isinstance(evt.obj, CustomObject):
            self.cube = evt.obj
            self.set_cube_coords(evt.obj.pose)
            self.see_cube = True
            self.cube_pose = evt.obj.pose
            print('found cube at coords: ')
            print(self.cube_coords)
            print('Cozmo coords:')
            print(self.cozmo_coords)
            self.set_diff_vector()
            self.calc_goto_coords()
        return

    def handle_object_disappeared(self, evt, **kw):
        if isinstance(evt.obj, CustomObject):
            print('I lost the cube')
            self.see_cube = False
        return

    ########################################################################################
    #setters
    def set_cube_coords(self, coords):
        self.cube_coords = coords

    def set_cozmo_coords(self, coords):
        self.cozmo_coords = coords

    def set_motor_speed(self, left, right):
        self.l_motor_speed = left
        self.r_motor_speed = right

    def set_diff_vector(self):
        if (self.cube_coords is not False) and (self.cozmo_coords is not False):
            self.diff_vector = self.cube_coords - self.cozmo_coords

    def set_last_state(self, state):
        self.last_state = state

    #########################################################################################
    #getters
    def get_cube_coords(self):
        return self.cube_coords

    def get_cozmo_coords(self):
        return self.cozmo_coords

    def get_current_state(self):
        return self.current_state.identifier

    def get_last_state(self):
        return self.last_state

    def get_l_motor_speed(self):
        return self.max_speed * self.l_motor_speed

    def get_r_motor_speed(self):
        return self.max_speed * self.r_motor_speed

    #big math! does the translation of coord matricies
    def calc_goto_coords(self):
        #try catch block because first instance won't be calculated
        try:
            self.set_cube_coords(self.cube.pose)
            box_deg = self.cube_coords.rotation.angle_z.radians
            bot_deg = self.cozmo_coords.rotation.angle_z.radians
            diff_deg = bot_deg
     
            #need to 
            cozmo_x = self.cozmo_coords.position.x
            cozmo_y = self.cozmo_coords.position.y

            cube_x = self.cube_coords.position.x
            cube_y = self.cube_coords.position.y

            #cos and sin
            cos = math.cos(diff_deg)
            sin = math.sin(diff_deg)

            translation_matrix = np.matrix([[cos, -sin, cozmo_x],
                                            [sin, cos, cozmo_y],
                                            [0, 0, 1]])

            inverse_trans_matrix = translation_matrix.I

            cubeMatrix = np.matrix([[cube_x],
                                    [cube_y],
                                    [1]])

            #calc coords to travel
            new_x = (inverse_trans_matrix * cubeMatrix).A[0][0]
            new_y = (inverse_trans_matrix * cubeMatrix).A[1][0]

            #set them in state_machine
            self.goto_x = new_x
            self.goto_y = new_y

            return new_y, new_x

        except:
            return 'Coordinates haven\'t been calculated'

    #calculated the distance the cube is from cosmo
    def calc_dist_from_cozmo(self):
        x_squared = math.pow(self.goto_x, 2)
        y_squared = math.pow(self.goto_y, 2)
        return math.sqrt(x_squared + y_squared)

#main()
async def main(robot: cozmo.robot.Robot):
    #initialize the machine state
    state_machine = CozmoStates()
    state_machine.set_cozmo_coords(robot.pose.position)
    #event handlers
    robot.add_event_handler(cozmo.objects.EvtObjectAppeared, state_machine.handle_object_appeared)
    robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, state_machine.handle_object_disappeared)

    #are we looking for cube 2?
    look_for_cube_2 = False

    #cube1 definition
    await robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                         state_machine.cube_arr[state_machine.current_cube],
                                         44,
                                         30, 30, False)

    while True:
        print('######looping#####')
        #get camera image
        event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
        
        #refresh the coords
        state_machine.set_cozmo_coords(robot.pose)

        #triggers when we want the first cube
        if state_machine.current_cube == 1:
            print('cube = 1')
            if state_machine.cube_coords == False:
                state_machine.cube_not_found()
            else:
                state_machine.calc_goto_coords()
                if (state_machine.calc_dist_from_cozmo() > state_machine.proximity_distance):
                    if state_machine.goto_y >= 20:
                        state_machine.found_cube_left()
                    elif state_machine.goto_y < -20:
                        state_machine.found_cube_right()
                    else:
                        print('Cube ahead!')
                        state_machine.found_cube_ahead()
                else:
                    robot.stop_all_motors()
                    state_machine.on_enter_near_cube()

        #triggers when we want cube 2
        if state_machine.current_cube == 2:
            print('cube = 2')
            #reset the data if this is the first instance
            if look_for_cube_2 == False:
                robot.stop_all_motors()
                print('\nPART 1 DONE! GO TO CUBE 2\n')
                await robot.say_text('Look for cube 2', in_parallel=True).wait_for_completed()
                state_machine.set_cozmo_coords(robot.pose)
                state_machine.calc_goto_coords()
                print(state_machine.calc_dist_from_cozmo())
                look_for_cube_2 = True
                state_machine.cube_not_found()
                state_machine.cube_coords = False

                #define the second cube
                await robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                                             state_machine.cube_arr[state_machine.current_cube],
                                                             44,
                                                             30, 30, False)

            #where does cozmo need to move?
            if (state_machine.cube_coords == False) or (state_machine.see_cube == False):
                if state_machine.get_current_state() == 'drive_straight':
                    if state_machine.goto_y < state_machine.mid_screen:
                        state_machine.found_cube_left()
                    elif state_machine.goto_y < -20:
                        state_machine.found_cube_right()
                    else:
                        print('Cube ahead!')
                        state_machine.found_cube_ahead()
                else:
                    
                    state_machine.cube_not_found()
            #cozmo sees cube
            else:
                state_machine.calc_goto_coords()
                if (state_machine.calc_dist_from_cozmo() > state_machine.proximity_distance):
                    if state_machine.goto_y < -20:
                        state_machine.found_cube_right()
                    elif state_machine.goto_y > 20:
                        state_machine.found_cube_left()
                    else:
                        state_machine.found_cube_ahead()
                else:
                    robot.stop_all_motors()
                    state_machine.on_enter_near_cube()

        #update states
        last_state = state_machine.get_last_state()
        current_state = state_machine.get_current_state()

        #was there a state transition? WIll only trigger if a cube was detected
        if last_state is not current_state:
            #audio cue, takes a long time, need to comment out
            #await robot.say_text('b', in_parallel = True).wait_for_completed()
            #print the state transition to terminal & reset the state
            print('Last state: ' + last_state)
            print('Current state: ' + current_state)
            print('Coords to cube: ')
            #some reason, cant print this with the Coords to cube: print statement
            print(state_machine.calc_goto_coords())
            #update the state
            state_machine.set_last_state(current_state)

            # the anki victor text stuff, taken and modified
            #
            faceImage = Image.new('RGBA', (128, 64), (0, 0, 0, 255))
            dc = ImageDraw.Draw(faceImage)
            try:
                font_file = ImageFont.truetype('arial.ttf', 20)
            except IOError:
                print('IoError')
                try:
                    font_file = ImageFont.truetype(
                        '/usr/share/fonts/noto/NotoSans-Medium.ttf', 20)
                except IOError:
                    print('IoError 2')
                    pass
            dc.text((0, 0), current_state, fill=(255, 255, 255, 255), font=font_file)
            screen_data = cozmo.oled_face.convert_image_to_screen_data(faceImage)
            robot.display_oled_face_image(screen_data, 600000000.0, in_parallel=True)

        #update how robot should move
        await robot.drive_wheels(state_machine.get_l_motor_speed(), state_machine.get_r_motor_speed())
    
        #time.sleep(0.1)

cozmo.run_program(main, use_viewer = True, force_viewer_on_top = False)