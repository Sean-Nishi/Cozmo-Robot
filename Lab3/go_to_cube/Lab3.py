#Lab 3 for Mobile Robotics
#Sean Nishi and Stanley Chan
#go_to_cubes.py

import time
import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from statemachine import StateMachine, State

#state machine
class CozmoStates(StateMachine):
    #State Machine states, will need more
    find_cube1 = State('Search for cube1',initial = True)
    move_to_cube1 = State('Moving towards cube1')
    stop_at_cube1 = State('Cozmo in front of cube1')
    
    #functions for the state machine, need to update
    cube_detected = find_cube1.to(move_to_cube1)
    stopped_at_cube1 = move_to_cube1.to(stop_at_cube1)

    #above is good. We have the states to deal with cube1 and the functions
    #to transition to the different states.

    #default behavior when state machine begins
    def on_cube_detected(self):
        print('find_cube1 -> move_to_cube')
        #ERROR HERE! DOESN'T ACCEPT "B" AS A STRING
        cozmo.robot.Robot.say_text("b").wait_for_completed()
        #may need different syntax but drive_wheels stays the same
        cozmo.robot.Robot.drive_wheels(15, 15)

    #cube1 detected, align and move forward
    def on_move_to_cube1(self):
        print('Cube1 detected, moving to cube1')
        cozmo.robot.say_text("b").wait_for_completed()
        #still need to align with center of the screen
        cozmo.robot.stop_all_motors()
        cozmo.robot.Robot.drive_wheels(15, 15)

    #stop movement
    def on_stop_at_cube1(self):
        print('Close to cube1, stopping movement')
        cozmo.robot.say_text('b').wait_for_completed()
        cozmo.robot.stop_all_motors()

        #cube has stopped moving 
        # transistion from moving state to stop state
        print('exiting moving state')
        print('entering stop state at cube1')
        self.stopped_at_cube1()

#global machine_state
machine_state = CozmoStates()

def handle_object_appeared(evt, **kw):
    # This will be called whenever an EvtObjectAppeared is dispatched -
    # whenever an Object comes into view.
    if isinstance(evt.obj, CustomObject):
        print("Cozmo started seeing a %s" % str(evt.obj.object_type))
        machine_state.cube_detected()
        #cozmo.robot.Robot.drive_wheels(-15, 15)


def handle_object_disappeared(evt, **kw):
    # This will be called whenever an EvtObjectDisappeared is dispatched -
    # whenever an Object goes out of view.
    if isinstance(evt.obj, CustomObject):
        print("Cozmo stopped seeing a %s" % str(evt.obj.object_type))


def custom_objects(robot: cozmo.robot.Robot):
    # Add event handlers for whenever Cozmo sees a new object
    robot.add_event_handler(cozmo.objects.EvtObjectAppeared, handle_object_appeared)
    robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, handle_object_disappeared)

    # define a unique cube (44mm x 44mm x 44mm) (approximately the same size as a light cube)
    # with a 30mm x 30mm Diamonds2 image on every face
    cube_obj = robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                              CustomObjectMarkers.Diamonds2,
                                              44,
                                              30, 30, True)
    
    #state machine initializaion
    #machine_state = CozmoStates()

    while True:
        time.sleep(0.1)


cozmo.run_program(custom_objects, use_viewer=True)
