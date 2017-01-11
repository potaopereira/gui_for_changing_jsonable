#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

import json

# import services defined in quad_control
from planner_example.srv import *


import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# get the file path for rospy_tutorials
import sys
sys.path.insert(0, rospack.get_path('planner_example')+'/src')
import jsonable_class_example
MISSIONS_DATABASE = jsonable_class_example.room_database

class Hotel():

    def __init__(self):
        pass

    # callback for when changing mission
    def _handle_service_change_mission(self,req):

        # dictionary = '{"sequence_inner_key":"","key":"","input_string":""}'
        dictionary = json.loads(req.dictionary)

        sequence_of_inners = dictionary["sequence_inner_key"]

        if sequence_of_inners:

            self.room_under_construction.change_inner_of_inner(**dictionary)
            if rospy.get_param('playing_back',True):
                self.message += "\n\n" + 'At ' + str(rospy.get_time() - self.message_time) + ' seconds'
                dictionary = json.loads(req.dictionary)
                sequence_of_inners = dictionary["sequence_inner_key"]      

                jsonable = self.room_under_construction.get_inner(sequence_of_inners)
                msg = jsonable.object_combined_description()
                self.message += "\n\n" + msg

        else:
            # change mission, if sequence is empty

            # mission name
            self.mission_name = dictionary["key"]

            # chosen class taken from dictionary
            MissionClass = MISSIONS_DATABASE[dictionary["key"]]
        
            aux = self.room_under_construction
            self.room_under_construction = MissionClass.from_string(dictionary["input_string"])            
            aux.to_do_before_finishing()

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvChangeJsonableObjectByStrResponse(received = True)   


    # callback for when changing mission
    def _handle_service_call_mission_method(self,req):

        # dictionary = '{"sequence_inner_key":"","func_name":"","input_to_func":""}'
        dictionary = json.loads(req.dictionary)

        msg = self.room_under_construction.call_method_inner_of_inner(**dictionary)

        # return message to Gui, to let it know resquest has been fulfilled
        return SrvChangeJsonableObjectByStrResponse(received = True)     


    def check_room(self):

        # start node 
        rospy.init_node('my_hotel_routine', anonymous=True)
        rate = rospy.Rate(1.0)

        #-----------------------------------------------------------------------#
        rospy.Service('ServiceChangeMission', SrvChangeJsonableObjectByStr, self._handle_service_change_mission)
        rospy.Service('ServiceChangeMissionCallMethod', SrvChangeJsonableObjectByStr, self._handle_service_call_mission_method)
        
        #-----------------------------------------------------------------------#
        # construct a default object
        self.room_under_construction = MISSIONS_DATABASE['Room1']()

        while not rospy.is_shutdown():
            
            print('room area = ' + str(self.room_under_construction.area))
            print('bed length = ' + str(self.room_under_construction.bed.length))

            # go to sleep
            rate.sleep()

if __name__ == '__main__':
    my_hotel = Hotel()
    try:
        my_hotel.check_room()
    except rospy.ROSInterruptException:
        pass