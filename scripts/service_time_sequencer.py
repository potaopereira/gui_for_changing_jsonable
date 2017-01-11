#!/usr/bin/env python
"""Class ServiceTimeSequencer that is used in node that accepts a sequence of services with trigger instants and sends these services according to these trigger instants"""

import rospy

# import services
from planner_example.srv import *

services_database = {}
services_database['ServiceChangeMission'] = SrvChangeJsonableObjectByStr
services_database['ServiceChangeMissionCallMethod'] = SrvChangeJsonableObjectByStr

import json
import std_msgs.msg

# TODO: reply may have other fields
def request_service(namespace,service_name_str,service_inputs_dic):
    """Sets up a service based on service name, and dictionary with inputs to service"""

    ServiceClass     = services_database[service_name_str]
    service_name_str = namespace+service_name_str
    try: 
        # time out of one second for waiting for service
        rospy.wait_for_service(service_name_str,1.0)
        try:
            # request service
            request = rospy.ServiceProxy(service_name_str, ServiceClass)
            reply   = request(**service_inputs_dic)

            if reply.received == True:
                rospy.logwarn('Service '+service_name_str+' provided')
        except rospy.ServiceException as exc:
            rospy.logwarn('Service '+service_name_str+' did not process request: ' + str(exc))
            return
    except rospy.ServiceException as exc:
        rospy.logwarn('Service '+service_name_str+' did not process request: ' + str(exc))
        return

class ServiceTimeSequencer():

    def __init__(self,frequency = 2.0,infinity  = 3600):
        """Construct node that provides services at frequency not larger than 'frequency' and 'infinity' is taken as an infinite time interval"""
        
        # frequency of node (Hz)
        self.frequency = frequency

        # infinite trigger instant in sec (approx one hour) 
        self.infinity  = infinity

    def handle_service_sequence(self,data = std_msgs.msg._String.String()):
        self.service_sequence = json.loads(data.data)
        self.initial_instant  = rospy.get_time()
        self.update()

    def handle_service_sequence_with_publish(self,data = ServiceSequenceRequest(service_sequence = '')):
        """handle for when new sequence of services is received"""

        self.pub_service_sequecer.publish(data.service_sequence)
        
        # loads string with sequence of services (which is a list where each element is a dictionary)
        # and each dictionary is of the form dictionary = {'trigger_instant':'','service_name':'','inputs_service':''}
        # inputs to service is on its own also a dictionary
        self.service_sequence = json.loads(data.service_sequence)
        self.initial_instant  = rospy.get_time()
        self.update()
        
        return ServiceSequenceResponse(received = True)
        

    def update(self):
        """Update next_trigger_instant, service_name and inputs_to_service"""
        
        # if service_sequence is non-empty
        if self.service_sequence:
            
            # remove first service in list of services
            service = self.service_sequence.pop(0)

            self.next_trigger_instant = service['trigger_instant']
            self.service_name         = service['service_name']
            self.inputs_service       = service['inputs_service']
            
        else:
            # next
            self.next_trigger_instant = self.infinity

    def update_and_request(self):
        """Request for the current service, and update next_trigger_instant, service_name and inputs_to_service"""

        # TODO: maybe comment this
        print("asking for: "+self.service_name)
        request_service(self.namespace,self.service_name,self.inputs_service)

        # update next service to be provided and corresponding trigger instant
        self.update()

    def loop(self):
        """Sets up service that accepts sequence of services and provides services sequentially"""

        # name node as service_time_sequencer
        rospy.init_node('service_time_sequencer', anonymous=True)

        if rospy.get_param('playing_back',True):
            print('kkkkkkkkkkkkkk\nllllllllllllll\n')
            # publish so that rosbag may be used to save this 
            self.sub_service_sequecer = rospy.Subscriber(
                name = 'ServiceSequencer',
                data_class = std_msgs.msg.String,
                callback = self.handle_service_sequence)
        else:
            # print('kkkkkkkkkkkkkk\nllllllllllllll\n')
            # service for selecting desired trajectory
            rospy.Service(
                name = 'ServiceSequencer', 
                service_class = ServiceSequence, 
                handler = self.handle_service_sequence_with_publish)

            # publish so that rosbag may be used to save this 
            self.pub_service_sequecer = rospy.Publisher(
                name = 'ServiceSequencer',
                data_class = std_msgs.msg.String,
                queue_size = 1)           

        rate = rospy.Rate(self.frequency)

        self.namespace = ''

        # initial_instant is reset every time a nw service_sequence is received
        self.initial_instant      = rospy.get_time()

        # next (not absolute) time instant when service is provided
        self.next_trigger_instant = self.infinity

        while not rospy.is_shutdown():

            # elapsed time
            elapsed_time = rospy.get_time() - self.initial_instant

            # if elapsed time is bigger than trigger instant
            if elapsed_time >= self.next_trigger_instant:
                self.update_and_request()

            # go to sleep
            rate.sleep()


if __name__ == '__main__':
    service_sequencer = ServiceTimeSequencer()
    try:
        service_sequencer.loop()
    except rospy.ROSInterruptException:
        pass