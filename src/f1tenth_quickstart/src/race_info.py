#!/usr/bin/env python
import rospy
from f1tenth_gym_ros.msg import RaceInfo

class Agent(object):
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/race_info', RaceInfo, self.race_info_feedback, queue_size=1)
        self.ego_finish_time = 0
        self.opp_finish_time = 0
        self.ego_last_lap_count = 0
        self.opp_last_lap_count = 0

    # feedback function for race_info topic
    def race_info_feedback(self, race_info_msg):
        return
        '''
        print("-------INFO-------")
        print("ego count:  ", race_info_msg.ego_lap_count)
        print("ego timing: ", race_info_msg.ego_elapsed_time)
        print("opp count:  ", race_info_msg.ego_lap_count)
        print("opp timing: ", race_info_msg.ego_elapsed_time)

        if (self.ego_last_lap_count == 0 and race_info_msg.ego_lap_count == 1):
            self.ego_finish_time = race_info_msg.ego_elapsed_time
        if (self.opp_last_lap_count == 0 and race_info_msg.opp_lap_count == 1):
            self.opp_finish_time = race_info_msg.opp_elapsed_time

        self.ego_last_lap_count = race_info_msg.ego_lap_count
        self.opp_last_lap_count = race_info_msg.opp_lap_count

        if (race_info_msg.ego_lap_count >= 1):
            print("============= ego finish time: ", self.ego_finish_time)
        if (race_info_msg.opp_lap_count >= 1):
            print("============= opp finish time: ", self.opp_finish_time)
        '''

if __name__ == '__main__':
    rospy.init_node('dummy_agent')
    dummy_agent = Agent()
    rospy.spin()
