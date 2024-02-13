#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import time


class TurtleBot:
    def __init__(self,easy,hard):
        rospy.init_node('point_navigator', anonymous=True)
        self.hardGoals = hard
        self.easyGoals = easy
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        self.selfpos_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_selfpos, queue_size=1)
        self.selfpos = PoseWithCovarianceStamped()

    def callback_selfpos(self, msg):
        self.selfpos = msg

    def get_nearest_goal(goalArray):
        dist = float('inf')
        currGoal = None

        for goal in goalArray: 
            x = goal[0]
            y = goal[1]
            newDist = math.sqrt((x - self.selfpos.pose.pose.position.x)**2 + (y - self.selfpos.pose.pose.position.y)**2)
            if dist > newDist:
                dist = newDist
                currGoal = goal

        return currGoal

    def navigate_to_point(self, currGoal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = currGoal[0]
        goal.target_pose.pose.position.y = currGoal[1]
        goal.target_pose.pose.orientation.w = 1.0
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result()

    def navigate_points(self,goalArray):
        while goalArray:
            currGoal = get_nearest_goal(goalArray)
            self.navigate_to_point(currGoal)
            time.sleep(3)  # Warte 3 Sekunden

    def easyGoals(self):
        self.navigate_points(self.easyGoals)

if __name__ == '__main__':
    try:
        hardPositions = [
            (1.7927051123065936, -0.48963289746345856),
            (2.256541542162795, -1.2000623722725514),
            (2.6552557672937134, -1.3339251596473334),
            (3.09991381491737, -0.7898910926578544),
            (2.548136220176166, 0.07529909904891244)
        ]

        easyPositions = [  
            (0.6765097209589571, -1.1908636674782875),
            (1.4735485270854725, -0.9925743975983549),
            (1.4440110453089945, -0.33752260314715615),
            (1.5605005636806843, 0.11472925183844497),
            (0.46509913429254096, -0.04579242410091182)
        ]

        bot = TurtleBot(easyPositions,hardPositions)
        bot.easyGoals()
    except rospy.ROSInterruptException:
        pass