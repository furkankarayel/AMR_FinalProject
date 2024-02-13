#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import time

class PointNavigator:
    def __init__(self, points):
        rospy.init_node('point_navigator', anonymous=True)
        self.points = points
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

    def navigate_to_point(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w = 1.0
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result()

    def navigate_points(self):
        for point in self.points:
            self.navigate_to_point(point)
            time.sleep(3)  # Warte 3 Sekunden

if __name__ == '__main__':
    try:
        points = [
            (0.6765097209589571, -1.1908636674782875),
            (1.4735485270854725, -0.9925743975983549),
            (1.4440110453089945, -0.33752260314715615),
            (1.5605005636806843, 0.11472925183844497),
            (0.46509913429254096, -0.04579242410091182),
            (1.7927051123065936, -0.48963289746345856),
            (2.256541542162795, -1.2000623722725514),
            (2.6552557672937134, -1.3339251596473334),
            (3.09991381491737, -0.7898910926578544),
            (2.548136220176166, 0.07529909904891244)
        ]
        navigator = PointNavigator(points)
        navigator.navigate_points()
    except rospy.ROSInterruptException:
        pass
