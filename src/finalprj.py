#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
import actionlib
import math
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan

class TurtleBot:
    def __init__(self,easy,hard):
        rospy.init_node('point_navigator', anonymous=True)
        self.hardGoals = hard
        self.easyGoals = easy
        self.skippedGoals = []
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        self.selfpos_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_selfpos, queue_size=1)
        self.selfpos = PoseWithCovarianceStamped()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_scan, queue_size=1)
        self.scan = LaserScan()


    def callback_scan(self, msg):
        self.scan = msg

    def callback_selfpos(self, msg):
        self.selfpos = msg

    def get_nearest_goal(self,goalArray):
        dist = float('inf')
        currGoal = None

        for goal in goalArray: 
            x = goal[0]
            y = goal[1]
            print('Goal position x: ', x)
            print('Goal position y: ', y)
            print('Bot position x: ', self.selfpos.pose.pose.position.x)
            print('Bot position y: ', self.selfpos.pose.pose.position.y)
            newDist = math.sqrt((x - self.selfpos.pose.pose.position.x)**2 + (y - self.selfpos.pose.pose.position.y)**2)
            if dist > newDist:
                print('New distance ', newDist)
                print('Old distance: ', dist)
                dist = newDist
                currGoal = goal

        return currGoal
    
    def recovery(self): 
        print('Recovery start')

        self.move_base.cancel_all_goals()
        rospy.loginfo("trying to fix")
        search = True 
        count = Twist() 
        count.linear.x = 0
        self.publisher.publish(count)
    
        while search:      
            
            freesport = 0
            for i in range(21):
                
                laser = self.scan.ranges[180 + i]
                

                if laser == 0 or laser > 0.40:
                    freesport += 1

                if freesport >= 10:
                    search = False
                    break
    
        rospy.sleep(0.5)
        count.linear.x = -0.2
        count.angular.z = 0
        self.publisher.publish(count)
        rospy.sleep(1.5)
        count.linear.x = 0
        count.angular.z = 0
        self.publisher.publish(count)



    def navigate_to_point(self, currGoal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = currGoal[0]
        goal.target_pose.pose.position.y = currGoal[1]
        goal.target_pose.pose.orientation.w = 1.0
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result()
        result = self.move_base.get_state()
        return result

    def remove_entry(self, array, entry):
        for item in array:
            if item == entry:
                array.remove(item)
                return
    def add_entry(self, array, entry):
        for item in array:
            if item == entry:
                array.append(item)
                return

    def navigate_points(self,goalArray):
        count = 0
        while goalArray:
            currGoal = self.get_nearest_goal(goalArray)
            drive_to_goal = True
            retry_count = 0
            count += 1
            print('Array index: ', count)

            clearCostmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

            while drive_to_goal:
                result = self.navigate_to_point(currGoal)

                if result == 3: # goal is reached
                    rospy.sleep(1)
                    drive_to_goal = False
                    clearCostmap()
                    self.remove_entry(goalArray,currGoal)

                if result == 4: # goal cant be reached
                    retry_count += 1
                    if retry_count >= 2:
                         print('recovery strategy needed here')
                         self.recovery()

                if retry_count >= 3: # cancel goal after 3 retries
                    drive_to_goal = False
                    self.add_entry(self.skippedGoals,currGoal)
                    print('skippedGoal')

                if len(goalArray) == 0:
                    return # no more positions in the list


    def driveEasyGoals(self):
        print('Easy Goals start')
        self.navigate_points(self.easyGoals)

    def driveHardGoals(self):
        print('Hard Goals start')
        self.navigate_points(self.hardGoals)

    def driveSkippedGoals(self):
        print('Skipped Goals start')
        self.navigate_points(self.skippedGoals)
        self.skippedGoals = [] # reset the skipped Goals, now the goals will be skipped for real



if __name__ == '__main__':
    try:
        hardPositions = [
            (1.7992810527102268, -0.5384085236560651),
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
        bot.driveEasyGoals()
        if(len(bot.skippedGoals) != 0):
            print('skipped Easygoals:',bot.skippedGoals)
            bot.driveSkippedGoals()
        bot.driveHardGoals()
        if(len(bot.skippedGoals) != 0):
            print('skipped Hardgoals:',bot.skippedGoals)
            bot.driveSkippedGoals()


    except rospy.ROSInterruptException:
        pass
