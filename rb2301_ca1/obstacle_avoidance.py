# https:#github.com/yonx30/rb2301_ca1/blob/main/src/rb2301_ca1/rb2301_ca1/obstacle_avoidance.py
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import floor # +++

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")
        self.other_init() # +++

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message

        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.


    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        # self.last_scan = np.array(msg.ranges)[::20] # Slices the 721 scan array to return only 36 scans. Feel free to edit
        self.last_scan = np.array(msg.ranges)
    

    # def sillyPrintMiddle(self): # only for scan_angle = 1
    #     if self.rangesCurrent == "NONE": return '---'

    #     i = (self.ranges[self.rangesCurrent][0] + self.ranges[self.rangesCurrent][1])/2
    #     id_ = i%len(self.last_scanTemp)
    #     return f"{self.last_scanTemp[int(id_)]:3f}"
    #     # is this correct?
    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        # cd rb2301_ca1
        # colcon build --symlink-install
        # ./gz_ca1.sh
        # ./ca1.sh
        # n = 9: 360 / 8 = 45 degree angles
        self.last_scanTemp = self.last_scan[::2][:-1] # only 360 please

        state = self.stateQ[0]
        count = self.states[state][0]
        mov = {"x": 0, "y": 0, "heading": 0}
        # self.get_logger().debug(str(self.last_scan))

        
        # \033
        # self.get_logger().debug(f"{state}:\tx{count}\thit: {self.analyseGeneralRays(*self.ranges["LEFT"], True):3f}")
        self.get_logger().debug(f"{state}:\tx{count}\thit: {self.analyseGeneralRays(*self.ranges[self.rangesCurrent], True):3f}")
        mov = self.STATE_SCAN(mov, state, count)

        self.move_2D(mov["x"] * self.move_mult, mov["y"] * self.move_mult, mov["heading"])

        ######################## MODIFY CODE HERE ########################
    def STATE_SCAN(self, mov, state, count):
        
        if (state == "SCAN FORWARD"):
            if (count > 0): # [counter] for this process
                self.rangesCurrent = "FORWARD"
                did_it_hit = self.analyseGeneralRays(*self.ranges["FORWARD"])
                if (did_it_hit): # [continue]
                    # console.log('still safe...')
                    
                    # if (self.scan_angle != 1):  # don't bother
                    #     mov['heading'] = 1
                    #     self.added_heading += 1 # turn left (to capture full if limited # of rays) 
                    self.decState(state)
                else : # [interrupt] hit something that way so it's blocked
                    # add = self.scan_angle - self.states[state] # pass counter to undo rotation
                    # self.registerState("UNDO SCAN", add)
                    self.registerState("SCAN LEFT") # next stage
                    self.endState(state)
                
            else : # [pass] counter over - it's safe
                # console.log('pass')
                # self.registerState("UNDO SCAN", self.scan_angle)
                self.registerState("MOVE FORWARD", self.ambient_walk) # idk how much
                self.endState(state)
        
        elif (state == "MOVE FORWARD"):
            if (count > 0): # [counter]
                mov["x"] = 1
                self.decState(state)
            
            else : # [pass]
                self.registerState("SCAN FORWARD", self.scan_angle)
                self.endState(state)
        # elif (state == "UNDO SCAN"):
        #     if (count > 0): # [counter] 
        #         # self.registerState("BLOCKED", 1000000000000000000000)
        #         # return mov
        #         if (self.scan_angle != 1):  # don't bother
        #             mov['heading'] = -1; self.added_heading -= 1

        #         self.decState(state)
        #     else : # [pass]
        #         # no registering state
        #         self.endState(state)

        elif (state == "BLOCKED"):
            if (count > 0): # [counter] 
                ...
                self.decState(state)
            else : # [pass]
                self.registerState("SCAN FORWARD", self.scan_angle)
                self.endState(state)
        # ------------------------------------------------------
        elif (state == "SCAN LEFT"): # pretty much same as other scans btw
            if (count > 0): # [counter] for this process
                self.rangesCurrent = "LEFT"
                did_it_hit = self.analyseGeneralRays(*self.ranges["LEFT"])
                if (did_it_hit): # [continue]
                    if (self.scan_angle != 1):  # don't bother
                        mov['heading'] = 1; self.added_heading += 1 # turn left 
                    
                    self.decState(state)
                else : # [interrupt]
                    # add = self.scan_angle - self.states[state] # pass counter to undo rotation
                    # self.registerState("UNDO SCAN", add) # p.s. this would effectively do nothing if scan_angle = 1
                    self.registerState("SCAN RIGHT") # next stage
                    self.endState(state)
            else : # [pass]
                # self.registerState("UNDO SCAN", self.scan_angle)
                self.registerState("MOVE LEFT", self.ambient_walk)
                self.endState(state)
            

        elif (state == "SCAN RIGHT"): 
            if (count > 0): # [counter] for this process
                self.rangesCurrent = "RIGHT"
                did_it_hit = self.analyseGeneralRays(*self.ranges["RIGHT"])
                if (did_it_hit): # [continue]
                    if (self.scan_angle != 1):  # don't bother
                        mov['heading'] = 1; self.added_heading += 1 # turn left 
                    
                    self.decState(state)
                else : # [interrupt]
                    # add = self.scan_angle - self.states[state] 
                    # self.registerState("UNDO SCAN", add)
                    self.registerState("BLOCKED", 10000) # you promise no backtracking 😡
                    self.endState(state)
            else : # [pass]
                # self.registerState("UNDO SCAN", self.scan_angle)
                self.registerState("MOVE RIGHT", self.ambient_walk)
                self.endState(state)
            

        elif (state == "MOVE LEFT"):
            if (count > 0): # [counter]
                mov["y"] = 1
                self.decState(state)
            
            else : # [pass]
                self.registerState("SCAN FORWARD", self.scan_angle)
                self.endState(state)
            
        
        elif (state == "MOVE RIGHT"):
            if (count > 0): # [counter]
                mov["y"] = -1
                self.decState(state)
            
            else : # [pass]
                self.registerState("SCAN FORWARD (RIGHT)", self.scan_angle)
                self.endState(state)
            

        elif (state == "SCAN FORWARD (RIGHT)"):
            if (count > 0): # [counter] for this process
                self.rangesCurrent = "FORWARD"
                did_it_hit = self.analyseGeneralRays(*self.ranges["FORWARD"])
                if (did_it_hit): # [continue]
                    if (self.scan_angle != 1):  # don't bother
                        mov['heading'] = 1; self.added_heading += 1 # turn left
                    
                    self.decState(state)
                else : # [interrupt]
                    # add = self.scan_angle - self.states[state] 
                    # self.registerState("UNDO SCAN", add)
                    # [B1] right bias
                    self.registerState("SCAN RIGHT") # next stage
                    self.endState(state)
                
            else : # [pass] counter over - it's safe
                # self.registerState("UNDO SCAN", self.scan_angle)
                self.registerState("MOVE FORWARD", self.ambient_walk) # idk how much
                self.endState(state)
            

        return mov


    def other_init(self):
        # self.size = [10, 10]
        self.ambient_walk = 20
        self.move_mult = 1.     # program is slow :(
        self.range_mult = 1.

        self.scan_angle = 1.   # doesn't work with other scan angles yet haha
        self.last_scanTemp = []
        self.loc = [0, 0]                 

        self.heading = 0
        # self.hit_size = .1
        self.states = {
            'SCAN FORWARD': [self.scan_angle, ],
            # 'BLOCKED': [], # assign default here
        }
        self.stateQ = ['SCAN FORWARD']
        self.ranges= {  # please adjust ranges because the lidar isn't actually centered on the robot as you see fit :)
            "FORWARD":       [330     , 360+30  , .25], # please make sure ranges move forward
            "BACKWARD":      [150     , 210     , .2],
            "RIGHT":         [210     , 330     , .2],
            "LEFT":          [30      , 150     , .2],
            "BACK RIGHT":    [210 -10 , 240 + 10, .2],
            "BACK LEFT":     [120 -10 , 150 + 10, .2],
            "FORWARD RIGHT": [30 - 10 , 60 + 10 , .2],
            "FORWARD LEFT":  [300 - 10, 330 + 10, .2],
            "ALL": [0, 359, 0.2],
            "NONE": [None, None]
        }
        for key in self.ranges.keys():
            if len(self.ranges[key]) >= 3:
                self.ranges[key][2] *= self.range_mult

        self.rangesCurrent = 'NONE'
        
        self.pred_loc = self.loc
        self.pred_heading = self.heading
        self.added_heading = 0

    def registerState(self, state, counter=1):
        self.stateQ.append(state)
        if (state not in self.states): 
            self.states[state] = []
        self.states[state].append(counter) 
    
    def endState(self, state):
        self.states[state] = self.states[state][1:]
        self.stateQ = self.stateQ[1:]
    
    def decState(self, state):
        self.states[state][0] -= 1

    def AInRange(self, a, l, u): # check if angle between 
        return (a - l)%360 <= (u - l)%360
    
    def analyseGeneralRays(self, startT, endT, hitDist=0.3, take_Min=False):

        # unnecessary:
        min_ = float('inf')
        if startT is None: return False

        arrL = len(self.last_scanTemp)
        s = floor(startT / self.scan_angle)
        e = floor(endT / self.scan_angle) 
        i = s


        while((i%arrL) != ((e + 1)%arrL) ):
            check = 0
            id_ = i%arrL
            start = self.scan_angle * (id_)
            target = self.scan_angle * (id_+1)%arrL
            current = self.scan_angle * (id_) + self.added_heading
            if (i == s):
                if (self.AInRange(current, startT, target)):   
                    check = 1
            elif (i == e): 
                if (self.AInRange(current, start, endT)):
                    check = 2
            else: 
                check = 3 

            # self.get_logger().debug(f"{ check } {id_}")
            if ((check)>0 and take_Min): 
                min_ = min(min_, self.last_scanTemp[id_])
            if ((check>0) and (self.last_scanTemp[id_] < hitDist)):

                if (not take_Min): return False
            
            i+=1
        if (take_Min): return min_
        
        return True 

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()