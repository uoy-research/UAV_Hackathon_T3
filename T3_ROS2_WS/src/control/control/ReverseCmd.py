import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from interfaces.msg import MoveIn, T, NewT, OpticalFlow, Landing, MoveOut, Commands, Obstacle

from enum import Enum

class State(Enum):
    Waiting = 0
    SignalReceived = 1
    WaitForSignal = 2
    Rotate = 3
    ReverseCommands = 4
    ReturnHome = 5
    ObstacleAvoidance = 6
    Landing = 7

def zero_twist():
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0314

    return twist


class ReverseCmd(Node):

    def __init__(self):
        super().__init__('ReverseCmd')

        #Initial state
        self.state = State.Waiting

        #Clocks
        self.T = 0

        #Variables
        self.command = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.commands = [[]]
        self.new_commands = [[]]
        self.t = 0
        self.new_t = 0
        self.i = 0
        self.t_max = 0 

        #constants
        self.timeout = 5
        # self.stop = (0.0, 0.0 ,0.0 , 0.0, 0.0, 0.0)
        # self.rotate = (0.0, 0.0 ,0.0 , 0.0, 0.0, 1.0)
        # self.land = (0.0, 0.0, -0.2, 0.0, 0.0, 0.0)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
	
        #twist message
        self.twist = zero_twist()

        self.stop = zero_twist()

        self.rotate = zero_twist()
        self.rotate.angular.z = 0.314


        #events
        ##ComandIn
        self.MoveInSub = self.create_subscription(Twist, "cmd_vel", self.MoveIn_callback, 10)
        self.MoveIn_event = False
        self.NewTSub = self.create_subscription(NewT, 'NewT', self.NewT_callback, 10)
        self.NewT_event = False
        ##SensorIn
        self.opticalFlow = 10000
        self.opticalFlow_event = False
        self.OpticalFlowSub = self.create_subscription(OpticalFlow, 'OpticalFlow', self.OpticalFlow_callback, 10)
        self.rotated = False
        self.obstacle_event = False
        self.ObstacleSub = self.create_subscription(Obstacle, 'Obstacle', self.Obstacle_callback, 10)
        self.landing_event = False
        self.LandingSub = self.create_subscription(Landing, 'Landing', self.Landing_callback, 10)
        #Operations
        self.MoveOut = self.create_publisher(Twist,"cmd_vel", 10) 
        self.PublishCommands = self.create_publisher(Commands, 'Commands', 10)
        self.Publish_t = self.create_publisher(T, 'T', 10)
    #event callbacks
    def MoveIn_callback(self,msg):
        self.command[0] = msg.linear.x
        self.command[1] = msg.linear.y
        self.command[2] = msg.linear.z
        self.command[3] = msg.angular.x
        self.command[4] = msg.angular.x
        self.command[5] = msg.angular.z
        self.MoveIn_event = True

    def NewT_callback(self,msg):
        self.NewT_event = True
        self.new_t = msg

    def Obstacle_callback(self,msg):
        self.obstacle_event = msg

    def Landing_callback(self,msg):
        self.landing_event = msg

    def OpticalFlow_callback(self,msg):
        self.opticalFlow = msg
        self.opticalFlow_event = True


    

    

    def control_loop(self):
        Exec = False
        while Exec == False:
            if self.state == State.Waiting:
                #condition
                if self.MoveIn_event == True:
                    #action
                    if self.i >= len(self.commands):
                    # Resize self.commands to accommodate self.t + 1 elements
                        self.commands.extend([[] for _ in range(len(self.commands), self.i+1)])
                    self.commands[self.i] = self.command.copy()
                    self.MoveIn_event = False
                    #next state
                    print("Enter SignalReceived")
                    self.state = State.SignalReceived
                #condition
                elif self.MoveIn_event == False:
                    #next state
                    self.state = State.Waiting
                    print("Enter Waiting")
                    #exec
                    Exec = True

            elif self.state == State.SignalReceived:
                #entry
                self.T = 0
                #next state
                print("enter state WaitForSignal")
                self.state = State.WaitForSignal
                #exec
                self.T = self.T + self.timer_period
                Exec = True

            elif self.state == State.WaitForSignal:
                #condition
                if self.T < self.timeout and self.MoveIn_event == True:
                    #action
                    self.i = self.i+1
                    if self.i >= len(self.commands):
                    # Resize self.commands to accommodate self.t + 1 elements
                        self.commands.extend([[] for _ in range(len(self.commands), self.i+1 )])
                    self.commands[self.i] = self.command.copy()
                    self.MoveIn_event = False
                    #next state
                    print("Enter SignalReceived")
                    self.state = State.SignalReceived

                #condition
                elif self.T < self.timeout and self.MoveIn_event == False:
                    #next state
                    ("Enter state WaitForSignal")
                    self.state = State.WaitForSignal
                    #exec
                    self.T = self.T+timer_period
                    Exec = True
                    

                #condition
                elif self.T >= self.timeout:
                    #action
                    self.MoveOut.publish(self.stop)
                    self.T = 0
                    #next state
                    print("Enter state Rotate")
                    self.state = State.Rotate
                    #exec
                    Exec = True

            elif self.state == State.Rotate:
                #print("self.commands in Rotate"+str(self.commands))
                #condition
                if self.T <= 10:
                    #action
                    self.MoveOut.publish(self.rotate)
                    #exec
                    self.state = State.Rotate
                    self.T = self.T + self.timer_period
                    print("Enter state Rotate")
                    Exec = True
                #condition
                elif self.T > 10:
                    #action
                    self.MoveOut.publish(self.stop)
                    #exec
                    self.state = State.ReverseCommands
                    print("enter state ReverseCommands")
                    Exec = True

            elif self.state == State.ReverseCommands:
                #entry
                #self.t = self.t +1
                #condition
                if self.i >= 0:
                    #action
                    i = self.i
                    if self.t >= len(self.new_commands):
                    # Resize self.commands to accommodate self.t + 1 elements
                        self.new_commands.extend([[] for _ in range(len(self.new_commands), self.t +1 )])
                    self.new_commands[self.t] = [-self.commands[i][0], -self.commands[i][1], -self.commands[i][2], -self.commands[i][3], -self.commands[i][4], -self.commands[i][5]]
                    #change state
                    self.t = self.t+1
                    self.i = self.i -1
                    self.state = State.ReverseCommands
                elif self.i <0:
                    #action
                    msg = Commands()
                    # Convert list of tuples to a flat list using list comprehension then publish
                    msg.cmd = [float(item) for sublist in self.new_commands for item in sublist]
                    #print(msg)
                    self.PublishCommands.publish(msg)
                    self.t_max = self.t - 1
                    self.t = 0
                    #change state
                    print(" Enter state ReturnHome")
                    self.state = State.ReturnHome
                    #exec
                    Exec = True

            elif self.state == State.ReturnHome:
                #entry
                
                t = self.t
                #condition
                if self.t <= self.t_max and self.obstacle_event == False:
                    #action
                    msg = Twist()
                    msg.linear.x = self.new_commands[t][0]
                    msg.linear.y = self.new_commands[t][1]
                    msg.linear.z = self.new_commands[t][2]
                    msg.angular.x = self.new_commands[t][3]
                    msg.angular.y = self.new_commands[t][4]
                    msg.angular.z = self.new_commands[t][5]
                    #print(msg)
                    self.MoveOut.publish(msg)
                    self.t = self.t+1
                    #exec
                    Exec = True
                #condition
                elif self.obstacle_event == True:
                    #action
                    self.MoveOut.publish(self.stop)
                    self.Publish_t.publish(t)
                    #next state
                    print("Enter state ObstacleAvoidance")
                    self.state = ObstacleAvoidance
                    #exec
                    Exec = True

                #condition
                elif self.t > self.t_max:
                    #next state
                    print("Enter state Landing")
                    self.state = State.Landing


            elif self.state == State.ObstacleAvoidance:
                #condition
                if self.NewT_event == True and landing_event == False:
                    #action
                    self.t = self.new_t
                    #next state
                    self.state = ReturnHome
                    self.NewT_event = False
                    #exec
                    Exec = True
                #condition
                elif self.NewT_event == False and landing_event == False:
                    #next state
                    print("Enter state ObstacleAvoidance")
                    self.state = ObstacleAvoidance
                    #exec
                    Exec = True
                #condition
                elif self.landing_event == True:
                    #action
                    self.MoveOut.publish(self.land)
                    #next state
                    print("Enter state Landing")
                    self.state = Landing
                    #exec
                    Exec = True

            elif self.state == State.Landing:
                #condition
                if self.opticalFlow_event == True and self.distanceToGround > 0.1:
                    #next state
                    self.state = Landing
                    #exec
                    Exec = True
                #condition
                elif self.opticalFlow_event == True and self.distanceToGround <= 0.1:
                    self.MoveOut.publish(stop)
                    print("Finished")
                #condition
                elif self.opticalFlow_event == False:
                    #exec
                    Exec = True

            else:
                pass

def main(args=None):
    rclpy.init(args=args)

    reverseCmd = ReverseCmd()

    rclpy.spin(reverseCmd)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reverseCmd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
