from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from functools import partial
import numpy as np
import math
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, SetModelState, SetEntityState
import os
from ament_index_python.packages import get_package_share_directory

class LimoController(Node):
    """
    # Need to go through and check if topics and services have the right paths

    This class defines all the methods to:
        - Publish actions to the agent (move the robot)
        - Subscribe to sensors of the agent (get laser scans, image, and robot position)
        - Reset the simulation

    Topics list:
        - /demo/cmd_vel - Twist(): linear and angular velocity of the robot
        - /demo/odom - Odometry(): odometry readings of the chassis of the robot
        - /demo/laser/out - LaserScan(): laser readings
        - /insert/path/here - Image(): camera readings
    
    Services used:
        - /demo/set_entity_state: sets the new state of the robot and target when an episode ends

    Services not used:
        - /reset_simulation : resets the gazebo simulation
        - /delete_entity : unspawns the robot from the simulation
        - /spawn_entity : spawns the robot in the simulation in a semi-random position
    """
    def __init__(self):
        super().__init__('limo_controller')
        self.get_logget().info("The robot controller node has been created")

        # PUBLISHERS AND SUBSCRIBERS

        # Action publisher
        self.action_pub_ = self.create_publisher(Twist,'/demo/cmd_vel',10)
        # Position subscriber
        self.pose_sub_ = self.create_subscription(Odometry,'/demo/odom',self.pose_callback,1)
        # Laser subscriber
        self.laser_sub_ = self.create_subscription(LaserScan,'/demo/laser/out',self.laser_callback,1)
        # Image subscriber
        self.camera_sub_ = self.create_subscription(Image,'/insert/path/here',self.image_callback,1)
        # Reset model state client - this resets the pose and velocity of a given model within the world
        self.client_state_srv_ = self.create_client(SetEntityState,'/demo/set_entity_state')

        # Reset simulation client - UNUSED
        self.client_sim = self.create_client(Empty,'/reset_simulation')

        # Get the directory of the sdf of the robot
        #  - I assume our directory will be slightly different
        self.pkg_dir_ = os.path.join(
            get_package_share_directory("limo_robot_spawner"),"models",
            "pioneer3at","model.sdf"
        )

        # Initialize attributes
        #  - these are re-written on start of simulation
        self.agent_location_ = np.array([np.float32(1),np.float32(16)])
        self.laser_reads_ = np.array([np.float32(10)] * 61)
        self.image_reads_ = np.zeros(shape=(INSERT_SHAPE_HERE))

    # Method to send action (velocity command) to robot
    def send_velocity_command(self,velocity):
        cmd = Twist()

        cmd.linear.x = float(velocity[0])
        cmd.angular.z = float(velocity[1])

        # Publishing action
        self.action_pub_.publish(cmd)
    
    # Method that saves the position of robot each time odometry topic recieves new message
    def pose_callback(self,msg: Odometry):
        #  - here np.clip takes the value and makes sure that it stays within a certain range
        #  - in this case the min/max x and y values for the simulated environment
        #  - I assume the clip will not be neccessary for real world

        # Need to add z dimension to agent_location_
        self.agent_location_ = np.array([np.float32(msg.pose.pose.position.x), np.float32(msg.pose.pose.position.y), np.float32(msg.pose.pose.position.z)])
        self.agent_orientation_ = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        # Letting the env know that pose has been updated
        self.done_pose_ = True

    # Method that saves the laser reading of robot each time laser topic recieves new message
    def laser_callback(self,msg: LaserScan):
        self.laser_reads_ = np.array(msg.ranges)
        # Converts infinite values to 10
        #  - Likely good to cap the distance an object can be for real world as well
        #  - Probably should say [laser_reads_ >= distance] = np.float()
        self.laser_reads_[self.laser_reads_ == np.inf] = np.float32(10)
        # Letting env know laser has been updated
        self.done_laser_ = True

    # Method that saves the camera image of robot each time image topic recieves new message
    def image_callback(self,msg: Image):
        # Hopefully the right call
        #  - can use msg._height / msg._width ?
        self.image_reads_ = np.array(msg.data)
        # Insert processing to grayscale here if needed

        # Letting env know image has been updated
        self.done_image_ = True

    # Method to set the state of the robot when an episode ends
    #  - robot_pose = [x, y, z, w]
    def call_set_robot_state_service(self,robot_pose=[1,16,-0.707,0.707]):
        while not self.client_state_srv_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service. . .")

        request = SetEntityState.Request()
        request.state.name = self.robot_name

        # Pose
        request.state.pose.position.x = float(robot_pose[0])
        request.state.pose.position.y = float(robot_pose[1])
        request.state.pose.orientation.z = float(robot_pose[2])
        request.state.pose.orientation.w = float(robot_pose[3])
        # Velocity
        request.state.twist.linear.x = float(0)
        request.state.twist.linear.y = float(0)
        request.state.twist.linear.z = float(0)
        request.state.twist.angular.x = float(0)
        request.state.twist.angular.y = float(0)
        request.state.twist.angular.z = float(0)

        future = self.client_state_srv_.call_async(request)
        future.add_done_callback(partial(self.callback_set_target_state))

    # Method that elaborates the future object 
    def callback_set_target_state(self,future):
        try:
            response = future.result()
            self.done_set_robot_state_ = True
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

    # UNUSED FUNCTIONS CAN BE ADDED LATER BELOW IF WE RUN INTO ISSUES
































