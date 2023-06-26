import rclpy
from gym import Env
from gym.spaces import Dict, Box
import numpy as np
import random
# need to replace with 'LimoController'
from hospital_robot_spawner.robot_controller import RobotController
import math

class LimoBotEnv(RobotController, Env):
    """
    
    Insert comments here on RL environment:

    """

    def __init__(self):

        # Initialize the RobotController Node which calls from rclpy.node
        super().__init__()
        self.get_logger().info("Started publishers and subscribers from RobotController Node")

        # ENVIRONMENT PARAMETERS (can I do this in separate file and make them global?)
        self.robot_name = "LimoBot"
        # Initializes starting location of the agent for each episode (x,y,theta)
        #  - only used if starting location is not randomized
        #  - need to input (x,y,theta) based on new simulation environment
        self.initial_agent_location_ = np.array([x,y,theta], dtype=np.float32) 
        # Initializes 4 inner corners of the simple square road
        #  - this needs to be changed if we create more complex maps
        #  - which we should (all right turns except for one left)
        #  - here we assume roads constant width of n units
        self.inner_four_corners_ = np.array([[x1,y1],[x2,y2],[x3,y3],[x4,y4]])
        self.inner_four_polar_corners_ = self.cart_to_pol(self.inner_four_corners_)
        # Initializes 4 outer corners of simple square road - see concerns above
        self.outer_four_corners_ = np.array([[x1,y1],[x2,y2],[x3,y3],[x4,y4]])
        self.outer_four_polar_corners_ = self.cart_to_pol(self.outer_four_corners_)
        # Defines the level of randomization of the env
        #  - more randomization makes the model more generalizable
        #  0: no randomization
        #  1: semi-randomize robots initial position
        #  2: evaluation level
        #  - could this also be different maps? ex train on square vs circle vs ect
        self.randomize_env_level_ = n
        # Decides whether or not the observation space is normalized between [0,1]
        self.normalize_obs_ = True
        # Decides whether or not the action space is normalized between [-1,1]
        self.normalize_act_ = True
        # Decides the method used to compute the reward for each step
        #  0: simple reward
        #  1: other reward ect ect.
        self.reward_method_ = 0
        # Initializes the maximum linear velocity used in actions
        self.max_linear_velocity_ = 1
        # Initializes the minimum linear velocity used in actions
        self.min_linear_velocity_ = 0
        # Initializes the angular velocity used in actions. Always symmetric,
        #  - if you want opposite direction just turn negative
        self.angular_velocity_ = 1
        # Initializes the minimum distance the LIMO can be from the road
        #  - if min_dist < radius of robot epsiode is concluded with failure
        #  - need to use shape of LIMO to find this value
        #  - note: radius = distance from center of robot to front corner
        self.min_dist_from_edge_ = radius

        # INSERT ALTERNATIVE HEURISTIC PARAMETERS HERE

        # Initialize step count
        self.num_steps_ = 0

        # Initialize episode count
        self.num_episodes_ = 0

        # Set seed if we are evaluating a model
        if (self.randomize_env_level_ == 3):
            np.random.seed(13)

        # Debug prints on console
        self.get_logger().info("Initial agent location: {}".format(self.initial_agent_location_))
        self.get_logger().info("Maximum linear velocity: {}".format(self.max_linear_velocity_))
        self.get_logger().info("Minimum linear velocity: {}".format(self.min_linear_velocity_))
        self.get_logger().info("Angular velocity: {}".format(self.angular_velocity_))
        self.get_logger().info("Minimum distance from edge: {}".format(self.min_dist_from_edge_))

        # ACTION SPACE
        if self.normalize_act_ == True:
            # Create normalized action space - 2D continuous (linear, angular)
            self.action_space = Box(low=np.array([-1,1]), high=np.array([1,1]), dtype=np.float32)
        else:
            # Create action space - 2D continuous (linear, angular)
            self.action_space = Box(low=np.array([self.min_linear_velocity_,-self.angular_velocity_]),
                                    high=np.array([self.max_linear_velocity_,self.angular_velocity_]),
                                    dtype=np.float32)
            
        # OBSERVATION SPACE
        if self.normalize_obs_ == True:
            # Create normalized observation space 
            #  - dictionary with: "Laser reads", "Image reads"
            self.observation_space = Dict(
                {
                    # Laser reads are a certain shape (61?) and can range from 0.08 to 10 
                    #  - according to git
                    "laser": Box(low=0,high=1,shape=(61,),dtype=np.float32),
                    # Image reads are presumably a certain shape
                    #  - should range from 0 to 255 after grayscale
                    "image": Box(low=0,high=1,shape=(row,column),dtype=np.float32)
                }
            )
        
        else:
            # Create observation space 
            #  - dictionary with: "Laser reads", "Image reads"
            self.observation_space = Dict(
                {
                    # Laser reads are a certain shape (61?) and can range from 0.08 to 10 
                    #  - according to git
                    "laser": Box(low=0,high=np.inf,shape=(61,),dtype=np.float32),
                    # Image reads are presumably a certain shape
                    #  - should range from 0 to 255 after grayscale
                    "image": Box(low=0,high=255,shape=(row,column),dtype=np.float32)
                }
            )
        
        # AGENT INIT LOCATIONS
        #  - [x, y, angle, x_lowerbound, x_upperbound, y_lowerbound, y_upperbound, angle_lowerbound, angle_upperbound]
        #  - x, y and angle defines the center of location and orientation
        #  - the bounds define the limits from the center in which the robot will spawn
        self.robot_locations = [
            [x1,y1,theta1,x_lb1,x_ub1,y_lb1,y_ub1,theta_lb1,theta_ub1],
            [x2,y2,theta2,x_lb2,x_ub2,y_lb2,y_ub2,theta_lb2,theta_ub2]
        ]

        # Creating variables to monitor number of successes and failures
        self.successes_ = 0
        self.failures_ = 0
        self.completed_paths_ = 0 # not sure if needed for our problem statement

    def step(self,action):

        # Increase step number
        self.num_steps_ += 1

        # De-normalize action to send command to robot
        if self.normalize_act_ == True:
            action = self.denormalize_action(action)
        
        # Apply the action
        #  - this function should come from the RobotController class
        #  - action here is (linear,angular) which gets turned into a Twist()
        self.send_velocity_command(action)

        # Spin the node until the laser and the image reads are updated
        #  - note that this is a singular spin unlike TurtleBot()
        self.spin()

        # Compute the new coordinates of the robot
        #  - update both x and y coords and update a self.on_road_
        self.transform_coordinates()

        # Update laser and image reads
        #  - function comes from this class
        observation = self.get_obs()

        # Update infos
        info = self.get_info()

        # Compute reward
        reward = self.compute_rewards(info)

        # Compute statistics for evaluation
        if (self.randomize_env_level_ == 3):
            self.compute_statistics(info)

        # Check if episode is terminated
        #  - terminate if off road or collision occurs
        done = (self.on_road_ == False) or (any(info["laser"] < self._minimum_dist_from_obstacles))

        self.get_logger().info("Episode {} done".format(self.num_episodes_))

        return observation, reward, done, info
    
    def render(self):
        # Not needed for this environment because all of our rendering occurs in Gazebo
        pass

    def reset(self,seed=None,options=None):
        #self.get_logger().info("Resetting the environment")

        # Increment episode counter
        self.num_episodes_ += 1

        # Randomize starting location of agent
        pose2d = self.randomize_robot_location()

        # Reset done variable
        #  - need to move the *_ in RobotController class
        self.done_set_rob_state_ = False 

        # Call the set robot position service
        self.call_set_robot_state_service(pose2d)

        # Spin node until we get connection from /set_entity_state service response
        #  - same as above, move *_
        #  - also check if this self.done var gets switched from True to False after exiting while loop
        while self.done_set_rob_state_ == False:
            rclpy.spin_once(self)

        # Compute initial observation 
        self.spin()
        self.polar_agent_location_ = self.cart_to_pol(self.agent_location_)

        # Updates state and additional infos
        observation = self.get_obs()
        info = self.get_info()

        # Reset the number of steps
        self.num_steps_ = 0

        return observation
    
    def get_obs(self):
        # Returns the current state of the system
        #  - I think it is still worth it to return the location of the agent
        #  - 1) see if robot has exited road in simulation
        #  - 2) see if robot has exited road in real world
        #  - even if processes for determining are different
        obs = {"laser": self.laser_reads_,"image": self.image_reads_}
        # Normalize observations
        if self.normalize_obs_ == True:
            obs = self.normalize_observation(obs)
        return obs

    def get_info(self):
        # Returns the laser and image reads
        #  - would normally include distance from agent to target
        return {
            "location": self.agent_location_,
            "polar_location": self.polar_agent_location_,
            "laser": self.laser_reads_,
            "image": self.image_reads_
        }
    
    def spin(self):
        # This function does a singular spin of an rclpy node until sensors get new data
        #  - probably need self.done_ for laser, image, and pose
        self.done_pose_ = False
        self.done_laser_ = False
        self.done_image_ = False
        while (self.done_pose_ == False) or (self.done_laser_ == False) or (self.done_image_ == False):
            rclpy.spin_once(self)
    
    def cart_to_pol(self,array):
        # assuming array is (n x 2) - x, y
        p_array = np.zeros(shape=array.shape)

        for i in range(array.shape[0]):
            p_array[i][0] = np.sqrt(array[i][0]**2 + array[i][1]**2)
            p_array[i][1] = np.arctan2(array[i][0], array[i][1])
        
        return p_array

    def randomize_robot_location(self):

        # Random level 0 or 2 - fixed position 
        if (self.randomize_env_level_ == 0) or (self.randomize_env_level_ == 2):
            position_x = float(self.initial_agent_location_[0])
            position_y = float(self.initial_agent_location_[1])
            angle = float(math.radians(self.initial_agent_location_[2]))
            orientation_z = float(math.sin(angle/2))
            orientation_w = float(math.cos(angle/2))

        # Random level 1 - semi-random position
        if self.randomize_env_level_ == 1:
            row = random.randint(0,self.robot_locations.shape[0])
            position_x = float(self.robot_locations[row][0]) + random.uniform(-1,1)
            position_y = float(self.robot_locations[row][1]) + random.uniform(-1,1)
            angle = float(math.radians(-90) + math.radians(random.uniform(-30,30)))
            orientation_z = float(math.sin(angle/2))
            orientation_w = float(math.cos(angle/2))

        return [position_x,position_y,orientation_z,orientation_w]
    
    def compute_rewards(self,info,action):
        # Computing rewards for a single step in an episode

        # Reward type is simple
        #  - yes, I know this only works for a square track centered at (0, 0)
        if self.reward_method_ == 0:
            # If polar location is further from center than inner corners
            #  - and closer to center than outer corners it gets a positive reward
            if (info["polar_location"][0] > self.inner_four_polar_corners_[0][0]):
                if (info["polar_location"][0] < self.outer_four_corners_[0][0]):
                    # reward is 3 if action is primarily linear_velocity
                    #  - if linear is twice as large as angular
                    if action[0] > 2 * action[1]:
                        reward = 3
                    #  - if if angular is similar or greater than linear
                    else: 
                        reward = 1
            # If agent has left the road it gets negative reward and episode ends
            else:
                reward = -10
                self.get_logger().info("AGENT HAS LEFT THE ROAD")

        # Reward type is other
        if self.reward_method_ == 1:
            # Insert different reward functions below
            reward = 0
            
        return reward
    
    def normalize_observation(self,observation):
        # This method normalizes the observations taken from the robot
        #  - observation contains "laser" and "image" reads
        
        # Laser reads will range from 0 to n (replace 10 with n)
        observation["laser"] = observation["laser"]/10
        # Image reads should never be RGB, only grayscale [0,255]
        #  - 0: black, 255: white
        observation["image"] = observation["image"]/255

        return observation
    
    def denormalize_action(self,norm_action):
        # This method de-normalizes the action before sending it to the robot
        action_linear = ((self.max_linear_velocity_*(norm_action[0]+1)) + (self.min_linear_velocity_*(1-norm_action[0])))/2
        action_angular = action_angular = ((self._angular_velocity*(norm_action[1]+1)) + (-self._angular_velocity*(1-norm_action[1])))/2

        return np.array([action_linear,action_angular],dtype=np.float32)
    
    def compute_statistics(self,info):
        # This method is used to compute statistics when in evaluation mode
        pass

    def close(self):
        # Shuts down the node to avoid creating multiple nodes on reinit of environment
        self.destroy_node()





