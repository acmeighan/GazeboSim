#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gym.envs.registration import register
from limo_robot_spawner.limobot_env import LimoBotEnv
import gym
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
import os
import numpy as np

class TrainedAgent(Node):

    def __init__(self):
        super().__init__("trained_limobot",allow_undeclared_parameters=True,automatically_declare_parameters_from_overrides=True)

def main(args=None):
    rclpy.init()
    node = TrainedAgent()
    node.get_logger().info("Trained agent node has been created")

    # Get directory where models are saved
    home_dir = os.path.expanduser('~')
    pkg_dir = 'ros2_ws/src/HospitalBot-Path-Planning/limo_robot_spawner'
    trained_model_path = os.path.join(home_dir, pkg_dir, 'rl_models', 'PPO_model_name.zip')

    # Register the gym environment
    register(
        id="LimoBotEnv-v0",
        entry_point = "limo_robot_spawner.limobot_env:LimoBotEnv",
        max_episode_steps = steps
    )

    env = gym.make("LimoBotEnv-v0")
    env = Monitor(env)
    
    # Showing 10 episodes so its not just a one and done
    episodes = 10

    # This is done to bypass the problem between using two different distros of ROS (humble and foxy)
    # They use different python versions, for this reason the action and observation space cannot be deserialized from the trained model
    # The solution is passing them as custom_objects, so that they won't be loaded from the model
    #  - I'm not sure this will actually be an issue for us, but I keep just in case
    custom_obj = {'action_space': env.action_space, 'observation_space': env.observation_space}

    # Load the trained model
    model = PPO.load(trained_model_path,env=env,custom_objects=custom_obj)

    # Evaluating the trained agent
    mean_ep_reward, num_steps = evaluate_policy(model,env=env,n_eval_episodes=100,
                                                return_episode_rewards=True,deterministic=True
                                            )
    
    # Print data
    node.get_logger().info("Mean Reward: " + str(np.mean(Mean_ep_rew)) + " - Std Reward: " + str(np.std(Mean_ep_rew)))
    node.get_logger().info("Max Reward: " + str(np.max(Mean_ep_rew)) + " - Min Reward: " + str(np.min(Mean_ep_rew)))
    node.get_logger().info("Mean episode length: " + str(np.mean(Num_steps)))

    # Close env to print information and destroy node
    env.close()

    node.get_logger().info("Completed evaluation, destroying node")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()