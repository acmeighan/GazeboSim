#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from gym.envs.registration import register
from limo_robot_spawner.limobot_env import LimoBotEnv
import gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
import os
import optuna
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor

class TrainingNode(Node):

    def __init__(self):
        super().__init__("limobot_training", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # Defines which action the script will perform: "training", "hyperparam_tuning"
        self.training_mode_ = "training"

def main(args=None):

    # Initialize the training node to get the desired parameters
    rclpy.init()
    node = TrainingNode()
    node.get_logger().info("Training node has been started")

    # Create the directory where the trained RL models will be saved
    home_dir = os.path.expanduser('~')
    pkg_dir = 'ros2_ws/src/Hospitalbot-Path-Planning/limo_robot_spawner'
    trained_models_dir = os.path.join(home_dir,pkg_dir,'rl_models')
    log_dir = os.path.join(home_dir,pkg_dir,'logs')

    # If the directories do not exist, we create them
    if not os.path.exists(trained_models_dir):
        os.makedirs(trained_models_dir)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # First, register the gym environment created in limobot_env
    register(
        id="LimoBotEnv-v0",
        entry_point="limo_robot_spawner.limobot_env:LimoBotEnv",
        max_episode_steps = steps
    )

    node.get_logger().info("The environment has been registered")

    # Making environment
    env = gym.make('LimoBotEnv-v0')
    env = Monitor(env)

    # Sample Observation and Action space for Debugging
    #node.get_logger().info("Observ sample: " + str(env.observation_space.sample()))
    #node.get_logger().info("Action sample: " + str(env.action_space.sample()))

    # Checking stable baselines can use our environment
    check_env(env)
    node.get_logger().info("Environment check complete")

    # Creating two callbacks to be executed during training
    #  - may have to change reward threshold to higher number depending on number of steps
    stop_callback = StopTrainingOnRewardThreshold(reward_threshold=1000,verbose=1)
    eval_callback = EvalCallback(env,callback_on_new_best=stop_callback,eval_freq=100000,
                                 best_model_save_path=trained_models_dir,n_eval_episodes=40)

    # Training mode is training!
    if node.training_mode_ == "training":
        # Insert model name here:
        model_name = "PPO_limo"
        
        # Train model
        model = PPO("MultiInputPolicy",env,
                    verbose=1,tensorboard_log=log_dir,n_steps=20480, 
                    gamma=0.9880614935504514, gae_lambda=0.9435887928788405, 
                    ent_coef=0.00009689939917928778, vf_coef=0.6330533453055319, 
                    learning_rate=0.00001177011863371444, clip_range=0.1482
                )
        
        # Execute training
        try:
            model.learn(total_timesteps=int(20000000), reset_num_timesteps=False, 
                        callback=eval_callback, tb_log_name=model_name)
        except KeyboardInterrupt:
            model.save(f"{trained_models_dir}/{model_name}")

        # Save the trained model
        model.save(f"{trained_models_dir}/{model_name}")

    # Training mode is parameter tuning
    elif node.training_mode_ == "hyperparam_tuning":
        # Delete previously created environment
        env.close()
        del env
        # Hyperparameter tuning using Optuna
        study = optuna.create_study(direction="maximize")
        study.optimize(optimize_agent,n_trials=10,n_jobs=1)
        # Printing best parameters
        node.get_logger().info("Best hyperparameters: {}".format(study.best_params))

    # Shutting down the node
    node.get_logger().info("The training is finished, now the node is destroyed")
    node.destroy_node()
    rclpy.shutdown()

def optimize_ppo(trial):
    ## This method defines the range of hyperparams to search fo the best tuning
    return {
        'n_steps': trial.suggest_int('n_steps', 2048, 8192), # Default: 2048
        'gamma': trial.suggest_loguniform('gamma', 0.8, 0.9999), # Default: 0.99
        'learning_rate': trial.suggest_loguniform('learning_rate', 1e-6, 1e-3), # Default: 3e-4
        'clip_range': trial.suggest_uniform('clip_range', 0.1, 0.4), # Default: 0.02
        'gae_lambda': trial.suggest_uniform('gae_lambda', 0.8, 0.99), # Default: 0.95
        'ent_coef': trial.suggest_loguniform('ent_coef', 0.00000001, 0.1), # Default: 0.0
        'vf_coef': trial.suggest_uniform('vf_coef', 0, 1), # Default: 0.5
    }

def optimize_ppo_refinement(trial):
    ## This method defines a smaller range of hyperparams to search fo the best tuning
    return {
        'n_steps': trial.suggest_int('n_steps', 2048, 14336), # Default: 2048
        'gamma': trial.suggest_loguniform('gamma', 0.96, 0.9999), # Default: 0.99
        'learning_rate': trial.suggest_loguniform('learning_rate', 1e-5, 9e-4), # Default: 3e-4
        'clip_range': trial.suggest_uniform('clip_range', 0.15, 0.37), # Default: 0.02
        'gae_lambda': trial.suggest_uniform('gae_lambda', 0.94, 0.99), # Default: 0.95
        'ent_coef': trial.suggest_loguniform('ent_coef', 0.00000001, 0.00001), # Default: 0.0
        'vf_coef': trial.suggest_uniform('vf_coef', 0.55, 0.65), # Default: 0.5
    }

def optimize_agent(trial):
    # This method is used to optimize the hyperparameters for our problem
    try:
        # Create environment
        env_opt = gym.make('LimoBotEnv-v0')

        # Set up directories
        HOME_DIR = os.path.expanduser('~')
        PKG_DIR = 'ros2_ws/src/Limobot-Path-Planning/limo_robot_spawner'
        LOG_DIR = os.path.join(HOME_DIR,PKG_DIR,'logs')
        SAVE_PATH = os.path.join(HOME_DIR,PKG_DIR,'tuning','trial_{}'.format(trial.number))

        # Set up the parameters
        #  - can use optimize_ppo_refinement() after optimize_ppo to get narrower results
        model_params = optimize_ppo(trial)

        # Set up the model
        model = PPO("MultiInputPolicy",env_opt,tensorboard_log=LOG_DIR,verbose=0,**model_params)
        model.learn(total_timesteps=150000)

        # Evaluate the model
        mean_reward, _ = evaluate_policy(model,env_opt,n_eval_episodes=20)

        # Close environment and delete
        env_opt.close()
        del env_opt

        model.save(SAVE_PATH)

        return mean_reward
    
    except Exception as e:
        return -10000

if __name__ == "__main__":
    main()