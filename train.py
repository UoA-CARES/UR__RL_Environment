import time
import os
import json
import urx
import logging
logging.basicConfig(level=logging.INFO)
from environment.main_environment_ur5 import Environment

# Need this for math3d lib issues
import collections
collections.Iterable = collections.abc.Iterable


def read_config_file():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, 'config', 'robot_config.json')
    with open(config_path) as f:
        config = json.load(f)
    return config


def read_camera_files():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, 'config')
    camera_matrix = config_path + '/matrix.txt'
    camera_distortion = config_path + '/distortion.txt'
    return camera_matrix, camera_distortion


def training_example_loop(env):
    # to fix later
    max_steps_training = 1000

    episode_timesteps = 0
    episode_reward = 0
    episode_num = 0

    state = env.environment_reset()

    for total_step_counter in range(int(max_steps_training)):
        episode_timesteps += 1

        sample_action = env.get_sample_pose()  # random action

        next_state, reward, done, truncated = env.step(sample_action)
        logging.info("State: %s", state)

        state = next_state


        episode_reward += reward

        if done or truncated:
            logging.info(f"Episode ends.\n"
                         f"Episode reward: {episode_reward}.\n"
                         f"Episode steps: {episode_timesteps}.\n")
            state = env.environment_reset()
            episode_timesteps = 0
            episode_reward = 0
            episode_num += 1


def main():
    config = read_config_file()
    camera_matrix, camera_distortion = read_camera_files()
    ip_address_robot = config['ip_robot']

    robot = urx.Robot(ip_address_robot)
    # robot = urx.Robot(ip_address_robot, use_rt=True, urFirm=config['urFirm'])

    env = Environment(robot,
                      velocity=config['velocity'],
                      acceleration=config['acceleration'],
                      camera_matrix=camera_matrix,
                      camera_distortion=camera_distortion)


    env.starting_position() # making sure the joint are in the right position for initialization purely joints position
    env.robot_home_position() # put the robot at home position
    training_example_loop(env)
    robot.close()


if __name__ == "__main__":
    main()


