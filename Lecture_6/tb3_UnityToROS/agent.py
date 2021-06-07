# -*- coding: utf-8 -*-
"""DDQN agent script 

This manages the testing phase of the off-policy DDQN.
"""

import random

import numpy as np
import tensorflow as tf

from utils.deepnetwork import DeepNetwork
from utils.robot_interface import Robot

class DDQN:
    """
    Class for the testing of the DDQN agent
    """

    def __init__(self):
        """Initialize the agent, its network and the robot interface
        """
        self.model = DeepNetwork.build()
        self.model.load_weights("results/models/TurtleBot3_DDQN_seed2_2x64_epoch157_success99.h5")
        self.robot = Robot()

    def get_action(self, state):
        """Get the action to perform

        Args:
            state (list): agent current state

        Returns:
            action (int): index of the sampled actions to perform
        """
        q_values = self.model.predict(np.array([state]))[0]
        return np.argmax(q_values)

    def round_state(self, state):
        """Round the robot state

        Args:
            state (list): state of the robot

        Returns:
            state (list): rounded state of the robot
        """

        return np.around(state, 3)

    def test(self, n_episodes):
        """Main loop for the agent's training phase

        Args:
            n_episodes (int): n° of episodes to perform

        Returns:
            None
        """

        for e in range(n_episodes):
            while True:

                # get robot state
                pos, rot = self.robot.get_odom()
                dist, heading = self.robot.get_goal_info(pos)

                if dist < 0.1:
                    self.robot.move(-1)
                    quit()
                    
                scan = self.robot.get_scan()

                state = np.concatenate(([dist, heading - rot], scan))
                state = self.round_state(state)
                
                action = self.get_action(state)
                self.robot.move(action)



   