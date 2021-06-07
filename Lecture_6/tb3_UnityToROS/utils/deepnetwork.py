# -*- coding: utf-8 -*-
"""DNN builder script

This manages the DNN creation and printing for the agent
"""

import tensorflow as tf
from tensorflow.keras.layers import Input, Dense
from tensorflow.keras.models import Model
from tensorflow.keras.utils import plot_model

class DeepNetwork:
    """
    Class for the DNN creation of both the actor and the critic
    """

    @staticmethod  
    def build():
        """Gets the DNN architecture and build it

        Args:
            env (gym): the gym env to take agent I/O
            params (dict): n° and size of hidden layers, print model for debug
            name (str): file name for the model

        Returns:
            model: the uncompiled DNN Model
        """

        input_size = 13
        action_size = 5

        state_input = Input(shape=(input_size,), name='input_layer')
        h = state_input

        h_size = 64
        h_layers = 2

        for i in range(h_layers):
            h = Dense(h_size, activation='relu', name='hidden_' + str(i))(h)

        y = Dense(action_size, activation='linear', name='output_layer')(h)
        
        model = Model(inputs=state_input, outputs=y)

        return model
