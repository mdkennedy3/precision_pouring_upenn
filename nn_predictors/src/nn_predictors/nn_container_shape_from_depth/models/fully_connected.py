import torch.nn as nn
import torch
import numpy as np
import torch.nn.functional as F
from collections import OrderedDict
from helpers import get_nonlinearity

class FullyConnected(nn.Module):
    def __init__(self,
                 num_input_channels=128,
                 num_output_channels=128,
                 num_hidden_channels=524,
                 num_hidden_layers=10,
                 dropout_prob=0.0,
                 use_batch_norm=False,
                 use_speed_and_angle=False,
                 num_init_conv_layers=0,
                 kernel_size=7,
                 add_one=True,
                 seperate_height=False,
                 nonlinearity='ReLU'):
        super(FullyConnected, self).__init__()
        self.add_one = add_one
        self.seperate_height = seperate_height

        if not type(num_hidden_channels) is list:
            num_hidden_channels = [num_hidden_channels] * (num_hidden_layers + 1)

        if seperate_height:
            num_input_channels -= 1
        layers = OrderedDict()
        if num_init_conv_layers == 0:
            if use_speed_and_angle:
                num_input_channels += 2

            layers['input'] = nn.Linear(num_input_channels, num_hidden_channels[0])
            self.conv_network = None
        else:
            conv_layers = OrderedDict()
            for i in range(num_init_conv_layers):
                conv_layers['conv'+str(i)] = nn.Conv1d(i * 8 + 1, (i+1) * 8 + 1, kernel_size)
                conv_layers[nonlinearity + '_' + str(i)] = get_nonlinearity(nonlinearity)
            self.conv_network = nn.Sequential(conv_layers)
            print ((i+1)*8+1), (num_input_channels -(i+1)* (kernel_size-1))
            num_additional_inputs = 0
            if use_speed_and_angle:
                num_additional_inputs += 2
            if seperate_height:
                num_additional_inputs += 1
            layers['input'] = nn.Linear(((i+1)*8+1) * (num_input_channels -(i+1)* (kernel_size-1)) + num_additional_inputs, num_hidden_channels[0])



        for i in range(num_hidden_layers):
            layers[nonlinearity + '_' + str(i)] = get_nonlinearity(nonlinearity)
            layers['dropout' + str(i)] = nn.Dropout(p=dropout_prob)
            layers['linear' + str(i)] = nn.Linear(num_hidden_channels[i], num_hidden_channels[i+1])
            if use_batch_norm:
                layers['norm' + str(i)] = nn.BatchNorm1d(num_hidden_channels)

        layers['final_' + nonlinearity] = get_nonlinearity(nonlinearity)
        layers['output'] = nn.Linear(num_hidden_channels[num_hidden_layers], num_output_channels)
        self.network = nn.Sequential(layers)

    def forward(self, x):
        if self.conv_network is None:
            out = self.network(x)
        else:
            raise NotImplementedException("Forward without speed and angle is not yet implemented for conv")

        if self.add_one:
            out = out + 1

    def forward_unseperated_height(self, x, speed, angle):
        if self.conv_network is None:
            x = torch.cat((x, speed.view(speed.shape[0], 1), angle.view(angle.shape[0], 1)), dim=1)
            return self.network(x)
        else:
            x = x.view(x.shape[0], 1, -1)
            x = self.conv_network(x)
            x = x.view(x.shape[0], -1)
            x = torch.cat((x, speed.view(speed.shape[0], 1), angle.view(angle.shape[0], 1)), dim=1)
            out = self.network(x)

        if self.add_one:
            out = out + 1
        return out

    def forward(self, x, speed, angle, height):
        if self.conv_network is None:
            x = torch.cat((x, speed.view(speed.shape[0], 1), angle.view(angle.shape[0], 1)), dim=1)
            return self.network(x)
        else:
            if not self.seperate_height:
                return self.forward_unseperated_height(x, speed, angle)

            x = x.view(x.shape[0], 1, -1)
            x = self.conv_network(x)
            x = x.view(x.shape[0], -1)
            x = torch.cat((x, speed.view(speed.shape[0], 1), angle.view(angle.shape[0], 1), height.view(height.shape[0], 1)), dim=1)
            out = self.network(x)

        if self.add_one:
            out = out + 1
        return out

