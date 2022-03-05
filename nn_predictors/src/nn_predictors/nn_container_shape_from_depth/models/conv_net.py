import torch.nn as nn
import numpy as np
import math
import torch
from collections import OrderedDict
from helpers import get_nonlinearity

class ConvNet(nn.Module):
    def __init__(self,
                 input_image_size=128,
                 num_output_channels=128, 
                 num_hidden_channels=1024,
                 max_pooling_kernel=2,
                 conv_kernel=5,
                 num_linear_layers=3,
                 dropout_prob=0.0,
                 use_speed_and_angle=False,
                 nonlinearity='ReLU'):

        super(ConvNet, self).__init__()
        self.num_hidden_channels = num_hidden_channels
        self.use_speed_and_angle = use_speed_and_angle

        conv_layers = OrderedDict()
        conv_layers['conv_0'] = nn.Conv2d(1, 64, 3, padding=1)
        conv_layers['norm_0'] = nn.BatchNorm2d(64)
        conv_layers['conv_relu_0'] = nn.ReLU()
        conv_layers['conv_1'] = nn.Conv2d(64, 64, 3, padding=1)
        conv_layers['norm_1'] = nn.BatchNorm2d(64)
        conv_layers['conv_relu_1'] = nn.ReLU()
        conv_layers['pool_1'] = nn.MaxPool2d(3)

        conv_layers['conv_2'] = nn.Conv2d(64, 128, 3, padding=1)
        conv_layers['norm_2'] = nn.BatchNorm2d(128)
        conv_layers['conv_relu_2'] = nn.ReLU()
        conv_layers['conv_3'] = nn.Conv2d(128, 128, 3, padding=1)
        conv_layers['norm_3'] = nn.BatchNorm2d(128)
        conv_layers['conv_relu_3'] = nn.ReLU()
        conv_layers['pool_2'] = nn.MaxPool2d(3)

        conv_layers['conv_3'] = nn.Conv2d(128, 256, 3, padding=1)
        conv_layers['norm_3'] = nn.BatchNorm2d(256)
        conv_layers['conv_relu_3'] = nn.ReLU()
        conv_layers['conv_4'] = nn.Conv2d(256, 256, 3, padding=1)
        conv_layers['norm_4'] = nn.BatchNorm2d(256)
        conv_layers['conv_relu_4'] = nn.ReLU()
        conv_layers['conv_5'] = nn.Conv2d(256, 256, 3, padding=1)
        conv_layers['norm_5'] = nn.BatchNorm2d(256)
        conv_layers['conv_relu_5'] = nn.ReLU()
        conv_layers['conv_6'] = nn.Conv2d(256, 256, 3, padding=1)
        conv_layers['norm_6'] = nn.BatchNorm2d(256)
        conv_layers['conv_relu_6'] = nn.ReLU()
        conv_layers['pool_3'] = nn.MaxPool2d(3)

        conv_layers['conv_7'] = nn.Conv2d(256, 512, 3, padding=1)
        conv_layers['norm_7'] = nn.BatchNorm2d(512)
        conv_layers['conv_relu_7'] = nn.ReLU()
        conv_layers['conv_8'] = nn.Conv2d(512, 512, 3, padding=1)
        conv_layers['norm_8'] = nn.BatchNorm2d(512)
        conv_layers['conv_relu_8'] = nn.ReLU()
        conv_layers['conv_9'] = nn.Conv2d(512, 512, 3, padding=1)
        conv_layers['norm_9'] = nn.BatchNorm2d(512)
        conv_layers['conv_relu_9'] = nn.ReLU()
        conv_layers['conv_10'] = nn.Conv2d(512, 512, 3, padding=1)
        conv_layers['norm_10'] = nn.BatchNorm2d(512)
        conv_layers['conv_relu_10'] = nn.ReLU()
        conv_layers['pool_4'] = nn.MaxPool2d(3)

        conv_layers['conv_11'] = nn.Conv2d(512, 512, 3, padding=1)
        conv_layers['norm_11'] = nn.BatchNorm2d(512)
        conv_layers['conv_relu_11'] = nn.ReLU()
        conv_layers['conv_12'] = nn.Conv2d(512, 512, 3, padding=1)
        conv_layers['norm_12'] = nn.BatchNorm2d(512)
        conv_layers['conv_relu_12'] = nn.ReLU()
        conv_layers['conv_13'] = nn.Conv2d(512, 512, 3, padding=1)
        conv_layers['norm_13'] = nn.BatchNorm2d(512)
        conv_layers['conv_relu_13'] = nn.ReLU()
        conv_layers['conv_14'] = nn.Conv2d(512, 512, 3, padding=1)
        conv_layers['norm_14'] = nn.BatchNorm2d(512)
        conv_layers['conv_relu_14'] = nn.ReLU()
        conv_layers['pool_5'] = nn.MaxPool2d(3)




        
        #conv_layers['conv_0'] = nn.Conv2d(1, num_hidden_channels/16, conv_kernel, padding=(conv_kernel-1)/2)
        #conv_layers['pooling_0'] = nn.MaxPool2d(max_pooling_kernel)
        #num_conv_layers = int(math.ceil(math.log(input_image_size, 2))) - 2

        #for i in range(1, num_conv_layers):
        #    conv_layers[nonlinearity + '_conv_0_' + str(i)] = get_nonlinearity(nonlinearity)
        #    #conv_layers['dropout_conv_' + str(i)] = nn.Dropout(p=dropout_prob)
        #    conv_layers['conv_0_' + str(i)] = nn.Conv2d(num_hidden_channels/16, num_hidden_channels/16, conv_kernel, padding=(conv_kernel-1)/2)
        #    conv_layers[nonlinearity + '_conv_1_' + str(i)] = get_nonlinearity(nonlinearity)
        #    #conv_layers['dropout_conv_' + str(i)] = nn.Dropout(p=dropout_prob)
        #    conv_layers['conv_1_' + str(i)] = nn.Conv2d(num_hidden_channels/16, num_hidden_channels/16, conv_kernel, padding=(conv_kernel-1)/2)
        #    conv_layers[nonlinearity + '_conv_2_' + str(i)] = get_nonlinearity(nonlinearity)
        #    #conv_layers['dropout_conv_' + str(i)] = nn.Dropout(p=dropout_prob)
        #    conv_layers['conv_2_' + str(i)] = nn.Conv2d(num_hidden_channels/16, num_hidden_channels/16, conv_kernel, padding=(conv_kernel-1)/2)

        #    conv_layers['pooling_' + str(i)] = nn.MaxPool2d(max_pooling_kernel)
        self.conv_network = nn.Sequential(conv_layers)
        self.conv_layers = conv_layers

        if use_speed_and_angle:
            num_hidden_channels += 2

        linear_layers = OrderedDict()
        for i in range(num_linear_layers):
            linear_layers[nonlinearity + '_linear_' + str(i)] = get_nonlinearity(nonlinearity)
            
            #linear_layers['dropout_linear_' + str(i)] = nn.Dropout(p=dropout_prob)
            linear_layers['linear_' + str(i)] = nn.Linear(num_hidden_channels, num_hidden_channels)

        linear_layers[nonlinearity + '_final'] = get_nonlinearity(nonlinearity)
        linear_layers['output'] = nn.Linear(num_hidden_channels, num_output_channels)
        self.fully_connected_network = nn.Sequential(linear_layers)



    def forward(self, x):
        if use_speed_and_angle:
            raise NotImplementedError('When use_speed_and_angle is true, network requires speed and angle')

        #print "input", x.shape
        #x = self.conv_network(x)
        for layer in self.conv_layers:
            #print "Layer", layer
            x = self.conv_layers[layer](x)
            #print "x", x.shape
            #print x
            #from IPython.core.debugger import Pdb
            #Pdb().set_trace()

        x = x.view(-1, x.shape[1] * x.shape[2] * x.shape[3])
        #print x.shape
        x = self.fully_connected_network(x)

        return x + 1.0


    def forward(self, x, speed, angle):
        #print "Input:", x.shape, speed.shape, angle.shape
        #print speed
        #print angle
        #x = self.conv_network(x)
        for layer in self.conv_layers:
            #print "layer:", layer
            x = self.conv_layers[layer](x)
            #print "x", x.shape
        x = x.view(-1, x.shape[1] * x.shape[2] * x.shape[3])

        #print "Before", x.shape, speed.shape, angle.shape
        x = torch.cat((x, speed.view(speed.shape[0], 1), angle.view(angle.shape[0], 1)), dim=1)
        #print "After", x.shape

        x = self.fully_connected_network(x)
        return x + 1.0
        


