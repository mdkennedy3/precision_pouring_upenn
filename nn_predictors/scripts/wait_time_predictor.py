#!/usr/bin/env python
import rospy
import os
import rospkg
import torch

from nn_predictors.srv import GetWaitTime, GetWaitTimeRequest, GetWaitTimeResponse
from pouring_msgs.msg import ContainerEdgeProfile, PouringTimeDelay, PourAngStateMsg

#from nn_container_shape_from_depth.models import FullyConnected
from nn_predictors.nn_container_shape_from_depth.models import FullyConnected


class WaitTimePredictor(object):
    def __init__(self):
        rospy.loginfo("Starting node")
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

        self.num_input_channels = rospy.get_param("num_input_channels", 128)
        self.cross_section = torch.Tensor([[0]])
        self.num_output_channels = rospy.get_param("num_output_channels", 1)
        num_hidden_layers = rospy.get_param("num_hidden_layers", 10)
        num_hidden_channels = rospy.get_param("num_hidden_channels", 512)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('nn_predictors')
        model_file = rospy.get_param("model_file", os.path.join(package_path, 'models', 'new_best_10x512.pt'))
        model_name = rospy.get_param("model_name", "stacked_hg")

        # For 10x512
        # self.model = FullyConnected(num_input_channels=self.num_input_channels +1, # Add one for height
        #                             num_output_channels=self.num_output_channels,
        #                             num_hidden_channels=num_hidden_channels,
        #                             num_hidden_layers=num_hidden_layers,
        #                             dropout_prob=0.0,
        #                             use_batch_norm=False,
        #                             add_one=True,
        #                             use_speed_and_angle=True,
        #                             nonlinearity='ReLU').to(self.device)
        # self.seperate_height = False


        # For 10x64_conv3x15
        model_file = os.path.join(package_path, 'models', 'best_10x64_conv3x15.pt')
        self.model = FullyConnected(num_input_channels=self.num_input_channels+1, # Plus 1 for height
                                    num_output_channels=1,
                                    num_hidden_channels=64,
                                    num_hidden_layers=10,
                                    dropout_prob=0.0,
                                    use_batch_norm=False,
                                    use_speed_and_angle=True,
                                    num_init_conv_layers=3,
                                    kernel_size=15,
                                    add_one=False,
                                    seperate_height=False,
                                    nonlinearity='ReLU').to(self.device)
        self.seperate_height = False


        # model_file = os.path.join(package_path, 'models', '5x64_conv3x15_thresh75.pt')
        # model_file = os.path.join(package_path, 'models', 'wait_times_5x64_lr00005_decay9999_nsa05_ncs0_batch_256_dr1_threshold90_final_3_noone_conv3_sepheight_nobad.pt')
        # self.model = FullyConnected(num_input_channels=self.num_input_channels+1, # Plus 1 for height
        #                             num_output_channels=1,
        #                             num_hidden_channels=64,
        #                             num_hidden_layers=5,
        #                             dropout_prob=0.0,
        #                             use_batch_norm=False,
        #                             use_speed_and_angle=True,
        #                             num_init_conv_layers=3,
        #                             kernel_size=15,
        #                             add_one=False,
        #                             seperate_height=True,
        #                             nonlinearity='ReLU').to(self.device)
        # self.seperate_height = True


        # model_file = os.path.join(package_path, 'models', 'wait_times_10x64_lr00005_decay9999_nsa05_ncs0_batch_12_dr1_threshold90_final_2_noone_conv3_sepheight_nobad.pt')
        # self.model = FullyConnected(num_input_channels=self.num_input_channels+1, # Plus 1 for height
        #                             num_output_channels=1,
        #                             num_hidden_channels=64,
        #                             num_hidden_layers=10,
        #                             dropout_prob=0.0,
        #                             use_batch_norm=False,
        #                             use_speed_and_angle=True,
        #                             num_init_conv_layers=3,
        #                             kernel_size=15,
        #                             add_one=False,
        #                             seperate_height=True,
        #                             nonlinearity='ReLU').to(self.device)
        # self.seperate_height = True


        # model_file = os.path.join(package_path, 'models', 'wait_times_5x64_lr00005_decay9999_nsa05_ncs1_batch_12_dr1_threshold75_final_3_noone_conv2_sepheight.pt')
        # self.model = FullyConnected(num_input_channels=self.num_input_channels+1, # Plus 1 for height
        #                             num_output_channels=1,
        #                             num_hidden_channels=64,
        #                             num_hidden_layers=5,
        #                             dropout_prob=0.0,
        #                             use_batch_norm=False,
        #                             use_speed_and_angle=True,
        #                             num_init_conv_layers=2,
        #                             kernel_size=15,
        #                             add_one=False,
        #                             seperate_height=True,
        #                             nonlinearity='ReLU').to(self.device)
        # self.seperate_height = True


        # model_file = os.path.join(package_path, 'models', 'wait_times_10x512_lr00005_decay9999_nsa05_ncs0_batch_256_dr1_threshold90_final_3_noone_conv3_sepheight_nobad.pt')
        # self.model = FullyConnected(num_input_channels=self.num_input_channels+1, # Plus 1 for height
        #                             num_output_channels=1,
        #                             num_hidden_channels=512,
        #                             num_hidden_layers=10,
        #                             dropout_prob=0.0,
        #                             use_batch_norm=False,
        #                             use_speed_and_angle=True,
        #                             num_init_conv_layers=3,
        #                             kernel_size=15,
        #                             add_one=True,
        #                             seperate_height=True,
        #                             nonlinearity='ReLU').to(self.device)
        # self.seperate_height = True



        checkpoint = torch.load(model_file, map_location=self.device)
        self.model.load_state_dict(checkpoint[model_name])


        self.model.eval()

        self.angle_sub = rospy.Subscriber('/pouring_control/pour_angle_state', PourAngStateMsg, self.get_wait_time)
        rospy.Subscriber('/pouring_control/container_edge_profile_pub_msg', ContainerEdgeProfile, self.update_volume_profile)
        self.wait_time_pub = rospy.Publisher('time_delay_msg', PouringTimeDelay, queue_size=1)

        rospy.loginfo("Spinning")
        rospy.spin()

    def get_wait_time(self, msg):
        msg.omega = max(0.0, msg.omega)
        speed = torch.FloatTensor([msg.omega]).to(self.device)
        # speed = torch.FloatTensor([0.02]).to(self.device)
        angle = torch.FloatTensor([msg.theta * 180 / 3.14159]).to(self.device)
        delay_msg = PouringTimeDelay()
        delay_msg.header.stamp = rospy.Time.now()
        if len(self.cross_section[0]) < self.num_input_channels:
            rospy.logwarn("Trying to get wait time before setting the profile")
            delay_msg.time_delay = 0
            self.wait_time_pub.publish(delay_msg)
            return

        with torch.no_grad():
            if self.seperate_height:
                wait_time = self.model(self.cross_section, speed, angle, self.height)
            else:
                wait_time = self.model(self.cross_section, speed, angle, 0)

        delay_msg.time_delay = float(wait_time.to('cpu'))
        self.wait_time_pub.publish(delay_msg)

    def update_volume_profile(self, msg):
        profile = list(msg.radius)
        if not self.seperate_height:
            profile.append(msg.container_height_m)
        else:
            self.height = torch.FloatTensor([msg.container_height_m]).to(self.device)
        self.cross_section = torch.Tensor([profile]).to(torch.float).to(self.device)
        print self.cross_section







if __name__ == '__main__':
    rospy.init_node('wait_time_predictor')
    WaitTimePredictor()
