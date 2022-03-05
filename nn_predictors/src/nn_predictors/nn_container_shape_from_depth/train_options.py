import argparse
from base_options import BaseTrainOptions

class TrainOptions(BaseTrainOptions):

    def __init__(self):
        self.parser = argparse.ArgumentParser()

        req = self.parser.add_argument_group('Required')
        req.add_argument('--name', required=True, help='Name of the experiment')

        task  = req.add_mutually_exclusive_group()
        task.add_argument('--cross_section', dest='task', action='store_const', const='cross_section')
        task.add_argument('--volume_profile', dest='task', action='store_const', const='volume_profile')
        task.add_argument('--wait_times', dest='task', action='store_const', const='wait_times')
        req.set_defaults(task='cross_section')

        source = req.add_mutually_exclusive_group()
        source.add_argument('--from_depth', dest='source', action='store_const', const='from_depth')
        source.add_argument('--from_cross_section', dest='source', action='store_const', const='from_cross_section')
        req.set_defaults(source='from_depth')

        gen = self.parser.add_argument_group('General')
        gen.add_argument('--time_to_run', type=int, default=3600, help='Total time to run in seconds')
        gen.add_argument('--resume', dest='resume', default=False, action='store_true', help='Resume from checkpoint (Use latest checkpoint by default')
        gen.add_argument('--num_workers', type=int, default=4, help='Number of processes used for data loading')
        pin = gen.add_mutually_exclusive_group()
        pin.add_argument('--pin_memory', dest='pin_memory', action='store_true', help='Number of processes used for data loading')
        pin.add_argument('--no_pin_memory', dest='pin_memory', action='store_false', help='Number of processes used for data loading')
        gen.set_defaults(pin_memory=True)


        io = self.parser.add_argument_group('io')
        io.add_argument('--dataset_dir', default='/NAS/home/nn_container_shape_from_depth/data', help='Path to the desired dataset')
        io.add_argument('--volume_dir', default='/NAS/home/nn_container_shape_from_depth/volume_profiles', help='Path to the volume profiles')
        io.add_argument('--log_dir', default='../logs', help='Directory to store logs')
        io.add_argument('--checkpoint', default=None, help='Path to checkpoint')
        io.add_argument('--from_json', default=None, help='Load options from json file instead of the command line')

        train = self.parser.add_argument_group('Training Options')
        train.add_argument('--center', dest='center', default=False, action='store_true', help='center the data')
        train.add_argument('--num_epochs', type=int, default=100, help='Total number of training epochs')
        train.add_argument('--batch_size', type=int, default=6, help='Batch size')
        train.add_argument('--test_batch_size', type=int, default=8, help='Batch size')
        shuffle_train = train.add_mutually_exclusive_group()
        shuffle_train.add_argument('--shuffle_train', dest='shuffle_train', action='store_true', help='Shuffle training data')
        shuffle_train.add_argument('--no_shuffle_train', dest='shuffle_train', action='store_false', help='Don\'t shuffle training data')
        shuffle_test = train.add_mutually_exclusive_group()
        shuffle_test.add_argument('--shuffle_test', dest='shuffle_test', action='store_true', help='Shuffle testing data')
        shuffle_test.add_argument('--no_shuffle_test', dest='shuffle_test', action='store_false', help='Don\'t shuffle testing data')
        train.set_defaults(shuffle_train=True, shuffle_test=True)
        train.add_argument('--summary_steps', type=int, default=100, help='Summary saving frequency')
        train.add_argument('--checkpoint_steps', type=int, default=10000, help='Chekpoint saving frequency')
        train.add_argument('--test_steps', type=int, default=500, help='Testing frequency')
        train.add_argument('--test_iters', type=int, default=200, help='Number of testing iterations')
        train.add_argument('--add_noise_speed_angle', type=float, default=0.0, help='Add noise to speed/angle')
        train.add_argument('--add_noise_cross_section', type=float, default=0.0, help='Add noise to cross section')
        train.add_argument('--threshold_fraction', type=float, default=0.75, help='Threshold fraction for calculating wait times')
        train.add_argument('--add_one', type=bool, default=True, help='Adds one to the output of the network')
        train.add_argument('--seperate_height', type=bool, default=False, help='Seperates the heights into their own tensor before passing it to the network')


        optim = self.parser.add_argument_group('Optimization')
        optim_type = optim.add_mutually_exclusive_group()
        optim_type.add_argument('--use_sgd', dest='optimizer', action='store_const', const='sgd',help='Use SGD (default Adam)')
        optim_type.add_argument('--use_rmsprop', dest='optimizer', action='store_const', const='rmsprop',help='Use  (default Adam)')
        optim_type.add_argument('--use_adam', dest='optimizer', action='store_const', const='adam',help='Use SGD (default Adam)')
        optim.add_argument('--adam_beta1', type=float, default=0.9, help='Value for Adam Beta 1')
        optim.add_argument('--sgd_momentum', type=float, default=0.0, help='Momentum for SGD')
        optim.add_argument("--lr", type=float, default=2.5e-4, help="Learning rate")
        optim.add_argument("--lr_decay", type=float, default=0.98, help="Exponential decay rate")
        optim.add_argument("--wd", type=float, default=0, help="Weight decay weight")
        optim.set_defaults(optimizer='rmsprop')

        loss_type = optim.add_mutually_exclusive_group()
        loss_type.add_argument('--mse', dest='loss', action='store_const', const='mse', help='Use MSE Loss')
        loss_type.add_argument('--l1', dest='loss', action='store_const', const='L1', help='Use L1 Loss')
        loss_type.add_argument('--smoothl1', dest='loss', action='store_const', const='SmoothL1', help='Use SmoothL1 Loss')

        arch = self.parser.add_argument_group('Architecture')
        arch.add_argument("--image_size", type=int, default=256, help="Scales all images to this size in pixels")
        arch.add_argument("--num_output_channels", type=int, default=128, help="Number of output channels")
        arch.add_argument("--num_hidden_channels", type=int, default=1024, help="Number of hidden channels")
        arch.add_argument("--num_hidden_layers", type=int, default=3, help="Number of hidden layers")
        arch.add_argument("--num_horz_divs", type=int, default=128, help="Number of horizontal divisions to approximate the container with")
        arch.add_argument("--dropout", type=float, default=0.0, help="Amount of dropout")
        arch.add_argument("--batch_norm", type=bool, default=False, help="If batch norm will be used")
        arch.add_argument("--num_init_conv_layers", type=int, default=0, help="The number of conv layers to start the fully connected network with")
        arch.add_argument("--kernel_size", type=int, default=7, help="The kernel size for the conv layers")

        nonlinearity = arch.add_mutually_exclusive_group()
        nonlinearity.add_argument('--use_LeakyReLU', dest='nonlinearity', action='store_const', const='LeakyReLU',help='Use LeakyReLU')
        nonlinearity.add_argument('--use_ReLU', dest='nonlinearity', action='store_const', const='ReLU',help='Use  ReLU')
        nonlinearity.set_defaults(nonlinearity='ReLU')
