import numpy as np
from torch.utils.data import Dataset
import os
from enum import Enum
import torch
import cv2
from sets import Set
class Shape(Enum):
    STRAIGHT=0
    FLASK=1
    SINUSOID=2



class PouringDataset(Dataset):
    def __init__(self, 
                 root_dir,
                 is_train,
                 load_volume=False,
                 load_depth_image=False,
                 volume_dir=None,
                 num_divisions=128,
                 image_size=128,
                 center=False,
                 add_noise_speed_angle=0.0,
                 add_noise_cross_section=0.0,
                 calc_wait_times=False,
                 threshold_fraction=0.75,
                 load_speed_angle_and_scale=False):

        self.num_divisions = num_divisions
        self.root_dir = root_dir
        self.center = center
        self.image_size = image_size
        self.load_volume = load_volume
        self.load_depth_image = load_depth_image
        self.volume_dir = volume_dir
        self.calc_wait_times = calc_wait_times
        self.load_speed_angle_and_scale = load_speed_angle_and_scale
        self.add_noise_speed_angle = 0.0
        self.add_noise_cross_section = 0.0
        self.threshold_fraction = threshold_fraction
        if is_train:
            self.add_noise_speed_angle = add_noise_speed_angle
            self.add_noise_cross_section = add_noise_cross_section

        if is_train:
            file_path = os.path.join(root_dir, "train.txt")
        else:
            file_path = os.path.join(root_dir, "test.txt")

        self.bad_set = Set([])
        self.bad_file = open("bad_file.txt", "r")
        for line in self.bad_file:
            self.bad_set.add(line.rstrip())
        print self.bad_set
        self.bad_file = open("bad_file.txt", "a")

        with open(file_path, 'r') as data_file:
            count = 0
            self.files = []
            for line in data_file:
                count += 1
                line = line.rstrip()
                if line not in self.bad_set:
                    self.files.append(line)
        print len(self.files), count

    def __len__(self):
        return len(self.files)


    def _get_container_profile(self, cfg_file_path):
        with open(cfg_file_path, 'r') as cfg_file:
            lines = cfg_file.read().splitlines()

            line_count = len(lines)

            if line_count == 6:
                shape = Shape.STRAIGHT
            elif line_count == 7:
                shape = Shape.FLASK
            elif line_count == 8: #Sinusoidal
                shape = Shape.SINUSOID
            else:
                print "Invalid cfg file format"

            neck_radius = 0
            base_radius = 0
            a = 0
            b = 0
            c = 0
            d = 0
            height = float(lines[2])
            for line in lines:
                split = line.split()
                # print split
                if split[0] == 'neck_radius:':
                    neck_radius = float(split[1])
                elif split[0] == 'base_radius:':
                    base_radius = float(split[1])
                elif len(split) == 9 and split[7] == 'da:':
                    a = float(split[8])
                elif split[0] == 'b:':
                    b = float(split[1])
                elif split[0] == 'c:':
                    c = float(split[1])
                elif split[0] == 'd:':
                    d = float(split[1])
                elif split[0] == 'neck_height:':
                    neck_height = float(split[1])
                elif split[0] == 'Radius':
                    # Hack because the flask cfgs were not correctly formatted
                    base_radius = float(split[-1])

            profile = np.zeros(self.num_divisions)

            if shape == Shape.STRAIGHT:
                profile = np.linspace(base_radius, neck_radius, self.num_divisions)

            elif shape == Shape.SINUSOID:
                profile = a * np.sin( b * np.linspace(0, height, self.num_divisions) + c) + d

            elif shape == Shape.FLASK:
                split = int(self.num_divisions * (1 - neck_height))
                profile[:split] = np.linspace(base_radius, neck_radius, split)
                profile[split:] = np.linspace(neck_radius, neck_radius, self.num_divisions - split)
            else:
                print "Invalid cfg file format"
            return profile, height

    def _get_volume_profile(self, data):
        
        max_volume = data[0, 1]
        # Flip volume to make it the amount in the receiving container
        # instead of the amount in the pouring container
        volumes = (max_volume - data[:, 0]) / 100000.0

        step_size = len(volumes) / self.num_divisions
        volume_profile = np.zeros(self.num_divisions)

        for i in range(self.num_divisions):
            volume_profile[i] = np.mean(volumes[i * step_size:(i+1)*step_size])

        return volume_profile

    def _find_wait_time(self, volume_data, start_index, i, threshold_fraction):
        start_vol = volume_data[start_index, 0]
        end_vol = volume_data[i-1, 0]

        threshold = start_vol - threshold_fraction * (start_vol - end_vol)

        for j in range(start_index, i):
            if volume_data[j, 0] < threshold:
                return volume_data[j, 3] - volume_data[start_index, 3]
        return volume_data[i-1, 3] - volume_data[start_index, 3]

    def _calc_wait_times(self, volume_data, threshold_fraction=0.75):
        wait_times = []
        start_index = -1

        for i in range(1, len(volume_data)):
            # If you find the start of a pause
            if start_index == -1 and volume_data[i, 2] == volume_data[i-1, 2]:
                start_index = i-1

            # If you found the end of a pause
            elif start_index != -1 and volume_data[i, 2] != volume_data[i-1, 2]:
                # Only counts the wait time if there was a significant difference
                # between the starting and the ending volumes
                if volume_data[start_index, 0] > volume_data[i-1, 0] + 10:
                    wait_times.append(self._find_wait_time(volume_data, start_index, i, threshold_fraction))
                start_index = -1


        if start_index != -1:
            wait_times.append(self._find_wait_time(volume_data, start_index, i, threshold_fraction))       
        bad = False
        # Sets ground truth to zero when container is empty
        if len(wait_times) == 0:
            wait_times.append(0)
        if volume_data[start_index, 0]- volume_data[i, 0] < 10:
            wait_times[0] = 0.0
            bad = True
        return torch.tensor(wait_times[0]).to(torch.float), bad


    def _load_params(self, file_name):
        file_path = os.path.join(self.root_dir, 'params', file_name.split('.')[0] + '_params.txt')

        with open(file_path, 'r') as param_file:
            for line in param_file.readlines():
                split = line.split(' ')
                if split[0] == 'rotation_speed':
                    speed = float(split[1])
                elif split[0] == 'stop_angle':
                    angle = float(split[1])
                elif split[0] == 'scaling_factor':
                    scaling_factor = float(split[1])
        return speed, angle, scaling_factor


    def __getitem__(self, idx):

        #file_name = '_'.join(self.files[idx].split('_')[:-1])
        file_name = self.files[idx].split('i')[0]
        cfg_file_path = os.path.join(self.root_dir, 'cfg_files', file_name + ".cfg")
        profile, height = self._get_container_profile(cfg_file_path)

        for i in range(len(profile)):
            profile[i] *= (1.0 - self.add_noise_cross_section + 
                           2 * self.add_noise_cross_section * np.random.random())
        #print self.files[idx]
        sample = {'cross_section_profile': profile,
                  'height': height, 
                  'name': torch.tensor(float(self.files[idx][14:-4].replace('iteration', '').split('_')[0]))}
        if self.load_depth_image:
            depth_image_path = os.path.join(self.root_dir, 'depth_images', self.files[idx])
            depth_image = cv2.imread(depth_image_path, cv2.IMREAD_GRAYSCALE)
            depth_image = cv2.resize(depth_image, (self.image_size, self.image_size))
            depth_image = 1.0 - depth_image.astype(float) / 255.0
            sample['depth_image'] = depth_image

        if self.load_volume:
            #volume_profile_path = os.path.join(self.volume_dir, file_name + ".text")
            volume_profile_path = os.path.join(self.volume_dir, self.files[idx].split('.')[0]+".text")
            volume_data = np.loadtxt(volume_profile_path)
            volume_profile = self._get_volume_profile(volume_data)
            sample['volume_profile'] = volume_profile

            if self.calc_wait_times:
                sample['wait_times'], bad = self._calc_wait_times(volume_data, self.threshold_fraction)
                if bad:
                    if(self.files[idx]) not in self.bad_set:
                        print self.files[idx]
                        self.bad_set.add(self.files[idx])
                        self.bad_file.write(self.files[idx] + "\n")
                        self.bad_file.flush()

        if self.load_speed_angle_and_scale:
            sample['speed'], sample['angle'], scale = self._load_params(self.files[idx])
            
            sample['speed'] *= (1.0 - self.add_noise_speed_angle +
                                2 * self.add_noise_speed_angle * np.random.random())
            sample['angle'] *= (1.0 - self.add_noise_speed_angle +
                                2 * self.add_noise_speed_angle * np.random.random())


        return sample

