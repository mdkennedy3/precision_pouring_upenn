import torch
import torch.nn as nn
import numpy as np
import cv2
from torch.utils.data import DataLoader
from torchvision.utils import make_grid

from data_loader import PouringDataset
from base_trainer import BaseTrainer

from tqdm import tqdm
tqdm.monitor_interval = 0

from models import ConvNet
from models import FullyConnected

class ShapeTrainer(BaseTrainer):

    def _init_fn(self):

        if self.options.task=='wait_times':
            self.use_speed_and_angle = True
        else:
            self.use_speed_and_angle = False


        if self.options.source == 'from_depth':
            self.model = ConvNet(input_image_size=self.options.image_size,
                                 num_output_channels=self.options.num_output_channels,
                                 num_hidden_channels=self.options.num_hidden_channels,
                                 num_linear_layers=self.options.num_hidden_layers,
                                 dropout_prob=self.options.dropout,
                                 use_speed_and_angle=self.use_speed_and_angle,
                                 nonlinearity=self.options.nonlinearity).to(self.device)
        elif self.options.source == 'from_cross_section':
            self.model = FullyConnected(num_input_channels=self.options.num_horz_divs+1, # Plus 1 for height
                                        num_output_channels=self.options.num_output_channels,
                                        num_hidden_channels=self.options.num_hidden_channels,
                                        num_hidden_layers=self.options.num_hidden_layers,
                                        dropout_prob=self.options.dropout,
                                        use_batch_norm=self.options.batch_norm,
                                        use_speed_and_angle=self.use_speed_and_angle,
                                        num_init_conv_layers=self.options.num_init_conv_layers,
                                        kernel_size=self.options.kernel_size,
                                        add_one=self.options.add_one,
                                        seperate_height=self.options.seperate_height,
                                        nonlinearity=self.options.nonlinearity).to(self.device)
        else:
            raise NotImplementedError('Invalid data source')


        self.train_ds = PouringDataset(self.options.dataset_dir,
                                       load_volume=self.options.task!='cross_section',
                                       load_depth_image=self.options.source=='from_depth',
                                       calc_wait_times=self.options.task=='wait_times',
                                       load_speed_angle_and_scale=self.use_speed_and_angle,
                                       volume_dir=self.options.volume_dir,
                                       num_divisions=self.options.num_horz_divs,
                                       image_size=self.options.image_size,
                                       center=self.options.center,
                                       add_noise_speed_angle=self.options.add_noise_speed_angle,
                                       add_noise_cross_section=self.options.add_noise_cross_section,
                                       threshold_fraction=self.options.threshold_fraction,
                                       is_train=True)
        self.test_ds = PouringDataset(self.options.dataset_dir,
                                      load_volume=self.options.task!='cross_section',
                                      load_depth_image=self.options.source=='from_depth',
                                      calc_wait_times=self.options.task=='wait_times',
                                      load_speed_angle_and_scale=self.use_speed_and_angle,
                                      volume_dir=self.options.volume_dir,
                                      num_divisions = self.options.num_horz_divs,
                                      image_size=self.options.image_size,
                                      center=self.options.center,
                                      add_noise_speed_angle=0,
                                      add_noise_cross_section=0,
                                      threshold_fraction=self.options.threshold_fraction,
                                      is_train=False)

        if self.options.optimizer == 'sgd':
            self.optimizer = torch.optim.SGD(params=self.model.parameters(), lr=self.options.lr, momentum=self.options.sgd_momentum, weight_decay=self.options.wd)
        elif self.options.optimizer == 'rmsprop':
            self.optimizer = torch.optim.RMSprop(params=self.model.parameters(), lr=self.options.lr, momentum=0, weight_decay=self.options.wd)
        else:
            self.optimizer = torch.optim.Adam(params=self.model.parameters(), lr=self.options.lr, betas=(self.options.adam_beta1, 0.999), weight_decay=self.options.wd)

        # pack all models and optimizers in dictionaries to interact with the checkpoint saver
        self.models_dict = {'stacked_hg': self.model}
        self.optimizers_dict = {'optimizer': self.optimizer}

        if self.options.loss == 'L1':
            self.criterion = nn.L1Loss(size_average=True).to(self.device)
        elif self.options.loss == 'SmoothL1':
            self.criterion = nn.SmoothL1Loss(size_average=True).to(self.device)
        else:
            self.criterion = nn.MSELoss(size_average=True).to(self.device)


    def _train_step(self, input_batch):
        self.model.train()
        return self._train_or_test_step(input_batch, True)



    def _train_summaries(self, input_batch, pred_profiles, loss, is_train=True):
        prefix = 'train/' if is_train else 'test/'
        if is_train:
            input_batch = [input_batch]
            pred_profiles = [pred_profiles]

        self.summary_writer.add_scalar(prefix + 'loss', loss, self.step_count)

        goal_key = ''
        if self.options.task == 'cross_section':
            goal_key = 'cross_section_profile'
        elif self.options.task == 'volume_profile':
            goal_key = 'volume_profile'
        elif self.options.task == 'wait_times':
            goal_key = 'wait_times'
        else:
            raise NotImplementedError('The requested task does not exist')

        percent_error = 0.0
        percent_usable = 0.0

        profile_images = []
        for j in range(len(pred_profiles)):
            for i in range(len(pred_profiles[j])):
                gt_profile = input_batch[j][goal_key][i]

                profile_image = self._make_profile_image(gt_profile, pred_profiles[j][i])
                profile_image = profile_image.to('cpu', dtype=torch.float32)

                if 'depth_image' in input_batch[j]:
                    resized_image = cv2.resize(input_batch[j]['depth_image'][i].cpu().numpy(), (profile_image.shape[1], profile_image.shape[2]))
                    resized_image = torch.from_numpy(resized_image)
                    color_image = torch.zeros_like(profile_image)
                    color_image[0, :, :] = resized_image
                    color_image[1, :, :] = resized_image
                    color_image[2, :, :] = resized_image
                else:
                    color_image = self._make_profile_image(input_batch[j]['cross_section_profile'][i]*128, np.zeros(len(input_batch[j]['cross_section_profile'][i])), speed=input_batch[j]['speed'][i], angle=input_batch[j]['angle'][i], name=input_batch[j]['name'][i])
                    color_image.to('cpu', dtype=torch.float32)

                profile_images.append(color_image)
                profile_images.append(profile_image)
                if torch.sum(torch.abs(gt_profile - pred_profiles[j][i])) < 0.5:
                    percent_usable += 1
                percent_error += torch.sum(torch.abs(gt_profile - pred_profiles[j][i]) / gt_profile)
        percent_error /= len(pred_profiles)
        percent_error /= len(pred_profiles[0])
        self.summary_writer.add_scalar(prefix + 'percent_error', percent_error, self.step_count)

        percent_usable /= len(pred_profiles)
        percent_usable /= len(pred_profiles[0])
        self.summary_writer.add_scalar(prefix + 'percent_usable', percent_usable, self.step_count)


        profile_image_grid = make_grid(profile_images, pad_value=1, nrow=4)
        self.summary_writer.add_image(prefix + 'profiles', profile_image_grid, self.step_count)
        if is_train:
            self.summary_writer.add_scalar('lr', self._get_lr(), self.step_count)

    def _make_profile_image(self, gt_profile, output_profile, im_size=128, speed=-1, angle=-1, name=-1):
        started_1d = False
        if len(gt_profile.shape) == 0:
            gt_profile = torch.tensor([gt_profile]).to(self.device)
            started_1d = True

        gt_profile = gt_profile.to(torch.float)

        image = torch.zeros((3, im_size, im_size))
        
        max_gt = gt_profile.max() * 1.5
        max_img = output_profile.max() * 1.1
        max_gt = max(max_gt, max_img)
        max_gt = 1.5
        #print output_profile.max(), output_profile.min()
        if self.options.task == 'wait_times':
            max_gt = 20

        pixels_per_band = im_size / len(gt_profile)
        for i in range(len(gt_profile)):
            gt_index = int(1.0 * im_size * gt_profile[i] / max_gt)
            output_index = int(1.0 * im_size * output_profile[i] / max_gt)
            
            start = im_size - pixels_per_band * (i + 1)
            end = im_size - pixels_per_band * (i)
            
            gt_index = max(0, gt_index)
            gt_index = min(im_size-1, gt_index)
            image[0, start:end, gt_index] = 1

            output_index = max(0, output_index)
            output_index = min(im_size-1, output_index)
            image[1, start:end, output_index] = 1
        if started_1d:
            text_image = np.zeros((im_size, im_size))
            cv2.putText(text_image,
                        str.format('{0:.3}/{1:.3}',float(gt_profile[0]), float(output_profile[0])),
                        (10, im_size - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        1,
                        2)
            text_image = torch.Tensor(text_image)
            image[0, :, :] += text_image
            image[1, :, :] += text_image
            image[2, :, :] += text_image

        if speed > 0:
            text_image = np.zeros((im_size, im_size))
            cv2.putText(text_image,
                        str.format('s: {0:.3}',float(speed)),
                        (10, 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        1,
                        2)
            text_image = torch.Tensor(text_image)
            image[0, :, :] += text_image
            image[1, :, :] += text_image
            image[2, :, :] += text_image

        if angle > 0:
            text_image = np.zeros((im_size, im_size))
            cv2.putText(text_image,
                        str.format('a: {0:.3}',float(angle)),
                        (10, im_size - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        1,
                        2)
            text_image = torch.Tensor(text_image)
            image[0, :, :] += text_image
            image[1, :, :] += text_image
            image[2, :, :] += text_image

        if name > -1:
            text_image = np.zeros((im_size, im_size))
            cv2.putText(text_image,
                        str.format('n: {0:.3}',float(name)),
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        1,
                        2)
            text_image = torch.Tensor(text_image)
            image[0, :, :] += text_image
            image[1, :, :] += text_image
            image[2, :, :] += text_image


        return image

    def test(self):
        test_data_loader = DataLoader(self.test_ds, batch_size=self.options.test_batch_size,
                                      num_workers=self.options.num_workers,
                                      pin_memory=self.options.pin_memory,
                                      shuffle=self.options.shuffle_test)
        all_profiles = []
        all_batches = []
        test_loss = torch.tensor(0.0, device=self.device)

        for tstep, batch in enumerate(tqdm(test_data_loader, desc='Testing')):
            batch = {k: v.to(self.device) for k,v in batch.items()}

            pred_profiles, loss = self._test_step(batch)

            test_loss += loss.data
            all_profiles.append(pred_profiles)
            all_batches.append(batch)

        self._train_summaries(all_batches, all_profiles, test_loss, is_train=False)


        
    def _test_step(self, input_batch):
        self.model.eval()
        return self._train_or_test_step(input_batch, False)


    def _train_or_test_step(self, input_batch, is_train):
        if self.options.source == 'from_depth':
            depth_images = input_batch['depth_image'].to(torch.float)
            input_data = depth_images.view(-1, 1, depth_images.shape[1], depth_images.shape[2])
        elif self.options.source == 'from_cross_section':
            input_data = input_batch['cross_section_profile'].to(torch.float)
            heights = input_batch['height'].to(torch.float)
            if not self.options.seperate_height:
                input_data = torch.cat((input_data, heights.view(heights.shape[0], 1)), dim=1)

        if self.options.task == 'cross_section':
            gt_profiles = input_batch['cross_section_profile'].to(torch.float)
        elif self.options.task == 'volume_profile':
            gt_profiles = input_batch['volume_profile'].to(torch.float)
        elif self.options.task == 'wait_times':
            gt_profiles = input_batch['wait_times'].to(torch.float)
        else:
            raise NotImplementedError('The requested task does not exist') 


        if self.use_speed_and_angle:
            speed = input_batch['speed'].to(torch.float)
            angle = input_batch['angle'].to(torch.float)

            with torch.set_grad_enabled(is_train):
               if not self.options.seperate_height:
                   pred_profiles = self.model(input_data, speed, angle)
               else:
                   pred_profiles = self.model(input_data, speed, angle, heights)
        else:
            with torch.set_grad_enabled(is_train):
                pred_profiles = self.model(input_data)


        loss = torch.tensor(0.0, device=self.device).to(torch.float)
        for i in range(len(pred_profiles)):
            loss += self.criterion(pred_profiles[i], gt_profiles[i])

        # Only do backprop when training
        if is_train:
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        return [profile.detach() for profile in pred_profiles], loss.detach()
