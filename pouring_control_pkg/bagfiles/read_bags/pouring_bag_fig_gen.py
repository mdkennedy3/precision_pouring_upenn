import rospy, sys, os
import numpy as np
import rosbag 
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.ticker import LinearLocator, FormatStrFormatter

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True

class PlotData(object):
    """docstring for PlotData"""
    def __init__(self):
        self.cwd = os.getcwd()
        self.diversity_dir = '/diversity/'
        self.stats_dir = '/stats/'
        self.trials = ['80','100','120']; self.times = ['8', '10', '12']
        self.figure_dir = self.cwd+'/figures/' 
        # self.trials = ['100']; self.times = ['10']

        #Diversity
        self.diversity_files = [[self.cwd+self.diversity_dir+'diversity_'+self.trials[idx]+'ml_'+self.times[jdx]+'s_trial_1.bag' for idx in range(len(self.trials))] for jdx in range(len(self.times))]
        # self.diversity_bags = [[ rosbag.Bag(self.cwd+self.diversity_dir+'diversity_'+self.trials[idx]+'ml_'+self.times[jdx]+'s_trial_1.bag') for idx in range(len(self.trials))] for jdx in range(len(self.times))]
        #Stat
        self.stat_files = [self.cwd+self.stats_dir+'stat_100ml_10s_trial_'+str(idx+1) + '.bag' for idx in range(50)]

        #Load these one by one
        # self.stat_bags = [rosbag.Bag(self.cwd+self.stats_dir+'stat_100ml_10s_trial_'+str(idx+1) + '.bag') for idx in range(1)]
        # self.stat_bags = [rosbag.Bag(self.cwd+self.stats_dir+'stat_100ml_10s_trial_'+str(idx+1) + '.bag') for idx in range(50)]
        #Define Messages
        self.mh_topic = '/mh_mh_dot' #.mh .mh_dot
        self.joint_states_topic = '/robot/joint_states'
        # self.theta_topic = '/robot/joint_states' #theta: .right_j6.pos
        # self.theta_vel = '/robot/joint_states/right_j6/vel' #true theta_vel
        # self.cmd_omega_topic = '/robot/limb/right/joint_command/velocity[0]' #pluck out the element
        self.cmd_omega_topic = '/robot/limb/right/joint_command' #pluck out the element
        self.scale_topic = '/scale_node/scale' #scale  .weight

    def Generic_response(self):
        print "plot generic response"
        #Msgs: mh, mh_dot,theta, omega
        #Axis: (ml, s) & (rad, s)
        #Trial stat_100ml ... Trial N (find N=1)
        bag = rosbag.Bag(self.stat_files[0])
        mh = {'data':[], 't':[]}
        mh_dot ={'data':[], 't':[]}
        th = {'data':[], 't':[]}
        th_dot = {'data':[], 't':[]}
        omega_input = {'data':[], 't':[]}
        scale = {'data':[], 't':[]}

        t_start = []
        bool_coolect_others = False
        for topic, msg, t in bag.read_messages(topics=[self.mh_topic, self.joint_states_topic, self.cmd_omega_topic,self.scale_topic]):
            if topic in self.mh_topic:
                if type(t_start) == list:
                    t_start = t.to_sec() #specifically put this in mh, and only extract positive terms of others
                    bool_coolect_others = True
                mh['data'].append(msg.mh)
                mh['t'].append(t.to_sec() - t_start)
                mh_dot['data'].append(msg.mh_dot)
                mh_dot['t'].append(t.to_sec()- t_start)
            elif topic in self.joint_states_topic and bool_coolect_others:
                th['data'].append(msg.position[7])
                th['t'].append(t.to_sec()- t_start)
                th_dot['data'].append(msg.velocity[7])
                th_dot['t'].append(t.to_sec()- t_start)
            elif topic in self.cmd_omega_topic and bool_coolect_others: 
                omega_input['data'].append(msg.velocity[0])
                omega_input['t'].append(t.to_sec()- t_start)
            elif topic in self.scale_topic and bool_coolect_others:
                scale['data'].append(msg.weight)
                scale['t'].append(t.to_sec()- t_start)






        #Figure 1: scale,h,hdot vs 10s
        fig1 = plt.figure(); ax1 = fig1.add_subplot(111)
        ax1.set_xlabel('$t$ (s)', fontsize=20); ax1.set_ylabel('$ml$', fontsize=25)
        
        ax1.plot(mh['t'],mh['data'],'b', label="$m_h$")
        ax1.plot(mh_dot['t'],mh_dot['data'],'--b', label="$\dot{m}_h$")
        ax1.plot(scale['t'],scale['data'],'--r', label="$scale$")

        ax1.legend(loc=2,fontsize=20)
        # ax1.set_xticks(np.arange(0,1,.1))
        # ax1.set_yticks(np.arange(0,7,1))
        plt.grid()
        # fig1.set_size_inches(18.5, 10.5)
        f1name = self.figure_dir + 'gen_resp_h_dh_scale_100ml_10s.pdf'
        fig1.savefig(f1name, dpi=100)


        #Figure 2: omega, theta vs 10s
        fig2 = plt.figure(); ax2 = fig2.add_subplot(111)
        ax2.set_xlabel('$t (s)$', fontsize=20); ax2.set_ylabel('$rad$', fontsize=25)
        ax2.plot(th['t'],th['data'],'-b', label="$\\theta_{act}$")
        ax2.plot(th_dot['t'],th_dot['data'],'--b', label="$\dot{\\theta}_{act}$")
        ax2.plot(omega_input['t'],omega_input['data'],'-r', label="$\omega_{input}$")
        # ax2.set_xticks(np.arange(0,1,.1))
        # ax2.set_yticks(np.arange(0,7,1))
        plt.grid()
        ax2.legend(loc=0,fontsize=20)

        # fig2.set_size_inches(18.5, 10.5)
        f2name = self.figure_dir + 'gen_resp_omega_theta_100ml_10s.pdf'
        fig2.savefig(f2name, dpi=100)

        plt.show()





    def Gen_Div_plots(self):
        print "Diversity Plots"
        #Msgs: mh, mh_dot 
        #Axis: ml, sec

        line = [None,'--',':']
        #Figure 1: 8s, (80,100,120)
        fig1 = plt.figure(); ax1 = fig1.add_subplot(111)
        ax1.set_xlabel('$t(s)$', fontsize=20); ax1.set_ylabel('$ml$', fontsize=25)
        for idx in range(3):
            bag = rosbag.Bag(self.diversity_files[0][idx])
            mh = {'data':[], 't':[]}
            mh_dot ={'data':[], 't':[]}
            scale = {'data':[], 't':[]}
            bool_coolect_others = False
            t_start = []
            for topic, msg, t in bag.read_messages(topics=[self.mh_topic, self.scale_topic]):
                if topic in self.mh_topic:
                    if type(t_start) == list:
                        t_start = t.to_sec() #specifically put this in mh, and only extract positive terms of others
                        bool_coolect_others = True
                    mh['data'].append(msg.mh)
                    mh['t'].append(t.to_sec() - t_start)
                    mh_dot['data'].append(msg.mh_dot)
                    mh_dot['t'].append(t.to_sec()- t_start)
                elif topic in self.scale_topic and bool_coolect_others:
                    scale['data'].append(msg.weight)
                    scale['t'].append(t.to_sec()- t_start)

            if idx > 0:
                marker1=line[idx]+'b'
                marker2=line[idx]+'g' 
                marker3=line[idx]+'r'
            else:
                marker1='b'
                marker2='g' 
                marker3='r'


            mh_tarray = np.array(mh['t'])
            mhdot_tarray = np.array(mh_dot['t'])
            scale_tarray = np.array(scale['t'])

            mh_t_ind = np.hstack(np.argwhere(mh_tarray<9.5))
            mhdot_t_ind = np.hstack( np.argwhere(mhdot_tarray<9.5))
            scale_t_ind = np.hstack(np.argwhere(scale_tarray<9.5))

            mh_t_ind = [int(mh_t_ind[ldx]) for ldx in range(len(mh_t_ind))]
            mhdot_t_ind = [int(mhdot_t_ind[ldx]) for ldx in range(len(mhdot_t_ind))]
            scale_t_ind = [int(scale_t_ind[ldx]) for ldx in range(len(scale_t_ind))]
            

            ax1.plot(mh['t'][:mh_t_ind[-1]],mh['data'][:mh_t_ind[-1]],marker1, label="$m_h$  %s $ml$" %self.trials[idx])
            ax1.plot(mh_dot['t'][:mhdot_t_ind[-1]] ,mh_dot['data'][:mhdot_t_ind[-1]],marker2, label="$\dot{m}_h$ %s $ml$" %self.trials[idx])
            ax1.plot(scale['t'][:scale_t_ind[-1]],scale['data'][:scale_t_ind[-1]],marker3, label="scale %s $ml$" %self.trials[idx])
            bag.close()

        ax1.legend(loc=2,ncol=1,fontsize=10)
        plt.grid()
        # fig1.set_size_inches(18.5, 10.5)
        f1name = self.figure_dir + 'diversity_8sec_traj.pdf'
        fig1.savefig(f1name, dpi=100)



        #Figure 2: 10s, (80,100,120)
        fig2 = plt.figure(); ax2 = fig2.add_subplot(111)
        ax2.set_xlabel('$t(s)$ ', fontsize=20); ax2.set_ylabel('$ml$', fontsize=25)
        mh = {'data':[], 't':[]}
        mh_dot ={'data':[], 't':[]}
        for idx in range(3):
            bag = rosbag.Bag(self.diversity_files[1][idx])
            mh = {'data':[], 't':[]}
            mh_dot ={'data':[], 't':[]}
            scale = {'data':[], 't':[]}
            bool_coolect_others = False
            t_start = []
            for topic, msg, t in bag.read_messages(topics=[self.mh_topic, self.scale_topic]):
                if topic in self.mh_topic:
                    if type(t_start) == list:
                        t_start = t.to_sec() #specifically put this in mh, and only extract positive terms of others
                        bool_coolect_others = True
                    mh['data'].append(msg.mh)
                    mh['t'].append(t.to_sec() - t_start)
                    mh_dot['data'].append(msg.mh_dot)
                    mh_dot['t'].append(t.to_sec()- t_start)
                elif topic in self.scale_topic and bool_coolect_others:
                    scale['data'].append(msg.weight)
                    scale['t'].append(t.to_sec()- t_start)

            if idx > 0:
                marker1=line[idx]+'b'
                marker2=line[idx]+'g' 
                marker3=line[idx]+'r'
            else:
                marker1='b'
                marker2='g' 
                marker3='r'


            mh_tarray = np.array(mh['t'])
            mhdot_tarray = np.array(mh_dot['t'])
            scale_tarray = np.array(scale['t'])

            mh_t_ind = np.hstack(np.argwhere(mh_tarray<11.5))
            mhdot_t_ind = np.hstack( np.argwhere(mhdot_tarray<11.5))
            scale_t_ind = np.hstack(np.argwhere(scale_tarray<11.5))

            mh_t_ind = [int(mh_t_ind[ldx]) for ldx in range(len(mh_t_ind))]
            mhdot_t_ind = [int(mhdot_t_ind[ldx]) for ldx in range(len(mhdot_t_ind))]
            scale_t_ind = [int(scale_t_ind[ldx]) for ldx in range(len(scale_t_ind))]
            

            ax2.plot(mh['t'][:mh_t_ind[-1]],mh['data'][:mh_t_ind[-1]],marker1, label="$m_h$  %s $ml$" %self.trials[idx])
            ax2.plot(mh_dot['t'][:mhdot_t_ind[-1]] ,mh_dot['data'][:mhdot_t_ind[-1]],marker2, label="$\dot{m}_h$ %s $ml$" %self.trials[idx])
            ax2.plot(scale['t'][:scale_t_ind[-1]],scale['data'][:scale_t_ind[-1]],marker3, label="$scale$ %s $ml$" %self.trials[idx])
            bag.close()



        ax2.legend(loc=2,ncol=1,fontsize=10)
        plt.grid()
        # fig2.set_size_inches(18.5, 10.5)
        f2name = self.figure_dir + 'diversity_10sec_traj.pdf'
        fig2.savefig(f2name, dpi=100)


        #Figure 3: 12s, (80,100,120)
        fig3 = plt.figure(); ax3 = fig3.add_subplot(111)
        ax3.set_xlabel('$t(s)$', fontsize=20); ax3.set_ylabel('$ml$', fontsize=25)
        mh = {'data':[], 't':[]}
        mh_dot ={'data':[], 't':[]}
        for idx in range(3):
            bag = rosbag.Bag(self.diversity_files[2][idx])
            mh = {'data':[], 't':[]}
            mh_dot ={'data':[], 't':[]}
            scale = {'data':[], 't':[]}
            bool_coolect_others = False
            t_start = []
            for topic, msg, t in bag.read_messages(topics=[self.mh_topic, self.scale_topic]):
                if topic in self.mh_topic:
                    if type(t_start) == list:
                        t_start = t.to_sec() #specifically put this in mh, and only extract positive terms of others
                        bool_coolect_others = True
                    mh['data'].append(msg.mh)
                    mh['t'].append(t.to_sec() - t_start)
                    mh_dot['data'].append(msg.mh_dot)
                    mh_dot['t'].append(t.to_sec()- t_start)
                elif topic in self.scale_topic and bool_coolect_others:
                    scale['data'].append(msg.weight)
                    scale['t'].append(t.to_sec()- t_start)

            if idx > 0:
                marker1=line[idx]+'b'
                marker2=line[idx]+'g' 
                marker3=line[idx]+'r'
            else:
                marker1='b'
                marker2='g' 
                marker3='r'

            mh_tarray = np.array(mh['t'])
            mhdot_tarray = np.array(mh_dot['t'])
            scale_tarray = np.array(scale['t'])

            mh_t_ind = np.hstack(np.argwhere(mh_tarray<13.5))
            mhdot_t_ind = np.hstack( np.argwhere(mhdot_tarray<13.5))
            scale_t_ind = np.hstack(np.argwhere(scale_tarray<13.5))

            mh_t_ind = [int(mh_t_ind[ldx]) for ldx in range(len(mh_t_ind))]
            mhdot_t_ind = [int(mhdot_t_ind[ldx]) for ldx in range(len(mhdot_t_ind))]
            scale_t_ind = [int(scale_t_ind[ldx]) for ldx in range(len(scale_t_ind))]
            

            ax3.plot(mh['t'][:mh_t_ind[-1]],mh['data'][:mh_t_ind[-1]],marker1, label="$m_h$  %s $ml$" %self.trials[idx])
            ax3.plot(mh_dot['t'][:mhdot_t_ind[-1]] ,mh_dot['data'][:mhdot_t_ind[-1]],marker2, label="$\dot{m}_h$ %s $ml$" %self.trials[idx])
            ax3.plot(scale['t'][:scale_t_ind[-1]],scale['data'][:scale_t_ind[-1]],marker3, label="$scale$ %s $ml$" %self.trials[idx])
            bag.close()
        ax3.legend(loc=2,ncol=1,fontsize=10)
        # ax3.legend(bbox_to_anchor=(1.02, 2.), loc=1, borderaxespad=0.)
        plt.grid()
        # fig3.set_size_inches(18.5, 10.5)
        f3name = self.figure_dir + 'diversity_12sec_traj.pdf'
        fig3.savefig(f3name, dpi=100)

        # plt.show()


    def time_match_difference(self,x1,t1,x2,t2):
        #Given t1 has less hz than t2: 
        if len(x1) == len(t1) and len(x2) == len(t2):
            x2_new = []
            for idx in range(len(t1)):
                t2_minus_t1 = [np.abs(t2[jdx] - t1[idx]) for jdx in range(len(t2))]
                ind_min = np.argmin(t2_minus_t1)
                # print "\nind min: ", ind_min
                x2_new.append(x2[ind_min])

            error = [x1[idx] - x2_new[idx] for idx in range(len(x1))]

        else:
            print "inputs do not have same size"
            error = []

        return error

    def Stat_plots(self):
        print "Stats Plot"
        #Msgs: mh
        #Axis: ml, sec
        #Figure: h_i, std_dev(shaded region), avg
        fig1 = plt.figure(); ax1 = fig1.add_subplot(111)
        mh_stat = []
        # scale_stat = []
        error_stat = []
        error_time_stat = []
        t_stat = []
        t_stat_scale = []
        ax1.set_xlabel('$t(s)$ ', fontsize=20); ax1.set_ylabel('ml', fontsize=25)

        fig2 = plt.figure(); ax2 = fig2.add_subplot(111)
        ax2.set_xlabel('$t(s)$ ', fontsize=20); ax2.set_ylabel('ml', fontsize=25)

        num_pts = 50

        for idx in range(num_pts):
            bag = rosbag.Bag(self.stat_files[idx])
            mh = {'data':[], 't':[]}
            mh_dot ={'data':[], 't':[]}
            scale = {'data':[], 't':[]}
            bool_coolect_others = False
            t_start = []
            for topic, msg, t in bag.read_messages(topics=[self.mh_topic, self.scale_topic]):
                if topic in self.mh_topic:
                    if type(t_start) == list:
                        t_start = t.to_sec() #specifically put this in mh, and only extract positive terms of others
                        bool_coolect_others = True
                    mh['data'].append(msg.mh)
                    mh['t'].append(t.to_sec() - t_start)
                    mh_dot['data'].append(msg.mh_dot)
                    mh_dot['t'].append(t.to_sec()- t_start)
                elif topic in self.scale_topic and bool_coolect_others:
                    scale['data'].append(msg.weight)
                    scale['t'].append(t.to_sec()- t_start)
            bag.close()

            err_vect = self.time_match_difference(scale['data'],scale['t'],mh['data'],mh['t'])

            error_stat.append(err_vect)

            mh_stat.append(mh)
            # scale_stat.append(scale)
            if len(t_stat) == 0:
                t_stat = mh['t']
            elif len(t_stat) > len(mh['t']):
                t_stat = mh['t'] #find min length time
            #Now for scale

            if len(t_stat_scale) == 0:
                t_stat_scale = scale['t']
            elif len(t_stat_scale) > len(scale['t']):
                t_stat_scale = scale['t'] #find min length time

        #Now calculate statistics
        #1. Given array of t's, a) find longest time (as settling occurs) b) 
        # print "mh stat",mh_stat
        mh_stat_new = [mh_stat[idx]['data'][:len(t_stat)] for idx in range(num_pts)]

        for kdx in range(num_pts):
            # print "tstat: ", t_stat, " mhstat", mh_stat_new[kdx]
            ax1.plot(t_stat,mh_stat_new[kdx],lw=1.)


        m_array = np.vstack(mh_stat_new)
        m_avg = np.mean(m_array, axis=0)
        m_stddev = np.std(m_array, axis=0)
        t = t_stat
        x = m_avg
        e = m_stddev
        # print "len of t: ", len(t), " len of x:", len(x)
        ax1.plot(t,x, 'k',lw=7, label="$m_h$  avg"); 
        ax1.fill_between(t, x-e, x+e, alpha=0.3, edgecolor='#CC4F1B', facecolor='#FF9848')

        ax1.legend(loc=2,fontsize=15)
        ax1.grid()
        # fig1.set_size_inches(18.5, 10.5)
        f1name = self.figure_dir + 'stat_100ml_10s_50trials.pdf'
        fig1.savefig(f1name, dpi=100)

        #Now calculate statistics
        #1. Given array of t's, a) find longest time (as settling occurs) b) 

        error_stat_new = [error_stat[idx][:len(t_stat_scale)] for idx in range(num_pts)]

        for kdx in range(num_pts):
            ax2.plot(t_stat_scale,error_stat_new[kdx],lw=1.)


        err_array = np.vstack(error_stat_new)
        err_avg = np.mean(err_array, axis=0)
        err_stddev = np.std(err_array, axis=0)

        t = t_stat_scale
        x = err_avg
        e = err_stddev
        # print "len of t: ", len(t), " len of x:", len(x)
        ax2.plot(t,x, 'k',lw=7, label="$(scale - m_h )$  avg"); 
        ax2.fill_between(t, x-e, x+e, alpha=0.3, edgecolor='#CC4F1B', facecolor='#FF9848')

        ax2.legend(loc=2,fontsize=15)
        ax2.grid()
        # fig2.set_size_inches(18.5, 10.5)
        f2name = self.figure_dir + 'error_stat_100ml_10s_50trials.pdf'
        fig2.savefig(f2name, dpi=100)



        plt.show()




def main():
    plt_obj = PlotData()

    #Plot General Data
    # plt_obj.Generic_response()

    #Plot diversity
    # plt_obj.Gen_Div_plots()

    #Plot Statistics
    plt_obj.Stat_plots()


if __name__ == '__main__':
    main()