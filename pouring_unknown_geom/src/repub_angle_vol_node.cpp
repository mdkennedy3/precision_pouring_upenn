/*
* Copyright (c) <2017>, <Monroe Kennedy III>
* All rights reserved.
*/

// Purpose of this script is to make the messages th,V easily accessible for python script in one message

#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/JointState.h>  //for angle
#include <pouring_control_pkg/Hfilt.h> //for height/volume
#include <pouring_control_pkg/MeasMsg.h> //for height/volume
#include <pouring_unknown_geom/PourVolAng.h>

#include <pouring_unknown_geom/PourAngStateMsg.h>

using namespace message_filters;
typedef sync_policies::ApproximateTime<pouring_unknown_geom::PourAngStateMsg, pouring_control_pkg::MeasMsg> MySyncPolicy;
// typedef sync_policies::ApproximateTime<pouring_unknown_geom::PourAngStateMsg, pouring_control_pkg::Hfilt> MySyncPolicy;
// typedef sync_policies::ApproximateTime<sensor_msgs::JointState, pouring_control_pkg::MeasMsg> MySyncPolicy;

class VolAngleRepub
{
  public:
    VolAngleRepub();
    void publish();

  private:
    ros::NodeHandle pnh_;
    ros::Publisher vol_ang_pub_;
    //ros::Subscriber pour_ang_state_sub_, height_sub_;
    boost::shared_ptr<message_filters::Subscriber<pouring_unknown_geom::PourAngStateMsg>> pour_ang_state_sub_;
    boost::shared_ptr<message_filters::Subscriber<pouring_control_pkg::MeasMsg>> height_sub_;
    // boost::shared_ptr<message_filters::Subscriber<pouring_control_pkg::Hfilt>> height_sub_;

    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    void joint_msg_callback(const sensor_msgs::JointStateConstPtr& msg);
    //void volume_msg_callback(const pouring_control_pkg::HfiltConstPtr& msg);
    //void callback(const pouring_unknown_geom::PourAngStateMsgConstPtr& jnt_msg, const pouring_control_pkg::HfiltConstPtr& meas_msg);
    void callback(const pouring_unknown_geom::PourAngStateMsgConstPtr& jnt_msg, const pouring_control_pkg::MeasMsgConstPtr& meas_msg);

    void volume_msg_callback(const pouring_control_pkg::MeasMsgConstPtr& msg);
    //void callback(const sensor_msgs::JointStateConstPtr& jnt_msg, const pouring_control_pkg::MeasMsgConstPtr& meas_msg);

    float curr_vol_;
    float curr_ang_;
};

VolAngleRepub::VolAngleRepub()
{
  ROS_INFO("repub node running");

  pnh_ = ros::NodeHandle("~");

  curr_vol_ = 0.0;
  curr_ang_ = 0.0;
  //pour_ang_state_sub_ = pnh_.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, boost::bind(&VolAngleRepub::joint_msg_callback, this, _1));
  //height_sub_ = pnh_.subscribe<pouring_control_pkg::Hfilt>("/mh_mh_dot", 1, boost::bind(&VolAngleRepub::volume_msg_callback, this, _1));

  pour_ang_state_sub_.reset(new message_filters::Subscriber<pouring_unknown_geom::PourAngStateMsg>(pnh_, "pour_angle_state", 1));
  //height_sub_.reset(new message_filters::Subscriber<pouring_control_pkg::Hfilt>(pnh_, "h_filtered", 1));
  height_sub_.reset(new message_filters::Subscriber<pouring_control_pkg::MeasMsg>(pnh_, "mh_mh_dot", 1));

  vol_ang_pub_ = pnh_.advertise<pouring_unknown_geom::PourVolAng>("pour_vol_ang", 1);

  //Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), *pour_ang_state_sub_, *height_sub_);
  //sync.registerCallback(boost::bind(&VolAngleRepub::callback, this, _1, _2));
  sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100), *pour_ang_state_sub_, *height_sub_));
  sync_->registerCallback(boost::bind(&VolAngleRepub::callback, this, _1, _2));
}

void VolAngleRepub::callback(const pouring_unknown_geom::PourAngStateMsgConstPtr& pour_vol_ang_msg, const pouring_control_pkg::MeasMsgConstPtr& meas_msg)
{
  ROS_INFO("got ang vol");

  // curr_vol_ = meas_msg->hfilt;
  curr_vol_ = meas_msg->mh;  //This msg has taken the receiving container shape into account
  int index_last = 7;
  curr_ang_ = pour_vol_ang_msg->theta;

  pouring_unknown_geom::PourVolAng vol_ang;
  vol_ang.header.stamp = ros::Time::now();
  vol_ang.volume = curr_vol_;
  vol_ang.angle = curr_ang_;

  vol_ang_pub_.publish(vol_ang);
}

void VolAngleRepub::joint_msg_callback(const sensor_msgs::JointStateConstPtr& msg)
{
  //This function listens for the joint msg (which runs slower than robot states), and
  int index_last = 7;
  curr_ang_ = msg->position[index_last];
}

void VolAngleRepub::volume_msg_callback(const pouring_control_pkg::MeasMsgConstPtr& msg)
{

  //This function listens for the camera msg (which runs slower than robot states), and
  // curr_vol_ = msg->hfilt;
  curr_vol_ = msg->mh;  //This msg has taken the receiving container shape into account

  pouring_unknown_geom::PourVolAng vol_ang;
  vol_ang.header.stamp = ros::Time::now();
  vol_ang.volume = curr_vol_;
  vol_ang.angle = curr_ang_;

  vol_ang_pub_.publish(vol_ang);

}

void VolAngleRepub::publish()
{
  pouring_unknown_geom::PourVolAng vol_ang;
  vol_ang.header.stamp = ros::Time::now();
  vol_ang.volume = curr_vol_;
  vol_ang.angle = curr_ang_;

  vol_ang_pub_.publish(vol_ang);
  ROS_INFO("publishing");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "repub_angle_vol_node");
  ROS_INFO("repub_angle_vol_node_running");

  VolAngleRepub vol_ang; //Call the class, service callback is in the constructor, and looped by the ros spin
  ros::spin();

  // ros::Rate loop_rate(10);

  // while (ros::ok())
  // {
  //   ros::spinOnce();

  //   //Publish latest joint angles and vol
  //   vol_ang.publish();

  //   loop_rate.sleep();
  // }

  return 0;
}
