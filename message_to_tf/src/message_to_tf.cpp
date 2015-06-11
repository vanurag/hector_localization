#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf/transform_datatypes.h>

#include <topic_tools/shape_shifter.h>

std::string g_odometry_topic;
std::string g_pose_topic;
std::string g_imu_topic;
std::string g_gps_topic("");
std::string g_topic;
std::string g_frame_id;
std::string g_footprint_frame_id;
std::string g_position_frame_id;
std::string g_stabilized_frame_id;
std::string g_child_frame_id;

bool g_publish_roll_pitch;

std::string g_tf_prefix;

tf::TransformBroadcaster *g_transform_broadcaster;
tf::StampedTransform imu_gps_tf;
std::vector<geometry_msgs::TransformStamped> imu_gps_transforms;
ros::Publisher g_pose_publisher;
ros::Publisher g_euler_publisher;


////////////////////// Calibration Data //////////////////////
// imu0(vi-sensor) to mavros(px4 state estimator)
tf::Matrix3x3 R_imu0_mavros(0.0047009 , -0.99992521,  0.0112908,
                           -0.99997648, -0.00475693, -0.00494021,
                            0.00499355, -0.01126731, -0.99992405);
tf::Vector3 t_imu0_mavros(-0.03532628,  0.03224448,  0.01215357);

// mavros(px4 state estimator) to world
tf::Matrix3x3 R_mavros_W;
tf::Vector3 t_mavros_W;

// cam1(monocular vi-sensor) to imu0(vi-sensor)
tf::Matrix3x3 R_cam1_imu0(0.00111144, 0.99999923, 0.00055929,
                         -0.42368471, 0.00097752, -0.9058092,
                         -0.90580905, 0.00076979, 0.42368546);
tf::Vector3 t_cam1_imu0(-0.01888825, -0.22702259, 0.00829739);

// pre-computed part of cam1 to world
tf::Matrix3x3 R_cam1_W_precomp = R_imu0_mavros * R_cam1_imu0;
tf::Vector3 t_cam1_W_precomp = R_imu0_mavros * t_cam1_imu0 + t_imu0_mavros;

// cam1 to world will be computed as new pose updates arrive
tf::Matrix3x3 R_cam1_W;
tf::Vector3 t_cam1_W;
//////////////////////////////////////////////////////////////

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void addTransform(std::vector<geometry_msgs::TransformStamped>& transforms, const tf::StampedTransform& tf)
{
  transforms.resize(transforms.size()+1);
  tf::transformStampedTFToMsg(tf, transforms.back());
}

void sendTransform(geometry_msgs::Pose const &pose, const std_msgs::Header& header, std::string child_frame_id = "")
{
  std::vector<geometry_msgs::TransformStamped> transforms;

  tf::StampedTransform tf;
  tf.stamp_ = header.stamp;

  tf.frame_id_ = header.frame_id;
  if (!g_frame_id.empty()) tf.frame_id_ = g_frame_id;
  tf.frame_id_ = tf::resolve(g_tf_prefix, tf.frame_id_);

  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.orientation, orientation);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  tf::Point position;
  tf::pointMsgToTF(pose.position, position);

  // position intermediate transform (x,y,z)
  if( !g_position_frame_id.empty() && child_frame_id != g_position_frame_id) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, g_position_frame_id);
    tf.setOrigin(tf::Vector3(position.x(), position.y(), position.z() ));
    tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    addTransform(transforms, tf);
  }

  // footprint intermediate transform (x,y,yaw)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_footprint_frame_id) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, g_footprint_frame_id);
    tf.setOrigin(tf::Vector3(position.x(), position.y(), 0.0));
    tf.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, yaw));
    addTransform(transforms, tf);

    yaw = 0.0;
    position.setX(0.0);
    position.setY(0.0);
    tf.frame_id_ = tf::resolve(g_tf_prefix, g_footprint_frame_id);
  }

  // stabilized intermediate transform (z)
  if (!g_footprint_frame_id.empty() && child_frame_id != g_stabilized_frame_id) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, g_stabilized_frame_id);
    tf.setOrigin(tf::Vector3(0.0, 0.0, position.z()));
    tf.setBasis(tf::Matrix3x3::getIdentity());
    addTransform(transforms, tf);

    position.setZ(0.0);
    tf.frame_id_ = tf::resolve(g_tf_prefix, g_stabilized_frame_id);
  }

  // base_link transform (roll, pitch)
  if (g_publish_roll_pitch) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, child_frame_id);
    tf.setOrigin(position);
    tf.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
    addTransform(transforms, tf);
  }

  g_transform_broadcaster->sendTransform(transforms);

  // publish pose message
  if (g_pose_publisher) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header = header;
    g_pose_publisher.publish(pose_stamped);
  }

  // publish pose message
  if (g_euler_publisher) {
    geometry_msgs::Vector3Stamped euler_stamped;
    euler_stamped.vector.x = roll;
    euler_stamped.vector.y = pitch;
    euler_stamped.vector.z = yaw;
    euler_stamped.header = header;
    g_euler_publisher.publish(euler_stamped);
  }
}

void odomCallback(nav_msgs::Odometry const &odometry) {
  sendTransform(odometry.pose.pose, odometry.header, odometry.child_frame_id);
}

void poseCallback(geometry_msgs::PoseStamped const &pose) {
  sendTransform(pose.pose, pose.header);
}

void imuCallback(sensor_msgs::Imu const &imu) {
  std::vector<geometry_msgs::TransformStamped> transforms;
  std::string child_frame_id;

  tf::StampedTransform tf;
  tf.stamp_ = imu.header.stamp;

  tf.frame_id_ = tf::resolve(g_tf_prefix, g_stabilized_frame_id);
  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imu.orientation, orientation);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
  tf::Quaternion rollpitch = tf::createQuaternionFromRPY(roll, pitch, 0.0);

  // base_link transform (roll, pitch)
  if (g_publish_roll_pitch) {
    tf.child_frame_id_ = tf::resolve(g_tf_prefix, child_frame_id);
    tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf.setRotation(rollpitch);
    addTransform(transforms, tf);
  }

  if (!transforms.empty()) g_transform_broadcaster->sendTransform(transforms);

  // publish pose message
  if (g_pose_publisher) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = imu.header.stamp;
    pose_stamped.header.frame_id = g_stabilized_frame_id;
    tf::quaternionTFToMsg(rollpitch, pose_stamped.pose.orientation);
    g_pose_publisher.publish(pose_stamped);
  }
}

void imu_gpsCallback1(sensor_msgs::Imu const &imu) {
  
  std::string child_frame_id;

  imu_gps_tf.stamp_ = imu.header.stamp;

  imu_gps_tf.frame_id_ = tf::resolve(g_tf_prefix, g_stabilized_frame_id);
  if (!g_child_frame_id.empty()) child_frame_id = g_child_frame_id;
  if (child_frame_id.empty()) child_frame_id = "base_link";

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imu.orientation, orientation);
  tfScalar yaw, pitch, roll;
  R_mavros_W = tf::Matrix3x3(orientation);
  R_cam1_W = R_mavros_W * R_cam1_W_precomp;
  
  R_cam1_W.getEulerYPR(yaw, pitch, roll);
  tf::Quaternion cam_orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);

  imu_gps_tf.child_frame_id_ = tf::resolve(g_tf_prefix, child_frame_id);
  imu_gps_tf.setRotation(cam_orientation);

  // publish pose message
  if (g_pose_publisher) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = imu.header.stamp;
    pose_stamped.header.frame_id = g_stabilized_frame_id;
    tf::quaternionTFToMsg(orientation, pose_stamped.pose.orientation);
    g_pose_publisher.publish(pose_stamped);
  }
}

void imu_gpsCallback2(sensor_msgs::NavSatFix const &gps) {
  std::string child_frame_id;

  t_mavros_W = tf::Vector3(gps.longitude, gps.latitude, gps.altitude);
  t_cam1_W = R_mavros_W * t_cam1_W_precomp + t_mavros_W;
  imu_gps_tf.setOrigin(t_mavros_W);
  addTransform(imu_gps_transforms, imu_gps_tf);

  if (!imu_gps_transforms.empty()) g_transform_broadcaster->sendTransform(imu_gps_transforms);
}

void multiCallback(topic_tools::ShapeShifter const &input) {
  if (input.getDataType() == "nav_msgs/Odometry") {
    nav_msgs::Odometry::ConstPtr odom = input.instantiate<nav_msgs::Odometry>();
    odomCallback(*odom);
    return;
  }

  if (input.getDataType() == "geometry_msgs/PoseStamped") {
    geometry_msgs::PoseStamped::ConstPtr pose = input.instantiate<geometry_msgs::PoseStamped>();
    poseCallback(*pose);
    return;
  }

  if (input.getDataType() == "sensor_msgs/Imu") {
    sensor_msgs::Imu::ConstPtr imu = input.instantiate<sensor_msgs::Imu>();
    imuCallback(*imu);
    return;
  }

  ROS_ERROR_THROTTLE(1.0, "message_to_tf received a %s message. Supported message types: nav_msgs/Odometry geometry_msgs/PoseStamped sensor_msgs/Imu", input.getDataType().c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "message_to_tf");

  g_footprint_frame_id = "base_footprint";
  g_stabilized_frame_id = "base_stabilized";
  // g_position_frame_id = "base_position";
  // g_child_frame_id = "base_link";

  ros::NodeHandle priv_nh("~");
  priv_nh.getParam("odometry_topic", g_odometry_topic);
  priv_nh.getParam("pose_topic", g_pose_topic);
  priv_nh.getParam("imu_topic", g_imu_topic);
  priv_nh.getParam("gps_topic", g_gps_topic);
  priv_nh.getParam("topic", g_topic);
  priv_nh.getParam("frame_id", g_frame_id);
  priv_nh.getParam("footprint_frame_id", g_footprint_frame_id);
  priv_nh.getParam("position_frame_id", g_position_frame_id);
  priv_nh.getParam("stabilized_frame_id", g_stabilized_frame_id);
  priv_nh.getParam("child_frame_id", g_child_frame_id);

  // get topic from the commandline
  if (argc > 1) {
      g_topic = argv[1];
      g_odometry_topic.clear();
      g_pose_topic.clear();
      g_imu_topic.clear();
      g_gps_topic.clear();
  }

  g_publish_roll_pitch = true;
  priv_nh.getParam("publish_roll_pitch", g_publish_roll_pitch);

  g_tf_prefix = tf::getPrefixParam(priv_nh);
  g_transform_broadcaster = new tf::TransformBroadcaster;

  ros::NodeHandle node;
  ros::Subscriber sub1, sub2, sub3, sub4, sub5, sub6;
  int subscribers = 0;
  if (!g_odometry_topic.empty()) {
      sub1 = node.subscribe(g_odometry_topic, 10, &odomCallback);
      subscribers++;
  }
  if (!g_pose_topic.empty()) {
      sub2 = node.subscribe(g_pose_topic, 10, &poseCallback);
      subscribers++;
  }
  if (!g_imu_topic.empty() && g_gps_topic.empty()) {
      sub3 = node.subscribe(g_imu_topic, 10, &imuCallback);
      subscribers++;
  }
  if (! (g_imu_topic.empty() || g_gps_topic.empty()) ) {
      sub5 = node.subscribe(g_imu_topic, 10, &imu_gpsCallback1);
      sub6 = node.subscribe(g_gps_topic, 10, &imu_gpsCallback2);
      subscribers++;
  }
  if (!g_topic.empty()) {
      sub4 = node.subscribe(g_topic, 10, &multiCallback);
      subscribers++;
  }

  if (subscribers == 0) {
    ROS_FATAL("Usage: rosrun message_to_tf message_to_tf <topic>");
    return 1;
  } else if (subscribers > 1) {
    ROS_FATAL("More than one of the parameters odometry_topic, pose_topic, imu_topic and topic are set.\n"
              "Please specify exactly one of them or simply add the topic name to the command line.");
    return 1;
  }

  bool publish_pose = true;
  priv_nh.getParam("publish_pose", publish_pose);
  if (publish_pose) {
    std::string publish_pose_topic;
    priv_nh.getParam("publish_pose_topic", publish_pose_topic);

    if (!publish_pose_topic.empty())
      g_pose_publisher = node.advertise<geometry_msgs::PoseStamped>(publish_pose_topic, 10);
    else
      g_pose_publisher = priv_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
  }

  bool publish_euler = true;
  priv_nh.getParam("publish_euler", publish_euler);
  if (publish_euler) {
    std::string publish_euler_topic;
    priv_nh.getParam("publish_euler_topic", publish_euler_topic);

    if (!publish_euler_topic.empty())
      g_euler_publisher = node.advertise<geometry_msgs::Vector3Stamped>(publish_euler_topic, 10);
    else
      g_euler_publisher = priv_nh.advertise<geometry_msgs::Vector3Stamped>("euler", 10);
  }

  ros::spin();
  delete g_transform_broadcaster;
  return 0;
}
