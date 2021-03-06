#ifndef CVISIONGRID_H
#define CVISIONGRID_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "grid.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <std_msgs/Float32MultiArray.h>

class CVisionGrid
{
  ros::NodeHandle n_;
  tf::TransformListener* tf_listener_;
  ros::Publisher grid_pub_;
  ros::Publisher prob_pub_;
  ros::Publisher proj_pub_;
  ros::Publisher marker_pub_;
  Velocity_t velocity_;
  ros::Duration diff_time_;
  ros::Time last_time_;
  CGrid* grid_;
  cv::KalmanFilter KFTracker_;
  cv::Mat KFmeasurement_;
  std::vector<PolarPose> cstate_;
  std::vector<PolarPose> cmeas_;
  nav_msgs::Odometry encoder_reading_;
  std::vector<PolarPose> meas_;
  autonomy_human::raw_detections torso_reading_;
  std::vector<bool> match_meas_;
  geometry_msgs::PoseArray prob_;
  ros::Time last_seen_torso_;
  int probability_projection_step;
  visualization_msgs::MarkerArray marker_array;

  void init();
  void initKF();
  void initTfListener();
  void initGrid();
  void makeStates();
  void clearStates();
  void addLastStates();
  void addMeasurements();
  void filterStates();
  void updateKF();
  void callbackClear();
  void computeObjectVelocity();
  void publishProbability();
  void publishOccupancyGrid();
  void passStates();
  void KeepLastTorso();
  void publishProjection();
  void publishMarkers();

public:

  CVisionGrid();
  CVisionGrid(ros::NodeHandle _n, tf::TransformListener* _tf_listener, int _probability_projection_step);
  ~CVisionGrid();

  void syncCallBack(const autonomy_human::raw_detectionsConstPtr& torso_msg,
                    const nav_msgs::OdometryConstPtr& encoder_msg);

  void spin();

};

#endif // CVISIONGRID_H
