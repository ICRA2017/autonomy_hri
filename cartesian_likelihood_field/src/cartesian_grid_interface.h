#ifndef CARTESIAN_GRID_INTERFACE_H
#define CARTESIAN_GRID_INTERFACE_H
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <hark_msgs/HarkSource.h>
#include <hark_msgs/HarkSourceVal.h>
#include <autonomy_human/human.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "cartesian_grid.h"


class CartesianGridInterface
{
private:
    ros::NodeHandle n;
    tf::TransformListener* tf_listener;

    // Leg
    ros::Publisher legs_grid_pub;
    ros::Publisher leg_occupancy_grid_pub;
    std::string leg_frame_id;
    CartesianGrid* leg_grid;
    ros::Time last_leg_time;
    ros::Duration diff_leg_time;
    bool leg_detection_enable;
    unsigned int leg_counter;
    sensor_msgs::PointCloud leg_pointcloud_grid;
    nav_msgs::OccupancyGrid leg_occupancy_grid;


    // Face
    ros::Publisher face_grid_pub;
    ros::Publisher face_occupancy_grid_pub;
    std::string face_frame_id;
    CartesianGrid* face_grid;
    ros::Time last_face_time;
    ros::Duration diff_face_time;
    bool torso_detection_enable;
    unsigned int torso_counter;
    sensor_msgs::PointCloud face_pointcloud_grid;
    nav_msgs::OccupancyGrid face_occupancy_grid;



    // Sound
    ros::Publisher sound_grid_pub;
    ros::Publisher sound_occupancy_grid_pub;
    std::string sound_frame_id;
    CartesianGrid* sound_grid;
    ros::Time last_sound_time;
    ros::Duration diff_sound_time;
    bool sound_detection_enable;
    unsigned int sound_counter;
    sensor_msgs::PointCloud sound_pointcloud_grid;
    nav_msgs::OccupancyGrid sound_occupancy_grid;


    // Laser
    ros::Publisher laser_grid_pub;
    ros::Publisher laser_occupancy_grid_pub;
    std::string laser_frame_id;
    CartesianGrid* laser_grid;
    ros::Time last_laser_time;
    ros::Duration diff_laser_time;
    bool laser_detection_enable;
    unsigned int laser_counter;
    sensor_msgs::PointCloud laser_pointcloud_grid;
    nav_msgs::OccupancyGrid laser_occupancy_grid;


    // Human
    ros::Publisher human_grid_pub;
    ros::Publisher human_occupancy_grid_pub;
    std::string human_frame_id;
    CartesianGrid* human_grid;
    //ros::Publisher highest_point_pub;
    unsigned int accept_counter;
    unsigned int reject_counter;
    sensor_msgs::PointCloud human_pointcloud_grid;
    nav_msgs::OccupancyGrid human_occupancy_grid;


    double prior_threshold;
    double update_rate;
    int loop_rate;
    int number_of_sensors;
    int sensitivity;
    CellProbability_t cell_probability;
    SensorFOV_t fov;

    int map_size;
    double map_resolution;

    void init();
    void initWorldGrids();
    bool transformToBase(geometry_msgs::PointStamped& source_point,
                         geometry_msgs::PointStamped& target_point,
                         bool debug = false);
    bool transformToOdom(geometry_msgs::PointStamped& source_point,
                         geometry_msgs::PointStamped target_point,
                         bool debug = false);
    void pointCloudGrid(CartesianGrid* grid,
                        sensor_msgs::PointCloud* pointcloud_grid);
    void occupancyGrid(CartesianGrid* grid,
                       nav_msgs::OccupancyGrid *occupancy_grid);

    void initLegs(SensorFOV_t sensor_fov);              //NEEDED FOR EVERY HUMAN FEATURE
    void initFaces(SensorFOV_t sensor_fov);
    void initSound(SensorFOV_t sensor_fov);
    void initLaser(SensorFOV_t sensor_fov);
    void initHuman();
    void publish();                                         // TO BE MODIFIED FOR EVERY HUMAN FEATURE

public:
    CartesianGridInterface();
    CartesianGridInterface(ros::NodeHandle _n, tf::TransformListener* _tf_listener);   // TO BE MODIFIED FOR EVERY HUMAN FEATURE
    void legCallBack(const geometry_msgs::PoseArray& msg);      //NEEDED FOR EVERY HUMAN FEATURE
    void faceCallBack(const autonomy_human::human& msg);
    void soundCallBack(const hark_msgs::HarkSource& msg);
    void laserCallBack(const sensor_msgs::LaserScan& msg);
    void spin();                            // TO BE MODIFIED FOR EVERY HUMAN FEATURE
    ~CartesianGridInterface();                             // TO BE MODIFIED FOR EVERY HUMAN FEATURE
};


#endif