//
// Created by biao on 24-8-16.
//

#ifndef RANDOM_FOREAST_H
#define RANDOM_FOREAST_H

#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <sensor_msgs/msg/point_cloud2.hpp>

class random_forest : public rclcpp::Node {
    void RandomMapGenerate();

    void RandomMapGenerateCylinder();

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::random_device rd;
    std::default_random_engine eng;
    std::uniform_real_distribution<double> rand_x;
    std::uniform_real_distribution<double> rand_y;
    std::uniform_real_distribution<double> rand_w;
    std::uniform_real_distribution<double> rand_h;
    std::uniform_real_distribution<double> rand_inf;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clicked_points_publisher_;

    std::vector<double> _state;

    int _obs_num;
    double _x_size, _y_size, _z_size;
    double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
    double _z_limit, _sensing_range, _resolution, _init_x, _init_y;
    double _min_dist;

    bool _map_ok = false;
    bool _has_odom = false;

    int circle_num_;
    double radius_l_, radius_h_, z_l_, z_h_;
    double theta_;
    std::uniform_real_distribution<double> rand_radius_;
    std::uniform_real_distribution<double> rand_radius2_;
    std::uniform_real_distribution<double> rand_theta_;
    std::uniform_real_distribution<double> rand_z_;

    sensor_msgs::msg::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;

    sensor_msgs::msg::PointCloud2 localMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

public:
    random_forest();

    ~random_forest() override = default;

    void pubSensedPoints();

    double _sense_rate;
};


#endif //RANDOM_FOREAST_H
