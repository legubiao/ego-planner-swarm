//
// Created by biao on 24-8-16.
//

#include <map_generator/random_forest.h>
#include <pcl_conversions/pcl_conversions.h>

void random_forest::RandomMapGenerate() {
    pcl::PointXYZ pt_random;

    rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
    rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
    rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
    rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);

    rand_radius_ = std::uniform_real_distribution<double>(radius_l_, radius_h_);
    rand_radius2_ = std::uniform_real_distribution<double>(radius_l_, 1.2);
    rand_theta_ = std::uniform_real_distribution<double>(-theta_, theta_);
    rand_z_ = std::uniform_real_distribution<double>(z_l_, z_h_);

    // generate polar obs
    for (int i = 0; i < _obs_num; i++) {
        double x = rand_x(eng);
        double y = rand_y(eng);
        double w = rand_w(eng);

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil(w / _resolution);

        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                double h = rand_h(eng);
                int heiNum = ceil(h / _resolution);
                for (int t = -20; t < heiNum; t++) {
                    pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
                    pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
                    pt_random.z = (t + 0.5) * _resolution + 1e-2;
                    cloudMap.points.push_back(pt_random);
                }
            }
    }

    // generate circle obs
    for (int i = 0; i < circle_num_; ++i) {
        double x, y, z;
        x = rand_x(eng);
        y = rand_y(eng);
        z = rand_z_(eng);

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        z = floor(z / _resolution) * _resolution + _resolution / 2.0;

        Eigen::Vector3d translate(x, y, z);

        double theta = rand_theta_(eng);
        Eigen::Matrix3d rotate;
        rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
                1;

        double radius1 = rand_radius_(eng);
        double radius2 = rand_radius2_(eng);

        // draw a circle centered at (x,y,z)
        Eigen::Vector3d cpt;
        for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
            cpt(0) = 0.0;
            cpt(1) = radius1 * cos(angle);
            cpt(2) = radius2 * sin(angle);

            // inflate
            Eigen::Vector3d cpt_if;
            for (int ifx = -0; ifx <= 0; ++ifx)
                for (int ify = -0; ify <= 0; ++ify)
                    for (int ifz = -0; ifz <= 0; ++ifz) {
                        cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                                       ifz * _resolution);
                        cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
                        pt_random.x = cpt_if(0);
                        pt_random.y = cpt_if(1);
                        pt_random.z = cpt_if(2);
                        cloudMap.push_back(pt_random);
                    }
        }
    }
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    RCLCPP_INFO(get_logger(), "Finished generate random map ");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}

void random_forest::RandomMapGenerateCylinder() {
    pcl::PointXYZ pt_random;

    std::vector<Eigen::Vector2d> obs_position;

    rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
    rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
    rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
    rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);
    rand_inf = std::uniform_real_distribution<double>(0.5, 1.5);

    rand_radius_ = std::uniform_real_distribution<double>(radius_l_, radius_h_);
    rand_radius2_ = std::uniform_real_distribution<double>(radius_l_, 1.2);
    rand_theta_ = std::uniform_real_distribution<double>(-theta_, theta_);
    rand_z_ = std::uniform_real_distribution<double>(z_l_, z_h_);

    // generate polar obs
    for (int i = 0; i < _obs_num && rclcpp::ok(); i++) {
        double x, y, w, h, inf;
        x = rand_x(eng);
        y = rand_y(eng);
        w = rand_w(eng);
        inf = rand_inf(eng);

        bool flag_continue = false;
        for (auto p: obs_position)
            if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/) {
                i--;
                flag_continue = true;
                break;
            }
        if (flag_continue) continue;

        obs_position.push_back(Eigen::Vector2d(x, y));


        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil((w * inf) / _resolution);
        double radius = (w * inf) / 2;

        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                h = rand_h(eng);
                int heiNum = ceil(h / _resolution);
                for (int t = -20; t < heiNum; t++) {
                    double temp_x = x + (r + 0.5) * _resolution + 1e-2;
                    double temp_y = y + (s + 0.5) * _resolution + 1e-2;
                    double temp_z = (t + 0.5) * _resolution + 1e-2;
                    if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius) {
                        pt_random.x = temp_x;
                        pt_random.y = temp_y;
                        pt_random.z = temp_z;
                        cloudMap.points.push_back(pt_random);
                    }
                }
            }
    }

    // generate circle obs
    for (int i = 0; i < circle_num_; ++i) {
        double x, y, z;
        x = rand_x(eng);
        y = rand_y(eng);
        z = rand_z_(eng);

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        z = floor(z / _resolution) * _resolution + _resolution / 2.0;

        Eigen::Vector3d translate(x, y, z);

        double theta = rand_theta_(eng);
        Eigen::Matrix3d rotate;
        rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
                1;

        double radius1 = rand_radius_(eng);
        double radius2 = rand_radius2_(eng);

        // draw a circle centered at (x,y,z)
        Eigen::Vector3d cpt;
        for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
            cpt(0) = 0.0;
            cpt(1) = radius1 * cos(angle);
            cpt(2) = radius2 * sin(angle);

            // inflate
            Eigen::Vector3d cpt_if;
            for (int ifx = -0; ifx <= 0; ++ifx)
                for (int ify = -0; ify <= 0; ++ify)
                    for (int ifz = -0; ifz <= 0; ++ifz) {
                        cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                                       ifz * _resolution);
                        cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
                        pt_random.x = cpt_if(0);
                        pt_random.y = cpt_if(1);
                        pt_random.z = cpt_if(2);
                        cloudMap.push_back(pt_random);
                    }
        }
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    RCLCPP_INFO(get_logger(), "Finished generate random map ");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}

random_forest::random_forest() : Node("random_map_sensing") {
    local_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_cloud", 1);
    all_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("global_cloud", 1);

    declare_parameter("init_state_x", 0.0);
    get_parameter("init_state_x", _init_x);
    declare_parameter("init_state_y", 0.0);
    get_parameter("init_state_y", _init_y);

    declare_parameter("map/x_size", 50.0);
    get_parameter("map/x_size", _x_size);
    declare_parameter("map/y_size", 50.0);
    get_parameter("map/y_size", _y_size);
    declare_parameter("map/z_size", 5.0);
    get_parameter("map/z_size", _z_size);
    declare_parameter("map/obs_num", 30);
    get_parameter("map/obs_num", _obs_num);
    declare_parameter("map/resolution", 0.1);
    get_parameter("map/resolution", _resolution);
    declare_parameter("map/circle_num", 30);
    get_parameter("map/circle_num", circle_num_);

    declare_parameter("ObstacleShape/lower_rad", 0.3);
    get_parameter("ObstacleShape/lower_rad", _w_l);
    declare_parameter("ObstacleShape/upper_rad", 0.8);
    get_parameter("ObstacleShape/upper_rad", _w_h);
    declare_parameter("ObstacleShape/lower_hei", 3.0);
    get_parameter("ObstacleShape/lower_hei", _h_l);
    declare_parameter("ObstacleShape/upper_hei", 7.0);
    get_parameter("ObstacleShape/upper_hei", _h_h);

    declare_parameter("ObstacleShape/radius_l", 7.0);
    get_parameter("ObstacleShape/radius_l", radius_l_);
    declare_parameter("ObstacleShape/radius_h", 7.0);
    get_parameter("ObstacleShape/radius_h", radius_h_);
    declare_parameter("ObstacleShape/z_l", 7.0);
    get_parameter("ObstacleShape/z_l", z_l_);
    declare_parameter("ObstacleShape/z_h", 7.0);
    get_parameter("ObstacleShape/z_h", z_h_);
    declare_parameter("ObstacleShape/theta", 7.0);
    get_parameter("ObstacleShape/theta", theta_);

    declare_parameter("sensing/radius", 10.0);
    get_parameter("sensing/radius", _sensing_range);
    declare_parameter("sensing/rate", 10.0);
    get_parameter("sensing/rate", _sense_rate);

    declare_parameter("min_distance", 1.0);
    get_parameter("min_distance", _min_dist);


    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;

    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;
    _obs_num = std::min(_obs_num, static_cast<int>(_x_size) * 10);
    _z_limit = _z_size;

    const unsigned int seed = rd();
    RCLCPP_INFO(get_logger(), "Random seed: %d", seed);
    eng.seed(seed);

    RandomMapGenerateCylinder();

    rclcpp::Rate loop_rate(_sense_rate);
}

void random_forest::pubSensedPoints() {
    toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    all_map_publisher_->publish(globalMap_pcd);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<random_forest>();
    rclcpp::Rate loop_rate(node->_sense_rate);

    while (rclcpp::ok()) {
        node->pubSensedPoints();
        spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
