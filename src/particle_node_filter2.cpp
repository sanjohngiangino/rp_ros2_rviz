#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/opencv.hpp>
#include <random>
#include <Eigen/Dense>
#include <unordered_set>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

struct Particle {
    double x, y, theta;
    double weight;
};

class ParticleFilterNode : public rclcpp::Node {
public:
    ParticleFilterNode()
    : Node("particle_node_filter")
      , rd_()
      , gen_(rd_())
      , dist_uni_(0.0, 1.0)
      , tf_buffer_(this->get_clock())
      , tf_listener_(tf_buffer_)
    {
        // Parametri
        this->declare_parameter<int>("num_particles_max", 2000);
        this->declare_parameter<int>("num_particles_min", 100);
        this->declare_parameter<double>("sensor_sigma", 0.2);
        this->declare_parameter<double>("odom_alpha1", 0.1);
        this->declare_parameter<double>("odom_alpha2", 0.1);
        this->declare_parameter<double>("odom_alpha3", 0.1);
        this->declare_parameter<double>("odom_alpha4", 0.1);
        this->declare_parameter<double>("kld_epsilon", 0.05);
        this->declare_parameter<double>("kld_delta", 0.99);
        this->get_parameter("num_particles_max", N_max_);
        this->get_parameter("num_particles_min", N_min_);
        this->get_parameter("sensor_sigma", sigma_s_);
        this->get_parameter("odom_alpha1", alpha1_);
        this->get_parameter("odom_alpha2", alpha2_);
        this->get_parameter("odom_alpha3", alpha3_);
        this->get_parameter("odom_alpha4", alpha4_);
        this->get_parameter("kld_epsilon", eps_);
        this->get_parameter("kld_delta", delta_);
        dt_ = 0.1;

        // Subscriber e Publisher
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1),
            std::bind(&ParticleFilterNode::mapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10),
            std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::QoS(10),
            std::bind(&ParticleFilterNode::scanCallback, this, std::placeholders::_1));
        particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particles", rclcpp::QoS(10));

        initialized_ = false;
    }

private:
    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = *msg;
        buildDistanceField();
        globalInitialize();
        initialized_ = true;
        publishParticles();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!initialized_) return;
        double dx = msg->twist.twist.linear.x * dt_;
        double dth = msg->twist.twist.angular.z * dt_;
        motionUpdate(dx, dth);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (!initialized_) return;
        sensorUpdate(*scan);
        resample();
        publishParticles();
    }

    // Utilities
    void buildDistanceField() {
        int w = map_.info.width;
        int h = map_.info.height;
        cv::Mat occ(h, w, CV_8UC1);
        for(int y = 0; y < h; y++){
            for(int x = 0; x < w; x++){
                int idx = x + y * w;
                occ.at<unsigned char>(y, x) = (map_.data[idx] > 50 ? 0 : 255);
            }
        }
        cv::distanceTransform(occ, dmap_, cv::DIST_L2, 5);
        dmap_ *= map_.info.resolution;
    }

    void globalInitialize() {
        N_ = N_max_;
        particles_.resize(N_);
        std::uniform_int_distribution<int> idx_w(0, map_.info.width - 1);
        std::uniform_int_distribution<int> idx_h(0, map_.info.height - 1);
        int count = 0;
        while(count < N_) {
            int xi = idx_w(gen_);
            int yi = idx_h(gen_);
            if(map_.data[xi + yi * map_.info.width] == 0) {
                particles_[count].x = map_.info.origin.position.x + xi * map_.info.resolution;
                particles_[count].y = map_.info.origin.position.y + yi * map_.info.resolution;
                particles_[count].theta = dist_uni_(gen_) * 2*M_PI - M_PI;
                particles_[count].weight = 1.0 / N_;
                count++;
            }
        }
    }

    void motionUpdate(double delta_trans, double delta_theta) {
        std::normal_distribution<double> noise_trans(alpha1_ * std::fabs(delta_trans));
        std::normal_distribution<double> noise_rot(alpha3_ * std::fabs(delta_theta));
        for(auto &p : particles_) {
            double nt = delta_trans + noise_trans(gen_);
            p.x += nt * std::cos(p.theta);
            p.y += nt * std::sin(p.theta);
            p.theta += delta_theta + noise_rot(gen_);
            p.weight = 1.0 / N_;
        }
    }

    void sensorUpdate(const sensor_msgs::msg::LaserScan &scan) {
        for(auto &p : particles_) {
            double w = 1.0;
            double ang = scan.angle_min;
            for(double r : scan.ranges) {
                if(std::isfinite(r)) {
                    double rx = p.x + r * std::cos(p.theta + ang);
                    double ry = p.y + r * std::sin(p.theta + ang);
                    int mx = static_cast<int>((rx - map_.info.origin.position.x) / map_.info.resolution);
                    int my = static_cast<int>((ry - map_.info.origin.position.y) / map_.info.resolution);
                    if(mx >= 0 && mx < dmap_.cols && my >= 0 && my < dmap_.rows) {
                        float dist = dmap_.at<float>(my, mx);
                        w *= std::exp(-(dist*dist) / (2 * sigma_s_ * sigma_s_));
                    }
                }
                ang += scan.angle_increment;
            }
            p.weight = w;
        }
        double sum = 0;
        for(auto &p : particles_) sum += p.weight;
        for(auto &p : particles_) p.weight /= sum;
    }

    void resample() {
        int K = countOccupiedBins();
        double z = chi2inv(1 - delta_, (K > 1 ? K - 1 : 1));
        int N_req = static_cast<int>(std::ceil(z / (2 * eps_)));
        N_ = std::min(std::max(N_req, N_min_), N_max_);

        std::vector<double> cdf(particles_.size());
        cdf[0] = particles_[0].weight;
        for(size_t i = 1; i < particles_.size(); ++i)
            cdf[i] = cdf[i-1] + particles_[i].weight;

        std::vector<Particle> newp(N_);
        double r = dist_uni_(gen_) / N_;
        size_t idx = 0;
        for(int i = 0; i < N_; ++i) {
            double u = r + i * (1.0 / N_);
            while(u > cdf[idx]) ++idx;
            newp[i] = particles_[idx];
            newp[i].weight = 1.0 / N_;
        }
        particles_.swap(newp);
    }

    int countOccupiedBins() {
        std::unordered_set<long> bins;
        for(auto &p : particles_) {
            int bx = static_cast<int>(std::floor(p.x));
            int by = static_cast<int>(std::floor(p.y));
            long id = (static_cast<long>(bx) << 32) | static_cast<unsigned long>(by);
            bins.insert(id);
        }
        return static_cast<int>(bins.size());
    }

    double chi2inv(double /*p*/, int v) {
        return static_cast<double>(v);
    }

    void publishParticles() {
        auto arr = geometry_msgs::msg::PoseArray();
        arr.header.stamp = this->now();
        arr.header.frame_id = map_.header.frame_id;
        double ox = map_.info.origin.position.x;
        double oy = map_.info.origin.position.y;
        double res = map_.info.resolution;
        int h = map_.info.height;
        for(auto &p : particles_) {
            int px = static_cast<int>(std::floor((p.x - ox) / res));
            int py = h - 1 - static_cast<int>(std::floor((p.y - oy) / res));
            geometry_msgs::msg::Pose ps;
            ps.position.x = px;
            ps.position.y = py;
            tf2::Quaternion q;
            q.setRPY(0, 0, p.theta);
            ps.orientation.x = q.x();
            ps.orientation.y = q.y();
            ps.orientation.z = q.z();
            ps.orientation.w = q.w();
            arr.poses.push_back(ps);
        }
        particles_pub_->publish(arr);
    }

    // Membri
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
    nav_msgs::msg::OccupancyGrid map_;
    cv::Mat dmap_;
    bool initialized_;
    std::vector<Particle> particles_;
    int N_, N_max_, N_min_;
    double sigma_s_, alpha1_, alpha2_, alpha3_, alpha4_, eps_, delta_, dt_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_uni_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
