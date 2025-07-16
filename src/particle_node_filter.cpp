#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <random>     // per il generatore casuale
#include <cmath>  
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tuple>
#include <algorithm>
#include <chrono>    // per std::chrono::milliseconds
#include <iterator>  // per std::distance
#include <functional> // per std::bind e std::placeholders


struct Particle {
    double x, y, theta, weight;
};


class ParticleFilterNode : public rclcpp::Node {
public:

    ParticleFilterNode() : Node("particle_filter_node") {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&ParticleFilterNode::mapCallback, this, std::placeholders::_1));
        particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particles", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1)
        );
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ParticleFilterNode::scanCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "🚀 ParticleFilterNode ready.");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 5 Hz ad esempio
            std::bind(&ParticleFilterNode::publishParticlesAsPoseArray, this)
        );
        
    }

private:
        cv::Mat map_img_;
        double  map_resolution_;
        double  map_origin_x_, map_origin_y_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        cv::Mat rasterizeScan(const std::vector<std::pair<double,double>>& scan, double theta);
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        cv::Mat dmap_;
        bool dmap_computed_ = false;
        std::vector<Particle> particles;
        bool particles_initialized_ = false;
        rclcpp::TimerBase::SharedPtr timer_;
        double prev_x_ = 0.0, prev_y_ = 0.0, prev_theta_ = 0.0;
        bool has_prev_odom_ = false;

        // Stato per inizializzazione da scan
        bool map_received_           = false;
        sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

        // Parametri di init da scan matching
        int    K_candidates_      = 5;     // numero di orientamenti × top-K
        int    N_per_candidate_   = 20;    // particelle per candidato
        double sigma_init_        = 0.2;   // deviazione per il jitter iniziale
        std::mt19937 gen_{std::random_device{}()};

        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;

        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            if (dmap_computed_) return;  // Blocca dopo il primo calcolo

            int width = msg->info.width;
            int height = msg->info.height;

            cv::Mat binary_map(height, width, CV_8UC1);

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int i = x + (height - 1 - y) * width;
                    int val = msg->data[i];
                    binary_map.at<uchar>(y, x) = (val >= 50) ? 0 : 255;
                }
            }

            map_img_        = binary_map.clone();
            map_resolution_ = msg->info.resolution;
            map_origin_x_   = msg->info.origin.position.x;
            map_origin_y_   = msg->info.origin.position.y;

            // Calcola la Distance Map
            cv::Mat bin_thresh;
            cv::threshold(binary_map, bin_thresh, 128, 1, cv::THRESH_BINARY);
            cv::distanceTransform(bin_thresh, dmap_, cv::DIST_L2, 3);
            dmap_.convertTo(dmap_, CV_32F);

            dmap_computed_ = true;
            RCLCPP_INFO(this->get_logger(), "📏 DMap calcolata (una volta sola): %dx%d", dmap_.cols, dmap_.rows);
            map_received_ = true;
            //initializeParticlesNear(351,251,0,4000); 
            initializeParticles(2000);

        }

        
        void publishParticlesAsPoseArray() {
            if (!particles_initialized_) return;
        
            geometry_msgs::msg::PoseArray pose_array;
            pose_array.header.stamp = this->get_clock()->now();
            pose_array.header.frame_id = "map";
        
            for (const auto& p : particles) {
                geometry_msgs::msg::Pose pose;
                pose.position.x = p.x;
                pose.position.y = p.y;
                pose.position.z = 0.0;
        
                tf2::Quaternion q;
                q.setRPY(0, 0, p.theta);
                pose.orientation = tf2::toMsg(q);
        
                pose_array.poses.push_back(pose);
            }
        
            particles_pub_->publish(pose_array);
            RCLCPP_INFO(this->get_logger(), "📤 Published %zu particles as PoseArray.", particles.size());
        }

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            if (!particles_initialized_) return;
        
            // Estrai odometria corrente
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
            const auto& q = msg->pose.pose.orientation;
            double siny = 2 * (q.w * q.z + q.x * q.y);
            double cosy = 1 - 2 * (q.y*q.y + q.z*q.z);
            double theta = std::atan2(siny, cosy);
        
            // Primo callback: solo inizializza prev_
            if (!has_prev_odom_) {
                prev_x_ = x;  prev_y_ = y;  prev_theta_ = theta;
                has_prev_odom_ = true;
                return;
            }
        
            // Calcola delta traslazione e rotazione
            double dx = x - prev_x_;
            double dy = y - prev_y_;
            double delta_trans = std::sqrt(dx*dx + dy*dy);
            double delta_theta = theta - prev_theta_;
        
            // Parametri di rumore (da tarare a piacere)
            double sigma_trans = 0.05 * delta_trans;   // es. 5% della traslazione
            double sigma_rot   = 0.02 * std::fabs(delta_theta); // es. 2% della rotazione
        
            // Generatore random (meglio tenerlo come membro di classe per efficienza)
            static std::mt19937 gen(std::random_device{}());
            std::normal_distribution<double> noise_trans(0.0, sigma_trans);
            std::normal_distribution<double> noise_rot(0.0, sigma_rot);
        
            // Applica motion update con rumore
            for (size_t i = 0; i < particles.size(); ++i) {
                auto& p = particles[i];
        
                // campiona rumore
                double noisy_trans = delta_trans + noise_trans(gen);
                double noisy_rot   = delta_theta + noise_rot(gen);
        
                // muovi la particella
                p.x     += noisy_trans * std::cos(p.theta);
                p.y     += noisy_trans * std::sin(p.theta);
                p.theta += noisy_rot;
                p.theta = std::atan2(std::sin(p.theta), std::cos(p.theta));  // normalizza
        
                double theta_deg = p.theta * 180.0 / M_PI;
                RCLCPP_DEBUG(this->get_logger(),
                             "📍 Particle[%zu]: Δtrans=%.3f, Δθ=%.3f → θ=%.2f°",
                             i, noisy_trans, noisy_rot, theta_deg);
            }
        
            // Aggiorna odometria precedente
            prev_x_ = x;
            prev_y_ = y;
            prev_theta_ = theta;
        
            RCLCPP_INFO(this->get_logger(),
                        "🔄 Predicted particles using odometry+noise. Δtrans=%.3f Δθ=%.3f",
                        delta_trans, delta_theta);
        }
        
        bool is_scan_informative(const sensor_msgs::msg::LaserScan& scan) {
            int valid = std::count_if(scan.ranges.begin(), scan.ranges.end(),
                [&](float r) { return r < scan.range_max * 0.95; });
            return valid > 3;
        }
      
        std::vector<float> simulateLaserFromPose(
            double x, double y, double theta,
            const sensor_msgs::msg::LaserScan& scan,
            float step_size = 1.0f,
            float dmap_hit_thresh = 1.0f) {
        
            std::vector<float> simulated_ranges;
            if (dmap_.empty()) return simulated_ranges;
        
            float angle = scan.angle_min;
            for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
                float beam_angle = theta + angle;
                float dist = scan.range_max;
        
                for (float d = 0.0f; d <= scan.range_max; d += step_size) {
                    float beam_x = x + d * std::cos(beam_angle);
                    float beam_y = y + d * std::sin(beam_angle);
        
                    int px = static_cast<int>(beam_x);
                    int py = static_cast<int>(beam_y);
        
                    if (px < 0 || py < 0 || px >= dmap_.cols || py >= dmap_.rows) {
                        dist = d;
                        break;
                    }
        
                    float dmap_val = dmap_.at<float>(py, px);
                    if (dmap_val < dmap_hit_thresh) {
                        dist = d;
                        break;
                    }
                }
        
                simulated_ranges.push_back(dist);
            }
        
            return simulated_ranges;
        }
        
        std::vector<std::pair<double,double>> preprocessScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
            std::vector<std::pair<double,double>> clean;
            for (size_t i = 0; i < scan->ranges.size(); ++i) {
              double r = scan->ranges[i];
              double ang = scan->angle_min + i * scan->angle_increment;
              if (r >= scan->range_min && r <= scan->range_max && std::isfinite(r)) {
                clean.emplace_back(r, ang);
              }
            }
            return clean;
          }

          void initializeParticlesNear(double center_x, double center_y, double center_theta, int num_particles) {
            std::random_device rd;
            std::mt19937 gen(rd());
        
            std::normal_distribution<double> dist_x(center_x, 10.0);       // ±10 pixel
            std::normal_distribution<double> dist_y(center_y, 10.0);
        
            // Orientamenti cardinali (dx, giù, sx, su)
            std::vector<double> fixed_orientations = {0.0, M_PI_2, M_PI, -M_PI_2};
            std::uniform_int_distribution<> fixed_theta_dist(0, fixed_orientations.size() - 1);
        
            particles.clear();
            double weight = 1.0 / num_particles;
        
            for (int i = 0; i < num_particles; ++i) {
                Particle p;
                p.x = dist_x(gen);
                p.y = dist_y(gen);
                p.theta = fixed_orientations[fixed_theta_dist(gen)];
                p.weight = weight;
        
                // Controlla se la cella è libera
                int px = static_cast<int>(p.x);
                int py = static_cast<int>(p.y);
                if (px >= 0 && px < dmap_.cols && py >= 0 && py < dmap_.rows && dmap_.at<float>(py, px) > 0.0f) {
                    particles.push_back(p);
                }
            }
        
            particles_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "🎯 Particelle inizializzate (cardinali) attorno a (%.1f, %.1f)", 
                        center_x, center_y);
        }
        void initializeParticles(int num_particles) {
            if (dmap_.empty()) {
                RCLCPP_WARN(this->get_logger(), "⚠️ DMap not ready. Cannot initialize particles.");
                return;
            }
        
            std::vector<cv::Point> free_cells;
        
            for (int y = 0; y < dmap_.rows; ++y) {
                for (int x = 0; x < dmap_.cols; ++x) {
                    if (dmap_.at<float>(y, x) > 0.0f) {  // cella libera = distanza > 0
                        free_cells.emplace_back(x, y);
                    }
                }
            }
        
            if (free_cells.empty()) {
                RCLCPP_ERROR(this->get_logger(), "❌ No free cells found in map.");
                return;
            }
        
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> idx_dist(0, free_cells.size() - 1);

            std::vector<double> fixed_orientations = {0, M_PI_2, M_PI, -M_PI_2};  // dx, giù, sx, su
            std::uniform_int_distribution<> orientation_idx(0, fixed_orientations.size() - 1);

            particles.clear();
            double weight = 1.0 / num_particles;
        
            for (int i = 0; i < num_particles; ++i) {
                cv::Point p = free_cells[idx_dist(gen)];
                Particle part;
                part.x = p.x;
                part.y = p.y;
                part.theta = fixed_orientations[orientation_idx(gen)];
                part.weight = weight;
                particles.push_back(part);
            }
        
            RCLCPP_INFO(this->get_logger(), "✅ Initialized %d particles.", num_particles);
            particles_initialized_ = true;
        }

        std::vector<std::tuple<double,double,double>> correlativeScanMatching(
            const std::vector<std::pair<double,double>>& scan, int K) {
          std::vector<std::tuple<double,double,double>> candidates;
          for (double theta_test : { -M_PI/6.0, -M_PI/12.0, 0.0, M_PI/12.0, M_PI/6.0 }) {
            cv::Mat scan_img = rasterizeScan(scan, theta_test);
            cv::Mat result;
            cv::matchTemplate(scan_img, map_img_, result, cv::TM_CCOEFF_NORMED);
            for (int k = 0; k < K; ++k) {
              double maxVal; cv::Point maxLoc;
              cv::minMaxLoc(result, nullptr, &maxVal, nullptr, &maxLoc);
              double wx = maxLoc.x;
              double wy = maxLoc.y;
              candidates.emplace_back(wx, wy, theta_test);
              // Sopprimi il picco
              cv::circle(result, maxLoc, 5, cv::Scalar(0), -1);
            }
          }
          // Ordina per score se li memorizzi, altrimenti restituisci in ordine
          return candidates;
        }
        

        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
            if (!particles_initialized_ || dmap_.empty())
                return;

                    
            // 1. Controllo informatività
            int valid = std::count_if(scan->ranges.begin(), scan->ranges.end(),
                                      [&](float r) { return r < scan->range_max * 0.95; });
            if (valid <= 3) {
                RCLCPP_WARN(this->get_logger(), "📡 Scan non informativo (%d valid beams). Skipping update.", valid);
                return;
            }
        
            // 2. Simulazione e pesatura
            double sigma = 10.0;
            for (auto& p : particles) {
                std::vector<float> z_sim = simulateLaserFromPose(p.x, p.y, p.theta, *scan);        
                double rmse = 0.0;
                int count = std::min(z_sim.size(), scan->ranges.size());
                for (int i = 0; i < count; ++i) {
                    float z_real = scan->ranges[i];
                    float z_est = z_sim[i];
                    rmse += std::pow(z_real - z_est, 2);
                }
                rmse = std::sqrt(rmse / count);
                p.weight = std::exp(- (rmse * rmse) / (2 * sigma * sigma));
            }
        
            // 3. Normalizza i pesi
            double total_weight = 0.0;
            for (const auto& p : particles) total_weight += p.weight;
            if (total_weight > 0.0) {
                for (auto& p : particles) p.weight /= total_weight;
            } else {
                RCLCPP_WARN(this->get_logger(), "❌ Somma dei pesi nulla dopo aggiornamento.");
                return;
            }
        
            // 4. Controlla varianza → trigger resampling
            double weight_sq_sum = 0.0;
            for (const auto& p : particles) weight_sq_sum += p.weight * p.weight;
            double neff = 1.0 / weight_sq_sum;
        
            if (neff < particles.size() * 0.5) {
                resampleParticles();
                RCLCPP_INFO(this->get_logger(), "🔁 Resample triggered (Neff = %.2f)", neff);
            } else {
                RCLCPP_INFO(this->get_logger(), "✅ Skip resample (Neff = %.2f)", neff);
            }
        
            // (opzionale) pubblica il nuovo PoseArray delle particelle
            publishParticlesAsPoseArray();
        }
        
        void resampleParticles() {
            if (particles.empty()) return;
            const size_t N = particles.size();
            const size_t inject_count = static_cast<size_t>(0.05 * N);
            const size_t keep_count   = N - inject_count;
        
            // 1) Calcola Neff e salta se troppo alto
            double weight_sq_sum = 0.0;
            for (const auto &p : particles) weight_sq_sum += p.weight * p.weight;
            double neff = (weight_sq_sum > 0) ? 1.0 / weight_sq_sum : 0.0;
            if (neff >= 0.3 * N) {
                RCLCPP_INFO(get_logger(), "↪️ Skip resample (Neff=%.1f >= %.1f)", neff, 0.3 * N);
                return;
            }
        
            // 2) Trova la particella con peso massimo ("best")
            auto best_it = std::max_element(
                particles.begin(), particles.end(),
                [](const Particle &a, const Particle &b){ return a.weight < b.weight; }
            );
            Particle best = *best_it;
        
            // 3) Costruisci cumulativa
            std::vector<double> cumulative(N);
            cumulative[0] = particles[0].weight;
            for (size_t i = 1; i < N; ++i) cumulative[i] = cumulative[i-1] + particles[i].weight;
            double total = cumulative.back();
            if (total <= 0.0) {
                RCLCPP_WARN(get_logger(), "⚠️ Pesi tutti nulli, salto resampling");
                return;
            }
        
            // 4) RNG & distribuzioni
            std::mt19937 gen(std::random_device{}());
            std::uniform_real_distribution<double> uni_dist(0.0, total);
            std::normal_distribution<double> noise_pos(5.0, 10.0);    // ±0.2 pixel
        
            // jitter injection for position around best
            std::normal_distribution<double> inj_x(best.x, 2.0);
            std::normal_distribution<double> inj_y(best.y, 2.0);
    
            auto normalize_angle = [](double a) {
                return std::atan2(std::sin(a), std::cos(a));
            };
            auto isFree = [&](double x, double y) {
                int xi = static_cast<int>(std::round(x));
                int yi = static_cast<int>(std::round(y));
                if (xi < 0 || yi < 0 || xi >= dmap_.cols || yi >= dmap_.rows) return false;
                return dmap_.at<float>(yi, xi) > 0.0f;
            };
        
            // 5) Resampling ponderato + jitter su posizione
            std::vector<Particle> new_particles;
            new_particles.reserve(N);
            for (size_t i = 0; i < keep_count; ++i) {
                double r = uni_dist(gen);
                auto it = std::lower_bound(cumulative.begin(), cumulative.end(), r);
                size_t idx = std::distance(cumulative.begin(), it);
        
                Particle p = particles[idx];
                // jitter solo su posizione
                p.x = p.x + noise_pos(gen);
                p.y = p.y + noise_pos(gen);
                p.theta = normalize_angle(p.theta);
        
                if (!isFree(p.x, p.y)) {
                    p.x = best.x + noise_pos(gen);
                    p.y = best.y + noise_pos(gen);
                }
                p.weight = 1.0 / N;
                new_particles.push_back(p);
            }
        
            // 6) Injection attorno al best: posizione jitter, orientamento best
            for (size_t k = 0; k < inject_count; ++k) {
                Particle p;
                p.x = inj_x(gen);
                p.y = inj_y(gen);
                p.theta = normalize_angle(best.theta);  // orientamento fisso sul best
                if (!isFree(p.x, p.y)) {
                    p.x = best.x + noise_pos(gen);
                    p.y = best.y + noise_pos(gen);
                }
                p.weight = 1.0 / N;
                new_particles.push_back(p);
            }
        
            // 7) Sostituisci
            particles = std::move(new_particles);
            RCLCPP_INFO(get_logger(),
                "🔁 Resampling: kept=%zu + injected=%zu tot=%zu (Neff=%.1f)",
                keep_count, inject_count, particles.size(), neff);
        }
        
};

cv::Mat ParticleFilterNode::rasterizeScan(
    const std::vector<std::pair<double,double>>& scan, double theta)
{
    const int T = 200;
    cv::Mat img = cv::Mat::zeros(T, T, CV_8UC1);
    cv::Point center(T/2, T/2);
    for (auto& beam : scan) {
        double r = beam.first, a = beam.second;
        double ang = a + theta;
        int dx = static_cast<int>(r * std::cos(ang) / map_resolution_);
        int dy = static_cast<int>(r * std::sin(ang) / map_resolution_);
        cv::Point pt = center + cv::Point(dx, -dy);
        if (pt.inside(cv::Rect(0,0,T,T)))
            img.at<uchar>(pt) = 255;
    }
    return img;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ParticleFilterNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}