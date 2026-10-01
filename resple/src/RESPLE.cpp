#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <bondcpp/bond.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <atomic>
#include <chrono>
#include <queue>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <boost/make_shared.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/empty.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "livox_interfaces/msg/custom_msg.hpp"
#include "livox_ros_driver/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "estimate_msgs/msg/calib.hpp"
#include "estimate_msgs/msg/spline.hpp"
#include "estimate_msgs/msg/estimate.hpp"
#include "Estimator.h"

// Global rather than a RESPLE member because ikd-Tree spawns its own
// background rebuild thread tied to this object's lifetime, independent of
// RESPLE's own ROS lifecycle state. See the guard in on_activate() for why a
// second Build() in the same process is refused rather than attempted.
KD_TREE<pcl::PointXYZINormal> ikdtree;

class RESPLE : public rclcpp_lifecycle::LifecycleNode
{

public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit RESPLE(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("RESPLE", options)
    {
        bond_topic_name_ = this->declare_parameter<std::string>("bond_topic_name", "/bond");
        bond_id_ = this->declare_parameter<std::string>("bond_id", "resple");
        enable_bond_ = this->declare_parameter<bool>("enable_bond", true);
        bond_heartbeat_period_s_ = this->declare_parameter<double>("bond_heartbeat_period_s", 0.1);
        bond_heartbeat_timeout_s_ = this->declare_parameter<double>("bond_heartbeat_timeout_s", 1.0);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        readParameters();
        if (!if_lidar_only) {
            std::string imu_type = CommonUtils::readParam<std::string>(*this, "topic_imu");
            sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(imu_type, 2000000, std::bind(&RESPLE::getImuCallback, this, std::placeholders::_1));
        }
        pub_est = this->create_publisher<estimate_msgs::msg::Estimate>("est_window", 50);
        // transient_local so a late-joining subscriber (e.g. a freshly
        // respawned glim_ros checking "how long has resple actually been
        // running" before it starts consuming its stream) still gets this
        // one-shot value, even though it was published long before that
        // subscriber existed.
        pub_start_time = this->create_publisher<std_msgs::msg::Int64>(
            "start_time", rclcpp::QoS(1).transient_local());
        pub_cur_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>("current_scan", 2);
        br = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        auto lidar_names = this->declare_parameter<std::vector<std::string>>("lidars", std::vector<std::string>());
        assert(this->get_parameter({"lidars"}, lidar_names));
        if (lidar_names.empty()) {
            LidarConfig lidar(*this, "");
            lidars.emplace(lidar.type, lidar);
            lidars_data.emplace(std::piecewise_construct, std::make_tuple(lidar.type), std::make_tuple());
        } else {
            for (const auto& lidar_name : lidar_names) {
                LidarConfig lidar(*this, lidar_name + ".");
                lidars.emplace(lidar.type, lidar);
                lidars_data.emplace(std::piecewise_construct, std::make_tuple(lidar.type), std::make_tuple());
            }
        }
        for (const auto& [lidar_name, lidar] : lidars) {
            if (!lidar.type.compare("Ouster")) {
                sub_ouster = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, 200000, std::bind(&RESPLE::ousterLidarCallback<ouster_ros::Point>, this, std::placeholders::_1));
            } else if (!lidar.type.compare("Mid70Avia")) {
                sub_livox = this->create_subscription<livox_ros_driver::msg::CustomMsg>(
                        lidar.topic, 200000, std::bind(&RESPLE::livoxLidarCallback, this, std::placeholders::_1));
            } else if (!lidar.type.compare("HAP360")) {
                sub_livox2 = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                        lidar.topic, 200000, std::bind(&RESPLE::livoxLidar2Callback, this, std::placeholders::_1));
            } else if (!lidar.type.compare("AviaResple")) {
                sub_livox_avia = this->create_subscription<livox_interfaces::msg::CustomMsg>(
                        lidar.topic, 200000, std::bind(&RESPLE::livoxAVIACallback, this, std::placeholders::_1));
            } else if (!lidar.type.compare("Hesai")) {
                sub_hesai = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, 200000, std::bind(&RESPLE::hesaiLidarCallback, this, std::placeholders::_1));
            } else if (!lidar.type.compare("Mid360Boxi")) {
                sub_livox_mid360_boxi = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar.topic, 200000, std::bind(&RESPLE::livoxMid360BoxiCallback, this, std::placeholders::_1));
            }
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        // ikdtree.Build() (called below via initialization() once processData()
        // starts) unconditionally frees the previous tree's nodes before
        // building the new one, but ikd-Tree's own background rebuild thread
        // (running continuously since process start, independent of our ROS
        // lifecycle state) may still be reading/rewriting that same tree. A
        // second Build() in the same process is a real use-after-free risk, not
        // just a "stale map" inconvenience -- so refuse to activate again in a
        // process that has already built the tree once. Returning FAILURE alone
        // would leave this node stuck inactive indefinitely (respawn:true only
        // triggers on the OS process actually exiting, not on a failed
        // transition) -- exit the process instead, so pulse.yaml's
        // respawn/respawn_delay gives a genuinely fresh one within seconds,
        // matching how every other unrecoverable failure in this stack is
        // already handled (crash -> respawn), rather than a new silent-stall
        // failure mode.
        if (ikdtree.Root_Node != nullptr) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Refusing to activate: this process already built the ikd-Tree "
                "local map once. Rebuilding it in place is unsafe (ikd-Tree's "
                "background rebuild thread may still be touching the old tree "
                "when Build() frees it). Exiting this process so respawn:true "
                "gives a fresh one instead of leaving this node stuck inactive.");
            rclcpp::shutdown();
            // rclcpp::shutdown() alone does not terminate the process -- main()'s
            // normal unwind would still run destructors while a still-running
            // background thread could be touching now-torn-down objects. Force
            // an immediate, real exit instead, same fix as glim_ros's identical
            // guard.
            _exit(1);
        }

        pub_est->on_activate();
        pub_start_time->on_activate();
        pub_cur_scan->on_activate();

        // A reactivation on the same still-running process (bond broke but the
        // process didn't crash) needs to redo initFilter()/build a fresh local
        // map, since if_init_filter/if_init_map gate that in processData().
        if_init_filter = false;
        if_init_map = false;
        for (auto& [lidar_name, lidar_data] : lidars_data) {
            lidar_data.mtx_pc.lock();
            lidar_data.pc_buff.clear();
            lidar_data.t_buff.clear();
            lidar_data.mtx_pc.unlock();
            lidar_data.pt_buff.clear();
        }
        pc_world.clear();
        accum_nearest_points.clear();

        processing_thread_finished_ = false;
        running_ = true;
        processing_thread_ = std::make_shared<std::thread>(&RESPLE::processData, this);

        start_bond();
        return CallbackReturn::SUCCESS;
    }

    // processData() can hang indefinitely under a still-unresolved, pre-existing
    // condition (seen with the robot stationary for a long time). Before the
    // lifecycle conversion that hang was invisible at the RPC level -- nothing
    // else was waiting on this thread. Now on_deactivate()/on_shutdown() call
    // processing_thread_->join(), which runs on the SAME executor thread that
    // services lifecycle service calls -- an unbounded join() there means a
    // stuck processData() takes the whole node's lifecycle RPCs down with it
    // (matches what was observed: even plain `ros2 lifecycle get` stopped
    // responding). Poll processing_thread_finished_ (set by processData()
    // right before it returns) with a bounded timeout instead of blocking on
    // join() directly; join() itself is still safe to call afterwards since it
    // returns immediately once the thread has actually finished.
    bool join_processing_thread_with_timeout(int timeout_ms)
    {
        if (!processing_thread_) {
            return true;
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (!processing_thread_finished_.load()) {
            if (std::chrono::steady_clock::now() >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        processing_thread_->join();
        processing_thread_.reset();
        return true;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        stop_bond();
        running_ = false;
        if (!join_processing_thread_with_timeout(5000)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "processData() did not stop within 5s of deactivate -- it is "
                "stuck (known pre-existing hang, not resolved here). Exiting "
                "this process so respawn:true gives a fresh one instead of "
                "leaving the whole node unresponsive to lifecycle calls.");
            rclcpp::shutdown();
            // Confirmed live: without a hard exit here, the still-stuck
            // processData() thread outlives rclcpp::shutdown() and the process
            // later segfaults instead of exiting cleanly (rather than just
            // hanging, which is what this guard was originally written to
            // avoid) -- so respawn:true never got a chance to kick in reliably
            // either way. Force the real exit directly.
            _exit(1);
        }

        pub_est->on_deactivate();
        pub_start_time->on_deactivate();
        pub_cur_scan->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        sub_imu.reset();
        sub_ouster.reset();
        sub_livox.reset();
        sub_livox2.reset();
        sub_livox_avia.reset();
        sub_hesai.reset();
        sub_livox_mid360_boxi.reset();
        pub_est.reset();
        pub_start_time.reset();
        pub_cur_scan.reset();
        br.reset();
        lidars.clear();
        lidars_data.clear();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        stop_bond();
        running_ = false;
        if (!join_processing_thread_with_timeout(5000)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "processData() did not stop within 5s of shutdown -- it is "
                "stuck (known pre-existing hang, not resolved here). Forcing "
                "process exit so respawn:true can recover.");
            rclcpp::shutdown();
            _exit(1);
        }
        return CallbackReturn::SUCCESS;
    }

    void processData()
    {
        rclcpp::Rate rate(20);
        int64_t max_spl_knots = 0;
        int64_t t_last_map_upd = 0;
        while (running_.load()) {
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (!lidar_data.t_buff.empty()) {
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_frame(new pcl::PointCloud<pcl::PointXYZINormal>());
                    lidar_data.mtx_pc.lock();
                    pc_frame->points = lidar_data.pc_buff.front();
                    lidar_data.pc_buff.pop_front();
                    int64_t time_begin = lidar_data.t_buff.front();
                    lidar_data.t_buff.pop_front();
                    lidar_data.mtx_pc.unlock();
                    std::vector<int> indices;
                    pcl::removeNaNFromPointCloud(*pc_frame, *pc_frame, indices);
                    pc_last_ds->clear();

                    ds_filter_body.setInputCloud(pc_frame);
                    ds_filter_body.filter(*pc_last_ds);
                    sort(pc_last_ds->points.begin(), pc_last_ds->points.end(), &CommonUtils::time_list);
                    const LidarConfig& lidar = lidars.at(lidar_name);
                    for (size_t i = 0; i < pc_last_ds->points.size(); i++) {
                        PointData pt(pc_last_ds->points[i], time_begin, lidar.q_bl, lidar.t_bl, lidar.w_pt);
                        lidar_data.pt_buff.push_back(pt);
                    }
                }
            }            
            if (!if_lidar_only && !imu_int_buff.empty()) {
                m_buff.lock();
                Eigen::aligned_vector<sensor_msgs::msg::Imu::SharedPtr> imu_buff_msg = imu_int_buff;
                imu_int_buff.clear();
                m_buff.unlock();
                for (size_t i = 0; i < imu_buff_msg.size(); i++) {
                    const auto imu_msg = imu_buff_msg[i];
                    int64_t t_ns = rclcpp::Time(imu_msg->header.stamp).nanoseconds();
                    Eigen::Vector3d acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
                    if (acc_ratio) acc *= 9.81;
                    Eigen::Vector3d gyro(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                    ImuData imu(t_ns, gyro, acc); 
                    imu_buff.push_back(imu);
                }
            }
            if(!initialization()) {
                rate.sleep();
                continue;
            }
            while (collectMeasurements()) {
                int64_t max_time_ns = pt_meas.back().time_ns;
                if (if_lidar_only) {
                    estimator_lo.propRCP(max_time_ns);
                    estimator_lo.updateIEKFLiDAR(pt_meas, &ikdtree, param.nn_thresh, param.coeff_cov);  
                } else {
                    if (!imu_meas.empty()) {
                        max_time_ns = std::max(imu_meas.back().time_ns, max_time_ns);
                    }
                    while (!imu_meas.empty() && imu_meas.front().time_ns < spline->maxTimeNs() - spline->getKnotTimeIntervalNs()) {
                        imu_meas.pop_front();
                    }                         
                    estimator_lio.propRCP(max_time_ns);
                    estimator_lio.updateIEKFLiDARInertial(pt_meas, &ikdtree, param.nn_thresh, imu_meas, gravity, param.cov_acc, param.cov_gyro, param.coeff_cov);  
                }
                #pragma omp parallel for num_threads(NUM_OF_THREAD)
                for (size_t i = 0; i < pt_meas.size(); i++) {
                    PointData& pt_data = pt_meas[i];            
                    Association::pointBodyToWorld(pt_data.time_ns, spline, pt_data.pt, pt_data.pt_w, pt_data.t_bl, pt_data.q_bl);
                }            
                for (size_t i = 0; i < pt_meas.size(); i++) {
                    PointData& pt_data = pt_meas[i];
                    pc_world.points.push_back(pt_data.pt_w);
                    accum_nearest_points.push_back(pt_data.nearest_points);
                }
                pt_meas.clear();
                if (spline->numKnots() > max_spl_knots) {
                    estimate_msgs::msg::Spline spline_msg;
                    spline->getSplineMsg(spline_msg, std::max(int(max_spl_knots-1),0));
                    estimate_msgs::msg::Estimate est_msg;
                    est_msg.spline = spline_msg;
                    est_msg.if_full_window.data = (spline->numKnots() >= 4);
                    est_msg.runtime.data = 0;
                    pub_est->publish(est_msg);  
                    max_spl_knots = spline->numKnots();       
                }
                if (max_time_ns >= t_last_map_upd + 1e8) {
                    mapIncremental();
                    publishFrameWorld();
                    lasermapFovSegment();
                    pc_world.clear();
                    accum_nearest_points.clear();
                    t_last_map_upd = max_time_ns;
                }
            }
        }
        processing_thread_finished_ = true;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    std::string node_name = "RESPLE";
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ouster;
    rclcpp::Subscription<livox_ros_driver::msg::CustomMsg>::SharedPtr sub_livox;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox2;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_livox_avia;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_hesai;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_livox_mid360_boxi;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cur_scan;
    rclcpp_lifecycle::LifecyclePublisher<estimate_msgs::msg::Estimate>::SharedPtr pub_est;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr pub_start_time;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    const std::string frame_id = "body";
    const std::string odom_id = "world";

    std::atomic_bool running_{false};
    std::atomic_bool processing_thread_finished_{false};
    std::shared_ptr<std::thread> processing_thread_;

    std::string bond_topic_name_;
    std::string bond_id_;
    bool enable_bond_ = true;
    double bond_heartbeat_period_s_ = 0.1;
    double bond_heartbeat_timeout_s_ = 1.0;
    std::atomic_bool bond_formed_{false};
    std::atomic_bool bond_broken_{false};
    std::unique_ptr<bond::Bond> lifecycle_bond_;

    void start_bond()
    {
        if (!enable_bond_) {
            return;
        }

        lifecycle_bond_ = std::make_unique<bond::Bond>(
            bond_topic_name_, bond_id_,
            get_node_base_interface(),
            get_node_logging_interface(),
            get_node_parameters_interface(),
            get_node_timers_interface(),
            get_node_topics_interface(),
            std::bind(&RESPLE::bond_broken_callback, this),
            std::bind(&RESPLE::bond_formed_callback, this));
        lifecycle_bond_->setHeartbeatPeriod(bond_heartbeat_period_s_);
        lifecycle_bond_->setHeartbeatTimeout(bond_heartbeat_timeout_s_);
        lifecycle_bond_->start();
    }

    void stop_bond()
    {
        if (lifecycle_bond_) {
            lifecycle_bond_->breakBond();
            lifecycle_bond_.reset();
        }
        bond_formed_ = false;
        bond_broken_ = false;
    }

    void bond_formed_callback()
    {
        bond_formed_ = true;
        bond_broken_ = false;
        RCLCPP_INFO(this->get_logger(), "Lifecycle bond formed on %s", bond_topic_name_.c_str());
    }

    void bond_broken_callback()
    {
        bond_formed_ = false;
        bond_broken_ = true;
        RCLCPP_WARN(this->get_logger(), "Lifecycle bond broken on %s", bond_topic_name_.c_str());
    }

    std::map<std::string, LidarConfig> lidars;
    float ds_lm_voxel;
    pcl::VoxelGrid<pcl::PointXYZINormal> ds_filter_body;    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last_ds;
    pcl::PointCloud<pcl::PointXYZINormal> pc_world;
    int point_filter_num = 1;
    int64_t time_offset = 0;

    std::vector<BoxPointType> cub_needrm;
    BoxPointType LocalMap_Points;
    std::vector<Eigen::aligned_vector<pcl::PointXYZINormal>> accum_nearest_points;
    double cube_len = 2000; 
    const float MOV_THRESHOLD = 1.5f;
    float det_range = 100.0;
    bool if_init_map = false;
    struct LidarData {
        Eigen::aligned_deque<Eigen::aligned_vector<pcl::PointXYZINormal>> pc_buff;
        std::deque<int64_t> t_buff;
        std::mutex mtx_pc;
        Eigen::aligned_deque<PointData> pt_buff;
    };
    std::map<std::string, LidarData> lidars_data;    
    Eigen::aligned_deque<PointData> pt_meas;    

    bool if_lidar_only;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    Eigen::aligned_deque<ImuData> imu_buff;
    Eigen::aligned_deque<ImuData> imu_meas;
    Eigen::aligned_vector<sensor_msgs::msg::Imu::SharedPtr> imu_int_buff;    
    std::mutex m_buff;
    bool acc_ratio;
    Eigen::Vector3d cov_ba;
    Eigen::Vector3d cov_bg;    
    Eigen::Vector3d gravity;
    
    bool if_init_filter = false;
    Estimator<24> estimator_lo;
    Estimator<30> estimator_lio;
    SplineState* spline;
    double cov_P0 = 0.02;
    double cov_RCP_pos_old = 0.02;
    double cov_RCP_ort_old = 0.02;
    double cov_RCP_pos_new = 0.1;    
    double cov_RCP_ort_new = 0.1;    
    double cov_sys_pos = 0.1;    
    double cov_sys_ort = 0.01;    
    Parameters param;
    int64_t dt_ns;
    int num_points_upd;
    
    const std::string baselink_frame = "base_link";
    const std::string odom_frame = "odom";

    void readParameters()
    {
        ds_lm_voxel = CommonUtils::readParam<float>(*this, "ds_lm_voxel");
        float ds_scan_voxel = CommonUtils::readParam<float>(*this, "ds_scan_voxel");
        ds_filter_body.setLeafSize(ds_scan_voxel, ds_scan_voxel, ds_scan_voxel);
        param.nn_thresh = CommonUtils::readParam<double>(*this, "nn_thresh");
        if_lidar_only = CommonUtils::readParam<bool>(*this, "if_lidar_only");
        if (!if_lidar_only) {
            acc_ratio = CommonUtils::readParam<bool>(*this, "acc_ratio");
            std::vector<double> bias_acc_var = CommonUtils::readParam<std::vector<double>>(*this, "cov_ba");
            cov_ba << bias_acc_var.at(0), bias_acc_var.at(1), bias_acc_var.at(2);
            std::vector<double> bias_gyro_var = CommonUtils::readParam<std::vector<double>>(*this, "cov_bg");
            cov_bg << bias_gyro_var.at(0), bias_gyro_var.at(1), bias_gyro_var.at(2);
            std::vector<double> acc_var = CommonUtils::readParam<std::vector<double>>(*this, "cov_acc");
            param.cov_acc << acc_var.at(0), acc_var.at(1), acc_var.at(2);
            std::vector<double> gyro_var = CommonUtils::readParam<std::vector<double>>(*this, "cov_gyro");
            param.cov_gyro << gyro_var.at(0), gyro_var.at(1), gyro_var.at(2);
        }

        dt_ns = 1e9 / CommonUtils::readParam<int>(*this, "knot_hz");
        double dt_s = double(dt_ns) * 1e-9;
        cov_P0 = CommonUtils::readParam<double>(*this, "cov_P0");
        cov_P0 *= (dt_s*dt_s);
        cov_RCP_pos_old = CommonUtils::readParam<double>(*this, "cov_RCP_pos_old");
        cov_RCP_ort_old = CommonUtils::readParam<double>(*this, "cov_RCP_ort_old");
        cov_RCP_pos_new = CommonUtils::readParam<double>(*this, "cov_RCP_pos_new");
        cov_RCP_ort_new = CommonUtils::readParam<double>(*this, "cov_RCP_ort_new");
        double std_pos = CommonUtils::readParam<double>(*this, "std_sys_pos");
        double std_ort = CommonUtils::readParam<double>(*this, "std_sys_ort");
        cov_sys_pos = std_pos*std_pos*dt_s*dt_s;
        cov_sys_ort = std_ort*std_ort*dt_s*dt_s;
        param.coeff_cov = CommonUtils::readParam<double>(*this, "coeff_cov", 10);

        cube_len = CommonUtils::readParam<double>(*this, "cube_len");
        point_filter_num = CommonUtils::readParam<int>(*this, "point_filter_num");
        num_points_upd = CommonUtils::readParam<int>(*this, "num_points_upd");
        if (if_lidar_only) {
            estimator_lo.n_iter = CommonUtils::readParam<int>(*this, "n_iter");
        } else {
            estimator_lio.n_iter = CommonUtils::readParam<int>(*this, "n_iter");
        }
        pc_last.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        pc_last_ds.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        NUM_MATCH_POINTS = CommonUtils::readParam<int>(*this, "num_nn", 5);
        double lidar_time_offset = CommonUtils::readParam<double>(*this, "lidar_time_offset", 0.0);
        time_offset = 1e9*lidar_time_offset;
    }

    void initFilter(int64_t start_t_ns, Eigen::Vector3d t_init = Eigen::Vector3d::Zero(), Eigen::Quaterniond q_init = Eigen::Quaterniond::Identity())
    {
        Eigen::Matrix<double, 24, 24> cov_RCPs = cov_P0 * Eigen::Matrix<double, 24, 24>::Identity();
        Eigen::Matrix<double, 30, 30> Q = Eigen::Matrix<double, 30, 30>::Zero();
        Eigen::Matrix<double, 6, 6> Q_block_old = Eigen::Matrix<double, 6, 6>::Zero();
        Q_block_old.topLeftCorner<3, 3>() = cov_RCP_pos_old*cov_sys_pos *Eigen::Matrix3d::Identity();
        Q_block_old.bottomRightCorner<3, 3>() = cov_RCP_ort_old*cov_sys_ort *Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 6> Q_block_new = Eigen::Matrix<double, 6, 6>::Zero();
        Q_block_new.topLeftCorner<3, 3>() = cov_RCP_pos_new*cov_sys_pos *Eigen::Matrix3d::Identity();
        Q_block_new.bottomRightCorner<3, 3>() = cov_RCP_ort_new*cov_sys_ort *Eigen::Matrix3d::Identity();        
        Q.topLeftCorner<6, 6>() = Q_block_old;
        Q.block<6, 6>(6, 6) = Q_block_old;
        Q.block<6, 6>(12, 12) = Q_block_old;
        Q.bottomRightCorner<6, 6>() = Q_block_new;  
        if (if_lidar_only) {
            estimator_lo.setState(dt_ns, start_t_ns, t_init, q_init, Q.topLeftCorner<24, 24>(), cov_RCPs);  
            spline = estimator_lo.getSpline();
        } else {
            Eigen::Matrix<double, 30, 30> cov_x = Eigen::Matrix<double, 30, 30>::Zero();
            cov_x.topLeftCorner<24, 24>() = cov_RCPs;
            cov_x.block<3, 3>(24, 24) = cov_ba.asDiagonal();
            cov_x.block<3, 3>(27, 27) = cov_bg.asDiagonal();            
            estimator_lio.setState(dt_ns, start_t_ns, t_init, q_init, Q, cov_x);   
            spline = estimator_lio.getSpline(); 
        }
    }     

    void getImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        m_buff.lock();
        imu_int_buff.push_back(imu_msg);
        m_buff.unlock();        
    }    

    template<typename T>
    void ousterLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ouster_msg_in)
    {
        std::string name = "Ouster";
        const LidarConfig& lidar = lidars.at(name);        
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());
        typename pcl::PointCloud<T>::Ptr pc_last_ouster(new typename pcl::PointCloud<T>());
        pcl::fromROSMsg(*ouster_msg_in, *pc_last_ouster);
        size_t plsize = pc_last_ouster->size();
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(ouster_msg_in->header.stamp).nanoseconds() - time_offset;
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        pcl::PointXYZINormal pt;
        float blind = lidar.blind;
        for (unsigned int i = 0; i < plsize; ++i) {
            if (i % point_filter_num == 0) {
                pt.x = pc_last_ouster->points[i].x;
                pt.y = pc_last_ouster->points[i].y;
                pt.z = pc_last_ouster->points[i].z;
                pt.intensity = float (pc_last_ouster->points[i].t) / float (1e6); // unit: ms
                pt.curvature = pc_last_ouster->points[i].intensity;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind) && pc_last_ouster->points[i].t + time_begin > last_t_ns) {
                    pc_last->points.push_back(pt);
                    int64_t ofs = pc_last_ouster->points[i].t;
                    max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                }
            }
        }
        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_last->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();        
        last_t_ns = time_begin + max_ofs_ns;
    }    

    void livoxLidarCallback(const livox_ros_driver::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        std::string name = "Mid70Avia";
        const LidarConfig& lidar = lidars.at(name);   
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());     
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        int valid_point_num = 0;
        pcl::PointXYZINormal pt_pre;
        pt_pre.x = livox_msg_in->points[0].x;
        pt_pre.y = livox_msg_in->points[0].y;
        pt_pre.z = livox_msg_in->points[0].z;
        int N_SCAN_LINES = lidar.scan_line;
        float blind = lidar.blind;        
        for (int i = 1; i < plsize; ++i) {
            if ((livox_msg_in->points[i].line < N_SCAN_LINES) && ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00)) {
                valid_point_num++;
                if (valid_point_num % point_filter_num == 0) {
                    pcl::PointXYZINormal pt;
                    pt.x = livox_msg_in->points[i].x;
                    pt.y = livox_msg_in->points[i].y;
                    pt.z = livox_msg_in->points[i].z;
                    pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); // unit: ms
                    pt.curvature = livox_msg_in->points[i].reflectivity;
                    if (pt.intensity >= 0 && ((abs(pt.x - pt_pre.x) > 1e-7) || (abs(pt.y - pt_pre.y) > 1e-7) || (abs(pt.z - pt_pre.z) > 1e-7))
                                            && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind)&& livox_msg_in->points[i].offset_time + time_begin > last_t_ns) {
                        int64_t ofs = livox_msg_in->points[i].offset_time;
                        max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                        pc_last->points.push_back(pt);
                    }
                    pt_pre = pt;
                }

            } 
        }
        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_last->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();
        last_t_ns = time_begin + max_ofs_ns;
    }    

    void livoxLidar2Callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg_in)
    {
        std::string name = "HAP360";
        const LidarConfig& lidar = lidars.at(name);     
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());        
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        int valid_point_num = 0;
        pcl::PointXYZINormal pt_pre;
        pt_pre.x = livox_msg_in->points[0].x;
        pt_pre.y = livox_msg_in->points[0].y;
        pt_pre.z = livox_msg_in->points[0].z;
        int N_SCAN_LINES = lidar.scan_line;
        float blind = lidar.blind;          
        for (int i = 1; i < plsize; ++i) {
            if ((livox_msg_in->points[i].line < N_SCAN_LINES) && ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00)) {
                valid_point_num++;
                if (valid_point_num % point_filter_num == 0) {
                    pcl::PointXYZINormal pt;
                    pt.x = livox_msg_in->points[i].x;
                    pt.y = livox_msg_in->points[i].y;
                    pt.z = livox_msg_in->points[i].z;
                    pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); 
                    pt.curvature = livox_msg_in->points[i].reflectivity;
                    if (pt.intensity >= 0 && ((abs(pt.x - pt_pre.x) > 1e-7) || (abs(pt.y - pt_pre.y) > 1e-7) || (abs(pt.z - pt_pre.z) > 1e-7))
                                            && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind) && livox_msg_in->points[i].offset_time + time_begin > last_t_ns) {
                        int64_t ofs = livox_msg_in->points[i].offset_time;
                        max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                        pc_last->points.push_back(pt);
                    }
                    pt_pre = pt;
                }
            } 
        }
        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_last->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();        
        last_t_ns = time_begin + max_ofs_ns;
    }

     void livoxAVIACallback(const livox_interfaces::msg::CustomMsg::SharedPtr livox_msg_in)
     {
        std::string name = "AviaResple";
        const LidarConfig& lidar = lidars.at(name);       
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());      
        int plsize = livox_msg_in->point_num;
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        int64_t time_begin = rclcpp::Time(livox_msg_in->header.stamp).nanoseconds();
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;
        int valid_point_num = 0;
        pcl::PointXYZINormal pt_pre;
        pt_pre.x = livox_msg_in->points[0].x;
        pt_pre.y = livox_msg_in->points[0].y;
        pt_pre.z = livox_msg_in->points[0].z;
        int N_SCAN_LINES = lidar.scan_line;
        float blind = lidar.blind;           
        for (int i = 1; i < plsize; ++i) {
            if ((livox_msg_in->points[i].line < N_SCAN_LINES) && ((livox_msg_in->points[i].tag & 0x30) == 0x10 || (livox_msg_in->points[i].tag & 0x30) == 0x00) && livox_msg_in->points[i].offset_time + time_begin > last_t_ns) {
                valid_point_num++;
                if (valid_point_num % point_filter_num == 0) {
                    pcl::PointXYZINormal pt;
                    pt.x = livox_msg_in->points[i].x;
                    pt.y = livox_msg_in->points[i].y;
                    pt.z = livox_msg_in->points[i].z;
                    pt.intensity = float (livox_msg_in->points[i].offset_time) / float (1e6); 
                    pt.curvature = livox_msg_in->points[i].reflectivity;
                    if (pt.intensity >= 0 && ((abs(pt.x - pt_pre.x) > 1e-7) || (abs(pt.y - pt_pre.y) > 1e-7) ||
                                            (abs(pt.z - pt_pre.z) > 1e-7))
                                            && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind)) {
                        int64_t ofs = livox_msg_in->points[i].offset_time;
                        max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                        pc_last->points.push_back(pt);
                    }
                    pt_pre = pt;
                }
            }
        }
        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_last->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();
        last_t_ns = time_begin + max_ofs_ns;
     }        

    void hesaiLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr hesai_msg_in)
	{
        std::string name = "Hesai";
        const LidarConfig& lidar = lidars.at(name);    
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());    
        pcl::PointCloud<hesai_ros::Point>::Ptr pc_last_hesai(new pcl::PointCloud<hesai_ros::Point>());
        pcl::fromROSMsg(*hesai_msg_in, *pc_last_hesai);
        size_t plsize = pc_last_hesai->size();
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(hesai_msg_in->header.stamp);
        int64_t time_begin = timestamp_begin.nanoseconds();
        static int64_t last_t_ns = time_begin;
        int64_t max_ofs_ns = 0;        
        pcl::PointXYZINormal pt;
        float blind = lidar.blind;
        for (unsigned int i = 0; i < plsize; ++i) {
            if (i % point_filter_num == 0) {
                pt.x = pc_last_hesai->points[i].x;
                pt.y = pc_last_hesai->points[i].y;
                pt.z = pc_last_hesai->points[i].z;
                double timestamp_s;
                double timestamp_ns = std::modf(pc_last_hesai->points[i].timestamp, &timestamp_s);
                rclcpp::Time timestamp_ros(static_cast<int32_t>(timestamp_s), static_cast<int32_t>(timestamp_ns * 1.0e9),
                    rcl_clock_type_t::RCL_ROS_TIME);
                pt.intensity = (timestamp_ros - timestamp_begin).seconds() * 1.0e3; 
                pt.curvature = pc_last_hesai->points[i].intensity;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind) && CommonUtils::ms2ns(pt.intensity) + time_begin > last_t_ns) {
                    int64_t ofs = CommonUtils::ms2ns(pt.intensity);
                    max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                    pc_last->points.push_back(pt);
                }
            }
        }
        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_last->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();        
        last_t_ns = time_begin + max_ofs_ns;
	}     

    void livoxMid360BoxiCallback(const sensor_msgs::msg::PointCloud2::SharedPtr livox_msg_in)
	{
        std::string name = "Mid360Boxi";
        const LidarConfig& lidar = lidars.at(name);   
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_last(new pcl::PointCloud<pcl::PointXYZINormal>());     
        pcl::PointCloud<livox_mid360_boxi::Point>::Ptr pc_last_livox(new pcl::PointCloud<livox_mid360_boxi::Point>());
        pcl::fromROSMsg(*livox_msg_in, *pc_last_livox);
        size_t plsize = pc_last_livox->size();
        if (plsize == 0) return;
        pc_last->reserve(plsize);
        rclcpp::Time timestamp_begin = rclcpp::Time(livox_msg_in->header.stamp);
        int64_t time_begin = timestamp_begin.nanoseconds();
        static int64_t last_t_ns = time_begin;   
        int64_t max_ofs_ns = 0;         
        pcl::PointXYZINormal pt;
        float blind = lidar.blind;
        for (unsigned int i = 0; i < plsize; ++i) {
            if (i % point_filter_num == 0) {
                pt.x = pc_last_livox->points[i].x;
                pt.y = pc_last_livox->points[i].y;
                pt.z = pc_last_livox->points[i].z;
                rclcpp::Time timestamp_ros(static_cast<int64_t>(pc_last_livox->points[i].timestamp),
                    rcl_clock_type_t::RCL_ROS_TIME);
                pt.intensity = (timestamp_ros - timestamp_begin).seconds() * 1.0e3;
                pt.curvature = pc_last_livox->points[i].intensity;
                if (pt.intensity >= 0 && pt.x*pt.x+pt.y*pt.y+pt.z*pt.z > (blind * blind) && CommonUtils::ms2ns(pt.intensity) + time_begin > last_t_ns) {
                    int64_t ofs = CommonUtils::ms2ns(pt.intensity);
                    max_ofs_ns = max_ofs_ns > ofs ? max_ofs_ns : ofs;
                    pc_last->points.push_back(pt);
                }
            }
        }
        LidarData& lidar_buffs = lidars_data.at(name);
        lidar_buffs.mtx_pc.lock();
        lidar_buffs.pc_buff.push_back(pc_last->points);
        lidar_buffs.t_buff.push_back(time_begin);
        lidar_buffs.mtx_pc.unlock();     
        last_t_ns = time_begin + max_ofs_ns;   
	}      

    void publishFrameWorld() 
    {
        int size = pc_world.points.size();
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudWorld(new pcl::PointCloud<pcl::PointXYZI>(size, 1));
        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = pc_world.points[i].x;
            laserCloudWorld->points[i].y = pc_world.points[i].y;
            laserCloudWorld->points[i].z = pc_world.points[i].z;
            laserCloudWorld->points[i].intensity = pc_world.points[i].curvature; 
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = rclcpp::Time(spline->maxTimeNs());
        laserCloudmsg.header.frame_id = odom_id;
        pub_cur_scan->publish(laserCloudmsg);
    }   

    bool initialization()
    {
        if (if_init_filter && if_init_map) {
            return true;
        } 
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            if (lidar_data.pt_buff.empty()) {
                return false;
            }
        }
        int64_t start_t_ns = std::numeric_limits<int64_t>::max();
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            start_t_ns = std::min(start_t_ns, std::max(lidar_data.pt_buff.front().time_ns, int64_t(0)));
        }        
        if (!if_init_filter) {
            Eigen::Quaterniond q_WI = Eigen::Quaterniond::Identity();   
            if (!if_lidar_only) {
                Eigen::Vector3d gravity_sum(0, 0, 0);
                m_buff.lock();
                int buff_size = imu_buff.size();
                int n_imu = std::min(15, buff_size);
                for (int i = 0; i < n_imu; i++) {
                    gravity_sum += imu_buff.at(i).accel;
                }
                while (!imu_buff.empty() && imu_buff.front().time_ns < start_t_ns) {
                    imu_buff.pop_front();
                }                    
                m_buff.unlock();
                gravity_sum /= n_imu;
                Eigen::Vector3d gravity_ave = gravity_sum.normalized() * 9.81;
                Eigen::Matrix3d R0 = CommonUtils::g2R(gravity_ave);
                double yaw = CommonUtils::R2ypr(R0).x();
                R0 = CommonUtils::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
                Eigen::Quaterniond q0(R0);
                q_WI = Quater::positify(q0);
                gravity = q_WI * gravity_ave;
            } 
            initFilter(start_t_ns, Eigen::Vector3d(0, 0, 0), q_WI);
            if_init_filter = true;            
            std_msgs::msg::Int64 start_time;
            start_time.data = start_t_ns;
            pub_start_time->publish(start_time);    
        }
        if (!if_init_map) {
            if(ikdtree.Root_Node == nullptr) {
                ikdtree.set_downsample_param(ds_lm_voxel);
            }
            if (if_lidar_only) {
                estimator_lo.propRCP(start_t_ns);  
            } else {
                estimator_lio.propRCP(start_t_ns);
            }
            int feats_down_size = 0;
            for (const auto& [lidar_name, lidar_data] : lidars_data) {
                for (size_t i = 0; i < lidar_data.pt_buff.size(); i++) {
                    if (lidar_data.pt_buff[i].time_ns < start_t_ns + 1e8) {
                        feats_down_size++;
                    } else {
                        break;
                    }
                }
            }            
            if(feats_down_size < 100) {
                return false;
            }            
            pc_world.clear();
            pc_world.resize(feats_down_size); 
            int world_i = 0;
            for (const auto& [lidar_name, lidar_data] : lidars_data) {
                for (size_t i = 0; i < lidar_data.pt_buff.size(); i++) {
                    if (lidar_data.pt_buff[i].time_ns < start_t_ns + 1e8) {
                        Association::pointBodyToWorld(start_t_ns, spline, lidar_data.pt_buff[i].pt,
                            pc_world.points[world_i], lidar_data.pt_buff[i].t_bl, lidar_data.pt_buff[i].q_bl);
                        world_i++;
                    } else {
                        break;
                    }
                }
            }
            for (auto& [lidar_name, lidar_data] : lidars_data) {
                while (!lidar_data.pt_buff.empty() && lidar_data.pt_buff.front().time_ns < start_t_ns + 1e8) {
                    lidar_data.pt_buff.pop_front();
                }
            }            
            ikdtree.Build(pc_world.points); 
            pc_world.clear();
            if_init_map = true;
        }       
        return false; 
    }

    bool collectMeasurements()
    {
        rclcpp::Rate rate(20);
        int64_t pt_min_time = std::numeric_limits<int64_t>::max();
        int64_t pt_max_time = std::numeric_limits<int64_t>::max();            
        for (const auto& [lidar_name, lidar_data] : lidars_data) {
            if (lidar_data.pt_buff.empty()) {
                rate.sleep();
                return false;
            }
            pt_min_time = std::min(pt_min_time, lidar_data.pt_buff.front().time_ns);
            pt_max_time = std::min(pt_max_time, lidar_data.pt_buff.back().time_ns);
        }            
        if (pt_max_time <= spline->maxTimeNs() + dt_ns) {
            rate.sleep();
            return false;
        }      
        if (!if_lidar_only && (imu_buff.empty() || imu_buff.back().time_ns <= spline->maxTimeNs())) {
            rate.sleep();
            return false;
        }                   
        int64_t max_time_ns = std::min(spline->maxTimeNs(), pt_min_time + dt_ns);
        if (pt_min_time > max_time_ns) {
            if (if_lidar_only) {
                estimator_lo.propRCP(pt_min_time);
            } else {
                estimator_lio.propRCP(pt_min_time);
            }                
            max_time_ns = spline->maxTimeNs();
        }     
        if (spline->numKnots() > 4) {
            max_time_ns = spline->maxTimeNs();
        }
        int cnt = 0;
        for (auto& [lidar_name, lidar_data] : lidars_data) {
            while (!lidar_data.pt_buff.empty() && lidar_data.pt_buff.front().time_ns <= max_time_ns &&
                    cnt < num_points_upd) {
                if (spline->numKnots() < 10 || lidar_data.pt_buff.front().time_ns >= spline->maxTimeNs() - dt_ns) {
                    pt_meas.emplace_back(lidar_data.pt_buff.front());
                }
                lidar_data.pt_buff.pop_front();
                cnt++;
            }
        }                    
        if (!if_lidar_only) {
            while (!imu_buff.empty() && imu_buff.front().time_ns < spline->minTimeNs()) {
                imu_buff.pop_front();
            }                
            while (!imu_buff.empty() && imu_buff.front().time_ns <= max_time_ns) {
                imu_meas.emplace_back(imu_buff.front());
                imu_buff.pop_front();
            } 
        } 
        return true;
 
    }

    Eigen::Vector3d getPositionLiDAR(int64_t t_ns, const Eigen::Vector3d& t_bl)
    {
        if (if_lidar_only) {
            estimator_lo.propRCP(t_ns);
        } else {
            estimator_lio.propRCP(t_ns);
        }
        Eigen::Quaterniond orient_interp;
        Eigen::Vector3d t_interp = spline->itpPosition(t_ns);
        spline->itpQuaternion(t_ns, &orient_interp);
        Eigen::Vector3d t = orient_interp * t_bl + t_interp;
        return t;
    }       

    void lasermapFovSegment()
    {
        static bool Localmap_Initialized = false;
        cub_needrm.shrink_to_fit();
        Eigen::Vector3d pos_lidar_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        Eigen::Vector3d pos_lidar_max(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(),
                std::numeric_limits<double>::min());
        for (const auto& [lidar_name, lidar] : lidars) {
            Eigen::Vector3d pos_lidar = getPositionLiDAR(spline->maxTimeNs(), lidar.t_bl);
            pos_lidar_min = pos_lidar_min.array().min(pos_lidar.array()).matrix();
            pos_lidar_max = pos_lidar_max.array().max(pos_lidar.array()).matrix();
        }        
        if (!Localmap_Initialized){
            for (int i = 0; i < 3; i++){
                LocalMap_Points.vertex_min[i] = pos_lidar_min(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_lidar_max(i) + cube_len / 2.0;                
            }
            Localmap_Initialized = true;
            return;
        }
        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++){
            dist_to_map_edge[i][0] = fabs(pos_lidar_min(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_lidar_max(i) - LocalMap_Points.vertex_max[i]);            
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range || dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range) need_move = true;
        }
        if (!need_move) return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
        float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * det_range) * 0.5 * 0.9, double(det_range * (MOV_THRESHOLD -1)));
        for (int i = 0; i < 3; i++){
            tmp_boxpoints = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * det_range){
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.emplace_back(tmp_boxpoints);
            } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * det_range){
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.emplace_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        if(cub_needrm.size() > 0) {
            ikdtree.Delete_Point_Boxes(cub_needrm);
        }
    }    

    void mapIncremental()
    {
        Eigen::aligned_vector<pcl::PointXYZINormal> PointToAdd;
        Eigen::aligned_vector<pcl::PointXYZINormal> PointNoNeedDownsample;
        int feats_down_size = pc_world.points.size();
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        for(int i = 0; i < feats_down_size; i++) {     
            const pcl::PointXYZINormal& point = pc_world.points[i];       
            if (!accum_nearest_points[i].empty()) {
                const Eigen::aligned_vector<pcl::PointXYZINormal> &points_near = accum_nearest_points[i];
                bool need_add = true;
                pcl::PointXYZINormal downsample_result, mid_point; 
                
                mid_point.x = floor(point.x/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                mid_point.y = floor(point.y/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                mid_point.z = floor(point.z/ds_lm_voxel)*ds_lm_voxel + 0.5 * ds_lm_voxel;
                if (fabs(points_near[0].x - mid_point.x) > 0.866 * ds_lm_voxel || fabs(points_near[0].y - mid_point.y) > 0.866 * ds_lm_voxel || fabs(points_near[0].z - mid_point.z) > 0.866 * ds_lm_voxel){
                    PointNoNeedDownsample.emplace_back(pc_world.points[i]);
                    continue;
                }
                for (size_t readd_i = 0; readd_i < points_near.size(); readd_i ++) {
                    if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * ds_lm_voxel && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * ds_lm_voxel && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * ds_lm_voxel) {
                        need_add = false;
                        break;
                    }
                }
                if (need_add) PointToAdd.emplace_back(point);
            } else {
                PointNoNeedDownsample.emplace_back(point);
            }
        }
        ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto resple = std::make_shared<RESPLE>(options);
    RCLCPP_INFO_STREAM(resple->get_logger(), "RESPLE starts!");
    rclcpp::spin(resple->get_node_base_interface());
    // spin() also returns when on_activate()'s guard already called
    // rclcpp::shutdown() itself (forcing a process exit) -- calling it again
    // on an already-shutdown context throws, so only call it if still needed.
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}
