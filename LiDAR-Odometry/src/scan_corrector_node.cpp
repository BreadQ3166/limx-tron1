#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <cmath>
#include <deque>
#include <algorithm>
#include <limits> // 必须包含，用于生成 infinity

using LaserScanMsg = sensor_msgs::msg::LaserScan;
using ImuMsg = sensor_msgs::msg::Imu;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<LaserScanMsg, ImuMsg>;

class ScanPitchCorrector : public rclcpp::Node
{
public:
  ScanPitchCorrector() : Node("scan_corrector")
  {
    // ================= 1. 声明功能开关参数 =================
    this->declare_parameter<bool>("enable_debug_info", true);       // 开关：是否输出调试信息
    this->declare_parameter<bool>("enable_imu_compensation", true); // 开关：是否启用IMU倾斜补偿
    this->declare_parameter<bool>("enable_ground_filtering", false); // 开关：是否启用动态地面剔除

    // ================= 2. 声明算法数值参数 =================
    this->declare_parameter<int>("filter_size", 8);        // IMU滑动窗口滤波大小
    this->declare_parameter<double>("max_angle_rad", 0.4); // 最大补偿角度阈值(rad)
    this->declare_parameter<double>("lidar_height", 0.1);       // 雷达距离地面的安装高度 (米)
    this->declare_parameter<double>("ground_tolerance", 0.15);  // 地面起伏与测量误差容忍度 (米)

    // ================= 3. 初始化订阅者与同步器 =================
    // 使用 SensorDataQoS 适配实机可能的 BestEffort 策略
    auto sensor_qos = rclcpp::SensorDataQoS();
    scan_sub_.subscribe(this, "/scan", sensor_qos.get_rmw_qos_profile());
    imu_sub_.subscribe(this, "/imu_plugin/out", sensor_qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(100), scan_sub_, imu_sub_);

    sync_->registerCallback(std::bind(&ScanPitchCorrector::sync_callback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    // ================= 4. 初始化发布者 =================
    corrected_pub_ = this->create_publisher<LaserScanMsg>("/scan_corrected", 10);

    RCLCPP_INFO(this->get_logger(), "Scan Pitch Corrector 启动成功！");
    RCLCPP_INFO(this->get_logger(), "您可以通过设置参数动态开关: enable_debug_info, enable_imu_compensation, enable_ground_filtering");
  }

private:
  void sync_callback(const LaserScanMsg::ConstSharedPtr &scan_msg,
                     const ImuMsg::ConstSharedPtr &imu_msg)
  {
    // ================= 获取最新参数状态 =================
    // 功能开关
    bool enable_debug = this->get_parameter("enable_debug_info").as_bool();
    bool enable_imu = this->get_parameter("enable_imu_compensation").as_bool();
    bool enable_ground = this->get_parameter("enable_ground_filtering").as_bool();
    
    // 数值参数
    size_t filter_size = this->get_parameter("filter_size").as_int();
    double max_angle_rad = this->get_parameter("max_angle_rad").as_double();
    double lidar_height = this->get_parameter("lidar_height").as_double();
    double ground_tolerance = this->get_parameter("ground_tolerance").as_double();

    // ================= 提取并平滑 IMU 数据 =================
    // 即使关闭了补偿，我们也保持滤波器更新，防止突然开启时数据跳变
    tf2::Quaternion q(
        imu_msg->orientation.x, imu_msg->orientation.y,
        imu_msg->orientation.z, imu_msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    pitch_filter_.push_back(pitch);
    roll_filter_.push_back(roll);
    if (pitch_filter_.size() > filter_size) {
      pitch_filter_.pop_front();
      roll_filter_.pop_front();
    }

    double pitch_filtered = 0.0, roll_filtered = 0.0;
    for (auto p : pitch_filter_) pitch_filtered += p;
    for (auto r : roll_filter_) roll_filtered += r;
    pitch_filtered /= pitch_filter_.size();
    roll_filtered /= roll_filter_.size();

    // 限制最大补偿角度
    pitch_filtered = std::clamp(pitch_filtered, -max_angle_rad, max_angle_rad);
    roll_filtered = std::clamp(roll_filtered, -max_angle_rad, max_angle_rad);

    // 确定实际应用于计算的 Roll 和 Pitch
    double pitch_applied = enable_imu ? pitch_filtered : 0.0;
    double roll_applied  = enable_imu ? roll_filtered : 0.0;

    // ================= 打印调试信息 (受开关控制) =================
    if (enable_debug) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[调试] IMU补偿: %s | 地面过滤: %s | 应用的Pitch: %.3f | 应用的Roll: %.3f",
          enable_imu ? "开" : "关", 
          enable_ground ? "开" : "关", 
          pitch_applied, roll_applied);
    }

    // ================= 构建新的 Scan 数据 =================
    auto corrected_scan = std::make_shared<LaserScanMsg>(*scan_msg);
    corrected_scan->ranges.clear(); 
    
    double angle = scan_msg->angle_min;
    
    // 构造旋转矩阵：如果 enable_imu 为 false，这里构造的就是一个单位矩阵(不发生旋转)
    tf2::Matrix3x3 R;
    R.setRPY(roll_applied, -pitch_applied, 0.0);

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
      double r = scan_msg->ranges[i];
      
      // 1. 无效点过滤 (始终执行)
      if (std::isnan(r) || r < scan_msg->range_min || r > scan_msg->range_max)
      {
        corrected_scan->ranges.push_back(std::numeric_limits<float>::infinity());
        angle += scan_msg->angle_increment;
        continue;
      }

      // 2. 如果开启了 IMU补偿 或 地面过滤，则进行复杂的三维几何计算
      if (enable_imu || enable_ground) 
      {
        // 构造射线在雷达局部坐标系下的方向向量 (极坐标转笛卡尔，模长为1)
        tf2::Vector3 ray_dir(std::cos(angle), std::sin(angle), 0.0);
        
        // 将射线根据 IMU 数据旋转至绝对水平面 (若IMU补偿关闭，ray_dir_corrected == ray_dir)
        tf2::Vector3 ray_dir_corrected = R * ray_dir;

        // 3. 动态地面点剔除逻辑 (受开关控制)
        // 注意：只有当雷达发生倾斜，光束打向地面(Z分量小于0)时，才能触发地面剔除
        if (enable_ground && ray_dir_corrected.z() < -0.05) 
        {
          // 计算在这个倾斜角度下，射线打到水平地面的理论距离
          double expected_ground_dist = -lidar_height / ray_dir_corrected.z();
          
          // 如果测到的距离 >= (理论地面距离 - 容忍误差)，直接作为地面点剔除
          if (r > expected_ground_dist - ground_tolerance) 
          {
            corrected_scan->ranges.push_back(std::numeric_limits<float>::infinity());
            angle += scan_msg->angle_increment;
            continue; 
          }
        }

        // 4. 计算投影到水平面后的 2D 距离 (IMU补偿逻辑)
        if (enable_imu) 
        {
          double x = r * ray_dir.x();
          double y = r * ray_dir.y();
          tf2::Vector3 p(x, y, 0.0);
          
          // 旋转点云抵消车体倾斜
          tf2::Vector3 p_corrected = R * p;

          // 重新计算水平面上的 2D 距离
          double r_corrected = std::sqrt(p_corrected.x() * p_corrected.x() + p_corrected.y() * p_corrected.y());
          corrected_scan->ranges.push_back(r_corrected);
        } 
        else 
        {
          // 如果仅开启了地面过滤，未开启补偿，则原样保留距离
          corrected_scan->ranges.push_back(r);
        }
      }
      else
      {
        // 如果 IMU补偿 和 地面过滤 都关闭，则直接保留原始数据，节省CPU算力
        corrected_scan->ranges.push_back(r);
      }

      // 步进角度
      angle += scan_msg->angle_increment;
    }

    // ================= 发布处理后的 Scan =================
    corrected_pub_->publish(*corrected_scan);
  }

  // 类成员变量
  message_filters::Subscriber<LaserScanMsg> scan_sub_;
  message_filters::Subscriber<ImuMsg> imu_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<LaserScanMsg>::SharedPtr corrected_pub_;
  
  std::deque<double> pitch_filter_;
  std::deque<double> roll_filter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanPitchCorrector>());
  rclcpp::shutdown();
  return 0;
}