#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <controller_msgs/msg/imu_data.hpp>
#include <chrono>

using std::placeholders::_1;

class ImuConverterNode : public rclcpp::Node
{
public:
    ImuConverterNode() : Node("imu_converter_node"), initialized_(false)
    {
        // --- 参数声明 ---
        // NUC时间戳的单位转换系数。假设imustamp是微秒(us)，则系数为 1e-6。如果是毫秒(ms)则为 1e-3
        this->declare_parameter<double>("imustamp_scale", 1e-6);
        // EMA滤波系数 (0.0 到 1.0)。越小越平滑但跟随漂移越慢；越大越能快速跟随但受网络抖动影响大
        this->declare_parameter<double>("alpha", 0.01);
        this->declare_parameter<std::string>("frame_id", "imu_link");

        this->get_parameter("imustamp_scale", imustamp_scale_);
        this->get_parameter("alpha", alpha_);
        this->get_parameter("frame_id", frame_id_);

        // --- 订阅与发布 ---
        sub_ = this->create_subscription<controller_msgs::msg::IMUData>(
            "/ImuData", 100, std::bind(&ImuConverterNode::imu_callback, this, _1));
        
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 100);

        RCLCPP_INFO(this->get_logger(), "IMU Sync Converter Node Started.");
    }

private:
    void imu_callback(const controller_msgs::msg::IMUData::SharedPtr msg)
    {
        rclcpp::Time orin_recv_time = this->now();

        // 1. 将NUC的时间戳转换为ROS Time (秒和纳秒)
        double nuc_time_sec = msg->imustamp * imustamp_scale_;
        uint64_t nuc_time_ns = static_cast<uint64_t>(nuc_time_sec * 1e9);
        rclcpp::Time nuc_time(nuc_time_ns, RCL_ROS_TIME);

        // 2. 计算当前时间偏移量
        rclcpp::Duration current_offset = orin_recv_time - nuc_time;

        // 3. 使用EMA平滑时间偏移量
        if (!initialized_) {
            smoothed_offset_ = current_offset;
            initialized_ = true;
        } else {
            // 平滑处理: Smoothed = alpha * Current + (1 - alpha) * Smoothed
            double offset_ns = alpha_ * current_offset.nanoseconds() + 
                               (1.0 - alpha_) * smoothed_offset_.nanoseconds();
            smoothed_offset_ = rclcpp::Duration(std::chrono::nanoseconds(static_cast<int64_t>(offset_ns)));
        }

        // 4. 计算对齐后的Orin系统时间
        rclcpp::Time aligned_time = nuc_time + smoothed_offset_;

        // 5. 数据转换
        sensor_msgs::msg::Imu imu_out;
        imu_out.header.stamp = aligned_time;
        imu_out.header.frame_id = frame_id_;

        // 四元数 (注意: 确认你的通讯协议中 quat 是 [x,y,z,w] 还是 [w,x,y,z])
        // 此处假设为 ROS 标准的 [x, y, z, w]
        imu_out.orientation.x = msg->quat[0];
        imu_out.orientation.y = msg->quat[1];
        imu_out.orientation.z = msg->quat[2];
        imu_out.orientation.w = msg->quat[3];

        // 角速度 (Gyro)
        imu_out.angular_velocity.x = msg->gyro[0];
        imu_out.angular_velocity.y = msg->gyro[1];
        imu_out.angular_velocity.z = msg->gyro[2];

        // 线加速度 (Acc)
        imu_out.linear_acceleration.x = msg->acc[0];
        imu_out.linear_acceleration.y = msg->acc[1];
        imu_out.linear_acceleration.z = msg->acc[2];

        // 赋予一个默认的小协方差矩阵 (LIO系统通常需要协方差不能全为0)
        for (int i = 0; i < 9; ++i) {
            imu_out.orientation_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
            imu_out.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
            imu_out.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        }

        // 6. 发布对齐并转换后的标准IMU消息
        pub_->publish(imu_out);
    }

    rclcpp::Subscription<controller_msgs::msg::IMUData>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

    double imustamp_scale_;
    double alpha_;
    std::string frame_id_;

    bool initialized_;
    rclcpp::Duration smoothed_offset_{0, 0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuConverterNode>());
    rclcpp::shutdown();
    return 0;
}