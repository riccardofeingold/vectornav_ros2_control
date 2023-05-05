#ifndef MAGNECKO_VECTORNAV_HARDWARE__BODY_IMU_SENSOR_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_component_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "magnecko_vectornav/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <functional>
#include <queue>

#if __linux__ || __CYGWIN__
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/common_group.hpp"
#include "vectornav_msgs/msg/gps_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "vectornav_msgs/msg/time_group.hpp"
#include "vectornav_msgs/action/mag_cal.hpp"

// VectorNav libvncxx
#include "vn/util.h"
#include "vn/ezasyncdata.h"
#include "vn/thread.h"

namespace magnecko_vectornav 
{
    class MagneckoVectornav : public hardware_interface::SensorInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(MagneckoVectornav);
            
            MAGNECKO_VECTORNAV_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

            MAGNECKO_VECTORNAV_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);

            MAGNECKO_VECTORNAV_HARDWARE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            MAGNECKO_VECTORNAV_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            MAGNECKO_VECTORNAV_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            MAGNECKO_VECTORNAV_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

            MAGNECKO_VECTORNAV_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;            

            MAGNECKO_VECTORNAV_HARDWARE_PUBLIC
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            
        private:
            std::string frame_id_;
            double hw_start_sec_;
            double hw_stop_sec_;
            
            bool configure_sensor();
            bool connect(const std::string port, const int baud);
            void readingIMU();
            //
            // Member Variables
            //
            std::string port_;
            int baud_;
            std::chrono::milliseconds reconnect_ms_;
            bool adjustROSTimeStamp_ = false;

            sensor_msgs::msg::Imu imu_data_;
            double time_stamp_sec_;
            double time_stamp_nano_;
            
            /// VectorNav Sensor Handle
            vn::sensors::EzAsyncData* ez_;

            /// ROS header time stamp adjustments
            double averageTimeDifference_{0};
    };
}
#endif