#include <magnecko_vectornav/magnecko_vectornav.hpp>

#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <regex>
#include <stdio.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#define real_hardware true



std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
  }



namespace magnecko_vectornav 
{
    hardware_interface::CallbackReturn MagneckoVectornav::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SensorInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
        hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

        //get the serial port
        std::string ls_result = exec("ls -l /dev/serial/by-id/");

        std::stringstream ss(ls_result);
        std::string to;
        std::string device;

        if (!ls_result.empty())
        {
        while(std::getline(ss,to,'\n')){
            std::regex name(".*FTDI_USB-RS232_Cable_AU05REYI-if00-port0.*", std::regex_constants::ECMAScript);
            if(std::regex_match(to,name)){
                std::regex search(".*FTDI_USB-RS232_Cable_AU05REYI-if00-port0.*\.\.\/\.\.\/([a-zA-Z0-9]*)(\n[\S\s]*)?$", std::regex_constants::ECMAScript);
                device = std::regex_replace(to,search,"/dev/$1");
                RCLCPP_INFO_STREAM(rclcpp::get_logger("MagneckoVectornav"), 
                      "connecting to: " << device);
            }

        } 
        }

        RCLCPP_INFO(
            rclcpp::get_logger("MagneckoVectornav"), "Initializing ...please wait...");

    
        if(device.empty()){
            return hardware_interface::CallbackReturn::ERROR;
        }

        port_ = device;
        // getting parameters from urdf
        baud_ = std::stoi(info.sensors[0].parameters.at("baud"));
        frame_id_ = info.sensors[0].parameters.at("frame_id");
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MagneckoVectornav::on_configure(const rclcpp_lifecycle::State& previous_state)
    {        
        // Connect to the sensor
        #if real_hardware
        if (!connect(port_, baud_))
            return CallbackReturn::ERROR;
        #else
        RCLCPP_INFO(rclcpp::get_logger("MagneckoVectornav"), "Imitating connection");
        #endif
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MagneckoVectornav::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &imu_data_.orientation.x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &imu_data_.orientation.y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &imu_data_.orientation.z));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &imu_data_.orientation.w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &imu_data_.angular_velocity.x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &imu_data_.angular_velocity.y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &imu_data_.angular_velocity.z));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &imu_data_.linear_acceleration.x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &imu_data_.linear_acceleration.y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &imu_data_.linear_acceleration.z));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[10].name, &time_stamp_sec_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[11].name, &time_stamp_nano_));
        
        return state_interfaces;
    }

    hardware_interface::CallbackReturn MagneckoVectornav::on_activate(const rclcpp_lifecycle::State & previous_state)
    {

        RCLCPP_INFO(
            rclcpp::get_logger("MagneckoVectornav"), "Activating ...please wait...");

        for (int i = 0; i < hw_start_sec_; i++)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(
            rclcpp::get_logger("MagneckoVectornav"), "%.1f seconds left...",
            hw_start_sec_ - i);
        }

        RCLCPP_INFO(
            rclcpp::get_logger("MagneckoVectornav"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MagneckoVectornav::on_cleanup(const rclcpp_lifecycle::State& previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MagneckoVectornav::on_error(const rclcpp_lifecycle::State& previous_state)
    {
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MagneckoVectornav::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        #if real_hardware
        ez_->disconnect();

        delete ez_;
        #endif
        RCLCPP_INFO(rclcpp::get_logger("Magnecko"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MagneckoVectornav::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        time_stamp_sec_ = time.seconds();
        time_stamp_nano_ =time.nanoseconds();

        MagneckoVectornav::readingIMU();
        return hardware_interface::return_type::OK;
    }

    void MagneckoVectornav::readingIMU()
    {
        // TODO: How to use time to store time stamp value
        // TODO: Use ez_->getNextData() instead of currentData()
        vn::sensors::CompositeData cd = ez_->currentData();
        
        // defining local variables to make conversion to standard 
        // ros messages easier
        vn::math::vec4f quaternions;
        vn::math::vec3f lin_acceleration;
        vn::math::vec3f angular_acceleration;

        // getting quaternions
        #if real_hardware
        if (cd.hasAnyAttitude())
        {
            quaternions = cd.anyAttitude().quat();
        }

        // getting linear acceleration
        if (cd.hasAcceleration())
        {
            lin_acceleration = cd.acceleration();
        }

        // getting angular acceleration
        if (cd.hasAngularRate())
        {
            angular_acceleration = cd.angularRate();
        }

        // quaternions
        imu_data_.orientation.set__x(quaternions.x);
        imu_data_.orientation.set__y(quaternions.y);
        imu_data_.orientation.set__z(quaternions.z);
        imu_data_.orientation.set__w(quaternions.w);

        // linear acceleration
        imu_data_.linear_acceleration.set__x(lin_acceleration.x);
        imu_data_.linear_acceleration.set__y(lin_acceleration.y);
        imu_data_.linear_acceleration.set__z(lin_acceleration.z);

        // angular rate
        imu_data_.angular_velocity.set__x(angular_acceleration.x);
        imu_data_.angular_velocity.set__y(angular_acceleration.y);
        imu_data_.angular_velocity.set__z(angular_acceleration.z);

        #else
        // setting the imu message value to the measurement values
        // orientation
        imu_data_.orientation.set__x(10);
        imu_data_.orientation.set__y(10);
        #endif
    }


    /**
   * Connect to a sensor
   *
   * \param port serial port path, eg /dev/ttyUSB0
   * \param baud baud rate to use, 0 for automatic
   *             Will automatically try all supported rates on failure and configure
   *             the device for the requested baud rate.
   * \return     true: OK, false: FAILURE
   */

    bool MagneckoVectornav::connect(const std::string port, const int baud)
    {
        imu_data_.header.set__frame_id(frame_id_);

        // establish connection Riccardo's version
        try {            
            ez_ = vn::sensors::EzAsyncData::connect(port_, baud_);
        } catch (...)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MagneckoVectornav"), "Connection failed => did you plug in the vectornav?");
        }

        if (ez_->sensor()->isConnected())
            return MagneckoVectornav::configure_sensor();
            // return true;
        else
            return false;
    }

      /**
     * Configures a sensor based on the parameter configuration loaded by the node
     * 
     * \return true for OK configuration, false for an error
     */
    bool MagneckoVectornav::configure_sensor(){
        vn::sensors::VnSensor* vs_ = ez_->sensor();
        // Async Output Type
        // 5.2.7
        RCLCPP_INFO(rclcpp::get_logger("configuration"), "async output type");
        auto AsyncDataOutputType = (vn::protocol::uart::AsciiAsync)std::stoi(info_.sensors[0].parameters.at("AsyncDataOutputType"));
        vs_->writeAsyncDataOutputType(AsyncDataOutputType);

        // Async output Frequency (Hz)
        // 5.2.8
        RCLCPP_INFO(rclcpp::get_logger("configuration"), "async output frequency");
        int AsyncDataOutputFreq = std::stoi(info_.sensors[0].parameters.at("AsyncDataOutputFrequency"));
        vs_->writeAsyncDataOutputFrequency(AsyncDataOutputFreq);

        // Sync control
        // 5.2.9
        RCLCPP_INFO(rclcpp::get_logger("configuration"), "sync control");
        vn::sensors::SynchronizationControlRegister configSync = {
            (vn::protocol::uart::SyncInMode)std::stoi(info_.sensors[0].parameters.at("syncInMode")),
            (vn::protocol::uart::SyncInEdge)std::stoi(info_.sensors[0].parameters.at("syncInEdge")),
            static_cast<uint16_t>(std::stoi(info_.sensors[0].parameters.at("syncInSkipFactor"))),
            (vn::protocol::uart::SyncOutMode)std::stoi(info_.sensors[0].parameters.at("syncOutMode")),
            (vn::protocol::uart::SyncOutPolarity)std::stoi(info_.sensors[0].parameters.at("syncOutPolarity")),
            static_cast<uint16_t>(std::stoi(info_.sensors[0].parameters.at("syncOutSkipFactor"))),
            static_cast<uint32_t>(std::stoi(info_.sensors[0].parameters.at("syncOutPulseWidth_ns")))
        };
        vs_->writeSynchronizationControl(configSync);

        // Communication Protocol Control
        // 5.2.10
        RCLCPP_INFO(rclcpp::get_logger("configuration"), "communication protocol");
        vn::sensors::CommunicationProtocolControlRegister configComm = {
            (vn::protocol::uart::CountMode)std::stoi(info_.sensors[0].parameters.at("serialCount")),
            (vn::protocol::uart::StatusMode)std::stoi(info_.sensors[0].parameters.at("serialStatus")),
            (vn::protocol::uart::CountMode)std::stoi(info_.sensors[0].parameters.at("spiCount")),
            (vn::protocol::uart::StatusMode)std::stoi(info_.sensors[0].parameters.at("spiStatus")),
            (vn::protocol::uart::ChecksumMode)std::stoi(info_.sensors[0].parameters.at("serialChecksum")),
            (vn::protocol::uart::ChecksumMode)std::stoi(info_.sensors[0].parameters.at("spiChecksum")),
            (vn::protocol::uart::ErrorMode)std::stoi(info_.sensors[0].parameters.at("errorMode"))
        };

        vs_->writeCommunicationProtocolControl(configComm);

        auto boRegs = std::vector<std::string>{"BO1", "BO2", "BO3"};
        auto boConfigs = std::vector<vn::sensors::BinaryOutputRegister>();

        for(auto name : boRegs){
            vn::sensors::BinaryOutputRegister configBO = {
                (vn::protocol::uart::AsyncMode)std::stoi(info_.sensors[0].parameters.at(name + ".asyncMode")),
                static_cast<uint16_t>(std::stoi(info_.sensors[0].parameters.at(name + ".rateDivisor"))),
                (vn::protocol::uart::CommonGroup)std::stol(info_.sensors[0].parameters.at(name + ".commonField"), nullptr, 16),
                (vn::protocol::uart::TimeGroup)std::stol(info_.sensors[0].parameters.at(name + ".timeField"), nullptr, 16),
                (vn::protocol::uart::ImuGroup)std::stol(info_.sensors[0].parameters.at(name + ".imuField"), nullptr, 16),
                (vn::protocol::uart::GpsGroup)std::stol(info_.sensors[0].parameters.at(name + ".gpsField"), nullptr, 16),
                (vn::protocol::uart::AttitudeGroup)std::stol(info_.sensors[0].parameters.at(name + ".attitudeField"), nullptr, 16),
                (vn::protocol::uart::InsGroup)std::stol(info_.sensors[0].parameters.at(name + ".insField"), nullptr, 16),
                (vn::protocol::uart::GpsGroup)std::stol(info_.sensors[0].parameters.at(name + ".gps2Field"), nullptr, 16)
            };
            boConfigs.push_back(configBO);
        }

        // Binary Output Register 1
        // 5.2.11
        vs_->writeBinaryOutput1(boConfigs.at(0));

        // Binary Output Register 2
        // 5.2.12
        vs_->writeBinaryOutput2(boConfigs.at(1));

        // Binary Output Register 3
        // 5.2.13
        vs_->writeBinaryOutput3(boConfigs.at(2));


        // Connection Successful
        return true;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  magnecko_vectornav::MagneckoVectornav,
  hardware_interface::SensorInterface)