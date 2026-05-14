#include "odrive_can_node.hpp"
#include "odrive_enums.h"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>

const double VEL_GAIN_DEFAULT            = 0.16;
const double VEL_INTEGRATOR_GAIN_DEFAULT = 0.33;

enum CmdId : uint32_t {
    kHeartbeat = 0x001,            // ControllerStatus  - publisher
    kGetError = 0x003,             // SystemStatus      - publisher
    kRxSdo,
    kTxSdo,
    kSetAxisState = 0x007,         // SetAxisState      - service
    kGetEncoderEstimates = 0x009,  // ControllerStatus  - publisher
    kSetControllerMode = 0x00b,    // ControlMessage    - subscriber
    kSetInputPos,                  // ControlMessage    - subscriber
    kSetInputVel,                  // ControlMessage    - subscriber
    kSetInputTorque,               // ControlMessage    - subscriber
    kGetIq = 0x014,                // ControllerStatus  - publisher
    kGetTemp,                      // SystemStatus      - publisher
    kGetBusVoltageCurrent = 0x017, // SystemStatus      - publisher
    kClearErrors = 0x018,          // ClearErrors       - service
    kSetVelGains = 0x01b,
    kGetTorques = 0x01c,           // ControllerStatus  - publisher
};

enum ControlMode : uint64_t {
    kVoltageControl,
    kTorqueControl,
    kVelocityControl,
    kPositionControl,
};

enum NodeId : uint32_t {
    kMotorLeft  = 0x00,
    kMotorRight = 0x01,
};

enum OpcodeId : uint8_t {
    kRead  = 0x00,
    kWrite = 0x01,
};

ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {
    
    rclcpp::Node::declare_parameter<std::string>("interface", "can0");
    rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);
    rclcpp::Node::declare_parameter<bool>("axis_idle_on_shutdown", false);
    rclcpp::Node::declare_parameter<std::string>("json_file_path", "PATH_TO_SHARE/PACKAGE/FIRMWARE_VERSION/flat_endpoints.json");

    // get endpoint id
    std::string json_file_path = rclcpp::Node::get_parameter("json_file_path").as_string();
    params_.init(json_file_path);

    // initiate parameter callback of "vel_gain"
    std::string vel_gain_param_name = "vel_gain";
    rclcpp::Node::declare_parameter<double>(vel_gain_param_name, VEL_GAIN_DEFAULT);
    vel_gain_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    std::function<void(const rclcpp::Parameter&)> vel_gain_cb =
        [this](const rclcpp::Parameter & p) {
            double vel_gain = p.as_double();
            this->vel_gain_ = vel_gain;
            while(rclcpp::ok()) {
                this->set_vel_gains();
                if(this->is_gain_correct()) break;
            }
        };
    vel_gain_cb_handle_ =
        vel_gain_subscriber_->add_parameter_callback(
                vel_gain_param_name,
                vel_gain_cb
                );

    // initiate parameter callback of "vel_integrator_gain"
    std::string vel_integrator_gain_param_name = "vel_integrator_gain";
    rclcpp::Node::declare_parameter<double>(vel_integrator_gain_param_name, VEL_INTEGRATOR_GAIN_DEFAULT);
    vel_integrator_gain_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    std::function<void(const rclcpp::Parameter&)> vel_integrator_gain_cb =
        [this](const rclcpp::Parameter & p) {
            double vel_integrator_gain = p.as_double();
            this->vel_integrator_gain_ = vel_integrator_gain;
            while(true) {
                this->set_vel_gains();
                if(this->is_gain_correct()) break;
            }
        };
    vel_integrator_gain_cb_handle_ =
        vel_integrator_gain_subscriber_->add_parameter_callback(
                vel_integrator_gain_param_name,
                vel_integrator_gain_cb
                );

    rclcpp::QoS ctrl_stat_qos(rclcpp::KeepAll{});
    ctrl_publisher_ = rclcpp::Node::create_publisher<ControllerStatus>("controller_status", ctrl_stat_qos);
    
    rclcpp::QoS odrv_stat_qos(rclcpp::KeepAll{});
    odrv_publisher_ = rclcpp::Node::create_publisher<ODriveStatus>("odrive_status", odrv_stat_qos);

    rclcpp::QoS ctrl_msg_qos(rclcpp::KeepAll{});
    subscriber_ = rclcpp::Node::create_subscription<ControlMessage>("control_message", ctrl_msg_qos, std::bind(&ODriveCanNode::subscriber_callback, this, _1));

    rclcpp::QoS srv_qos(rclcpp::KeepAll{});

#if RCLCPP_VERSION_MAJOR >= 18 
    // For ros2 jazzy and above. 
    // PR about deprecation of get_rmw_qos_profile: 
    //  - https://github.com/ros2/rclcpp/pull/713
    //  - https://github.com/ros2/rclcpp/pull/1969
    auto srv_qos_profile = srv_qos;
#else
    auto srv_qos_profile = srv_qos.get_rmw_qos_profile();
#endif

    service_ = rclcpp::Node::create_service<AxisState>("request_axis_state", std::bind(&ODriveCanNode::service_callback, this, _1, _2), srv_qos_profile);
    service_clear_errors_ = rclcpp::Node::create_service<Empty>("clear_errors", std::bind(&ODriveCanNode::service_clear_errors_callback, this, _1, _2), srv_qos_profile);
}

void ODriveCanNode::deinit() {
    if (axis_idle_on_shutdown_) {
        struct can_frame frame = {};
        frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
        write_le<uint32_t>(ODriveAxisState::AXIS_STATE_IDLE, frame.data);
        frame.can_dlc = 4;
        can_intf_.send_can_frame(frame);
    }

    sub_evt_.deinit();
    srv_evt_.deinit();
    can_intf_.deinit();
}

bool ODriveCanNode::init(EpollEventLoop* event_loop) {

    node_id_ = rclcpp::Node::get_parameter("node_id").as_int();
    axis_idle_on_shutdown_ = rclcpp::Node::get_parameter("axis_idle_on_shutdown").as_bool();
    std::string interface = rclcpp::Node::get_parameter("interface").as_string();

    if (!can_intf_.init(interface, event_loop, std::bind(&ODriveCanNode::recv_callback, this, _1))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize socket can interface: %s", interface.c_str());
        return false;
    }
    if (!sub_evt_.init(event_loop, std::bind(&ODriveCanNode::ctrl_msg_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize subscriber event");
        return false;
    }
    if (!srv_evt_.init(event_loop, std::bind(&ODriveCanNode::request_state_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize service event");
        return false;
    }
    if (!srv_clear_errors_evt_.init(event_loop, std::bind(&ODriveCanNode::request_clear_errors_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize clear errors service event");
        return false;
    }
    RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d", node_id_);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "interface: %s", interface.c_str());
    return true;
}

void ODriveCanNode::set_vel_gains() {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kSetVelGains;
    {
        write_le<float>(vel_gain_,            frame.data + 0);
        write_le<float>(vel_integrator_gain_, frame.data + 4);
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::recv_callback(const can_frame& frame) {

    if(((frame.can_id >> 5) & 0x3F) != node_id_) return;

    switch(frame.can_id & 0x1F) {
        case CmdId::kHeartbeat: {
            if (!verify_length("kHeartbeat", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.active_errors    = read_le<uint32_t>(frame.data + 0);
            ctrl_stat_.axis_state        = read_le<uint8_t>(frame.data + 4);
            ctrl_stat_.procedure_result  = read_le<uint8_t>(frame.data + 5);
            ctrl_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
            ctrl_pub_flag_ |= 0b0001;
            fresh_heartbeat_.notify_one();
            break;
        }
        case CmdId::kTxSdo: {
            if (!verify_length("kTxSdo", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            //uint8_t  reserved0    = read_le<uint8_t>(frame.data + 0);
            uint16_t endpoint_id = read_le<uint16_t>(frame.data + 1);
            //uint8_t  reserved1    = read_le<uint8_t>(frame.data + 3);
            uint32_t raw_val     = read_le<uint32_t>(frame.data + 4);

            if(params_.get_type(endpoint_id) == typeid(bool).name()) {
                bool val;
                memcpy(&val, &raw_val, sizeof(bool));
                params_.set_fresh<bool>(endpoint_id, val);
                RCLCPP_INFO(rclcpp::Node::get_logger(), "recv_callback: node_id: %d, endpoint_id: %d,"
                        " type: bool, value: %d", node_id_, endpoint_id, val);
            } else if(params_.get_type(endpoint_id) == typeid(uint8_t).name()) {
                uint8_t val;
                memcpy(&val, &raw_val, sizeof(uint8_t));
                RCLCPP_ERROR(rclcpp::Node::get_logger(), "recv_callback: uint8_t parameter is not currently supported");
                //params_.set_fresh<uint8_t>(endpoint_id, val);
            } else if(params_.get_type(endpoint_id) == typeid(uint16_t).name()) {
                uint16_t val;
                memcpy(&val, &raw_val, sizeof(uint16_t));
                RCLCPP_ERROR(rclcpp::Node::get_logger(), "recv_callback: uint16_t parameter is not currently supported");
                //params_.set_fresh<uint16_t>(endpoint_id, val);
            } else if(params_.get_type(endpoint_id) == typeid(uint32_t).name()) {
                uint32_t val;
                memcpy(&val, &raw_val, sizeof(uint32_t));
                RCLCPP_ERROR(rclcpp::Node::get_logger(), "recv_callback: uint32_t parameter is not currently supported");
                //params_.set_fresh<uint32_t>(endpoint_id, val);
            } else if(params_.get_type(endpoint_id) == typeid(uint64_t).name()) {
                uint64_t val;
                memcpy(&val, &raw_val, sizeof(uint64_t));
                RCLCPP_ERROR(rclcpp::Node::get_logger(), "recv_callback: uint64_t parameter is not currently supported");
                //params_.set_fresh<uint64_t>(endpoint_id, val);
            } else if(params_.get_type(endpoint_id) == typeid(int32_t).name()) {
                int32_t val;
                memcpy(&val, &raw_val, sizeof(int32_t));
                RCLCPP_ERROR(rclcpp::Node::get_logger(), "recv_callback: int32_t parameter is not currently supported");
                //params_.set_fresh<int32_t>(endpoint_id, val);
            } else if(params_.get_type(endpoint_id) == typeid(int64_t).name()) {
                int64_t val;
                memcpy(&val, &raw_val, sizeof(int64_t));
                RCLCPP_ERROR(rclcpp::Node::get_logger(), "recv_callback: int64_t parameter is not currently supported");
                //params_.set_fresh<int64_t>(endpoint_id, val);
            } else if(params_.get_type(endpoint_id) == typeid(float).name()) {
                float val;
                memcpy(&val, &raw_val, sizeof(float));
                params_.set_fresh<float>(endpoint_id, val);
                RCLCPP_INFO(rclcpp::Node::get_logger(), "recv_callback: node_id: %d, endpoint_id: %d,"
                        " type: float, value: %f", node_id_, endpoint_id, val);
            } else {
                // type does not exist
                RCLCPP_ERROR(rclcpp::Node::get_logger(),
                        "recv_callback: endpoint_id %d does not exist or callback "
                        "process of the endpoint_id is not implemented", endpoint_id);
            }
            break;
        }
        case CmdId::kGetError: {
            if (!verify_length("kGetError", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
            odrv_pub_flag_ |= 0b001;
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            if (!verify_length("kGetEncoderEstimates", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
            ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0010;
            break;
        }
        case CmdId::kGetIq: {
            if (!verify_length("kGetIq", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0100;
            break;
        }
        case CmdId::kGetTemp: {
            if (!verify_length("kGetTemp", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.fet_temperature   = read_le<float>(frame.data + 0);
            odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b010;
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            if (!verify_length("kGetBusVoltageCurrent", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
            odrv_stat_.bus_current = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b100;
            break;
        }
        case CmdId::kGetTorques: {
            if (!verify_length("kGetTorques", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.torque_target   = read_le<float>(frame.data + 0);
            ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b1000; 
            break;
        }
        case CmdId::kSetAxisState:
        case CmdId::kSetControllerMode:
        case CmdId::kSetInputPos:
        case CmdId::kSetInputVel:
        case CmdId::kSetInputTorque:
        case CmdId::kClearErrors: {
            break; // Ignore commands coming from another master/host on the bus
        }
        default: {
            RCLCPP_WARN(rclcpp::Node::get_logger(), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
            break;
        }
    }
    
    if (ctrl_pub_flag_ == 0b1111) {
        ctrl_publisher_->publish(ctrl_stat_);
        ctrl_pub_flag_ = 0;
    }
    
    if (odrv_pub_flag_ == 0b111) {
        odrv_publisher_->publish(odrv_stat_);
        odrv_pub_flag_ = 0;
    }
}

void ODriveCanNode::subscriber_callback(const ControlMessage::SharedPtr msg) {
    std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
    ctrl_msg_ = *msg;
    sub_evt_.set();
}

void ODriveCanNode::service_callback(const std::shared_ptr<AxisState::Request> request, std::shared_ptr<AxisState::Response> response) {
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state_ = request->axis_requested_state;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting axis state: %d", axis_state_);
    }
    srv_evt_.set();

    // Wait for at least 1 second for a new heartbeat to arrive.
    // If the requested state is something other than CLOSED_LOOP_CONTROL, also
    // wait for the procedure to complete (procedure_result != BUSY).
    std::unique_lock<std::mutex> guard(ctrl_stat_mutex_); // define lock for controller status
    auto call_time = std::chrono::steady_clock::now();
    fresh_heartbeat_.wait(guard, [this, &call_time, &request]() {
        bool is_busy = this->ctrl_stat_.procedure_result == ODriveProcedureResult::PROCEDURE_RESULT_BUSY;
        bool requested_closed_loop = request->axis_requested_state == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
        bool minimum_time_passed = (std::chrono::steady_clock::now() - call_time >= std::chrono::seconds(1));
        bool complete = (requested_closed_loop || !is_busy) && minimum_time_passed;
        return complete;
        }); // wait for procedure_result
    
    response->axis_state = ctrl_stat_.axis_state;
    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}

void ODriveCanNode::service_clear_errors_callback(const std::shared_ptr<Empty::Request> /*request*/, std::shared_ptr<Empty::Response> /*response*/) {
    RCLCPP_INFO(rclcpp::Node::get_logger(), "clearing errors");
    srv_clear_errors_evt_.set();
}

void ODriveCanNode::request_state_callback() {
    uint32_t axis_state;
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state = axis_state_;
    }

    struct can_frame frame;

    if (axis_state != 0) {
        // Clear errors if requested state is not IDLE
        frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
        write_le<uint8_t>(0, frame.data);
        frame.can_dlc = 1;
        can_intf_.send_can_frame(frame);
    }

    // Set state
    frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
    write_le<uint32_t>(axis_state, frame.data);
    frame.can_dlc = 4;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::request_clear_errors_callback() {
    struct can_frame frame = {};
    frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
    write_le<uint8_t>(0, frame.data);
    frame.can_dlc = 1;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::ctrl_msg_callback() {

    uint32_t control_mode;
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetControllerMode;
    {
        std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
        write_le<uint32_t>(ctrl_msg_.control_mode, frame.data);
        write_le<uint32_t>(ctrl_msg_.input_mode,   frame.data + 4);
        control_mode = ctrl_msg_.control_mode;
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
    
    frame = can_frame{};
    switch (control_mode) {
        case ControlMode::kVoltageControl: {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "Voltage Control Mode (0) is not currently supported");
            return;
        }
        case ControlMode::kTorqueControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_torque");
            frame.can_id = node_id_ << 5 | kSetInputTorque;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_torque, frame.data);
            frame.can_dlc = 4;
            break;
        }
        case ControlMode::kVelocityControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_vel");
            frame.can_id = node_id_ << 5 | kSetInputVel;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_vel,       frame.data);
            write_le<float>(ctrl_msg_.input_torque, frame.data + 4);
            frame.can_dlc = 8;
            break;
        }
        case ControlMode::kPositionControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_pos");
            frame.can_id = node_id_ << 5 | kSetInputPos;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_pos,  frame.data);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_vel) * 1000)),    frame.data + 4);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_torque) * 1000)), frame.data + 6);
            frame.can_dlc = 8;
            break;
        }    
        default: 
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported control_mode: %d", control_mode);
            return;
    }

    can_intf_.send_can_frame(frame);
}

inline bool ODriveCanNode::verify_length(const std::string&name, uint8_t expected, uint8_t length) {
    bool valid = expected == length;
    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
    if (!valid) RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
}

bool ODriveCanNode::is_gain_correct() {
    float vel_gain_actual, vel_integrator_gain_actual;
    get_arbitrary_parameter<float>(params_.get_id("axis0.controller.config.vel_gain"), vel_gain_actual);
    get_arbitrary_parameter<float>(params_.get_id("axis0.controller.config.vel_integrator_gain"), vel_integrator_gain_actual);
    bool is_vel_gain_correct = vel_gain_ == vel_gain_actual;
    bool is_vel_integrator_gain_correct = vel_integrator_gain_ == vel_integrator_gain_actual;
    return is_vel_gain_correct & is_vel_integrator_gain_correct;
}

template <typename T>
void ODriveCanNode::get_arbitrary_parameter(uint16_t endpoint_id, T &output_val) {
    // check existence
    if(!params_.contains(endpoint_id)) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "endpoint id %d does not exist", endpoint_id);
        return;
    }
    // check variable type
    if(params_.get_type(endpoint_id) != typeid(T).name()) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "%hu is %s type, not %s", endpoint_id,
                params_.get_type_demangled(endpoint_id).c_str(),
                demangle(typeid(T).name()).c_str());
        return;
    }

    // send can frame
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kRxSdo;

    uint8_t reserved = 0;
    frame.can_dlc = 4;
    {
        write_le<uint8_t>(OpcodeId::kRead, frame.data + 0);
        write_le<uint16_t>(endpoint_id,    frame.data + 1);
        write_le<uint8_t>(reserved,        frame.data + 3);
    }
    can_intf_.send_can_frame(frame);

    // receive can frame
    params_.get_fresh<T>(endpoint_id, output_val);
    return;
}

template <typename T>
void ODriveCanNode::set_arbitrary_parameter(uint16_t endpoint_id, T val) {
    struct can_frame frame;
    for(int i=0; i<CAN_MAX_DLEN; i++) frame.data[i] = 0;

    frame.can_id = node_id_ << 5 | CmdId::kRxSdo;

    uint8_t reserved = 0;
    frame.can_dlc = 4 + sizeof(T);

    write_le<uint8_t>(OpcodeId::kWrite, frame.data + 0);
    write_le<uint16_t>(endpoint_id,     frame.data + 1);
    write_le<uint8_t>(reserved,         frame.data + 3);
    write_le<T>(val,                    frame.data + 4);
    can_intf_.send_can_frame(frame);
}
