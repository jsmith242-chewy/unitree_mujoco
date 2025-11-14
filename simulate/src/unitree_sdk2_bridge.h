#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/idl/hg/HandCmd_.hpp>

#include "param.h"
#include "physics_joystick.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    int have_imu_ = false;
    int have_frame_sensor_ = false;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        for (int i = dim_motor_sensor_; i < mj_model_->nsensor; i++)
        {
            const char *name = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
            if (strcmp(name, "imu_quat") == 0) {
                have_imu_ = true;
            }
            if (strcmp(name, "frame_pos") == 0) {
                have_frame_sensor_ = true;
            }
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            if(have_frame_sensor_) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
            }
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(have_frame_sensor_ && highstate->trylock()) {
            highstate->msg_.position()[0] = mj_data_->sensordata[dim_motor_sensor_ + 10];
            highstate->msg_.position()[1] = mj_data_->sensordata[dim_motor_sensor_ + 11];
            highstate->msg_.position()[2] = mj_data_->sensordata[dim_motor_sensor_ + 12];
            highstate->msg_.velocity()[0] = mj_data_->sensordata[dim_motor_sensor_ + 13];
            highstate->msg_.velocity()[1] = mj_data_->sensordata[dim_motor_sensor_ + 14];
            highstate->msg_.velocity()[2] = mj_data_->sensordata[dim_motor_sensor_ + 15];
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;
    }

    void run() override
    {
        RobotBridge::run();

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }


    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    std::unique_ptr<BmsState_t> bmsstate;
};

// Extremely cursed model for using the 29 DOF with hands
// It's going to be a bit hacky, with a lot of hard coded constants for now, based on the model to start
class G1WithHandsBridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1WithHandsBridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        std::cout << "G1WithHandsBridge constructor" << std::endl;
        std::cout << "param::config.robot: " << param::config.robot << std::endl;
        std::cout << "num_motor_: " << num_motor_ << std::endl;
        if (param::config.robot.find("g1_with_hands") != std::string::npos) {
            auto* g1_with_hands_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_with_hands_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_with_hands_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }
        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;
        // TODO: Hand Publishers and Subscribers for "lf" low frequency topics?
        left_hand_state = std::make_unique<HandState_t>("rt/dex3/left/state");
        left_hand_state_lf = std::make_unique<HandState_t>("rt/lf/dex3/left/state");
        left_hand_state->msg_.motor_state().resize(7);
        left_hand_state_lf->msg_.motor_state().resize(7);
        right_hand_state = std::make_unique<HandState_t>("rt/dex3/right/state");
        right_hand_state_lf = std::make_unique<HandState_t>("rt/lf/dex3/right/state");
        right_hand_state->msg_.motor_state().resize(7);
        right_hand_state_lf->msg_.motor_state().resize(7);
        left_hand_cmd = std::make_unique<HandCmd_t>("rt/dex3/left/cmd");
        right_hand_cmd = std::make_unique<HandCmd_t>("rt/dex3/right/cmd");
    }

    void run() override
    {
        // Need to have our own loop here to run, since the hands and body have separate state and control topics
        // First, we need to get the state and control messages for the body
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        
        // Handle Body Commands and report state (lowcmd and lowstate)
        
        // lowcmd processing (body commands)
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);

            // Handle up to left wrist (Mujoco Data Index 0-21)
            for(int i(0); i<=21; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }

            // Handle right arm chain through right wrist (Mujoco Data Index 29-35)
            // Mujoco Data indices 29-35 match the low_data indicies 22-28.
            // Offset of 7 is the left hand joints reported in the mujoco data.
            for(int i(22); i<=28; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i+7] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i + 7]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + 7 + num_motor_]);
            }
        }

        // Left Hand Command Processing (Mujoco Data Index 22-28)
        {
            std::lock_guard<std::mutex> lock(left_hand_cmd->mutex_);
            // Checking size in case command hasn't arrived yet
            if (left_hand_cmd->msg_.motor_cmd().size() == 7) {
                // Left hand indicies in mujoco data are 22-28
                for(int i(0); i<7; i++) {
                    auto & m = left_hand_cmd->msg_.motor_cmd()[i];
                    mj_data_->ctrl[i + 22] = m.tau() +
                                        m.kp() * (m.q() - mj_data_->sensordata[i + 22]) +
                                        m.kd() * (m.dq() - mj_data_->sensordata[i + 22 + num_motor_]);
                }
            }
        }

        // Right Hand Command Processing (Mujoco Data Index 36-42)
        {
            std::lock_guard<std::mutex> lock(right_hand_cmd->mutex_);
            // Right hand indicies in mujoco data are 36-42
            // Checking size in case command hasn't arrived yet
            if (right_hand_cmd->msg_.motor_cmd().size() == 7) {
                for(int i(0); i<7; i++) {
                    auto & m = right_hand_cmd->msg_.motor_cmd()[i];
                    mj_data_->ctrl[i + 36] = m.tau() +
                                                m.kp() * (m.q() - mj_data_->sensordata[i + 36]) +
                                                m.kd() * (m.dq() - mj_data_->sensordata[i + 36 + num_motor_]);
                }
            }
        }

        // lowstate processing
        if(lowstate->trylock()) 
        {
            for(int i(0); i<=21; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }

            for(int i(22); i<=28; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i + 7];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + 7 + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 7 + 2 * num_motor_];
            }
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }

        // Left hand state processing (Mujoco Data Index 22-28)
        if (left_hand_state->trylock()) {
            for(int i(0); i<7; i++) {
                left_hand_state->msg_.motor_state()[i].q() = mj_data_->sensordata[i + 22];
                left_hand_state->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + 22 + num_motor_];
                left_hand_state->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 22 + 2 * num_motor_];
            }
            // Debug print message before publishing
            left_hand_state->unlockAndPublish();
        }
        if (tick_count_ % lf_decimation_factor == 0 && left_hand_state_lf->trylock()) {
            for(int i(0); i<7; i++) {
                left_hand_state_lf->msg_.motor_state()[i].q() = mj_data_->sensordata[i + 22];
                left_hand_state_lf->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + 22 + num_motor_];
                left_hand_state_lf->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 22 + 2 * num_motor_];
            }
            left_hand_state_lf->unlockAndPublish();
        }

        // Righthand state processing (Mujoco Data Index 36-42)
        if (right_hand_state->trylock()) {
            for(int i(0); i<7; i++) {
                right_hand_state->msg_.motor_state()[i].q() = mj_data_->sensordata[i + 36];
                right_hand_state->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + 36 + num_motor_];
                right_hand_state->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 36 + 2 * num_motor_];
            }
            right_hand_state->unlockAndPublish();
        } 
        if (tick_count_ % lf_decimation_factor == 0 && right_hand_state_lf->trylock()) {
            for(int i(0); i<7; i++) {
                right_hand_state_lf->msg_.motor_state()[i].q() = mj_data_->sensordata[i + 36];
                right_hand_state_lf->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + 36 + num_motor_];
                right_hand_state_lf->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 36 + 2 * num_motor_];
            }
            right_hand_state_lf->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
        tick_count_++;
    }

    // Constants for actuator and joint indicies and offsets
    // Actuator Mapping
    //  29DOF With Hands Model                                         | 29 DOF Base Model
    //          0-21  (Same)                                           |    0-21 Same
    //         22-28  (Left Hand Thumb0,1,2, Middle 0,1, Index 0,1)    |    N/A
    //         39-35                                                   |    22-28
    //         36-42  (Right Hand)                                     |    N/A  
    // Num Motors should be 43
    // Joint index + 1 since Joint 0 is the unactuated floating base
    // Joint Senor Indicies are
    //    Position (Actuator Index)
    //    Velocity (Actuator Index + 43)
    //    Torque (Actuator Index + 2*43)

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    using HandState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::HandState_>;
    std::unique_ptr<HandState_t> left_hand_state;
    std::unique_ptr<HandState_t> right_hand_state;
    std::unique_ptr<HandState_t> left_hand_state_lf;
    std::unique_ptr<HandState_t> right_hand_state_lf;
    using HandCmd_t = unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::HandCmd_>;
    std::unique_ptr<HandCmd_t> left_hand_cmd;
    std::unique_ptr<HandCmd_t> right_hand_cmd;
    uint64_t tick_count_ = 0;
    const uint64_t lf_decimation_factor = 20;

};
