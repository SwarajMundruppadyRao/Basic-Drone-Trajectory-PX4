/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <atomic>
#include <cmath>

// ----------------------
// Adjust these as needed
// ----------------------

#define FLIGHT_ALTITUDE  -1.0f   // Starting altitude (NED => negative = above ground)
#define RATE             30      // loop rate Hz
#define RADIUS           1.0f    // Helix radius in XY
#define CYCLE_S          20      // time (s) for one revolution
#define STEPS_PER_REV    (RATE * CYCLE_S) // e.g. 20s * 30Hz = 600
#define WAIT_STEPS        (3 * RATE)  // 3 seconds at 30 Hz = 90 steps
#define TOT_STEPS         (2 * STEPS_PER_REV + 2 * WAIT_STEPS)  // Add wait times



#define PI 3.14159265358979323846

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// ------------------------------------------------

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ =
            this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ =
            this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        vehicle_command_listener_ =
            this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode",
                                                        qos,
                                                        [this](const VehicleControlMode::UniquePtr msg)
                                                        {
                                                            c_mode = *msg;
                                                        });

        offboard_setpoint_counter_ = 0;

        InitPath(); // Build the up+down helix path in the array

        // Timer at 50 ms => 20 Hz
        // We want to keep publishing offboard commands at ~20 Hz or more
        // (The path is prepared for 30 Hz, but there's no strict requirement
        //  as long as you keep the times in sync with the flight controller.)
        timer_ = this->create_wall_timer(50ms, [this]() {
            // Publish offboard_control_mode less frequently
            double intpart;
            if(std::modf((double)offboard_setpoint_counter_ / 2.0, &intpart) == 0.0)
            {
                publish_offboard_control_mode();
            }
            publish_trajectory_setpoint();
        });
    }

private:
    // Node members
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_command_listener_;

    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_; // cycles over 0..TOT_STEPS-1

    // We store TOT_STEPS points for the up+down path
    TrajectorySetpoint path[TOT_STEPS];

    VehicleControlMode c_mode;

    // Private methods
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void InitPath();
};

// ------------------------------------------------

void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    // We'll do position-based control, so position=true
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    // Current time in microseconds
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
    // Move to the next path index
    offboard_setpoint_counter_++;
    if(offboard_setpoint_counter_ >= TOT_STEPS)
    {
        offboard_setpoint_counter_ = 0;
    }

    // Update the timestamp in the setpoint
    path[offboard_setpoint_counter_].timestamp =
        this->get_clock()->now().nanoseconds() / 1000;

    // Only publish if the flight controller is in Offboard mode
    if(c_mode.flag_control_offboard_enabled == 1)
    {
        trajectory_setpoint_publisher_->publish(path[offboard_setpoint_counter_]);
        printf("(%3lu/%3d) x: %7.3f y: %7.3f z: %7.3f yaw: %7.1f\n",
            (unsigned long)offboard_setpoint_counter_,
            TOT_STEPS,
            path[offboard_setpoint_counter_].position[0],
            path[offboard_setpoint_counter_].position[1],
            path[offboard_setpoint_counter_].position[2],
            path[offboard_setpoint_counter_].yaw * 180.0f / PI);
    }
    else
    {
        // If not in offboard, reset to 0 so that we start from the beginning
        offboard_setpoint_counter_ = 0;
    }
}

// ------------------------------------------------
// Build the up+down helix path in a single array
// ------------------------------------------------
void OffboardControl::InitPath()
{
    const int STEPS_UP = STEPS_PER_REV;
    const int STEPS_DOWN = STEPS_PER_REV;
    const int STEPS_WAIT = WAIT_STEPS; // 3-second pause

    const double dt = 1.0 / RATE;
    const double omega_up = (2.0 * PI) / CYCLE_S;
    const double omega_down = -omega_up;

    const double r = RADIUS;
    const double z0 = FLIGHT_ALTITUDE;

    const double climb_rate_up = -0.05;  // Going more negative in NED (up)
    const double climb_rate_down = +0.05; // Going less negative in NED (down)

    int idx = 0;

    // -----------------------
    // 1) Up Helix (ascending)
    // -----------------------
    for (int i = 0; i < STEPS_UP; i++, idx++)
    {
        double t = i * dt;
        double angle = omega_up * t;

        path[idx].position[0] = r * std::cos(angle);
        path[idx].position[1] = r * std::sin(angle);
        path[idx].position[2] = z0 + climb_rate_up * t;

        path[idx].velocity[0] = -r * omega_up * std::sin(angle);
        path[idx].velocity[1] =  r * omega_up * std::cos(angle);
        path[idx].velocity[2] = climb_rate_up;

        path[idx].acceleration[0] = -r * (omega_up * omega_up) * std::cos(angle);
        path[idx].acceleration[1] = -r * (omega_up * omega_up) * std::sin(angle);
        path[idx].acceleration[2] = 0.0;

        path[idx].yaw = std::atan2(-path[idx].position[1], -path[idx].position[0]);
    }

    // -----------------------
    // 2) Wait at Top (3 seconds)
    // -----------------------
    for (int i = 0; i < STEPS_WAIT; i++, idx++)
    {
        path[idx] = path[idx - 1];  // Copy last position and hold
    }

    // -----------------------
    // 3) Down Helix (descending)
    // -----------------------
    for (int i = 0; i < STEPS_DOWN; i++, idx++)
    {
        double t2 = i * dt;
        double angle_down = (2.0 * PI) + (omega_down * t2);

        path[idx].position[0] = r * std::cos(angle_down);
        path[idx].position[1] = r * std::sin(angle_down);
        path[idx].position[2] = (z0 + (climb_rate_up * CYCLE_S)) + climb_rate_down * t2;

        path[idx].velocity[0] = -r * omega_down * std::sin(angle_down);
        path[idx].velocity[1] =  r * omega_down * std::cos(angle_down);
        path[idx].velocity[2] = climb_rate_down;

        path[idx].acceleration[0] = -r * (omega_down * omega_down) * std::cos(angle_down);
        path[idx].acceleration[1] = -r * (omega_down * omega_down) * std::sin(angle_down);
        path[idx].acceleration[2] = 0.0;

        path[idx].yaw = std::atan2(-path[idx].position[1], -path[idx].position[0]);
    }

    // -----------------------
    // 4) Wait at Bottom (3 seconds)
    // -----------------------
    for (int i = 0; i < STEPS_WAIT; i++, idx++)
    {
        path[idx] = path[idx - 1];  // Copy last position and hold
    }

    // -----------------------
    // 5) Calculate Yawspeed for Smooth Rotation
    // -----------------------
    for (int i = 0; i < TOT_STEPS; i++)
    {
        int next_i = (i + 1) % TOT_STEPS;
        double next_yaw = path[next_i].yaw;
        double curr_yaw = path[i].yaw;

        if ((next_yaw - curr_yaw) < -PI) { next_yaw += 2.0 * PI; }
        if ((next_yaw - curr_yaw) >  PI) { next_yaw -= 2.0 * PI; }

        path[i].yawspeed = (next_yaw - curr_yaw) * RATE;
    }
}


// ------------------------------------------------

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node (Up+Down Helix)..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();

    return 0;
}
