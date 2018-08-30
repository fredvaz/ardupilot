/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  ROV/AUV/Submarine simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"
#include "SIM_Frame.h"

/* ADDED */
#include <AP_HAL/utility/Socket.h>


namespace SITL {

/*
  a submarine simulator
 */


class Submarine : public Aircraft {
public:
    Submarine(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Submarine(home_str, frame_str);
    }

    /*  ADDED: Create and set in/out socket for Gazebo simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out);


private:
    /*
      packet sent to Gazebo
    */
    struct servo_packet {
      // size matches sitl_input upstream
        float motor_speed[16];
    };

    /*
      reply packet sent from Gazebo to ArduPilot
    */
    struct fdm_packet {
      double timestamp;  // in seconds
      double imu_angular_velocity_rpy[3];
      double imu_linear_acceleration_xyz[3];
      double imu_orientation_quat[4];
      double velocity_xyz[3];
      double position_xyz[3];
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);
    void drain_sockets();

    double last_timestamp;

    SocketAPM socket_sitl;
    const char *_gazebo_address = "127.0.0.1";
    int _gazebo_port = 9002;
    static const uint64_t GAZEBO_TIMEOUT_US = 5000000;

protected:

    // This is in Gazebo? not need?
    const float water_density = 1023.6; // (kg/m^3) At a temperature of 25 °C, salinity of 35 g/kg and 1 atm pressure

    const class FrameConfig {
    public:
        FrameConfig() = default;
        float length = 0.457; // x direction (meters)
        float width  = 0.338; // y direction (meters)
        float height = 0.254; // z direction (meters)
        float weight = 10.5;  // (kg)
        float net_bouyancy = 2.0; // (N)

        float bouyancy_acceleration = GRAVITY_MSS + net_bouyancy/weight;
    } frame_proprietary;

    // ??
    bool on_ground() const override;

    // calculate rotational and linear accelerations
    //void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    // calculate buoyancy
    //float calculate_buoyancy_acceleration();

    Frame *frame;
};


// ??
class Thruster {
public:
    Thruster(int8_t _servo, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, float lat_fac) :
        servo(_servo)
    {
        linear = Vector3f(forward_fac, lat_fac, -throttle_fac);
        rotational = Vector3f(roll_fac, pitch_fac, yaw_fac);
    };
    int8_t servo;
    Vector3f linear;
    Vector3f rotational;
};
}
