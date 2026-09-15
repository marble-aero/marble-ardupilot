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
  simple plane simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_ICEngine.h"
#include <Filter/LowPassFilter.h>
#include <AP_JSON/AP_JSON.h>

namespace SITL {

/*
  a very simple plane simulator
 */
class Plane : public Aircraft {
public:
    Plane(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW Plane(frame_str);
    }

protected:
    float angle_of_attack;
    float beta;

    const struct Coefficients {
        // from last_letter skywalker_2013/aerodynamics.yaml
        // thanks to Georacer!

        /*
          mass and inertia. These default to zero/unity so that models
          which don't specify them keep the legacy behaviour.
         */
        // total vehicle mass in kg. Zero means use the frame string default
        // (2kg, or 8kg for -heavy, 22kg for -jet)
        float mass = 0;
        // moment of inertia about the body x/y/z axes in kg.m^2. The legacy
        // model applied aerodynamic torque directly as angular acceleration,
        // which is equivalent to unity inertia, so that is the default
        Vector3f moment_inertia{1, 1, 1};

        /*
          propulsion
         */
        // thrust in newtons at full throttle. Zero means derive it from
        // hover_throttle, which is the legacy behaviour
        float max_thrust = 0;
        // throttle fraction (0 to 1) at which thrust equals weight. Only used
        // when max_thrust is zero
        float hover_throttle = 0.7;

        /*
          battery model. Only used for the plain plane model, the quadplane
          takes voltage and current from the multicopter frame
         */
        // volts dropped from SIM_BATT_VOLTAGE at full throttle
        float batt_volt_drop = 0.7;
        // amps drawn at full throttle
        float batt_max_amps = 50;
        // amps drawn by the forward motor at full throttle (quadplane only,
        // added on top of the VTOL frame current)
        float fwd_batt_amps = 20;

        // height of the model origin above the ground in m
        float frame_height = 0.1;

        float s = 0.45;
        float b = 1.88;
        float c = 0.24;
        float c_lift_0 = 0.56;
        float c_lift_deltae = 0;
        float c_lift_a = 6.9;
        float c_lift_q = 0;
        float mcoeff = 50;
        float oswald = 0.9;
        float alpha_stall = 0.4712;
        float c_drag_q = 0;
        float c_drag_deltae = 0.0;
        float c_drag_p = 0.1;
        float c_y_0 = 0;
        float c_y_b = -0.98;
        float c_y_p = 0;
        float c_y_r = 0;
        float c_y_deltaa = 0;
        float c_y_deltar = -0.2;
        float c_l_0 = 0;
        float c_l_p = -1.0;
        float c_l_b = -0.12;
        float c_l_r = 0.14;
        float c_l_deltaa = 0.25;
        float c_l_deltar = -0.037;
        float c_m_0 = 0.045;
        float c_m_a = -0.7;
        float c_m_q = -20;
        float c_m_deltae = 1.0;
        float c_n_0 = 0;
        float c_n_b = 0.25;
        float c_n_p = 0.022;
        float c_n_r = -1;
        float c_n_deltaa = 0.00;
        float c_n_deltar = 0.1;
        float deltaa_max = 0.3491;
        float deltae_max = 0.3491;
        float deltar_max = 0.3491;
        // the X CoG offset should be -0.02, but that makes the plane too tail heavy
        // in manual flight. Adjusted to -0.15 gives reasonable flight
        Vector3f CGOffset{-0.15, 0, -0.05};
    } default_coefficients;

    struct Coefficients coefficient;

    float thrust_scale;
    bool reverse_thrust;
    bool elevons;
    bool vtail;
    bool dspoilers;
    bool redundant;
    bool reverse_elevator_rudder;
    bool ice_engine;
    bool tailsitter;
    bool aerobatic;
    bool copter_tailsitter;
    bool have_launcher;
    bool have_steering;
    float launch_accel;
    float launch_time;
    uint64_t launch_start_ms;

    const uint8_t throttle_servo = 2;
    const int8_t choke_servo = 14;
    const int8_t ignition_servo = 12;
    const int8_t starter_servo = 13;
    const float slewrate = 100;
    ICEngine icengine{
        throttle_servo,
        choke_servo,
        ignition_servo,
        starter_servo,
        slewrate,
        true
    };

    // load aero coefficients from a json model file
    void load_coeffs(const char *model_json);
    float liftCoeff(float alpha) const;
    float dragCoeff(float alpha) const;
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel);

private:
    // json parsing helpers (TODO reduce code duplication)
    void parse_float(AP_JSON::value val, const char* label, float &param);
    void parse_vector3(AP_JSON::value val, const char* label, Vector3f &param);
};

} // namespace SITL
