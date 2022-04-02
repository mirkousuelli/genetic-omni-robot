#include "omni_odometry.h"

#include <boost/math/special_functions/sign.hpp>

omni_odometry::omni_odometry(double deltaT) : dt(deltaT), t(0.0), state(3), u{0.0, 0.0, 0.0, 0.0}, omni_params_set(false)
{
    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
}

void omni_odometry::setInitialState(double x0, double y0, double theta0)
{
    // Initial state values
    state[0] = x0;
    state[1] = y0;
    state[2] = theta0;
}

void omni_odometry::setOmniParams(double r, double l, double w, double T)
{
    // Initialize vehicle parameters
    this->r = r;
    this->l = l;
    this->w = w;
    this->T = T;

    omni_params_set = true;
}

void omni_odometry::getCommands(double* u_wheels) {
    for (int i = 0; i < WHEELS; i++) {
        u_wheels[i] = u[i];
    }
};

void omni_odometry::setCommands(double* u_wheels)
{
    for (int i = 0; i < WHEELS; i++) {
        u[i] = u_wheels[i];
    }
}

void omni_odometry::integrate()
{
    // Check vehicle parameters are set
    if (!omni_params_set) {
        throw std::invalid_argument( "Vehicle parameters not set!" );
    }

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&omni_odometry::vehicle_ode, this, _1, _2, _3), state, t, dt);

    // Update time and steering
    t += dt;
}

void omni_odometry::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];

    // Vehicle equations
    // TODO : write omni robot kinematic
    // dstate[0] = V*std::cos(theta);    // dx
    // dstate[1] = V*std::sin(theta);    // dy
    // dstate[2] = V*std::tan(phi)/L;    // dtheta
}