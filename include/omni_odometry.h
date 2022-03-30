#include <queue>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

// Simulation of a bicycle kiematic model, rear-wheel drive
// Control inputs: longitudinal velocity, steer

class omni_odometry {
    public:
            bicycle_ode(double deltaT);

            void setInitialState(double x0, double y0, double theta0);
            void setVehicleParams(double L);

            void integrate();
            
            void setReferenceCommands(double velocity, double steer);
            
            void getPose(double &x, double &y, double &theta) { x = state[0]; y = state[1]; theta = state[2]; };
            void getCommands(double &velocity, double &steer) { velocity = V; steer = phi; };
            void getTime(double &time) { time = t; };

        private:
            // Simulator and integrator variables
            double t, dt;
            double V, phi;
            double L;

            bool vehicleParams_set;

            state_type state;
            runge_kutta_dopri5 < state_type > stepper;

            // ODE function
            void vehicle_ode(const state_type &state, state_type &dstate, double t);
};
