#include <queue>
#include <boost/numeric/odeint.hpp>

#define WHEELS 4

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;


class omni_odometry {
    public:
        omni_odometry(double deltaT);

        void setInitialState(double x0, double y0, double theta0);
        void setOmniParams(double r, double l, double w, double T);

        void integrate();
        
        void setCommands(double* u_wheels);
        
        void getPose(double &x, double &y, double &theta) {
            x = state[0];
            y = state[1];
            theta = state[2]; 
        };

        void getCommands(double* u_wheels);

        void getTime(double &time) {
            time = t;
        };

    private:
        double t, dt;  // Simulator and integrator variables
        double u[WHEELS];  // Actuation commands
        double r;  // Wheel radius
        double l;  // Wheel position along x
        double w;  // Wheel position along y
        double T;  // Gear ratio  

        bool omni_params_set;

        state_type state;
        runge_kutta_dopri5 < state_type > stepper;  // TODO : to be changed

        void vehicle_ode(const state_type &state, state_type &dstate, double t);

        void RungeKuttaOdometry(state_type &state, state_type &dstate);

        void ExactOdometry(state_type &state, state_type &dstate);
};
