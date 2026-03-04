//LIBRARIES
#include <iostream>
#include <cmath>
#include <math.h>

// VARIABLES

// Defining time interval dt
double dt = 1;

//Defining pressure at time 't', 180 mmHg at default
double P = 180;

//Defining the time constant tau
double tau = 1;

// Healthy Systolic and Diastolic BP (mmHg)
int SBP_healthy = 110;
int DBP_healthy = 65;

// Hypertension Systolic and Diastolic BP (mmHg)
int SBP_hyper = 135;
int DBP_hyper = 80;

// Hypotension Systolic and Diastolic BP (mmHg)
int SBP_hypo = 80;
int DBP_hypo = 60;

// Heartbeat Event Ticker (false by default)
bool heartbeat_event = false;

// Condition Selection Tickers
// healthy is true by default
bool isHealthy = true;
bool isHyper = false;
bool isHypo = false;


// FUNCTIONS

// Exponential Decay Function, returns P(dt)
double decay(double dt, double tau) {
    return P = P * exp((-1 * dt) / tau);
}


// Gaussian Scaling Function, returns scaling factor
double gaussian(double P, int mean_pressure) {

    //Assume a standard deviation of 5
    int sigma = 5.0;

    //exponential term
    double term1 = ((P - mean_pressure)*(P - mean_pressure)) / (2*sigma*sigma);
    //standardizing term
    double term2 = 1 / (sigma * sqrt(2 * 3.14));

    //Returning scaling factor
    double scaling_factor = (term2) * exp(-1 * (term1));
    return scaling_factor;
}



// Main Loop

int main() {

    //declaring global SBP, DBP
    int global_SBP;
    int global_DBP;

    //Deciding which condition is selected
    if (isHealthy = true) {
        int global_SBP = SBP_healthy;
        int global_DBP = DBP_healthy;
    } else if (isHyper = true) {
        int global_SBP = SBP_hyper;
        int global_DBP = DBP_hyper;
    } else if (isHypo = true) {
        int global_SBP = SBP_hypo;
        int global_DBP = DBP_hypo;
    }


    //While the simulation is running, just set upper bound to some arbitrary time T
    int T = 1000;
    while (dt < T) {
        
        //calculate current cuff pressure, initial pressure of 180 mmHg
        double P_current = decay(dt, tau);
        
        // check to see if current P is within effective measurement range
        if (P_current > global_DBP && P_current < global_SBP && heartbeat_event) {
            
            //If it is, scale peak
            double scaling_factor = gaussian(P_current, global_SBP);
            double peak = P_current * scaling_factor;

            //Add peak to current pressure
            P_current += peak;
        }

        //advance time by a tick
        dt++;

        //Output current time
        std::cout << P_current << std::endl;
    }
}
