// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <iostream>
#include <fstream>
//using namespace std;
ofstream fout;

void autopilot(void)
// Autopilot to adjust the engine throttle, parachute and attitude control
{
    // INSERT YOUR CODE HERE
    double Kh, Kp, h, e, delta, CURRENT_LANDER_WEIGHT, Pout;

    //lander weight = mass*gravity
    CURRENT_LANDER_WEIGHT = (UNLOADED_LANDER_MASS + (FUEL_CAPACITY * fuel) * FUEL_DENSITY)*(GRAVITY*MARS_MASS/position.abs2());

    //altitude
    h = position.abs() - MARS_RADIUS;

    /*delta determines the thrust value when Pout=0
    When the error is 0; we want the thrust to balance out the weight exactly
    for the descent speed to decrease linearly as planned */
    delta = CURRENT_LANDER_WEIGHT / MAX_THRUST;

    //scenario1 successful (finite fuel)
    /*Kh = 0.02;
    Kp = 1;*/

    Kh = 0.017;
    Kp = 1;
    e = -(0.5 + Kh * h + velocity*position.norm());
    Pout = Kp * e;

    if (Pout <= -delta) {
        throttle = 0;
    }
    else if (Pout <1 - delta && Pout>-delta) {
        throttle = delta + Pout;
    }
    else {
        throttle = 1;
    }


    //cout << "Target descent rate" << Kh * h << endl;
    //cout << "Actual descent rate" << -1.0 * velocity * position.norm() << endl;

    if (simulation_time == 0) { fout.open("results.txt"); }   
    else { fout.open("results.txt", fstream::app); }    
    if (fout) { // file opened successfully       
        fout << h <<' ' << Kh*h << ' ' << -1.0 * velocity * position.norm() << endl;
    }   
    else { // file did not open successfully
        cout << "Could not open trajectory file for writing" << endl;
    }   
    fout.close();

}



void numerical_dynamics(void)
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
    // INSERT YOUR CODE HERE
    static vector3d previous_position;
    vector3d new_position;
    static vector3d GRAVITY_FORCE;
    double CURRENT_LANDER_MASS;
    static vector3d ACCELERATION;
    static vector3d DRAG_FORCE;

    CURRENT_LANDER_MASS = UNLOADED_LANDER_MASS + (FUEL_CAPACITY * fuel) * FUEL_DENSITY;
    GRAVITY_FORCE = (GRAVITY * MARS_MASS * CURRENT_LANDER_MASS * (-position.norm())) / (position.abs2());

    if (parachute_status == DEPLOYED) {
        DRAG_FORCE = (-0.5 * (atmospheric_density(position)) * DRAG_COEF_LANDER * (3.1415 * pow(LANDER_SIZE, 2)) * velocity.abs2() * velocity.norm()) + (-0.5 * (atmospheric_density(position)) * DRAG_COEF_CHUTE * (5 * pow(2 * LANDER_SIZE, 2)) * velocity.abs2() * velocity.norm());
    }
    else if (parachute_status == NOT_DEPLOYED) {
        DRAG_FORCE = -0.5 * (atmospheric_density(position)) * DRAG_COEF_LANDER * (3.1415 * pow(LANDER_SIZE, 2)) * velocity.abs2() * velocity.norm();
    }
    else {

    }

    ACCELERATION = (thrust_wrt_world() + GRAVITY_FORCE + DRAG_FORCE) / CURRENT_LANDER_MASS;

    if (simulation_time == 0.0) {
        new_position = position + delta_t * velocity;
        velocity = velocity + delta_t * ACCELERATION;
        /*do an Euler update for the first iteration
        i.e. new_position = ... (Euler update, using position and velocity)
        velocity = ... (Euler update, using acceleration)*/
    }

    else {
        new_position = 2 * position - previous_position + pow(delta_t, 2) * ACCELERATION;
        velocity = (new_position - position) / delta_t;
        /* do a Verlet update on all subsequent iterations
        i.e. new_position = ... (Verlet update, using position and previous position)
        velocity = .... (Verlet update, using new_position and position)*/
    }

    previous_position = position;
    position = new_position;
    

    // Here we can apply an autopilot to adjust the thrust, parachute and attitude   
    if (autopilot_enabled) autopilot();

    // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards     
    if (stabilized_attitude) attitude_stabilization();
}



void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
