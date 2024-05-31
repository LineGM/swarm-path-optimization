#pragma once

/*!
	@author Gleb Gudkov
	@date 15.02.2024
	@file
	@brief This file contains a declaration of the Params structure
*/

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@struct Params
	@brief This structure defines the core parameters used to run the PSO algorithm
*/
struct Params
{
	//! Number of particles in Swarm
	int SWARM_SIZE = 100;
	//! Number of Iterations
	int NO_OF_ITERS = 500;
	//! Min Inertial Weight
	double WMIN = 0.2;
	//! Max Inertial Weight
	double WMAX = 0.7;
	//! Acceleration coefficient
	double C1 = 0.20;
	//! Acceleration coefficient
	double C2 = 0.60;

	//! Start X-Coordinate
	double START_X = 1000.0;
	//! Start Y-Coordinate
	double START_Y = 1000.0;
	//! Destination X-Coordinate
	double DESTINATION_X = -500.0;
	//! Destination Y-Coordinate
	double DESTINATION_Y = -1000.0;

	//! Search Space Lower bound
	double LOWER_BOUNDARY = -1500.0;
	//! Search Space Upper bound
	double UPPER_BOUNDARY = 1500.0;

	//! Max Particle Velocity
	double V_MAX = (UPPER_BOUNDARY - LOWER_BOUNDARY) / 10;
	//! Position Multiplier
	double POS_MULTIPLE = (UPPER_BOUNDARY - LOWER_BOUNDARY) / 2;
	//! Velocity Multiplier
	double VEL_MULTIPLE = (UPPER_BOUNDARY - LOWER_BOUNDARY) / 20;

	//! Number of obstacles
	int NUM_OBSTACLE = 10;
	//! Obstacles radius
	double R = 200.0;

	//! Tolerance for convergence
	double TARGET_TOLERANCE = 150.0;
	//! Tolerance for local minima convergence
	double LOCAL_CONV_TOLERANCE = (NO_OF_ITERS / 20);
};