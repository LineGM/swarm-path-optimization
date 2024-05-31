/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@file
	@brief This file contains the main definitions of the PSOLibrary
*/

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <time.h>

#include "PSO.hpp"
#include "PSOUtils.hpp"
#include "Params.hpp"
#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>

namespace PSOLibrary
{
double *generate_random(int swarm_size, double lower_bound, double upper_bound, int i)
{
	srand(static_cast<unsigned int>(time(NULL)));
	// std::default_random_engine re;
	std::mt19937 re;
	// initialize the random number generator with time-dependent seed
	uint64_t timeSeed = static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count()) + static_cast<uint64_t>(i++);
	std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
	re.seed(ss);
	std::uniform_real_distribution<double> unif(lower_bound, upper_bound);

	double *a_random_array = new double[static_cast<unsigned long>(swarm_size)];
	const int nSimulations = swarm_size;
	for (int j = nSimulations; j--;)
	{
		double a_random_double = unif(re);
		a_random_array[j] = a_random_double;
	}
	return a_random_array;
}

namespace FitnessFunctions
{
double shortfit(Coord input, Coord goal)
{
	double distance;
	distance = sqrt(pow(static_cast<int>(input.x) - static_cast<int>(goal.x), 2.0) + pow(static_cast<int>(input.y) - static_cast<int>(goal.y), 2.0));
	return distance;
}

double smoothfit(Coord input, Coord goal, Coord gbest)
{
	double num = ((input.x - goal.x) * (gbest.x - goal.x)) + ((input.y - goal.y) * (gbest.y - goal.y));
	double den = sqrt(pow((input.x - goal.x), 2.0) + pow((input.y - goal.y), 2.0)) * sqrt(pow((gbest.x - goal.x), 2.0) + pow((gbest.y - goal.y), 2.0));
	double smooth = acos(num / den);
	return smooth;
}

double fitness(Coord input, Coord gbest, Params OptimizationParams)
{
	Coord target; // Goal to reach
	target.x = OptimizationParams.DESTINATION_X;
	target.y = OptimizationParams.DESTINATION_Y;
	double distance = shortfit(input, target);
	double smooth = smoothfit(input, target, gbest);
	double alpha1 = 1, alpha2 = 1;
	double total_fitness = alpha1 * distance + alpha2 * smooth;
	return (1 / total_fitness);
}

double obs_distance(Coord input1, Coord input0, double center_x, double center_y, double radius)
{
	double slope = (input1.y - input0.y) / (input1.x - input0.x);
	double num = fabs((slope * center_x) - center_y + input0.y - (slope * input0.x));
	double den = sqrt(1 + pow(slope, 2.0));
	double obsDist = ((num / den) - radius);
	return obsDist;
}

Coord new_particle(Coord input1, Coord input0, double center_x, double center_y, double radius, int seed)
{
	double thresh = radius * 0.3;
	Coord newPart1, newPart2, newPartFinal;
	double slope = (input1.y - input0.y) / (input1.x - input0.x);
	double invslope = -1 * (1 / slope);
	// double obs = obs_distance(input1, input0, center_x, center_y, radius);
	double *rn = generate_random(1, 0, 1, seed);
	double safedistance = radius + (rn[0] * thresh);
	delete[] (rn);

	double newX1 = (safedistance / sqrt(1 + pow(invslope, 2.0))) + center_x;
	double newY1 = ((safedistance * invslope) / sqrt(1 + pow(invslope, 2.0))) + center_y;
	newPart1.x = newX1;
	newPart1.y = newY1;

	double newX2 = -1 * (safedistance / sqrt(1 + pow(invslope, 2.0))) + center_x;
	double newY2 = -1 * ((safedistance * invslope) / sqrt(1 + pow(invslope, 2.0))) + center_y;
	newPart2.x = newX2;
	newPart2.y = newY2;
	if (shortfit(input0, newPart1) <= shortfit(input0, newPart2))
	{
		newPartFinal = newPart1;
	}
	else
	{
		newPartFinal = newPart2;
	}
	return newPartFinal;
}
} // namespace FitnessFunctions

namespace Obstacles
{
bool inCircle(double x, double y, double center_x, double center_y, double r)
{
	double radius = sqrt(pow(x - center_x, 2.0) + pow(y - center_y, 2.0));
	if (radius <= 1.2 * r)
		return true;
	else
		return false;
}

// bool obstacle_avoidance(int num_obstacle, Coord &input1, Coord input0, double center_x[], double center_y[], double r, double thresh, int seed, double lower_boundary, double upper_boundary)
bool obstacle_avoidance(int num_obstacle, Coord &input1, Coord input0, std::vector<double> center_x, std::vector<double> center_y, double r, double thresh, int seed)
{
	std::vector<bool> valid_init(static_cast<size_t>(num_obstacle));
	std::vector<bool> obs_dist_arr_init(static_cast<size_t>(num_obstacle));
	std::vector<bool> incircle(static_cast<size_t>(num_obstacle));
	bool final_val_init = false;
	for (int k = num_obstacle; k--;)
	{
		double obs_dist = FitnessFunctions::obs_distance(input1, input0, center_x[static_cast<size_t>(k)], center_y[static_cast<size_t>(k)], r);
		if ((obs_dist <= thresh))
		{
			// cout << "Start Path crossing obstacle, Infeasible.Create particle near boundary of obstacle" << endl;
			Coord newPart = FitnessFunctions::new_particle(input1, input0, center_x[static_cast<size_t>(k)], center_y[static_cast<size_t>(k)], r, seed++);
			if (std::isnan(newPart.x) == 1 || std::isnan(newPart.y) == 1)
			{
				input1 = input0;
			}
			else
			{
				input1.x = newPart.x;
				input1.y = newPart.y;
			}
		}

		incircle[static_cast<size_t>(k)] = inCircle(input1.x, input1.y, center_x[static_cast<size_t>(k)], center_y[static_cast<size_t>(k)], r);
		if (incircle[static_cast<size_t>(k)] == true)
			valid_init[static_cast<size_t>(k)] = true;
		else
			valid_init[static_cast<size_t>(k)] = false;
	}
	// Check if all generated particles path are free of obstacles
	for (int k = num_obstacle; k--;)
	{
		double obs_dist = FitnessFunctions::obs_distance(input1, input0, center_x[static_cast<size_t>(k)], center_y[static_cast<size_t>(k)], r);
		if (obs_dist <= thresh)
			obs_dist_arr_init[static_cast<size_t>(k)] = true;
		else
			obs_dist_arr_init[static_cast<size_t>(k)] = false;
	}
	// Check if all particles are free of obstacles
	if (all_of(valid_init.begin(), valid_init.end(), [](bool io) { return !io; }) && all_of(obs_dist_arr_init.begin(), obs_dist_arr_init.end(), [](bool io) { return !io; }))
	{
		final_val_init = false;
		// cout << "Particles free. \n ";
	}
	else
	{
		final_val_init = true;
		// cout << "Initial particles in obstacle, re-initializing \n ";
	}

	return final_val_init;
}

void generate_obstacles(std::vector<double> &center_x, std::vector<double> &center_y, int seed, Params OptimizationParams)
{
	for (int i = OptimizationParams.NUM_OBSTACLE; i--;)
	{
	init:
		double *rnd_x = generate_random(1, -1, 1, seed++);
		double *rnd_y = generate_random(1, -1, 1, seed++);
		int c_x = static_cast<int>((OptimizationParams.POS_MULTIPLE - 200) * (rnd_x[0]));
		int c_y = static_cast<int>((OptimizationParams.POS_MULTIPLE - 200) * (rnd_y[0]));
		bool stinobs = Obstacles::inCircle(OptimizationParams.START_X, OptimizationParams.START_Y, c_x, c_y, OptimizationParams.R);
		bool tarinobs = Obstacles::inCircle(OptimizationParams.DESTINATION_X, OptimizationParams.DESTINATION_Y, c_x, c_y, OptimizationParams.R);
		delete[] (rnd_x);
		delete[] (rnd_y);
		if (!stinobs && !tarinobs)
		{
			std::cout << c_x << ", " << c_y << std::endl;
			center_x.push_back(c_x);
			center_y.push_back(c_y);
		}
		else
		{
			std::cout << "start or target in obstacle..... Replanning" << std::endl;
			goto init;
		}
	}
}

void readObstaclesMap(std::filesystem::path pathToMap, std::vector<double> &center_x, std::vector<double> &center_y, Params OptimizationParams)
{
	std::ifstream mapfile(pathToMap);
	if (mapfile.is_open())
	{
		int obs_number = 0;
		mapfile >> obs_number;
		int obs_radius = 0;
		mapfile >> obs_radius;

		OptimizationParams.NUM_OBSTACLE = obs_number;
		OptimizationParams.R = obs_radius;

		double x, y;
		while (mapfile >> x >> y)
		{
			center_x.push_back(x);
			center_y.push_back(y);
		}
	}
	mapfile.close();
}
} // namespace Obstacles

namespace Algorithm
{
void epochDisplay(int iter_ctr, double global_best_fit, Coord global_best_pos)
{
	std::cout << "Iteration :" << iter_ctr << ".........." << std::endl;
	std::cout << "New Global Best fitness: " << global_best_fit * pow(10, 3) << std::endl;
	std::cout << "New Global Best Position: (" << global_best_pos.x << "," << global_best_pos.y << ")" << std::endl;
}

void readParams(std::filesystem::path pathToParamsFile, Params &OptimizationParams)
{
	std::ifstream paramfile(pathToParamsFile);
	if (paramfile.is_open())
	{
		std::string name;
		double value;

		paramfile >> name >> value;
		OptimizationParams.SWARM_SIZE = static_cast<int>(value);

		paramfile >> name >> value;
		OptimizationParams.NO_OF_ITERS = static_cast<int>(value);

		paramfile >> name >> value;
		OptimizationParams.WMIN = value;

		paramfile >> name >> value;
		OptimizationParams.WMAX = value;

		paramfile >> name >> value;
		OptimizationParams.C1 = value;

		paramfile >> name >> value;
		OptimizationParams.C2 = value;

		paramfile >> name >> value;
		OptimizationParams.START_X = value;

		paramfile >> name >> value;
		OptimizationParams.START_Y = value;

		paramfile >> name >> value;
		OptimizationParams.DESTINATION_X = value;

		paramfile >> name >> value;
		OptimizationParams.DESTINATION_Y = value;

		paramfile >> name >> value;
		OptimizationParams.LOWER_BOUNDARY = value;

		paramfile >> name >> value;
		OptimizationParams.UPPER_BOUNDARY = value;

		paramfile >> name >> value;
		OptimizationParams.NUM_OBSTACLE = static_cast<int>(value);

		paramfile >> name >> value;
		OptimizationParams.R = value;

		paramfile >> name >> value;
		OptimizationParams.TARGET_TOLERANCE = value;

		OptimizationParams.V_MAX = (OptimizationParams.UPPER_BOUNDARY - OptimizationParams.LOWER_BOUNDARY) / 10;
		OptimizationParams.POS_MULTIPLE = (OptimizationParams.UPPER_BOUNDARY - OptimizationParams.LOWER_BOUNDARY) / 2;
		OptimizationParams.VEL_MULTIPLE = (OptimizationParams.UPPER_BOUNDARY - OptimizationParams.LOWER_BOUNDARY) / 20;
		OptimizationParams.LOCAL_CONV_TOLERANCE = (OptimizationParams.NO_OF_ITERS / 20);
	}
	paramfile.close();
}

void particleSwarmOptimization(const Coord &start, const Coord &target, std::vector<double> &center_x, std::vector<double> &center_y, Params OptimizationParams)
{
	// Variables Initialization
	//  Random Numbers used for updating position and velocity of particles
	//  Positions // Velocities

	int seed = 1; // Variable to induce different timeseeds in each call for different random number generation
	double lower_bound = -1;
	double upper_bound = 1;

	double *position_x = generate_random(OptimizationParams.SWARM_SIZE, lower_bound, upper_bound, seed++);
	double *position_y = generate_random(OptimizationParams.SWARM_SIZE, lower_bound, upper_bound, seed++);
	double *velocity_x = generate_random(OptimizationParams.SWARM_SIZE, lower_bound, upper_bound, seed++);
	double *velocity_y = generate_random(OptimizationParams.SWARM_SIZE, lower_bound, upper_bound, seed++);

	// Structure to store random data for position and velocities
	std::vector<Coord> States(static_cast<size_t>(OptimizationParams.SWARM_SIZE));
	for (int i = OptimizationParams.SWARM_SIZE; i--;)
	{
		States[static_cast<size_t>(i)].x = position_x[i];
		States[static_cast<size_t>(i)].y = position_y[i];
		States[static_cast<size_t>(i)].velx = velocity_x[i];
		States[static_cast<size_t>(i)].vely = velocity_y[i];
	}

	// New Initializations
	std::vector<Coord> particle_states(static_cast<size_t>(OptimizationParams.SWARM_SIZE));	  // position and velocities of particles
	std::vector<double> particle_fitness(static_cast<size_t>(OptimizationParams.SWARM_SIZE)); // fitness of particles
	std::vector<Coord> local_best_pos(static_cast<size_t>(OptimizationParams.SWARM_SIZE));	  // local best position of each particle across iterations
	std::vector<double> local_best_fit(static_cast<size_t>(OptimizationParams.SWARM_SIZE));	  // local best fitness of each particle across iterations
	Coord global_best_pos;																	  // global best position across all particles across iterations
	double global_best_fit;

	// Initialize all arrays to 0
	global_best_pos.x = start.x;
	global_best_pos.y = start.y;

	int cols = OptimizationParams.SWARM_SIZE + 1;
	int rows = OptimizationParams.NO_OF_ITERS + 1;
	int initial_value = 0;

	int counter = 1;
	std::vector<Coord> global_pos(static_cast<size_t>(rows));
	global_pos[0].x = global_best_pos.x;
	global_pos[0].y = global_best_pos.y;

	std::vector<std::vector<double>> particle_pos_x;
	particle_pos_x.resize(static_cast<size_t>(rows), std::vector<double>(static_cast<size_t>(cols), initial_value));
	std::vector<std::vector<double>> particle_pos_y;
	particle_pos_y.resize(static_cast<size_t>(rows), std::vector<double>(static_cast<size_t>(cols), initial_value));

	std::vector<Coord> path_coord;

	std::vector<std::vector<double>> local_pos_x;
	local_pos_x.resize(static_cast<size_t>(rows), std::vector<double>(static_cast<size_t>(cols), initial_value));
	std::vector<std::vector<double>> local_pos_y;
	local_pos_y.resize(static_cast<size_t>(rows), std::vector<double>(static_cast<size_t>(cols), initial_value));

	global_best_fit = FitnessFunctions::fitness(global_best_pos, global_pos[0], OptimizationParams);

	// Initialize obstacle parameters (Obstacles at random positions)
	// ==================================================================================//
	// bool incircle[NUM_OBSTACLE];
	// double center_x[num_obstacle] = {-600, -300, 550, 600, 250};
	// double center_y[num_obstacle] = {-100, 500, 500, -200, -1000};
	// ==================================================================================//

	// First Iteration. Initialize swarm with Random Position and Velocity vectors for each particle.
	double thresh = (OptimizationParams.R * 0.1);
	for (int i = OptimizationParams.SWARM_SIZE; i--;)
	{
		bool final_val_init = false;
		int ct = 500;
		do
		{
			particle_states[static_cast<size_t>(i)].x = OptimizationParams.POS_MULTIPLE * (States[static_cast<size_t>(i)].x);
			particle_states[static_cast<size_t>(i)].y = OptimizationParams.POS_MULTIPLE * (States[static_cast<size_t>(i)].y);
			double *new_x = generate_random(1, 0, 1, seed++);
			States[static_cast<size_t>(i)].x = new_x[0];
			double *new_y = generate_random(1, 0, 1, seed++);
			States[static_cast<size_t>(i)].y = new_y[0];
			delete[] (new_x);
			delete[] (new_y);
			if (particle_states[static_cast<size_t>(i)].x > OptimizationParams.UPPER_BOUNDARY)
				particle_states[static_cast<size_t>(i)].x = OptimizationParams.UPPER_BOUNDARY;
			if (particle_states[static_cast<size_t>(i)].y > OptimizationParams.UPPER_BOUNDARY)
				particle_states[static_cast<size_t>(i)].y = OptimizationParams.UPPER_BOUNDARY;
			if (particle_states[static_cast<size_t>(i)].x < OptimizationParams.LOWER_BOUNDARY)
				particle_states[static_cast<size_t>(i)].x = OptimizationParams.LOWER_BOUNDARY;
			if (particle_states[static_cast<size_t>(i)].y < OptimizationParams.LOWER_BOUNDARY)
				particle_states[static_cast<size_t>(i)].y = OptimizationParams.LOWER_BOUNDARY;

			// OBSTACLE AVOIDANCE MODULE
			//================================================================================================================================================//
			// final_val_init = obstacle_avoidance(NUM_OBSTACLE, particle_states[i], start, center_x, center_y, R, thresh, seed, LOWER_BOUNDARY, UPPER_BOUNDARY);
			final_val_init = Obstacles::obstacle_avoidance(OptimizationParams.NUM_OBSTACLE, particle_states[static_cast<size_t>(i)], start, center_x, center_y, OptimizationParams.R, thresh, seed);
			//================================================================================================================================================//
			ct--;

		} while ((final_val_init) && (ct != 0));

		particle_states[static_cast<size_t>(i)].velx = OptimizationParams.VEL_MULTIPLE * (States[static_cast<size_t>(i)].x);
		particle_states[static_cast<size_t>(i)].vely = OptimizationParams.VEL_MULTIPLE * (States[static_cast<size_t>(i)].y);

		// Representing fitness function as the distance between the particles and local best
		particle_fitness[static_cast<size_t>(i)] = FitnessFunctions::fitness(particle_states[static_cast<size_t>(i)], global_pos[0], OptimizationParams);
		local_best_pos[static_cast<size_t>(i)].x = particle_states[static_cast<size_t>(i)].x;
		local_best_pos[static_cast<size_t>(i)].y = particle_states[static_cast<size_t>(i)].y;
		local_best_fit[static_cast<size_t>(i)] = particle_fitness[static_cast<size_t>(i)];

		if (local_best_fit[static_cast<size_t>(i)] > global_best_fit)
		{
			global_best_pos.x = particle_states[static_cast<size_t>(i)].x;
			global_best_pos.y = particle_states[static_cast<size_t>(i)].y;
			global_best_fit = local_best_fit[static_cast<size_t>(i)];
		}

		// Write data to csv
		local_pos_x[1][static_cast<size_t>(i)] = local_best_pos[static_cast<size_t>(i)].x;
		local_pos_y[1][static_cast<size_t>(i)] = local_best_pos[static_cast<size_t>(i)].y;
		global_pos[1].x = global_best_pos.x;
		global_pos[1].y = global_best_pos.y;
		particle_pos_x[1][static_cast<size_t>(i)] = particle_states[static_cast<size_t>(i)].x;
		particle_pos_y[1][static_cast<size_t>(i)] = particle_states[static_cast<size_t>(i)].y;
	}

	// Initial Display
	epochDisplay(1, global_best_fit, global_best_pos);

	// Calculations for the second and subsequent iterations
	// int flag = 1, cnt = 0;
	for (int iter_ctr = 2; iter_ctr <= OptimizationParams.NO_OF_ITERS; iter_ctr++)
	{

		// Updating Velocity - applying Psycho-Social Criteria
		for (int i = OptimizationParams.SWARM_SIZE; i--;)
		{
			bool final_val = false;
			int ct = 500;
			do
			{
				double *rp = generate_random(1, 0, 1, seed++);
				double *rg = generate_random(1, 0, 1, seed++);
				// w = wmax - (((wmax - wmin) / (no_of_iters)) * iter_ctr);
				// following update from "Intelligent Vehicle Global Path Planning Based on Improved Particle Swarm Optimization"
				double w, c = 2.3; // Inertial Weight, constant
				w = OptimizationParams.WMIN + (OptimizationParams.WMAX - OptimizationParams.WMIN) * exp(-pow(((c * iter_ctr) / OptimizationParams.NO_OF_ITERS), 2.0));
				// cout << "w value : " << w << endl;
				particle_states[static_cast<size_t>(i)].velx = (w * particle_states[static_cast<size_t>(i)].velx + OptimizationParams.C1 * rp[0] * (local_best_pos[static_cast<size_t>(i)].x - particle_states[static_cast<size_t>(i)].x)) +
															   OptimizationParams.C2 * rg[0] * (global_best_pos.x - particle_states[static_cast<size_t>(i)].x);
				particle_states[static_cast<size_t>(i)].vely = (w * particle_states[static_cast<size_t>(i)].vely + OptimizationParams.C1 * rp[0] * (local_best_pos[static_cast<size_t>(i)].y - particle_states[static_cast<size_t>(i)].y)) +
															   OptimizationParams.C2 * rg[0] * (global_best_pos.y - particle_states[static_cast<size_t>(i)].y);
				delete[] (rp);
				delete[] (rg);
				if (particle_states[static_cast<size_t>(i)].velx > OptimizationParams.V_MAX)
					particle_states[static_cast<size_t>(i)].velx = OptimizationParams.V_MAX;
				if (particle_states[static_cast<size_t>(i)].vely > OptimizationParams.V_MAX)
					particle_states[static_cast<size_t>(i)].vely = OptimizationParams.V_MAX;
				if (particle_states[static_cast<size_t>(i)].velx < -OptimizationParams.V_MAX)
					particle_states[static_cast<size_t>(i)].velx = -OptimizationParams.V_MAX;
				if (particle_states[static_cast<size_t>(i)].vely < -OptimizationParams.V_MAX)
					particle_states[static_cast<size_t>(i)].vely = -OptimizationParams.V_MAX;

				particle_states[static_cast<size_t>(i)].x = particle_states[static_cast<size_t>(i)].x + particle_states[static_cast<size_t>(i)].velx;
				particle_states[static_cast<size_t>(i)].y = particle_states[static_cast<size_t>(i)].y + particle_states[static_cast<size_t>(i)].vely;
				// if (particle_states[i].x > UPPER_BOUNDARY)
				//     particle_states[i].x = UPPER_BOUNDARY;
				// if (particle_states[i].y > UPPER_BOUNDARY)
				//     particle_states[i].y = UPPER_BOUNDARY;
				// if (particle_states[i].x < LOWER_BOUNDARY)
				//     particle_states[i].x = LOWER_BOUNDARY;
				// if (particle_states[i].y < LOWER_BOUNDARY)
				//     particle_states[i].y = LOWER_BOUNDARY;

				// OBSTACLE AVOIDANCE MODULE
				// ====================================================================================================================================================//
				// final_val = obstacle_avoidance(NUM_OBSTACLE, particle_states[i], global_best_pos, center_x, center_y, R, thresh, seed, LOWER_BOUNDARY, UPPER_BOUNDARY);
				final_val = Obstacles::obstacle_avoidance(OptimizationParams.NUM_OBSTACLE, particle_states[static_cast<size_t>(i)], global_best_pos, center_x, center_y, OptimizationParams.R, thresh, seed);
				//=====================================================================================================================================================//
				ct--;

			} while ((final_val) && (ct != 0));

			// Write data to csv
			particle_pos_x[static_cast<size_t>(iter_ctr)][static_cast<size_t>(i)] = particle_states[static_cast<size_t>(i)].x;
			particle_pos_y[static_cast<size_t>(iter_ctr)][static_cast<size_t>(i)] = particle_states[static_cast<size_t>(i)].y;
		}

		// Assign Local & Global Best position & Fitness for each particle evaluate fitness
		// If fitness(xt) > fitness(gbest) then gbest=xt
		// If fitness(xt) > fitness(pbest) then pbest=xt
		for (int i = OptimizationParams.SWARM_SIZE; i--;)
		{
			// Set Particle Fitness
			particle_fitness[static_cast<size_t>(i)] = FitnessFunctions::fitness(particle_states[static_cast<size_t>(i)], global_pos[static_cast<size_t>(iter_ctr - 1)], OptimizationParams);

			// Updating local/particle best position
			if (particle_fitness[static_cast<size_t>(i)] > local_best_fit[static_cast<size_t>(i)])
			{
				local_best_pos[static_cast<size_t>(i)].x = particle_states[static_cast<size_t>(i)].x;
				local_best_pos[static_cast<size_t>(i)].y = particle_states[static_cast<size_t>(i)].y;
				local_best_fit[static_cast<size_t>(i)] = FitnessFunctions::fitness(local_best_pos[static_cast<size_t>(i)], global_pos[static_cast<size_t>(iter_ctr - 1)], OptimizationParams);
				// Updating global best position
				if (local_best_fit[static_cast<size_t>(i)] > global_best_fit)
				{
					global_best_pos.x = local_best_pos[static_cast<size_t>(i)].x;
					global_best_pos.y = local_best_pos[static_cast<size_t>(i)].y;
					global_best_fit = FitnessFunctions::fitness(global_best_pos, global_pos[static_cast<size_t>(iter_ctr - 1)], OptimizationParams);

					global_best_pos.velx = particle_states[static_cast<size_t>(i)].velx;
					global_best_pos.vely = particle_states[static_cast<size_t>(i)].vely;
				}
			}
			// Write data to csv
			local_pos_x[static_cast<size_t>(iter_ctr)][static_cast<size_t>(i)] = local_best_pos[static_cast<size_t>(i)].x;
			local_pos_y[static_cast<size_t>(iter_ctr)][static_cast<size_t>(i)] = local_best_pos[static_cast<size_t>(i)].y;
			global_pos[static_cast<size_t>(iter_ctr)].x = global_best_pos.x;
			global_pos[static_cast<size_t>(iter_ctr)].y = global_best_pos.y;
		}

		if (iter_ctr > 3)
		{
			if (((global_pos[static_cast<size_t>(iter_ctr)].x - global_pos[static_cast<size_t>(iter_ctr - 1)].x) < 0.2) && ((global_pos[static_cast<size_t>(iter_ctr)].x - global_pos[static_cast<size_t>(iter_ctr - 2)].x) < 0.2) &&
				((global_pos[static_cast<size_t>(iter_ctr)].x - global_pos[static_cast<size_t>(iter_ctr - 3)].x) < 0.2) && ((global_pos[static_cast<size_t>(iter_ctr)].y - global_pos[static_cast<size_t>(iter_ctr - 1)].y) < 0.2) &&
				((global_pos[static_cast<size_t>(iter_ctr)].y - global_pos[static_cast<size_t>(iter_ctr - 2)].y) < 0.2) && ((global_pos[static_cast<size_t>(iter_ctr)].y - global_pos[static_cast<size_t>(iter_ctr - 3)].y) < 0.2))
			{
				std::cout << "Local Minima detected: " << std::endl;
				// Implementing Local Minima avoidance from "Shortest Path Planning PSO"
				// Applies a slight perturbation to the Global best particles so that it doesn't get stuck in a local minima.
				// The particles are updated by the given velocity formula
				// Check if particles are within boundary

				bool final_val = false;
				std::vector<bool> valid;
				int cs = 500;

				do
				{
					double alpha = 0.2, beta = 0.3; // Arbitrarily generated constants
					double r3 = generate_random(1, 0, 1, seed++)[0];
					global_best_pos.velx = (alpha * global_best_pos.velx) + (beta * r3);
					global_best_pos.vely = (alpha * global_best_pos.vely) + (beta * r3);
					Coord gbest;
					gbest.x = global_best_pos.x + global_best_pos.velx;
					gbest.y = global_best_pos.y + global_best_pos.vely;
					double gfit = FitnessFunctions::fitness(gbest, global_pos[static_cast<size_t>(iter_ctr - 1)], OptimizationParams);
					global_best_pos = gbest;
					global_best_fit = gfit;

					// OBSTACLE AVOIDANCE MODULE
					//==================================================================================================================================================//
					// final_val = obstacle_avoidance(NUM_OBSTACLE, global_best_pos, global_best_pos, center_x, center_y, R, thresh, seed, LOWER_BOUNDARY, UPPER_BOUNDARY);
					final_val = Obstacles::obstacle_avoidance(OptimizationParams.NUM_OBSTACLE, global_best_pos, global_best_pos, center_x, center_y, OptimizationParams.R, thresh, seed);
					//==================================================================================================================================================//

					cs--;
				} while (final_val && cs != 0);
			}
		}

		// All Displays
		epochDisplay(iter_ctr, global_best_fit, global_best_pos);

		// Check if converged on goal
		counter++;
		if (FitnessFunctions::shortfit(global_best_pos, target) <= OptimizationParams.TARGET_TOLERANCE)
		{
			std::cout << "\n\nReached near goal. SUCCESS " << std::endl;
			// flag = 0;
			break;
		}
		else if (iter_ctr >= OptimizationParams.LOCAL_CONV_TOLERANCE)
		{
			if (((global_pos[static_cast<size_t>(iter_ctr)].x - global_pos[static_cast<size_t>(iter_ctr - 1)].x) < 0.1) && ((global_pos[static_cast<size_t>(iter_ctr)].x - global_pos[static_cast<size_t>(iter_ctr - 2)].x) < 0.1) &&
				((global_pos[static_cast<size_t>(iter_ctr)].x - global_pos[static_cast<size_t>(iter_ctr - 3)].x) < 0.1) && ((global_pos[static_cast<size_t>(iter_ctr)].y - global_pos[static_cast<size_t>(iter_ctr - 1)].y) < 0.1) &&
				((global_pos[static_cast<size_t>(iter_ctr)].y - global_pos[static_cast<size_t>(iter_ctr - 2)].y) < 0.1) && ((global_pos[static_cast<size_t>(iter_ctr)].y - global_pos[static_cast<size_t>(iter_ctr - 3)].y) < 0.1))
			{
				std::cout << "\n\nCouldn't find a feasible path / Converged at a local minima. FAILED" << std::endl;
				// flag = 1;
				break;
			}
		}
	}

	// Optimizing path generating
	Coord path_cd;
	path_cd.x = start.x;
	path_cd.y = start.y;
	path_coord.insert(path_coord.begin() + static_cast<long>(path_coord.size()), path_cd);
	for (int iter_ctr = 0; iter_ctr < counter; iter_ctr++)
	{
		if (iter_ctr > 3)
		{
			if ((FitnessFunctions::shortfit(global_pos[static_cast<size_t>(iter_ctr)], target) < FitnessFunctions::shortfit(global_pos[static_cast<size_t>(iter_ctr - 1)], target)) &&
				(FitnessFunctions::shortfit(global_pos[static_cast<size_t>(iter_ctr)], target) < FitnessFunctions::shortfit(global_pos[static_cast<size_t>(iter_ctr - 2)], target)) &&
				(FitnessFunctions::shortfit(global_pos[static_cast<size_t>(iter_ctr)], target) < FitnessFunctions::shortfit(global_pos[static_cast<size_t>(iter_ctr - 3)], target)))
			{
				path_coord.insert(path_coord.begin() + static_cast<long>(path_coord.size()), global_pos[static_cast<size_t>(iter_ctr)]);
			}
			else
			{
				path_coord.insert(path_coord.begin() + static_cast<long>(path_coord.size()), global_pos[static_cast<size_t>(iter_ctr - 1)]);
			}
		}
		else
		{
			path_coord.insert(path_coord.begin() + static_cast<long>(path_coord.size()), global_pos[0]);
		}
	}

	std::cout << std::endl << "Final Global Best Position: (" << global_best_pos.x << "," << global_best_pos.y << ")" << std::endl;

	// Writing to csv all the data for plotting
	PSOUtils::write_to_csv(particle_pos_x, particle_pos_y, global_pos, local_pos_x, local_pos_x, OptimizationParams.SWARM_SIZE, counter, center_x, center_y, OptimizationParams.R, OptimizationParams.NUM_OBSTACLE, path_coord);
	std::cout << "Run Successful" << std::endl;

	// Free allocated memory
	delete[] (position_x);
	delete[] (position_y);
	delete[] (velocity_x);
	delete[] (velocity_y);
}
} // namespace Algorithm
} // namespace PSOLibrary