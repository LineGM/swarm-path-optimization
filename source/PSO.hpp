#pragma once

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@file
	@brief This file contains the main declarations of the PSOLibrary
*/

#include "Coord.hpp"
#include <vector>

namespace PSOLibrary
{
/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn double *generate_random(int swarm_size, double lower_bound, double upper_bound, int i)
	The function of generating random numbers within the specified limits
	@param swarm_size
	@param lower_bound
	@param upper_bound
	@param i
	@return random number between lower and upper bounds
*/
double *generate_random(int swarm_size, double lower_bound, double upper_bound, int i);

// Implementing Fitness function update from "Multi-Objective PSO-based Algorithm for Robot Path Planning"
namespace FitnessFunctions
{
/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn double shortfit(Coord input, Coord goal)
	Find shortest path
	@param input
	@param goal
	@return shortest path between input and goal points
*/
double shortfit(Coord input, Coord goal);

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn double smoothfit(Coord input, Coord goal, Coord gbest)
	Find smoothest path
	@param input
	@param goal
	@param gbest
	@return smoothest path between input and goal points
*/
double smoothfit(Coord input, Coord goal, Coord gbest);

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn double fitness(Coord input, Coord gbest)
	Overall fitness function
	@param input
	@param gbest
	@return calculated value of the fitness function
*/
double fitness(Coord input, Coord gbest);

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn double obs_distance(Coord input1, Coord input0, double center_x, double center_y, double radius)
	Shortest distance from obstacle
	@param input1
	@param input0
	@param center_x
	@param center_y
	@param radius
	@return shortest distance from obstacle
*/
double obs_distance(Coord input1, Coord input0, double center_x, double center_y, double radius);

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn Coord new_particle(Coord input1, Coord input0, double center_x, double center_y, double radius, int seed)
	Creating a particle near the boundary of an obstacle
	@param input1
	@param input0
	@param center_x
	@param center_y
	@param radius
	@param seed
	@return coords of new particle
*/
Coord new_particle(Coord input1, Coord input0, double center_x, double center_y, double radius, int seed);
} // namespace FitnessFunctions

namespace Obstacles
{
// Implementing obstacles
/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn bool inCircle(double x, double y, double center_x, double center_y, double r)
	Is the point inside the circular obstacle zone
	@param x
	@param y
	@param center_x
	@param center_y
	@param r
	@return is the point inside the obstacle zone
*/
bool inCircle(double x, double y, double center_x, double center_y, double r);

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn bool obstacle_avoidance(int num_obstacle, Coord &input1, Coord input0, std::vector<double> center_x, std::vector<double> center_y, double r, double thresh, int seed)
	Obstacle avoidance module
	@param num_obstacle
	@param input1
	@param input0
	@param center_x
	@param center_y
	@param r
	@param thresh
	@param seed
	@return is all particles are free of obstacles
*/
// bool obstacle_avoidance(int num_obstacle, Coord &input1, Coord input0, double center_x[], double center_y[], double r, double thresh, int seed, double lower_boundary, double upper_boundary)
bool obstacle_avoidance(int num_obstacle, Coord &input1, Coord input0, std::vector<double> center_x, std::vector<double> center_y, double r, double thresh, int seed);
} // namespace Obstacles

namespace Algorithm
{
/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn void epochDisplay(int iter_ctr, double global_best_fit, Coord global_best_pos)
	Displaying intermediate results at each iteration
	@param iter_ctr
	@param global_best_fit
	@param global_best_pos
*/
void epochDisplay(int iter_ctr, double global_best_fit, Coord global_best_pos);

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn void particleSwarmOptimization()
	The main function implementing the particle swarm algorithm
*/
void particleSwarmOptimization();
} // namespace Algorithm
} // namespace PSOLibrary
