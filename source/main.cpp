/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@file
	@brief This file contains the main entry point of the program
*/

#include "PSO.hpp"
#include "Params.hpp"
#include <cstdlib>
#include <ctime>
#include <iostream>

// Pseudo code
/*
	Start
	Initialize swarm with Random Position (x0) and velocity vectors (v0)
	for Each Particle
	Evaluate Fitness
	If fitness(xt) > fitness(gbest)
	gbest=xt
	If fitness(xt) > fitness(pbest)
	pbest=xt
	Update Velocity
	v(t+1)=W*vt + c1*rand(0,1)*(pbest-xt)+c2*rand(0,1)*gbest-xt)
	Update Position
	X(t+1) = Xt+V(t+1)

	Go to next particle

	If Terminate
	gbest is the output
	Else goto for Each Particle
*/

int main()
{
	// Initial and Control Parameters
	std::srand(static_cast<unsigned int>(std::time(0)));
	clock_t startTime = clock();
	std::cout << "Initiating Optimization ..." << std::endl;

	Params OptimizationParams = Params();

	Coord target = Coord();
	target.x = OptimizationParams.DESTINATION_X;
	target.y = OptimizationParams.DESTINATION_Y;
	Coord start = Coord();
	start.x = OptimizationParams.START_X;
	start.y = OptimizationParams.START_Y;

	std::vector<double> center_x;
	std::vector<double> center_y;
	int seed = 123;
	PSOLibrary::Obstacles::generate_obstacles(center_x, center_y, seed, OptimizationParams);
	// PSOLibrary::Obstacles::readObstaclesMap("/home/linegm/Documents/Obstacle.map", center_x, center_y, OptimizationParams);

	std::cout << "Obstacles: " << std::endl;
	for (auto i = 0; i < OptimizationParams.NUM_OBSTACLE; i++)
	{
		std::cout << i+1 << ": {" << center_x[static_cast<size_t>(i)] << ", " << center_y[static_cast<size_t>(i)] << "}" << std::endl;
	}

	PSOLibrary::Algorithm::particleSwarmOptimization(start, target, center_x, center_y, OptimizationParams);

	std::cout << "Optimization Completed in " << static_cast<float>(startTime);
}