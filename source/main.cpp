/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@file
	@brief This file contains the main entry point of the program
*/

#include "PSO.hpp"
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
	PSOLibrary::Algorithm::particleSwarmOptimization();
	std::cout << "Optimization Completed in " << static_cast<float>(startTime);
}