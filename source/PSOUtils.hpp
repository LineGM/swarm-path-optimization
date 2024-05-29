#pragma once

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@file
	@brief This file contains declarations of auxiliary functions of the PSOLibrary
*/

#include "Coord.hpp"
#include <vector>

namespace PSOUtils
{
/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@fn void write_to_csv(std::vector<std::vector<double>> particle_pos_x, std::vector<std::vector<double>> particle_pos_y, std::vector<Coord> global_pos, std::vector<std::vector<double>> local_pos_x, std::vector<std::vector<double>> local_pos_y, int
   swarm_size, int counter, std::vector<double> center_x, std::vector<double> center_y, double r, int num_obstacles, std::vector<Coord> path_coord) Write data to csv file
	@param particle_pos_x
	@param particle_pos_y
	@param global_pos
	@param local_pos_x
	@param local_pos_y
	@param swarm_size
	@param counter
	@param center_x
	@param center_y
	@param r
	@param num_obstacles
	@param path_coord
*/
void write_to_csv(std::vector<std::vector<double>> particle_pos_x, std::vector<std::vector<double>> particle_pos_y, std::vector<Coord> global_pos, std::vector<std::vector<double>> local_pos_x, std::vector<std::vector<double>> local_pos_y, int swarm_size,
				  int counter, std::vector<double> center_x, std::vector<double> center_y, double r, int num_obstacles, std::vector<Coord> path_coord);
} // namespace PSOUtils