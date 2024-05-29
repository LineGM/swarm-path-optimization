/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@file
	@brief This file contains definitions of auxiliary functions of the PSOLibrary
*/

#include "PSOUtils.hpp"
#include <fstream>
#include <iostream>

namespace PSOUtils
{
void write_to_csv(std::vector<std::vector<double>> particle_pos_x, std::vector<std::vector<double>> particle_pos_y, std::vector<Coord> global_pos, std::vector<std::vector<double>> local_pos_x, std::vector<std::vector<double>> local_pos_y, int swarm_size,
				  int counter, std::vector<double> center_x, std::vector<double> center_y, double r, int num_obstacles, std::vector<Coord> path_coord)
{
	// create an ofstream for the file output
	std::ofstream outputFile;
	std::ofstream fs;
	// create a name for the file output
	std::string filename = "graph_data.csv";
	// create and open the .csv file
	outputFile.open(filename.c_str());

	// write data to the file
	// Columns: particles
	// Rows: iterations
	for (int i = 0; i < swarm_size; i++)
	{
		outputFile << "Particle_x" << i + 1 << ","
				   << "Particle_y" << i + 1 << ",";
	}

	outputFile << "global_position_x"
			   << ","
			   << "global_position_y"
			   << ",";
	outputFile << "path_x"
			   << ","
			   << "path_y"
			   << ",";

	for (int i = 0; i < swarm_size; i++)
	{
		outputFile << "local_position_x" << i + 1 << ","
				   << "local_position_y" << i + 1 << ",";
	}

	outputFile << "num_obstacles"
			   << ",";

	for (int i = 0; i < num_obstacles; i++)
	{
		outputFile << "center_x" << i + 1 << ","
				   << "center_y" << i + 1 << ","
				   << "r"
				   << ",";
	}

	outputFile << "\n";

	for (int iter = 0; iter < counter; iter++)
	{
		for (int i = 0; i < swarm_size; i++)
		{
			outputFile << particle_pos_x[static_cast<size_t>(iter)][static_cast<size_t>(i)] << "," << particle_pos_y[static_cast<size_t>(iter)][static_cast<size_t>(i)] << ",";
		}

		if (iter == counter - 1)
		{
			outputFile << global_pos[static_cast<size_t>(iter + 1)].x << "," << global_pos[static_cast<size_t>(iter + 1)].y << ",";
		}
		else
		{
			outputFile << global_pos[static_cast<size_t>(iter)].x << "," << global_pos[static_cast<size_t>(iter)].y << ",";
		}

		outputFile << path_coord[static_cast<size_t>(iter)].x << "," << path_coord[static_cast<size_t>(iter)].y << ",";

		for (int i = 0; i < swarm_size; i++)
		{
			outputFile << local_pos_x[static_cast<size_t>(iter)][static_cast<size_t>(i)] << "," << local_pos_y[static_cast<size_t>(iter)][static_cast<size_t>(i)] << ",";
		}

		outputFile << num_obstacles << ",";

		for (int i = 0; i < num_obstacles; i++)
		{
			outputFile << center_x[static_cast<size_t>(i)] << "," << center_y[static_cast<size_t>(i)] << "," << r << ",";
		}

		outputFile << "\n";
	}

	// close the output file
	outputFile.close();
	std::cout << "Values written to csv file" << std::endl;
}
} // namespace PSOUtils