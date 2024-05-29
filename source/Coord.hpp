#pragma once

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@file
	@brief This file contains a declaration of the Coord structure
*/

/*!
	@author Gleb Gudkov
	@date 17.02.2024
	@struct Coord
	@brief This structure describes a particle that has coordinates and velocities along the X and Y axes
*/
struct Coord
{
	//! X-axis coordinate
	double x{0.0};
	//! Y-axis coordinate
	double y{0.0};
	//! X-axis velocity component
	double velx{0.0};
	//! Y-axis velocity component
	double vely{0.0};
};