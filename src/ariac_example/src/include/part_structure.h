/*
 *@file include/part_structure.h
 *@brief define struct Part_Structure to store the information of each part
 *@author Shaotu Jia
 *@date March 25, 2018
 */

#ifndef INCLUDE_PART_STRUCTURE_H_
#define INCLUDE_PART_STRUCTURE_H_

#include <string>

// data structure for part to store part_type and part number
struct Part_Structure {

	std::string part_type = "";		// the type of part e.g gear
	int part_num = 0;				// the quantity of this part; e.g 1

	// initialize struct
	Part_Structure(std::string type, int num) : part_type(type), part_num(num){}
};


#endif /* INCLUDE_PART_STRUCTURE_H_ */
