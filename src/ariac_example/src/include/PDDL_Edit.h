/*
 *@file include/PDDL_Edit.h
 *@brief header file for PDDL_Edit.cpp
 *@author Shaotu Jia
 *@date March 25, 2018
 */

#ifndef INCLUDE_PDDL_EDIT_H_
#define INCLUDE_PDDL_EDIT_H_

#include <vector>
#include <string>
#include "part_structure.h"



class update_PDDL {
private:

	// the parenthesis for start and end in each section
	std::string starter = "(";
	std::string ender = ")\n";

	// the space used in different level section
	std::string level_0_space = "";
	std::string level_1_space = "  ";
	std::string level_2_space = "    ";


public:

	std::string define_set(const std::string& project_name);	///> define problem name
	std::string domain_set(const std::string& project_name);	///> set up domain
	std::string object_set(const std::vector<std::string>& part_types);	///> set object
	std::string init_set(const std::vector<Part_Structure>& bin_part_list, \
			const std::vector<Part_Structure>& tray_part_list); ///> set init
	std::string goal_set(const std::vector<Part_Structure>& goal_part_list); ///> set goal

	std::string PDDL_Problem_Text(const std::string& project_name, \
			const std::vector<std::string>& part_types,\
			const std::vector<Part_Structure>& init_bin_part_list, \
			const std::vector<Part_Structure>& init_tray_part_list, \
			const std::vector<Part_Structure>& goal_part_list);

	void write_to_PDDL(const std::string& project_name, \
			const std::string& file_name, \
			const std::vector<std::string> part_types, \
			const std::vector<Part_Structure>& init_bin_part_list, \
			const std::vector<Part_Structure>& init_tray_part_list, \
			const std::vector<Part_Structure>& goal_part_list);
	
};



#endif /* INCLUDE_PDDL_EDIT_H_ */
