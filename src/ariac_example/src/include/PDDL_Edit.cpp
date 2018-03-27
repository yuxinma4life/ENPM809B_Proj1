/*
 *@file PDDL_Edit.cpp
 *@brief source file
 *@author Shaotu Jia
 *@date March 25, 2018
 */

#include <fstream>
#include <string>
#include "include/PDDL_Edit.h"
#include "include/part_structure.h"


/**
 * @brief set the text for define
 * @param project_name The name of this project
 * @return text for define in PDDL problem file
 */
std::string update_PDDL::define_set(const std::string& project_name) {
	return "define (problem " + project_name + "problem)" + "\n";
}

/**
 * @brief set the text for domain
 * @param project_name The name of this project
 * @return text for domain in PDDL problem file
 */
std::string update_PDDL::domain_set(const std::string& project_name) {
	return "  (:domain " + project_name + "domain)" + "\n";
}

/**
 * @brief set object section
 * @param part_types The part type that will occur in this project
 * @return text for object section in PDDL
 */
std::string update_PDDL::object_set(const std::vector<std::string>& part_types) {
	std::string initial_text = ":objects\n";
	std::string robot = level_2_space + "robot - robot\n";
	std::string tray = level_2_space + "tray - tray\n";

	std::string part_type_list = "";	///> the parts we have in bins
	std::string bin_type_list = "";		///> the bins we have containing parts
	for (auto& pt : part_types) {

		part_type_list += level_2_space + pt + " - parttype" + "\n";
		bin_type_list += level_2_space + "bin-for-type-" + pt + "-part - bin" + "\n";
	}

	return level_1_space + starter + initial_text + robot + tray + part_type_list \
			+ bin_type_list + level_2_space + ender;

}

/**
 * @brief set init section
 * @param bin_part_list The 'initial' part type and its amount in 'bin'
 * @param tray_part_list The 'initial' part type and its amount in the 'tray'
 * @return text for init section in PDDL
 */
std::string update_PDDL::init_set(const std::vector<Part_Structure>& bin_part_list, \
		const std::vector<Part_Structure>& tray_part_list) {
	std::string initial_text = level_0_space + ":init" + "\n";
	std::string handempty = level_2_space + "(handempty robot)" + "\n";
	std::string over_tray = level_2_space + "(overtray robot tray)" + "\n";

	std::string parts_quantity_in_bin = "";
	std::string parts_quantity_in_tray = "";
	std::string not_over_bin = "";

	for (auto& p : bin_part_list) {

			parts_quantity_in_bin += level_2_space + "(=(current-quantity-partType-in-bin  " \
					+ p.part_type + " bin-for-type-rod-part ) "+ std::to_string(p.part_num) + ")" + "\n";



			not_over_bin += level_2_space + "(not(overbin robot bin-for-type-" \
					+ p.part_type + "-part))" + "\n";
		}

	for (auto& t : tray_part_list) {
		parts_quantity_in_tray += level_2_space + "(= (current-quantity-parttype-in-tray " \
							+ t.part_type + " tray) " + std::to_string(t.part_num) + ")" + "\n";
	}


	return level_1_space + starter + initial_text + handempty + parts_quantity_in_bin \
			+ parts_quantity_in_tray + not_over_bin + over_tray + level_2_space + ender;


}

/**
 * @brief set goal section
 * @param goal_part_list The part type and its amount in tray
 * @return text for goal section in PDDL
 */
std::string update_PDDL::goal_set(const std::vector<Part_Structure>& goal_part_list) {
	std::string initial_text = level_0_space + ":goal (and" + "\n";
	std::string handempty = level_2_space + "(handempty robot)" + "\n";
	std::string parts_quantity_in_tray = "";

	for (auto& p : goal_part_list) {
		parts_quantity_in_tray += level_2_space + "(= (current-quantity-parttype-in-tray " \
				+ p.part_type + " tray) " + std::to_string(p.part_num) +")" + "\n";
	}

	return level_1_space + starter + initial_text + handempty + parts_quantity_in_tray \
			+ level_2_space + ender + level_1_space + ender;

}

/**
 * @brief combine each section to a complete PDDL_Probelm file
 * @param project_name The name of this project
 * @param part_types All part types that will occur in this project
 * @param init_bin_part_list The initial state of part type and quantity in bin
 * @param init_tray_part_list The initial state of part type and quantity in tray
 * @param goal_part_list The part type and its amount in tray
 */
std::string update_PDDL::PDDL_Problem_Text(const std::string& project_name, \
		const std::vector<std::string>& part_types,
		const std::vector<Part_Structure>& init_bin_part_list, \
		const std::vector<Part_Structure>& init_tray_part_list,
		const std::vector<Part_Structure>& goal_part_list) {
	std::string define_section = define_set(project_name);
	std::string domain_section = domain_set(project_name);
	std::string object_section = object_set(part_types);
	std::string init_section = init_set(init_bin_part_list, init_tray_part_list);
	std::string goal_section = goal_set(goal_part_list);

	return level_0_space + starter + define_section + "\n" + domain_section + "\n"\
			+ object_section + "\n" + init_section + "\n" + goal_section + ender;

}

/**
 * @brief combine each section to a complete PDDL_Probelm file
 * @param project_name The name of this project
 * @param file_name The output file name
 * @param part_types All part types will occur in this project e.g (gear, pully, rod)
 * @param init_bin_part_list The initial state of part type and quantity in bin
 * @param init_tray_part_list The initial state of part type and quantity in tray
 * @param goal_part_list The part type and its amount in tray
 */
void update_PDDL::write_to_PDDL(const std::string& project_name, \
		const std::string& file_name,
		const std::vector<std::string> part_types,
		const std::vector<Part_Structure>& init_bin_part_list, \
		const std::vector<Part_Structure>& init_tray_part_list,
		const std::vector<Part_Structure>& goal_part_list) {

	// the text of PDDL Problem file
	std::string PDDL_text = PDDL_Problem_Text(project_name, part_types, init_bin_part_list,\
			init_tray_part_list, goal_part_list);

	std::ofstream fileout(file_name);

	fileout << PDDL_text;


}
