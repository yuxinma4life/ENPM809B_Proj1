#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <osrf_gear/AGVControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include "MyCompetitionClass.h"

void assign_tray_to_kit(int tray_num,int &active_kit, int &order_tf_index, MyCompetitionClass &comp_class,std::map <int,Kit > &kit_world_priority_queue,bool &initialize,tf::TransformListener &listener);

void pick_place_action(const int& active_kit,std::map<int,Kit > &kit_world_priority_queue,std::map <std::string, std::vector<std::vector<float> > > &bin_manager,move_arm::Pick &Movesrv,ros::ServiceClient &client,MyCompetitionClass &comp_class,std::vector<int> &tray_occupied,osrf_gear::AGVControl &Go_delivery,ros::ServiceClient &client_agv1,ros::ServiceClient &client_agv2 );
void Pick_place_faulty(move_arm::Pick &Movesrv,ros::ServiceClient &client,MyCompetitionClass &comp_class);

int main(int argc, char ** argv) {
        // Last argument is the default name of the node.
        ros::init(argc, argv, "ariac_example_node");
        ros::NodeHandle node;
        tf::TransformListener listener;
        // Instance of custom class from above.
        MyCompetitionClass comp_class(node);
        ros::Rate rate(10.0);
        // Subscribe to the '/ariac/orders' topic.
        ros::Subscriber orders_subscriber = node.subscribe(
                "/ariac/orders", 10,
                &MyCompetitionClass::order_callback, &comp_class);
        // Subscribe to the '/ariac/agv1/state' topic
        ros::Subscriber agv1_state = node.subscribe("/ariac/agv1/state",10,&MyCompetitionClass::agv1_state_callback, &comp_class);
        // Subscribe to the '/ariac/agv2/state' topic
        ros::Subscriber agv2_state = node.subscribe("/ariac/agv2/state",10,&MyCompetitionClass::agv2_state_callback, &comp_class);
        // Sucscribe to the
        ros::Subscriber faulty_part = node.subscribe("/tf",10,&MyCompetitionClass::agv_faulty_part_detect, &comp_class);

        bool initialize = false;
        int active_kit = 0;
        int order_tf_index = 0;
        std::map <int, Kit > kit_world_priority_queue;
        std::vector<int> tray_occupied(2,0);
        // Parts locations on the bin, using map
        std::map <std::string, std::vector<std::vector<float> > > bin_manager;
        std::vector<std::vector<float> > bin_gear = {{-0.37,-1.13,0.745},{-0.37,-1.26,0.745},{-0.37,-1.4,0.745},{-0.23,-1.26,0.745},{-0.5,-1.53,0.745},{-0.5,-1.26,0.745},{-0.5,-1.13,0.745}};
        std::vector<std::vector<float> > bin_rod  = {{-0.4,-0.635,0.745},{-0.2,-0.435,0.745},{-0.2,-0.635,0.745},{-0.4,-0.435,0.745}};
        std::vector<std::vector<float> > bin_gasket = {{-0.4,0.13,0.76},{-0.4,0.33,0.76},{-0.4,0.13,0.76},{-0.2,0.33,0.76}};
        std::vector<std::vector<float> > bin_disk = {{-0.4,0.9,0.75},{-0.4,1.1,0.75},{-0.2,1.1,0.75},{-0.2,0.9,0.75}};
        bin_manager.insert(std::pair<std::string,std::vector<std::vector<float> > >("gear_part",bin_gear));
        bin_manager.insert(std::pair<std::string,std::vector<std::vector<float> > >("piston_rod_part",bin_rod));
        bin_manager.insert(std::pair<std::string,std::vector<std::vector<float> > >("gasket_part",bin_gasket));
        bin_manager.insert(std::pair<std::string,std::vector<std::vector<float> > >("disk_part",bin_disk));
// MoveIt service
        move_arm::Pick Movesrv;
        osrf_gear::AGVControl Go_delivery;
        ros::ServiceClient client = node.serviceClient<move_arm::Pick>("/move_arm/toPose");
        ros::ServiceClient client_agv1 = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
        ros::ServiceClient client_agv2 = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");

        while(ros::ok()) {
                if(!comp_class.kit_priority_queue.empty()) {

                        // initialize the current highest priority kit(from 1, 2,3 ...), set the active_kit as its priority, the lastest kit has higher priority
                        // Only initialize once
                        if(!initialize) {
                                ROS_INFO_STREAM("initialize kit");
                                assign_tray_to_kit(1,active_kit,order_tf_index,comp_class,kit_world_priority_queue,initialize,listener);
                        }
                        // If the active_kit is the highest priority kit, do the following
                        if(active_kit == comp_class.kit_priority_queue.rbegin()->first) {
                                pick_place_action( active_kit,kit_world_priority_queue,bin_manager,Movesrv,client,comp_class,tray_occupied,Go_delivery,client_agv1,client_agv2);
                        }
                        // Which means new kit is comming and it's desired location is assigned respect to AGV1
                        else if(comp_class.agv1_ready_position && tray_occupied[0] == 0 )
                                assign_tray_to_kit(1,active_kit,order_tf_index,comp_class,kit_world_priority_queue,initialize,listener);
                        else if(comp_class.agv2_ready_position && tray_occupied[1] == 0)
                                assign_tray_to_kit(2,active_kit,order_tf_index,comp_class,kit_world_priority_queue,initialize,listener);
                        else {
                                if(comp_class.agv1_ready_position && tray_occupied[0] != 0)
                                        pick_place_action( tray_occupied[0],kit_world_priority_queue,bin_manager,Movesrv,client,comp_class,tray_occupied,Go_delivery,client_agv1,client_agv2);
                                if(comp_class.agv2_ready_position && tray_occupied[1] != 0)
                                        pick_place_action( tray_occupied[1],kit_world_priority_queue,bin_manager,Movesrv,client,comp_class,tray_occupied,Go_delivery,client_agv1,client_agv2);
                        }
                }
                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}

void assign_tray_to_kit(int tray_num,int &active_kit, int &order_tf_index, MyCompetitionClass &comp_class,std::map <int,Kit > &kit_world_priority_queue,bool &initialize,tf::TransformListener &listener) {
        Kit kit_world;
        active_kit = comp_class.kit_priority_queue.rbegin()->first;
        int current_index = order_tf_index;
        for( order_tf_index; order_tf_index < current_index + comp_class.kit_priority_queue[active_kit].order_in_kit_list.size(); order_tf_index++) {
                ROS_INFO_STREAM("This kit "<<active_kit<<" has "<< comp_class.kit_priority_queue[active_kit].order_in_kit_list.size()<< " of objects");
                comp_class.orderPoseBroadcaster(comp_class.kit_priority_queue[active_kit].order_in_kit_list[order_tf_index - current_index].pose,order_tf_index,tray_num);
                geometry_msgs::Pose world_pose = comp_class.desired_part_pose(listener,order_tf_index,tray_num);
                if (world_pose.position.x != 0) {
                        initialize = true;
                        order order_world;
                        order_world.pose = world_pose;
                        order_world.type = comp_class.kit_priority_queue[active_kit].order_in_kit_list[order_tf_index -current_index].type;
                        kit_world.order_in_kit_list.push_back(order_world);
                        kit_world.tray_num = tray_num;
                }
        }
        if(!kit_world.order_in_kit_list.empty()) {
                ROS_INFO_STREAM("Assign world Kit list");
                kit_world.priority = active_kit;
                kit_world_priority_queue.insert(std::pair<int,Kit>(kit_world.priority,kit_world));
        }
        else
                active_kit = 0;
}

void pick_place_action(const int& active_kit,std::map<int,Kit > &kit_world_priority_queue,std::map <std::string, std::vector<std::vector<float> > > &bin_manager,move_arm::Pick &Movesrv,ros::ServiceClient &client,MyCompetitionClass &comp_class,std::vector<int> &tray_occupied,osrf_gear::AGVControl &Go_delivery,ros::ServiceClient &client_agv1,ros::ServiceClient &client_agv2){
        ROS_INFO_STREAM("Active kit is "<< active_kit);
        if (!kit_world_priority_queue[active_kit].order_in_kit_list.empty()) {
                // Pick the object
                if(bin_manager.find(kit_world_priority_queue[active_kit].order_in_kit_list.front().type) != bin_manager.end() ) {
                        // Find the desired part location on the bin from the  bin_manager, and choose the first avaiable part
                        Movesrv.request.pose.position.x = bin_manager.find(kit_world_priority_queue[active_kit].order_in_kit_list.front().type)->second.front()[0];
                        Movesrv.request.pose.position.y = bin_manager.find(kit_world_priority_queue[active_kit].order_in_kit_list.front().type)->second.front()[1];
                        Movesrv.request.pose.position.z = bin_manager.find(kit_world_priority_queue[active_kit].order_in_kit_list.front().type)->second.front()[2];
                        Movesrv.request.mode = 1;
                        ROS_INFO_STREAM("Picking part "<< kit_world_priority_queue[active_kit].order_in_kit_list.front().type<<" at location "<< Movesrv.request.pose.position.x<<Movesrv.request.pose.position.y);
                        client.call(Movesrv);
                        // if successfully pick the part from the bin then delete the picked part from the part location list
                        bin_manager.find(kit_world_priority_queue[active_kit].order_in_kit_list.front().type)->second.erase(bin_manager.find(kit_world_priority_queue[active_kit].order_in_kit_list.front().type)->second.begin());
                }
                else {
                        ROS_INFO_STREAM("No desired part "<< comp_class.kit_priority_queue[active_kit].order_in_kit_list.front().type);
                }

                // place the object
                Movesrv.request.pose.position.x = kit_world_priority_queue[active_kit].order_in_kit_list.front().pose.position.x;    // comp_class.pick_place_list[0].second.position.x;
                Movesrv.request.pose.position.y = kit_world_priority_queue[active_kit].order_in_kit_list.front().pose.position.y; // comp_class.pick_place_list[0].second.position.y;
                ROS_INFO_STREAM(Movesrv.request.pose.position.x);
                ROS_INFO_STREAM(Movesrv.request.pose.position.y);
                Movesrv.request.pose.position.z = 0.9;
                Movesrv.request.mode = 2;
                client.call(Movesrv);
                //  if successfully place the part to desired location, delete this order in the kit
                tray_occupied[kit_world_priority_queue[active_kit].tray_num -1] = active_kit;
                //  kit_world_priority_queue[active_kit].order_in_kit_list.erase(kit_world_priority_queue[active_kit].order_in_kit_list.begin());
                //  ROS_INFO_STREAM("Occupied tray is "<<kit_world_priority_queue[active_kit].tray_num<<" by kit"<<  tray_occupied[kit_world_priority_queue[active_kit].tray_num -1]);
                ros::Duration(1).sleep();
                ros::spinOnce();
                if (comp_class.faulty_on_agv) {
                        Pick_place_faulty(Movesrv,client,comp_class);
                        comp_class.faulty_on_agv = false;
                }
                else {
                        kit_world_priority_queue[active_kit].order_in_kit_list.erase(kit_world_priority_queue[active_kit].order_in_kit_list.begin());
                        ROS_INFO_STREAM("Occupied tray is "<<kit_world_priority_queue[active_kit].tray_num<<" by kit"<<  tray_occupied[kit_world_priority_queue[active_kit].tray_num -1]);
                }

                // TUDO if not successfully placed
        }
        else {
                ROS_INFO_STREAM("Kit is finished, delete it");
                std::string kitname = "kit";
                std::string num = std::to_string(active_kit);
                kitname = kitname + num;
                Go_delivery.request.kit_type = kitname;
                ros::Duration(1).sleep();
                if(kit_world_priority_queue[active_kit].tray_num == 1)
                        client_agv1.call(Go_delivery);
                if(kit_world_priority_queue[active_kit].tray_num == 2)
                        client_agv2.call(Go_delivery);
                tray_occupied[kit_world_priority_queue[active_kit].tray_num -1] = 0;
                comp_class.kit_priority_queue.erase(active_kit);
                ros::Duration(1).sleep();
        }
}

void Pick_place_faulty(move_arm::Pick &Movesrv,ros::ServiceClient &client,MyCompetitionClass &comp_class) {
        // Pick the faulty part
        Movesrv.request.pose.position.x = comp_class.faulty_pose.position.x;
        Movesrv.request.pose.position.y = comp_class.faulty_pose.position.y;
        Movesrv.request.pose.position.z = 0.78;
        Movesrv.request.mode = 1;
        ROS_INFO_STREAM("Picking the faulty part "<<" at location "<< Movesrv.request.pose.position.x<<Movesrv.request.pose.position.y);
        client.call(Movesrv);
        // Place the faulty part
        Movesrv.request.pose.position.x =-0.4;
        Movesrv.request.pose.position.y =  2.2;
        Movesrv.request.pose.position.z =  1.3;
        ROS_INFO_STREAM(Movesrv.request.pose.position.x);
        ROS_INFO_STREAM(Movesrv.request.pose.position.y);
        Movesrv.request.mode = 5;
        client.call(Movesrv);
}
