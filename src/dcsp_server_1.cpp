//
//  main.cpp
//  DCSPNode
//
//  Created by Dulitha Dabare on 11/10/17.
//  Copyright © 2017 Dulitha Dabare. All rights reserved.
//

#include <iostream>
#include "ros/ros.h"
#include "dcsp_new/dcsp_srv.h"


#include <list>
#include <vector>
#include <math.h>


class Agent {
    
    
    struct point {
        double x;
        double y;
        double z;
        double yaw;
    };
    
    struct agent_view_element {
        std::string agent_identifier;
        point value;
    };
    
    /* TEMP struct defintions in place of service request and response */
    struct req {
        
        std::string init_agent_identifier;
        point init_point;
        std::vector<std::string> agents;
        std::vector<point> my_domain;
        std::string ok_agent_identifier;
        point ok_value;
        std::vector<agent_view_element> no_goods;
        bool init;
        bool ok;
        bool nogood;
        
    };
    
    struct res {
        
        double x;
        double y;
        double z;

        
    };
    
    
    
private:
    
    std::vector<std::string> higher_priority_agents;
    std::vector<std::string> lower_priority_agents;
    
    std::string my_agent_identifier;
    //std::string my_current_point;
   // point my_current_point;
    point my_current_point = {
        0.0,
        0.0,
        0.0,
        0.0
    };
    std::vector<point> my_domain;
    std::vector<point> my_point_domain; //temporary because we have to change the my_domain to be of point type vector
    std::vector<agent_view_element> agent_view;
    std::vector<agent_view_element> no_goods_list;
    bool is_agent_initialized = false;
    
    
public:
  //  Agent(std::vector<Agent> &agents,const std::string &init_agent_identifier,const point &init_value,const std::vector<point> &init_my_domain , point &init_point);
    void agent_init(std::vector<std::string> &agents,const std::string &init_agent_identifier,const point &init_value,const std::vector<point> &init_my_domain);
    void recieve_ok_msg(const std::string &agent_identifier,const point &value);
    void receive_nogood_msg(const std::vector<agent_view_element> &no_goods);
    void send_ok_msg(Agent &agent);
    void send_nogood_msg(const std::vector<agent_view_element> &no_goods, const std::string lowest_priority_agent_identifier);
    void check_agent_view();
    bool check_consistency();
    bool check_compatibility ();
    bool same_points(const point &point1, const point &point2);
    void backtrack();
    void select_best_point();
    bool select_method(dcsp_new::dcsp_srv::Request &req, dcsp_new::dcsp_srv::Request &res);
    void dcsp_srv_2_recieve_ok();
    void dcsp_srv_3_recieve_ok();
    
    
    
};
/*
Agent::Agent(std::vector<Agent> &agents,const std::string &init_agent_identifier,const point &init_value,const std::vector<point> &init_my_domain , point &init_point){
   
    my_agent_identifier = init_agent_identifier;
    my_current_point = init_value;
    my_domain = init_my_domain;
    my_current_point = init_point;
    
    std::vector<Agent>::iterator it;
    
    for (it = agents.begin(); it != agents.end(); ++it){
        
        if (my_agent_identifier.compare(it->my_agent_identifier) > 0) {
            
            higher_priority_agents.push_back(*it);
            
        }else if (my_agent_identifier.compare(it->my_agent_identifier) < 0) {
            
            lower_priority_agents.push_back(*it);
        }
        
        // what if the agent identifier is the same as the my_agent_identifier?
        
        
    }
    // select the point giving the best euclidean distance
    
    // point best_point = select_best_point();
    // send ok message to all the elements in the lower_priority_agents
    

    
    
}
*/

void Agent::agent_init(std::vector<std::string> &agents,const std::string &init_agent_identifier,const point &init_value,const std::vector<point> &init_my_domain){
    
    my_agent_identifier = init_agent_identifier;
    my_current_point = init_value;
    my_domain = init_my_domain;
    //my_current_point = init_point;
    
    
    std::vector<std::string>::iterator it;
    
    for (it = agents.begin(); it != agents.end(); ++it){
        
        if (my_agent_identifier.compare(*it) > 0) {
            
            higher_priority_agents.push_back(*it);
            
        }else if (my_agent_identifier.compare(*it) < 0) {
            
            lower_priority_agents.push_back(*it);
        }
        
        // what if the agent identifier is the same as the my_agent_identifier?
        
        
    }
    // select the point giving the best euclidean distance
    
    select_best_point();

    // send ok message to all the elements in the lower_priority_agents
    is_agent_initialized = true;
    if(my_agent_identifier == "firefly1"){
        dcsp_srv_2_recieve_ok();
        dcsp_srv_3_recieve_ok();
    }else if(my_agent_identifier == "firefly2"){
        dcsp_srv_3_recieve_ok();
    }
    
    
    
    
}

void Agent::recieve_ok_msg(const std::string &agent_identifier, const point &value){
    
    bool is_agent_in_agent_view = false;
    
    std::vector<agent_view_element>::iterator it;

    
    for (it = agent_view.begin(); it != agent_view.end(); ++it){
        
        if(it->agent_identifier == agent_identifier){
            
            is_agent_in_agent_view = true;
        }
        
    }
    
    if (!is_agent_in_agent_view) {
        agent_view_element new_element;
        new_element.agent_identifier = agent_identifier;
        new_element.value = value;
        agent_view.push_back(new_element);
    }
    
    check_agent_view();
    
}

void Agent::receive_nogood_msg(const std::vector<agent_view_element> &no_goods){
    
    no_goods_list.insert(no_goods_list.end(), no_goods.begin(),no_goods.end());
    check_agent_view();
    
    
}

void Agent::check_agent_view(){
    
    if (!check_consistency()) {

        ROS_INFO("not consistent");
        bool is_new_value_consistent = false;
        
        std::vector<point>::iterator it;
        
        for (it = my_domain.begin(); it != my_domain.end(); ++it){
            
            if (!same_points(*it, my_current_point)) {
                
                std::vector<agent_view_element>::iterator it2;
                
                
                for (it2 = agent_view.begin(); it2 != agent_view.end(); ++it2){
                    
                    if(same_points(it2->value, *it)){
                        is_new_value_consistent = false;
                        break;
                        
                    }else{
                        is_new_value_consistent = true;
                    }
                    
                }
            }
            
            if (is_new_value_consistent) {

                ROS_INFO("new consistent value found");
                my_current_point = *it;
                
                std::vector<std::string>::iterator it3;
                
                
                for (it3 = lower_priority_agents.begin(); it3 != lower_priority_agents.end(); ++it3){
                    
                    std::string lower_priority_agent = *it3;
                    //lower_priority_agent.recieve_ok_msg(my_agent_identifier, my_current_point);
                    
                    /* Call the corresponding firefly node's service. Example : for firefly1 call dcspe_srv_1 service */
                    
                    
                    std::string service_name;
                    
                    if (lower_priority_agent == "firefly1") {
                        service_name = "dcsp_srv_1";
                    }else if (lower_priority_agent == "firefly2") {
                        service_name = "dcsp_srv_2";
                    }else if (lower_priority_agent == "firefly3") {
                        service_name = "dcsp_srv_3";
                    }
                    
                    
                     
                     ros::NodeHandle n;
                     ros::ServiceClient client = n.serviceClient<dcsp_new::dcsp_srv>(service_name);
                     dcsp_new::dcsp_srv srv;
                    // srv.request.init_agent_identifier = ""NULL"";
                    // srv.request.init_point = NULL;
                    // srv.request.agents = NULL;
                    // srv.request.my_domain = NULL;
                    // srv.request.ok_agent_identifier = my_agent_identifier;
                     srv.request.ok_value.x = my_current_point.x;
                     srv.request.ok_value.y = my_current_point.y;
                   //  srv.request.ok_value.x = my_current_point.x;
                    // srv.request.no_goods = NULL;
                     srv.request.init = false;
                     srv.request.ok = true;
                     srv.request.nogood = false;
                     
                     if (client.call(srv))
                     {
                     //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
                     }
                     else
                     {
                     //  ROS_ERROR("Failed to call service add_two_ints");
                     //  return 1;
                     }
                     
               
                    
                    
                }
                
                break;
                
            }
            
            
            
        }
        
        if (!is_new_value_consistent) {
            backtrack();
        }
        
    }else{
        ROS_INFO("FIREFLY 1 value is consistent");
    }
    
}

bool Agent::check_consistency(){
    
    bool is_consistent = false;
    bool is_agent_view_consistent = false;
    
    std::vector<agent_view_element>::iterator it;
    
    for (it = agent_view.begin(); it != agent_view.end(); ++it){
        
        if (same_points(it->value, my_current_point)) {
            is_agent_view_consistent = false;
        }
        
    }
    
    bool is_compatible = check_compatibility();
    
    if (is_agent_view_consistent && (!is_compatible)) {
        is_consistent = true;
    }else{
        is_consistent = false;
    }
    
    return is_consistent;
    
}

bool Agent::check_compatibility(){
    
    bool is_compatible = false;
    
    std::vector<agent_view_element>::iterator it;
    
    for (it = no_goods_list.begin(); it != no_goods_list.end(); ++it){
        
        if (it->agent_identifier == my_agent_identifier) {
            
            if (same_points(it->value, my_current_point)) {
                is_compatible = true;
                continue;
            }else{
                
                is_compatible = false;
                break;
            }
        }
        
        
        std::vector<agent_view_element>::iterator agent_view_it;
        
        for (agent_view_it = agent_view.begin(); agent_view_it != agent_view.end(); ++agent_view_it){
            
            if (same_points(agent_view_it->value, it->value)) {
                is_compatible = true;
                continue;
            }else{
                
                is_compatible = false;
                break;
            }
        }
        
        if (is_compatible) {
            continue;
        }else{
            break;
        }
        
        
    }
    
    return is_compatible;
}


void Agent::backtrack(){
    
    // Add inconsistent subset of agent_view to a new vector called no_goods
    
    
    std::vector<agent_view_element> no_goods;
    
    std::vector<agent_view_element>::iterator it;
    
    for (it = agent_view.begin(); it != agent_view.end(); ++it){
        
        if (same_points(it->value, my_current_point) ) {
            
            no_goods.push_back(*it);
        }
        
    }
    
    if (no_goods.empty()) {
        puts("Algortithm terminated");
    }else{
        
        agent_view_element first_agent_view_element = no_goods.front();
        std::string lowest_priority_agent_identifier = first_agent_view_element.agent_identifier;
        
        
        std::vector<agent_view_element>::iterator it2;
        
        for (it2 = agent_view.begin(); it2 != agent_view.end(); ++it2){
            
            if ((lowest_priority_agent_identifier.compare(it2->agent_identifier)) < 0) {
                
                lowest_priority_agent_identifier = it2->agent_identifier;
                
            }
            
        }
        
        send_nogood_msg(no_goods, lowest_priority_agent_identifier);
        
    }
    
}

void Agent::send_nogood_msg(const std::vector<agent_view_element> &no_goods, const std::string lowest_priority_agent_identifier){
    
    std::vector<std::string>::iterator it;
    
    for (it = higher_priority_agents.begin(); it != higher_priority_agents.end(); ++it){
        
        if (*it == lowest_priority_agent_identifier) {
            
         //   it->receive_nogood_msg(no_goods);
            
            /* Call the corresponding firefly node's service. Example : for firefly1 call dcspe_srv_1 service */
           
            
            std::string service_name;
            
            if (lowest_priority_agent_identifier == "firefly1") {
                service_name = "dcsp_srv_1";
            }else if (lowest_priority_agent_identifier == "firefly2") {
                service_name = "dcsp_srv_2";
            }else if (lowest_priority_agent_identifier == "firefly3") {
                service_name = "dcsp_srv_3";
            }
            
          
             /*   ROS SERVICE CALL TO THE CORRESPONDING FIREFLY DCSP SERVICE with NOGOOD flag */
             
            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<dcsp_new::dcsp_srv>(service_name);
            dcsp_new::dcsp_srv srv;
            // srv.request.init_agent_identifier = NULL;
            // srv.request.init_point = NULL;
            // srv.request.agents = NULL;
            // srv.request.my_domain = NULL;
            // srv.request.ok_agent_identifier = NULL;
            // srv.request.ok_value = NULL;
            srv.request.no_goods = no_goods;
            srv.request.init = false;
            srv.request.ok = false;
            srv.request.nogood = true;
            
            if (client.call(srv))
            {
                    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
                    }
            else
                {
                  //  ROS_ERROR("Failed to call service add_two_ints");
                  //  return 1;
                    }
            
             
            
        }
        
    }
    
}

void Agent::select_best_point(){
    
    
    double closest_euclidean_distance = 0.0;
    
    std::vector<point>::iterator it;
    
    for (it = my_domain.begin(); it != my_domain.end(); ++it){
        
        double x = it->x - my_current_point.x;
        double y = it->y - my_current_point.y;
        double z = it->z - my_current_point.z;
        double dist;
        
        dist = pow(x,2)+pow(y,2)+pow(z,2);
        
        double euclidean_distance = sqrt(dist);
        
        if (euclidean_distance<closest_euclidean_distance) {
            closest_euclidean_distance = euclidean_distance;
            my_current_point = *it;
        }
        
    }
    
    
    
}

bool Agent::same_points(const Agent::point &point1, const Agent::point &point2){
    
    bool is_same_point = false;
    
    if((point1.x == point2.x) && (point1.y == point2.y) ){
        is_same_point = true;
    }
    
    return is_same_point;
    
}

void dcsp_srv_2_recieve_ok(){

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dcsp_new::dcsp_srv>("dcsp_srv_2");
    dcsp_new::dcsp_srv srv;
    // srv.request.init_agent_identifier = NULL;
    // srv.request.init_point = NULL;
    // srv.request.agents = NULL;
    // srv.request.my_domain = NULL;
    srv.request.ok_agent_identifier = my_agent_identifier;
    srv.request.ok_value = my_current_point;
    // srv.request.no_goods = NULL;
    srv.request.init = false;
    srv.request.ok = true;
    srv.request.nogood = false;
    
    if (client.call(srv))
    {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
    //  ROS_ERROR("Failed to call service add_two_ints");
    //  return 1;
    }

}

void dcsp_srv_3_recieve_ok(){
    
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<dcsp_new::dcsp_srv>("dcsp_srv_3");
        dcsp_new::dcsp_srv srv;
        // srv.request.init_agent_identifier = NULL;
        // srv.request.init_point = NULL;
        // srv.request.agents = NULL;
        // srv.request.my_domain = NULL;
        srv.request.ok_agent_identifier = my_agent_identifier;
        srv.request.ok_value = my_current_point;
        // srv.request.no_goods = NULL;
        srv.request.init = false;
        srv.request.ok = true;
        srv.request.nogood = false;
        
        if (client.call(srv))
        {
        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
        }
        else
        {
        //  ROS_ERROR("Failed to call service add_two_ints");
        //  return 1;
        }
    
    }
bool Agent::select_method(dcsp_new::dcsp_srv::Request &req, dcsp_new::dcsp_srv::Request &res){
    
    //Agent::point point;
    //Agent::req request;
    //Agent::req response;
    
    

    if(req.init){
        
        agent_init(req.agents, req.init_agent_identifier,req.init_point ,req.my_domain);
        
    }else if (req.ok){
        
        recieve_ok_msg(req.ok_agent_identifier, req.ok_value);
        return true;
    }else if (req.nogood){
        receive_nogood_msg(req.no_goods);
    }
    
    res.x = my_current_point.x;
    res.y = my_current_point.y;
    res.z = my_current_point.z;

    return true;
    
}


int main(int argc, const char * argv[]) {
    // insert code here...
    ros::init(argc, argv, "dcsp_server_1");
    ros::NodeHandle n;
    Agent agent_temp;
    ros::ServiceServer service = n.advertiseService("dcsp_srv_1", &Agent::select_method,&agent_temp);
    ROS_INFO("DCSP SERVICE 1 STARTED");
    ros::spin();
    return 0;
}

