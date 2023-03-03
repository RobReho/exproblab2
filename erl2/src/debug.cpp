#include "erl2/InterfaceAction.h"
#include "ros/ros.h"
#include <cstdlib>
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <erl2/PlanningAction.h>
#include <erl2/Consistent.h>
#include <string> 



ros::ServiceClient cons_client;
ros::ServiceClient sol_client;



int main(int argc, char **argv) 
{
	ros::init(argc, argv, "go_to_next_point", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	
	cons_client = nh.serviceClient<erl2::Consistent>("/checkconsistency");
	sol_client = nh.serviceClient<erl2::Consistent>("/ask_solution");

    // debug
        while(1){
            erl2::Consistent srv;
            std::cout<<"Calling consistency service "<<std::endl;
            srv.request.req = true;
            cons_client.call(srv);
            std::cout<<"Consistency service: "<<std::to_string(srv.response.res)<<std::endl;
            if(srv.response.res==false){
                std::cout<<"returned false: "<<srv.response.res<<std::endl;
            }
            if(srv.response.res==true){
                std::cout<<"returned true: "<<srv.response.res<<std::endl;
            }
            //
            erl2::Consistent data;
            std::cout<<"Calling solution service "<<std::endl;
            data.request.req =true;
            sol_client.call(data);
            std::cout<<"Solution service: "<<std::to_string(data.response.res)<<std::endl;
            if(data.response.res==false){
                std::cout<<"returned false: "<<data.response.res<<std::endl;
            }
            if(data.response.res==true){
                std::cout<<"returned true: "<<data.response.res<<std::endl;
            }
            sleep(3);
        }


	return 0;
}
