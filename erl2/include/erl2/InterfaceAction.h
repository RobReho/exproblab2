#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"

namespace KCL_rosplan {
    
    class GoToNextPointInterface: public RPActionInterface
    {
        private:
        public:
            /* constructor */
            GoToNextPointInterface(ros::NodeHandle &nh);
            /* listen to and process action_dispatch topic */
            bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };

    class LeaveOracleInterface: public RPActionInterface
    {
        private:
        public:
            /* constructor */
            LeaveOracleInterface(ros::NodeHandle &nh);
            /* listen to and process action_dispatch topic */
            bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };

    class GoToOracleInterface: public RPActionInterface
    {
        private:
        public:
            /* constructor */
            GoToOracleInterface(ros::NodeHandle &nh);
            /* listen to and process action_dispatch topic */
            bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };


    class CollectHintInterface: public RPActionInterface
    {
        private:
        public:
            /* constructor */
            CollectHintInterface(ros::NodeHandle &nh);
            /* listen to and process action_dispatch topic */
            bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };

    class CompleteQueryInterface: public RPActionInterface
    {
        private:
        public:
            /* constructor */
            CompleteQueryInterface(ros::NodeHandle &nh);
            /* listen to and process action_dispatch topic */
            bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };

    class SolutionQueryInterface: public RPActionInterface
    {
        private:
        public:
            /* constructor */
            SolutionQueryInterface(ros::NodeHandle &nh);
            /* listen to and process action_dispatch topic */
            bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };

}
