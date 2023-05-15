#include <iostream>
#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// #include <gazebo_ros/conversions/builtin_interfaces.hpp>
// #include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
// #ifdef IGN_PROFILER_ENABLE
// #include <ignition/common/Profiler.hh>
// #endif
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sdf/sdf.hh>
#include <rclcpp/rclcpp.hpp>

// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>
using namespace std;

//Podcar Gazebo-to-ROS plugin.   Simulates sensors and recieves wheel commands.


namespace gazebo
{
  MyPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			{// int argc=0;
			// char** argv;		
			// ros::init(argc, argv, "GazeboPlugin");
            ros_node_ = gazebo_ros::Node();
            RCLCPP_INFO(ros_node_->get_logger(), "Loading  Gazebo Plugin");

			this->model = _parent;   //*er to my physical model, that I'm controlling
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyPlugin::OnUpdate, this, _1));      //GAZEBO (not ROS) callback
			// plugin is subscribing from the speedmetersec and wheelAngleCmd topics 
            
            this->speedsub = ros_node_.create_subscription<std_msgs::msg::Float64>("speedcmd_metersec", 1, &MyPlugin::wheelSpeedCallback, (MyPlugin*)this);
            this->wheelAnglesub = ros_node_.create_subscription<std_msgs::msg::Float64>("wheelAngleCmd", 1, &MyPlugin::wheelAngleCallback, (MyPlugin*)this);
            
            //publish the groundtruth odometry
            this->_pub_gnd = ros_node_.create_publisher<nav_msgs::msg::Odometry>("odometry/groundTruth", 1000);
			
			this-> timeOut = 2;
			this-> lastSpeedCmdTime = time(NULL);
			this-> lastAngleCmdTime = time(NULL);
			this-> timedOut = false;

			this->fR = this->model->GetJoint("tracking_right_pivot_joint");
			this->fL = this->model->GetJoint("tracking_left_pivot_joint");

			this->bR = this->model->GetJoint("back_right_joint");
			this->bL = this->model->GetJoint("back_left_joint");

			this->angleController = common::PID(2, 0, 1);

			this->model->GetJointController()->SetPositionPID(fL->GetScopedName(), this->angleController);
			this->model->GetJointController()->SetPositionPID(fR->GetScopedName(), this->angleController);
		}

		public: void wheelSpeedCallback(const std_msgs::Float64& msg){
			this-> timedOut = false;
			this->lastSpeedCmdTime = time(NULL);
			
			bL->SetVelocity(0, msg.data/0.17775); //divide by wheel radius
			bR->SetVelocity(0, msg.data/0.17775);
		}

		public: void wheelAngleCallback(const std_msgs::Float64& msg) {
			this-> timedOut = false;
			this->lastAngleCmdTime = time(NULL);

			this->model->GetJointController()->SetPositionTarget(fL->GetScopedName(), msg.data);
			this->model->GetJointController()->SetPositionTarget(fR->GetScopedName(), msg.data);
		}

		public: void brake(){
			bL->SetVelocity(0, 0);   
			bR->SetVelocity(0, 0);  

			physics::JointPtr jointFL = this->model->GetJoint("front_left_joint");
			physics::JointPtr jointFR = this->model->GetJoint("front_right_joint");

			jointFL->SetVelocity(0, 0);   
			jointFR->SetVelocity(0, 0);  
		}

		public: void OnUpdate(const common::UpdateInfo & /*_info*/){
			//cout << "update " << endl;

			std::time_t timeNow = time(NULL);
			if(timeNow >= this->lastSpeedCmdTime+this->timeOut && !this->timedOut){
				this-> timedOut = true;
				this->brake();
			}
			else if(timeNow >= this->lastAngleCmdTime+this->timeOut && !this->timedOut){
				this-> timedOut = true;
				this->brake();
			}
            auto pose = this->model->GetWorldPose();
            nav_msgs::Odometry odomOut;
            this->odomOut.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
            this->odomOut.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());
            this->_pub_gnd.publish(odomOut);
			// math::Pose pose = this->model->GetWorldPose();
			// math::Vector3& pos = pose.pos;
			// math::Quaternion& quat = pose.rot;
			// double x = pos.x;
			// double y = pos.y;
			// double z = pos.z;
			// double qx = quat.x;
			// double qy = quat.y;
			// double qz = quat.z;
			// double qw = quat.w;
			// nav_msgs::Odometry odomOut;
			// odomOut.header.stamp = ros::Time::now(); 
			// odomOut.header.frame_id = "map";
			// odomOut.pose.pose.position.x = x;
			// odomOut.pose.pose.position.y = y;
			// odomOut.pose.pose.position.z = z;    				//z=height
			// odomOut.pose.pose.orientation.x = qx;       //from TF quat to ROS msg (yuk!)
			// odomOut.pose.pose.orientation.y = qy;
			// odomOut.pose.pose.orientation.z = qz;
			// odomOut.pose.pose.orientation.w = qw;
			// this->_pub_gnd.publish(odomOut);
		}

		// public: ros::NodeHandle _n;
		// protected: ros::Subscriber speedSub;
		// protected: ros::Subscriber wheelAngleSub;
		// protected: ros::Publisher _pub_gnd;
        protected: rclcpp::Subscription speedSub;
		protected: rclcpp::Subscription wheelAngleSub;
		protected: rclcpp::Publisher _pub_gnd;
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		
		private: std::time_t lastSpeedCmdTime;
		private: std::time_t lastAngleCmdTime;
		private: std::time_t timeOut;
		private: bool timedOut;

		public: common::PID angleController;

		private: physics::JointPtr fR;
		private: physics::JointPtr fL;
		private: physics::JointPtr bR;
		private: physics::JointPtr bL;
};

	GZ_REGISTER_MODEL_PLUGIN(MyPlugin)   //tell Gazebo I'm here
}
