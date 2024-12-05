#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"



using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
class VisionBasedControlNode : public rclcpp::Node
{
    public:
        VisionBasedControlNode()
        : Node("ros2_kdl_vision_control"), 
        node_handle_(std::shared_ptr<VisionBasedControlNode>(this))
        {
            // declare task to be executed (positioning, look-at-point)
            declare_parameter("task", "positioning"); // defaults to "positioning"
            get_parameter("task", task_);
            RCLCPP_INFO(get_logger(),"Current task is: '%s'", task_.c_str());

            if (!(task_ == "positioning" || task_ == "look-at-point"))
            {
                RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
            }


            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false;
            aruco_available_ = false;

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");

            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj);
            joint_efforts_.resize(nj);

            joint_velocities_command.resize(nj);
            q_in.resize(nj);


            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&VisionBasedControlNode::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Subscriber to aruco
            arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&VisionBasedControlNode::aruco_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!aruco_available_){
                RCLCPP_INFO(this->get_logger(), "No ArUco data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }


            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;
            

            //Aruco to Base Frame with a Position Offset and an Orientation Offset
            KDL::Frame cam_T_object_offset(aruco_frame_.M*KDL::Rotation::RotY(-0.2), 
                        KDL::Vector(aruco_frame_.p.data[0]-0.05, aruco_frame_.p.data[1], aruco_frame_.p.data[2] - 0.4));

            base_T_object = robot_->getEEFrame() * cam_T_object_offset;


            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q_in);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            KDLController controller_(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0));

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position;
            //end_position << init_position[0], -init_position[1], init_position[2];
            //end_position << aruco_x , aruco_y , aruco_z;
            end_position = toEigen(base_T_object.p);

           // Plan trajectory
            double acc_duration=0.5;
            double t = 0.0;
            double radius = 0.0;
            double traj_duration = 1.5;

            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);
            //std::cout<< "Positioning with Linear Trajectory and Cubic Polynomial \n";
            // Retrieve the first trajectory point
            trajectory_point p = planner_.compute_trajectory(t);

            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            

            /////////////////////////////////PUBLISHER//////////////////////////////////////////

            // Create cmd publisher
            if(task_=="positioning"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&VisionBasedControlNode::cmd_publisher_positioning, this));
                
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }

                RCLCPP_INFO(this->get_logger(), "Starting Positioning with Velocity Controller... \n");
            }

            else if(task_=="look-at-point"){

                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&VisionBasedControlNode::cmd_publisher_look_at_point, this));
                
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }

                RCLCPP_INFO(this->get_logger(), "Starting look-at-point with Velocity Controller... \n");

            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

        }

    private:

        void cmd_publisher_positioning(){

            iteration_ = iteration_ + 1;

            double total_time;
            int trajectory_len;

            total_time = 1.5;
            trajectory_len = 50;

            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
            KDLController controller_(*robot_);

            if (t_ < total_time){

                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_);

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();

                KDL::Twist cartv = robot_->getEEVelocity();

                // Compute desired Frame
                KDL::Frame desFrame;
                desFrame.M = cartpos.M;
                desFrame.p = toKDL(p.pos);
                

                //Desired Velocity
                KDL::Twist des_vel = KDL::Twist::Zero();
                des_vel.rot = cartv.rot;
                des_vel.vel = toKDL(p.vel);            

                //Desired Acceleration
                KDL::Twist des_acc = KDL::Twist::Zero();
                des_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]), KDL::Vector::Zero());    


                Eigen::Vector3d omega_d(des_vel.rot.data);
                Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);

                // Compute Errors
                Eigen::Vector3d error = computeLinearError(Eigen::Vector3d(base_T_object.p.data), Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d dot_error = computeLinearError(p.vel, Eigen::Vector3d(cartv.vel.data));

                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(base_T_object.M)); 
                Eigen::Vector3d o_dot_error = computeOrientationVelocityError(omega_d, omega_e,toEigen(init_cart_pose_.M), toEigen(cartpos.M));


                //std::cout << "The error norm is : " << error.norm() << std::endl;
                //std::cout << aruco_x <<std::endl;
                //std::cout << aruco_y <<std::endl;
                //std::cout << aruco_z <<std::endl;

                //VELOCITY INTERFACE

                // Compute differential IK
                Vector6d cartvel; cartvel << 0.02*p.vel + 5*error, 0.05*o_error;
                joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

            }

            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Positioning Task Executed Successfully ... \n");
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }

        }


        void cmd_publisher_look_at_point(){

            // Publisher for look-at-point task

            //Object Frame
            cam_T_object = KDL::Frame(aruco_frame_.M, KDL::Vector(aruco_frame_.p.data[0], aruco_frame_.p.data[1], aruco_frame_.p.data[2]));

            Eigen::Matrix<double,3,1> cP_o = toEigen(cam_T_object.p);

            //Unit Norm Axis Connecting the Origin of the Camera Frame and the Position of the Object
            Eigen::Matrix<double,3,1> s = cP_o/cP_o.norm();

            //tool0 (End-Effector) Frame
            KDL::Frame base_T_tool0 = robot_->getEEFrame();

            //Rotation and Traslation of the Camera Specified in iiwa_description)/urdf/camera.xacro for the Camera Joint
            KDL::Frame tool0_T_cam(KDL::Rotation::RPY(3.14, -1.57, 0.0), KDL::Vector(0.0, 0.0, 0.0));

            //Base to Camera Frame
            KDL::Frame base_T_cam = base_T_tool0 * tool0_T_cam;

            //Camera Rotation Matrix
            Eigen::Matrix<double,3,3> R_c = toEigen(base_T_cam.M);

            Eigen::Matrix<double,6,6> R_c_big = Eigen::Matrix<double,6,6>::Zero();

            //Diagunal Matrix
            R_c_big.block(0,0,3,3) = R_c;
            R_c_big.block(3,3,3,3) = R_c;

            //L Matrix which maps Linear adn Angular Velocities of the Camera to changes in s
            Eigen::Matrix<double,3,3> L_block = (-1/cP_o.norm())*(Eigen::Matrix<double,3,3>::Identity() - s*s.transpose());
            Eigen::Matrix<double,3,6> L = Eigen::Matrix<double,3,6>::Zero();
            L.block(0,0,3,3) = L_block;
            L.block(0,3,3,3) = skew(s);
            L = L*(R_c_big.transpose());

            //Desired Value
            Eigen::Vector3d sd;
            sd = Eigen::Vector3d(0,0,1);


            KDL::Jacobian J_cam = robot_->getEEJacobian();
            
            Eigen::MatrixXd LJ = L*J_cam.data;
            Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();

            // Matrix spanning the Null-Space of LJ
            Eigen::MatrixXd N = Eigen::Matrix<double,7,7>::Identity() - (LJ_pinv*LJ);

            //std::cout <<"s"<< s.transpose() << std::endl;

            //s Error
            double s_error=(sd-s).norm();

            //A variable that allows me to stop the Joint Velocity Command when the
            //manipulator loses sight of the ArUco tag during movement, before reaching the desired position
            double s_error_old;

            std::cout << "s Error: "<< s_error << std::endl;

            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                std::cout << "Joint " << (i + 1) << " Velocity Command: " << joint_velocities_(i) << std::endl;
            }

            //If error > 0.02 AND ArUco tag is available
            if (s_error>0.02 && s_error!=s_error_old)
            {
                joint_velocities_.data =2*LJ_pinv*sd + N*(q_in.data - joint_positions_.data); 
                s_error_old=s_error;
            }
            //If error<=0.02E OR ArUco tag is not available
            else if(s_error<=0.02 || s_error==s_error_old)
            {
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) 
                    joint_velocities_.data[i]=0;
                    if (s_error<0.02) std::cout <<"Look-at-Point Task Successfully Executed" << std::endl;
            }

            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_(i);
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);


            }



        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){


            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
                joint_efforts_.data[i] = sensor_msg.effort[i];
            }
        }

        //Subscriber to Aruco Pose
        void aruco_subscriber(const geometry_msgs::msg::PoseStamped& aruco_pose_msg)
        {
 
            aruco_available_ = true;
            
            //Position
            aruco_x = aruco_pose_msg.pose.position.x,
            aruco_y = aruco_pose_msg.pose.position.y,
            aruco_z = aruco_pose_msg.pose.position.z;
            
            //Quaternion
            aruco_q1 = aruco_pose_msg.pose.orientation.x,
            aruco_q2 = aruco_pose_msg.pose.orientation.y,
            aruco_q3 = aruco_pose_msg.pose.orientation.z,
            aruco_q4 = aruco_pose_msg.pose.orientation.w;

            KDL::Rotation rot_= KDL::Rotation::Quaternion(aruco_q1,aruco_q2,aruco_q3,aruco_q4);
            KDL::Vector trasl_(aruco_x,aruco_y,aruco_z);
         
            aruco_frame_.p = trasl_;
            aruco_frame_.M = rot_;
 
        }


        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;

        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_efforts_;
        KDL::JntArray joint_velocities_command;

        KDL::JntArray joint_accelerations_;
        KDL::JntArray zero_array_;

        double Kp, Kd;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        int iteration_;
        bool joint_state_available_;
        bool aruco_available_;
        double t_;
        std::string cmd_interface_;
        std::string task_;
            
        KDL::Frame init_cart_pose_;
        KDL::Frame base_T_object;
        KDL::Frame cam_T_object;

        KDL::JntArray q_in;

        KDL::Frame aruco_frame_;
        double aruco_x, aruco_y, aruco_z, aruco_q1, aruco_q2, aruco_q3, aruco_q4;

};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionBasedControlNode>());
    rclcpp::shutdown();
    return 1;
}
