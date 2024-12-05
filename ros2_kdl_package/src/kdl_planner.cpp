#include "kdl_planner.h"

KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}


//Constructor for Linear Trajectory with Trapezoidal Velocity Profile 
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

//Constructor for Linear Trajectory with Cubic Polynomial 
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

//Constructor for Circular Trajectory with Cubic Polynomial
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}


//Constructor for Circular Trajectory with Trapezoidal Velocity Profile
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;    
}



void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}


//Trapezoidal Velocity
void KDLPlanner::trapezoidal_vel(double t, double tc, double &s, double &s_dot, double &s_ddot){

    //s0 = 0;
    //sf = 1; 
    //Compute Acceleration
    double sc_ddot = 4.0/(std::pow(trajDuration_,2));

    //Acceleration
    if (t <= tc)
    {
        s = 0.5*sc_ddot*std::pow(t,2);
        s_dot = sc_ddot*t;
        s_ddot = sc_ddot; 
    }
    //Cruise
    else if (t <= trajDuration_- tc)
    {
        s = sc_ddot*tc*(t-tc/2);
        s_dot = sc_ddot*tc;
        s_ddot = 0;
    }
    //Deceleration 
    else
    {
        s = 1 - 0.5*sc_ddot*std::pow(trajDuration_-t,2);
        s_dot = sc_ddot*(trajDuration_-t);
        s_ddot = -sc_ddot;
    }
}

//Cubic Polynomial
void KDLPlanner::cubic_polinomial(double t, double &s, double &s_dot, double &s_ddot){
    //Remember that s(t) = a3*t^3 + a2*t^2 + a1*t + a0
    //Offline Coefficients Computation imposing Boundary Conditions

    double a0, a1, a2, a3;
    a0 = 0.0;
    a1 = 0.0;
    a2 = 3/(std::pow(trajDuration_,2));
    a3 = -2/(std::pow(trajDuration_,3));

    s = a3*std::pow(t,3) + a2*std::pow(t,2) + a1*t + a0;
    s_dot = 3*a3*std::pow(t,2) + 2*a2*t +a1;    //Parabolic Velocity Profile
    s_ddot = 6*a3*t +2*a2;  //Linear Acceleration Profile
}


// 1. Linear Trajectory with Trapezoidal Velocity Profile (you have to define traj_duration, acc_duration, init_position, end_position)
// 2. Linear Trajectory with Cubic Polynomial (you have to define traj_duration, init_position, end_position)
// 3. Circular Trajectory with Trapezoidal Velocity Profile (you have to define traj_duration, acc_duration, init_position, radius)
// 4. Circular Trajectory with Cubic Polynomial (you have to define traj_duration, init_position, radius)

trajectory_point KDLPlanner::compute_trajectory(double time)
{
    trajectory_point traj; 
    double s, s_dot, s_ddot;

    if((trajRadius_ == 0 && accDuration_ != 0) || (trajRadius_ == 0 && accDuration_ == 0))    //Linear Trajectory
    {
        if(trajRadius_ == 0 && accDuration_ != 0)
        {
            trapezoidal_vel(time, accDuration_, s, s_dot, s_ddot);
        }
        else
        {
            cubic_polinomial(time, s, s_dot, s_ddot);
        }

        traj.pos = trajInit_ + s*(trajEnd_-trajInit_);
        traj.vel = s_dot * (trajEnd_-trajInit_);
        traj.acc = s_ddot * (trajEnd_-trajInit_);
    }

    else if((trajRadius_ != 0 && accDuration_ != 0) || (trajRadius_ != 0 && accDuration_ == 0)) //Circular Trajectory
    {
        if(trajRadius_ != 0 && accDuration_ != 0)
        {
            trapezoidal_vel(time, accDuration_, s, s_dot, s_ddot);
        }
        else
        {
            cubic_polinomial(time, s, s_dot, s_ddot);
        }
        //Circular Trajectory with the Center of the Trajectory in the Vertical Plane Containing the End-Effector
        //The center of the Trajectory is given by adding to the y-component of the initial point the radius

        //Position
        traj.pos.x() = trajInit_.x();
        traj.pos.y() = trajInit_.y() + trajRadius_ - trajRadius_*cos(2*M_PI*s);
        traj.pos.z() = trajInit_.z() - trajRadius_*sin(2*M_PI*s);

        //Velocity
        traj.vel.x() = 0;
        traj.vel.y() = 2*M_PI*trajRadius_*s_dot*sin(2*M_PI*s);
        traj.vel.z() = -2*M_PI*trajRadius_*s_dot*cos(2*M_PI*s);

        //Acceleration
        traj.acc.x() = 0;
        traj.acc.y() = 2*M_PI*trajRadius_*s_ddot*sin(2*M_PI*s) + 4*std::pow(M_PI,2)*trajRadius_*std::pow(s_dot,2)*cos(2*M_PI*s);
        traj.acc.z() = -2*M_PI*trajRadius_*s_ddot*cos(2*M_PI*s) + 4*std::pow(M_PI,2)*trajRadius_*std::pow(s_dot,2)*sin(2*M_PI*s);
    }
    else
    {
        std::cout<< "The trajectory is not well defined. Modify planner_ in ros2_kdl_node.cpp and try again.";
    }
 
    return traj;

}

/*
int KDLPlanner::TrajectorySelection(double _accDuration, double _trajRadius)
{
    int index = 0;
    accDuration_ = _accDuration;
    trajRadius_ = _trajRadius;

    if(trajRadius_ == 0 && accDuration_ != 0)
    {
        index = 1;
        std::cout<< "The trajectory chosen is: Linear Trajectory with Trapezoidal Velocity Profile";

    }
    else if(trajRadius_ == 0 && accDuration_ == 0)
    {
        index = 2;
        std::cout<< "The trajectory chosen is: Linear Trajectory with Cubic Polynomial";
    }
    else if(trajRadius_ != 0 && accDuration_ != 0)
    {
        index = 3;
        std::cout<< "The trajectory chosen is: Circular Trajectory with Trapezoidal Velocity Profile";
    }
    else if (trajRadius_ != 0 && accDuration_ == 0)
    {
        index = 4;
        std::cout<< "The trajectory chosen is: Circular Trajectory with Cubic Polynomial";
    }

    return index;
}
*/