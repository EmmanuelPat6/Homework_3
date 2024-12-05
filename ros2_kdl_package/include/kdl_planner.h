#ifndef KDLPlanner_H
#define KDLPlanner_H

#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>
#include "Eigen/Dense"
#include <cmath>

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

//Curvilinear Abscissa
/*
struct curvilinear_abscissa{
    double s;
    double sdot;
    double sddot;
}
*/

class KDLPlanner
{

public:

    KDLPlanner();
    KDLPlanner(double _maxVel, double _maxAcc);
    void CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                    double _radius, double _eqRadius);
    void createCircPath(KDL::Frame &_F_start,
                        KDL::Vector &_V_centre,
                        KDL::Vector &_V_base_p,
                        KDL::Rotation &_R_base_end,
                        double alpha,
                        double eqradius);

    KDL::Trajectory* getTrajectory();

    //////////////// CONSTRUCTORS /////////////////

    //Constructor for Linear Trajectory with Trapezoidal Velocity Profile 
    // _trajDuration = Final Time
    // _accDuration = Acceleration Time
    // _trajInit = Trajectory Starting Point
    // _trajEnd = Trajectory Final Point
    KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);

    //Constructor for Linear Trajectory with Cubic Polynomial 
    // _trajDuration = Final Time
    // _trajInit = Trajectory Starting Point
    // _trajEnd = Trajectory Final Point
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd);

    //Constructor for Circular Trajectory with Cubic Polynomial
    // _trajDuration = Final Time
    // _trajInit = Trajectory Starting Point
    // _trajRadius = Circumference Radius
    KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius);


    //Constructor for Circular Trajectory with Trapezoidal Velocity Profile
    // _trajDuration = Final Time
    // _accDuration = Acceleration Time
    // _trajInit = Trajectory Starting Point
    // _trajRadius = Circumference Radius
    KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius);



/////////////////////////////////////////////////////////////////////////
    trajectory_point compute_trajectory(double time);

    //Trapezoidal Velocity
    void trapezoidal_vel(double t, double tc, double &s, double &s_dot, double &s_ddot);

    //Cubic Polynomial
    void cubic_polinomial(double t, double &s, double &s_dot, double &s_ddot);
  

    //int TrajectorySelection(double _accDuration, double _trajRadius);

private:

    KDL::Path_RoundedComposite* path_;
    KDL::Path_Circle* path_circle_;
	KDL::VelocityProfile* velpref_;
	KDL::Trajectory* traject_;

    //////////////////////////////////
    double trajDuration_, accDuration_, trajRadius_;
    Eigen::Vector3d trajInit_, trajEnd_;
    trajectory_point p;

};

#endif
