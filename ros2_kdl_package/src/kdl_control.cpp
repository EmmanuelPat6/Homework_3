#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() /*+ robot_->getGravity()*/ /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

    // Compute Gain Matrices
    //_Kpp for position; _Kpo for orientation; _Kdp for velocity; _Kdo for ang_velocity
    //Angle-Axis Representation in such a way to use the Geometric Jacobian instead of the Analytical Jacobian
    Eigen::Matrix<double,6,6> Kp, Kd;
    Kp=Eigen::MatrixXd::Zero(6,6);
    Kd=Eigen::MatrixXd::Zero(6,6);
    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();

    // Read the Current State
    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;  //Jacobian
    Eigen::Matrix<double,7,7> I = Eigen::Matrix<double,7,7>::Identity();    //Identity Matrix
    Eigen::Matrix<double,7,7> B = robot_->getJsim();    //Inertia Matrix
    //Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);
    Eigen::Matrix<double,7,6> Jpinv = weightedPseudoInverse(B,J);
    /*
    The weighted pseudo-inverse allows calculating a least-squares solution that accounts
    for the robot's inertia. In other words, the resulting solution minimizes the robot's kinetic energy.
    */

    //Position
    Eigen::Vector3d pd(_desPos.p.data);
    Eigen::Vector3d pe(robot_->getEEFrame().p.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
    R_d = matrixOrthonormalization(R_d);
    R_e = matrixOrthonormalization(R_e);

    //Velocity
    Eigen::Vector3d dot_pd(_desVel.vel.data);
    Eigen::Vector3d dot_pe(robot_->getEEVelocity().vel.data);
    Eigen::Vector3d omega_d(_desVel.rot.data);
    Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);

    //Acceleration
    Eigen::Matrix<double,6,1> dotdot_xd;
    Eigen::Matrix<double,3,1> dotdot_pd(_desAcc.vel.data);
    Eigen::Matrix<double,3,1> dotdot_rd(_desAcc.rot.data);

    //Linear Errors
    Eigen::Matrix<double,3,1> e_p = computeLinearError(pd,pe);
    Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_pd,dot_pe);

    //Orientation Errors
    Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
    Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,omega_e,R_d,R_e);

    Eigen::Matrix<double,6,1> x_tilde;
    Eigen::Matrix<double,6,1> dotx_tilde;
    x_tilde << e_p, e_o;
    dotx_tilde << dot_e_p, dot_e_o;
    dotdot_xd << dotdot_pd, dotdot_rd;

/*
    //Analitycal Jacobian                                     
    Eigen::Matrix<double,3,1> euler_angles = computeEulerAngles(R_e);                       
    Eigen::Matrix<double,6,7> JA = AnalitycalJacobian(J,euler_angles);
    Eigen::Matrix<double,7,6> JApinv = weightedPseudoInverse(B,JA); 

    // Compute TA_inv
    Eigen::Matrix<double,6,6> TA;                               
    TA.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    TA.block(3,3,3,3) = computeT(euler_angles);
    Eigen::Matrix<double,6,6> TA_inv = TA.inverse();

    // Compute TA_dot
    Eigen::Matrix<double,6,6> TA_dot;
    TA_dot.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    TA_dot.block(3,3,3,3) = computeTdot(euler_angles);
    Eigen::Matrix<double,6,7> Jdot = robot_->getEEJacDot().data; 
    // Compute JA_dot 
    Eigen::Matrix<double,6,7> JAdot = TA_inv*(Jdot - TA_dot*JA); 
*/

   //Inverse Dynamics in the Operational Space

    Eigen::Matrix<double,6,1> y;
    
    Eigen::VectorXd Jdot_qdot=robot_->getEEJacDotqDot();
    
    //y << Jpinv * (dotdot_xd + Kd*dotx_tilde + Kp*x_tilde - Jdot_qdot);

    y << dotdot_xd - Jdot_qdot + Kd*dotx_tilde + Kp*x_tilde;

    return B * (Jpinv*y + (I-Jpinv*J)*(- 1*robot_->getJntVelocities()))
            /*+ robot_->getGravity()*/ + robot_->getCoriolis();

    //return B*y + /*robot_->getGravity()*/ + robot_->getCoriolis();


}

