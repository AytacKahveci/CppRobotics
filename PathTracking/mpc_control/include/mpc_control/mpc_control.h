// Copyright 2022 Aytac Kahveci
#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <limits.h>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <plotting/plotting.h>

#define DEG2RAD(deg) (deg * M_PI / 180.0);

extern double pi2pi(double angle);

/**
 * @brief Robot state
 * 
*/
struct State {
  State() {}

  State(double sx, double sy, double syaw, double sv, double sdelta)
    : x(sx), y(sy), yaw(syaw), v(sv), delta(sdelta) { }

  double x;  //!< X position
  double y;  //!< Y position
  double yaw;  //!< Yaw angle
  double v;  //!< Velocity
  double delta;  //!< Wheel steering angle
};


/**
 * @brief MPC parameters
*/
struct MPCParam {
  int nx = 4;  //!< x = x, y, v, yaw
  int nu = 2;  //!< a = accel, steer
  int T = 5;  //!< horizon length

  Eigen::MatrixXd R;  //!< input cost matrix
  Eigen::MatrixXd Rd;  //!< input difference cost matrix
  Eigen::MatrixXd Q;  //!< state cost matrix
  Eigen::MatrixXd Qf;  //!< state final matrix

  double goal_distance = 1.5;  //!< goal distance
  double stop_speed = 0.5 / 3.6;  //!< stop speed

  int max_iter = 20;  //!< max iteration
  double du_th = 0.1;  //!< iteration finish param
  double dt = 0.2;  //!< control period

  // Vehicle parameters
  double length = 4.5;
  double width = 2.0;
  double backtowheel = 1.0;
  double wheel_len = 0.3;
  double wheel_width = 0.2;
  double tread = 0.7;
  double wb = 2.5;

  double max_steer = DEG2RAD(45.0);  //!< maximum steering angle [rad]
  double max_dsteer = DEG2RAD(30.0);  //!< maximum steering speed [rad/s]
  double max_speed = 0.2;  //!< maximum speed [m/s]
  double min_speed = -0.2;  //!< minimum speed [m/s]
  double max_accel = 1.0;  //!< maximum acceleration
};


/**
 * @brief MPC Controller class
 * 
*/
class MPCControl {
 public:
  MPCControl() {}

  ~MPCControl() {}

  /**
   * @brief Update robot state
   * 
   * @param[in] a acceleration
   * @param[in] delta steering angle
  */
  void updateState(const double& a, const double& delta);

  /**
   * @brief Get Linear model
   * 
   * @param[in] v robot velocity
   * @param[in] phi robot yaw angle
   * @param[in] delta robot steering angle
   * @param[out] A linear model matrix
   * @param[out] B linear model matrix
   * @param[out] C linear model matrix
  */
  void getLinearModelMatrix(const double& v,
                            const double& phi,
                            const double& delta,
                            Eigen::MatrixXd* A,
                            Eigen::MatrixXd* B,
                            Eigen::VectorXd* C);

  /**
   * @brief Calculate nearest index to the path from the current state
   * 
   * @param[in] rx reference path x positions
   * @param[in] ry reference path y positions
   * @param[in] ryaw reference path yaw angles
   * @param[in] pind previous path index
   * @param[out] ind nearest path index
   * @param[out] mind nearest path distance
  */
  void calcNearestIndex(const std::vector<double>& rx,
                        const std::vector<double>& ry,
                        const std::vector<double>& ryaw,
                        const int& pind,
                        int* ind,
                        double* mind);

  /**
   * @brief Calculate reference trajectory
   * 
   * @param[in] rx reference path x positions
   * @param[in] ry reference path y positions
   * @param[in] ryaw reference path yaw angles
   * @param[in] rk reference path curvature
   * @param[in] sp speed profile
   * @param[in] dl path resolution
   * @param[in] pind previous path index
   * @param[out] xref reference matrix
   * @param[out] ind path index
   * @param[out] uref control ref matrix
  */
  void calcRefTrajectory(const std::vector<double>& rx,
                         const std::vector<double>& ry,
                         const std::vector<double>& ryaw,
                         const std::vector<double>& rk,
                         const std::vector<double>& sp,
                         const double& dl,
                         const int& pind,
                         Eigen::MatrixXd* xref,
                         int* ind,
                         Eigen::MatrixXd* uref);

  /**
   * @brief Calculate speed profile
   * 
   * @param rx reference path x positions
   * @param ry reference path y positions
   * @param ryaw reference path yaw angles
   * @param target_speed target speed
   * @param sp speed profile
   */
  void calcSpeedProfile(const std::vector<double>& rx,
                        const std::vector<double>& ry,
                        const std::vector<double>& ryaw,
                        const double& target_speed,
                        std::vector<double>* sp);

  /**
   * @brief Check the goal if it is reached
   * 
   * @param goal goal array (x, y)
   * @param tind target index
   * @param nind total index
   * @return true if the goal is reached
   * @return false otherwise
   */
  bool checkGoal(const std::vector<double>& goal,
                 const int& tind,
                 const int& nind);

  /**
   * @brief Calculate MPC Control
   * 
   * @param xref reference matrix
   * @param dref control ref matrix
   * @param res result control array
   * @return true if solution is converged
   * @return false if solution is not converged
   */
  bool linearMPCControl(const Eigen::MatrixXd& xref,
                        Eigen::MatrixXd* uref,
                        double* res);

  /**
   * @brief Do simulation
   * 
   * @param[in] rx reference path x positions
   * @param[in] ry reference path y positions
   * @param[in] ryaw reference path yaw angles
   * @param[in] rk reference path curvature
   * @param[in] sp speed profile
   * @param[in] dl path resolution
   * @param[in] initial_state initial state
   * @param[in] plotting plotting interface pointer
  */
  void doSimulation(const std::vector<double>& rx,
                    const std::vector<double>& ry,
                    const std::vector<double>& ryaw,
                    const std::vector<double>& rk,
                    const std::vector<double>& sp,
                    const double& dl,
                    const State& initial_state,
                    void* plotting);

  /**
   * @brief Smooth yaw angles vector
   * 
   * @param yaw yaw angles vector to be processed
   */
  void smoothYaw(std::vector<double>* yaw);

 private:
  double dt_;  //!< period
  State state_;  //!< current robot state
  MPCParam params_;  //!< mpc parameters

  qpOASES::QProblem qproblem_;
  bool solver_init_;
};





