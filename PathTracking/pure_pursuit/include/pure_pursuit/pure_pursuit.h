// Copyright 2022 Aytac Kahveci
#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <limits>
#include <plotting/plotting.h>

/**
 * @brief Transform angle between pi and -pi
 * 
 * @param angle angle to be transformed
 * @return double angle between pi and -pi
 */
extern double pi2pi(double angle);

/**
 * @brief Vehicle state structure
 * 
 */
struct State {
  State() {}

  /**
   * @brief Construct a new State object
   * 
   * @param swb wheelbase
   * @param sx x pos
   * @param sy y pos
   * @param syaw yaw angle
   * @param sv velocity
   */
  State(double swb, double sx = 0.0, double sy = 0.0,
        double syaw = 0.0, double sv = 0.0)
      : x(sx), y(sy), yaw(syaw), v(sv), wb(swb) {
    rear_x = x - (wb / 2.0 * cos(yaw));
    rear_y = y - (wb / 2.0 * sin(yaw));
  }

  /**
   * @brief Update state
   * 
   * @param a acceleration
   * @param delta steering angle
   * @param dt period
   */
  void update(const double& a, const double& delta, const double& dt) {
    x += v * cos(yaw) * dt;
    y += v * sin(yaw) * dt;
    yaw += v / wb * tan(delta) * dt;
    yaw = pi2pi(yaw);
    v += a * dt;
    rear_x = x - ((wb / 2.0) * cos(yaw));
    rear_y = y - ((wb / 2.0) * sin(yaw));
  }

  /**
   * @brief Calculate distance from current position to the given position
   * 
   * @param point_x x coordinate
   * @param point_y y coordinate
   * @return double calculated distance
   */
  double calcDistance(const double& point_x, const double& point_y) {
    return hypotf(x - point_x, y - point_y);
  }

  double x, y, yaw, v;
  double wb;
  double rear_x, rear_y;
};


class PurePursuit {
 public:
  PurePursuit();

  ~PurePursuit();

  /**
   * @brief Search target index
   * 
   * @param rx ref path x positions
   * @param ry ref path y positions
   * @param ryaw ref path yaw angles
   * @param target_ind found target index
   */
  void searchTargetIndex(const std::vector<double>& rx,
                         const std::vector<double>& ry,
                         const std::vector<double>& ryaw,
                         int* target_ind);

  /**
   * @brief Calculate pure pursuit control command
   * 
   * @param lx lookahead point x position
   * @param ly lookahead point y position
   * @param delta calculated steering angle
   */
  void purePursuitControl(const double& lx, const double& ly, double* delta);

  /**
   * @brief Do simulation
   * 
   * @param rx reference path x positions
   * @param ry reference path y positions
   * @param ryaw reference path yaw angles
   * @param rk reference path curvature
   * @param dl path resolution
   * @param target_speed target speed
   * @param initial_state initial state
   * @param plotting plotting interface pointer
  */
  void doSimulation(const std::vector<double>& rx,
                    const std::vector<double>& ry,
                    const std::vector<double>& ryaw,
                    const std::vector<double>& rk,
                    const double& dl,
                    const double& target_speed,
                    const State& initial_state,
                    void* plotting);

 private:
  State state_;  //!< Current vehicle state
  double wb_;    //!< Vehicle wheelbase
  double lf_;    //!< Lookahead distance
  double lfc_;   //!< Lookahead distance
  double k_;     //!< look forward gain
  double kp_;    //!< speed proportional gain
  double dt_;    //!< period

  int last_ind_;
};
