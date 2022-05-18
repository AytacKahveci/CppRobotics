// Copyright 2022 Aytac Kahveci
#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>

/**
 * @brief Cubic Spline class
 * */
class Spline {
 public:
  Spline() {}

  Spline(const std::vector<double>& x, const std::vector<double>& y);

  ~Spline();

  /**
   * @brief Calc position
   * 
   * @param pos position output
   * @param t time which the position is calculated at
  */
  void calc(double* pos, const double& t);

  /**
   * @brief Calc velocity
   * 
   * @param vel velocity output
   * @param t time which the velocity is calculated at
  */
  void calcd(double* vel, const double& t);

  /**
   * @brief Calc acceleration
   * 
   * @param acc acceleration output
   * @param t time which the acceleration is calculated at
  */
  void calcdd(double* acc, const double& t);

  /**
   * @brief Get cubic spline coefficients
  */
  void getCoeffs(std::vector<double>* a, std::vector<double>* b,
                 std::vector<double>* c, std::vector<double>* d);

 private:
  std::vector<double> x_;
  std::vector<double> y_;
  size_t nx_;

  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;

 private:
  /**
   * @brief Search data segment index
   * 
   * @param t time to be searched
   * @return index
  */
  int searchIndex(const double& t);

  /**
   * @brief Calc A matrix for spline coefficient
   * 
   * @param h x[i+1] - x[i]
   * @param A A matrix
  */
  void calcA(const std::vector<double>& h, Eigen::MatrixXd* A);

  /**
   * @brief Calc B vector for spline coefficient
   * 
   * @param h x[i+1] - x[i]
   * @param B B vector
  */
  void calcB(const std::vector<double>& h, Eigen::VectorXd* B);
};


/**
 * @brief 2D Cubic Spline class
*/
class Spline2D {
 public:
  Spline2D() {}

  Spline2D(const std::vector<double>& x, const std::vector<double>& y);

  ~Spline2D() {}

  /**
   * @brief Calculate segment lengths
  */
  void calcS(const std::vector<double>& x,
             const std::vector<double>& y,
             std::vector<double>* s);

  /**
   * @brief Calculate position
   * 
   * @param[in] s segment length
   * @param[out] x calculated x position
   * @param[out] y calculated y position
  */
  void calcPos(const double& s, double* x, double* y);

  /**
   * @brief Calculate curvature
   * 
   * @param[in] s segment length
   * @param[out] curvature calculated curvature
  */
  void calcCurvature(const double& s, double* curvature);

  /**
   * @brief Calculate yaw
   * 
   * @param[in] s segment length
   * @param[out] yaw calculated yaw
  */
  void calcYaw(const double& s, double* yaw);

  /**
   * @brief Calculate spline course
   * 
   * @param[in] ds path resolution
   * @param[out] rx x position array
   * @param[out] ry y position array
   * @param[out] ryaw yaw angle array
   * @param[out] rk curvature array
  */
  void calcSplineCourse(const double& ds,
                        std::vector<double>* rx,
                        std::vector<double>* ry,
                        std::vector<double>* ryaw,
                        std::vector<double>* rk);

 private:
  Spline sx_, sy_;

 public:
  std::vector<double> s_;
};
