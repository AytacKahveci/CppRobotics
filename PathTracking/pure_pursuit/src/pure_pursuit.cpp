// Copyright 2022 Aytac Kahveci
#include <pure_pursuit/pure_pursuit.h>


double pi2pi(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

PurePursuit::PurePursuit() {
  wb_ = 2.9;  //!< Vehicle wheelbase
  lf_ = 2.0;  //!< Lookahead distance
  lfc_ = 2.0;  //!< Lookahead distance
  k_ = 0.1;  //!< look forward gain
  kp_ = 1.0;  //!< speed proportional gain
  dt_ = 0.1;  //!< period

  last_ind_ = 0;
}

PurePursuit::~PurePursuit() {}

void PurePursuit::searchTargetIndex(
    const std::vector<double>& rx,
    const std::vector<double>& ry,
    const std::vector<double>& ryaw,
    int* target_ind) {
  // Calculate nearest point in the path
  int nearest_ind;
  double mind = std::numeric_limits<double>::max();
  for (int i = 0; i < rx.size(); ++i) {
    double dist = hypotf(state_.x - rx[i], state_.y - ry[i]);
    if (dist < mind) {
      nearest_ind = i;
      last_ind_ = nearest_ind;
      mind = dist;
    }
  }

  double lf = k_ * state_.v + lfc_;  // update lookahead distance
  bool found = false;
  for (int i = nearest_ind; i < rx.size(); ++i) {
    double d = hypotf(rx[nearest_ind] - rx[i], ry[nearest_ind] - ry[i]);
    if (d >= lf) {
      *target_ind = i;
      found = true;
      break;
    }
  }

  if (found == false) {
    *target_ind = rx.size() - 1;
  }
}

void PurePursuit::purePursuitControl(
    const double& lx,
    const double& ly,
    double* delta) {
  double alpha = atan2(ly - state_.rear_y, lx - state_.rear_x) - state_.yaw;

  *delta = atan2(2.0 * wb_ * sin(alpha) / lf_, 1.0);
}

void PurePursuit::doSimulation(
    const std::vector<double>& rx,
    const std::vector<double>& ry,
    const std::vector<double>& ryaw,
    const std::vector<double>& rk,
    const double& dl,
    const double& target_speed,
    const State& initial_state,
    void* plotting) {
  double T = 100.0;  // max sim time
  double time = 0.0;
  double dt = 0.1;
  int target_ind = 0;
  state_ = initial_state;
  std::vector<double> cx;
  std::vector<double> cy;
  while (T >= time && last_ind_ < rx.size()) {
    searchTargetIndex(rx, ry, ryaw, &target_ind);

    double lx = rx[target_ind];
    double ly = ry[target_ind];
    double di;
    purePursuitControl(lx, ly, &di);
    double ai = kp_ * (target_speed - state_.v);
    /* if (ai > 0.5)
      ai = 0.5;
    else if (ai < -0.5)
      ai = -0.5; */

    if (plotting != nullptr) {
      cx.push_back(state_.x);
      cy.push_back(state_.y);

      ((Plotting*)plotting)->plot(rx, ry, cx, cy);
    }

    std::cout << "Target: " << std::endl;
    std::cout << "X: " << lx
              << " Y: " << ly << std::endl;
    std::cout << "State: " << std::endl;
    std::cout << "X: " << state_.x
              << " Y: " << state_.y
              << " Yaw: " << state_.yaw << std::endl;
    std::cout << "Control: " << std::endl;
    std::cout << "ai: " << ai
              << " di: " << di << std::endl;

    state_.update(ai, di, dt);

    time += dt;
    usleep(100000);
  }
}
