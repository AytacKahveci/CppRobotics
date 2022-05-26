// Copyright 2022 Aytac Kahveci
#include <mpc_control/mpc_control.h>

double pi2pi(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

void MPCControl::updateState(const double& a, const double& delta) {
  // input check
  if (delta >= params_.max_steer) {
    state_.delta = params_.max_steer;
  } else if (delta <= -params_.max_steer) {
    state_.delta = -params_.max_steer;
  }

  state_.x += state_.v * cos(state_.yaw) * params_.dt;
  state_.y += state_.v * sin(state_.yaw) * params_.dt;
  state_.yaw += state_.v / params_.wb * tan(delta) * params_.dt;
  state_.v += a * params_.dt;

  if (state_.v > params_.max_speed) {
    state_.v = params_.max_speed;
  } else if (state_.v < params_.min_speed) {
    state_.v = params_.min_speed;
  }
}

void MPCControl::getLinearModelMatrix(const double& v,
                                      const double& phi,
                                      const double& delta,
                                      Eigen::MatrixXd* A,
                                      Eigen::MatrixXd* B,
                                      Eigen::VectorXd* C) {
  (*A) = Eigen::MatrixXd::Zero(params_.nx, params_.nx);
  (*A)(0, 0) = 1.0;
  (*A)(1, 1) = 1.0;
  (*A)(2, 2) = 1.0;
  (*A)(3, 3) = 1.0;
  (*A)(0, 2) = params_.dt * cos(phi);
  (*A)(0, 3) = -params_.dt * v * sin(phi);
  (*A)(1, 2) = params_.dt * sin(phi);
  (*A)(1, 3) = params_.dt * v * cos(phi);
  (*A)(3, 2) = params_.dt * tan(delta) / params_.wb;

  (*B) = Eigen::MatrixXd::Zero(params_.nx, params_.nu);
  (*B)(2, 0) = params_.dt;
  (*B)(3, 1) = params_.dt * v / (params_.wb * cos(delta) * cos(delta));

  (*C) = Eigen::VectorXd::Zero(params_.nx);
  (*C)(0) = params_.dt * v * sin(phi) * phi;
  (*C)(1) = -params_.dt * v * cos(phi) * phi;
  (*C)(3) = -params_.dt * v * delta / (params_.wb * cos(delta) * cos(delta));
}


void MPCControl::calcNearestIndex(const std::vector<double>& rx,
                                  const std::vector<double>& ry,
                                  const std::vector<double>& ryaw,
                                  const int& pind,
                                  int* ind,
                                  double* mind) {
  *mind = std::numeric_limits<double>::max();
  for (size_t i = 0; i < rx.size(); ++i) {
    double dist = hypotf(state_.x - rx[i], state_.y - ry[i]);
    if (dist < *mind) {
      *ind = i;
      *mind = dist;
    }
  }
  double angle = pi2pi(ryaw[*ind] - atan2(ry[*ind] - state_.y,
                                          rx[*ind] - state_.x));
  if (angle < 0.0) {
    *mind *= -1;
  }
}

void MPCControl::calcRefTrajectory(const std::vector<double>& rx,
                                   const std::vector<double>& ry,
                                   const std::vector<double>& ryaw,
                                   const std::vector<double>& rk,
                                   const std::vector<double>& sp,
                                   const double& dl,
                                   const int& pind,
                                   Eigen::MatrixXd* xref,
                                   int* ind,
                                   Eigen::MatrixXd* uref) {
  (*xref) = Eigen::MatrixXd::Zero(params_.T * params_.nx, 1);
  (*uref) = Eigen::MatrixXd::Zero(params_.T * params_.nu, 1);

  int ncourse = rx.size();
  double mind;
  calcNearestIndex(rx, ry, ryaw, pind, ind, &mind);
  if (pind >= (*ind))
    *ind = pind;

  (*xref)(0, 0) = rx[(*ind)];
  (*xref)(1, 0) = ry[(*ind)];
  (*xref)(2, 0) = sp[(*ind)];
  (*xref)(3, 0) = ryaw[(*ind)];
  double dir = 1.0;
  if (sp[(*ind)] < 0.0)
    dir = -1.0;
  (*uref)(0, 0) = dir * 0.1;  // acceleration
  (*uref)(1, 0) = 0.0;  // steer operational point should be 0

  double travel = 0.0;
  int dind = 0;
  for (size_t i = 0; i < params_.T; ++i) {
    travel += fabs(state_.v) * params_.dt;
    // int dind = static_cast<int>(round(travel / dl));
    dind++;
    if (((*ind) + dind) < ncourse) {
      (*xref)(i * params_.nx, 0) = rx[(*ind) + dind];
      (*xref)(i * params_.nx + 1, 0) = ry[(*ind) + dind];
      (*xref)(i * params_.nx + 2, 0) = sp[(*ind) + dind];
      (*xref)(i * params_.nx + 3, 0) = ryaw[(*ind) + dind];
      double dir = 1.0;
      if (sp[(*ind) + dind] < 0.0)
        dir = -1.0;
      (*uref)(i * params_.nu, 0) = dir * 0.1;
      (*uref)(i * params_.nu + 1, 0) = 0.0;
    } else {
      (*xref)(i * params_.nx, 0) = rx[ncourse - 1];
      (*xref)(i * params_.nx + 1, 0) = ry[ncourse - 1];
      (*xref)(i * params_.nx + 2, 0) = sp[ncourse - 1];
      (*xref)(i * params_.nx + 3, 0) = ryaw[ncourse - 1];
      double dir = 1.0;
      if (sp[ncourse - 1] < 0.0)
        dir = -1.0;
      (*uref)(i * params_.nu, 0) = dir * 0.1;
      (*uref)(i * params_.nu + 1, 0) = 0.0;
    }
  }
}

void MPCControl::calcSpeedProfile(const std::vector<double>& rx,
                                  const std::vector<double>& ry,
                                  const std::vector<double>& ryaw,
                                  const double& target_speed,
                                  std::vector<double>* sp) {
  sp->resize(rx.size(), target_speed);
  double direction = 1.0;

  for (int i = 0; i < rx.size() - 1; ++i) {
    double dx = rx[i+1] - rx[i];
    double dy = ry[i+1] - ry[i];
    double move_dir = atan2(dy, dx);

    if (dx != 0.0 && dy != 0.0) {
      double dangle = fabs(pi2pi(move_dir - ryaw[i]));
      if (dangle >= M_PI_4)
        direction = -1.0;
      else
        direction = 1.0;

      if (direction != 1.0)
        sp->at(i) = -target_speed;
      else
        sp->at(i) = target_speed;
    }
  }

  sp->back() = 0.0;
}


bool MPCControl::checkGoal(const std::vector<double>& goal,
                           const int& tind,
                           const int& nind) {
  double dist = hypotf(state_.x - goal[0], state_.y - goal[1]);

  bool isgoal = dist <= params_.goal_distance;
  if (abs(tind - nind) >= 5)
    isgoal = false;

  bool isstop = (fabs(state_.v) <= params_.stop_speed);

  if (isgoal && isstop)
    return true;

  return false;
}

bool MPCControl::linearMPCControl(const Eigen::MatrixXd& xref,
                                  Eigen::MatrixXd* uref,
                                  double* res) {
  Eigen::MatrixXd Psi =
      Eigen::MatrixXd::Zero(params_.nx * params_.T, params_.nx);
  Eigen::MatrixXd Omega =
      Eigen::MatrixXd::Zero(params_.nx * params_.T, params_.nu);
  Eigen::MatrixXd Theta =
      Eigen::MatrixXd::Zero(params_.nx * params_.T, params_.nu * params_.T);

  Eigen::MatrixXd Q =
      Eigen::MatrixXd::Identity(params_.nx * params_.T, params_.nx * params_.T);
  Eigen::MatrixXd R =
      Eigen::MatrixXd::Identity(params_.nu * params_.T, params_.nu * params_.T);

  Eigen::MatrixXd Qq = Eigen::MatrixXd::Identity(params_.nx, params_.nx);
  Qq(0, 0) = 1.0;
  Qq(1, 1) = 1.0;
  Qq(2, 2) = 0.5;
  Qq(3, 3) = 1.0;
  Eigen::MatrixXd Rq = Eigen::MatrixXd::Identity(params_.nu, params_.nu);
  Rq(0, 0) = 0.1;
  Rq(1, 1) = 0.1;


  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(params_.nx, params_.nx);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(params_.nx, params_.nu);
  Eigen::VectorXd CC = Eigen::VectorXd::Zero(params_.nx);
  getLinearModelMatrix(state_.v, state_.yaw, state_.delta, &A, &B, &CC);
  Eigen::MatrixXd C = Eigen::MatrixXd::Ones(params_.nx, params_.nx);
  /* C(0, 0) = 0.0;
  C(1, 1) = 0.0;
  C(2, 2) = 1.0;
  C(3, 3) = 1.0;
 */
  for (int i = 0; i < params_.T; ++i) {
    int idx_x_i = i * params_.nx;
    int idx_x_i_prev = (i - 1) * params_.nx;

    Q.block(idx_x_i, idx_x_i, params_.nx, params_.nx) = Qq;
    R.block(i * params_.nu, i * params_.nu, params_.nu, params_.nu) = Rq;

    if (i == 0) {
      Psi.block(0, 0, params_.nx, params_.nx) = A;
      Omega.block(0, 0, params_.nx, params_.nu) = B;
      Theta.block(0, 0, params_.nx, params_.nu) = B;
      Q.block(0, 0, params_.nx, params_.nx) = Qq;
      R.block(0, 0, params_.nu, params_.nu) = Rq;
    } else {
      Psi.block(idx_x_i, 0, params_.nx, params_.nx) =
          A * Psi.block(idx_x_i_prev, 0, params_.nx, params_.nx);
      Omega.block(idx_x_i, 0, params_.nx, params_.nu) =
          Omega.block(idx_x_i_prev, 0, params_.nx, params_.nu)
          + Psi.block(idx_x_i - 1, 0, params_.nx, params_.nx) * B;

      for (int j = 0; j <= i; ++j) {
        int idx_u_j = j * params_.nu;
        Theta.block(idx_x_i, idx_u_j, params_.nx, params_.nu) =
            Omega.block((i - j) * params_.nx, 0, params_.nx, params_.nu);
      }
      if (i <= params_.nu) {
        int idx_u_j = i * params_.nu;
        Theta.block(idx_x_i, idx_u_j, params_.nx, params_.nu) = B;
      }
    }
  }

  /* std::cout << "A:" << std::endl;
  std::cout << A << std::endl;
  std::cout << "B:" << std::endl;
  std::cout << B << std::endl;
  std::cout << "C:" << std::endl;
  std::cout << C << std::endl;
  std::cout << "Psi:" << std::endl;
  std::cout << Psi << std::endl;
  std::cout << "Omega:" << std::endl;
  std::cout << Omega << std::endl;
  std::cout << "Theta:" << std::endl;
  std::cout << Theta << std::endl;
  std::cout << "Q:" << std::endl;
  std::cout << Q << std::endl;
  std::cout << "R:" << std::endl;
  std::cout << R << std::endl; */

  Eigen::MatrixXd Cex =
      Eigen::MatrixXd::Zero(1, params_.nx * params_.T);
  std::cout << "Cex:" << std::endl;
  std::cout << Cex << std::endl;
  Eigen::MatrixXd CPsi =
      Eigen::MatrixXd::Zero(params_.T * params_.nx, params_.nx);

  Eigen::MatrixXd COmega =
      Eigen::MatrixXd::Zero(params_.T * params_.nx, params_.nu);
  Eigen::MatrixXd CTheta =
      Eigen::MatrixXd::Zero(params_.T * params_.nx, params_.T * params_.nu);
  for (int i = 0; i < params_.T; ++i) {
    int idx_x_i = i * params_.nx;

    if (i == 0) {
      CPsi.block(0, 0, params_.nx, params_.nx) = C*A;
      COmega.block(0, 0, params_.nx, params_.nu) = C*B;
      CTheta.block(0, 0, params_.nx, params_.nu) = C*B;
    } else {
      CPsi.block(idx_x_i, 0, params_.nx, params_.nx) =
          C * Psi.block(idx_x_i, 0, params_.nx, params_.nx);
      COmega.block(idx_x_i, 0, params_.nx, params_.nu) =
          C * Omega.block(idx_x_i, 0, params_.nx, params_.nu);

      for (int j = 0; j <= i; ++j) {
        int idx_u_j = j * params_.nu;
        CTheta.block(idx_x_i, idx_u_j, params_.nx, params_.nu) =
            COmega.block((i - j) * params_.nx, 0, params_.nx, params_.nu);
      }
      /* if (i <= params_.nu) {
        int idx_u_j = i * params_.nu;
        CTheta.block(idx_x_i, idx_u_j, params_.nx, params_.nu) = C*B;
      } */
    }
  }

  /* std::cout << "Cex:" << std::endl;
  std::cout << Cex << std::endl;
  std::cout << "CPsi:" << std::endl;
  std::cout << CPsi << std::endl;
  std::cout << "COmega:" << std::endl;
  std::cout << COmega << std::endl;
  std::cout << "CTheta:" << std::endl;
  std::cout << CTheta << std::endl; */

  Eigen::MatrixXd Sp = xref;
  Eigen::MatrixXd Z;
  Eigen::VectorXd x(params_.nx);
  x << state_.x, state_.y, state_.v, state_.yaw;
  Eigen::VectorXd u(params_.nu);
  double acceleration, delta;
  u << (*uref)(0, 0), (*uref)(1, 0);

  Eigen::MatrixXd Err = Sp - CPsi * x - COmega * u;
  std::cout << "Err: " << std::endl;
  std::cout << Err << std::endl;
  Eigen::MatrixXd g = 2.0 * (CTheta.transpose()) * Q * Err;
  Eigen::MatrixXd H = (CTheta.transpose()) * Q * CTheta + R;

  Eigen::MatrixXd H_inv = H.inverse();
  Eigen::MatrixXd DU = H_inv * g * 0.5;
  for (int i = 0; i < params_.T; ++i) {
    if (DU(i * params_.nu, 0) > 0.1) {
      DU(i * params_.nu, 0) = 0.1;
    } else if (DU(i * params_.nu, 0) < -0.1) {
      DU(i * params_.nu, 0) = -0.1;
    }

    DU(i * params_.nu + 1, 0) = pi2pi(DU(i * params_.nu + 1, 0));
  }

  for (int i = 0; i < params_.T; ++i) {
    (*uref)(i * params_.nu, 0) += DU(i * params_.nu, 0);
    (*uref)(i * params_.nu + 1, 0) += DU(i * params_.nu + 1, 0);
    if ((*uref)(i * params_.nu, 0) > 0.1) {
      (*uref)(i * params_.nu, 0) = 0.1;
    } else if ((*uref)(i * params_.nu, 0) < -0.1) {
      (*uref)(i * params_.nu, 0) = -0.1;
    }
    (*uref)(i * params_.nu + 1, 0) = pi2pi((*uref)(i * params_.nu + 1, 0));
  }
  /*
  {
    // QPOASES implementation
    const int kNumH = H.rows() * H.cols();
    double h_mat[kNumH];
    int ind = 0;
    for (int i = 0; i < H.rows(); ++i) {
      for (int j = 0; j < H.cols(); ++j) {
        h_mat[ind] = H(i, j);
        ind++;
      }
    }

    const int kNumG = g.rows() * g.cols();
    double g_mat[kNumG];
    ind = 0;
    for (int i = 0; i < g.rows(); ++i) {
      for (int j = 0; j < g.cols(); ++j) {
        g_mat[ind] = g(i, j);
        ind++;
      }
    }

    double lb[1];
    double ub[1];
    lb[0] = 1.0;
    ub[0] = 1.0;

    double a_constraint[2] = {1.0, 1.0};

    const int kNumVariables = params_.nu * params_.T;
    const int kNumConstraints = 0;
    if (!solver_init_) {
      qproblem_ = qpOASES::QProblem(kNumVariables, kNumConstraints);

      auto ret = qproblem_.init(
        h_mat,
        g_mat,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        params_.max_iter);

      if (ret != qpOASES::SUCCESSFUL_RETURN) {
        std::cerr << "Error in init" << std::endl;
        return false;
      }

      solver_init_ = true;
    }

    qproblem_.hotstart(
      g_mat,
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      params_.max_iter);

    qproblem_.getPrimalSolution(res);
  } */

  return true;
}

void MPCControl::doSimulation(const std::vector<double>& rx,
                              const std::vector<double>& ry,
                              const std::vector<double>& ryaw,
                              const std::vector<double>& rk,
                              const std::vector<double>& sp,
                              const double& dl,
                              const State& initial_state,
                              void* plotting) {
  std::vector<double> goal = {rx.back(), ry.back()};

  state_ = initial_state;

  // initial yaw compensation
  if (state_.yaw - ryaw[0] >= M_PI)
    state_.yaw -= 2.0 * M_PI;
  else if (state_.yaw - ryaw[0] <= -M_PI)
    state_.yaw += 2.0 * M_PI;

  double time = 0.0;
  std::vector<double> x = {state_.x};
  std::vector<double> y = {state_.y};
  std::vector<double> yaw = {state_.yaw};
  std::vector<double> v = {state_.v};
  std::vector<double> t = {0.0};
  std::vector<double> d = {0.0};
  std::vector<double> a = {0.0};

  int target_ind;
  double mind;
  calcNearestIndex(rx, ry, ryaw, 0, &target_ind, &mind);

  std::vector<double> odelta;
  std::vector<double> oa;
  std::vector<double> rryaw = ryaw;
  smoothYaw(&rryaw);

  Eigen::MatrixXd u = Eigen::MatrixXd::Zero(params_.T * params_.nu, 1);

  while (time <= 500.0) {
    Eigen::MatrixXd xref;
    Eigen::MatrixXd uref;
    calcRefTrajectory(rx, ry, rryaw, rk, sp, dl, target_ind,
                      &xref, &target_ind, &uref);

    const int kNumVariables = params_.nu * params_.T;
    double res[kNumVariables] = {0.0};
    linearMPCControl(xref, &u, res);


    /* double ai = res[kNumVariables - 2];
    double di = res[kNumVariables - 1]; */
    double ai = u(0, 0);
    double di = u(1, 0);
    updateState(ai, di);
    time = time + params_.dt;

    x.push_back(state_.x);
    y.push_back(state_.y);
    yaw.push_back(state_.yaw);
    v.push_back(state_.v);
    t.push_back(time);
    d.push_back(di);
    a.push_back(ai);

    if (plotting != nullptr) {
      ((Plotting*)plotting)->plot(rx, ry, x, y);
    }

    std::cout << "Target: " << std::endl;
    for (int i = 0; i < params_.T; ++i) {
      std::cout << "X: " << xref(i * params_.nx, 0)
                << " Y: " << xref(i * params_.nx + 1, 0) << std::endl;
    }
    std::cout << "State: " << std::endl;
    std::cout << "X: " << state_.x
              << " Y: " << state_.y
              << " Yaw: " << state_.yaw << std::endl;
    std::cout << "Control: " << std::endl;
    for (int i = 0; i < params_.T; ++i) {
      std::cout << "ai: " << u(i * params_.nu, 0)
                << " di: " << u(i * params_.nu + 1, 0) << std::endl;
    }

    if (checkGoal(goal, target_ind, rx.size())) {
      std::cout << "Goal reached" << std::endl;
      break;
    }
  }
}

void MPCControl::smoothYaw(std::vector<double>* yaw) {
  for (int i = 0; i < yaw->size() - 1; ++i) {
    double dyaw = yaw->at(i+1) - yaw->at(i);

    while (dyaw >= M_PI / 2.0) {
      yaw->at(i+1) -= 2.0 * M_PI;
      dyaw = yaw->at(i+1) - yaw->at(i);
    }

    while (dyaw <= -M_PI / 2.0) {
      yaw->at(i+1) += 2.0 * M_PI;
      dyaw = yaw->at(i+1) - yaw->at(i);
    }
  }
}

