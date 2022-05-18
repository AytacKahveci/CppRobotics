// Copyright 2022 Aytac Kahveci
#include <cubic_spline_planner/cubic_spline_planner.h>

Spline::Spline(const std::vector<double>& x, const std::vector<double>& y)
    : x_(x), y_(y) {
  std::vector<double> h;
  nx_ = x_.size();

  // find diff
  for (size_t i = 0; i < nx_ - 1; ++i) {
    h.push_back(x[i+1] - x[i]);
  }

  // calc coefficient a
  for (auto iy : y) {
    a_.push_back(iy);
  }

  // calc coefficient c
  Eigen::MatrixXd A;
  Eigen::VectorXd B;
  calcA(h, &A);
  calcB(h, &B);
  Eigen::VectorXd c = A.colPivHouseholderQr().solve(B);
  c_.resize(c.size());
  Eigen::VectorXd::Map(&c_[0], c.size()) = c;

  // calc coefficients b and d
  for (size_t i = 0; i < nx_ - 1; ++i) {
    d_.push_back((c_[i+1] - c_[i]) / (3.0 * h[i]));
    b_.push_back((a_[i+1] - a_[i]) / h[i] - \
                  h[i] * (c_[i+1] + 2.0 * c_[i]) / 3.0);
  }
}

Spline::~Spline() {}

void Spline::calc(double* pos, const double& t) {
  if (t < x_.front() || t > x_.back()) {
    pos = nullptr;
  }

  int i = searchIndex(t);
  double dx = t - x_[i];
  (*pos) = a_[i] + b_[i] * dx + c_[i] * std::pow(dx, 2) \
         + d_[i] * std::pow(dx, 3);
}

void Spline::calcd(double* vel, const double& t) {
  if (t < x_.front() || t > x_.back()) {
    vel = nullptr;
  }

  int i = searchIndex(t);
  double dx = t - x_[i];
  (*vel) = b_[i] + 2.0 * c_[i] * dx \
         + 3.0 * d_[i] * std::pow(dx, 2);
}

void Spline::calcdd(double* acc, const double& t) {
  if (t < x_.front() || t > x_.back()) {
    acc = nullptr;
  }

  int i = searchIndex(t);
  double dx = t - x_[i];
  (*acc) = 2.0 * c_[i] * dx + 6.0 * d_[i] * dx;
}

int Spline::searchIndex(const double& t) {
  auto it = std::lower_bound(x_.begin(), x_.end(), t);
  return std::distance(x_.begin(), it);
}

void Spline::calcA(const std::vector<double>& h, Eigen::MatrixXd* A) {
  (*A) = Eigen::MatrixXd::Zero(nx_, nx_);
  (*A)(0, 0) = 1.0;
  for (int i = 0; i < nx_ - 1; ++i) {
    if (i != nx_ - 2) {
      (*A)(i+1, i+1) = 2.0 * (h[i] + h[i+1]);
    }

    (*A)(i+1, i) = h[i];
    (*A)(i, i+1) = h[i];
  }
  (*A)(0, 1) = 0.0;
  (*A)(nx_-1, nx_-2) = 0.0;
  (*A)(nx_-1, nx_-1) = 1.0;
}

void Spline::calcB(const std::vector<double>& h, Eigen::VectorXd* B) {
  (*B) = Eigen::VectorXd::Zero(nx_);
  for (int i = 0; i < nx_ - 2; ++i) {
    (*B)(i+1) = 3.0 * (a_[i+2] - a_[i+1]) / h[i+1] \
              - 3.0 * (a_[i+1] - a_[i]) / h[i];
  }
}

void Spline::getCoeffs(std::vector<double>* a, std::vector<double>* b,
                       std::vector<double>* c, std::vector<double>* d) {
  *a = a_;
  *b = b_;
  *c = c_;
  *d = d_;
}





Spline2D::Spline2D(const std::vector<double>& x, const std::vector<double>& y) {
  calcS(x, y, &s_);
  sx_ = Spline(s_, x);
  sy_ = Spline(s_, y);
}

void Spline2D::calcS(const std::vector<double>& x,
                     const std::vector<double>& y,
                     std::vector<double>* s) {
  double tot = 0;
  s->clear();
  s->push_back(tot);
  for (size_t i = 0; i < x.size() - 1; ++i) {
    double ds = hypotf(x[i+1] - x[i], y[i+1] - y[i]);
    tot += ds;
    s->push_back(tot);
  }
}

void Spline2D::calcPos(const double& s, double* x, double* y) {
  sx_.calc(x, s);
  sy_.calc(y, s);
}

void Spline2D::calcCurvature(const double& s, double* curvature) {
  double dx, ddx;
  sx_.calcd(&dx, s);
  sx_.calcdd(&ddx, s);

  double dy, ddy;
  sy_.calcd(&dy, s);
  sy_.calcdd(&ddy, s);

  *curvature = (ddy * dx - ddx * dy) /  \
               (std::pow((dx * dx + dy * dy), 3.0 / 2.0));
}

void Spline2D::calcYaw(const double& s, double* yaw) {
  double dx, dy;
  sx_.calcd(&dx, s);
  sy_.calcd(&dy, s);
  *yaw = atan2(dy, dx);
}

void Spline2D::calcSplineCourse(const double& ds,
                                std::vector<double>* rx,
                                std::vector<double>* ry,
                                std::vector<double>* ryaw,
                                std::vector<double>* rk) {
  for (double i = 0.0; i <= s_.back(); i+=ds) {
    double x, y;
    calcPos(i, &x, &y);
    rx->push_back(x);
    ry->push_back(y);
    double yaw;
    calcYaw(i, &yaw);
    ryaw->push_back(yaw);
    double curvature;
    calcCurvature(i, &curvature);
    rk->push_back(curvature);
  }
}
