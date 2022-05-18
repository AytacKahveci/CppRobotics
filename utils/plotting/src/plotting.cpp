// Copyright 2022 Aytac Kahveci
#include "plotting/plotting.h"

Plotting::~Plotting() {}

void Plotting::plot(const std::vector<double>& x,
                    const std::vector<double>& y,
                    const std::vector<double>& robot_x,
                    const std::vector<double>& robot_y) {
  QVector<double> cx, cy;
  cx = QVector<double>::fromStdVector(x);
  cy = QVector<double>::fromStdVector(y);

  std::vector<double> tx = x;
  std::vector<double> ty = y;
  double minx, maxx;
  std::sort(tx.begin(), tx.end(), [] (auto a, auto b) { return a < b; });
  minx = tx.front();
  maxx = tx.back();

  double miny, maxy;
  std::sort(ty.begin(), ty.end(), [] (auto a, auto b) { return a < b; });
  miny = ty.front();
  maxy = ty.back();

  // create graph and assign data to it:
  custom_plot_->graph(0)->setData(cx, cy);
  // give the axes some labels:
  custom_plot_->xAxis->setLabel("x");
  custom_plot_->yAxis->setLabel("y");
  // set axes ranges, so we see all data:
  custom_plot_->xAxis->setRange(minx, maxx);
  custom_plot_->yAxis->setRange(miny, maxy);

  QVector<double> rx, ry;
  rx = QVector<double>::fromStdVector(robot_x);
  ry = QVector<double>::fromStdVector(robot_y);
  custom_plot_->graph(1)->setPen(QPen(Qt::red));
  custom_plot_->graph(1)->setData(rx, ry);

  custom_plot_->replot();
  app_->processEvents();
}

void Plotting::init(int &argc, char** argv) {
  app_.reset(new QApplication(argc, argv));

  window_.reset(new QWidget);
  window_->setFixedSize(900, 800);

  custom_plot_.reset(new QCustomPlot(window_.get()));
  custom_plot_->setGeometry(0, 0, 900, 800);
  custom_plot_->addGraph();
  custom_plot_->addGraph();

  window_->show();
}