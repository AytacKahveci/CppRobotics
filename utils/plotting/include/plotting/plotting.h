// Copyright 2022 Aytac Kahveci
#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <QApplication>
#include <qcustomplot/qcustomplot.h>

class Plotting : public QObject{
  Q_OBJECT
 public:
  /**
   * @brief Construct a new Plotting object
   * 
   * @param parent parent object
   */
  Plotting(QObject* parent = 0) : QObject(parent) {}

  /**
   * @brief Destroy the Plotting object
   * 
   */
  ~Plotting();

  /**
   * @brief Plot target path and the robot path
   * 
   * @param x target path x positions
   * @param y target path y positions
   * @param robot_x robot path x positions
   * @param robot_y robot path y positions
   */
  void plot(const std::vector<double>& x,
            const std::vector<double>& y,
            const std::vector<double>& robot_x,
            const std::vector<double>& robot_y);

  /**
   * @brief Init Qt application
   * 
   * @param argc argument count
   * @param argv argument values
   */
  void init(int &argc, char** argv);

 private:
  boost::shared_ptr<QApplication> app_; //<! Qt application
  boost::shared_ptr<QWidget> window_;  //<! Qt window
  boost::shared_ptr<QCustomPlot> custom_plot_;  //<! plot object
  boost::shared_ptr<boost::thread> t_;
};
