/********************************************************************
 *   MIT License
 *
 *   Copyright (c) 2017 Zejiang Zeng
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/

/** @file PIDcontroller.cpp

 *  @brief The  source file for PIDcontroller class
 *  A basic PID controller that calculate the proporatoinal,
 * integral and derivative term to get the total output

 *  @author Zejiang Zeng
 *  @date   12/2017
 */
#include<cmath>
#include"../include/ROS_opencv_improcess/PIDcontroller.hpp"

double PIDcontroller::calculatePID(double error) {
// proporatoinal term
  double Pout = Pterm(error);
// integrate term
  double Iout = Iterm(error);
// derivative term
  double Dout = Dterm(error);
// calculate the total term
  double TotalOut = Pout + Iout + Dout;

  return TotalOut;
}

double PIDcontroller::Pterm(double error) {
  return Kp * error;
}

double PIDcontroller::Iterm(double error) {
  integral += error * dt;
  return Ki * integral;
}
double PIDcontroller::Dterm(double error) {
  double derivative = (error - pre_error) / dt;
  pre_error = error;
  return Kd * derivative;
}
