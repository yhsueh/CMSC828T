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

/** @file PIDcontroller.hpp
 *  @brief The  header file for PIDcontroller class
 *  A basic PID controller that calculate the proporatoinal,
 * integral and derivative term to get the total output. The P,I,D
 * gains are set based on the experiment

 *  @author Zejiang Zeng
 *  @date   12/2017
 */
#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

class PIDcontroller {
 private:
  double dt = 0.66;
  double Kp = 0.002;
  double Ki = 0.001;
  double Kd = 0.00;
  double pre_error = 0;
  double integral = 0;
 public:
  /**
   *   @brief  calculate the proporatoinal term
   *
   *   @param  double
   *   @return double
   */
  double Pterm(double error);
  /**
   *   @brief  calculate the integral term
   *
   *   @param  double
   *   @return double
   */
  double Iterm(double error);
  /**
   *   @brief  calculate the derivative term
   *
   *   @param  double
   *   @return double
   */
  double Dterm(double error);
  /**
   *   @brief  calculate the output by summing all PID terms
   *
   *   @param  double
   *   @return double
   */
  double calculatePID(double error);
};

#endif // _PIDCONTROLLER_H_
