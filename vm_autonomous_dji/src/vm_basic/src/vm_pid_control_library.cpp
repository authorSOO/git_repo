/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "../include/vm_pid_control_library.h"


class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};


void PID::PID_set( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{
    
    // Calculate error
    double error = fabs(setpoint - pv);

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif

#if 0
/*    
 *  The PID library is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as 
 *  published by the Free Software Foundation, either version 3 of the 
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>   // abs()
#include "pid.h"

namespace Pid {
  PID::PID(const uint32_t _setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t _qn, const FeedbackDirection _feedbackDirection, const ProportionalGain proportionalGain)
      : feedbackDirection(_feedbackDirection), qn(_qn), setpoint(_setpoint) {
      this->setTunings(kp, ki, kd, proportionalGain);
  }

  PID::PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn, const FeedbackDirection feedbackDirection)
      :PID::PID(setpoint, kp, ki, kd, qn, feedbackDirection, proportionalToError) {}

  PID::PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn)
      :PID::PID(setpoint, kp, ki, kd, qn, feedbackNegative) {}

  /**
      Calculate a new output and store update all internal state variables. Note: The input is a simple number without
      any fixed point fractional precision. The reason is, that we need to scale the output according to the DAC
      precision anyway.
      
      QN should be scaled to that it is Q(DAC_resolution).(32-DAC_resolution).
      @param input The new input variable in Q32.0 format, that is with zero frational precision. The value is
        an unsigned integer in Offset Binary notation.
      @param *output The output in Q(DAC_resolution).0 format and is determinded by the ki, kp, kd
        parameters used in the multiplications. The encoding is Offset Binary, so it can be directly fed to most DACs.
  */
  uint32_t PID::compute(const uint32_t input) {
      // Calcualte P term
      // Note: the calculation is (uint32_t)setpoint - (uint32_t)(input) = (int32_t)error (using signed math)
      // This is true for offset binary values!
      const int32_t error = signed_subtract_saturated_32_and_32(this->setpoint, input);    // Subtract using saturating math
      // Calcualte I term
      // TODO: Think about taking into account the previous result as well
      // -> Bilinear Transform instead of Backward difference
      // https://en.wikipedia.org/wiki/Bilinear_transform
      errorSum = signed_multiply_accumulate_saturated_32_and_32QN(    // Calculate (errorSum + ki*error) using saturating math
          this->errorSum,
          this->ki,
          error
      );
      // Calculate D term (Note: We actually calculate -dInput)
      // We do not calculate dError, because this would cause an output spike every time someone changes the setpoint
      // dError = d(Setpoint - Input)_n - d(Setpoint - Input)_(n-1)
      //        = dSetpoint - dInput
      // We would like to get rid of the setpoint dependence and during normal operation, there is no difference
      //        ≈ -dInput
      const int32_t dInputNegative = signed_subtract_saturated_32_and_32(this->previousInput, input);    // Subtract using saturating math

      // Store the input to calculate the D-term next time
      this->previousInput = input;

      if (UNLIKELY(proportionalGain == proportionalToInput)) {
          errorSum = signed_multiply_accumulate_saturated_32_and_32QN(
              this->errorSum,
              this->kp,
              dInputNegative
          );
      }

      // This will prevent integral windup
      this->errorSum = clamp(errorSum, outputMin, outputMax);

      int32_t output = signed_multiply_accumulate_saturated_32_and_32QN(
          this->errorSum,
          this->kd,
          dInputNegative
      );

      // Normal PID
      if (LIKELY(proportionalGain == proportionalToError)) {
          output = signed_multiply_accumulate_saturated_32_and_32QN(
              output,
              this->kp,
              error
          );
      }

      output = clamp(output, outputMin, outputMax);
      output ^= 0x80000000;   // Convert from 2s complement to offset Binary
      return (uint32_t)output >> this->qn;
  }

  /** Note: ki and kd must be normalized to the sampling time
   */
  void PID::setTunings(const int32_t kp, const int32_t ki, const int32_t kd, const ProportionalGain proportionalGain) {
      // We calculate error as: error = setpoint - input
      // If the error is positive, the control output will be positive as well,
      // so we need to invert the output if the plant response to an error is negative
      this->kp = (this->feedbackDirection == feedbackPositive) ? abs(kp) : -abs(kp);
      this->ki = (this->feedbackDirection == feedbackPositive) ? abs(ki) : -abs(ki);
      this->kd = (this->feedbackDirection == feedbackPositive) ? abs(kd) : -abs(kd);

      this->proportionalGain = proportionalGain;
  }

  void PID::setTunings(const int32_t kp, const int32_t ki, const int32_t kd) {
     this->setTunings(kp, ki, kd, this->proportionalGain);
  }

  void PID::setOutputMin(const uint32_t value) {
      this->outputMin = (int32_t)((value << this->qn) ^ 0x80000000);    // Convert from Offset Binary to Integer
  }

  void PID::setOutputMax(const uint32_t value) {
      this->outputMax = (int32_t)((value << this->qn) ^ 0x80000000);    // Convert from Offset Binary to Integer
  }

  void PID::setSetpoint(const uint32_t value) {
      this->setpoint = value;
  }

  uint32_t PID::getSetpoint() {
      return this->setpoint;
  }

  int32_t PID::getIntegratorError() {
      return this->errorSum;
  }

  void PID::init(const uint32_t initialInput) {
      PID::init(initialInput, 0);
  }

  void PID::init(const uint32_t initialInput, const int32_t initialErrorSum) {
      this->previousInput = initialInput;
      this->errorSum = clamp(initialErrorSum, this->outputMin, this->outputMax);
  }

  void PID::updateOutput(const uint32_t value) {
      this->errorSum = clamp((value << this->qn) ^ 0x80000000, this->outputMin, this->outputMax);
  }

  void PID::setControllerFeedback(const FeedbackDirection feedbackDirection) {
    this->feedbackDirection = feedbackDirection;

    this->setTunings(this->kp, this->ki, this->kd);
  }

  uint32_t PID::getKp() {
      return this->kp;
  }

  uint32_t PID::getKi() {
      return  this->ki;
  }

  uint32_t PID::getKd() {
      return this->kd;
  }
}   // namespace Pid/*    
 *  The PID library is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as 
 *  published by the Free Software Foundation, either version 3 of the 
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>   // abs()
#include "pid.h"

namespace Pid {
  PID::PID(const uint32_t _setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t _qn, const FeedbackDirection _feedbackDirection, const ProportionalGain proportionalGain)
      : feedbackDirection(_feedbackDirection), qn(_qn), setpoint(_setpoint) {
      this->setTunings(kp, ki, kd, proportionalGain);
  }

  PID::PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn, const FeedbackDirection feedbackDirection)
      :PID::PID(setpoint, kp, ki, kd, qn, feedbackDirection, proportionalToError) {}

  PID::PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn)
      :PID::PID(setpoint, kp, ki, kd, qn, feedbackNegative) {}

  /**
      Calculate a new output and store update all internal state variables. Note: The input is a simple number without
      any fixed point fractional precision. The reason is, that we need to scale the output according to the DAC
      precision anyway.
      
      QN should be scaled to that it is Q(DAC_resolution).(32-DAC_resolution).
      @param input The new input variable in Q32.0 format, that is with zero frational precision. The value is
        an unsigned integer in Offset Binary notation.
      @param *output The output in Q(DAC_resolution).0 format and is determinded by the ki, kp, kd
        parameters used in the multiplications. The encoding is Offset Binary, so it can be directly fed to most DACs.
  */
  uint32_t PID::compute(const uint32_t input) {
      // Calcualte P term
      // Note: the calculation is (uint32_t)setpoint - (uint32_t)(input) = (int32_t)error (using signed math)
      // This is true for offset binary values!
      const int32_t error = signed_subtract_saturated_32_and_32(this->setpoint, input);    // Subtract using saturating math
      // Calcualte I term
      // TODO: Think about taking into account the previous result as well
      // -> Bilinear Transform instead of Backward difference
      // https://en.wikipedia.org/wiki/Bilinear_transform
      errorSum = signed_multiply_accumulate_saturated_32_and_32QN(    // Calculate (errorSum + ki*error) using saturating math
          this->errorSum,
          this->ki,
          error
      );
      // Calculate D term (Note: We actually calculate -dInput)
      // We do not calculate dError, because this would cause an output spike every time someone changes the setpoint
      // dError = d(Setpoint - Input)_n - d(Setpoint - Input)_(n-1)
      //        = dSetpoint - dInput
      // We would like to get rid of the setpoint dependence and during normal operation, there is no difference
      //        ≈ -dInput
      const int32_t dInputNegative = signed_subtract_saturated_32_and_32(this->previousInput, input);    // Subtract using saturating math

      // Store the input to calculate the D-term next time
      this->previousInput = input;

      if (UNLIKELY(proportionalGain == proportionalToInput)) {
          errorSum = signed_multiply_accumulate_saturated_32_and_32QN(
              this->errorSum,
              this->kp,
              dInputNegative
          );
      }

      // This will prevent integral windup
      this->errorSum = clamp(errorSum, outputMin, outputMax);

      int32_t output = signed_multiply_accumulate_saturated_32_and_32QN(
          this->errorSum,
          this->kd,
          dInputNegative
      );

      // Normal PID
      if (LIKELY(proportionalGain == proportionalToError)) {
          output = signed_multiply_accumulate_saturated_32_and_32QN(
              output,
              this->kp,
              error
          );
      }

      output = clamp(output, outputMin, outputMax);
      output ^= 0x80000000;   // Convert from 2s complement to offset Binary
      return (uint32_t)output >> this->qn;
  }

  /** Note: ki and kd must be normalized to the sampling time
   */
  void PID::setTunings(const int32_t kp, const int32_t ki, const int32_t kd, const ProportionalGain proportionalGain) {
      // We calculate error as: error = setpoint - input
      // If the error is positive, the control output will be positive as well,
      // so we need to invert the output if the plant response to an error is negative
      this->kp = (this->feedbackDirection == feedbackPositive) ? abs(kp) : -abs(kp);
      this->ki = (this->feedbackDirection == feedbackPositive) ? abs(ki) : -abs(ki);
      this->kd = (this->feedbackDirection == feedbackPositive) ? abs(kd) : -abs(kd);

      this->proportionalGain = proportionalGain;
  }

  void PID::setTunings(const int32_t kp, const int32_t ki, const int32_t kd) {
     this->setTunings(kp, ki, kd, this->proportionalGain);
  }

  void PID::setOutputMin(const uint32_t value) {
      this->outputMin = (int32_t)((value << this->qn) ^ 0x80000000);    // Convert from Offset Binary to Integer
  }

  void PID::setOutputMax(const uint32_t value) {
      this->outputMax = (int32_t)((value << this->qn) ^ 0x80000000);    // Convert from Offset Binary to Integer
  }

  void PID::setSetpoint(const uint32_t value) {
      this->setpoint = value;
  }

  uint32_t PID::getSetpoint() {
      return this->setpoint;
  }

  int32_t PID::getIntegratorError() {
      return this->errorSum;
  }

  void PID::init(const uint32_t initialInput) {
      PID::init(initialInput, 0);
  }

  void PID::init(const uint32_t initialInput, const int32_t initialErrorSum) {
      this->previousInput = initialInput;
      this->errorSum = clamp(initialErrorSum, this->outputMin, this->outputMax);
  }

  void PID::updateOutput(const uint32_t value) {
      this->errorSum = clamp((value << this->qn) ^ 0x80000000, this->outputMin, this->outputMax);
  }

  void PID::setControllerFeedback(const FeedbackDirection feedbackDirection) {
    this->feedbackDirection = feedbackDirection;

    this->setTunings(this->kp, this->ki, this->kd);
  }

  uint32_t PID::getKp() {
      return this->kp;
  }

  uint32_t PID::getKi() {
      return  this->ki;
  }

  uint32_t PID::getKd() {
      return this->kd;
  }
}   // namespace Pid
#endif