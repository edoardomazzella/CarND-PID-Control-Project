#include "PID.h"

#include <iostream>
#include <math.h>

/*============================================================================*
 *                             Public Functions                               *
 *============================================================================*/

void PID::TwiddleStep(double cte)
{
  std::cout << "\nIndex = " << index_ << std::endl;
  std::cout << "State = " << state_ << std::endl;
  switch(state_)
  {
    case kInitialization:
    {
      coefficients_ = {0.0, 0.0, 0.0};
      d_coefficients_ = {0.1, 0.1, 0.1};
      tol_ = 0.02;
      index_ = 0;
      best_cte_ = cte;
      if (GetTwiddleSum() > GetTwiddleTolerance())
      {
        state_ = kNewCoefficient;
      }
      else
      {
        state_ = kStop;
      }
      break;
    }
    case kNewCoefficient:
    {
      best_cte_ = cte;
      coefficients_[index_] += d_coefficients_[index_];
      state_ = kPostIncrement;
      break;
    }
    case kPostIncrement:
    {
      if(cte < best_cte_)
      {
        best_cte_ = cte;
        d_coefficients_[index_] *= 1.1;
        index_ = (index_ + 1) % 3;
        if (GetTwiddleSum() > GetTwiddleTolerance())
        {
          state_ = kNewCoefficient;
        }
        else
        {
          state_ = kStop;
        }
      }
      else
      {
        coefficients_[index_] -= 2 * d_coefficients_[index_];
        state_ = kPostDecrement;
      }
      break;
    }
    case kPostDecrement:
    {
      if(cte < best_cte_)
      {
        best_cte_ = cte;
        d_coefficients_[index_] *= 1.1;
      }
      else
      {
        coefficients_[index_] += d_coefficients_[index_];
        d_coefficients_[index_] *= 0.9;
      }
      index_ = (index_ + 1) % 3;
      if (GetTwiddleSum() > GetTwiddleTolerance())
      {
        state_ = kNewCoefficient;
      }
      else
      {
        state_ = kStop;
      }
      break;
    }
    default:
      break;
  }
  std::cout << "PID coefficients = [" << coefficients_[0] << ", "
            << coefficients_[1] << ", " << coefficients_[2] << ']' << std::endl;
  std::cout << "Twiddle increments = [" << d_coefficients_[0] << ", "
            << d_coefficients_[1] << ", " << d_coefficients_[2] << ']'
            << std::endl;
}

/*============================================================================*/

double PID::ComputeControlVariable(double cte)
{
  double saturation_value = 1.0;
  UpdateError_(cte);

  double control_variable = - coefficients_[0] * cte_ - coefficients_[1] *
                              acc_cte_ - coefficients_[2] * diff_cte_;

  control_variable = (control_variable <= saturation_value)?
                      control_variable : saturation_value;
  return (control_variable >= -saturation_value)?
          control_variable : -saturation_value;
}

/*============================================================================*
 *                            Private Functions                               *
 *============================================================================*/

void PID::UpdateError_(double cte)
{
  diff_cte_ = cte - cte_;
  acc_cte_ += cte;
  cte_ = cte;

  std::cout << "CTE = " << cte_ << std::endl;
  std::cout << "CTE accumulation = " << acc_cte_ << std::endl;
  std::cout << "CTE difference = " << diff_cte_ << std::endl;
}

/*============================================================================*/