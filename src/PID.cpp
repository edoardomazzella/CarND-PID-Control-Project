#include "PID.h"

#include <iostream>
#include <math.h>

/*============================================================================*
 *                             Public Functions                               *
 *============================================================================*/

void PID::TwiddleStep(double cte)
{
  std::cout << "\n\nCTE = " << cte_ << std::endl;
  std::cout << "CTE accumulation = " << acc_cte_ << std::endl;
  std::cout << "CTE difference = " << diff_cte_ << std::endl;
  std::cout << "Coefficient Index = " << index_ << std::endl;
  std::cout << "State = " << state_ << std::endl;

  acc_cte_ = 0.0;
  switch(state_)
  {
    case kInitialization:
    {
      coefficients_ = {0.0, 0.0, 0.0};
      d_coefficients_ = {5.0, 5.0, 5.0};
      tol_ = 0.2;
      index_ = 0;
      best_cte_ = cte;
      if (GetTwiddleSum_() > tol_)
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
      if(std::fabs(cte) < std::fabs(best_cte_))
      {
        best_cte_ = cte;
        d_coefficients_[index_] *= 1.5;
        index_ = (index_ + 1) % 3;
        if (GetTwiddleSum_() > tol_)
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
      if(std::fabs(cte) < std::fabs(best_cte_))
      {
        best_cte_ = cte;
        d_coefficients_[index_] *= 1.5;
      }
      else
      {
        coefficients_[index_] += d_coefficients_[index_];
        d_coefficients_[index_] *= 0.5;
      }
      index_ = (index_ + 1) % 3;
      if (GetTwiddleSum_() > tol_)
      {
        state_ = kNewCoefficient;
      }
      else
      {
        std::cout << "\nTwiddle is stopped" << std::endl;
        state_ = kStop;
      }
      break;
    }
    default:
      state_ = kStop;
      break;
  }
  std::cout << "PID coefficients after Twiddle step = [" << coefficients_[0]
            << ", " << coefficients_[1] << ", " << coefficients_[2] << ']'
            << std::endl;
  std::cout << "Twiddle increments after Twiddle step = ["
            << d_coefficients_[0] << ", " << d_coefficients_[1] << ", "
            << d_coefficients_[2] << ']' << std::endl;
}

/*============================================================================*/

double PID::ComputeControlVariable(double cte)
{
  UpdateError_(cte);

  double control_variable = - coefficients_[0] * cte_ - coefficients_[1] *
                              acc_cte_ - coefficients_[2] * diff_cte_;

  return control_variable;
}

/*============================================================================*
 *                            Private Functions                               *
 *============================================================================*/

void PID::UpdateError_(double cte)
{
  diff_cte_ = cte - cte_;
  acc_cte_ += cte;
  cte_ = cte;
}

/*============================================================================*/