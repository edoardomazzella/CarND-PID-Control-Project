#ifndef PID_H_
#define PID_H_

#include <array>
#include <numeric>

/*============================================================================*
 *                             Class Definition                               *
 *============================================================================*/

class PID
{
public:
  enum TwiddleState
  {
    kInitialization = 0,
    kNewCoefficient,
    kPostIncrement,
    kPostDecrement,
    kStop,
  };

  /**
   * Constructor
   * Initialize PID.
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  PID(double k_p = 0, double k_i = 0, double k_d = 0) : coefficients_{k_p, k_i, k_d} { }

  /**
   * Destructor.
   */
  virtual ~PID() { }

  /**
   * Twiddle algorithm step for tuning parameters
   * @param cte The current cross track error
   */
  void TwiddleStep(double cte);

  /**
   * Compute the control variable given cross track error
   * @param cte The current cross track error
   * @return The control variable value
   */
  double ComputeControlVariable(double cte);

  /**
   * Returns the Twiddle tolerance
   * @return The Twiddle tolerance
   */
  double GetTwiddleState() { return state_; }

private:
  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError_(double cte);

  /**
   * Returns the sum of the coefficient increments
   * @return The sum of the coefficient increments
   */
  double GetTwiddleSum_()
  {
    return std::accumulate(d_coefficients_.begin(), d_coefficients_.end(), 0.0);
  }

  /**
   * PID Coefficients Kp, Ki and Kd
   */
  std::array<double, 3> coefficients_ = {0.0, 0.0, 0.0};;

  /**
   * PID Errors
   */
  double cte_ = 0.0;      // error
  double acc_cte_ = 0.0;  // accumulated error
  double diff_cte_ = 0.0; // error difference

  /* Twiddle variables */
  double tol_ = 0;
  std::array<double, 3> d_coefficients_ = {0.0, 0.0, 0.0};
  TwiddleState state_ = kInitialization;
  int index_ = 0;
  double best_cte_ = 0;
};

#endif // PID_H_