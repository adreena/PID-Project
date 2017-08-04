#ifndef PID_H
#define PID_H
#include <vector>
class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  bool re_init;
  int cte_counter;
  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  double current_err;
  double best_err;
  std::vector<double> dp;
  int counter;
  int bad_update_count;
  double cte_previous;
  bool is_twiddling;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  void UpdateParams(int index, int coef);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  double Totaldp();
  double twiddle(double new_err, double best_err);
};

#endif /* PID_H */
