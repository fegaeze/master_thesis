/*
 * File: trimf.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Dec-2023 10:55:08
 */

/* Include Files */
#include "trimf.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double x
 *                const double params[3]
 * Return Type  : double
 */
double trimf(double x, const double params[3])
{
  double y;
  y = 0.0;
  if ((params[0] != params[1]) && (params[0] < x) && (x < params[1])) {
    y = (x - params[0]) * (1.0 / (params[1] - params[0]));
  }
  if ((params[1] != params[2]) && (params[1] < x) && (x < params[2])) {
    y = (params[2] - x) * (1.0 / (params[2] - params[1]));
  }
  if (x == params[1]) {
    y = 1.0;
  }
  return y;
}

/*
 * File trailer for trimf.c
 *
 * [EOF]
 */
