/*
 * File: trapmf.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 21-Dec-2023 10:11:11
 */

/* Include Files */
#include "trapmf.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double x
 *                const double params[4]
 * Return Type  : double
 */
double trapmf(double x, const double params[4])
{
  double b_x;
  double y;
  b_x = 0.0;
  y = 0.0;
  if (x >= params[1]) {
    b_x = 1.0;
  }
  if (x < params[0]) {
    b_x = 0.0;
  }
  if ((params[0] <= x) && (x < params[1])) {
    b_x = (x - params[0]) * (1.0 / (params[1] - params[0]));
  }
  if (x <= params[2]) {
    y = 1.0;
  }
  if (x > params[3]) {
    y = 0.0;
  }
  if ((params[2] < x) && (x <= params[3])) {
    y = (params[3] - x) * (1.0 / (params[3] - params[2]));
  }
  return fmin(b_x, y);
}

/*
 * File trailer for trapmf.c
 *
 * [EOF]
 */
