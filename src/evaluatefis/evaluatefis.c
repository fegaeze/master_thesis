/*
 * File: evaluatefis.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Dec-2023 10:55:08
 */

/* Include Files */
#include "evaluatefis.h"
#include "rt_nonfinite.h"
#include "trimf.h"

/* Function Definitions */
/*
 * Arguments    : double x
 * Return Type  : double
 */
double evaluatefis(double x)
{
  static const double outputMFCache[153] = {0.0,
                                            0.0,
                                            0.0,
                                            0.43999999999999984,
                                            0.0,
                                            0.0,
                                            0.87999999999999967,
                                            0.0,
                                            0.0,
                                            0.89333333333333331,
                                            0.10666666666666665,
                                            0.0,
                                            0.7466666666666667,
                                            0.25333333333333324,
                                            0.0,
                                            0.60000000000000009,
                                            0.39999999999999986,
                                            0.0,
                                            0.45333333333333337,
                                            0.54666666666666663,
                                            0.0,
                                            0.30666666666666675,
                                            0.69333333333333325,
                                            0.0,
                                            0.16000000000000011,
                                            0.83999999999999986,
                                            0.0,
                                            0.013333333333333367,
                                            0.98666666666666658,
                                            0.0,
                                            0.0,
                                            0.86666666666666692,
                                            0.13333333333333311,
                                            0.0,
                                            0.72000000000000008,
                                            0.27999999999999986,
                                            0.0,
                                            0.57333333333333336,
                                            0.42666666666666658,
                                            0.0,
                                            0.42666666666666692,
                                            0.573333333333333,
                                            0.0,
                                            0.28000000000000014,
                                            0.71999999999999986,
                                            0.0,
                                            0.13333333333333366,
                                            0.86666666666666625,
                                            0.0,
                                            0.0,
                                            0.9973333333333334,
                                            0.0,
                                            0.0,
                                            0.968,
                                            0.0,
                                            0.0,
                                            0.93866666666666665,
                                            0.0,
                                            0.0,
                                            0.90933333333333333,
                                            0.0,
                                            0.0,
                                            0.88,
                                            0.0,
                                            0.0,
                                            0.85066666666666668,
                                            0.0,
                                            0.0,
                                            0.82133333333333336,
                                            0.0,
                                            0.0,
                                            0.79199999999999993,
                                            0.0,
                                            0.0,
                                            0.7626666666666666,
                                            0.0,
                                            0.0,
                                            0.73333333333333339,
                                            0.0,
                                            0.0,
                                            0.70400000000000007,
                                            0.0,
                                            0.0,
                                            0.67466666666666664,
                                            0.0,
                                            0.0,
                                            0.64533333333333331,
                                            0.0,
                                            0.0,
                                            0.61599999999999988,
                                            0.0,
                                            0.0,
                                            0.58666666666666678,
                                            0.0,
                                            0.0,
                                            0.55733333333333335,
                                            0.0,
                                            0.0,
                                            0.528,
                                            0.0,
                                            0.0,
                                            0.49866666666666665,
                                            0.0,
                                            0.0,
                                            0.46933333333333332,
                                            0.0,
                                            0.0,
                                            0.44,
                                            0.0,
                                            0.0,
                                            0.41066666666666662,
                                            0.0,
                                            0.0,
                                            0.3813333333333333,
                                            0.0,
                                            0.0,
                                            0.35200000000000004,
                                            0.0,
                                            0.0,
                                            0.32266666666666671,
                                            0.0,
                                            0.0,
                                            0.29333333333333345,
                                            0.0,
                                            0.0,
                                            0.26400000000000012,
                                            0.0,
                                            0.0,
                                            0.23466666666666661,
                                            0.0,
                                            0.0,
                                            0.20533333333333331,
                                            0.0,
                                            0.0,
                                            0.17600000000000002,
                                            0.0,
                                            0.0,
                                            0.14666666666666672,
                                            0.0,
                                            0.0,
                                            0.11733333333333319,
                                            0.0,
                                            0.0,
                                            0.087999999999999884,
                                            0.0,
                                            0.0,
                                            0.058666666666666596,
                                            0.0,
                                            0.0,
                                            0.029333333333333527,
                                            0.0,
                                            0.0,
                                            0.0};
  static const double b_x[51] = {-0.002,
                                 -0.0017800000000000001,
                                 -0.0015600000000000002,
                                 -0.00134,
                                 -0.0011200000000000001,
                                 -0.00090000000000000019,
                                 -0.00068,
                                 -0.00046000000000000012,
                                 -0.0002400000000000002,
                                 -2.0000000000000052E-5,
                                 0.00019999999999999966,
                                 0.0004199999999999998,
                                 0.00063999999999999994,
                                 0.00085999999999999965,
                                 0.0010799999999999998,
                                 0.0012999999999999995,
                                 0.0015199999999999997,
                                 0.0017399999999999998,
                                 0.00196,
                                 0.0021799999999999996,
                                 0.0023999999999999994,
                                 0.00262,
                                 0.0028399999999999996,
                                 0.0030600000000000002,
                                 0.00328,
                                 0.0034999999999999996,
                                 0.0037199999999999993,
                                 0.00394,
                                 0.00416,
                                 0.00438,
                                 0.0045999999999999991,
                                 0.00482,
                                 0.0050399999999999993,
                                 0.00526,
                                 0.00548,
                                 0.0056999999999999993,
                                 0.00592,
                                 0.00614,
                                 0.0063599999999999993,
                                 0.006579999999999999,
                                 0.0067999999999999988,
                                 0.0070199999999999985,
                                 0.00724,
                                 0.00746,
                                 0.0076799999999999993,
                                 0.007899999999999999,
                                 0.00812,
                                 0.00834,
                                 0.00856,
                                 0.0087799999999999979,
                                 0.009};
  static const double dv[3] = {-100.0, -10.0, 0.0};
  static const double dv1[3] = {0.0, 10.0, 100.0};
  static const signed char iv[3] = {1, 3, 2};
  double orr[153];
  double arr[51];
  double dv2[3];
  double w[3];
  double aggVal;
  double d;
  double sw;
  double y;
  int ruleID;
  int sampleID;
  w[0] = trimf(x, dv);
  w[1] = trimf(x, dv1);
  dv2[0] = -10.0;
  dv2[1] = 0.0;
  dv2[2] = 10.0;
  w[2] = trimf(x, dv2);
  sw = 0.0;
  for (ruleID = 0; ruleID < 3; ruleID++) {
    d = w[ruleID];
    sw += d;
    for (sampleID = 0; sampleID < 51; sampleID++) {
      int orr_tmp;
      aggVal = outputMFCache[(iv[ruleID] + 3 * sampleID) - 1];
      orr_tmp = sampleID + 51 * ruleID;
      orr[orr_tmp] = aggVal;
      if (aggVal > d) {
        orr[orr_tmp] = d;
      } else {
        orr[orr_tmp] = aggVal;
      }
    }
  }
  for (sampleID = 0; sampleID < 51; sampleID++) {
    aggVal = orr[sampleID];
    for (ruleID = 0; ruleID < 2; ruleID++) {
      d = orr[sampleID + 51 * (ruleID + 1)];
      if ((aggVal < d) || rtIsNaN(aggVal)) {
        aggVal = d;
      }
    }
    arr[sampleID] = aggVal;
  }
  if (sw == 0.0) {
    y = 0.0034999999999999996;
  } else {
    y = 0.0;
    aggVal = 0.0;
    for (ruleID = 0; ruleID < 51; ruleID++) {
      aggVal += arr[ruleID];
    }
    if (aggVal == 0.0) {
      y = 0.0034999999999999996;
    } else {
      for (ruleID = 0; ruleID < 51; ruleID++) {
        y += b_x[ruleID] * arr[ruleID];
      }
      y *= 1.0 / aggVal;
    }
  }
  return y;
}

/*
 * File trailer for evaluatefis.c
 *
 * [EOF]
 */
