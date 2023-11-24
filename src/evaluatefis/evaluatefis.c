/*
 * File: evaluatefis.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 24-Nov-2023 19:23:44
 */

/* Include Files */
#include "evaluatefis.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
double evaluatefis(const double x[3])
{
  static const double outputMFCache[255] = {1.0,
                                            0.018459638322121712,
                                            1.1611618288729833E-7,
                                            2.4889071146549008E-16,
                                            1.8179042415601171E-28,
                                            0.97477375670805777,
                                            0.034082489222870896,
                                            4.0607397717839452E-7,
                                            1.6486383099571136E-15,
                                            2.2808251398248632E-27,
                                            0.90284939966747046,
                                            0.059792550806940174,
                                            1.3493518978624083E-6,
                                            1.037647257534328E-14,
                                            2.7190715531105954E-26,
                                            0.79457419323094458,
                                            0.099671389611528577,
                                            4.2604252131495561E-6,
                                            6.2055710802444325E-14,
                                            3.0800441244004651E-25,
                                            0.664448391501802,
                                            0.1578707244816685,
                                            1.2781691316044015E-5,
                                            3.5263175877705049E-13,
                                            3.3151325444408694E-24,
                                            0.52795354462114275,
                                            0.23759666890734074,
                                            3.643605596507604E-5,
                                            1.9040079120264234E-12,
                                            3.3904122800946686E-23,
                                            0.39860048467575288,
                                            0.3397713454599105,
                                            9.8692023351050717E-5,
                                            9.76840666050445E-12,
                                            3.2946689628102938E-22,
                                            0.28594834787033507,
                                            0.46167973193424017,
                                            0.00025400391000825667,
                                            4.7619668765211332E-11,
                                            3.04213661856843E-21,
                                            0.19491489023739239,
                                            0.596077220278889,
                                            0.00062116418593593777,
                                            2.2057519209882776E-10,
                                            2.6690290435966359E-20,
                                            0.1262438121965038,
                                            0.73126013435786064,
                                            0.0014433779641588121,
                                            9.70810844183055E-10,
                                            2.2250283167234582E-19,
                                            0.077693169719145777,
                                            0.85241076848238706,
                                            0.0031868482966047615,
                                            4.0599457306097041E-9,
                                            1.7624850501663644E-18,
                                            0.0454321444541942,
                                            0.94413394835986975,
                                            0.0066857535965923007,
                                            1.6132939112299555E-8,
                                            1.3265480886332783E-17,
                                            0.02524359999240439,
                                            0.99363288669193539,
                                            0.013327448094988132,
                                            6.09136211451586E-8,
                                            9.486984942979549E-17,
                                            0.013327448094988145,
                                            0.9936328866919355,
                                            0.025243599992404359,
                                            2.1853600410382515E-7,
                                            6.4467537449166888E-16,
                                            0.0066857535965923007,
                                            0.94413394835986975,
                                            0.0454321444541942,
                                            7.4497077280841463E-7,
                                            4.1625705561402795E-15,
                                            0.0031868482966047615,
                                            0.85241076848238706,
                                            0.077693169719145777,
                                            2.41303226019613E-6,
                                            2.5538171048727022E-14,
                                            0.0014433779641588134,
                                            0.73126013435786075,
                                            0.12624381219650369,
                                            7.4266794350093771E-6,
                                            1.4887632226843749E-13,
                                            0.00062116418593593777,
                                            0.59607722027888921,
                                            0.19491489023739236,
                                            2.1718702957588506E-5,
                                            8.2464898521275185E-13,
                                            0.00025400391000825759,
                                            0.46167973193424033,
                                            0.28594834787033496,
                                            6.0350482907401956E-5,
                                            4.3403053365127893E-12,
                                            9.8692023351050717E-5,
                                            0.3397713454599105,
                                            0.39860048467575288,
                                            0.00015934383682965789,
                                            2.1705964757129367E-11,
                                            3.643605596507604E-5,
                                            0.23759666890734074,
                                            0.52795354462114275,
                                            0.00039975826160692758,
                                            1.0314437795591635E-10,
                                            1.2781691316044083E-5,
                                            0.15787072448166867,
                                            0.66444839150180157,
                                            0.00095294378553551623,
                                            4.65714420119707E-10,
                                            4.2604252131495561E-6,
                                            0.099671389611528619,
                                            0.79457419323094436,
                                            0.002158463819700222,
                                            1.9980275236869972E-9,
                                            1.3493518978624083E-6,
                                            0.059792550806940174,
                                            0.90284939966747058,
                                            0.0046454727977842461,
                                            8.1449965456885138E-9,
                                            4.0607397717839452E-7,
                                            0.034082489222870944,
                                            0.97477375670805766,
                                            0.00949998076847594,
                                            3.1549174427449667E-8,
                                            1.1611618288729833E-7,
                                            0.018459638322121712,
                                            1.0,
                                            0.018459638322121712,
                                            1.1611618288729833E-7,
                                            3.1549174427449667E-8,
                                            0.00949998076847594,
                                            0.97477375670805766,
                                            0.034082489222870944,
                                            4.0607397717839452E-7,
                                            8.1449965456885138E-9,
                                            0.0046454727977842461,
                                            0.90284939966747058,
                                            0.059792550806940174,
                                            1.3493518978624083E-6,
                                            1.9980275236869972E-9,
                                            0.002158463819700222,
                                            0.79457419323094436,
                                            0.099671389611528619,
                                            4.2604252131495561E-6,
                                            4.65714420119707E-10,
                                            0.00095294378553551623,
                                            0.66444839150180157,
                                            0.15787072448166867,
                                            1.2781691316044083E-5,
                                            1.0314437795591635E-10,
                                            0.00039975826160692758,
                                            0.52795354462114275,
                                            0.23759666890734074,
                                            3.643605596507604E-5,
                                            2.1705964757129367E-11,
                                            0.00015934383682965789,
                                            0.39860048467575288,
                                            0.3397713454599105,
                                            9.8692023351050717E-5,
                                            4.3403053365127893E-12,
                                            6.0350482907401956E-5,
                                            0.28594834787033496,
                                            0.46167973193424033,
                                            0.00025400391000825759,
                                            8.2464898521275185E-13,
                                            2.1718702957588506E-5,
                                            0.19491489023739236,
                                            0.59607722027888921,
                                            0.00062116418593593777,
                                            1.4887632226843749E-13,
                                            7.4266794350093771E-6,
                                            0.12624381219650369,
                                            0.73126013435786075,
                                            0.0014433779641588134,
                                            2.5538171048727022E-14,
                                            2.41303226019613E-6,
                                            0.077693169719145777,
                                            0.85241076848238706,
                                            0.0031868482966047615,
                                            4.1625705561402795E-15,
                                            7.4497077280841463E-7,
                                            0.0454321444541942,
                                            0.94413394835986975,
                                            0.0066857535965923007,
                                            6.4467537449166888E-16,
                                            2.1853600410382515E-7,
                                            0.025243599992404359,
                                            0.9936328866919355,
                                            0.013327448094988145,
                                            9.486984942979549E-17,
                                            6.09136211451586E-8,
                                            0.013327448094988132,
                                            0.99363288669193539,
                                            0.02524359999240439,
                                            1.3265480886332783E-17,
                                            1.6132939112299555E-8,
                                            0.0066857535965923007,
                                            0.94413394835986975,
                                            0.0454321444541942,
                                            1.7624850501663644E-18,
                                            4.0599457306097041E-9,
                                            0.0031868482966047615,
                                            0.85241076848238706,
                                            0.077693169719145777,
                                            2.2250283167234582E-19,
                                            9.70810844183055E-10,
                                            0.0014433779641588121,
                                            0.73126013435786064,
                                            0.1262438121965038,
                                            2.6690290435966359E-20,
                                            2.2057519209882776E-10,
                                            0.00062116418593593777,
                                            0.596077220278889,
                                            0.19491489023739239,
                                            3.04213661856843E-21,
                                            4.7619668765211332E-11,
                                            0.00025400391000825667,
                                            0.46167973193424017,
                                            0.28594834787033507,
                                            3.2946689628102938E-22,
                                            9.76840666050445E-12,
                                            9.8692023351050717E-5,
                                            0.3397713454599105,
                                            0.39860048467575288,
                                            3.3904122800946686E-23,
                                            1.9040079120264234E-12,
                                            3.643605596507604E-5,
                                            0.23759666890734074,
                                            0.52795354462114275,
                                            3.3151325444408694E-24,
                                            3.5263175877705049E-13,
                                            1.2781691316044015E-5,
                                            0.1578707244816685,
                                            0.664448391501802,
                                            3.0800441244004651E-25,
                                            6.2055710802444325E-14,
                                            4.2604252131495561E-6,
                                            0.099671389611528577,
                                            0.79457419323094458,
                                            2.7190715531105954E-26,
                                            1.037647257534328E-14,
                                            1.3493518978624083E-6,
                                            0.059792550806940174,
                                            0.90284939966747046,
                                            2.2808251398248632E-27,
                                            1.6486383099571136E-15,
                                            4.0607397717839452E-7,
                                            0.034082489222870896,
                                            0.97477375670805777,
                                            1.8179042415601171E-28,
                                            2.4889071146549008E-16,
                                            1.1611618288729833E-7,
                                            0.018459638322121712,
                                            1.0};
  static const double b_x[51] = {-0.02,
                                 -0.019200000000000002,
                                 -0.0184,
                                 -0.0176,
                                 -0.016800000000000002,
                                 -0.016,
                                 -0.0152,
                                 -0.014400000000000001,
                                 -0.013600000000000001,
                                 -0.0128,
                                 -0.012,
                                 -0.0112,
                                 -0.010400000000000001,
                                 -0.0096000000000000009,
                                 -0.0088,
                                 -0.008,
                                 -0.0072000000000000007,
                                 -0.0064,
                                 -0.0056,
                                 -0.0048000000000000004,
                                 -0.004,
                                 -0.0032,
                                 -0.0024000000000000002,
                                 -0.0016,
                                 -0.0008,
                                 0.0,
                                 0.0008,
                                 0.0016,
                                 0.0024000000000000002,
                                 0.0032,
                                 0.004,
                                 0.0048000000000000004,
                                 0.0056,
                                 0.0064,
                                 0.0072000000000000007,
                                 0.008,
                                 0.0088,
                                 0.0096000000000000009,
                                 0.010400000000000001,
                                 0.0112,
                                 0.012,
                                 0.0128,
                                 0.013600000000000001,
                                 0.014400000000000001,
                                 0.0152,
                                 0.016,
                                 0.016800000000000002,
                                 0.0176,
                                 0.0184,
                                 0.019200000000000002,
                                 0.02};
  static const signed char iv[72] = {
      1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4,
      1, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2,
      1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3};
  static const signed char iv1[24] = {2, 4, 5, 5, 2, 3, 4, 4, 2, 4, 5, 5,
                                      2, 3, 4, 4, 2, 3, 4, 4, 2, 3, 4, 5};
  double orr[1224];
  double arr[51];
  double dv[9];
  double aggVal;
  double d;
  double ex;
  double sw;
  double y;
  int i;
  int idx;
  int k;
  int ruleID;
  boolean_T exitg1;
  dv[0] = exp(-((x[0] - -5.0) * (x[0] - -5.0)) / 56.349728);
  dv[1] = exp(-(x[0] * x[0]) / 56.349728);
  dv[2] = exp(-((x[0] - 15.0) * (x[0] - 15.0)) / 56.349728);
  dv[3] = exp(-((x[0] - 30.0) * (x[0] - 30.0)) / 56.349728);
  dv[4] = exp(-(x[1] * x[1]) / 0.010019616799999999);
  dv[5] = exp(-((x[1] - 0.2) * (x[1] - 0.2)) / 0.010019616799999999);
  dv[6] = exp(-((x[2] - -0.04) * (x[2] - -0.04)) / 0.030702419999999998);
  dv[7] = exp(-((x[2] - -0.28) * (x[2] - -0.28)) / 0.030702419999999998);
  dv[8] = exp(-((x[2] - -0.607) * (x[2] - -0.607)) / 0.030702419999999998);
  sw = 0.0;
  for (ruleID = 0; ruleID < 24; ruleID++) {
    double dv1[3];
    dv1[0] = dv[iv[ruleID] - 1];
    dv1[1] = dv[iv[ruleID + 24] + 3];
    dv1[2] = dv[iv[ruleID + 48] + 5];
    if (!rtIsNaN(dv1[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= 3)) {
        if (!rtIsNaN(dv1[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = dv1[0];
    } else {
      ex = dv1[idx - 1];
      i = idx + 1;
      for (k = i; k < 4; k++) {
        d = dv1[k - 1];
        if (ex > d) {
          ex = d;
        }
      }
    }
    sw += ex;
    for (k = 0; k < 51; k++) {
      d = outputMFCache[(iv1[ruleID] + 5 * k) - 1];
      idx = k + 51 * ruleID;
      orr[idx] = d;
      if (d > ex) {
        orr[idx] = ex;
      } else {
        orr[idx] = d;
      }
    }
  }
  for (k = 0; k < 51; k++) {
    aggVal = orr[k];
    for (idx = 0; idx < 23; idx++) {
      d = orr[k + 51 * (idx + 1)];
      if ((aggVal < d) || rtIsNaN(aggVal)) {
        aggVal = d;
      }
    }
    arr[k] = aggVal;
  }
  if (sw == 0.0) {
    y = 0.0;
  } else {
    if (!rtIsNaN(arr[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 52)) {
        if (!rtIsNaN(arr[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = arr[0];
      idx = 1;
    } else {
      ex = arr[idx - 1];
      i = idx + 1;
      for (k = i; k < 52; k++) {
        d = arr[k - 1];
        if (ex < d) {
          ex = d;
          idx = k;
        }
      }
    }
    y = b_x[idx - 1];
    aggVal = fabs(y);
    for (idx = 0; idx < 51; idx++) {
      if (arr[idx] == ex) {
        d = b_x[idx];
        sw = fabs(d);
        if (sw < aggVal) {
          aggVal = sw;
          y = d;
        }
      }
    }
  }
  return y;
}

/*
 * File trailer for evaluatefis.c
 *
 * [EOF]
 */