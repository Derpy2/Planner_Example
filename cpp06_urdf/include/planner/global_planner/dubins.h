#pragma once

namespace global_planner {

enum ErrorCode {
  EDUBOK = 0,
  EDUBCOCONFIGS = 1,
  EDUBPARAM = 2,
  EDUBBADRHO = 3,
  EDUBNOPATH = 4
};

struct DubinsPath {
  double qi[3];     // the initial configuration
  double param[3];  // the lengths of the three segments
  double rho;       // model forward velocity / model angular velocity
  int type;         // path type. one of LSL, LSR, ...
};

int dubins_init_normalised(double alpha, double beta, double d,
                           DubinsPath* path);

int dubins_init(double q0[3], double q1[3], double rho, DubinsPath* path);

double dubins_path_length(DubinsPath* path);

void dubins_segment(double t, double qi[3], double qt[3], int type);

int dubins_path_sample(DubinsPath* path, double t, double q[3]);

int dubins_LSL(double alpha, double beta, double d, double* outputs);
int dubins_RSR(double alpha, double beta, double d, double* outputs);
int dubins_LSR(double alpha, double beta, double d, double* outputs);
int dubins_RSL(double alpha, double beta, double d, double* outputs);
int dubins_LRL(double alpha, double beta, double d, double* outputs);
int dubins_RLR(double alpha, double beta, double d, double* outputs);

}  // namespace global_planner