#include <Meth.h> //Math

double deg2rad(double angle_d) {
  return ((double)angle_d) * ((double)PI) / ((double)180.0);
}

double rad2deg(double angle_r) {
  return ((double)angle_r) * ((double)180.0) / ((double)PI);
}

double logab(double a, double b) {
  return log(b) / log(a);
}
