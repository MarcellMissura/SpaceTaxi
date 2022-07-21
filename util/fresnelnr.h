#ifndef FRESNELNR_H_
#define FRESNELNR_H_
#include <math.h>
#include <complex.h>
#include "globals.h"
#define EPS 6.0e-8
#define MAXIT 100
#define FPMIN 1.0e-30
#define ONE Complex(1.0,0.0)

typedef struct FCOMPLEX {float r,i;} fcomplex;

fcomplex Cadd(fcomplex a, fcomplex b);
fcomplex Csub(fcomplex a, fcomplex b);
fcomplex Cmul(fcomplex a, fcomplex b);
fcomplex Complex(float re, float im);
fcomplex Conjg(fcomplex z);
fcomplex Cdiv(fcomplex a, fcomplex b);
float Cabs(fcomplex z);
fcomplex Csqrt(fcomplex z);
fcomplex RCmul(float x, fcomplex a);
void fresnel(double x, double *s, double *c);

#endif /* FRESNELNR_H_ */
