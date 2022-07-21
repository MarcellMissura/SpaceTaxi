#include "KalmanFilter.h"
#include <QDebug>

KalmanFilter::KalmanFilter()
{
	lastX = 0;
	lastV = 0;
	smoothing = 0.1;
	timeStep = 0.01;
	x = 0;
	v = 0;
	a = 0;
	px = 0;
	pv = 0;
	pa = 0;

	// When the gain is adaptive, the smoothing config parameters don't make any difference.
	adaptiveGain = true;

	init();
}

// Prepares (resets) the noise matrices and the kalman gain, but preserves the current state.
// This is good for changing the smoothing on the fly without messing with the state estimate.
void KalmanFilter::init()
{
	// Process noise.
	Q = Mat<double>(3,3);
	Q << 1.0 << 0.0 << 0.0 << endr
	  << 0.0 << 1.0 << 0.0 << endr
	  << 0.0 << 0.0 << 1.0 << endr;

	// Measurement noise.
	R = Mat<double>(3,3);
	R << 0.001 + smoothing*100.0 << 0.0 << 0.0 << endr
	  << 0.0 << 0.001 + smoothing*100.0 << 0.0 << endr
	  << 0.0 << 0.0 << 0.001 + smoothing*100.0 << endr;

	// Passive linear system dynamics matrix A.
	A = Mat<double>(3,3);
	A << 1.0 << timeStep << 0.0 << endr
	  << 0.0 << 1.0 << timeStep << endr
	  << 0.0 << 0.0 << 1.0 << endr;

	// State and measurement vectors.
	X = Col<double>(3);
	Z = Col<double>(3);

	H = eye(3,3);
	I = eye(3,3);
	K = eye(3,3);
	P = eye(3,3);

	K = eye(3,3);
	P = zeros(3,3);

	X(0) = x;
	X(1) = v;
	X(2) = a;
}

// Sets the smoothing parameter.
void KalmanFilter::setSmoothing(double s)
{
	smoothing = s;
	init();
}

// Sets the time interval between the updates. It's set to 10 milliseconds by default.
void KalmanFilter::setTimeStep(double t)
{
	timeStep = t;
	init();
}

// Resets the state to the provided parameters, but doesn't change the noise and the kalman gain.
// This is good to set the state estimate on the fly without messing with the Kalman gain.
void KalmanFilter::reset(double xx, double vv, double aa)
{
	X(0) = xx;
	X(1) = vv;
	X(2) = aa;

	x = X(0);
	v = X(1);
	a = X(2);

	lastX = xx;
	lastV = vv;
}

// Updates the process. Provide a position measurement z.
void KalmanFilter::update(double z)
{
	Z(0) = z;
	Z(1) = (z - lastX) / timeStep;
	Z(2) = (Z(1) - lastV) / timeStep;

	if (adaptiveGain)
	{
		X = A*X;
		P = A*P*trans(A) + Q;
		K = P*trans(H) * inv(H*P*trans(H) + R);
		X = X + K*(Z - H*X);
		P = (I - K*H)*P;
	}
	else
	{
		X = A*X;
		K = Q*trans(H) * inv(H*Q*trans(H) + R);
		X = X + K*(Z - H*X);
	}

	lastX = Z(0);
	lastV = Z(1);

	x = X(0);
	v = X(1);
	a = X(2);

	px = P(0,0);
	pv = P(1,1);
	pa = P(2,2);
}
