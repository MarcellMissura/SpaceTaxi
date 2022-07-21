#ifndef IMU_H_
#define IMU_H_

#include "util/Vec2.h"
#include "util/Vec3.h"

class IMU
{

public:

	IMU();
    ~IMU(){}

	Vec2 accVinG;
	Vec2 rawAngleRate;
    Vec2 rawAngleRateV2;
	Vec2 rawAccAngle;

	Vec2 accAngle;
	Vec2 lastAccAngle;
	Vec2 angleRate;
	Vec2 lastAngleRate;
	Vec2 biaslessGyroAngle;

	Vec2 gyroBias;
	double calibrationLambda;

	Vec2 fusedTrunkAngle;
	Vec2 lastFusedAngle;
	Vec2 fusedAngle;
	Vec2 DfusedAngle;

    Vec2 update(Vec3 accelerometer, Vec3 angleRateIn);
	void calibrate();
};

#endif
