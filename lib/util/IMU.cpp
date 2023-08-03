#include "IMU.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include <QDebug>

IMU::IMU()
{
	calibrationLambda = 0.0;
	gyroBias.x = 0;
	gyroBias.y = 0;
}

// Triggers the calibration procedure, where the gyro bias is estimated.
void IMU::calibrate()
{
	calibrationLambda = 0.0;
}

// Calculates the new state of the IMU.
Vec2 IMU::update(Vec3 angleIn, Vec3 angleRateIn)
{
	accAngle = angleIn;
    angleRate = angleRateIn;

	// Filter out suspicious signal jumps.
	if (angleRate.norm() > 4.0 or accAngle.norm() > 1.6)
	{
		accAngle = lastAccAngle;
		angleRate = lastAngleRate;
	}
	lastAccAngle = accAngle;
	lastAngleRate = angleRate;

	// Integrate the gyro rate without bias to a biasless angle estimation.
	biaslessGyroAngle += config.systemIterationTime * angleRate;
	state.biaslessGyroAngle = biaslessGyroAngle;
	state.accAngle = accAngle;

	// Correct the gyro rate with the current bias.
	angleRate -= gyroBias;

	// Estimate a new gyro bias.
	if (state.time > 3.0)
		calibrationLambda = qMin(1.0, calibrationLambda + config.systemIterationTime*0.33);
	if (state.size() >= 200)
	{
		// Check how much the angle changed in the last 200 ticks (approx 2.4 secs) according
		// to the accelerometers and according to the unbiased integrated gyro rate
		// and take the difference. That's gonna be our new gyro bias.
		Vec2 gyro_delta_V = state.biaslessGyroAngle - state[200].biaslessGyroAngle;
		Vec2 acc_delta_V = state.accAngle - state[200].accAngle;
		Vec2 angle_bias_delta_V = (gyro_delta_V - acc_delta_V) / (200*config.systemIterationTime);

		// Integrate the difference of the two sensors slowly into the gyro bias.
		gyroBias += (0.001 + (1.0 - calibrationLambda) * 0.02) * (angle_bias_delta_V - gyroBias);
	}

	// Integrate (fuse) the angle reported by the accelerometers and the gyro rate into the current angle estimation.
	fusedTrunkAngle = calibrationLambda * (0.99*(fusedTrunkAngle + config.systemIterationTime * angleRate) + 0.01*accAngle)
			+ (1.0 - calibrationLambda) * (fusedTrunkAngle + 0.5*(accAngle-fusedTrunkAngle));

	// There is a systematic error to correct.
	lastFusedAngle = fusedAngle;
	fusedAngle.x = fusedTrunkAngle.x + config.fusedAngleCorrectionX;
	fusedAngle.y = fusedTrunkAngle.y + config.fusedAngleCorrectionY;

	DfusedAngle = angleRate;
	//DfusedAngle = (fusedAngle - lastFusedAngle)/config.systemIterationTime;

	return fusedAngle;
}
