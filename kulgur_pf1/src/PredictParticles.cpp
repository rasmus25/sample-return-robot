#include "PredictParticles.h"

#include "KulgurOdometry.h"

#include <random>

using namespace std;
using namespace Eigen;

//
//
//

default_random_engine generator;

void AddNoise(Particles* inout_particles, const Matrix<double, 3, 2>& predictionJacobian, 
	const Vector4d& wheelRotations, const Vector2d& turnWheelAngles)
{
	// TODO: refactor these constants
	double turnNoise 		= 2.0 * M_PI / 180;
	double rotationsNoise 	= 0.1 * KulgurOdometry::WheelRadius * (wheelRotations[0] + wheelRotations[1]) / 2;

	Particles noiseMatrix;

	normal_distribution<double> turnNoiseDist(0, turnNoise);
	normal_distribution<double> rotNoiseDist(0, rotationsNoise);

	for (int i = 0; i < noiseMatrix.cols() ; ++i)
	{
		Vector3d noiseVec 	= predictionJacobian * Vector2d(0, turnNoiseDist(generator));
		noiseMatrix.col(i) 	= noiseVec;

		cout<<noiseVec<<endl<<endl;
	}

	*inout_particles += noiseMatrix;
}

//
//
//

void PredictParticles(Particles* inout_particles, const Vector3d& poseChange)
{
	for (int i = 0; i < inout_particles->cols() ; ++i)
	{
		double currentAng = (*inout_particles)(2,i);

		Vector2d particlePoseChange 	= Rotation2D<double>(currentAng) * poseChange.block(0,0,2,1);

		inout_particles->block(0,i,2,1) += particlePoseChange;
		(*inout_particles)(2,i) 		+= poseChange[2];
	}
}

//
//
//

void PredictParticlesAddNoise(Particles* inout_particles, 
	const Vector4d& wheelRotations, const Vector2d& turnWheelAngles,
	bool AddNoise_Conf)
{
	// TODO: Make necessary methods static
	KulgurOdometry odometry;

	// TODO: refactor these constants
	const double turnNoise 		= 20 * M_PI / 180;
	const double rotationsNoise = 0.2 * KulgurOdometry::WheelRadius * (wheelRotations[0] + wheelRotations[1]) / 2;

	//
	Matrix<double, 3, 2> predictionJacobian = odometry.PredictJacobian(wheelRotations, turnWheelAngles);
	normal_distribution<double> turnNoiseDist(0, turnNoise);
	normal_distribution<double> rotNoiseDist(0, rotationsNoise);

	Vector3d poseChange = odometry.PoseChange(wheelRotations, turnWheelAngles);

	for (int i = 0; i < inout_particles->cols() ; ++i)
	{
		//
		double currentAng = (*inout_particles)(2,i);
		Vector3d noiseVec(0,0,0);

		if(AddNoise_Conf)
		{
			noiseVec = predictionJacobian * Vector2d(rotNoiseDist(generator), turnNoiseDist(generator));

			// cout<<endl<<noiseVec<<endl;
		}

		Vector2d particlePosChange 		= Rotation2D<double>(currentAng) * (poseChange.block(0,0,2,1) + noiseVec.block(0,0,2,1));

		inout_particles->block(0,i,2,1) += particlePosChange;
		(*inout_particles)(2,i) 		+= poseChange[2] + noiseVec[2];
	}	
}