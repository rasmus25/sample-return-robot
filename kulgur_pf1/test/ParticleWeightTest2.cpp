#include <iostream>

#include <gtest/gtest.h>

#include "ParticleWeight.h"

using namespace Eigen;
using namespace kulgur1;
using namespace std;

TEST(ParticleWeightTest2, DISABLED_Test2)
{
	Matrix2d cov;
	cov << 	0.1, 0,
			0, 0.5;

	double lambda = 0.01;	

	Particles particles;
	
	//
	// Generate a grid of particles
	//
	{

		int i = 0;
		for(int xi = 0 ; xi < 10 ; xi++)
		{
			double x = -11 + 2*xi;

			for(int yi = 0 ; yi < 10 ; yi++)
			{
				double y = -9 + 2*yi;

				for(int phii = 0 ; phii < 10 ; phii++)
				{
					double phi = -0.97*M_PI + phii*2*M_PI/10;

					assert(i < particles.cols());

					particles.col(i) = Vector3d(x, y, phi);
					i++;
				}
			}
		}

		ASSERT_TRUE(i == particles.cols());
	}

	//
	// Generate a grid of measurements
	//
	MeasurementsVector msGrid;

	for(int bi = 0 ; bi < 30 ; bi++)	
	{
		for(int di = 0 ; di < 30 ; di++)
		{
			msGrid.push_back(Vector2d(-M_PI + bi*2*M_PI/30, 0.13 + 0.57*di));
		}
	}

	LandmarksVector lms;
	lms.push_back(Vector2d(-7.8, 2.77));

	vector<double> 	weights;
	vector<bool> 	outliers;

	for(int m = 0 ; m < msGrid.size() ; m++)
	{
		MeasurementsVector mv;
		mv.push_back(msGrid[m]);

		ParticleWeights(mv, particles, lms, cov, lambda, &weights, &outliers);

		if(outliers[0]) continue; 	// If the measurement is outlier, then all particles have weight 1.

		double maxWeight = -1;
		int maxWeightP;

		for(int p = 0 ; p < weights.size() ; p++)
		{
			if(weights[p] > maxWeight)
			{
				maxWeight = weights[p];
				maxWeightP = p;
			}
		}

		Vector3d bestPose = particles.col(maxWeightP);

		auto MeasPred = [](Vector3d pose, Vector2d lm)
		{
			Vector2d meas;

			meas[0] = mrpt::math::wrapToPi(atan2(lm[1] - pose[1], lm[0] - pose[0]) - pose[2]);
			meas[1] = (Vector2d(pose[0], pose[1]) - lm).norm();

			return meas;
		};

		Vector2d bestMeasPred = MeasPred(bestPose, lms[0]);
		Vector2d bestMeasErrVec = (bestMeasPred - mv[0]);

		mrpt::math::wrapToPiInPlace(bestMeasErrVec[0]);

		Vector2d bestMeasErr(fabs(bestMeasErrVec[0]), fabs(bestMeasErrVec[1]));

		for(int p = 0 ; p < particles.cols() ; p++)
		{
			if(p == maxWeightP) continue;

			Vector2d mPred = MeasPred(particles.col(p), lms[0]);

			Vector2d measErrVec = (mPred - mv[0]);

			mrpt::math::wrapToPiInPlace(measErrVec[0]);

			Vector2d measErr(fabs(measErrVec[0]), fabs(measErrVec[1]));

			if(measErr[0] < bestMeasErr[0] && measErr[1] < bestMeasErr[1])
			{
				cout<<endl<<"Measurement"<<endl<<mv[0]<<endl;

				cout<<endl<<"Best particle weight "<<maxWeight<<endl<<endl;
				cout<<endl<<"Test particle weight "<<weights[p]<<endl<<endl;

				cout<<endl<<"Measurement pred of best particle"<<endl<<bestMeasPred<<endl;

				cout<<endl<<"Measurement pred of test particle"<<endl<<mPred<<endl;	

				cout<<endl<<"Measurement error on best particle"<<endl<<bestMeasErr<<endl;
				cout<<endl<<"Measurement error of test particle"<<endl<<measErr<<endl;
			}

			ASSERT_FALSE(measErr[0] < bestMeasErr[0] && measErr[1] < bestMeasErr[1]);
		}
	}
}

TEST(ParticleWeightTest2, AllWeightsTest1)
{
	Matrix2d cov;
	cov << 	0.1, 0,
			0, 0.5;

	double lambda = 0.01;

	Particles particles;

	SetAllParticles(&particles, Vector3d(0,0,0));

	particles.col(0) = Vector3d(0, 0, M_PI/4);
	particles.col(1) = Vector3d(1, 1, M_PI/4);
	particles.col(2) = Vector3d(2, 2, M_PI/4);

	LandmarksVector lms;

	lms.push_back(Vector2d(3, 3));
	lms.push_back(Vector2d(1, 0));

	MeasurementsVector mVec;

	mVec.push_back(Vector2d(M_PI/2, 3));  // This shouldnt match any landmark.
	mVec.push_back(Vector2d(0, sqrt(8))); // This should be correct measurement of particle 1 landmark 0.

	vector<double> 	weights;
	vector<bool> 	outliers;

	ParticleWeights(mVec, particles, lms, cov, lambda, &weights, &outliers);

	// Particle 1 should have the best weight
	ASSERT_TRUE(weights[0] < weights[1]);
	ASSERT_TRUE(weights[2] < weights[1]);

	ASSERT_FALSE(outliers[1]);
	ASSERT_TRUE(outliers[0]);

	double wSum = 0;

	for(auto it = weights.begin() ; it != weights.end() ; it++) wSum += *it;

	ASSERT_NEAR(wSum, 1.0, 1e-3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}