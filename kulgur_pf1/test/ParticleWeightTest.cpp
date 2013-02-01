#include <iostream>

#include <gtest/gtest.h>

#include "ParticleWeight.h"

using namespace Eigen;
using namespace kulgur1;
using namespace std;

TEST(TestMeasurementLikelihood, DISABLED_testDiffParticleAssoc)
{
	//
	// Two different particles see the same landmark the same way.
	// Weights and associations should be the same
	// 
	Particles particles;

	Matrix2d cov;
	cov << 	0.1, 0,
			0, 1.0;

	double lambda = 0.0001;	

	LandmarksVector lms;
	lms.push_back(Vector2d(3, 1));
	lms.push_back(Vector2d(2, 2));		// This is the observed lm
	lms.push_back(Vector2d(0, -1));

	Vector3d pose1(1, 1, M_PI/4);
	Vector3d pose2(3, 3, 3*M_PI/4);

	particles.col(2) = pose1;
	particles.col(5) = pose2;

	Vector2d m1 = MeasurementPrediction(pose1, lms[1] + Vector2d(0.1, 0.1));
	Vector2d m2 = MeasurementPrediction(pose2, lms[1] + Vector2d(0.1, 0.1));

	bool outlier1, outlier2;
	vector<double> weights1, weights2;
	vector<int> idxs1, idxs2;

	AssociateMeasurement(m1, particles, lms, cov, lambda, &outlier1, &weights1, &idxs1);
	AssociateMeasurement(m2, particles, lms, cov, lambda, &outlier2, &weights2, &idxs2);

	ASSERT_EQ(idxs1[2], 1);
	ASSERT_EQ(idxs1[2], idxs2[5]);
	ASSERT_NEAR(weights1[2], weights2[5], 1e-6);

	//
	// Two different particles see two different landmarks the same way.
	// Weights should be the same, false association weights should be smalled, associations should be correct
	//
	Vector3d pose0(1.2, 1.2, 0.9 * M_PI);

	pose1 = Vector3d(1, 1, M_PI);
	pose2 = Vector3d(-1, -1, -M_PI/2);

	particles.col(0) = pose0;
	particles.col(1) = pose1;
	particles.col(2) = pose2;

	lms[1] = Vector2d(0, 1);
	lms[2] = Vector2d(-1, -2);

	m1 = Vector2d(0, 0.99);
	m2 = Vector2d(0, 1.02);

	AssociateMeasurement(m1, particles, lms, cov, lambda, &outlier1, &weights1, &idxs1);
	AssociateMeasurement(m2, particles, lms, cov, lambda, &outlier2, &weights2, &idxs2);

	// Check correct associations
	ASSERT_EQ(idxs1[1], 1);
	ASSERT_EQ(idxs2[2], 2);

	// Check that weights are similar
	ASSERT_NEAR(weights1[1], weights2[2], 1e-3);

	// Check that other particles don't get better weights
	ASSERT_TRUE(weights1[1] > weights1[0]);
	ASSERT_TRUE(weights2[2] > weights2[0]);
}

TEST(TestMeasurementLikelihood, DISABLED_testAssociation_ALOT)
{
	Particles 	particles;
	Vector3d	pose;

	Matrix2d cov;
	cov << 	0.1, 0,
			0, 1.0;

	double lambda = 0.0001;

	LandmarksVector lms;
	lms.push_back(Vector2d(3, 1));
	lms.push_back(Vector2d(-1, 0));
	lms.push_back(Vector2d(0, -1));

	bool _50percReported = false;

	for(double x = -5 ; x < 5 ; x += 0.5)
	{
		if(!_50percReported && x > 0)
		{
			cout<<"50 perc done"<<endl;
			_50percReported = true;
		}

		for (double y = -5 ; y < 5 ; y += 0.5)
		{
			for(double phi = -2*M_PI+1e-3 ; phi < 2*M_PI ; phi += 0.6)
			{
				pose = Vector3d(x, y, phi);
				SetAllParticles(&particles, pose);

				for(int l = 0 ; l < lms.size() ; l++)
				{
					Vector2d lmPlusNoise = lms[l] + Vector2d(0.01, 0.01);
					Vector2d measurement = MeasurementPrediction(pose, lmPlusNoise);

					bool outlier;
					vector<double> weights;
					vector<int> idxs;

					AssociateMeasurement(measurement, particles, lms, cov, lambda, &outlier, &weights, &idxs);

					// if(outlier)
					// {
					// 	cout<<"Pose"<<endl<<pose<<endl<<endl;
					// 	cout<<"LmPlusNoise"<<endl<<lmPlusNoise<<endl<<endl;
					// 	cout<<"Measurement"<<endl<<measurement<<endl<<endl;
					// 	cout<<"Exact measurement"<<endl<<MeasurementPrediction(pose, lms[l])<<endl<<endl;
					// }

					ASSERT_FALSE(outlier);
					ASSERT_EQ(l, idxs[0]);
				}
			}
		}
	}
}

TEST(TestMeasurementLikelihood, DISABLED_testAssociation)
{
	Particles 	particles;
	Vector3d 	pose(0,0,0);

	SetAllParticles(&particles, pose);

	Matrix2d cov;
	cov << 	0.1, 0,
			0, 1.0;

	LandmarksVector lms;
	lms.push_back(Vector2d(3, 0));
	lms.push_back(Vector2d(1, 0));
	lms.push_back(Vector2d(0, 1));

	Vector2d measurement;

	double lambda = 0.3; // in lab2 code, it was 0.0001?!

	bool 			outlier;
	vector<double> 	weights;
	vector<int> 	lmIdxs;

	//
	measurement = Vector2d(0, 0.95);

	AssociateMeasurement(measurement, particles, lms, cov, lambda, &outlier, &weights, &lmIdxs);

	ASSERT_TRUE(lmIdxs[0] == 1);

	//
	// Creating measurements that are near true landmarks
	//
	for(int i = 0 ; i < lms.size() ; i++)
	{
		measurement = MeasurementPrediction(particles.col(0), lms[i] + Vector2d(0.1, 0.1));
		AssociateMeasurement(measurement, particles, lms, cov, lambda, &outlier, &weights, &lmIdxs);

		ASSERT_TRUE(lmIdxs[0] == i);
		ASSERT_FALSE(outlier);
	}

	//
	// Random distant measurement
	//
	measurement = Vector2d(0, 10);
	AssociateMeasurement(measurement, particles, lms, cov, lambda, &outlier, &weights, &lmIdxs);
	ASSERT_TRUE(outlier);
}

TEST(TestMeasurementLikelihood, DISABLED_testWeights)
{
	Vector3d pose(1, 1, M_PI/4);

	Matrix2d cov;

	cov << 	0.1, 0,
			0, 1.0;

	// True landmark pos
	Vector2d lm(-0.5, -0.5);

	// measurements
	Vector2d m1 = MeasurementPrediction(pose, Vector2d(1.0, 0));
	Vector2d m2 = MeasurementPrediction(pose, Vector2d(0, 0));
	Vector2d m3 = MeasurementPrediction(pose, Vector2d(-0.3, -0.3));

	double w1 = MeasurementLikelihood(m1, pose, lm, cov);
	double w2 = MeasurementLikelihood(m2, pose, lm, cov);
	double w3 = MeasurementLikelihood(m3, pose, lm, cov);

	ASSERT_TRUE(w1 < w2 && w2 < w3);

	// Test for correct measurement
	Vector2d m 	= MeasurementPrediction(pose, lm);

	ASSERT_NEAR(mrpt::math::wrapToPi(m[0] - M_PI), 0, 1e-3);

	double w 	= MeasurementLikelihood(m, pose, lm, cov);

	// Test that all measurements near the ideal have weight < w.
	for (double dx = -0.1 ; dx < 0.1 ; dx += 0.003) 		// NB! Step should not divide with initial dx, dy!
	{
		for(double dy = -0.1 ; dy < 0.1 ; dy += 0.003)
		{
			Vector2d testM = MeasurementPrediction(pose, lm + Vector2d(dx, dy));

			double testW = MeasurementLikelihood(testM, pose, lm, cov);

			ASSERT_TRUE(testW < w);
		}
	}
}

TEST(TestMeasurementLikelihood, DISABLED_testMeasurementLikelihoodBetween0and1)
{
	Vector3d pose(0, 0, 0);
	Vector2d lm(1, 1);
	Matrix2d cov;

	cov << 	0.1, 0,
			0, 1.0;

	//
	// Moving the true landmark around. Visible landmark is lm.
	//
	for (double x = -3; x < 3; x += 0.01)
	{
		for (double y = -3; y < 3 ; y += 0.01)
		{
			double w = MeasurementLikelihood(MeasurementPrediction(pose, Vector2d(x, y)), pose, lm, cov);

			ASSERT_TRUE(w > 0);
			ASSERT_TRUE(w <= 1);
		}
	}

	//
	// Moving the visible landmark around. True landmark is lm.
	//
	for (double x = -3; x < 3; x += 0.01)
	{
		for (double y = -3; y < 3 ; y += 0.01)
		{
			double w = MeasurementLikelihood(MeasurementPrediction(pose, lm), pose, Vector2d(x, y), cov);

			ASSERT_TRUE(w > 0);
			ASSERT_TRUE(w <= 1);
		}
	}

	//
	// Moving the robot around. 
	//

}


TEST(TestMeasurementLikelihood, DISABLED_testLandmarkPrediction)
{
	Vector3d pose(0, 0, 0);
	Vector2d lm(10, 0);

	// LandmarkMeasurement predLm = MeasurementPrediction(pose, lm);
	// ASSERT_DOUBLE_EQ(predLm.bearing, 0);

	// [0] - bearing
	// [1] - distance

	ASSERT_DOUBLE_EQ(MeasurementPrediction(pose, Vector2d(10, 0))[0], 0);
	ASSERT_DOUBLE_EQ(MeasurementPrediction(pose, Vector2d(10, 0))[1], 10);

	ASSERT_NEAR(MeasurementPrediction(pose, Vector2d(-10, 1e-6))[0], M_PI, 1e-3);
	ASSERT_NEAR(MeasurementPrediction(pose, Vector2d(-10, -1e-6))[0], -M_PI, 1e-3);

	ASSERT_NEAR(MeasurementPrediction(pose, Vector2d(-10, 1e-6))[1], 10, 1e-3);

	ASSERT_NEAR(MeasurementPrediction(pose, Vector2d(3, 4))[1], 5, 1e-3);

	// Testing wrapToPi
	pose = Vector3d(0, 10, -3*M_PI/4);
	ASSERT_NEAR(MeasurementPrediction(pose, Vector2d(0, 0))[0], M_PI/4, 1e-3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}