#include "ParticleWeight.h"

#include <cmath>

using namespace Eigen;
using namespace kulgur1;
using namespace std;

void ParticleWeights(const MeasurementsVector& measurements, const Particles& particles, 
						const LandmarksVector& landmarks, const Eigen::Matrix2d& measurementCov, double lambda, 
						std::vector<double>* out_weights, std::vector<bool>* out_outliers)
{
	vector<vector<double> > weights(measurements.size());
	vector<bool> 			outliers(measurements.size());

	for(int m = 0 ; m < measurements.size() ; m++)
	{
		bool outlier;

		AssociateMeasurement(measurements[m], particles, landmarks, measurementCov, lambda,
			&outlier, &(weights[m]));

		outliers[m] = outlier;
	}

	out_weights->resize(particles.cols());

	bool allOutliers = true;
	for(auto it = outliers.begin() ; it != outliers.end() ; it++)
	{
		if(!*it) 
		{
			allOutliers = false;
			break;
		}
	}

	double weightsSum = 0;

	for(int p = 0 ; p < particles.cols() ; p++)
	{
		(*out_weights)[p] = 1;

		if(!allOutliers)
		{
			for(int m = 0 ; m < weights.size() ; m++)
			{
				if(outliers[m]) continue;			
				(*out_weights)[p] *= weights[m][p];
			}
		}

		weightsSum += (*out_weights)[p];
	}

	// Normalise weights so sum will equal 1.
	for(auto it = out_weights->begin() ; it != out_weights->end() ; it++ ) *it /= weightsSum;

	if(out_outliers != NULL) *out_outliers = outliers;
}

void AssociateMeasurement(const Eigen::Vector2d& measurement, const Particles& particles, 
	const LandmarksVector& landmarks, const Eigen::Matrix2d& measurementCov, double lambda, 
	bool* out_outlier, vector<double>* out_weights, std::vector<int>* out_lmIdxs)
{
	double landmarkLhSum = 0;

	out_weights->resize(particles.cols());

	if(out_lmIdxs != NULL) out_lmIdxs->resize(particles.cols());

	for(int p = 0 ; p < particles.cols() ; p++)
	{
		double maxLikelihood = -1;
		double maxIdx;

		for(int l = 0 ; l < landmarks.size() ; l++)
		{
			Vector2d mPred 		= MeasurementPrediction(particles.col(p), landmarks[0]);	
			double likelihood	= MeasurementLikelihood(measurement, particles.col(p), landmarks[l], measurementCov);

			if (likelihood > maxLikelihood)
			{
				maxLikelihood 	= likelihood;
				maxIdx 			= l;
			}
		}

		landmarkLhSum 		+= maxLikelihood;
		(*out_weights)[p] 	= maxLikelihood;

		if(out_lmIdxs != NULL) (*out_lmIdxs)[p] 	= maxIdx;
	}

	landmarkLhSum /= particles.cols();

	if (landmarkLhSum < lambda)
	{
		cout<<endl<<landmarkLhSum<<endl;
		assert(false);
		*out_outlier = true;
	}
	else
	{
		*out_outlier = false;
	}
}

Vector2d MeasurementPrediction(const Eigen::Vector3d& pose, const Eigen::Vector2d& landmark)
{
	// Euclidean vector
	Vector2d landmarkVec 	= (landmark - pose.block(0,0,2,1));

	double distance 		= landmarkVec.norm();
	double bearing  		= mrpt::math::wrapToPi(atan2(landmarkVec[1], landmarkVec[0]) - pose[2]);

	// return polar vector
	return Eigen::Vector2d(bearing, distance);
}

//

double MeasurementLikelihood(const Eigen::Vector2d& measurement, const Eigen::Vector3d& pose,
	const Eigen::Vector2d& landmark, const Eigen::Matrix2d& measurementCov)
{
	Vector2d predVec 	= MeasurementPrediction(pose, landmark);
	Vector2d nu 		= measurement - predVec;

	mrpt::math::wrapToPiInPlace(nu[0]);

	// cout<<endl<<"D"<<endl<<measurementCov<<endl<<endl;
	// cout<<endl<<"nu"<<endl<<nu<<endl<<endl;

	// See for example Thrun's book. p 217. line 16.
	double D 			= nu.transpose() * measurementCov.inverse() * nu;
	double w_unscaled 	= exp( -0.5 * D );
	double w_scaled 	= w_unscaled / ( 2 * M_PI * sqrt( measurementCov.det() ) );

	// cout<<endl<<"w_scaled "<<w_scaled<<endl<<endl;

	assert(0 <= w_scaled);

	return w_scaled;
}

// Just converts to vector [bearing, distance], not from polar to euclidean vec
const Eigen::Vector2d toVector2d(const kulgur1::LandmarkMeasurement& measurement)
{
	return Vector2d(measurement.bearing, measurement.distance);
}