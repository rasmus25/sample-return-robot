#pragma once

#include "kulgur_msgs/LandmarkMeasurementArray.h"

#include "Particles.h"

#include <mrpt/base.h>
#include <Eigen/StdVector>

Eigen::Vector2d MeasurementPrediction(const Eigen::Vector3d& pose, const Eigen::Vector2d& landmark);

double MeasurementLikelihood(const Eigen::Vector2d& measurement, const Eigen::Vector3d& pose,
	const Eigen::Vector2d& landmark, const Eigen::Matrix2d& measurementCov);

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > LandmarksVector;

//
// measurement - [bearing; distance]
// particles
// landmarks - vector<Vector2d>{[x1,y1],[x2,y2],...}
// measurementCov - Matrix2d [sigm_phiphi sigm_phidist; sigma_distphi sigma_distdist]
// lambda - if average likelihood of this measurement for all particles is less than lambda, measurement is outlier. Scaled inversely with covariance.
// ...
// out_outlier - true if the measurement likelihood is below threshold lambda
// out_weights - likelihood of a measurement(lmIdxs) for all particles
// out_lmIdxs - most likely landmark idx for each particle
//
void AssociateMeasurement(const Eigen::Vector2d& measurement, const Particles& particles, 
	const LandmarksVector& landmarks, const Eigen::Matrix2d& measurementCov, double lambda, 
	bool* out_outlier, std::vector<double>* out_weights, std::vector<int>* out_lmIdxs = NULL);

//
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > MeasurementsVector;

//
//
//
void ParticleWeights(const MeasurementsVector& measurements, const Particles& particles, 
						const LandmarksVector& landmarks, const Eigen::Matrix2d& measurementCov, double lambda, 
						std::vector<double>* out_weights, std::vector<bool>* out_outliers = NULL);
