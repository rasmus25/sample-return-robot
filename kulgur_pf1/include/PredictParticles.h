#pragma once

#include "Particles.h"

void PredictParticlesAddNoise(Particles* inout_particles, 
	const Eigen::Vector4d& wheelRotations, const Eigen::Vector2d& turnWheelAngles,
	bool AddNoise_Conf = true);

void PredictParticles(Particles* inout_particles, const Eigen::Vector3d& poseChange);

void AddNoise(Particles* inout_particles, const Eigen::Matrix<double, 3, 2>& predictionJacobian, 
	const Eigen::Vector4d& wheelRotations, const Eigen::Vector2d& turnWheelAngles);


