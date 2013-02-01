#pragma once

#include <mrpt/base.h>

const int ParticleCount = 1000;
typedef Eigen::Matrix<double, 3, ParticleCount>  Particles;

void SetAllParticles(Particles* inout_particles, const Eigen::Vector3d& state);