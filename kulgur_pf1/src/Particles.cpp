#include "Particles.h"

using namespace Eigen;

void SetAllParticles(Particles* inout_particles, const Vector3d& state)
{
	(*inout_particles) << 
							MatrixXd::Constant(1, inout_particles->cols(), state[0]) , 
							MatrixXd::Constant(1, inout_particles->cols(), state[1]) ,
							MatrixXd::Constant(1, inout_particles->cols(), state[2]); 
}