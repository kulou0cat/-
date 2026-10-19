#pragma once

#include <Eigen/Dense>

struct SpringChain
{
	double mass = 1.0;
	double damping = 0.4;
	Eigen::MatrixXd K;
	Eigen::VectorXd x;
	Eigen::VectorXd v;
	Eigen::VectorXd externalForce;

	explicit SpringChain(int count = 5, double stiffness = 800.0) {
		K = Eigen::MatrixXd::Zero(count, count);
		x = Eigen::VectorXd::Zero(count);
		v = Eigen::VectorXd::Zero(count);
		externalForce = Eigen::VectorXd::Constant(count, mass * 9.81);

		K(0, 0) += stiffness;
		for (int i = 1; i < count; ++i) {
			K(i - 1, i - 1) += stiffness;
			K(i, i) += stiffness;
			K(i - 1, i) -= stiffness;
			K(i, i - 1) -= stiffness;
		}
	}


	void stepSemiImplicit(double dt) {
		Eigen::VectorXd force = externalForce - damping * v - K * x;
		v += dt * force / mass;
		x += dt * v;
	}

	void stepBackwardEuler(double dt) {
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x.size(), x.size());
		Eigen::MatrixXd A = (mass + dt * damping) * I + dt * dt * K;
		Eigen::VectorXd b = (mass * v) + dt * (externalForce - K * x);
		v = A.ldlt().solve(b);
		x += v * dt;
	}

	double energy() const {
		Eigen::VectorXd equilibrium = K.ldlt().solve(externalForce);
		Eigen::VectorXd offset = x - equilibrium;
		return 0.5 * mass * v.squaredNorm() + 0.5 * offset.dot(K * offset);
	}
};