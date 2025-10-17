//
// Created by penguin on 10/17/25.
//

#pragma once

#ifndef CPHYSICSENGINE_RIGIDBODY_H
#define CPHYSICSENGINE_RIGIDBODY_H

#include "../Core/Vector3.h"

class RigidBody {
public:
	float inverseMass{};
	Vector3 position;
	Vector3 velocity;
	Vector3 acceleration;

private:
	Vector3 accumulatedForces;

public:
	RigidBody() = default;

	void addForce(Vector3 force) {
		accumulatedForces += force;
	}

	void clearAccumulatedForces() {
		accumulatedForces = Vector3(0, 0, 0);
	}

	// delta-time denotes change over the simulation step
	void integrate(float dt) {
		if (inverseMass <= 0.0) {
			return;
		}

		acceleration = accumulatedForces * inverseMass;
		velocity += acceleration * dt;
		position += velocity * dt;
		clearAccumulatedForces();
	}
};

#endif //CPHYSICSENGINE_RIGIDBODY_H
