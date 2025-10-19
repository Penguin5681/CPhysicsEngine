//
// Created by penguin on 10/17/25.
//

#pragma once

#ifndef CPHYSICSENGINE_RIGIDBODY_H
#define CPHYSICSENGINE_RIGIDBODY_H

#include "../Core/Vector3.h"
#include "../Core/Quaternion.h"
#include "../Core/Matrix3x3.h"
#include "../Collision/BoundingSphere.h"

class RigidBody {
public:
	float inverseMass{};
	Vector3 position;
	Vector3 velocity;
	Vector3 acceleration;

	Quaternion orientation;
	Vector3 angularVelocity;
	Matrix3x3 inverseInertiaTensor;

	float restitution; // this will range between 0.0 to 1.0 inclusive

	Matrix3x3 rotationMatrix;

	CollisionShape* shape;

private:
	Vector3 accumulatedForces;
	Vector3 accumulatedTorque;

public:
	RigidBody() : orientation(1, 0, 0,0), shape(nullptr), restitution(0.7f) {}

	void addForce(const Vector3 force) {
		accumulatedForces += force;
	}

	void addTorque(const Vector3 torque) {
		accumulatedTorque += torque;
	}

	void clearAccumulatedForces() {
		accumulatedForces = Vector3(0, 0, 0);
	}

	void clearAccumulatedTorque() {
		accumulatedTorque = Vector3(0, 0, 0);
	}

	void clearAccumulators() {
		clearAccumulatedForces();
		clearAccumulatedTorque();
	}

	void addForceAtPoint(const Vector3 force, const Vector3 worldPoint) {
		this->addForce(force);

		// r is the lever-arm vector
		const Vector3 r = worldPoint - this->position;
		const Vector3 torque = r.cross(force);

		this->addTorque(torque);
	}

	// delta-time denotes change over the simulation step
	// In RigidBody.h

	void integrate(float dt) {
		if (inverseMass > 0.0f) {
			Vector3 angularAcceleration = inverseInertiaTensor.transform(accumulatedTorque);
			angularVelocity += angularAcceleration * dt;

			orientation.updateByAngularVelocity(angularVelocity, dt);
		}

		orientation.normalize();
		this->rotationMatrix.setOrientation(this->orientation);

		if (inverseMass <= 0.0f) {
			clearAccumulators();
			return;
		}

		acceleration = accumulatedForces * inverseMass;
		velocity += acceleration * dt;
		position += velocity * dt;

		clearAccumulators();
	}
};

#endif //CPHYSICSENGINE_RIGIDBODY_H
