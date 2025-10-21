//
// Created by penguin on 10/17/25.
//

#pragma once

#ifndef CPHYSICSENGINE_RIGIDBODY_H
#define CPHYSICSENGINE_RIGIDBODY_H

#include "../Collision/BoundingBox.h"
#include "../Core/Vector3.h"
#include "../Core/Quaternion.h"
#include "../Core/Matrix3x3.h"
#include "../Collision/BoundingSphere.h"

constexpr float sleepEpsilon = 0.01f;
constexpr float sleepDuration = 0.5f;

class RigidBody {
public:
	float inverseMass{};
	float restitution; // note: this will range between 0.0 to 1.0 (both inclusive)
	float sleepTimer;
	float motion;

	bool isAwake;

	Vector3 position;
	Vector3 velocity;
	Vector3 acceleration;
	Vector3 angularVelocity;
	Vector3 worldAABBMin;
	Vector3 worldAABBMax;

	Quaternion orientation;

	Matrix3x3 inverseInertiaTensor;
	Matrix3x3 rotationMatrix;

	CollisionShape* shape;

private:
	Vector3 accumulatedForces;
	Vector3 accumulatedTorque;

public:
	RigidBody() : restitution(0.7f), sleepTimer(0.0f), motion(FLT_MAX), isAwake(true), orientation(1, 0, 0, 0),
	              shape(nullptr) {}

	void setAwake(bool awake = true) {
		if (awake) {
			isAwake = true;
			sleepTimer = 0.0f;
			motion = FLT_MAX;
		} else {
			isAwake = false;
			velocity = Vector3(0, 0, 0);
			angularVelocity = Vector3(0, 0, 0);
			motion = 0.0f;
		}
	}

	void updateSleepState(float dt, float sleepEpsilonSq, float sleepDuration) {
		if (!isAwake) {
			return;
		}

		float currentMotionSq = velocity.magnitudeSquared() + angularVelocity.magnitudeSquared();

		if (currentMotionSq < sleepEpsilonSq) {
			sleepTimer += dt;

			if (sleepTimer >= sleepDuration) {
				setAwake(false);
			}
		} else {
			sleepTimer = 0.0f;
		}
	}

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


	// void integrate(float dt) {
	// 	if (inverseMass > 0.0f) {
	// 		Vector3 angularAcceleration = inverseInertiaTensor.transform(accumulatedTorque);
	// 		angularVelocity += angularAcceleration * dt;
	// 		orientation.updateByAngularVelocity(angularVelocity, dt);
	// 	}
	//
	// 	orientation.normalize();
	// 	this->rotationMatrix.setOrientation(this->orientation);
	//
	// 	updateWorldAABB();
	//
	// 	clearAccumulators();
	//
	// 	if (inverseMass <= 0.0f) {
	// 		return;
	// 	}
	//
	// 	acceleration = accumulatedForces * inverseMass;
	// 	velocity += acceleration * dt;
	// 	position += velocity * dt;
	//
	// 	updateWorldAABB();
	// }

	void integrate(float dt) {
		if (!isAwake) {
			return;
		}

		Vector3 linearAcceleration = Vector3(0, 0, 0);
		Vector3 angularAcceleration = Vector3(0, 0, 0);

		if (inverseMass > 0.0f) {
			linearAcceleration = accumulatedForces * inverseMass;
			angularAcceleration = inverseInertiaTensor.transform(accumulatedTorque);
		}

		if (inverseMass > 0.0f) {
			velocity += linearAcceleration * dt;
			angularVelocity += angularAcceleration * dt;
		}

		if (inverseMass > 0.0f) {
			position += velocity * dt;
			orientation.updateByAngularVelocity(angularVelocity, dt);
			orientation.normalize();
		}

		this->rotationMatrix.setOrientation(this->orientation);
		updateWorldAABB();

		clearAccumulators();
	}

private:
	void updateWorldAABB() {
		if (shape == nullptr) {
			worldAABBMin = position;
			worldAABBMax = position;
			return;
		}

		/*

		 *
		 */

		if (shape->getType() == SPHERE) {
			auto* sphere = (BoundingSphere*)shape;
			float r = sphere->radius;
			worldAABBMin = Vector3(position.x - r, position.y - r, position.z - r);
			worldAABBMax = Vector3(position.x + r, position.y + r, position.z + r);
		}
		else if (shape->getType() == BOX) {
			auto* box = (BoundingBox*)shape;
			Vector3 halfExtents = box->halfExtents;

			Vector3 worldX = rotationMatrix.getColumn(0);
			Vector3 worldY = rotationMatrix.getColumn(1);
			Vector3 worldZ = rotationMatrix.getColumn(2);

			float extentX = halfExtents.x * std::abs(worldX.x) + halfExtents.y * std::abs(worldY.x) + halfExtents.z *
				std::abs(worldZ.x);
			float extentY = halfExtents.x * std::abs(worldX.y) + halfExtents.y * std::abs(worldY.y) + halfExtents.z *
				std::abs(worldZ.y);
			float extentZ = halfExtents.x * std::abs(worldX.z) + halfExtents.z * std::abs(worldY.z) + halfExtents.z *
				std::abs(worldZ.z);

			worldAABBMin = Vector3(position.x - extentX, position.y - extentY, position.z - extentZ);
			worldAABBMax = Vector3(position.x + extentX, position.y + extentY, position.z + extentZ);
		}
	}
};

#endif //CPHYSICSENGINE_RIGIDBODY_H
