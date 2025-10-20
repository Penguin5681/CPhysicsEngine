//
// Created by penguin on 10/18/25.
//

#ifndef CPHYSICSENGINE_COLLISIONMANIFOLD_H
#define CPHYSICSENGINE_COLLISIONMANIFOLD_H

#pragma once
#include "../Dynamics/RigidBody.h"

struct CollisionManifold {
	RigidBody* bodyA;
	RigidBody* bodyB;

	Vector3 contactNormal;
	std::vector<Vector3> contactPoints;
	int numContactPoints;

	float penetrationDepth;

	CollisionManifold() : bodyA(nullptr), bodyB(nullptr), numContactPoints(0), penetrationDepth(0.0f) {}

	CollisionManifold(RigidBody* a, RigidBody* b, Vector3 normal, float depth) : bodyA(a), bodyB(b),
		contactNormal(normal), numContactPoints(0),
		penetrationDepth(depth) {}

	void addContactPoint(const Vector3& point) {
		contactPoints.push_back(point);
		numContactPoints++;
	}
};

#endif //CPHYSICSENGINE_COLLISIONMANIFOLD_H
