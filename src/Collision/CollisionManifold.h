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
	Vector3 contactPoint;

	float penetrationDepth;

	CollisionManifold(RigidBody* a, RigidBody* b, Vector3 normal, float depth, Vector3 contact) : bodyA(a), bodyB(b),
		contactNormal(normal), contactPoint(contact),
		penetrationDepth(depth) {}
};

#endif //CPHYSICSENGINE_COLLISIONMANIFOLD_H
