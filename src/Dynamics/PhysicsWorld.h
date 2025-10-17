//
// Created by penguin on 10/17/25.
//

#pragma once

#ifndef CPHYSICSENGINE_PHYSICSWORLD_H
#define CPHYSICSENGINE_PHYSICSWORLD_H
#include  <vector>

#include "RigidBody.h"

class PhysicsWorld {
	std::vector<RigidBody*> bodies;
	Vector3 gravity;

public:
	PhysicsWorld(Vector3 gravity) {
		this->gravity = gravity;
	}

	void addBody(RigidBody* body) {
		bodies.push_back(body);
	}

	void step(const float dt) {
		for (const auto body : bodies) {
			if (body->inverseMass > 0.0) {
				body->addForce(gravity * (1.0f / body->inverseMass));
			}

			body->integrate(dt);
		}
	}
};

#endif //CPHYSICSENGINE_PHYSICSWORLD_H
