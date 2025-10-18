//
// Created by penguin on 10/17/25.
//

#pragma once

#ifndef CPHYSICSENGINE_PHYSICSWORLD_H
#define CPHYSICSENGINE_PHYSICSWORLD_H
#include <iostream>
#include <vector>
#include <algorithm>

#include "RigidBody.h"
#include "../Collision/CollisionManifold.h"

class PhysicsWorld {
	std::vector<RigidBody*> bodies;
	Vector3 gravity;
	std::vector<CollisionManifold> collisions;

public:
	PhysicsWorld(Vector3 gravity) {
		this->gravity = gravity;
	}

	void addBody(RigidBody* body) {
		bodies.push_back(body);
	}

	void step(const float dt) {
		collisions.clear();

		for (const auto body : bodies) {
			if (body->inverseMass > 0.0) {
				body->addForce(gravity * (1.0f / body->inverseMass));
			}

			body->integrate(dt);
		}
		detectCollisions();
		resolveCollisions();
	}

private:
	void detectCollisions() {
		for (size_t i = 0; i < bodies.size(); i++) {
			for (size_t j = i + 1; j < bodies.size(); j++) {
				RigidBody* body1 = bodies[i];
				RigidBody* body2 = bodies[j];

				if (body1->shape == nullptr || body2->shape == nullptr) {
					continue;
				}

				const Vector3 dir = body2->position - body1->position;
				float dist = dir.magnitude();
				const float totalRadius = body1->shape->radius + body2->shape->radius;

				if (dist < totalRadius) {
					const Vector3 normal = (dist > 0.0f) ? dir / dist : Vector3(0, 1, 0);
					const float penetration = totalRadius - dist;
					const Vector3 contact = body1->position + normal * (body1->shape->radius - penetration / 2.0f);

					collisions.push_back(CollisionManifold(body1, body2, normal, penetration, contact));

					std::cout << "COLLISION: Body " << i << " is overlapping Body " << j << std::endl;
				}
			}
		}
	}

	void resolveCollisions() {
		for (auto manifold : collisions) {
			RigidBody* A = manifold.bodyA;
			RigidBody* B = manifold.bodyB;
			Vector3 normal = manifold.contactNormal;
			Vector3 contactPoint = manifold.contactPoint;

			Vector3 rA = contactPoint - A->position;
			Vector3 rB = contactPoint - B->position;

			Vector3 velA = A->velocity + A->angularVelocity.cross(rA);
			Vector3 velB = B->velocity + B->angularVelocity.cross(rB);
			Vector3 relVel = velB - velA;

			float velAlongNormal = relVel.dot(normal);
			float penetration = manifold.penetrationDepth;
			if (velAlongNormal > 0.0f) {
				continue;
			}

			float e = std::min(A->restitution, B->restitution);

			// computation of the rotational mass
			Vector3 termA = (A->inverseInertiaTensor.transform(rA.cross(normal))).cross(rA);
			Vector3 termB = (B->inverseInertiaTensor.transform(rB.cross(normal))).cross(rB);

			float denominator = A->inverseMass + B->inverseMass + (termA + termB).dot(normal);

			if (denominator <= 0.0f) {
				continue;
			}

			float j = -(1.0f + e) * velAlongNormal;
			j /= denominator;

			const float totalInverseMass = A->inverseMass + B->inverseMass;

			Vector3 impulse = normal * j;

			A->velocity -= impulse * A->inverseMass;
			B->velocity += impulse * B->inverseMass;

			Vector3 torqueImpulseA = rA.cross(impulse) * -1.0f;
			Vector3 torqueImpulseB = rB.cross(impulse);

			A->angularVelocity += A->inverseInertiaTensor.transform(torqueImpulseA);
			B->angularVelocity += B->inverseInertiaTensor.transform(torqueImpulseB);

			float percent = 0.2f;
			float slop = 0.01f;

			if (totalInverseMass <= 0.0f) {
				continue;
			}

			Vector3 correction = normal * (std::max(penetration - slop, 0.0f) / (A->inverseMass + B->inverseMass)) * percent;

			A->position -= correction * A->inverseMass;
			B->position += correction * B->inverseMass;
		}
	}
};

#endif //CPHYSICSENGINE_PHYSICSWORLD_H
