//
// Created by penguin on 10/17/25.
//

#pragma once

#ifndef CPHYSICSENGINE_PHYSICSWORLD_H
#define CPHYSICSENGINE_PHYSICSWORLD_H
#include <iostream>
#include <vector>
#include <algorithm>
#include <cfloat>
#include <cmath>

#include "RigidBody.h"
#include "../Collision/CollisionManifold.h"
#include "../Collision/BoundingSphere.h"
#include "../Collision/BoundingBox.h"

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
	struct Projection {
		float min;
		float max;
	};

	void detectCollisions() {
		for (size_t i = 0; i < bodies.size(); i++) {
			for (size_t j = i + 1; j < bodies.size(); j++) {
				RigidBody* bodyA = bodies[i];
				RigidBody* bodyB = bodies[j];

				if (bodyA->shape == nullptr || bodyB->shape == nullptr) {
					continue;
				}

				findCollisionFeatures(bodyA, bodyB);
			}
		}
	}

	void findCollisionFeatures(RigidBody* bodyA, RigidBody* bodyB) {
		ShapeType typeA = bodyA->shape->getType();
		ShapeType typeB = bodyB->shape->getType();

		// this is our dispatch table
		if (typeA == SPHERE and typeB == SPHERE) {
			checkSphereSphere(bodyA, bodyB);
		}
		else if (typeA == BOX and typeB == BOX) {
			checkSat(bodyA, bodyB);
		}
		else if (typeA == BOX and typeB == SPHERE) {
			checkSphereBox(bodyB, bodyA);
		}
		else if (typeA == SPHERE and typeB == BOX) {
			checkSphereBox(bodyA, bodyB);
		}
	}

	void checkSphereSphere(RigidBody* sphereA, RigidBody* sphereB) {
		BoundingSphere* shapeA = static_cast<BoundingSphere*>(sphereA->shape);
		BoundingSphere* shapeB = static_cast<BoundingSphere*>(sphereB->shape);

		const Vector3 dir = sphereB->position - sphereA->position;
		float dist = dir.magnitude();
		const float totalRadius = shapeA->radius + shapeB->radius;

		if (dist < totalRadius) {
			const Vector3 normal = (dist > 0.0f) ? dir / dist : Vector3(0, 1, 0);
			const float penetration = totalRadius - dist;
			const Vector3 contact = sphereA->position + normal * (shapeA->radius - penetration / 2.0f);

			collisions.push_back(CollisionManifold(sphereA, sphereB, normal, penetration, contact));
		}
	}

	void checkSat(RigidBody* boxA, RigidBody* boxB) {
		Vector3 axesA[3];
		axesA[0] = boxA->rotationMatrix.getColumn(0);
		axesA[1] = boxA->rotationMatrix.getColumn(1);
		axesA[2] = boxA->rotationMatrix.getColumn(2);

		Vector3 axesB[3];
		axesB[0] = boxB->rotationMatrix.getColumn(0);
		axesB[1] = boxB->rotationMatrix.getColumn(1);
		axesB[2] = boxB->rotationMatrix.getColumn(2);

		Vector3 testAxes[15];
		int axisCount = 0;

		testAxes[axisCount++] = axesA[0];
		testAxes[axisCount++] = axesA[1];
		testAxes[axisCount++] = axesA[2];

		testAxes[axisCount++] = axesB[0];
		testAxes[axisCount++] = axesB[1];
		testAxes[axisCount++] = axesB[2];

		for (size_t i = 0; i < 3; i++) {
			for (size_t j = 0; j < 3; j++) {
				Vector3 cross = axesA[i].cross(axesB[j]);
				float magSq = cross.magnitudeSquared();
				if (magSq > 0.001f) {
					testAxes[axisCount++] = cross * (1.0f / std::sqrt(magSq));
				}
			}
		}


		float minOverlap = FLT_MAX;
		Vector3 collisionNormal = Vector3(0, 0, 0);

		for (size_t i = 0; i < axisCount; i++) {
			Vector3 axis = testAxes[i];


			Projection pA = project(boxA, axis);
			Projection pB = project(boxB, axis);


			if (projectionOverlap(pA, pB) == false) {
				std::cout << "SAT: Separating axis found, no collision" << std::endl;
				return;
			}

			float overlap = getOverlap(pA, pB);
			std::cout << "SAT:   Overlap amount: " << overlap << std::endl;

			if (overlap < minOverlap) {
				minOverlap = overlap;
				collisionNormal = axis;
			}
		}

		std::cout << "SAT: Final minimum overlap: " << minOverlap << std::endl;
		std::cout << "SAT: Collision normal: (" << collisionNormal.x << ", "
			<< collisionNormal.y << ", " << collisionNormal.z << ")" << std::endl;

		Vector3 dir = boxB->position - boxA->position;
		if (dir.dot(collisionNormal) < 0.0f) {
			collisionNormal = collisionNormal * -1.0f;
			std::cout << "SAT: Flipped collision normal: (" << collisionNormal.x << ", "
				<< collisionNormal.y << ", " << collisionNormal.z << ")" << std::endl;
		}

		Vector3 contactPoint = (boxA->position + boxB->position) * 0.5f;

		collisions.push_back(CollisionManifold(boxA, boxB, collisionNormal, minOverlap, contactPoint));
	}

	float clamp(float value, float minVal, float maxVal) {
		return std::max(minVal, std::min(value, maxVal));
	}

	void checkSphereBox(RigidBody* sphereBody, RigidBody* boxBody) {
		BoundingSphere* sphere = static_cast<BoundingSphere*>(sphereBody->shape);
		BoundingBox* box = static_cast<BoundingBox*>(boxBody->shape);

		Vector3 sphereCenterWorld = sphereBody->position;
		Vector3 sphereCenterLocal = boxBody->rotationMatrix.transformTranspose(
			sphereCenterWorld - boxBody->position
		);

		Vector3 closestPointLocal;
		closestPointLocal.x = clamp(sphereCenterLocal.x, -box->halfExtents.x, box->halfExtents.x);
		closestPointLocal.y = clamp(sphereCenterLocal.y, -box->halfExtents.y, box->halfExtents.y);
		closestPointLocal.z = clamp(sphereCenterLocal.z, -box->halfExtents.z, box->halfExtents.z);

		Vector3 closestPointWorld = boxBody->rotationMatrix.transform(closestPointLocal) + boxBody->position;

		Vector3 dir = sphereCenterWorld - closestPointWorld;
		float distanceSquared = dir.magnitudeSquared();
		float radiusSquared = sphere->radius * sphere->radius;

		if (distanceSquared < radiusSquared) {
			// this means there is a collision
			float distance = std::sqrt(distanceSquared);
			Vector3 normal = (distance > 0.0f) ? dir / distance : Vector3(0, 1, 0);
			Vector3 normal_B_to_A = (distance > 0.0f) ? dir / distance : Vector3(0, 1, 0);
			float penetration = sphere->radius - distance;
			Vector3 contactPoint = closestPointWorld;

			Vector3 normal_A_to_B = normal_B_to_A * -1.0f;

			std::cout << "COLLISION: Sphere(" << sphereBody << ") vs Box(" << boxBody << ")" << std::endl;

			collisions.push_back(CollisionManifold(sphereBody, boxBody, normal_A_to_B, penetration, contactPoint));
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

	// Projection project(RigidBody* boxBody, const Vector3& axis) {
	// 	Vector3 normAxis = axis.normalized();
	//
	// 	BoundingBox* box = static_cast<BoundingBox*>(boxBody->shape);
	// 	const Matrix3x3& rot = boxBody->rotationMatrix;
	// 	const Vector3& center = boxBody->position;
	// 	const Vector3& halfExtents = box->halfExtents;
	//
	// 	float centerProj = normAxis.dot(center);
	//
	// 	float radiusProj = halfExtents.x * std::abs(normAxis.dot(rot.getColumn(0))) +
	// 										 halfExtents.y * std::abs(normAxis.dot(rot.getColumn(1))) +
	// 										 halfExtents.z * std::abs(normAxis.dot(rot.getColumn(2)));
	//
	// 	float minP = centerProj - radiusProj;
	// 	float maxP = centerProj + radiusProj;
	//
	// 	return {minP, maxP};
	// }

	Projection project(RigidBody* boxBody, const Vector3& axis) {
		BoundingBox* box = dynamic_cast<BoundingBox*>(boxBody->shape);
		const Vector3& halfExtents = box->halfExtents;
		const Matrix3x3& rot = boxBody->rotationMatrix;
		const Vector3& center = boxBody->position;

		Vector3 vertices[8] = {
			Vector3(halfExtents.x, halfExtents.y, halfExtents.z),
			Vector3(-halfExtents.x, halfExtents.y, halfExtents.z),
			Vector3(halfExtents.x, -halfExtents.y, halfExtents.z),
			Vector3(halfExtents.x, halfExtents.y, -halfExtents.z),
			Vector3(-halfExtents.x, -halfExtents.y, halfExtents.z),
			Vector3(-halfExtents.x, halfExtents.y, -halfExtents.z),
			Vector3(halfExtents.x, -halfExtents.y, -halfExtents.z),
			Vector3(-halfExtents.x, -halfExtents.y, -halfExtents.z)
		};

		Vector3 worldVertex = rot.transform(vertices[0]) + center;
		float minP = axis.dot(worldVertex);
		float maxP = minP;

		for (size_t i = 1; i < 8; ++i) {
			worldVertex = rot.transform(vertices[i]) + center;
			float p = axis.dot(worldVertex);
			if (p < minP) {
				minP = p;
			}
			if (p > maxP) {
				maxP = p;
			}
		}

		return {minP, maxP};
	}

	static bool projectionOverlap(Projection pA, Projection pB) {
		return (pA.max > pB.min and pA.min < pB.max);
	}

	static float getOverlap(Projection pA, Projection pB) {
		return std::min(pA.max, pB.max) - std::max(pA.min, pB.min);
	}
};

#endif //CPHYSICSENGINE_PHYSICSWORLD_H
