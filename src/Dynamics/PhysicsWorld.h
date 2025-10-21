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
#include "../Broadphase/UniformGrid.h"
#include "../Collision/CollisionManifold.h"
#include "../Collision/BoundingSphere.h"
#include "../Collision/BoundingBox.h"

class PhysicsWorld {
	std::vector<RigidBody*> bodies;
	std::vector<CollisionManifold> collisions;

	Vector3 gravity;

	UniformGrid broadphaseGrid;

	float sleepEpsilonSq = 0.01f * 0.01f;
	float sleepDuration = 0.5f;
	float wakeImpulseThresholdSq = 0.1f * 0.1f;

public:
	PhysicsWorld(Vector3 gravity, float gridCellSize = 4.0f) : broadphaseGrid(gridCellSize) {
		this->gravity = gravity;
	}

	void addBody(RigidBody* body) {
		bodies.push_back(body);
	}

	void step(const float dt) {
		collisions.clear();

		for (const auto body : bodies) {
			if (body->isAwake and body->inverseMass > 0.0f) {
				body->addForce(gravity * (1.0f / body->inverseMass));
			}
			body->integrate(dt);
		}

		broadphaseGrid.clear();
		for (const auto body : bodies) {
			broadphaseGrid.addBody(body);
		}

		std::vector<std::pair<RigidBody*, RigidBody*>> potentialPairs = broadphaseGrid.getPotentialCollisions();

		detectCollisions(potentialPairs);

		resolveCollisions();

		for (const auto body : bodies) {
			body->updateSleepState(dt, sleepEpsilonSq, sleepDuration);
		}
	}

private:
	struct Projection {
		float min;
		float max;
	};

	// void detectCollisions() {
	// 	for (size_t i = 0; i < bodies.size(); i++) {
	// 		for (size_t j = i + 1; j < bodies.size(); j++) {
	// 			RigidBody* bodyA = bodies[i];
	// 			RigidBody* bodyB = bodies[j];
	//
	// 			if (bodyA->shape == nullptr || bodyB->shape == nullptr) {
	// 				continue;
	// 			}
	//
	// 			findCollisionFeatures(bodyA, bodyB);
	// 		}
	// 	}
	// }

	void detectCollisions(const std::vector<std::pair<RigidBody*, RigidBody*>>& pairs) {
		for (const auto pair : pairs) {
			RigidBody* bodyA = pair.first;
			RigidBody* bodyB = pair.second;

			if ((!bodyA->isAwake and bodyA->inverseMass == 0.0f) and (!bodyB->isAwake and bodyB->inverseMass == 0.0f)) {
				continue;
			}

			if (bodyA->shape != nullptr and bodyB->shape != nullptr) {
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

			CollisionManifold manifold(sphereA, sphereB, normal, penetration);
			manifold.addContactPoint(contact);
			collisions.push_back(manifold);
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

		for (auto i : axesA) {
			for (auto j : axesB) {
				Vector3 cross = i.cross(j);
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
		}

		BoundingBox* boxShapeB = (BoundingBox*)boxB->shape;
		const Matrix3x3& rotB = boxB->rotationMatrix;
		const Vector3& centerB = boxB->position;
		const Vector3& halfExtentsB = boxShapeB->halfExtents;

		Vector3 verticesB_local[8];
		verticesB_local[0].x = halfExtentsB.x;
		verticesB_local[0].y = halfExtentsB.y;
		verticesB_local[0].z = halfExtentsB.z;

		verticesB_local[1].x = -halfExtentsB.x;
		verticesB_local[1].y = halfExtentsB.y;
		verticesB_local[1].z = halfExtentsB.z;

		verticesB_local[2].x = halfExtentsB.x;
		verticesB_local[2].y = -halfExtentsB.y;
		verticesB_local[2].z = halfExtentsB.z;

		verticesB_local[3].x = halfExtentsB.x;
		verticesB_local[3].y = halfExtentsB.y;
		verticesB_local[3].z = -halfExtentsB.z;

		verticesB_local[4].x = -halfExtentsB.x;
		verticesB_local[4].y = -halfExtentsB.y;
		verticesB_local[4].z = halfExtentsB.z;

		verticesB_local[5].x = -halfExtentsB.x;
		verticesB_local[5].y = halfExtentsB.y;
		verticesB_local[5].z = -halfExtentsB.z;

		verticesB_local[6].x = halfExtentsB.x;
		verticesB_local[6].y = -halfExtentsB.y;
		verticesB_local[6].z = -halfExtentsB.z;

		verticesB_local[7].x = -halfExtentsB.x;
		verticesB_local[7].y = -halfExtentsB.y;
		verticesB_local[7].z = -halfExtentsB.z;

		float maxPenetration = -FLT_MAX;
		Vector3 contactPoint = centerB;

		for (auto i : verticesB_local) {
			Vector3 vertexB_world = rotB.transform(i) + centerB;
			float vertexPenetration = (boxA->position - vertexB_world).dot(collisionNormal);

			if (vertexPenetration > maxPenetration) {
				maxPenetration = vertexPenetration;
				contactPoint = vertexB_world;
			}
		}

		CollisionManifold manifold(boxA, boxB, collisionNormal, minOverlap);
		manifold.addContactPoint(contactPoint);
		collisions.push_back(manifold);

		// Vector3 contactPoint = (boxA->position + boxB->position) * 0.5f;
		//
		// collisions.push_back(CollisionManifold(boxA, boxB, collisionNormal, minOverlap, contactPoint));
	}

	static float clamp(float value, float minVal, float maxVal) {
		return std::max(minVal, std::min(value, maxVal));
	}

	void checkSphereBox(RigidBody* sphereBody, RigidBody* boxBody) {
		auto* sphere = dynamic_cast<BoundingSphere*>(sphereBody->shape);
		auto* box = dynamic_cast<BoundingBox*>(boxBody->shape);

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
			Vector3 normal_B_to_A = (distance > 0.0f) ? dir / distance : Vector3(0, 1, 0);
			float penetration = sphere->radius - distance;
			Vector3 contactPoint = closestPointWorld;

			Vector3 normal_A_to_B = normal_B_to_A * -1.0f;

			CollisionManifold manifold(sphereBody, boxBody, normal_A_to_B, penetration);
			manifold.addContactPoint(contactPoint);
			collisions.push_back(manifold);
		}
	}

	void resolveCollisions() {
		for (auto manifold : collisions) {
			if (manifold.numContactPoints == 0) {
				continue;
			}

			auto* A = manifold.bodyA;
			auto* B = manifold.bodyB;

			// note: wake up check start
			bool bodyAWasAsleep = !A->isAwake;
			bool bodyBWasAsleep = !B->isAwake;
			if (bodyAWasAsleep or bodyBWasAsleep) {
				Vector3 relVel = B->velocity - A->velocity;
				if (relVel.magnitudeSquared() > sleepEpsilonSq) {
					if (bodyAWasAsleep and A->inverseMass > 0.0f)
						A->setAwake(true);
					if (bodyBWasAsleep and B->inverseMass > 0.0f)
						B->setAwake(true);
				}
			}
			// note: wake up check ends here

			// note: skipping resolution if A and B are both asleep
			if ((!A->isAwake and B->inverseMass == 0.0f) and (!B->isAwake and B->inverseMass == 0.0f))
				continue;
			float totalImpulseMagnitudeSq = 0.0f;
			float totalMagnitudeSq = 0.0f;
			for (int i = 0; i < manifold.numContactPoints; i++) {
				Vector3 contactPoint = manifold.contactPoints[i];
				Vector3 rA = contactPoint - A->position;
				Vector3 rB = contactPoint - B->position;
				Vector3 velA = A->velocity + A->angularVelocity.cross(rA);
				Vector3 velB = B->velocity + B->angularVelocity.cross(rB);
				Vector3 normal = manifold.contactNormal;
				float penetration = manifold.penetrationDepth;
				float velAlongNormal = (velB - velA).dot(manifold.contactNormal);
				if (velAlongNormal > 0.0f) {
					continue;
				}

				float e = std::min(A->restitution, B->restitution);

				Vector3 termA = (A->inverseInertiaTensor.transform(rA.cross(manifold.contactNormal))).cross(rA);
				Vector3 termB = (B->inverseInertiaTensor.transform(rB.cross(manifold.contactNormal))).cross(rB);

				float j = -(1.0f + e) * velAlongNormal;
				float denominator = A->inverseMass + B->inverseMass + (termA + termB).dot(manifold.contactNormal);

				if (denominator <= 0.0f) {
					continue;
				}

				Vector3 impulse = normal * j;
				A->velocity -= impulse * A->inverseMass;
				B->velocity += impulse * B->inverseMass;

				Vector3 torqueImpulseA = rA.cross(impulse) * -1.0f;
				Vector3 torqueImpulseB = rB.cross(impulse);

				A->angularVelocity += A->inverseInertiaTensor.transform(torqueImpulseA);
				B->angularVelocity += B->inverseInertiaTensor.transform(torqueImpulseB);

				totalImpulseMagnitudeSq += impulse.magnitudeSquared();
			}

			if ((bodyAWasAsleep or bodyBWasAsleep) and (totalImpulseMagnitudeSq > wakeImpulseThresholdSq)) {
				if (bodyAWasAsleep and A->inverseMass > 0.0f) {
					A->setAwake(true);
				}
				if (bodyBWasAsleep and B->inverseMass > 0.0f) {
					B->setAwake(true);
				}
			}

			if (A->isAwake or B->isAwake) {
				const float totalInverseMass = A->inverseMass + B->inverseMass;
				if (totalInverseMass <= 0.0f) {
					continue;
				}

				const float percent = 0.2f;
				const float slop = 0.01f;
				Vector3 correction = manifold.contactNormal * (std::max(manifold.penetrationDepth - slop, 0.0f) /
					totalInverseMass) * percent;

				A->position -= correction * A->inverseMass;
				B->position += correction * B->inverseMass;
			}
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
