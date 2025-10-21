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

	struct Face {
		std::vector<Vector3> vertices;
		Vector3 normal;
	};

	Face getBoxFace(const RigidBody* boxBody, int faceIndex) {
		Face face;
		BoundingBox* box = (BoundingBox*)boxBody->shape;
		const Matrix3x3& rot = boxBody->rotationMatrix;
		const Vector3& center = boxBody->position;
		const Vector3& h = box->halfExtents;

		Vector3 localVerts[4];
		if (faceIndex == 0) {
			face.normal = rot.getColumn(0);
			localVerts[0] = Vector3(h.x, h.y, h.z);
			localVerts[1] = Vector3(h.x, h.y, -h.z);
			localVerts[2] = Vector3(h.x, -h.y, -h.z);
			localVerts[3] = Vector3(h.x, -h.y, h.z);
		}
		else if (faceIndex == 1) {
			face.normal = rot.getColumn(0) * -1.0f; // -X direction
			localVerts[0] = Vector3(-h.x, h.y, -h.z);
			localVerts[1] = Vector3(-h.x, h.y, h.z);
			localVerts[2] = Vector3(-h.x, -h.y, h.z);
			localVerts[3] = Vector3(-h.x, -h.y, -h.z);
		}
		else if (faceIndex == 2) {
			face.normal = rot.getColumn(1); // +Y direction
			localVerts[0] = Vector3(-h.x, h.y, -h.z);
			localVerts[1] = Vector3(-h.x, h.y, h.z);
			localVerts[2] = Vector3(h.x, h.y, h.z);
			localVerts[3] = Vector3(h.x, h.y, -h.z);
		}
		else if (faceIndex == 3) {
			face.normal = rot.getColumn(1) * -1.0f; // -Y direction
			localVerts[0] = Vector3(-h.x, -h.y, h.z);
			localVerts[1] = Vector3(-h.x, -h.y, -h.z);
			localVerts[2] = Vector3(h.x, -h.y, -h.z);
			localVerts[3] = Vector3(h.x, -h.y, h.z);
		}
		else if (faceIndex == 4) {
			face.normal = rot.getColumn(2); // +Z direction
			localVerts[0] = Vector3(-h.x, h.y, h.z);
			localVerts[1] = Vector3(-h.x, -h.y, h.z);
			localVerts[2] = Vector3(h.x, -h.y, h.z);
			localVerts[3] = Vector3(h.x, h.y, h.z);
		}
		else if (faceIndex == 5) {
			face.normal = rot.getColumn(2) * -1.0f; // -Z direction
			localVerts[0] = Vector3(-h.x, -h.y, -h.z);
			localVerts[1] = Vector3(-h.x, h.y, -h.z);
			localVerts[2] = Vector3(h.x, h.y, -h.z);
			localVerts[3] = Vector3(h.x, -h.y, -h.z);
		}

		for (int i = 0; i < 4; i++) {
			face.vertices.push_back(rot.transform(localVerts[i]) + center);
		}

		return face;
	}

	std::vector<Vector3> clipPolygonAgainstPlane(
		const std::vector<Vector3>& inVertices,
		const Vector3& planeNormal,
		float planeDist
	) {
		std::vector<Vector3> outVertices;
		if (inVertices.empty()) {
			return outVertices;
		}

		Vector3 prevVertex = inVertices.back();
		float prevDist = planeNormal.dot(prevVertex) - planeDist;

		for (auto currentVertex : inVertices) {
			float currentDist = planeNormal.dot(currentVertex) - planeDist;

			if (prevDist * currentDist < 0.0f) {
				float t = prevDist / (prevDist - currentDist);
				Vector3 intersection = prevVertex + (currentVertex - prevVertex) * t;
				outVertices.push_back(intersection);
			}

			if (currentDist >= 0.0f) {
				outVertices.push_back(currentVertex);
			}

			prevVertex = currentVertex;
			prevDist = currentDist;
		}

		return outVertices;
	}

	int findBestFace(const RigidBody* boxBody, const Vector3& normal) {
		float maxDot = -FLT_MAX;
		int bestFaceIndex = -1;
		const Matrix3x3& rot = boxBody->rotationMatrix;

		if (std::abs(normal.dot(rot.getColumn(0))) > maxDot) {
			maxDot = std::abs(normal.dot(rot.getColumn(0)));
			bestFaceIndex = (normal.dot(rot.getColumn(0)) > 0) ? 0 : 1;
		}
		if (std::abs(normal.dot(rot.getColumn(1))) > maxDot) {
			maxDot = std::abs(normal.dot(rot.getColumn(1)));
			bestFaceIndex = (normal.dot(rot.getColumn(1)) > 0) ? 2 : 3;
		}
		if (std::abs(normal.dot(rot.getColumn(2))) > maxDot) {
			maxDot = std::abs(normal.dot(rot.getColumn(2)));
			bestFaceIndex = (normal.dot(rot.getColumn(2)) > 0) ? 4 : 5;
		}

		return bestFaceIndex;
	}

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

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				Vector3 cross = axesA[i].cross(axesB[j]);
				float magSq = cross.magnitudeSquared();
				if (magSq > 0.001f) {
					testAxes[axisCount++] = cross * (1.0f / std::sqrt(magSq));
				}
			}
		}

		float minOverlap = FLT_MAX;
		Vector3 collisionNormal = Vector3(0, 0, 0);
		int bestAxisIndex = -1;

		for (int i = 0; i < axisCount; ++i) {
			Vector3 axis = testAxes[i];
			if (axis.magnitudeSquared() < 0.0001f) continue;

			Projection pA = project(boxA, axis);
			Projection pB = project(boxB, axis);

			if (!projectionOverlap(pA, pB)) {
				return;
			}

			float overlap = getOverlap(pA, pB);
			if (overlap < minOverlap) {
				minOverlap = overlap;
				collisionNormal = axis;
				bestAxisIndex = i;
			}
		}

		if (bestAxisIndex == -1) {
			return;
		}


		Vector3 dir = boxB->position - boxA->position;
		if (dir.dot(collisionNormal) < 0.0f) {
			collisionNormal = collisionNormal * -1.0f;
		}

		CollisionManifold manifold(boxA, boxB, collisionNormal, minOverlap);

		if (bestAxisIndex < 6) {
			RigidBody* refBody = (bestAxisIndex < 3) ? boxA : boxB;
			RigidBody* incBody = (bestAxisIndex < 3) ? boxB : boxA;
			Vector3 refNormal = (bestAxisIndex < 3) ? collisionNormal : collisionNormal * -1.0f;

			int incFaceIndex = findBestFace(incBody, refNormal * -1.0f);
			Face incFace = getBoxFace(incBody, incFaceIndex);

			int refFaceIndex = findBestFace(refBody, refNormal);
			Face refFace = getBoxFace(refBody, refFaceIndex);

			std::vector<Vector3> clippedVertices = incFace.vertices;

			for (int side = 0; side < 4; ++side) {
				Vector3 edgeStart = refFace.vertices[side];
				Vector3 edgeEnd = refFace.vertices[(side + 1) % 4];
				Vector3 edgeDir = edgeEnd - edgeStart;
				Vector3 sidePlaneNormal = edgeDir.cross(refNormal).normalized();
				float sidePlaneDist = sidePlaneNormal.dot(edgeStart);

				clippedVertices = clipPolygonAgainstPlane(clippedVertices, sidePlaneNormal, sidePlaneDist);
				if (clippedVertices.size() < 3) break;
			}


			float refPlaneDist = refNormal.dot(refFace.vertices[0]);
			for (const Vector3& vertex : clippedVertices) {
				float distToRefPlane = refNormal.dot(vertex) - refPlaneDist;
				if (distToRefPlane <= 0.01f && distToRefPlane >= -minOverlap * 1.01f) {
					manifold.addContactPoint(vertex);
				}
			}

			if (manifold.numContactPoints == 0) {
				Vector3 contactPoint = (boxA->position + boxB->position) * 0.5f;
				manifold.addContactPoint(contactPoint);
			}
		}
		else {
			// TODO: Implement proper Edge-Edge closest points calculation.
			Vector3 contactPoint = (boxA->position + boxB->position) * 0.5f;
			manifold.addContactPoint(contactPoint);
		}

		if (manifold.numContactPoints > 0) {
			collisions.push_back(manifold);
		}
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
		for (auto& manifold : collisions) {
			if (manifold.numContactPoints == 0) {
				continue;
			}

			RigidBody* A = manifold.bodyA;
			RigidBody* B = manifold.bodyB;

			if ((!A->isAwake && A->inverseMass == 0.0f) && (!B->isAwake && B->inverseMass == 0.0f)) {
				continue;
			}


			bool bodyAWasAsleep = !A->isAwake;
			bool bodyBWasAsleep = !B->isAwake;
			if (bodyAWasAsleep || bodyBWasAsleep) {
				Vector3 initialRelVel = B->velocity - A->velocity;
				if (initialRelVel.magnitudeSquared() > sleepEpsilonSq * 4.0f) {
					if (bodyAWasAsleep && A->inverseMass > 0.0f) A->setAwake(true);
					if (bodyBWasAsleep && B->inverseMass > 0.0f) B->setAwake(true);
					bodyAWasAsleep = !A->isAwake;
					bodyBWasAsleep = !B->isAwake;
				}
			}

			float staticFriction = (A->staticFriction + B->staticFriction) * 0.5f;
			float dynamicFriction = (A->dynamicFriction + B->dynamicFriction) * 0.5f;

			float totalImpulseMagnitudeSq = 0.0f;

			for (int i = 0; i < manifold.numContactPoints; i++) {
				Vector3 contactPoint = manifold.contactPoints[i];
				Vector3 rA = contactPoint - A->position;
				Vector3 rB = contactPoint - B->position;
				Vector3 normal = manifold.contactNormal;

				Vector3 velA_pre = A->velocity + A->angularVelocity.cross(rA);
				Vector3 velB_pre = B->velocity + B->angularVelocity.cross(rB);
				Vector3 relVel_pre = velB_pre - velA_pre;
				float velAlongNormal = relVel_pre.dot(normal);

				float restitutionBias = 0.0f;
				float e = std::min(A->restitution, B->restitution);
				float j_n = 0.0f;

				if (velAlongNormal < -restitutionBias) {
					Vector3 termA = (A->inverseInertiaTensor.transform(rA.cross(normal))).cross(rA);
					Vector3 termB = (B->inverseInertiaTensor.transform(rB.cross(normal))).cross(rB);
					float denominator = A->inverseMass + B->inverseMass + (termA + termB).dot(normal);

					if (denominator > 0.0001f) {
						j_n = -(1.0f + e) * velAlongNormal / denominator;

						Vector3 impulseN = normal * j_n;
						A->velocity -= impulseN * A->inverseMass;
						B->velocity += impulseN * B->inverseMass;
						A->angularVelocity += A->inverseInertiaTensor.transform(rA.cross(impulseN) * -1.0f);
						B->angularVelocity += B->inverseInertiaTensor.transform(rB.cross(impulseN));

						totalImpulseMagnitudeSq += impulseN.magnitudeSquared();
					}
				}

				Vector3 velA_post = A->velocity + A->angularVelocity.cross(rA);
				Vector3 velB_post = B->velocity + B->angularVelocity.cross(rB);
				Vector3 relVel_post = velB_post - velA_post;

				Vector3 tangentVel = relVel_post - normal * relVel_post.dot(normal);
				float tangentSpeedSq = tangentVel.magnitudeSquared();

				if (tangentSpeedSq > 0.0001f) {
					Vector3 tangentDir = tangentVel / std::sqrt(tangentSpeedSq);

					Vector3 termA_t = (A->inverseInertiaTensor.transform(rA.cross(tangentDir))).cross(rA);
					Vector3 termB_t = (B->inverseInertiaTensor.transform(rB.cross(tangentDir))).cross(rB);
					float denominator_t = A->inverseMass + B->inverseMass + (termA_t + termB_t).dot(tangentDir);

					if (denominator_t > 0.0001f) {
						float jt_mag_needed = -relVel_post.dot(tangentDir) / denominator_t;

						float frictionLimit = staticFriction * std::abs(j_n);
						float jt_mag_clamped = std::max(-frictionLimit, std::min(jt_mag_needed, frictionLimit));

						Vector3 impulseT = tangentDir * jt_mag_clamped;
						A->velocity -= impulseT * A->inverseMass;
						B->velocity += impulseT * B->inverseMass;
						A->angularVelocity += A->inverseInertiaTensor.transform(rA.cross(impulseT) * -1.0f);
						B->angularVelocity += B->inverseInertiaTensor.transform(rB.cross(impulseT));

						totalImpulseMagnitudeSq += impulseT.magnitudeSquared();
					}
				}
			}

			if ((bodyAWasAsleep || bodyBWasAsleep) && (totalImpulseMagnitudeSq > wakeImpulseThresholdSq)) {
				if (bodyAWasAsleep && A->inverseMass > 0.0f) {
					A->setAwake(true);
				}
				if (bodyBWasAsleep && B->inverseMass > 0.0f) {
					B->setAwake(true);
				}
			}
			if (A->isAwake || B->isAwake) {
				const float totalInverseMass = A->inverseMass + B->inverseMass;
				if (totalInverseMass > 0.0f) {
					const float percent = 0.3f;
					const float slop = 0.01f;
					Vector3 correction = manifold.contactNormal * (std::max(manifold.penetrationDepth - slop, 0.0f) /
						totalInverseMass) * percent;

					A->position -= correction * A->inverseMass;
					B->position += correction * B->inverseMass;

					A->updateWorldAABB();
					B->updateWorldAABB();
				}
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
