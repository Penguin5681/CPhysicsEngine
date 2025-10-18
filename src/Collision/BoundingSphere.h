//
// Created by penguin on 10/18/25.
//

#ifndef CPHYSICSENGINE_BOUNDINGSPHERE_H
#define CPHYSICSENGINE_BOUNDINGSPHERE_H

#pragma once
#include "../Core/Vector3.h"

class BoundingSphere {
public:
	float radius;

	BoundingSphere(const float radius) : radius(radius) {}

	bool overlaps(const Vector3 thisPosition, const Vector3 otherPosition, BoundingSphere* other) const {
		const float distanceSquared = (thisPosition - otherPosition).magnitudeSquared();
		const float totalRadius = this->radius + other->radius;
		const float totalRadiusSquared = totalRadius * totalRadius;

		return distanceSquared < totalRadiusSquared;
	}
};

#endif //CPHYSICSENGINE_BOUNDINGSPHERE_H
