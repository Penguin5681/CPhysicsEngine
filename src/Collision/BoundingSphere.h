//
// Created by penguin on 10/18/25.
//

#ifndef CPHYSICSENGINE_BOUNDINGSPHERE_H
#define CPHYSICSENGINE_BOUNDINGSPHERE_H

#pragma once
#include "../Core/Vector3.h"
#include "CollisionShape.h"

class BoundingSphere : public CollisionShape{
public:
	float radius;

	BoundingSphere(const float radius) : radius(radius) {}

	ShapeType getType() const override {
		return SPHERE;
	}
};

#endif //CPHYSICSENGINE_BOUNDINGSPHERE_H
