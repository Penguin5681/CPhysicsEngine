//
// Created by penguin on 10/19/25.
//

#ifndef CPHYSICSENGINE_BOUNDINGBOX_H
#define CPHYSICSENGINE_BOUNDINGBOX_H

#pragma once
#include "CollisionShape.h"
#include "../Core/Vector3.h"

class BoundingBox : public CollisionShape {
public:
	Vector3 halfExtents;

	BoundingBox(Vector3 halfExtents) : halfExtents(halfExtents) {}

	ShapeType getType() const override {
		return BOX;
	};
};

#endif //CPHYSICSENGINE_BOUNDINGBOX_H