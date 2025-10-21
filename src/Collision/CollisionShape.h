//
// Created by penguin on 10/19/25.
//

#ifndef CPHYSICSENGINE_COLLISIONSHAPE_H
#define CPHYSICSENGINE_COLLISIONSHAPE_H

#pragma once

// TODO: Add more shapes later on
enum ShapeType {
	SPHERE,
	BOX
};

class CollisionShape {
public:
	virtual ~CollisionShape() = default;
	[[nodiscard]] virtual ShapeType getType() const = 0;
};

#endif //CPHYSICSENGINE_COLLISIONSHAPE_H