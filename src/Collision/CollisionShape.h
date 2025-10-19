//
// Created by penguin on 10/19/25.
//

#ifndef CPHYSICSENGINE_COLLISIONSHAPE_H
#define CPHYSICSENGINE_COLLISIONSHAPE_H

#pragma once

enum ShapeType {
	SPHERE,
	BOX
};

class CollisionShape {
public:
	virtual ~CollisionShape() {}
	virtual ShapeType getType() const = 0;
};

#endif //CPHYSICSENGINE_COLLISIONSHAPE_H