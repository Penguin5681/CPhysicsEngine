#include "./src/Core/Vector3.h"
#include "./src/Core/Quaternion.h"
#include "./src/Dynamics/PhysicsWorld.h"
#include "./src/Dynamics/RigidBody.h"
#include "./src/Collision/BoundingBox.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <SFML/Graphics.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr float PIXELS_PER_METER = 20.0f;
constexpr int WINDOW_WIDTH = 1200;
constexpr int WINDOW_HEIGHT = 800;

sf::Vector2f toScreenPos(const Vector3& worldPos) {
	return {
		worldPos.x * PIXELS_PER_METER + WINDOW_WIDTH / 2.0f,
		-worldPos.y * PIXELS_PER_METER + WINDOW_HEIGHT / 2.0f
	};
}

float toDegrees(const Matrix3x3& rotMatrix) {
	float rad = atan2f(rotMatrix.data[3], rotMatrix.data[0]);
	return std::isnan(rad) ? 0.0f : -rad * 180.0f / M_PI;
}

RigidBody* createBox(Vector3 position, Vector3 halfExtents, float mass, float restitution) {
	RigidBody* box = new RigidBody();
	box->position = position;
	box->inverseMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
	box->restitution = restitution;
	BoundingBox* shapeData = new BoundingBox(halfExtents);
	box->shape = shapeData;
	Vector3 fullDims = halfExtents * 2.0f;
	box->inverseInertiaTensor.setInverseInertiaTensorCuboid(mass, fullDims);
	box->orientation.normalize();
	return box;
}

int main() {
	sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "CPhysicsEngine - Uniform Grid Test");
	window.setFramerateLimit(60);

	Vector3 gravity = Vector3(0.0f, -9.81f, 0.0f);
	PhysicsWorld world(gravity, 4.0f);
	float timeStep = 1.0f / 60.0f;

	std::vector<RigidBody*> bodies;
	std::vector<sf::Shape*> shapes;
	std::vector<CollisionShape*> shapeDataToDelete;

	RigidBody* floorBody = createBox(Vector3(0.0f, -18.0f, 0.0f), Vector3(30.0f, 1.0f, 1.0f), 0.0f, 0.5f);
	// Mass 0 = static
	world.addBody(floorBody);
	bodies.push_back(floorBody);
	shapeDataToDelete.push_back(floorBody->shape);

	auto* floorGfx = new sf::RectangleShape();
	auto* floorBB = static_cast<BoundingBox*>(floorBody->shape);
	floorGfx->setSize(sf::Vector2f(floorBB->halfExtents.x * 2.0f * PIXELS_PER_METER,
	                               floorBB->halfExtents.y * 2.0f * PIXELS_PER_METER));
	floorGfx->setOrigin(floorBB->halfExtents.x * PIXELS_PER_METER,
	                    floorBB->halfExtents.y * PIXELS_PER_METER);
	floorGfx->setFillColor(sf::Color::White);
	shapes.push_back(floorGfx);


	int numBoxesX = 200;
	float spacing = 5.0f;
	float startX = -(numBoxesX - 1.0f) * spacing / 2.0f;

	for (int i = 0; i < numBoxesX; ++i) {
		float xPos = startX + i * spacing;
		RigidBody* fallingBox = createBox(Vector3(xPos, 10.0f + (i % 2) * 2.0f, 0.0f), Vector3(1.0f, 1.0f, 1.0f), 10.0f,
		                                  0.5f);
		fallingBox->orientation = Quaternion(cosf(i * 0.1f), 0, 0, sinf(i * 0.1f));
		fallingBox->orientation.normalize();

		world.addBody(fallingBox);
		bodies.push_back(fallingBox);
		shapeDataToDelete.push_back(fallingBox->shape);

		sf::RectangleShape* boxGfx = new sf::RectangleShape();
		BoundingBox* boxBB = static_cast<BoundingBox*>(fallingBox->shape);
		boxGfx->setSize(sf::Vector2f(boxBB->halfExtents.x * 2.0f * PIXELS_PER_METER,
		                             boxBB->halfExtents.y * 2.0f * PIXELS_PER_METER));
		boxGfx->setOrigin(boxBB->halfExtents.x * PIXELS_PER_METER,
		                  boxBB->halfExtents.y * PIXELS_PER_METER);
		boxGfx->setFillColor(sf::Color(255, 50 + i * 40, 10 + i * 12));
		shapes.push_back(boxGfx);
	}

	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				window.close();
			}
		}

		world.step(timeStep);

		for (size_t i = 0; i < bodies.size(); ++i) {
			RigidBody* body = bodies[i];
			sf::Shape* shape = shapes[i];

			shape->setPosition(toScreenPos(body->position));
			shape->setRotation(toDegrees(body->rotationMatrix));
		}

		window.clear(sf::Color::Black);
		for (const auto* shape : shapes) {
			window.draw(*shape);
		}
		window.display();
	}

	for (CollisionShape* s : shapeDataToDelete) {
		delete s;
	}
	for (RigidBody* b : bodies) {
		delete b;
	}
	for (sf::Shape* s : shapes) {
		delete s;
	}
}
