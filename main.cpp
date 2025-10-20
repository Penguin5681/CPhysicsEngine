#include "./src/Core/Vector3.h"
#include "./src/Core/Quaternion.h"
#include "./src/Dynamics/PhysicsWorld.h"
#include "./src/Dynamics/RigidBody.h"
#include "./src/Collision/BoundingBox.h"
#include <cmath>
#include <vector>
#include <SFML/Graphics.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr float PIXELS_PER_METER = 30.0f;
constexpr int WINDOW_WIDTH = 1200;
constexpr int WINDOW_HEIGHT = 800;

sf::Vector2f toScreenPos(const Vector3& worldPos) {
	return {
		worldPos.x * PIXELS_PER_METER + WINDOW_WIDTH / 2.0f,
		-worldPos.y * PIXELS_PER_METER + WINDOW_HEIGHT / 2.0f
	};
}

float toDegrees(const Matrix3x3& rotMatrix) {
	const float radians = atan2f(rotMatrix.data[3], rotMatrix.data[0]);
	return -radians * 180.0f / M_PI;
}

int main() {
	sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "CPhysicsEngine - Box Collision Test");
	window.setFramerateLimit(60);

	Vector3 gravity = Vector3(0.0f, -5.0f, 0.0f);
	PhysicsWorld world(gravity);
	float timeStep = 1.0f / 60.0f;

	std::vector<RigidBody*> bodies;

	auto* box1 = new RigidBody();
	box1->position = Vector3(-10.0f, 0.0f, 0.0f);
	box1->velocity = Vector3(5.0f, 2.0f, 0.0f);
	box1->inverseMass = 1.0f / 5.0f;
	box1->restitution = 0.6f;
	auto shape1Data = new BoundingBox(Vector3(1.5f, 1.0f, 1.0f));
	box1->shape = shape1Data;
	box1->inverseInertiaTensor.setInverseInertiaTensorCuboid(5.0f, Vector3(3.0f, 2.0f, 2.0f));
	box1->orientation = Quaternion(cosf(0.1f), 0, 0, sinf(0.1f));
	box1->orientation.normalize();

	auto* box2 = new RigidBody();
	box2->position = Vector3(10.0f, 2.0f, 0.0f);
	box2->velocity = Vector3(-5.0f, 0.0f, 0.0f);
	box2->inverseMass = 1.0f / 8.0f;
	box2->restitution = 0.6f;
	auto* shape2Data = new BoundingBox(Vector3(1.0f, 1.5f, 1.0f));
	box2->shape = shape2Data;
	box2->inverseInertiaTensor.setInverseInertiaTensorCuboid(8.0f, Vector3(2.0f, 3.0f, 2.0f));
	box2->orientation = Quaternion(cosf(-0.3f), 0, 0, sinf(-0.3f));
	box2->orientation.normalize();

	world.addBody(box1);
	world.addBody(box2);
	bodies.push_back(box1);
	bodies.push_back(box2);

	std::vector<sf::RectangleShape> shapes;

	sf::RectangleShape box1Gfx;
	box1Gfx.setSize(sf::Vector2f(shape1Data->halfExtents.x * 2.0f * PIXELS_PER_METER,
	                             shape1Data->halfExtents.y * 2.0f * PIXELS_PER_METER));
	box1Gfx.setOrigin(shape1Data->halfExtents.x * PIXELS_PER_METER,
	                  shape1Data->halfExtents.y * PIXELS_PER_METER);
	box1Gfx.setFillColor(sf::Color::Blue);
	shapes.push_back(box1Gfx);

	sf::RectangleShape box2Gfx;
	box2Gfx.setSize(sf::Vector2f(shape2Data->halfExtents.x * 2.0f * PIXELS_PER_METER,
	                             shape2Data->halfExtents.y * 2.0f * PIXELS_PER_METER));
	box2Gfx.setOrigin(shape2Data->halfExtents.x * PIXELS_PER_METER,
	                  shape2Data->halfExtents.y * PIXELS_PER_METER);
	box2Gfx.setFillColor(sf::Color::Green);
	shapes.push_back(box2Gfx);

	while (window.isOpen()) {
		sf::Event event{};
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				window.close();
			}
		}

		world.step(timeStep);

		for (size_t i = 0; i < bodies.size(); ++i) {
			RigidBody* body = bodies[i];
			sf::RectangleShape& shape = shapes[i];

			shape.setPosition(toScreenPos(body->position));
			shape.setRotation(toDegrees(body->rotationMatrix));
		}

		window.clear(sf::Color::Black);
		for (const auto& shape : shapes) {
			window.draw(shape);
		}
		window.display();
	}

	delete shape1Data;
	delete shape2Data;
	delete box1;
	delete box2;

	return 0;
}
