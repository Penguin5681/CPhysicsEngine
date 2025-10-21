#include <SFML/Graphics.hpp>
#include <SFML/System/Clock.hpp>
#include <imgui.h>
#include <imgui-SFML.h>

#include "./src/Core/Vector3.h"
#include "./src/Core/Quaternion.h"
#include "./src/Dynamics/PhysicsWorld.h"
#include "./src/Dynamics/RigidBody.h"
#include "./src/Collision/BoundingBox.h"
#include "./src/Collision/CollisionShape.h"

#include <vector>
#include <cmath>
#include <iostream>
#include <optional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr float PIXELS_PER_METER = 25.0f;
constexpr unsigned int WINDOW_WIDTH = 1200;
constexpr unsigned int WINDOW_HEIGHT = 800;

sf::Vector2f toScreenPos(const Vector3& worldPos) {
  return sf::Vector2f(
   worldPos.x * PIXELS_PER_METER + WINDOW_WIDTH / 2.0f,
   -worldPos.y * PIXELS_PER_METER + WINDOW_HEIGHT / 2.0f
  );
}

sf::Angle toAngle(const Matrix3x3& rotMatrix) {
  float rad = atan2f(rotMatrix.data[3], rotMatrix.data[0]);
  return std::isnan(rad) ? sf::degrees(0.0f) : sf::radians(-rad);
}

int main() {
  sf::RenderWindow window(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}), "Simple Physics Test");
  window.setFramerateLimit(60);

  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  if (!ImGui::SFML::Init(window)) {
   std::cerr << "Failed to initialize ImGui-SFML!" << std::endl;
   return -1;
  }
  sf::Clock deltaClock;

  Vector3 gravity = Vector3(0.0f, -9.81f, 0.0f);
  PhysicsWorld world(gravity, 4.0f);
  float timeStep = 1.0f / 60.0f;

  std::vector<RigidBody*> bodies;
  std::vector<sf::Shape*> shapes;
  std::vector<CollisionShape*> shapeDataToDelete;

  RigidBody* floorBody = new RigidBody();
  floorBody->position = Vector3(0.0f, -15.0f, 0.0f);
  floorBody->inverseMass = 0.0f;
  floorBody->restitution = 0.4f;
  floorBody->staticFriction = 0.7f;
  floorBody->dynamicFriction = 0.6f;
  BoundingBox* floorShapeData = new BoundingBox(Vector3(25.0f, 1.0f, 1.0f));
  floorBody->shape = floorShapeData;
  floorBody->updateWorldAABB();

  world.addBody(floorBody);
  bodies.push_back(floorBody);
  shapeDataToDelete.push_back(floorBody->shape);

  sf::RectangleShape* floorGfx = new sf::RectangleShape();
  floorGfx->setSize({
   floorShapeData->halfExtents.x * 2.0f * PIXELS_PER_METER,
   floorShapeData->halfExtents.y * 2.0f * PIXELS_PER_METER
  });
  floorGfx->setOrigin({
   floorShapeData->halfExtents.x * PIXELS_PER_METER,
   floorShapeData->halfExtents.y * PIXELS_PER_METER
  });
  floorGfx->setFillColor(sf::Color::White);
  shapes.push_back(floorGfx);

  RigidBody* fallingBox = new RigidBody();
  fallingBox->position = Vector3(0.0f, 10.0f, 0.0f);
  fallingBox->inverseMass = 1.0f / 10.0f;
  fallingBox->restitution = 0.3f;
  fallingBox->staticFriction = 0.5f;
  fallingBox->dynamicFriction = 0.4f;
  BoundingBox* boxShapeData = new BoundingBox(Vector3(1.5f, 1.5f, 1.0f));
  fallingBox->shape = boxShapeData;
  fallingBox->inverseInertiaTensor.setInverseInertiaTensorCuboid(10.0f, boxShapeData->halfExtents * 2.0f);

  world.addBody(fallingBox);
  bodies.push_back(fallingBox);
  shapeDataToDelete.push_back(fallingBox->shape);

  auto* boxGfx = new sf::RectangleShape();
  boxGfx->setSize({
   boxShapeData->halfExtents.x * 2.0f * PIXELS_PER_METER,
   boxShapeData->halfExtents.y * 2.0f * PIXELS_PER_METER
  });
  boxGfx->setOrigin({
   boxShapeData->halfExtents.x * PIXELS_PER_METER,
   boxShapeData->halfExtents.y * PIXELS_PER_METER
  });
  boxGfx->setFillColor(sf::Color::Cyan);
  shapes.push_back(boxGfx);

	// Game Loop
	while (window.isOpen()) {
		std::optional<sf::Event> event = window.pollEvent();
		while (event) {
			ImGui::SFML::ProcessEvent(window, *event);

			if (event->is<sf::Event::Closed>()) {
				window.close();
			}
			event = window.pollEvent();
		}

		sf::Time deltaTime = deltaClock.restart();
		ImGui::SFML::Update(window, deltaTime);

		world.step(timeStep);

		ImGui::Begin("Info");
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
		ImGui::End();

		for (size_t i = 0; i < bodies.size(); ++i) {
			RigidBody* body = bodies[i];
			sf::Shape* shape = shapes[i];

			shape->setPosition(toScreenPos(body->position));
			shape->setRotation(toAngle(body->rotationMatrix));
		}

		window.clear(sf::Color(40, 40, 60));
		for (const auto* shape : shapes) {
			window.draw(*shape);
		}

		ImGui::SFML::Render(window);

		window.display();
	}

  ImGui::SFML::Shutdown();
  ImGui::DestroyContext();

  for (CollisionShape* s : shapeDataToDelete) { delete s; }
  for (RigidBody* b : bodies) { delete b; }
  for (sf::Shape* s : shapes) { delete s; }

  return 0;
}