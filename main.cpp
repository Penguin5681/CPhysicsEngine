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

// --- Constants ---
constexpr float PIXELS_PER_METER = 25.0f;
constexpr int WINDOW_WIDTH = 1200;
constexpr int WINDOW_HEIGHT = 900;

// --- Helper Functions ---
sf::Vector2f toScreenPos(const Vector3& worldPos) {
    return sf::Vector2f(
        worldPos.x * PIXELS_PER_METER + WINDOW_WIDTH / 2.0f,
        -worldPos.y * PIXELS_PER_METER + WINDOW_HEIGHT / 2.0f
    );
}

float toDegrees(const Matrix3x3& rotMatrix) {
    float rad = atan2f(rotMatrix.data[3], rotMatrix.data[0]);
    return std::isnan(rad) ? 0.0f : -rad * 180.0f / M_PI;
}

RigidBody* createBox(Vector3 position, Vector3 halfExtents, float mass, float restitution, float staticFriction, float dynamicFriction) {
    RigidBody* box = new RigidBody();
    box->position = position;
    box->inverseMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    box->restitution = restitution;
    box->staticFriction = staticFriction;
    box->dynamicFriction = dynamicFriction;
    BoundingBox* shapeData = new BoundingBox(halfExtents);
    box->shape = shapeData;
    Vector3 fullDims = halfExtents * 2.0f;
    if (mass > 0.0f) {
        box->inverseInertiaTensor.setInverseInertiaTensorCuboid(mass, fullDims);
    }
    box->orientation.normalize();
    return box;
}

int main() {
    // --- Window Setup ---
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "CPhysicsEngine - Ramp Friction Test");
    window.setFramerateLimit(60);

    // --- Physics Setup ---
    Vector3 gravity = Vector3(0.0f, -9.81f, 0.0f);
    PhysicsWorld world(gravity, 4.0f); // Grid cell size = 4.0
    float timeStep = 1.0f / 60.0f;

    std::vector<RigidBody*> bodies;
    std::vector<sf::Shape*> shapes;
    std::vector<CollisionShape*> shapeDataToDelete;

    // --- Create the Ramp (Static Rotated Box) ---
    float rampAngleDegrees = 25.0f;
    float rampAngleRadians = rampAngleDegrees * M_PI / 180.0f;
    Vector3 rampHalfExtents(25.0f, 1.0f, 10.0f); // Long, wide ramp
    RigidBody* rampBody = createBox(Vector3(0.0f, -10.0f, 0.0f), rampHalfExtents, 0.0f, 0.4f, 0.7f, 0.6f); // Mass 0 = static, high friction
    // Rotate ramp around Z-axis
    rampBody->orientation = Quaternion(cosf(rampAngleRadians / 2.0f), 0, 0, sinf(rampAngleRadians / 2.0f));
    rampBody->orientation.normalize();
    // Manually update matrix once for static body with initial rotation
    rampBody->rotationMatrix.setOrientation(rampBody->orientation);
    rampBody->updateWorldAABB(); // Update AABB once after setting rotation/position

    world.addBody(rampBody);
    bodies.push_back(rampBody);
    shapeDataToDelete.push_back(rampBody->shape);

    sf::RectangleShape* rampGfx = new sf::RectangleShape();
    rampGfx->setSize(sf::Vector2f(rampHalfExtents.x * 2.0f * PIXELS_PER_METER, rampHalfExtents.y * 2.0f * PIXELS_PER_METER));
    rampGfx->setOrigin(rampHalfExtents.x * PIXELS_PER_METER, rampHalfExtents.y * PIXELS_PER_METER);
    rampGfx->setFillColor(sf::Color(100, 100, 150)); // Grey-blue color
    shapes.push_back(rampGfx);

    // --- Create Falling Boxes ---
    Vector3 boxHalfExtents(1.0f, 1.0f, 1.0f);
    float boxMass = 5.0f;
    float boxRestitution = 0.2f; // Low bounce
    float boxStaticFriction = 0.6f;
    float boxDynamicFriction = 0.5f;

    // Box 1
    RigidBody* box1 = createBox(Vector3(-8.0f, 5.0f, 0.0f), boxHalfExtents, boxMass, boxRestitution, boxStaticFriction, boxDynamicFriction);
    world.addBody(box1);
    bodies.push_back(box1);
    shapeDataToDelete.push_back(box1->shape);
    sf::RectangleShape* box1Gfx = new sf::RectangleShape();
    box1Gfx->setSize(sf::Vector2f(boxHalfExtents.x * 2.0f * PIXELS_PER_METER, boxHalfExtents.y * 2.0f * PIXELS_PER_METER));
    box1Gfx->setOrigin(boxHalfExtents.x * PIXELS_PER_METER, boxHalfExtents.y * PIXELS_PER_METER);
    box1Gfx->setFillColor(sf::Color::Red);
    shapes.push_back(box1Gfx);

    // Box 2
    RigidBody* box2 = createBox(Vector3(-5.0f, 10.0f, 0.0f), boxHalfExtents, boxMass, boxRestitution, boxStaticFriction, boxDynamicFriction);
    world.addBody(box2);
    bodies.push_back(box2);
    shapeDataToDelete.push_back(box2->shape);
    sf::RectangleShape* box2Gfx = new sf::RectangleShape();
    box2Gfx->setSize(sf::Vector2f(boxHalfExtents.x * 2.0f * PIXELS_PER_METER, boxHalfExtents.y * 2.0f * PIXELS_PER_METER));
    box2Gfx->setOrigin(boxHalfExtents.x * PIXELS_PER_METER, boxHalfExtents.y * PIXELS_PER_METER);
    box2Gfx->setFillColor(sf::Color::Green);
    shapes.push_back(box2Gfx);

    // Box 3
    RigidBody* box3 = createBox(Vector3(-2.0f, 15.0f, 0.0f), boxHalfExtents, boxMass, boxRestitution, boxStaticFriction, boxDynamicFriction);
    world.addBody(box3);
    bodies.push_back(box3);
    shapeDataToDelete.push_back(box3->shape);
    sf::RectangleShape* box3Gfx = new sf::RectangleShape();
    box3Gfx->setSize(sf::Vector2f(boxHalfExtents.x * 2.0f * PIXELS_PER_METER, boxHalfExtents.y * 2.0f * PIXELS_PER_METER));
    box3Gfx->setOrigin(boxHalfExtents.x * PIXELS_PER_METER, boxHalfExtents.y * PIXELS_PER_METER);
    box3Gfx->setFillColor(sf::Color::Blue);
    shapes.push_back(box3Gfx);

    // --- Game Loop ---
    while (window.isOpen()) {
        // --- Handle Events ---
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            // Add mouse dragging later if needed
        }

        // --- Physics Step ---
        world.step(timeStep);

        // --- Update Graphics ---
        for (size_t i = 0; i < bodies.size(); ++i) {
            RigidBody* body = bodies[i];
            sf::Shape* shape = shapes[i];

            shape->setPosition(toScreenPos(body->position));
            shape->setRotation(toDegrees(body->rotationMatrix));
        }

        // --- Draw ---
        window.clear(sf::Color(40, 40, 60)); // Dark background
        for (const auto* shape : shapes) {
            window.draw(*shape);
        }
        window.display();
    }

    // --- Cleanup ---
    for(CollisionShape* s : shapeDataToDelete) { delete s; }
    for(RigidBody* b : bodies) { delete b; }
    for(sf::Shape* s : shapes) { delete s; }

    return 0;
}