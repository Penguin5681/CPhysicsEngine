#include <SFML/Graphics.hpp>

int main() {
	sf::RenderWindow window(sf::VideoMode(800, 600), "C++ Physics Engine");

	sf::CircleShape circle(50.f);
	circle.setFillColor(sf::Color::Green);

	circle.setPosition(800.f / 2 - circle.getRadius(), 600.f / 2 - circle.getRadius());

	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				window.close();
			}
		}

		window.clear(sf::Color::Black);

		window.draw(circle);

		window.display();
	}

	return 0;
}
