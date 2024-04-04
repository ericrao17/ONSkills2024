#include "Button.h"

Button::Button(const vex::controller::button *thisButton) {
    this->thisButton = thisButton;
}

void Button::update() {
    lastPressed = isPressed;
    isPressed = thisButton->pressing();
}

bool Button::justPressed() {
    return isPressed && !lastPressed;
}

bool Button::justReleased() {
    return !isPressed && lastPressed;
}

bool Button::pressed() {
    return isPressed;
}
