#pragma once

#include "vex.h"
#include <cmath>
#include <bits/stdc++.h>

using namespace vex;

class Button {
    public:
        Button(const vex::controller::button *thisButton);
        void update();
        bool justPressed();
        bool justReleased();
        bool pressed();
    private:
        const vex::controller::button *thisButton;
        bool isPressed = false;
        bool lastPressed = false;


};