//
// Created by kaloyantsankov - 2024.
//

#ifndef UI_H
#define UI_H
#include <iostream>
#include <string>
#include <thread>
#include "uiconfig.h"
#include "../helper/templates/templates.h"

using namespace std;

namespace UI {
    class menu {
    public:
        menu();

        void launch();

        void logo();

        void main_screen();

        void templates();

        void template_display();

        void config_property(string &sprop);

        void config_property(double &fprop);

        void config_property(int &iprop);

        void config_property(bool &bprop);
    };
} // ui

#endif //UI_H
