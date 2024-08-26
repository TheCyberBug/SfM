#include "SfMPipeline.h"
#include "UI/menu.h"
#include "helper/templates/templates.h"

int main() {
    UI::menu userInterface;
    while (true) {
        userInterface.launch();
        sfm::SfMPipeline::run();
    }
    return 0;
}
