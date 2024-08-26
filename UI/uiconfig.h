//
// Created by kaloyantsankov on 2024
//

#ifndef UICONFIG_H
#define UICONFIG_H
#include <string>
#include <vector>
using namespace std;

namespace UI::msg {
    inline int TITLE_SPACING = 2;
    inline int FOOTER_SPACING = 3;
    inline string U_MSG_SELECT = "Select an option: ";
    inline string U_MSG_FOOTER_PREV = "R - Return to previous screen";
    inline string U_MSG_FOOTER_EXIT = "E - Exit";

    namespace logo {
        inline int LOGO_DELAY = 1500;
        inline string LOGO_ART[] = {
            R"(      ___           ___           ___           ___           ___     )",
            R"(     /\  \         /\  \         /\  \         /\__\         /\  \    )",
            R"(    /::\  \       /::\  \       /::\  \       /::|  |       /::\  \   )",
            R"(   /::::\  \     /:/\ \  \     /:/\:\  \     /:|:|  |      /::::\  \  )",
            R"(  /::::::\  \   _\:\~\ \  \   /::\~\:\  \   /:/|:|__|__   /::::::\  \ )",
            R"( /::::::::\__\ /\ \:\ \ \__\ /:/\:\ \:\__\ /:/ |::::\__\ /::::::::\__\)",
            R"( \::::::::/  / \:\ \:\ \/__/ \/__\:\ \/__/ \/__/~~/:/  / \::::::::/  /)",
            R"(  \::::::/  /   \:\ \:\__\        \:\__\         /:/  /   \::::::/  / )",
            R"(   \::::/  /     \:\/:/  /         \/__/        /:/  /     \::::/  /  )",
            R"(    \::/  /       \::/  /                      /:/  /       \::/  /   )",
            R"(     \/__/         \/__/                       \/__/         \/__/    )"
        };
    }

    namespace main {
        inline string U_MSG_MENU_TITLE = "3D-SFM: 3D Reconstruction / Structure from Motion";
        inline string U_MSG_MENU = "Please choose an option:";
        inline string U_MSG_MENU_TEMPLATE_SELECTION = "1. Choose from the predefined templates";
        inline string U_MSG_MENU_MANUAL = "2. Run Manually";
        inline string U_MSG_MENU_EXIT = "3. Exit the application";
    }

    namespace templ {
        inline string U_MSG_TEMPLATE_TITLE = "3D-SFM: Template Selection";
        inline string U_MSG_TEMPLATE_SELECTION = "Select a template:";
        inline string U_MSG_TEMPLATE_SELECTED;
        inline string U_MSG_TEMPLATE_OPTION1 = "1. Statuette (Turn-table dataset)";
        inline string U_MSG_TEMPLATE_OPTION2 = "2. Herz-Jesu-Kirche Church";
        inline string U_MSG_TEMPLATE_OPTION3 = "3. Fountain";
    }

    namespace templ::dsp {
        inline string U_MSG_TEMPDSP_TITLE = "3D-SFM: Template Properties >> ";
        inline string U_MSG_TEMPDSP_CONFIGURE = "\"N\": Edit the N-th property value";
        inline string U_MSG_TEMPDSP_RUN = "C - Continue with execution";
        inline string U_MSG_TEMPDSP_KCALIB = "K - Camera Calibration: ";

        inline string U_MSG_TEMPDSP_FOLDERPATH = ". Path: ";
        inline string U_MSG_TEMPDSP_FILEEXT = ". File extension: ";
        inline string U_MSG_TEMPDSP_IMGSTART = ". Start from image : ";
        inline string U_MSG_TEMPDSP_IMGEND = ". Stop after image (0 = END): ";
        inline string U_MSG_TEMPDSP_RESRAT = ". Resize (0-1, affects performance): ";
        inline string U_MSG_TEMPDSP_MAXTHREADS = ". Hyper-threading (cores to use): ";
        inline string U_MSG_TEMPDSP_ETW = ". Error threshold width: ";
        inline string U_MSG_TEMPDSP_ETH = ". Error threshold height: ";
        inline string U_MSG_TEMPDSP_VIS = ". Visualise keypoints: ";
        inline string U_MSG_TEMPDSP_CCV = ". Use custom replacements for CV functions: ";
    }
}

#endif //UICONFIG_H
