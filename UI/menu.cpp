//
// Created by kaloyantsankov - 2024.
//

#include "menu.h"
#define tspacer std::string(msg::TITLE_SPACING, '\n');
#define fspacer std::string(msg::FOOTER_SPACING, '\n');


UI::menu::menu() = default;

void UI::menu::launch() {
    logo();
    main_screen();
}

void UI::menu::logo() {
    system("clear");
    for (const auto &logo_line: msg::logo::LOGO_ART) {
        cout << logo_line << endl;
    }
    cout << endl << endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(msg::logo::LOGO_DELAY));
}

void UI::menu::main_screen() {
    system("clear");
    cout << msg::main::U_MSG_MENU_TITLE << endl;
    cout << msg::main::U_MSG_MENU << endl;
    cout << tspacer;
    cout << msg::main::U_MSG_MENU_TEMPLATE_SELECTION << endl;
    cout << msg::main::U_MSG_MENU_MANUAL << endl;
    cout << fspacer;
    cout << msg::U_MSG_FOOTER_EXIT << endl;

    char choice;
    cout << msg::U_MSG_SELECT;
    cin >> choice;
    switch (choice) {
        case '1': templates();
            break;
        case '2':
            msg::templ::U_MSG_TEMPLATE_SELECTED = "MANUAL";
            template_display();
            break;
        case 'E':
        case 'e': exit(0);
            break;
        default: main_screen();
            break;
    }
}

void UI::menu::templates() {
    system("clear");
    cout << msg::templ::U_MSG_TEMPLATE_TITLE << endl;
    cout << msg::templ::U_MSG_TEMPLATE_SELECTION << endl;
    cout << tspacer;
    cout << msg::templ::U_MSG_TEMPLATE_OPTION1 << endl;
    cout << msg::templ::U_MSG_TEMPLATE_OPTION2 << endl;
    cout << msg::templ::U_MSG_TEMPLATE_OPTION3 << endl;
    cout << fspacer;
    cout << msg::U_MSG_FOOTER_PREV << endl;
    cout << msg::U_MSG_FOOTER_EXIT << endl;

    char choice;
    cout << msg::U_MSG_SELECT;
    cin >> choice;
    switch (choice) {
        case '1':
            templates::statue();
            msg::templ::U_MSG_TEMPLATE_SELECTED = "Statue";
            break;
        case '2':
            templates::herz();
            msg::templ::U_MSG_TEMPLATE_SELECTED = "Herz-Jesu-Kirche Church";
            break;
        case '3':
            templates::fountain();
            msg::templ::U_MSG_TEMPLATE_SELECTED = "Fountain";
            break;
        case 'R':
        case 'r': main_screen();
            break;
        case 'E':
        case 'e': exit(0);
            break;
        default: templates();
            break;
    }
    template_display();
}

void UI::menu::template_display() {
    system("clear");
    cout << msg::templ::dsp::U_MSG_TEMPDSP_TITLE << msg::templ::U_MSG_TEMPLATE_SELECTED << endl;
    cout << tspacer;

    int i = 0;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_FOLDERPATH << dops::folderPath << endl;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_FILEEXT << dops::fileExt << endl;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_IMGSTART << dops::IMAGE_START << endl;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_IMGEND << dops::IMAGE_END << endl;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_RESRAT << dops::RESIZE_RATIO << endl;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_MAXTHREADS << feat::MAX_THREADS << endl;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_ETW << obj::E_THRESHOLD_WIDTH << endl;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_ETH << obj::E_THRESHOLD_HEIGHT << endl;
    cout << endl;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_VIS << viz << endl;
    cout << i++ << msg::templ::dsp::U_MSG_TEMPDSP_CCV << ccv << endl;
    cout << msg::templ::dsp::U_MSG_TEMPDSP_KCALIB << kcalib::CALIBRATE << endl;
    cout << endl;
    cout << msg::templ::dsp::U_MSG_TEMPDSP_CONFIGURE << endl;
    cout << fspacer;
    cout << msg::templ::dsp::U_MSG_TEMPDSP_RUN << endl;
    cout << msg::U_MSG_FOOTER_PREV << endl;
    cout << msg::U_MSG_FOOTER_EXIT << endl;

    char choice;
    cout << msg::U_MSG_SELECT;
    cin >> choice;
    switch (choice) {
        case '0':
            config_property(dops::folderPath);
            break;
        case '1':
            config_property(dops::fileExt);
            break;
        case '2':
            config_property(dops::IMAGE_START);
            break;
        case '3':
            config_property(dops::IMAGE_END);
            break;
        case '4':
            config_property(dops::RESIZE_RATIO);
            break;
        case '5':
            config_property(feat::MAX_THREADS);
            break;
        case '6':
            config_property(obj::E_THRESHOLD_WIDTH);
            break;
        case '7':
            config_property(obj::E_THRESHOLD_HEIGHT);
            break;
        case '8':
            config_property(viz);
            break;
        case '9':
            config_property(ccv);
            break;
        case 'K':
        case 'k':
            config_property(kcalib::CALIBRATE);
            break;
        case 'C':
        case 'c':
            system("clear");
            break;
        case 'R':
        case 'r': templates();
            break;
        case 'E':
        case 'e': exit(0);
            break;
        default: template_display();
            break;
    }
}

// Should have selected C++20... where auto &prop would be valid
void UI::menu::config_property(string &sprop) {
    cout << "Current settings: " << sprop << endl;
    cout << "New settings: ";
    cin >> sprop;
    template_display();
}

void UI::menu::config_property(double &fprop) {
    cout << "Current settings: " << fprop << endl;
    cout << "New settings: ";
    string temp;
    try {
        cin >> temp;
        fprop = stof(temp);
    } catch (...) {
    }
    template_display();
}

void UI::menu::config_property(int &iprop) {
    cout << "Current settings: " << iprop << endl;
    cout << "New settings: ";
    string temp;
    try {
        cin >> temp;
        iprop = stoi(temp);
    } catch (...) {
    }
    template_display();
}

void UI::menu::config_property(bool &bprop) {
    bprop = !bprop;
    template_display();
}
