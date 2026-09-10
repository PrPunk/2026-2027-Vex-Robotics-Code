/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       brain-display.h                                           */
/*    Author:       Josiah Schatz                                             */
/*    Created:      Sat 23 Aug 2025                                           */
/*    Description:  Team 986B bot for 2025-2026                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <string.h>
#include <array>
#include "images.h"

struct Button {
    unsigned int x;
    unsigned int y;
    unsigned int width;
    unsigned int height;
    char text[20];
    unsigned int color;
    void (*callback)(int);
    int param;
};

struct Page {
    Button *buttons[10];
    bool hasImage;

    Page() {
        for (int i = 0; i < 10; i++) {
            buttons[i] = nullptr;   // initialize
        }
    }
};