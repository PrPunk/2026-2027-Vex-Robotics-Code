/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       brain-display.cpp                                         */
/*    Author:       Josiah Schatz                                             */
/*    Created:      Sat 23 Aug 2025                                           */
/*    Description:  Team 986B bot for 2025-2026                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <brain-display.h>

class BrainDisplay {
    public:
        Page pages[5];
        int currentPage = 0;


        
        void createButton(unsigned int pageId, unsigned int buttonId, Button newButton) {
            if (!(pageId < 5 && pageId >= 0)) {
                //error: pageId is not within valid range
                printf("pageId is not within valid range\n");
                return;
            }
            std::array<int, 10> reservedButtons = usedButtons(pageId);

            for (int i = 0; i < 10; i++) {
                if (reservedButtons[i] == -1) break;  // stop at unused slots
                if (reservedButtons[i] == buttonId) {
                    printf("buttonId is in use\n");
                    return;
                }
            }
            if (newButton.width == 0 || newButton.height == 0 || newButton.callback == NULL) {
                //error: button is incomplete. Missing required values.
                printf("button is incomplete. Missing required values.\n");
                return;
            }
            drawButton(newButton);
            pages[pageId].buttons[buttonId] = new Button(newButton);
            printf("button created\n");

        }

        void screenPressed (void) {
            int x = Brain.Screen.xPosition();
            int y = Brain.Screen.yPosition();
            for (int i = 0; i < 10; i++) {
                Button* questioningButton = pages[currentPage].buttons[i];
                if (questioningButton != nullptr &&
                    x >= questioningButton->x &&
                    x <= (questioningButton->x + questioningButton->width) &&
                    y >= questioningButton->y &&
                    y <= (questioningButton->y + questioningButton->height)) {
                    questioningButton->callback(questioningButton->param);
                }
            }
        }

        void switchPage (unsigned int pageId) {
            if (pageId < 5) {
                Brain.Screen.clearScreen();
                currentPage = pageId;
                if (pages[pageId].hasImage) {
                    drawLogo();
                }
                for (int i = 0; i < 10; i++) {
                    if (pages[pageId].buttons[i] != nullptr) {
                        drawButton(*pages[pageId].buttons[i]);
                    }
                }
            } else {
                //error: pageId is out of range.
            }
        }

        void addImage(unsigned int pageId) {
            if (pageId < 5) {
                pages[pageId].hasImage = true;
                if (currentPage == pageId) {
                    switchPage(pageId);
                }
            } else {
                //error: pageId is out of range.
            }
        }

    private:
        bool isHexColor(int value) {
            return value >= 0x000000 && value <= 0xFFFFFF;
        }
        void drawButton(Button newButton) {
            if (isHexColor(newButton.color)) {
                Brain.Screen.setFillColor(newButton.color);
                Brain.Screen.setPenColor(newButton.color);
                Brain.Screen.drawRectangle(newButton.x, newButton.y, newButton.width, newButton.height);
            } else {
                Brain.Screen.setFillColor(black);
            }
            if (strlen(newButton.text) > 0) {
                unsigned int midX = newButton.x + (newButton.width/2) - (strlen(newButton.text)*5);
                unsigned int midY = newButton.y + (newButton.height/2) - 8;
                Brain.Screen.setPenColor(white);
                Brain.Screen.printAt(midX, midY, newButton.text);
            }
        }
        std::array<int, 10> usedButtons(unsigned int pageId) {
            std::array<int, 10> buttonsUsed;
            buttonsUsed.fill(-1);  // sentinel for “empty”
            int usedCount = 0;

            for (int i = 0; i < 10; i++) {
                if (pages[pageId].buttons[i] != nullptr && pages[pageId].buttons[i]->x != 0) {
                    buttonsUsed[usedCount] = i;
                    usedCount++;
                }
            }

            return buttonsUsed;
        }

};
