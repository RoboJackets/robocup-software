#ifndef _BUTTON_H_
#define _BUTTON_H_

enum { KEY_NONE = 0, KEY_CENTER, KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT };

extern volatile uint8_t gButtonTimeout;

void Button_Init(void);
char getkey(void);
char getch(void);

#endif  // _BUTTON_H_
