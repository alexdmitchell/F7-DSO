//gui.h file
#ifndef _gui_h
#define _gui_h

void drawmain (void);
void drawstatus (void);
void drawrun (void);
void drawstop (void);
void cleartrace (void);
void clearstatus(void);
void drawcontrollimit (void);
void drawnosignal (void);
void drawground (int yval);
void drawfault (void);
void drawquickmeasure (float vmin, float vmax, float freq);
#endif
