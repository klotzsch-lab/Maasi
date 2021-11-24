#include "Nextion.h"

// Nextion 
// (Page, Object, Name)

NexText t0 = NexText(0, 11, "t0");
NexText t1 = NexText(0, 12, "t1");
NexText t2 = NexText(0, 15, "t2");
NexText t3 = NexText(0, 16, "t3");
NexText t4 = NexText(0, 17, "t4");
NexText p2t4 = NexText(2, 11, "p2t4");


NexPage p0 = NexPage(0, 0, "page0");
NexPage p1 = NexPage(1, 0, "page1");
NexPage p2 = NexPage(2, 0, "page2");

// Note that if you delete Objects in the Nextion program the Object IDs might change.


NexButton b1 = NexButton(0, 1, "b1");
NexButton b2 = NexButton(0, 4, "b2");
NexButton b3 = NexButton(0, 6, "b3");
NexButton b4 = NexButton(0, 7, "b4");
NexButton b5 = NexButton(0, 8, "b5");
NexButton b6 = NexButton(0, 11, "b6");
NexButton p1b0 = NexButton(1, 10, "p1b0");
NexButton p1b1 = NexButton(1, 11, "p1b1");
NexButton p1b2 = NexButton(1, 5, "p1b2");
NexButton p1b3 = NexButton(1, 6, "p1b3");
NexButton p1b4 = NexButton(1, 14, "p1b4");
NexButton p1b5 = NexButton(1, 15, "p1b5");
NexButton p1b6 = NexButton(1, 18, "p1b6");
NexButton p1b7 = NexButton(1, 19, "p1b7");
NexButton p2b0 = NexButton(2, 12, "p2b0");
NexButton p2b1 = NexButton(2, 1, "p2b1");

NexNumber n0 = NexNumber(0, 3, "n0");
NexNumber n1 = NexNumber(0, 5, "n1");
NexNumber p1n0 = NexNumber(1, 4, "p1n0");
NexNumber p1n1 = NexNumber(1, 9, "p1n1");
NexNumber p1n2 = NexNumber(1, 13, "p1n2");
NexNumber p1n3 = NexNumber(1, 17, "p1n3");
NexNumber p2n0 = NexNumber(2, 4, "p2n0");
NexNumber p2n1 = NexNumber(2, 5, "p2n1");


NexProgressBar p2j0 = NexProgressBar(2, 2, "j0");

NexTouch *nex_listen_list[] = 
{
    &b1,
    &b2,
    &b3,
    &b4,
    &b5,
    &b6,
    &p1b0,
    &p1b1,    
    &p1b2,
    &p1b3,
    &p1b4,
    &p1b5,    
    &p1b6,
    &p1b7,    
    &p2b0,
    &p2b1,        
    NULL
};
