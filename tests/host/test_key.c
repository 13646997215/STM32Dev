#include "test.h"
/* PA0 semantics: a short press starts one unified round, a 2 s hold interrupts
   (or clears a latched fault, or stops a manual session). A fault must never be
   started by a short press, and a long hold must never read as a start. */
int main(void){
 /* short press: released before 500 ms, nothing armed, round idle/finished */
 CHECK(Key_Event(120,1,MS_IDLE,0)==KEY_START);
 CHECK(Key_Event(120,1,MS_ARRIVED,0)==KEY_START);
 CHECK(Key_Event(120,1,MS_FAULT,0)==KEY_NONE);       /* clear deliberately */
 CHECK(Key_Event(120,1,MS_ALIGN,0)==KEY_NONE);
 CHECK(Key_Event(120,1,MS_BALANCE,0)==KEY_NONE);
 CHECK(Key_Event(120,1,MS_RETURN,0)==KEY_NONE);
 CHECK(Key_Event(120,1,MS_IDLE,1)==KEY_NONE);        /* already armed */
 /* still held: no event yet */
 CHECK(Key_Event(120,0,MS_IDLE,0)==KEY_NONE);
 /* released after the short-press window but before 2 s: ignore */
 CHECK(Key_Event(900,1,MS_IDLE,0)==KEY_NONE);
 CHECK(Key_Event(0,1,MS_IDLE,0)==KEY_NONE);
 /* 2 s hold interrupts everything that is busy, and clears a fault */
 CHECK(Key_Event(2000,0,MS_ALIGN,1)==KEY_STOP);
 CHECK(Key_Event(2000,0,MS_BALANCE,1)==KEY_STOP);
 CHECK(Key_Event(2000,0,MS_RETURN,1)==KEY_STOP);
 CHECK(Key_Event(2000,0,MS_FAULT,0)==KEY_STOP);
 CHECK(Key_Event(2000,0,MS_IDLE,1)==KEY_STOP);       /* manual session armed */
 CHECK(Key_Event(2000,0,MS_IDLE,0)==KEY_NONE);       /* nothing to stop */
 CHECK(Key_Event(5000,0,MS_BALANCE,1)==KEY_STOP);
 /* a long hold must never read as a start */
 CHECK(Key_Event(2000,1,MS_IDLE,0)!=KEY_START);
 DONE();}
