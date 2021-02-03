// Hacks to get gnuboy to compile 
#include "rc.h"
#include "pcm.h"
#include "fb.h"
#include "hw.h"
#include "sys.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define GB_WIDTH 160
#define GB_HEIGHT 144
#define GB_OFFSET 0

uint8_t * fb_ram;
uint32_t * fb_mono;
///static SemaphoreHandle_t fb_mutex;
SemaphoreHandle_t fb_mutex;
volatile int done_drawing;

int framenum;
  
extern uint8_t* SCREENMEMORY[256+1];
extern uint8_t * fb_ram;
extern uint8_t DEFAULTPAL;

extern uint8_t JOY_UP;
extern uint8_t JOY_DOWN;
extern uint8_t JOY_LEFT;
extern uint8_t JOY_RIGHT;
extern uint8_t JOY_CROSS; //A
extern uint8_t JOY_SQUARE; //B
extern uint8_t JOY_CIRCLE;
extern uint8_t JOY_TRIANGLE;
extern uint8_t JOY_SHARE; //(START)
extern uint8_t JOY_OPTIONS; //(SELECT)

extern void pwm_audio_write_(uint8_t *inbuf, size_t len, size_t *bytes_written, TickType_t ticks_to_wait);

// badge button constants 
enum button_t {
  BUTTON_UP     =  1,
  BUTTON_DOWN   =  2,
  BUTTON_LEFT   =  3,
  BUTTON_RIGHT  =  4,

  BUTTON_A      =  6,
  BUTTON_B      =  7,
  BUTTON_SELECT =  8,
  BUTTON_START  =  9,
  BUTTON_FLASH  = 10,

  // Number of buttons on the badge
  BUTTONS       = 10,
}; 

// What is "black" versus "white" on the mono display?
#define THRESHOLD 0x60

extern uint32_t badge_input_button_state;

void vid_setpal(int i, int r, int g, int b)
{
	// should set the pallete; ignore for now
}

struct pcm pcm;
struct fb fb;

// audio to process
int pcm_submit() { 
   if (pcm.buf)  {
      if (pcm.pos >= pcm.len) pcm.pos = pcm.len;
      size_t *bytes_written_=0;
      pwm_audio_write_((uint8_t*)pcm.buf, pcm.pos ,&bytes_written_, 1);
      pcm.pos=0;
   } else {
      pcm.buf=ps_malloc(2048*2);
      pcm.len=2048;
      pcm.stereo=0;
   }
   return 0; 
}


// timers are just uint32_t
void * sys_timer()
{
	 static unsigned long timer;
	 return &timer;
}

extern bool buttonchange;

void doevents()
{  
   uint32_t button=buttonchange; ///
	 if (button == 0) return;
   uint32_t buttons = badge_input_button_state;
	 static uint32_t old_buttons;
	 uint32_t delta = buttons ^ old_buttons;

	 if (delta == 0)	return;

	 //printf("%d %08x\n", button, buttons, delta);
  
	 if (delta & (1 << BUTTON_UP)) pad_set(PAD_UP, buttons & (1 << BUTTON_UP));
	 if (delta & (1 << BUTTON_DOWN)) pad_set(PAD_DOWN, buttons & (1 << BUTTON_DOWN));
	 if (delta & (1 << BUTTON_LEFT)) pad_set(PAD_LEFT, buttons & (1 << BUTTON_LEFT));
	 if (delta & (1 << BUTTON_RIGHT)) pad_set(PAD_RIGHT, buttons & (1 << BUTTON_RIGHT));
	 if (delta & (1 << BUTTON_START)) pad_set(PAD_START, buttons & (1 << BUTTON_START));
	 if (delta & (1 << BUTTON_SELECT)) pad_set(PAD_SELECT, buttons & (1 << BUTTON_SELECT));
	 if (delta & (1 << BUTTON_A)) pad_set(PAD_A, buttons & (1 << BUTTON_A));
	 if (delta & (1 << BUTTON_B))	pad_set(PAD_B, buttons & (1 << BUTTON_B));

  	old_buttons = badge_input_button_state;
   buttonchange=false;
 	 pad_refresh();
}

void fb_draw_task(void * arg) {
	 (void) arg;
	 while(1) {	
		  if (!xSemaphoreTake(fb_mutex, 1000)) {
			   printf("waiting for frame\n");
			   continue;
		  }

      done_drawing = 1;

      if (DEFAULTPAL==1)
      for (uint32_t tmp=0;tmp<160*144;tmp++) {
         uint8_t RED=0;
         uint8_t GREEN=0;
         uint8_t BLUE=0;
         uint32_t c = fb_ram[tmp*2];
         c |= (fb_ram[tmp*2+1]<<16);

         RED=(c&0b000001100000000000000000 )>>17;
         GREEN=(c&0b110000000000000000000000)>>22;
         BLUE=(c&0b000000000000000000011000)>>3; //OK.      
         c=(RED<<4 | GREEN<<2 | BLUE<<0);
         SCREENMEMORY[(tmp/160)* 15/9][(tmp%160)*3 /2]=c;
         SCREENMEMORY[(tmp/160)* 15/9 + 1][(tmp%160)*3 /2]=c;
         SCREENMEMORY[(tmp/160)* 15/9][(tmp%160)*3 /2  + 1]=c;
         SCREENMEMORY[(tmp/160)* 15/9 + 1][(tmp%160)*3 /2 +1]=c;  
      } 
      lcd_write_frame(0,0,240,240);  
///		done_drawing = 1;
	 }
}


void vid_init()
{
  ///  fb_ram = calloc(BADGE_EINK_WIDTH*GB_HEIGHT, 1);
  fb_ram = ps_calloc(GB_WIDTH*GB_HEIGHT * 2 , 1);
	if (!fb_ram) die("fb alloc failed\n");

	// use the pitch of the eink display, but the width of
	// the game boy display.
	fb.w = GB_WIDTH;
	fb.h = GB_HEIGHT;
  fb.pitch = GB_WIDTH*2;

	fb.ptr = fb_ram;
	fb.pelsize = 2;
	fb.indexed = 0;

	// we have no color, but we pack r/g/b into 8 bits
	fb.cc[0].r = 5;
	fb.cc[1].r = 5;
	fb.cc[2].r = 6;

	fb.cc[0].l = 0;
	fb.cc[1].l = 3;
	fb.cc[2].l = 6;
	fb.enabled = 1;
	fb.dirty = 1;
 
	// setup our redraw task
	fb_mutex = xSemaphoreCreateMutex();
	xSemaphoreTake(fb_mutex, 0);

	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;

  ///xTaskCreatePinnedToCore(&fb_draw_task, "fb_draw_task", 2048, NULL, 1, NULL, 0 );

	/* Create the task, storing the handle. */

	xReturned = xTaskCreate(
		fb_draw_task,       // Function that implements the task. 
		"eink",          // Text name for the task. 
		8192,      // Stack size in words, not bytes. 
		NULL,    // Parameter passed into the task. 
		tskIDLE_PRIORITY,// Priority at which the task is created.   
   	&xHandle      // Used to pass out the created task's handle. 
	);


	if (!xReturned)die("unable to create fb thread\n");
}

void vid_begin()
{
	//printf("begin\n");
}

void vid_end()
{
	 static unsigned long last_draw;
	 static unsigned skipped;

	 if(!fb.enabled){
		  if (done_drawing) fb.enabled = 1;
		  skipped++;
		  return;
	 }

	 framenum++;
	 unsigned long delta = sys_micros() - last_draw;
	 printf("frame %d: skip %d %lu\n", framenum, skipped, delta);
	 skipped = 0;

  	// mark that the fb is in use and wake the drawing task
	 done_drawing = fb.enabled = 0;
	 xSemaphoreGive(fb_mutex);
	 last_draw = sys_micros();
}
