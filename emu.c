


#include "defs.h"
#include "regs.h"
#include "hw.h"
#include "cpu.h"
#include "mem.h"
#include "lcd.h"
#include "rc.h"
#include "rtc.h"
#include "sys.h"
#include "fb.h"
#include "sound.h"
#include "cpu.h"



extern uint8_t* SCREENMEMORY[256+1];
extern byte *vdest;
extern uint8_t * fb_ram;

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

extern int GBC_EXIT;

static int framelen = 16743;
static int framecount = 0;

void emu_init()
{
	
}


/*
 * emu_reset is called to initialize the state of the emulated
 * system. It should set cpu registers, hardware registers, etc. to
 * their appropriate values at powerup time.
 */

void emu_reset()
{
	hw_reset();
	lcd_reset();
	cpu_reset();
	mbc_reset();
	sound_reset();
}





void emu_step()
{
	cpu_emulate(cpu.lcdc);
}



/* This mess needs to be moved to another module; it's just here to
 * make things work in the mean time. */

void *sys_timer();

void emu_run()
{
	void *timer = sys_timer();
	int delay_;

	vid_begin();
	lcd_begin();
	while (GBC_EXIT==0)
	{
    if (JOY_SHARE == 1 && JOY_OPTIONS == 1) GBC_EXIT=1;
    
    vTaskDelay(1);
		cpu_emulate(2280);
		while (R_LY > 0 && R_LY < 144)
			emu_step();



    
		vid_end();
		rtc_tick();
		sound_mix();
		if (!pcm_submit())
		{
			delay_ = framelen - sys_elapsed(timer);
///printf("PC=%04x %d usec left fb=%d\n", cpu.pc.w[LO], delay_, fb.enabled);
			sys_sleep(delay_);
			sys_elapsed(timer);
		}
		doevents();

		vid_begin();
		if (framecount) { if (!--framecount) die("finished\n"); }
		
		if (!(R_LCDC & 0x80))
			cpu_emulate(32832);
		
		while (R_LY > 0) // wait for next frame 
			emu_step();
	}
}
