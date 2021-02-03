//*******************************************************************************//
//*  COMPOSITE VIDEO OUT                                                        *//
//*******************************************************************************//

#define NTSC_COLOR_CLOCKS_PER_SCANLINE 228       // really 227.5 for NTSC but want to avoid half phase fiddling for now
#define NTSC_FREQUENCY (315000000.0/88) ///315000000.0/88
#define NTSC_LINES 262 ///262

#define PAL_COLOR_CLOCKS_PER_SCANLINE 284        // really 283.75 ?
#define PAL_FREQUENCY 4433618.75
#define PAL_LINES 312

int _pal_ = 1;
lldesc_t _dma_desc[4] = {0};
intr_handle_t _isr_handle;

//================================================================================
//================================================================================
// low level HW setup of DAC/DMA/APLL/PWM

// Color clock frequency is 315/88 (3.57954545455)
// DAC_MHZ is 315/11 or 8x color clock
// 455/2 color clocks per line, round up to maintain phase
// HSYNCH period is 44/315*455 or 63.55555..us
// Field period is 262*44/315*455 or 16651.5555us

#define IRE(_x)          ((uint32_t)(((_x)+40)*255/3.3/147.5) << 8)   // 3.3V DAC
#define SYNC_LEVEL       IRE(-40)
#define BLANKING_LEVEL   IRE(0)
#define BLACK_LEVEL      IRE(7.5)
#define GRAY_LEVEL       IRE(50)
#define WHITE_LEVEL      IRE(100)

#define P0 (color >> 16)
#define P1 (color >> 8)
#define P2 (color)
#define P3 (color << 8)

volatile int _line_counter = 0;
volatile int _frame_counter = 0;

int _active_lines;
int _line_count;

int _line_width;
int _samples_per_cc;
const uint32_t* _palette;

float _sample_rate;

int _hsync;
int _hsync_long;
int _hsync_short;
int _burst_start;
int _burst_width;
int _active_start;

int16_t* _burst0 = 0; // pal bursts
int16_t* _burst1 = 0;

//--------------------------------------------------------------------------------
int usec(float us)
{
  uint32_t r = (uint32_t)(us * _sample_rate);
  return ((r + _samples_per_cc) / (_samples_per_cc << 1)) * (_samples_per_cc << 1); // multiple of color clock, word align
}
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
void IRAM_ATTR blit_pal(uint8_t* src, uint16_t* dst)
{
  uint32_t c, color;
  bool even = _line_counter & 1;
  const uint32_t* p = even ? _palette : _palette + 256;

  int left = 0;
  int right = 256;
  uint8_t mask = 0xFF;
  uint8_t c0, c1, c2, c3, c4;
  uint8_t y1, y2, y3;


      // 192 of 288 color clocks wide: roughly correct aspect ratio
      mask = 0x3F;
      if (!even)
        p = _palette + 64;
      dst += 88;
   
  // 4 pixels over 3 color clocks, 12 samples
  // do the blitting
  for (int i = left; i < right; i += 4) {
    c = *((uint32_t*)(src + i));
    color = p[c & mask];
    dst[0 ^ 1] = P0;
    dst[1 ^ 1] = P1;
    dst[2 ^ 1] = P2;
    color = p[(c >> 8) & mask];
    dst[3 ^ 1] = P3;
    dst[4 ^ 1] = P0;
    dst[5 ^ 1] = P1;
    color = p[(c >> 16) & mask];
    dst[6 ^ 1] = P2;
    dst[7 ^ 1] = P3;
    dst[8 ^ 1] = P0;
    color = p[(c >> 24) & mask];
    dst[9 ^ 1] = P1;
    dst[10 ^ 1] = P2;
    dst[11 ^ 1] = P3;
    dst += 12;
  }
}
//--------------------------------------------------------------------------------
void IRAM_ATTR burst_pal(uint16_t* line)
{
  line += _burst_start;
  int16_t* b = (_line_counter & 1) ? _burst0 : _burst1;
  for (int i = 0; i < _burst_width; i += 2) {
    line[i ^ 1] = b[i];
    line[(i + 1) ^ 1] = b[i + 1];
  }
}

//================================================================================
//================================================================================
// ntsc tables

// cc == 3 gives 684 samples per line, 3 samples per cc, 3 pixels for 2 cc
// cc == 4 gives 912 samples per line, 4 samples per cc, 2 pixels per cc

// draw a line of game in NTSC
void IRAM_ATTR blit(uint8_t* src, uint16_t* dst)
{
  uint32_t* d = (uint32_t*)dst;
  const uint32_t* p = _palette;
  uint32_t color, c;
  uint32_t mask = 0xFF;
  int i;

  if (_pal_) {
    blit_pal(src, dst);
    return;
  }

      mask = 0x3F;
      // AAA ABB BBC CCC
      // 4 pixels, 3 color clocks, 4 samples per cc
      // each pixel gets 3 samples, 192 color clocks wide
      for (i = 0; i < 256; i += 4) {
        c = *((uint32_t*)(src + i));
        color = p[c & mask];
        dst[0 ^ 1] = P0;
        dst[1 ^ 1] = P1;
        dst[2 ^ 1] = P2;
        color = p[(c >> 8) & mask];
        dst[3 ^ 1] = P3;
        dst[4 ^ 1] = P0;
        dst[5 ^ 1] = P1;
        color = p[(c >> 16) & mask];
        dst[6 ^ 1] = P2;
        dst[7 ^ 1] = P3;
        dst[8 ^ 1] = P0;
        color = p[(c >> 24) & mask];
        dst[9 ^ 1] = P1;
        dst[10 ^ 1] = P2;
        dst[11 ^ 1] = P3;
        dst += 12;
      }
}
//--------------------------------------------------------------------------------
void IRAM_ATTR burst(uint16_t* line)
{
  if (_pal_) {
    burst_pal(line);
    return;
  }

  int i, phase;
  switch (_samples_per_cc) {
    case 4:
      // 4 samples per color clock
      for (i = _hsync; i < _hsync + (4 * 10); i += 4) {
        line[i + 1] = BLANKING_LEVEL;
        line[i + 0] = BLANKING_LEVEL + BLANKING_LEVEL / 2;
        line[i + 3] = BLANKING_LEVEL;
        line[i + 2] = BLANKING_LEVEL - BLANKING_LEVEL / 2;
      }
      break;
    case 3:
      // 3 samples per color clock
      phase = 0.866025 * BLANKING_LEVEL / 2;
      for (i = _hsync; i < _hsync + (3 * 10); i += 6) {
        line[i + 1] = BLANKING_LEVEL;
        line[i + 0] = BLANKING_LEVEL + phase;
        line[i + 3] = BLANKING_LEVEL - phase;
        line[i + 2] = BLANKING_LEVEL;
        line[i + 5] = BLANKING_LEVEL + phase;
        line[i + 4] = BLANKING_LEVEL - phase;
      }
      break;
  }
}
//--------------------------------------------------------------------------------
void IRAM_ATTR sync(uint16_t* line, int syncwidth)
{
  for (int i = 0; i < syncwidth; i++)
    line[i] = SYNC_LEVEL;
}
//--------------------------------------------------------------------------------

void IRAM_ATTR blanking(uint16_t* line, bool vbl)
{
  int syncwidth = vbl ? _hsync_long : _hsync;
  sync(line, syncwidth);
  for (int i = syncwidth; i < _line_width; i++)
    line[i] = BLANKING_LEVEL;
  if (!vbl)
    burst(line);    // no burst during vbl
}
//--------------------------------------------------------------------------------
// Fancy pal non-interlace
void IRAM_ATTR pal_sync2(uint16_t* line, int width, int swidth)
{
  swidth = swidth ? _hsync_long : _hsync_short;
  int i;
  for (i = 0; i < swidth; i++)
    line[i] = SYNC_LEVEL;
  for (; i < width; i++)
    line[i] = BLANKING_LEVEL;
}
//--------------------------------------------------------------------------------
uint8_t DRAM_ATTR _sync_type[8] = {0, 0, 0, 3, 3, 2, 0, 0};
void IRAM_ATTR pal_sync(uint16_t* line, int i)
{
  uint8_t t = _sync_type[i - 304];
  pal_sync2(line, _line_width / 2, t & 2);
  pal_sync2(line + _line_width / 2, _line_width / 2, t & 1);
}
//--------------------------------------------------------------------------------
// ISR handles audio and video updates
static inline void IRAM_ATTR video_isr(volatile void* vbuf)
{
  if (!SCREENMEMORY) return;
 
  int i = _line_counter++;
  uint16_t* buf = (uint16_t*)vbuf;

  if (_pal_) {
    // pal
    if (i < 32) {
      blanking(buf, false);               // pre render/black 0-32
    } else if (i < _active_lines + 32) {    // active video 32-272
      sync(buf, _hsync);
      burst(buf);
      ///            blit(_lines[i-32],buf + _active_start);
      blit(SCREENMEMORY[i - 32], buf + _active_start);

    } else if (i < 304) {                   // post render/black 272-304
      if (i < 272)                        // slight optimization here, once you have 2 blanking buffers
        blanking(buf, false);
    } else {
      pal_sync(buf, i);                   // 8 lines of sync 304-312
    }
  } else {
    // ntsc
    if (i < _active_lines) {                // active video
      sync(buf, _hsync);
      burst(buf);
      ///            blit(_lines[i],buf + _active_start);
      blit(SCREENMEMORY[i], buf + _active_start);


    } else if (i < (_active_lines + 5)) {   // post render/black
      blanking(buf, false);

    } else if (i < (_active_lines + 8)) {   // vsync
      blanking(buf, true);

    } else {                                // pre render/black
      blanking(buf, false);
    }
  }

  if (_line_counter == _line_count) {
    _line_counter = 0;                      // frame is done
    _frame_counter++;
  }
}

// simple isr
static inline void IRAM_ATTR i2s_intr_handler_video(void *arg)
{
  if (I2S0.int_st.out_eof) video_isr(((lldesc_t*)I2S0.out_eof_des_addr)->buf); // get the next line of video
  I2S0.int_clr.val = I2S0.int_st.val;                     // reset the interrupt
}
//--------------------------------------------------------------------------------
static esp_err_t start_dma(int line_width, int samples_per_cc, int ch = 1)
{
  periph_module_enable(PERIPH_I2S0_MODULE);


  // setup interrupt (DO NOT USE: ESP_INTR_FLAG_IRAM = BLUETOOTH PROBLEM)
  if (esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 /* | ESP_INTR_FLAG_IRAM  | ESP_INTR_FLAG_SHARED*/,
                     i2s_intr_handler_video, 0, &_isr_handle) != ESP_OK)
    return -1;

  // reset conf
  I2S0.conf.val = 1;
  I2S0.conf.val = 0;
  I2S0.conf.tx_right_first = 1;
  I2S0.conf.tx_mono = (ch == 2 ? 0 : 1);


  I2S0.conf2.lcd_en = 1;
  I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
  I2S0.sample_rate_conf.tx_bits_mod = 16;
  I2S0.conf_chan.tx_chan_mod = (ch == 2) ? 0 : 1;

  // Create TX DMA buffers
  for (int i = 0; i < 2; i++) {
    int n = line_width * 2 * ch;
    if (n >= 4092) {
      printf("DMA chunk too big:%s\n", n);
      return -1;
    }
    _dma_desc[i].buf = (uint8_t*)heap_caps_calloc(1, n, MALLOC_CAP_DMA);
    if (!_dma_desc[i].buf) return -1;

    _dma_desc[i].owner = 1;
    _dma_desc[i].eof = 1;
    _dma_desc[i].length = n;
    _dma_desc[i].size = n;
    _dma_desc[i].empty = (uint32_t)(i == 1 ? _dma_desc : _dma_desc + 1);
  }
  I2S0.out_link.addr = (uint32_t)_dma_desc;

//===============================================================================

  //  Setup up the apll: See ref 3.2.7 Audio PLL
  //  f_xtal = (int)rtc_clk_xtal_freq_get() * 1000000;
  //  f_out = xtal_freq * (4 + sdm2 + sdm1/256 + sdm0/65536); // 250 < f_out < 500
  //  apll_freq = f_out/((o_div + 2) * 2)
  //  operating range of the f_out is 250 MHz ~ 500 MHz
  //  operating range of the apll_freq is 16 ~ 128 MHz.
  //  select sdm0,sdm1,sdm2 to produce nice multiples of colorburst frequencies

  //  see calc_freq() for math: (4+a)*10/((2 + b)*2) mhz
  //  up to 20mhz seems to work ok:
  //  rtc_clk_apll_enable(1,0x00,0x00,0x4,0);   // 20mhz for fancy DDS

  if (!_pal_) {
    switch (samples_per_cc) {
      case 3: rtc_clk_apll_enable(1, 0x46, 0x97, 0x4, 2);   break; // 10.7386363636 3x NTSC (10.7386398315mhz)
///      case 4: rtc_clk_apll_enable(1, 0x46, 0x97, 0x4, 1);   break; // 14.3181818182 4x NTSC (14.3181864421mhz)
      case 4: rtc_clk_apll_enable(1, 70, 151, 4, 1);   break; // 14.3181818182 4x NTSC (14.3181864421mhz)
    }
  } else {
    rtc_clk_apll_enable(1, 0x04, 0xA4, 0x6, 1); // 17.734476mhz ~4x PAL
  }


///      rtc_clk_apll_enable(1, 0x46, 0x97, 0x4, 1);   // 14.3181818182 4x NTSC (14.3181864421mhz)
      ///rtc_clk_apll_enable(1,0x00,0x00,8,0);  //30MHz is MAXIMUM
//===============================================================================	
///	  rtc_clk_apll_enable(1, 93, 116, 7,0);   //28,636363636 MHz (doubled)    
///	  rtc_clk_apll_enable(1, 44, 116, 7,0);   //28,636363636 MHz (doubled)    
///	  rtc_clk_apll_enable(1, 44, 116, 7,0);   //28,636363636 MHz (doubled)   tunned
///	  rtc_clk_apll_enable(1, 52, 116, 7,0);   //28,636363636 MHz (doubled)   tunned


///	  rtc_clk_apll_enable(1, 222, 115, 7,0);   //28,636363636 MHz (doubled)   ///tunned maybe ok


//-------------------------------------------------------------------------------	  
	  ///rtc_clk_apll_enable(1, sdm0, sdm1, sdm2, o_div);                               // enable APLL clock 
	  ///APL_CLK = 40 * (4 + (sdm0 / 65536) + (sdm1 / 256) + sdm2) / (2 * (o_div + 2));  // final freq /4
//-------------------------------------------------------------------------------      
//===============================================================================	


  I2S0.clkm_conf.clkm_div_num = 1;            // I2S clock divider’s integral value.
  I2S0.clkm_conf.clkm_div_b = 0;              // Fractional clock divider’s numerator value.
  I2S0.clkm_conf.clkm_div_a = 1;              // Fractional clock divider’s denominator value
///  I2S0.sample_rate_conf.tx_bck_div_num = 1;

//===============================================================================	
///  I2S0.sample_rate_conf.rx_bck_div_num = 2;  /// WE HAVE DOUBLED FREQ SO MUST DIVIDE BY 2
  I2S0.sample_rate_conf.tx_bck_div_num = 1;  /// WE HAVE DOUBLED FREQ SO MUST DIVIDE BY 2
//===============================================================================	
  
  I2S0.clkm_conf.clka_en = 1;                 // Set this bit to enable clk_apll.
  I2S0.fifo_conf.tx_fifo_mod = (ch == 2) ? 0 : 1; // 32-bit dual or 16-bit single channel data

///  I2S0.fifo_conf.rx_fifo_mod = (ch == 2) ? 0 : 1; // 32-bit dual or 16-bit single channel data

  dac_output_enable(VIDEO_OUT);                // DAC, video out

  dac_output_disable(DAC_GPIO25_CHANNEL);                

  
  dac_i2s_enable();                           // start DAC!

///adc_i2s_mode_init(ADC_UNIT_1, ADC_CHANNEL_6); ///NEED THIS!
///adc_set_i2s_data_source(ADC_I2S_DATA_SRC_ADC);
///adc_power_on();

  I2S0.conf.tx_start = 1;                     // start DMA!
  I2S0.int_clr.val = 0xFFFFFFFF;
  I2S0.int_ena.out_eof = 1;
  I2S0.out_link.start = 1;

///  I2S0.conf.rx_start = 1;                     // start DMA! for OSCILLOSCOPE
  
  return esp_intr_enable(_isr_handle);        // start interruprs!
}
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
void video_init_hw(int line_width, int samples_per_cc)
{
  // setup apll 4x NTSC or PAL colorburst rate
  start_dma(line_width, samples_per_cc, 1);
}
//--------------------------------------------------------------------------------
//================================================================================
//================================================================================
// PAL

void pal_init()
{
  int cc_width = 4;
  _sample_rate = PAL_FREQUENCY * cc_width / 1000000.0;   // DAC rate in mhz
  _line_width = PAL_COLOR_CLOCKS_PER_SCANLINE * cc_width;
  _line_count = PAL_LINES;
  _hsync_short = usec(2);
  _hsync_long = usec(30);
  _hsync = usec(4.7);
  _burst_start = usec(5.6);
  _burst_width = (int)(10 * cc_width + 4) & 0xFFFE;
  _active_start = usec(10.4);

  // make colorburst tables for even and odd lines
  _burst0 = new int16_t[_burst_width];
  _burst1 = new int16_t[_burst_width];
  float phase = 2 * M_PI / 2;
  for (int i = 0; i < _burst_width; i++)
  {
    _burst0[i] = BLANKING_LEVEL + sin(phase + 3 * M_PI / 4) * BLANKING_LEVEL / 1.5;
    _burst1[i] = BLANKING_LEVEL + sin(phase - 3 * M_PI / 4) * BLANKING_LEVEL / 1.5;
    phase += 2 * M_PI / cc_width;
  }
}
void video_init(int samples_per_cc, int machine, const uint32_t* palette, int ntsc)
{
  _samples_per_cc = samples_per_cc;
  ///_machine = machine;
  _palette = palette;

  if (ntsc) {
    _sample_rate = 315.0 / 88 * samples_per_cc; // DAC rate
    _line_width = NTSC_COLOR_CLOCKS_PER_SCANLINE * samples_per_cc;
    _line_count = NTSC_LINES;
    _hsync_long = usec(63.555 - 4.7);
    _active_start = usec(samples_per_cc == 4 ? 10 : 10.5);
    _hsync = usec(4.7);
    _pal_ = 0;
  } else {
    pal_init();
    _pal_ = 1;
  }

  _active_lines = 240;  
  video_init_hw(_line_width, _samples_per_cc);   // init the hardware
}
//********************************************************************************
