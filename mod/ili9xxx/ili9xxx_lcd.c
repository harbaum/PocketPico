#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "../../inc/debug.h"
#include "ili9xxx_lcd.h"

#if ILI9341
#include "hardware/spi.h"
#else
#include "ili9225_lcd.pio.h"
#endif

/**
 * TODO: Make these definitions part of configuration.
 */
#define ILI9XXX_PIN_CS 17
#define ILI9XXX_PIN_CLK 18
#define ILI9XXX_PIN_DIN 19
#define ILI9XXX_PIN_RS 20
#define ILI9XXX_PIN_RESET 21
#define ILI9XXX_PIN_BL 22

#if !ILI9341
/**
 * Clock division for LCD PIO.
 * PocketPico MCU is overclocked to 266 MHz.
 * Maximum clock frequency for ILI9225 LCD is 50-60 MHz.
 * PIO state machine is capable of producing clock period every
 * 2 clock cycles -> 266 / 2 = 133 MHz. So we need to slow PIO down
 * by a factor of 2 -> 266 / 2 / 2 = 62.5 MHz.
 *
 * TODO: Calculate this value from CPU and LCD frequencies.
 */
#define SERIAL_CLK_DIV 2.f

static PIO pio = pio1;
static uint sm = 0;
#endif
static int dma_chan;

struct reg_dat_pair {
    uint16_t reg;
    uint16_t dat;
};


void ili9xxx_set_rc_cs(bool rc, bool cs) {
    sleep_us(1);
    gpio_put_masked(
        (1u << ILI9XXX_PIN_RS) | (1u << ILI9XXX_PIN_CS),
        !!rc << ILI9XXX_PIN_RS | !!cs << ILI9XXX_PIN_CS
    );
    sleep_us(1);
}

#if !ILI9341
void ili9225_write_cmd(const uint16_t cmd) {
    ili9225_lcd_wait_idle(pio, sm);
    ili9xxx_set_rc_cs(0, 0);
    ili9225_lcd_put(pio, sm, cmd);
    ili9225_lcd_wait_idle(pio, sm);
    ili9xxx_set_rc_cs(1, 1);
}

void ili9225_write_data(const uint16_t data) {
    ili9225_lcd_wait_idle(pio, sm);
    ili9xxx_set_rc_cs(1, 0);
    ili9225_lcd_put(pio, sm, data);
    ili9225_lcd_wait_idle(pio, sm);
    ili9xxx_set_rc_cs(1, 1);
}

void ili9225_set_register(uint16_t reg, uint16_t data) {
    ili9225_write_cmd(reg);
    ili9225_write_data(data);
}
#endif

#if ILI9341
void ili9341_tx_byte(uint8_t byte) {
  spi_write_blocking(spi0, &byte, 1);
}

void ili9341_sendCommand(uint8_t commandByte, uint8_t *dataBytes, uint8_t numDataBytes) {
  ili9xxx_set_rc_cs(0, 0);         // RS and CS low
  ili9341_tx_byte(commandByte);
  ili9xxx_set_rc_cs(1, 0);         // RS high, CS stays low
  for(int i=0;i<numDataBytes;i++)
    ili9341_tx_byte(dataBytes[i]);
  ili9xxx_set_rc_cs(1, 1); 
}
#endif

void ili9xxx_init(void) {
    /* Setup all the GPIO directions for our LCD. */
    gpio_init(ILI9XXX_PIN_CS);
    gpio_init(ILI9XXX_PIN_RS);
    gpio_init(ILI9XXX_PIN_RESET);
    gpio_init(ILI9XXX_PIN_BL);
    gpio_set_dir(ILI9XXX_PIN_CS, GPIO_OUT);
    gpio_set_dir(ILI9XXX_PIN_RS, GPIO_OUT);
    gpio_set_dir(ILI9XXX_PIN_RESET, GPIO_OUT);
    gpio_set_dir(ILI9XXX_PIN_BL, GPIO_OUT);
    gpio_put(ILI9XXX_PIN_BL, 0);

    /* Reset the LCD. */
    gpio_put(ILI9XXX_PIN_RESET, 1);
    ili9xxx_set_rc_cs(0, 1);
    sleep_ms(1);
    gpio_put(ILI9XXX_PIN_RESET, 0);
    sleep_ms(10);
    gpio_put(ILI9XXX_PIN_RESET, 1);
    sleep_ms(50);

#if ILI9341
    // when using x1.5, the number of pixels is 240x216. At 16bpp and
    // 60 Hz, this is 49766400 bit/s. The screen memory is then 103680 bytes
    
    // init SPI at 60Mhz, mode 3
    spi_init(spi0, 60000000);
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    DBG_INFO("SPI baudrate %d\n", spi_get_baudrate(spi0));    
#else
    /* Setup PIO with ILI9225 program. */
    uint offset = pio_add_program(pio, &ili9225_lcd_program);
    ili9225_lcd_program_init(pio, sm, offset, ILI9XXX_PIN_DIN, ILI9XXX_PIN_CLK, SERIAL_CLK_DIV);

    /* Setup DMA.
     * We will transfer 16bits data from the buffer given by user to
     * the PIO program. */
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_PIO1_TX0);
    dma_channel_configure(
        dma_chan,
        &c,
        &pio1_hw->txf[0], // Write address.
        NULL,             // Don't provide a read address yet.
        0,                // Don't provide a read length yet.
        false             // Don't start yet.
    );
#endif
    
#if ILI9341
    DBG_INFO("ILI9341 init\n");
    
    #define W16(a)    (a>>8), (a&0xff)    // store 16 bit parameter

    static const uint8_t init_cmd[] = {
      0xEF, 3, 0x03, 0x80, 0x02,
      0xCF, 3, 0x00, 0xC1, 0x30,
      0xED, 4, 0x64, 0x03, 0x12, 0x81,
      0xE8, 3, 0x85, 0x00, 0x78,
      0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
      0xF7, 1, 0x20,
      0xEA, 2, 0x00, 0x00,
      0xC0, 1, 0x23,                    // Power control VRH[5:0]
      0xC1, 1, 0x10,                    // Power control SAP[2:0];BT[3:0]
      0xC5, 2, 0x3e, 0x28,              // VCM control
      0xC7, 1, 0x86,                    // VCM control2
      0x36, 1, 0x48,                    // Memory Access Control, with x/y order reversed
      0x37, 1, 0x00,                    // Vertical scroll zero
      0x3A, 1, 0x55,
      0xB1, 2, 0x00, 0x18,              // Framerate control
      0xB6, 3, 0x08, 0x82, 0x27,        // Display Function Control
      0xF2, 1, 0x00,                    // 3Gamma Function Disable
      0x26, 1, 0x01,                    // Gamma curve selected
      0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
      0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
      0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
      0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
      0x11, 0,                          // Exit Sleep
      0xff, 150,                        // 150ms delay
      0x29, 0,                          // Display on
      0xff, 150,                        // 150ms delay
      0x00                              // End of list
    };
    
    uint8_t cmd;
    const uint8_t *addr = init_cmd;
    while(cmd = *addr++) {
      const uint8_t num = *addr++;
      if(cmd != 0xff) {
	ili9341_sendCommand(cmd, (uint8_t*)addr, num);
	addr += num;
      } else
	sleep_ms(num);
    }

    // Grab some unused dma channels
    dma_chan = dma_claim_unused_channel(true);
    
    dma_channel_config spi_dma_config = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&spi_dma_config, DMA_SIZE_8);
    channel_config_set_dreq(&spi_dma_config, spi_get_dreq(spi_default, true));

    dma_channel_configure(dma_chan, &spi_dma_config,
                          &spi_get_hw(spi0)->dr, // write address
                          NULL, 0, false); // don't start yet
#else
    /* Switch on power control. */
    {
        /* VCI set to 2.58V. */
        /* FIXME: CTRL3 VGH is set to 2.58*6=15.48, but it could be
         * FIXME: lower ( 9 < VGH < 16.5).
         * FIXME: CTRL3 VGL is set to 2.58*-4=-10.32, but it could be
         * FIXME: higher ( -4 < VGL < -16.5).
         * FIXME: BT could be set to 0b000 for VGH=10, VGL=-7.5. */
        /* CTRL4: GVDD is set to 4.68V. */
        /* CTRL5: Set VCM to 0.8030V. Set VML to 1.104V. */
        /* CTRL1: Set driving capability to "Medium Fast 1". */
        const uint8_t regs[] = {
            ILI9225_REG_PWR_CTRL2, ILI9225_REG_PWR_CTRL3,
            ILI9225_REG_PWR_CTRL4, ILI9225_REG_PWR_CTRL5,
            ILI9225_REG_PWR_CTRL1
        };
        const uint16_t dat[] = {
            0x0018, 0x6121, 0x006F, 0x495F, 0x0800
        };

        for(uint8_t i = 0; i < sizeof(regs); i++)
            ili9225_set_register(regs[i], dat[i]);
    }
    sleep_ms(10);

    /* Enable automatic booster operation, and amplifiers.
     * Set VCI1 to 2.76V.
     * FIXME: why is VCI1 changed from 2.58 to 2.76V? */
    ili9225_set_register(ILI9225_REG_PWR_CTRL2, 0x103B);
    sleep_ms(50);

    {
        const struct reg_dat_pair cmds[] = {
            /* GRAM address of 0x0000 should be in top-left corner
             * when looking at the display in the "landscape" mode.
             * Set active lines NL to 528 * 220 dots. */
            { ILI9225_REG_DRIVER_OUTPUT_CTRL,    0x031C },
            /* Set LCD inversion to disabled. */
            { ILI9225_REG_LCD_AC_DRIVING_CTRL,   0x0100 },
            /* Increment both vertical and horizontal address.
             * Use horizontal image. */
            { ILI9225_REG_ENTRY_MODE,        0x1038 },
            /* Turn off all display outputs. */
            { ILI9225_REG_DISPLAY_CTRL,      0x0000 },
            /* Set porches to 8 lines. */
            { ILI9225_REG_BLANK_PERIOD_CTRL,     0x0808 },
            /* Use 1-clock delay to gate output and edge. */
            { ILI9225_REG_FRAME_CYCLE_CTRL,      0x1100 },
            /* Ignore RGB interface settings. */
            { ILI9225_REG_INTERFACE_CTRL,        0x0000 },
            /* Set oscillation frequency to 266.6 kHz. */
            { ILI9225_REG_OSC_CTRL,          0x0701 },
            /* Set VCI recycling to 2 clocks. */
            { ILI9225_REG_VCI_RECYCLING,     0x0020 },
            /* Initialise RAM Address to 0x0 px. */
            { ILI9225_REG_RAM_ADDR_SET1,     0x0000 },
            { ILI9225_REG_RAM_ADDR_SET2,     0x0000 },

            /* Set scanning position to full screen. */
            { ILI9225_REG_GATE_SCAN_CTRL,        0x0000 },
            /* Set end scan position to 219 + 1 px (0xDB). */
            { ILI9225_REG_VERT_SCROLL_CTRL1, 0x00DB },
            /* Set start scan position to 0 px. */
            { ILI9225_REG_VERT_SCROLL_CTRL2, 0x0000 },
            /* Set scroll step to 0 px (no scrolling). */
            { ILI9225_REG_VERT_SCROLL_CTRL3, 0x0000 },
            /* Set partial screen driving end to 219 + 1 px
             * (0xDB). */
            { ILI9225_REG_PART_DRIVING_POS1, 0x00DB },
            /* Set partial screen driving start to 0 px. */
            { ILI9225_REG_PART_DRIVING_POS2, 0x0000 },
            /* Set window to 176 x 220 px (full screen). */
            { ILI9225_REG_HORI_WIN_ADDR1,        0x00AF },
            { ILI9225_REG_HORI_WIN_ADDR2,        0x0000 },
            { ILI9225_REG_VERT_WIN_ADDR1,        0x00DB },
            { ILI9225_REG_VERT_WIN_ADDR2,        0x0000 },

            /* Gamma curve data. */
            { ILI9225_REG_GAMMA_CTRL1,       0x0000 },
            { ILI9225_REG_GAMMA_CTRL2,       0x0808 },
            { ILI9225_REG_GAMMA_CTRL3,       0x080A },
            { ILI9225_REG_GAMMA_CTRL4,       0x000A },
            { ILI9225_REG_GAMMA_CTRL5,       0x0A08 },
            { ILI9225_REG_GAMMA_CTRL6,       0x0808 },
            { ILI9225_REG_GAMMA_CTRL7,       0x0000 },
            { ILI9225_REG_GAMMA_CTRL8,       0x0A00 },
            { ILI9225_REG_GAMMA_CTRL9,       0x0710 },
            { ILI9225_REG_GAMMA_CTRL10,      0x0710 },

            /* Enable full colour display. */
            { ILI9225_REG_DISPLAY_CTRL,      0x0012 }
        };

        for(uint_fast8_t i = 0; i < (uint_fast8_t)ARRAYSIZE(cmds); i++)
            ili9225_set_register(cmds[i].reg, cmds[i].dat);
    }
    sleep_ms(50);

    /**
     * TEMON: disabled, we don't have access to the FLM signal.
     * GON: Enable display.
     * CL: Use full colour.
     * REV: reverse greyscale levels.
     * D: Switch on display.
     */
    ili9225_set_register(ILI9225_REG_DISPLAY_CTRL, 0x0017);
    sleep_ms(50);
#endif
    gpio_put(ILI9XXX_PIN_BL, 1);   // some displays seem to have this inverted
}

void ili9xxx_fill(uint16_t color) {
    ili9xxx_set_window(0, ILI9XXX_SCREEN_WIDTH, 0, ILI9XXX_SCREEN_HEIGHT);
    ili9xxx_write_pixels_start(0, 0);
    for(uint32_t i = 0; i < (ILI9XXX_SCREEN_WIDTH * ILI9XXX_SCREEN_HEIGHT); i++) {
#if !ILI9341
        ili9225_lcd_put(pio, sm, color);
        ili9225_lcd_wait_idle(pio, sm);
#else
	color = ((color & 0x00ff)<<8) | ((color & 0xff00)>>8);
	ili9xxx_write_pixels_chunk(&color, 1);
#endif
    }

#if ILI9341
    // even if the dma is done we get unwritten data if CS is
    // de-asserted immediately after the dma is done
    dma_channel_wait_for_finish_blocking(dma_chan);
    while(spi_is_busy(spi0));
#endif
    
    ili9xxx_set_rc_cs(1, 1);
}

#if !ILI9341
void ili9225_set_xy(uint8_t x, uint8_t y) {
    ili9225_set_register(ILI9225_REG_RAM_ADDR_SET1, y);
    ili9225_set_register(ILI9225_REG_RAM_ADDR_SET2, x);
}
#endif

void ili9xxx_write_pixels_start(uint8_t x, uint8_t y) {
#if !ILI9341
    ili9225_set_xy(x, y);
    ili9225_write_cmd(ILI9225_REG_GRAM_RW);
    ili9225_lcd_wait_idle(pio, sm);
#endif
    ili9xxx_set_rc_cs(1, 0);
}

#define SWAP16(a)  ((((a) & 0x00ff)<<8) | (((a) & 0xff00)>>8))

#ifdef NO_COLORMAPPING
#include <string.h>

// white/grey
// #define GRAY16(a)  SWAP16(((((a)&0x3e)<<10) | ((a)<<5) | (((a)&0x3e)>>1)))

// greenish
#define GRAY16(a)  SWAP16(((((a)&0x3c)<<9) | ((a)<<5) | (((a)&0x38)>>3)))

// very slight yellow
// #define GRAY16(a)  SWAP16(((((a)&0x3e)<<10) | ((a)<<5) | (((a)&0x3c)>>2)))

// yellow
// #define GRAY16(a)  SWAP16(((((a)&0x3e)<<10) | ((a)<<5) | (((a)&0x38)>>3)))

// This writes "raw" gameboy pixels which aren't colormapped.
void ili9xxx_write_raw_pixels_chunk(const uint8_t *data, uint16_t length) {
  static const uint16_t colors4[] = { GRAY16(63), GRAY16(42), GRAY16(21), GRAY16(0) };  
  dma_channel_wait_for_finish_blocking(dma_chan);

#ifdef ENABLE_XSCALE_1_5
  static int l = 0;
  
  // input colors range from 0..3, so output ranges from 0..6 (not 7!)
  static const uint16_t colors8[] =
    { GRAY16(63), GRAY16(52), GRAY16(42), GRAY16(31),
      GRAY16(21), GRAY16(10),  GRAY16(0),  GRAY16(0) };  

  static uint16_t c[240];

  // for vertical scaling an additonal line is inserted before
  // every second line
  static uint8_t last_data[160];
  if(!l) {
    // save this line for the averaging in the next
    memcpy(last_data, data, 160);
  } else {
    static uint16_t x[240];
    
    // input colors range from 0..3, so output ranges from 0..12 (not 15!)
    static const uint16_t colors16[] =
      { GRAY16(63), GRAY16(58), GRAY16(52), GRAY16(47),
	GRAY16(42), GRAY16(36), GRAY16(31), GRAY16(26),
	GRAY16(21), GRAY16(16), GRAY16(10),  GRAY16(5),  
	 GRAY16(0),  GRAY16(0),  GRAY16(0),  GRAY16(0) };  

    // every second line we insert an averaged line between the previous and the current one
    uint16_t *d = x;
    uint8_t *ldata = last_data;
    for(int i=0;i<length/2;i++) {
      *d++ = colors8[(data[0]&3) + (ldata[0]&3)];
      *d++ = colors16[(data[0]&3) + (data[1]&3) + (ldata[0]&3) + (ldata[1]&3)];
      *d++ = colors8[(data[1]&3) + (ldata[1]&3)];
      data+=2;
      ldata+=2;
    }
    data -= 160;  // "rewind" for next line

    dma_channel_transfer_from_buffer_now(dma_chan, x, 3*length);
  }
  
  // map raw pixels onto 16 bit greyscale
  uint16_t *d = c;
  for(int i=0;i<length/2;i++) {
    *d++ = colors4[(data[0]&3)];
    *d++ = colors8[(data[0]&3) + (data[1]&3)];
    *d++ = colors4[(data[1]&3)];
    data+=2;
  }
  dma_channel_wait_for_finish_blocking(dma_chan);
  dma_channel_transfer_from_buffer_now(dma_chan, c, 3*length);

  l = !l;
#else
  static uint16_t c[160];

    // map raw pixels onto 16 bit greyscale
    uint16_t *d = c;
    for(int i=0;i<length;i++,data++) *d++ = colors[*data&3];
    
    dma_channel_transfer_from_buffer_now(dma_chan, c, 2*length);
#endif
}
#endif

void ili9xxx_write_pixels_chunk(uint16_t *data, uint16_t length) {
    static uint16_t c[160];  // use a local buffer, so the DMA buffer is not modified
  
    dma_channel_wait_for_finish_blocking(dma_chan);
#if !ILI9341
    dma_channel_transfer_from_buffer_now(dma_chan, data, length);
#else
    // SPI works at 8 bit and so transfers 2*length bytes

    // swap each bytes
    uint16_t *d = c;
    for(int i=0;i<length;i++,data++)
      *d++ = SWAP16(*data);
    
    dma_channel_transfer_from_buffer_now(dma_chan, c, 2*length);
#endif
}

void ili9xxx_write_pixels_end(void) {
#if !ILI9341
    dma_channel_wait_for_finish_blocking(dma_chan);
    ili9xxx_set_rc_cs(0, 1);
#endif
}

void ili9xxx_write_pixels_wait(void) {
#if !ILI9341
    dma_channel_wait_for_finish_blocking(dma_chan);
#endif
}

#if !ILI9341
void ili9225_write_pixel(uint16_t color) {
    ili9225_set_register(ILI9225_REG_GRAM_RW, color);
}
#endif

void ili9xxx_set_window(uint16_t x_start, uint16_t x_length, uint16_t y_start, uint16_t y_length) {
    DBG_INFO("ili9xxx_set_window(%d,%d,%d,%d)\n", x_start, x_length, y_start, y_length);
#if ILI9341  
    uint8_t xwin[] = { W16(x_start), W16(x_start+x_length-1) };
    ili9341_sendCommand(0x2a, xwin, 4); // Column address set
    uint8_t ywin[] = { W16(y_start), W16(y_start+y_length-1) };  // <--- this is broken in the emu
    ili9341_sendCommand(0x2b, ywin, 4); // Row address set
    ili9341_sendCommand(0x2c, NULL, 0);
#else    
    ili9225_set_register(ILI9225_REG_HORI_WIN_ADDR1, y_start + y_length - 1);
    ili9225_set_register(ILI9225_REG_HORI_WIN_ADDR2, y_start);
    ili9225_set_register(ILI9225_REG_VERT_WIN_ADDR1, x_start + x_length - 1);
    ili9225_set_register(ILI9225_REG_VERT_WIN_ADDR2, x_start);
    ili9225_set_register(ILI9225_REG_RAM_ADDR_SET1, y_start);
    ili9225_set_register(ILI9225n_REG_RAM_ADDR_SET2, x_start);
#endif
}
