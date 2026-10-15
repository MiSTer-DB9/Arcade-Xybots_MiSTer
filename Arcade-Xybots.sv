//============================================================================
//
//  Xybots (Atari Games, 1987) MiSTer FPGA core — top-level glue (emu).
//
//  This is the MiSTer `emu` wrapper around rtl/xybots_core.sv, which holds the
//  whole game: the SP-313 main board, the SYNGEN raster and video pipeline,
//  the motion-object engine and the JSA I audio PCB.  Everything here is
//  framework wiring — PLL clocks, hps_io (ROM download + controls), the /SW
//  bit assembly, the analog-alignment stage, arcade_video, audio and the SDRAM
//  chip pins.
//
//  Xybots runs on a HORIZONTAL monitor (MAME ROT0), so there is no rotation:
//  the raster goes straight through arcade_video.  The Orientation option only
//  offers a 180-degree flip (for a monitor mounted the other way up), which
//  screen_rotate implements via the DDR framebuffer with no_rotate held high.
//
//  Clocking: clk_sys = 57.272727 MHz = 4 x the SP-313 sheet-4 14.318181 MHz
//  crystal.  ce_7m = clk_sys/8 is BOTH the 7.159091 MHz VIDCLK pixel rate and
//  the 68000 clock; ce_ym = /16 and ce_6502 = /32 cover the JSA I board
//  because its 3.579545 MHz crystal is exactly the video crystal / 4.  See
//  rtl/xybots_core.sv.
//
//  This program is free software under the GNU GPL v3 or later.
//
//============================================================================

`timescale 1ns/1ps

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_OSD + per-pin push-pull mask, USER_IO widened to 8 bits
	output        USER_OSD,
	output  [7:0] USER_PP,
	input   [7:0] USER_IN,
	output  [7:0] USER_OUT,
	// [MiSTer-DB9 END]

	input         OSD_STATUS
);

///////// Default values for ports not used in this core /////////

assign ADC_BUS  = 'Z;
// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: USER_PP driven by wrapper; USER_OUT driven by joydb (USER_OUT_DRIVE) below
assign USER_PP = USER_PP_DRIVE;
// [MiSTer-DB9 END]
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign VGA_F1        = 0;
assign VGA_SCALER    = 0;
assign VGA_DISABLE   = 0;
assign HDMI_FREEZE   = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT= 0;
assign FB_FORCE_BLANK= 0;

assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

//////////////////////////////////////////////////////////////////

// Horizontal cabinet: the native raster is 336 x 240 on a 4:3 monitor, and the
// Orientation option never rotates (only flips 180), so "Original" is 4:3 in
// every state.  VIDEO_ARX/ARY are actually driven by video_freak further down;
// these are the ratios it starts from.
wire  [1:0] ar  = status[122:121];
wire [11:0] arx = (ar == 2'd0) ? 12'd4 : 12'(ar - 1'd1);
wire [11:0] ary = (ar == 2'd0) ? 12'd3 : 12'd0;

`include "build_id.v"
localparam CONF_STR = {
	"Xybots;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[4],Orientation,Original,Flip;",
	"O[22:20],Scale,Normal,V-Integer,HV-Integer,Narrower HV-Integer;",
	"-;",
	// Analog alignment (CRT H-Size / H-Position, VGA H-Shift / V-Shift), the same
	// page as the Arcade-Toobin_MiSTer and Arcade-Klax_MiSTer cores.  Every
	// field is plain two's complement,
	// so the list is 0,+1..+max then -min..-1 and the decode is a $signed() --
	// see rtl/video/xybots_analog_adjust.sv.  None of these touch the core's
	// timing; the stage sits between xybots_core and arcade_video.
	"P1,Analog alignment;",
	"P1-;",
	"P1O[27:23],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[34:28],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,+49,+50,+51,+52,+53,+54,+55,+56,+57,+58,+59,+60,+61,+62,+63,-64,-63,-62,-61,-60,-59,-58,-57,-56,-55,-54,-53,-52,-51,-50,-49,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[40:35],Analog VGA H-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[46:41],Analog VGA V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	// No DIP page.  Xybots has NO dip switches: MAME's INPUT_PORTS_START(xybots)
	// has only the 0x806100 control word and the 0x806200 service/status word,
	// and every operator setting lives in the X2804 EEPROM at 0x805000, changed
	// from the game's own self-test.  A "DIP;" line here
	// would ask MiSTer Main to splice in a menu the MRA can never populate.
	"O[6],Service,Off,On;",
	// Freeze the game while the OSD is open, as other MiSTer arcade cores do.
	// Default On (status[7] = 0).  See "OSD pause" in rtl/xybots_core.sv.
	"O[7],Pause when OSD is open,On,Off;",
	"-;",
	// [MiSTer-DB9-Pro BEGIN] - Saturn-first joy_type (canonical bit notation)
	"O[127:126],UserIO Joystick,Off,Saturn,DB9MD,DB15;",
	"O[125],UserIO Players, 1 Player,2 Players;",
	// [MiSTer-DB9-Pro END]
	"T[0],Reset;",
	// Control panel (per player): 8-way stick + Fire + Twist Left + Twist Right
	// + Start/Zap; coins are on the JSA audio board.  See the 0x806100 assembly
	// below.
	"J1,Fire,Turn Left,Turn Right,Start/Zap,Coin;",
	"jn,A,B,X,Y,R;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire        direct_video;
wire [21:0] gamma_bus;

wire        ioctl_download;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] ioctl_index;
wire        ioctl_wait;
wire        ioctl_upload, ioctl_upload_req;
wire  [7:0] ioctl_upload_index, ioctl_din;

wire [31:0] joystick_0, joystick_1;

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper
wire         CLK_JOY = CLK_50M;                 // Assign clock between 40-50Mhz
wire   [1:0] joy_type_raw    = status[127:126]; // 0=Off, 1=Saturn, 2=DB9MD, 3=DB15
wire         joy_2p          = status[125];
// SNAC cores: replace 1'b0 with the core's SNAC enable expression so SNAC
// preempts the joydb wrapper on shared USER_IO pins. Default 1'b0 is no-op.
wire         snac_active     = 1'b0;
// MT32-pi cores on primary USER_IO: replace 1'b0 with the core's MT32-active
// expression (e.g. `mt32_use` under `ifndef SECOND_MT32`, `~mt32_disable` for
// TRS-80's inverted polarity). Suppresses the OSD-open autodetect probe so it
// doesn't read the RPi's I2C master traffic as a ghost Saturn signature.
wire         mt32_primary_active = 1'b0;
wire   [1:0] joy_type        = snac_active ? 2'd0 : joy_type_raw;
wire         joy_db9md_en    = (joy_type == 2'd2);
wire         joy_db15_en     = (joy_type == 2'd3);
wire         joy_any_en      = |joy_type;
// [MiSTer-DB9 END]

// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
wire         saturn_unlocked;                   // driven by hps_io UIO_DB9_KEY (0xFE)
// [MiSTer-DB9-Pro END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joydb wrapper wires + instance
wire   [7:0] USER_OUT_DRIVE;
wire   [7:0] USER_PP_DRIVE;
wire  [15:0] joydb_1, joydb_2;
wire         joydb_1ena, joydb_2ena;
wire  [15:0] joy_raw_payload;
// Programmable-remap matrix (joydb_remap inside joydb): clk_sys carries the
// 0xFD selector load (HPS-bus domain). joydb_*_mapped are the MiSTer-standard
// joystick words the core consumes at its joy0_USB merge point. db9_remap_*
// are driven by the hps_io instance (see the .db9_remap_* bindings there).
// Until a core consumes joydb_*_mapped (Layer B), the matrix stays at identity
// and synthesis prunes it -- binding these is dormant + fleet-safe.
wire  [15:0] joydb_1_mapped, joydb_2_mapped;
wire         db9_remap_cmd;
wire   [5:0] db9_remap_byte_cnt;
wire  [15:0] db9_remap_din;

// [MiSTer-DB9 BEGIN] - DB9 remap factory default (used until Main_MiSTer streams UIO 0xFD)
// Derived from CONF_STR J1, same rule as db9_map.cpp; lets the core work on a stock MiSTer binary.
// DB15:  Fire=A, Turn Left=B, Turn Right=C, Start/Zap=START, Coin=SELECT
// DB9MD: Fire=A, Turn Left=B, Turn Right=C, Start/Zap=START, Coin=MODE
wire  [35:0] db9_remap_default_db15  = 36'hFFFFBA654;
wire  [35:0] db9_remap_default_db9md = 36'hFFFFBA654;
// [MiSTer-DB9 END]
joydb joydb (
  .clk             ( CLK_JOY         ),
  .clk_sys         ( clk_sys            ),
  .USER_IN         ( USER_IN         ),
  .OSD_STATUS          ( OSD_STATUS          ),
  .snac_active         ( snac_active         ),
  .mt32_primary_active ( mt32_primary_active ),
  .joy_type        ( joy_type        ),
  .joy_2p          ( joy_2p          ),
  .saturn_unlocked ( saturn_unlocked ),
  .USER_OUT_DRIVE  ( USER_OUT_DRIVE  ),
  .USER_PP_DRIVE   ( USER_PP_DRIVE   ),
  .USER_OSD        ( USER_OSD        ),
  .joydb_1         ( joydb_1         ),
  .joydb_2         ( joydb_2         ),
  .joydb_1ena      ( joydb_1ena      ),
  .joydb_2ena      ( joydb_2ena      ),
  .remap_cmd       ( db9_remap_cmd      ),
  .remap_byte_cnt  ( db9_remap_byte_cnt ),
  .remap_din       ( db9_remap_din      ),
  .remap_default_db15  ( db9_remap_default_db15  ),
  .remap_default_db9md ( db9_remap_default_db9md ),
  .joydb_1_mapped  ( joydb_1_mapped     ),
  .joydb_2_mapped  ( joydb_2_mapped     ),
  .joy_raw         ( joy_raw_payload )
);

assign USER_OUT = USER_OUT_DRIVE;
// [MiSTer-DB9 END]

// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: rename USB joystick wires
wire [31:0] joystick_0_USB, joystick_1_USB;
// [MiSTer-DB9 END]

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),
	.direct_video(direct_video),

	.buttons(buttons),
	.status(status),
	.status_menumask(16'd0),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),

	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index),
	.ioctl_din(ioctl_din),

	.joystick_0(joystick_0_USB),
	.joystick_1(joystick_1_USB),

	.ps2_key(ps2_key),
	// [MiSTer-DB9 BEGIN] - DB9/SNAC8 support: joy_raw
	.joy_raw(joy_raw_payload),
	// programmable remap matrix selector load (UIO_DB9_MAP 0xFD)
	.db9_remap_cmd(db9_remap_cmd),
	.db9_remap_byte_cnt(db9_remap_byte_cnt),
	.db9_remap_din(db9_remap_din),
	// [MiSTer-DB9 END]
	// [MiSTer-DB9-Pro BEGIN] - Saturn key gate
	.saturn_unlocked(saturn_unlocked)
	// [MiSTer-DB9-Pro END]
);

// [MiSTer-DB9-Pro BEGIN] - DB controllers muted while OSD is open
// joydb_*_mapped is already in MiSTer-standard J1 order, identical to the USB
// joystick word, so it replaces the USB input as-is.
assign joystick_0 = joydb_1ena ? (OSD_STATUS ? 16'b0 : joydb_1_mapped) : joystick_0_USB;
assign joystick_1 = joydb_2ena ? (OSD_STATUS ? 16'b0 : joydb_2_mapped) : joydb_1ena ? joystick_0_USB : joystick_1_USB;
// [MiSTer-DB9-Pro END]

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;               // 57.272727 MHz = 4 x 14.318181 MHz
wire clk_sdram;             // same rate, +13095 ps for the SDRAM_CLK pin
wire pll_locked;

pll pll
(
	.refclk(CLK_50M),
	.rst(1'b0),
	.outclk_0(clk_sys),
	.outclk_1(clk_sdram),
	.locked(pll_locked)
);

wire reset = RESET | status[0] | buttons[1] | ~pll_locked;

///////////////////////   CONTROLS   /////////////////////////////

// The /SW word at 0x806100 in its SCHEMATIC bit order, but ACTIVE HIGH
// ("pressed") — the 18F/21E LS244 inversion lives in rtl/main/xybots_inputs.sv,
// which is where the sheet-4 wiring is transcribed and verified
// (MAME's FFE100 agrees bit for bit):
//   D0  P2 Start    D1  P2 Fire   D2  P2 Turn R   D3  P2 Turn L
//   D4  P2 Right    D5  P2 Left   D6  P2 Down     D7  P2 Up
//   D8  P1 Start    D9  P1 Fire   D10 P1 Turn R   D11 P1 Turn L
//   D12 P1 Right    D13 P1 Left   D14 P1 Down     D15 P1 Up
// MiSTer joystick bits: [0]=Right [1]=Left [2]=Down [3]=Up, then the CONF_STR
// J1 list "Fire,Turn Left,Turn Right,Start/Zap,Coin" gives [4]=Fire [5]=Turn Left
// [6]=Turn Right [7]=Start/Zap [8]=Coin — the same order as the MRA's
// <buttons names="Fire,Turn Left,Turn Right,Start/Zap,Coin">, which overrides these
// labels in the OSD mapping screen.  "Start/Zap" is the cabinet's own legend: the
// same centre button starts the game and, in play, fires the zap (the manual:
// it stuns every monster in the player's view).  The knob is two ordinary
// switches, not a stick direction, so Turn L / Turn R are independent bits.
wire [15:0] panel = {
	joystick_0[3], joystick_0[2], joystick_0[1], joystick_0[0],   // D15..D12 P1 Up/Down/Left/Right
	joystick_0[5], joystick_0[6], joystick_0[4], joystick_0[7],   // D11..D8  P1 TurnL/TurnR/Fire/Start
	joystick_1[3], joystick_1[2], joystick_1[1], joystick_1[0],   // D7..D4   P2 Up/Down/Left/Right
	joystick_1[5], joystick_1[6], joystick_1[4], joystick_1[7]    // D3..D0   P2 TurnL/TurnR/Fire/Start
};

// Coins are read on the JSA audio board and are SWAPPED on Xybots (MAME
// set_swapped_coins(true)); the swap is inside rtl/sound/xybots_jsa_io.sv, so
// coin1 here is the LEFT chute / player 1 exactly as the harness labels it.
// The Service switch is the JSA board's SW1: it fans out to /RDIO bit 7 on the
// sound board AND over JSCOM-1 to the main board's /SYSIN D8, so it is one
// signal handed to both models inside xybots_core.
wire coin1     = joystick_0[8];
wire coin2     = joystick_1[8];
wire self_test = status[6];

// OSD pause: a level request; xybots_core starts and ends the freeze on a
// frame boundary so the game resumes exactly where it stopped.
wire pause_req = OSD_STATUS & ~status[7];

///////////////////////   CORE   /////////////////////////////////

wire        ce_pix;
wire  [7:0] core_r, core_g, core_b;
wire        core_hs, core_vs, core_hb, core_vb;
wire signed [15:0] aud_l, aud_r;

xybots_core u_core
(
	.clk_sys(clk_sys), .reset(reset), .init_reset(~pll_locked),
	.pause(pause_req),

	.ioctl_download(ioctl_download), .ioctl_wr(ioctl_wr), .ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout), .ioctl_index(ioctl_index), .ioctl_wait(ioctl_wait),
	.ioctl_upload(ioctl_upload), .ioctl_upload_req(ioctl_upload_req),
	.ioctl_upload_index(ioctl_upload_index), .ioctl_din(ioctl_din),

	.panel(panel), .self_test(self_test), .coin1(coin1), .coin2(coin2),

	.ce_pix(ce_pix), .vga_r(core_r), .vga_g(core_g), .vga_b(core_b),
	.hsync(core_hs), .vsync(core_vs), .hblank(core_hb), .vblank(core_vb),

	.aud_l(aud_l), .aud_r(aud_r),

	.SDRAM_DQ(SDRAM_DQ), .SDRAM_A(SDRAM_A), .SDRAM_BA(SDRAM_BA), .SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH), .SDRAM_CKE(SDRAM_CKE), .SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nRAS(SDRAM_nRAS), .SDRAM_nCAS(SDRAM_nCAS), .SDRAM_nWE(SDRAM_nWE)
);

// SDRAM clock pin from the phase-shifted PLL output, not from ~clk_sys.  The
// +13095 ps (270 deg) phase is the centre of the working window measured on
// real hardware (tap 66; the window runs 225-315 deg).  Neither simulation nor
// STA can decide it — the behavioural SDRAM model clocks on the controller's
// own edge — so a new fit is re-checked on a board.  The graphics ROMs live
// here, so a wrong phase shows as corrupt tiles/sprites with a correct picture
// layout, not as a black screen.
assign SDRAM_CLK = clk_sdram;

///////////////////////   AUDIO   ////////////////////////////////

assign AUDIO_S = 1'b1;              // signed samples
assign AUDIO_L = aud_l;
assign AUDIO_R = aud_r;

///////////////////////   VIDEO   ////////////////////////////////

// Analog alignment (CRT H-Size / H-Position, VGA H-Shift / V-Shift).  Placed
// ahead of arcade_video so the mixer, scandoubler and video_freak all see the
// adjusted raster.  With H-Size active the output pixel rate is no longer
// ce_pix, which is why arcade_video is clocked from adj_ce rather than ce_pix.
// At the zero settings the stage is a bit-exact bypass.
wire [7:0] adj_r, adj_g, adj_b;
wire       adj_hs, adj_vs, adj_hb, adj_vb, adj_ce;

xybots_analog_adjust #(.H_TOTAL(456), .V_TOTAL(263), .CLK_PER_PIX(8)) u_analog_adjust
(
	.clk        (clk_sys),
	.ce_pix     (ce_pix),
	.osd_hsize  (status[27:23]),
	.osd_hpos   (status[34:28]),
	.osd_hshift (status[40:35]),
	.osd_vshift (status[46:41]),
	.r_in(core_r), .g_in(core_g), .b_in(core_b),
	.hs_in(core_hs), .vs_in(core_vs), .hb_in(core_hb), .vb_in(core_vb),
	.r_out(adj_r), .g_out(adj_g), .b_out(adj_b),
	.hs_out(adj_hs), .vs_out(adj_vs), .hb_out(adj_hb), .vb_out(adj_vb),
	.ce_out(adj_ce)
);

wire [7:0] R = adj_r;
wire [7:0] G = adj_g;
wire [7:0] B = adj_b;
wire       HSync  = adj_hs;
wire       VSync  = adj_vs;
wire       HBlank = adj_hb;
wire       VBlank = adj_vb;

// The cabinet raster is presented natively; MiSTer's forced_scandoubler stays
// framework-controlled for displays that need it.
wire [2:0] fx = 3'b000;

// Orientation (status[4]).  Xybots is ROT0 so the core NEVER rotates:
// screen_rotate is held with no_rotate = 1, where it derives
// do_flip = no_rotate && flip and fb_en = ~no_rotate | flip
// (sys/arcade_video.v), i.e. "Original" is a pure passthrough with the DDR
// framebuffer disabled, and "Flip" is a 180-degree flip through the
// framebuffer.  direct_video bypasses the scaler entirely, so it forces the
// passthrough state (the standard MiSTer arcade idiom).
wire       rotate_ccw = 1'b0;
wire       no_rotate  = 1'b1;
wire       flip       = status[4] & ~direct_video;
wire       video_rotated;

wire vga_de_raw;

// WIDTH = 336 = the visible width of the SP-313 raster (456 x 263 total).
// DW = 24: CRAM is I:R:G:B 4:4:4:4, but the intensity bit makes the sheet-9
// resistor-ladder DAC produce more than 16 levels per channel, so the core
// hands over the DAC's own 8-bit-per-channel output (rtl/video/xybots_dac.sv).
arcade_video #(.WIDTH(336), .DW(24)) arcade_video
(
	.clk_video (clk_sys),
	.ce_pix    (adj_ce),
	.RGB_in    ({R, G, B}),
	.HBlank    (HBlank),
	.VBlank    (VBlank),
	.HSync     (HSync),
	.VSync     (VSync),

	.CLK_VIDEO (CLK_VIDEO),
	.CE_PIXEL  (CE_PIXEL),
	.VGA_R     (VGA_R),
	.VGA_G     (VGA_G),
	.VGA_B     (VGA_B),
	.VGA_HS    (VGA_HS),
	.VGA_VS    (VGA_VS),
	.VGA_DE    (vga_de_raw),
	.VGA_SL    (VGA_SL),

	.fx                 (fx),
	.forced_scandoubler (forced_scandoubler),
	.gamma_bus          (gamma_bus)
);

// Scale (status[22:20]) maps onto video_freak's SCALE encoding, documented at
// sys/video_freak.sv as 0 normal, 1 V-integer, 2 HV-Integer-, 3 HV-Integer+,
// 4 HV-Integer.  "Narrower HV-Integer" is therefore 2 and plain "HV-Integer"
// is 4 — they are not adjacent, so menu order and encoding deliberately differ.
wire [2:0] scale_sel = (status[22:20] == 3'd0) ? 3'd0 :   // Normal
                       (status[22:20] == 3'd1) ? 3'd1 :   // V-Integer
                       (status[22:20] == 3'd2) ? 3'd4 :   // HV-Integer
                                                 3'd2;    // Narrower HV-Integer

video_freak video_freak
(
	.CLK_VIDEO  (CLK_VIDEO),
	.CE_PIXEL   (CE_PIXEL),
	.VGA_VS     (VGA_VS),
	.HDMI_WIDTH (HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE     (VGA_DE),
	.VIDEO_ARX  (VIDEO_ARX),
	.VIDEO_ARY  (VIDEO_ARY),
	.VGA_DE_IN  (vga_de_raw),
	.ARX        (arx),
	.ARY        (ary),
	.CROP_SIZE  (12'd0),
	.CROP_OFF   (5'd0),
	.SCALE      (scale_sel)
);

screen_rotate screen_rotate (.*);

///////////////////////   STATUS LED   ///////////////////////////

assign LED_USER = ioctl_download;

endmodule
