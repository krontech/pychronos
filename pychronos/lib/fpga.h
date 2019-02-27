/****************************************************************************
 *  Copyright (C) 2019 Kron Technologies Inc <http://www.krontech.ca>.      *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.   *
 ****************************************************************************/
#ifndef _FPGA_H
#define _FPGA_H

#include <stdint.h>
#include <stdio.h>

#define	GPMC_BASE                   0x50000000
#define	GPMC_RANGE_BASE             0x1000000

#define GPMC_REGISTER_OFFSET        0x0
#define	GPMC_REGISTER_LEN           0x1000000
#define GPMC_RAM_OFFSET             0x1000000
#define GPMC_RAM_LEN                0x1000000

/* FPGA Timing clock runs at 100MHz */
#define FPGA_TIMEBASE_HZ            100000000

#define FPGA_FRAME_WORD_SIZE        32

/* Mapping of FPGA control blocks. */
#define FPGA_SENSOR_BASE            0x0000
#define FPGA_SEQUENCER_BASE         0x0040
#define FPGA_TRIGGER_BASE           0x00A0
#define FPGA_SEQPROGRAM_BASE        0x0100
#define FPGA_PAGE_OFFSET_BASE       0x0200
#define FPGA_DISPLAY_BASE           0x0400
#define FPGA_CONFIG_BASE            0x0500
#define FPGA_COL_GAIN_BASE          0x1000
#define FPGA_VRAM_BASE              0x2000
#define FPGA_SCI_BASE               0x3000
#define FPGA_SEQPGM_BASE            0x4000
#define FPGA_COL_OFFSET_BASE        0x5000
#define FPGA_IO_BASE                0x6000
#define FPGA_TIMING_BASE            0x6100
#define FPGA_PIPELINE_BASE          0x7000
#define FPGA_VIDSRC_BASE            0x7100
#define FPGA_CALSRC_BASE            0x7200
#define FPGA_OVERLAY_BASE           0x8000
#define FPGA_COL_CURVE_BASE         0xD000

struct fpga_sensor {
    uint32_t control;
    uint32_t clk_phase;
    uint32_t sync_token;
    uint32_t data_correct;
    uint32_t fifo_start;
    uint32_t fifo_stop;
    uint32_t __reserved0[1];
    uint32_t frame_period;
    uint32_t int_time;
    uint32_t sci_control;
    uint32_t sci_address;
    uint32_t sci_datalen;
    uint32_t sci_fifo_write;
    uint32_t sci_fifo_read;
    uint32_t __reserved1[12]; /* Overlap with sequencer block */
    uint32_t start_delay;
    uint32_t line_period;
};

#define SENSOR_CTL_RESET_MASK           (1 << 0)
#define SENSOR_CTL_EVEN_SLOT_MASK       (1 << 1)

#define SENSOR_SCI_CONTROL_RUN_MASK     (1 << 0)    /* Write 1 to start, reads 1 while busy, 0 when done */
#define SENSOR_SCI_CONTROL_RW_MASK      (1 << 1)    /* Read == 1, Write == 0 */
#define SENSOR_SCI_CONTROL_FULL_MASK    (1 << 2)    /* Reading 1 indicates FIFO is full */
#define SENSOR_SCI_CONTROL_RESET_MASK   (1 << 15)   /* Write 1 to reset the FIFO */

struct fpga_seq {
    uint32_t control;
    uint32_t status;
    uint32_t frame_size;
    uint32_t region_start;
    uint32_t region_stop;
    uint32_t live_addr[3];
    uint32_t trig_delay;
    uint32_t md_fifo_read;
    uint32_t __reserved0[2]; /* start_delay and line_period from display block */
    uint32_t write_addr;
    uint32_t last_addr;
};
#define SEQ_CTL_SOFTWARE_TRIG               (1 << 0)
#define SEQ_CTL_START_RECORDING             (1 << 1)
#define SEQ_CTL_STOP_RECORDING              (1 << 2)
#define SEQ_CTL_TRIG_DELAY_MODE             (1 << 3) /* an undocumented mystery! */

#define SEQ_STATUS_RECORDING                (1 << 0)
#define SEQ_STATUS_FIFO_EMPTY               (1 << 1)

struct fpga_trigger {
    /* Trigger Logic */
    uint32_t    enable;
    uint32_t    invert;
    uint32_t    debounce;

    /* IO Configuration */
    uint32_t    io_output;
    uint32_t    io_source;
    uint32_t    io_invert;
    uint32_t    io_input;

    /* External Shutter Control */
    uint32_t    ext_shutter;
};

#define EXT_SHUTTER_EXPOSURE_ENABLE     (1 << 0)
#define EXT_SHUTTER_GATING_ENABLE       (1 << 1)
#define EXT_SHUTTER_SOURCE_OFFSET       2
#define EXT_SHUTTER_SOURCE_MASK         (0x3f << EXT_SHUTTER_SOURCE_OFFSET)
#define EXT_SHUTTER_SOURCE(_x_)         (1 << ((_x_) + EXT_SHUTTER_SOURCE_OFFSET)

/* Video Display block */
struct fpga_display {
    uint32_t control;
    uint32_t frame_address;
    uint32_t fpn_address;
    uint32_t gain;
    uint32_t h_period;
    uint32_t v_period;
    uint32_t h_sync_len;
    uint32_t v_sync_len;
    uint32_t h_back_porch;
    uint32_t v_back_porch;
    uint32_t h_res;
    uint32_t v_res;
    uint32_t h_out_res;
    uint32_t __reserved0[3];
    uint32_t v_out_res;
    uint32_t peaking_thresh;
    uint32_t pipeline;
    uint32_t __reserved1[1];
    uint32_t manual_sync;
    uint32_t gainctl;
    uint32_t __reserved2[26];

    /* Color and White Balance */
    uint32_t ccm_red[3];
    uint32_t __reserved3[1];
    uint32_t ccm_green[3];
    uint32_t ccm_blue[3];
    uint32_t __reserved4[2];
    uint32_t wbal[3];
    uint32_t __reserved5[1];
};

#define DISPLAY_CTL_ADDRESS_SELECT      (1 << 0)
#define DISPLAY_CTL_SCALER_NN           (1 << 1)
#define DISPLAY_CTL_SYNC_INHIBIT        (1 << 2)
#define DISPLAY_CTL_READOUT_INHIBIT     (1 << 3)
#define DISPLAY_CTL_COLOR_MODE          (1 << 4)
#define DISPLAY_CTL_FOCUS_PEAK_ENABLE   (1 << 5)
#define DISPLAY_CTL_FOCUS_PEAK_COLOR    (0x7 << 6)
#define DISPLAY_CTL_ZEBRA_ENABLE        (1 << 9)
#define DISPLAY_CTL_BLACK_CAL_MODE      (1 << 10)

#define DISPLAY_PIPELINE_BYPASS_FPN         (1<<0)
#define DISPLAY_PIPELINE_BYPASS_GAIN        (1<<1)
#define DISPLAY_PIPELINE_BYPASS_DEMOSAIC    (1<<2)
#define DISPLAY_PIPELINE_BYPASS_COLOR_MATRIX (1<<3)
#define DISPLAY_PIPELINE_BYPASS_GAMMA_TABLE (1<<4)
#define DISPLAY_PIPELINE_RAW_12BPP          (1<<5)
#define DISPLAY_PIPELINE_RAW_16BPP          (1<<6)
#define DISPLAY_PIPELINE_RAW_16PAD          (1<<7)
#define DISPLAY_PIPELINE_TEST_PATTERN       (1<<15)
#define DISPLAY_PIPELINE_RAW_MODES          (0x7 << 5)

#define DISPLAY_GAINCTL_3POINT              (1 << 0)

/* Video RAM readout */
struct fpga_vram {
    uint32_t identifier;    /* Should always read 0x0010 */
    uint32_t version;
    uint32_t subver;
    uint32_t control;
    uint32_t status;
    uint32_t __reserved0[3];
    uint32_t address;
    uint32_t burst;
    uint8_t __reserved1[0x200 - 0x28]; /* Align to offset 0x200 */
    uint16_t buffer[1024];
};
#define VRAM_IDENTIFIER     0x10

#define VRAM_CTL_TRIG_READ  (1 << 0)
#define VRAM_CTL_TRIG_WRITE (1 << 1)

struct fpga_overlay {
    /* Overlay control registers. */
    uint16_t identifier;    /* Always reads 0x0055 */
    uint16_t version;
    uint16_t subver;
    uint16_t control;
    uint16_t status;
    uint16_t text0_xpos;
    uint16_t text0_ypos;
    uint16_t text0_xsize;
    uint16_t text0_ysize;
    uint16_t text1_xpos;
    uint16_t text1_ypos;
    uint16_t text1_xsize;
    uint16_t text1_ysize;
    uint16_t wmark_xpos;
    uint16_t wmark_ypos;
    uint8_t  text0_xoffset;
    uint8_t  text1_xoffset;
    uint8_t  text0_yoffset;
    uint8_t  text1_yoffset;
    uint16_t logo_xpos;
    uint16_t logo_ypos;
    uint16_t logo_xsize;
    uint16_t logo_ysize;
    uint8_t  text0_abgr[4];
    uint8_t  text1_abgr[4];
    uint8_t  wmark_abgr[4];
    uint8_t __reserved0[0x8100 - 0x8036]; /* Align to offset 0x100 */

    /* Text buffer registers. */
#define OVERLAY_TEXT_LENGTH     128
    uint8_t text0_buffer[OVERLAY_TEXT_LENGTH];
    uint8_t __reserved1[128];
    uint8_t text1_buffer[OVERLAY_TEXT_LENGTH];
    uint8_t __reserved2[128];
    uint8_t logo_red_lut[256];
    uint8_t logo_green_lut[256];
    uint8_t logo_blue_lut[256];
    uint8_t __reserved3[0x9000 - 0x8600]; /* Align to offset 0x1000 */

    /* Bitmap display registers */
    uint16_t text0_bitmap[4096];
    uint16_t text0_reserved[2048];
    uint16_t text1_bitmap[4096];
    uint16_t text1_reserved[2048];
    uint16_t logo[20480];
};

#define OVERLAY_IDENTIFIER                  0x0055

#define OVERLAY_CONTROL_TEXTBOX0            (1 << 4)
#define OVERLAY_CONTROL_TEXTBOX1            (1 << 5)
#define OVERLAY_CONTROL_WATERMARK           (1 << 6)
#define OVERLAY_CONTROL_LOGO_MARK           (1 << 7)
#define OVERLAY_CONTROL_FONT_SIZE(_x_)      ((_x_) << 8)
#define OVERLAY_CONTROL_FONT_SIZE_MASK      (0x3F << 8)
#define OVERLAY_CONTROL_BANK_SELECT         (1 << 15)

/* Recording Segment Data Entry */
struct fpga_segment_entry {
    uint32_t start;
    uint32_t end;
    uint32_t last;
    uint32_t data;
};

/* Recording Segment Readout */
struct fpga_segments {
    uint32_t identifier;
    uint32_t version;
    uint32_t subver;
    uint32_t control;
    uint32_t status;
    uint32_t __reserved0[3];
    uint32_t blockno;
    uint32_t __reserved1[504];
    /* Aligned to offset 0x800 */
    struct fpga_segment_entry data[128];
};

#define SEGMENT_IDENTIFIER      0x0040

#define SEGMENT_CONTROL_RESET   (1 << 0)    /* Erase the segment data. */

#define SEGMENT_DATA_PC         0xf0000000  /* Sequencer program counter */
#define SEGMENT_DATA_BLOCKNO    0x00ffffff  /* Segment block number. */

/* Configuration registers. */
struct fpga_config {
    /* Write-Leveling Delay Tuning */
    uint32_t wl_delay[4];
    uint32_t __reserved3[4];

    /* DDR/SODIMM MMU configuration */
    uint32_t mmu_config;
    uint32_t __reserved4[55];

    /* Version and Reset */
    uint32_t sys_reset;
    uint32_t version;
    uint32_t subver;
};

#define MMU_INVERT_CS                   (1 << 0)
#define MMU_SWITCH_STUFFED              (1 << 1)

#define SENSOR_DATA_WIDTH		        12
#define COLOR_MATRIX_INT_BITS	        3
#define COLOR_MATRIX_FRAC_BITS          12
#define COLOR_MATRIX_MAXVAL	            ((1 << (COLOR_MATRIX_FRAC_BITS + COLOR_MATRIX_INT_BITS)) - 1)
#define COLOR_MATRIX_MINVAL             (-1 << (COLOR_MATRIX_FRAC_BITS + COLOR_MATRIX_INT_BITS))

#endif /* _FPGA_H */
