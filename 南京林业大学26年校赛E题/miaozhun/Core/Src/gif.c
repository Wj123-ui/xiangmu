/**
 * gif.c — Minimal GIF89a decoder for STM32 embedded platforms
 *
 * Features:
 *   - GIF89a / GIF87a header parsing
 *   - Global + local color tables → RGB565
 *   - Graphic Control Extension (delay, disposal, transparency)
 *   - LZW decompression with variable code size
 *   - Interlaced + non-interlaced frame decoding
 *   - Partial frame support (frame-specific fx,fy,fw,fh)
 *   - Disposal method 2 (restore to background)
 *   - Rewind for seamless looping
 *
 * Memory (excluding caller's frame buffer):
 *   ~20 KB (color tables 1 KB, LZW tables 16 KB, stack 4 KB)
 */

#include "gif.h"

/* Static LZW tables — in BSS, not on stack (saves ~16KB of stack space) */
static uint8_t  gif_suffix[4096];
static uint16_t gif_prefix[4096];
static uint8_t  gif_stack_buf[4096];

/* ------------------------------------------------------------------ */
/*  Byte-level I/O on in-memory GIF data                              */
/* ------------------------------------------------------------------ */

static int gif_read_byte(GIF_State *gif, uint8_t *out)
{
    if (gif->pos >= gif->size) return -1;
    *out = gif->data[gif->pos++];
    return 0;
}

static int gif_read_u16(GIF_State *gif, uint16_t *out)
{
    uint8_t lo, hi;
    if (gif_read_byte(gif, &lo)) return -1;
    if (gif_read_byte(gif, &hi)) return -1;
    *out = (uint16_t)lo | ((uint16_t)hi << 8);
    return 0;
}

/* Sub-block reader (for LZW image data) */
static int gif_next_block(GIF_State *gif)
{
    /* Advance past previous block's consumed data to reach next block size */
    gif->pos += gif->block_len;
    gif->block_len = 0;

    if (gif->pos >= gif->size) return -1;
    uint8_t len;
    if (gif_read_byte(gif, &len)) return -1;
    if (len == 0) return -1;       /* terminator — end of image data */
    gif->block_len = len;
    gif->block_pos = 0;
    return 0;
}

static int gif_sub_byte(GIF_State *gif, uint8_t *out)
{
    if (gif->block_pos >= gif->block_len) {
        if (gif_next_block(gif) < 0) {
            /* gif_next_block read the 0x00 terminator and advanced pos past it.
               Rewind so the cleanup code can find and consume the terminator. */
            if (gif->pos > 0) gif->pos--;
            return -1;
        }
    }
    if (gif->pos + gif->block_pos >= gif->size) return -1;
    *out = gif->data[gif->pos + gif->block_pos];
    gif->block_pos++;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  RGB888 → RGB565 conversion for color tables                       */
/* ------------------------------------------------------------------ */

static void gif_load_ct(const uint8_t *src, uint16_t *dst, int entries)
{
    for (int i = 0; i < entries; i++) {
        uint8_t r = src[i * 3 + 0];
        uint8_t g = src[i * 3 + 1];
        uint8_t b = src[i * 3 + 2];
        dst[i] = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    }
}

/* ------------------------------------------------------------------ */
/*  LZW decoder (GIF-flavored variable-length codes)                  */
/* ------------------------------------------------------------------ */

static int gif_lzw_init(GIF_State *gif, int min_code_size)
{
    if (min_code_size < 2 || min_code_size > 8) return -1;

    gif->code_size  = min_code_size + 1;
    gif->clear_code = 1 << min_code_size;
    gif->eoi_code   = gif->clear_code + 1;
    gif->next_code  = gif->eoi_code + 1;
    gif->cur_code   = -1;
    gif->old_code   = -1;
    gif->fin_char   = 0;
    gif->sp         = 0;
    gif->bits       = 0;
    gif->bits_left  = 0;
    gif->block_len  = 0;
    gif->block_pos  = 0;

    gif_next_block(gif);
    return 0;
}

/** Read one variable-length LZW code from the bitstream */
static int gif_lzw_code(GIF_State *gif)
{
    int mask = (1 << gif->code_size) - 1;

    while (gif->bits_left < gif->code_size) {
        uint8_t b;
        if (gif_sub_byte(gif, &b) < 0) {
            return gif->eoi_code;    /* underflow → treat as end */
        }
        gif->bits |= (uint32_t)b << gif->bits_left;
        gif->bits_left += 8;
    }

    int code = (int)(gif->bits & (uint32_t)mask);
    gif->bits >>= gif->code_size;
    gif->bits_left -= gif->code_size;
    return code;
}

/* Interlace pass parameters: row offset and step */
static const int interlace_start[4] = { 0, 4, 2, 1 };
static const int interlace_step[4]  = { 8, 8, 4, 2 };

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int GIF_Init(GIF_State *gif, const uint8_t *data, uint32_t size, uint8_t *frame)
{
    /* zero-init entire struct (no memset — linker may discard libc) */
    {
        uint8_t *p = (uint8_t *)gif;
        uint32_t i;
        for (i = 0; i < sizeof(*gif); i++) p[i] = 0;
    }
    /* hook LZW tables to static storage (avoid stack overflow) */
    gif->suffix = gif_suffix;
    gif->prefix = gif_prefix;
    gif->stack  = gif_stack_buf;

    gif->data       = data;
    gif->size       = size;
    gif->pos        = 0;
    gif->frame      = frame;
    gif->trans_index = -1;
    gif->disposal   = 0;

    /* "GIF89a" or "GIF87a" */
    if (size < 13) return -1;
    if (data[0] != 'G' || data[1] != 'I' || data[2] != 'F') return -2;
    if (data[3] != '8' || (data[4] != '7' && data[4] != '9') || data[5] != 'a')
        return -2;
    gif->pos = 6;

    /* Logical Screen Descriptor */
    uint16_t w, h;
    if (gif_read_u16(gif, &w)) return -3;
    if (gif_read_u16(gif, &h)) return -3;
    gif->width  = w;
    gif->height = h;

    if (w > GIF_MAX_WIDTH || h > GIF_MAX_HEIGHT) return -4;

    uint8_t packed, bg, aspect;
    if (gif_read_byte(gif, &packed)) return -3;
    if (gif_read_byte(gif, &bg))    return -3;
    if (gif_read_byte(gif, &aspect)) return -3;
    gif->bg_index = bg;
    (void)aspect;

    /* Global Color Table */
    if (packed & 0x80) {
        int entries = 2 << (packed & 0x07);
        gif->gct_size = entries;
        if (gif->pos + entries * 3 > size) return -5;
        gif_load_ct(&data[gif->pos], gif->gct, entries);
        gif->pos += entries * 3;
    } else {
        gif->gct_size = 0;
    }
    gif->ct = gif->gct;

    return 0;
}

void GIF_Rewind(GIF_State *gif)
{
    if (!gif || !gif->data) return;

    gif->pos = 6;
    uint16_t w, h;
    uint8_t packed, bg, aspect;
    gif_read_u16(gif, &w);
    gif_read_u16(gif, &h);
    gif_read_byte(gif, &packed);
    gif_read_byte(gif, &bg);
    gif_read_byte(gif, &aspect);
    (void)w; (void)h; (void)bg; (void)aspect;

    if (packed & 0x80) {
        int entries = 2 << (packed & 0x07);
        gif->pos += entries * 3;
    }

    gif->trans_index = -1;
    gif->disposal    = 0;
    gif->bits        = 0;
    gif->bits_left   = 0;
}

/**
 * Decode one GIF frame. Main loop:
 *   1. Read blocks until we hit an Image Descriptor (0x2C)
 *   2. Parse Image Descriptor → fx, fy, fw, fh, interlace, local CT
 *   3. LZW-decode pixels into frame buffer at (fx,fy)
 *
 * Returns:  1 = frame decoded,  0 = EOF/trailer,  <0 = error
 */
int GIF_NextFrame(GIF_State *gif)
{
    if (!gif || !gif->data) return -1;

    /* Handle disposal from PREVIOUS frame before decoding next */
    if (gif->disposal == 2 && gif->fw > 0 && gif->fh > 0) {
        /* Restore to background: fill previous frame region with bg color index */
        uint8_t bg = gif->bg_index;
        for (uint16_t row = gif->fy; row < gif->fy + gif->fh; row++) {
            if (row >= gif->height) break;
            uint8_t *dst = &gif->frame[row * gif->width + gif->fx];
            uint16_t col;
            for (col = 0; col < gif->fw; col++) dst[col] = bg;
        }
    }

    /* Default a whole-image frame descriptor for the first frame */
    gif->fx = 0; gif->fy = 0;
    gif->fw = gif->width;
    gif->fh = gif->height;
    gif->interlaced = 0;
    gif->disposal = 0;
    gif->delay    = 10;  /* default 100ms */

    /* ---- Scan for extensions & image descriptor ---- */
    uint8_t block_id;
    for (;;) {
        if (gif->pos >= gif->size) return 0;

        if (gif_read_byte(gif, &block_id)) return 0;

        if (block_id == 0x2C) {
            /* Image Descriptor found → decode it */
            goto decode_image;
        }
        else if (block_id == 0x21) {
            /* Extension block */
            uint8_t label;
            if (gif_read_byte(gif, &label)) return 0;

            if (label == 0xF9) {
                /* Graphic Control Extension */
                uint8_t bsize;
                if (gif_read_byte(gif, &bsize)) return 0;
                if (bsize != 4) return -10;

                uint8_t packed;
                if (gif_read_byte(gif, &packed)) return 0;
                gif->disposal = (packed >> 2) & 0x07;
                int has_trans = packed & 0x01;

                uint16_t delay;
                if (gif_read_u16(gif, &delay)) return 0;
                gif->delay = delay;

                uint8_t ti;
                if (gif_read_byte(gif, &ti)) return 0;
                gif->trans_index = has_trans ? (int)ti : -1;

                /* Block terminator (0x00) */
                uint8_t term;
                if (gif_read_byte(gif, &term)) return 0;
            }
            else {
                /* Comment / App / Plain Text — skip entirely */
                for (;;) {
                    uint8_t b;
                    if (gif_read_byte(gif, &b)) return 0;
                    if (b == 0) break;
                    gif->pos += b;
                }
            }
        }
        else if (block_id == 0x3B) {
            /* Trailer */
            return 0;
        }
        else {
            gif->debug_pos  = gif->pos - 1;   /* pos was advanced by read */
            gif->debug_byte = block_id;
            return -20;  /* unexpected block */
        }
    }

decode_image:
    /* Image Left, Top, Width, Height */
    if (gif_read_u16(gif, &gif->fx)) return -11;
    if (gif_read_u16(gif, &gif->fy)) return -11;
    if (gif_read_u16(gif, &gif->fw)) return -11;
    if (gif_read_u16(gif, &gif->fh)) return -11;

    uint8_t img_packed;
    if (gif_read_byte(gif, &img_packed)) return -11;

    gif->interlaced = (img_packed & 0x40) ? 1 : 0;
    int has_lct     = (img_packed & 0x80) ? 1 : 0;

    if (has_lct) {
        int entries = 2 << (img_packed & 0x07);
        if (gif->pos + entries * 3 > gif->size) return -12;
        gif_load_ct(&gif->data[gif->pos], gif->lct, entries);
        gif->pos += entries * 3;
        gif->ct = gif->lct;
    } else {
        gif->ct = gif->gct;
    }

    /* LZW Minimum Code Size */
    uint8_t lzw_min;
    if (gif_read_byte(gif, &lzw_min)) return -13;
    if (gif_lzw_init(gif, lzw_min) < 0) return -14;

    /* ---- LZW-decode pixels into frame buffer at (fx, fy) ---- */
    {
        int      interlace_pass   = 0;
        int      interlace_row    = 0;   /* row counter within current pass */
        int      px               = 0;   /* x within this frame (0..fw-1) */
        int      py               = 0;   /* y within this frame (0..fh-1) */
        uint8_t *fb               = gif->frame;

        for (;;) {
            gif->cur_code = gif_lzw_code(gif);
            if (gif->cur_code < 0) return -15;

            if (gif->cur_code == gif->eoi_code) break;

            if (gif->cur_code == gif->clear_code) {
                /* Reset LZW table */
                gif->code_size = lzw_min + 1;
                gif->next_code = gif->eoi_code + 1;
                gif->cur_code  = gif_lzw_code(gif);
                if (gif->cur_code < 0) return -15;
                if (gif->cur_code == gif->eoi_code) break;

                /* Output literal */
                uint8_t c = (uint8_t)gif->cur_code;
                if (c != (uint8_t)gif->trans_index) {
                    uint16_t abs_y = gif->fy + py;
                    uint16_t abs_x = gif->fx + px;
                    if (abs_y < gif->height && abs_x < gif->width) {
                        fb[abs_y * gif->width + abs_x] = c;
                    }
                }
                px++;
                if (px >= (int)gif->fw) {
                    px = 0;
                    if (gif->interlaced) {
                        interlace_row++;
                        py = interlace_start[interlace_pass]
                           + interlace_row * interlace_step[interlace_pass];
                        while (py >= (int)gif->fh) {
                            interlace_pass++;
                            if (interlace_pass >= 4) { py = gif->fh; break; }
                            interlace_row = 0;
                            py = interlace_start[interlace_pass];
                        }
                    } else {
                        py++;
                    }
                }
                gif->old_code = gif->cur_code;
                gif->fin_char = c;
                continue;
            }

            int code = gif->cur_code;
            gif->sp = 0;

            if (code >= gif->next_code) {
                gif->stack[gif->sp++] = (uint8_t)gif->fin_char;
                code = gif->old_code;
            }

            while (code >= gif->clear_code && gif->sp < 4096) {
                gif->stack[gif->sp++] = gif->suffix[code];
                code = gif->prefix[code];
            }

            gif->fin_char = (uint8_t)code;
            gif->stack[gif->sp++] = (uint8_t)gif->fin_char;

            /* Add to string table */
            if (gif->next_code < 4096) {
                gif->prefix[gif->next_code] = (uint16_t)gif->old_code;
                gif->suffix[gif->next_code] = (uint8_t)gif->fin_char;
                gif->next_code++;
                if (gif->next_code >= (1 << gif->code_size) && gif->code_size < 12) {
                    gif->code_size++;
                }
            }

            gif->old_code = gif->cur_code;

            /* Output stack (pixels in reverse) */
            while (gif->sp > 0) {
                uint8_t c = gif->stack[--gif->sp];
                if (c != (uint8_t)gif->trans_index) {
                    uint16_t abs_y = gif->fy + py;
                    uint16_t abs_x = gif->fx + px;
                    if (abs_y < gif->height && abs_x < gif->width) {
                        fb[abs_y * gif->width + abs_x] = c;
                    }
                }
                px++;
                if (px >= (int)gif->fw) {
                    px = 0;
                    if (gif->interlaced) {
                        interlace_row++;
                        py = interlace_start[interlace_pass]
                           + interlace_row * interlace_step[interlace_pass];
                        while (py >= (int)gif->fh) {
                            interlace_pass++;
                            if (interlace_pass >= 4) { py = gif->fh; break; }
                            interlace_row = 0;
                            py = interlace_start[interlace_pass];
                        }
                    } else {
                        py++;
                    }
                }
            }
        }
    }

    /* ---- Position cleanup: advance gif->pos past all LZW data ----
       The LZW loop may have exited early (e.g. EOI in mid-stream).
       gif->pos sits at the SIZE BYTE of the current sub-block.
       gif->block_pos = number of data bytes already consumed from it.
       We must skip: (a) remaining data in current sub-block,
                     (b) all subsequent sub-blocks,
                     (c) the 0x00 terminator.
       After this, gif->pos will point past the terminator —
       exactly where the block-scanning loop expects the next GIF block.
    */
    gif->pos += gif->block_len - gif->block_pos;   /* skip rest of current sub-block */
    while (gif->pos < gif->size) {
        uint8_t sb = gif->data[gif->pos++];
        if (sb == 0) break;          /* terminator */
        gif->pos += sb;              /* skip this sub-block's data */
    }
    /* Consume any extra 0x00 bytes (handles 0-length sub-blocks at end) */
    while (gif->pos < gif->size && gif->data[gif->pos] == 0x00) {
        gif->pos++;
    }

    return 1;
}
