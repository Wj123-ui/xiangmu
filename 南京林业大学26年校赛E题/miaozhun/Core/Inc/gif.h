#ifndef __GIF_H
#define __GIF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Maximum supported GIF dimensions */
#define GIF_MAX_WIDTH   240
#define GIF_MAX_HEIGHT  320

/** GIF frame decoded callback — user provides the frame buffer */
typedef struct {
    const uint8_t  *data;          /* compressed GIF data in memory    */
    uint32_t        size;          /* total data size                  */
    uint32_t        pos;           /* current read position            */

    uint16_t        width;         /* image width                      */
    uint16_t        height;        /* image height                     */
    uint8_t         bg_index;      /* background color index           */

    /* color tables as RGB565 */
    uint16_t        gct[256];      /* global color table               */
    uint16_t        lct[256];      /* local color table                */
    uint16_t       *ct;            /* active pointer (gct or lct)      */
    uint16_t        gct_size;      /* number of gct entries            */

    int             trans_index;   /* transparent index, -1 = none     */

    /* frame image descriptor */
    uint16_t        fx, fy;        /* frame origin                     */
    uint16_t        fw, fh;        /* frame dimensions                 */
    int             interlaced;
    int             disposal;      /* disposal method 0-3              */
    uint16_t        delay;         /* delay in 10ms units              */

    /* LZW decoder state */
    int             code_size;     /* current code size in bits        */
    int             clear_code;
    int             eoi_code;
    int             next_code;
    int             cur_code;
    int             old_code;
    int             fin_char;

    uint8_t        *suffix;        /* points to static LZW suffix table */
    uint16_t       *prefix;        /* points to static LZW prefix table */
    uint8_t        *stack;         /* points to static LZW stack        */
    int             sp;            /* stack pointer                    */

    uint32_t        bits;          /* bit reservoir                    */
    int             bits_left;

    int             block_pos;     /* position within sub-block        */
    int             block_len;     /* remaining bytes in sub-block     */

    /* final decoded frame — caller provides this buffer */
    uint8_t        *frame;         /* width*height indexed pixels      */

    /* debug */
    uint32_t        debug_pos;     /* pos where error occurred         */
    uint8_t         debug_byte;    /* the unexpected byte value        */
} GIF_State;


/**
 * Initialize GIF decoder from in-memory GIF data.
 * @param gif    decoder state
 * @param data   pointer to GIF bytes
 * @param size   total byte count
 * @param frame  caller-provided width*height byte buffer
 * @return 0 on success, negative on error
 */
int  GIF_Init(GIF_State *gif, const uint8_t *data, uint32_t size, uint8_t *frame);

/**
 * Decode the next frame into gif->frame.
 * GIF must have been successfully initialized with GIF_Init().
 * @return 1  = frame decoded, advance to next
 *         0  = end of animation reached
 *        <0  = decode error
 */
int  GIF_NextFrame(GIF_State *gif);

/**
 * Rewind the GIF stream to the first frame.
 * Call after GIF_Init() and decoding some frames to restart.
 */
void GIF_Rewind(GIF_State *gif);

#ifdef __cplusplus
}
#endif

#endif /* __GIF_H */
