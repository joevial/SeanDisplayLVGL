/*******************************************************************************
 * Size: 12 px
 * Bpp: 1
 * Opts: --bpp 1 --size 12 --font C:/Users/moeburn/Documents/seanlvgl2/assets/NaruMonoDemo-Regular.ttf -o C:/Users/moeburn/Documents/seanlvgl2/assets\ui_font_Font12.c --format lvgl -r 0x20-0x7f --no-compress --no-prefilter
 ******************************************************************************/

#include "../ui.h"

#ifndef UI_FONT_FONT12
#define UI_FONT_FONT12 1
#endif

#if UI_FONT_FONT12

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0xfd,

    /* U+0022 "\"" */
    0xb6, 0x80,

    /* U+0023 "#" */
    0x14, 0x49, 0xf9, 0x42, 0x9f, 0x92, 0x28,

    /* U+0024 "$" */
    0x21, 0xca, 0xa8, 0x60, 0xe2, 0x69, 0x78, 0x80,

    /* U+0025 "%" */
    0xe1, 0x4a, 0xa7, 0x82, 0xc6, 0x54, 0x86,

    /* U+0026 "&" */
    0x72, 0x8, 0x9f, 0x8a, 0x28, 0x9e,

    /* U+0027 "'" */
    0xe0,

    /* U+0028 "(" */
    0x5a, 0xaa, 0x50,

    /* U+0029 ")" */
    0xa5, 0x55, 0xa0,

    /* U+002A "*" */
    0x23, 0x9c, 0x40,

    /* U+002B "+" */
    0x10, 0x23, 0xf8, 0x81, 0x0,

    /* U+002C "," */
    0x80,

    /* U+002D "-" */
    0xe0,

    /* U+002E "." */
    0x80,

    /* U+002F "/" */
    0x4, 0xa4, 0x94, 0x90,

    /* U+0030 "0" */
    0x7b, 0x38, 0x61, 0x86, 0x1c, 0xde,

    /* U+0031 "1" */
    0x65, 0x8, 0x42, 0x10, 0x9f,

    /* U+0032 "2" */
    0x7a, 0x38, 0x41, 0x8, 0xc4, 0x3f,

    /* U+0033 "3" */
    0x38, 0x88, 0x10, 0xc0, 0x78, 0x51, 0x9e,

    /* U+0034 "4" */
    0x20, 0x84, 0x12, 0x8b, 0xf0, 0x82,

    /* U+0035 "5" */
    0x7a, 0xf, 0x83, 0x6, 0x18, 0xde,

    /* U+0036 "6" */
    0x21, 0x7, 0x93, 0x86, 0x1c, 0xde,

    /* U+0037 "7" */
    0xf8, 0x44, 0x22, 0x11, 0x8,

    /* U+0038 "8" */
    0x7a, 0x18, 0x5e, 0xce, 0x18, 0x5e,

    /* U+0039 "9" */
    0x7a, 0x18, 0x61, 0x78, 0x21, 0x4,

    /* U+003A ":" */
    0x84,

    /* U+003B ";" */
    0x84,

    /* U+003C "<" */
    0xc, 0xcc, 0x18, 0x18,

    /* U+003D "=" */
    0xf8, 0x3e,

    /* U+003E ">" */
    0x81, 0xc0, 0xc4, 0xe0,

    /* U+003F "?" */
    0x7b, 0x18, 0x42, 0x10, 0x80, 0x4,

    /* U+0040 "@" */
    0x7c, 0x5, 0xac, 0xd8, 0xb1, 0x5f, 0x0,

    /* U+0041 "A" */
    0x22, 0x94, 0xa7, 0x46, 0x31,

    /* U+0042 "B" */
    0xf4, 0x63, 0xe8, 0xc6, 0x3e,

    /* U+0043 "C" */
    0x79, 0x18, 0x20, 0x82, 0x4, 0x5e,

    /* U+0044 "D" */
    0xf2, 0x28, 0x61, 0x86, 0x18, 0xbc,

    /* U+0045 "E" */
    0xfc, 0x21, 0xf8, 0x42, 0x1f,

    /* U+0046 "F" */
    0xfc, 0x21, 0xf8, 0x42, 0x10,

    /* U+0047 "G" */
    0x7b, 0x28, 0x60, 0x9e, 0x1c, 0xdd,

    /* U+0048 "H" */
    0x8c, 0x63, 0xf8, 0xc6, 0x31,

    /* U+0049 "I" */
    0xf9, 0x8, 0x42, 0x10, 0x9f,

    /* U+004A "J" */
    0x18, 0x42, 0x10, 0xc6, 0x2e,

    /* U+004B "K" */
    0x8a, 0x4a, 0x30, 0xa2, 0x49, 0x22,

    /* U+004C "L" */
    0x84, 0x21, 0x8, 0x42, 0x1f,

    /* U+004D "M" */
    0x44, 0x89, 0x13, 0x6a, 0xb5, 0x64, 0xc9,

    /* U+004E "N" */
    0x87, 0x1c, 0x69, 0x96, 0x58, 0xe1,

    /* U+004F "O" */
    0x7b, 0x28, 0x61, 0x86, 0x14, 0x9e,

    /* U+0050 "P" */
    0xf4, 0x63, 0x1f, 0x42, 0x10,

    /* U+0051 "Q" */
    0x7b, 0x38, 0x61, 0x86, 0x1d, 0x9e, 0xc,

    /* U+0052 "R" */
    0xf2, 0x28, 0xbc, 0xa2, 0x48, 0xa2,

    /* U+0053 "S" */
    0x72, 0x28, 0x10, 0x38, 0x18, 0x5e,

    /* U+0054 "T" */
    0xf9, 0x8, 0x42, 0x10, 0x84,

    /* U+0055 "U" */
    0x8c, 0x63, 0x18, 0xc6, 0x2e,

    /* U+0056 "V" */
    0x8c, 0x62, 0xa5, 0x29, 0x44,

    /* U+0057 "W" */
    0x93, 0x26, 0xab, 0x66, 0xcd, 0x9b, 0x16,

    /* U+0058 "X" */
    0x89, 0x45, 0x8, 0x21, 0x48, 0xa2,

    /* U+0059 "Y" */
    0x8c, 0x54, 0xa2, 0x10, 0x84,

    /* U+005A "Z" */
    0xf8, 0x84, 0x42, 0x22, 0x1f,

    /* U+005B "[" */
    0xf2, 0x49, 0x24, 0x93, 0x80,

    /* U+005C "\\" */
    0x92, 0x44, 0x91, 0x24,

    /* U+005D "]" */
    0xe4, 0x92, 0x49, 0x27, 0x80,

    /* U+005E "^" */
    0x21, 0x14, 0xa8, 0xc4,

    /* U+005F "_" */
    0xfc,

    /* U+0060 "`" */
    0xa0,

    /* U+0061 "a" */
    0x77, 0x38, 0x61, 0xcd, 0xd0,

    /* U+0062 "b" */
    0x82, 0xb, 0xb3, 0x86, 0x1c, 0xee,

    /* U+0063 "c" */
    0x74, 0x61, 0x8, 0xb8,

    /* U+0064 "d" */
    0x4, 0x17, 0x73, 0x86, 0x1c, 0xdd,

    /* U+0065 "e" */
    0x74, 0x7f, 0xc, 0xb8,

    /* U+0066 "f" */
    0x39, 0x3e, 0x42, 0x10, 0x84,

    /* U+0067 "g" */
    0x77, 0x38, 0x61, 0xcd, 0xd0, 0x5e,

    /* U+0068 "h" */
    0x84, 0x2d, 0x98, 0xc6, 0x31,

    /* U+0069 "i" */
    0x20, 0x38, 0x42, 0x10, 0x9f,

    /* U+006A "j" */
    0x10, 0xf1, 0x11, 0x11, 0x1f,

    /* U+006B "k" */
    0x84, 0x25, 0x4c, 0x52, 0x51,

    /* U+006C "l" */
    0xe1, 0x8, 0x42, 0x10, 0x9f,

    /* U+006D "m" */
    0xff, 0x26, 0x4c, 0x99, 0x32, 0x40,

    /* U+006E "n" */
    0xb6, 0x63, 0x18, 0xc4,

    /* U+006F "o" */
    0x74, 0x63, 0x18, 0xb8,

    /* U+0070 "p" */
    0xbb, 0x38, 0x61, 0xce, 0xe8, 0x20,

    /* U+0071 "q" */
    0x77, 0x38, 0x61, 0xcd, 0xd0, 0x41,

    /* U+0072 "r" */
    0xec, 0x64, 0x89, 0x2, 0x1f, 0x0,

    /* U+0073 "s" */
    0x74, 0x60, 0xf8, 0xb8,

    /* U+0074 "t" */
    0x21, 0x3e, 0x42, 0x10, 0x87,

    /* U+0075 "u" */
    0x8c, 0x63, 0x18, 0xbc,

    /* U+0076 "v" */
    0x8c, 0x54, 0xa5, 0x10,

    /* U+0077 "w" */
    0x93, 0x25, 0xd3, 0x66, 0xc5, 0x80,

    /* U+0078 "x" */
    0x8a, 0x88, 0x45, 0x44,

    /* U+0079 "y" */
    0x8c, 0x63, 0x18, 0xbc, 0x3e,

    /* U+007A "z" */
    0xf2, 0x44, 0x8f,

    /* U+007B "{" */
    0x74, 0x44, 0x48, 0x44, 0x44, 0x70,

    /* U+007C "|" */
    0xff, 0xe0,

    /* U+007D "}" */
    0xe2, 0x22, 0x21, 0x22, 0x22, 0xe0,

    /* U+007E "~" */
    0x71, 0x38
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 115, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 115, .box_w = 1, .box_h = 8, .ofs_x = 3, .ofs_y = 0},
    {.bitmap_index = 2, .adv_w = 115, .box_w = 3, .box_h = 3, .ofs_x = 2, .ofs_y = 5},
    {.bitmap_index = 4, .adv_w = 115, .box_w = 7, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 11, .adv_w = 115, .box_w = 6, .box_h = 10, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 19, .adv_w = 115, .box_w = 7, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 26, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 32, .adv_w = 115, .box_w = 1, .box_h = 3, .ofs_x = 3, .ofs_y = 5},
    {.bitmap_index = 33, .adv_w = 115, .box_w = 2, .box_h = 10, .ofs_x = 3, .ofs_y = -1},
    {.bitmap_index = 36, .adv_w = 115, .box_w = 2, .box_h = 10, .ofs_x = 3, .ofs_y = -1},
    {.bitmap_index = 39, .adv_w = 115, .box_w = 5, .box_h = 4, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 42, .adv_w = 115, .box_w = 7, .box_h = 5, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 47, .adv_w = 115, .box_w = 1, .box_h = 2, .ofs_x = 3, .ofs_y = -1},
    {.bitmap_index = 48, .adv_w = 115, .box_w = 3, .box_h = 1, .ofs_x = 2, .ofs_y = 3},
    {.bitmap_index = 49, .adv_w = 115, .box_w = 1, .box_h = 1, .ofs_x = 3, .ofs_y = 0},
    {.bitmap_index = 50, .adv_w = 115, .box_w = 3, .box_h = 10, .ofs_x = 2, .ofs_y = -1},
    {.bitmap_index = 54, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 60, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 65, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 71, .adv_w = 115, .box_w = 7, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 78, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 84, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 90, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 96, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 101, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 107, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 113, .adv_w = 115, .box_w = 1, .box_h = 6, .ofs_x = 3, .ofs_y = 0},
    {.bitmap_index = 114, .adv_w = 115, .box_w = 1, .box_h = 7, .ofs_x = 3, .ofs_y = -1},
    {.bitmap_index = 115, .adv_w = 115, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 119, .adv_w = 115, .box_w = 5, .box_h = 3, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 121, .adv_w = 115, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 125, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 131, .adv_w = 115, .box_w = 7, .box_h = 7, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 138, .adv_w = 116, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 143, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 148, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 154, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 160, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 165, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 170, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 176, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 181, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 186, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 191, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 197, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 202, .adv_w = 115, .box_w = 7, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 209, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 215, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 221, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 226, .adv_w = 115, .box_w = 6, .box_h = 9, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 233, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 239, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 245, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 250, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 255, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 260, .adv_w = 115, .box_w = 7, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 267, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 273, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 278, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 283, .adv_w = 115, .box_w = 3, .box_h = 11, .ofs_x = 2, .ofs_y = -1},
    {.bitmap_index = 288, .adv_w = 115, .box_w = 3, .box_h = 10, .ofs_x = 2, .ofs_y = -1},
    {.bitmap_index = 292, .adv_w = 115, .box_w = 3, .box_h = 11, .ofs_x = 2, .ofs_y = -1},
    {.bitmap_index = 297, .adv_w = 115, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 301, .adv_w = 115, .box_w = 6, .box_h = 1, .ofs_x = 1, .ofs_y = -1},
    {.bitmap_index = 302, .adv_w = 115, .box_w = 2, .box_h = 2, .ofs_x = 3, .ofs_y = 7},
    {.bitmap_index = 303, .adv_w = 115, .box_w = 6, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 308, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 314, .adv_w = 115, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 318, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 324, .adv_w = 115, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 328, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 333, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 339, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 344, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 349, .adv_w = 115, .box_w = 4, .box_h = 10, .ofs_x = 2, .ofs_y = -2},
    {.bitmap_index = 354, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 359, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 364, .adv_w = 115, .box_w = 7, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 370, .adv_w = 115, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 374, .adv_w = 115, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 378, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 384, .adv_w = 115, .box_w = 6, .box_h = 8, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 390, .adv_w = 115, .box_w = 7, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 396, .adv_w = 115, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 400, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 405, .adv_w = 115, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 409, .adv_w = 115, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 413, .adv_w = 115, .box_w = 7, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 419, .adv_w = 115, .box_w = 5, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 423, .adv_w = 115, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 428, .adv_w = 92, .box_w = 4, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 431, .adv_w = 115, .box_w = 4, .box_h = 11, .ofs_x = 2, .ofs_y = -1},
    {.bitmap_index = 437, .adv_w = 115, .box_w = 1, .box_h = 11, .ofs_x = 3, .ofs_y = -1},
    {.bitmap_index = 439, .adv_w = 115, .box_w = 4, .box_h = 11, .ofs_x = 2, .ofs_y = -1},
    {.bitmap_index = 445, .adv_w = 115, .box_w = 7, .box_h = 2, .ofs_x = 0, .ofs_y = 2}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 95, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t ui_font_Font12 = {
#else
lv_font_t ui_font_Font12 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 12,          /*The maximum line height required by the font*/
    .base_line = 2,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if UI_FONT_FONT12*/

