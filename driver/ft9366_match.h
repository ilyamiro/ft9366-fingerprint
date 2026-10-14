/* Open fingerprint matcher for the 64x80 FocalTech FT9366 sensor.
 *
 * Own implementation (no vendor code or data). Pipeline:
 *   raw sensor frame -> 8-bit image -> scale-space keypoints with 256-bit binary descriptors
 *   + ridge bit map -> keypoint consensus (rigid transform) -> ridge-map overlay agreement.
 * The module is independent of libfprint so it can be tested offline.
 */
#pragma once
#include <stdint.h>

#define FT_W 64
#define FT_H 80
#define FT_NPIX (FT_W * FT_H)
#define FT_MAXKP 160

typedef struct
{
  float   x, y, ori;
  uint8_t desc[32];
} FtKeypoint;

typedef struct
{
  uint32_t   n;
  FtKeypoint kp[FT_MAXKP];
  uint8_t    map[FT_NPIX];   /* bit0: ridge bit, bit1: valid pixel */
} FtTemplate;

#define FT_TEMPLATE_PACKED_SIZE (4 + FT_MAXKP * 44 + FT_NPIX)

typedef struct
{
  int   score;     /* consensus score after the overlay gate */
  float ratio;     /* fraction of overlapping pixels with locally agreeing ridge map */
  int   overlap;   /* overlapping valid pixels */
} FtMatchResult;

/* Accept rule, chosen from genuine/impostor measurements (see docs/HOW_IT_WORKS.md). */
#define FT_ACCEPT_SCORE 7
#define FT_ACCEPT_RATIO 0.7f

/* raw/base: 64x80 sensor samples in host order (base = average of the two empty-sensor init frames) */
void ft_convert_image (const uint16_t *raw, const uint16_t *base, uint8_t *img);

/* returns number of keypoints; fills t completely */
int  ft_template_build (const uint8_t *img, FtTemplate *t);

void ft_match_dir (const FtTemplate *probe, const FtTemplate *ref, FtMatchResult *r);
/* best of both directions */
void ft_compare (const FtTemplate *a, const FtTemplate *b, FtMatchResult *r);
int  ft_is_match (const FtMatchResult *r);

/* fixed-size serialisation (little endian floats) */
void ft_template_pack (const FtTemplate *t, uint8_t *out);
int  ft_template_unpack (const uint8_t *in, uint32_t len, FtTemplate *t);
