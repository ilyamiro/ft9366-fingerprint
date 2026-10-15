/*
 * Partial-print matcher for the 64x80 FocalTech FT9366 sensor
 *
 * Copyright (C) 2026 ilyamiro
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once
#include <stdint.h>

#define FT9366_W 64
#define FT9366_H 80
#define FT9366_NPIX (FT9366_W * FT9366_H)
#define FT9366_MAXKP 160

typedef struct
{
  float   x, y, ori;
  uint8_t desc[32];
} Ft9366Keypoint;

typedef struct
{
  uint32_t       n;
  Ft9366Keypoint kp[FT9366_MAXKP];
  uint8_t        map[FT9366_NPIX]; /* bit0: ridge bit, bit1: valid pixel */
} Ft9366Template;

#define FT9366_TEMPLATE_PACKED_SIZE (4 + FT9366_MAXKP * 44 + FT9366_NPIX)

typedef struct
{
  int   score;     /* consensus score after the overlay gate */
  float ratio;     /* fraction of overlapping pixels with locally agreeing ridge map */
  int   overlap;   /* overlapping valid pixels */
} Ft9366MatchResult;

/* Accept rule, chosen from genuine/impostor measurements (see docs/HOW_IT_WORKS.md). */
#define FT9366_ACCEPT_SCORE 7
#define FT9366_ACCEPT_RATIO 0.7f

/* raw/base: 64x80 sensor samples in host order (base = average of the two empty-sensor init frames) */
void ft9366_convert_image (const uint16_t *raw,
                           const uint16_t *base,
                           uint8_t        *img);

/* returns number of keypoints; fills t completely */
int  ft9366_template_build (const uint8_t  *img,
                            Ft9366Template *t);

void ft9366_match_dir (const Ft9366Template *probe,
                       const Ft9366Template *ref,
                       Ft9366MatchResult    *r);
/* best of both directions */
void ft9366_compare (const Ft9366Template *a,
                     const Ft9366Template *b,
                     Ft9366MatchResult    *r);
int  ft9366_is_match (const Ft9366MatchResult *r);

/* fixed-size serialisation (little endian floats) */
void ft9366_template_pack (const Ft9366Template *t,
                           uint8_t              *out);
int  ft9366_template_unpack (const uint8_t  *in,
                             uint32_t        len,
                             Ft9366Template *t);
