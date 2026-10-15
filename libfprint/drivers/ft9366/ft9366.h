/*
 * Driver for FocalTech FT9366 fingerprint sensors behind a Realtek RTS5811 USB
 * bridge (USB 2808:a658)
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

#include "fpi-device.h"
#include "fpi-ssm.h"

#include "ft9366_match.h"

#define FT9366_EP_OUT 0x01
#define FT9366_EP_IN 0x82
#define FT9366_INTF 0
#define FT9366_FRAME_LEN (FT9366_NPIX * 2)
#define FT9366_TIMEOUT_MS 3000
#define FT9366_CYCLE_MS 150
#define FT9366_FRAME_GAP_MS 60
#define FT9366_RESET_WAIT_MS 500

/* frames per touch that are skipped (partial contact) and then averaged */
#define FT9366_SKIP_FRAMES 2
#define FT9366_USE_FRAMES 3
#define FT9366_MAX_FRAMES (FT9366_SKIP_FRAMES + FT9366_USE_FRAMES)

#define FT9366_FLAG_DOWN 0x0002

/* A touch needs at least this many keypoints to be usable. */
#define FT9366_MIN_KEYPOINTS 40
/* Touches per enrolment: the matcher works per touch, more touches cover more of the finger. */
#define FT9366_ENROLL_STAGES 12
#define FT9366_PRINT_VERSION 1

/* Steps of the recorded command scripts in ft9366_script.h */
typedef enum {
  FT9366_STEP_WRITE,    /* bulk write of the payload */
  FT9366_STEP_READ,     /* bulk read of len bytes */
  FT9366_STEP_DELAY,    /* wait len milliseconds */
} Ft9366StepType;

typedef struct
{
  Ft9366StepType type;
  gsize          len;
  const guint8  *data;
} Ft9366Step;

#define FT9366_WRITE(...) \
  { FT9366_STEP_WRITE, sizeof ((const guint8[]){ __VA_ARGS__ }), (const guint8[]){ __VA_ARGS__ } }
#define FT9366_READ(n)  { FT9366_STEP_READ, (n), NULL }
#define FT9366_DELAY(ms) { FT9366_STEP_DELAY, (ms), NULL }

/* Large buffers live outside the GObject instance (GObject limits it to 64 KiB). */
typedef struct
{
  guint8  reply[FT9366_FRAME_LEN];
  guint8  frame[FT9366_FRAME_LEN];
  guint16 ref_a[FT9366_NPIX];
  guint16 base[FT9366_NPIX];   /* empty-sensor reference, taken from the init scans */
  guint32 sum[FT9366_NPIX];
  guint8  img[FT9366_NPIX];    /* converted image of the last touch */
} Ft9366Buf;

G_DECLARE_FINAL_TYPE (FpiDeviceFt9366, fpi_device_ft9366, FPI, DEVICE_FT9366, FpDevice)

struct _FpiDeviceFt9366
{
  FpDevice   parent;

  Ft9366Buf *b;

  /* script interpreter cursor */
  const Ft9366Step *script;
  gsize             script_len;
  gsize             script_pos;
  gsize             reply_len;
  gboolean          have_frame;

  gboolean          collect_ref;
  int               init_frames;

  /* current operation */
  FpiSsm    *active;
  int        frames;
  int        used;
  int        clear;
  int        stage;
  GPtrArray *enrolled;      /* Ft9366Template * collected while enrolling */
};
