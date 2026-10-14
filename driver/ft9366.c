/*
 * Driver for FocalTech FT9366 fingerprint sensors behind a Realtek RTS5811 USB
 * bridge (USB 2808:a658)
 *
 * Protocol knowledge comes from observing the reader's USB traffic.
 * The sensor delivers a 64x80 frame of 16-bit samples. The sensor is only about
 * 3x4 mm, too small for minutiae based matching, so this driver does its own
 * matching (ft9366_match.c: keypoints + binary descriptors + ridge map overlay)
 * and stores one template per enrolment touch.
 *
 * Everything runs on the main context using asynchronous USB transfers and state
 * machines; there are no threads.
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
 */

#define FP_COMPONENT "ft9366"

#include <math.h>
#include "drivers_api.h"
#include "ft9366_script.h"
#include "ft9366_match.c"   /* single translation unit: the matcher is not libfprint specific */

#define FT_EP_OUT       0x01
#define FT_EP_IN        0x82
#define FT_INTF         0
#define FT_WIDTH        FT_W
#define FT_HEIGHT       FT_H
#define FT_FRAME_LEN    (FT_NPIX * 2)
#define FT_TIMEOUT_MS   3000
#define FT_CYCLE_MS     150
#define FT_FRAME_GAP_MS 60
#define FT_RESET_WAIT_MS 500

/* frames per touch that are skipped (partial contact) and then averaged */
#define FT_SKIP_FRAMES 2
#define FT_USE_FRAMES  3
#define FT_MAX_FRAMES  (FT_SKIP_FRAMES + FT_USE_FRAMES)

#define FT_FLAG_DOWN 0x0002

/* A touch needs at least this many keypoints to be usable. */
#define FT_MIN_KEYPOINTS 40
/* Touches per enrolment: the matcher works per touch, more touches cover more of the finger. */
#define FT_ENROLL_STAGES 12
#define FT_PRINT_VERSION 1

/* Large buffers live outside the GObject instance (GObject limits it to 64 KiB). */
typedef struct
{
  guint8  reply[FT_FRAME_LEN];
  guint8  frame[FT_FRAME_LEN];
  guint16 ref_a[FT_NPIX];
  guint16 base[FT_NPIX];   /* empty-sensor reference, taken from the init scans */
  guint32 sum[FT_NPIX];
  guint8  img[FT_NPIX];    /* converted image of the last touch */
} FtBuf;

struct _FpiDeviceFt9366
{
  FpDevice      parent;

  FtBuf        *b;

  /* script interpreter cursor */
  const guint8 *script;
  gsize         script_size;
  gsize         script_pos;
  gsize         reply_len;
  gboolean      have_frame;

  gboolean      collect_ref;
  int           init_frames;

  /* current operation */
  FpiSsm       *active;
  int           frames;
  int           used;
  int           clear;
  int           stage;
  GPtrArray    *enrolled;   /* FtTemplate * collected while enrolling */
};
G_DECLARE_FINAL_TYPE (FpiDeviceFt9366, fpi_device_ft9366, FPI, DEVICE_FT9366, FpDevice);
G_DEFINE_TYPE (FpiDeviceFt9366, fpi_device_ft9366, FP_TYPE_DEVICE);

static const FpIdEntry id_table[] = {
  { .vid = 0x2808, .pid = 0xa658 },
  { .vid = 0, .pid = 0 },
};

static void
frame_to_u16 (const guint8 *raw, guint16 *out)
{
  int i;

  for (i = 0; i < FT_NPIX; i++)
    out[i] = (raw[2 * i] << 8) | raw[2 * i + 1];
}

static double
u16_mean (const guint16 *v)
{
  double s = 0;
  int i;

  for (i = 0; i < FT_NPIX; i++)
    s += v[i];
  return s / FT_NPIX;
}

/* ------------------------------------------------------------------------ */
/* Script interpreter: bulk write / bulk read / delay, as a sub state machine */
/* ------------------------------------------------------------------------ */

enum { SCRIPT_STEP, SCRIPT_NUM_STATES };

static void
frame_received (FpiDeviceFt9366 *self)
{
  memcpy (self->b->frame, self->b->reply, FT_FRAME_LEN);
  self->have_frame = TRUE;

  if (!self->collect_ref)
    return;

  /* The 3rd and 4th scans of the init sequence are empty-sensor frames. */
  if (self->init_frames == 2)
    {
      frame_to_u16 (self->b->frame, self->b->ref_a);
    }
  else if (self->init_frames == 3)
    {
      guint16 b[FT_NPIX];
      int i;

      frame_to_u16 (self->b->frame, b);
      for (i = 0; i < FT_NPIX; i++)
        self->b->base[i] = (self->b->ref_a[i] + b[i]) / 2;
      fp_dbg ("reference frame mean %.0f", u16_mean (self->b->base));
      if (u16_mean (self->b->base) < 1500)
        fp_warn ("reference frame looks like it has a finger on it");
    }
  self->init_frames++;
}

static void
script_transfer_cb (FpiUsbTransfer *transfer,
                    FpDevice       *dev,
                    gpointer        user_data,
                    GError         *error)
{
  FpiSsm *ssm = user_data;
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  if (error)
    {
      fpi_ssm_mark_failed (ssm, error);
      return;
    }

  if (transfer->endpoint == FT_EP_IN)
    {
      if (transfer->actual_length != (gssize) transfer->length ||
          transfer->length > sizeof (self->b->reply))
        {
          fpi_ssm_mark_failed (ssm, fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                              "short read from sensor"));
          return;
        }
      memcpy (self->b->reply, transfer->buffer, transfer->length);
      self->reply_len = transfer->length;
      if (self->reply_len == FT_FRAME_LEN)
        frame_received (self);
    }

  fpi_ssm_jump_to_state (ssm, SCRIPT_STEP);
}

static void
script_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  FpiUsbTransfer *transfer;
  guint8 op;
  gsize len;

  if (self->script_pos + 3 > self->script_size)
    {
      fpi_ssm_mark_completed (ssm);
      return;
    }

  op = self->script[self->script_pos];
  len = (self->script[self->script_pos + 1] << 8) | self->script[self->script_pos + 2];
  self->script_pos += 3;

  switch (op)
    {
    case 0: /* bulk write */
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk_full (transfer, FT_EP_OUT,
                                       (guint8 *) self->script + self->script_pos, len, NULL);
      self->script_pos += len;
      fpi_usb_transfer_submit (transfer, FT_TIMEOUT_MS, NULL, script_transfer_cb, ssm);
      break;

    case 1: /* bulk read */
      transfer = fpi_usb_transfer_new (dev);
      fpi_usb_transfer_fill_bulk (transfer, FT_EP_IN, len);
      fpi_usb_transfer_set_short_error (transfer, TRUE);
      fpi_usb_transfer_submit (transfer, FT_TIMEOUT_MS, NULL, script_transfer_cb, ssm);
      break;

    case 2: /* delay */
      fpi_ssm_jump_to_state_delayed (ssm, SCRIPT_STEP, len);
      break;

    default:
      fpi_ssm_mark_failed (ssm, fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                          "bad script opcode"));
    }
}

static void
start_script (FpiSsm *parent, FpiDeviceFt9366 *self, const guint8 *script, gsize size)
{
  self->script = script;
  self->script_size = size;
  self->script_pos = 0;
  self->have_frame = FALSE;
  self->reply_len = 0;
  fpi_ssm_start_subsm (parent, fpi_ssm_new (FP_DEVICE (self), script_run_state,
                                            SCRIPT_NUM_STATES));
}

#define START_SCRIPT(ssm, self, name) \
  start_script (ssm, self, ft9366_ ## name ## _script, sizeof (ft9366_ ## name ## _script))

/* The idle script's last reply is the interrupt flag register. */
static gboolean
get_flag (FpiSsm *ssm, FpiDeviceFt9366 *self, guint16 *flag)
{
  if (self->reply_len != 2)
    {
      fpi_ssm_mark_failed (ssm, fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                          "no interrupt flag in reply"));
      return FALSE;
    }
  *flag = (self->b->reply[0] << 8) | self->b->reply[1];
  return TRUE;
}

/* ------------------------------------------------------------------------ */
/* Operation state machines                                                  */
/* ------------------------------------------------------------------------ */

static void capture_ready (FpiDeviceFt9366 *self);
static void start_wait_touch (FpiDeviceFt9366 *self);

static GError *
cancelled_error (void)
{
  return g_error_new_literal (G_IO_ERROR, G_IO_ERROR_CANCELLED, "Operation was cancelled");
}

/* Wait for a finger and capture it. */
enum {
  POLL_CYCLE,        /* run idle script: wake chip, finger-detect check */
  POLL_EVAL,         /* look at the interrupt flag */
  POLL_SLEEP,
  POLL_SCAN,         /* run scan script: read one frame */
  POLL_ACCUM,
  POLL_FLAG,         /* re-read the interrupt flag */
  POLL_FLAG_EVAL,
  POLL_FINISH,
  POLL_NUM_STATES,
};

static void
poll_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  guint16 flag;

  if (fpi_device_action_is_cancelled (dev))
    {
      fpi_ssm_mark_failed (ssm, cancelled_error ());
      return;
    }

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case POLL_CYCLE:
      START_SCRIPT (ssm, self, idle);
      break;

    case POLL_EVAL:
      if (!get_flag (ssm, self, &flag))
        return;
      if (flag & FT_FLAG_DOWN)
        {
          memset (self->b->sum, 0, sizeof (self->b->sum));
          self->frames = 0;
          self->used = 0;
          fpi_ssm_jump_to_state (ssm, POLL_SCAN);
        }
      else
        {
          fpi_ssm_next_state (ssm);
        }
      break;

    case POLL_SLEEP:
      fpi_ssm_jump_to_state_delayed (ssm, POLL_CYCLE, FT_CYCLE_MS);
      break;

    case POLL_SCAN:
      START_SCRIPT (ssm, self, scan);
      break;

    case POLL_ACCUM:
      if (!self->have_frame)
        {
          fpi_ssm_mark_failed (ssm, fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                              "scan returned no image"));
          return;
        }
      if (self->frames >= FT_SKIP_FRAMES)
        {
          guint16 tmp[FT_NPIX];
          int i;

          frame_to_u16 (self->b->frame, tmp);
          for (i = 0; i < FT_NPIX; i++)
            self->b->sum[i] += tmp[i];
          self->used++;
        }
      self->frames++;
      fpi_ssm_next_state (ssm);
      break;

    case POLL_FLAG:
      START_SCRIPT (ssm, self, idle);
      break;

    case POLL_FLAG_EVAL:
      if (!get_flag (ssm, self, &flag))
        return;
      if (self->frames < FT_MAX_FRAMES && (flag & FT_FLAG_DOWN))
        fpi_ssm_jump_to_state_delayed (ssm, POLL_SCAN, FT_FRAME_GAP_MS);
      else
        fpi_ssm_jump_to_state (ssm, POLL_FINISH);
      break;

    case POLL_FINISH:
      if (self->used == 0)
        {
          /* touch too short to get a usable frame */
          fpi_ssm_jump_to_state_delayed (ssm, POLL_CYCLE, FT_CYCLE_MS);
        }
      else
        {
          guint16 raw[FT_NPIX];
          int i;

          for (i = 0; i < FT_NPIX; i++)
            raw[i] = self->b->sum[i] / self->used;
          ft_convert_image (raw, self->b->base, self->b->img);
          fpi_ssm_mark_completed (ssm);
        }
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
poll_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  self->active = NULL;
  if (error)
    {
      fpi_device_action_error (dev, error);
      return;
    }
  fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_PRESENT, FP_FINGER_STATUS_NEEDED);
  capture_ready (self);
}

/* Wait for the finger to be removed (between enrolment touches). */
enum {
  OFF_CYCLE,
  OFF_EVAL,
  OFF_NUM_STATES,
};

static void
off_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  guint16 flag;

  if (fpi_device_action_is_cancelled (dev))
    {
      fpi_ssm_mark_failed (ssm, cancelled_error ());
      return;
    }

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case OFF_CYCLE:
      START_SCRIPT (ssm, self, idle);
      break;

    case OFF_EVAL:
      if (!get_flag (ssm, self, &flag))
        return;
      self->clear = (flag & FT_FLAG_DOWN) ? 0 : self->clear + 1;
      if (self->clear >= 2)
        fpi_ssm_mark_completed (ssm);
      else
        fpi_ssm_jump_to_state_delayed (ssm, OFF_CYCLE, FT_CYCLE_MS);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
off_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  self->active = NULL;
  if (error)
    {
      fpi_device_action_error (dev, error);
      return;
    }
  fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_NONE, FP_FINGER_STATUS_PRESENT);
  start_wait_touch (self);
}

static void
start_wait_touch (FpiDeviceFt9366 *self)
{
  g_assert (self->active == NULL);

  fpi_device_report_finger_status_changes (FP_DEVICE (self), FP_FINGER_STATUS_NEEDED, FP_FINGER_STATUS_NONE);
  self->active = fpi_ssm_new (FP_DEVICE (self), poll_run_state, POLL_NUM_STATES);
  fpi_ssm_start (self->active, poll_done);
}

static void
start_wait_removal (FpiDeviceFt9366 *self)
{
  g_assert (self->active == NULL);

  self->clear = 0;
  self->active = fpi_ssm_new (FP_DEVICE (self), off_run_state, OFF_NUM_STATES);
  fpi_ssm_start (self->active, off_done);
}

/* ------------------------------------------------------------------------ */
/* Templates <-> FpPrint                                                     */
/* ------------------------------------------------------------------------ */

static GVariant *
templates_to_variant (GPtrArray *tpls)
{
  GVariantBuilder b;
  guint i;
  guint8 *buf = g_malloc (FT_TEMPLATE_PACKED_SIZE);

  g_variant_builder_init (&b, G_VARIANT_TYPE ("aay"));
  for (i = 0; i < tpls->len; i++)
    {
      ft_template_pack (g_ptr_array_index (tpls, i), buf);
      g_variant_builder_add_value (&b, g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, buf,
                                                                  FT_TEMPLATE_PACKED_SIZE, 1));
    }
  g_free (buf);
  return g_variant_new ("(uaay)", (guint32) FT_PRINT_VERSION, &b);
}

/* returns a GPtrArray of FtTemplate *, or NULL if the print is not ours */
static GPtrArray *
templates_from_print (FpPrint *print)
{
  g_autoptr(GVariant) data = NULL;
  g_autoptr(GVariant) list = NULL;
  GPtrArray *out;
  guint32 version;
  gsize i, n;

  g_object_get (print, "fpi-data", &data, NULL);
  if (!data || !g_variant_is_of_type (data, G_VARIANT_TYPE ("(uaay)")))
    return NULL;
  g_variant_get (data, "(u@aay)", &version, &list);
  if (version != FT_PRINT_VERSION)
    return NULL;

  out = g_ptr_array_new_with_free_func (g_free);
  n = g_variant_n_children (list);
  for (i = 0; i < n; i++)
    {
      g_autoptr(GVariant) item = g_variant_get_child_value (list, i);
      gsize len;
      gconstpointer p = g_variant_get_fixed_array (item, &len, 1);
      FtTemplate *t = g_new0 (FtTemplate, 1);

      if (ft_template_unpack (p, len, t) != 0)
        {
          g_free (t);
          continue;
        }
      g_ptr_array_add (out, t);
    }
  return out;
}

static gboolean
print_matches (FtTemplate *probe, GPtrArray *tpls)
{
  guint i;

  for (i = 0; i < tpls->len; i++)
    {
      FtMatchResult r;

      ft_compare (probe, g_ptr_array_index (tpls, i), &r);
      fp_dbg ("template %u: score %d ratio %.2f overlap %d", i, r.score, r.ratio, r.overlap);
      if (ft_is_match (&r))
        return TRUE;
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */
/* A touch has been captured                                                 */
/* ------------------------------------------------------------------------ */

static void
capture_ready (FpiDeviceFt9366 *self)
{
  FpDevice *dev = FP_DEVICE (self);
  FpiDeviceAction action = fpi_device_get_current_action (dev);
  FtTemplate *probe = g_new0 (FtTemplate, 1);
  int nk = ft_template_build (self->b->img, probe);

  fp_dbg ("touch captured, %d keypoints", nk);

  if (action == FPI_DEVICE_ACTION_ENROLL)
    {
      FpPrint *print = NULL;

      fpi_device_get_enroll_data (dev, &print);
      if (nk < FT_MIN_KEYPOINTS)
        {
          g_free (probe);
          fpi_device_enroll_progress (dev, self->stage, NULL,
                                      fpi_device_retry_new_msg (FP_DEVICE_RETRY_CENTER_FINGER,
                                                                "Not enough fingerprint detail, press the finger firmly on the sensor"));
        }
      else
        {
          g_ptr_array_add (self->enrolled, probe);
          self->stage++;
          fpi_device_enroll_progress (dev, self->stage, NULL, NULL);
        }

      if (self->stage >= FT_ENROLL_STAGES)
        {
          g_object_set (print, "fpi-data", templates_to_variant (self->enrolled), NULL);
          g_ptr_array_set_size (self->enrolled, 0);
          fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
          return;
        }
      start_wait_removal (self);
      return;
    }

  if (action == FPI_DEVICE_ACTION_VERIFY || action == FPI_DEVICE_ACTION_IDENTIFY)
    {
      if (nk < FT_MIN_KEYPOINTS)
        {
          GError *err = fpi_device_retry_new_msg (FP_DEVICE_RETRY_CENTER_FINGER,
                                                  "Not enough fingerprint detail, press the finger firmly on the sensor");
          if (action == FPI_DEVICE_ACTION_VERIFY)
            {
              fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL, err);
              fpi_device_verify_complete (dev, NULL);
            }
          else
            {
              fpi_device_identify_report (dev, NULL, NULL, err);
              fpi_device_identify_complete (dev, NULL);
            }
        }
      else if (action == FPI_DEVICE_ACTION_VERIFY)
        {
          FpPrint *enrolled = NULL;
          GPtrArray *tpls;
          gboolean ok = FALSE;

          fpi_device_get_verify_data (dev, &enrolled);
          tpls = templates_from_print (enrolled);
          if (tpls)
            {
              ok = print_matches (probe, tpls);
              g_ptr_array_unref (tpls);
            }
          fpi_device_verify_report (dev, ok ? FPI_MATCH_SUCCESS : FPI_MATCH_FAIL, NULL, NULL);
          fpi_device_verify_complete (dev, NULL);
        }
      else
        {
          GPtrArray *prints = NULL;
          FpPrint *found = NULL;
          guint i;

          fpi_device_get_identify_data (dev, &prints);
          for (i = 0; i < prints->len && !found; i++)
            {
              FpPrint *p = g_ptr_array_index (prints, i);
              GPtrArray *tpls = templates_from_print (p);

              if (tpls)
                {
                  if (print_matches (probe, tpls))
                    found = p;
                  g_ptr_array_unref (tpls);
                }
            }
          fpi_device_identify_report (dev, found, NULL, NULL);
          fpi_device_identify_complete (dev, NULL);
        }
      g_free (probe);
      return;
    }

  g_free (probe);
  fpi_device_action_error (dev, fpi_device_error_new (FP_DEVICE_ERROR_NOT_SUPPORTED));
}

/* ------------------------------------------------------------------------ */
/* Open: reset, init sequence, collect the reference frames                  */
/* ------------------------------------------------------------------------ */

enum {
  INIT_WAIT,
  INIT_RUN,
  INIT_CHECK,
  INIT_NUM_STATES,
};

static void
init_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case INIT_WAIT:
      /* give the bridge time to come back after the USB reset */
      fpi_ssm_next_state_delayed (ssm, FT_RESET_WAIT_MS);
      break;

    case INIT_RUN:
      self->collect_ref = TRUE;
      self->init_frames = 0;
      START_SCRIPT (ssm, self, init);
      break;

    case INIT_CHECK:
      self->collect_ref = FALSE;
      if (self->init_frames < 4)
        fpi_ssm_mark_failed (ssm, fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                            "sensor init incomplete"));
      else
        fpi_ssm_mark_completed (ssm);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
init_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  fpi_device_open_complete (dev, error);
}

/* ------------------------------------------------------------------------ */
/* libfprint vfuncs                                                          */
/* ------------------------------------------------------------------------ */

static void
dev_open (FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  GUsbDevice *usb = fpi_device_get_usb_device (dev);
  GError *error = NULL;
  FpiSsm *ssm;

  self->active = NULL;
  g_clear_pointer (&self->b, g_free);
  self->b = g_new0 (FtBuf, 1);
  g_clear_pointer (&self->enrolled, g_ptr_array_unref);
  self->enrolled = g_ptr_array_new_with_free_func (g_free);

  /* the sensor wants a clean state: start from a USB reset */
  if (!g_usb_device_reset (usb, &error) ||
      !g_usb_device_claim_interface (usb, FT_INTF, 0, &error))
    {
      fpi_device_open_complete (dev, error);
      return;
    }

  ssm = fpi_ssm_new (dev, init_run_state, INIT_NUM_STATES);
  fpi_ssm_start (ssm, init_done);
}

static void
dev_close (FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  GError *error = NULL;

  g_clear_pointer (&self->enrolled, g_ptr_array_unref);
  g_usb_device_release_interface (fpi_device_get_usb_device (dev), FT_INTF, 0, &error);
  fpi_device_close_complete (dev, error);
}

static void
dev_enroll (FpDevice *dev)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (dev);
  FpPrint *print = NULL;

  fpi_device_get_enroll_data (dev, &print);
  fpi_print_set_type (print, FPI_PRINT_RAW);
  g_ptr_array_set_size (self->enrolled, 0);
  self->stage = 0;
  start_wait_touch (self);
}

static void
dev_match (FpDevice *dev)
{
  start_wait_touch (FPI_DEVICE_FT9366 (dev));
}

static void
fpi_device_ft9366_init (FpiDeviceFt9366 *self)
{
}

static void
fpi_device_ft9366_finalize (GObject *object)
{
  FpiDeviceFt9366 *self = FPI_DEVICE_FT9366 (object);

  g_clear_pointer (&self->enrolled, g_ptr_array_unref);
  g_clear_pointer (&self->b, g_free);
  G_OBJECT_CLASS (fpi_device_ft9366_parent_class)->finalize (object);
}

static void
fpi_device_ft9366_class_init (FpiDeviceFt9366Class *klass)
{
  GObjectClass *obj_class = G_OBJECT_CLASS (klass);
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  obj_class->finalize = fpi_device_ft9366_finalize;

  dev_class->id = "ft9366";
  dev_class->full_name = "FocalTech FT9366 (RTS5811 bridge)";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages = FT_ENROLL_STAGES;
  dev_class->temp_hot_seconds = -1;   /* polling for a finger does not heat the sensor */

  dev_class->open = dev_open;
  dev_class->close = dev_close;
  dev_class->enroll = dev_enroll;
  dev_class->verify = dev_match;
  dev_class->identify = dev_match;

  fpi_device_class_auto_initialize_features (dev_class);
}
