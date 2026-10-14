/* Minimal libfprint client for testing a reader with the ft9366 driver.
 *   harness enroll [stages_timeout_s]   - run an enrollment (drives the same USB traffic as real use)
 *   harness capture N prefix            - raw image capture, written as PGM
 * Links at runtime against whatever libfprint-2.so.2 is first on the library path. */
#include <libfprint-2/fprint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void write_pgm(FpImage *img, const char *path)
{
  guint w = fp_image_get_width(img), h = fp_image_get_height(img);
  gsize len = 0;
  const guint8 *d = fp_image_get_data(img, &len);
  FILE *f = fopen(path, "wb");
  if (!f) { perror(path); return; }
  fprintf(f, "P5\n%u %u\n255\n", w, h);
  fwrite(d, 1, (size_t)w * h, f);
  fclose(f);
  g_print("wrote %s (%ux%u, %zu bytes)\n", path, w, h, len);
}

static gpointer cancel_later(gpointer data)
{
  gpointer *a = data;
  g_usleep(GPOINTER_TO_INT(a[1]) * G_USEC_PER_SEC);
  g_cancellable_cancel(a[0]);
  return NULL;
}

static void progress(FpDevice *dev, gint done, FpPrint *print, gpointer ud, GError *error)
{
  g_print(">>> enroll progress: %d stage(s) done%s%s -- lift, then touch again\n", done,
          error ? ", error: " : "", error ? error->message : "");
}

int main(int argc, char **argv)
{
  const char *mode = argc > 1 ? argv[1] : "enroll";
  g_autoptr(FpContext) ctx = fp_context_new();
  GPtrArray *devs = fp_context_get_devices(ctx);
  if (!devs || devs->len == 0) { g_printerr("no fingerprint device found\n"); return 1; }
  FpDevice *dev = g_ptr_array_index(devs, 0);
  g_print("device: %s driver=%s\n", fp_device_get_name(dev), fp_device_get_driver(dev));
  g_autoptr(GError) err = NULL;
  if (!fp_device_open_sync(dev, NULL, &err)) { g_printerr("open failed: %s\n", err->message); return 2; }

  if (!strcmp(mode, "enroll")) {
    int timeout = argc > 2 ? atoi(argv[2]) : 120;
    g_autoptr(GCancellable) c = g_cancellable_new();
    gpointer a[2] = { c, GINT_TO_POINTER(timeout) };
    g_thread_new("cancel", cancel_later, a);
    g_print("TOUCH the sensor now (you have %d s). Repeat each time it asks.\n", timeout);
    g_autoptr(GError) e2 = NULL;
    FpPrint *tmpl = fp_print_new(dev);
    FpPrint *res = fp_device_enroll_sync(dev, tmpl, c, progress, NULL, &e2);
    g_print("enroll %s%s%s\n", res ? "finished OK" : "failed", e2 ? ": " : "", e2 ? e2->message : "");
    if (res && argc > 3 && !strcmp(argv[3], "verify")) {
      int tries = argc > 4 ? atoi(argv[4]) : 6, ok = 0;
      for (int i = 0; i < tries; i++) {
        g_print("\n[verify %d/%d] touch the sensor (20 s)...\n", i + 1, tries);
        g_autoptr(GCancellable) vc = g_cancellable_new();
        gpointer va[2] = { vc, GINT_TO_POINTER(20) };
        g_thread_new("cancel", cancel_later, va);
        g_autoptr(GError) ve = NULL;
        gboolean match = FALSE;
        FpPrint *np = NULL;
        gboolean okcall = fp_device_verify_sync(dev, res, vc, NULL, NULL, &match, &np, &ve);
        g_print(">>> verify: %s%s%s\n", !okcall ? "ERROR" : (match ? "MATCH" : "NO MATCH"), ve ? ": " : "", ve ? ve->message : "");
        if (okcall && match) ok++;
        g_usleep(2 * G_USEC_PER_SEC);
      }
      g_print("\nverify summary: %d/%d matched\n", ok, tries);
    }
  } else {
    int n = argc > 2 ? atoi(argv[2]) : 3;
    const char *prefix = argc > 3 ? argv[3] : "capture";
    for (int i = 0; i < n; i++) {
      g_print("[%d/%d] place finger...\n", i + 1, n);
      g_autoptr(GError) e2 = NULL;
      FpImage *img = fp_device_capture_sync(dev, TRUE, NULL, &e2);
      if (!img) { g_printerr("capture failed: %s\n", e2 ? e2->message : "?"); continue; }
      g_autofree char *p = g_strdup_printf("%s_%d.pgm", prefix, i);
      write_pgm(img, p);
      g_object_unref(img);
      g_usleep(2 * G_USEC_PER_SEC);
    }
  }
  fp_device_close_sync(dev, NULL, NULL);
  return 0;
}
