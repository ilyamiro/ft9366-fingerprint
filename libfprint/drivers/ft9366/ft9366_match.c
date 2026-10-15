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

#include "ft9366_match.h"
#include "ft9366_pairs.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI_F 3.14159265358979323846f

/* ------------------------------------------------------------------------------------------
* raw sensor frame -> 8-bit image
* ---------------------------------------------------------------------------------------- */

static void
white_point_fix (int32_t *im)
{
  for (int i = 2; i < 78; i++)
    for (int j = 2; j < 62; j++)
      {
        int32_t c = im[i * FT9366_W + j];
        int32_t d[4] = { im[i * FT9366_W + j - 1], im[i * FT9366_W + j + 1], im[(i - 1) * FT9366_W + j], im[(i + 1) * FT9366_W + j] };
        int32_t t2 = c < 0x33 ? 0 : c - 0x32;
        int c1 = 0, c2 = 0;
        for (int k = 0; k < 4; k++)
          {
            if (c + 0x32 < d[k])
              c1++;
            if (d[k] < t2)
              c2++;
          }
        if (c1 > 2 || c2 > 2)
          im[i * FT9366_W + j] = (d[0] + d[1] + d[2] + d[3]) >> 2;
      }
}

static void
smooth_hist (float *h, int n, int rad)
{
  float tmp[256];

  for (int i = 0; i < n; i++)
    {
      int a = i - rad < 0 ? 0 : i - rad;
      int b = i + rad < n ? i + rad : n - 1;
      float s = 0;
      for (int j = a; j <= b; j++)
        s += h[j];
      tmp[i] = s / (float) (b - a + 1);
    }
  memcpy (h, tmp, n * sizeof (float));
}

static float
sum_hist (const float *h, int a, int b)
{
  float s = 0;

  for (int i = a; i <= b; i++)
    s += h[i];
  return s;
}

static int
hist_extremum (const float *h, int bins, int idx_start, int rad, float thr)
{
  int cnt = 0, i = idx_start - 10, idx_end;

  for (;;)
    {
      idx_end = idx_start;
      if (i < 0)
        break;
      int s = i - rad < 0 ? 0 : i - rad;
      int e = rad + i < idx_start ? rad + i : idx_start;
      float cur = sum_hist (h, s, e);
      if (thr <= cur)
        {
          if (cnt > 5)
            {
              idx_end = i;
              break;
            }
          cnt = 0;
        }
      else
        {
          cnt++;
        }
      i--;
    }
  if (idx_end < 50)
    idx_end = bins;
  if (idx_start == bins)
    {
      idx_end += 30;
      if (idx_end > 250)
        idx_end = 250;
    }
  return idx_end;
}

static void
reflect_box (const float *src, float *dst, int w, int h, int k, int reflect101)
{
  /* separable normalised box filter, border replicate (reflect101 = 0) or reflect-101 */
  int r = k / 2;
  float *tmp = malloc (sizeof (float) * w * h);

  #define IDX(i, n) ((i) < 0 ? (reflect101 ? -(i) : 0) : ((i) >= (n) ? (reflect101 ? 2 * (n) - 2 - (i) : (n) - 1) : (i)))
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float s = 0;
        for (int d = -r; d <= r; d++)
          s += src[y * w + IDX (x + d, w)];
        tmp[y * w + x] = s;
      }
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float s = 0;
        for (int d = -r; d <= r; d++)
          s += tmp[IDX (y + d, h) * w + x];
        dst[y * w + x] = s / (float) (k * k);
      }
  #undef IDX
  free (tmp);
}

void
ft9366_convert_image (const uint16_t *raw, const uint16_t *base, uint8_t *img)
{
  int32_t *im = malloc (sizeof (int32_t) * FT9366_NPIX);
  int32_t *v = malloc (sizeof (int32_t) * FT9366_NPIX);
  float hist[256], part[250];
  float *f = malloc (sizeof (float) * FT9366_NPIX), *m = malloc (sizeof (float) * FT9366_NPIX);

  for (int i = 0; i < FT9366_NPIX; i++)
    im[i] = raw[i] < base[i] ? base[i] - raw[i] : 0;
  for (int y = 0; y < FT9366_H; y++)
    {
      im[y * FT9366_W + 1] = im[y * FT9366_W + 2];
      im[y * FT9366_W + 0] = im[y * FT9366_W + 1];
      im[y * FT9366_W + 62] = im[y * FT9366_W + 61];
      im[y * FT9366_W + 63] = im[y * FT9366_W + 62];
    }
  for (int x = 0; x < FT9366_W; x++)
    {
      im[x] = im[FT9366_W + x];
      im[79 * FT9366_W + x] = im[78 * FT9366_W + x];
    }
  white_point_fix (im);

  /* invert, clamp the dark tail, shift to zero */
  int64_t tot = 0;
  for (int i = 0; i < FT9366_NPIX; i++)
    {
      v[i] = -im[i];
      tot += v[i];
    }
  int avg = (int) (tot / (FT9366_NPIX + 1));
  int lo = 2 * avg < -512 ? 2 * avg : -512;
  if (lo < -1024)
    lo = -1024;
  int vmin = 0x1000, vmax = 0;
  for (int i = 0; i < FT9366_NPIX; i++)
    {
      if (v[i] < lo)
        v[i] = lo;
      if (v[i] < vmin)
        vmin = v[i];
    }
  for (int i = 0; i < FT9366_NPIX; i++)
    {
      v[i] -= vmin;
      if (v[i] > vmax)
        vmax = v[i];
    }

  memset (hist, 0, sizeof (hist));
  for (int i = 0; i < FT9366_NPIX; i++)
    {
      int b = (int) ((double) (v[i] * 256) / ((double) vmax + 1e-6));
      if (b > 255)
        b = 255;
      hist[b] += 1;
    }
  for (int k = 0; k < 3; k++)
    smooth_hist (hist, 256, 2);
  memcpy (part, hist, sizeof (part));
  for (int k = 0; k < 2; k++)
    smooth_hist (part, 250, 2);
  float s1 = sum_hist (part, 0, 199), s2 = sum_hist (part, 200, 249);
  int idx_start;
  float thr;
  if (s2 <= s1 * 0.8f)
    {
      idx_start = 250;
      thr = 75.f;
    }
  else
    {
      idx_start = 200;
      thr = 100.f * s1 / s2;
    }
  int idx_end = hist_extremum (part, 250, idx_start, 2, thr);
  int hi = (vmax * idx_end) / 250;

  for (int i = 0; i < FT9366_NPIX; i++)
    {
      double d = floor ((double) (v[i] * 255) / ((double) hi + 1e-6));
      f[i] = d < 0 ? 0 : d > 255 ? 255 : (float) d;
    }
  /* local mean removal, centred on mid-grey */
  reflect_box (f, m, FT9366_W, FT9366_H, 11, 0);
  for (int i = 0; i < FT9366_NPIX; i++)
    {
      float x = f[i] - m[i] + 128.f;
      img[i] = x < 0 ? 0 : x > 255 ? 255 : (uint8_t) x;
    }
  free (im);
  free (v);
  free (f);
  free (m);
}

/* ------------------------------------------------------------------------------------------
* scale-space keypoints (Lowe-style DoG detector)
* ---------------------------------------------------------------------------------------- */

#define NOL 3                      /* layers per octave */
#define MAX_OCT 8
#define CONTRAST_T 0.01f
#define EDGE_T 20.f
#define SIGMA0 1.6f
#define BORDER 5

typedef struct { int    w, h;
                 float *d;
} Plane;
typedef struct { float x, y, ang, resp;
} Kp;

static int
refl (int i, int n)
{
  if (i < 0)
    i = -i;
  if (i >= n)
    i = 2 * n - 2 - i;
  return i < 0 ? 0 : i;
}

static void
gauss_blur (const Plane *src, Plane *dst, float sigma)
{
  int k = (int) lrintf (sigma * 8 + 1) | 1;
  int r = k / 2;
  float *ker = malloc (sizeof (float) * k), sum = 0;

  for (int i = 0; i < k; i++)
    {
      float x = (float) (i - r);
      ker[i] = expf (-x * x / (2 * sigma * sigma));
      sum += ker[i];
    }
  for (int i = 0; i < k; i++)
    ker[i] /= sum;
  int w = src->w, h = src->h;
  float *tmp = malloc (sizeof (float) * w * h);
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float s = 0;
        for (int d = -r; d <= r; d++)
          s += ker[d + r] * src->d[y * w + refl (x + d, w)];
        tmp[y * w + x] = s;
      }
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        float s = 0;
        for (int d = -r; d <= r; d++)
          s += ker[d + r] * tmp[refl (y + d, h) * w + x];
        dst->d[y * w + x] = s;
      }
  free (tmp);
  free (ker);
}

static Plane
plane_new (int w, int h)
{
  Plane p = { w, h, calloc ((size_t) w * h, sizeof (float)) };

  return p;
}

static int
solve3 (const float A[3][3], const float b[3], float x[3])
{
  float m[3][4];

  for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
        m[i][j] = A[i][j];
      m[i][3] = b[i];
    }
  for (int c = 0; c < 3; c++)
    {
      int p = c;
      for (int r = c + 1; r < 3; r++)
        if (fabsf (m[r][c]) > fabsf (m[p][c]))
          p = r;
      if (fabsf (m[p][c]) < 1e-12f)
        return 0;
      if (p != c)
        for (int j = 0; j < 4; j++)
          {
            float t = m[c][j];
            m[c][j] = m[p][j];
            m[p][j] = t;
          }
      for (int r = c + 1; r < 3; r++)
        {
          float f = m[r][c] / m[c][c];
          for (int j = c; j < 4; j++)
            m[r][j] -= f * m[c][j];
        }
    }
  for (int i = 2; i >= 0; i--)
    {
      float s = m[i][3];
      for (int j = i + 1; j < 3; j++)
        s -= m[i][j] * x[j];
      x[i] = s / m[i][i];
    }
  return 1;
}

static float
orientation_hist (const Plane *img, int px, int py, int radius, float sigma, float *hist)
{
  const int n = 36;
  float temp[n + 4];
  float scale = -1.f / (2.f * sigma * sigma);

  memset (temp, 0, sizeof (temp));
  for (int i = -radius; i <= radius; i++)
    {
      int y = py + i;
      if (y <= 0 || y >= img->h - 1)
        continue;
      for (int j = -radius; j <= radius; j++)
        {
          int x = px + j;
          if (x <= 0 || x >= img->w - 1)
            continue;
          float dx = img->d[y * img->w + x + 1] - img->d[y * img->w + x - 1];
          float dy = img->d[(y - 1) * img->w + x] - img->d[(y + 1) * img->w + x];
          float w = expf ((float) (i * i + j * j) * scale);
          float ang = atan2f (dy, dx) * 180.f / PI_F;
          if (ang < 0)
            ang += 360.f;
          int bin = (int) lrintf (n / 360.f * ang);
          if (bin >= n)
            bin -= n;
          if (bin < 0)
            bin += n;
          temp[bin + 2] += w * sqrtf (dx * dx + dy * dy);
        }
    }
  temp[1] = temp[n + 1];
  temp[0] = temp[n];
  temp[n + 2] = temp[2];
  temp[n + 3] = temp[3];
  float omax = 0;
  for (int i = 0; i < n; i++)
    {
      hist[i] = (temp[i] + temp[i + 4]) * (1.f / 16.f) + (temp[i + 1] + temp[i + 3]) * (4.f / 16.f) + temp[i + 2] * (6.f / 16.f);
      if (hist[i] > omax)
        omax = hist[i];
    }
  return omax;
}

static int
detect_keypoints (const uint8_t *img, Kp **out)
{
  int cap = 512, nk = 0;
  Kp *kps = malloc (sizeof (Kp) * cap);
  Plane gauss[MAX_OCT][NOL + 3], dog[MAX_OCT][NOL + 2];
  float sig[NOL + 3], kfac = powf (2.f, 1.f / NOL);
  int noct;

  /* upsample 2x (bilinear), base blur */
  Plane src = plane_new (FT9366_W, FT9366_H);

  for (int i = 0; i < FT9366_NPIX; i++)
    src.d[i] = img[i];
  Plane up = plane_new (FT9366_W * 2, FT9366_H * 2);
  for (int y = 0; y < up.h; y++)
    for (int x = 0; x < up.w; x++)
      {
        float fx = (x + 0.5f) / 2 - 0.5f, fy = (y + 0.5f) / 2 - 0.5f;
        int x0 = (int) floorf (fx), y0 = (int) floorf (fy);
        float ax = fx - x0, ay = fy - y0;
        int xa = x0 < 0 ? 0 : x0, xb = x0 + 1 > FT9366_W - 1 ? FT9366_W - 1 : x0 + 1;
        int ya = y0 < 0 ? 0 : y0, yb = y0 + 1 > FT9366_H - 1 ? FT9366_H - 1 : y0 + 1;
        up.d[y * up.w + x] = (1 - ay) * ((1 - ax) * src.d[ya * FT9366_W + xa] + ax * src.d[ya * FT9366_W + xb]) +
                             ay * ((1 - ax) * src.d[yb * FT9366_W + xa] + ax * src.d[yb * FT9366_W + xb]);
      }
  free (src.d);
  {
    float sd = sqrtf (fmaxf (SIGMA0 * SIGMA0 - 1.f, 0.01f));
    gauss[0][0] = plane_new (up.w, up.h);
    gauss_blur (&up, &gauss[0][0], sd);
    free (up.d);
  }
  noct = (int) lrintf (log2f ((float) (FT9366_W * 2 < FT9366_H * 2 ? FT9366_W * 2 : FT9366_H * 2)) - 2) + 1;
  if (noct > MAX_OCT)
    noct = MAX_OCT;
  sig[0] = SIGMA0;
  for (int i = 1; i < NOL + 3; i++)
    {
      float prev = powf (kfac, (float) (i - 1)) * SIGMA0, tot = prev * kfac;
      sig[i] = sqrtf (tot * tot - prev * prev);
    }
  for (int o = 0; o < noct; o++)
    for (int i = 0; i < NOL + 3; i++)
      {
        if (o == 0 && i == 0)
          continue;
        if (i == 0)
          {
            const Plane *s = &gauss[o - 1][NOL];
            gauss[o][0] = plane_new (s->w / 2, s->h / 2);
            for (int y = 0; y < gauss[o][0].h; y++)
              for (int x = 0; x < gauss[o][0].w; x++)
                gauss[o][0].d[y * gauss[o][0].w + x] = s->d[(2 * y) * s->w + 2 * x];
          }
        else
          {
            gauss[o][i] = plane_new (gauss[o][0].w, gauss[o][0].h);
            gauss_blur (&gauss[o][i - 1], &gauss[o][i], sig[i]);
          }
      }
  for (int o = 0; o < noct; o++)
    for (int i = 0; i < NOL + 2; i++)
      {
        dog[o][i] = plane_new (gauss[o][0].w, gauss[o][0].h);
        for (int p = 0; p < dog[o][i].w * dog[o][i].h; p++)
          dog[o][i].d[p] = gauss[o][i + 1].d[p] - gauss[o][i].d[p];
      }

  const float threshold = floorf (0.5f * CONTRAST_T / NOL * 255.f);
  const float img_scale = 1.f / 255.f, deriv_scale = img_scale * 0.5f, sderiv = img_scale, cderiv = img_scale * 0.25f;

  for (int o = 0; o < noct; o++)
    for (int layer0 = 1; layer0 <= NOL; layer0++)
      {
        int rows = dog[o][0].h, cols = dog[o][0].w;
        for (int r0 = BORDER; r0 < rows - BORDER; r0++)
          for (int c0 = BORDER; c0 < cols - BORDER; c0++)
            {
              const float *cur = dog[o][layer0].d, *prv = dog[o][layer0 - 1].d, *nxt = dog[o][layer0 + 1].d;
              float val = cur[r0 * cols + c0];
              if (fabsf (val) <= threshold)
                continue;
              int ext = 1;
              const float *pl[3] = { prv, cur, nxt };
              for (int a = 0; a < 3 && ext; a++)
                for (int dy = -1; dy <= 1 && ext; dy++)
                  for (int dx = -1; dx <= 1; dx++)
                    {
                      float nv = pl[a][(r0 + dy) * cols + c0 + dx];
                      if (val > 0 ? val < nv : val > nv)
                        {
                          ext = 0;
                          break;
                        }
                    }
              if (!ext)
                continue;

              int r = r0, c = c0, layer = layer0, iter, ok = 1;
              float xi = 0, xr = 0, xc = 0, dD[3] = { 0 }, X[3], dxx = 0, dyy = 0, dxy = 0;
              for (iter = 0; iter < 5; iter++)
                {
                  const float *im = dog[o][layer].d, *pv = dog[o][layer - 1].d, *nx = dog[o][layer + 1].d;
                  dD[0] = (im[r * cols + c + 1] - im[r * cols + c - 1]) * deriv_scale;
                  dD[1] = (im[(r + 1) * cols + c] - im[(r - 1) * cols + c]) * deriv_scale;
                  dD[2] = (nx[r * cols + c] - pv[r * cols + c]) * deriv_scale;
                  float v2 = im[r * cols + c] * 2;
                  dxx = (im[r * cols + c + 1] + im[r * cols + c - 1] - v2) * sderiv;
                  dyy = (im[(r + 1) * cols + c] + im[(r - 1) * cols + c] - v2) * sderiv;
                  float dss = (nx[r * cols + c] + pv[r * cols + c] - v2) * sderiv;
                  dxy = (im[(r + 1) * cols + c + 1] - im[(r + 1) * cols + c - 1] - im[(r - 1) * cols + c + 1] + im[(r - 1) * cols + c - 1]) * cderiv;
                  float dxs = (nx[r * cols + c + 1] - nx[r * cols + c - 1] - pv[r * cols + c + 1] + pv[r * cols + c - 1]) * cderiv;
                  float dys = (nx[(r + 1) * cols + c] - nx[(r - 1) * cols + c] - pv[(r + 1) * cols + c] + pv[(r - 1) * cols + c]) * cderiv;
                  float Hm[3][3] = { { dxx, dxy, dxs }, { dxy, dyy, dys }, { dxs, dys, dss } };
                  if (!solve3 (Hm, dD, X))
                    {
                      ok = 0;
                      break;
                    }
                  xi = -X[2];
                  xr = -X[1];
                  xc = -X[0];
                  if (fabsf (xi) < 0.5f && fabsf (xr) < 0.5f && fabsf (xc) < 0.5f)
                    break;
                  if (fabsf (xi) > 1e8f || fabsf (xr) > 1e8f || fabsf (xc) > 1e8f)
                    {
                      ok = 0;
                      break;
                    }
                  c += (int) lrintf (xc);
                  r += (int) lrintf (xr);
                  layer += (int) lrintf (xi);
                  if (layer < 1 || layer > NOL || c < BORDER || c >= cols - BORDER || r < BORDER || r >= rows - BORDER)
                    {
                      ok = 0;
                      break;
                    }
                }
              if (!ok || iter >= 5)
                continue;
              {
                const float *im = dog[o][layer].d;
                float t = dD[0] * X[0] + dD[1] * X[1] + dD[2] * X[2];
                float contr = im[r * cols + c] * img_scale + t * 0.5f;
                if (fabsf (contr) * NOL < CONTRAST_T)
                  continue;
                float tr = dxx + dyy, det = dxx * dyy - dxy * dxy;
                if (det <= 0 || tr * tr * EDGE_T >= (EDGE_T + 1) * (EDGE_T + 1) * det)
                  continue;

                float octscale = (float) (1 << o);
                float px = (c + xc) * octscale * 0.5f, py = (r + xr) * octscale * 0.5f;
                float scl_octv = SIGMA0 * powf (2.f, (layer + xi) / NOL);
                float hist[36];
                float omax = orientation_hist (&gauss[o][layer], c, r, (int) lrintf (4.5f * scl_octv), 1.5f * scl_octv, hist);
                float mag_thr = omax * 0.8f;
                for (int j = 0; j < 36; j++)
                  {
                    int l = j > 0 ? j - 1 : 35, rr = j < 35 ? j + 1 : 0;
                    if (hist[j] > hist[l] && hist[j] > hist[rr] && hist[j] >= mag_thr)
                      {
                        float bin = j + 0.5f * (hist[l] - hist[rr]) / (hist[l] - 2 * hist[j] + hist[rr]);
                        bin = bin < 0 ? 36 + bin : bin >= 36 ? bin - 36 : bin;
                        float ang = 360.f - (360.f / 36) * bin;
                        if (fabsf (ang - 360.f) < 1.2e-7f)
                          ang = 0.f;
                        if (nk == cap)
                          {
                            cap *= 2;
                            kps = realloc (kps, sizeof (Kp) * cap);
                          }
                        kps[nk++] = (Kp){ px, py, ang * PI_F / 180.f, fabsf (contr) };
                      }
                  }
              }
            }
      }

  for (int o = 0; o < noct; o++)
    {
      for (int i = 0; i < NOL + 3; i++)
        free (gauss[o][i].d);
      for (int i = 0; i < NOL + 2; i++)
        free (dog[o][i].d);
    }
  *out = kps;
  return nk;
}

static int
kp_cmp (const void *a, const void *b)
{
  const Kp *x = a, *y = b;

  if (x->resp > y->resp)
    return -1;
  if (x->resp < y->resp)
    return 1;
  return 0;
}

/* ------------------------------------------------------------------------------------------
* descriptors + ridge bit map
* ---------------------------------------------------------------------------------------- */

static void
median3 (const uint8_t *src, uint8_t *dst)
{
  for (int y = 0; y < FT9366_H; y++)
    for (int x = 0; x < FT9366_W; x++)
      {
        uint8_t v[9];
        int n = 0;
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++)
            {
              int yy = y + dy < 0 ? 0 : y + dy >= FT9366_H ? FT9366_H - 1 : y + dy;
              int xx = x + dx < 0 ? 0 : x + dx >= FT9366_W ? FT9366_W - 1 : x + dx;
              v[n++] = src[yy * FT9366_W + xx];
            }
        for (int i = 1; i < 9; i++)
          {
            uint8_t t = v[i];
            int j = i - 1;
            while (j >= 0 && v[j] > t)
              {
                v[j + 1] = v[j];
                j--;
              }
            v[j + 1] = t;
          }
        dst[y * FT9366_W + x] = v[4];
      }
}

static void
make_map (const uint8_t *img, uint8_t *map)
{
  uint8_t *med = malloc (FT9366_NPIX);
  float *f = malloc (sizeof (float) * FT9366_NPIX), *loc = malloc (sizeof (float) * FT9366_NPIX);

  median3 (img, med);
  for (int i = 0; i < FT9366_NPIX; i++)
    f[i] = med[i];
  reflect_box (f, loc, FT9366_W, FT9366_H, 5, 1);
  for (int i = 0; i < FT9366_NPIX; i++)
    map[i] = (f[i] > loc[i] * 0.9f ? 1 : 0) | (img[i] > 0 ? 2 : 0);
  free (med);
  free (f);
  free (loc);
}

static void
make_descriptors (const uint8_t *img, const Kp *kps, int n, Ft9366Template *t)
{
  float pts[45][2];
  int np = 0;

  pts[np][0] = 0;
  pts[np][1] = 0;
  np++;
  static const struct { float r;
                        int   n;
  } rings[3] = { { 3, 8 }, { 6, 16 }, { 9, 20 } };
  for (int g = 0; g < 3; g++)
    for (int k = 0; k < rings[g].n; k++)
      {
        float a = 2 * PI_F * k / rings[g].n;
        pts[np][0] = rings[g].r * cosf (a);
        pts[np][1] = rings[g].r * sinf (a);
        np++;
      }
  /* smoothed copy (sigma 2, 13 taps, rounded to 8 bit) */
  Plane s = plane_new (FT9366_W, FT9366_H), b = plane_new (FT9366_W, FT9366_H);
  for (int i = 0; i < FT9366_NPIX; i++)
    s.d[i] = img[i];
  {
    float sigma = 2.f;
    int k = 13, r = 6;
    float ker[13], sum = 0;
    for (int i = 0; i < k; i++)
      {
        float x = (float) (i - r);
        ker[i] = expf (-x * x / (2 * sigma * sigma));
        sum += ker[i];
      }
    for (int i = 0; i < k; i++)
      ker[i] /= sum;
    float *tmp = malloc (sizeof (float) * FT9366_NPIX);
    for (int y = 0; y < FT9366_H; y++)
      for (int x = 0; x < FT9366_W; x++)
        {
          float a = 0;
          for (int d = -r; d <= r; d++)
            a += ker[d + r] * s.d[y * FT9366_W + refl (x + d, FT9366_W)];
          tmp[y * FT9366_W + x] = a;
        }
    for (int y = 0; y < FT9366_H; y++)
      for (int x = 0; x < FT9366_W; x++)
        {
          float a = 0;
          for (int d = -r; d <= r; d++)
            a += ker[d + r] * tmp[refl (y + d, FT9366_H) * FT9366_W + x];
          a = rintf (a);
          b.d[y * FT9366_W + x] = a < 0 ? 0 : a > 255 ? 255 : a;
        }
    free (tmp);
  }
  for (int i = 0; i < n; i++)
    {
      float c = cosf (kps[i].ang), sn = sinf (kps[i].ang), vals[45];
      for (int p = 0; p < 45; p++)
        {
          float px = kps[i].x + pts[p][0] * c - pts[p][1] * sn;
          float py = kps[i].y + pts[p][0] * sn + pts[p][1] * c;
          if (px < 0 || px > FT9366_W - 1 || py < 0 || py > FT9366_H - 1)
            {
              vals[p] = -1;
              continue;
            }
          int x0 = (int) floorf (px), y0 = (int) floorf (py);
          int x1 = x0 + 1 > FT9366_W - 1 ? FT9366_W - 1 : x0 + 1, y1 = y0 + 1 > FT9366_H - 1 ? FT9366_H - 1 : y0 + 1;
          float ax = px - x0, ay = py - y0;
          vals[p] = (1 - ay) * ((1 - ax) * b.d[y0 * FT9366_W + x0] + ax * b.d[y0 * FT9366_W + x1]) +
                    ay * ((1 - ax) * b.d[y1 * FT9366_W + x0] + ax * b.d[y1 * FT9366_W + x1]);
        }
      Ft9366Keypoint *k = &t->kp[i];
      k->x = kps[i].x;
      k->y = kps[i].y;
      k->ori = kps[i].ang;
      memset (k->desc, 0, 32);
      for (int bit = 0; bit < 256; bit++)
        {
          float a = vals[FT9366_PAIRS[bit][0]], bb = vals[FT9366_PAIRS[bit][1]];
          if (a >= 0 && bb >= 0 && bb < a)
            k->desc[bit >> 3] |= 1 << (bit & 7);
        }
    }
  free (s.d);
  free (b.d);
}

int
ft9366_template_build (const uint8_t *img, Ft9366Template *t)
{
  Kp *kps = NULL;
  int n = detect_keypoints (img, &kps);

  memset (t, 0, sizeof (*t));
  qsort (kps, n, sizeof (Kp), kp_cmp);   /* strongest first */
  if (n > FT9366_MAXKP)
    n = FT9366_MAXKP;
  t->n = n;
  make_descriptors (img, kps, n, t);
  make_map (img, t->map);
  free (kps);
  return n;
}

/* ------------------------------------------------------------------------------------------
* matching
* ---------------------------------------------------------------------------------------- */

typedef struct { float th, dx, dy;
} Rigid;

static Rigid
rigid_fit (const float (*ref)[2], const float (*test)[2], const int *idx, int n)
{
  double cr[2] = { 0, 0 }, ct[2] = { 0, 0 };

  for (int i = 0; i < n; i++)
    {
      cr[0] += ref[idx[i]][0];
      cr[1] += ref[idx[i]][1];
      ct[0] += test[idx[i]][0];
      ct[1] += test[idx[i]][1];
    }
  cr[0] /= n;
  cr[1] /= n;
  ct[0] /= n;
  ct[1] /= n;
  double num = 0, den = 0;
  for (int i = 0; i < n; i++)
    {
      double ax = test[idx[i]][0] - ct[0], ay = test[idx[i]][1] - ct[1];
      double bx = ref[idx[i]][0] - cr[0], by = ref[idx[i]][1] - cr[1];
      num += ax * by - ay * bx;
      den += ax * bx + ay * by;
    }
  double th = atan2 (num, den), c = cos (th), s = sin (th);
  Rigid r = { (float) th, (float) (cr[0] - (c * ct[0] - s * ct[1])), (float) (cr[1] - (s * ct[0] + c * ct[1])) };
  return r;
}

static void
rigid_apply (const Rigid *r, float x, float y, float *ox, float *oy)
{
  float c = cosf (r->th), s = sinf (r->th);

  *ox = c * x - s * y + r->dx;
  *oy = s * x + c * y + r->dy;
}

#define MAXC FT9366_MAXKP

/* returns consensus size (0 if none) and the fitted transform */
static int
consistent_subset (const float (*ref)[2], const float (*test)[2], int n, Rigid *fit)
{
  static uint8_t D[MAXC][MAXC], D2[MAXC][MAXC];
  int deg[MAXC], deg2[MAXC], L[MAXC], keep[MAXC + 1], inl[MAXC];
  float vr[MAXC][2], vt[MAXC][2];

  if (n < 5)
    return 0;
  for (int a = 0; a < n; a++)
    {
      deg[a] = 0;
      for (int b = 0; b < n; b++)
        {
          float dr = hypotf (ref[a][0] - ref[b][0], ref[a][1] - ref[b][1]);
          float dt = hypotf (test[a][0] - test[b][0], test[a][1] - test[b][1]);
          D[a][b] = (a == b) || fabsf (dr - dt) < 2.0f;
          deg[a] += D[a][b];
        }
    }
  /* two seeds with highest degree (stable) */
  int seeds[2] = { -1, -1 };
  for (int s = 0; s < 2; s++)
    {
      int best = -1;
      for (int a = 0; a < n; a++)
        {
          if (s == 1 && a == seeds[0])
            continue;
          if (best < 0 || deg[a] > deg[best])
            best = a;
        }
      seeds[s] = best;
    }
  for (int si = 0; si < 2; si++)
    {
      int seed = seeds[si], nl = 0;
      for (int j = 0; j < n; j++)
        if (D[seed][j] && deg[j] > 4)
          L[nl++] = j;
      if (nl < 3)
        continue;
      for (int k = 0; k < nl; k++)
        {
          vr[k][0] = ref[L[k]][0] - ref[seed][0];
          vr[k][1] = ref[L[k]][1] - ref[seed][1];
          vt[k][0] = test[L[k]][0] - test[seed][0];
          vt[k][1] = test[L[k]][1] - test[seed][1];
        }
      for (int a = 0; a < nl; a++)
        for (int b = 0; b < nl; b++)
          {
            float cr = vr[a][0] * vr[b][1] - vr[a][1] * vr[b][0];
            float ct = vt[a][0] * vt[b][1] - vt[a][1] * vt[b][0];
            float d2r = hypotf (ref[L[a]][0] - ref[L[b]][0], ref[L[a]][1] - ref[L[b]][1]);
            float d2t = hypotf (test[L[a]][0] - test[L[b]][0], test[L[a]][1] - test[L[b]][1]);
            D2[a][b] = (cr * ct >= 0) && fabsf (d2r - d2t) < 1.0f;
          }
      for (int k = 0; k < nl; k++)
        if (L[k] == seed)
          for (int b = 0; b < nl; b++)
            {
              D2[k][b] = 0;
              D2[b][k] = 0;
            }
      int dmax = 0, kbest = 0;
      for (int a = 0; a < nl; a++)
        {
          D2[a][a] = 1;
          deg2[a] = 0;
          for (int b = 0; b < nl; b++)
            deg2[a] += D2[a][b];
          if (deg2[a] > dmax)
            {
              dmax = deg2[a];
              kbest = a;
            }
        }
      if (dmax < 3)
        continue;
      int nk = 0;
      for (int l = 0; l < nl; l++)
        if (D2[kbest][l] && deg2[l] > 3 && L[l] != seed)
          keep[nk++] = L[l];
      keep[nk++] = seed;
      if (nk < 5)
        continue;
      Rigid f = rigid_fit (ref, test, keep, nk);
      int ni = 0;
      for (int i = 0; i < n; i++)
        {
          float px, py;
          rigid_apply (&f, test[i][0], test[i][1], &px, &py);
          if (fabsf (px - ref[i][0]) < 1.2f && fabsf (py - ref[i][1]) < 1.2f)
            inl[ni++] = i;
        }
      if (ni > 4)
        {
          *fit = rigid_fit (ref, test, inl, ni);
          return nk < ni ? nk : ni;
        }
    }
  return 0;
}

static void
unpack_map (const Ft9366Template *t, uint8_t *bits, uint8_t *valid)
{
  for (int i = 0; i < FT9366_NPIX; i++)
    {
      bits[i] = t->map[i] & 1;
      valid[i] = (t->map[i] >> 1) & 1;
    }
}

static void
overlay (const Ft9366Template *s, const Ft9366Template *r, const Rigid *f, float *ratio, int *overlap)
{
  uint8_t sb[FT9366_NPIX], sv[FT9366_NPIX], rb[FT9366_NPIX], rv[FT9366_NPIX];
  float agree[FT9366_NPIX];
  uint8_t ov[FT9366_NPIX];

  unpack_map (s, sb, sv);
  unpack_map (r, rb, rv);
  float c = cosf (f->th), sn = sinf (f->th);
  int nov = 0;
  for (int y = 0; y < FT9366_H; y++)
    for (int x = 0; x < FT9366_W; x++)
      {
        float ox = x - f->dx, oy = y - f->dy;
        int sx = (int) floorf (c * ox + sn * oy + 0.5f), sy = (int) floorf (-sn * ox + c * oy + 0.5f);
        uint8_t wb = 0, wv = 0;
        if (sx >= 0 && sx < FT9366_W && sy >= 0 && sy < FT9366_H)
          {
            wb = sb[sy * FT9366_W + sx];
            wv = sv[sy * FT9366_W + sx];
          }
        ov[y * FT9366_W + x] = wv && rv[y * FT9366_W + x];
        agree[y * FT9366_W + x] = wb == rb[y * FT9366_W + x];
        nov += ov[y * FT9366_W + x];
      }
  int nu = 0;
  for (int y = 0; y < FT9366_H; y++)
    for (int x = 0; x < FT9366_W; x++)
      {
        if (!ov[y * FT9366_W + x])
          continue;
        int cnt = 0;
        for (int dy = -2; dy <= 2; dy++)
          for (int dx = -2; dx <= 2; dx++)
            cnt += agree[refl (y + dy, FT9366_H) * FT9366_W + refl (x + dx, FT9366_W)] > 0.5f;
        if (cnt >= 19)
          nu++;                /* >= 75% of the 5x5 neighbourhood */
      }
  *ratio = nu / (nov + 1.f);
  *overlap = nov;
}

void
ft9366_match_dir (const Ft9366Template *ps, const Ft9366Template *pt, Ft9366MatchResult *res)
{
  static const int margin_tab[9] = { 1, 1, 1, 1, 1, 2, 2, 1, 1 };
  int ci[FT9366_MAXKP], cj[FT9366_MAXKP], nc = 0;

  res->score = 0;
  res->ratio = 0;
  res->overlap = 0;
  if (ps->n == 0 || pt->n < 2)
    return;

  for (uint32_t i = 0; i < ps->n; i++)
    {
      int best = -1, second = -1, bj = -1;
      for (uint32_t j = 0; j < pt->n; j++)
        {
          int h = 0;
          const uint64_t *a = (const uint64_t *) ps->kp[i].desc, *b = (const uint64_t *) pt->kp[j].desc;
          for (int w = 0; w < 4; w++)
            h += __builtin_popcountll (a[w] ^ b[w]);
          int sim = 256 - h;
          if (sim > best)
            {
              second = best;
              best = sim;
              bj = (int) j;
            }
          else if (sim > second)
            {
              second = sim;
            }
        }
      if (best > 159 && second < best - margin_tab[best >> 5])
        {
          ci[nc] = (int) i;
          cj[nc] = bj;
          nc++;
        }
    }
  if (nc < 4)
    return;

  int bins[FT9366_MAXKP];
  for (int k = 0; k < nc; k++)
    {
      float d = fabsf (ps->kp[ci[k]].ori - pt->kp[cj[k]].ori);
      if (d > PI_F)
        d = 2 * PI_F - d;
      bins[k] = (int) floorf (d * 5.729578f);
    }
  int bestm = 0;
  Rigid bestf = { 0, 0, 0 };
  for (int w = 1; w < 18; w++)
    {
      float ref[FT9366_MAXKP][2], test[FT9366_MAXKP][2];
      int n = 0;
      for (int k = 0; k < nc; k++)
        if (bins[k] == w || bins[k] == w - 1)
          {
            ref[n][0] = pt->kp[cj[k]].x;
            ref[n][1] = pt->kp[cj[k]].y;
            test[n][0] = ps->kp[ci[k]].x;
            test[n][1] = ps->kp[ci[k]].y;
            n++;
          }
      if (n < 4)
        continue;
      Rigid f;
      int m = consistent_subset (ref, test, n, &f);
      if (m > bestm)
        {
          bestm = m;
          bestf = f;
        }
    }
  if (!bestm)
    return;

  int cnt = 0;
  for (uint32_t i = 0; i < ps->n; i++)
    {
      float px, py;
      rigid_apply (&bestf, ps->kp[i].x, ps->kp[i].y, &px, &py);
      float o = ps->kp[i].ori + bestf.th;
      o = fmodf (o + PI_F, 2 * PI_F);
      if (o < 0)
        o += 2 * PI_F;
      o -= PI_F;
      for (uint32_t j = 0; j < pt->n; j++)
        {
          if (fabsf (pt->kp[j].x - px) >= 1.5f || fabsf (pt->kp[j].y - py) >= 1.5f)
            continue;
          float d = fabsf (o - pt->kp[j].ori);
          if (d > PI_F)
            d = 2 * PI_F - d;
          if (d <= 0.2f)
            {
              cnt++;
              break;
            }
        }
    }
  int score = cnt ? cnt : bestm;
  float ratio;
  int nov;
  overlay (ps, pt, &bestf, &ratio, &nov);
  if (ratio < 0.82f)
    score = (int) (score * 1.11f * ratio);
  if (nov < 400)
    score = 0;
  res->score = score;
  res->ratio = ratio;
  res->overlap = nov;
}

void
ft9366_compare (const Ft9366Template *a, const Ft9366Template *b, Ft9366MatchResult *r)
{
  Ft9366MatchResult r2;

  ft9366_match_dir (a, b, r);
  ft9366_match_dir (b, a, &r2);
  if (r2.score > r->score)
    *r = r2;
}

int
ft9366_is_match (const Ft9366MatchResult *r)
{
  return r->score >= FT9366_ACCEPT_SCORE && r->ratio >= FT9366_ACCEPT_RATIO;
}

/* ------------------------------------------------------------------------------------------ */

void
ft9366_template_pack (const Ft9366Template *t, uint8_t *out)
{
  memcpy (out, &t->n, 4);
  for (int i = 0; i < FT9366_MAXKP; i++)
    {
      memcpy (out + 4 + i * 44, &t->kp[i].x, 4);
      memcpy (out + 4 + i * 44 + 4, &t->kp[i].y, 4);
      memcpy (out + 4 + i * 44 + 8, &t->kp[i].ori, 4);
      memcpy (out + 4 + i * 44 + 12, t->kp[i].desc, 32);
    }
  memcpy (out + 4 + FT9366_MAXKP * 44, t->map, FT9366_NPIX);
}

int
ft9366_template_unpack (const uint8_t *in, uint32_t len, Ft9366Template *t)
{
  if (len != FT9366_TEMPLATE_PACKED_SIZE)
    return -1;
  memcpy (&t->n, in, 4);
  if (t->n > FT9366_MAXKP)
    return -1;
  for (int i = 0; i < FT9366_MAXKP; i++)
    {
      memcpy (&t->kp[i].x, in + 4 + i * 44, 4);
      memcpy (&t->kp[i].y, in + 4 + i * 44 + 4, 4);
      memcpy (&t->kp[i].ori, in + 4 + i * 44 + 8, 4);
      memcpy (t->kp[i].desc, in + 4 + i * 44 + 12, 32);
    }
  memcpy (t->map, in + 4 + FT9366_MAXKP * 44, FT9366_NPIX);
  return 0;
}
