/* offline evaluation of the open matcher on the captured dataset (development tool) */
#include "ft9366_match.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static int rd (const char *p, uint16_t *o)
{ uint8_t b[FT_NPIX * 2]; FILE *f = fopen (p, "rb"); if (!f || fread (b, 1, sizeof b, f) != sizeof b) return -1; fclose (f);
  for (int i = 0; i < FT_NPIX; i++) o[i] = (b[2 * i] << 8) | b[2 * i + 1]; return 0; }
int main (int argc, char **argv)
{
  FILE *mf = fopen (argv[1], "r"); char line[8192]; static FtTemplate T[128]; char lab[128][64]; int n = 0;
  while (fgets (line, sizeof line, mf))
    {
      char *tok[600]; int nt = 0;
      for (char *p = strtok (line, " \t\n"); p && nt < 600; p = strtok (NULL, " \t\n")) tok[nt++] = p;
      if (nt < 6) continue;
      uint16_t a[FT_NPIX], b[FT_NPIX], base[FT_NPIX], raw[FT_NPIX], f[FT_NPIX]; uint32_t acc[FT_NPIX] = { 0 }; uint8_t img[FT_NPIX];
      int nfr = atoi (tok[4]);
      if (rd (tok[2], a) || rd (tok[3], b)) return 1;
      for (int i = 0; i < FT_NPIX; i++) base[i] = (a[i] + b[i]) / 2;
      for (int k = 0; k < nfr; k++) { if (rd (tok[5 + k], f)) return 1; for (int i = 0; i < FT_NPIX; i++) acc[i] += f[i]; }
      for (int i = 0; i < FT_NPIX; i++) raw[i] = acc[i] / nfr;
      ft_convert_image (raw, base, img);
      int nk = ft_template_build (img, &T[n]);
      strncpy (lab[n], tok[1], 63);
      printf ("T %d %s %d\n", n, lab[n], nk);
      if (argc > 2) { char p[256]; snprintf (p, sizeof p, "%s/img_%03d.pgm", argv[2], n); FILE *o = fopen (p, "wb"); fprintf (o, "P5\n64 80\n255\n"); fwrite (img, 1, FT_NPIX, o); fclose (o); }
      n++;
    }
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++)
      { FtMatchResult r; ft_compare (&T[i], &T[j], &r); printf ("S %d %d %d %.4f %d\n", i, j, r.score, r.ratio, r.overlap); }
  return 0;
}
