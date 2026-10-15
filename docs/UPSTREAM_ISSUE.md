# Draft: libfprint issue

**Title:** Add driver for FocalTech FT9366 (2808:a658, Realtek RTS5811 bridge) with a driver-private partial-print matcher

---

Hi,

I have a working driver for the **FocalTech FT9366** fingerprint reader (USB `2808:a658`, behind a Realtek RTS5811
USB bridge). The ID is currently in libfprint's "known but unsupported" list (it shows up in laptops, e.g. [list the
laptop models you know of]). I would like to contribute it, but it needs a design decision from you first, so I'm
asking before opening a merge request.

## The device

- Plain USB device, vendor-specific interface, bulk OUT `0x01` / bulk IN `0x82`, no encryption or pairing.
- It delivers a **64x80 pixel image, about 3.2 x 4 mm (~500 dpi)** — much smaller than the sensors NBIS was built for.
- It is **not** match-on-chip: the host gets raw images and has to do the matching.
- It is not the same hardware as the existing `focaltech_moc` driver (those are match-on-chip devices with a
  different protocol).

## The problem: NBIS cannot match this sensor

I first tried to use libfprint's regular image-device path (mindtct + bozorth3), running libfprint's bundled NBIS
exactly as `FpImageDevice` does on saved frames:

- mindtct finds **0–1 minutiae per frame** (5–9 if the image is mirror-padded by 16 px so borders are not
  invalidated). Bozorth3 needs more than that to produce a meaningful score.
- Cross-touch bozorth3 scores of the same finger were **0** for every pre-processing variant I tried (smoothing,
  contrast normalisation, 2x/3x upscaling, padding).
- Stitching several touches into a mosaic to get more minutiae did not help: alignment drifted and the mosaic
  still yielded only ~7 minutiae.
- On hardware, a driver built on the NBIS path matched 0 of 6 genuine touches.

So the standard `FpImageDevice` + NBIS path cannot work for this sensor.

## What I implemented

A regular `FpDevice` driver (async `FpiSsm`, no threads) that does its own matching and stores the result in an
`FpPrint` as raw data, similar in spirit to match-on-host drivers that don't use NBIS:

- Each enrollment touch (12 per print) is converted to an 8-bit image, then reduced to scale-space keypoints with
  256-bit binary descriptors plus a binary ridge map and a valid-pixel mask (about 13 KB per touch).
- Verify/identify compares a touch against every stored template: descriptor matches, a rigid-transform
  consensus, then ridge-map overlay agreement. The decision is two thresholds on the resulting score and
  agreement ratio.
- The matcher is plain C with no dependencies besides libm, in its own source file
  (`ft9366_match.c`) separate from the USB code.

Measured on a small data set of my own fingers (75 touches, 5 fingers, deliberately varied placement):

- 0 false accepts in 2231 different-finger comparisons;
- about 55% of genuine touches accepted with 10 touches enrolled (a retry is needed fairly often).

These numbers come from one person and were also used to choose the thresholds, so they are optimistic. I would
like to collect data from more people before asking for a merge, and I can describe the test method in more
detail.

## Questions

1. Would you accept a driver that ships its own matcher instead of using NBIS? If so, is there anything in how
   it is integrated you would want different (separate source file in `drivers/ft9366/`, how prints are stored,
   `FP_DEVICE_FEATURE_*` expectations)?
2. The sensor is small, so false rejects are common (~45% per touch here). Is that acceptable for an
   "experimental" driver, and is there a convention for marking a driver as such?
3. Template size is ~13 KB × 12 per print (~160 KB). Is that a problem for fprintd's storage?
4. For tests I plan to add a umockdev capture (`tests/ft9366/`) plus a unit test of the matcher. Is a capture
   of a real session acceptable if it contains only the scan of a finger, or would you rather have a synthetic one?
5. The USB bridge misbehaves when auto-suspended, so I propose adding `2808:a658` to the denylist in
   `fprint-list-udev-hwdb.c` so it does not get `ID_AUTOSUSPEND=1`. Is that the right place?

## Provenance

I want to be upfront about how this was made, since it matters for licensing:

- **Protocol:** recorded with a USB capture of the vendor's closed-source driver talking to the reader. I also
  looked at the vendor driver binary with a disassembler/decompiler (static analysis only) to understand the
  structure and register/command names. The command sequences the driver replays come from the capture; the
  command table in the source is annotated only with meanings inferred from the traffic.
- **Matcher:** the *approach* (keypoints with 256-bit binary descriptors, rigid-transform consensus, ridge-map
  overlay agreement) was informed by that static analysis of the vendor library, and the standard techniques
  behind it are public (SIFT/BRIEF-style features, RANSAC-style consensus). The *code* is my own C
  implementation. I did not copy vendor source code, binaries, trained classifier data or tables. The descriptor
  sampling pairs are generated from a seeded random generator (`tools/gen_pairs.py`), not taken from the vendor.
- **Parameters:** the accept thresholds were chosen from my own genuine/impostor measurements. One of them
  (minimum consensus score 7) happens to equal the minimum score I saw configured in the vendor library. The
  8-bit image conversion was checked against the vendor's output on my data set, so it behaves the same way on
  that data.
- No vendor code, binary or data file is included in the repository.

If this is not acceptable for inclusion, or you would want a clean-room re-derivation of any part (for example the
image conversion or the thresholds) before a merge request, I'd like to know before investing more time.

The code is LGPL-2.1-or-later. The work is at: https://github.com/ilyamiro/ft9366-fingerprint (branch `upstream-prep`
has it laid out like libfprint's tree).

Hardware: [laptop model], `lsusb`: `Bus xxx Device xxx: ID 2808:a658 FocalTech Systems Co., Ltd`.
