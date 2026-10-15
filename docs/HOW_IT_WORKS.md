# How it works

## Why a custom matcher
The sensor is a 64x80 pixel array, about 3x4 mm. libfprint's built-in matcher (NBIS) looks for
fingerprint minutiae and finds almost none in an area this small, so it cannot match anything.
The driver therefore does its own matching of partial prints.

## Reading the sensor
The reader is a USB bridge with two bulk endpoints (OUT 0x01, IN 0x82). The driver resets it,
replays an initialisation sequence (recorded from the device's USB traffic), and then polls an
interrupt-flag register for a finger. On a touch it reads a few 64x80 frames of 16-bit samples,
skips the first two (partial contact) and averages the next three. The empty-sensor frames taken
during initialisation serve as the reference image.

## From frame to template
1. Subtract the reference, repair outliers, and stretch the contrast to an 8-bit image.
2. Remove the local mean (11x11) so ridges are centred on mid-grey.
3. Find scale-space (difference-of-Gaussian) keypoints and keep the 160 strongest.
4. Describe each keypoint with a 256-bit binary descriptor (brightness comparisons on a rotated
   sampling pattern).
5. Store a binary ridge map and a valid-pixel mask next to the keypoints.

One template is stored per enrollment touch (12 touches, about 13 KB each).

## Matching a touch against a template
1. Candidate keypoint pairs from descriptor similarity (best match clearly better than the second).
2. Group candidates by rotation difference and find the largest set that agrees on one rigid
   transform (rotation + translation).
3. Count how many keypoints land on a template keypoint with a consistent orientation.
4. Warp the ridge maps together and measure how many overlapping pixels agree locally; a poor
   agreement lowers the score.
5. Accept when the score is >= 7 and the agreement ratio is >= 0.7 (`FT_ACCEPT_*` in
   `libfprint/drivers/ft9366/ft9366_match.h`). A touch matches if it matches any stored template.

## Measured behaviour
On a test set of 75 touches from 5 fingers of one person, with deliberately varied finger placement:
- 0 false accepts in 2231 different-finger comparisons;
- about 55% of genuine touches accepted when 10 touches are enrolled.

These numbers come from a single person and were used to pick the thresholds, so treat them as
optimistic. More testing on other people's fingers is welcome. Expect to retry a touch sometimes;
rejecting a genuine touch is the safe failure.
