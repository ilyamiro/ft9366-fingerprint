# ft9366-fingerprint

Open-source fingerprint driver for the **FocalTech FT9366** reader (USB `2808:a658`, behind a Realtek
RTS5811 bridge), as found in several laptops. Works with `fprintd`, so you get `fprintd-enroll`,
`fprintd-verify`, and fingerprint login/`sudo`. Packaged for **NixOS**.

## Install (NixOS with flakes)
```nix
# flake.nix
inputs.ft9366.url = "github:ilyamiro/ft9366-fingerprint";

# in your nixosSystem modules:
ft9366.nixosModules.default
```

## Install (NixOS without flakes)
```nix
# configuration.nix
imports = [ /path/to/ft9366-fingerprint/driver/nixos-ft9366.nix ];
```

Then:
```
sudo nixos-rebuild switch
fprintd-enroll      # press the same finger 12 times, a bit differently each time
fprintd-verify
```
Optional login / sudo: `security.pam.services.sudo.fprintAuth = true;`
(keep your password as a fallback).

## What to expect
The sensor is tiny (3x4 mm). On the author's fingers roughly half to two thirds of genuine touches
match on the first try, and other fingers were rejected in every test. Press flat and firmly, cover the
whole sensor, and retry on a miss. Thresholds were tuned on one person's fingers; please report how it
works for you. See [docs/HOW_IT_WORKS.md](docs/HOW_IT_WORKS.md).

## Try it without installing
```
cd driver && nix-build -E 'with import <nixpkgs> {}; callPackage ./libfprint-ft9366.nix {}' -o result
cd ../test && ./build_test.sh && ./run_driver_test.sh 8   # stop fprintd first
```

## About this implementation
This is an independent implementation. It was written from observation of the reader's USB traffic and
standard, published image-matching techniques. It contains no vendor source code, no vendor binaries, and
no vendor tables or trained data. It is not affiliated with or endorsed by FocalTech or Realtek.

## Licence
LGPL-2.1-or-later (same as libfprint).
