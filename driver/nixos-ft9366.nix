# NixOS module for the FocalTech FT9366 (USB 2808:a658) reader with the open-source driver.
# NOT applied automatically. To use it, import it from your configuration.nix:
#
#   imports = [ /path/to/ft9366-fingerprint/driver/nixos-ft9366.nix ];
#
# then `sudo nixos-rebuild switch` and enroll with `fprintd-enroll` (12 touches).
{ config, lib, pkgs, ... }:

let
  libfprint-ft9366 = pkgs.callPackage ./libfprint-ft9366.nix { libfprint = pkgs.libfprint; };
in
{
  services.fprintd = {
    enable = true;
    package = pkgs.fprintd.override { libfprint = libfprint-ft9366; };
  };

  # The USB bridge misbehaves when it is autosuspended; keep it powered.
  services.udev.extraRules = ''
    ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2808", ATTR{idProduct}=="a658", TEST=="power/control", ATTR{power/control}="on"
  '';

  # Use the reader for login / sudo (optional, adjust to taste):
  # security.pam.services.login.fprintAuth = true;
  # security.pam.services.sudo.fprintAuth = true;
}
