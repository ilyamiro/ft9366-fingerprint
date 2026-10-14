{
  description = "Open-source libfprint driver for the FocalTech FT9366 reader (USB 2808:a658)";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
    in
    {
      # libfprint with the ft9366 driver added
      packages.${system} = rec {
        libfprint-ft9366 = pkgs.callPackage ./driver/libfprint-ft9366.nix { libfprint = pkgs.libfprint; };
        default = libfprint-ft9366;
      };

      overlays.default = final: prev: {
        libfprint-ft9366 = final.callPackage ./driver/libfprint-ft9366.nix { libfprint = prev.libfprint; };
      };

      # services.fprintd with the driver + udev rule that keeps the USB bridge awake
      nixosModules.default = ./driver/nixos-ft9366.nix;
    };
}
