{ libfprint }:

# libfprint with the FocalTech FT9366 (USB 2808:a658) driver added.
#
# The driver sources live in ../libfprint/drivers/ft9366 (same layout as upstream libfprint) and
# the changes to libfprint's own files are in ../patches (these are what an upstream merge request
# would contain), so this package builds exactly the tree that is proposed upstream.
libfprint.overrideAttrs (old: {
  pname = "libfprint-ft9366";

  patches = (old.patches or [ ]) ++ [ ../patches/0001-ft9366-register-driver.patch ];

  postPatch = (old.postPatch or "") + ''
    cp -r ${../libfprint/drivers/ft9366} libfprint/drivers/ft9366
    chmod -R u+w libfprint/drivers/ft9366
  '';
})
