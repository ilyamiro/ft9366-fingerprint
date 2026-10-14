{ libfprint }:

# libfprint with the FocalTech FT9366 (USB 2808:a658) driver added.
libfprint.overrideAttrs (old: {
  pname = "libfprint-ft9366";

  postPatch = (old.postPatch or "") + ''
    cp ${./ft9366.c} libfprint/drivers/ft9366.c
    cp ${./ft9366_script.h} libfprint/drivers/ft9366_script.h
    cp ${./ft9366_match.c} libfprint/drivers/ft9366_match.c
    cp ${./ft9366_match.h} libfprint/drivers/ft9366_match.h
    cp ${./ft9366_pairs.h} libfprint/drivers/ft9366_pairs.h

    substituteInPlace libfprint/meson.build \
      --replace-fail "'nb1010' : files('drivers/nb1010.c')," \
                     "'nb1010' : files('drivers/nb1010.c'),
    'ft9366' : files('drivers/ft9366.c'),"
    substituteInPlace meson.build \
      --replace-fail "'nb1010': {}," \
                     "'nb1010': {},
    'ft9366': {},"

    # The device is no longer "known but unsupported". Also keep it out of the
    # autosuspend hwdb: this sensor misbehaves when the bridge is autosuspended.
    sed -i '/\.pid = 0xa658 }/d' libfprint/fprint-list-udev-hwdb.c
    substituteInPlace libfprint/fprint-list-udev-hwdb.c \
      --replace-fail "  /* https://bugs.freedesktop.org/show_bug.cgi?id=66659 */" \
                     "  { .vid = 0x2808, .pid = 0xa658 },
      /* https://bugs.freedesktop.org/show_bug.cgi?id=66659 */"
    sed -i '/usb:v2808pA658/d' data/autosuspend.hwdb
  '';
})
