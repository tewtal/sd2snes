/* sd2snes - SD card based universal cartridge for the SNES
   Copyright (C) 2009-2010 Maximilian Rehkopf <otakon@gmx.net>

   USB2P v3 path handling (README.usb2p.md §10.1).  All path/filename fields on
   the wire are UTF-8; the bundled FatFs is configured for OEM code page 1252.
   This module transcodes UTF-8 -> CP-1252 at the protocol boundary, normalizes
   the result, and rejects anything FatFs cannot represent or that would escape
   the SD root — so the FS layer above never sees a malformed or unsafe path.
*/

#include <stdint.h>

#include "fileops.h"
#include "usb2p.h"
#include "usb2p_internal.h"

/* Validated/transcoded path scratch.  One op runs at a time, so two buffers
   suffice (RENAME needs both source and destination). */
char path_payload[USB2P_MAX_PATH_BYTES + 1];
char path_payload2[USB2P_MAX_PATH_BYTES + 1];

/* Map a Unicode code point to its CP-1252 byte (the FatFs OEM code page), or 0
   if not representable.  U+0000 maps to 0 (not-representable) too — a NUL is
   never a valid path char and the caller rejects it via the returned 0.
   CP-1252 == Latin-1 except 0x80..0x9F, which carries a scatter of typographic
   code points (the explicit table); the rest is identity for U+0000..U+00FF. */
static uint8_t usb2p_cp1252_from_codepoint(uint32_t cp) {
  static const uint16_t high[32] = {
    0x20AC, 0x0000, 0x201A, 0x0192, 0x201E, 0x2026, 0x2020, 0x2021, /* 80-87 */
    0x02C6, 0x2030, 0x0160, 0x2039, 0x0152, 0x0000, 0x017D, 0x0000, /* 88-8F */
    0x0000, 0x2018, 0x2019, 0x201C, 0x201D, 0x2022, 0x2013, 0x2014, /* 90-97 */
    0x02DC, 0x2122, 0x0161, 0x203A, 0x0153, 0x0000, 0x017E, 0x0178  /* 98-9F */
  };
  uint8_t i;

  if (cp == 0) {
    return 0;
  }
  if (cp < 0x80 || (cp >= 0xA0 && cp <= 0xFF)) {
    return (uint8_t)cp;  /* identity range (ASCII + Latin-1 supplement) */
  }
  for (i = 0; i < 32; i++) {
    if (high[i] && high[i] == cp) {
      return (uint8_t)(0x80 + i);
    }
  }
  return 0;  /* not representable in CP-1252 */
}

/* Decode one UTF-8 sequence at p[0..len-1].  On success store the code point in
   *cp, advance *consumed by the sequence length, and return 1.  Reject
   malformed, truncated, overlong, surrogate, and out-of-range sequences. */
static uint8_t usb2p_utf8_decode(const uint8_t *p, uint32_t len,
                                 uint32_t *cp, uint32_t *consumed) {
  uint8_t b0 = p[0];
  uint32_t value;
  uint32_t n;
  uint32_t i;
  uint32_t min;

  if (b0 < 0x80) {
    *cp = b0;
    *consumed = 1;
    return 1;
  }
  if (b0 < 0xC0) {
    return 0;  /* stray continuation byte */
  } else if (b0 < 0xE0) {
    n = 2; value = b0 & 0x1F; min = 0x80;
  } else if (b0 < 0xF0) {
    n = 3; value = b0 & 0x0F; min = 0x800;
  } else if (b0 < 0xF8) {
    n = 4; value = b0 & 0x07; min = 0x10000;
  } else {
    return 0;  /* 5/6-byte forms are not valid UTF-8 */
  }
  if (n > len) {
    return 0;  /* truncated */
  }
  for (i = 1; i < n; i++) {
    if ((p[i] & 0xC0) != 0x80) {
      return 0;  /* bad continuation */
    }
    value = (value << 6) | (p[i] & 0x3F);
  }
  if (value < min) {
    return 0;  /* overlong encoding */
  }
  if (value > 0x10FFFF || (value >= 0xD800 && value <= 0xDFFF)) {
    return 0;  /* out of range / surrogate */
  }
  *cp = value;
  *consumed = n;
  return 1;
}

/* Decode a UTF-8 path into CP-1252 bytes in `tmp`, rejecting malformed UTF-8 or
   characters CP-1252 cannot represent.  Also rejects control chars and the
   FAT-reserved ':' and '\\'.  '/' is the only separator and passes through.
   Returns the CP-1252 byte length in *out_len, or 0 on reject. */
static uint8_t usb2p_path_to_cp1252(char *tmp, const uint8_t *payload,
                                    uint64_t length, uint32_t *out_len) {
  uint32_t in = 0;
  uint32_t out = 0;

  if (length == 0 || length > USB2P_MAX_PATH_BYTES) {
    return 0;
  }
  while (in < (uint32_t)length) {
    uint32_t cp;
    uint32_t consumed;
    uint8_t byte;

    if (!usb2p_utf8_decode(payload + in, (uint32_t)length - in, &cp, &consumed)) {
      return 0;  /* malformed UTF-8 -> EUNREPRESENTABLE */
    }
    byte = usb2p_cp1252_from_codepoint(cp);
    if (byte == 0) {
      return 0;  /* not representable (incl. embedded NUL) */
    }
    if (byte < 0x20 || byte == ':' || byte == '\\') {
      return 0;  /* control / FAT-reserved */
    }
    if (out >= USB2P_MAX_PATH_BYTES) {
      return 0;  /* transcoded path exceeds the full-path byte cap */
    }
    tmp[out++] = (char)byte;
    in += consumed;
  }
  tmp[out] = 0;
  *out_len = out;
  return 1;
}

/* Normalize a CP-1252 path in place (§6.8): collapse '//' and '.' segments,
   apply '..' by popping the previous segment, and reject any '..' that would
   escape the SD root.  A leading '/' (absolute) is preserved; the result never
   contains '.' or '..' segments.  Returns 1 on success, 0 if the path escapes
   root or empties to nothing.  Operates on a NUL-terminated string of len. */
static uint8_t usb2p_path_normalize(char *path, uint32_t len) {
  uint32_t r = 0;          /* read cursor */
  uint32_t w = 0;          /* write cursor */
  uint8_t absolute = (len > 0 && path[0] == '/');

  if (absolute) {
    path[w++] = '/';
    r = 1;
  }
  while (r < len) {
    uint32_t seg_start = r;
    uint32_t seg_len;

    while (r < len && path[r] != '/') {
      r++;
    }
    seg_len = r - seg_start;
    if (r < len) {
      r++;  /* skip the separator */
    }

    if (seg_len == 0) {
      continue;  /* collapse empty segment ("//" or trailing "/") */
    }
    if (seg_len == 1 && path[seg_start] == '.') {
      continue;  /* drop "." */
    }
    if (seg_len == 2 && path[seg_start] == '.' && path[seg_start + 1] == '.') {
      /* Pop the previous segment; refuse to climb above root. */
      if (w == 0 || (absolute && w == 1)) {
        return 0;  /* escapes the SD root */
      }
      if (w > 0 && path[w - 1] == '/') {
        w--;  /* step back over the separator we wrote after the last segment */
      }
      while (w > 0 && path[w - 1] != '/') {
        w--;
      }
      continue;
    }
    /* Ordinary segment: ensure a separator before it (except the very first in a
       relative path) and copy it. */
    if (w > 0 && path[w - 1] != '/') {
      path[w++] = '/';
    }
    {
      uint32_t k;
      for (k = 0; k < seg_len; k++) {
        path[w++] = path[seg_start + k];
      }
    }
  }

  /* Strip a trailing separator (except a lone root "/"). */
  if (w > 1 && path[w - 1] == '/') {
    w--;
  }
  if (w == 0) {
    return 0;  /* normalized to empty */
  }
  path[w] = 0;
  return 1;
}

/* Validate + transcode + normalize a wire path into dst.  Returns:
     USB2P_STATUS_OK              dst holds a normalized CP-1252 path
     USB2P_STATUS_EINVAL          structural fault (empty / over the byte cap)
     USB2P_STATUS_EUNREPRESENTABLE malformed UTF-8, char not in CP-1252, a
                                  reserved char, or a '..' that escapes root.
   The EINVAL/EUNREPRESENTABLE split lets callers map to NAK vs RSP(status) per
   the §6.2/§6.3 taxonomy (a well-formed-but-unrepresentable path is a backend
   reject, not a framing fault). */
uint8_t usb2p_copy_path_status_to(char *dst, const uint8_t *payload,
                                  uint64_t length) {
  uint32_t len;

  if (length == 0 || length > USB2P_MAX_PATH_BYTES) {
    return USB2P_STATUS_EINVAL;
  }
  if (!usb2p_path_to_cp1252(dst, payload, length, &len)) {
    return USB2P_STATUS_EUNREPRESENTABLE;
  }
  if (!usb2p_path_normalize(dst, len)) {
    return USB2P_STATUS_EUNREPRESENTABLE;
  }
  return USB2P_STATUS_OK;
}

uint8_t usb2p_copy_path_status(const uint8_t *payload, uint64_t length) {
  return usb2p_copy_path_status_to(path_payload, payload, length);
}

uint32_t usb2p_fattime_from_filinfo(const FILINFO *fno) {
  return ((uint32_t)fno->fdate << 16) | fno->ftime;
}
