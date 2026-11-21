/*
 * This is an OpenSSL-compatible implementation of the RSA Data Security, Inc.
 * MD5 Message-Digest Algorithm (RFC 1321).
 *
 * Homepage:
 * http://openwall.info/wiki/people/solar/software/public-domain-source-code/md5
 *
 * Author:
 * Alexander Peslyak, better known as Solar Designer <solar at openwall.com>
 *
 * This software was written by Alexander Peslyak in 2001.  No copyright is
 * claimed, and the software is hereby placed in the public domain.
 * In case this attempt to disclaim copyright and place the software in the
 * public domain is deemed null and void, then the software is
 * Copyright (c) 2001 Alexander Peslyak and it is hereby released to the
 * general public under the following terms:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted.
 *
 * There's ABSOLUTELY NO WARRANTY, express or implied.
 *
 * See md5.c for more information.
 *
 * This version is used internally by the WavPack library (for the raw input
 * checksum feature) and the symbols have been renamed with a "Wavpack" prefix
 * so that they cannot clash with the identical code in the command-line
 * programs (or any other MD5 implementation an application might use).
 */
#ifndef _WAVPACK_MD5_H
#define _WAVPACK_MD5_H

/* Any 32-bit or wider unsigned integer data type will do */
typedef unsigned int WavpackMD5_u32plus;

typedef struct {
	WavpackMD5_u32plus lo, hi;
	WavpackMD5_u32plus a, b, c, d;
	unsigned char buffer[64];
	WavpackMD5_u32plus block[16];
} WavpackMD5_CTX;

extern void WavpackMD5_Init(WavpackMD5_CTX *ctx);
extern void WavpackMD5_Update(WavpackMD5_CTX *ctx, const void *data, unsigned long size);
extern void WavpackMD5_Final(unsigned char *result, WavpackMD5_CTX *ctx);

#endif
