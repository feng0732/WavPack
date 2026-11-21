////////////////////////////////////////////////////////////////////////////
//                           **** WAVPACK ****                            //
//                  Hybrid Lossless Wavefile Compressor                   //
//                Copyright (c) 1998 - 2024 David Bryant.                 //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// raw_checksum.c

// This module implements the raw input checksum feature (OPEN_RAW_CHECKSUM).
// When enabled, the library interposes a pass-through reader between itself
// and the application's reader callbacks. The interposed reader accumulates
// an MD5 checksum of the raw bytes of the input file(s) exactly as they are
// stored on disk -- every byte consumed from the input stream, including
// headers, metadata, wrapper bytes and skipped garbage, whether or not those
// bytes are ever used for decoding. This is fundamentally different from the
// regular MD5 checksum (WavpackGetMD5Sum()) which covers the decoded PCM
// samples, and it is independent of the selected decode mode (integer PCM,
// floating point, or DSD converted to PCM) because it operates below the
// sample decoding layer entirely.
//
// The checksum is computed over the contiguous prefix of the file: bytes are
// incorporated into the running MD5 strictly in file order. Bytes that are
// re-read (after a backward seek or a push_back_byte()) are not counted
// twice, while a forward seek or skip over unread data makes it impossible
// for the checksum to ever represent the entire stored file, and that
// condition (along with the file position where continuity was lost) is
// latched so it can be reported to the application. A short read is taken as
// end-of-file (the rest of the library makes the same assumption), and until
// EOF is observed the checksum is reported as "in progress" rather than
// final, which is important for streaming decoders that have not yet read
// the entire input.

#include <stdlib.h>
#include <string.h>

#include "wavpack_local.h"

///////////////////////////// reader interposition /////////////////////////////

// Read bytes from the underlying stream and update the raw checksum state.
// Only the bytes actually returned are checksummed (which correctly handles
// short reads), and the position tracking handles sequential reads, re-reads
// of previously consumed data, and gaps caused by seeks or skips.

static int32_t raw_checksum_read_bytes (void *id, void *data, int32_t bcount)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    RawChecksumState *rcstate = rcs->parent;
    int32_t bytes_read = rcstate->real_reader->read_bytes (rcs->real_id, data, bcount);

    if (bytes_read < 0) {           // read error: the checksum can no longer be trusted
        if (!rcs->discontinuous) {
            rcs->discontinuous = TRUE;
            rcs->error_pos = rcs->next_pos;
        }

        rcs->cur_pos = -1;
        return bytes_read;
    }

    if (bytes_read > 0) {
        int64_t start_pos = rcs->cur_pos;

        if (start_pos >= 0) {
            if (!rcs->discontinuous) {
                if (start_pos > rcs->next_pos) {
                    // a gap: bytes were skipped that can never be inserted into
                    // the running checksum, so latch the discontinuity and the
                    // position where the contiguous coverage ended

                    rcs->discontinuous = TRUE;
                    rcs->error_pos = rcs->next_pos;
                }
                else if (start_pos == rcs->next_pos) {
                    // the normal sequential case

                    WavpackMD5_Update (&rcs->md5, data, bytes_read);
                    rcs->next_pos += bytes_read;
                }
                else if (start_pos + bytes_read > rcs->next_pos) {
                    // a partial re-read (e.g., after a push_back_byte() or a
                    // backward seek): only the previously unseen tail counts

                    uint32_t overlap = (uint32_t) (rcs->next_pos - start_pos);

                    WavpackMD5_Update (&rcs->md5, (unsigned char *) data + overlap, bytes_read - overlap);
                    rcs->next_pos += bytes_read - overlap;
                }

                // else the read lies entirely within already-checksummed data
                // (a pure re-read) and does not affect the checksum
            }

            rcs->cur_pos = start_pos + bytes_read;
        }
        else if (!rcs->discontinuous) {
            // The tracked position was lost (only possible after a seek
            // relative to the end of a stream of unknown length), so the
            // best we can do is assume the data arrives sequentially.

            WavpackMD5_Update (&rcs->md5, data, bytes_read);
            rcs->next_pos += bytes_read;
        }
    }

    if (bytes_read < bcount)
        rcs->eof = TRUE;            // a short read indicates end-of-file

    return bytes_read;
}

static int32_t raw_checksum_write_bytes (void *id, void *data, int32_t bcount)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    return rcs->parent->real_reader->write_bytes (rcs->real_id, data, bcount);
}

static int64_t raw_checksum_get_pos (void *id)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    return rcs->parent->real_reader->get_pos (rcs->real_id);
}

static int raw_checksum_set_pos_abs (void *id, int64_t pos)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    int result = rcs->parent->real_reader->set_pos_abs (rcs->real_id, pos);

    if (!result)
        rcs->cur_pos = pos;

    return result;
}

static int raw_checksum_set_pos_rel (void *id, int64_t delta, int mode)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    RawChecksumState *rcstate = rcs->parent;
    int result = rcstate->real_reader->set_pos_rel (rcs->real_id, delta, mode);

    if (!result) {
        if (mode == SEEK_SET)
            rcs->cur_pos = delta;
        else if (mode == SEEK_CUR) {
            if (rcs->cur_pos >= 0)
                rcs->cur_pos += delta;
        }
        else if (mode == SEEK_END) {
            int64_t length = rcstate->real_reader->get_length ?
                rcstate->real_reader->get_length (rcs->real_id) : -1;

            rcs->cur_pos = (length > 0) ? length + delta : -1;
        }
    }

    return result;
}

static int raw_checksum_push_back_byte (void *id, int c)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    int result = rcs->parent->real_reader->push_back_byte (rcs->real_id, c);

    // the pushed-back byte will be re-read, so back up our tracked position;
    // the re-read logic in raw_checksum_read_bytes() will not count it twice

    if (result != EOF && rcs->cur_pos > 0)
        rcs->cur_pos--;

    return result;
}

static int64_t raw_checksum_get_length (void *id)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    return rcs->parent->real_reader->get_length (rcs->real_id);
}

static int raw_checksum_can_seek (void *id)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    return rcs->parent->real_reader->can_seek (rcs->real_id);
}

static int raw_checksum_truncate_here (void *id)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    return rcs->parent->real_reader->truncate_here (rcs->real_id);
}

static int raw_checksum_close (void *id)
{
    RawChecksumStream *rcs = (RawChecksumStream *) id;
    return rcs->parent->real_reader->close (rcs->real_id);
}

// Called from WavpackOpenFileInputEx64() when OPEN_RAW_CHECKSUM is specified,
// after wpc->reader, wpc->wv_in and wpc->wvc_in have been set. Replaces them
// with the interposed checksumming reader so that every byte the library
// consumes (from either stream) is accounted for. Returns FALSE on memory
// allocation failure.

int raw_checksum_init (WavpackContext *wpc)
{
    RawChecksumState *rcstate = (RawChecksumState *) calloc (1, sizeof (RawChecksumState));

    if (!rcstate)
        return FALSE;

    rcstate->real_reader = wpc->reader;

    rcstate->wrapper.read_bytes = raw_checksum_read_bytes;
    rcstate->wrapper.get_pos = raw_checksum_get_pos;
    rcstate->wrapper.set_pos_abs = raw_checksum_set_pos_abs;
    rcstate->wrapper.set_pos_rel = raw_checksum_set_pos_rel;
    rcstate->wrapper.push_back_byte = raw_checksum_push_back_byte;
    rcstate->wrapper.get_length = raw_checksum_get_length;
    rcstate->wrapper.can_seek = raw_checksum_can_seek;

    // these callbacks are optional in the reader, so only interpose when present

    if (rcstate->real_reader->write_bytes)
        rcstate->wrapper.write_bytes = raw_checksum_write_bytes;

    if (rcstate->real_reader->truncate_here)
        rcstate->wrapper.truncate_here = raw_checksum_truncate_here;

    if (rcstate->real_reader->close)
        rcstate->wrapper.close = raw_checksum_close;

    // initialize the per-stream state for the main stream (and the correction
    // stream, if one was provided); the tracked position starts wherever the
    // stream is currently positioned (normally zero) and is maintained by us
    // from here on (so it works even for streams that don't report positions)

    rcstate->wv.real_id = wpc->wv_in;
    rcstate->wv.parent = rcstate;
    rcstate->wv.cur_pos = rcstate->real_reader->get_pos (wpc->wv_in);
    rcstate->wv.error_pos = -1;
    WavpackMD5_Init (&rcstate->wv.md5);

    if (rcstate->wv.cur_pos < 0)
        rcstate->wv.cur_pos = 0;

    rcstate->wv.next_pos = rcstate->wv.cur_pos;

    if (wpc->wvc_in) {
        rcstate->has_wvc = TRUE;
        rcstate->wvc.real_id = wpc->wvc_in;
        rcstate->wvc.parent = rcstate;
        rcstate->wvc.cur_pos = rcstate->real_reader->get_pos (wpc->wvc_in);
        rcstate->wvc.error_pos = -1;
        WavpackMD5_Init (&rcstate->wvc.md5);

        if (rcstate->wvc.cur_pos < 0)
            rcstate->wvc.cur_pos = 0;

        rcstate->wvc.next_pos = rcstate->wvc.cur_pos;
    }

    wpc->reader = &rcstate->wrapper;
    wpc->wv_in = &rcstate->wv;
    wpc->wvc_in = rcstate->has_wvc ? &rcstate->wvc : NULL;
    wpc->raw_checksum = rcstate;

    return TRUE;
}

// Called from WavpackCloseFile() to release the raw checksum state. This runs
// after the underlying streams have been closed (the close callbacks are
// forwarded through the interposed reader above).

void raw_checksum_free (WavpackContext *wpc)
{
    if (wpc->raw_checksum) {
        free (wpc->raw_checksum);
        wpc->raw_checksum = NULL;
    }
}

/////////////////////////////// public query API ///////////////////////////////

// Common status/result logic for the two query functions below. The checksum
// is only copied to the caller's buffer when the entire stream has been
// consumed contiguously through end-of-file (RAW_CHECKSUM_COMPLETE).

static int raw_checksum_get (RawChecksumState *rcstate, RawChecksumStream *rcs, unsigned char data [16])
{
    if (rcs->discontinuous)
        return RAW_CHECKSUM_DISCONTINUOUS;

    // the checksum is final once we have seen EOF, or once the contiguous
    // coverage extends to the (known) end of the stream

    if (!rcs->eof) {
        int64_t length = rcstate->real_reader->get_length ?
            rcstate->real_reader->get_length (rcs->real_id) : 0;

        if (length <= 0 || rcs->next_pos < length)
            return RAW_CHECKSUM_IN_PROGRESS;
    }

    if (data) {
        WavpackMD5_CTX final_ctx = rcs->md5;    // finalize a copy so the state survives
        WavpackMD5_Final (data, &final_ctx);
    }

    return RAW_CHECKSUM_COMPLETE;
}

// Query the raw checksum of the main WavPack input stream. Returns one of the
// RAW_CHECKSUM_* status codes defined in wavpack.h; the 16-byte MD5 checksum
// of the raw file contents is stored at "data" only when the return value is
// RAW_CHECKSUM_COMPLETE.

int WavpackGetRawChecksum (WavpackContext *wpc, unsigned char data [16])
{
    if (!wpc || !wpc->raw_checksum)
        return RAW_CHECKSUM_NOT_ENABLED;

    return raw_checksum_get (wpc->raw_checksum, &wpc->raw_checksum->wv, data);
}

// Identical to WavpackGetRawChecksum(), but applies to the "correction" (.wvc)
// stream. Returns RAW_CHECKSUM_NOT_ENABLED if no correction stream was supplied
// when the file was opened.

int WavpackGetRawChecksum2 (WavpackContext *wpc, unsigned char data [16])
{
    if (!wpc || !wpc->raw_checksum || !wpc->raw_checksum->has_wvc)
        return RAW_CHECKSUM_NOT_ENABLED;

    return raw_checksum_get (wpc->raw_checksum, &wpc->raw_checksum->wvc, data);
}

// If either raw checksum stream has lost continuity (a seek or skip over
// unread data, or a read error), return the file offset in that stream where
// the contiguous coverage ended, which is where the discrepancy begins.
// Returns -1 if the feature is not enabled or no discontinuity was detected.
// This state is latched and remains available until the file is closed.

int64_t WavpackGetRawChecksumErrorPos (WavpackContext *wpc)
{
    if (!wpc || !wpc->raw_checksum)
        return -1;

    if (wpc->raw_checksum->wv.discontinuous)
        return wpc->raw_checksum->wv.error_pos;

    if (wpc->raw_checksum->has_wvc && wpc->raw_checksum->wvc.discontinuous)
        return wpc->raw_checksum->wvc.error_pos;

    return -1;
}
