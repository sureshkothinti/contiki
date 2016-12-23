/**
 * \file base64.h
 *
 * \brief RFC 1521 base64 encoding/decoding
 *
 */
#ifndef BASE64_H
#define BASE64_H

#include <string.h>

#define ERR_BASE64_BUFFER_TOO_SMALL               -0x002A  /**< Output buffer too small. */
#define ERR_BASE64_INVALID_CHARACTER              -0x002C  /**< Invalid character in input. */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          Encode a buffer into base64 format
 *
 * \param dst      destination buffer
 * \param dlen     size of the buffer
 * \param src      source buffer
 * \param slen     amount of data to be encoded
 *
 * \return         0 if successful, or POLARSSL_ERR_BASE64_BUFFER_TOO_SMALL.
 *                 *dlen is always updated to reflect the amount
 *                 of data that has (or would have) been written.
 *
 * \note           Call this function with *dlen = 0 to obtain the
 *                 required buffer size in *dlen
 */
int base64_encode( unsigned char *dst, size_t *dlen,
                   const unsigned char *src, size_t slen );

/**
 * \brief          Decode a base64-formatted buffer
 *
 * \param dst      destination buffer (can be NULL for checking size)
 * \param dlen     size of the buffer
 * \param src      source buffer
 * \param slen     amount of data to be decoded
 *
 * \return         0 if successful, POLARSSL_ERR_BASE64_BUFFER_TOO_SMALL, or
 *                 POLARSSL_ERR_BASE64_INVALID_CHARACTER if the input data is
 *                 not correct. *dlen is always updated to reflect the amount
 *                 of data that has (or would have) been written.
 *
 * \note           Call this function with *dst = NULL or *dlen = 0 to obtain
 *                 the required buffer size in *dlen
 */
int base64_decode( unsigned char *dst, size_t *dlen,
                   const unsigned char *src, size_t slen );


#ifdef __cplusplus
}
#endif

#endif /* base64.h */
