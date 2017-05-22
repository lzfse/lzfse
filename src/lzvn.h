/*
Copyright (c) 2015-2016, Apple Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:  

1.  Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2.  Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.

3.  Neither the name of the copyright holder(s) nor the names of any contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LZVN_H
#define LZVN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#define LZVN_EXTERN extern "C"
#else
#define LZVN_EXTERN extern
#endif
#define LZVN_LIB_API __attribute__((visibility("default")))

/*! @abstract Get the required scratch buffer size to compress using LZVN.   */
LZVN_EXTERN size_t lzvn_encode_scratch_size(void) LZVN_LIB_API;

/*! @abstract Compress a buffer using LZVN.
 *
 *  @param dst_buffer
 *  Pointer to the first byte of the destination buffer.
 *
 *  @param dst_size
 *  Size of the destination buffer in bytes.
 *
 *  @param src_buffer
 *  Pointer to the first byte of the source buffer.
 *
 *  @param src_size
 *  Size of the source buffer in bytes.
 *
 *  @param scratch_buffer
 *  If non-NULL, a pointer to scratch space for the routine to use as workspace;
 *  the routine may use up to lzvn_encode_scratch_size( ) bytes of workspace
 *  during its operation, and will not perform any internal allocations. If
 *  NULL, the routine may allocate its own memory to use during operation via
 *  a single call to malloc( ), and will release it by calling free( ) prior
 *  to returning. For most use, passing NULL is perfectly satisfactory, but if
 *  you require strict control over allocation, you will want to pass an
 *  explicit scratch buffer.
 *
 *  @return
 *  The number of bytes written to the destination buffer if the input is
 *  successfully compressed. If the input cannot be compressed to fit into
 *  the provided buffer, or an error occurs, zero is returned, and the
 *  contents of dst_buffer are unspecified.                                   */
LZVN_EXTERN size_t lzvn_encode_buffer(uint8_t *__restrict dst_buffer,
                                      size_t dst_size,
                                      const uint8_t *__restrict src_buffer,
                                      size_t src_size,
                                      void *__restrict scratch_buffer) LZVN_LIB_API;

/*! @abstract Get the required scratch buffer size to decompress using LZVN. */
LZVN_EXTERN size_t lzvn_decode_scratch_size(void) LZVN_LIB_API;

/*! @abstract Decompress a buffer using LZVN.
 *
 *  @param dst_buffer
 *  Pointer to the first byte of the destination buffer.
 *
 *  @param dst_size
 *  Size of the destination buffer in bytes.
 *
 *  @param src_buffer
 *  Pointer to the first byte of the source buffer.
 *
 *  @param src_size
 *  Size of the source buffer in bytes.
 *
 *  @param scratch_buffer
 *  If non-NULL, a pointer to scratch space for the routine to use as workspace;
 *  the routine may use up to lzvn_decode_scratch_size( ) bytes of workspace
 *  during its operation, and will not perform any internal allocations. If
 *  NULL, the routine may allocate its own memory to use during operation via
 *  a single call to malloc( ), and will release it by calling free( ) prior
 *  to returning. For most use, passing NULL is perfectly satisfactory, but if
 *  you require strict control over allocation, you will want to pass an
 *  explicit scratch buffer.
 *
 *  @return
 *  The number of bytes written to the destination buffer if the input is
 *  successfully decompressed. If there is not enough space in the destination
 *  buffer to hold the entire expanded output, only the first dst_size bytes
 *  will be written to the buffer and dst_size is returned. Note that this
 *  behavior differs from that of lzvn_encode_buffer.                        */
LZVN_EXTERN size_t lzvn_decode_buffer(uint8_t *__restrict dst_buffer,
                                      size_t dst_size,
                                      const uint8_t *__restrict src_buffer,
                                      size_t src_size,
                                      void *__restrict scratch_buffer) LZVN_LIB_API;

#endif // LZVN_H
