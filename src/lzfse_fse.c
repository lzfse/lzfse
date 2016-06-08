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

#include "lzfse_internal.h"

// Compare 64-bit unsigned integers for qsort.
static int compare_counts(const void *a, const void *b) {
  uint64_t aa = load8(a);
  uint64_t bb = load8(b);
  if (aa < bb)
    return -1;
  else if (aa > bb)
    return 1;
  return 0;
}

// Initialize encoder table T[NSYMBOLS].
// NSTATES = sum FREQ[i] is the number of states (a power of 2)
// NSYMBOLS is the number of symbols.
// FREQ[NSYMBOLS] is a normalized histogram of symbol frequencies, with FREQ[i]
// >= 0.
// Some symbols may have a 0 frequency.  In that case, they should not be
// present in the data.
void fse_init_encoder_table(int nstates, int nsymbols,
                            const uint16_t *__restrict freq,
                            fse_encoder_entry *__restrict t) {
  int offset = 0; // current offset
  int n_clz = __builtin_clz(nstates);
  for (int i = 0; i < nsymbols; i++) {
    int f = (int)freq[i];
    if (f == 0)
      continue; // skip this symbol, no occurrences
    int k =
        __builtin_clz(f) - n_clz; // shift needed to ensure N <= (F<<K) < 2*N
    t[i].s0 = (int16_t)((f << k) - nstates);
    t[i].k = (int16_t)k;
    t[i].delta0 = (int16_t)(offset - f + (nstates >> k));
    t[i].delta1 = (int16_t)(offset - f + (nstates >> (k - 1)));
    offset += f;
  }
}

// Initialize decoder table T[NSTATES].
// NSTATES = sum FREQ[i] is the number of states (a power of 2)
// NSYMBOLS is the number of symbols.
// FREQ[NSYMBOLS] is a normalized histogram of symbol frequencies, with FREQ[i]
// >= 0.
// Some symbols may have a 0 frequency.  In that case, they should not be
// present in the data.
int fse_init_decoder_table(int nstates, int nsymbols,
                           const uint16_t *__restrict freq,
                           int32_t *__restrict t) {
  assert(nsymbols <= 256);
  assert(fse_check_freq(freq, nsymbols, nstates) == 0);
  int n_clz = __builtin_clz(nstates);
  int sum_of_freq = 0;
  for (int i = 0; i < nsymbols; i++) {
    int f = (int)freq[i];
    if (f == 0)
      continue; // skip this symbol, no occurrences

    sum_of_freq += f;

    if (sum_of_freq > nstates) {
      return -1;
    }

    int k =
        __builtin_clz(f) - n_clz; // shift needed to ensure N <= (F<<K) < 2*N
    int j0 = ((2 * nstates) >> k) - f;

    // Initialize all states S reached by this symbol: OFFSET <= S < OFFSET + F
    for (int j = 0; j < f; j++) {
      fse_decoder_entry e;

      e.symbol = (uint8_t)i;
      if (j < j0) {
        e.k = (int8_t)k;
        e.delta = (int16_t)(((f + j) << k) - nstates);
      } else {
        e.k = (int8_t)(k - 1);
        e.delta = (int16_t)((j - j0) << (k - 1));
      }

      memcpy(t, &e, sizeof(e));
      t++;
    }
  }

  return 0; // OK
}

// Initialize value decoder table T[NSTATES].
// NSTATES = sum FREQ[i] is the number of states (a power of 2)
// NSYMBOLS is the number of symbols.
// FREQ[NSYMBOLS] is a normalized histogram of symbol frequencies, with FREQ[i]
// >= 0.
// SYMBOL_VBITS[NSYMBOLS] and SYMBOLS_VBASE[NSYMBOLS] are the number of value
// bits to read and the base value for each symbol.
// Some symbols may have a 0 frequency.  In that case, they should not be
// present in the data.
void fse_init_value_decoder_table(int nstates, int nsymbols,
                                  const uint16_t *__restrict freq,
                                  const uint8_t *__restrict symbol_vbits,
                                  const int32_t *__restrict symbol_vbase,
                                  fse_value_decoder_entry *__restrict t) {
  assert(nsymbols <= 256);
  assert(fse_check_freq(freq, nsymbols, nstates) == 0);

  int n_clz = __builtin_clz(nstates);
  for (int i = 0; i < nsymbols; i++) {
    int f = (int)freq[i];
    if (f == 0)
      continue; // skip this symbol, no occurrences

    int k =
        __builtin_clz(f) - n_clz; // shift needed to ensure N <= (F<<K) < 2*N
    int j0 = ((2 * nstates) >> k) - f;

    fse_value_decoder_entry ei = {0};
    ei.value_bits = symbol_vbits[i];
    ei.vbase = symbol_vbase[i];

    // Initialize all states S reached by this symbol: OFFSET <= S < OFFSET + F
    for (int j = 0; j < f; j++) {
      fse_value_decoder_entry e = ei;

      if (j < j0) {
        e.total_bits = (uint8_t)k + e.value_bits;
        e.delta = (int16_t)(((f + j) << k) - nstates);
      } else {
        e.total_bits = (uint8_t)(k - 1) + e.value_bits;
        e.delta = (int16_t)((j - j0) << (k - 1));
      }

      memcpy(t, &e, 8);
      t++;
    }
  }
}

// Normalize a table T[NSYMBOLS] of symbols,occurrences to FREQ[NSYMBOLS].
// IMPORTANT: T will be modified (sorted) by this call.
// Return 1 if OK, and 0 on failure.
int fse_normalize_freq(int nstates, int nsymbols, fse_occurrence_entry *t,
                       uint16_t *freq) {
  // Sort T in increasing count order. Entry seen as a 64-bit value is (count <<
  // 32) + symbol_id
  qsort(t, nsymbols, sizeof(t[0]), compare_counts);

  // Get sum of T.count
  uint32_t s_count = 0;
  for (int i = 0; i < nsymbols; i++)
    s_count += t[i].count;

  // Start from low values
  uint32_t available = (uint32_t)nstates; // remaining available states
  for (int i = 0; i < nsymbols; i++) {
    uint32_t symbol = t[i].symbol;
    uint32_t count = t[i].count;

    if (count == 0) {
      freq[symbol] = 0;
      continue;
    } // no states used
    if (available <= 0 || s_count <= 0)
      return 0; // failed
    uint32_t k =
        (uint32_t)((double)count * (double)available / (double)s_count);
    if (k == 0)
      k = 1; // if count is not 0, we need at least 1 state to represent it
    if (i == nsymbols - 1)
      k = available; // force usage of all states
    freq[symbol] = (uint16_t)k;
    s_count -= count;
    available -= k;
  }
  return 1; // OK
}
