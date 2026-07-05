// Hardware entropy / random number generation for SiliCa
// JIS X 6319-4 compatible card implementation

#pragma once
#include <stddef.h>
#include <stdint.h>

// Generate random bytes using hardware entropy (ADC noise + LFSR).
// Fills the provided buffer with random data.
void generate_random_bytes(uint8_t *buf, size_t len);
