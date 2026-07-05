// Hardware entropy / random number generation for SiliCa
// JIS X 6319-4 compatible card implementation
//
// Entropy is gathered from the on-chip ADC (temperature sensor noise) and
// mixed with a Galois LFSR, seeded from uninitialized SRAM at power-up.

#include "entropy.h"
#include <avr/io.h>
#include <stdint.h>

// LFSR state for entropy mixing
static uint16_t lfsr_state = 0xACE1;

// LFSR step function (Galois LFSR with taps at 16, 14, 13, 11)
static uint16_t lfsr_next() {
  uint16_t bit = ((lfsr_state >> 0) ^ (lfsr_state >> 2) ^ (lfsr_state >> 3) ^
                  (lfsr_state >> 5)) &
                 1;
  lfsr_state = (lfsr_state >> 1) | (bit << 15);
  return lfsr_state;
}

// Initialize ADC for entropy collection
static void init_adc_entropy() {
  ADC0.CTRLA = 0;                   // Disable ADC first
  ADC0.CTRLB = ADC_SAMPNUM_ACC1_gc; // No accumulation
  ADC0.CTRLC = ADC_PRESC_DIV2_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm;
  ADC0.CTRLD = ADC_INITDLY_DLY16_gc;
  ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc; // Temperature sensor
  ADC0.CTRLA = ADC_ENABLE_bm;            // Enable ADC
}

// Read single ADC value
static uint16_t read_adc_raw() {
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {
  }
  return ADC0.RES;
}

// Seed LFSR from uninitialized SRAM and ADC
static void seed_entropy() {
  // Read memory locations that may have random values after power-up
  volatile uint8_t *sram = (volatile uint8_t *)0x3F00; // Upper SRAM area

  for (int i = 0; i < 16; i++) {
    lfsr_state ^= sram[i];
    lfsr_state ^= (uint16_t)sram[i + 16] << 8;
    lfsr_next();
  }

  // Additional mixing from ADC
  for (int i = 0; i < 8; i++) {
    lfsr_state ^= read_adc_raw();
    lfsr_next();
  }
}

// Generate one entropy byte using ADC noise and LFSR
static uint8_t generate_entropy_byte() {
  uint8_t entropy = 0;

  for (int i = 0; i < 8; i++) {
    // Read ADC multiple times and XOR together for more noise
    uint16_t val1 = read_adc_raw();
    uint16_t val2 = read_adc_raw();
    uint16_t val3 = read_adc_raw();
    uint16_t val4 = read_adc_raw();

    // XOR the differences between consecutive readings
    uint16_t diff1 = val2 - val1;
    uint16_t diff2 = val4 - val3;

    // Mix into LFSR state
    lfsr_state ^= (diff1 ^ diff2);

    // Use XOR of multiple LSBs for one entropy bit
    uint8_t bit = ((val1 ^ val2 ^ val3 ^ val4) & 0x01);
    bit ^= ((diff1 ^ diff2) >> 1) & 0x01;

    entropy = (entropy << 1) | bit;
  }

  // Final mix with LFSR output
  entropy ^= (uint8_t)lfsr_next();
  entropy ^= (uint8_t)(lfsr_next() >> 8);

  return entropy;
}

// Generate random bytes using hardware entropy
void generate_random_bytes(uint8_t *buf, size_t len) {
  // Initialize ADC
  init_adc_entropy();

  // Seed the LFSR
  seed_entropy();

  // Generate each byte
  for (size_t i = 0; i < len; i++) {
    buf[i] = generate_entropy_byte();
  }

  // Disable ADC
  ADC0.CTRLA = 0;
}
