#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "matrix.pio.h"
#include "bsp/board.h"
#include "tusb.h"


// keyboard wiring matrix
const uint8_t midi_matrix[64] = {
    96, 95, 94, 93, 92, 91, 90, 89,
    88, 87, 86, 85, 84, 83, 82, 81,
    80, 79, 78, 77, 76, 75, 74, 73,
    72, 71, 70, 69, 68, 67, 66, 65,
    64, 63, 62, 61, 60, 59, 58, 57,
    56, 55, 54, 53, 52, 51, 50, 49,
    48, 47, 46, 45, 44, 43, 42, 41,
    40, 39, 38, 37, 36,  0,  0,  0
};

// row masks to set the corresponding row low to scan it
const uint8_t row_masks[8] __attribute__((aligned(8))) = {
    0b11111110, // Row 0 active
    0b11111101, // Row 1 active
    0b11111011, // Row 2 active
    0b11110111, // Row 3 active
    0b11101111, // Row 4 active
    0b11011111, // Row 5 active
    0b10111111, // Row 6 active
    0b01111111  // Row 7 active
};

uint8_t scanned_state[8] __attribute__((aligned(8))) = {0};

void setup_pio(PIO pio, uint sm) {
    // load the assembly into the pio memory
    uint offset = pio_add_program(pio, &matrix_program);
    pio_sm_config c = matrix_program_get_default_config(offset);

    // init output pins and set them high
    sm_config_set_out_pins(&c, 0, 8);
    for (int i = 0; i < 8; i++) pio_gpio_init(pio, i);
    pio_sm_set_consecutive_pindirs(pio, sm, 0, 8, true);

    // init input pins with pull up resistors
    sm_config_set_in_pins(&c, 8);
    for (int i = 8; i < 16; i++) {
        pio_gpio_init(pio, i);
        gpio_pull_up(i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, 8, 8, false);

    // setup shift registers to pull/push 8 bits at a time
    sm_config_set_out_shift(&c, false, false, 8);
    sm_config_set_in_shift(&c, false, false, 8);

    // slow the clock to 1 MHz (from 133 MHz)
    sm_config_set_clkdiv(&c, 133.0f);

    // init and start the state machine
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void setup_dma(PIO pio, uint sm) {
    // get DMA channels for transmit (row masks to pio) and receive (scanned_state data)
    int dma_tx = dma_claim_unused_channel(true);
    int dma_rx = dma_claim_unused_channel(true);

    // Tx - feed row masks to PIO
    dma_channel_config c_tx = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_8); // moving 8 bits at a time
    channel_config_set_read_increment(&c_tx, true);           // move down the row_masks array
    channel_config_set_write_increment(&c_tx, false);         // always write to the same PIO slot

    // The Ring Buffer: Wrap the READ pointer every 8 bytes (2^3 = 8)
    channel_config_set_ring(&c_tx, false, 3);

    // DREQ (Data Request): Only send data when the PIO specifically asks for it
    channel_config_set_dreq(&c_tx, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        dma_tx, &c_tx,
        &pio->txf[sm], // destination: PIO Transmit FIFO
        row_masks,     // source: Our row_masks array
        0xFFFFFFFF,    // transfer count: Maximum possible number (run forever)
        true           // start immediately
    );

    // Rx - Get each scanned column from PIO
    dma_channel_config c_rx = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_8); // moving 8 bits at a time
    channel_config_set_read_increment(&c_rx, false);          // always read from the same PIO slot
    channel_config_set_write_increment(&c_rx, true);          // increments down scanned_state

    // The Ring Buffer: Wrap the WRITE pointer every 8 bytes (2^3 = 8)
    channel_config_set_ring(&c_rx, true, 3); 
    
    // DREQ (Data Request): Only grab data when the PIO says it has pushed a new result
    channel_config_set_dreq(&c_rx, pio_get_dreq(pio, sm, false));

    dma_channel_configure(
        dma_rx, &c_rx,
        scanned_state, // destination: Our C++ landing pad array
        &pio->rxf[sm], // source: PIO Receive FIFO
        0xFFFFFFFF,    // transfer count: Maximum possible number (run forever)
        true           // start immediately
    );
}


int main() {
    // init board and USB stack
    board_init();
    tusb_init();
    stdio_init_all();

    // turn on pio and dma
    setup_pio(pio0, 0);
    setup_dma(pio0, 0);

    uint64_t previous_keys = 0;

    while (true) {
        // let TinyUSB handle background traffic
        tud_task();

        // read the 8 byte scanned_state array as a 64 bit int each bit representing a key
        // and also NOT it so that 1 is pressed and 0 is released
        uint64_t current_keys = ~(*((volatile uint64_t*)scanned_state));

        // use a bitwise XOR to find all the bits that have changed between states
        uint64_t changed_keys = current_keys ^ previous_keys;

        while (changed_keys != 0) {
            // finds the index of the lowest set bit (changed key) in a single clock cycle
            int i = __builtin_ctzll(changed_keys);
            uint8_t note = midi_matrix[i];

            if (note != 0) {
                // check if the key was pressed or released
                if ((current_keys >> i) & 1ULL) {
                    uint8_t msg[3] = { 0x90, note, 127 };
                    tud_midi_stream_write(0, msg, 3);
                } else {
                    uint8_t msg[3] = { 0x80, note, 0 };
                    tud_midi_stream_write(0, msg, 3);
                }
            }
            // trick to clear the lowest set bit to process the next
            // when you -1 from changed_keys the lowest set bit will
            // always flip as it is borrowed from in the subtraction
            // then bitwise AND will set any flipped bit to 0
            changed_keys &= (changed_keys - 1);
        }
        previous_keys = current_keys;
    }
    return 0;
}
