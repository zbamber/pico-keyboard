#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "matrix.pio.h"
#include "bsp/board.h"
#include "tusb.h"
#include "pico/time.h"


enum PanelButton {
    BTN_NONE,             // 0
    BTN_INTRO_ENDING,     // 1
    BTN_START_STOP,       // 2
    BTN_MAIN_VOL_UP,      // 3
    BTN_MAIN_VOL_DOWN,    // 4
    BTN_TEMPO_UP,         // 5
    BTN_TEMPO_DOWN,       // 6
    BTN_ACCOMP_VOL_UP,    // 7
    BTN_ACCOMP_VOL_DOWN,  // 8
    BTN_TRANSPOSE_UP,     // 9
    BTN_TRANSPOSE_DOWN,   // 10
    BTN_SUSTAIN,          // 11
    BTN_VIBRATO,          // 12
    BTN_SYNC,             // 13
    BTN_SINGLE,           // 14
    BTN_FINGERED,         // 15
    BTN_FILL,             // 16
    BTN_METRONOME,        // 17
    BTN_SPLIT,            // 18
    BTN_REC,              // 19
    BTN_PROGRAM,          // 20
    BTN_PLAYBACK,         // 21
    BTN_MEMORY,           // 22
    BTN_M1,               // 23
    BTN_M2,               // 24
    BTN_PERCUSSION,       // 25
    BTN_PLAY_PAUSE,       // 26
    BTN_SKIP_BACK,        // 27
    BTN_SKIP,             // 28
    BTN_VOL_DOWN,         // 29
    BTN_VOL_UP,           // 30
    BTN_TEACH1,           // 31
    BTN_TEACH2,           // 32
    BTN_TONE,             // 33
    BTN_RYTHM,            // 34
    BTN_DEMO,             // 35
    BTN_7,                // 36
    BTN_8,                // 37
    BTN_9,                // 38
    BTN_MINUS,            // 39
    BTN_4,                // 40
    BTN_5,                // 41
    BTN_6,                // 42
    BTN_PLUS,             // 43
    BTN_1,                // 44
    BTN_2,                // 45
    BTN_3,                // 46
    BTN_0                 // 47
};

struct ButtonMap {
    PanelButton btn;
    uint16_t min_val;
    uint16_t max_val;
};

class ButtonQueue {
    private:
        static constexpr int SIZE = 3; // max 3 btns pressed at once
        
        PanelButton buffer[SIZE];
        int head = 0;
        int tail = 0;
        int count = 0;

    public:
        bool push(PanelButton btn) {
            if (count == SIZE) {
                return false; // full
            }

            buffer[tail] = btn;
            tail = (tail + 1) % SIZE;
            count++;

            return true;
        }

        bool pop(PanelButton& btn) {
            if (count == 0) {
                return false; // empty
            }

            btn = buffer[head];
            head = (head + 1) % SIZE;
            count--;

            return true;
        }

        int size() const {
            return count;
        }
};

const ButtonMap ladder_map[] = {
    {BTN_PLAY_PAUSE, 3995, 4195},
    {BTN_SKIP_BACK, 2130, 2330},
    {BTN_SKIP, 2880, 3080},
    {BTN_VOL_DOWN, 3360, 3560},
    {BTN_VOL_UP, 3740, 3940}
};

// keyboard wiring matrix
// // transpose and reverse
// const uint8_t midi_matrix[64] = {
//     96, 95, 94, 93, 92, 91, 90, 89,
//     88, 87, 86, 85, 84, 83, 82, 81,
//     80, 79, 78, 77, 76, 75, 74, 73,
//     72, 71, 70, 69, 68, 67, 66, 65,
//     64, 63, 62, 61, 60, 59, 58, 57,
//     56, 55, 54, 53, 52, 51, 50, 49,
//     48, 47, 46, 45, 44, 43, 42, 41,
//     40, 39, 38, 37, 36,  0,  0,  0
// };

const uint8_t midi_matrix[64] = {
    36, 37, 38, 39, 40, 41, 42, 43,
    44, 45, 46, 47, 48, 49, 50, 51,
    52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67,
    68, 69, 70, 71, 72, 73, 74, 75,
    76, 77, 78, 79, 80, 81, 82, 83,
    84, 85, 86, 87, 88, 89, 90, 91,
    92, 93, 94, 95, 96,  0,  0,  0
};

// // transpose
// const PanelButton btn_matrix[64] = {
//     // Row 8
//     BTN_7, BTN_6, BTN_5, BTN_4, BTN_3, BTN_2, BTN_1, BTN_0,
//     // Row 9
//     BTN_ACCOMP_VOL_DOWN, BTN_START_STOP, BTN_TRANSPOSE_UP, BTN_MAIN_VOL_UP, BTN_MINUS, BTN_PLUS, BTN_9, BTN_8,
//     // Row 10
//     BTN_PLAYBACK, BTN_REC, BTN_PROGRAM, BTN_SYNC, BTN_FILL, BTN_FINGERED, BTN_SINGLE, BTN_NONE,
//     // Row 11
//     BTN_NONE, BTN_TEACH2, BTN_TEACH1, BTN_VIBRATO, BTN_TEMPO_DOWN, BTN_RYTHM, BTN_TONE, BTN_NONE,
//     // Row 12
//     BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_PERCUSSION,
//     // Row 13
//     BTN_NONE, BTN_NONE, BTN_SUSTAIN, BTN_TEMPO_UP, BTN_TRANSPOSE_DOWN, BTN_MAIN_VOL_DOWN, BTN_DEMO, BTN_NONE,
//     // Row 14
//     BTN_NONE, BTN_M2, BTN_M1, BTN_MEMORY, BTN_ACCOMP_VOL_UP, BTN_INTRO_ENDING, BTN_SPLIT, BTN_METRONOME,
//     // Row 15 (Empty row at the bottom)
//     BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE
// };

const PanelButton btn_matrix[64] = {
    // Physical Column 0 (Software Row 0)
    BTN_7, BTN_ACCOMP_VOL_DOWN, BTN_PLAYBACK, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE,
    // Physical Column 1 (Software Row 1)
    BTN_6, BTN_START_STOP, BTN_REC, BTN_TEACH2, BTN_NONE, BTN_NONE, BTN_M2, BTN_NONE,
    // Physical Column 2 (Software Row 2)
    BTN_5, BTN_TRANSPOSE_UP, BTN_PROGRAM, BTN_TEACH1, BTN_NONE, BTN_SUSTAIN, BTN_M1, BTN_NONE,
    // Physical Column 3 (Software Row 3)
    BTN_4, BTN_MAIN_VOL_UP, BTN_SYNC, BTN_VIBRATO, BTN_NONE, BTN_TEMPO_UP, BTN_MEMORY, BTN_NONE,
    // Physical Column 4 (Software Row 4)
    BTN_3, BTN_MINUS, BTN_FILL, BTN_TEMPO_DOWN, BTN_NONE, BTN_TRANSPOSE_DOWN, BTN_ACCOMP_VOL_UP, BTN_NONE,
    // Physical Column 5 (Software Row 5)
    BTN_2, BTN_PLUS, BTN_FINGERED, BTN_RYTHM, BTN_NONE, BTN_MAIN_VOL_DOWN, BTN_INTRO_ENDING, BTN_NONE,
    // Physical Column 6 (Software Row 6)
    BTN_1, BTN_9, BTN_SINGLE, BTN_TONE, BTN_NONE, BTN_DEMO, BTN_SPLIT, BTN_NONE,
    // Physical Column 7 (Software Row 7)
    BTN_0, BTN_8, BTN_NONE, BTN_NONE, BTN_PERCUSSION, BTN_NONE, BTN_METRONOME, BTN_NONE
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

uint16_t scanned_state[8] __attribute__((aligned(16))) = {0};

void setup_pio(PIO pio, uint sm) {
    // load the assembly into the pio memory
    uint offset = pio_add_program(pio, &matrix_program);
    pio_sm_config c = matrix_program_get_default_config(offset);

    // init output pins and set them high
    sm_config_set_out_pins(&c, 0, 8);
    for (int i = 0; i < 8; i++) pio_gpio_init(pio, i);
    pio_sm_set_consecutive_pindirs(pio, sm, 0, 8, true);

    // init input pins with pull up resistors (GP8 to GP22)
    // and also 23 or it will float low
    sm_config_set_in_pins(&c, 8);
    for (int i = 8; i < 23; i++) {
        pio_gpio_init(pio, i);
        gpio_pull_up(i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, 8, 15, false); // set the 15 pins as inputs

    // setup shift registers
    sm_config_set_out_shift(&c, false, false, 8); // outputting 8 bit row masks
    sm_config_set_in_shift(&c, false, false, 16); // inputting 16 bit btn states

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
    channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_16); // moving 16 bits at a time
    channel_config_set_read_increment(&c_rx, false);          // always read from the same PIO slot
    channel_config_set_write_increment(&c_rx, true);          // increments down scanned_state

    // The Ring Buffer: Wrap the WRITE pointer every 16 bytes (2^4 = 16)
    channel_config_set_ring(&c_rx, true, 4);
    
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

void setup_adc() {
    adc_init();
    adc_gpio_init(28);
    gpio_pull_down(28);
    adc_select_input(2);
}


PanelButton scan_ladder_btns() {
    uint32_t adc_sum = 0;
    const int NUM_SAMPLES = 8;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adc_sum += adc_read();
    }

    // average to smooth noise
    uint16_t adc_val = adc_sum / NUM_SAMPLES;

    if (adc_val < 100) return BTN_NONE;

    int num_btns = sizeof(ladder_map) / sizeof(ladder_map[0]);

    for (int i = 0; i < num_btns; i++) {
        if (adc_val >= ladder_map[i].min_val && adc_val <= ladder_map[i].max_val) {
            return ladder_map[i].btn;
        }
    }

    return BTN_NONE;
}


int main() {
    // init board and USB stack
    board_init();
    tusb_init();
    stdio_init_all();

    // setup pio, dma and adc
    setup_pio(pio0, 0);
    setup_dma(pio0, 0);
    setup_adc();

    uint64_t previous_keys = 0;
    uint64_t previous_btns = 0;

    ButtonQueue btn_queue;
    // PanelButton previous_ladder_btn = BTN_NONE;

    PanelButton stable_ladder_btn = BTN_NONE;    // correct state 
    PanelButton candidate_ladder_btn = BTN_NONE; // fluctuating state
    uint32_t candidate_start_time = 0;           // timestamp for debounce

    while (true) {
        // let TinyUSB handle background traffic
        tud_task();

        uint64_t current_keys = 0;
        uint64_t current_btns = 0;

        for (int row = 0; row < 8; row++) {
            // invert the 16 bit row so 1 is pressed 0 is released
            uint16_t inverted_row = ~scanned_state[row];

            // extract the lower 8 bits (Keys on GP8-15) and shift them into place
            current_keys |= ((uint64_t)(inverted_row & 0x00FF) << (row * 8));

            // extract the upper 8 bits and discard the most significant bit GP23 and shift them into place
            current_btns |= ((uint64_t)((inverted_row >> 8) & 0x7F) << (row * 8));
        }

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
            // trick to clear the lowest set bit to process the next.
            // when you -1 from changed_keys the lowest set bit will
            // always flip as it is borrowed from in the subtraction
            // then bitwise AND will set any flipped bit to 0
            changed_keys &= (changed_keys - 1);
        }
        previous_keys = current_keys;

        uint64_t pressed_btns = current_btns & ~previous_btns;

        // discard if more than 3 btns pressed
        if (__builtin_popcountll(current_btns) <= 2) {
            while (pressed_btns != 0) {
                int i = __builtin_ctzll(pressed_btns);
                PanelButton btn = btn_matrix[i];

                if (btn != BTN_NONE) {
                    btn_queue.push(btn);
                }

                pressed_btns &= (pressed_btns - 1);
            }
        }

        previous_btns = current_btns;

        // Resistor Ladder Button Processing
        PanelButton raw_ladder_btn = scan_ladder_btns();

        if (raw_ladder_btn != candidate_ladder_btn) {
            candidate_ladder_btn = raw_ladder_btn;
            candidate_start_time = to_ms_since_boot(get_absolute_time());
        } else {
            if ((to_ms_since_boot(get_absolute_time()) - candidate_start_time) > 30) {
                if (stable_ladder_btn != candidate_ladder_btn) {
                    stable_ladder_btn = candidate_ladder_btn;

                    if (stable_ladder_btn != BTN_NONE) {
                        btn_queue.push(stable_ladder_btn);
                    }
                }
            }
        }

        // PanelButton current_ladder_btn = scan_ladder_btns();

        // if (current_ladder_btn != BTN_NONE && current_ladder_btn != previous_ladder_btn) {
        //     btn_queue.push(current_ladder_btn);
        // }

        // previous_ladder_btn = current_ladder_btn;

        while (btn_queue.size() > 0) {
            PanelButton btn;
            btn_queue.pop(btn);

            // testing - send a midi cc message on channel 1 
            uint8_t cc_msg[3] = { 0xB0, (uint8_t)btn, 127 };
            tud_midi_stream_write(0, cc_msg, 3);
        }

    }
    return 0;
}


// // Scanning the btn matrices to map them
// int main () {
//     stdio_init_all();

//     // init all pins as inputs with pullup resistors 
//     for (int i{0}; i < 15; i++) {
//         gpio_init(i);
//         gpio_set_dir(i, GPIO_IN);
//         gpio_pull_up(i);
//     }

//     // scanning
//     int out_pin{0};
//     while (true) {
//         // set pin to output and low
//         gpio_set_dir(out_pin, GPIO_OUT);
//         gpio_put(out_pin, 0);

//         sleep_us(100);
//         // scan all other pins for btn press
//         for (int in_pin{0}; in_pin < 15; in_pin++) {
//             if (in_pin != out_pin) {
//                 if (gpio_get(in_pin) == 0) {
//                     printf("Button pressed: Pin %d and Pin %d bridged\n", out_pin, in_pin);
//                 }
//             }
//         }

//         // return pin to input with pull-up
//         gpio_set_dir(out_pin, GPIO_IN);
//         gpio_pull_up(out_pin);
        
//         // loop
//         if (out_pin < 14) {
//             out_pin++;
//         } else {
//             out_pin = 0;
//             printf("--- Scan Complete ---\n");
//             sleep_ms(500);
//         }
//     }

//     return 0;
// }