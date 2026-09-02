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
    BTN_NONE,
    BTN_INTRO_ENDING,
    BTN_START_STOP,
    BTN_MAIN_VOL_UP,
    BTN_MAIN_VOL_DOWN,
    BTN_TEMPO_UP,
    BTN_TEMPO_DOWN,
    BTN_ACCOMP_VOL_UP,
    BTN_ACCOMP_VOL_DOWN,
    BTN_TRANSPOSE_UP,
    BTN_TRANSPOSE_DOWN,
    BTN_SUSTAIN,
    BTN_VIBRATO,
    BTN_SYNC,
    BTN_SINGLE,
    BTN_FINGERED,
    BTN_FILL,
    BTN_METRONOME,
    BTN_SPLIT,
    BTN_REC,
    BTN_PROGRAM,
    BTN_PLAYBACK,
    BTN_MEMORY,
    BTN_M1,
    BTN_M2,
    BTN_PERCUSSION,
    BTN_PLAY_PAUSE,
    BTN_SKIP_BACK,
    BTN_SKIP,
    BTN_VOL_DOWN,
    BTN_VOL_UP,
    BTN_TEACH1,
    BTN_TEACH2,
    BTN_TONE,
    BTN_RYTHM,
    BTN_DEMO,
    BTN_7,
    BTN_8,
    BTN_9,
    BTN_MINUS,
    BTN_4,
    BTN_5,
    BTN_6,
    BTN_PLUS,
    BTN_1,
    BTN_2,
    BTN_3,
    BTN_0
};

struct ButtonMap {
    PanelButton btn;
    uint16_t min_val;
    uint16_t max_val;
};

const ButtonMap ladder_map[] = {
    {BTN_PLAY_PAUSE, 3995, 4195},
    {BTN_SKIP_BACK, 2130, 2330},
    {BTN_SKIP, 2880, 3080},
    {BTN_VOL_DOWN, 3360, 3560},
    {BTN_VOL_UP, 3740, 3940}
};

const PanelButton btn_matrix[64] = {
    BTN_7, BTN_ACCOMP_VOL_DOWN, BTN_PLAYBACK, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE, BTN_NONE,
    BTN_6, BTN_START_STOP, BTN_REC, BTN_TEACH2, BTN_NONE, BTN_NONE, BTN_M2, BTN_NONE,
    BTN_5, BTN_TRANSPOSE_UP, BTN_PROGRAM, BTN_TEACH1, BTN_NONE, BTN_SUSTAIN, BTN_M1, BTN_NONE,
    BTN_4, BTN_MAIN_VOL_UP, BTN_SYNC, BTN_VIBRATO, BTN_NONE, BTN_TEMPO_UP, BTN_MEMORY, BTN_NONE,
    BTN_3, BTN_MINUS, BTN_FILL, BTN_TEMPO_DOWN, BTN_NONE, BTN_TRANSPOSE_DOWN, BTN_ACCOMP_VOL_UP, BTN_NONE,
    BTN_2, BTN_PLUS, BTN_FINGERED, BTN_RYTHM, BTN_NONE, BTN_MAIN_VOL_DOWN, BTN_INTRO_ENDING, BTN_NONE,
    BTN_1, BTN_9, BTN_SINGLE, BTN_TONE, BTN_NONE, BTN_DEMO, BTN_SPLIT, BTN_NONE,
    BTN_0, BTN_8, BTN_NONE, BTN_NONE, BTN_PERCUSSION, BTN_NONE, BTN_METRONOME, BTN_NONE
};

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


class ButtonQueue {
    private:
        static constexpr int SIZE = 3; // max 3 btns pressed at once
        PanelButton buffer[SIZE];
        int head = 0;
        int tail = 0;
        int count = 0;

    public:
        bool push(PanelButton btn) {
            if (count == SIZE) return false; // full

            buffer[tail] = btn;
            tail = (tail + 1) % SIZE;
            count++;

            return true;
        }

        bool pop(PanelButton& btn) {
            if (count == 0) return false; // empty

            btn = buffer[head];
            head = (head + 1) % SIZE;
            count--;

            return true;
        }

        int size() const {
            return count;
        }
};

// --- HARDWARE SETUP ---

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

// --- HARDWARE CONTROL ---

class InputController {
private:
    uint64_t previous_keys = 0;
    uint64_t previous_btns = 0;
    ButtonQueue btn_queue;

    PanelButton stable_ladder_btn = BTN_NONE;    // correct state 
    PanelButton candidate_ladder_btn = BTN_NONE; // fluctuating state
    uint32_t candidate_start_time = 0;           // timestamp for debounce


    PanelButton get_raw_ladder_button() {
        uint32_t adc_sum = 0;
        
        for (int i = 0; i < 8; i++) {
            adc_sum += adc_read();
        }

        // divide by 8 to average smoothing electrical noise
        uint16_t adc_val = adc_sum >> 3;

        if (adc_val < 100) return BTN_NONE;

        for (const auto& map : ladder_map) {
            if (adc_val >= map.min_val && adc_val <= map.max_val) {
                return map.btn;
            }
        }

        return BTN_NONE;
    }

    void process_matrix() {
        uint64_t current_keys = 0;
        uint64_t current_btns = 0;

        // get and split the state into keys and buttons
        for (int row = 0; row < 8; row++) {
            // invert the 16 bit row so 1 is pressed 0 is released
            uint16_t inverted_row = ~scanned_state[row];

            // extract the lower 8 bits (Keys on GP8-15) and shift them into place
            current_keys |= ((uint64_t)(inverted_row & 0x00FF) << (row * 8));

            // extract the upper 8 bits and discard the most significant bit GP23 and shift them into place
            current_btns |= ((uint64_t)((inverted_row >> 8) & 0x7F) << (row * 8));
        }

        // --- PROCESS KEYS ---

        
        uint64_t changed_keys = current_keys ^ previous_keys;
        while (changed_keys) {
            // finds the index of the lowest set bit (changed key) in a single clock cycle
            int i = __builtin_ctzll(changed_keys);
            uint8_t note = midi_matrix[i];
            
            if (note) {
                // differentiate press or release
                uint8_t velocity = ((current_keys >> i) & 1ULL) ? 127 : 0;
                uint8_t status = velocity ? 0x90 : 0x80;
                uint8_t msg[3] = { status, note, velocity };

                tud_midi_stream_write(0, msg, 3);
            }
            // trick to clear the lowest set bit to process the next.
            // when you -1 from changed_keys the lowest set bit will
            // always flip as it is borrowed from in the subtraction,
            // then bitwise AND will set any flipped bit to 0
            changed_keys &= (changed_keys - 1);
        }
        previous_keys = current_keys;

        // --- PROCESS BUTTONS ---

        uint64_t pressed_btns = current_btns & ~previous_btns;
        if (__builtin_popcountll(current_btns) <= 2) { // discard if more than 2 btns pressed
            while (pressed_btns) {
                int i = __builtin_ctzll(pressed_btns);
                
                if (btn_matrix[i] != BTN_NONE) {
                    btn_queue.push(btn_matrix[i]);
                }

                pressed_btns &= (pressed_btns - 1);
            }
        }

        previous_btns = current_btns;
    }

    void process_ladder() {
        PanelButton raw_ladder_btn = get_raw_ladder_button();
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (raw_ladder_btn != candidate_ladder_btn) {
            candidate_ladder_btn = raw_ladder_btn;
            candidate_start_time = now;
        } else if ((now - candidate_start_time) > 30){
            if (stable_ladder_btn != candidate_ladder_btn) {
                stable_ladder_btn = candidate_ladder_btn;

                if (stable_ladder_btn != BTN_NONE) {
                    btn_queue.push(stable_ladder_btn);
                }
            }
        }
    }

    void dispatch_events() {
        while (btn_queue.size() > 0) {
            PanelButton btn;
            btn_queue.pop(btn);
            uint8_t msg[3] = { 0xB0, static_cast<uint8_t>(btn), 127 };
            tud_midi_stream_write(0, msg, 3);
        }
    }

public:
    void update() {
        tud_task();
        process_matrix();
        process_ladder();
        dispatch_events();
    }
};

int main() {
    // init board and USB stack
    board_init();
    tusb_init();
    stdio_init_all();

    // setup pio, dma and adc
    setup_pio(pio0, 0);
    setup_dma(pio0, 0);
    setup_adc();

    InputController input_controller;

    while (true) {
        input_controller.update();
    }

    return 0;
}