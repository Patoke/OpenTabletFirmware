#ifndef PEN_H
#define PEN_H

// main tablet header

#include "gd32f3x0.h"
#include <stdint.h>

#define EXPRESS_BUTTON_1 (BIT(0))
#define EXPRESS_BUTTON_2 (BIT(1))
#define EXPRESS_BUTTON_3 (BIT(2))
#define EXPRESS_BUTTON_4 (BIT(3))
#define EXPRESS_BUTTON_5 (BIT(4))
#define EXPRESS_BUTTON_6 (BIT(5))
#define EXPRESS_BUTTON_7 (BIT(6))
#define EXPRESS_BUTTON_8 (BIT(7))
#define EXPRESS_BUTTON_9 (BIT(8))
#define EXPRESS_BUTTON_10 (BIT(9))

enum tip_states_t
{
    TIP_STATE_HOVER = 0,
    TIP_STATE_ACTIVE = 1,
    TIP_STATE_MAX_PRESSURE = 2
};

typedef struct
{
    uint32_t pen_x_pos;
    uint32_t pen_y_pos;
    
    int signal_window_buffer[5];

    bool is_in_range;

    int pen_tilt_x;
    int pen_tilt_y;
} pen_inputs;

typedef struct
{
    uint8_t x_coil_idx;
    uint8_t y_coil_idx;

    int x_signal_buffer[8];
    int y_signal_buffer[8];

    pen_inputs inputs;
} pen_data;

typedef struct
{
    uint8_t active_coil_idx;
    uint8_t tip_state;
    uint32_t signal_magnitude;
    uint32_t best_sample;
    uint32_t last_sample;
} analog_sample;

typedef struct __attribute__((packed))
{
    int keep_mask;
    uint16_t set_value;
} multiplexer_cfg_t;

typedef struct
{
  uint32_t interface;
  uint32_t protocol;
  uint32_t idlestate;
  uint8_t data[8];
} tablet_hid_handler;

void rcu_config(void);

void init_timer1(void);
void init_timer2(void);
void init_timer5(void);

//void read_express_buttons();
void read_pen_data(pen_data* data, multiplexer_cfg_t* y_mux_data, multiplexer_cfg_t* x_mux_data);

void do_sample(void);

#endif