#include "pen.h"
#include "core_cmFunc.h"
#include "drv_usb_hw.h"
#include "gd32f3x0_gpio.h"
#include "adc.h"

#define USE_FPU 0

#define Y_COIL_COUNT 25
#define X_COIL_COUNT 39

#define PEN_X_RESOLUTION 50800
#define PEN_Y_RESOLUTION 31750

// the number 2 comes from the margin gaps between the coils
#define PEN_X_OVERSCAN 51 // physical space used by internal components
#define PEN_Y_OVERSCAN 10 // physical space missing from the bottom of the coil grid

#define PEN_X_PITCH ((PEN_X_RESOLUTION / (X_COIL_COUNT - 2)))
#define PEN_Y_PITCH ((PEN_Y_RESOLUTION / (Y_COIL_COUNT - 2)) + PEN_Y_OVERSCAN)

#define NUM_ADC_SAMPLES 3
#define PEN_ADC_CHANNEL 7

#define COIL_NUM_PULSES 29

typedef struct 
{
  uint32_t val;
  uint32_t idx;
} iterator_t;

typedef struct
{
    int correction_factor;
    int frequency;
} pressure_cal_t;

// @note: for now, won't implement any parsing of the touch ring or tablet buttons 
pen_data pen;
analog_sample pen_sample;

int global_signal_threshold = 300;
uint8_t current_coil_group = 9;

pressure_cal_t pressure_cal_values[13] = 
{
    {0, 0},
    {1, 452830},
    {2, 461538},
    {3, 470588},
    {4, 480000},
    {5, 489796},
    {6, 500000},
    {7, 503497},
    {8, 510638},
    {9, 517986},
    {0xA, 525547},
    {0xB, 533333},
    {0xC, 545455},
};

multiplexer_cfg_t y_mux_search_data[Y_COIL_COUNT] = 
{
    {0xFBFF0001, 6},
    {0xF7FF0002, 1},
    {0xF7FF0003, 5},
    {0xF7FF0004, 4},
    {0xEFFF0005, 0},
    {0xEFFF0006, 7},
    {0xDFFF0007, 2},
    {0xDFFF0008, 3},
    {0xDFFF0009, 6},
    {0xBFFF000A, 1},
    {0xBFFF000B, 5},
    {0xBFFF000C, 4},
    {0x7FFF000D, 0},
    {0x7FFF000E, 7},
    {0xFEFF000F, 2},
    {0xFEFF0010, 3},
    {0xFEFF0011, 6},
    {0xFDFF0012, 2},
    {0xFDFF0013, 1},
    {0xFDFF0014, 0},
    {0xFDFF0015, 3},
    {0xFDFF0016, 5},
    {0xFDFF0017, 7},
    {0xFDFF0018, 6},
    {0xFDFF0019, 4},
};
multiplexer_cfg_t x_mux_search_data[X_COIL_COUNT] =
{
    {0xFBFF0001, 2},
    {0xFBFF0002, 1},
    {0xFBFF0003, 0},
    {0xFBFF0004, 3},
    {0xFBFF0005, 5},
    {0xFBFF0006, 7},
    {0xFBFF0007, 4},
    {0xF7FF0008, 2},
    {0xF7FF0009, 0},
    {0xF7FF000A, 3},
    {0xF7FF000B, 7},
    {0xF7FF000C, 6},
    {0xEFFF000D, 2},
    {0xEFFF000E, 1},
    {0xEFFF000F, 3},
    {0xEFFF0010, 5},
    {0xEFFF0011, 6},
    {0xEFFF0012, 4},
    {0xDFFF0013, 1},
    {0xDFFF0014, 0},
    {0xDFFF0015, 5},
    {0xDFFF0016, 7},
    {0xDFFF0017, 4},
    {0xBFFF0018, 2},
    {0xBFFF0019, 0},
    {0xBFFF001A, 3},
    {0xBFFF001B, 7},
    {0xBFFF001C, 6},
    {0x7FFF001D, 2},
    {0x7FFF001E, 1},
    {0x7FFF001F, 3},
    {0x7FFF0020, 5},
    {0x7FFF0021, 6},
    {0x7FFF0022, 4},
    {0xFEFF0023, 1},
    {0xFEFF0024, 0},
    {0xFEFF0025, 5},
    {0xFEFF0026, 7},
    {0xFEFF0027, 4},
};

int tilt_table_y_edge[8];
int tilt_table_x_edge[8];

int tilt_y_sample_buffer[9];
int tilt_x_sample_buffer[9];

int tilt_scan_phase;

void find_top_three_peaks(int* data, int count, int* out_first_peak, int* out_second_peak, int* out_third_peak)
{
    int val_first = 0;
    int val_second = 0;
    int val_third = 0;

    *out_first_peak = 0;
    *out_second_peak = 0;
    *out_third_peak = 0;

    for (int i = 1; i < (count + 1); i = (i + 1))
    {
        int current_val = data[i];

        if (current_val <= val_first)
        {
            if (current_val <= val_second)
            {
                if (current_val > val_third)
                {
                    val_third = current_val;
                    *out_third_peak = i;
                }
            }
            else
            {
                val_third = val_second;
                *out_third_peak = *out_second_peak;

                val_second = current_val;
                *out_second_peak = i;
            }
        }
        else
        {
            val_third = val_second;
            *out_third_peak = *out_second_peak;

            val_second = val_first;
            *out_second_peak = *out_first_peak;

            val_first = current_val;
            *out_first_peak = i;
        }
    }
}

int get_highest_value_index(int* data, int count)
{
    iterator_t it;
    it.idx = 1;
    it.val = 0;

    int highest_value = data[0];
    while (it.idx < count)
    {
        int current_val = data[it.idx];

        if (current_val > highest_value)
        {
            highest_value = current_val;
            it.val = it.idx;
        }

        it.idx++;
    }

    return it.val;
}

int do_median_filter(int* data, int count)
{
    // bubble sort
    for (int i = 0; i < count - 1; i++)
    {
        for (int j = 0; i < count - i - 1; j++)
        {
            if (data[j] > data[j + 1])
            {
                int value = data[j];
                data[j] = data[j + 1];
                data[j + 1] = value;
            }
        }
    }

    return data[(count - 1) / 2];
}

int do_parabolic_interp(int peak_idx, analog_sample *sample, pen_data* data)
{
    // edge coils
    if (sample->active_coil_idx == 1)
    {
        return (pressure_cal_values[2].frequency + pressure_cal_values[1].frequency) / 2;
    }
    if (sample->active_coil_idx == 12)
    {
        return (pressure_cal_values[12].frequency + pressure_cal_values[11].frequency) / 2;
    }

    // get signal strengths
    int y_left   = data->inputs.signal_window_buffer[peak_idx - 1];
    int y_center = data->inputs.signal_window_buffer[peak_idx];
    int y_right  = data->inputs.signal_window_buffer[peak_idx + 1];

    // get physical positions
    int x_left   = pressure_cal_values[sample->active_coil_idx - 1].frequency;
    int x_center = pressure_cal_values[sample->active_coil_idx].frequency;
    int x_right  = pressure_cal_values[sample->active_coil_idx + 1].frequency;

    int pitch_left = x_center - x_left;
    int pitch_right = x_right - x_center;
    
    int pitch = pitch_right; 

    int signal_diff = y_right - y_left;
    int curvature = (2 * y_center) - y_left - y_right;

    // don't divide by zero
    if (curvature == 0)
    {
        return x_center;
    }

    int offset = (signal_diff * pitch) / (2 * curvature);

    int final_pos = x_center + offset;

    if (final_pos > x_right)
    {
        final_pos = x_right;
    }
    if (final_pos < x_left)
    {
        final_pos = x_left;
    }

    return final_pos;
}

int div_u32(unsigned int numerator, unsigned int denominator)
{
  int quotient = 0;
  int shift = 32;
  while ( shift-- > 0 )
  {
    if ( numerator >> shift >= denominator )
    {
      numerator -= denominator << shift;
      quotient += 1 << shift;
    }
  }
  return quotient;
}

int calc_offset_parabolic(int* samples, int pitch)
{
#if USE_FPU == 1
    float y_left   = (float)samples[0];
    float y_center = (float)samples[1];
    float y_right  = (float)samples[2];
    float f_pitch  = (float)pitch;

    float numerator = (y_right - y_left) * f_pitch;
    float curvature = (2.0f * y_center) - y_left - y_right;

    if (curvature < 0.1f)
    {
        return 0; 
    }

    float offset_f = numerator / (2.0f * curvature);

    return (int)offset_f;
#else
    int offset;

    if (samples[3] < samples[1])
    {
        offset = (pitch >> 1) - div_u32((samples[1] - samples[3]) * pitch, samples[2] - samples[0] + samples[1] - samples[3]);
    }
    else
    {
        offset = div_u32((samples[3] - samples[1]) * pitch, samples[2] - samples[4] + samples[3] - samples[1]) + (pitch >> 1); 
    }

    if (offset > (pitch + 150))
    {
        offset = pitch + 150;
    }
    if (offset < -150)
    {
        return -150;
    }

    return offset;
#endif
}

int calc_offset_linear(int* samples, int pitch)
{
#if USE_FPU == 1
    float y_left   = (float)samples[1];
    float y_center = (float)samples[2];
    float y_right  = (float)samples[3];

    float numerator = (y_center - y_left) * pitch;
    float denominator = (2.0f * y_center) - y_left - y_right;

    if (denominator < 1.0f)
    {
        return 0.0f;
    }

    return numerator / denominator;
#else
    int offset = div_u32((samples[2] - samples[1]) * pitch, samples[2] - samples[1] + samples[2] - samples[3]);
    if (offset > pitch)
    {
        return pitch;
    }
    if (offset < 0)
    {
        return 0;
    }

    return offset;
#endif
}

int sample_pen_adc(int adc_mode)
{
    int samples[NUM_ADC_SAMPLES] = {};

    adc_config(PEN_ADC_CHANNEL);
    usb_udelay(30);
    if (pen_sample.tip_state == TIP_STATE_MAX_PRESSURE)
    {
        usb_udelay(20);
    }

    gpio_bit_reset(GPIOA, GPIO_PIN_5);
    usb_udelay(30);

    for (int i = 0; i < NUM_ADC_SAMPLES; i++)
    {
        samples[i] = adc_channel_sample(PEN_ADC_CHANNEL);
    }

    int out_value = do_median_filter(samples, NUM_ADC_SAMPLES);

    gpio_bit_set(GPIOA, GPIO_PIN_5);
    GPIO_CTL(GPIOB) |= 0xFF00;

    if (adc_mode == 1)
    {
        usb_udelay(120);
    }
    else
    {
        usb_udelay(30);
    }

    return out_value;
}

void power_coil(int coil_group)
{
    // @note: this was a big switch case but the functions that emit pulses are exactly the same between cases, so i'll just inline it
    if (coil_group < 1 || coil_group > 12)
    {
        return;
    }

    int pulse_counter = COIL_NUM_PULSES;

    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    GPIO_BC(GPIOA) = GPIO_PIN_4;

    gpio_bit_set(GPIOA, GPIO_PIN_6);

    bool finished = FALSE;
    while (1)
    {
        finished = (pulse_counter == 0);
        pulse_counter--;
        if (finished)
        {
            break;
        }

        GPIO_BOP(GPIOA) = GPIO_PIN_4;
        GPIO_BC(GPIOA) = GPIO_PIN_4;
    }

    GPIO_BC(GPIOA) = GPIO_PIN_4;

    // @note: unknown function, manually inlined
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_4);

    gpio_bit_reset(GPIOA, GPIO_PIN_6);
}

int do_frequency_sample(multiplexer_cfg_t* mux_data, int coil_group, int adc_mode)
{
    GPIO_OCTL(GPIOB) = (GPIO_OCTL(GPIOB) & mux_data->keep_mask);
    GPIO_OCTL(GPIOB) &= ~0xFFFF0007;
    GPIO_OCTL(GPIOB) |= mux_data->set_value;

    power_coil(coil_group);

    return sample_pen_adc(adc_mode);
}

int do_frequency_sample_pos(multiplexer_cfg_t x_mux_data, multiplexer_cfg_t y_mux_data, int coil_group, int adc_mode)
{
    GPIO_OCTL(GPIOB) = (GPIO_OCTL(GPIOB) & x_mux_data.keep_mask);
    GPIO_OCTL(GPIOB) &= ~0xFFFF0007;
    GPIO_OCTL(GPIOB) |= x_mux_data.set_value;

    power_coil(coil_group);

    GPIO_OCTL(GPIOB) |= 0xFF00;

    GPIO_OCTL(GPIOB) = (GPIO_OCTL(GPIOB) & y_mux_data.keep_mask);
    GPIO_OCTL(GPIOB) &= ~0xFFFF0007;
    GPIO_OCTL(GPIOB) |= y_mux_data.set_value;

    return sample_pen_adc(adc_mode);
}

int scan_frequency_spectrum(int scan_count, int* out_buffer, multiplexer_cfg_t* mux_data, int coil_group)
{
    int first_peak_idx, second_peak_idx, third_peak_idx;

    int scan_idx = 0;
    int out_peak_idx = 0;

    while (scan_idx < scan_count)    
    {
        out_buffer[scan_idx + 1] = do_frequency_sample(mux_data, coil_group, 0);
        scan_idx++;
    }

    find_top_three_peaks(out_buffer, scan_count, &first_peak_idx, &second_peak_idx, &third_peak_idx);

    int second_peak_value = out_buffer[second_peak_idx];
    int third_peak_value = out_buffer[third_peak_idx];

    if (out_buffer[first_peak_idx] < (global_signal_threshold * 2 / 3))
    {
        return 0;
    }

    int next_peak_value = out_buffer[first_peak_idx + 1];
    int last_peak_value = out_buffer[first_peak_idx - 1];
    if (first_peak_idx == 1)
    {
        if (next_peak_value == second_peak_value)
        {
            out_peak_idx = 1;
        }
    }
    else if (first_peak_idx == scan_count)
    {
        if (last_peak_value == second_peak_value)
        {
            out_peak_idx = scan_count;
        }
    }
    else if ((next_peak_value == second_peak_value && last_peak_value == third_peak_value)
            || (next_peak_value == third_peak_value && next_peak_value == second_peak_value))
    {
        out_peak_idx = first_peak_idx;
    }
    else
    {
        out_peak_idx = 0;
    }

    if (out_buffer[first_peak_idx] > (3 * global_signal_threshold) >> 1)
    {
        out_peak_idx = first_peak_idx;
    }

    return out_peak_idx;
}

void do_input_processing(pen_data* data)
{
    // @note: does SMA and EMA filters, skip completely, we don't care
    if (data->inputs.is_in_range)
    {
        if (data->inputs.pen_x_pos > PEN_X_RESOLUTION)
        {
            data->inputs.pen_x_pos = PEN_X_RESOLUTION;
        }
        if (data->inputs.pen_y_pos > PEN_Y_RESOLUTION)
        {
            data->inputs.pen_y_pos = PEN_Y_RESOLUTION;
        }
    }
    else
    {
    
    }
}

int calculate_position_wide_scan(pen_data* data, analog_sample* sample)
{
    data->inputs.signal_window_buffer[0] = 0;
    data->inputs.signal_window_buffer[1] = 0;
    // @note: unk var being set to 0 here

    for (int i = 2; i < 11; i++)
    {
        multiplexer_cfg_t* x_mux_data = &x_mux_search_data[data->x_coil_idx - 1];
        multiplexer_cfg_t* y_mux_data = &x_mux_search_data[data->y_coil_idx - 1];
        data->inputs.signal_window_buffer[i] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, i + 1, i != 10);
    }

    int highest_sample_idx = get_highest_value_index(data->inputs.signal_window_buffer, 12);
    sample->active_coil_idx = highest_sample_idx;
    sample->best_sample = data->inputs.signal_window_buffer[highest_sample_idx];

    // if we're above the global signal threshold, use the center coil
    if (data->inputs.signal_window_buffer[highest_sample_idx] < global_signal_threshold)
    {
        sample->active_coil_idx = 8;
        sample->signal_magnitude = pressure_cal_values[8].frequency;
    }
    else if (highest_sample_idx)
    {
        switch (highest_sample_idx)
        {
        case 1:
            sample->signal_magnitude = (pressure_cal_values[2].frequency + pressure_cal_values[1].frequency) / 2;
            break;
        case 2:
            sample->signal_magnitude = (pressure_cal_values[3].frequency + pressure_cal_values[2].frequency) / 2;
            break;
        case 10:
            sample->signal_magnitude = (pressure_cal_values[11].frequency + pressure_cal_values[10].frequency) / 2;
            break;
        case 11:
            sample->signal_magnitude = (pressure_cal_values[12].frequency + pressure_cal_values[11].frequency) / 2;
            break;
        default:
            sample->signal_magnitude = do_parabolic_interp(highest_sample_idx, sample, data);
            break;
        }
    }
    else
    {
        sample->signal_magnitude = (pressure_cal_values[2].frequency + pressure_cal_values[1].frequency) / 2;
    }
    
    sample->last_sample = sample->best_sample;
    return highest_sample_idx;
}

int calculate_pen_position(pen_data* data, analog_sample* sample)
{
    int curr_window = 0;

    int last_x_coil_idx = data->x_coil_idx - 1;

    while (curr_window < 5)
    {
        if ((sample->active_coil_idx + curr_window) <= 2 || (sample->active_coil_idx + curr_window) > 14)
        {
            data->inputs.signal_window_buffer[curr_window] = 0;
        }
        else
        {
            multiplexer_cfg_t* x_mux_data = &x_mux_search_data[data->x_coil_idx - 1];
            multiplexer_cfg_t* y_mux_data = &x_mux_search_data[data->y_coil_idx - 1];
            data->inputs.signal_window_buffer[curr_window] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, sample->active_coil_idx - 2 + curr_window, curr_window < 4);
        }

        curr_window++;
    }

    int peak_index = get_highest_value_index(data->inputs.signal_window_buffer, 5);
    if ((sample->active_coil_idx + peak_index) > 2 && (sample->active_coil_idx + peak_index) <= 14)
    {
        sample->active_coil_idx = sample->active_coil_idx + peak_index - 2;
    }

    sample->best_sample = data->inputs.signal_window_buffer[peak_index];

    if ((sample->last_sample / 3) <= sample->best_sample && peak_index != 0 && peak_index != 4)
    {
        if (peak_index >= 1 && peak_index <= 3)
        {
            sample->signal_magnitude = do_parabolic_interp(peak_index, sample, data);
        }
    }
    else
    {
        peak_index = calculate_position_wide_scan(data, sample);
    }

    sample->last_sample = sample->best_sample;
    return peak_index;
}

void update_pressure_state(analog_sample* sample, int signal_strength)
{
    if (sample->tip_state == TIP_STATE_ACTIVE)
    {
        if (signal_strength < 700)
        {
            sample->tip_state = TIP_STATE_HOVER;
            // @note: unknown function, manually inlined
            gpio_bit_reset(GPIOC, GPIO_PIN_15);
        }
        if (signal_strength > 3700)
        {
            sample->tip_state = TIP_STATE_MAX_PRESSURE;
            // @note: unknown function, manually inlined
            gpio_bit_set(GPIOC, GPIO_PIN_15);
        }
    }
    else
    {
        if (sample->tip_state != TIP_STATE_HOVER && signal_strength >= 1500)
        {
            return;
        }
        else if (signal_strength <= 2500)
        {
            return;
        }

        sample->tip_state = TIP_STATE_ACTIVE;
        // @note: unknown function, manually inlined
        gpio_bit_set(GPIOC, GPIO_PIN_15);
    }
}

void sample_tilt_y(pen_data* data, analog_sample* sample)
{
    for (int i = 0; i < 6; i++)
    {
        if ((data->y_coil_idx + i) > 3)
        {
            if ((data->y_coil_idx + i - 3) <= 25)
            {
                multiplexer_cfg_t* x_mux_data = &x_mux_search_data[data->x_coil_idx - 1];
                multiplexer_cfg_t* y_mux_data = &x_mux_search_data[data->y_coil_idx - 4 + i];
                tilt_table_y_edge[i] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, sample->active_coil_idx, 0);

                // @note: do another sample? mux data used is 1:1 between the 2, seems weird
                if (data->y_coil_idx == 2 || data->y_coil_idx == 1)
                {
                    tilt_table_y_edge[i] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, sample->active_coil_idx, 0);
                }
            }
            else
            {
                tilt_table_y_edge[i] = 0;
            }
        }
        else
        {
            tilt_table_y_edge[i] = 0;
        }
    }
}

void sample_tilt_x(pen_data* data, analog_sample* sample)
{
    for (int i = 0; i < 6; i++)
    {
        if ((data->x_coil_idx + i) > 3)
        {
            if ((data->x_coil_idx + i - 3) <= X_COIL_COUNT)
            {
                multiplexer_cfg_t* x_mux_data = &x_mux_search_data[data->x_coil_idx - 4 + i];
                multiplexer_cfg_t* y_mux_data = &x_mux_search_data[data->y_coil_idx - 1];
                tilt_table_x_edge[i] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, sample->active_coil_idx, 0);

                // @note: do another sample? mux data used is 1:1 between the 2, seems weird
                if (data->x_coil_idx == 2 || data->x_coil_idx == 1)
                {
                    tilt_table_x_edge[i] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, sample->active_coil_idx, 0);
                }
            }
            else
            {
                tilt_table_x_edge[i] = 0;
            }
        }
        else
        {
            tilt_table_x_edge[i] = 0;
        }
    }
}

int calculate_tilt_ratio(int* tilt_buffer)
{
    int tilt_direction, peak_val;

    int peak_idx = -1;

    if ((tilt_buffer[9] + tilt_buffer[10]) < (tilt_buffer[0] + tilt_buffer[1]))
    {
        tilt_direction = 0; // left

        for (int i = 1; i <= 4; ++i)
        {
            if (tilt_buffer[i - 1] >= tilt_buffer[i] && tilt_buffer[i] <= tilt_buffer[i + 1])
            {
                peak_idx = i;
            }
        }
        peak_val = tilt_buffer[peak_idx];
        for (int j = peak_idx; j >= 0; --j)
        {
            int temp_val;
            if (tilt_buffer[j] <= peak_val)
            {
                temp_val = peak_val;
            }
            else
            {
                temp_val = tilt_buffer[j];
            }
            peak_val = temp_val;
        }
    }
    else
    {
        tilt_direction = 1; // right

        for (int i = 6; i <= 9; ++i)
        {
            if (tilt_buffer[i - 1] >= tilt_buffer[i] && tilt_buffer[i] <= tilt_buffer[i + 1])
            {
                peak_idx = i;
            }
        }
        peak_val = tilt_buffer[peak_idx];
        for (int j = peak_idx; j <= 10; ++j)
        {
            int temp_val;
            if (tilt_buffer[j] <= peak_val)
            {
                temp_val = peak_val;
            }
            else
            {
                temp_val = tilt_buffer[j];
            }
            peak_val = temp_val;
        }
    }

    int center_val = tilt_buffer[5];
    if (peak_idx == -1)
    {
        return 0;
    }

    if (tilt_direction == 1)
    {
        return (peak_val * 1024) / center_val;
    }
    else
    {
        return -((peak_val * 1024) / center_val);
    }
}

void calculate_tilt_angle_y(int scan_phase)
{
    tilt_y_sample_buffer[2] = tilt_table_y_edge[0];
    tilt_y_sample_buffer[3] = tilt_table_y_edge[1];
    tilt_y_sample_buffer[4] = tilt_table_y_edge[2];
    tilt_y_sample_buffer[5] = tilt_table_y_edge[3];
    tilt_y_sample_buffer[6] = tilt_table_y_edge[4];
    tilt_y_sample_buffer[7] = tilt_table_y_edge[5];
    tilt_y_sample_buffer[8] = tilt_table_y_edge[6];

    if (scan_phase == 1)
    {
        if (pen.y_coil_idx <= 20)
        {
            for (int i = 0; i <= 1; i++)
            {
                multiplexer_cfg_t* x_mux_data = &x_mux_search_data[pen.x_coil_idx - 1];
                multiplexer_cfg_t* y_mux_data = &x_mux_search_data[pen.y_coil_idx + 3 + i];
                tilt_y_sample_buffer[i + 9] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, pen_sample.active_coil_idx, 0);
            }
        }
        if (pen.y_coil_idx > 5)
        {
            for (int i = 0; i <= 1; i++)
            {
                multiplexer_cfg_t* x_mux_data = &x_mux_search_data[pen.x_coil_idx - 1];
                multiplexer_cfg_t* y_mux_data = &x_mux_search_data[pen.y_coil_idx - 6 + i];
                tilt_y_sample_buffer[i + 9] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, pen_sample.active_coil_idx, 0);
            }
        }
    }

    pen.inputs.pen_tilt_y = calculate_tilt_ratio(tilt_y_sample_buffer);
    if (pen.inputs.pen_tilt_y <= 170)
    {
        if (pen.inputs.pen_tilt_y >= -180)
        {
            pen.inputs.pen_tilt_y = 0;
        }
        else
        {
            pen.inputs.pen_tilt_y += 180;
        }
    }
    else 
    {
        pen.inputs.pen_tilt_y -= 170;
    }

    pen.inputs.pen_tilt_y = -(pen.inputs.pen_tilt_y / 8);
}

void calculate_tilt_angle_x(int scan_phase)
{
    tilt_x_sample_buffer[2] = tilt_table_y_edge[0];
    tilt_x_sample_buffer[3] = tilt_table_y_edge[1];
    tilt_x_sample_buffer[4] = tilt_table_y_edge[2];
    tilt_x_sample_buffer[5] = tilt_table_y_edge[3];
    tilt_x_sample_buffer[6] = tilt_table_y_edge[4];
    tilt_x_sample_buffer[7] = tilt_table_y_edge[5];
    tilt_x_sample_buffer[8] = tilt_table_y_edge[6];

    if (scan_phase == 0)
    {
        if (pen.x_coil_idx <= 34)
        {
            for (int i = 0; i <= 1; i++)
            {
                multiplexer_cfg_t* x_mux_data = &x_mux_search_data[pen.x_coil_idx + 3 + i];
                multiplexer_cfg_t* y_mux_data = &x_mux_search_data[pen.y_coil_idx - 1];
                tilt_x_sample_buffer[i + 9] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, pen_sample.active_coil_idx, 0);
            }
        }
        if (pen.x_coil_idx > 5)
        {
            for (int i = 0; i <= 1; i++)
            {
                multiplexer_cfg_t* x_mux_data = &x_mux_search_data[pen.x_coil_idx - 6 + i];
                multiplexer_cfg_t* y_mux_data = &x_mux_search_data[pen.y_coil_idx - 1];
                tilt_x_sample_buffer[i + 9] = do_frequency_sample_pos(*x_mux_data, *y_mux_data, pen_sample.active_coil_idx, 0);
            }
        }
    }

    pen.inputs.pen_tilt_x = calculate_tilt_ratio(tilt_x_sample_buffer);
    if (pen.inputs.pen_tilt_x <= 160)
    {
        if (pen.inputs.pen_tilt_x >= -140)
        {
            pen.inputs.pen_tilt_x = 0;
        }
        else
        {
            pen.inputs.pen_tilt_x += 140;
        }
    }
    else 
    {
        pen.inputs.pen_tilt_x -= 160;
    }

    pen.inputs.pen_tilt_x = pen.inputs.pen_tilt_x / 8;
}

void sample_pen_data(pen_data* data, analog_sample* sample)
{
    if (++tilt_scan_phase > 1)
    {
        tilt_scan_phase = 0;
    }

    __disable_irq();
    int position_index = calculate_pen_position(data, sample);
    update_pressure_state(sample, data->inputs.signal_window_buffer[position_index]);
    sample_tilt_y(data, sample);
    sample_tilt_x(data, sample);
    __enable_irq();

    int y_tilt_peak = get_highest_value_index(tilt_table_y_edge, 6);
    if (data->y_coil_idx == 1 || data->y_coil_idx == Y_COIL_COUNT)
    {
        data->y_signal_buffer[0] = tilt_table_y_edge[1];
        data->y_signal_buffer[4] = tilt_table_y_edge[5];
        data->y_signal_buffer[2] = tilt_table_y_edge[3];
        data->y_signal_buffer[1] = tilt_table_y_edge[2];
        data->y_signal_buffer[3] = tilt_table_y_edge[4];
    }
    else if (y_tilt_peak > 2 || data->y_coil_idx == 1)
    {
        if ((y_tilt_peak == 4 || y_tilt_peak == 5) && data->y_coil_idx != Y_COIL_COUNT)
        {
            ++data->y_coil_idx;
            data->y_signal_buffer[0] = tilt_table_y_edge[2];
            data->y_signal_buffer[4] = tilt_table_y_edge[6];
            data->y_signal_buffer[2] = tilt_table_y_edge[4];
            data->y_signal_buffer[1] = tilt_table_y_edge[3];
            data->y_signal_buffer[3] = tilt_table_y_edge[5];
        }
        else
        {
            data->y_signal_buffer[0] = tilt_table_y_edge[1];
            data->y_signal_buffer[4] = tilt_table_y_edge[5];
            data->y_signal_buffer[2] = tilt_table_y_edge[3];
            data->y_signal_buffer[1] = tilt_table_y_edge[2];
            data->y_signal_buffer[3] = tilt_table_y_edge[4];

            __disable_irq();
            calculate_tilt_angle_y(tilt_scan_phase);
            __enable_irq();
        }
    }
    else
    {
        --data->y_coil_idx;
        data->y_signal_buffer[0] = tilt_table_y_edge[0];
        data->y_signal_buffer[4] = tilt_table_y_edge[4];
        data->y_signal_buffer[2] = tilt_table_y_edge[2];
        data->y_signal_buffer[1] = tilt_table_y_edge[1];
        data->y_signal_buffer[3] = tilt_table_y_edge[3];
    }

    int x_tilt_peak = get_highest_value_index(tilt_table_y_edge, 6);
    if (data->x_coil_idx == 1 || data->x_coil_idx == X_COIL_COUNT)
    {
        data->x_signal_buffer[0] = tilt_table_x_edge[1];
        data->x_signal_buffer[4] = tilt_table_x_edge[5];
        data->x_signal_buffer[2] = tilt_table_x_edge[3];
        data->x_signal_buffer[1] = tilt_table_x_edge[2];
        data->x_signal_buffer[3] = tilt_table_x_edge[4];
    }
    else if (x_tilt_peak > 2 || data->x_coil_idx == 1)
    {
        if ((x_tilt_peak == 4 || x_tilt_peak == 5) && data->x_coil_idx != X_COIL_COUNT)
        {
            ++data->x_coil_idx;
            data->x_signal_buffer[0] = tilt_table_x_edge[2];
            data->x_signal_buffer[4] = tilt_table_x_edge[6];
            data->x_signal_buffer[2] = tilt_table_x_edge[4];
            data->x_signal_buffer[1] = tilt_table_x_edge[3];
            data->x_signal_buffer[3] = tilt_table_x_edge[5];
        }
        else
        {
            data->x_signal_buffer[0] = tilt_table_x_edge[1];
            data->x_signal_buffer[4] = tilt_table_x_edge[5];
            data->x_signal_buffer[2] = tilt_table_x_edge[3];
            data->x_signal_buffer[1] = tilt_table_x_edge[2];
            data->x_signal_buffer[3] = tilt_table_x_edge[4];
            __disable_irq();
            calculate_tilt_angle_x(tilt_scan_phase);
            __enable_irq();
        }
    }
    else
    {
        --data->x_coil_idx;
        data->x_signal_buffer[0] = tilt_table_x_edge[0];
        data->x_signal_buffer[4] = tilt_table_x_edge[4];
        data->x_signal_buffer[2] = tilt_table_x_edge[2];
        data->x_signal_buffer[1] = tilt_table_x_edge[1];
        data->x_signal_buffer[3] = tilt_table_x_edge[3];
    }
}

void get_pen_absolute_coords(pen_data* data)
{
#if USE_FPU == 1
    // @note: finish implementing so this uses the FPU rather than integer division (ew)
#else
    int y_fine_offset, y_coil_base_pos;
    int x_fine_offset, x_coil_base_pos;

    if (data->y_coil_idx == 1)
    {
        if (data->y_signal_buffer[2] > global_signal_threshold || data->y_signal_buffer[2] > (global_signal_threshold >> 1) && (data->x_coil_idx == X_COIL_COUNT || data->x_coil_idx == 1))
        {
            y_fine_offset = 0;
            y_coil_base_pos = 0;
        }
        else 
        {
            data->inputs.is_in_range = FALSE;
        }
    }
    else if (data->y_coil_idx == Y_COIL_COUNT)
    {
        if (data->y_signal_buffer[2] > global_signal_threshold || data->y_signal_buffer[2] > (global_signal_threshold >> 1) && (data->x_coil_idx == X_COIL_COUNT || data->x_coil_idx == 1))
        {
            y_fine_offset = 0;
            y_coil_base_pos = PEN_Y_RESOLUTION;
        }
        else 
        {
            data->inputs.is_in_range = FALSE;
        }
    }
    else if (data->y_coil_idx <= 1 || data->y_coil_idx >= Y_COIL_COUNT)
    {
        data->inputs.is_in_range = FALSE;
    }
    else if (data->y_signal_buffer[2] <= (global_signal_threshold >> 1))
    {
        data->inputs.is_in_range = FALSE;
    }
    else if (data->y_coil_idx == 2)
    {
        if (data->y_signal_buffer[3] >= data->y_signal_buffer[1])
        {
            if (data->y_signal_buffer[2] >= data->y_signal_buffer[3])
            {
                if (data->y_signal_buffer[2] >= data->y_signal_buffer[1])
                {
                    int parabola = calc_offset_parabolic(data->y_signal_buffer, PEN_Y_PITCH);
                    y_fine_offset = (parabola + calc_offset_linear(data->y_signal_buffer, PEN_Y_PITCH)) / 2;
                }
                else
                {
                    y_fine_offset = -150;
                }
            }
            else
            {
                y_fine_offset = 1540;
            }
        }
        else
        {
            y_fine_offset = calc_offset_linear(data->y_signal_buffer, PEN_Y_PITCH);
        }
        y_coil_base_pos = 0;
    }
    else if (data->y_coil_idx == Y_COIL_COUNT - 1)
    {
        if (data->y_signal_buffer[3] <= data->y_signal_buffer[1])
        {
            if (data->y_signal_buffer[2] >= data->y_signal_buffer[3])
            {
                if (data->y_signal_buffer[2] >= data->y_signal_buffer[1])
                {
                    int parabola = calc_offset_parabolic(data->y_signal_buffer, PEN_Y_PITCH);
                    y_fine_offset = (parabola + calc_offset_linear(data->y_signal_buffer, PEN_Y_PITCH)) / 2;
                }
                else
                {
                    y_fine_offset = -150;
                }
            }
            else
            {
                y_fine_offset = PEN_Y_PITCH + 150;
            }
        }
        else
        {
            y_fine_offset = calc_offset_linear(data->y_signal_buffer, PEN_Y_PITCH);
        }
        y_coil_base_pos = PEN_Y_PITCH * (data->y_coil_idx - 3) + PEN_Y_PITCH;
    }
    else
    {
        if (data->y_signal_buffer[1] <= data->y_signal_buffer[2])
        {
            if (data->y_signal_buffer[3] <= data->y_signal_buffer[2])
            {
                int parabola = calc_offset_parabolic(data->y_signal_buffer, PEN_Y_PITCH);
                y_fine_offset = (parabola + calc_offset_linear(data->y_signal_buffer, PEN_Y_PITCH)) / 2;
            }
            else
            {
                y_fine_offset = PEN_Y_PITCH + 150;
            }
        }
        else
        {
            y_fine_offset = -150;
        }
        y_coil_base_pos = PEN_Y_PITCH * (data->y_coil_idx - 3) + PEN_Y_PITCH;
    }

    if (data->x_coil_idx == 1)
    {
        if (data->x_signal_buffer[2] > global_signal_threshold || data->x_signal_buffer[2] > (global_signal_threshold >> 1) && (data->y_coil_idx == Y_COIL_COUNT || data->y_coil_idx == 1))
        {
            x_fine_offset = 0;
            x_coil_base_pos = 0;
        }
        else 
        {
            data->inputs.is_in_range = FALSE;
        }
    }
    else if (data->x_coil_idx == X_COIL_COUNT)
    {
        if (data->x_signal_buffer[2] > global_signal_threshold || data->x_signal_buffer[2] > (global_signal_threshold >> 1) && (data->y_coil_idx == Y_COIL_COUNT || data->y_coil_idx == 1))
        {
            x_fine_offset = 0;
            x_coil_base_pos = PEN_X_RESOLUTION;
        }
        else 
        {
            data->inputs.is_in_range = FALSE;
        }
    }
    else if (data->x_coil_idx > 1 && data->x_coil_idx < X_COIL_COUNT)
    {
        if (data->x_signal_buffer[2] <= (global_signal_threshold >> 1))
        {
            data->inputs.is_in_range = FALSE;
        }
        else if (data->x_coil_idx == 2)
        {
            x_coil_base_pos = 0;
            if (data->x_signal_buffer[3] >= data->x_signal_buffer[1])
            {
                if (data->x_signal_buffer[2] >= data->x_signal_buffer[3])
                {
                    if (data->x_signal_buffer[2] >= data->x_signal_buffer[1])
                    {
                        int parabola = calc_offset_parabolic(data->x_signal_buffer, PEN_X_PITCH + PEN_X_OVERSCAN);
                        x_fine_offset = (parabola + calc_offset_linear(data->y_signal_buffer, PEN_X_PITCH + PEN_X_OVERSCAN)) / 2;
                    }
                    else
                    {
                        x_fine_offset = -150;
                    }
                }
                else
                {
                    x_fine_offset = PEN_X_PITCH + 150;
                }
            }
            else
            {
                x_fine_offset = calc_offset_linear(data->x_signal_buffer, PEN_X_PITCH + PEN_X_OVERSCAN);
            }
        }
        else if (data->x_coil_idx == X_COIL_COUNT - 1)
        {
            if (data->x_signal_buffer[3] <= data->x_signal_buffer[1])
            {
                if (data->x_signal_buffer[2] >= data->x_signal_buffer[3])
                {
                    if (data->x_signal_buffer[2] >= data->x_signal_buffer[1])
                    {
                        int parabola = calc_offset_parabolic(data->x_signal_buffer, PEN_X_PITCH);
                        x_fine_offset = (parabola + calc_offset_linear(data->y_signal_buffer, PEN_X_PITCH)) / 2;
                    }
                    else
                    {
                        x_fine_offset = -150;
                    }
                }
                else
                {
                    x_fine_offset = PEN_X_PITCH + 150;
                }
            }
            else
            {
                x_fine_offset = calc_offset_linear(data->x_signal_buffer, PEN_X_PITCH);
            }
            x_coil_base_pos = PEN_X_PITCH * (data->x_coil_idx - 3) + PEN_X_PITCH + PEN_X_OVERSCAN;
        }
        else
        {
            if (data->x_signal_buffer[1] <= data->x_signal_buffer[2])
            {
                if (data->x_signal_buffer[3] <= data->x_signal_buffer[2])
                {
                    int parabola = calc_offset_parabolic(data->x_signal_buffer, PEN_X_PITCH);
                    x_fine_offset = (parabola + calc_offset_linear(data->y_signal_buffer, PEN_X_PITCH)) / 2;
                }
                else
                {
                    x_fine_offset = PEN_X_PITCH + 150;
                }
            }
            else
            {
                x_fine_offset = -150;
            }
            x_coil_base_pos = PEN_X_PITCH * (data->x_coil_idx - 3) + PEN_X_PITCH + PEN_X_OVERSCAN;
        }
    }

    if (data->inputs.is_in_range)
    {
        data->inputs.pen_y_pos = y_coil_base_pos + y_fine_offset;
        data->inputs.pen_x_pos = x_coil_base_pos + x_fine_offset;

        if (data->y_signal_buffer[1] <= data->y_signal_buffer[2])
        {
            if (data->y_signal_buffer[3] <= data->y_signal_buffer[2])
            {
                data->y_coil_idx = data->y_coil_idx;
            }
            else if (data->y_coil_idx == Y_COIL_COUNT)
            {
                data->y_coil_idx = data->y_coil_idx;
            }
            else
            {
                ++data->y_coil_idx;
            }
        }
        else if (data->y_coil_idx == 1)
        {
            data->y_coil_idx = data->y_coil_idx;
        }
        else
        {
            --data->y_coil_idx;
        }

        if (data->x_signal_buffer[1] <= data->x_signal_buffer[2])
        {
            if (data->x_signal_buffer[3] <= data->x_signal_buffer[2])
            {
                data->x_coil_idx = data->x_coil_idx;
            }
            else if (data->x_coil_idx == X_COIL_COUNT)
            {
                data->x_coil_idx = data->x_coil_idx;
            }
            else
            {
                ++data->x_coil_idx;
            }
        }
        else if (data->x_coil_idx == 1)
        {
            data->x_coil_idx = data->x_coil_idx;
        }
        else
        {
            --data->x_coil_idx;
        }
    }
    else
    {
        data->inputs.is_in_range = FALSE;
        data->inputs.pen_x_pos = 0;
        data->inputs.pen_y_pos = 0;
    }
#endif
}

void read_pen_data(pen_data* data, multiplexer_cfg_t* y_mux_data, multiplexer_cfg_t* x_mux_data)
{
    int y_search_buffer[Y_COIL_COUNT + 1];
    int x_search_buffer[X_COIL_COUNT + 1];

    // tracking mode    
    if (data->inputs.is_in_range)
    {
        sample_pen_data(data, &pen_sample);
        get_pen_absolute_coords(data);
        do_input_processing(data);
    }
    // search mode
    else
    {
        // @note: unknown function, manually inlined
        gpio_bit_reset(GPIOC, GPIO_PIN_15);
        
        pen_sample.tip_state = TIP_STATE_HOVER;
        __disable_irq();
        data->y_coil_idx = scan_frequency_spectrum(Y_COIL_COUNT, y_search_buffer, y_mux_data, current_coil_group);
        __enable_irq();

        if (data->y_coil_idx)
        {
            pen_sample.active_coil_idx = current_coil_group;

            __disable_irq();
            data->x_coil_idx = scan_frequency_spectrum(X_COIL_COUNT, x_search_buffer, x_mux_data, current_coil_group);
            __enable_irq();

            if (data->x_coil_idx)
            {
                data->inputs.is_in_range = TRUE;
            }
            else
            {
                data->inputs.is_in_range = FALSE;
            }
        }
        else
        {
            data->inputs.is_in_range = FALSE;

            if (current_coil_group >= 10)
            {
                current_coil_group = 3;
            }
            else
            {
                ++current_coil_group;
            }
        }
    }
}

void do_sample(void)
{
    read_pen_data(&pen, y_mux_search_data, x_mux_search_data);
    // @note: handle express buttons
    // @note: handle touch ring
}