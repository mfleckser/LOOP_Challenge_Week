#include <stdio.h>

#define MAX 0xffffffff

typedef __uint32_t uint32;

int decode_int(int *);
void HAL_TIM_IC_CaptureCallback(int, int);
void encode_int(int, int *);
void add_to_array(int, int, int **);
void print_arr(int *, int);
void print_encoded(int);

int prev_time = 0;
int is_first_value = 1;
int is_counting = 0;
int bit_index = 0;
int bits[5];


int main() {
    /* ----- Encoding ----- */
    print_encoded(6);
    print_encoded(14);
    print_encoded(0);
    print_encoded(9);


    /* ----- Decoding ----- */
    int i;
    // 1 00 11111 (0) (1) (0) (1) (1) 11111 00 (0) (1) (1) (0)
    int times[] = {1,5,1,5,1,5,1,5};
    int starting_value = 0;
    int counter = 0;

    for (i = 0; i < sizeof(times) / sizeof(times[0]); i++) {
        counter += times[i];
        HAL_TIM_IC_CaptureCallback(counter, (i + starting_value) % 2);
    }


    return 0;
}


void print_encoded(int n) {
    int digits[15];
    encode_int(n, digits);
    print_arr(digits, 15);
    printf("\n");
}


void encode_int(int n, int *bits) {
    int digit, i;
    int filler = 100;
    int *start = bits;

    /* High timing period and leading 0 bit */
    add_to_array(filler, 5, &bits);
    add_to_array(0, 2, &bits);

    /* Add binary digits (stored as 5's and 0's), n < 16 so only 4 digits */
    for (i = 0; i < 4; i++) {
        digit = filler * (n & 1);
        n = n >> 1;
        add_to_array(digit, 2, &bits);
    }

    /* Decrement trailing high values */
    if (filler - 1 && 0) {
        for (i = 0; i < bits - start; i++) {
            int next = start[(i + 1) % (bits - start - 1)];
            if (!next && start[i]) {
                start[i]--;
            }
        }
    }
}


void add_to_array(int n, int count, int **arr) {
    int i;

    for (i = 0; i < count; i++) {
        **arr = n;
        (*arr)++;
    }
}


void print_arr(int *arr, int size) {
    int i;

    for (i = 0; i < size; i++) {
        printf("%d, ", arr[i]);
    }
}


void HAL_TIM_IC_CaptureCallback(int curr_time, int group_val) {
    if (is_first_value) { // only for very first value read
        prev_time = curr_time;
        is_first_value = 0;
    } else {
        int dif;

        // find dif (delta_t * 5) and update prev_time
        // (dif * 5) ms since last edge
        if (curr_time > prev_time) {
            dif = curr_time - prev_time;
        } else {
            dif = (MAX - prev_time) + curr_time;
        }

        prev_time = curr_time;

        if (!is_counting) { // watch for 25ms high state timing packet
            if (dif % 2 == 1) { // odd length packet signifies timing packet
                is_counting = 1;
            }
        } else { // counting
            while (bit_index < 5 && dif > 0) {
                bits[bit_index] = group_val;
                bit_index++;
                dif -= 2;
            }

            if (bit_index == 5) {
                bit_index = 0;
                is_counting = dif > 0;
                printf("%d\n", decode_int(bits));
            }

        }
    }
}

int decode_int(int *bits) {
	return bits[1] + 2 * bits[2] + 4 * bits[3] + 8 * bits[4];
}