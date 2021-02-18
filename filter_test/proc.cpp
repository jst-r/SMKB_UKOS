//#include "proc.h"

#include <fstream>
#include <iostream>

using namespace std;

#define buff_size 500
#define time_window 100  // ms
#define sample_time 1   // ms


#define n_delays 5

const unsigned char delays[] = {1, 2, 3, 4};
unsigned char buff[buff_size];
unsigned char mask[buff_size];

static const int threshold = 15;

void init_mask() {
    for (int i = 0; i < buff_size; i++) {
        mask[i] = 0;
        buff[i] = 0;
    }

    mask[0] = 1;
    mask[1] = 1;
    mask[2] = 1;
    int t = 1;
    for (int i = 0; i < n_delays; i++) {
        t += delays[i] * 1000 / time_window;
        mask[t - 1] = 1;
        mask[t] = 1;
        mask[t + 1] = 1;
    }
}

double matched_filter() {
    double max_score = 0;
    double score;
    for (int offset = 0; offset < buff_size; offset++) {
        score = 0;
        for (int i = 0; i < buff_size; i++) {
            score += mask[i] * buff[(i + offset) % buff_size];
        }
        max_score = max(max_score, score);
    }
    return max_score;
}

int main() {

    int n;
    init_mask();
    for (int i = 0; i < buff_size; i++) cout << mask[i] << " "; cout << endl;

    ifstream fin("dat.txt");

    int i = 0;
    int t = 0;  // ms

    unsigned char v;

    unsigned char max_val;

    for (int i = 0; i < buff_size; i++) {
        max_val = 0;
        for (int j = 0; j < time_window / sample_time; j++) {
            fin >> v;
            // procesed_val = procesed_val + alpha * (v - procesed_val);
            max_val = max(max_val, v);
        }
        buff[i] = max_val;
    }

    // for (int i = 0; i < buff_size; i++) cout << (int)buff[i] << " "; cout <<
    // endl;

    cout << matched_filter() << endl;
}