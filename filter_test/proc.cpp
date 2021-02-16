#include <fstream>
#include <iostream>

#include "proc.h"

using namespace std;

static const int sig_len = 100;
static const int intervals[] = {1000, 1000, 2000, 2000};
static const int max_t_err = 100;
static const int min_right_intervals = 3;

void listen(int* sig, int& n){
    sig
}

int process_activations(int act[], int n_act) {
    int errors = 0;
    
    int curr_interval = 0;

    for (int i = 0; i < n_act - 1; i++) {
        int dt = 0;
        int j = i;
        while (j < n_act && dt < intervals[curr_interval] - max_t_err) {
            j++;
            dt += act[j] - act[j - 1];
        }
        if (j == n_act){
            //TODO fallback for end case
        }
        if (abs(dt - intervals[curr_interval]) < max_t_err){
            curr_interval++;
        }
        cout << dt << endl;
    }
    return 0;
}

int main() {
    int* sig;
    int n;
    listen

    ifstream fin("dat.txt");

    int i = 0;
    double v;
    double procesed_val = 0;
    double alpha = .01;

    int n_activations = 0;
    int activations[100];

    int prev_time;

    while (fin >> v) {
        procesed_val = procesed_val + alpha * (v - procesed_val);
        if (i % (sig_len / 2) == 0) {
            //cout << i << " " << procesed_val << endl;
            if (procesed_val > 20) {
                cout << i << " " << procesed_val << endl;
                activations[n_activations++] = i;
            }
        }
        i++;
    }

    for (int i = 0; i < n_activations; i++) cout << activations[i] << " ";
    cout << endl;
    process_activations(activations, n_activations);
}