#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <unistd.h>
#include <hidapi/hidapi.h>
#include <time.h>

#define MAX_STR 255
#define REPORT_SIZE 65
#define SIGNATURE 0x55

// Function to get current time in milliseconds
long long current_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

int main(int argc, char* argv[])
{
    int res;
    unsigned char buf[REPORT_SIZE];
    hid_device *handle;
    wchar_t wstr[MAX_STR];

    // Initialize the hidapi library
    res = hid_init();
    printf("hid_init result: %d\n", res);

    // Open the device using the VID, PID (VID: 2563, PID: 0526)
    handle = hid_open(0x2563, 0x0526, NULL);  
    if (!handle) {
        printf("Unable to open device\n");
        return 1;
    }

    // Read the Manufacturer String
    res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
    printf("Manufacturer String: %ls\n", wstr);

    // Read the Product String
    res = hid_get_product_string(handle, wstr, MAX_STR);
    printf("Product String: %ls\n", wstr);

    // Read the Serial Number String
    res = hid_get_serial_number_string(handle, wstr, MAX_STR);
    printf("Serial Number String: %ls\n", wstr);

    const int iterations = 10000;
    const long long interval_ms = 10; // 10ms for 100Hz

    for (int i = 0; i < iterations; ++i) {
        long long start_time = current_time_ms();

        printf("Iteration %3d - Buttons: ", i + 1);

        // Read the gamepad input (button states)
        res = hid_read_timeout(handle, buf, sizeof(buf), 100);
        if (res > 0) {
            // Assuming buttons are in the first byte or so
            for (int j = 0; j < 16; ++j) {  // Checking first 8 buttons (modify based on your device)
                if (buf[0] & (1 << j)) {
                    printf("Button %d pressed ", j + 1);
                }
            }

            // Check for joystick (you can adapt this based on how joysticks are reported)
            if (buf[1] > 128) {
                printf("Left Joystick X Axis: %d ", buf[1] - 128);  // Adjust axis range as necessary
            }
            if (buf[2] > 128) {
                printf("Left Joystick Y Axis: %d ", buf[2] - 128);  // Adjust axis range as necessary
            }

            printf("\n");
        } else if (res == 0) {
            printf("TIMEOUT\n");
        } else {
            printf("ERROR\n");
        }

        long long end_time = current_time_ms();
        long long duration = end_time - start_time;

        if (duration < interval_ms) {
            usleep((interval_ms - duration) * 1000);
        }
    }

    // Close the device
    hid_close(handle);

    // Finalize the hidapi library
    res = hid_exit();

    printf("Press Enter to exit...");
    getchar();

    return 0;
}
