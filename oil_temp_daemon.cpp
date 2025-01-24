#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>

// ADS1115 I2C address (0x48 by default)
#define ADS1115_ADDRESS 0x48

// ADS1115 Register Addresses
#define REG_POINTER 0x00
#define REG_CONVERSION 0x00
#define REG_CONFIG 0x01

// Function to initialize I2C communication
int initialize_i2c(const char* device, int address) {
    int file;
    if ((file = open(device, O_RDWR)) < 0) {
        std::cerr << "Failed to open the I2C bus\n";
        return -1;
    }
    if (ioctl(file, I2C_SLAVE, address) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave\n";
        close(file);
        return -1;
    }
    return file;
}

// Function to configure ADS1115
bool configure_ads1115(int file) {
    // Configuration for single-ended input on A0, continuous conversion, 4.096V FSR, 128 SPS
    // Data sheet reference: https://www.ti.com/lit/ds/symlink/ads1115.pdf
    uint8_t config[3];
    config[0] = REG_CONFIG;
    config[1] = 0xC3; // 11000011: OS=1 (start single conversion), MUX=100 (AIN0 single-ended), PGA=001 (-4.096V to +4.096V), MODE=1 (continuous)
    config[2] = 0x83; // DR=100 (128 SPS), COMP_MODE=1, COMP_POL=1, COMP_LAT=0, COMP_QUE=11 (disabled)

    if (write(file, config, 3) != 3) {
        std::cerr << "Failed to write configuration to ADS1115\n";
        return false;
    }
    return true;
}

// Function to read ADC value
int16_t read_ads1115(int file) {
    uint8_t reg = REG_CONVERSION;
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Failed to write to ADS1115 conversion register\n";
        return 0;
    }

    // ADS1115 continuous conversion mode updates the conversion register automatically
    uint8_t data[2];
    if (read(file, data, 2) != 2) {
        std::cerr << "Failed to read conversion data\n";
        return 0;
    }

    // Convert the two bytes to a single 16-bit value
    int16_t raw = (data[0] << 8) | data[1];
    return raw;
}

// Function to convert raw ADC value to voltage
double adc_to_voltage(int16_t adc_val) {
    // ADS1115 is a 16-bit ADC, with a range of +/-4.096V (configured)
    // Calculate voltage based on ADS1115 configuration
    // Formula: Voltage = (ADC_VALUE * FSR) / 32768
    double voltage = (adc_val * 4.096) / 32768.0;
    return voltage;
}

// Function to calculate sensor resistance from voltage divider
double calculate_resistance(double voltage, double R_fixed) {
    // Voltage divider formula: V_out = V_in * (R_sensor / (R_sensor + R_fixed))
    // Rearranged: R_sensor = (V_out * R_fixed) / (V_in - V_out)
    double R_sensor = (voltage * R_fixed) / (3.3 - voltage);
    return R_sensor;
}

// Function to map resistance to temperature (Fahrenheit)
// Replace this with your actual resistance-temperature table
double resistance_to_temperature(double R_sensor) {
    // Example linear approximation based on sensor specs
    // VDO 323-057: 322 Ω at 120°F and 18.6 Ω at 300°F
    // Assuming linearity between these points (for simplicity)
    double temp;
    if (R_sensor >= 322.0) {
        temp = 120.0;
    }
    else if (R_sensor <= 18.6) {
        temp = 300.0;
    }
    else {
        // Linear interpolation
        temp = ((R_sensor - 322.0) / (18.6 - 322.0)) * (300.0 - 120.0) + 120.0;
    }
    return temp;
}

// Function to write temperature to file
bool write_temperature(const std::string& filepath, double temperature) {
    std::ofstream outfile(filepath, std::ios::out | std::ios::trunc);
    if (!outfile.is_open()) {
        std::cerr << "Failed to open file for writing: " << filepath << "\n";
        return false;
    }
    outfile << temperature;
    outfile.close();
    return true;
}

int main() {
    const char* i2c_device = "/dev/i2c-1";
    int ads1115_address = ADS1115_ADDRESS;
    int i2c_file = initialize_i2c(i2c_device, ads1115_address);
    if (i2c_file < 0) {
        return 1;
    }

    if (!configure_ads1115(i2c_file)) {
        close(i2c_file);
        return 1;
    }

    const double R_fixed = 100.0; // 100-ohm resistor
    const std::string output_file = "/data/local/tmp/oil_temp";

    while (true) {
        int16_t adc_val = read_ads1115(i2c_file);
        double voltage = adc_to_voltage(adc_val);
        double R_sensor = calculate_resistance(voltage, R_fixed);
        double temperature = resistance_to_temperature(R_sensor);

        if (!write_temperature(output_file, temperature)) {
            std::cerr << "Error writing temperature to file\n";
        }
        else {
            std::cout << "Temperature: " << temperature << "°F\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 500ms delay
    }

    close(i2c_file);
    return 0;
}
