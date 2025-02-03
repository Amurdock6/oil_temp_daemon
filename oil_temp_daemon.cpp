#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <limits>

// ADS1115 I2C address
#define ADS1115_ADDRESS 0x48

// Registers
#define REG_CONVERSION 0x00
#define REG_CONFIG     0x01

static bool g_shouldStop = false;

// Signal handler to cleanly exit
void handle_signal(int signal) {
    std::cerr << "[daemon] Caught signal " << signal << ", stopping...\n";
    g_shouldStop = true;
}

// ---------------------------------------------------------
// 1) Open I2C bus and set up communication
// ---------------------------------------------------------
int initialize_i2c(const char* device, int address) {
    int file = open(device, O_RDWR);
    if (file < 0) {
        std::cerr << "[daemon] Failed to open I2C bus: " << device << "\n";
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, address) < 0) {
        std::cerr << "[daemon] Failed to talk to slave at 0x"
                  << std::hex << address << "\n";
        close(file);
        return -1;
    }
    return file;
}

// ---------------------------------------------------------
// 2) Configure ADS1115 (continuous mode, single-ended on A1, ±4.096V, 128SPS)
// ---------------------------------------------------------
bool configure_ads1115(int fd) {
    // Example config: AIN0 single-ended, +/-4.096V, continuous mode, 128 SPS
    //   High byte: 1100 0011 = 0xC3
    //   Low byte:  1000 0011 = 0x83
    uint8_t config[3];
    config[0] = REG_CONFIG;
    config[1] = 0xC3;
    config[2] = 0x83;

    if (write(fd, config, 3) != 3) {
        std::cerr << "[daemon] Failed to write ADS1115 config\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------
// 3) Read ADS1115 raw 16-bit ADC value
// ---------------------------------------------------------
int16_t read_ads1115(int fd) {
    uint8_t reg = REG_CONVERSION;
    if (write(fd, &reg, 1) != 1) {
        std::cerr << "[daemon] Failed to set ADS1115 pointer\n";
        return 0;
    }
    uint8_t data[2];
    if (read(fd, data, 2) != 2) {
        std::cerr << "[daemon] Failed to read ADS1115 conversion data\n";
        return 0;
    }
    int16_t raw = (data[0] << 8) | data[1];
    return raw;
}

// ---------------------------------------------------------
// 4) Convert raw ADC -> voltage
//    With ±4.096 V range => 32768 counts = 4.096 V
// ---------------------------------------------------------
double adc_to_voltage(int16_t raw) {
    return (static_cast<double>(raw) * 4.096) / 32768.0;
}

// ---------------------------------------------------------
// 5) Compute sensor resistance from measured voltage
//    R_sensor = R_pullup * (V_meas / (V_supply - V_meas))
//
//    Returns 0.0 if voltage < 0
//    Returns INFINITY if V_meas ~ V_supply
// ---------------------------------------------------------
double compute_sensor_resistance(double voltage, double r_pullup, double v_supply) {
    // If voltage is near supply, or negative, handle edge cases
    if (voltage >= (v_supply - 0.01)) {
        return std::numeric_limits<double>::infinity();
    } else if (voltage < 0.0) {
        return 0.0;
    }
    return r_pullup * (voltage / (v_supply - voltage));
}

// ---------------------------------------------------------
// 6) VDO table (temp_F, ohms). 
//    Sensor R decreases as temperature increases.
// ---------------------------------------------------------
static const std::vector<std::pair<double,double>> vdo_table = {
    {130, 360.0}, {132, 341.9}, {149, 258.0}, {158, 219.4},
    {159.7, 200.0}, {161, 185.8}, {171, 164.5}, {179.5, 140.6},
    {180, 139.1},  {187, 123.3}, {190, 117.8}, {197, 102.2},
    {200, 100.9},  {209, 87.9},  {210, 86.3}
};

// ---------------------------------------------------------
// 7) Interpolate R -> Temp (°F) using the VDO table
//
//    If R > 360 => below min temp (we return a sentinel, e.g. 100.0 or 0.0)
//    If R < 86.3 => above max temp (e.g. 250 or 300.0)
//    Otherwise do piecewise linear interpolation
//
//    Return NaN if something is off, though we'll try to avoid that with
//    sentinel logic for extremes.
// ---------------------------------------------------------
double resistance_to_temp_f(double R) {
    // Largest ohms (lowest temp) is table.front().second = 360 ohms => ~130°F
    // Smallest ohms (highest temp) is table.back().second = 86.3 ohms => ~210°F

    // If R is bigger than the coldest ohms => "colder than table"
    if (R > vdo_table.front().second) {
        // Could return something like 100°F or 0°F 
        // to indicate "too cold / out of range"
        return std::numeric_limits<double>::quiet_NaN();
    }

    // If R is smaller than the hottest ohms => "hotter than table"
    if (R < vdo_table.back().second) {
        // Could return something like 250°F or 300°F
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Now do piecewise interpolation across the table
    for (size_t i = 0; i < vdo_table.size() - 1; ++i) {
        double t1 = vdo_table[i].first;
        double r1 = vdo_table[i].second;
        double t2 = vdo_table[i+1].first;
        double r2 = vdo_table[i+1].second;

        // r1 >= R >= r2
        if (r1 >= R && R >= r2) {
            double fraction = (R - r1) / (r2 - r1);  // between 0 and 1
            return t1 + fraction * (t2 - t1);
        }
    }
    // If we somehow fall through, return NaN
    return std::numeric_limits<double>::quiet_NaN();
}

// ---------------------------------------------------------
// 8) Write temperature to file
// ---------------------------------------------------------
bool write_temperature(const std::string& filepath, double temperature) {
    std::ofstream outfile(filepath, std::ios::out | std::ios::trunc);
    if (!outfile.is_open()) {
        std::cerr << "[daemon] Could not open file for writing: " << filepath << "\n";
        return false;
    }
    outfile << temperature;
    outfile.close();
    return true;
}

// ---------------------------------------------------------
// 9) Main: continuously read from ADS1115, compute temp, write to file
// ---------------------------------------------------------
int main(int argc, char* argv[]) {
    // Handle Ctrl-C, kill signals
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // 9a) Open & configure ADS1115
    const char* i2c_device = "/dev/i2c-1";
    int fd = initialize_i2c(i2c_device, ADS1115_ADDRESS);
    if (fd < 0) {
        // Already printed an error
        return 1;
    }
    if (!configure_ads1115(fd)) {
        close(fd);
        return 1;
    }

    // 9b) Setup known resistor & supply voltage
    const double R_PULLUP = 330.0; // ohms
    const double V_SUPPLY = 3.3;   // volts

    // 9c) Where to write our file
    const std::string output_file = "/data/local/tmp/oil_temp.txt";

    // 9d) Main loop
    while (!g_shouldStop) {
        // 1) Read raw ADC
        int16_t raw_adc = read_ads1115(fd);
        double voltage = adc_to_voltage(raw_adc);

        // 2) Convert to resistance
        double R_sensor = compute_sensor_resistance(voltage, R_PULLUP, V_SUPPLY);

        // 3) Interpret R -> temperature
        double temp_f = resistance_to_temp_f(R_sensor);

        // 4) Logging / error handling
        if (std::isinf(R_sensor)) {
            // Probably sensor is open or not connected
            std::cout << "[daemon] Voltage=" << voltage 
                      << " => R=inf (sensor open?) => writing NaN\n";
            temp_f = std::numeric_limits<double>::quiet_NaN();
        } else if (R_sensor == 0.0) {
            // Possibly shorted to ground or negative reading
            std::cout << "[daemon] Voltage=" << voltage 
                      << " => R=0 (short/negative?) => writing NaN\n";
            temp_f = std::numeric_limits<double>::quiet_NaN();
        } else if (std::isnan(temp_f)) {
            // We ended up out of table range
            if (R_sensor > vdo_table.front().second) {
                // colder than minimum
                std::cout << "[daemon] Voltage=" << voltage
                          << "  R=" << R_sensor 
                          << " => below table min => writing NaN\n";
            } else if (R_sensor < vdo_table.back().second) {
                // hotter than maximum
                std::cout << "[daemon] Voltage=" << voltage
                          << "  R=" << R_sensor
                          << " => above table max => writing NaN\n";
            } else {
                // Some other weird case
                std::cout << "[daemon] Voltage=" << voltage
                          << "  R=" << R_sensor
                          << " => can't interpolate => writing NaN\n";
            }
        } else {
            // Normal case
            std::cout << "[daemon] Voltage=" << voltage
                      << "  R=" << R_sensor
                      << " => " << temp_f << " °F\n";
        }

        // 5) Write the temperature (NaN or real value) to file
        if (!write_temperature(output_file, temp_f)) {
            std::cerr << "[daemon] Error writing temperature to " 
                      << output_file << "\n";
        }

        // Sleep 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Cleanup
    close(fd);
    std::cout << "[daemon] Exiting cleanly.\n";
    return 0;
}
