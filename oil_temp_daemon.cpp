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
// 1) Open I2C and configure ADS1115
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

bool configure_ads1115(int fd) {
    // Example config: AIN0 single-ended, +/-4.096V, continuous mode, 128 SPS
    uint8_t config[3];
    config[0] = REG_CONFIG;
    // High byte
    config[1] = 0xC3; 
    // Low byte
    config[2] = 0x83; 

    if (write(fd, config, 3) != 3) {
        std::cerr << "[daemon] Failed to write ADS1115 config\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------
// 2) Read ADS1115 raw ADC
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
// 3) Convert raw ADC -> voltage
//    with +/- 4.096 V range => 32768 counts = 4.096 V
// ---------------------------------------------------------
double adc_to_voltage(int16_t raw) {
    return (static_cast<double>(raw) * 4.096) / 32768.0;
}

// ---------------------------------------------------------
// 4) Compute sensor resistance from measured voltage
//    R_sensor = R_pullup * (V_meas / (V_supply - V_meas))
// ---------------------------------------------------------
double compute_sensor_resistance(double voltage, double r_pullup, double v_supply) {
    // If voltage is near supply, or negative, handle edge cases:
    if (voltage >= (v_supply - 0.01)) {
        return INFINITY;
    } else if (voltage < 0.0) {
        return 0.0;
    }
    return r_pullup * (voltage / (v_supply - voltage));
}

// ---------------------------------------------------------
// 5) Table-based interpolation: (temp_F, ohms).
// ---------------------------------------------------------
static const std::vector<std::pair<double,double>> vdo_table = {
    {130, 360.0}, {132, 341.9}, {149, 258.0}, {158, 219.4},
    {159.7, 200.0}, {161, 185.8}, {171, 164.5}, {179.5, 140.6},
    {180, 139.1}, {187, 123.3}, {190, 117.8}, {197, 102.2},
    {200, 100.9}, {209, 87.9}, {210, 86.3}
};

// ---------------------------------------------------------
// 6) Interpolate R -> temp 
// ---------------------------------------------------------
double resistance_to_temp_f(double R, const std::vector<std::pair<double,double>>& table) {
    // table[i] = (tempF, ohms), in ascending temperature order
    // If R is larger than the ohms at the coldest entry => below the min temp
    if (R > table.front().second) {
        return NAN; // "colder than table"
    }
    // If R is smaller than the ohms at the hottest entry => above the max temp
    if (R < table.back().second) {
        return NAN; // "hotter than table"
    }

    // Search the table segments
    for (size_t i = 0; i < table.size() - 1; ++i) {
        double t1 = table[i].first;
        double r1 = table[i].second;
        double t2 = table[i+1].first;
        double r2 = table[i+1].second;

        // Check if R is between r1 and r2. 
        // Note that r1 > r2 because sensor R goes down as temp goes up.
        if (r1 >= R && R >= r2) {
            double fraction = (R - r1) / (r2 - r1);
            return t1 + fraction * (t2 - t1);
        }
    }
    // If we didn’t find it, R is out of table range
    return NAN;
}

// ---------------------------------------------------------
// 7) Write temperature to file
// ---------------------------------------------------------
bool write_temperature(const std::string& filepath, double temperature) {
    std::ofstream outfile(filepath, std::ios::out | std::ios::trunc);
    if (!outfile.is_open()) {
        std::cerr << "[daemon] Could not open file: " << filepath << "\n";
        return false;
    }
    outfile << temperature;
    outfile.close();
    return true;
}

// ---------------------------------------------------------
// 8) Main daemon entry
// ---------------------------------------------------------
int main(int argc, char* argv[]) {
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // 8a) Open and configure ADS1115
    const char* i2c_device = "/dev/i2c-1";
    int fd = initialize_i2c(i2c_device, ADS1115_ADDRESS);
    if (fd < 0) {
        return 1;
    }
    if (!configure_ads1115(fd)) {
        close(fd);
        return 1;
    }

    // 8b) Setup constants
    const double R_PULLUP = 330.0; // ohms
    const double V_SUPPLY = 3.3;   // volts

    // 8c) Output path for the temperature
    const std::string output_file = "/data/local/tmp/oil_temp";

    // 8d) Main loop
    while (!g_shouldStop) {
        // Read ADC, convert to voltage
        int16_t raw_adc = read_ads1115(fd);
        double voltage = adc_to_voltage(raw_adc);

        // Compute sensor R, then temperature
        double R_sensor = compute_sensor_resistance(voltage, R_PULLUP, V_SUPPLY);
        double temp_f   = resistance_to_temp_f(R_sensor, vdo_table);

        // Print/log
        if (std::isnan(temp_f)) {
            std::cout << "[daemon] Voltage=" << voltage
                      << "  R=" << R_sensor 
                      << " => out of range!\n";
        } else {
            std::cout << "[daemon] Voltage=" << voltage 
                      << "  R=" << R_sensor 
                      << " => " << temp_f << " °F\n";
        }

        // Write to file
        if (!write_temperature(output_file, temp_f)) {
            std::cerr << "[daemon] Error writing temperature\n";
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    close(fd);
    std::cout << "[daemon] Exiting cleanly.\n";
    return 0;
}
