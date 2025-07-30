#include <iostream>
#include <cmath>
#include <modbus/modbus.h>

float cal_theta2(float x, float y) {
    return atan2(y, x);
}

bool send_theta(modbus_t *ctx, int slave_id, int reg_addr, float theta_rad) {
    int scaled_value = static_cast<int>(theta_rad * 10);

    if (scaled_value < 0 || scaled_value > 0xFFFF) {
        std::cerr << "❌ Scaled angle out of range: " << scaled_value << std::endl;
        return false;
    }

    // Set the Modbus slave ID
    if (modbus_set_slave(ctx, slave_id) == -1) {
        std::cerr << "❌ Failed to set slave ID: " << modbus_strerror(errno) << std::endl;
        return false;
    }

    // Send the value to the register
    int rc = modbus_write_register(ctx, reg_addr, scaled_value);
    if (rc == -1) {
        std::cerr << "❌ Failed to write register: " << modbus_strerror(errno) << std::endl;
        return false;
    }

    std::cout << "✅ Sent " << theta_rad << " rad as scaled value " << scaled_value << std::endl;
    return true;
}

int main() {
    // Create Modbus RTU context
    modbus_t *ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);
    if (ctx == nullptr) {
        std::cerr << "❌ Unable to create Modbus context\n";
        return 1;
    }

    if (modbus_connect(ctx) == -1) {
        std::cerr << "❌ Connection failed: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        return 1;
    }

    int slave_id = 1;
    int register_addr = 0;

    // Example input
    float x = 1.0;
    float y = 1.0;

    float theta = cal_theta2(x, y);
    std::cout << "Calculated theta (rad): " << theta << std::endl;

    send_theta(ctx, slave_id, register_addr, theta);

    // Close and free Modbus
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}
