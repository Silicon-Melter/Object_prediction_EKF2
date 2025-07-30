#include <iostream>
#include <cmath>
#include <modbus/modbus.h>
#include <windows.h>

float cal_theta2(float x, float y) {
    return atan2f(y, x); 
}

bool send_theta(modbus_t *ctx, int slave_id, int reg_addr, float theta_rad) {
    int scaled_value = static_cast<int>(theta_rad * 10);

    if (scaled_value < 0 || scaled_value > 0xFFFF) {
        std::cerr << "❌ Scaled value out of range: " << scaled_value << std::endl;
        return false;
    }

    if (modbus_set_slave(ctx, slave_id) == -1) {
        std::cerr << "❌ Set slave failed: " << modbus_strerror(errno) << std::endl;
        return false;
    }

    int rc = modbus_write_register(ctx, reg_addr, scaled_value);
    if (rc == -1) {
        std::cerr << "❌ Write failed: " << modbus_strerror(errno) << std::endl;
        return false;
    }

    std::cout << "✅ Sent angle " << theta_rad << " rad as " << scaled_value << std::endl;
    return true;
}

int main() {
    // COM port format for Windows: "COM6"
    modbus_t *ctx = modbus_new_rtu("COM6", 9600, 'N', 8, 1);
    if (ctx == nullptr) {
        std::cerr << "❌ Cannot create Modbus context\n";
        return 1;
    }

    if (modbus_connect(ctx) == -1) {
        std::cerr << "❌ Modbus connection failed: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        return 1;
    }

    int slave_id = 1;
    int reg_addr = 0;

    // Example input values
    float x = 1.0f;
    float y = 1.0f;

    float theta = cal_theta2(x, y);
    std::cout << "Calculated angle: " << theta << " radians\n";

    send_theta(ctx, slave_id, reg_addr, theta);

    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
