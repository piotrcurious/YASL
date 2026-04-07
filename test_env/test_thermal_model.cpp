
#include "Arduino.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "Testing Non-Ideal Buck Converter Model with Thermal Drift\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Duty\tTempC\tVsolar\tIsolar(mA)\tIout(mA)\tVbat\n";
    std::cout << "------------------------------------------------------------\n";

    sim.batteryV = 3.5f;
    sim.solarOCV = 18.0f;
    sim.R_conv_base = 0.2f;

    float test_duties[] = {0.2f, 0.5f, 0.8f};
    float test_temps[] = {25.0f, 75.0f};

    for (float d : test_duties) {
        OCR1A = (int)(d * 1023.0f);
        for (float t : test_temps) {
            sim.tempC = t;
            // Run a few updates to let it stabilize if there's any state,
            // though our current model is instantaneous inside update_sim for V/I.
            update_sim();

            float duty = (float)OCR1A / 1023.0f;
            float Iout = sim.solarCurrentMA / duty;

            std::cout << duty << "\t"
                      << sim.tempC << "\t"
                      << sim.solarBusV << "\t"
                      << sim.solarCurrentMA << "\t\t"
                      << Iout << "\t\t"
                      << sim.batteryV << "\n";
        }
    }

    return 0;
}
