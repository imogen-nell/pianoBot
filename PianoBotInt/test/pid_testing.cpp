#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
// #include "Logger.h"

// using namespace std::chrono_literals;

// class HvacPreconditioningTest {
// public:
//     void run() {
//         Logger::info("=== Starting HVAC Preconditioning Test @ -20°C ===");

//         // 1. Setup simulated environment
//         SensorSimulator sensors;
//         sensors.setAmbientTemperature(-20.0);
//         sensors.setCabinTemperature(-20.0);

//         HvacEcu hvac;
//         CanInterface canBus;
//         hvac.connect(canBus);

//         // 2. Power on ECU and wait for init
//         hvac.powerOn();
//         std::this_thread::sleep_for(2s);

//         // 3. Send remote start command
//         canBus.sendFrame({0x200, {0x01}}); // 0x200 = Preconditioning request
//         Logger::info("Sent preconditioning request.");

//         // 4. Monitor response for 20 minutes (simulated)
//         bool targetReached = false;
//         for (int minute = 0; minute < 20; ++minute) {
//             hvac.update();
//             sensors.feedback(hvac);

//             double cabinTemp = sensors.getCabinTemperature();
//             Logger::data("Minute " + std::to_string(minute),
//                          "CabinTemp=" + std::to_string(cabinTemp));

//             if (cabinTemp >= 22.0) {
//                 Logger::pass("Target temperature reached at " +
//                              std::to_string(minute) + " minutes.");
//                 targetReached = true;
//                 break;
//             }

//             std::this_thread::sleep_for(1s); // time compression
//         }

//         // 5. Verify final state
//         if (!targetReached) Logger::fail("Did not reach target temperature in 20 minutes.");

//         // 6. Check CAN status message
//         auto status = canBus.readFrame(0x3A0);
//         if (status.id == 0x3A0 && (status.data[0] & 0x01))
//             Logger::pass("HVAC status ACTIVE confirmed.");
//         else
//             Logger::fail("HVAC status message missing or incorrect.");

//         Logger::info("=== Test Complete ===");
//     }
// };

// int main() {
//     HvacPreconditioningTest test;
//     test.run();
//     return 0;
// }
