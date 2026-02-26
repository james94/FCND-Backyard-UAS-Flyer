// This shows the recommended Model B: telemetry thread updates
// the "latest sample", control thread runs "Tick()" at fixed rate.

// main.cpp wiring for the UdaciDrone bridge (BridgeVehicle)

// Once you implement "BridgeTransport" + "BridgeVehicle" (Section 122.4),
// the "placeholder" receiver thread becomes:

// - a thread inside "BridgeTransport" that reads telemetry JSONL from Python
// - a "BridgeVehicle" that sends command JSONL to Python

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "backyard/config.hpp"
#include "backyard/controller.hpp"
#include "backyard/telemetry.hpp"

#include "backyard/bridge_transport.hpp"
#include "backyard/vehicle.hpp"

// TODO: replace this with BridgeVehicle or MavsdkVehicle
// class DummyVehicle : public backyard::IVehicle {
// public:
//     void TakeControl() override {}
//     void ReleaseControl() override {}
//     void Arm() override {}
//     void Disarm() override {}
//     void Takeoff(double) override {}
//     void Land() override {}
//     void CmdPosition(const backyard::NED&, double) override {}
//     void Stop() override {}
// };

int main() {
    backyard::Config cfg;
    
    DummyVehicle vehicle;
    backyard::BackyardFlyerController controller(cfg, vehicle);

    std::mutex m;
    backyard::TelemetrySample latest;
    bool has_sample = false;
    std::atomic<bool> running{true};

    // Telemetry receiver thread (placeholder)
    std::thread rx([&]() {
        while (running.load()) {
            backyard::TelemetrySample s;
            // TODO: fill s from your transport
            {
                std::lock_guard<std::mutex> lk(m);
                latest = s;
                has_sample = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    controller.StartMission();

    const auto period = std::chrono::duration<double>(1.0 / cfg.control_rate_hz);
    auto next = std::chrono::steady_clock::now();

    while (controller.in_mission()) {
        next += std::chrono::duration_cast<std::chrono:steady_clock::duration>(period);

        backyard::TelemetrySample local;
        bool ok = false;
        {
            std::lock_guard<std::mutex> lk(m);
            ok = has_sample;
            local = latest;
        }

        if (ok) {
            controller.UpdateTelemetry(local);
            controller.Tick();
        }

        std::this_thread::sleep_until(next);
    }

    running.store(false);
    rx.join();

    return 0;
}
