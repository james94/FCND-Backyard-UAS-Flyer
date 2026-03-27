#include "udacidrone_driver_cpp/uas_state_repository.hpp"

namespace udacidrone_driver_cpp {

void UasStateRepository::setConnected(bool connected) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.connected = connected;
}

void UasStateRepository::updateState(const UasStateTelemetry &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.state = msg;
}

void UasStateRepository::updatePosition(const UasPositionTelemetry &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.position = msg;
}

void UasStateRepository::updateVelocity(const UasVelocityTelemetry &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.velocity = msg;
}

UasStateSnapshot UasStateRepository::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

} // namespace udacidrone_driver_cpp
