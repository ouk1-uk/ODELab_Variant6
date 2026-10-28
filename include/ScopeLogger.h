#pragma once

#include <iostream>
#include <ostream>
#include <string>
#include <utility>

// RAII: destructor executes even if the surrounding scope is left by an exception.
class ScopeLogger {
    std::string name_;
    std::ostream* output_;
public:
    explicit ScopeLogger(std::string name, std::ostream& output = std::cout)
        : name_(std::move(name)), output_(&output) {
        *output_ << "[enter] " << name_ << '\n';
    }

    ScopeLogger(const ScopeLogger&) = delete;
    ScopeLogger& operator=(const ScopeLogger&) = delete;

    ~ScopeLogger() noexcept {
        try { *output_ << "[exit] " << name_ << '\n'; }
        catch (...) { /* A destructor must not propagate during unwinding. */ }
    }
};
