#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

// Base class for all domain-specific exceptions in this library.
class ODEException : public std::runtime_error {
public:
    explicit ODEException(const std::string& message)
        : std::runtime_error(message) {}
};

class NonFiniteValueException : public ODEException {
    std::string parameter_;
    double value_;
public:
    NonFiniteValueException(std::string parameter, double value)
        : ODEException("Non-finite ODE parameter: " + parameter),
          parameter_(std::move(parameter)), value_(value) {}
    const std::string& parameter() const noexcept { return parameter_; }
    double value() const noexcept { return value_; }
};

class InvalidDerivativeException : public ODEException {
    double x_;
    double y_;
    double value_;
public:
    InvalidDerivativeException(double x, double y, double value)
        : ODEException("ODE derivative returned NaN or infinity"),
          x_(x), y_(y), value_(value) {}
    double x() const noexcept { return x_; }
    double y() const noexcept { return y_; }
    double value() const noexcept { return value_; }
};

class StepLimitException : public ODEException {
    std::size_t limit_;
    std::size_t requested_;
public:
    StepLimitException(std::size_t limit, std::size_t requested)
        : ODEException("Maximum number of ODE integration steps exceeded"),
          limit_(limit), requested_(requested) {}
    std::size_t limit() const noexcept { return limit_; }
    std::size_t requested() const noexcept { return requested_; }
};
