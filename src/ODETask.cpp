#include "ODETask.h"
#include "ODEExceptions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

void ODETask::validateParameters(double xStart, double yInitial,
                                 double xEnd, double step,
                                 std::size_t maxSteps) {
    if (!std::isfinite(xStart))
        throw NonFiniteValueException("xStart", xStart);
    if (!std::isfinite(yInitial))
        throw NonFiniteValueException("yInitial", yInitial);
    if (!std::isfinite(xEnd))
        throw NonFiniteValueException("xEnd", xEnd);
    if (!std::isfinite(step))
        throw NonFiniteValueException("step", step);
    if (step <= 0.0)
        throw std::invalid_argument("Integration step must be positive");
    if (xStart >= xEnd)
        throw std::invalid_argument("Integration interval requires xStart < xEnd");
    if (!std::isfinite(xEnd - xStart))
        throw std::overflow_error("Integration interval width overflow");
    if (maxSteps == 0)
        throw std::invalid_argument("Maximum number of steps must be positive");
}

ODETask::ODETask(double xStart, double yInitial, double xEnd,
                 double step, std::size_t maxSteps)
    : xStart_(xStart), yInitial_(yInitial), xEnd_(xEnd),
      step_(step), maxSteps_(maxSteps) {
    validateParameters(xStart, yInitial, xEnd, step, maxSteps);
}

ODEPoint ODETask::pointAt(std::size_t index) const {
    return trajectory_.at(index);  // std::out_of_range on an invalid index
}

void ODETask::setParameters(double xStart, double yInitial, double xEnd,
                             double step, std::size_t maxSteps) {
    validateParameters(xStart, yInitial, xEnd, step, maxSteps);
    // Commit stage: only noexcept assignments and vector::clear of trivial elements.
    xStart_ = xStart;
    yInitial_ = yInitial;
    xEnd_ = xEnd;
    step_ = step;
    maxSteps_ = maxSteps;
    trajectory_.clear();
}

double ODETask::checkedDerivative(const Derivative& derivative,
                                   double x, double y) {
    if (!std::isfinite(x) || !std::isfinite(y))
        throw std::overflow_error("Non-finite ODE intermediate state");
    const double value = derivative(x, y);
    if (!std::isfinite(value))
        throw InvalidDerivativeException(x, y, value);
    return value;
}

double ODETask::nextY(const Derivative& derivative, ODEMethod method,
                      double x, double y, double h) {
    const auto checkedState = [](double value) {
        if (!std::isfinite(value))
            throw std::overflow_error("ODE arithmetic overflow");
        return value;
    };

    double result;
    switch (method) {
    case ODEMethod::Euler: {
        const double k1 = checkedDerivative(derivative, x, y);
        result = checkedState(y + checkedState(h * k1));
        break;
    }
    case ODEMethod::RungeKutta4: {
        const double halfH = h / 2.0;
        const double xHalf = checkedState(x + halfH);
        const double xNext = checkedState(x + h);
        const double k1 = checkedDerivative(derivative, x, y);
        const double k2 = checkedDerivative(derivative, xHalf,
                                    checkedState(y + checkedState(halfH * k1)));
        const double k3 = checkedDerivative(derivative, xHalf,
                                    checkedState(y + checkedState(halfH * k2)));
        const double k4 = checkedDerivative(derivative, xNext,
                                    checkedState(y + checkedState(h * k3)));
        // Weight before summation to reduce intermediate overflow risk.
        const double slope = checkedState(checkedState(k1 / 6.0 + k2 / 3.0)
                                                + checkedState(k3 / 3.0 + k4 / 6.0));
        result = checkedState(y + checkedState(h * slope));
        break;
    }
    default:
        throw std::invalid_argument("Unknown ODE integration method");
    }
    return result;
}

void ODETask::solve(const Derivative& derivative, ODEMethod method) {
    if (!derivative)
        throw std::invalid_argument("Derivative callback must not be empty");
    if (method != ODEMethod::Euler && method != ODEMethod::RungeKutta4)
        throw std::invalid_argument("Unknown ODE integration method");

    // Prepare everything in a local container. Never mutate trajectory_ here.
    std::vector<ODEPoint> prepared;
    prepared.push_back({xStart_, yInitial_});

    double x = xStart_;
    double y = yInitial_;
    std::size_t steps = 0;
    while (x < xEnd_) {
        if (steps == maxSteps_)
            throw StepLimitException(maxSteps_, steps + 1);
        const double remaining = xEnd_ - x;
        const double proposed = std::min(step_, remaining);
        double nextX = x + proposed;
        if (!std::isfinite(nextX))
            throw std::overflow_error("ODE x-coordinate overflow");
        if (nextX > xEnd_) nextX = xEnd_;
        // Avoid an artificial extra step caused solely by accumulation round-off.
        const double endpointScale = std::max(std::abs(nextX), std::abs(xEnd_));
        if (nextX < xEnd_ &&
            xEnd_ - nextX <= 2.0 * std::numeric_limits<double>::epsilon() * endpointScale)
            nextX = xEnd_;
        if (nextX <= x)
            throw std::overflow_error("ODE step cannot advance x in double precision");
        const double actualStep = nextX - x;
        const double nextValue = nextY(derivative, method, x, y, actualStep);
        prepared.push_back({nextX, nextValue});
        x = nextX;
        y = nextValue;
        ++steps;
    }

    // Commit: std::vector::swap with std::allocator is noexcept.
    trajectory_.swap(prepared);
}
