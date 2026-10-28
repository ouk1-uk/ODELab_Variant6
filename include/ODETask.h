#pragma once

#include <cstddef>
#include <functional>
#include <vector>

struct ODEPoint {
    double x;
    double y;
};

enum class ODEMethod {
    Euler,
    RungeKutta4
};

// Solve y'(x) = f(x,y), y(xStart)=yInitial, over [xStart,xEnd].
// The right endpoint is included; the last integration step may be shortened.
class ODETask {
public:
    using Derivative = std::function<double(double, double)>;

    ODETask(double xStart, double yInitial, double xEnd,
            double step, std::size_t maxSteps = 10000);

    double xStart() const noexcept { return xStart_; }
    double yInitial() const noexcept { return yInitial_; }
    double xEnd() const noexcept { return xEnd_; }
    double step() const noexcept { return step_; }
    std::size_t maxSteps() const noexcept { return maxSteps_; }
    const std::vector<ODEPoint>& trajectory() const noexcept { return trajectory_; }
    ODEPoint pointAt(std::size_t index) const;

    // STRONG: validate the complete new configuration, then commit without throwing.
    // A successful change invalidates/clears the previously computed trajectory.
    void setParameters(double xStart, double yInitial, double xEnd,
                       double step, std::size_t maxSteps);

    // STRONG: construct a new trajectory separately, then commit with a noexcept swap.
    // On ANY exception, the previous trajectory and task parameters are unchanged.
    void solve(const Derivative& derivative, ODEMethod method);

private:
    double xStart_;
    double yInitial_;
    double xEnd_;
    double step_;
    std::size_t maxSteps_;
    std::vector<ODEPoint> trajectory_;

    static void validateParameters(double xStart, double yInitial, double xEnd,
                                   double step, std::size_t maxSteps);
    static double checkedDerivative(const Derivative& derivative,
                                    double x, double y);
    static double nextY(const Derivative& derivative, ODEMethod method,
                        double x, double y, double h);
};
