#include "ODETask.h"
#include "ODEExceptions.h"
#include "ScopeLogger.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

void levelThree(ODETask& task) {
    ScopeLogger guard("levelThree");
    task.solve([](double, double) {
        return std::numeric_limits<double>::quiet_NaN();
    }, ODEMethod::Euler);
}

void levelTwo(ODETask& task) {
    ScopeLogger guard("levelTwo");
    try {
        levelThree(task);
    } catch (const InvalidDerivativeException& ex) {
        std::cout << "[middle] invalid derivative at x=" << ex.x()
                  << ", y=" << ex.y() << "; rethrow\n";
        throw;  // preserve the dynamic type
    }
}

void levelOne(ODETask& task) {
    ScopeLogger guard("levelOne");
    levelTwo(task);
}

int main() {
    std::cout << std::fixed << std::setprecision(7);
    ODETask task(0.0, 1.0, 1.0, 0.1, 100);
    const ODETask::Derivative derivative = [](double, double y) { return y; };

    task.solve(derivative, ODEMethod::Euler);
    std::cout << "Euler: " << task.pointAt(task.trajectory().size() - 1).y << '\n';

    task.solve(derivative, ODEMethod::RungeKutta4);
    const double previous = task.pointAt(task.trajectory().size() - 1).y;
    std::cout << "RK4:   " << previous << " (exact exp(1)=" << std::exp(1.0) << ")\n";

    std::cout << "\n--- Strong exception guarantee ---\n";
    try {
        task.solve([](double x, double) {
            return x >= 0.35 ? std::numeric_limits<double>::infinity() : 1.0;
        }, ODEMethod::Euler);
    } catch (const InvalidDerivativeException& ex) {
        std::cout << "Caught InvalidDerivativeException at x=" << ex.x() << '\n';
    }
    std::cout << "Previous trajectory preserved: " << std::boolalpha
              << (task.pointAt(task.trajectory().size() - 1).y == previous) << '\n';

    std::cout << "\n--- Additional input error ---\n";
    try {
        ODETask invalid(0.0, 1.0, 1.0, 0.0);
    } catch (const std::invalid_argument& ex) {
        std::cout << "Caught std::invalid_argument: " << ex.what() << '\n';
    }

    std::cout << "\n--- Stack unwinding + RAII + rethrow ---\n";
    try {
        levelOne(task);
    } catch (const InvalidDerivativeException& ex) {
        std::cout << "[top] caught InvalidDerivativeException: " << ex.what() << '\n';
    } catch (const ODEException& ex) {
        std::cout << "[top] caught ODEException: " << ex.what() << '\n';
    } catch (const std::exception& ex) {
        std::cout << "[top] caught std::exception: " << ex.what() << '\n';
    }
    return 0;
}
