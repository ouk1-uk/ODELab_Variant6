#include <gtest/gtest.h>

#include "ODEExceptions.h"
#include "ODETask.h"
#include "ScopeLogger.h"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace {
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

void expectSameTrajectory(const std::vector<ODEPoint>& expected,
                          const ODETask& task) {
    ASSERT_EQ(expected.size(), task.trajectory().size());
    for (std::size_t i = 0; i != expected.size(); ++i) {
        EXPECT_DOUBLE_EQ(expected[i].x, task.trajectory()[i].x);
        EXPECT_DOUBLE_EQ(expected[i].y, task.trajectory()[i].y);
    }
}
}

TEST(ODETaskTest, InitialInvariantsAndGetters) {
    const ODETask task(0.0, 1.0, 2.0, 0.1, 25);
    EXPECT_DOUBLE_EQ(task.xStart(), 0.0);
    EXPECT_DOUBLE_EQ(task.yInitial(), 1.0);
    EXPECT_DOUBLE_EQ(task.xEnd(), 2.0);
    EXPECT_DOUBLE_EQ(task.step(), 0.1);
    EXPECT_EQ(task.maxSteps(), 25u);
    EXPECT_TRUE(task.trajectory().empty());
}

TEST(ODETaskTest, EulerConstantDerivative) {
    ODETask task(0.0, 2.0, 1.0, 0.25);
    task.solve([](double, double) { return 3.0; }, ODEMethod::Euler);
    ASSERT_EQ(task.trajectory().size(), 5u);
    EXPECT_NEAR(task.pointAt(4).x, 1.0, 1e-14);
    EXPECT_NEAR(task.pointAt(4).y, 5.0, 1e-14);
}

TEST(ODETaskTest, EulerExponentialApproximation) {
    ODETask task(0, 1, 1, 0.1);
    task.solve([](double, double y) { return y; }, ODEMethod::Euler);
    EXPECT_NEAR(task.trajectory().back().y, std::pow(1.1, 10), 1e-12);
}

TEST(ODETaskTest, RungeKutta4ApproximatesExponential) {
    ODETask task(0, 1, 1, 0.1);
    task.solve([](double, double y) { return y; }, ODEMethod::RungeKutta4);
    EXPECT_NEAR(task.trajectory().back().y, std::exp(1.0), 3e-6);
}

TEST(ODETaskTest, ShortensFinalStepToReachRightEndpoint) {
    ODETask task(0.0, 0.0, 1.0, 0.3);
    task.solve([](double, double) { return 1.0; }, ODEMethod::RungeKutta4);
    ASSERT_EQ(task.trajectory().size(), 5u);
    EXPECT_DOUBLE_EQ(task.trajectory().back().x, 1.0);
    EXPECT_NEAR(task.trajectory().back().y, 1.0, 1e-12);
}

TEST(ODETaskTest, InitialConditionsAtNegativeX) {
    ODETask task(-2.0, 5.0, -1.0, 0.5);
    task.solve([](double x, double) { return x; }, ODEMethod::RungeKutta4);
    ASSERT_EQ(task.trajectory().size(), 3u);
    EXPECT_NEAR(task.trajectory().back().y, 3.5, 1e-12);
}

TEST(ODETaskTest, RejectsZeroOrNegativeStep) {
    EXPECT_THROW(ODETask(0, 1, 2, 0), std::invalid_argument);
    EXPECT_THROW(ODETask(0, 1, 2, -0.1), std::invalid_argument);
}

TEST(ODETaskTest, RejectsInvalidInterval) {
    EXPECT_THROW(ODETask(1, 1, 1, 0.1), std::invalid_argument);
    EXPECT_THROW(ODETask(2, 1, 1, 0.1), std::invalid_argument);
}

TEST(ODETaskTest, RejectsNonFiniteInitialValues) {
    EXPECT_THROW(ODETask(kNaN, 1, 2, 0.1), NonFiniteValueException);
    EXPECT_THROW(ODETask(0, kNaN, 2, 0.1), NonFiniteValueException);
    EXPECT_THROW(ODETask(0, kInf, 2, 0.1), NonFiniteValueException);
    EXPECT_THROW(ODETask(0, -kInf, 2, 0.1), NonFiniteValueException);
}

TEST(ODETaskTest, RejectsNonFiniteEndAndStep) {
    EXPECT_THROW(ODETask(0, 1, kInf, 0.1), NonFiniteValueException);
    EXPECT_THROW(ODETask(0, 1, 2, kNaN), NonFiniteValueException);
    EXPECT_THROW(ODETask(0, 1, 2, kInf), NonFiniteValueException);
}

TEST(ODETaskTest, NonFiniteExceptionProvidesDiagnosticData) {
    try {
        ODETask invalid(0, kInf, 1, 0.1);
        FAIL() << "Expected NonFiniteValueException";
    } catch (const NonFiniteValueException& ex) {
        EXPECT_EQ(ex.parameter(), "yInitial");
        EXPECT_TRUE(std::isinf(ex.value()));
    }
}

TEST(ODETaskTest, RejectsUnrepresentableIntervalWidth) {
    EXPECT_THROW(ODETask(-1e308, 1.0, 1e308, 1.0), std::overflow_error);
}

TEST(ODETaskTest, RejectsZeroMaximumSteps) {
    EXPECT_THROW(ODETask(0, 1, 2, 0.1, 0), std::invalid_argument);
}

TEST(ODETaskTest, ReachesExactAllowedStepLimit) {
    ODETask task(0, 1, 1, 0.25, 4);
    EXPECT_NO_THROW(task.solve([](double, double) { return 0.0; }, ODEMethod::Euler));
    EXPECT_EQ(task.trajectory().size(), 5u);
}

TEST(ODETaskTest, RejectsExceedingStepLimitWithDetails) {
    ODETask task(0, 1, 1, 0.25, 3);
    try {
        task.solve([](double, double) { return 1.0; }, ODEMethod::Euler);
        FAIL() << "Expected StepLimitException";
    } catch (const StepLimitException& ex) {
        EXPECT_EQ(ex.limit(), 3u);
        EXPECT_EQ(ex.requested(), 4u);
    }
}

TEST(ODETaskTest, InvalidDerivativeNanThrowsCustomException) {
    ODETask task(0, 1, 1, 0.1);
    EXPECT_THROW(task.solve([](double, double) { return kNaN; }, ODEMethod::Euler),
                 InvalidDerivativeException);
}

TEST(ODETaskTest, InvalidDerivativeInfThrowsCustomException) {
    ODETask task(0, 1, 1, 0.1);
    EXPECT_THROW(task.solve([](double, double) { return kInf; }, ODEMethod::RungeKutta4),
                 InvalidDerivativeException);
}

TEST(ODETaskTest, DerivativeExceptionHasCoordinates) {
    ODETask task(0, 2, 1, 0.5);
    try {
        task.solve([](double, double) { return kNaN; }, ODEMethod::Euler);
        FAIL() << "Expected InvalidDerivativeException";
    } catch (const InvalidDerivativeException& ex) {
        EXPECT_DOUBLE_EQ(ex.x(), 0.0);
        EXPECT_DOUBLE_EQ(ex.y(), 2.0);
        EXPECT_TRUE(std::isnan(ex.value()));
    }
}

TEST(ODETaskTest, OverflowDuringNumericIntegrationThrows) {
    ODETask task(0, 1e308, 1, 1);
    EXPECT_THROW(task.solve([](double, double) { return 1e308; }, ODEMethod::Euler),
                 std::overflow_error);
}

TEST(ODETaskTest, RejectsUnknownMethodAndEmptyCallback) {
    ODETask task(0, 1, 1, 0.1);
    EXPECT_THROW(task.solve({}, ODEMethod::Euler), std::invalid_argument);
    EXPECT_THROW(task.solve([](double, double) { return 1.; },
                            static_cast<ODEMethod>(42)), std::invalid_argument);
}

TEST(ODETaskTest, InvalidIndexThrowsStandardException) {
    ODETask task(0, 1, 1, 0.5);
    EXPECT_THROW(task.pointAt(0), std::out_of_range);
    task.solve([](double, double) { return 1.; }, ODEMethod::Euler);
    EXPECT_THROW(task.pointAt(3), std::out_of_range);
}

TEST(ODETaskTest, SolveStrongGuaranteeAfterInvalidDerivative) {
    ODETask task(0, 1, 1, 0.1);
    task.solve([](double, double) { return 1.; }, ODEMethod::Euler);
    const auto before = task.trajectory();
    EXPECT_THROW(task.solve([](double x, double) {
                     return x > 0.3 ? kNaN : 2.;
                 }, ODEMethod::Euler), InvalidDerivativeException);
    expectSameTrajectory(before, task);
}

TEST(ODETaskTest, StepLimitFailureLeavesInitialTrajectoryUnchanged) {
    ODETask task(0, 1, 1, 0.25, 3);
    const auto before = task.trajectory();
    EXPECT_THROW(task.solve([](double, double) { return 1.; }, ODEMethod::Euler),
                 StepLimitException);
    expectSameTrajectory(before, task);
}

TEST(ODETaskTest, SolveStrongGuaranteeAfterCallbackThrows) {
    ODETask task(0, 1, 1, 0.2);
    task.solve([](double, double) { return 1.; }, ODEMethod::Euler);
    const auto before = task.trajectory();
    EXPECT_THROW(task.solve([](double x, double) -> double {
                     if (x > 0.3) throw std::runtime_error("callback error");
                     return 0.0;
                 }, ODEMethod::Euler), std::runtime_error);
    expectSameTrajectory(before, task);
}

TEST(ODETaskTest, SolveStrongGuaranteeAfterOverflow) {
    ODETask task(0, 1, 2, 0.25);
    task.solve([](double, double) { return 1.; }, ODEMethod::Euler);
    const auto before = task.trajectory();
    EXPECT_THROW(task.solve([](double, double) { return 1e308; },
                            ODEMethod::RungeKutta4), std::overflow_error);
    expectSameTrajectory(before, task);
}

TEST(ODETaskTest, SetParametersStrongGuaranteeOnInvalidStep) {
    ODETask task(0, 1, 1, 0.25, 4);
    task.solve([](double, double) { return 1.; }, ODEMethod::Euler);
    const auto before = task.trajectory();
    EXPECT_THROW(task.setParameters(0, 500, 2, -0.5, 40), std::invalid_argument);
    EXPECT_DOUBLE_EQ(task.yInitial(), 1.0);
    EXPECT_DOUBLE_EQ(task.xEnd(), 1.0);
    EXPECT_DOUBLE_EQ(task.step(), 0.25);
    EXPECT_EQ(task.maxSteps(), 4u);
    expectSameTrajectory(before, task);
}

TEST(ODETaskTest, SuccessfulSetParametersClearsStaleTrajectory) {
    ODETask task(0, 1, 1, 0.5);
    task.solve([](double, double) { return 1.; }, ODEMethod::Euler);
    task.setParameters(2, 4, 3, 0.5, 8);
    EXPECT_TRUE(task.trajectory().empty());
    EXPECT_DOUBLE_EQ(task.xStart(), 2.0);
    EXPECT_DOUBLE_EQ(task.yInitial(), 4.0);
    EXPECT_DOUBLE_EQ(task.xEnd(), 3.0);
    EXPECT_EQ(task.maxSteps(), 8u);
}

TEST(ScopeLoggerTest, LogsReverseDestructionOrder) {
    std::ostringstream log;
    try {
        ScopeLogger outer("outer", log);
        ScopeLogger inner("inner", log);
        throw std::runtime_error("trigger unwind");
    } catch (const std::runtime_error&) {}
    EXPECT_EQ(log.str(), "[enter] outer\n[enter] inner\n[exit] inner\n[exit] outer\n");
}

TEST(ScopeLoggerTest, DestructorIsNoThrow) {
    static_assert(std::is_nothrow_destructible_v<ScopeLogger>);
    EXPECT_TRUE(std::is_nothrow_destructible_v<ScopeLogger>);
}
