// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/vehicle/control/control_math.h"

#include <cmath>
#include <limits>
#include <numbers>
#include <string_view>

#include "gtest/gtest.h"

namespace {

using intrinsic::vehicle::control::BackCalculation;
using intrinsic::vehicle::control::BoundedIntegrator;
using intrinsic::vehicle::control::ControlMathCode;
using intrinsic::vehicle::control::ControlMathResult;
using intrinsic::vehicle::control::HeadingError;
using intrinsic::vehicle::control::IntegrateBackCalculation;
using intrinsic::vehicle::control::ShortestSignedAngle;

constexpr double kPi = std::numbers::pi;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

constexpr std::string_view kAngleMessage = "angle must be finite";
constexpr std::string_view kHeadingMessage = "heading must be finite";
constexpr std::string_view kHeadingDifferenceMessage =
    "heading difference is not finite";
constexpr std::string_view kLimitsMessage =
    "integrator limits must be finite and ordered";
constexpr std::string_view kInitialStateMessage =
    "integrator state must be finite and inside the limits";
constexpr std::string_view kIntegratorInvalidMessage =
    "integrator is not valid";
constexpr std::string_view kInputMessage = "integrator input must be finite";
constexpr std::string_view kTimeStepMessage =
    "time step must be finite and greater than or equal to zero";
constexpr std::string_view kStepMessage = "integrator step is not finite";
constexpr std::string_view kBackCalculationMessage =
    "back-calculation arguments must be finite";
constexpr std::string_view kBackCalculationTermMessage =
    "back-calculation term is not finite";
constexpr std::string_view kBackCalculationSumMessage =
    "back-calculation integrator input is not finite";

void ExpectInOpenClosedPi(double angle) {
  EXPECT_TRUE(std::isfinite(angle));
  EXPECT_GT(angle, -kPi);
  EXPECT_LE(angle, kPi);
}

double SaturateSymmetric(double command, double limit) {
  if (command > limit) {
    return limit;
  }
  if (command < -limit) {
    return -limit;
  }
  return command;
}

void ExpectRejected(const ControlMathResult<double>& result,
                    std::string_view message) {
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ControlMathCode::kInvalidArgument);
  EXPECT_EQ(result.status().message, message);
  EXPECT_EQ(result.value(), 0.0);
  EXPECT_TRUE(std::isfinite(result.value()));
}

TEST(ShortestSignedAngle, ZeroErrorIsZero) {
  const ControlMathResult<double> result = ShortestSignedAngle(0);
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.value(), 0.0);
  ExpectInOpenClosedPi(result.value());
}

TEST(ShortestSignedAngle, KeepsASmallSignedError) {
  const ControlMathResult<double> positive = ShortestSignedAngle(0.3);
  const ControlMathResult<double> negative = ShortestSignedAngle(-0.3);
  EXPECT_TRUE(positive.ok());
  EXPECT_TRUE(negative.ok());
  EXPECT_NEAR(positive.value(), 0.3, 1e-15);
  EXPECT_NEAR(negative.value(), -0.3, 1e-15);
  ExpectInOpenClosedPi(positive.value());
  ExpectInOpenClosedPi(negative.value());
}

TEST(ShortestSignedAngle, CoversPositiveAndNegativePiBoundaries) {
  const ControlMathResult<double> positive = ShortestSignedAngle(kPi);
  const ControlMathResult<double> negative = ShortestSignedAngle(-kPi);
  EXPECT_TRUE(positive.ok());
  EXPECT_TRUE(negative.ok());
  ExpectInOpenClosedPi(positive.value());
  ExpectInOpenClosedPi(negative.value());
  EXPECT_NEAR(positive.value(), kPi, 1e-12);
  EXPECT_NEAR(std::abs(negative.value()), kPi, 1e-12);
  EXPECT_NEAR(std::sin(positive.value()), 0.0, 1e-12);
  EXPECT_NEAR(std::cos(positive.value()), -1.0, 1e-12);
  EXPECT_NEAR(std::sin(negative.value()), 0.0, 1e-12);
  EXPECT_NEAR(std::cos(negative.value()), -1.0, 1e-12);
}

TEST(ShortestSignedAngle, WrapsAcrossTheBranchCut) {
  const ControlMathResult<double> above = ShortestSignedAngle(kPi + 0.1);
  const ControlMathResult<double> below = ShortestSignedAngle(-kPi - 0.1);
  EXPECT_TRUE(above.ok());
  EXPECT_TRUE(below.ok());
  EXPECT_NEAR(above.value(), -kPi + 0.1, 1e-12);
  EXPECT_NEAR(below.value(), kPi - 0.1, 1e-12);
  ExpectInOpenClosedPi(above.value());
  ExpectInOpenClosedPi(below.value());
}

TEST(ShortestSignedAngle, FullTurnsAndRightAngles) {
  const ControlMathResult<double> full = ShortestSignedAngle(2 * kPi);
  const ControlMathResult<double> negative_full = ShortestSignedAngle(-2 * kPi);
  const ControlMathResult<double> three_halves = ShortestSignedAngle(1.5 * kPi);
  const ControlMathResult<double> negative_three_halves =
      ShortestSignedAngle(-1.5 * kPi);
  EXPECT_TRUE(full.ok());
  EXPECT_TRUE(negative_full.ok());
  EXPECT_TRUE(three_halves.ok());
  EXPECT_TRUE(negative_three_halves.ok());
  EXPECT_NEAR(full.value(), 0.0, 1e-12);
  EXPECT_NEAR(negative_full.value(), 0.0, 1e-12);
  EXPECT_NEAR(three_halves.value(), -0.5 * kPi, 1e-12);
  EXPECT_NEAR(negative_three_halves.value(), 0.5 * kPi, 1e-12);
}

TEST(ShortestSignedAngle, MatchesSineAndCosineOfTheInput) {
  const double samples[] = {0.0, 0.4, -1.2, 2.2, -4.0, 7.5, kPi, -kPi};
  for (double sample : samples) {
    const ControlMathResult<double> result = ShortestSignedAngle(sample);
    EXPECT_TRUE(result.ok()) << sample;
    ExpectInOpenClosedPi(result.value());
    EXPECT_NEAR(std::sin(result.value()), std::sin(sample), 1e-12) << sample;
    EXPECT_NEAR(std::cos(result.value()), std::cos(sample), 1e-12) << sample;
  }
}

TEST(ShortestSignedAngle, RepeatedCallsMatch) {
  const ControlMathResult<double> first = ShortestSignedAngle(2.5);
  const ControlMathResult<double> second = ShortestSignedAngle(2.5);
  EXPECT_TRUE(first.ok());
  EXPECT_EQ(first.value(), second.value());
}

TEST(ShortestSignedAngle, RejectsNonFiniteInput) {
  ExpectRejected(ShortestSignedAngle(kNan), kAngleMessage);
  ExpectRejected(ShortestSignedAngle(kInf), kAngleMessage);
  ExpectRejected(ShortestSignedAngle(-kInf), kAngleMessage);
}

TEST(HeadingError, ZeroErrorIsZero) {
  const ControlMathResult<double> result = HeadingError(0.2, 0.2);
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.value(), 0.0);
}

TEST(HeadingError, PositiveWhenMeasuredIsShortOfTheReference) {
  const ControlMathResult<double> result = HeadingError(0.4, 0.1);
  EXPECT_TRUE(result.ok());
  EXPECT_NEAR(result.value(), 0.3, 1e-15);
}

TEST(HeadingError, WrapsEquivalentHeadingsToZero) {
  const ControlMathResult<double> opposite_pi = HeadingError(kPi, -kPi);
  const ControlMathResult<double> swapped = HeadingError(-kPi, kPi);
  EXPECT_TRUE(opposite_pi.ok());
  EXPECT_TRUE(swapped.ok());
  EXPECT_NEAR(opposite_pi.value(), 0.0, 1e-12);
  EXPECT_NEAR(swapped.value(), 0.0, 1e-12);
  ExpectInOpenClosedPi(opposite_pi.value());
}

TEST(HeadingError, UsesTheShortestSignedDifference) {
  const ControlMathResult<double> result = HeadingError(0.1, 2 * kPi - 0.2);
  EXPECT_TRUE(result.ok());
  EXPECT_NEAR(result.value(), 0.3, 1e-12);
  ExpectInOpenClosedPi(result.value());
}

TEST(HeadingError, MatchesTheWrappedDifference) {
  const double reference = 1.25;
  const double measured = -2.5;
  const ControlMathResult<double> error = HeadingError(reference, measured);
  const ControlMathResult<double> wrapped =
      ShortestSignedAngle(reference - measured);
  EXPECT_TRUE(error.ok());
  EXPECT_TRUE(wrapped.ok());
  EXPECT_EQ(error.value(), wrapped.value());
}

TEST(HeadingError, RejectsNonFiniteArguments) {
  ExpectRejected(HeadingError(kNan, 0), kHeadingMessage);
  ExpectRejected(HeadingError(0, kInf), kHeadingMessage);
  ExpectRejected(HeadingError(-kInf, kInf), kHeadingMessage);
  ExpectRejected(HeadingError(1e308, -1e308), kHeadingDifferenceMessage);
}

TEST(BoundedIntegrator, IntegratesInsideTheLimits) {
  ControlMathResult<BoundedIntegrator> created =
      BoundedIntegrator::Create(0.25, -1, 1);
  ASSERT_TRUE(created.ok());
  BoundedIntegrator integrator = created.value();
  const ControlMathResult<double> step = integrator.Integrate(0.5, 0.5);
  EXPECT_TRUE(step.ok());
  EXPECT_EQ(step.value(), 0.5);
  EXPECT_EQ(integrator.state(), 0.5);
}

TEST(BoundedIntegrator, ZeroTimeStepLeavesTheStateUnchanged) {
  ControlMathResult<BoundedIntegrator> created =
      BoundedIntegrator::Create(0.25, -1, 1);
  ASSERT_TRUE(created.ok());
  BoundedIntegrator integrator = created.value();
  const ControlMathResult<double> step = integrator.Integrate(10, 0);
  EXPECT_TRUE(step.ok());
  EXPECT_EQ(step.value(), 0.25);
  EXPECT_EQ(integrator.state(), 0.25);
}

TEST(BoundedIntegrator, EntersAndExitsTheUpperLimit) {
  ControlMathResult<BoundedIntegrator> created =
      BoundedIntegrator::Create(0, -1, 1);
  ASSERT_TRUE(created.ok());
  BoundedIntegrator integrator = created.value();
  EXPECT_TRUE(integrator.Integrate(0.5, 1).ok());
  EXPECT_EQ(integrator.state(), 0.5);
  EXPECT_TRUE(integrator.Integrate(0.5, 1).ok());
  EXPECT_EQ(integrator.state(), 1.0);
  EXPECT_TRUE(integrator.Integrate(10, 1).ok());
  EXPECT_EQ(integrator.state(), 1.0);
  EXPECT_TRUE(integrator.Integrate(-0.25, 1).ok());
  EXPECT_EQ(integrator.state(), 0.75);
}

TEST(BoundedIntegrator, EntersAndExitsTheLowerLimit) {
  ControlMathResult<BoundedIntegrator> created =
      BoundedIntegrator::Create(0, -1, 1);
  ASSERT_TRUE(created.ok());
  BoundedIntegrator integrator = created.value();
  EXPECT_TRUE(integrator.Integrate(-0.5, 1).ok());
  EXPECT_EQ(integrator.state(), -0.5);
  EXPECT_TRUE(integrator.Integrate(-1, 1).ok());
  EXPECT_EQ(integrator.state(), -1.0);
  EXPECT_TRUE(integrator.Integrate(-3, 1).ok());
  EXPECT_EQ(integrator.state(), -1.0);
  EXPECT_TRUE(integrator.Integrate(0.25, 1).ok());
  EXPECT_EQ(integrator.state(), -0.75);
}

TEST(BoundedIntegrator, EqualLimitsStaySaturated) {
  ControlMathResult<BoundedIntegrator> created =
      BoundedIntegrator::Create(0.5, 0.5, 0.5);
  ASSERT_TRUE(created.ok());
  BoundedIntegrator integrator = created.value();
  EXPECT_TRUE(integrator.Integrate(4, 1).ok());
  EXPECT_EQ(integrator.state(), 0.5);
  EXPECT_TRUE(integrator.Integrate(-4, 1).ok());
  EXPECT_EQ(integrator.state(), 0.5);
}

TEST(BoundedIntegrator, RejectsABadTimeStepWithoutWriting) {
  ControlMathResult<BoundedIntegrator> created =
      BoundedIntegrator::Create(0.25, -1, 1);
  ASSERT_TRUE(created.ok());
  BoundedIntegrator integrator = created.value();
  const ControlMathResult<double> negative = integrator.Integrate(1, -0.1);
  EXPECT_FALSE(negative.ok());
  EXPECT_EQ(negative.status().message, kTimeStepMessage);
  EXPECT_EQ(negative.value(), 0.25);
  EXPECT_EQ(integrator.state(), 0.25);
  const ControlMathResult<double> nan_dt = integrator.Integrate(1, kNan);
  EXPECT_EQ(nan_dt.status().message, kTimeStepMessage);
  EXPECT_EQ(integrator.state(), 0.25);
  const ControlMathResult<double> inf_dt = integrator.Integrate(1, kInf);
  EXPECT_EQ(inf_dt.status().message, kTimeStepMessage);
  EXPECT_EQ(integrator.state(), 0.25);
}

TEST(BoundedIntegrator, RejectsANonFiniteInputWithoutWriting) {
  ControlMathResult<BoundedIntegrator> created =
      BoundedIntegrator::Create(0.25, -1, 1);
  ASSERT_TRUE(created.ok());
  BoundedIntegrator integrator = created.value();
  const ControlMathResult<double> nan_input = integrator.Integrate(kNan, 0);
  EXPECT_FALSE(nan_input.ok());
  EXPECT_EQ(nan_input.status().message, kInputMessage);
  EXPECT_EQ(nan_input.value(), 0.25);
  EXPECT_EQ(integrator.state(), 0.25);
  EXPECT_EQ(integrator.Integrate(kInf, 0.1).status().message, kInputMessage);
  EXPECT_EQ(integrator.state(), 0.25);
}

TEST(BoundedIntegrator, RejectsANonFiniteStepWithoutWriting) {
  ControlMathResult<BoundedIntegrator> created =
      BoundedIntegrator::Create(1e308, -1e308, 1e308);
  ASSERT_TRUE(created.ok());
  BoundedIntegrator integrator = created.value();
  const ControlMathResult<double> step = integrator.Integrate(1e308, 1);
  EXPECT_FALSE(step.ok());
  EXPECT_EQ(step.status().message, kStepMessage);
  EXPECT_EQ(step.value(), 1e308);
  EXPECT_EQ(integrator.state(), 1e308);
}

TEST(BoundedIntegrator, RejectsBadLimitsAndAStateOutsideThem) {
  const ControlMathResult<BoundedIntegrator> reversed =
      BoundedIntegrator::Create(0, 1, -1);
  EXPECT_FALSE(reversed.ok());
  EXPECT_EQ(reversed.status().message, kLimitsMessage);
  EXPECT_FALSE(reversed.value().valid());
  const ControlMathResult<BoundedIntegrator> nan_limit =
      BoundedIntegrator::Create(0, kNan, 1);
  EXPECT_EQ(nan_limit.status().message, kLimitsMessage);
  const ControlMathResult<BoundedIntegrator> outside =
      BoundedIntegrator::Create(2, -1, 1);
  EXPECT_EQ(outside.status().message, kInitialStateMessage);
  EXPECT_FALSE(outside.value().valid());
  BoundedIntegrator invalid = outside.value();
  const ControlMathResult<double> ignored = invalid.Integrate(1, 1);
  EXPECT_EQ(ignored.status().message, kIntegratorInvalidMessage);
  EXPECT_EQ(ignored.value(), 0.0);
  EXPECT_FALSE(invalid.valid());
}

TEST(BoundedIntegrator, RepeatedStepsMatch) {
  auto Step = [](double input) {
    BoundedIntegrator integrator =
        BoundedIntegrator::Create(0.1, -2, 2).value();
    EXPECT_TRUE(integrator.Integrate(input, 0.25).ok());
    return integrator.state();
  };
  EXPECT_EQ(Step(0.4), Step(0.4));
}

TEST(BackCalculation, ZeroWhenTheCommandIsUnsaturated) {
  const ControlMathResult<double> term = BackCalculation(0.5, 0.5, 3);
  EXPECT_TRUE(term.ok());
  EXPECT_EQ(term.value(), 0.0);
}

TEST(BackCalculation, UsesTheSaturatedMinusUnsaturatedCommand) {
  const ControlMathResult<double> high = BackCalculation(3, 1, 0.5);
  const ControlMathResult<double> low = BackCalculation(-4, -1, 2);
  EXPECT_TRUE(high.ok());
  EXPECT_TRUE(low.ok());
  EXPECT_EQ(high.value(), -1.0);
  EXPECT_EQ(low.value(), 6.0);
}

TEST(BackCalculation, AcceptsANegativeFiniteGain) {
  const ControlMathResult<double> term = BackCalculation(3, 1, -2);
  EXPECT_TRUE(term.ok());
  EXPECT_EQ(term.value(), 4.0);
}

TEST(BackCalculation, RejectsNonFiniteArguments) {
  ExpectRejected(BackCalculation(kNan, 1, 1), kBackCalculationMessage);
  ExpectRejected(BackCalculation(1, kInf, 1), kBackCalculationMessage);
  ExpectRejected(BackCalculation(1, 1, kNan), kBackCalculationMessage);
  ExpectRejected(BackCalculation(1e308, -1e308, 1e308),
                 kBackCalculationTermMessage);
}

TEST(IntegrateBackCalculation, OneSaturatedStepFeedsTheIntegrator) {
  BoundedIntegrator integrator = BoundedIntegrator::Create(1.5, -4, 4).value();
  const double unsaturated = integrator.state();
  const double saturated = 1;
  const ControlMathResult<double> step = IntegrateBackCalculation(
      integrator, /*integrator_input=*/1, unsaturated, saturated,
      /*k_aw=*/1, /*dt=*/0.5);
  EXPECT_TRUE(step.ok());
  EXPECT_EQ(step.value(), 1.75);
  EXPECT_EQ(integrator.state(), 1.75);
  EXPECT_EQ(BackCalculation(unsaturated, saturated, 1).value(), -0.5);
}

TEST(IntegrateBackCalculation, ZeroTimeStepLeavesTheStateUnchanged) {
  BoundedIntegrator integrator = BoundedIntegrator::Create(1.5, -4, 4).value();
  const ControlMathResult<double> step =
      IntegrateBackCalculation(integrator, 1, integrator.state(), 1, 1, 0);
  EXPECT_TRUE(step.ok());
  EXPECT_EQ(step.value(), 1.5);
  EXPECT_EQ(integrator.state(), 1.5);
}

// Scalar recurrence. The command is saturated to +/- kCommandLimit.
// k_aw is an input fixture. This test has no plant and no gain policy.
TEST(IntegrateBackCalculation, StaysBoundedAndRecovers) {
  constexpr double kLower = -4;
  constexpr double kUpper = 4;
  constexpr double kCommandLimit = 1;
  constexpr double kDt = 0.5;
  constexpr double kAw = 1;

  BoundedIntegrator with_tracking =
      BoundedIntegrator::Create(0, kLower, kUpper).value();
  BoundedIntegrator clamped_only =
      BoundedIntegrator::Create(0, kLower, kUpper).value();
  for (int i = 0; i < 40; ++i) {
    const double tracked_command = with_tracking.state();
    ASSERT_TRUE(IntegrateBackCalculation(
                    with_tracking, /*integrator_input=*/1, tracked_command,
                    SaturateSymmetric(tracked_command, kCommandLimit), kAw, kDt)
                    .ok());
    EXPECT_GE(with_tracking.state(), kLower);
    EXPECT_LE(with_tracking.state(), kUpper);

    const double clamped_command = clamped_only.state();
    ASSERT_TRUE(IntegrateBackCalculation(
                    clamped_only, /*integrator_input=*/1, clamped_command,
                    SaturateSymmetric(clamped_command, kCommandLimit),
                    /*k_aw=*/0, kDt)
                    .ok());
  }
  EXPECT_NEAR(with_tracking.state(), 2.0, 1e-9);
  EXPECT_GT(with_tracking.state(), kCommandLimit);
  EXPECT_LT(with_tracking.state(), kUpper);
  EXPECT_EQ(clamped_only.state(), kUpper);

  for (int i = 0; i < 40; ++i) {
    const double command = with_tracking.state();
    ASSERT_TRUE(IntegrateBackCalculation(
                    with_tracking, /*integrator_input=*/0, command,
                    SaturateSymmetric(command, kCommandLimit), kAw, kDt)
                    .ok());
    EXPECT_GE(with_tracking.state(), kLower);
    EXPECT_LE(with_tracking.state(), kUpper);
  }
  EXPECT_NEAR(with_tracking.state(), kCommandLimit, 1e-9);
  EXPECT_NEAR(
      BackCalculation(with_tracking.state(), kCommandLimit, kAw).value(), 0.0,
      1e-9);
}

TEST(IntegrateBackCalculation, RejectsNonFiniteValuesWithoutWriting) {
  BoundedIntegrator integrator = BoundedIntegrator::Create(0.25, -4, 4).value();
  const ControlMathResult<double> bad_command =
      IntegrateBackCalculation(integrator, 1, kNan, 1, 1, 0.1);
  EXPECT_FALSE(bad_command.ok());
  EXPECT_EQ(bad_command.status().message, kBackCalculationMessage);
  EXPECT_EQ(bad_command.value(), 0.25);
  EXPECT_EQ(integrator.state(), 0.25);

  const ControlMathResult<double> bad_input =
      IntegrateBackCalculation(integrator, kInf, 0, 0, 1, 0.1);
  EXPECT_EQ(bad_input.status().message, kInputMessage);
  EXPECT_EQ(integrator.state(), 0.25);

  const ControlMathResult<double> bad_dt =
      IntegrateBackCalculation(integrator, 1, 0, 0, 1, -0.2);
  EXPECT_EQ(bad_dt.status().message, kTimeStepMessage);
  EXPECT_EQ(integrator.state(), 0.25);

  // integrator_input 1e308 plus tracking term 1e308 overflows.
  const ControlMathResult<double> overflow =
      IntegrateBackCalculation(integrator, 1e308, -1e308, 0, 1, 1);
  EXPECT_EQ(overflow.status().message, kBackCalculationSumMessage);
  EXPECT_EQ(integrator.state(), 0.25);
}

TEST(IntegrateBackCalculation, RejectsAnInvalidIntegrator) {
  BoundedIntegrator integrator;
  const ControlMathResult<double> step =
      IntegrateBackCalculation(integrator, 1, 0, 0, 1, 0.1);
  EXPECT_EQ(step.status().message, kIntegratorInvalidMessage);
  EXPECT_EQ(integrator.state(), 0.0);
  EXPECT_FALSE(integrator.valid());
}

}  // namespace
