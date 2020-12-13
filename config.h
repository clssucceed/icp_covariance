#pragma once
namespace icp_cov {
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = 1.0 / kRadToDeg;

constexpr double kTargetXCoordinateInEgo = 10;               // unit: m
constexpr double kTargetYCoordinateInEgo = 5;                // unit: m
constexpr double kEgoXRange = kTargetXCoordinateInEgo + 10;  // unit: m
constexpr double kEgoYRange = 20;                            // unit: m
constexpr double kLaserAngleResolution = 0.2;                // unit: degree
constexpr double kPixelResolution =
    kTargetXCoordinateInEgo * 0.2 * kDegToRad / 10;  // unit: m / pixel
}  // namespace icp_cov