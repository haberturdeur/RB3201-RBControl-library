#pragma once

#include "angle.hpp"

namespace rb {

// Polyfill for the original angle
using lx16a::Angle;
using lx16a::operator""_deg;
using lx16a::operator""_rad;

}; // namespace rb
