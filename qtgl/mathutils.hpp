#pragma once

namespace qtgl {

struct MathUtils {
  constexpr static double PI = 3.1415926;
  inline static double toRadians(double d) { return d * (PI / 180); }
  inline static double toDegree(double r) { return r * (180 / PI); }
  inline static double limit(double v, double l, double r) { return v > r ? r : (v < l ? l : v); }

  template <typename T>
  inline static bool equal(T a, T b, T bias) {
    return a >= b - bias && a <= b + bias;
  }
};

}  // namespace qtgl