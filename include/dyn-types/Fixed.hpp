#pragma once
#include <bits/stdc++.h>

template <size_t N>
struct ChooseBaseType
    : std::type_identity<std::conditional_t<
          (N == 8), int8_t,
          std::conditional_t<
              (N == 16), int16_t,
              std::conditional_t<
                  (N == 32), int32_t,
                  std::conditional_t<(N == 64), int64_t, void>>>>> {};

template <size_t N>
struct ChooseFastBaseType
    : std::type_identity<std::conditional_t<
          (N <= 8), int_fast8_t,
          std::conditional_t<
              (N <= 16), int_fast16_t,
              std::conditional_t<
                  (N <= 32), int_fast32_t,
                  std::conditional_t<(N <= 64), int_fast64_t, void>>>>> {
};

template <size_t N, size_t K, bool fast = false>
struct Fixed {
  using BaseT =
      std::conditional_t<fast, typename ChooseFastBaseType<N>::type,
                         typename ChooseBaseType<N>::type>;

  static_assert(!std::is_void_v<BaseT>, "Size not supported");

  template <size_t N2, size_t K2, bool F2>
  constexpr Fixed(const Fixed<N2, K2, F2>& other) noexcept {
    if constexpr (K > K2) {
      v = other.v << (K - K2);
    } else {
      v = other.v >> (K2 - K);
    }
  }
  constexpr Fixed(int v) noexcept
      : v(static_cast<uint64_t>(v) << K) {
  }
  template <std::floating_point T>
  constexpr Fixed(T f) noexcept
      : v(f * (1ULL << K)) {
  }
  constexpr Fixed()
      : v(0) {
  }

  Fixed& operator+=(const Fixed& other) {
    v += other.v;
    return *this;
  }

  Fixed& operator-=(const Fixed& other) {
    v -= other.v;
    return *this;
  }

  Fixed& operator*=(const Fixed& other) {
    v = (static_cast<__int128_t>(v) * other.v) >> K;
    return *this;
  }

  Fixed& operator/=(const Fixed& other) {
    if (other.v == 0) {
      throw std::runtime_error("division by zero");
    }
    v = (static_cast<__int128_t>(v) << K) / other.v;
    return *this;
  }

  template <std::floating_point T>
  constexpr explicit operator T() const noexcept {
    return static_cast<T>(v) / (1ULL << K);
  }

  static constexpr auto from_raw(int32_t x) noexcept -> Fixed {
    Fixed ret;
    ret.v = x;
    return ret;
  }

  constexpr auto operator-() const {
    return Fixed::from_raw(-v);
  }

  auto operator<=>(const Fixed&) const = default;
  bool operator==(const Fixed&) const  = default;

  BaseT v;
};

template <typename T, size_t N, size_t K, bool F>
concept ConvertibleToFixed = std::constructible_from<Fixed<N, K, F>, T>;

template <size_t N, size_t K>
using FastFixed = Fixed<N, K, true>;

template <size_t N, size_t K, bool F, ConvertibleToFixed<N, K, F> T>
auto operator<=>(const Fixed<N, K, F>& a, T b) {
  return a <=> Fixed<N, K, F>(b);
}

template <size_t N1, size_t K1, bool F1, std::floating_point T>
auto operator<=>(T a, const Fixed<N1, K1, F1>& b) {
  return a <=> static_cast<T>(b);
}

template <size_t N, size_t K, bool F, ConvertibleToFixed<N, K, F> T>
auto operator==(const Fixed<N, K, F>& a, T b) -> bool {
  return a == Fixed<N, K, F>(b);
}

template <size_t N1, size_t K1, bool F1, std::floating_point T>
auto operator==(T a, const Fixed<N1, K1, F1>& b) -> bool {
  return a == static_cast<T>(b);
}

template <size_t N2, size_t K2, bool F2, std::floating_point T>
T& operator+=(T& a, Fixed<N2, K2, F2> b) {
  return a += static_cast<T>(b);
}

template <size_t N2, size_t K2, bool F2, std::floating_point T>
T& operator-=(T& a, Fixed<N2, K2, F2> b) {
  return a -= static_cast<T>(b);
}

template <size_t N2, size_t K2, bool F2, std::floating_point T>
T& operator*=(T& a, Fixed<N2, K2, F2> b) {
  return a *= static_cast<T>(b);
}

template <size_t N2, size_t K2, bool F2, std::floating_point T>
T& operator/=(T& a, Fixed<N2, K2, F2> b) {
  return a /= static_cast<T>(b);
}

template <size_t N, size_t K, bool F, ConvertibleToFixed<N, K, F> T>
Fixed<N, K, F> operator+(Fixed<N, K, F> a, T b) {
  return a += Fixed<N, K, F>(b);
}

template <size_t N, size_t K, bool F, std::floating_point T>
T operator+(T a, Fixed<N, K, F> b) {
  return a += b;
}

template <size_t N, size_t K, bool F, ConvertibleToFixed<N, K, F> T>
Fixed<N, K, F> operator-(Fixed<N, K, F> a, T b) {
  return a -= Fixed<N, K, F>(b);
}

template <size_t N, size_t K, bool F, std::floating_point T>
T operator-(T a, Fixed<N, K, F> b) {
  return a -= b;
}

template <size_t N, size_t K, bool F, ConvertibleToFixed<N, K, F> T>
Fixed<N, K, F> operator*(Fixed<N, K, F> a, T b) {
  return a *= Fixed<N, K, F>(b);
}

template <size_t N, size_t K, bool F, std::floating_point T>
T operator*(T a, Fixed<N, K, F> b) {
  return a *= b;
}

template <size_t N, size_t K, bool F, ConvertibleToFixed<N, K, F> T>
Fixed<N, K, F> operator/(Fixed<N, K, F> a, T b) {
  return a /= Fixed<N, K, F>(b);
}

template <size_t N, size_t K, bool F, std::floating_point T>
T operator/(T a, Fixed<N, K, F> b) {
  return a /= b;
}
