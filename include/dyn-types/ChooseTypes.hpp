#pragma once
#include "dyn-types/Common.hpp"
#include "dyn-types/Fixed.hpp"
#include <bits/stdc++.h>

#ifndef TYPES
#define TYPES
#endif

#define TYPES_STRING STRINGIFY((TYPES))
constexpr static std::string_view typesString = TYPES_STRING;

// clang-format off
#define FIXED(n, k) Fixed<n,k>
#define FAST_FIXED(n, k) FastFixed<n,k>
// clang-format on
#define DOUBLE double
#define FLOAT float
#define TYPES_CORRECT TYPES

template <typename... Ts>
struct CountTypes;

template <typename T>
struct CountTypes<T> : std::integral_constant<size_t, 1> {};

template <typename T, typename... Ts>
struct CountTypes<T, Ts...>
    : std::integral_constant<size_t, 1 + CountTypes<Ts...>::value> {};

template <typename... Ts>
constexpr static auto CountTypes_v = CountTypes<Ts...>::value;

constexpr static auto TypesNum = CountTypes_v<TYPES_CORRECT>;

constexpr auto splitTypes(std::string_view typesStr) {
  std::array<std::string_view, CountTypes_v<TYPES_CORRECT>> res;
  size_t start   = 1;
  int wordsCount = 0;
  for (size_t i = 1; i < typesStr.length() - 1; i++) {
    if (typesStr[i] == '(') {
      while (typesStr[i] != ')') {
        i++;
      }
    }
    if (typesStr[i] == ',') {
      res[wordsCount] = typesStr.substr(start, i - start);
      wordsCount++;
      start = i + 1;
    }
  }
  res[wordsCount] = typesStr.substr(start, typesStr.length() - start - 1);
  return res;
}

constexpr std::array<std::string_view, TypesNum> typeNames =
    splitTypes(typesString);

template <typename F, typename... Ts, size_t... Is>
auto chooseFunc(std::string_view name, F f,
                std::index_sequence<Is...> /*unused*/) -> void {
  bool found = false;
  (
      [&]<size_t i> {
        if (!found && typeNames[i] == name) {
          found = true;
          f.template operator()<std::remove_cvref_t<
              std::tuple_element_t<i, std::tuple<Ts...>>>>();
        }
      }.template operator()<Is>(),
      ...);

  if (!found) {
    std::cerr << "Unknown type: " << name << std::endl;
    std::cerr << "Expected one of: " << typesString << std::endl;
    abort();
  }
}

void map(std::string_view arg, auto f) {
  chooseFunc<decltype(f), TYPES_CORRECT>(
      arg, f, std::index_sequence_for<TYPES_CORRECT>{});
}
