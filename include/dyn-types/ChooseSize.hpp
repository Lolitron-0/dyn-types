#pragma once
#include <bits/stdc++.h>

#ifndef SIZES
#define SIZES
#endif

template <size_t Rows, size_t Cols>
struct Size {
  static constexpr auto rows = Rows;
  static constexpr auto cols = Cols;
};

#define S(n, m) Size<n, m>

template <typename F, typename... Ts, size_t... Is>
auto chooseSize(size_t n, size_t m, F f) {
}

void mapSize(size_t n, size_t m, auto f) {

  []<typename... Ts, size_t... Is>(size_t n, size_t m, auto f,
                                   std::index_sequence<Is...>) {
    bool found  = false;
    using Tuple = std::tuple<Ts...>;
    (
        [&]<size_t i> {
          if (!found && n == std::tuple_element_t<i, Tuple>::rows &&
              m == std::tuple_element_t<i, Tuple>::cols) {
            found = true;
            f.template operator()<std::tuple_element_t<i, Tuple>>();
          }
        }.template operator()<Is>(),
        ...);

    if (!found) {
      std::cout << "Non precompiled size used\n";
      f.template operator()<Size<0, 0>>();
    }
  }.template operator()<SIZES>(n, m, f, std::index_sequence_for<SIZES>{});
}

