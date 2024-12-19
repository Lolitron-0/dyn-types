#include "dyn-types/ChooseTypes.hpp"
#include "dyn-types/Fixed.hpp"
#include <array>
#include <bits/stdc++.h>
#include <cxxopts.hpp>
#include <iterator>
#include <random>
#include <tuple>
#include <type_traits>
#include <utility>

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

template <typename T, typename Size>
class Matrix {
  static constexpr bool Fast = (Size::rows * Size::cols > 0);
  using ArrayType =
      std::conditional_t<Fast, std::array<T, Size::rows * Size::cols>,
                         std::vector<T>>;

public:
  Matrix(size_t rows, size_t cols)
      : rows{ rows },
        cols{ cols } {
    if constexpr (!Fast) {
      data.resize(rows * cols, T{});
    }
  }

  T& operator()(size_t i, size_t j) {
    if constexpr (Fast) {
      return data[i * Size::cols + j];
    } else {
      return data[i * cols + j];
    }
  }

  void clear() {
    std::fill(std::begin(data), std::end(data), T{});
  }

  [[nodiscard]] auto getRows() const -> size_t {
    return rows;
  }
  [[nodiscard]] auto getCols() const -> size_t {
    return cols;
  }

private:
  ArrayType data;
  size_t rows;
  size_t cols;
};

// constexpr size_t N = 36, M = 84;
// constexpr size_t N = 14, M = 5;
constexpr size_t T = 1'000'000;
constexpr std::array<std::pair<int, int>, 4> deltas{
  { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } }
};

template <typename T, typename Size>
struct VectorField {
  VectorField(size_t rows, size_t cols)
      : v(rows, cols) {
  }

  Matrix<std::array<T, deltas.size()>, Size> v;
  T& add(int x, int y, int dx, int dy, T dv) {
    return get(x, y, dx, dy) += dv;
  }

  T& get(int x, int y, int dx, int dy) {
    size_t i =
        std::ranges::find(deltas, std::pair(dx, dy)) - deltas.begin();
    assert(i < deltas.size());
    return v(x, y)[i];
  }

  void clear() {
    v.clear();
  }
};

double random01() {
  static std::mt19937 rnd(1337);
  static std::uniform_real_distribution<double> dist(0, 1);
  return dist(rnd);
}

template <typename GType, typename Size>
struct FluidSimProps {
  GType g;
  Matrix<char, Size> field;
};

template <typename PType, typename VelocityType, typename VelFlowType,
          typename Size>
class FluidSim {
  using GType   = VelocityType;
  using RhoType = VelocityType;

  struct ParticleParams {
    char type{};
    PType cur_p;
    std::array<VelocityType, deltas.size()> v;
    Matrix<char, Size>& field;
    VectorField<VelocityType, Size>& velocity;
    Matrix<PType, Size>& p;

    explicit ParticleParams(Matrix<char, Size>& field,
                            VectorField<VelocityType, Size>& velocity,
                            Matrix<PType, Size>& p)
        : field(field),
          velocity(velocity),
          p(p){};

    void swap_with(int x, int y) {
      std::swap(field(x, y), type);
      std::swap(p(x, y), cur_p);
      swap(velocity.v(x, y), v);
    }
  };

private:
  GType g;
  size_t N, M;
  size_t curStep{ 1 };
  Matrix<Fixed<32, 0>, Size> dirs{};
  Matrix<char, Size> field;
  VectorField<VelocityType, Size> velocity{};
  VectorField<VelFlowType, Size> velocity_flow{};
  Matrix<int, Size> last_use{};
  int UT = 0;
  std::array<RhoType, 256> rho;

  Matrix<PType, Size> p{}, old_p;

public:
  explicit FluidSim(const FluidSimProps<GType, Size>& props)
      : g{ props.g },
        N{ props.field.getRows() },
        M{ props.field.getCols() },
        field(props.field),
        p(N, M),
        old_p(N, M),
        dirs(N, M),
        velocity(N, M),
        velocity_flow(N, M),
        last_use(N, M) {
    // clang-format off
      

    rho[' '] = 0.01;
    rho['.'] = 1000;

    for (size_t x = 0; x < N; ++x) {
      for (size_t y = 0; y < M; ++y) {
        if (field(x,y) == '#') {
          continue;
        }
        for (auto [dx, dy] : deltas) {
          dirs(x,y) +=
              static_cast<std::remove_cvref_t<decltype(dirs(x, y))>>(
                  field(x + dx,y + dy) != '#');
        }
      }
    }
  }

  void step() {
    PType total_delta_p = 0;
    // Apply external forces
    for (size_t x = 0; x < N; ++x) {
      for (size_t y = 0; y < M; ++y) {
        if (field(x,y) == '#') {
          continue;
        }
        if (field(x + 1,y) != '#') {
          velocity.add(x, y, 1, 0, g);
        }
      }
    }

    // Apply forces from p
    old_p = p;
    for (size_t x = 0; x < N; ++x) {
      for (size_t y = 0; y < M; ++y) {
        if (field(x,y) == '#') {
          continue;
        }
        for (auto [dx, dy] : deltas) {
          int nx = x + dx;
          int ny = y + dy;
          if (field(nx,ny) != '#' && old_p(nx,ny) < old_p(x,y)) {
            auto delta_p = old_p(x,y) - old_p(nx,ny);
            auto force   = delta_p;
            auto& contr  = velocity.get(nx, ny, -dx, -dy);
            if (contr * rho[(int)field(nx,ny)] >= force) {
              contr -= force / rho[(int)field(nx,ny)];
              continue;
            }
            force -= contr * rho[(int)field(nx,ny)];
            contr = 0;
            velocity.add(x, y, dx, dy,
                         VelocityType(force / rho[(int)field(x, y)]));
            p(x,y) -= force / dirs(x,y);
            total_delta_p -= force / dirs(x,y);
          }
        }
      }
    }

    // Make flow from velocities
    velocity_flow.clear();
    bool prop     = false;
    do {
      UT += 2;
      prop = false;
      for (size_t x = 0; x < N; ++x) {
        for (size_t y = 0; y < M; ++y) {
          if (field(x, y) != '#' && last_use(x, y) != UT) {
            auto [t, local_prop, _] = propagate_flow(x, y, 1);
            if (t > 0) {
              prop = true;
            }
          }
        }
      }
    } while (prop);

    // Recalculate p with kinetic energy
    for (size_t x = 0; x < N; ++x) {
      for (size_t y = 0; y < M; ++y) {
        if (field(x, y) == '#') {
          continue;
        }
        for (auto [dx, dy] : deltas) {
          auto old_v = velocity.get(x, y, dx, dy);
          auto new_v = velocity_flow.get(x, y, dx, dy);
          if (old_v > 0) {
            assert(new_v <= old_v);
            velocity.get(x, y, dx, dy) = VelocityType(new_v);
            auto force =
                (old_v - new_v) * rho[static_cast<int>(field(x, y))];
            if (field(x, y) == '.') {
              force *= 0.8;
            }
            if (field(x + dx, y + dy) == '#') {
              p(x, y) += force / dirs(x, y);
              total_delta_p += force / dirs(x, y);
            } else {
              p(x + dx, y + dy) += force / dirs(x + dx, y + dy);
              total_delta_p += force / dirs(x + dx, y + dy);
            }
          }
        }
      }
    }

    UT += 2;
    prop = false;
    for (size_t x = 0; x < N; ++x) {
      for (size_t y = 0; y < M; ++y) {
        if (field(x, y) != '#' && last_use(x, y) != UT) {
          if (random01() < move_prob(x, y)) {
            prop = true;
            propagate_move(x, y, true);
          } else {
            propagate_stop(x, y, true);
          }
        }
      }
    }

    if (prop) {
      std::cout << "Tick " << curStep << ":\n";
      for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) {
        std::cout << field(i, j) ;
        }
        std::cout << '\n';
      }
    }

    curStep++;
  }

private:
  std::tuple<VelFlowType, bool, std::pair<int, int>> propagate_flow(
      int x, int y, VelFlowType lim) {
    last_use(x, y)  = UT - 1;
    VelFlowType ret = 0;
    for (auto [dx, dy] : deltas) {
      int nx = x + dx;
      int ny = y + dy;
      if (field(nx, ny) != '#' && last_use(nx, ny) < UT) {
        auto cap  = velocity.get(x, y, dx, dy);
        auto flow = velocity_flow.get(x, y, dx, dy);
        if (flow == cap) {
          continue;
        }
        // assert(v >= velocity_flow.get(x, y, dx, dy));
        auto vp = std::min(lim, decltype(lim)(cap - flow));
        if (last_use(nx, ny) == UT - 1) {
          velocity_flow.add(x, y, dx, dy, vp);
          last_use(x, y) = UT;
          // cerr << x << " " << y << " -> " << nx << " " << ny << " "
          // << vp
          // << " / " << lim << "\n";
          return { vp, 1, { nx, ny } };
        }
        auto [t, prop, end] = propagate_flow(nx, ny, vp);
        ret += t;
        if (prop) {
          velocity_flow.add(x, y, dx, dy, t);
          last_use(x, y) = UT;
          // cerr << x << " " << y << " -> " << nx << " " << ny << " "
          // <<
          // t
          // << " / " << lim << "\n";
          return { t, prop && end != std::pair(x, y), end };
        }
      }
    }
    last_use(x, y) = UT;
    return { ret, 0, { 0, 0 } };
  }

  bool propagate_move(int x, int y, bool is_first) {
    last_use(x, y) = UT - is_first;
    bool ret       = false;
    int nx         = -1;
    int ny         = -1;
    do {
      std::array<VelocityType, deltas.size()> tres;
      VelocityType sum = 0;
      for (size_t i = 0; i < deltas.size(); ++i) {
        auto [dx, dy] = deltas[i];
        int nx        = x + dx;
        int ny        = y + dy;
        if (field(nx, ny) == '#' || last_use(nx, ny) == UT) {
          tres[i] = sum;
          continue;
        }
        auto v = velocity.get(x, y, dx, dy);
        if (v < 0) {
          tres[i] = sum;
          continue;
        }
        sum += v;
        tres[i] = sum;
      }

      if (sum == 0) {
        break;
      }

      VelocityType p = random01() * sum;
      size_t d       = std::ranges::upper_bound(tres, p) - tres.begin();

      auto [dx, dy] = deltas[d];
      nx            = x + dx;
      ny            = y + dy;
      assert(velocity.get(x, y, dx, dy) > 0 && field(nx, ny) != '#' &&
             last_use(nx, ny) < UT);

      ret = (last_use(nx, ny) == UT - 1 || propagate_move(nx, ny, false));
    } while (!ret);
    last_use(x, y) = UT;
    for (auto [dx, dy] : deltas) {
      int nx = x + dx;
      int ny = y + dy;
      if (field(nx, ny) != '#' && last_use(nx, ny) < UT - 1 &&
          velocity.get(x, y, dx, dy) < 0) {
        propagate_stop(nx, ny);
      }
    }
    if (ret) {
      if (!is_first) {
        ParticleParams pp{ field, velocity, p };
        pp.swap_with(x, y);
        pp.swap_with(nx, ny);
        pp.swap_with(x, y);
      }
    }
    return ret;
  }

  void propagate_stop(int x, int y, bool force = false) {
    if (!force) {
      bool stop = true;
      for (auto [dx, dy] : deltas) {
        int nx = x + dx;
        int ny = y + dy;
        if (field(nx, ny) != '#' && last_use(nx, ny) < UT - 1 &&
            velocity.get(x, y, dx, dy) > 0) {
          stop = false;
          break;
        }
      }
      if (!stop) {
        return;
      }
    }
    last_use(x, y) = UT;
    for (auto [dx, dy] : deltas) {
      int nx = x + dx;
      int ny = y + dy;
      if (field(nx, ny) == '#' || last_use(nx, ny) == UT ||
          velocity.get(x, y, dx, dy) > 0) {
        continue;
      }
      propagate_stop(nx, ny);
    }
  }

  VelocityType move_prob(int x, int y) {
    VelocityType sum = 0;
    for (auto [dx, dy] : deltas) {
      int nx = x + dx;
      int ny = y + dy;
      if (field(nx, ny) == '#' || last_use(nx, ny) == UT) {
        continue;
      }
      auto v = velocity.get(x, y, dx, dy);
      if (v < 0) {
        continue;
      }
      sum += v;
    }
    return sum;
  }
};

template <typename T1, typename T2, typename T3>
void map(const char* name, std::function<void()> f) {
  std::cout << name << "\n";
  f();
}

auto main(int argc, char** argv) -> int {

  cxxopts::Options cliOptions{
    "dyn-types", "Fluid simulation with dynamic type choice"
  };
  std::string pTypeStr;
  std::string velocityTypeStr;
  std::string velFlowTypeStr;
  // clang-format off
  cliOptions.add_options()
      ("h,help", "Show help")
      ("p-type", "Type of p, required",
       cxxopts::value<std::string>(pTypeStr))
      ("v-type", "Type of velocity, required",
       cxxopts::value<std::string>(velocityTypeStr))
      ("v-flow-type", "Type of velocity flow, required", cxxopts::value<std::string>(velFlowTypeStr));
  // clang-format on

  auto result{ cliOptions.parse(argc, argv) };

  if (result.count("help")) {
    std::cout << cliOptions.help() << std::endl;
    return 0;
  }

  if (result.count("p-type") + result.count("v-type") +
          result.count("v-flow-type") <
      3) {
    std::cout << cliOptions.help() << std::endl;
    return 0;
  }

  std::vector<std::string> fieldLines;
  std::ifstream f{ "field.txt" };
  std::string line;
  size_t m = 0;
  while (std::getline(f, line)) {
    if (fieldLines.empty()) {
      m = line.size();
    }
    assert(line.size() == m);
    fieldLines.push_back(line);
  }
  map(pTypeStr, [&]<typename PType> {
    map(velocityTypeStr, [&]<typename VelocityType> {
      map(velFlowTypeStr, [&]<typename VelFlowType> {
        mapSize(fieldLines.size(), m, [&]<typename Size>() {
          Matrix<char, Size> field{ fieldLines.size(), m };
          for (int i = 0; i < fieldLines.size(); i++) {
            for (int j = 0; j < m; j++) {
              field(i, j) = fieldLines[i][j];
            }
          }

          FluidSimProps<VelocityType, Size> props{ 0.1, field };
          FluidSim<PType, VelocityType, VelFlowType, Size> sim(props);
          for (int i = 0; i < T; i++) {
            sim.step();
          }
        });
      });
    });
  });
}