#ifndef KC_UTILS_HPP_
#define KC_UTILS_HPP_

#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

namespace kc
{
  [[nodiscard]] inline auto count_lines(std::ifstream &istrm) -> std::size_t
  {
    return static_cast<std::size_t>(
        std::count(std::istreambuf_iterator<char>(istrm),
                   std::istreambuf_iterator<char>(), '\n'));
  }

  template<typename T>
  [[nodiscard]] inline auto read_xyz(const std::string &filename) -> std::vector<T>
  {
    std::ifstream istrm(filename, std::ios::in);
    if (!istrm.is_open())
    {
      std::cerr << "Could not open file " << filename << "\n";
      std::exit(EXIT_FAILURE);
    }

    const std::size_t n_lines = count_lines(istrm);
    istrm.clear();
    istrm.seekg(0);

    std::vector<T> out;
    out.reserve(n_lines);

    T t;
    while (istrm >> t)
      out.emplace_back(t);

    return out;
  }

  template<std::size_t N>
  void report(const double *const a, const double *const alpha, const double *const d, const double *const theta)
  {
    std::cout << std::right << std::setw(4) << "Link"
              << std::right << std::setw(14) << "a [m]"
              << std::right << std::setw(19) << "alpha [rad]"
              << std::right << std::setw(13) << "d [m]"
              << std::right << std::setw(19) << "theta [rad]"
              << "\n";
    for (std::size_t i{}; i < N; ++i)
    {
      std::cout << std::right << std::setw(4) << i + 1 << "  "
                << std::right << std::fixed << std::setprecision(6) << std::setw(13) << a[i] << "   "
                << std::right << std::fixed << std::setprecision(6) << std::setw(13) << alpha[i] << "   "
                << std::right << std::fixed << std::setprecision(6) << std::setw(13) << d[i] << "   "
                << std::right << std::fixed << std::setprecision(6) << std::setw(13) << theta[i] << "\n";
    }
    std::cout << "\n\n";
  }

} // namespace kc

#endif