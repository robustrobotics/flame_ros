/**
 * This file is part of flame_ros.
 * Copyright (C) 2017 W. Nicholas Greene (wng@csail.mit.edu)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * @file utils.h
 * @author W. Nicholas Greene
 * @date 2017-07-26 17:22:39 (Wed)
 */

#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <tuple>

#include <yaml-cpp/yaml.h>

namespace dataset_utils {

/**
 * @brief Associate two vectors of arbitrary types.
 *
 * Adapted from associate.py in the TUM utilities.
 *
 * @tparam T Type of first vector.
 * @tparam U Type of second vector.
 * @tparam DiffFunctor Returns difference between elements of two vectors.
 * @param[in] a First vector.
 * @param[in] b Second vector.
 * @param[out] aidxs Indexes into a that are associated.
 * @param[out] bidxs Indexes into b that are associated.
 * @param[in] diff Functor that computes difference between vector elements.
 * @param[in] max_diff Maximum allowed difference to count as a match.
 */
template <typename T, typename U, typename DiffFunctor>
void associate(const std::vector<T>& a, const std::vector<U>& b,
               std::vector<std::size_t>* aidxs, std::vector<std::size_t>* bidxs,
               DiffFunctor diff, float max_diff = 0.02f) {
  // Find potential matches.
  std::vector<std::tuple<float, std::size_t, std::size_t> > potential_matches;
  for (std::size_t aidx = 0; aidx < a.size(); ++aidx) {
    for (std::size_t bidx = 0; bidx < b.size(); ++bidx) {
      float diffab = diff(a[aidx], b[bidx]);
      if (diffab < max_diff) {
        potential_matches.emplace_back(diffab, aidx, bidx);
      }
    }
  }

  // Sort potential matches by distance.
  auto comp = [](const std::tuple<float, std::size_t, std::size_t>& x,
                 const std::tuple<float, std::size_t, std::size_t>& y) {
    return std::get<0>(x) < std::get<0>(y);
  };
  std::sort(potential_matches.begin(), potential_matches.end(), comp);

  // Select best matches with no repeats.
  std::unordered_set<std::size_t> as, bs;
  aidxs->clear();
  bidxs->clear();
  for (const auto& match : potential_matches) {
    std::size_t aidx = std::get<1>(match);
    std::size_t bidx = std::get<2>(match);

    if ((as.count(aidx) == 0) && (bs.count(bidx) == 0)) {
      aidxs->push_back(aidx);
      bidxs->push_back(bidx);
      as.insert(aidx);
      bs.insert(bidx);
    }
  }

  // Finally sort selected matches.
  std::sort(aidxs->begin(), aidxs->end());
  std::sort(bidxs->begin(), bidxs->end());

  return;
}

/**
 * @brief Read a file and return the lines in a vector.
 *
 * @param[in] file Input filename.
 */
std::vector<std::string> readLines(const std::string& file);

/**
 * @brief Split a string with given delimiter.
 *
 * Adapted from:
 *   https://stackoverflow.com/questions/236129/most-elegant-way-to-split-a-string
 *
 * @param[in] str Input string.
 * @param[in] delim Delimiter that separates tokens.
 */
std::vector<std::string> split(const std::string &str, char delim = ' ');

/**
 * @brief Read a matrix of a given size from a yaml node.
 *
 * @tparam Scalar Element type of matrix.
 * @param[in] yaml Yaml node.
 * @param[in] name Name of matrix in yaml node.
 * @param[in] rows Expected matrix rows.
 * @param[in] cols Expected matrix cols.
 * @param[out] mat Matrix in row-major order (must be pre-allocated).
 */
template <typename Scalar>
bool readMatrix(const YAML::Node& yaml, const std::string& name,
                const int rows, const int cols, Scalar* mat) {
  if ((rows != yaml[name]["rows"].as<int>()) ||
      (cols != yaml[name]["cols"].as<int>())) {
    return false;
  }

  for (int ii = 0; ii < rows; ++ii) {
    for (int jj = 0; jj < cols; ++jj) {
      int idx = ii * cols + jj;
      mat[idx] = yaml[name]["data"][idx].as<double>();
    }
  }

  return true;
}

/**
 * @brief Recursion base case.
 *
 * Converts a delimited stringstream to an output argument.
 *
 * @param[in] ss Input stringstream.
 * @param[in] delim Delimeter character.
 * @param[out] t0 Output value.
 */
template <typename T0>
void parseStream(std::stringstream& ss, char delim, T0* t0) {
  std::string token;
  std::getline(ss, token, delim);
  std::stringstream sst(token);
  sst >> (*t0);
  return;
}

/**
 * @brief Recursive variadic template function to parse stringstream.
 *
 * Uses template metaprogramming magic to parse a delimited stringstream into
 * given types.
 *
 * @param[in] ss Input stringstream.
 * @param[in] delim Delimeter character.
 * @param[out] t0 Output value that will be set.
 * @param[out] t1 Output value that will be recursed.
 * @param[out] t2s Output values that will be recursed.
 */
template <typename T0, typename T1, typename ... T2s>
void parseStream(std::stringstream& ss, char delim, T0* t0, T1* t1, T2s* ... t2s) {
  std::string token;
  std::getline(ss, token, delim);
  std::stringstream sst(token);
  sst >> (*t0);
  parseStream<T1, T2s...>(ss, delim, t1, t2s...);
  return;
}

/**
 * @brief Parse a delimited string into different types.
 *
 * Uses template metaprogramming magic to parse a delimited string into given
 * types.
 *
 * Usage:
 *  int a;
 *  float b;
 *  char c;
 *  parse("1,1.1,a", ',', &a, &b, &c);
 *
 * @param[in] line Input string.
 * @param[in] delim Delimiter character.
 * @param[out] args Output arguments.
 */
template <typename ... Ts>
void parse(const std::string& line, char delim, Ts* ... args) {
  std::stringstream ss(line);
  parseStream<Ts...>(ss, delim, args...);
  return;
}

}  // namespace dataset_utils
