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
 * @file utils.cc
 * @author W. Nicholas Greene
 * @date 2017-07-26 17:24:19 (Wed)
 */

#include "dataset_utils/utils.h"

#include <fstream>
#include <algorithm>
#include <sstream>

namespace dataset_utils {

std::vector<std::string> readLines(const std::string& file) {
  std::ifstream stream(file.c_str());
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(stream, line)) {
    // Remove carriage returns.
    line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
    lines.push_back(line);
  }

  return lines;
}

std::vector<std::string> split(const std::string &str, char delim) {
  std::vector<std::string> tokens;
  std::stringstream ss;
  ss.str(str);
  std::string token;
  while (std::getline(ss, token, delim)) {
    tokens.push_back(token);
  }

  return tokens;
}

}  // namespace dataset_utils
