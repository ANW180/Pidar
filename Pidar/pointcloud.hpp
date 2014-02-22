//
// stock.hpp
// ~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef SERIALIZATION_STOCK_HPP
#define SERIALIZATION_STOCK_HPP

#include <string>

namespace pointcloud_connection {

/// Structure to hold information about a single stock.
struct pcl_data
{
  int id;
  double x[1080];
  double y[1080];
  double z[1080];

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & id;
    ar & x;
    ar & y;
    ar & z;
  }
};

} //

#endif // SERIALIZATION_STOCK_HPP
