#pragma once

#include <vector>
#include "primitive.hpp"

namespace qtgl {

class GLModel {
 public:
  virtual std::vector<GLPrimitive> getPrimitives() = 0;
};

}  // namespace qtgl