// clang-format off
#include <stdint.h>
// #include "transfer_functions.h"

#define START_TO_OUTPUT_MARGIN 0.01

float fuzzy_edge_remover(const float raw, const float highEdge, const float lowEdge, const float margin) {
  if (raw < lowEdge) {
    if (raw > -(margin) + lowEdge)
      return lowEdge;
    else
      return raw;
  } else if (raw > highEdge) {
    if (raw < highEdge + margin)
      return highEdge;
    else
      return raw;
  } else {
    if (raw < START_TO_OUTPUT_MARGIN + lowEdge)
      return lowEdge;
    else
      return raw;
  }
}
