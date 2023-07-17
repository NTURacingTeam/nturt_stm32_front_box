// clang-format off
#include <stdint.h>
// #include "transfer_functions.h"

#define START_TO_OUTPUT_MARGIN 0.01
#define OUT_OF_BOUNDS_MARGIN 0.01  // TODO: where do we put these fuzzy bound constants

float fuzzy_edge_remover(const float raw, const float highEdge,
                         const float lowEdge) {
  if (raw < lowEdge) {
    if (raw > -(OUT_OF_BOUNDS_MARGIN) + lowEdge)
      return lowEdge;
    else
      return raw;
  } else if (raw > highEdge) {
    if (raw < highEdge + OUT_OF_BOUNDS_MARGIN)
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
