#include "common_dbc.h"

namespace {

const Signal sigs_1361[] = {
    {
      .name = "CHECKSUM",
      .b1 = 40,
      .b2 = 8,
      .bo = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "GAS_COMMAND",
      .b1 = 0,
      .b2 = 16,
      .bo = 48,
      .is_signed = false,
      .factor = 0.0507968128,
      .offset = -22.85856576,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "GAS_COMMAND2",
      .b1 = 16,
      .b2 = 16,
      .bo = 32,
      .is_signed = false,
      .factor = 0.1015936256,
      .offset = -22.85856576,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "IDX",
      .b1 = 36,
      .b2 = 4,
      .bo = 24,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "ENABLE",
      .b1 = 32,
      .b2 = 1,
      .bo = 31,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_1362[] = {
    {
      .name = "CHECKSUM",
      .b1 = 40,
      .b2 = 8,
      .bo = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "INTERCEPTOR_GAS",
      .b1 = 0,
      .b2 = 16,
      .bo = 48,
      .is_signed = false,
      .factor = 0.0507968128,
      .offset = -22.85856576,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "INTERCEPTOR_GAS2",
      .b1 = 16,
      .b2 = 16,
      .bo = 32,
      .is_signed = false,
      .factor = 0.1015936256,
      .offset = -22.85856576,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "STATE",
      .b1 = 36,
      .b2 = 4,
      .bo = 24,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "IDX",
      .b1 = 32,
      .b2 = 4,
      .bo = 28,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};

const Msg msgs[] = {
  {
    .name = "GAS_COMMAND",
    .address = 0x551,
    .size = 6,
    .num_sigs = ARRAYSIZE(sigs_1361),
    .sigs = sigs_1361,
  },
  {
    .name = "GAS_SENSOR",
    .address = 0x552,
    .size = 6,
    .num_sigs = ARRAYSIZE(sigs_1362),
    .sigs = sigs_1362,
  },
};

const Val vals[] = {
};

}

const DBC tesla_can_pedal = {
  .name = "tesla_can_pedal",
  .num_msgs = ARRAYSIZE(msgs),
  .msgs = msgs,
  .vals = vals,
  .num_vals = ARRAYSIZE(vals),
};

dbc_init(tesla_can_pedal)