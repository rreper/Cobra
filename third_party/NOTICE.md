# Third-party code vendored into this repository

| Directory | Origin | License | Why vendored |
|---|---|---|---|
| `lcm/lcm/lcm_coretypes.h` | LCM (https://github.com/lcm-proj/lcm), 1.5.x | LGPL-2.1-or-later | The generated LCM message classes need only this header to encode/decode; vendoring it avoids building liblcm (CMake, glib). Used unmodified, by inclusion only. |
| `nlohmann/json.hpp` | JSON for Modern C++ (https://github.com/nlohmann/json), v3.11.3 single header | MIT | Config-file loading (`config/JsonConfig.hpp`) and registry dumps. Used unmodified. |
| `aspn23_lcm/aspn23_lcm/*.hpp` | aspn-generated (https://github.com/is4s/aspn-generated), `lcm/cpp/aspn23_lcm`, commit `8edae7eeccbab1313dea3520f1b08d9b7973a82d` | Apache-2.0 | lcm-gen output for the ASPN-23 messages, header-only. Copied so that it can be included without enabling aspn-generated's `lcm-generated-cpp` option (which requires liblcm). Regenerate by re-copying from the pinned subproject. |

Local modification: `aspn23_lcm/type_image_feature.hpp` uses an `int16_t` descriptor and the matching hash `0x961ed1e790449286`, which is what the `aspn23_lcm` Python package (and therefore the Cobra example log) encodes; the pinned aspn-generated commit had already changed the field to `byte`.
