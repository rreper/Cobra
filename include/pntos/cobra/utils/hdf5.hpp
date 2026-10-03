// Minimal, dependency-free HDF5 writer (port of the writing half of pntos.cobra.utils.hdf5).
//
// Produces a version-0 superblock file with a flat root group of contiguous datasets, readable by
// h5py / libhdf5 / MATLAB. Only what the diagnostic log needs is implemented: little-endian
// int64 / float64 / uint8 N-d arrays and fixed-length ASCII string arrays, no attributes, no
// compression, no groups below the root.
#pragma once

#include <pntos/api/api.hpp>

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace pntos::cobra::utils {

class Hdf5Writer {
 public:
  enum class Type { Int64, Float64, UInt8, String };

  /// Adds a dataset. `dims` are C-order (row-major) dimensions, `raw` the little-endian payload
  /// (strings: fixed-length `string_size` bytes each, null padded). Names must be unique.
  void add_dataset(const std::string& name, Type type, std::vector<std::uint64_t> dims, std::vector<std::uint8_t> raw,
                   std::uint32_t string_size = 0);
  void add_int64(const std::string& name, const std::vector<std::int64_t>& values);
  void add_float64(const std::string& name, const std::vector<std::uint64_t>& dims, const std::vector<double>& values);
  void add_bools(const std::string& name, const std::vector<bool>& values);
  /// Array of strings as one row (dims {1, n}) or several rows (dims {rows, n}) of fixed-length ASCII.
  void add_strings(const std::string& name, std::vector<std::uint64_t> dims, const std::vector<std::string>& values);

  /// Serialises the file. Throws std::runtime_error if the file cannot be written.
  void write(const std::string& path) const;
  std::vector<std::uint8_t> serialise() const;

  std::size_t size() const { return datasets_.size(); }

 private:
  struct Dataset {
    std::string name;
    Type type;
    std::uint32_t string_size;
    std::vector<std::uint64_t> dims;
    std::vector<std::uint8_t> raw;
  };
  std::vector<Dataset> datasets_;
};

/// Writes a registry-notification store (key -> every value seen) to `path` the way the Python
/// save_to_hdf5_file does: scalars become 1-D arrays, matrices N×(rows×cols) arrays, string arrays
/// fixed-length string matrices. Returns false (after logging) if nothing could be written. Keys
/// whose values are not all of one type, or hold Messages (pickled in Python), are skipped with a WARN.
using Hdf5Log = std::function<void(api::LoggingLevel, const std::string&)>;
bool save_to_hdf5_file(const std::string& path, const std::map<std::string, std::vector<api::RegistryValue>>& store,
                       const Hdf5Log& log);

}  // namespace pntos::cobra::utils
