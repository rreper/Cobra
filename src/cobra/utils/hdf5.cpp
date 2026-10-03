#include <pntos/cobra/utils/hdf5.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace pntos::cobra::utils {

namespace {
constexpr std::uint64_t kUndef = ~std::uint64_t{0};

struct Buf {
  std::vector<std::uint8_t> b;
  std::size_t pos() const { return b.size(); }
  void u8(std::uint8_t v) { b.push_back(v); }
  void u16(std::uint16_t v) { for (int i = 0; i < 2; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i))); }
  void u32(std::uint32_t v) { for (int i = 0; i < 4; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i))); }
  void u64(std::uint64_t v) { for (int i = 0; i < 8; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i))); }
  void bytes(const void* p, std::size_t n) {
    const auto* c = static_cast<const std::uint8_t*>(p);
    b.insert(b.end(), c, c + n);
  }
  void zeros(std::size_t n) { b.insert(b.end(), n, 0); }
  void pad8() { while (b.size() % 8) b.push_back(0); }
  void patch_u64(std::size_t at, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) b[at + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(v >> (8 * i));
  }
};

std::size_t round8(std::size_t n) { return (n + 7) & ~std::size_t{7}; }

// Object header v1 with a list of (type, data) messages. Returns the header's address.
std::uint64_t write_object_header(Buf& f, const std::vector<std::pair<std::uint16_t, std::vector<std::uint8_t>>>& msgs) {
  f.pad8();
  const std::uint64_t addr = f.pos();
  std::size_t chunk = 0;
  for (const auto& m : msgs) chunk += 8 + round8(m.second.size());
  f.u8(1);  // version
  f.u8(0);
  f.u16(static_cast<std::uint16_t>(msgs.size()));
  f.u32(1);  // reference count
  f.u32(static_cast<std::uint32_t>(chunk));
  f.zeros(4);  // align the message chunk to 8 bytes
  for (const auto& m : msgs) {
    f.u16(m.first);
    f.u16(static_cast<std::uint16_t>(round8(m.second.size())));
    f.u8(0);  // flags
    f.zeros(3);
    f.bytes(m.second.data(), m.second.size());
    f.zeros(round8(m.second.size()) - m.second.size());
  }
  return addr;
}

std::vector<std::uint8_t> dataspace_message(const std::vector<std::uint64_t>& dims) {
  Buf m;
  m.u8(1);  // version 1
  m.u8(static_cast<std::uint8_t>(dims.size()));
  m.u8(0);  // flags: no max dims
  m.zeros(5);
  for (auto d : dims) m.u64(d);
  return m.b;
}

std::vector<std::uint8_t> datatype_message(Hdf5Writer::Type type, std::uint32_t string_size) {
  Buf m;
  switch (type) {
    case Hdf5Writer::Type::Int64:
    case Hdf5Writer::Type::UInt8: {
      const bool is64 = type == Hdf5Writer::Type::Int64;
      m.u8(0x10 | 0);                // version 1, class 0 (fixed point)
      m.u8(is64 ? 0x08 : 0x00);      // bit0 LE, bit3 signed
      m.u8(0);
      m.u8(0);
      m.u32(is64 ? 8 : 1);           // size
      m.u16(0);                      // bit offset
      m.u16(is64 ? 64 : 8);          // precision
      break;
    }
    case Hdf5Writer::Type::Float64:
      m.u8(0x10 | 1);  // version 1, class 1 (floating point)
      m.u8(0x20);      // LE, mantissa normalisation = implied msb
      m.u8(63);        // sign bit location
      m.u8(0);
      m.u32(8);
      m.u16(0);     // bit offset
      m.u16(64);    // precision
      m.u8(52);     // exponent location
      m.u8(11);     // exponent size
      m.u8(0);      // mantissa location
      m.u8(52);     // mantissa size
      m.u32(1023);  // exponent bias
      break;
    case Hdf5Writer::Type::String:
      m.u8(0x10 | 3);  // version 1, class 3 (string)
      m.u8(0x01);      // null padded, ASCII (what numpy 'S' dtypes map to)
      m.u8(0);
      m.u8(0);
      m.u32(string_size);
      break;
  }
  return m.b;
}

std::vector<std::uint8_t> layout_message(std::uint64_t data_addr, std::uint64_t size) {
  Buf m;
  m.u8(3);  // version 3
  m.u8(1);  // contiguous
  m.u64(data_addr);
  m.u64(size);
  return m.b;
}
}  // namespace

void Hdf5Writer::add_dataset(const std::string& name, Type type, std::vector<std::uint64_t> dims,
                             std::vector<std::uint8_t> raw, std::uint32_t string_size) {
  if (name.empty() || name.find('/') != std::string::npos)
    throw std::invalid_argument("Hdf5Writer: dataset names must be non-empty and contain no '/'");
  for (const auto& d : datasets_)
    if (d.name == name) throw std::invalid_argument("Hdf5Writer: duplicate dataset name " + name);
  datasets_.push_back(Dataset{name, type, string_size, std::move(dims), std::move(raw)});
}

void Hdf5Writer::add_int64(const std::string& name, const std::vector<std::int64_t>& values) {
  std::vector<std::uint8_t> raw(values.size() * 8);
  if (!values.empty()) std::memcpy(raw.data(), values.data(), raw.size());
  add_dataset(name, Type::Int64, {values.size()}, std::move(raw));
}

void Hdf5Writer::add_float64(const std::string& name, const std::vector<std::uint64_t>& dims,
                             const std::vector<double>& values) {
  std::vector<std::uint8_t> raw(values.size() * 8);
  if (!values.empty()) std::memcpy(raw.data(), values.data(), raw.size());
  add_dataset(name, Type::Float64, dims, std::move(raw));
}

void Hdf5Writer::add_bools(const std::string& name, const std::vector<bool>& values) {
  std::vector<std::uint8_t> raw(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) raw[i] = values[i] ? 1 : 0;
  add_dataset(name, Type::UInt8, {values.size()}, std::move(raw));
}

void Hdf5Writer::add_strings(const std::string& name, std::vector<std::uint64_t> dims,
                             const std::vector<std::string>& values) {
  std::size_t width = 1;
  for (const auto& v : values) width = std::max(width, v.size());
  std::vector<std::uint8_t> raw(values.size() * width, 0);
  for (std::size_t i = 0; i < values.size(); ++i) std::memcpy(raw.data() + i * width, values[i].data(), values[i].size());
  add_dataset(name, Type::String, std::move(dims), std::move(raw), static_cast<std::uint32_t>(width));
}

std::vector<std::uint8_t> Hdf5Writer::serialise() const {
  // Symbol table nodes are binary-searched by name, so entries must be sorted.
  std::vector<const Dataset*> sorted;
  for (const auto& d : datasets_) sorted.push_back(&d);
  std::sort(sorted.begin(), sorted.end(), [](const Dataset* a, const Dataset* b) { return a->name < b->name; });
  const std::size_t n = sorted.size();
  const std::uint16_t leaf_k = static_cast<std::uint16_t>(std::max<std::size_t>(4, (n + 1) / 2));  // 2K >= n
  const std::uint16_t internal_k = 16;

  Buf f;
  // ---- superblock (v0), 96 bytes
  const std::uint8_t sig[8] = {0x89, 'H', 'D', 'F', '\r', '\n', 0x1a, '\n'};
  f.bytes(sig, 8);
  f.u8(0); f.u8(0); f.u8(0); f.u8(0);  // superblock, free-space, root entry, reserved
  f.u8(0); f.u8(8); f.u8(8); f.u8(0);  // shared header, sizeof offsets, sizeof lengths, reserved
  f.u16(leaf_k);
  f.u16(internal_k);
  f.u32(0);  // consistency flags
  f.u64(0);  // base address
  f.u64(kUndef);  // free-space info
  const std::size_t eof_at = f.pos();
  f.u64(0);  // end of file address (patched)
  f.u64(kUndef);  // driver info
  // root symbol table entry
  f.u64(0);  // link name offset
  const std::size_t root_oh_at = f.pos();
  f.u64(0);  // object header address (patched)
  f.u32(1);  // cache type: group with B-tree / heap in scratch
  f.u32(0);
  const std::size_t root_btree_at = f.pos();
  f.u64(0);  // B-tree address (patched)
  const std::size_t root_heap_at = f.pos();
  f.u64(0);  // local heap address (patched)

  // ---- local heap: "" then every dataset name, each 8-byte aligned, then a free block
  std::vector<std::uint64_t> name_offsets(n);
  Buf heap_data;
  heap_data.u8(0);
  heap_data.pad8();
  for (std::size_t i = 0; i < n; ++i) {
    name_offsets[i] = heap_data.pos();
    heap_data.bytes(sorted[i]->name.data(), sorted[i]->name.size());
    heap_data.u8(0);
    heap_data.pad8();
  }
  const std::uint64_t free_off = heap_data.pos();
  heap_data.u64(1);   // next free block: 1 = none
  heap_data.u64(16);  // size of this free block
  f.pad8();
  const std::uint64_t heap_addr = f.pos();
  f.bytes("HEAP", 4);
  f.u8(0);
  f.zeros(3);
  f.u64(heap_data.pos());
  f.u64(free_off);
  f.u64(heap_addr + 32);  // data segment follows the header
  f.bytes(heap_data.b.data(), heap_data.b.size());

  // ---- root object header: one symbol table message (B-tree + heap addresses, patched later)
  f.pad8();
  const std::size_t root_oh_pos = f.pos();
  {
    Buf st;
    st.u64(0);
    st.u64(heap_addr);
    write_object_header(f, {{0x0011, st.b}});
  }
  const std::size_t root_st_btree_at = root_oh_pos + 16 + 8;  // prefix 16 + message header 8

  // ---- symbol table node with every dataset entry (object header addresses patched later)
  f.pad8();
  const std::uint64_t snod_addr = f.pos();
  f.bytes("SNOD", 4);
  f.u8(1);
  f.u8(0);
  f.u16(static_cast<std::uint16_t>(n));
  std::vector<std::size_t> entry_oh_at(n);
  for (std::size_t i = 0; i < 2 * static_cast<std::size_t>(leaf_k); ++i) {
    if (i < n) {
      f.u64(name_offsets[i]);
      entry_oh_at[i] = f.pos();
      f.u64(0);
      f.u32(0);
      f.u32(0);
      f.zeros(16);
    } else {
      f.zeros(40);
    }
  }

  // ---- group B-tree (one leaf-level node pointing at the SNOD)
  f.pad8();
  const std::uint64_t btree_addr = f.pos();
  f.bytes("TREE", 4);
  f.u8(0);  // group node
  f.u8(0);  // level 0
  f.u16(n ? 1 : 0);
  f.u64(kUndef);
  f.u64(kUndef);
  f.u64(0);  // key 0: offset of ""
  f.u64(snod_addr);
  f.u64(n ? name_offsets[n - 1] : 0);  // key 1: largest name in the child
  // remaining (unused) key/child slots of a node sized for 2K entries
  f.zeros((2 * static_cast<std::size_t>(internal_k) - 1) * 16);

  // ---- datasets: object header then raw data
  for (std::size_t i = 0; i < n; ++i) {
    const Dataset& d = *sorted[i];
    f.pad8();
    const std::uint64_t oh_pos = f.pos();
    // header size is known in advance, so the data can follow it directly
    std::vector<std::pair<std::uint16_t, std::vector<std::uint8_t>>> msgs = {
        {0x0001, dataspace_message(d.dims)}, {0x0003, datatype_message(d.type, d.string_size)}, {0x0008, layout_message(0, d.raw.size())}};
    std::size_t header_size = 16;
    for (const auto& m : msgs) header_size += 8 + round8(m.second.size());
    const std::uint64_t data_addr = d.raw.empty() ? kUndef : round8(oh_pos + header_size);
    msgs[2].second = layout_message(data_addr, d.raw.size());
    write_object_header(f, msgs);
    f.pad8();
    f.bytes(d.raw.data(), d.raw.size());
    f.patch_u64(entry_oh_at[i], oh_pos);
  }
  f.pad8();

  f.patch_u64(eof_at, f.pos());
  f.patch_u64(root_oh_at, root_oh_pos);
  f.patch_u64(root_btree_at, btree_addr);
  f.patch_u64(root_heap_at, heap_addr);
  f.patch_u64(root_st_btree_at, btree_addr);
  return f.b;
}

void Hdf5Writer::write(const std::string& path) const {
  auto bytes = serialise();
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) throw std::runtime_error("Hdf5Writer: cannot open " + path + " for writing");
  out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!out) throw std::runtime_error("Hdf5Writer: write to " + path + " failed");
}

bool save_to_hdf5_file(const std::string& path, const std::map<std::string, std::vector<api::RegistryValue>>& store,
                       const Hdf5Log& log) {
  Hdf5Writer w;
  for (const auto& [key, values] : store) {
    if (values.empty()) continue;
    const std::size_t first = values.front().index();
    if (!std::all_of(values.begin(), values.end(), [&](const api::RegistryValue& v) { return v.index() == first; })) {
      log(api::LoggingLevel::WARN, "Multiple types within list is not supported (key \"" + key + "\").");
      return false;  // as the Python: abandon the file
    }
    const api::RegistryValue& v0 = values.front();
    if (std::holds_alternative<std::string>(v0)) {
      std::vector<std::string> s;
      for (const auto& v : values) s.push_back(std::get<std::string>(v));
      w.add_strings(key, {s.size()}, s);
    } else if (std::holds_alternative<api::StringArray>(v0)) {
      const std::size_t cols = std::get<api::StringArray>(v0).size();
      std::vector<std::string> s;
      bool ok = true;
      for (const auto& v : values) {
        const auto& arr = std::get<api::StringArray>(v);
        if (arr.size() != cols) ok = false;
        s.insert(s.end(), arr.begin(), arr.end());
      }
      if (!ok) {
        log(api::LoggingLevel::WARN, "String arrays of varying length are not supported (key \"" + key + "\").");
        continue;
      }
      w.add_strings(key, {values.size(), cols}, s);
    } else if (std::holds_alternative<bool>(v0)) {
      std::vector<bool> b;
      for (const auto& v : values) b.push_back(std::get<bool>(v));
      w.add_bools(key, b);
    } else if (std::holds_alternative<std::int64_t>(v0)) {
      std::vector<std::int64_t> i;
      for (const auto& v : values) i.push_back(std::get<std::int64_t>(v));
      w.add_int64(key, i);
    } else if (std::holds_alternative<double>(v0)) {
      std::vector<double> d;
      for (const auto& v : values) d.push_back(std::get<double>(v));
      w.add_float64(key, {d.size()}, d);
    } else if (std::holds_alternative<api::Matrix>(v0)) {
      const auto& m0 = std::get<api::Matrix>(v0);
      const auto rows = m0.rows(), cols = m0.cols();
      std::vector<double> d;
      d.reserve(values.size() * static_cast<std::size_t>(rows * cols));
      bool ok = true;
      for (const auto& v : values) {
        const auto& m = std::get<api::Matrix>(v);
        if (m.rows() != rows || m.cols() != cols) ok = false;
        for (Eigen::Index r = 0; r < m.rows(); ++r)
          for (Eigen::Index c = 0; c < m.cols(); ++c) d.push_back(m(r, c));  // C order
      }
      if (!ok) {
        log(api::LoggingLevel::WARN, "Arrays of varying shape are not supported (key \"" + key + "\").");
        continue;
      }
      // A registry Matrix with one column is the C++ form of a 1-D array (numpy shape (n,)), so it is
      // written as (N, n) like the Python; anything else keeps its (N, rows, cols) shape.
      if (cols == 1)
        w.add_float64(key, {values.size(), static_cast<std::uint64_t>(rows)}, d);
      else
        w.add_float64(key, {values.size(), static_cast<std::uint64_t>(rows), static_cast<std::uint64_t>(cols)}, d);
    } else {
      log(api::LoggingLevel::WARN, "Message values are pickled by the Python diagnostic log and are not written by this port (key \"" + key + "\").");
    }
  }
  try {
    w.write(path);
  } catch (const std::exception& e) {
    log(api::LoggingLevel::ERROR, e.what());
    return false;
  }
  return true;
}

}  // namespace pntos::cobra::utils
