#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <string_view>
#include <filesystem>

#include "Interactive.hpp"
#include "System.hpp"
#include "Mcm.hpp"
#include "Hart.hpp"
#include "Memory.hpp"
#include "HartConfig.hpp"
#include "DecodedInst.hpp"

using namespace WdRiscv;
namespace py = pybind11;

template <size_t N, size_t M>
static constexpr auto concat(const char* a, const char* b)
{
  std::array<char, N + M - 1> result = {};

  for (size_t i = 0; i < N - 1; ++i)
    result[i] = a[i];

  for (size_t j = 0; j < M; ++j)
    result[N - 1 + j] = b[j];

  return result;
}

template <typename T, size_t N>
static constexpr auto concat(const char (&a)[N])
{
  constexpr bool isRv32 = std::is_same_v<T, uint32_t>;
  return (isRv32)? concat<sizeof(a), 3>(a, "32") : concat<sizeof(a), 3>(a, "64");
}


struct File
{
  FILE* file_ = nullptr;
};


template <typename M>
static void defineFile(M m)
{
  auto m_file = m.def_submodule("file", "File I/O");

  auto fileName = "File";
  py::class_<File>(m_file, fileName)
    .def("__enter__", [](File& self) { return &self; })
    .def("__exit__", [](File& self, [[maybe_unused]] py::object exc_type, [[maybe_unused]] py::object exc_value, [[maybe_unused]] py::object traceback) {
        if (fclose(self.file_))
          throw std::runtime_error("file::close: failed to close file");
      });

  // Whisper operates on FILE*s, which does not exist as a concept in Python
  m_file.def("open", [](const std::filesystem::path& file) {
        FILE* f = fopen(file.c_str(), "w");
        if (not f)
          std::cerr << "Warning: file::open: failed to open file" << std::endl;
        return File(f);
      }, py::arg("Path to file to open."), py::doc("Opens a file for whisper use."));

  m_file.def("close", [](File file) {
        if (fclose(file.file_))
          throw std::runtime_error("file::close: failed to close file");
      }, py::arg("File to close."), py::doc("Closes a file."));
}


template <typename M>
static void defineEnums(M m)
{
  auto m_enum = m.def_submodule("enums", "trapEnums.hpp");
  py::enum_<PrivilegeMode>(m_enum, "PrivilegeMode")
    .value("User", PrivilegeMode::User)
    .value("Supervisor", PrivilegeMode::Supervisor)
    .value("Reserved", PrivilegeMode::Reserved)
    .value("Machine", PrivilegeMode::Machine)
    .export_values();
}


template <typename T>
static T memPeek(const Memory& self, uint64_t address)
{
  T value;
  if (not self.peek<T>(address, value, false))
    throw std::runtime_error("Memory::peek: failed to peek memory");
  return value;
}


template <typename T>
static void memPoke(Memory& self, uint64_t address, T data)
{
  if (not self.poke<T>(address, data, false))
    throw std::runtime_error("Memory::poke: failed to poke memory");
}


template <typename M>
static void defineMemory(M m)
{
  auto m_system = m.def_submodule("system", "System.hpp");
  auto m_memory = m_system.def_submodule("memory", "Memory.hpp");

  auto memoryName = "Memory";
  py::class_<Memory, std::shared_ptr<Memory>>(m_memory, memoryName)
    .def("peek8",  &memPeek<uint8_t>,  py::arg("address"), py::doc("Peek memory with data size of 1B."))
    .def("peek16", &memPeek<uint16_t>, py::arg("address"), py::doc("Peek memory with data size of 2B."))
    .def("peek32", &memPeek<uint32_t>, py::arg("address"), py::doc("Peek memory with data size of 4B."))
    .def("peek64", &memPeek<uint64_t>, py::arg("address"), py::doc("Peek memory with data size of 8B."))
    .def("poke8",  &memPoke<uint8_t>,  py::arg("address"), py::arg("data"), py::doc("Poke memory with data size of 1B."))
    .def("poke16", &memPoke<uint16_t>, py::arg("address"), py::arg("data"), py::doc("Poke memory with data size of 2B."))
    .def("poke32", &memPoke<uint32_t>, py::arg("address"), py::arg("data"), py::doc("Poke memory with data size of 4B."))
    .def("poke64", &memPoke<uint64_t>, py::arg("address"), py::arg("data"), py::doc("Poke memory with data size of 8B."));
}


template <typename M>
static void defineDecodedInst(M m)
{
  auto m_di = m.def_submodule("decoded_inst", "DecodedInst.hpp");

  auto name = "DecodedInst";
  py::class_<DecodedInst>(m_di, name)
    .def("inst", &DecodedInst::inst, py::doc("Get 32b instruction."))
    .def("address", &DecodedInst::address, py::doc("Get VA associated with instruction."))
    .def("phys_address", &DecodedInst::physAddress, py::doc("Get PA associated with instruction."))
    .def("ith_operand", &DecodedInst::ithOperand, py::arg("i"), py::doc("Get the ith operand."))
    .def("ith_operand_type", &DecodedInst::ithOperandType, py::arg("i"), py::doc("Get the ith operand type."))
    .def("ith_operand_mode", &DecodedInst::ithOperandMode, py::arg("i"), py::doc("Get the ith operand mode."));
}


template <typename T>
static auto attr(Hart<T>& self, const std::string& attr)
{
  unsigned reg;
  if (self.findIntReg(attr, reg))
    {
      T val;
      if (not self.peekIntReg(reg, val))
        throw py::attribute_error("Hart::__getattr__: invalid int register access");
      return py::cast(val);
    }
  else if (self.findFpReg(attr, reg))
    {
      uint64_t val;
      if (not self.peekFpReg(reg, val))
        throw py::attribute_error("Hart::__getattr__: invalid fp register access");
      return py::cast(val);
    }
  else if (self.findVecReg(attr, reg))
    {
      std::vector<uint8_t> val;
      if (not self.peekVecReg(reg, val))
        throw py::attribute_error("Hart::__getattr__: invalid vec register access");
      return py::cast(val);
    }

  auto csr = self.findCsr(attr);
  if (csr)
    {
      T val;
      if (not self.peekCsr(csr->getNumber(), val))
        throw py::attribute_error("Hart::__getattr__: invalid csr access");
      return py::cast(val);
    }
  else if (attr == "pc")
    {
      T val = self.peekPc();
      return py::cast(val);
    }
  else if (attr == "int_regs")
    {
      std::vector<T> vals;
      for (unsigned i = 0; i < self.intRegCount(); ++i)
        {
          T val;
          if (self.peekIntReg(i, val))
            vals.push_back(val);
        }
      return py::cast(vals);
    }
  else if (attr == "fp_regs")
    {
      std::vector<uint64_t> vals;
      for (unsigned i = 0; i < self.fpRegCount(); ++i)
        {
          uint64_t val;
          if (self.peekFpReg(i, val))
            vals.push_back(val);
        }
      return py::cast(vals);
    }
  else if (attr == "vec_regs")
    {
      std::vector<std::vector<uint8_t>> vals;
      for (unsigned i = 0; i < self.vecRegCount(); ++i)
        {
          std::vector<uint8_t> val;
          if (self.peekVecReg(i, val))
            vals.push_back(std::move(val));
        }
      return py::cast(vals);
    }
  else if (attr == "cs_regs")
    {
      std::unordered_map<std::string, T> vals;
      for (size_t i = 0; i <= size_t(CsrNumber::MAX_CSR_); ++i)
        {
          CsrNumber csr = static_cast<CsrNumber>(i);
          std::string_view name; T val;
          if (self.peekCsr(csr, val, name))
            vals[std::string(name)] = val;
        }
      return py::cast(vals);
    }
  else if (attr == "priv_mode")
    return py::cast(self.privilegeMode());
  else if (attr == "virt_mode")
    return py::cast(self.virtMode());
  throw py::attribute_error("Hart::__getattr__: unknown attribute");
}

template <typename T>
static void attr(Hart<T>& self, const std::string& attr, const py::object& value)
{
  unsigned reg;
  auto csr = self.findCsr(attr);

  if (self.findIntReg(attr, reg))
    {
      if (not py::isinstance<py::int_>(value))
        throw py::type_error("Hart::__setattr__: must assign int to int register");

      auto val = py::cast<T>(value);
      if (not self.pokeIntReg(reg, val))
        throw py::attribute_error("Hart::__setattr___: invalid int register access");
    }
  else if (self.findFpReg(attr, reg))
    {
      // we don't allow float assignment
      if (not py::isinstance<py::int_>(value))
        throw py::type_error("Hart::__setattr___: must assign int to fp register");

      auto val = py::cast<uint64_t>(value);
      if (not self.pokeFpReg(reg, val))
        throw py::attribute_error("Hart::__setattr___: invalid fp register access");
    }
  else if (self.findVecReg(attr, reg))
    {
      if (not py::isinstance<py::list>(value))
        throw py::type_error("Hart::__setattr___: must assign list of ints to vec register");

      auto val = py::cast<std::vector<uint8_t>>(value);
      if (not self.pokeVecReg(reg, val))
        throw py::attribute_error("Hart::__setattr___: invalid vec register access");
    }
  else if (csr)
    {
      if (not py::isinstance<py::int_>(value))
        throw py::type_error("Hart::__setattr___: must assign int to csr");

      auto val = py::cast<T>(value);
      if (not self.pokeCsr(csr->getNumber(), val))
        throw py::attribute_error("Hart::__setattr___: invalid csr access");
    }
  else if (attr == "pc")
    {
      if (not py::isinstance<py::int_>(value))
        throw py::type_error("Hart::__setattr__: must assign int to pc");

      auto val = py::cast<T>(value);
      self.pokePc(val);
    }
  else if (attr == "int_regs")
    {
      if (not py::isinstance<py::list>(value))
        throw py::type_error("Hart::__setattr___: must assign list of ints to int_regs");

      auto val = py::cast<std::vector<T>>(value);
      for (unsigned i = 0; i < val.size(); ++i)
        if (not self.pokeIntReg(i, val.at(i)))
          throw py::attribute_error("Hart::__setattr___: invalid int register access");
    }
  else if (attr == "fp_regs")
    {
      if (not py::isinstance<py::list>(value))
        throw py::type_error("Hart::__setattr___: must assign list of ints to fp_regs");

      auto val = py::cast<std::vector<uint64_t>>(value);
      for (unsigned i = 0; i < val.size(); ++i)
        if (not self.pokeFpReg(i, val.at(i)))
          throw py::attribute_error("Hart::__setattr___: invalid fp register access");
    }
  else if (attr == "vec_regs")
    {
      if (not py::isinstance<py::list>(value))
        throw py::type_error("Hart::__setattr___: must assign list of list of ints to vec_regs");

      auto val = py::cast<std::vector<std::vector<uint8_t>>>(value);
      for (unsigned i = 0; i < val.size(); ++i)
          if (not self.pokeVecReg(i, val.at(i)))
            throw py::attribute_error("Hart::__setattr___: invalid vec register access");
    }
  else if (attr == "cs_regs")
    {
      if (not py::isinstance<py::dict>(value))
        throw py::type_error("Hart::__setattr___: must assign dict of name, value to cs_regs");

      auto val = py::cast<std::unordered_map<std::string, T>>(value);
      for (const auto& [k, v] : val)
        {
          auto csr = self.findCsr(k);
          if (not csr or not self.pokeCsr(csr->getNumber(), v))
            throw py::attribute_error("Hart::__setattr___: invalid csr access");
        }
    }
  else
    throw py::attribute_error("Hart::__setattr__: unknown attribute");
}

template <typename T, typename M>
static void defineHart(M m)
{
  auto m_system = m.def_submodule("system", "System.hpp");
  auto m_hart = m_system.def_submodule("hart", "Hart.hpp");

  // We mark Hart class' "holder" type as shared_ptr to prevent double frees
  auto hartName = concat<T>("Hart");
  py::class_<Hart<T>, std::shared_ptr<Hart<T>>>(m_hart, hartName.data())
    .def("step", [](Hart<T>& self, bool verbose, File file) {
          DecodedInst di;
          self.singleStep(di, verbose? file.file_ : nullptr);

          std::vector<py::object> changes;
          int regIx = self.lastIntReg();
          if (regIx > 0)
            changes.push_back(py::cast("x" + std::to_string(regIx)));

          int fpIx = self.lastFpReg();
          if (fpIx >= 0)
            changes.push_back(py::cast("f" + std::to_string(fpIx)));

          unsigned groupSize = 0;
          int vecIx = self.lastVecReg(di, groupSize);
          if (vecIx >= 0)
            for (unsigned ix = 0; ix < groupSize; ++ix)
              changes.push_back(py::cast("v" + std::to_string(vecIx + ix)));

          std::vector<CsrNumber> csrs;
          self.lastCsr(csrs);
          for (CsrNumber csrn : csrs)
            {
              auto csr = self.findCsr(csrn);
              if (csr)
                changes.push_back(py::cast(csr->getName()));
            }

          uint64_t addr, value;
          unsigned size = self.lastStore(addr, value);
          if (size)
            {
              auto p = std::make_pair("m" + std::to_string(addr), size);
              changes.push_back(py::cast(p));
            }
          else
            {
              const VecLdStInfo & vec_ld_st_info = self.getLastVectorMemory();
              if (not vec_ld_st_info.elems_.empty()) {
                for (size_t i = 0; i < vec_ld_st_info.elems_.size(); ++i)
                  {
                    auto p = std::make_pair("m" + std::to_string(vec_ld_st_info.elems_.at(i).va_), vec_ld_st_info.elemSize_);
                    changes.push_back(py::cast(p));
                  }
              }
            }

          return std::make_tuple(self.hasTargetProgramFinished(), di, changes);
        }, py::arg("verbose") = false, py::arg("file") = File(stdout), py::doc("Step a single instruction. Returns a tuple of (true if target program is finished, list of modified resources)."))
    .def("run", [](Hart<T>& self, bool verbose, File file) {
          self.run(verbose? file.file_ : nullptr);
        }, py::arg("verbose") = false, py::arg("file") = File(stdout), py::doc("Run until hart reaches stopping point."))
    .def("disass_inst", [](Hart<T>& self, uint32_t inst) {
          std::string str;
          self.disassembleInst(inst, str);
          return str;
        }, py::arg("inst"), py::doc("Disassemble 32-bit instruction."))
    .def("__getattr__", py::overload_cast<Hart<T>&, const std::string&>(&attr<T>))
    .def("__setattr__", py::overload_cast<Hart<T>&, const std::string&, const py::object&>(&attr<T>))
    .def("mcm_read", [](Hart<T>& self, uint64_t time, uint64_t tag, uint64_t addr,
                        unsigned size, uint64_t data, unsigned elemIx, unsigned field) {
          if (not self.mcm())
            return false;
          return self.mcm()->readOp(self, time, tag, addr, size, data, elemIx, field);
        }, py::doc("MCM read operation."))
    .def("mcm_mb_write", [](Hart<T>& self, uint64_t time, uint64_t addr,
                            const std::vector<uint8_t>& data,
                            const std::vector<bool>& mask) {
          if (not self.mcm())
            return false;
          return self.mcm()->mergeBufferWrite(self, time, addr, data, mask);
        }, py::doc("MCM merge buffer write operation."))
    .def("mcm_mb_insert", [](Hart<T>& self, uint64_t time, uint64_t tag,
                             uint64_t addr, unsigned size, uint64_t data) {
          if (not self.mcm())
            return false;
          return self.mcm()->mergeBufferInsert(self, time, tag, addr, size, data);
        }, py::doc("MCM merge buffer insert operation."))
    .def("mcm_bypass", [](Hart<T>& self, uint64_t time, uint64_t tag,
                          uint64_t addr, unsigned size, uint64_t data) {
          if (not self.mcm())
            return false;
          return self.mcm()->bypassOp(self, time, tag, addr, size, data);
        }, py::doc("MCM merge buffer bypass operation."))
    .def("mcm_ifetch", &Hart<T>::mcmIFetch, py::doc("MCM instruction fetch operation."))
    .def("mcm_ievict", &Hart<T>::mcmIEvict, py::doc("MCM instruction cache eviction operation."));
};


template <typename T, typename M>
static void defineSystem(M m)
{
  auto m_system = m.def_submodule("system", "System.hpp");

  auto systemName = concat<T>("System");
  auto stringify = [] (const auto& files) {
    std::vector<std::string> stringified;
    std::transform(files.begin(), files.end(), std::back_inserter(stringified), [] (const auto& files) {
      return files.string();
    });
    return stringified;
  };

  py::class_<System<T>>(m_system, systemName.data())
    .def(py::init([](const std::filesystem::path& configFile, unsigned coreCount, unsigned hartsPerCore, uint64_t memorySize, uint64_t pageSize) {
          std::string isa;
          HartConfig config;

          auto system = std::unique_ptr<System<T>>(new System<T>(coreCount, hartsPerCore, hartsPerCore, memorySize, pageSize));

          if (configFile.empty()
              or not config.loadConfigFile(configFile.string())
              or not config.getIsa(isa)
              or not config.configHarts(*system, false, false)
              or not config.configMemory(*system, false))
            throw std::invalid_argument("System::System: failed configuration");

          for (unsigned i = 0; i < system->hartCount(); ++i)
            {
              auto& hart = *(system->ithHart(i));
              // raw mode
              hart.enableNewlib(false);
              hart.enableLinux(false);
              if (not isa.empty())
                if (not hart.configIsa(isa, false))
                  throw std::invalid_argument("System::System: failed isa configuration");
              hart.reset();
            }
          return system;
        }),
        py::arg("config_file"), py::arg("core_count") = 1, py::arg("harts_per_core") = 1, py::arg("memory_size") = (uint64_t(1) << 32), py::arg("page_size") = 4096)
    .def("load_hex_files", [stringify](System<T>& self, const std::vector<std::filesystem::path>& files, bool verbose) {
          std::vector<std::string> stringified = stringify(files);
          return self.loadHexFiles(stringified, verbose);
        }, py::arg("files"), py::arg("verbose") = false, py::doc("Hex files to load. Returns true on success."))
    .def("load_bin_files", [stringify](System<T>& self, const std::vector<std::filesystem::path>& files, uint64_t offset, bool verbose) {
          std::vector<std::string> stringified = stringify(files);
          return self.loadBinaryFiles(stringified, offset, verbose);
        }, py::arg("files"), py::arg("offset"), py::arg("verbose") = false, py::doc("Binary files to load. Returns true on success."))
    .def("load_elf_files", [stringify](System<T>& self, const std::vector<std::filesystem::path>& files, bool verbose) {
          std::vector<std::string> stringified = stringify(files);
          return self.loadElfFiles(stringified, false, verbose);
        }, py::arg("files"), py::arg("verbose") = false, py::doc("ELF files to load. Returns true on success."))
#ifdef LZ4_COMPRESS
    .def("load_lz4_files", [stringify](System<T>& self, const std::vector<std::filesystem::path>& files, uint64_t offset, bool verbose) {
          std::vector<std::string> stringified = stringify(files);
          return self.loadLz4Files(stringified, offset, verbose);
        })
#endif
    .def("harts", [](System<T>& self) {
          std::vector<std::shared_ptr<Hart<T>>> harts;
          for (unsigned i = 0; i < self.hartCount(); ++i)
            harts.push_back(self.ithHart(i));
          return harts;
        }, py::doc("Get all of the harts in the system in a list."))
    .def("memory", [](System<T>& self) {
          return self.memory();
        }, py::doc("Get memory instance from system."));
}


PYBIND11_MODULE(pywhisper, m) {
  m.doc() = "Interactive module for whisper";

  using RV32 = uint32_t;
  using RV64 = uint64_t;

  defineFile(m);
  defineEnums(m);
  defineMemory(m);
  defineDecodedInst(m);
  defineHart<RV32>(m);
  defineSystem<RV32>(m);
  defineHart<RV64>(m);
  defineSystem<RV64>(m);
}
