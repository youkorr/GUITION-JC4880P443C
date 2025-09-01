#pragma once
#include "esphome/core/gpio.h"
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#ifdef USE_ESP_IDF
#include "sdmmc_cmd.h"
#endif

namespace esphome {
namespace sd_mmc_card {

enum MemoryUnits : short { Byte = 0, KiloByte = 1, MegaByte = 2, GigaByte = 3, TeraByte = 4, PetaByte = 5 };

#ifdef USE_SENSOR
struct FileSizeSensor {
  sensor::Sensor *sensor{nullptr};
  std::string path;

  FileSizeSensor() = default;
  FileSizeSensor(sensor::Sensor *, std::string const &path);
};
#endif

struct FileInfo {
  std::string path;
  size_t size;
  bool is_directory;

  FileInfo(std::string const &, size_t, bool);
};

class SdMmc : public Component {
#ifdef USE_SENSOR
  SUB_SENSOR(used_space)
  SUB_SENSOR(total_space)
  SUB_SENSOR(free_space)
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(sd_card_type)
#endif
 public:
  enum ErrorCode {
    ERR_PIN_SETUP,
    ERR_MOUNT,
    ERR_NO_CARD,
  };
  void setup() override;
  void loop() override;
  void dump_config() override;
  void write_file(const char *path, const uint8_t *buffer, size_t len, const char *mode);
  void write_file(const char *path, const uint8_t *buffer, size_t len);
  void append_file(const char *path, const uint8_t *buffer, size_t len);
  void write_file_chunked(const char *path, const uint8_t *buffer, size_t len, size_t chunk_size);
  bool delete_file(const char *path);
  bool delete_file(std::string const &path);
  bool create_directory(const char *path);
  bool remove_directory(const char *path);
  std::vector<uint8_t> read_file(char const *path);
  std::vector<uint8_t> read_file(std::string const &path);
  std::vector<uint8_t> read_file_chunked(char const *path, size_t offset, size_t chunk_size);
  std::vector<uint8_t> read_file_chunked(std::string const &path, size_t offset, size_t chunk_size);
  bool is_directory(const char *path);
  bool is_directory(std::string const &path);
  std::vector<std::string> list_directory(const char *path, uint8_t depth);
  std::vector<std::string> list_directory(std::string path, uint8_t depth);
  std::vector<FileInfo> list_directory_file_info(const char *path, uint8_t depth);
  std::vector<FileInfo> list_directory_file_info(std::string path, uint8_t depth);
  size_t file_size(const char *path);
  size_t file_size(std::string const &path);
  void read_file_stream(const char *path, size_t offset, size_t chunk_size, std::function<void(const uint8_t*, size_t)> callback);
#ifdef USE_SENSOR
  void add_file_size_sensor(sensor::Sensor *, std::string const &path);
#endif

  void set_clk_pin(uint8_t);
  void set_cmd_pin(uint8_t);
  void set_data0_pin(uint8_t);
  void set_data1_pin(uint8_t);
  void set_data2_pin(uint8_t);
  void set_data3_pin(uint8_t);
  void set_mode_1bit(bool);
  void set_power_ctrl_pin(GPIOPin *);

  void set_pwr_ctrl_by_on_chip_ldo(bool enable) { pwr_ctrl_by_on_chip_ldo_ = enable; }
  void set_frequency(uint32_t frequency) { frequency_ = frequency; }

  void set_slot(uint8_t slot) { this->slot_ = slot; }

 protected:
  ErrorCode init_error_;
  uint8_t clk_pin_;
  uint8_t cmd_pin_;
  uint8_t data0_pin_;
  uint8_t data1_pin_;
  uint8_t data2_pin_;
  uint8_t data3_pin_;
  bool mode_1bit_;
  GPIOPin *power_ctrl_pin_{nullptr};

  uint8_t slot_ = 0;  // Par défaut slot 0

  bool pwr_ctrl_by_on_chip_ldo_{false};
  uint32_t frequency_{40000};  // 40kHz par défaut

#ifdef USE_ESP_IDF
  sdmmc_card_t *card_;
#endif
#ifdef USE_SENSOR
  std::vector<FileSizeSensor> file_size_sensors_{};
#endif
  void update_sensors();

#ifdef USE_ESP_IDF
  std::string sd_card_type() const;
#endif
  std::vector<FileInfo> &list_directory_file_info_rec(const char *path, uint8_t depth, std::vector<FileInfo> &list);
  static std::string error_code_to_string(ErrorCode);
};

template<typename... Ts> class SdMmcWriteFileAction : public Action<Ts...> {
 public:
  SdMmcWriteFileAction(SdMmc *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, path)
  TEMPLATABLE_VALUE(std::vector<uint8_t>, data)

  void play(Ts... x) {
    auto path = this->path_.value(x...);
    auto buffer = this->data_.value(x...);
    this->parent_->write_file(path.c_str(), buffer.data(), buffer.size());
  }

 protected:
  SdMmc *parent_;
};

template<typename... Ts> class SdMmcWriteFileChunkedAction : public Action<Ts...> {
 public:
  SdMmcWriteFileChunkedAction(SdMmc *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, path)
  TEMPLATABLE_VALUE(std::vector<uint8_t>, data)
  TEMPLATABLE_VALUE(size_t, chunk_size)

  void play(Ts... x) {
    auto path = this->path_.value(x...);
    auto buffer = this->data_.value(x...);
    auto chunk_size = this->chunk_size_.value(x...);
    this->parent_->write_file_chunked(path.c_str(), buffer.data(), buffer.size(), chunk_size);
  }

 protected:
  SdMmc *parent_;
};

template<typename... Ts> class SdMmcAppendFileAction : public Action<Ts...> {
 public:
  SdMmcAppendFileAction(SdMmc *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, path)
  TEMPLATABLE_VALUE(std::vector<uint8_t>, data)

  void play(Ts... x) {
    auto path = this->path_.value(x...);
    auto buffer = this->data_.value(x...);
    this->parent_->append_file(path.c_str(), buffer.data(), buffer.size());
  }

 protected:
  SdMmc *parent_;
};

template<typename... Ts> class SdMmcCreateDirectoryAction : public Action<Ts...> {
 public:
  SdMmcCreateDirectoryAction(SdMmc *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, path)

  void play(Ts... x) {
    auto path = this->path_.value(x...);
    this->parent_->create_directory(path.c_str());
  }

 protected:
  SdMmc *parent_;
};

template<typename... Ts> class SdMmcRemoveDirectoryAction : public Action<Ts...> {
 public:
  SdMmcRemoveDirectoryAction(SdMmc *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, path)

  void play(Ts... x) {
    auto path = this->path_.value(x...);
    this->parent_->remove_directory(path.c_str());
  }

 protected:
  SdMmc *parent_;
};

template<typename... Ts> class SdMmcDeleteFileAction : public Action<Ts...> {
 public:
  SdMmcDeleteFileAction(SdMmc *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, path)

  void play(Ts... x) {
    auto path = this->path_.value(x...);
    this->parent_->delete_file(path.c_str());
  }

 protected:
  SdMmc *parent_;
};

template<typename... Ts> class SdMmcReadFileChunkedAction : public Action<Ts...> {
 public:
  SdMmcReadFileChunkedAction(SdMmc *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, path)
  TEMPLATABLE_VALUE(size_t, offset)
  TEMPLATABLE_VALUE(size_t, chunk_size)

  void play(Ts... x) {
    auto path = this->path_.value(x...);
    auto offset = this->offset_.value(x...);
    auto chunk_size = this->chunk_size_.value(x...);
    this->parent_->read_file_chunked(path.c_str(), offset, chunk_size);
  }

 protected:
  SdMmc *parent_;
};

long double convertBytes(uint64_t, MemoryUnits);

}  // namespace sd_mmc_card
}  // namespace esphome





