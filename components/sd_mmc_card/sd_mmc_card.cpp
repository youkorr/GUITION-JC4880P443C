#include "sd_mmc_card.h"
#include "esp_task_wdt.h"

#include <algorithm>
#include <vector>
#include <cstdio>

#include "math.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP_IDF
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_types.h"

// Pour ESP32-P4 - Gestion du contrôle d'alimentation
#if CONFIG_IDF_TARGET_ESP32P4
  #ifdef __has_include
    #if __has_include("sd_pwr_ctrl_by_on_chip_ldo.h")
      #include "sd_pwr_ctrl_by_on_chip_ldo.h"
      #define HAS_LDO_PWR_CTRL 1
    #else
      #define HAS_LDO_PWR_CTRL 0
      ESP_LOGW("sd_mmc_card", "sd_pwr_ctrl_by_on_chip_ldo.h not available, using GPIO control fallback");
    #endif
  #else
    #define HAS_LDO_PWR_CTRL 0
  #endif

  #if !HAS_LDO_PWR_CTRL
    typedef struct {
        int ldo_chan_id;
    } sd_pwr_ctrl_ldo_config_t;
    typedef void* sd_pwr_ctrl_handle_t;
    #define ESP_ERR_NOT_SUPPORTED 0x106
  #endif
#else
  #define HAS_LDO_PWR_CTRL 0
#endif

int constexpr SD_OCR_SDHC_CAP = (1 << 30);
#endif

namespace esphome {
namespace sd_mmc_card {

static const char *TAG = "sd_mmc_card";

#ifdef USE_ESP_IDF
static constexpr size_t FILE_PATH_MAX = ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN;
static const std::string MOUNT_POINT("/sdcard");

std::string build_path(const char *path) { return MOUNT_POINT + path; }
#endif

#ifdef USE_SENSOR
FileSizeSensor::FileSizeSensor(sensor::Sensor *sensor, std::string const &path) : sensor(sensor), path(path) {}
#endif

void SdMmc::loop() {
  static uint32_t last_update = 0;
  uint32_t now = millis();
  if (now - last_update > 30000) {
    update_sensors();
    last_update = now;
  }
}

void SdMmc::dump_config() {
  ESP_LOGCONFIG(TAG, "SD MMC Component");
  ESP_LOGCONFIG(TAG, "  Mode 1 bit: %s", TRUEFALSE(this->mode_1bit_));
  ESP_LOGCONFIG(TAG, "  Slot: %d", this->slot_); 
  ESP_LOGCONFIG(TAG, "  CLK Pin: %d", this->clk_pin_);
  ESP_LOGCONFIG(TAG, "  CMD Pin: %d", this->cmd_pin_);
  ESP_LOGCONFIG(TAG, "  DATA0 Pin: %d", this->data0_pin_);
  if (!this->mode_1bit_) {
    ESP_LOGCONFIG(TAG, "  DATA1 Pin: %d", this->data1_pin_);
    ESP_LOGCONFIG(TAG, "  DATA2 Pin: %d", this->data2_pin_);
    ESP_LOGCONFIG(TAG, "  DATA3 Pin: %d", this->data3_pin_);
  }
  if (this->power_ctrl_pin_ != nullptr) {
    LOG_PIN("  Power Ctrl Pin: ", this->power_ctrl_pin_);
  }

#ifdef USE_ESP_IDF
  if (!this->is_failed() && this->card_ != nullptr) {
    const char *freq_unit = card_->real_freq_khz < 1000 ? "kHz" : "MHz";
    const float freq = card_->real_freq_khz < 1000 ? card_->real_freq_khz : card_->real_freq_khz / 1000.0;
    const char *max_freq_unit = card_->max_freq_khz < 1000 ? "kHz" : "MHz";
    const float max_freq = card_->max_freq_khz < 1000 ? card_->max_freq_khz : card_->max_freq_khz / 1000.0;
    ESP_LOGCONFIG(TAG, "  Card Speed:  %.2f %s (limit: %.2f %s)%s", freq, freq_unit, max_freq, max_freq_unit, card_->is_ddr ? ", DDR" : "");
  }
#endif

#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Used space", this->used_space_sensor_);
  LOG_SENSOR("  ", "Total space", this->total_space_sensor_);
  LOG_SENSOR("  ", "Free space", this->free_space_sensor_);
  for (auto &sensor : this->file_size_sensors_) {
    if (sensor.sensor != nullptr)
      LOG_SENSOR("  ", "File size", sensor.sensor);
  }
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "SD Card Type", this->sd_card_type_text_sensor_);
#endif
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Setup failed : %s", SdMmc::error_code_to_string(this->init_error_).c_str());
    return;
  }
}

#ifdef USE_ESP_IDF
void SdMmc::setup() {
  ESP_LOGI(TAG, "Setting up SD MMC for ESP32-P4...");
  
  bool power_controlled = false;

#if CONFIG_IDF_TARGET_ESP32P4 && HAS_LDO_PWR_CTRL
  sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;
  ESP_LOGI(TAG, "Attempting to use on-chip LDO power control...");
  sd_pwr_ctrl_ldo_config_t pwr_ctrl_config = {
    .ldo_chan_id = 4
  };

  esp_err_t pwr_ret = sd_pwr_ctrl_new_on_chip_ldo(&pwr_ctrl_config, &pwr_ctrl_handle);
  if (pwr_ret == ESP_OK && pwr_ctrl_handle != NULL) {
    ESP_LOGI(TAG, "LDO power control created successfully");

  #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_err_t pwr_on_ret = sd_pwr_ctrl_on(pwr_ctrl_handle);
    if (pwr_on_ret == ESP_OK) {
      power_controlled = true;
      ESP_LOGI(TAG, "LDO power activated (legacy API)");
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      ESP_LOGE(TAG, "Failed to activate LDO power: %s", esp_err_to_name(pwr_on_ret));
      pwr_ctrl_handle = NULL;
    }
  #else
    power_controlled = true;
    ESP_LOGI(TAG, "LDO power assumed active (ESP-IDF v5+ API)");
    vTaskDelay(pdMS_TO_TICKS(200));
  #endif

  } else {
    ESP_LOGW(TAG, "LDO power control creation failed: %s", esp_err_to_name(pwr_ret));
  }
#endif

  if (!power_controlled && this->power_ctrl_pin_ != nullptr) {
    ESP_LOGI(TAG, "Using GPIO power control as fallback");
    this->power_ctrl_pin_->setup();
    this->power_ctrl_pin_->digital_write(true);
    power_controlled = true;
    ESP_LOGI(TAG, "GPIO power control activated");
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  if (!power_controlled) {
    ESP_LOGW(TAG, "No power control configured - ensure SD card has stable power supply");
  }

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 8,
    .allocation_unit_size = 64 * 1024
  };

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.slot = SDMMC_HOST_SLOT_0 + this->slot_;
  host.max_freq_khz = SDMMC_FREQ_PROBING;
  host.flags = this->mode_1bit_ ? SDMMC_HOST_FLAG_1BIT : SDMMC_HOST_FLAG_4BIT;

  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = this->mode_1bit_ ? 1 : 4;

#ifdef SOC_SDMMC_USE_GPIO_MATRIX
  slot_config.clk = static_cast<gpio_num_t>(this->clk_pin_);
  slot_config.cmd = static_cast<gpio_num_t>(this->cmd_pin_);
  slot_config.d0 = static_cast<gpio_num_t>(this->data0_pin_);
  if (!this->mode_1bit_) {
    slot_config.d1 = static_cast<gpio_num_t>(this->data1_pin_);
    slot_config.d2 = static_cast<gpio_num_t>(this->data2_pin_);
    slot_config.d3 = static_cast<gpio_num_t>(this->data3_pin_);
  }
  ESP_LOGI(TAG, "Using GPIO matrix for pin configuration");
#else
  ESP_LOGW(TAG, "GPIO matrix not available - using default pins");
#endif

  slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

#if CONFIG_IDF_TARGET_ESP32P4 && HAS_LDO_PWR_CTRL && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
  if (power_controlled && pwr_ctrl_handle != NULL) {
    slot_config.pwr_ctrl_handle = pwr_ctrl_handle;
    ESP_LOGI(TAG, "LDO power control attached to slot configuration");
  }
#endif

  ESP_LOGI(TAG, "Initializing SDMMC slot %d...", this->slot_);
  esp_err_t slot_ret = sdmmc_host_init_slot(host.slot, &slot_config);
  if (slot_ret != ESP_OK) {
    ESP_LOGE(TAG, "SDMMC slot initialization failed: %s (0x%x)", esp_err_to_name(slot_ret), slot_ret);
    this->init_error_ = ErrorCode::ERR_PIN_SETUP;
    mark_failed();
    return;
  }
  ESP_LOGI(TAG, "SDMMC slot initialized successfully");
  esp_err_t slot_ret = sdmmc_host_init_slot(host.slot, &slot_config);
  if (slot_ret != ESP_OK) {
    ESP_LOGE(TAG, "SDMMC slot initialization failed: %s (0x%x)", esp_err_to_name(slot_ret), slot_ret);
    this->init_error_ = ErrorCode::ERR_PIN_SETUP;
    mark_failed();
    return;
  }
  ESP_LOGI(TAG, "SDMMC slot initialized successfully");

  // Tentatives de montage avec stratégie progressive
  esp_err_t mount_ret = ESP_FAIL;
  const int max_attempts = 5;
  const int delays_ms[] = {100, 250, 500, 1000, 2000};
  
  for (int attempt = 0; attempt < max_attempts; attempt++) {
    ESP_LOGI(TAG, "Mounting SD Card on slot %d (attempt %d/%d)...", this->slot_, attempt + 1, max_attempts);
    
    mount_ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT.c_str(), &host, &slot_config, &mount_config, &this->card_);
    
    if (mount_ret == ESP_OK) {
      ESP_LOGI(TAG, "✓ SD Card successfully mounted on slot %d!", this->slot_);
      break;
    }
    
    // Log détaillé selon le type d'erreur
    switch (mount_ret) {
      case ESP_ERR_TIMEOUT:
        ESP_LOGW(TAG, "Attempt %d: Timeout - card may not be responding", attempt + 1);
        break;
      case ESP_ERR_INVALID_STATE:
        ESP_LOGW(TAG, "Attempt %d: Invalid state - driver issue", attempt + 1);
        break;
      case ESP_ERR_NOT_FOUND:
        ESP_LOGW(TAG, "Attempt %d: Card not found - check insertion", attempt + 1);
        break;
      default:
        ESP_LOGW(TAG, "Attempt %d: Error %s (0x%x)", attempt + 1, esp_err_to_name(mount_ret), mount_ret);
    }
    
    // Attendre avant la prochaine tentative
    if (attempt < max_attempts - 1) {
      vTaskDelay(pdMS_TO_TICKS(delays_ms[attempt]));
    }
  }

  // Gestion finale des échecs
  if (mount_ret != ESP_OK) {
    switch (mount_ret) {
      case ESP_ERR_INVALID_STATE:
        ESP_LOGE(TAG, "✗ SD card driver not properly initialized");
        this->init_error_ = ErrorCode::ERR_PIN_SETUP;
        break;
      case ESP_ERR_NO_MEM:
        ESP_LOGE(TAG, "✗ Insufficient memory for SD card operations");
        this->init_error_ = ErrorCode::ERR_MOUNT;
        break;
      case ESP_ERR_INVALID_ARG:
        ESP_LOGE(TAG, "✗ Invalid configuration parameters");
        this->init_error_ = ErrorCode::ERR_PIN_SETUP;
        break;
      case ESP_ERR_NOT_FOUND:
      case ESP_ERR_TIMEOUT:
        ESP_LOGE(TAG, "✗ No SD card detected or card not responding");
        ESP_LOGE(TAG, "   Check: card insertion, power supply, pin connections");
        this->init_error_ = ErrorCode::ERR_NO_CARD;
        break;
      case ESP_FAIL:
      default:
        ESP_LOGE(TAG, "✗ Failed to mount filesystem: %s", esp_err_to_name(mount_ret));
        this->init_error_ = ErrorCode::ERR_MOUNT;
        break;
    }
    mark_failed();
    return;
  }

  // Optimisation de la fréquence après montage réussi
  if (this->card_ && this->card_->max_freq_khz > SDMMC_FREQ_PROBING) {
    uint32_t target_freq = std::min(static_cast<uint32_t>(SDMMC_FREQ_HIGHSPEED), this->card_->max_freq_khz);
    esp_err_t freq_ret = sdmmc_host_set_card_clk(host.slot, target_freq);
    if (freq_ret == ESP_OK) {
      ESP_LOGI(TAG, "Card frequency optimized to %d kHz", target_freq);
    } else {
      ESP_LOGW(TAG, "Failed to optimize frequency: %s", esp_err_to_name(freq_ret));
    }
  }

  // Affichage détaillé des informations
  ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║           SD Card Information          ║");
  ESP_LOGI(TAG, "╠════════════════════════════════════════╣");
  ESP_LOGI(TAG, "║ Slot:     %-28d ║", this->slot_);
  ESP_LOGI(TAG, "║ Name:     %-28s ║", this->card_->cid.name);
  ESP_LOGI(TAG, "║ Type:     %-28s ║", sd_card_type().c_str());
  ESP_LOGI(TAG, "║ Speed:    %d kHz (max: %d kHz)%*s ║", 
           this->card_->real_freq_khz, this->card_->max_freq_khz,
           (int)(28 - snprintf(nullptr, 0, "%d kHz (max: %d kHz)", this->card_->real_freq_khz, this->card_->max_freq_khz)), "");
  
  uint64_t card_size = ((uint64_t)this->card_->csd.capacity * this->card_->csd.sector_size);
  ESP_LOGI(TAG, "║ Size:     %llu MB (%llu bytes)%*s ║", 
           card_size / (1024 * 1024), card_size,
           (int)(28 - snprintf(nullptr, 0, "%llu MB (%llu bytes)", card_size / (1024 * 1024), card_size)), "");
  ESP_LOGI(TAG, "║ Sectors:  %d bytes/sector%*s ║", 
           this->card_->csd.sector_size,
           (int)(28 - snprintf(nullptr, 0, "%d bytes/sector", this->card_->csd.sector_size)), "");
  ESP_LOGI(TAG, "╚════════════════════════════════════════╝");

  // Mise à jour initiale des capteurs
  update_sensors();
}

void SdMmc::write_file(const char *path, const uint8_t *buffer, size_t len, const char *mode) {
  ESP_LOGV(TAG, "Writing to file: %s (mode: %s, size: %zu bytes)", path, mode, len);
  std::string absolut_path = build_path(path);
  FILE *file = fopen(absolut_path.c_str(), mode);
  if (file == nullptr) {
    ESP_LOGE(TAG, "Failed to open file for writing: %s", strerror(errno));
    return;
  }
  
  size_t written = fwrite(buffer, 1, len, file);
  if (written != len) {
    ESP_LOGE(TAG, "Write incomplete: expected %zu bytes, wrote %zu", len, written);
  }
  
  fclose(file);
  this->update_sensors();
}

void SdMmc::write_file(const char *path, const uint8_t *buffer, size_t len) {
  write_file(path, buffer, len, "w");
}

void SdMmc::append_file(const char *path, const uint8_t *buffer, size_t len) {
  write_file(path, buffer, len, "a");
}

void SdMmc::write_file_chunked(const char *path, const uint8_t *buffer, size_t len, size_t chunk_size) {
  ESP_LOGV(TAG, "Writing chunked file: %s (total: %zu bytes, chunk: %zu bytes)", path, len, chunk_size);
  std::string absolut_path = build_path(path);
  FILE *file = fopen(absolut_path.c_str(), "w");
  if (file == nullptr) {
    ESP_LOGE(TAG, "Failed to open file for chunked writing: %s", strerror(errno));
    return;
  }

  size_t written = 0;
  while (written < len) {
    size_t to_write = std::min(chunk_size, len - written);
    size_t chunk_written = fwrite(buffer + written, 1, to_write, file);
    if (chunk_written != to_write) {
      ESP_LOGE(TAG, "Chunk write failed at offset %zu", written);
      break;
    }
    written += chunk_written;
    
    // Reset WDT pour les gros fichiers
    if (written % (64 * 1024) == 0) {
      esp_task_wdt_reset();
    }
  }
  
  fclose(file);
  ESP_LOGD(TAG, "Chunked write completed: %zu/%zu bytes", written, len);
  this->update_sensors();
}

bool SdMmc::delete_file(const char *path) {
  ESP_LOGV(TAG, "Deleting file: %s", path);
  if (this->is_directory(path)) {
    ESP_LOGE(TAG, "Cannot delete file: path is a directory");
    return false;
  }
  std::string absolut_path = build_path(path);
  if (remove(absolut_path.c_str()) != 0) {
    ESP_LOGE(TAG, "Failed to delete file: %s", strerror(errno));
    return false;
  }
  this->update_sensors();
  return true;
}

bool SdMmc::create_directory(const char *path) {
  ESP_LOGV(TAG, "Creating directory: %s", path);
  std::string absolut_path = build_path(path);
  if (mkdir(absolut_path.c_str(), 0777) < 0) {
    ESP_LOGE(TAG, "Failed to create directory: %s", strerror(errno));
    return false;
  }
  this->update_sensors();
  return true;
}

bool SdMmc::remove_directory(const char *path) {
  ESP_LOGV(TAG, "Removing directory: %s", path);
  if (!this->is_directory(path)) {
    ESP_LOGE(TAG, "Cannot remove directory: path is not a directory");
    return false;
  }
  std::string absolut_path = build_path(path);
  if (remove(absolut_path.c_str()) != 0) {
    ESP_LOGE(TAG, "Failed to remove directory: %s", strerror(errno));
    return false;
  }
  this->update_sensors();
  return true;
}

std::vector<uint8_t> SdMmc::read_file(const char *path) {
  ESP_LOGV(TAG, "Reading file: %s", path);

  size_t file_size = this->file_size(path);
  if (file_size == static_cast<size_t>(-1)) {
    ESP_LOGE(TAG, "Cannot determine file size");
    return {};
  }
  
  // Limite de sécurité pour éviter les problèmes de mémoire
  constexpr size_t MAX_SAFE_SIZE = 2 * 1024 * 1024; // 2MB
  
  if (file_size > MAX_SAFE_SIZE) {
    ESP_LOGE(TAG, "File too large for direct reading: %zu bytes (max: %zu). Use read_file_stream instead.", 
             file_size, MAX_SAFE_SIZE);
    return {};
  }

  std::string absolut_path = build_path(path);
  FILE *file = fopen(absolut_path.c_str(), "rb");
  if (file == nullptr) {
    ESP_LOGE(TAG, "Failed to open file for reading: %s", strerror(errno));
    return {};
  }

  std::vector<uint8_t> result(file_size);
  size_t read_len = fread(result.data(), 1, file_size, file);
  fclose(file);

  if (read_len != file_size) {
    ESP_LOGE(TAG, "Read incomplete: expected %zu bytes, got %zu", file_size, read_len);
    return {};
  }

  ESP_LOGD(TAG, "Successfully read %zu bytes from %s", read_len, path);
  return result;
}

std::vector<uint8_t> SdMmc::read_file_chunked(const char *path, size_t offset, size_t chunk_size) {
  ESP_LOGV(TAG, "Reading file chunk: %s (offset: %zu, size: %zu)", path, offset, chunk_size);
  
  std::string absolut_path = build_path(path);
  FILE *file = fopen(absolut_path.c_str(), "rb");
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for chunked reading: %s", strerror(errno));
    return {};
  }

  if (fseek(file, offset, SEEK_SET) != 0) {
    ESP_LOGE(TAG, "Failed to seek to offset %zu: %s", offset, strerror(errno));
    fclose(file);
    return {};
  }

  std::vector<uint8_t> result(chunk_size);
  size_t read_len = fread(result.data(), 1, chunk_size, file);
  fclose(file);

  result.resize(read_len); // Ajuster à la taille réellement lue
  ESP_LOGD(TAG, "Read chunk: %zu bytes at offset %zu", read_len, offset);
  return result;
}

bool SdMmc::is_directory(const char *path) {
  std::string absolut_path = build_path(path);
  DIR *dir = opendir(absolut_path.c_str());
  if (dir) {
    closedir(dir);
    return true;
  }
  return false;
}

std::vector<std::string> SdMmc::list_directory(const char *path, uint8_t depth) {
  std::vector<std::string> list;
  std::vector<FileInfo> infos = list_directory_file_info(path, depth);
  list.reserve(infos.size());
  std::transform(infos.cbegin(), infos.cend(), std::back_inserter(list), 
                 [](const FileInfo &info) { return info.path; });
  return list;
}

std::vector<FileInfo> SdMmc::list_directory_file_info(const char *path, uint8_t depth) {
  std::vector<FileInfo> list;
  list_directory_file_info_rec(path, depth, list);
  return list;
}

std::vector<FileInfo> &SdMmc::list_directory_file_info_rec(const char *path, uint8_t depth,
                                                           std::vector<FileInfo> &list) {
  ESP_LOGV(TAG, "Listing directory file info: %s\n", path);
  std::string absolut_path = build_path(path);
  DIR *dir = opendir(absolut_path.c_str());
  if (!dir) {
    ESP_LOGE(TAG, "Failed to open directory: %s", strerror(errno));
    return list;
  }
  char entry_absolut_path[FILE_PATH_MAX];
  char entry_path[FILE_PATH_MAX];
  const size_t dirpath_len = MOUNT_POINT.size();
  size_t entry_path_len = strlen(path);
  strlcpy(entry_path, path, sizeof(entry_path));
  strlcpy(entry_path + entry_path_len, "/", sizeof(entry_path) - entry_path_len);
  entry_path_len = strlen(entry_path);

  strlcpy(entry_absolut_path, MOUNT_POINT.c_str(), sizeof(entry_absolut_path));
  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    size_t file_size = 0;
    strlcpy(entry_path + entry_path_len, entry->d_name, sizeof(entry_path) - entry_path_len);
    strlcpy(entry_absolut_path + dirpath_len, entry_path, sizeof(entry_absolut_path) - dirpath_len);
    if (entry->d_type != DT_DIR) {
      struct stat info;
      if (stat(entry_absolut_path, &info) < 0) {
        ESP_LOGE(TAG, "Failed to stat file: %s '%s' %s", strerror(errno), entry->d_name, entry_absolut_path);
      } else {
        file_size = info.st_size;
      }
    }
    list.emplace_back(entry_path, file_size, entry->d_type == DT_DIR);
    if (entry->d_type == DT_DIR && depth)
      list_directory_file_info_rec(entry_absolut_path, depth - 1, list);
  }
  closedir(dir);
  return list;
}

size_t SdMmc::file_size(const char *path) {
  std::string absolut_path = build_path(path);
  struct stat info;
  size_t file_size = 0;
  if (stat(absolut_path.c_str(), &info) < 0) {
    ESP_LOGE(TAG, "Failed to stat file: %s", strerror(errno));
    return -1;
  }
  return info.st_size;
}

std::string SdMmc::sd_card_type() const {
  if (this->card_->is_sdio) {
    return "SDIO";
  } else if (this->card_->is_mmc) {
    return "MMC";
  } else {
    return (this->card_->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC";
  }
  return "UNKNOWN";
}

void SdMmc::update_sensors() {
#ifdef USE_SENSOR
  if (this->card_ == nullptr)
    return;

  FATFS *fs;
  DWORD fre_clust, fre_sect, tot_sect;
  uint64_t total_bytes = -1, free_bytes = -1, used_bytes = -1;
  auto res = f_getfree(MOUNT_POINT.c_str(), &fre_clust, &fs);
  if (!res) {
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;

    total_bytes = static_cast<uint64_t>(tot_sect) * FF_SS_SDCARD;
    free_bytes = static_cast<uint64_t>(fre_sect) * FF_SS_SDCARD;
    used_bytes = total_bytes - free_bytes;
  }

  if (this->used_space_sensor_ != nullptr)
    this->used_space_sensor_->publish_state(used_bytes);
  if (this->total_space_sensor_ != nullptr)
    this->total_space_sensor_->publish_state(total_bytes);
  if (this->free_space_sensor_ != nullptr)
    this->free_space_sensor_->publish_state(free_bytes);

  for (auto &sensor : this->file_size_sensors_) {
    if (sensor.sensor != nullptr)
      sensor.sensor->publish_state(this->file_size(sensor.path));
  }
#endif
}

// Lecture en streaming par chunks avec reset du WDT
void SdMmc::read_file_stream(const char *path, size_t offset, size_t chunk_size,
                             std::function<void(const uint8_t*, size_t)> callback) {
  std::string absolut_path = build_path(path);
  FILE *file = fopen(absolut_path.c_str(), "rb");
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file: %s", absolut_path.c_str());
    return;
  }

  std::unique_ptr<FILE, decltype(&fclose)> file_guard(file, fclose);

  if (fseek(file, offset, SEEK_SET) != 0) {
    ESP_LOGE(TAG, "Failed to seek to position %zu in file: %s (errno: %d)", offset, absolut_path.c_str(), errno);
    return;
  }

  std::vector<uint8_t> buffer(chunk_size);
  size_t read = 0;
  size_t bytes_since_reset = 0;

  while ((read = fread(buffer.data(), 1, chunk_size, file)) > 0) {
    callback(buffer.data(), read);
    bytes_since_reset += read;

    if (bytes_since_reset >= 64 * 1024) {
      esp_task_wdt_reset();
      bytes_since_reset = 0;
    }
  }

  if (ferror(file)) {
    ESP_LOGE(TAG, "Error reading file: %s", absolut_path.c_str());
  }
}

#endif

// Implementations pour les surcharges de méthodes avec std::string
bool SdMmc::delete_file(std::string const &path) { 
  return this->delete_file(path.c_str()); 
}

bool SdMmc::is_directory(std::string const &path) { 
  return this->is_directory(path.c_str()); 
}

std::vector<uint8_t> SdMmc::read_file(std::string const &path) { 
  return this->read_file(path.c_str()); 
}

std::vector<uint8_t> SdMmc::read_file_chunked(std::string const &path, size_t offset, size_t chunk_size) {
  return this->read_file_chunked(path.c_str(), offset, chunk_size);
}

std::vector<std::string> SdMmc::list_directory(std::string path, uint8_t depth) {
  return this->list_directory(path.c_str(), depth);
}

std::vector<FileInfo> SdMmc::list_directory_file_info(std::string path, uint8_t depth) {
  return this->list_directory_file_info(path.c_str(), depth);
}

size_t SdMmc::file_size(std::string const &path) { return this->file_size(path.c_str()); }

#ifdef USE_SENSOR
void SdMmc::add_file_size_sensor(sensor::Sensor *sensor, std::string const &path) {
  this->file_size_sensors_.emplace_back(sensor, path);
}
#endif

void SdMmc::set_clk_pin(uint8_t pin) { this->clk_pin_ = pin; }

void SdMmc::set_cmd_pin(uint8_t pin) { this->cmd_pin_ = pin; }

void SdMmc::set_data0_pin(uint8_t pin) { this->data0_pin_ = pin; }

void SdMmc::set_data1_pin(uint8_t pin) { this->data1_pin_ = pin; }

void SdMmc::set_data2_pin(uint8_t pin) { this->data2_pin_ = pin; }

void SdMmc::set_data3_pin(uint8_t pin) { this->data3_pin_ = pin; }

void SdMmc::set_mode_1bit(bool b) { this->mode_1bit_ = b; }

void SdMmc::set_power_ctrl_pin(GPIOPin *pin) { this->power_ctrl_pin_ = pin; }

std::string SdMmc::error_code_to_string(SdMmc::ErrorCode code) {
  switch (code) {
    case ErrorCode::ERR_PIN_SETUP:
      return "Failed to set pins";
    case ErrorCode::ERR_MOUNT:
      return "Failed to mount card";
    case ErrorCode::ERR_NO_CARD:
      return "No card found";
    default:
      return "Unknown error";
  }
}

long double convertBytes(uint64_t value, MemoryUnits unit) {
  return value * 1.0 / pow(1024, static_cast<uint64_t>(unit));
}

FileInfo::FileInfo(std::string const &path, size_t size, bool is_directory)
    : path(path), size(size), is_directory(is_directory) {}

}  // namespace sd_mmc_card
}  // namespace esphome

















