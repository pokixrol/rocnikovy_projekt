#include "../DataTypes/TaskIndex.h"

#include "../../ESPEasy_common.h"

taskIndex_t    INVALID_TASK_INDEX    = TASKS_MAX;
userVarIndex_t INVALID_USERVAR_INDEX = USERVAR_MAX_INDEX;
taskVarIndex_t INVALID_TASKVAR_INDEX = VARS_PER_TASK;
#include "../DataTypes/PluginID.h"

pluginID_t INVALID_PLUGIN_ID = 0;

#include "../DataTypes/NPluginID.h"

npluginID_t      INVALID_N_PLUGIN_ID     = 0;

#include "../DataTypes/ControllerIndex.h"

#include "../CustomBuild/ESPEasyLimits.h"

controllerIndex_t INVALID_CONTROLLER_INDEX = CONTROLLER_MAX;
#include "../DataTypes/DeviceIndex.h"

#include "../CustomBuild/ESPEasyLimits.h"

deviceIndex_t INVALID_DEVICE_INDEX = PLUGIN_MAX;

#include "../DataTypes/ESPEasyFileType.h"

#include "../../ESPEasy_common.h"

#include "../Globals/ResetFactoryDefaultPref.h"

bool matchFileType(const String& filename, FileType::Enum filetype)
{
  if (filename.startsWith(F("/"))) {
    return matchFileType(filename.substring(1), filetype);
  }
  return filename.equalsIgnoreCase(getFileName(filetype));
}

bool isProtectedFileType(const String& filename)
{
  return matchFileType(filename, FileType::CONFIG_DAT) ||
         matchFileType(filename, FileType::SECURITY_DAT) ||
         matchFileType(filename, FileType::NOTIFICATION_DAT) ||
         matchFileType(filename, FileType::PROVISIONING_DAT);
}

const __FlashStringHelper * getFileName(FileType::Enum filetype) {

  switch (filetype)
  {
    case FileType::CONFIG_DAT:       return F("config.dat");
    case FileType::NOTIFICATION_DAT: return F("notification.dat");
    case FileType::SECURITY_DAT:     return F("security.dat");
    case FileType::PROVISIONING_DAT: return F("provisioning.dat");
    case FileType::RULES_TXT:
      // Use getRulesFileName
      break;
    case FileType::FIRMWARE:
      // File name may differ each time.
      break;

    case FileType::MAX_FILETYPE:
      break;
  }
  return F("");
}

String getFileName(FileType::Enum filetype, unsigned int filenr) {
  if (filetype == FileType::RULES_TXT) {
    return getRulesFileName(filenr);
  }
  return getFileName(filetype);
}

// filenr = 0...3 for files rules1.txt ... rules4.txt
String getRulesFileName(unsigned int filenr) {
  String result;

  if (filenr < RULESETS_MAX) {
    result += F("rules");
    result += filenr + 1;
    result += F(".txt");
  }
  return result;
}

bool getDownloadFiletypeChecked(FileType::Enum filetype, unsigned int filenr) {
  bool isChecked = false;

  switch (filetype) {
    case FileType::CONFIG_DAT: isChecked       = ResetFactoryDefaultPreference.fetchConfigDat(); break;
    case FileType::SECURITY_DAT: isChecked     = ResetFactoryDefaultPreference.fetchSecurityDat(); break;
    case FileType::NOTIFICATION_DAT: isChecked = ResetFactoryDefaultPreference.fetchNotificationDat(); break;
    case FileType::RULES_TXT: isChecked        = ResetFactoryDefaultPreference.fetchRulesTXT(filenr); break;
    case FileType::PROVISIONING_DAT: isChecked = ResetFactoryDefaultPreference.fetchProvisioningDat(); break;
      break;

    case FileType::FIRMWARE: // FIXME TD-er: Must decide what to do with firmware description/protection on provisioning settings
    case FileType::MAX_FILETYPE:
      break;
  }
  return isChecked;
}
#include "../DataTypes/WiFiConnectionProtocol.h"

const __FlashStringHelper * toString(WiFiConnectionProtocol proto) {
  switch (proto) {
    case WiFiConnectionProtocol::WiFi_Protocol_11b:
      return F("802.11b");
    case WiFiConnectionProtocol::WiFi_Protocol_11g:
      return F("802.11g");
    case WiFiConnectionProtocol::WiFi_Protocol_11n:
      return F("802.11n");
    case WiFiConnectionProtocol::Unknown:
      break;;
  }
  return F("-");
}
#include "../DataTypes/SPI_options.h"

#ifdef ESP32
const __FlashStringHelper* getSPI_optionToString(SPI_Options_e option) {
  switch (option) {
    case SPI_Options_e::None:
      return F("Disabled");
    case SPI_Options_e::Vspi:
      return F("VSPI: CLK=GPIO-18, MISO=GPIO-19, MOSI=GPIO-23");
    case SPI_Options_e::Hspi:
      return F("HSPI: CLK=GPIO-14, MISO=GPIO-12, MOSI=GPIO-13");
    case SPI_Options_e::UserDefined:
      return F("User-defined: CLK, MISO, MOSI GPIO-pins");
  }
  return F("Unknown");
}

const __FlashStringHelper* getSPI_optionToShortString(SPI_Options_e option) {
  switch (option) {
    case SPI_Options_e::None:
      return F("Disabled");
    case SPI_Options_e::Vspi:
      return F("VSPI");
    case SPI_Options_e::Hspi:
      return F("HSPI");
    case SPI_Options_e::UserDefined:
      return F("User-defined SPI");
  }
  return F("Unknown");
}

#endif // ifdef ESP32

#include "../DataTypes/NotifierIndex.h"

#include "../CustomBuild/ESPEasyLimits.h"

notifierIndex_t INVALID_NOTIFIER_INDEX = NOTIFICATION_MAX;

#include "../DataTypes/ProtocolIndex.h"

#include "../CustomBuild/ESPEasyLimits.h"

protocolIndex_t   INVALID_PROTOCOL_INDEX   = CPLUGIN_MAX;
#include "../DataTypes/CPluginID.h"

cpluginID_t INVALID_C_PLUGIN_ID = 0;

#include "../DataTypes/NetworkMedium.h"

bool isValid(NetworkMedium_t medium) {
  switch (medium) {
    case NetworkMedium_t::WIFI:
    case NetworkMedium_t::Ethernet:
      return true;

      // Do not use default: as this allows the compiler to detect any missing cases.
  }
  return false;
}

const __FlashStringHelper * toString(NetworkMedium_t medium) {
  switch (medium) {
    case NetworkMedium_t::WIFI:     return F("WiFi");
    case NetworkMedium_t::Ethernet: return F("Ethernet");

      // Do not use default: as this allows the compiler to detect any missing cases.
  }
  return F("Unknown");
}

#include "../DataTypes/TimeSource.h"

const __FlashStringHelper* toString(ExtTimeSource_e timeSource)
{
  switch (timeSource) {
    case ExtTimeSource_e::None: break;
    case ExtTimeSource_e::DS1307:  return F("DS1307");
    case ExtTimeSource_e::DS3231:  return F("DS3231");
    case ExtTimeSource_e::PCF8523: return F("PCF8523");
    case ExtTimeSource_e::PCF8563: return F("PCF8563");
  }
  return F("-");
}

#include "../DataTypes/EthernetParameters.h"

bool isValid(EthClockMode_t clockMode) {
  switch (clockMode) {
    case  EthClockMode_t::Ext_crystal_osc:
    case  EthClockMode_t::Int_50MHz_GPIO_0:
    case  EthClockMode_t::Int_50MHz_GPIO_16:
    case  EthClockMode_t::Int_50MHz_GPIO_17_inv:
      return true;

      // Do not use default: as this allows the compiler to detect any missing cases.
  }
  return false;
}

const __FlashStringHelper * toString(EthClockMode_t clockMode) {
  switch (clockMode) {
    case  EthClockMode_t::Ext_crystal_osc:       return F("External crystal oscillator");
    case  EthClockMode_t::Int_50MHz_GPIO_0:      return F("50MHz APLL Output on GPIO0");
    case  EthClockMode_t::Int_50MHz_GPIO_16:     return F("50MHz APLL Output on GPIO16");
    case  EthClockMode_t::Int_50MHz_GPIO_17_inv: return F("50MHz APLL Inverted Output on GPIO17");

      // Do not use default: as this allows the compiler to detect any missing cases.
  }
  return F("Unknown");
}

bool isValid(EthPhyType_t phyType) {
  switch (phyType) {
    case EthPhyType_t::LAN8710:
    case EthPhyType_t::TLK110:
      return true;
    case EthPhyType_t::RTL8201:
    case EthPhyType_t::DP83848:
    case EthPhyType_t::DM9051:
    #if ESP_IDF_VERSION_MAJOR > 3
      return true; // FIXME TD-er: Must check if supported per IDF version
    #else 
      return false;
    #endif

      // Do not use default: as this allows the compiler to detect any missing cases.
  }
  return false;
}

const __FlashStringHelper * toString(EthPhyType_t phyType) {
  switch (phyType) {
    case EthPhyType_t::LAN8710: return F("LAN8710/LAN8720");
    case EthPhyType_t::TLK110:  return F("TLK110");
    case EthPhyType_t::RTL8201: return F("RTL8201");
    case EthPhyType_t::DP83848: return F("DP83848");
    case EthPhyType_t::DM9051:  return F("DM9051");

      // Do not use default: as this allows the compiler to detect any missing cases.
  }
  return F("Unknown");
}

#include "../DataTypes/SettingsType.h"

#include "../CustomBuild/StorageLayout.h"
#include "../DataStructs/ControllerSettingsStruct.h"
#include "../DataStructs/ExtraTaskSettingsStruct.h"
#include "../DataStructs/NotificationSettingsStruct.h"
#include "../DataStructs/SecurityStruct.h"
#include "../DataTypes/ESPEasyFileType.h"
#include "../Globals/Settings.h"

const __FlashStringHelper * SettingsType::getSettingsTypeString(Enum settingsType) {
  switch (settingsType) {
    case Enum::BasicSettings_Type:             return F("Settings");
    case Enum::TaskSettings_Type:              return F("TaskSettings");
    case Enum::CustomTaskSettings_Type:        return F("CustomTaskSettings");
    case Enum::ControllerSettings_Type:        return F("ControllerSettings");
    case Enum::CustomControllerSettings_Type:  return F("CustomControllerSettings");
    case Enum::NotificationSettings_Type:      
    #if FEATURE_NOTIFIER
        return F("NotificationSettings");
    #else
        break;
    #endif
    case Enum::SecuritySettings_Type:          return F("SecuritySettings");
    case Enum::ExtdControllerCredentials_Type: return F("ExtendedControllerCredentials");

    case Enum::SettingsType_MAX: break;
  }
  return F("");
}

/********************************************************************************************\
   Offsets in settings files
 \*********************************************************************************************/
bool SettingsType::getSettingsParameters(Enum settingsType, int index, int& max_index, int& offset, int& max_size, int& struct_size) {
  // The defined offsets should be used with () just in case they are the result of a formula in the defines.
  struct_size = 0;

  switch (settingsType) {
    case Enum::BasicSettings_Type:
    {
      max_index   = 1;
      offset      = 0;
      max_size    = (DAT_BASIC_SETTINGS_SIZE);
      struct_size = sizeof(SettingsStruct);
      break;
    }
    case Enum::TaskSettings_Type:
    {
      max_index   = TASKS_MAX;
      offset      = (DAT_OFFSET_TASKS) + (index * (DAT_TASKS_DISTANCE));
      max_size    = DAT_TASKS_SIZE;
      struct_size = sizeof(ExtraTaskSettingsStruct);
      break;
    }
    case Enum::CustomTaskSettings_Type:
    {
      getSettingsParameters(Enum::TaskSettings_Type, index, max_index, offset, max_size, struct_size);
      offset  += (DAT_TASKS_CUSTOM_OFFSET);
      max_size = DAT_TASKS_CUSTOM_SIZE;

      // struct_size may differ.
      struct_size = 0;
      break;
    }
    case Enum::ControllerSettings_Type:
    {
      max_index   = CONTROLLER_MAX;
      offset      = (DAT_OFFSET_CONTROLLER) + (index * (DAT_CONTROLLER_SIZE));
      max_size    = DAT_CONTROLLER_SIZE;
      struct_size = sizeof(ControllerSettingsStruct);
      break;
    }
    case Enum::CustomControllerSettings_Type:
    {
      max_index = CONTROLLER_MAX;
      offset    = (DAT_OFFSET_CUSTOM_CONTROLLER) + (index * (DAT_CUSTOM_CONTROLLER_SIZE));
      max_size  = DAT_CUSTOM_CONTROLLER_SIZE;

      // struct_size may differ.
      struct_size = 0;
      break;
    }
    case Enum::NotificationSettings_Type:
    {
#if FEATURE_NOTIFIER
      max_index   = NOTIFICATION_MAX;
      offset      = index * (DAT_NOTIFICATION_SIZE);
      max_size    = DAT_NOTIFICATION_SIZE;
      struct_size = sizeof(NotificationSettingsStruct);
#endif
      break;
    }
    case Enum::SecuritySettings_Type:
    {
      max_index   = 1;
      offset      = 0;
      max_size    = DAT_SECURITYSETTINGS_SIZE;
      struct_size = sizeof(SecurityStruct);
      break;
    }
    case Enum::ExtdControllerCredentials_Type:
    {
      max_index = 1;
      offset    = DAT_EXTDCONTR_CRED_OFFSET;
      max_size  = DAT_EXTDCONTR_CRED_SIZE;

      // struct_size may differ.
      struct_size = 0;
      break;
    }
    case Enum::SettingsType_MAX:
    {
      max_index = -1;
      offset    = -1;
      return false;
    }
  }
  return index >= 0 && index < max_index;
}

bool SettingsType::getSettingsParameters(Enum settingsType, int index, int& offset, int& max_size) {
  int max_index = -1;
  int struct_size;

  if (!getSettingsParameters(settingsType, index, max_index, offset, max_size, struct_size)) {
    return false;
  }

  if ((index >= 0) && (index < max_index)) { return true; }
  offset = -1;
  return false;
}

int SettingsType::getMaxFilePos(Enum settingsType) {
  int max_index, offset, max_size;
  int struct_size = 0;

  getSettingsParameters(settingsType, 0,             max_index, offset, max_size, struct_size);
  getSettingsParameters(settingsType, max_index - 1, offset,    max_size);
  return offset + max_size - 1;
}

int SettingsType::getFileSize(Enum settingsType) {
  SettingsType::SettingsFileEnum file_type = SettingsType::getSettingsFile(settingsType);
  int max_file_pos                         = 0;

  for (int st = 0; st < static_cast<int>(Enum::SettingsType_MAX); ++st) {
    if (SettingsType::getSettingsFile(static_cast<Enum>(st)) == file_type) {
      int filePos = SettingsType::getMaxFilePos(static_cast<Enum>(st));

      if (filePos > max_file_pos) {
        max_file_pos = filePos;
      }
    }
  }
  return max_file_pos;
}

#ifndef BUILD_MINIMAL_OTA
unsigned int SettingsType::getSVGcolor(Enum settingsType) {
  switch (settingsType) {
    case Enum::BasicSettings_Type:
      return 0x5F0A87;
    case Enum::TaskSettings_Type:
      return 0xEE6352;
    case Enum::CustomTaskSettings_Type:
      return 0x59CD90;
    case Enum::ControllerSettings_Type:
      return 0x3FA7D6;
    case Enum::CustomControllerSettings_Type:
      return 0xFAC05E;
    case Enum::NotificationSettings_Type:
      return 0xF79D84;

    case Enum::SecuritySettings_Type:
      return 0xff00a2;
    case Enum::ExtdControllerCredentials_Type:
      return 0xc300ff;
    case Enum::SettingsType_MAX:
      break;
  }
  return 0;
}

#endif // ifndef BUILD_MINIMAL_OTA

SettingsType::SettingsFileEnum SettingsType::getSettingsFile(Enum settingsType)
{
  switch (settingsType) {
    case Enum::BasicSettings_Type:
    case Enum::TaskSettings_Type:
    case Enum::CustomTaskSettings_Type:
    case Enum::ControllerSettings_Type:
    case Enum::CustomControllerSettings_Type:
      return SettingsFileEnum::FILE_CONFIG_type;
    case Enum::NotificationSettings_Type:
      return SettingsFileEnum::FILE_NOTIFICATION_type;
    case Enum::SecuritySettings_Type:
    case Enum::ExtdControllerCredentials_Type:
      return SettingsFileEnum::FILE_SECURITY_type;

    case Enum::SettingsType_MAX:
      break;
  }
  return SettingsFileEnum::FILE_UNKNOWN_type;
}

String SettingsType::getSettingsFileName(Enum settingsType) {
  return getSettingsFileName(getSettingsFile(settingsType));
}

const __FlashStringHelper * SettingsType::getSettingsFileName(SettingsType::SettingsFileEnum file_type) {
  switch (file_type) {
    case SettingsFileEnum::FILE_CONFIG_type:        return getFileName(FileType::CONFIG_DAT);
    case SettingsFileEnum::FILE_NOTIFICATION_type:  return getFileName(FileType::NOTIFICATION_DAT);
    case SettingsFileEnum::FILE_SECURITY_type:      return getFileName(FileType::SECURITY_DAT);
    case SettingsFileEnum::FILE_UNKNOWN_type:       break;
  }
  return F("");
}

size_t SettingsType::getInitFileSize(SettingsType::SettingsFileEnum file_type) {
  switch (file_type) {
    case SettingsFileEnum::FILE_CONFIG_type:        return CONFIG_FILE_SIZE;
    case SettingsFileEnum::FILE_NOTIFICATION_type:  return 4096;
    case SettingsFileEnum::FILE_SECURITY_type:      return 4096;
    case SettingsFileEnum::FILE_UNKNOWN_type:       break;
  }
  return 0;
}
#include "../DataTypes/ESPEasyTimeSource.h"

#include <Arduino.h>

#include "../../ESPEasy_common.h"

const __FlashStringHelper* toString(timeSource_t timeSource)
{
  switch (timeSource) {
    case timeSource_t::GPS_PPS_time_source:     return F("GPS PPS");
    case timeSource_t::GPS_time_source:         return F("GPS");
    case timeSource_t::NTP_time_source:         return F("NTP");
    case timeSource_t::Manual_set:              return F("Manual");
    case timeSource_t::ESP_now_peer:            return F(ESPEASY_NOW_NAME " peer");
    case timeSource_t::External_RTC_time_source: return F("Ext. RTC at boot");
    case timeSource_t::Restore_RTC_time_source: return F("RTC at boot");
    case timeSource_t::No_time_source:          return F("No time set");
  }
  return F("Unknown");
}

bool isExternalTimeSource(timeSource_t timeSource)
{
  // timeSource_t::ESP_now_peer should NOT be considered "external"
  // It may be an unreliable source if no other source is present in the network.

  switch (timeSource) {
    case timeSource_t::GPS_PPS_time_source:
    case timeSource_t::GPS_time_source:
    case timeSource_t::NTP_time_source:
    case timeSource_t::External_RTC_time_source:
    case timeSource_t::Manual_set:
      return true;
    default:
      return false;
  }
}

