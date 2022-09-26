#include "../Helpers/StringGenerator_System.h"


/*********************************************************************************************\
   ESPEasy specific strings
\*********************************************************************************************/


#if FEATURE_MQTT

#include <PubSubClient.h>
#include "../Globals/MQTT.h"

const __FlashStringHelper * getMQTT_state() {
  switch (MQTTclient.state()) {
    case MQTT_CONNECTION_TIMEOUT: return F("Connection timeout");
    case MQTT_CONNECTION_LOST: return F("Connection lost");
    case MQTT_CONNECT_FAILED: return F("Connect failed");
    case MQTT_DISCONNECTED: return F("Disconnected");
    case MQTT_CONNECTED: return F("Connected");
    case MQTT_CONNECT_BAD_PROTOCOL: return F("Connect bad protocol");
    case MQTT_CONNECT_BAD_CLIENT_ID: return F("Connect bad client_id");
    case MQTT_CONNECT_UNAVAILABLE: return F("Connect unavailable");
    case MQTT_CONNECT_BAD_CREDENTIALS: return F("Connect bad credentials");
    case MQTT_CONNECT_UNAUTHORIZED: return F("Connect unauthorized");
    default: break;
  }
  return F("");
}

#endif // if FEATURE_MQTT

/********************************************************************************************\
   Get system information
 \*********************************************************************************************/
const __FlashStringHelper * getLastBootCauseString() {
  switch (lastBootCause)
  {
    case BOOT_CAUSE_MANUAL_REBOOT: return F("Manual Reboot");
    case BOOT_CAUSE_DEEP_SLEEP:    return F("Deep Sleep");
    case BOOT_CAUSE_COLD_BOOT:     return F("Cold Boot");
    case BOOT_CAUSE_EXT_WD:        return F("External Watchdog");
    case BOOT_CAUSE_SOFT_RESTART:  return F("Soft Reboot");
    case BOOT_CAUSE_SW_WATCHDOG:   return F("SW Watchdog");
    case BOOT_CAUSE_EXCEPTION:     return F("Exception");
    case BOOT_CAUSE_POWER_UNSTABLE: return F("PWR Unstable"); // ESP32 only
  }
  return F("Unknown");
}

#ifdef ESP32
 #ifdef ESP32S2
  #include <esp32s2/rom/rtc.h>
 #else
  #include <rom/rtc.h>
 #endif


// See https://github.com/espressif/esp-idf/blob/master/components/esp32/include/rom/rtc.h
const __FlashStringHelper * getResetReasonString_f(uint8_t icore, bool& isDEEPSLEEP_RESET) {
  isDEEPSLEEP_RESET = false;

  #ifdef ESP32S2

	// See tools\sdk\esp32\include\esp_rom\include\esp32s2\rom\rtc.h
  switch (rtc_get_reset_reason(icore)) {
    case POWERON_RESET:          return F("Vbat power on reset");                              // 1
    case RTC_SW_SYS_RESET:       return F("Software reset digital core");                      // 3
    case DEEPSLEEP_RESET:        isDEEPSLEEP_RESET = true; break;                              // 5
    case TG0WDT_SYS_RESET:       return F("Timer Group0 Watch dog reset digital core");        // 7
    case TG1WDT_SYS_RESET:       return F("Timer Group1 Watch dog reset digital core");        // 8
    case RTCWDT_SYS_RESET:       return F("RTC Watch dog Reset digital core");                 // 9
    case INTRUSION_RESET:        return F("Instrusion tested to reset CPU");                   // 10
    case TG0WDT_CPU_RESET:       return F("Time Group0 reset CPU");                            // 11
    case RTC_SW_CPU_RESET:       return F("Software reset CPU");                               // 12
    case RTCWDT_CPU_RESET:       return F("RTC Watch dog Reset CPU");                          // 13
    case RTCWDT_BROWN_OUT_RESET: return F("Reset when the vdd voltage is not stable");         // 15
    case RTCWDT_RTC_RESET:       return F("RTC Watch dog reset digital core and rtc module");  // 16
    case TG1WDT_CPU_RESET:       return F("Time Group1 reset CPU");                            // 17
    case SUPER_WDT_RESET:        return F("Super watchdog reset digital core and rtc module"); // 18
    case GLITCH_RTC_RESET:       return F("Glitch reset digital core and rtc module");         // 19
    case EFUSE_RESET:            return F("EFUSE_RESET"); // FIXME TD-er: No idea what may cause this
    case NO_MEAN:                break; // Undefined, "No Meaning"
  }

  #else

  // See https://github.com/espressif/esp-idf/blob/master/components/esp32/include/rom/rtc.h
  switch (rtc_get_reset_reason((RESET_REASON)icore)) {
    case NO_MEAN:                return F("NO_MEAN");
    case POWERON_RESET:          return F("Vbat power on reset");
    case SW_RESET:               return F("Software reset digital core");
    case OWDT_RESET:             return F("Legacy watch dog reset digital core");
    case DEEPSLEEP_RESET:        isDEEPSLEEP_RESET = true; break;
    case SDIO_RESET:             return F("Reset by SLC module, reset digital core");
    case TG0WDT_SYS_RESET:       return F("Timer Group0 Watch dog reset digital core");
    case TG1WDT_SYS_RESET:       return F("Timer Group1 Watch dog reset digital core");
    case RTCWDT_SYS_RESET:       return F("RTC Watch dog Reset digital core");
    case INTRUSION_RESET:        return F("Instrusion tested to reset CPU");
    case TGWDT_CPU_RESET:        return F("Time Group reset CPU");
    case SW_CPU_RESET:           return F("Software reset CPU");
    case RTCWDT_CPU_RESET:       return F("RTC Watch dog Reset CPU");
    case EXT_CPU_RESET:          return F("for APP CPU, reseted by PRO CPU");
    case RTCWDT_BROWN_OUT_RESET: return F("Reset when the vdd voltage is not stable");
    case RTCWDT_RTC_RESET:       return F("RTC Watch dog reset digital core and rtc module");
    default: break;
  }
  #endif
  return F("");
}


String getResetReasonString(uint8_t icore) {
  bool isDEEPSLEEP_RESET(false);
  const String res = getResetReasonString_f(icore, isDEEPSLEEP_RESET);
  if (!res.isEmpty()) return res;

  if (isDEEPSLEEP_RESET) {
    String reason = F("Deep Sleep, Wakeup reason (");
    reason += rtc_get_wakeup_cause();
    reason += ')';

/*
  switch (reason) {
  #if CONFIG_IDF_TARGET_ESP32
    case POWERON_RESET:
    case SW_CPU_RESET:
    case DEEPSLEEP_RESET:
    case SW_RESET:
  #elif CONFIG_IDF_TARGET_ESP32S2
    case POWERON_RESET:
    case RTC_SW_CPU_RESET:
    case DEEPSLEEP_RESET:
    case RTC_SW_SYS_RESET:
  #endif
  }
*/
    return reason;
  }

  return getUnknownString();
}

#endif // ifdef ESP32

String getResetReasonString() {
  #ifdef ESP32
  String reason = F("CPU0: ");
  reason += getResetReasonString(0);
  if (getChipCores() > 1) { // Only report if we really have more than 1 core
    reason += F(" CPU1: ");
    reason += getResetReasonString(1);
  }
  return reason;
  #else // ifdef ESP32
  return ESP.getResetReason();
  #endif // ifdef ESP32
}

String getSystemBuildString() {
  String result;

  result += BUILD;
  result += ' ';
  result += F(BUILD_NOTES);
  return result;
}

String getPluginDescriptionString() {
  String result;

  #ifdef PLUGIN_BUILD_NORMAL
  result += F(" [Normal]");
  #endif // ifdef PLUGIN_BUILD_NORMAL
  #ifdef PLUGIN_BUILD_COLLECTION
  result += F(" [Collection]");
  #endif // ifdef PLUGIN_BUILD_COLLECTION
  #ifdef PLUGIN_BUILD_DEV
  result += F(" [Development]");
  #endif // ifdef PLUGIN_BUILD_DEV
  #ifdef PLUGIN_DESCR
  result += F(" [");
  result += F(PLUGIN_DESCR);
  result += ']';
  #endif // ifdef PLUGIN_DESCR
  #if FEATURE_NON_STANDARD_24_TASKS && defined(ESP8266)
  result += F(" 24tasks");
  #endif // if FEATURE_NON_STANDARD_24_TASKS && defined(ESP8266)
  result.trim();
  return result;
}

String getSystemLibraryString() {
  String result;

  #if defined(ESP32)
  result += F("ESP32 SDK ");
  result += ESP.getSdkVersion();
  #else // if defined(ESP32)
  result += F("ESP82xx Core ");
  result += ESP.getCoreVersion();
  result += F(", NONOS SDK ");
  result += system_get_sdk_version();
  result += F(", LWIP: ");
  result += getLWIPversion();
  #endif // if defined(ESP32)

  if (puyaSupport()) {
    result += F(" PUYA support");
  }
  return result;
}

#ifdef ESP8266
String getLWIPversion() {
  String result;

  result += LWIP_VERSION_MAJOR;
  result += '.';
  result += LWIP_VERSION_MINOR;
  result += '.';
  result += LWIP_VERSION_REVISION;

  if (LWIP_VERSION_IS_RC) {
    result += F("-RC");
    result += LWIP_VERSION_RC;
  } else if (LWIP_VERSION_IS_DEVELOPMENT) {
    result += F("-dev");
  }
  return result;
}

#endif // ifdef ESP8266

#include "../Helpers/ESPEasy_time.h"

#include "../../ESPEasy_common.h"

#include "../DataTypes/TimeSource.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"

#include "../Globals/EventQueue.h"
#include "../Globals/NetworkState.h"
#include "../Globals/RTC.h"
#include "../Globals/Settings.h"
#include "../Globals/TimeZone.h"

#include "../Helpers/Convert.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Networking.h"
#include "../Helpers/Numerical.h"

#include "../Helpers/ESPEasy_time_calc.h"

#include <time.h>

#if FEATURE_EXT_RTC
#include <RTClib.h>
#endif




ESPEasy_time::ESPEasy_time() {
  memset(&tm,      0, sizeof(tm));
  memset(&tsRise,  0, sizeof(tm));
  memset(&tsSet,   0, sizeof(tm));
  memset(&sunRise, 0, sizeof(tm));
  memset(&sunSet,  0, sizeof(tm));
}

struct tm ESPEasy_time::addSeconds(const struct tm& ts, int seconds, bool toLocalTime) const {
  unsigned long time = makeTime(ts);

  time += seconds;

  if (toLocalTime) {
    time = time_zone.toLocal(time);
  }
  struct tm result;
  breakTime(time, result);
  return result;
}

void ESPEasy_time::breakTime(unsigned long timeInput, struct tm& tm) {
  uint32_t time = (uint32_t)timeInput;
  tm.tm_sec  = time % 60;
  time      /= 60;                   // now it is minutes
  tm.tm_min  = time % 60;
  time      /= 60;                   // now it is hours
  tm.tm_hour = time % 24;
  time      /= 24;                   // now it is days
  tm.tm_wday = ((time + 4) % 7) + 1; // Sunday is day 1

  int      year = 1970;
  unsigned long days = 0;
  while ((unsigned)(days += (isLeapYear(year) ? 366 : 365)) <= time) {
    year++;
  }
  tm.tm_year = year - 1900; // tm_year starts at 1900

  days -= isLeapYear(year) ? 366 : 365;
  time -= days;      // now it is days in this year, starting at 0

  uint8_t month = 0;
  for (month = 0; month < 12; month++) {
    const uint8_t monthLength = getMonthDays(year, month);
    if (time >= monthLength) {
      time -= monthLength;
    } else {
      break;
    }
  }
  tm.tm_mon  = month;     // Jan is month 0
  tm.tm_mday = time + 1;  // day of month start at 1
}


void ESPEasy_time::restoreFromRTC()
{
  static bool firstCall = true;
  uint32_t unixtime = 0;
  if (ExtRTC_get(unixtime)) {
    setExternalTimeSource(unixtime, timeSource_t::External_RTC_time_source);
    firstCall = false;
    return;
  }

  if (firstCall && RTC.lastSysTime != 0 && RTC.deepSleepState != 1) {
    firstCall = false;
    setExternalTimeSource(RTC.lastSysTime, timeSource_t::Restore_RTC_time_source);
    // Do not add the current uptime as offset. This will be done when calling now()
    lastSyncTime = 0;
    initTime();
  }
}

void ESPEasy_time::setExternalTimeSource(double time, timeSource_t source) {
  timeSource         = source;
  externalUnixTime_d = time;
  lastSyncTime       = millis();
  initTime();
}

uint32_t ESPEasy_time::getUnixTime() const
{
  return static_cast<uint32_t>(sysTime);
}

void ESPEasy_time::initTime()
{
  nextSyncTime = 0;
  now();
}

unsigned long ESPEasy_time::now() {
  // calculate number of seconds passed since last call to now()
  bool timeSynced        = false;
  const long msec_passed = timePassedSince(prevMillis);

  sysTime    += static_cast<double>(msec_passed) / 1000.0;
  prevMillis += msec_passed;

  if (nextSyncTime <= sysTime) {
    // nextSyncTime & sysTime are in seconds
    double unixTime_d = -1.0;

    if (externalUnixTime_d > 0.0) {
      unixTime_d = externalUnixTime_d;

      // Correct for the delay between the last received external time and applying it
      unixTime_d        += (timePassedSince(lastSyncTime) / 1000.0);
      externalUnixTime_d = -1.0;
    }

    // Try NTP if the time source is not external.
    bool updatedTime = (unixTime_d > 0.0);
    if (!isExternalTimeSource(timeSource) 
        || timeSource_t::NTP_time_source == timeSource 
        || timePassedSince(lastSyncTime) > static_cast<long>(1000 * syncInterval)) {
      if (getNtpTime(unixTime_d)) {
        updatedTime = true;
      } else {
        uint32_t tmp_unixtime = 0;;
        if (ExtRTC_get(tmp_unixtime)) {
          unixTime_d = tmp_unixtime;
          timeSource = timeSource_t::External_RTC_time_source;
          updatedTime = true;
        }
      }
    }
    if (updatedTime) {
      const double time_offset = unixTime_d - sysTime - (timePassedSince(prevMillis) / 1000.0);

      if (statusNTPInitialized && time_offset < 1.0) {
        // Clock instability in msec/second
        timeWander = ((time_offset * 1000000.0) / timePassedSince(lastTimeWanderCalculation));
      }

      prevMillis = millis(); // restart counting from now (thanks to Korman for this fix)
      lastTimeWanderCalculation = prevMillis;
      
      timeSynced = true;

      sysTime = unixTime_d;
      ExtRTC_set(sysTime);
      {
        const unsigned long abs_time_offset_ms = std::abs(time_offset) * 1000;

        if (timeSource == timeSource_t::NTP_time_source) {
          // May need to lessen the load on the NTP servers, randomize the sync interval
          if (abs_time_offset_ms < 1000) {
            // offset is less than 1 second, so we consider it a regular time sync.
            if (abs_time_offset_ms < 100) {
              // Good clock stability, use 5 - 6 hour interval
              syncInterval = random(18000, 21600);
            } else {
              // Dynamic interval between 30 minutes ... 5 hours.
              syncInterval = 1800000 / abs_time_offset_ms;
            }
          } else {
            syncInterval = 3600;
          }
          if (syncInterval <= 3600) {
            syncInterval = random(3600, 4000);
          }
        } else {
          syncInterval = 3600;
        }
      }

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log         = F("Time set to ");
        log += doubleToString(unixTime_d, 3);

        if ((-86400 < time_offset) && (time_offset < 86400)) {
          // Only useful to show adjustment if it is less than a day.
          log += F(" Time adjusted by ");
          log += doubleToString(time_offset * 1000.0);
          log += F(" msec. Wander: ");
          log += doubleToString(timeWander, 3);
          log += F(" msec/second");
          log += F(" Source: ");
          log += toString(timeSource);
        }
        addLogMove(LOG_LEVEL_INFO, log);
      }

      time_zone.applyTimeZone(unixTime_d);
      nextSyncTime = (uint32_t)unixTime_d + syncInterval;
      if (isExternalTimeSource(timeSource)) {
        #ifdef USES_ESPEASY_NOW
        ESPEasy_now_handler.sendNTPbroadcast();
        #endif
      }
    }
  }
  RTC.lastSysTime = static_cast<unsigned long>(sysTime);
  uint32_t localSystime = time_zone.toLocal(sysTime);
  breakTime(localSystime, tm);

  if (timeSynced) {
    calcSunRiseAndSet();

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("Local time: ");
      log += getDateTimeString('-', ':', ' ');
      addLogMove(LOG_LEVEL_INFO, log);
    }
    {
      // Notify plugins the time has been set.
      String dummy;
      PluginCall(PLUGIN_TIME_CHANGE, 0, dummy);
    }

    if (Settings.UseRules) {
      if (statusNTPInitialized) {
        eventQueue.add(F("Time#Set"));
      } else {
        eventQueue.add(F("Time#Initialized"));
      }
    }
    statusNTPInitialized = true; // @giig1967g: setting system variable %isntp%
  }
  return (unsigned long)localSystime;
}

bool ESPEasy_time::reportNewMinute()
{
  now();

  if (!systemTimePresent()) {
    return false;
  }

  if (tm.tm_min == PrevMinutes)
  {
    return false;
  }
  PrevMinutes = tm.tm_min;
  return true;
}

bool ESPEasy_time::systemTimePresent() const {
  switch (timeSource) {
    case timeSource_t::No_time_source: 
      break;
    case timeSource_t::NTP_time_source:  
    case timeSource_t::Restore_RTC_time_source: 
    case timeSource_t::External_RTC_time_source:
    case timeSource_t::GPS_time_source:
    case timeSource_t::GPS_PPS_time_source:
    case timeSource_t::ESP_now_peer:
    case timeSource_t::Manual_set:
      return true;
  }
  return nextSyncTime > 0 || Settings.UseNTP() || externalUnixTime_d > 0.0;
}

bool ESPEasy_time::getNtpTime(double& unixTime_d)
{
  if (!Settings.UseNTP() || !NetworkConnected(10)) {
    return false;
  }
  if (lastNTPSyncTime != 0) {
    if (timePassedSince(lastNTPSyncTime) < static_cast<long>(1000 * syncInterval)) {
      // Make sure not to flood the NTP servers with requests.
      return false;
    }
  }

  IPAddress timeServerIP;
  String    log = F("NTP  : NTP host ");

  bool useNTPpool = false;

  if (Settings.NTPHost[0] != 0) {
    resolveHostByName(Settings.NTPHost, timeServerIP);
    log += Settings.NTPHost;

    // When single set host fails, retry again in 20 seconds
    nextSyncTime = sysTime + random(20, 60);
  } else  {
    // Have to do a lookup each time, since the NTP pool always returns another IP
    String ntpServerName = String(random(0, 3));
    ntpServerName += F(".pool.ntp.org");
    resolveHostByName(ntpServerName.c_str(), timeServerIP);
    log += ntpServerName;

    // When pool host fails, retry can be much sooner
    nextSyncTime = sysTime + random(5, 20);
    useNTPpool = true;
  }

  log += F(" (");
  log += timeServerIP.toString();
  log += ')';

  if (!hostReachable(timeServerIP)) {
    log += F(" unreachable");
    addLogMove(LOG_LEVEL_INFO, log);
    return false;
  }

  WiFiUDP udp;

  if (!beginWiFiUDP_randomPort(udp)) {
    return false;
  }

  const int NTP_PACKET_SIZE = 48;     // NTP time is in the first 48 bytes of message
  uint8_t packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming & outgoing packets

  log += F(" queried");
#ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG_MORE, log);
#endif // ifndef BUILD_NO_DEBUG

  while (udp.parsePacket() > 0) { // discard any previously received packets
  }
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0]  = 0b11100011; // LI, Version, Mode
  packetBuffer[1]  = 0;          // Stratum, or type of clock
  packetBuffer[2]  = 6;          // Polling Interval
  packetBuffer[3]  = 0xEC;       // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  FeedSW_watchdog();
  if (udp.beginPacket(timeServerIP, 123) == 0) { // NTP requests are to port 123
    FeedSW_watchdog();
    udp.stop();
    return false;
  }
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();


  uint32_t beginWait = millis();

  while (!timeOutReached(beginWait + 1000)) {
    int size       = udp.parsePacket();
    int remotePort = udp.remotePort();

    if ((size >= NTP_PACKET_SIZE) && (remotePort == 123)) {
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer

      if ((packetBuffer[0] & 0b11000000) == 0b11000000) {
        // Leap-Indicator: unknown (clock unsynchronized)
        // See: https://github.com/letscontrolit/ESPEasy/issues/2886#issuecomment-586656384
        if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
          String log = F("NTP  : NTP host (");
          log += timeServerIP.toString();
          log += F(") unsynchronized");
          addLogMove(LOG_LEVEL_ERROR, log);
        }

        if (!useNTPpool) {
          // Does not make sense to try it very often if a single host is used which is not synchronized.
          nextSyncTime = sysTime + 120;
        }
        udp.stop();
        return false;
      }

      // For more detailed info on improving accuracy, see:
      // https://github.com/lettier/ntpclient/issues/4#issuecomment-360703503
      // For now, we simply use half the reply time as delay compensation.

      unsigned long secsSince1900;

      // convert four bytes starting at location 40 to a long integer
      // TX time is used here.
      secsSince1900  = (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];

      if (secsSince1900 == 0) {
        // No time stamp received

        if (!useNTPpool) {
          // Retry again in a minute.
          nextSyncTime = sysTime + 60;
        }
        udp.stop();
        return false;
      }
      uint32_t txTm = secsSince1900 - 2208988800UL;

      unsigned long txTm_f;
      txTm_f  = (unsigned long)packetBuffer[44] << 24;
      txTm_f |= (unsigned long)packetBuffer[45] << 16;
      txTm_f |= (unsigned long)packetBuffer[46] << 8;
      txTm_f |= (unsigned long)packetBuffer[47];

      // Convert seconds to double
      unixTime_d = static_cast<double>(txTm);

      // Add fractional part.
      unixTime_d += (static_cast<double>(txTm_f) / 4294967295.0);

      long total_delay = timePassedSince(beginWait);
      lastSyncTime = millis();

      // compensate for the delay by adding half the total delay
      // N.B. unixTime_d is in seconds and delay in msec.
      double delay_compensation = static_cast<double>(total_delay) / 2000.0;
      unixTime_d += delay_compensation;

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("NTP  : NTP replied: delay ");
        log += total_delay;
        log += F(" mSec");
        log += F(" Accuracy increased by ");
        double fractpart, intpart;
        fractpart = modf(unixTime_d, &intpart);

        if (fractpart < delay_compensation) {
          // We gained more than 1 second in accuracy
          fractpart += 1.0;
        }
        log += doubleToString(fractpart, 3);
        log += F(" seconds");
        addLogMove(LOG_LEVEL_INFO, log);
      }
      udp.stop();
      timeSource = timeSource_t::NTP_time_source;
      lastNTPSyncTime = millis();
      CheckRunningServices(); // FIXME TD-er: Sometimes services can only be started after NTP is successful
      return true;
    }
    delay(10);
  }

  // Timeout.
  if (!useNTPpool) {
    // Retry again in a minute.
    nextSyncTime = sysTime + 60;
  }

#ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG_MORE, F("NTP  : No reply"));
#endif // ifndef BUILD_NO_DEBUG
  udp.stop();
  return false;
}

/********************************************************************************************\
   Date/Time string formatters
 \*********************************************************************************************/
String ESPEasy_time::getDateString(char delimiter) const
{
  return getDateString(tm, delimiter);
}

String ESPEasy_time::getDateString(const struct tm& ts, char delimiter) {
  // time format example with ':' delimiter: 23:59:59 (HH:MM:SS)
  char DateString[20]; // 19 digits plus the null char
  const int year = 1900 + ts.tm_year;

  sprintf_P(DateString, PSTR("%4d%c%02d%c%02d"), year, delimiter, ts.tm_mon + 1, delimiter, ts.tm_mday);
  return DateString;
}

String ESPEasy_time::getTimeString(char delimiter, bool show_seconds /*=true*/, char hour_prefix /*='\0'*/) const
{
  return getTimeString(tm, delimiter, false, show_seconds, hour_prefix);
}

String ESPEasy_time::getTimeString_ampm(char delimiter, bool show_seconds /*=true*/, char hour_prefix /*='\0'*/) const
{
  return getTimeString(tm, delimiter, true, show_seconds, hour_prefix);
}

// returns the current Time separated by the given delimiter
// time format example with ':' delimiter: 23:59:59 (HH:MM:SS)
String ESPEasy_time::getTimeString(const struct tm& ts, char delimiter, bool am_pm, bool show_seconds, char hour_prefix /*='\0'*/)
{
  char TimeString[20]; // 19 digits plus the null char
  char hour_prefix_s[2] = { 0 };

  if (am_pm) {
    uint8_t hour(ts.tm_hour % 12);

    if (hour == 0) { hour = 12; }
    const char a_or_p = ts.tm_hour < 12 ? 'A' : 'P';
    if (hour < 10) { hour_prefix_s[0] = hour_prefix; }

    if (show_seconds) {
      sprintf_P(TimeString, PSTR("%s%d%c%02d%c%02d %cM"),
                hour_prefix_s, hour, delimiter, ts.tm_min, delimiter, ts.tm_sec, a_or_p);
    } else {
      sprintf_P(TimeString, PSTR("%s%d%c%02d %cM"),
                hour_prefix_s, hour, delimiter, ts.tm_min, a_or_p);
    }
  } else {
    if (show_seconds) {
      sprintf_P(TimeString, PSTR("%02d%c%02d%c%02d"),
                ts.tm_hour, delimiter, ts.tm_min, delimiter, ts.tm_sec);
    } else {
      if (ts.tm_hour < 10) { hour_prefix_s[0] = hour_prefix; }
      sprintf_P(TimeString, PSTR("%s%d%c%02d"),
                hour_prefix_s, ts.tm_hour, delimiter, ts.tm_min);
    }
  }
  return TimeString;
}

String ESPEasy_time::getDateTimeString(char dateDelimiter, char timeDelimiter,  char dateTimeDelimiter) const {
  return getDateTimeString(tm, dateDelimiter, timeDelimiter, dateTimeDelimiter, false);
}

String ESPEasy_time::getDateTimeString_ampm(char dateDelimiter, char timeDelimiter,  char dateTimeDelimiter) const {
  return getDateTimeString(tm, dateDelimiter, timeDelimiter, dateTimeDelimiter, true);
}

String ESPEasy_time::getDateTimeString(const struct tm& ts, char dateDelimiter, char timeDelimiter,  char dateTimeDelimiter, bool am_pm)
{
  // if called like this: getDateTimeString('\0', '\0', '\0');
  // it will give back this: 20161231235959  (YYYYMMDDHHMMSS)
  String ret = getDateString(ts, dateDelimiter);

  if (dateTimeDelimiter != '\0') {
    ret += dateTimeDelimiter;
  }
  ret += getTimeString(ts, timeDelimiter, am_pm, true);
  return ret;
}

/********************************************************************************************\
   Get current time/date
 \*********************************************************************************************/
int ESPEasy_time::year(unsigned long t)
{
  struct tm tmp;

  breakTime(t, tmp);
  return 1900 + tmp.tm_year;
}

int ESPEasy_time::weekday(unsigned long t)
{
  struct tm tmp;

  breakTime(t, tmp);
  return tmp.tm_wday;
}

String ESPEasy_time::weekday_str(int wday)
{
  const String weekDays = F("SunMonTueWedThuFriSat");

  return weekDays.substring(wday * 3, wday * 3 + 3);
}

String ESPEasy_time::weekday_str() const
{
  return weekday_str(weekday() - 1);
}

/********************************************************************************************\
   Sunrise/Sunset calculations
 \*********************************************************************************************/
int ESPEasy_time::getSecOffset(const String& format) {
  int position_minus = format.indexOf('-');
  int position_plus  = format.indexOf('+');

  if ((position_minus == -1) && (position_plus == -1)) {
    return 0;
  }
  int sign_position    = _max(position_minus, position_plus);
  int position_percent = format.indexOf('%', sign_position);

  if (position_percent == -1) {
    return 0;
  }

  int value;
  if (!validIntFromString(format.substring(sign_position, position_percent), value)) {
    return 0;
  }

  switch (format.charAt(position_percent - 1)) {
    case 'm':
    case 'M':
      return value * 60;
    case 'h':
    case 'H':
      return value * 3600;
  }
  return value;
}

String ESPEasy_time::getSunriseTimeString(char delimiter) const {
  return getTimeString(sunRise, delimiter, false, false);
}

String ESPEasy_time::getSunsetTimeString(char delimiter) const {
  return getTimeString(sunSet, delimiter, false, false);
}

String ESPEasy_time::getSunriseTimeString(char delimiter, int secOffset) const {
  if (secOffset == 0) {
    return getSunriseTimeString(delimiter);
  }
  return getTimeString(getSunRise(secOffset), delimiter, false, false);
}

String ESPEasy_time::getSunsetTimeString(char delimiter, int secOffset) const {
  if (secOffset == 0) {
    return getSunsetTimeString(delimiter);
  }
  return getTimeString(getSunSet(secOffset), delimiter, false, false);
}

float ESPEasy_time::sunDeclination(int doy) {
  // Declination of the sun in radians
  // Formula 2008 by Arnold(at)Barmettler.com, fit to 20 years of average declinations (2008-2027)
  return 0.409526325277017 * sin(0.0169060504029192 * (doy - 80.0856919827619));
}

float ESPEasy_time::diurnalArc(float dec, float lat) {
  // Duration of the half sun path in hours (time from sunrise to the highest level in the south)
  float rad    = 0.0174532925f; // = pi/180.0
  float height = -50.0f / 60.0f * rad;
  float latRad = lat * rad;

  return 12.0f * acos((sin(height) - sin(latRad) * sin(dec)) / (cos(latRad) * cos(dec))) / M_PI;
}

float ESPEasy_time::equationOfTime(int doy) {
  // Difference between apparent and mean solar time
  // Formula 2008 by Arnold(at)Barmettler.com, fit to 20 years of average equation of time (2008-2027)
  return -0.170869921174742 * sin(0.0336997028793971 * doy + 0.465419984181394) - 0.129890681040717 * sin(
    0.0178674832556871 * doy - 0.167936777524864);
}

int ESPEasy_time::dayOfYear(int year, int month, int day) {
  // Algorithm borrowed from DateToOrdinal by Ritchie Lawrence, www.commandline.co.uk
  int z = 14 - month;

  z /= 12;
  int y = year + 4800 - z;
  int m = month + 12 * z - 3;
  int j = 153 * m + 2;
  j = j / 5 + day + y * 365 + y / 4 - y / 100 + y / 400 - 32045;
  y = year + 4799;
  int k = y * 365 + y / 4 - y / 100 + y / 400 - 31738;
  return j - k + 1;
}

void ESPEasy_time::calcSunRiseAndSet() {
  int   doy  = dayOfYear(tm.tm_year, tm.tm_mon + 1, tm.tm_mday);
  float eqt  = equationOfTime(doy);
  float dec  = sunDeclination(doy);
  float da   = diurnalArc(dec, Settings.Latitude);
  float rise = 12 - da - eqt;
  float set  = 12 + da - eqt;

  tsRise.tm_hour = rise;
  tsRise.tm_min  = (rise - static_cast<int>(rise)) * 60.0f;
  tsSet.tm_hour  = set;
  tsSet.tm_min   = (set - static_cast<int>(set)) * 60.0f;
  tsRise.tm_mday = tsSet.tm_mday = tm.tm_mday;
  tsRise.tm_mon  = tsSet.tm_mon = tm.tm_mon;
  tsRise.tm_year = tsSet.tm_year = tm.tm_year;

  // Now apply the longitude
  int secOffset_longitude = -1.0f * (Settings.Longitude / 15.0f) * 3600;
  tsSet  = addSeconds(tsSet, secOffset_longitude, false);
  tsRise = addSeconds(tsRise, secOffset_longitude, false);

  breakTime(time_zone.toLocal(makeTime(tsRise)), sunRise);
  breakTime(time_zone.toLocal(makeTime(tsSet)),   sunSet);
}

struct tm ESPEasy_time::getSunRise(int secOffset) const {
  return addSeconds(tsRise, secOffset, true);
}

struct tm ESPEasy_time::getSunSet(int secOffset) const {
  return addSeconds(tsSet, secOffset, true);
}

bool ESPEasy_time::ExtRTC_get(uint32_t &unixtime)
{
  unixtime = 0;
  switch (Settings.ExtTimeSource()) {
    case ExtTimeSource_e::None:
      return false;
    case ExtTimeSource_e::DS1307:
      {
        #if FEATURE_EXT_RTC
        I2CSelect_Max100kHz_ClockSpeed(); // Only supports upto 100 kHz
        RTC_DS1307 rtc;
        if (!rtc.begin()) {
          // Not found
          break;
        }
        if (!rtc.isrunning()) {
          // not running
          break;
        }
        unixtime = rtc.now().unixtime();
        #endif // if FEATURE_EXT_RTC
        break;
      }
    case ExtTimeSource_e::DS3231:
      {
        #if FEATURE_EXT_RTC
        RTC_DS3231 rtc;
        if (!rtc.begin()) {
          // Not found
          break;
        }
        if (rtc.lostPower()) {
          // Cannot get the time from the module
          break;
        }
        unixtime = rtc.now().unixtime();
        #endif // if FEATURE_EXT_RTC
        break;
      }
      
    case ExtTimeSource_e::PCF8523:
      {
        #if FEATURE_EXT_RTC
        RTC_PCF8523 rtc;
        if (!rtc.begin()) {
          // Not found
          break;
        }
        if (rtc.lostPower() || !rtc.initialized() || !rtc.isrunning()) {
          // Cannot get the time from the module
          break;
        }
        unixtime = rtc.now().unixtime();
        #endif // if FEATURE_EXT_RTC
        break;
      }
    case ExtTimeSource_e::PCF8563:
      {
        #if FEATURE_EXT_RTC
        RTC_PCF8563 rtc;
        if (!rtc.begin()) {
          // Not found
          break;
        }
        if (rtc.lostPower() || !rtc.isrunning()) {
          // Cannot get the time from the module
          break;
        }
        unixtime = rtc.now().unixtime();
        #endif // if FEATURE_EXT_RTC
        break;
      }

  }
  if (unixtime != 0) {
    String log = F("ExtRTC: Read external time source: ");
    log += unixtime;
    addLogMove(LOG_LEVEL_INFO, log);
    return true;
  }
  addLog(LOG_LEVEL_ERROR, F("ExtRTC: Cannot get time from external time source"));
  return false;
}

bool ESPEasy_time::ExtRTC_set(uint32_t unixtime)
{
  if (timeSource == timeSource_t::External_RTC_time_source || 
      !isExternalTimeSource(timeSource)) {
    // Do not adjust the external RTC time if we already used it as a time source.
    return true;
  }
  #if FEATURE_EXT_RTC
  bool timeAdjusted = false;
  #endif // if FEATURE_EXT_RTC
  switch (Settings.ExtTimeSource()) {
    case ExtTimeSource_e::None:
      return false;
    case ExtTimeSource_e::DS1307:
      {
        #if FEATURE_EXT_RTC
        I2CSelect_Max100kHz_ClockSpeed(); // Only supports upto 100 kHz
        RTC_DS1307 rtc;
        if (rtc.begin()) {
          rtc.adjust(DateTime(unixtime));
          timeAdjusted = true;
        }
        #endif // if FEATURE_EXT_RTC
        break;
      }
    case ExtTimeSource_e::DS3231:
      {
        #if FEATURE_EXT_RTC
        RTC_DS3231 rtc;
        if (rtc.begin()) {
          rtc.adjust(DateTime(unixtime));
          timeAdjusted = true;
        }
        #endif // if FEATURE_EXT_RTC
        break;
      }
      
    case ExtTimeSource_e::PCF8523:
      {
        #if FEATURE_EXT_RTC
        RTC_PCF8523 rtc;
        if (rtc.begin()) {
          rtc.adjust(DateTime(unixtime));
          rtc.start();
          timeAdjusted = true;
        }
        #endif // if FEATURE_EXT_RTC
        break;
      }
    case ExtTimeSource_e::PCF8563:
      {
        #if FEATURE_EXT_RTC
        RTC_PCF8563 rtc;
        if (rtc.begin()) {
          rtc.adjust(DateTime(unixtime));
          rtc.start();
          timeAdjusted = true;
        }
        #endif // if FEATURE_EXT_RTC
        break;
      }
  }
  #if FEATURE_EXT_RTC
  if (timeAdjusted) {
    String log = F("ExtRTC: External time source set to: ");
    log += unixtime;
    addLogMove(LOG_LEVEL_INFO, log);
    return true;
  }
  #endif // if FEATURE_EXT_RTC
  addLog(LOG_LEVEL_ERROR, F("ExtRTC: Cannot set time to external time source"));
  return false;
}

#include "../Helpers/Audio.h"

#include "../Globals/RamTracker.h"
#include "../Helpers/Hardware.h"


/********************************************************************************************\
   Generate a tone of specified frequency on pin
 \*********************************************************************************************/
bool tone_espEasy(int8_t _pin, unsigned int frequency, unsigned long duration) {
  if (_pin<0) return false;

  // Duty cycle can be used as some kind of volume.
  if (!set_Gpio_PWM_pct(_pin, 50, frequency)) return false;
  if (duration > 0) {
    delay(duration);
    return set_Gpio_PWM(_pin, 0, frequency);
  }
  return true;
  
}

/********************************************************************************************\
   Play RTTTL string on specified pin
 \*********************************************************************************************/
#if FEATURE_RTTTL
bool play_rtttl(int8_t _pin, const char *p)
{
  if (_pin<0) return false;
  
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("play_rtttl"));
  #endif
  #define OCTAVE_OFFSET 0

  // FIXME: Absolutely no error checking in here

  const int notes[] = { 0,
                        262, 277,   294,  311,   330,  349,  370,  392,  415,  440,  466,  494,
                        523, 554,   587,  622,   659,  698,  740,  784,  831,  880,  932,  988,
                        1047,1109,  1175, 1245,  1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
                        2093,2217,  2349, 2489,  2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951
  };


  uint8_t default_dur = 4;
  uint8_t default_oct = 6;
  int  bpm         = 63;
  int  num;
  long wholenote;
  long duration;
  uint8_t note;
  uint8_t scale;

  // format: d=N,o=N,b=NNN:
  // find the start (skip name, etc)

  while (*p != ':') { 
    p++; // ignore name
    if (*p == 0) return false;
  }
  p++;                     // skip ':'

  // get default duration
  if (*p == 'd')
  {
    p++; p++; // skip "d="
    num = 0;

    while (isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }

    if (num > 0) { default_dur = num; }
    p++; // skip comma
  }

  // get default octave
  if (*p == 'o')
  {
    p++; p++; // skip "o="
    num = *p++ - '0';

    if ((num >= 3) && (num <= 7)) { default_oct = num; }
    p++; // skip comma
  }

  // get BPM
  if (*p == 'b')
  {
    p++; p++; // skip "b="
    num = 0;

    while (isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    bpm = num;
    p++; // skip colon
  }

  // BPM usually expresses the number of quarter notes per minute
  wholenote = (60 * 1000L / bpm) * 4; // this is the time for whole note (in milliseconds)

  // now begin note loop
  while (*p)
  {
    // first, get note duration, if available
    num = 0;

    while (isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }

    if (num) { duration = wholenote / num; }
    else { duration = wholenote / default_dur; // we will need to check if we are a dotted note after
    }

    // now get the note
    switch (*p)
    {
      case 'c':
        note = 1;
        break;
      case 'd':
        note = 3;
        break;
      case 'e':
        note = 5;
        break;
      case 'f':
        note = 6;
        break;
      case 'g':
        note = 8;
        break;
      case 'a':
        note = 10;
        break;
      case 'b':
        note = 12;
        break;
      case 'p':
      default:
        note = 0;
    }
    p++;

    // now, get optional '#' sharp
    if (*p == '#')
    {
      note++;
      p++;
    }

    // now, get optional '.' dotted note
    if (*p == '.')
    {
      duration += duration / 2;
      p++;
    }

    // now, get scale
    if (isdigit(*p))
    {
      scale = *p - '0';
      p++;
    }
    else
    {
      scale = default_oct;
    }

    scale += OCTAVE_OFFSET;

    if (*p == ',') {
      p++; // skip comma for next note (or we may be at the end)
    }

    // now play the note
    if (note)
    {
      if (!tone_espEasy(_pin, notes[(scale - 4) * 12 + note], duration)) {
        return false;
      }
    }
    else
    {
      delay(duration / 10);
    }
  }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("play_rtttl2"));
  #endif
  return true;
}
#endif // if FEATURE_RTTTL

#include "../Helpers/CRC_functions.h"


int calc_CRC16(const String& text) {
  return calc_CRC16(text.c_str(), text.length());
}

int calc_CRC16(const char *ptr, int count)
{
  int crc;

  crc = 0;

  if (ptr == nullptr) {
    return crc;
  }

  while (--count >= 0)
  {
    crc = crc ^ static_cast<int>(*ptr++) << 8;
    char i = 8;

    do
    {
      if (crc & 0x8000) {
        crc = crc << 1 ^ 0x1021;
      }
      else {
        crc = crc << 1;
      }
    } while(--i);
  }
  return crc;
}

uint32_t calc_CRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;

  if (data != nullptr) {
    while (length--) {
      uint8_t c = *data++;

      for (uint32_t i = 0x80; i > 0; i >>= 1) {
        bool bit = crc & 0x80000000;

        if (c & i) {
          bit = !bit;
        }
        crc <<= 1;

        if (bit) {
          crc ^= 0x04c11db7;
        }
      }
    }
  }
  return crc;
}

#include "../Helpers/ESPEasy_FactoryDefault.h"

#include "../../ESPEasy_common.h"
#include "../../_Plugin_Helper.h"

#include "../CustomBuild/StorageLayout.h"

#include "../DataStructs/ControllerSettingsStruct.h"
#include "../DataStructs/FactoryDefaultPref.h"
#include "../DataStructs/GpioFactorySettingsStruct.h"

#include "../ESPEasyCore/ESPEasyWifi.h"
#include "../ESPEasyCore/Serial.h"

#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/RTC.h"
#include "../Globals/ResetFactoryDefaultPref.h"
#include "../Globals/SecuritySettings.h"

#include "../Helpers/_CPlugin_Helper.h"
#include "../Helpers/ESPEasyRTC.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/Misc.h"

/********************************************************************************************\
   Reset all settings to factory defaults
 \*********************************************************************************************/
void ResetFactory()
{
  #if FEATURE_CUSTOM_PROVISIONING
  if (ResetFactoryDefaultPreference.getPreference() == 0)
  {
    ResetFactoryDefaultPreference.setDeviceModel(static_cast<DeviceModel>(DEFAULT_FACTORY_DEFAULT_DEVICE_MODEL));
    ResetFactoryDefaultPreference.fetchRulesTXT(0, DEFAULT_PROVISIONING_FETCH_RULES1);
    ResetFactoryDefaultPreference.fetchRulesTXT(1, DEFAULT_PROVISIONING_FETCH_RULES2);
    ResetFactoryDefaultPreference.fetchRulesTXT(2, DEFAULT_PROVISIONING_FETCH_RULES3);
    ResetFactoryDefaultPreference.fetchRulesTXT(3, DEFAULT_PROVISIONING_FETCH_RULES4);
    ResetFactoryDefaultPreference.fetchNotificationDat(DEFAULT_PROVISIONING_FETCH_NOTIFICATIONS);
    ResetFactoryDefaultPreference.fetchSecurityDat(DEFAULT_PROVISIONING_FETCH_SECURITY);
    ResetFactoryDefaultPreference.fetchConfigDat(DEFAULT_PROVISIONING_FETCH_CONFIG);
    ResetFactoryDefaultPreference.fetchProvisioningDat(DEFAULT_PROVISIONING_FETCH_PROVISIONING);
    ResetFactoryDefaultPreference.saveURL(DEFAULT_PROVISIONING_SAVE_URL);
    ResetFactoryDefaultPreference.storeCredentials(DEFAULT_PROVISIONING_SAVE_CREDENTIALS);
    ResetFactoryDefaultPreference.allowFetchByCommand(DEFAULT_PROVISIONING_ALLOW_FETCH_COMMAND);
  }
  #endif


  const GpioFactorySettingsStruct gpio_settings(ResetFactoryDefaultPreference.getDeviceModel());
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("ResetFactory"));
  #endif

  // Direct Serial is allowed here, since this is only an emergency task.
  serialPrint(F("RESET: Resetting factory defaults... using "));
  serialPrint(getDeviceModelString(ResetFactoryDefaultPreference.getDeviceModel()));
  serialPrintln(F(" settings"));
  delay(1000);

  if (readFromRTC())
  {
    serialPrint(F("RESET: Warm boot, reset count: "));
    serialPrintln(String(RTC.factoryResetCounter));

    if (RTC.factoryResetCounter >= 3)
    {
      serialPrintln(F("RESET: Too many resets, protecting your flash memory (powercycle to solve this)"));
      return;
    }
  }
  else
  {
    serialPrintln(F("RESET: Cold boot"));
    initRTC();

    // TODO TD-er: Store set device model in RTC.
  }

  RTC.flashCounter = 0; // reset flashcounter, since we're already counting the number of factory-resets. we dont want to hit a flash-count
                        // limit during reset.
  RTC.factoryResetCounter++;
  saveToRTC();

  // always format on factory reset, in case of corrupt FS
//  ESPEASY_FS.end();
  serialPrintln(F("RESET: formatting..."));
  FS_format();
  serialPrintln(F("RESET: formatting done..."));

  if (!ESPEASY_FS.begin())
  {
    serialPrintln(F("RESET: FORMAT FS FAILED!"));
    return;
  }

#if FEATURE_CUSTOM_PROVISIONING
  {
    MakeProvisioningSettings(ProvisioningSettings);
    if (AllocatedProvisioningSettings()) {
      ProvisioningSettings.setUser(F(DEFAULT_PROVISIONING_USER));
      ProvisioningSettings.setPass(F(DEFAULT_PROVISIONING_PASS));
      ProvisioningSettings.setUrl(F(DEFAULT_PROVISIONING_URL));
      ProvisioningSettings.ResetFactoryDefaultPreference = ResetFactoryDefaultPreference.getPreference();
      saveProvisioningSettings(ProvisioningSettings);
    }
  }
#endif

  // pad files with extra zeros for future extensions
  InitFile(SettingsType::SettingsFileEnum::FILE_CONFIG_type);
  InitFile(SettingsType::SettingsFileEnum::FILE_SECURITY_type);
  #if FEATURE_NOTIFIER
  InitFile(SettingsType::SettingsFileEnum::FILE_NOTIFICATION_type);
  #endif // if FEATURE_NOTIFIER

  InitFile(getRulesFileName(0), 0);

  Settings.clearMisc();

  if (!ResetFactoryDefaultPreference.keepNTP()) {
    Settings.clearTimeSettings();
    Settings.UseNTP(DEFAULT_USE_NTP);
    strcpy_P(Settings.NTPHost, PSTR(DEFAULT_NTP_HOST));
    Settings.TimeZone = DEFAULT_TIME_ZONE;
    Settings.DST      = DEFAULT_USE_DST;
  }

  if (!ResetFactoryDefaultPreference.keepNetwork()) {
    Settings.clearNetworkSettings();

    // TD-er Reset access control
    str2ip(F(DEFAULT_IPRANGE_LOW),  SecuritySettings.AllowedIPrangeLow);
    str2ip(F(DEFAULT_IPRANGE_HIGH), SecuritySettings.AllowedIPrangeHigh);
    SecuritySettings.IPblockLevel = DEFAULT_IP_BLOCK_LEVEL;

    #if DEFAULT_USE_STATIC_IP
    str2ip((char *)DEFAULT_IP,     Settings.IP);
    str2ip((char *)DEFAULT_DNS,    Settings.DNS);
    str2ip((char *)DEFAULT_GW,     Settings.Gateway);
    str2ip((char *)DEFAULT_SUBNET, Settings.Subnet);
    #endif // if DEFAULT_USE_STATIC_IP
    Settings.IncludeHiddenSSID(DEFAULT_WIFI_INCLUDE_HIDDEN_SSID);
  }

  Settings.clearNotifications();
  Settings.clearControllers();
  Settings.clearTasks();

  if (!ResetFactoryDefaultPreference.keepLogSettings()) {
    Settings.clearLogSettings();
    str2ip((char *)DEFAULT_SYSLOG_IP, Settings.Syslog_IP);

    setLogLevelFor(LOG_TO_SYSLOG, DEFAULT_SYSLOG_LEVEL);
    setLogLevelFor(LOG_TO_SERIAL, DEFAULT_SERIAL_LOG_LEVEL);
    setLogLevelFor(LOG_TO_WEBLOG, DEFAULT_WEB_LOG_LEVEL);
    setLogLevelFor(LOG_TO_SDCARD, DEFAULT_SD_LOG_LEVEL);
    Settings.SyslogFacility = DEFAULT_SYSLOG_FACILITY;
    Settings.UseValueLogger = DEFAULT_USE_SD_LOG;
  }

  if (!ResetFactoryDefaultPreference.keepUnitName()) {
    Settings.clearUnitNameSettings();
    Settings.Unit = UNIT;
    strcpy_P(Settings.Name, PSTR(DEFAULT_NAME));
    Settings.UDPPort = DEFAULT_SYNC_UDP_PORT;
  }

  if (!ResetFactoryDefaultPreference.keepWiFi()) {
    strcpy_P(SecuritySettings.WifiSSID,  PSTR(DEFAULT_SSID));
    strcpy_P(SecuritySettings.WifiKey,   PSTR(DEFAULT_KEY));
    strcpy_P(SecuritySettings.WifiSSID2, PSTR(DEFAULT_SSID2));
    strcpy_P(SecuritySettings.WifiKey2,  PSTR(DEFAULT_KEY2));
    strcpy_P(SecuritySettings.WifiAPKey, PSTR(DEFAULT_AP_KEY));
    SecuritySettings.WifiSSID2[0] = 0;
    SecuritySettings.WifiKey2[0]  = 0;
  }
  strcpy_P(SecuritySettings.Password, PSTR(DEFAULT_ADMIN_PASS));

  Settings.ResetFactoryDefaultPreference = ResetFactoryDefaultPreference.getPreference();

  // now we set all parameters that need to be non-zero as default value


  Settings.PID     = ESP_PROJECT_PID;
  Settings.Version = VERSION;
  Settings.Build   = BUILD;

  //  Settings.IP_Octet				 = DEFAULT_IP_OCTET;
  Settings.Delay                   = DEFAULT_DELAY;
  Settings.Pin_i2c_sda             = gpio_settings.i2c_sda;
  Settings.Pin_i2c_scl             = gpio_settings.i2c_scl;
  Settings.Pin_status_led          = gpio_settings.status_led;
  Settings.Pin_status_led_Inversed = DEFAULT_PIN_STATUS_LED_INVERSED;
  Settings.Pin_sd_cs               = -1;
  Settings.Pin_Reset               = DEFAULT_PIN_RESET_BUTTON;
  Settings.Protocol[0]             = DEFAULT_PROTOCOL;
  Settings.deepSleep_wakeTime      = 0; // Sleep disabled
  Settings.CustomCSS               = false;
  Settings.InitSPI                 = DEFAULT_SPI;

  for (taskIndex_t x = 0; x < TASKS_MAX; x++)
  {
    Settings.TaskDevicePin1[x]         = -1;
    Settings.TaskDevicePin2[x]         = -1;
    Settings.TaskDevicePin3[x]         = -1;
    Settings.TaskDevicePin1PullUp[x]   = true;
    Settings.TaskDevicePin1Inversed[x] = false;

    for (controllerIndex_t y = 0; y < CONTROLLER_MAX; y++) {
      Settings.TaskDeviceSendData[y][x] = true;
    }
    Settings.TaskDeviceTimer[x] = Settings.Delay;
  }

  // advanced Settings
  Settings.UseRules                         = DEFAULT_USE_RULES;
  Settings.ControllerEnabled[0]             = DEFAULT_CONTROLLER_ENABLED;
  Settings.MQTTRetainFlag_unused            = DEFAULT_MQTT_RETAIN;
  Settings.MessageDelay_unused              = DEFAULT_MQTT_DELAY;
  Settings.MQTTUseUnitNameAsClientId_unused = DEFAULT_MQTT_USE_UNITNAME_AS_CLIENTID;

  // allow to set default latitude and longitude
  #ifdef DEFAULT_LATITUDE
  Settings.Latitude = DEFAULT_LATITUDE;
  #endif // ifdef DEFAULT_LATITUDE
  #ifdef DEFAULT_LONGITUDE
  Settings.Longitude = DEFAULT_LONGITUDE;
  #endif // ifdef DEFAULT_LONGITUDE

  Settings.UseSerial = DEFAULT_USE_SERIAL;
  Settings.BaudRate  = DEFAULT_SERIAL_BAUD;

  Settings.ETH_Phy_Addr   = gpio_settings.eth_phyaddr;
  Settings.ETH_Pin_mdc    = gpio_settings.eth_mdc;
  Settings.ETH_Pin_mdio   = gpio_settings.eth_mdio;
  Settings.ETH_Pin_power  = gpio_settings.eth_power;
  Settings.ETH_Phy_Type   = gpio_settings.eth_phytype;
  Settings.ETH_Clock_Mode = gpio_settings.eth_clock_mode;
  Settings.NetworkMedium  = gpio_settings.network_medium;

  /*
          Settings.GlobalSync						= DEFAULT_USE_GLOBAL_SYNC;

          Settings.IP_Octet						= DEFAULT_IP_OCTET;
          Settings.WDI2CAddress					= DEFAULT_WD_IC2_ADDRESS;
          Settings.UseSSDP						= DEFAULT_USE_SSDP;
          Settings.ConnectionFailuresThreshold	= DEFAULT_CON_FAIL_THRES;
          Settings.WireClockStretchLimit			= DEFAULT_I2C_CLOCK_LIMIT;
   */
  Settings.I2C_clockSpeed = DEFAULT_I2C_CLOCK_SPEED;

  Settings.JSONBoolWithoutQuotes(DEFAULT_JSON_BOOL_WITHOUT_QUOTES);
  Settings.EnableTimingStats(DEFAULT_ENABLE_TIMING_STATS);

#ifdef PLUGIN_DESCR
  strcpy_P(Settings.Name, PSTR(PLUGIN_DESCR));
#endif // ifdef PLUGIN_DESCR

#ifndef LIMIT_BUILD_SIZE
  addPredefinedPlugins(gpio_settings);
  addPredefinedRules(gpio_settings);
#endif

#if DEFAULT_CONTROLLER
  {
    // Place in a scope to have its memory freed ASAP
    MakeControllerSettings(ControllerSettings); //-V522

    if (AllocatedControllerSettings()) {
      safe_strncpy(ControllerSettings.Subscribe,            F(DEFAULT_SUB),            sizeof(ControllerSettings.Subscribe));
      safe_strncpy(ControllerSettings.Publish,              F(DEFAULT_PUB),            sizeof(ControllerSettings.Publish));
      safe_strncpy(ControllerSettings.MQTTLwtTopic,         F(DEFAULT_MQTT_LWT_TOPIC), sizeof(ControllerSettings.MQTTLwtTopic));
      safe_strncpy(ControllerSettings.LWTMessageConnect,    F(DEFAULT_MQTT_LWT_CONNECT_MESSAGE),
                   sizeof(ControllerSettings.LWTMessageConnect));
      safe_strncpy(ControllerSettings.LWTMessageDisconnect, F(DEFAULT_MQTT_LWT_DISCONNECT_MESSAGE),
                   sizeof(ControllerSettings.LWTMessageDisconnect));
      str2ip((char *)DEFAULT_SERVER, ControllerSettings.IP);
      ControllerSettings.setHostname(F(DEFAULT_SERVER_HOST));
      ControllerSettings.UseDNS = DEFAULT_SERVER_USEDNS;
      ControllerSettings.useExtendedCredentials(DEFAULT_USE_EXTD_CONTROLLER_CREDENTIALS);
      ControllerSettings.Port = DEFAULT_PORT;
      setControllerUser(0, ControllerSettings, F(DEFAULT_CONTROLLER_USER));
      setControllerPass(0, ControllerSettings, F(DEFAULT_CONTROLLER_PASS));

      SaveControllerSettings(0, ControllerSettings);
    }
  }
#endif // if DEFAULT_CONTROLLER

  SaveSettings();
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("ResetFactory2"));
  #endif
  serialPrintln(F("RESET: Successful, rebooting. (you might need to press the reset button if you've just flashed the firmware)"));

  // NOTE: this is a known ESP8266 bug, not our fault. :)
  delay(1000);
  WiFi.persistent(true);  // use SDK storage of SSID/WPA parameters
  WiFiEventData.intent_to_reboot = true;
  WifiDisconnect();       // this will store empty ssid/wpa into sdk storage
  WiFi.persistent(false); // Do not use SDK storage of SSID/WPA parameters
  reboot(ESPEasy_Scheduler::IntendedRebootReason_e::ResetFactory);
}


/*********************************************************************************************\
   Collect the stored preference for factory default
\*********************************************************************************************/
void applyFactoryDefaultPref() {
  // TODO TD-er: Store it in more places to make it more persistent
  Settings.ResetFactoryDefaultPreference = ResetFactoryDefaultPreference.getPreference();
}

#include "../Helpers/Memory.h"


#ifdef ESP8266
extern "C" {
#include <user_interface.h>
}
#endif

#include "../../ESPEasy_common.h"

/*********************************************************************************************\
   Memory management
\*********************************************************************************************/


// For keeping track of 'cont' stack
// See: https://github.com/esp8266/Arduino/issues/2557
//      https://github.com/esp8266/Arduino/issues/5148#issuecomment-424329183
//      https://github.com/letscontrolit/ESPEasy/issues/1824
#ifdef ESP32

// FIXME TD-er: For ESP32 you need to provide the task number, or nullptr to get from the calling task.
uint32_t getCurrentFreeStack() {
  register uint8_t *sp asm ("a1");

  return sp - pxTaskGetStackStart(nullptr);
}

uint32_t getFreeStackWatermark() {
  return uxTaskGetStackHighWaterMark(nullptr);
}

#else // ifdef ESP32

uint32_t getCurrentFreeStack() {
  // https://github.com/esp8266/Arduino/issues/2557
  register uint32_t *sp asm ("a1");

  return 4 * (sp - g_pcont->stack);
}

uint32_t getFreeStackWatermark() {
  return cont_get_free_stack(g_pcont);
}

bool allocatedOnStack(const void *address) {
  register uint32_t *sp asm ("a1");

  if (sp < address) { return false; }
  return g_pcont->stack < address;
}

#endif // ESP32


/********************************************************************************************\
   Get free system mem
 \*********************************************************************************************/
unsigned long FreeMem()
{
  #if defined(ESP8266)
  return system_get_free_heap_size();
  #endif // if defined(ESP8266)
  #if defined(ESP32)
  return ESP.getFreeHeap();
  #endif // if defined(ESP32)
}

#ifdef USE_SECOND_HEAP
unsigned long FreeMem2ndHeap()
{
  HeapSelectIram ephemeral;
  return ESP.getFreeHeap();
}
#endif


unsigned long getMaxFreeBlock()
{
  const unsigned long freemem = FreeMem();
  // computing max free block is a rather extensive operation, so only perform when free memory is already low.
  if (freemem < 6144) {
  #if  defined(ESP32)
    return ESP.getMaxAllocHeap();
  #endif // if  defined(ESP32)
  #ifdef CORE_POST_2_5_0
    return ESP.getMaxFreeBlockSize();
  #endif // ifdef CORE_POST_2_5_0
  }
  return freemem;
}

#include "../Helpers/Convert.h"

/*********************************************************************************************\
   Convert bearing in degree to bearing string
\*********************************************************************************************/
const __FlashStringHelper * getBearing(int degrees)
{
  const int nr_directions = 16;
  float stepsize      = (360.0f / nr_directions);

  if (degrees < 0) { degrees += 360; } // Allow for bearing -360 .. 359
  int bearing_idx = int((degrees + (stepsize / 2.0f)) / stepsize) % nr_directions;

  if (bearing_idx >= 0) {
    switch (bearing_idx) {
      case 0: return F("N");
      case 1: return F("NNE");
      case 2: return F("NE");
      case 3: return F("ENE");
      case 4: return F("E");
      case 5: return F("ESE");
      case 6: return F("SE");
      case 7: return F("SSE");
      case 8: return F("S");
      case 9: return F("SSW");
      case 10: return F("SW");
      case 11: return F("WSW");
      case 12: return F("W");
      case 13: return F("WNW");
      case 14: return F("NW");
      case 15: return F("NNW");
    }
  }
  return F("");
}

float CelsiusToFahrenheit(float celsius) {
  return celsius * (9.0f / 5.0f) + 32;
}

int m_secToBeaufort(float m_per_sec) {
  if (m_per_sec < 0.3f) { return 0; }

  if (m_per_sec < 1.6f) { return 1; }

  if (m_per_sec < 3.4f) { return 2; }

  if (m_per_sec < 5.5f) { return 3; }

  if (m_per_sec < 8.0f) { return 4; }

  if (m_per_sec < 10.8f) { return 5; }

  if (m_per_sec < 13.9f) { return 6; }

  if (m_per_sec < 17.2f) { return 7; }

  if (m_per_sec < 20.8f) { return 8; }

  if (m_per_sec < 24.5f) { return 9; }

  if (m_per_sec < 28.5f) { return 10; }

  if (m_per_sec < 32.6f) { return 11; }
  return 12;
}

String centimeterToImperialLength(float cm) {
  return millimeterToImperialLength(cm * 10.0f);
}

String millimeterToImperialLength(float mm) {
  float inches = mm / 25.4f;
  int   feet   = inches / 12.0f;

  inches = inches - (feet * 12);
  String result;
  result.reserve(10);

  if (feet != 0) {
    result += feet;
    result += '\'';
  }
  result += toString(inches, 1);
  result += '"';
  return result;
}

float minutesToDay(int minutes) {
  return minutes / 1440.0f;
}

String minutesToDayHour(int minutes) {
  int  days  = minutes / 1440;
  int  hours = (minutes % 1440) / 60;
  char TimeString[8] = {0}; // 5 digits plus the null char minimum

  sprintf_P(TimeString, PSTR("%d%c%02d%c"), days, 'd', hours, 'h');
  return TimeString;
}

String minutesToHourMinute(int minutes) {
  int  hours = (minutes % 1440) / 60;
  int  mins  = (minutes % 1440) % 60;
  char TimeString[20] = {0};

  sprintf_P(TimeString, PSTR("%d%c%02d%c"), hours, 'h', mins, 'm');
  return TimeString;
}

String minutesToDayHourMinute(int minutes) {
  int  days  = minutes / 1440;
  int  hours = (minutes % 1440) / 60;
  int  mins  = (minutes % 1440) % 60;
  char TimeString[20] = {0};

  sprintf_P(TimeString, PSTR("%d%c%02d%c%02d%c"), days, 'd', hours, 'h', mins, 'm');
  return TimeString;
}

String secondsToDayHourMinuteSecond(int seconds) {
  int  sec     = seconds % 60;
  int  minutes = seconds / 60;
  int  days    = minutes / 1440;
  int  hours   = (minutes % 1440) / 60;
  int  mins    = (minutes % 1440) % 60;
  char TimeString[20] = {0};

  sprintf_P(TimeString, PSTR("%d%c%02d%c%02d%c%02d"), days, 'd', hours, ':', mins, ':', sec);
  return TimeString;
}

String format_msec_duration(int64_t duration) {
  String result;

  if (duration < 0) {
    result   = "-";
    duration = -1ll * duration;
  }

  if (duration < 10000ll) {
    result += static_cast<int32_t>(duration);
    result += F(" ms");
    return result;
  }
  duration /= 1000ll;

  if (duration < 3600ll) {
    int sec     = duration % 60ll;
    int minutes = duration / 60ll;

    if (minutes > 0ll) {
      result += minutes;
      result += F(" m ");
    }
    result += sec;
    result += F(" s");
    return result;
  }
  duration /= 60ll;

  if (duration < 1440ll) { return minutesToHourMinute(duration); }
  return minutesToDayHourMinute(duration);
}


// Compute the dew point temperature, given temperature and humidity (temp in Celsius)
// Formula: http://www.ajdesigner.com/phphumidity/dewpoint_equation_dewpoint_temperature.php
// Td = (f/100)^(1/8) * (112 + 0.9*T) + 0.1*T - 112
float compute_dew_point_temp(float temperature, float humidity_percentage) {
  return powf(humidity_percentage / 100.0f, 0.125f) *
         (112.0f + 0.9f*temperature) + 0.1f*temperature - 112.0f;
}

// Compute the humidity given temperature and dew point temperature (temp in Celsius)
// Formula: http://www.ajdesigner.com/phphumidity/dewpoint_equation_relative_humidity.php
// f = 100 * ((112 - 0.1*T + Td) / (112 + 0.9 * T))^8
float compute_humidity_from_dewpoint(float temperature, float dew_temperature) {
  return 100.0f * powf((112.0f - 0.1f * temperature + dew_temperature) /
                     (112.0f + 0.9f * temperature), 8);
}



/********************************************************************************************\
   Compensate air pressure for given altitude (in meters)
 \*********************************************************************************************/
float pressureElevation(float atmospheric, float altitude) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  return atmospheric / powf(1.0f - (altitude / 44330.0f), 5.255f);
}

float altitudeFromPressure(float atmospheric, float seaLevel)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  return 44330.0f * (1.0f - powf(atmospheric / seaLevel, 0.1903f));
}




/********************************************************************************************\
   In memory convert float to long
 \*********************************************************************************************/
unsigned long float2ul(float f)
{
  unsigned long ul;

  memcpy(&ul, &f, 4);
  return ul;
}

/********************************************************************************************\
   In memory convert long to float
 \*********************************************************************************************/
float ul2float(unsigned long ul)
{
  float f;

  memcpy(&f, &ul, 4);
  return f;
}

/*********************************************************************************************\
   Workaround for removing trailing white space when String() converts a float with 0 decimals
\*********************************************************************************************/
String toString(const float& value, unsigned int decimalPlaces)
{
  // This has been fixed in ESP32 code, not (yet) in ESP8266 code
  // https://github.com/espressif/arduino-esp32/pull/6138/files
//  #ifdef ESP8266
  char *buf = (char*)malloc(decimalPlaces + 42);
  if (nullptr == buf) {
    return F("nan");
  }
  String sValue(dtostrf(value, (decimalPlaces + 2), decimalPlaces, buf));
  free(buf);
//  #else
//  String sValue = String(value, decimalPlaces);
//  #endif

  sValue.trim();
  return sValue;
}

String doubleToString(const double& value, unsigned int decimalPlaces, bool trimTrailingZeros) {
  // This has been fixed in ESP32 code, not (yet) in ESP8266 code
  // https://github.com/espressif/arduino-esp32/pull/6138/files
//  #ifdef ESP8266
  unsigned int expectedChars = decimalPlaces + 4; // 1 dot, 2 minus signs and terminating zero
  if (value > 1e32 || value < -1e32) {
    expectedChars += 308; // Just assume the worst
  } else {
    expectedChars += 33;
  }
  char *buf = (char*)malloc(expectedChars);

  if (nullptr == buf) {
    return F("nan");
  }
  String res(dtostrf(value, (decimalPlaces + 2), decimalPlaces, buf));
  free(buf);

//  #else
//  String res(value, decimalPlaces);
//  #endif
  res.trim();

  if (trimTrailingZeros) {
    int dot_pos = res.lastIndexOf('.');
    if (dot_pos != -1) {
      bool someTrimmed = false;
      for (int i = res.length()-1; i > dot_pos && res[i] == '0'; --i) {
        someTrimmed = true;
        res[i] = ' ';
      }
      if (someTrimmed) {
        res.trim();
      }
      if (res.endsWith(F("."))) {
        res[dot_pos] = ' ';
        res.trim();
      }
    }
  }
  return res;
}

#include "../Helpers/Rules_calculate.h"

#include <Arduino.h>

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/RamTracker.h"
#include "../Helpers/ESPEasy_math.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/StringConverter.h"


RulesCalculate_t::RulesCalculate_t() {
  for (int i = 0; i < STACK_SIZE; ++i) {
    globalstack[i] = 0.0;
  }
}

/********************************************************************************************\
   Instance of the RulesCalculate to perform calculations
   These functions are wrapped in a class to
    - make it more clear what external functions to use
    - Make sure generic function names will not cause conflicts
    - Prevent external access to calculate only variables.
 \*********************************************************************************************/
RulesCalculate_t RulesCalculate;


/********************************************************************************************\
   Calculate function for simple expressions
 \*********************************************************************************************/
bool isError(CalculateReturnCode returnCode) {
  return returnCode != CalculateReturnCode::OK;
}

bool RulesCalculate_t::is_number(char oc, char c)
{
  // Check if it matches part of a number (identifier)
  return
    (c == '.')   ||                                // A decimal point of a floating point number.
    ((oc == '0') && ((c == 'x') || (c == 'b'))) || // HEX (0x) or BIN (0b) prefixes.
    isxdigit(c)  ||                                // HEX digit also includes normal decimal numbers
    (is_operator(oc) && (c == '-'))                // Beginning of a negative number after an operator.
  ;
}

bool RulesCalculate_t::is_operator(char c)
{
  return c == '+' || c == '-' || c == '*' || c == '/' || c == '^' || c == '%';
}

bool RulesCalculate_t::is_unary_operator(char c)
{
  const UnaryOperator op = static_cast<UnaryOperator>(c);
  return (op == UnaryOperator::Not || (
          c >= static_cast<char>(UnaryOperator::Log) &&
          c <= static_cast<char>(UnaryOperator::ArcTan_d)));
/*
  switch (op) {
    case UnaryOperator::Not:
    case UnaryOperator::Log:
    case UnaryOperator::Ln:
    case UnaryOperator::Abs:
    case UnaryOperator::Exp:
    case UnaryOperator::Sqrt:
    case UnaryOperator::Sq:
    case UnaryOperator::Round:
    case UnaryOperator::Sin:
    case UnaryOperator::Cos:
    case UnaryOperator::Tan:
    case UnaryOperator::ArcSin:
    case UnaryOperator::ArcCos:
    case UnaryOperator::ArcTan:
    case UnaryOperator::Sin_d:
    case UnaryOperator::Cos_d:
    case UnaryOperator::Tan_d:
    case UnaryOperator::ArcSin_d:
    case UnaryOperator::ArcCos_d:
    case UnaryOperator::ArcTan_d:
      return true;
  }
  return false;
  */
}

CalculateReturnCode RulesCalculate_t::push(double value)
{
  if (sp != sp_max) // Full
  {
    *(++sp) = value;
    return CalculateReturnCode::OK;
  }
  return CalculateReturnCode::ERROR_STACK_OVERFLOW;
}

double RulesCalculate_t::pop()
{
  if (sp != (globalstack - 1)) { // empty
    return *(sp--);
  }
  else {
    return 0.0;
  }
}

double RulesCalculate_t::apply_operator(char op, double first, double second)
{
  switch (op)
  {
    case '+':
      return first + second;
    case '-':
      return first - second;
    case '*':
      return first * second;
    case '/':
      return first / second;
    case '%':
      return static_cast<int>(round(first)) % static_cast<int>(round(second));
    case '^':
      return pow(first, second);
    default:
      return 0.0;
  }
}

double RulesCalculate_t::apply_unary_operator(char op, double first)
{
  double ret                = 0.0;
  const UnaryOperator un_op = static_cast<UnaryOperator>(op);

  switch (un_op) {
    case UnaryOperator::Not:
      return (approximatelyEqual(round(first), 0)) ? 1 : 0;
    case UnaryOperator::Log:
      return log10(first);
    case UnaryOperator::Ln:
      return log(first);
    case UnaryOperator::Abs:
      return fabs(first);
    case UnaryOperator::Exp:
      return exp(first);
    case UnaryOperator::Sqrt:
      return sqrt(first);
    case UnaryOperator::Sq:
      return first * first;
    case UnaryOperator::Round:
      return round(first);
    default:
      break;
  }

#if FEATURE_TRIGONOMETRIC_FUNCTIONS_RULES
  const bool useDegree = angleDegree(un_op);

  // First the trigonometric functions with angle as output
  switch (un_op) {
    case UnaryOperator::ArcSin:
    case UnaryOperator::ArcSin_d:
      ret = asin(first);
      return useDegree ? degrees(ret) : ret;
    case UnaryOperator::ArcCos:
    case UnaryOperator::ArcCos_d:
      ret = acos(first);
      return useDegree ? degrees(ret) : ret;
    case UnaryOperator::ArcTan:
    case UnaryOperator::ArcTan_d:
      ret = atan(first);
      return useDegree ? degrees(ret) : ret;
    default:
      break;
  }

  // Now the trigonometric functions with angle as input
  if (useDegree) {
    first = radians(first);
  }

  switch (un_op) {
    case UnaryOperator::Sin:
    case UnaryOperator::Sin_d:
      return sin(first);
    case UnaryOperator::Cos:
    case UnaryOperator::Cos_d:
      return cos(first);
    case UnaryOperator::Tan:
    case UnaryOperator::Tan_d:
      return tan(first);
    default:
      break;
  }
#else // if FEATURE_TRIGONOMETRIC_FUNCTIONS_RULES

  switch (un_op) {
    case UnaryOperator::Sin:
    case UnaryOperator::Sin_d:
    case UnaryOperator::Cos:
    case UnaryOperator::Cos_d:
    case UnaryOperator::Tan:
    case UnaryOperator::Tan_d:
    case UnaryOperator::ArcSin:
    case UnaryOperator::ArcSin_d:
    case UnaryOperator::ArcCos:
    case UnaryOperator::ArcCos_d:
    case UnaryOperator::ArcTan:
    case UnaryOperator::ArcTan_d:
      addLog(LOG_LEVEL_ERROR, F("FEATURE_TRIGONOMETRIC_FUNCTIONS_RULES not defined in build"));
      break;
    default:
      break;
  }
#endif // if FEATURE_TRIGONOMETRIC_FUNCTIONS_RULES
  return ret;
}

/*
   char * RulesCalculate_t::next_token(char *linep)
   {
   while (isspace(*(linep++))) {}

   while (*linep && !isspace(*(linep++))) {}
   return linep;
   }
 */
CalculateReturnCode RulesCalculate_t::RPNCalculate(char *token)
{
  CalculateReturnCode ret = CalculateReturnCode::OK;

  if (token[0] == 0) {
    return ret; // Don't bother for an empty string
  }

  if (is_operator(token[0]) && (token[1] == 0))
  {
    double second = pop();
    double first  = pop();

    ret = push(apply_operator(token[0], first, second));

// FIXME TD-er: Regardless whether it is an error, all code paths return ret;
//    if (isError(ret)) { return ret; }
  } else if (is_unary_operator(token[0]) && (token[1] == 0))
  {
    double first = pop();

    ret = push(apply_unary_operator(token[0], first));

// FIXME TD-er: Regardless whether it is an error, all code paths return ret;
//    if (isError(ret)) { return ret; }
  } else {
    // Fetch next if there is any
    double value = 0.0;
    validDoubleFromString(token, value);

    ret = push(value); // If it is a value, push to the stack

// FIXME TD-er: Regardless whether it is an error, all code paths return ret;
//    if (isError(ret)) { return ret; }
  }

  return ret;
}

// operators
// precedence   operators         associativity
// 4            !                 right to left
// 3            ^                 left to right
// 2            * / %             left to right
// 1            + -               left to right
int RulesCalculate_t::op_preced(const char c)
{
  if (is_unary_operator(c)) { return 4; // right to left
  }

  switch (c)
  {
    case '^':
      return 3;
    case '*':
    case '/':
    case '%':
      return 2;
    case '+':
    case '-':
      return 1;
  }
  return 0;
}

bool RulesCalculate_t::op_left_assoc(const char c)
{
  if (is_operator(c)) { return true;        // left to right
  }
/*
  // FIXME TD-er: Disabled the check as the return value is false anyway.
  if (is_unary_operator(c)) { return false; // right to left
  }
  */
  return false;
}

unsigned int RulesCalculate_t::op_arg_count(const char c)
{
  if (is_unary_operator(c)) { return 1; }

  if (is_operator(c)) { return 2; }
  return 0;
}

CalculateReturnCode RulesCalculate_t::doCalculate(const char *input, double *result)
{
  #define TOKEN_LENGTH 25
  #define OPERATOR_STACK_SIZE 32
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("Calculate"));
  #endif // ifndef BUILD_NO_RAM_TRACKER
  const char *strpos = input, *strend = input + strlen(input);
  char token[TOKEN_LENGTH];
  char c, oc, *TokenPos = token;
  char stack[OPERATOR_STACK_SIZE]; // operator stack
  unsigned int sl = 0;             // stack length
  char sc;                         // used for record stack element
  CalculateReturnCode error = CalculateReturnCode::OK;

  // *sp=0; // bug, it stops calculating after 50 times
  sp = globalstack - 1;
  oc = c = 0;

  if (input[0] == '=') {
    ++strpos;

    if (strpos < strend) {
      c = *strpos;
    }
  }

  while (strpos < strend)
  {
    if ((TokenPos - &token[0]) >= (TOKEN_LENGTH - 1)) { return CalculateReturnCode::ERROR_TOKEN_LENGTH_EXCEEDED; }

    // read one token from the input stream
    oc = c;
    c  = *strpos;

    if (c != ' ')
    {
      // If the token is a number (identifier), then add it to the token queue.
      if (is_number(oc, c))
      {
        *TokenPos = c;
        ++TokenPos;
      }

      // If the token is an operator, op1, then:
      else if (is_operator(c) || is_unary_operator(c))
      {
        *(TokenPos) = 0; // Mark end of token string
        error       = RPNCalculate(token);
        TokenPos    = token;

        if (isError(error)) { return error; }

        while (sl > 0 && sl < (OPERATOR_STACK_SIZE - 1))
        {
          sc = stack[sl - 1];

          // While there is an operator token, op2, at the top of the stack
          // op1 is left-associative and its precedence is less than or equal to that of op2,
          // or op1 has precedence less than that of op2,
          // The differing operator priority decides pop / push
          // If 2 operators have equal priority then associativity decides.
          if (is_operator(sc) &&
              (
                (op_left_assoc(c) && (op_preced(c) <= op_preced(sc))) ||
                (op_preced(c) < op_preced(sc))
              )
              )
          {
            // Pop op2 off the stack, onto the token queue;
            *TokenPos = sc;
            ++TokenPos;
            *(TokenPos) = 0; // Mark end of token string
            error       = RPNCalculate(token);
            TokenPos    = token;

            if (isError(error)) { return error; }
            sl--;
          }
          else {
            break;
          }
        }

        // push op1 onto the stack.
        stack[sl] = c;
        ++sl;
      }

      // If the token is a left parenthesis, then push it onto the stack.
      else if (c == '(')
      {
        if (sl >= OPERATOR_STACK_SIZE) { return CalculateReturnCode::ERROR_STACK_OVERFLOW; }
        stack[sl] = c;
        ++sl;
      }

      // If the token is a right parenthesis:
      else if (c == ')')
      {
        bool pe = false;

        // Until the token at the top of the stack is a left parenthesis,
        // pop operators off the stack onto the token queue
        while (sl > 0)
        {
          *(TokenPos) = 0; // Mark end of token string
          error       = RPNCalculate(token);
          TokenPos    = token;

          if (isError(error)) { return error; }

          if (sl > OPERATOR_STACK_SIZE) { return CalculateReturnCode::ERROR_STACK_OVERFLOW; }
          sc = stack[sl - 1];

          if (sc == '(')
          {
            pe = true;
            break;
          }
          else
          {
            *TokenPos = sc;
            ++TokenPos;
            sl--;
          }
        }

        // If the stack runs out without finding a left parenthesis, then there are mismatched parentheses.
        if (!pe) {
          return CalculateReturnCode::ERROR_PARENTHESES_MISMATCHED;
        }

        // Pop the left parenthesis from the stack, but not onto the token queue.
        sl--;

        // If the token at the top of the stack is a function token, pop it onto the token queue.
        // FIXME TD-er: This sc value is never used, it is re-assigned a new value before it is being checked.
        if ((sl > 0) && (sl < OPERATOR_STACK_SIZE)) {
          sc = stack[sl - 1];
        }
      }
      else {
        return CalculateReturnCode::ERROR_UNKNOWN_TOKEN;
      }
    }
    ++strpos;
  }

  // When there are no more tokens to read:
  // While there are still operator tokens in the stack:
  while (sl > 0)
  {
    sc = stack[sl - 1];

    if ((sc == '(') || (sc == ')')) {
      return CalculateReturnCode::ERROR_PARENTHESES_MISMATCHED;
    }

    *(TokenPos) = 0; // Mark end of token string
    error       = RPNCalculate(token);
    TokenPos    = token;

    if (isError(error)) { return error; }
    *TokenPos = sc;
    ++TokenPos;
    --sl;
  }

  *(TokenPos) = 0; // Mark end of token string
  error       = RPNCalculate(token);
  TokenPos    = token;

  if (isError(error))
  {
    *result = 0;
    return error;
  }
  *result = *sp;
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("Calculate2"));
  #endif // ifndef BUILD_NO_RAM_TRACKER
  return CalculateReturnCode::OK;
}

void preProcessReplace(String& input, UnaryOperator op) {
  String find = toString(op);

  if (find.isEmpty()) { return; }
  find += '('; // Add opening parenthesis.

  const String replace = String(static_cast<char>(op)) + '(';

  input.replace(find, replace);
}

bool angleDegree(UnaryOperator op)
{
  switch (op) {
    case UnaryOperator::Sin_d:
    case UnaryOperator::Cos_d:
    case UnaryOperator::Tan_d:
    case UnaryOperator::ArcSin_d:
    case UnaryOperator::ArcCos_d:
    case UnaryOperator::ArcTan_d:
      return true;
    default:
      break;
  }
  return false;
}

const __FlashStringHelper* toString(UnaryOperator op)
{
  switch (op) {
    case UnaryOperator::Not:
      break; // No need to replace
    case UnaryOperator::Log:
      return F("log");
    case UnaryOperator::Ln:
      return F("ln");
    case UnaryOperator::Abs:
      return F("abs");
    case UnaryOperator::Exp:
      return F("exp");
    case UnaryOperator::Sqrt:
      return F("sqrt");
    case UnaryOperator::Sq:
      return F("sq");
    case UnaryOperator::Round:
      return F("round");
    case UnaryOperator::Sin:
      return F("sin");
    case UnaryOperator::Sin_d:
      return F("sin_d");
    case UnaryOperator::Cos:
      return F("cos");
    case UnaryOperator::Cos_d:
      return F("cos_d");
    case UnaryOperator::Tan:
      return F("tan");
    case UnaryOperator::Tan_d:
      return F("tan_d");
    case UnaryOperator::ArcSin:
      return F("asin");
    case UnaryOperator::ArcSin_d:
      return F("asin_d");
    case UnaryOperator::ArcCos:
      return F("acos");
    case UnaryOperator::ArcCos_d:
      return F("acos_d");
    case UnaryOperator::ArcTan:
      return F("atan");
    case UnaryOperator::ArcTan_d:
      return F("atan_d");
  }
  return F("");
}

String RulesCalculate_t::preProces(const String& input)
{
  String preprocessed = input;

  preProcessReplace(preprocessed, UnaryOperator::Not);
  preProcessReplace(preprocessed, UnaryOperator::Log);
  preProcessReplace(preprocessed, UnaryOperator::Ln);
  preProcessReplace(preprocessed, UnaryOperator::Abs);
  preProcessReplace(preprocessed, UnaryOperator::Exp);
  preProcessReplace(preprocessed, UnaryOperator::Sqrt);
  preProcessReplace(preprocessed, UnaryOperator::Sq);
  preProcessReplace(preprocessed, UnaryOperator::Round);
#if FEATURE_TRIGONOMETRIC_FUNCTIONS_RULES

  // Try the "arc" functions first, or else "sin" is already replaced when "asin" is tried.
  if (preprocessed.indexOf(F("sin")) != -1) {
    preProcessReplace(preprocessed, UnaryOperator::ArcSin);
    preProcessReplace(preprocessed, UnaryOperator::ArcSin_d);
    preProcessReplace(preprocessed, UnaryOperator::Sin);
    preProcessReplace(preprocessed, UnaryOperator::Sin_d);
  }

  if (preprocessed.indexOf(F("cos")) != -1) {
    preProcessReplace(preprocessed, UnaryOperator::ArcCos);
    preProcessReplace(preprocessed, UnaryOperator::ArcCos_d);
    preProcessReplace(preprocessed, UnaryOperator::Cos);
    preProcessReplace(preprocessed, UnaryOperator::Cos_d);
  }

  if (preprocessed.indexOf(F("tan")) != -1) {
    preProcessReplace(preprocessed, UnaryOperator::ArcTan);
    preProcessReplace(preprocessed, UnaryOperator::ArcTan_d);
    preProcessReplace(preprocessed, UnaryOperator::Tan);
    preProcessReplace(preprocessed, UnaryOperator::Tan_d);
  }
#endif // if FEATURE_TRIGONOMETRIC_FUNCTIONS_RULES
  return preprocessed;
}

/*******************************************************************************************
* Helper functions to actually interact with the rules calculation functions.
* *****************************************************************************************/
int CalculateParam(const String& TmpStr) {
  int returnValue;

  // Minimize calls to the Calulate function.
  // Only if TmpStr starts with '=' then call Calculate(). Otherwise do not call it
  if (TmpStr[0] != '=') {
    validIntFromString(TmpStr, returnValue);
  } else {
    double param = 0;

    // Starts with an '=', so Calculate starting at next position
    CalculateReturnCode returnCode = Calculate(TmpStr.substring(1), param);

    if (!isError(returnCode)) {
#ifndef BUILD_NO_DEBUG

      if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
        String log = F("CALCULATE PARAM: ");
        log += TmpStr;
        log += F(" = ");
        log += round(param);
        addLogMove(LOG_LEVEL_DEBUG, log);
      }
#endif // ifndef BUILD_NO_DEBUG
    }
    returnValue = round(param); // return integer only as it's valid only for device and task id
  }
  return returnValue;
}

CalculateReturnCode Calculate(const String& input,
                              double      & result)
{
  CalculateReturnCode returnCode = RulesCalculate.doCalculate(
    RulesCalculate_t::preProces(input).c_str(),
    &result);

  if (isError(returnCode)) {
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log = F("Calculate: ");

      switch (returnCode) {
        case CalculateReturnCode::ERROR_STACK_OVERFLOW:
          log += F("Stack Overflow");
          break;
        case CalculateReturnCode::ERROR_BAD_OPERATOR:
          log += F("Bad Operator");
          break;
        case CalculateReturnCode::ERROR_PARENTHESES_MISMATCHED:
          log += F("Parenthesis mismatch");
          break;
        case CalculateReturnCode::ERROR_UNKNOWN_TOKEN:
          log += F("Unknown token");
          break;
        case CalculateReturnCode::ERROR_TOKEN_LENGTH_EXCEEDED:
          log += String(F("Exceeded token length (")) + TOKEN_LENGTH + ')';
          break;
        case CalculateReturnCode::OK:
          // Already handled, but need to have all cases here so the compiler can warn if we're missing one.
          break;
      }

      #ifndef BUILD_NO_DEBUG
      log += F(" input: ");
      log += input;
      log += F(" = ");

      const bool trimTrailingZeros = true;
      log += doubleToString(result, 6, trimTrailingZeros);
      #endif // ifndef BUILD_NO_DEBUG

      addLogMove(LOG_LEVEL_ERROR, log);
    }
  }
  return returnCode;
}

#include "../Helpers/Misc.h"

#include "../../ESPEasy-Globals.h"
#include "../../ESPEasy_common.h"
#include "../../_Plugin_Helper.h"
#include "../ESPEasyCore/ESPEasy_backgroundtasks.h"
#include "../ESPEasyCore/Serial.h"
#include "../Globals/ESPEasy_time.h"
#include "../Globals/Statistics.h"
#include "../Helpers/ESPEasy_FactoryDefault.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/PeriodicalActions.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringParser.h"

#if FEATURE_SD
#include <SD.h>
#endif


bool remoteConfig(struct EventStruct *event, const String& string)
{
  // FIXME TD-er: Why have an event here as argument? It is not used.
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("remoteConfig"));
  #endif // ifndef BUILD_NO_RAM_TRACKER
  bool   success = false;
  String command = parseString(string, 1);

  if (command == F("config"))
  {
    // Command: "config,task,<taskname>,<actual Set Config command>"
    if (parseString(string, 2) == F("task"))
    {
      String configTaskName = parseStringKeepCase(string, 3);

      // FIXME TD-er: This command is not using the tolerance setting
      // tolerantParseStringKeepCase(Line, 4);
      String configCommand = parseStringToEndKeepCase(string, 4);

      if ((configTaskName.isEmpty()) || (configCommand.isEmpty())) {
        return success;
      }
      taskIndex_t index = findTaskIndexByName(configTaskName);

      if (validTaskIndex(index))
      {
        event->setTaskIndex(index);
        success = PluginCall(PLUGIN_SET_CONFIG, event, configCommand);
      }
    } else {
      addLog(LOG_LEVEL_ERROR, F("Expected syntax: config,task,<taskname>,<config command>"));
    }
  }
  return success;
}

/********************************************************************************************\
   delay in milliseconds with background processing
 \*********************************************************************************************/
void delayBackground(unsigned long dsdelay)
{
  unsigned long timer = millis() + dsdelay;

  while (!timeOutReached(timer)) {
    backgroundtasks();
  }
}

/********************************************************************************************\
   Toggle controller enabled state
 \*********************************************************************************************/
bool setControllerEnableStatus(controllerIndex_t controllerIndex, bool enabled)
{
  if (!validControllerIndex(controllerIndex)) { return false; }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("setControllerEnableStatus"));
  #endif // ifndef BUILD_NO_RAM_TRACKER

  // Only enable controller if it has a protocol configured
  if ((Settings.Protocol[controllerIndex] != 0) || !enabled) {
    Settings.ControllerEnabled[controllerIndex] = enabled;
    return true;
  }
  return false;
}

/********************************************************************************************\
   Toggle task enabled state
 \*********************************************************************************************/
bool setTaskEnableStatus(struct EventStruct *event, bool enabled)
{
  if (!validTaskIndex(event->TaskIndex)) { return false; }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("setTaskEnableStatus"));
  #endif // ifndef BUILD_NO_RAM_TRACKER

  // Only enable task if it has a Plugin configured
  if (validPluginID(Settings.TaskDeviceNumber[event->TaskIndex]) || !enabled) {
    String dummy;

    if (!enabled) {
      PluginCall(PLUGIN_EXIT, event, dummy);
    }
    Settings.TaskDeviceEnabled[event->TaskIndex] = enabled;

    if (enabled) {
      if (!PluginCall(PLUGIN_INIT, event, dummy)) {
        return false;
      }

      // Schedule the task to be executed almost immediately
      Scheduler.schedule_task_device_timer(event->TaskIndex, millis() + 10);
    }
    return true;
  }
  return false;
}

/********************************************************************************************\
   Clear task settings for given task
 \*********************************************************************************************/
void taskClear(taskIndex_t taskIndex, bool save)
{
  if (!validTaskIndex(taskIndex)) { return; }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("taskClear"));
  #endif // ifndef BUILD_NO_RAM_TRACKER
  Settings.clearTask(taskIndex);
  Cache.clearTaskCaches();
  ExtraTaskSettings.clear(); // Invalidate any cached values.
  ExtraTaskSettings.TaskIndex = taskIndex;

  if (save) {
    SaveTaskSettings(taskIndex);
    SaveSettings();
  }
}

/********************************************************************************************\
   check the program memory hash
   The const MD5_MD5_MD5_MD5_BoundariesOfTheSegmentsGoHere... needs to remain unchanged as it will be replaced by
   - 16 bytes md5 hash, followed by
   - 4 * uint32_t start of memory segment 1-4
   - 4 * uint32_t end of memory segment 1-4
   currently there are only two segemts included in the hash. Unused segments have start adress 0.
   Execution time 520kb @80Mhz: 236ms
   Returns: 0 if hash compare fails, number of checked bytes otherwise.
   The reference hash is calculated by a .py file and injected into the binary.
   Caution: currently the hash sits in an unchecked segment. If it ever moves to a checked segment, make sure
   it is excluded from the calculation !
 \*********************************************************************************************/
#if defined(ARDUINO_ESP8266_RELEASE_2_3_0)
void dump(uint32_t addr) { // Seems already included in core 2.4 ...
  serialPrint(String(addr, HEX));
  serialPrint(": ");

  for (uint32_t a = addr; a < addr + 16; a++)
  {
    serialPrint(String(pgm_read_byte(a), HEX));
    serialPrint(" ");
  }
  serialPrintln();
}

#endif // if defined(ARDUINO_ESP8266_RELEASE_2_3_0)

/*
   uint32_t progMemMD5check(){
    checkRAM(F("progMemMD5check"));
 #define BufSize 10
    uint32_t calcBuffer[BufSize];
    CRCValues.numberOfCRCBytes = 0;
    memcpy (calcBuffer,CRCValues.compileTimeMD5,16);                                                  // is there still the dummy in memory
       ? - the dummy needs to be replaced by the real md5 after linking.
    if( memcmp (calcBuffer, "MD5_MD5_MD5_",12)==0){                                                   // do not memcmp with CRCdummy
       directly or it will get optimized away.
        addLog(LOG_LEVEL_INFO, F("CRC  : No program memory checksum found. Check output of crc2.py"));
        return 0;
    }
    MD5Builder md5;
    md5.begin();
    for (int l = 0; l<4; l++){                                                                            // check max segments,  if the
       pointer is not 0
        uint32_t *ptrStart = (uint32_t *)&CRCValues.compileTimeMD5[16+l*4];
        uint32_t *ptrEnd =   (uint32_t *)&CRCValues.compileTimeMD5[16+4*4+l*4];
        if ((*ptrStart) == 0) break;                                                                      // segment not used.
        for (uint32_t i = *ptrStart; i< (*ptrEnd) ; i=i+sizeof(calcBuffer)){                              // "<" includes last byte
             for (int buf = 0; buf < BufSize; buf ++){
                calcBuffer[buf] = pgm_read_dword((uint32_t*)i+buf);                                       // read 4 bytes
                CRCValues.numberOfCRCBytes+=sizeof(calcBuffer[0]);
             }
             md5.add(reinterpret_cast<const uint8_t *>(&calcBuffer[0]),(*ptrEnd-i)<sizeof(calcBuffer) ? (*ptrEnd-i):sizeof(calcBuffer) );
                    // add buffer to md5.
                At the end not the whole buffer. md5 ptr to data in ram.
        }
   }
   md5.calculate();
   md5.getBytes(CRCValues.runTimeMD5);
   if ( CRCValues.checkPassed())  {
      addLog(LOG_LEVEL_INFO, F("CRC  : program checksum       ...OK"));
      return CRCValues.numberOfCRCBytes;
   }
   addLog(LOG_LEVEL_INFO, F("CRC  : program checksum       ...FAIL"));
   return 0;
   }
 */

/********************************************************************************************\
   Handler for keeping ExtraTaskSettings up to date using cache
 \*********************************************************************************************/
String getTaskDeviceName(taskIndex_t TaskIndex) {
  return Cache.getTaskDeviceName(TaskIndex);
}

/********************************************************************************************\
   Handler for getting Value Names from TaskIndex

   - value names can be accessed with task variable index
   - maximum number of variables <= defined number of variables in plugin
 \*********************************************************************************************/
String getTaskValueName(taskIndex_t TaskIndex, uint8_t TaskValueIndex) {
  const int valueCount = getValueCountForTask(TaskIndex);
  if (TaskValueIndex < valueCount) {
    return Cache.getTaskDeviceValueName(TaskIndex, TaskValueIndex);
  }
  return EMPTY_STRING;
}

/********************************************************************************************\
   If RX and TX tied together, perform emergency reset to get the system out of boot loops
 \*********************************************************************************************/
void emergencyReset()
{
  // Direct Serial is allowed here, since this is only an emergency task.
  Serial.begin(115200);
  Serial.write(0xAA);
  Serial.write(0x55);
  delay(1);

  if (Serial.available() == 2) {
    if ((Serial.read() == 0xAA) && (Serial.read() == 0x55))
    {
      serialPrintln(F("\n\n\rSystem will reset to factory defaults in 10 seconds..."));
      delay(10000);
      ResetFactory();
    }
  }
}

/********************************************************************************************\
   Delayed reboot, in case of issues, do not reboot with high frequency as it might not help...
 \*********************************************************************************************/
void delayedReboot(int rebootDelay, ESPEasy_Scheduler::IntendedRebootReason_e reason)
{
  // Direct Serial is allowed here, since this is only an emergency task.
  while (rebootDelay != 0)
  {
    serialPrint(F("Delayed Reset "));
    serialPrintln(String(rebootDelay));
    rebootDelay--;
    delay(1000);
  }
  reboot(reason);
}

void reboot(ESPEasy_Scheduler::IntendedRebootReason_e reason) {
  prepareShutdown(reason);
  #if defined(ESP32)
  ESP.restart();
  #else // if defined(ESP32)
  ESP.reset();
  #endif // if defined(ESP32)
}

void FeedSW_watchdog()
{
  #ifdef ESP8266
  ESP.wdtFeed();
  #endif // ifdef ESP8266
}

void SendValueLogger(taskIndex_t TaskIndex)
{
#if !defined(BUILD_NO_DEBUG) || FEATURE_SD
  bool   featureSD = false;
  String logger;
  # if FEATURE_SD
  featureSD = true;
  # endif // if FEATURE_SD

  if (featureSD || loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(TaskIndex);

    if (validDeviceIndex(DeviceIndex)) {
      const uint8_t valueCount = getValueCountForTask(TaskIndex);

      for (uint8_t varNr = 0; varNr < valueCount; varNr++)
      {
        logger += node_time.getDateString('-');
        logger += ' ';
        logger += node_time.getTimeString(':');
        logger += ',';
        logger += Settings.Unit;
        logger += ',';
        logger += getTaskDeviceName(TaskIndex);
        logger += ',';
        logger += getTaskValueName(TaskIndex, varNr);
        logger += ',';
        logger += formatUserVarNoCheck(TaskIndex, varNr);
        logger += F("\r\n");
      }
      addLog(LOG_LEVEL_DEBUG, logger);
    }
  }
#endif // if !defined(BUILD_NO_DEBUG) || FEATURE_SD

#if FEATURE_SD
  String filename = F("VALUES.CSV");
  fs::File   logFile  = SD.open(filename, FILE_WRITE);

  if (logFile) {
    logFile.print(logger);
  }
  logFile.close();
#endif // if FEATURE_SD
}

// #######################################################################################################
// ############################ quite acurate but slow color converter####################################
// #######################################################################################################
// uses H 0..360 S 1..100 I/V 1..100 (according to homie convention)
// Source https://blog.saikoled.com/post/44677718712/how-to-convert-from-hsi-to-rgb-white

void HSV2RGB(float H, float S, float I, int rgb[3]) {
  int r, g, b;

  H = fmod(H, 360);                           // cycle H around to 0-360 degrees
  H = 3.14159f * H / static_cast<float>(180); // Convert to radians.
  S = S / 100;
  S = S > 0 ? (S < 1 ? S : 1) : 0;            // clamp S and I to interval [0,1]
  I = I / 100;
  I = I > 0 ? (I < 1 ? I : 1) : 0;

  // Math! Thanks in part to Kyle Miller.
  if (H < 2.09439f) {
    r = 255 * I / 3 * (1 + S * cosf(H) / cosf(1.047196667f - H));
    g = 255 * I / 3 * (1 + S * (1 - cosf(H) / cosf(1.047196667f - H)));
    b = 255 * I / 3 * (1 - S);
  } else if (H < 4.188787f) {
    H = H - 2.09439f;
    g = 255 * I / 3 * (1 + S * cosf(H) / cosf(1.047196667f - H));
    b = 255 * I / 3 * (1 + S * (1 - cosf(H) / cosf(1.047196667f - H)));
    r = 255 * I / 3 * (1 - S);
  } else {
    H = H - 4.188787f;
    b = 255 * I / 3 * (1 + S * cosf(H) / cosf(1.047196667f - H));
    r = 255 * I / 3 * (1 + S * (1 - cosf(H) / cosf(1.047196667f - H)));
    g = 255 * I / 3 * (1 - S);
  }
  rgb[0] = r;
  rgb[1] = g;
  rgb[2] = b;
}

// uses H 0..360 S 1..100 I/V 1..100 (according to homie convention)
// Source https://blog.saikoled.com/post/44677718712/how-to-convert-from-hsi-to-rgb-white

void HSV2RGBW(float H, float S, float I, int rgbw[4]) {
  int   r, g, b, w;
  float cos_h, cos_1047_h;

  H = fmod(H, 360);                           // cycle H around to 0-360 degrees
  H = 3.14159f * H / static_cast<float>(180); // Convert to radians.
  S = S / 100;
  S = S > 0 ? (S < 1 ? S : 1) : 0;            // clamp S and I to interval [0,1]
  I = I / 100;
  I = I > 0 ? (I < 1 ? I : 1) : 0;

  if (H < 2.09439f) {
    cos_h      = cosf(H);
    cos_1047_h = cosf(1.047196667f - H);
    r          = S * 255 * I / 3 * (1 + cos_h / cos_1047_h);
    g          = S * 255 * I / 3 * (1 + (1 - cos_h / cos_1047_h));
    b          = 0;
    w          = 255 * (1 - S) * I;
  } else if (H < 4.188787f) {
    H          = H - 2.09439f;
    cos_h      = cosf(H);
    cos_1047_h = cosf(1.047196667f - H);
    g          = S * 255 * I / 3 * (1 + cos_h / cos_1047_h);
    b          = S * 255 * I / 3 * (1 + (1 - cos_h / cos_1047_h));
    r          = 0;
    w          = 255 * (1 - S) * I;
  } else {
    H          = H - 4.188787f;
    cos_h      = cosf(H);
    cos_1047_h = cosf(1.047196667f - H);
    b          = S * 255 * I / 3 * (1 + cos_h / cos_1047_h);
    r          = S * 255 * I / 3 * (1 + (1 - cos_h / cos_1047_h));
    g          = 0;
    w          = 255 * (1 - S) * I;
  }

  rgbw[0] = r;
  rgbw[1] = g;
  rgbw[2] = b;
  rgbw[3] = w;
}

// Simple bitwise get/set functions

uint8_t get8BitFromUL(uint32_t number, uint8_t bitnr) {
  return (number >> bitnr) & 0xFF;
}

void set8BitToUL(uint32_t& number, uint8_t bitnr, uint8_t value) {
  uint32_t mask     = (0xFFUL << bitnr);
  uint32_t newvalue = ((value << bitnr) & mask);

  number = (number & ~mask) | newvalue;
}

uint8_t get4BitFromUL(uint32_t number, uint8_t bitnr) {
  return (number >> bitnr) &  0x0F;
}

void set4BitToUL(uint32_t& number, uint8_t bitnr, uint8_t value) {
  uint32_t mask     = (0x0FUL << bitnr);
  uint32_t newvalue = ((value << bitnr) & mask);

  number = (number & ~mask) | newvalue;
}

uint8_t get3BitFromUL(uint32_t number, uint8_t bitnr) {
  return (number >> bitnr) &  0x07;
}

void set3BitToUL(uint32_t& number, uint8_t bitnr, uint8_t value) {
  uint32_t mask     = (0x07UL << bitnr);
  uint32_t newvalue = ((value << bitnr) & mask);

  number = (number & ~mask) | newvalue;
}

uint8_t get2BitFromUL(uint32_t number, uint8_t bitnr) {
  return (number >> bitnr) &  0x03;
}

void set2BitToUL(uint32_t& number, uint8_t bitnr, uint8_t value) {
  uint32_t mask     = (0x03UL << bitnr);
  uint32_t newvalue = ((value << bitnr) & mask);

  number = (number & ~mask) | newvalue;
}

float getCPUload() {
  return 100.0f - Scheduler.getIdleTimePct();
}

int getLoopCountPerSec() {
  return loopCounterLast / 30;
}

int getUptimeMinutes() {
  return wdcounter / 2;
}

/******************************************************************************
 * scan an int array of specified size for a value
 *****************************************************************************/
bool intArrayContains(const int arraySize, const int array[], const int& value) {
  for (int i = 0; i < arraySize; i++) {
    if (array[i] == value) { return true; }
  }
  return false;
}

bool intArrayContains(const int arraySize, const uint8_t array[], const uint8_t& value) {
  for (int i = 0; i < arraySize; i++) {
    if (array[i] == value) { return true; }
  }
  return false;
}

#ifndef BUILD_NO_RAM_TRACKER
void logMemUsageAfter(const __FlashStringHelper *function, int value) {
  // Store free memory in an int, as subtracting may sometimes result in negative value.
  // The recorded used memory is not an exact value, as background (or interrupt) tasks may also allocate or free heap memory.
  static int last_freemem = ESP.getFreeHeap();
  const int  freemem_end  = ESP.getFreeHeap();

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log;
    if (log.reserve(128)) {
      log  = F("After ");
      log += function;

      if (value >= 0) {
        log += value;
      }

      while (log.length() < 30) { log += ' '; }
      log += F("Free mem after: ");
      log += freemem_end;

      while (log.length() < 55) { log += ' '; }
      log += F("diff: ");
      log += last_freemem - freemem_end;
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
  }

  last_freemem = freemem_end;
}

#endif // ifndef BUILD_NO_RAM_TRACKER

#include "../Helpers/StringGenerator_WiFi.h"

#include "../Helpers/StringConverter.h"

#include "../Globals/ESPEasyWiFiEvent.h"

const __FlashStringHelper * WiFi_encryptionType(uint8_t encryptionType) {
switch (encryptionType) {
  #ifdef ESP32
    case WIFI_AUTH_OPEN:             return F("open"); 
    case WIFI_AUTH_WEP:              return F("WEP"); 
    case WIFI_AUTH_WPA_PSK:          return F("WPA/PSK"); 
    case WIFI_AUTH_WPA2_PSK:         return F("WPA2/PSK"); 
    case WIFI_AUTH_WPA_WPA2_PSK:     return F("WPA/WPA2/PSK"); 
    case WIFI_AUTH_WPA2_ENTERPRISE:  return F("WPA2 Enterprise"); 
  #else // ifdef ESP32
    case ENC_TYPE_WEP:   return F("WEP"); 
    case ENC_TYPE_TKIP:  return F("WPA/PSK"); 
    case ENC_TYPE_CCMP:  return F("WPA2/PSK"); 
    case ENC_TYPE_NONE:  return F("open"); 
    case ENC_TYPE_AUTO:  return F("WPA/WPA2/PSK"); 
  #endif // ifdef ESP32
    default:
	break;
      
  }
  return F("-");
}


#ifdef ESP8266
#ifdef LIMIT_BUILD_SIZE
String SDKwifiStatusToString(uint8_t sdk_wifistatus)
{
  return String(sdk_wifistatus);
}
#else
const __FlashStringHelper * SDKwifiStatusToString(uint8_t sdk_wifistatus)
{
  switch (sdk_wifistatus) {
    case STATION_IDLE:           return F("STATION_IDLE");
    case STATION_CONNECTING:     return F("STATION_CONNECTING");
    case STATION_WRONG_PASSWORD: return F("STATION_WRONG_PASSWORD");
    case STATION_NO_AP_FOUND:    return F("STATION_NO_AP_FOUND");
    case STATION_CONNECT_FAIL:   return F("STATION_CONNECT_FAIL");
    case STATION_GOT_IP:         return F("STATION_GOT_IP");
  }
  return F("Unknown");
}
#endif
#endif

const __FlashStringHelper * ArduinoWifiStatusToFlashString(uint8_t arduino_corelib_wifistatus) {
  switch (arduino_corelib_wifistatus) {
    case WL_NO_SHIELD:       return F("WL_NO_SHIELD");
    case WL_IDLE_STATUS:     return F("WL_IDLE_STATUS");
    case WL_NO_SSID_AVAIL:   return F("WL_NO_SSID_AVAIL");
    case WL_SCAN_COMPLETED:  return F("WL_SCAN_COMPLETED");
    case WL_CONNECTED:       return F("WL_CONNECTED");
    case WL_CONNECT_FAILED:  return F("WL_CONNECT_FAILED");
    case WL_CONNECTION_LOST: return F("WL_CONNECTION_LOST");
    case WL_DISCONNECTED:    return F("WL_DISCONNECTED");
  }
  return F("-");
}

String ArduinoWifiStatusToString(uint8_t arduino_corelib_wifistatus) {
  #ifdef LIMIT_BUILD_SIZE
  return String(arduino_corelib_wifistatus);
  #else
  String log = ArduinoWifiStatusToFlashString(arduino_corelib_wifistatus);
  log += ' ';
  log += arduino_corelib_wifistatus;
  return log;
  #endif
}


const __FlashStringHelper * getLastDisconnectReason(WiFiDisconnectReason reason) {
  switch (reason) {
    case WIFI_DISCONNECT_REASON_UNSPECIFIED:                return F("Unspecified");              
    case WIFI_DISCONNECT_REASON_AUTH_EXPIRE:                return F("Auth expire");              
    case WIFI_DISCONNECT_REASON_AUTH_LEAVE:                 return F("Auth leave");               
    case WIFI_DISCONNECT_REASON_ASSOC_EXPIRE:               return F("Assoc expire");             
    case WIFI_DISCONNECT_REASON_ASSOC_TOOMANY:              return F("Assoc toomany");            
    case WIFI_DISCONNECT_REASON_NOT_AUTHED:                 return F("Not authed");               
    case WIFI_DISCONNECT_REASON_NOT_ASSOCED:                return F("Not assoced");              
    case WIFI_DISCONNECT_REASON_ASSOC_LEAVE:                return F("Assoc leave");              
    case WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED:           return F("Assoc not authed");         
    case WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD:        return F("Disassoc pwrcap bad");      
    case WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD:       return F("Disassoc supchan bad");     
    case WIFI_DISCONNECT_REASON_IE_INVALID:                 return F("IE invalid");               
    case WIFI_DISCONNECT_REASON_MIC_FAILURE:                return F("Mic failure");              
    case WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT:     return F("4way handshake timeout");   
    case WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT:   return F("Group key update timeout"); 
    case WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS:         return F("IE in 4way differs");       
    case WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID:       return F("Group cipher invalid");     
    case WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID:    return F("Pairwise cipher invalid");  
    case WIFI_DISCONNECT_REASON_AKMP_INVALID:               return F("AKMP invalid");             
    case WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION:      return F("Unsupp RSN IE version");    
    case WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP:         return F("Invalid RSN IE cap");       
    case WIFI_DISCONNECT_REASON_802_1X_AUTH_FAILED:         return F("802 1X auth failed");       
    case WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED:      return F("Cipher suite rejected");    
    case WIFI_DISCONNECT_REASON_BEACON_TIMEOUT:             return F("Beacon timeout");           
    case WIFI_DISCONNECT_REASON_NO_AP_FOUND:                return F("No AP found");              
    case WIFI_DISCONNECT_REASON_AUTH_FAIL:                  return F("Auth fail");                
    case WIFI_DISCONNECT_REASON_ASSOC_FAIL:                 return F("Assoc fail");               
    case WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT:          return F("Handshake timeout");        
    default:  return F("Unknown");
  }
}

String getLastDisconnectReason() {
  #ifndef LIMIT_BUILD_SIZE
  String reason = wrap_braces(String(WiFiEventData.lastDisconnectReason));
  reason += ' ';
  reason += getLastDisconnectReason(WiFiEventData.lastDisconnectReason);
  return reason;
  #else
  return wrap_braces(String(WiFiEventData.lastDisconnectReason));
  #endif
}

#include "../Helpers/_CPlugin_init.h"

#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../DataStructs/TimingStats.h"

#include "../DataTypes/ESPEasy_plugin_functions.h"

#include "../Globals/CPlugins.h"
#include "../Globals/Protocol.h"
#include "../Globals/Settings.h"

#include "../Helpers/Misc.h"

// ********************************************************************************
// Initialize all Controller CPlugins that where defined earlier
// and initialize the function call pointer into the CCPlugin array
// ********************************************************************************

// Uncrustify must not be used on macros, so turn it off.
// *INDENT-OFF*
#define ADDCPLUGIN(NNN, ID) if (addCPlugin(ID, x)) { CPlugin_ptr[x++] = &CPlugin_##NNN; }
// Uncrustify must not be used on macros, but we're now done, so turn Uncrustify on again.
// *INDENT-ON*

void CPluginInit()
{
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif
  ProtocolIndex_to_CPlugin_id[CPLUGIN_MAX] = INVALID_C_PLUGIN_ID;
  uint8_t x;

  // Clear pointer table for all plugins
  for (x = 0; x < CPLUGIN_MAX; x++)
  {
    CPlugin_ptr[x]                 = nullptr;
    ProtocolIndex_to_CPlugin_id[x] = INVALID_C_PLUGIN_ID;
    // Do not initialize CPlugin_id_to_ProtocolIndex[x] to an invalid value. (it is map)
  }

  x = 0;

#ifdef USES_C001
  ADDCPLUGIN(001, 1)
#endif

#ifdef USES_C002
  ADDCPLUGIN(002, 2)
#endif

#ifdef USES_C003
  ADDCPLUGIN(003, 3)
#endif

#ifdef USES_C004
  ADDCPLUGIN(004, 4)
#endif

#ifdef USES_C005
  ADDCPLUGIN(005, 5)
#endif

#ifdef USES_C006
  ADDCPLUGIN(006, 6)
#endif

#ifdef USES_C007
  ADDCPLUGIN(007, 7)
#endif

#ifdef USES_C008
  ADDCPLUGIN(008, 8)
#endif

#ifdef USES_C009
  ADDCPLUGIN(009, 9)
#endif

#ifdef USES_C010
  ADDCPLUGIN(010, 10)
#endif

#ifdef USES_C011
  ADDCPLUGIN(011, 11)
#endif

#ifdef USES_C012
  ADDCPLUGIN(012, 12)
#endif

#ifdef USES_C013
  ADDCPLUGIN(013, 13)
#endif

#ifdef USES_C014
  ADDCPLUGIN(014, 14)
#endif

#ifdef USES_C015
  ADDCPLUGIN(015, 15)
#endif

#ifdef USES_C016
  ADDCPLUGIN(016, 16)
#endif

#ifdef USES_C017
  ADDCPLUGIN(017, 17)
#endif

#ifdef USES_C018
  ADDCPLUGIN(018, 18)
#endif

#ifdef USES_C019
  ADDCPLUGIN(019, 19)
#endif

#ifdef USES_C020
  ADDCPLUGIN(020, 20)
#endif

#ifdef USES_C021
  ADDCPLUGIN(021, 21)
#endif

#ifdef USES_C022
  ADDCPLUGIN(022, 22)
#endif

#ifdef USES_C023
  ADDCPLUGIN(023, 23)
#endif

#ifdef USES_C024
  ADDCPLUGIN(024, 24)
#endif

#ifdef USES_C025
  ADDCPLUGIN(025, 25)
#endif

#ifdef USES_C026
  ADDCPLUGIN(026, 26)
#endif

#ifdef USES_C027
  ADDCPLUGIN(027, 27)
#endif

#ifdef USES_C028
  ADDCPLUGIN(028, 28)
#endif

#ifdef USES_C029
  ADDCPLUGIN(029, 29)
#endif

#ifdef USES_C030
  ADDCPLUGIN(030, 30)
#endif

#ifdef USES_C031
  ADDCPLUGIN(031, 31)
#endif

#ifdef USES_C032
  ADDCPLUGIN(032, 32)
#endif

#ifdef USES_C033
  ADDCPLUGIN(033, 33)
#endif

#ifdef USES_C034
  ADDCPLUGIN(034, 34)
#endif

#ifdef USES_C035
  ADDCPLUGIN(035, 35)
#endif

#ifdef USES_C036
  ADDCPLUGIN(036, 36)
#endif

#ifdef USES_C037
  ADDCPLUGIN(037, 37)
#endif

#ifdef USES_C038
  ADDCPLUGIN(038, 38)
#endif

#ifdef USES_C039
  ADDCPLUGIN(039, 39)
#endif

#ifdef USES_C040
  ADDCPLUGIN(040, 40)
#endif

#ifdef USES_C041
  ADDCPLUGIN(041, 41)
#endif

#ifdef USES_C042
  ADDCPLUGIN(042, 42)
#endif

#ifdef USES_C043
  ADDCPLUGIN(043, 43)
#endif

#ifdef USES_C044
  ADDCPLUGIN(044, 44)
#endif

#ifdef USES_C045
  ADDCPLUGIN(045, 45)
#endif

#ifdef USES_C046
  ADDCPLUGIN(046, 46)
#endif

#ifdef USES_C047
  ADDCPLUGIN(047, 47)
#endif

#ifdef USES_C048
  ADDCPLUGIN(048, 48)
#endif

#ifdef USES_C049
  ADDCPLUGIN(049, 49)
#endif

#ifdef USES_C050
  ADDCPLUGIN(050, 50)
#endif

#ifdef USES_C051
  ADDCPLUGIN(051, 51)
#endif

#ifdef USES_C052
  ADDCPLUGIN(052, 52)
#endif

#ifdef USES_C053
  ADDCPLUGIN(053, 53)
#endif

#ifdef USES_C054
  ADDCPLUGIN(054, 54)
#endif

#ifdef USES_C055
  ADDCPLUGIN(055, 55)
#endif

#ifdef USES_C056
  ADDCPLUGIN(056, 56)
#endif

#ifdef USES_C057
  ADDCPLUGIN(057, 57)
#endif

#ifdef USES_C058
  ADDCPLUGIN(058, 58)
#endif

#ifdef USES_C059
  ADDCPLUGIN(059, 59)
#endif

#ifdef USES_C060
  ADDCPLUGIN(060, 60)
#endif

#ifdef USES_C061
  ADDCPLUGIN(061, 61)
#endif

#ifdef USES_C062
  ADDCPLUGIN(062, 62)
#endif

#ifdef USES_C063
  ADDCPLUGIN(063, 63)
#endif

#ifdef USES_C064
  ADDCPLUGIN(064, 64)
#endif

#ifdef USES_C065
  ADDCPLUGIN(065, 65)
#endif

#ifdef USES_C066
  ADDCPLUGIN(066, 66)
#endif

#ifdef USES_C067
  ADDCPLUGIN(067, 67)
#endif

#ifdef USES_C068
  ADDCPLUGIN(068, 68)
#endif

#ifdef USES_C069
  ADDCPLUGIN(069, 69)
#endif

#ifdef USES_C070
  ADDCPLUGIN(070, 70)
#endif

#ifdef USES_C071
  ADDCPLUGIN(071, 71)
#endif

#ifdef USES_C072
  ADDCPLUGIN(072, 72)
#endif

#ifdef USES_C073
  ADDCPLUGIN(073, 73)
#endif

#ifdef USES_C074
  ADDCPLUGIN(074, 74)
#endif

#ifdef USES_C075
  ADDCPLUGIN(075, 75)
#endif

#ifdef USES_C076
  ADDCPLUGIN(076, 76)
#endif

#ifdef USES_C077
  ADDCPLUGIN(077, 77)
#endif

#ifdef USES_C078
  ADDCPLUGIN(078, 78)
#endif

#ifdef USES_C079
  ADDCPLUGIN(079, 79)
#endif

#ifdef USES_C080
  ADDCPLUGIN(080, 80)
#endif

#ifdef USES_C081
  ADDCPLUGIN(081, 81)
#endif

#ifdef USES_C082
  ADDCPLUGIN(082, 82)
#endif

#ifdef USES_C083
  ADDCPLUGIN(083, 83)
#endif

#ifdef USES_C084
  ADDCPLUGIN(084, 84)
#endif

#ifdef USES_C085
  ADDCPLUGIN(085, 85)
#endif

#ifdef USES_C086
  ADDCPLUGIN(086, 86)
#endif

#ifdef USES_C087
  ADDCPLUGIN(087, 87)
#endif

#ifdef USES_C088
  ADDCPLUGIN(088, 88)
#endif

#ifdef USES_C089
  ADDCPLUGIN(089, 89)
#endif

#ifdef USES_C090
  ADDCPLUGIN(090, 90)
#endif

#ifdef USES_C091
  ADDCPLUGIN(091, 91)
#endif

#ifdef USES_C092
  ADDCPLUGIN(092, 92)
#endif

#ifdef USES_C093
  ADDCPLUGIN(093, 93)
#endif

#ifdef USES_C094
  ADDCPLUGIN(094, 94)
#endif

#ifdef USES_C095
  ADDCPLUGIN(095, 95)
#endif

#ifdef USES_C096
  ADDCPLUGIN(096, 96)
#endif

#ifdef USES_C097
  ADDCPLUGIN(097, 97)
#endif

#ifdef USES_C098
  ADDCPLUGIN(098, 98)
#endif

#ifdef USES_C099
  ADDCPLUGIN(099, 99)
#endif

#ifdef USES_C100
  ADDCPLUGIN(100, 100)
#endif

#ifdef USES_C101
  ADDCPLUGIN(101, 101)
#endif

#ifdef USES_C102
  ADDCPLUGIN(102, 102)
#endif

#ifdef USES_C103
  ADDCPLUGIN(103, 103)
#endif

#ifdef USES_C104
  ADDCPLUGIN(104, 104)
#endif

#ifdef USES_C105
  ADDCPLUGIN(105, 105)
#endif

#ifdef USES_C106
  ADDCPLUGIN(106, 106)
#endif

#ifdef USES_C107
  ADDCPLUGIN(107, 107)
#endif

#ifdef USES_C108
  ADDCPLUGIN(108, 108)
#endif

#ifdef USES_C109
  ADDCPLUGIN(109, 109)
#endif

#ifdef USES_C110
  ADDCPLUGIN(110, 110)
#endif

#ifdef USES_C111
  ADDCPLUGIN(111, 111)
#endif

#ifdef USES_C112
  ADDCPLUGIN(112, 112)
#endif

#ifdef USES_C113
  ADDCPLUGIN(113, 113)
#endif

#ifdef USES_C114
  ADDCPLUGIN(114, 114)
#endif

#ifdef USES_C115
  ADDCPLUGIN(115, 115)
#endif

#ifdef USES_C116
  ADDCPLUGIN(116, 116)
#endif

#ifdef USES_C117
  ADDCPLUGIN(117, 117)
#endif

#ifdef USES_C118
  ADDCPLUGIN(118, 118)
#endif

#ifdef USES_C119
  ADDCPLUGIN(119, 119)
#endif

#ifdef USES_C120
  ADDCPLUGIN(120, 120)
#endif

#ifdef USES_C121
  ADDCPLUGIN(121, 121)
#endif

#ifdef USES_C122
  ADDCPLUGIN(122, 122)
#endif

#ifdef USES_C123
  ADDCPLUGIN(123, 123)
#endif

#ifdef USES_C124
  ADDCPLUGIN(124, 124)
#endif

#ifdef USES_C125
  ADDCPLUGIN(125, 125)
#endif

#ifdef USES_C126
  ADDCPLUGIN(126, 126)
#endif

#ifdef USES_C127
  ADDCPLUGIN(127, 127)
#endif

#ifdef USES_C128
  ADDCPLUGIN(128, 128)
#endif

#ifdef USES_C129
  ADDCPLUGIN(129, 129)
#endif

#ifdef USES_C130
  ADDCPLUGIN(130, 130)
#endif

#ifdef USES_C131
  ADDCPLUGIN(131, 131)
#endif

#ifdef USES_C132
  ADDCPLUGIN(132, 132)
#endif

#ifdef USES_C133
  ADDCPLUGIN(133, 133)
#endif

#ifdef USES_C134
  ADDCPLUGIN(134, 134)
#endif

#ifdef USES_C135
  ADDCPLUGIN(135, 135)
#endif

#ifdef USES_C136
  ADDCPLUGIN(136, 136)
#endif

#ifdef USES_C137
  ADDCPLUGIN(137, 137)
#endif

#ifdef USES_C138
  ADDCPLUGIN(138, 138)
#endif

#ifdef USES_C139
  ADDCPLUGIN(139, 139)
#endif

#ifdef USES_C140
  ADDCPLUGIN(140, 140)
#endif

#ifdef USES_C141
  ADDCPLUGIN(141, 141)
#endif

#ifdef USES_C142
  ADDCPLUGIN(142, 142)
#endif

#ifdef USES_C143
  ADDCPLUGIN(143, 143)
#endif

#ifdef USES_C144
  ADDCPLUGIN(144, 144)
#endif

#ifdef USES_C145
  ADDCPLUGIN(145, 145)
#endif

#ifdef USES_C146
  ADDCPLUGIN(146, 146)
#endif

#ifdef USES_C147
  ADDCPLUGIN(147, 147)
#endif

#ifdef USES_C148
  ADDCPLUGIN(148, 148)
#endif

#ifdef USES_C149
  ADDCPLUGIN(149, 149)
#endif

#ifdef USES_C150
  ADDCPLUGIN(150, 150)
#endif

#ifdef USES_C151
  ADDCPLUGIN(151, 151)
#endif

#ifdef USES_C152
  ADDCPLUGIN(152, 152)
#endif

#ifdef USES_C153
  ADDCPLUGIN(153, 153)
#endif

#ifdef USES_C154
  ADDCPLUGIN(154, 154)
#endif

#ifdef USES_C155
  ADDCPLUGIN(155, 155)
#endif

#ifdef USES_C156
  ADDCPLUGIN(156, 156)
#endif

#ifdef USES_C157
  ADDCPLUGIN(157, 157)
#endif

#ifdef USES_C158
  ADDCPLUGIN(158, 158)
#endif

#ifdef USES_C159
  ADDCPLUGIN(159, 159)
#endif

#ifdef USES_C160
  ADDCPLUGIN(160, 160)
#endif

#ifdef USES_C161
  ADDCPLUGIN(161, 161)
#endif

#ifdef USES_C162
  ADDCPLUGIN(162, 162)
#endif

#ifdef USES_C163
  ADDCPLUGIN(163, 163)
#endif

#ifdef USES_C164
  ADDCPLUGIN(164, 164)
#endif

#ifdef USES_C165
  ADDCPLUGIN(165, 165)
#endif

#ifdef USES_C166
  ADDCPLUGIN(166, 166)
#endif

#ifdef USES_C167
  ADDCPLUGIN(167, 167)
#endif

#ifdef USES_C168
  ADDCPLUGIN(168, 168)
#endif

#ifdef USES_C169
  ADDCPLUGIN(169, 169)
#endif

#ifdef USES_C170
  ADDCPLUGIN(170, 170)
#endif

#ifdef USES_C171
  ADDCPLUGIN(171, 171)
#endif

#ifdef USES_C172
  ADDCPLUGIN(172, 172)
#endif

#ifdef USES_C173
  ADDCPLUGIN(173, 173)
#endif

#ifdef USES_C174
  ADDCPLUGIN(174, 174)
#endif

#ifdef USES_C175
  ADDCPLUGIN(175, 175)
#endif

#ifdef USES_C176
  ADDCPLUGIN(176, 176)
#endif

#ifdef USES_C177
  ADDCPLUGIN(177, 177)
#endif

#ifdef USES_C178
  ADDCPLUGIN(178, 178)
#endif

#ifdef USES_C179
  ADDCPLUGIN(179, 179)
#endif

#ifdef USES_C180
  ADDCPLUGIN(180, 180)
#endif

#ifdef USES_C181
  ADDCPLUGIN(181, 181)
#endif

#ifdef USES_C182
  ADDCPLUGIN(182, 182)
#endif

#ifdef USES_C183
  ADDCPLUGIN(183, 183)
#endif

#ifdef USES_C184
  ADDCPLUGIN(184, 184)
#endif

#ifdef USES_C185
  ADDCPLUGIN(185, 185)
#endif

#ifdef USES_C186
  ADDCPLUGIN(186, 186)
#endif

#ifdef USES_C187
  ADDCPLUGIN(187, 187)
#endif

#ifdef USES_C188
  ADDCPLUGIN(188, 188)
#endif

#ifdef USES_C189
  ADDCPLUGIN(189, 189)
#endif

#ifdef USES_C190
  ADDCPLUGIN(190, 190)
#endif

#ifdef USES_C191
  ADDCPLUGIN(191, 191)
#endif

#ifdef USES_C192
  ADDCPLUGIN(192, 192)
#endif

#ifdef USES_C193
  ADDCPLUGIN(193, 193)
#endif

#ifdef USES_C194
  ADDCPLUGIN(194, 194)
#endif

#ifdef USES_C195
  ADDCPLUGIN(195, 195)
#endif

#ifdef USES_C196
  ADDCPLUGIN(196, 196)
#endif

#ifdef USES_C197
  ADDCPLUGIN(197, 197)
#endif

#ifdef USES_C198
  ADDCPLUGIN(198, 198)
#endif

#ifdef USES_C199
  ADDCPLUGIN(199, 199)
#endif

#ifdef USES_C200
  ADDCPLUGIN(200, 200)
#endif

#ifdef USES_C201
  ADDCPLUGIN(201, 201)
#endif

#ifdef USES_C202
  ADDCPLUGIN(202, 202)
#endif

#ifdef USES_C203
  ADDCPLUGIN(203, 203)
#endif

#ifdef USES_C204
  ADDCPLUGIN(204, 204)
#endif

#ifdef USES_C205
  ADDCPLUGIN(205, 205)
#endif

#ifdef USES_C206
  ADDCPLUGIN(206, 206)
#endif

#ifdef USES_C207
  ADDCPLUGIN(207, 207)
#endif

#ifdef USES_C208
  ADDCPLUGIN(208, 208)
#endif

#ifdef USES_C209
  ADDCPLUGIN(209, 209)
#endif

#ifdef USES_C210
  ADDCPLUGIN(210, 210)
#endif

#ifdef USES_C211
  ADDCPLUGIN(211, 211)
#endif

#ifdef USES_C212
  ADDCPLUGIN(212, 212)
#endif

#ifdef USES_C213
  ADDCPLUGIN(213, 213)
#endif

#ifdef USES_C214
  ADDCPLUGIN(214, 214)
#endif

#ifdef USES_C215
  ADDCPLUGIN(215, 215)
#endif

#ifdef USES_C216
  ADDCPLUGIN(216, 216)
#endif

#ifdef USES_C217
  ADDCPLUGIN(217, 217)
#endif

#ifdef USES_C218
  ADDCPLUGIN(218, 218)
#endif

#ifdef USES_C219
  ADDCPLUGIN(219, 219)
#endif

#ifdef USES_C220
  ADDCPLUGIN(220, 220)
#endif

#ifdef USES_C221
  ADDCPLUGIN(221, 221)
#endif

#ifdef USES_C222
  ADDCPLUGIN(222, 222)
#endif

#ifdef USES_C223
  ADDCPLUGIN(223, 223)
#endif

#ifdef USES_C224
  ADDCPLUGIN(224, 224)
#endif

#ifdef USES_C225
  ADDCPLUGIN(225, 225)
#endif

#ifdef USES_C226
  ADDCPLUGIN(226, 226)
#endif

#ifdef USES_C227
  ADDCPLUGIN(227, 227)
#endif

#ifdef USES_C228
  ADDCPLUGIN(228, 228)
#endif

#ifdef USES_C229
  ADDCPLUGIN(229, 229)
#endif

#ifdef USES_C230
  ADDCPLUGIN(230, 230)
#endif

#ifdef USES_C231
  ADDCPLUGIN(231, 231)
#endif

#ifdef USES_C232
  ADDCPLUGIN(232, 232)
#endif

#ifdef USES_C233
  ADDCPLUGIN(233, 233)
#endif

#ifdef USES_C234
  ADDCPLUGIN(234, 234)
#endif

#ifdef USES_C235
  ADDCPLUGIN(235, 235)
#endif

#ifdef USES_C236
  ADDCPLUGIN(236, 236)
#endif

#ifdef USES_C237
  ADDCPLUGIN(237, 237)
#endif

#ifdef USES_C238
  ADDCPLUGIN(238, 238)
#endif

#ifdef USES_C239
  ADDCPLUGIN(239, 239)
#endif

#ifdef USES_C240
  ADDCPLUGIN(240, 240)
#endif

#ifdef USES_C241
  ADDCPLUGIN(241, 241)
#endif

#ifdef USES_C242
  ADDCPLUGIN(242, 242)
#endif

#ifdef USES_C243
  ADDCPLUGIN(243, 243)
#endif

#ifdef USES_C244
  ADDCPLUGIN(244, 244)
#endif

#ifdef USES_C245
  ADDCPLUGIN(245, 245)
#endif

#ifdef USES_C246
  ADDCPLUGIN(246, 246)
#endif

#ifdef USES_C247
  ADDCPLUGIN(247, 247)
#endif

#ifdef USES_C248
  ADDCPLUGIN(248, 248)
#endif

#ifdef USES_C249
  ADDCPLUGIN(249, 249)
#endif

#ifdef USES_C250
  ADDCPLUGIN(250, 250)
#endif

#ifdef USES_C251
  ADDCPLUGIN(251, 251)
#endif

#ifdef USES_C252
  ADDCPLUGIN(252, 252)
#endif

#ifdef USES_C253
  ADDCPLUGIN(253, 253)
#endif

#ifdef USES_C254
  ADDCPLUGIN(254, 254)
#endif

#ifdef USES_C255
  ADDCPLUGIN(255, 255)
#endif

// When extending this, search for EXTEND_CONTROLLER_IDS 
// in the code to find all places that need to be updated too.

  #ifndef BUILD_NO_RAM_TRACKER
  logMemUsageAfter(F("ADDCPLUGIN(...)"));
  #endif

  CPluginCall(CPlugin::Function::CPLUGIN_PROTOCOL_ADD, 0);
  #ifndef BUILD_NO_RAM_TRACKER
  logMemUsageAfter(F("CPLUGIN_PROTOCOL_ADD"));
  #endif


  // Set all not supported cplugins to disabled.
  for (controllerIndex_t controller = 0; controller < CONTROLLER_MAX; ++controller) {
    if (!supportedCPluginID(Settings.Protocol[controller])) {
      Settings.ControllerEnabled[controller] = false;
    }
  }
  CPluginCall(CPlugin::Function::CPLUGIN_INIT_ALL, 0);
}

#include "../Helpers/PeriodicalActions.h"


#include "../../ESPEasy-Globals.h"

#include "../ControllerQueue/DelayQueueElements.h"
#include "../ControllerQueue/MQTT_queue_element.h"
#include "../DataStructs/TimingStats.h"
#include "../DataTypes/ESPEasy_plugin_functions.h"
#include "../ESPEasyCore/Controller.h"
#include "../ESPEasyCore/ESPEasyGPIO.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../ESPEasyCore/ESPEasyWifi.h"
#include "../ESPEasyCore/ESPEasyRules.h"
#include "../ESPEasyCore/Serial.h"
#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/ESPEasy_Scheduler.h"
#include "../Globals/ESPEasy_time.h"
#include "../Globals/EventQueue.h"
#include "../Globals/MainLoopCommand.h"
#include "../Globals/MQTT.h"
#include "../Globals/NetworkState.h"
#include "../Globals/RTC.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/Services.h"
#include "../Globals/Settings.h"
#include "../Globals/Statistics.h"
#include "../Globals/WiFi_AP_Candidates.h"
#include "../Helpers/ESPEasyRTC.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/Memory.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Networking.h"
#include "../Helpers/StringGenerator_System.h"
#include "../Helpers/StringGenerator_WiFi.h"
#include "../Helpers/StringProvider.h"

#ifdef USES_C015
#include "../../ESPEasy_fdwdecl.h"
#endif



#define PLUGIN_ID_MQTT_IMPORT         37


/*********************************************************************************************\
 * Tasks that run 50 times per second
\*********************************************************************************************/

void run50TimesPerSecond() {
  String dummy;
  {
    START_TIMER;
    PluginCall(PLUGIN_FIFTY_PER_SECOND, 0, dummy);
    STOP_TIMER(PLUGIN_CALL_50PS);
  }
  {
    START_TIMER;
    CPluginCall(CPlugin::Function::CPLUGIN_FIFTY_PER_SECOND, 0, dummy);
    STOP_TIMER(CPLUGIN_CALL_50PS);
  }
  processNextEvent();
}

/*********************************************************************************************\
 * Tasks that run 10 times per second
\*********************************************************************************************/
void run10TimesPerSecond() {
  String dummy;
  //@giig19767g: WARNING: Monitor10xSec must run before PLUGIN_TEN_PER_SECOND
  {
    START_TIMER;
    GPIO_Monitor10xSec();
    STOP_TIMER(PLUGIN_CALL_10PSU);
  }
  {
    START_TIMER;
    PluginCall(PLUGIN_TEN_PER_SECOND, 0, dummy);
    STOP_TIMER(PLUGIN_CALL_10PS);
  }
  {
    START_TIMER;
//    PluginCall(PLUGIN_UNCONDITIONAL_POLL, 0, dummyString);
    PluginCall(PLUGIN_MONITOR, 0, dummy);
    STOP_TIMER(PLUGIN_CALL_10PSU);
  }
  {
    START_TIMER;
    CPluginCall(CPlugin::Function::CPLUGIN_TEN_PER_SECOND, 0, dummy);
    STOP_TIMER(CPLUGIN_CALL_10PS);
  }
  
  #ifdef USES_C015
  if (NetworkConnected())
      Blynk_Run_c015();
  #endif
  #ifndef USE_RTOS_MULTITASKING
    web_server.handleClient();
  #endif
}


/*********************************************************************************************\
 * Tasks each second
\*********************************************************************************************/
void runOncePerSecond()
{
  START_TIMER;
  updateLogLevelCache();
  dailyResetCounter++;
  if (dailyResetCounter > 86400) // 1 day elapsed... //86400
  {
    RTC.flashDayCounter=0;
    saveToRTC();
    dailyResetCounter=0;
    addLog(LOG_LEVEL_INFO, F("SYS  : Reset 24h counters"));
  }

  if (Settings.ConnectionFailuresThreshold)
    if (WiFiEventData.connectionFailures > Settings.ConnectionFailuresThreshold)
      delayedReboot(60, ESPEasy_Scheduler::IntendedRebootReason_e::DelayedReboot);

  if (cmd_within_mainloop != 0)
  {
    switch (cmd_within_mainloop)
    {
      case CMD_WIFI_DISCONNECT:
        {
          WifiDisconnect();
          break;
        }
      case CMD_REBOOT:
        {
          reboot(ESPEasy_Scheduler::IntendedRebootReason_e::CommandReboot);
          break;
        }
    }
    cmd_within_mainloop = 0;
  }
  // clock events
  if (node_time.reportNewMinute()) {
    String dummy;
    PluginCall(PLUGIN_CLOCK_IN, 0, dummy);
    if (Settings.UseRules)
    {
      String event;
      event.reserve(21);
      event  = F("Clock#Time=");
      event += node_time.weekday_str();
      event += ',';

      if (node_time.hour() < 10) {
        event += '0';
      }
      event += node_time.hour();
      event += ':';

      if (node_time.minute() < 10) {
        event += '0';
      }
      event += node_time.minute();
      // TD-er: Do not add to the eventQueue, but execute right now.
      rulesProcessing(event);
    }
  }

//  unsigned long start = micros();
  String dummy;
  PluginCall(PLUGIN_ONCE_A_SECOND, 0, dummy);
//  unsigned long elapsed = micros() - start;


  // I2C Watchdog feed
  if (Settings.WDI2CAddress != 0)
  {
    Wire.beginTransmission(Settings.WDI2CAddress);
    Wire.write(0xA5);
    Wire.endTransmission();
  }

  checkResetFactoryPin();
  STOP_TIMER(PLUGIN_CALL_1PS);
}

/*********************************************************************************************\
 * Tasks each 30 seconds
\*********************************************************************************************/
void runEach30Seconds()
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAMtoLog();
  #endif
  wdcounter++;
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log;
    log.reserve(80);
    log = F("WD   : Uptime ");
    log += getUptimeMinutes();
    log += F(" ConnectFailures ");
    log += WiFiEventData.connectionFailures;
    log += F(" FreeMem ");
    log += FreeMem();
    bool logWiFiStatus = true;
    #if FEATURE_ETHERNET
    if(active_network_medium == NetworkMedium_t::Ethernet) {
      logWiFiStatus = false;
      log += F( " EthSpeedState ");
      log += getValue(LabelType::ETH_SPEED_STATE);
      log += F(" ETH status: ");
      log += EthEventData.ESPEasyEthStatusToString();
    }
    #endif // if FEATURE_ETHERNET
    if (logWiFiStatus) {
      log += F(" WiFiStatus ");
      log += ArduinoWifiStatusToString(WiFi.status());
      log += F(" ESPeasy internal wifi status: ");
      log += WiFiEventData.ESPeasyWifiStatusToString();
    }

//    log += F(" ListenInterval ");
//    log += WiFi.getListenInterval();
    addLogMove(LOG_LEVEL_INFO, log);
  }
  WiFi_AP_Candidates.purge_expired();
  #if FEATURE_ESPEASY_P2P
  sendSysInfoUDP(1);
  refreshNodeList();
  #endif

  // sending $stats to homie controller
  CPluginCall(CPlugin::Function::CPLUGIN_INTERVAL, 0);

  #if defined(ESP8266)
  #if FEATURE_SSDP
  if (Settings.UseSSDP)
    SSDP_update();

  #endif // if FEATURE_SSDP
  #endif
#if FEATURE_ADC_VCC
  if (!WiFiEventData.wifiConnectInProgress) {
    vcc = ESP.getVcc() / 1000.0f;
  }
#endif

  #if FEATURE_REPORTING
  ReportStatus();
  #endif // if FEATURE_REPORTING

}

#if FEATURE_MQTT


void scheduleNextMQTTdelayQueue() {
  if (MQTTDelayHandler != nullptr) {
    Scheduler.scheduleNextDelayQueue(ESPEasy_Scheduler::IntervalTimer_e::TIMER_MQTT_DELAY_QUEUE, MQTTDelayHandler->getNextScheduleTime());
  }
}

void schedule_all_MQTTimport_tasks() {
  controllerIndex_t ControllerIndex = firstEnabledMQTT_ControllerIndex();

  if (!validControllerIndex(ControllerIndex)) { return; }

  deviceIndex_t DeviceIndex = getDeviceIndex(PLUGIN_ID_MQTT_IMPORT); // Check if P037_MQTTimport is present in the build
  if (validDeviceIndex(DeviceIndex)) {
    for (taskIndex_t task = 0; task < TASKS_MAX; task++) {
      if ((Settings.TaskDeviceNumber[task] == PLUGIN_ID_MQTT_IMPORT) &&
          (Settings.TaskDeviceEnabled[task])) {
        // Schedule a call to each enabled MQTT import plugin to notify the broker connection state
        EventStruct event(task);
        event.Par1 = MQTTclient_connected ? 1 : 0;
        Scheduler.schedule_plugin_task_event_timer(DeviceIndex, PLUGIN_MQTT_CONNECTION_STATE, std::move(event));
      }
    }
  }
}

void processMQTTdelayQueue() {
  if (MQTTDelayHandler == nullptr) {
    return;
  }
  runPeriodicalMQTT(); // Update MQTT connected state.
  if (!MQTTclient_connected) {
    scheduleNextMQTTdelayQueue();
    return;
  }

  START_TIMER;
  MQTT_queue_element *element(MQTTDelayHandler->getNext());

  if (element == nullptr) { return; }

  if (MQTTclient.publish(element->_topic.c_str(), element->_payload.c_str(), element->_retained)) {
    if (WiFiEventData.connectionFailures > 0) {
      --WiFiEventData.connectionFailures;
    }
    MQTTDelayHandler->markProcessed(true);
  } else {
    MQTTDelayHandler->markProcessed(false);
#ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log = F("MQTT : process MQTT queue not published, ");
      log += MQTTDelayHandler->sendQueue.size();
      log += F(" items left in queue");
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
#endif // ifndef BUILD_NO_DEBUG
  }
  Scheduler.setIntervalTimerOverride(ESPEasy_Scheduler::IntervalTimer_e::TIMER_MQTT, 10); // Make sure the MQTT is being processed as soon as possible.
  scheduleNextMQTTdelayQueue();
  STOP_TIMER(MQTT_DELAY_QUEUE);
}

void updateMQTTclient_connected() {
  if (MQTTclient_connected != MQTTclient.connected()) {
    MQTTclient_connected = !MQTTclient_connected;
    if (!MQTTclient_connected) {
      if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
        String connectionError = F("MQTT : Connection lost, state: ");
        connectionError += getMQTT_state();
        addLogMove(LOG_LEVEL_ERROR, connectionError);
      }
      MQTTclient_must_send_LWT_connected = false;
    } else {
      // Now schedule all tasks using the MQTT controller.
      schedule_all_MQTTimport_tasks();
    }
    if (Settings.UseRules) {
      if (MQTTclient_connected) {
        eventQueue.add(F("MQTT#Connected"));
      } else {
        eventQueue.add(F("MQTT#Disconnected"));
      }
    }
  }
  if (!MQTTclient_connected) {
    // As suggested here: https://github.com/letscontrolit/ESPEasy/issues/1356
    if (timermqtt_interval < 30000) {
      timermqtt_interval += 5000;
    }
  } else {
    timermqtt_interval = 250;
  }
  Scheduler.setIntervalTimer(ESPEasy_Scheduler::IntervalTimer_e::TIMER_MQTT);
  scheduleNextMQTTdelayQueue();
}

void runPeriodicalMQTT() {
  // MQTT_KEEPALIVE = 15 seconds.
  if (!NetworkConnected(10)) {
    updateMQTTclient_connected();
    return;
  }
  //dont do this in backgroundtasks(), otherwise causes crashes. (https://github.com/letscontrolit/ESPEasy/issues/683)
  controllerIndex_t enabledMqttController = firstEnabledMQTT_ControllerIndex();
  if (validControllerIndex(enabledMqttController)) {
    if (!MQTTclient.loop()) {
      updateMQTTclient_connected();
      if (MQTTCheck(enabledMqttController)) {
        updateMQTTclient_connected();
      }
    }
  } else {
    if (MQTTclient.connected()) {
      MQTTclient.disconnect();
      updateMQTTclient_connected();
    }
  }
}

// FIXME TD-er: Must move to a more logical part of the code
controllerIndex_t firstEnabledMQTT_ControllerIndex() {
  for (controllerIndex_t i = 0; i < CONTROLLER_MAX; ++i) {
    protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(i);
    if (validProtocolIndex(ProtocolIndex)) {
      if (Protocol[ProtocolIndex].usesMQTT && Settings.ControllerEnabled[i]) {
        return i;
      }
    }
  }
  return INVALID_CONTROLLER_INDEX;
}


#endif //if FEATURE_MQTT



void logTimerStatistics() {
  uint8_t loglevel = LOG_LEVEL_DEBUG;
  updateLoopStats_30sec(loglevel);
#ifndef BUILD_NO_DEBUG
//  logStatistics(loglevel, true);
  if (loglevelActiveFor(loglevel)) {
    String queueLog = F("Scheduler stats: (called/tasks/max_length/idle%) ");
    queueLog += Scheduler.getQueueStats();
    addLogMove(loglevel, queueLog);
  }
#endif
}

void updateLoopStats_30sec(uint8_t loglevel) {
  loopCounterLast = loopCounter;
  loopCounter = 0;
  if (loopCounterLast > loopCounterMax)
    loopCounterMax = loopCounterLast;

  Scheduler.updateIdleTimeStats();

#ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(loglevel)) {
    String log = F("LoopStats: shortestLoop: ");
    log += shortestLoop;
    log += F(" longestLoop: ");
    log += longestLoop;
    log += F(" avgLoopDuration: ");
    log += loop_usec_duration_total / loopCounter_full;
    log += F(" loopCounterMax: ");
    log += loopCounterMax;
    log += F(" loopCounterLast: ");
    log += loopCounterLast;
    addLogMove(loglevel, log);
  }
#endif
  loop_usec_duration_total = 0;
  loopCounter_full = 1;
}


/********************************************************************************************\
   Clean up all before going to sleep or reboot.
 \*********************************************************************************************/
void flushAndDisconnectAllClients() {
  if (anyControllerEnabled()) {
#if FEATURE_MQTT
    bool mqttControllerEnabled = validControllerIndex(firstEnabledMQTT_ControllerIndex());
#endif //if FEATURE_MQTT
    unsigned long timer = millis() + 1000;
    while (!timeOutReached(timer)) {
      // call to all controllers (delay queue) to flush all data.
      CPluginCall(CPlugin::Function::CPLUGIN_FLUSH, 0);
#if FEATURE_MQTT      
      if (mqttControllerEnabled && MQTTclient.connected()) {
        MQTTclient.loop();
      }
#endif //if FEATURE_MQTT
    }
#if FEATURE_MQTT
    if (mqttControllerEnabled && MQTTclient.connected()) {
      MQTTclient.disconnect();
      updateMQTTclient_connected();
    }
#endif //if FEATURE_MQTT
    saveToRTC();
    delay(100); // Flush anything in the network buffers.
  }
  process_serialWriteBuffer();
}


void prepareShutdown(ESPEasy_Scheduler::IntendedRebootReason_e reason)
{
#if FEATURE_MQTT
  runPeriodicalMQTT(); // Flush outstanding MQTT messages
#endif // if FEATURE_MQTT
  process_serialWriteBuffer();
  flushAndDisconnectAllClients();
  saveUserVarToRTC();
  setWifiMode(WIFI_OFF);
  ESPEASY_FS.end();
  delay(100); // give the node time to flush all before reboot or sleep
  node_time.now();
  Scheduler.markIntendedReboot(reason);
  saveToRTC();
}



#include "../Helpers/_Plugin_SensorTypeHelper.h"

#include "../../_Plugin_Helper.h"
#include "../DataStructs/DeviceStruct.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/Device.h"
#include "../Globals/Plugins.h"

#include "../WebServer/Markup.h"

/*********************************************************************************************\
   Get value count from sensor type

   Only use this function to determine nr of output values when changing output type of a task
   To get the actual output values for a task, use getValueCountForTask
\*********************************************************************************************/
uint8_t getValueCountFromSensorType(Sensor_VType sensorType)
{
  switch (sensorType)
  {
    case Sensor_VType::SENSOR_TYPE_NONE:
      return 0;
    case Sensor_VType::SENSOR_TYPE_SINGLE: // single value sensor, used for Dallas, BH1750, etc
    case Sensor_VType::SENSOR_TYPE_SWITCH:
    case Sensor_VType::SENSOR_TYPE_DIMMER:
      return 1;
    case Sensor_VType::SENSOR_TYPE_LONG: // single LONG value, stored in two floats (rfid tags)
      return 1;
    case Sensor_VType::SENSOR_TYPE_TEMP_HUM:
    case Sensor_VType::SENSOR_TYPE_TEMP_BARO:
    case Sensor_VType::SENSOR_TYPE_DUAL:
      return 2;
    case Sensor_VType::SENSOR_TYPE_TEMP_HUM_BARO:
    case Sensor_VType::SENSOR_TYPE_TEMP_EMPTY_BARO: // Values 1 and 3 will contain data.
    case Sensor_VType::SENSOR_TYPE_TRIPLE:
    case Sensor_VType::SENSOR_TYPE_WIND:
      return 3;
    case Sensor_VType::SENSOR_TYPE_QUAD:
      return 4;
    case Sensor_VType::SENSOR_TYPE_STRING:  // String type data stored in the event->String2
      return 1;
    case Sensor_VType::SENSOR_TYPE_NOT_SET:  break;
  }
  addLog(LOG_LEVEL_ERROR, F("getValueCountFromSensorType: Unknown sensortype"));
  return 0;
}

const __FlashStringHelper * getSensorTypeLabel(Sensor_VType sensorType) {
  switch (sensorType) {
    case Sensor_VType::SENSOR_TYPE_SINGLE:           return F("Single");
    case Sensor_VType::SENSOR_TYPE_TEMP_HUM:         return F("Temp / Hum");
    case Sensor_VType::SENSOR_TYPE_TEMP_BARO:        return F("Temp / Baro");
    case Sensor_VType::SENSOR_TYPE_TEMP_EMPTY_BARO:  return F("Temp / - / Baro");
    case Sensor_VType::SENSOR_TYPE_TEMP_HUM_BARO:    return F("Temp / Hum / Baro");
    case Sensor_VType::SENSOR_TYPE_DUAL:             return F("Dual");
    case Sensor_VType::SENSOR_TYPE_TRIPLE:           return F("Triple");
    case Sensor_VType::SENSOR_TYPE_QUAD:             return F("Quad");
    case Sensor_VType::SENSOR_TYPE_SWITCH:           return F("Switch");
    case Sensor_VType::SENSOR_TYPE_DIMMER:           return F("Dimmer");
    case Sensor_VType::SENSOR_TYPE_LONG:             return F("Long");
    case Sensor_VType::SENSOR_TYPE_WIND:             return F("Wind");
    case Sensor_VType::SENSOR_TYPE_STRING:           return F("String");
    case Sensor_VType::SENSOR_TYPE_NONE:             return F("None");
    case Sensor_VType::SENSOR_TYPE_NOT_SET:  break;
  }
  return F("");
}

void sensorTypeHelper_webformLoad_allTypes(struct EventStruct *event, uint8_t pconfigIndex)
{
  uint8_t optionValues[12];

  optionValues[0]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_SINGLE);
  optionValues[1]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_TEMP_HUM);
  optionValues[2]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_TEMP_BARO);
  optionValues[3]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_TEMP_HUM_BARO);
  optionValues[4]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_DUAL);
  optionValues[5]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_TRIPLE);
  optionValues[6]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_QUAD);
  optionValues[7]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_SWITCH);
  optionValues[8]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_DIMMER);
  optionValues[9]  = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_LONG);
  optionValues[10] = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_WIND);
  optionValues[11] = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_STRING);
  sensorTypeHelper_webformLoad(event, pconfigIndex, 11, optionValues);
}

void sensorTypeHelper_webformLoad_header()
{
  addFormSubHeader(F("Output Configuration"));
}

void sensorTypeHelper_webformLoad_simple(struct EventStruct *event, uint8_t pconfigIndex)
{
  sensorTypeHelper_webformLoad_header();

  uint8_t optionValues[4];
  optionValues[0] = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_SINGLE);
  optionValues[1] = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_DUAL);
  optionValues[2] = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_TRIPLE);
  optionValues[3] = static_cast<uint8_t>(Sensor_VType::SENSOR_TYPE_QUAD);
  sensorTypeHelper_webformLoad(event, pconfigIndex, 4, optionValues);
}

void sensorTypeHelper_webformLoad(struct EventStruct *event, uint8_t pconfigIndex, int optionCount, const uint8_t options[])
{
  if (pconfigIndex >= PLUGIN_CONFIGVAR_MAX) {
    return;
  }
  Sensor_VType choice      = static_cast<Sensor_VType>(PCONFIG(pconfigIndex));
  const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(event->TaskIndex);
  if (!validDeviceIndex(DeviceIndex)) {
    choice = Sensor_VType::SENSOR_TYPE_NONE;
    PCONFIG(pconfigIndex) = static_cast<uint8_t>(choice);
  } else if (getValueCountFromSensorType(choice) != getValueCountForTask(event->TaskIndex)) {
    // Invalid value
    checkDeviceVTypeForTask(event);
    choice                = event->sensorType;
    PCONFIG(pconfigIndex) = static_cast<uint8_t>(choice);
  }
  String outputTypeLabel = F("Output Data Type");
  if (Device[DeviceIndex].OutputDataType ==  Output_Data_type_t::Simple) {
    switch(event->sensorType) {
      case Sensor_VType::SENSOR_TYPE_SINGLE:
      case Sensor_VType::SENSOR_TYPE_DUAL:
      case Sensor_VType::SENSOR_TYPE_TRIPLE:
      case Sensor_VType::SENSOR_TYPE_QUAD:
        // These are valid
        break;
      default:
      {
        choice = Device[DeviceIndex].VType;
        PCONFIG(pconfigIndex) = static_cast<uint8_t>(choice);
        break;
      }
    }
    outputTypeLabel = F("Number Output Values");
  }
  addRowLabel(outputTypeLabel);
  addSelector_Head(PCONFIG_LABEL(pconfigIndex));

  for (uint8_t x = 0; x < optionCount; x++)
  {
    String name     = getSensorTypeLabel(static_cast<Sensor_VType>(options[x]));
    addSelector_Item(name,
                     options[x],
                     choice == static_cast<Sensor_VType>(options[x]));
  }
  addSelector_Foot();
  {
    String note;
    note = F("Changing '");
    note += outputTypeLabel;
    note += F("' may affect behavior of some controllers (e.g. Domoticz)");
    addFormNote(note);
  }
}

void sensorTypeHelper_saveOutputSelector(struct EventStruct *event, uint8_t pconfigIndex, uint8_t valueIndex, const String& defaultValueName)
{
  if (defaultValueName.equals(ExtraTaskSettings.TaskDeviceValueNames[valueIndex])) {
    ZERO_FILL(ExtraTaskSettings.TaskDeviceValueNames[valueIndex]);
  }
  pconfig_webformSave(event, pconfigIndex);
}

void pconfig_webformSave(struct EventStruct *event, uint8_t pconfigIndex)
{
  PCONFIG(pconfigIndex) = getFormItemInt(PCONFIG_LABEL(pconfigIndex), 0);
}

void sensorTypeHelper_loadOutputSelector(
  struct EventStruct *event, uint8_t pconfigIndex, uint8_t valuenr,
  int optionCount, const __FlashStringHelper * options[], const int indices[])
{
  uint8_t   choice = PCONFIG(pconfigIndex);
  String label  = F("Value ");

  label += (valuenr + 1);
  addFormSelector(label, PCONFIG_LABEL(pconfigIndex), optionCount, options, indices, choice);
}


void sensorTypeHelper_loadOutputSelector(
  struct EventStruct *event, uint8_t pconfigIndex, uint8_t valuenr,
  int optionCount, const String options[], const int indices[])
{
  uint8_t   choice = PCONFIG(pconfigIndex);
  String label  = F("Value ");

  label += (valuenr + 1);
  addFormSelector(label, PCONFIG_LABEL(pconfigIndex), optionCount, options, indices, choice);
}

#include "../Helpers/ESPEasyStatistics.h"


#if FEATURE_TIMING_STATS

#include "../DataStructs/TimingStats.h"
#include "../WebServer/WebServer.h"
#include "../Globals/Protocol.h"
#include "../Helpers/Convert.h"

/*
   void logStatistics(uint8_t loglevel, bool clearStats) {
   if (loglevelActiveFor(loglevel)) {
    String log;
    log.reserve(80);
    for (auto& x: pluginStats) {
        if (!x.second.isEmpty()) {
            const int deviceIndex = x.first/256;
            log = F("PluginStats P_");
            log += deviceIndex + 1;
            log += '_';
            log += getPluginNameFromDeviceIndex(deviceIndex);
            log += ' ';
            log += getPluginFunctionName(x.first%256);
            log += ' ';
            log += getLogLine(x.second);
            addLog(loglevel, log);
            if (clearStats) x.second.reset();
        }
    }
    for (auto& x: miscStats) {
        if (!x.second.isEmpty()) {
            log = getMiscStatsName(x.first);
            log += F(" stats: ");
            log += getLogLine(x.second);
            addLog(loglevel, log);
            if (clearStats) x.second.reset();
        }
    }
    log = getMiscStatsName(TIME_DIFF_COMPUTE);
    log += F(" stats: Count: ");
    log += timediff_calls;
    log += F(" - CPU cycles per call: ");
    log += static_cast<float>(timediff_cpu_cycles_total) / static_cast<float>(timediff_calls);
    addLog(loglevel, log);
    if (clearStats) {
      timediff_calls = 0;
      timediff_cpu_cycles_total = 0;
    }
   }
   }
 */

void stream_json_timing_stats(const TimingStats& stats, long timeSinceLastReset) {
  uint64_t minVal, maxVal;
  uint64_t  count = stats.getMinMax(minVal, maxVal);
  float call_per_sec = static_cast<float>(count) / static_cast<float>(timeSinceLastReset) * 1000.0f;

  json_number(F("count"), ull2String(count));
  json_number(F("call-per-sec"),   toString(call_per_sec, 2));
  json_number(F("min"),   ull2String(minVal));
  json_number(F("max"),   ull2String(maxVal));
  json_number(F("avg"),   toString(stats.getAvg(), 2));
  json_prop(F("unit"), F("usec"));
}

void jsonStatistics(bool clearStats) {
  bool firstPlugin     = true;
  int  currentPluginId = -1;
  long timeSinceLastReset = timePassedSince(timingstats_last_reset);


  json_open(true, F("plugin"));

  for (auto& x: pluginStats) {
    if (!x.second.isEmpty()) {
      const int deviceIndex = x.first / 256;

      if (currentPluginId != deviceIndex) {
        // new plugin
        currentPluginId = deviceIndex;
        if (!firstPlugin) {
          json_close();
          json_close(true); // close previous function list
          json_close();     // close previous plugin
        }
        // Start new plugin stream
        json_open(); // open new plugin
        json_prop(F("name"), getPluginNameFromDeviceIndex(deviceIndex));
        json_prop(F("id"),   String(DeviceIndex_to_Plugin_id[deviceIndex]));
        json_open(true, F("function")); // open function
        json_open(); // open first function element
      }

      // Stream function timing stats
      json_open(false, getPluginFunctionName(x.first % 256));
      {
        stream_json_timing_stats(x.second, timeSinceLastReset);
      }
      json_close(false);
      if (clearStats) { x.second.reset(); }
      firstPlugin = false;
    }
  }
  if (!firstPlugin) {
    // We added some, so we must make sure to close the last entry
    json_close();     // close first function element
    json_close(true); // close previous function
    json_close();     // close previous plugin
  }
  json_close(true);   // Close plugin list


  json_open(true, F("controller"));
  bool firstController = true;
  int  currentProtocolIndex = -1;
  for (auto& x: controllerStats) {
    if (!x.second.isEmpty()) {      
      const int ProtocolIndex = x.first / 256;
      if (currentProtocolIndex != ProtocolIndex) {
        // new protocol
        currentProtocolIndex = ProtocolIndex;
        if (!firstController) {
          json_close();
          json_close(true); // close previous function list
          json_close();     // close previous protocol
        }
        // Start new protocol stream
        json_open(); // open new plugin
        json_prop(F("name"), getCPluginNameFromProtocolIndex(ProtocolIndex));
        json_prop(F("id"),   String(Protocol[ProtocolIndex].Number));
        json_open(true, F("function")); // open function
        json_open(); // open first function element

      }
      // Stream function timing stats
      json_open(false, getCPluginCFunctionName(static_cast<CPlugin::Function>(x.first % 256)));
      {
        stream_json_timing_stats(x.second, timeSinceLastReset);
      }
      json_close(false);
      if (clearStats) { x.second.reset(); }
      firstController = false;
    }
  }
  if (!firstController) {
    // We added some, so we must make sure to close the last entry
    json_close();     // close first function element
    json_close(true); // close previous function
    json_close();     // close previous plugin
  }

  json_close(true);   // Close controller list


  json_open(true, F("misc"));
  for (auto& x: miscStats) {
    if (!x.second.isEmpty()) {
      json_open(); // open new misc item
      json_prop(F("name"), getMiscStatsName(x.first));
      json_prop(F("id"),   String(x.first));
      json_open(true, F("function")); // open function
      json_open(); // open first function element
      // Stream function timing stats
      json_open(false, to_internal_string(getMiscStatsName(x.first), '-'));
      {
        stream_json_timing_stats(x.second, timeSinceLastReset);
      }
      json_close(false);
      json_close();     // close first function element
      json_close(true); // close function
      json_close();     // close misc item
      if (clearStats) { x.second.reset(); }
    }
  }

  json_close(true);   // Close misc list

  if (clearStats) {
    timingstats_last_reset = millis();
  }
}


#endif // if FEATURE_TIMING_STATS
#include "../Helpers/_CPlugin_DomoticzHelper.h"

#if FEATURE_DOMOTICZ

# include "../DataStructs/ESPEasy_EventStruct.h"
# include "../DataTypes/TaskIndex.h"

# include "../ESPEasyCore/ESPEasy_Log.h"

# include "../Globals/Cache.h"

# include "../Helpers/Convert.h"
# include "../Helpers/StringConverter.h"

# include "../../ESPEasy-Globals.h"

# ifdef USES_C002
#  include <ArduinoJson.h>
# endif // ifdef USES_C002


// HUM_STAT can be one of:

// 0=Normal
// 1=Comfortable
// 2=Dry
// 3=Wet
String humStatDomoticz(struct EventStruct *event, uint8_t rel_index) {
  userVarIndex_t userVarIndex = event->BaseVarIndex + rel_index;

  if (validTaskVarIndex(rel_index) && validUserVarIndex(userVarIndex)) {
    const int hum = UserVar[userVarIndex];

    if (hum < 30) { return formatUserVarDomoticz(2); }

    if (hum < 40) { return formatUserVarDomoticz(0); }

    if (hum < 59) { return formatUserVarDomoticz(1); }
  }
  return formatUserVarDomoticz(3);
}

int mapRSSItoDomoticz() {
  long rssi = WiFi.RSSI();

  if (-50 < rssi) { return 10; }

  if (rssi <= -98) { return 0;  }
  rssi = rssi + 97; // Range 0..47 => 1..9
  return (rssi / 5) + 1;
}

int mapVccToDomoticz() {
  # if FEATURE_ADC_VCC

  // Voltage range from 2.6V .. 3.6V => 0..100%
  if (vcc < 2.6f) { return 0; }
  return (vcc - 2.6f) * 100;
  # else // if FEATURE_ADC_VCC
  return 255;
  # endif // if FEATURE_ADC_VCC
}

// Format including trailing semi colon
String formatUserVarDomoticz(struct EventStruct *event, uint8_t rel_index) {
  String text = formatUserVarNoCheck(event, rel_index);

  text += ';';
  return text;
}

String formatUserVarDomoticz(int value) {
  String text;

  text += value;
  text.trim();
  text += ';';
  return text;
}

String formatDomoticzSensorType(struct EventStruct *event) {
  String values;

  switch (event->getSensorType())
  {
    case Sensor_VType::SENSOR_TYPE_SINGLE: // single value sensor, used for Dallas, BH1750, etc
      values = formatUserVarDomoticz(event, 0);
      break;
    case Sensor_VType::SENSOR_TYPE_LONG:   // single LONG value, stored in two floats (rfid tags)
      values = UserVar.getSensorTypeLong(event->TaskIndex);
      break;
    case Sensor_VType::SENSOR_TYPE_DUAL:   // any sensor that uses two simple values
      values  = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 1);
      break;
    case Sensor_VType::SENSOR_TYPE_TEMP_HUM:

      // temp + hum + hum_stat, used for DHT11
      // http://www.domoticz.com/wiki/Domoticz_API/JSON_URL%27s#Temperature.2Fhumidity
      values  = formatUserVarDomoticz(event, 0); // TEMP = Temperature
      values += formatUserVarDomoticz(event, 1); // HUM = Humidity
      values += humStatDomoticz(event, 1);       // HUM_STAT = Humidity status
      break;
    case Sensor_VType::SENSOR_TYPE_TEMP_HUM_BARO:

      // temp + hum + hum_stat + bar + bar_fore, used for BME280
      // http://www.domoticz.com/wiki/Domoticz_API/JSON_URL%27s#Temperature.2Fhumidity.2Fbarometer
      values  = formatUserVarDomoticz(event, 0); // TEMP = Temperature
      values += formatUserVarDomoticz(event, 1); // HUM = Humidity
      values += humStatDomoticz(event, 1);       // HUM_STAT = Humidity status
      values += formatUserVarDomoticz(event, 2); // BAR = Barometric pressure
      values += formatUserVarDomoticz(0);        // BAR_FOR = Barometer forecast
      break;
    case Sensor_VType::SENSOR_TYPE_TEMP_BARO:

      // temp + hum + hum_stat + bar + bar_fore, used for BMP085
      // http://www.domoticz.com/wiki/Domoticz_API/JSON_URL%27s#Temperature.2Fbarometer
      values  = formatUserVarDomoticz(event, 0); // TEMP = Temperature
      values += formatUserVarDomoticz(event, 1); // BAR = Barometric pressure
      values += formatUserVarDomoticz(0);        // BAR_FOR = Barometer forecast
      values += formatUserVarDomoticz(0);        // ALTITUDE= Not used at the moment, can be 0
      break;
    case Sensor_VType::SENSOR_TYPE_TEMP_EMPTY_BARO:

      // temp + bar + bar_fore, used for BMP280
      // http://www.domoticz.com/wiki/Domoticz_API/JSON_URL%27s#Temperature.2Fbarometer
      values  = formatUserVarDomoticz(event, 0); // TEMP = Temperature
      values += formatUserVarDomoticz(event, 2); // BAR = Barometric pressure
      values += formatUserVarDomoticz(0);        // BAR_FOR = Barometer forecast
      values += formatUserVarDomoticz(0);        // ALTITUDE= Not used at the moment, can be 0
      break;
    case Sensor_VType::SENSOR_TYPE_TRIPLE:
      values  = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 1);
      values += formatUserVarDomoticz(event, 2);
      break;
    case Sensor_VType::SENSOR_TYPE_QUAD:
      values  = formatUserVarDomoticz(event, 0);
      values += formatUserVarDomoticz(event, 1);
      values += formatUserVarDomoticz(event, 2);
      values += formatUserVarDomoticz(event, 3);
      break;
    case Sensor_VType::SENSOR_TYPE_WIND:

      // WindDir in degrees; WindDir as text; Wind speed average ; Wind speed gust; 0
      // http://www.domoticz.com/wiki/Domoticz_API/JSON_URL%27s#Wind
      values  = formatUserVarDomoticz(event, 0);          // WB = Wind bearing (0-359)
      values += getBearing(UserVar[event->BaseVarIndex]); // WD = Wind direction (S, SW, NNW, etc.)
      values += ';';                                      // Needed after getBearing
      // Domoticz expects the wind speed in (m/s * 10)
      values += toString((UserVar[event->BaseVarIndex + 1] * 10), Cache.getTaskDeviceValueDecimals(event->TaskIndex, 1));
      values += ';';                                      // WS = 10 * Wind speed [m/s]
      values += toString((UserVar[event->BaseVarIndex + 2] * 10), Cache.getTaskDeviceValueDecimals(event->TaskIndex, 2));
      values += ';';                                      // WG = 10 * Gust [m/s]
      values += formatUserVarDomoticz(0);                 // Temperature
      values += formatUserVarDomoticz(0);                 // Temperature Windchill
      break;
    case Sensor_VType::SENSOR_TYPE_SWITCH:
    case Sensor_VType::SENSOR_TYPE_DIMMER:

      // Too specific for HTTP/MQTT
      break;
    case Sensor_VType::SENSOR_TYPE_STRING:
      values = event->String2;
      break;
    default:
    {
      # ifndef BUILD_NO_DEBUG

      if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
        String log = F("Domoticz Controller: Not yet implemented sensor type: ");
        log += static_cast<uint8_t>(event->sensorType);
        log += F(" idx: ");
        log += event->idx;
        addLogMove(LOG_LEVEL_ERROR, log);
      }
      # endif // ifndef BUILD_NO_DEBUG
      break;
    }
  }

  // Now strip trailing semi colon.
  int index_last_char = values.length() - 1;

  if ((index_last_char > 0) && (values.charAt(index_last_char) == ';')) {
    values.setCharAt(index_last_char, ' ');
  }
  values.trim();
  {
    # ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F(" Domoticz: Sensortype: ");
      log += static_cast<uint8_t>(event->sensorType);
      log += F(" idx: ");
      log += event->idx;
      log += F(" values: ");
      log += values;
      addLogMove(LOG_LEVEL_INFO, log);
    }
    # endif // ifndef BUILD_NO_DEBUG
  }
  return values;
}

# ifdef USES_C002
#  include <ArduinoJson.h>

bool deserializeDomoticzJson(const String& json,
                             unsigned int& idx, float& nvalue, long& nvaluealt,
                             String& svalue1, String& switchtype) {
  DynamicJsonDocument root(512);

  deserializeJson(root, json);

  if (root.isNull()) {
    return false;
  }
  idx       = root[F("idx")];
  nvalue    = root[F("nvalue")];
  nvaluealt = root[F("nvalue")];

  // const char* name = root["name"]; // Not used
  // const char* svalue = root["svalue"]; // Not used
  const char *svalue1_c = root[F("svalue1")];

  if (svalue1_c != nullptr) {
    svalue1 = svalue1_c;
  }

  // const char* svalue2 = root["svalue2"]; // Not used
  // const char* svalue3 = root["svalue3"]; // Not used
  const char *switchtype_c = root[F("switchType")]; // Expect "On/Off" or "dimmer"

  // FIXME TD-er: Is this compare even useful?
  // nvalue is already assigned the same value as nvaluealt and not changed since.
  if (nvalue == 0) {
    nvalue = nvaluealt;
  }

  if (switchtype_c == nullptr) {
    switchtype = '?';
  } else {
    switchtype = switchtype_c;
  }
  return true;
}

String serializeDomoticzJson(struct EventStruct *event)
{
  String json;
  {
    json += '{';
    json += to_json_object_value(F("idx"), String(event->idx));
    json += ',';
    json += to_json_object_value(F("RSSI"), String(mapRSSItoDomoticz()));
    #  if FEATURE_ADC_VCC
    json += ',';
    json += to_json_object_value(F("Battery"), String(mapVccToDomoticz()));
    #  endif // if FEATURE_ADC_VCC

    const Sensor_VType sensorType = event->getSensorType();

    switch (sensorType)
    {
      case Sensor_VType::SENSOR_TYPE_SWITCH:
        json += ',';
        json += to_json_object_value(F("command"), F("switchlight"));

        if (essentiallyEqual(UserVar[event->BaseVarIndex], 0.0f)) {
          json += ',';
          json += to_json_object_value(F("switchcmd"), F("Off"));
        }
        else {
          json += ',';
          json += to_json_object_value(F("switchcmd"), F("On"));
        }
        break;
      case Sensor_VType::SENSOR_TYPE_DIMMER:
        json += ',';
        json += to_json_object_value(F("command"), F("switchlight"));

        if (essentiallyEqual(UserVar[event->BaseVarIndex], 0.0f)) {
          json += ',';
          json += to_json_object_value(F("switchcmd"), F("Off"));
        }
        else {
          json += ',';
          json += to_json_object_value(F("Set%20Level"), toString(UserVar[event->BaseVarIndex], 2));
        }
        break;

      case Sensor_VType::SENSOR_TYPE_SINGLE:
      case Sensor_VType::SENSOR_TYPE_LONG:
      case Sensor_VType::SENSOR_TYPE_DUAL:
      case Sensor_VType::SENSOR_TYPE_TRIPLE:
      case Sensor_VType::SENSOR_TYPE_QUAD:
      case Sensor_VType::SENSOR_TYPE_TEMP_HUM:
      case Sensor_VType::SENSOR_TYPE_TEMP_BARO:
      case Sensor_VType::SENSOR_TYPE_TEMP_EMPTY_BARO:
      case Sensor_VType::SENSOR_TYPE_TEMP_HUM_BARO:
      case Sensor_VType::SENSOR_TYPE_WIND:
      case Sensor_VType::SENSOR_TYPE_STRING:
      default:
        json += ',';
        json += to_json_object_value(F("nvalue"), F("0"));
        json += ',';
        json += to_json_object_value(F("svalue"), formatDomoticzSensorType(event), true);
        break;
    }
    json += '}';
  }

  return json;
}

# endif // ifdef USES_C002

#endif  // if FEATURE_DOMOTICZ

#include "../Helpers/RulesHelper.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/Settings.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/StringProvider.h"

/********************************************************************************************\
   Test for common mistake
   Return true if mistake was found (and corrected)
 \*********************************************************************************************/
bool rules_replace_common_mistakes(const String& from, const String& to, String& line)
{
  if (line.indexOf(from) == -1) {
    return false; // Nothing replaced
  }

  if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
    String log;

    if (log.reserve(32 + from.length() + to.length() + line.length())) {
      log  = F("Rules (Syntax Error, auto-corrected): '");
      log += from;
      log += F("' => '");
      log += to;
      log += F("' in: '");
      log += line;
      log += '\'';
      addLogMove(LOG_LEVEL_ERROR, log);
    }
  }
  line.replace(from, to);
  return true;
}

/********************************************************************************************\
   Check for common mistakes
   Return true if nothing strange found
 \*********************************************************************************************/
bool check_rules_line_user_errors(String& line)
{
  bool res = true;

  if (rules_replace_common_mistakes(F("if["), F("if ["), line)) {
    res = false;
  }

  if (rules_replace_common_mistakes(F("if%"), F("if %"), line)) {
    res = false;
  }

  return res;
}

/********************************************************************************************\
   Strip comment from the line.
   Return true when comment was stripped.
 \*********************************************************************************************/
bool rules_strip_trailing_comments(String& line)
{
  // Strip trailing comments
  int comment = line.indexOf(F("//"));

  if (comment >= 0) {
    line = line.substring(0, comment);
    line.trim();
    return true;
  }
  return false;
}

RulesHelperClass::RulesHelperClass()
{}

RulesHelperClass::~RulesHelperClass()
{
  closeAllFiles();
}

bool RulesHelperClass::findMatchingRule(const String& event, String& filename, size_t& pos)
{
  if (!_eventCache.isInitialized()) {
    init();
  }
  RulesEventCache_vector::const_iterator it = _eventCache.findMatchingRule(event, Settings.EnableRulesEventReorder());

  if (it == _eventCache.end()) { return false; }

  filename = it->_filename;
  pos      = it->_posInFile;
  return true;
}

void RulesHelperClass::init()
{
  if (_eventCache.isInitialized()) { return; }

  // Read all files to populate caches.

  for (uint8_t x = 0; x < RULESETS_MAX; x++) {
    // Read files
    const String filename        = getRulesFileName(x);
    size_t pos                   = 0;
    bool   moreAvailable         = true;
    const bool searchNextOnBlock = false;

    while (moreAvailable) {
      const size_t pos_start_line = pos;
      const String rulesLine      = readLn(filename, pos, moreAvailable, searchNextOnBlock);

      if (_eventCache.addLine(
            rulesLine,
            filename,
            pos_start_line)) {
#ifndef BUILD_NO_DEBUG

        if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
          String log = F("Cache rules event: ");
          log += filename;
          log += F(" pos: ");
          log += pos_start_line;
          log += ' ';
          log += rulesLine;
          addLogMove(LOG_LEVEL_DEBUG, log);
        }
#endif // ifndef BUILD_NO_DEBUG
      }
    }
  }
  _eventCache.initialize();
}

void RulesHelperClass::closeAllFiles() {
  for (auto it = _fileHandleMap.begin(); it != _fileHandleMap.end();) {
    #ifdef CACHE_RULES_IN_MEMORY
    it = _fileHandleMap.erase(it);
    #else // ifdef CACHE_RULES_IN_MEMORY
    it->second.close();
    it = _fileHandleMap.erase(it);
    #endif // ifdef CACHE_RULES_IN_MEMORY
  }
  _eventCache.clear();
}

#ifndef CACHE_RULES_IN_MEMORY
size_t RulesHelperClass::read(const String& filename, size_t& pos, uint8_t *buffer, size_t length)
{
  if (!Settings.UseRules || !fileExists(filename)) {
    return 0;
  }
  auto it = _fileHandleMap.begin();

  if (it != _fileHandleMap.end()) {
    if (!it->first.equals(filename)) {
      // Switched to a new file.
      // Close this handle first to make sure we don't keep too many file handles open.
      it->second.close();
      _fileHandleMap.erase(it);
      it = _fileHandleMap.end();
    }
  }

  if (it == _fileHandleMap.end()) {
    // No open file handle found, so try to open it.
    _fileHandleMap.emplace(filename, tryOpenFile(filename, "r+"));
    it = _fileHandleMap.find(filename);
  }

  if (!it->second) {
    return 0;
  }

  if (it->second.position() != pos) {
    it->second.seek(pos);
  }
  const size_t ret = it->second.read(buffer, length);

  pos = it->second.position();
  return ret;
}

#endif // ifndef CACHE_RULES_IN_MEMORY

bool RulesHelperClass::addChar(char c, String& line,   bool& firstNonSpaceRead,  bool& commentFound)
{
  switch (c)
  {
    case '\n':
    {
      // Line end, parse rule
      line.trim();

      if ((line.length() > 0) && !line.startsWith(F("//"))) {
        if (commentFound) {
          rules_strip_trailing_comments(line);
        }
        check_rules_line_user_errors(line);
        return true;
      }

      // Prepare for new line
      line.clear();
      firstNonSpaceRead = false;
      commentFound      = false;
      break;
    }
    case '\r': // Just skip this character
      break;
    case '\t': // tab
    case ' ':  // space
    {
      // Strip leading spaces.
      if (firstNonSpaceRead) {
        line += ' ';
      }
      break;
    }
    case '/':
    {
      if (!commentFound) {
        line += '/';

        if (line.endsWith(F("//"))) {
          // consider the rest of the line a comment
          commentFound = true;
        }
      }
      break;
    }
    default: // Any other character
    {
      firstNonSpaceRead = true;

      if (!commentFound) {
        line += c;
      }
      break;
    }
  }
  return false;
}

#ifdef CACHE_RULES_IN_MEMORY
String RulesHelperClass::readLn(const String& filename,
                                size_t      & pos,
                                bool        & moreAvailable,
                                bool          searchNextOnBlock)
{
  moreAvailable = false;
  auto it = _fileHandleMap.find(filename);

  if (it == _fileHandleMap.end()) {
    // Read lines from the file
    fs::File f = tryOpenFile(filename, "r+");

    if (f) {
      RulesLines lines;
      String     tmpStr;
      bool firstNonSpaceRead = false;
      bool commentFound      = false;

      // Keep track of which line we're reading for the event cache.
      size_t readPos = 0;

      while (f.available()) {
        if (addChar(char(f.read()), tmpStr, firstNonSpaceRead, commentFound)) {
          lines.push_back(tmpStr);
          ++readPos;

          firstNonSpaceRead = false;
          commentFound      = false;
          tmpStr.clear();
        }
      }

      if (tmpStr.length() > 0) {
        rules_strip_trailing_comments(tmpStr);
        check_rules_line_user_errors(tmpStr);
        lines.push_back(tmpStr);
        tmpStr.clear();
      }
# ifndef BUILD_NO_DEBUG

      if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
        String log = F("Rules : Read ");
        log += lines.size();
        log += F(" lines from ");
        log += filename;
        addLogMove(LOG_LEVEL_DEBUG, log);
      }
# endif // ifndef BUILD_NO_DEBUG
      _fileHandleMap.emplace(std::make_pair(filename, std::move(lines)));
      it = _fileHandleMap.find(filename);
    }
  }

  if (it != _fileHandleMap.end()) {
    while (pos < it->second.size()) {
      ++pos;
      moreAvailable = pos < it->second.size();

      if (!searchNextOnBlock ||
          it->second[pos - 1].substring(0, 3).equalsIgnoreCase(F("on "))) {
        return it->second[pos - 1];
      }
    }
  }
  return EMPTY_STRING;
}

#else // ifdef CACHE_RULES_IN_MEMORY

String RulesHelperClass::readLn(const String& filename,
                                size_t      & pos,
                                bool        & moreAvailable,
                                bool          searchNextOnBlock)
{
  std::vector<uint8_t> buf;

  buf.resize(RULES_BUFFER_SIZE);

  bool firstNonSpaceRead = false;
  bool commentFound      = false;

  // Try to get the best possible estimate on line length based on earlier parsing of the rules.
  static size_t longestLineSize = RULES_BUFFER_SIZE;
  String line;

  line.reserve(longestLineSize);

  bool done = false;

  while (!done) {
    const size_t startPos = pos;
    int len               = read(filename, pos, &buf[0], RULES_BUFFER_SIZE);
    moreAvailable = len != 0;

    if (!moreAvailable) { done = true; }

    for (int x = 0; x < len; x++) {
      int data = buf[x];

      if (addChar(char(data), line, firstNonSpaceRead, commentFound)) {
        if (line.length() > longestLineSize) {
          longestLineSize = line.length();
        }

        // A line may end on every position in the buffer,
        // so we must make sure the position is reflecting the end of the line.
        pos = startPos + x;

        if (!searchNextOnBlock ||
            line.substring(0, 3).equalsIgnoreCase(F("on ")))
        {
          done = true;

          return line;
        } else {
          // Not starting with "on " which we need, so continue to search for a matching line
          line.clear();
        }
      }
    }
  }
  rules_strip_trailing_comments(line);
  check_rules_line_user_errors(line);
  return line;
}

#endif // ifdef CACHE_RULES_IN_MEMORY

#include "../Helpers/ESPEasy_math.h"

#include <cmath>

int maxNrDecimals_double(const double& value)
{
  int res       = ESPEASY_DOUBLE_NR_DECIMALS;
  double factor = 1;

  while ((value / factor) > 10 && res > 2) {
    factor *= 10.0;
    --res;
  }
  return res;
}

bool approximatelyEqual(const double& a, const double& b, double epsilon)
{
  return std::abs(a - b) <= ((std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);
}

bool approximatelyEqual(const float& a, const float& b, float epsilon)
{
  return std::abs(a - b) <= ((std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);
}

bool definitelyGreaterThan(const double& a, const double& b, double epsilon)
{
  return (a - b) > ((std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);
}

bool definitelyGreaterThan(const float& a, const float& b, float epsilon)
{
  return (a - b) > ((std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);
}

bool definitelyLessThan(const double& a, const double& b, double epsilon)
{
  return (b - a) > ((std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);
}

bool definitelyLessThan(const float& a, const float& b, float epsilon)
{
  return (b - a) > ((std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);
}

bool essentiallyEqual(const double& a, const double& b, double epsilon)
{
  return std::abs(a - b) <= ((std::abs(a) > std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);
}

bool essentiallyEqual(const float& a, const float& b, float epsilon)
{
  return std::abs(a - b) <= ((std::abs(a) > std::abs(b) ? std::abs(b) : std::abs(a)) * epsilon);
}

#include "../Helpers/_NPlugin_init.h"

#if FEATURE_NOTIFIER

#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../DataStructs/TimingStats.h"

#include "../DataTypes/ESPEasy_plugin_functions.h"

#include "../Globals/NPlugins.h"
#include "../Globals/Protocol.h"
#include "../Globals/Settings.h"

#include "../Helpers/Misc.h"

// ********************************************************************************
// Initialize all Controller NPlugins that where defined earlier
// and initialize the function call pointer into the CNPlugin array
// ********************************************************************************

// Uncrustify must not be used on macros, so turn it off.
// *INDENT-OFF*
#define ADDNPLUGIN(NNN, ID) if (addNPlugin(ID, x)) { NPlugin_ptr[x++] = &NPlugin_##NNN; }
// Uncrustify must not be used on macros, but we're now done, so turn Uncrustify on again.
// *INDENT-ON*

void NPluginInit()
{
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif
//  ProtocolIndex_to_NPlugin_id[NPLUGIN_MAX] = INVALID_N_PLUGIN_ID;
  uint8_t x;

  // Clear pointer table for all plugins
  for (x = 0; x < NPLUGIN_MAX; x++)
  {
    NPlugin_ptr[x] = nullptr;
    NPlugin_id[x]  = INVALID_N_PLUGIN_ID;
//    ProtocolIndex_to_NPlugin_id[x] = INVALID_N_PLUGIN_ID;
    // Do not initialize NPlugin_id_to_ProtocolIndex[x] to an invalid value. (it is map)
  }

  x = 0;

#ifdef USES_N001
  ADDNPLUGIN(001, 1)
#endif

#ifdef USES_N002
  ADDNPLUGIN(002, 2)
#endif

#ifdef USES_N003
  ADDNPLUGIN(003, 3)
#endif

#ifdef USES_N004
  ADDNPLUGIN(004, 4)
#endif

#ifdef USES_N005
  ADDNPLUGIN(005, 5)
#endif

#ifdef USES_N006
  ADDNPLUGIN(006, 6)
#endif

#ifdef USES_N007
  ADDNPLUGIN(007, 7)
#endif

#ifdef USES_N008
  ADDNPLUGIN(008, 8)
#endif

#ifdef USES_N009
  ADDNPLUGIN(009, 9)
#endif

#ifdef USES_N010
  ADDNPLUGIN(010, 10)
#endif

#ifdef USES_N011
  ADDNPLUGIN(011, 11)
#endif

#ifdef USES_N012
  ADDNPLUGIN(012, 12)
#endif

#ifdef USES_N013
  ADDNPLUGIN(013, 13)
#endif

#ifdef USES_N014
  ADDNPLUGIN(014, 14)
#endif

#ifdef USES_N015
  ADDNPLUGIN(015, 15)
#endif

#ifdef USES_N016
  ADDNPLUGIN(016, 16)
#endif

#ifdef USES_N017
  ADDNPLUGIN(017, 17)
#endif

#ifdef USES_N018
  ADDNPLUGIN(018, 18)
#endif

#ifdef USES_N019
  ADDNPLUGIN(019, 19)
#endif

#ifdef USES_N020
  ADDNPLUGIN(020, 20)
#endif

#ifdef USES_N021
  ADDNPLUGIN(021, 21)
#endif

#ifdef USES_N022
  ADDNPLUGIN(022, 22)
#endif

#ifdef USES_N023
  ADDNPLUGIN(023, 23)
#endif

#ifdef USES_N024
  ADDNPLUGIN(024, 24)
#endif

#ifdef USES_N025
  ADDNPLUGIN(025, 25)
#endif

#ifdef USES_N026
  ADDNPLUGIN(026, 26)
#endif

#ifdef USES_N027
  ADDNPLUGIN(027, 27)
#endif

#ifdef USES_N028
  ADDNPLUGIN(028, 28)
#endif

#ifdef USES_N029
  ADDNPLUGIN(029, 29)
#endif

#ifdef USES_N030
  ADDNPLUGIN(030, 30)
#endif

#ifdef USES_N031
  ADDNPLUGIN(031, 31)
#endif

#ifdef USES_N032
  ADDNPLUGIN(032, 32)
#endif

#ifdef USES_N033
  ADDNPLUGIN(033, 33)
#endif

#ifdef USES_N034
  ADDNPLUGIN(034, 34)
#endif

#ifdef USES_N035
  ADDNPLUGIN(035, 35)
#endif

#ifdef USES_N036
  ADDNPLUGIN(036, 36)
#endif

#ifdef USES_N037
  ADDNPLUGIN(037, 37)
#endif

#ifdef USES_N038
  ADDNPLUGIN(038, 38)
#endif

#ifdef USES_N039
  ADDNPLUGIN(039, 39)
#endif

#ifdef USES_N040
  ADDNPLUGIN(040, 40)
#endif

#ifdef USES_N041
  ADDNPLUGIN(041, 41)
#endif

#ifdef USES_N042
  ADDNPLUGIN(042, 42)
#endif

#ifdef USES_N043
  ADDNPLUGIN(043, 43)
#endif

#ifdef USES_N044
  ADDNPLUGIN(044, 44)
#endif

#ifdef USES_N045
  ADDNPLUGIN(045, 45)
#endif

#ifdef USES_N046
  ADDNPLUGIN(046, 46)
#endif

#ifdef USES_N047
  ADDNPLUGIN(047, 47)
#endif

#ifdef USES_N048
  ADDNPLUGIN(048, 48)
#endif

#ifdef USES_N049
  ADDNPLUGIN(049, 49)
#endif

#ifdef USES_N050
  ADDNPLUGIN(050, 50)
#endif

#ifdef USES_N051
  ADDNPLUGIN(051, 51)
#endif

#ifdef USES_N052
  ADDNPLUGIN(052, 52)
#endif

#ifdef USES_N053
  ADDNPLUGIN(053, 53)
#endif

#ifdef USES_N054
  ADDNPLUGIN(054, 54)
#endif

#ifdef USES_N055
  ADDNPLUGIN(055, 55)
#endif

#ifdef USES_N056
  ADDNPLUGIN(056, 56)
#endif

#ifdef USES_N057
  ADDNPLUGIN(057, 57)
#endif

#ifdef USES_N058
  ADDNPLUGIN(058, 58)
#endif

#ifdef USES_N059
  ADDNPLUGIN(059, 59)
#endif

#ifdef USES_N060
  ADDNPLUGIN(060, 60)
#endif

#ifdef USES_N061
  ADDNPLUGIN(061, 61)
#endif

#ifdef USES_N062
  ADDNPLUGIN(062, 62)
#endif

#ifdef USES_N063
  ADDNPLUGIN(063, 63)
#endif

#ifdef USES_N064
  ADDNPLUGIN(064, 64)
#endif

#ifdef USES_N065
  ADDNPLUGIN(065, 65)
#endif

#ifdef USES_N066
  ADDNPLUGIN(066, 66)
#endif

#ifdef USES_N067
  ADDNPLUGIN(067, 67)
#endif

#ifdef USES_N068
  ADDNPLUGIN(068, 68)
#endif

#ifdef USES_N069
  ADDNPLUGIN(069, 69)
#endif

#ifdef USES_N070
  ADDNPLUGIN(070, 70)
#endif

#ifdef USES_N071
  ADDNPLUGIN(071, 71)
#endif

#ifdef USES_N072
  ADDNPLUGIN(072, 72)
#endif

#ifdef USES_N073
  ADDNPLUGIN(073, 73)
#endif

#ifdef USES_N074
  ADDNPLUGIN(074, 74)
#endif

#ifdef USES_N075
  ADDNPLUGIN(075, 75)
#endif

#ifdef USES_N076
  ADDNPLUGIN(076, 76)
#endif

#ifdef USES_N077
  ADDNPLUGIN(077, 77)
#endif

#ifdef USES_N078
  ADDNPLUGIN(078, 78)
#endif

#ifdef USES_N079
  ADDNPLUGIN(079, 79)
#endif

#ifdef USES_N080
  ADDNPLUGIN(080, 80)
#endif

#ifdef USES_N081
  ADDNPLUGIN(081, 81)
#endif

#ifdef USES_N082
  ADDNPLUGIN(082, 82)
#endif

#ifdef USES_N083
  ADDNPLUGIN(083, 83)
#endif

#ifdef USES_N084
  ADDNPLUGIN(084, 84)
#endif

#ifdef USES_N085
  ADDNPLUGIN(085, 85)
#endif

#ifdef USES_N086
  ADDNPLUGIN(086, 86)
#endif

#ifdef USES_N087
  ADDNPLUGIN(087, 87)
#endif

#ifdef USES_N088
  ADDNPLUGIN(088, 88)
#endif

#ifdef USES_N089
  ADDNPLUGIN(089, 89)
#endif

#ifdef USES_N090
  ADDNPLUGIN(090, 90)
#endif

#ifdef USES_N091
  ADDNPLUGIN(091, 91)
#endif

#ifdef USES_N092
  ADDNPLUGIN(092, 92)
#endif

#ifdef USES_N093
  ADDNPLUGIN(093, 93)
#endif

#ifdef USES_N094
  ADDNPLUGIN(094, 94)
#endif

#ifdef USES_N095
  ADDNPLUGIN(095, 95)
#endif

#ifdef USES_N096
  ADDNPLUGIN(096, 96)
#endif

#ifdef USES_N097
  ADDNPLUGIN(097, 97)
#endif

#ifdef USES_N098
  ADDNPLUGIN(098, 98)
#endif

#ifdef USES_N099
  ADDNPLUGIN(099, 99)
#endif

#ifdef USES_N100
  ADDNPLUGIN(100, 100)
#endif

#ifdef USES_N101
  ADDNPLUGIN(101, 101)
#endif

#ifdef USES_N102
  ADDNPLUGIN(102, 102)
#endif

#ifdef USES_N103
  ADDNPLUGIN(103, 103)
#endif

#ifdef USES_N104
  ADDNPLUGIN(104, 104)
#endif

#ifdef USES_N105
  ADDNPLUGIN(105, 105)
#endif

#ifdef USES_N106
  ADDNPLUGIN(106, 106)
#endif

#ifdef USES_N107
  ADDNPLUGIN(107, 107)
#endif

#ifdef USES_N108
  ADDNPLUGIN(108, 108)
#endif

#ifdef USES_N109
  ADDNPLUGIN(109, 109)
#endif

#ifdef USES_N110
  ADDNPLUGIN(110, 110)
#endif

#ifdef USES_N111
  ADDNPLUGIN(111, 111)
#endif

#ifdef USES_N112
  ADDNPLUGIN(112, 112)
#endif

#ifdef USES_N113
  ADDNPLUGIN(113, 113)
#endif

#ifdef USES_N114
  ADDNPLUGIN(114, 114)
#endif

#ifdef USES_N115
  ADDNPLUGIN(115, 115)
#endif

#ifdef USES_N116
  ADDNPLUGIN(116, 116)
#endif

#ifdef USES_N117
  ADDNPLUGIN(117, 117)
#endif

#ifdef USES_N118
  ADDNPLUGIN(118, 118)
#endif

#ifdef USES_N119
  ADDNPLUGIN(119, 119)
#endif

#ifdef USES_N120
  ADDNPLUGIN(120, 120)
#endif

#ifdef USES_N121
  ADDNPLUGIN(121, 121)
#endif

#ifdef USES_N122
  ADDNPLUGIN(122, 122)
#endif

#ifdef USES_N123
  ADDNPLUGIN(123, 123)
#endif

#ifdef USES_N124
  ADDNPLUGIN(124, 124)
#endif

#ifdef USES_N125
  ADDNPLUGIN(125, 125)
#endif

#ifdef USES_N126
  ADDNPLUGIN(126, 126)
#endif

#ifdef USES_N127
  ADDNPLUGIN(127, 127)
#endif

#ifdef USES_N128
  ADDNPLUGIN(128, 128)
#endif

#ifdef USES_N129
  ADDNPLUGIN(129, 129)
#endif

#ifdef USES_N130
  ADDNPLUGIN(130, 130)
#endif

#ifdef USES_N131
  ADDNPLUGIN(131, 131)
#endif

#ifdef USES_N132
  ADDNPLUGIN(132, 132)
#endif

#ifdef USES_N133
  ADDNPLUGIN(133, 133)
#endif

#ifdef USES_N134
  ADDNPLUGIN(134, 134)
#endif

#ifdef USES_N135
  ADDNPLUGIN(135, 135)
#endif

#ifdef USES_N136
  ADDNPLUGIN(136, 136)
#endif

#ifdef USES_N137
  ADDNPLUGIN(137, 137)
#endif

#ifdef USES_N138
  ADDNPLUGIN(138, 138)
#endif

#ifdef USES_N139
  ADDNPLUGIN(139, 139)
#endif

#ifdef USES_N140
  ADDNPLUGIN(140, 140)
#endif

#ifdef USES_N141
  ADDNPLUGIN(141, 141)
#endif

#ifdef USES_N142
  ADDNPLUGIN(142, 142)
#endif

#ifdef USES_N143
  ADDNPLUGIN(143, 143)
#endif

#ifdef USES_N144
  ADDNPLUGIN(144, 144)
#endif

#ifdef USES_N145
  ADDNPLUGIN(145, 145)
#endif

#ifdef USES_N146
  ADDNPLUGIN(146, 146)
#endif

#ifdef USES_N147
  ADDNPLUGIN(147, 147)
#endif

#ifdef USES_N148
  ADDNPLUGIN(148, 148)
#endif

#ifdef USES_N149
  ADDNPLUGIN(149, 149)
#endif

#ifdef USES_N150
  ADDNPLUGIN(150, 150)
#endif

#ifdef USES_N151
  ADDNPLUGIN(151, 151)
#endif

#ifdef USES_N152
  ADDNPLUGIN(152, 152)
#endif

#ifdef USES_N153
  ADDNPLUGIN(153, 153)
#endif

#ifdef USES_N154
  ADDNPLUGIN(154, 154)
#endif

#ifdef USES_N155
  ADDNPLUGIN(155, 155)
#endif

#ifdef USES_N156
  ADDNPLUGIN(156, 156)
#endif

#ifdef USES_N157
  ADDNPLUGIN(157, 157)
#endif

#ifdef USES_N158
  ADDNPLUGIN(158, 158)
#endif

#ifdef USES_N159
  ADDNPLUGIN(159, 159)
#endif

#ifdef USES_N160
  ADDNPLUGIN(160, 160)
#endif

#ifdef USES_N161
  ADDNPLUGIN(161, 161)
#endif

#ifdef USES_N162
  ADDNPLUGIN(162, 162)
#endif

#ifdef USES_N163
  ADDNPLUGIN(163, 163)
#endif

#ifdef USES_N164
  ADDNPLUGIN(164, 164)
#endif

#ifdef USES_N165
  ADDNPLUGIN(165, 165)
#endif

#ifdef USES_N166
  ADDNPLUGIN(166, 166)
#endif

#ifdef USES_N167
  ADDNPLUGIN(167, 167)
#endif

#ifdef USES_N168
  ADDNPLUGIN(168, 168)
#endif

#ifdef USES_N169
  ADDNPLUGIN(169, 169)
#endif

#ifdef USES_N170
  ADDNPLUGIN(170, 170)
#endif

#ifdef USES_N171
  ADDNPLUGIN(171, 171)
#endif

#ifdef USES_N172
  ADDNPLUGIN(172, 172)
#endif

#ifdef USES_N173
  ADDNPLUGIN(173, 173)
#endif

#ifdef USES_N174
  ADDNPLUGIN(174, 174)
#endif

#ifdef USES_N175
  ADDNPLUGIN(175, 175)
#endif

#ifdef USES_N176
  ADDNPLUGIN(176, 176)
#endif

#ifdef USES_N177
  ADDNPLUGIN(177, 177)
#endif

#ifdef USES_N178
  ADDNPLUGIN(178, 178)
#endif

#ifdef USES_N179
  ADDNPLUGIN(179, 179)
#endif

#ifdef USES_N180
  ADDNPLUGIN(180, 180)
#endif

#ifdef USES_N181
  ADDNPLUGIN(181, 181)
#endif

#ifdef USES_N182
  ADDNPLUGIN(182, 182)
#endif

#ifdef USES_N183
  ADDNPLUGIN(183, 183)
#endif

#ifdef USES_N184
  ADDNPLUGIN(184, 184)
#endif

#ifdef USES_N185
  ADDNPLUGIN(185, 185)
#endif

#ifdef USES_N186
  ADDNPLUGIN(186, 186)
#endif

#ifdef USES_N187
  ADDNPLUGIN(187, 187)
#endif

#ifdef USES_N188
  ADDNPLUGIN(188, 188)
#endif

#ifdef USES_N189
  ADDNPLUGIN(189, 189)
#endif

#ifdef USES_N190
  ADDNPLUGIN(190, 190)
#endif

#ifdef USES_N191
  ADDNPLUGIN(191, 191)
#endif

#ifdef USES_N192
  ADDNPLUGIN(192, 192)
#endif

#ifdef USES_N193
  ADDNPLUGIN(193, 193)
#endif

#ifdef USES_N194
  ADDNPLUGIN(194, 194)
#endif

#ifdef USES_N195
  ADDNPLUGIN(195, 195)
#endif

#ifdef USES_N196
  ADDNPLUGIN(196, 196)
#endif

#ifdef USES_N197
  ADDNPLUGIN(197, 197)
#endif

#ifdef USES_N198
  ADDNPLUGIN(198, 198)
#endif

#ifdef USES_N199
  ADDNPLUGIN(199, 199)
#endif

#ifdef USES_N200
  ADDNPLUGIN(200, 200)
#endif

#ifdef USES_N201
  ADDNPLUGIN(201, 201)
#endif

#ifdef USES_N202
  ADDNPLUGIN(202, 202)
#endif

#ifdef USES_N203
  ADDNPLUGIN(203, 203)
#endif

#ifdef USES_N204
  ADDNPLUGIN(204, 204)
#endif

#ifdef USES_N205
  ADDNPLUGIN(205, 205)
#endif

#ifdef USES_N206
  ADDNPLUGIN(206, 206)
#endif

#ifdef USES_N207
  ADDNPLUGIN(207, 207)
#endif

#ifdef USES_N208
  ADDNPLUGIN(208, 208)
#endif

#ifdef USES_N209
  ADDNPLUGIN(209, 209)
#endif

#ifdef USES_N210
  ADDNPLUGIN(210, 210)
#endif

#ifdef USES_N211
  ADDNPLUGIN(211, 211)
#endif

#ifdef USES_N212
  ADDNPLUGIN(212, 212)
#endif

#ifdef USES_N213
  ADDNPLUGIN(213, 213)
#endif

#ifdef USES_N214
  ADDNPLUGIN(214, 214)
#endif

#ifdef USES_N215
  ADDNPLUGIN(215, 215)
#endif

#ifdef USES_N216
  ADDNPLUGIN(216, 216)
#endif

#ifdef USES_N217
  ADDNPLUGIN(217, 217)
#endif

#ifdef USES_N218
  ADDNPLUGIN(218, 218)
#endif

#ifdef USES_N219
  ADDNPLUGIN(219, 219)
#endif

#ifdef USES_N220
  ADDNPLUGIN(220, 220)
#endif

#ifdef USES_N221
  ADDNPLUGIN(221, 221)
#endif

#ifdef USES_N222
  ADDNPLUGIN(222, 222)
#endif

#ifdef USES_N223
  ADDNPLUGIN(223, 223)
#endif

#ifdef USES_N224
  ADDNPLUGIN(224, 224)
#endif

#ifdef USES_N225
  ADDNPLUGIN(225, 225)
#endif

#ifdef USES_N226
  ADDNPLUGIN(226, 226)
#endif

#ifdef USES_N227
  ADDNPLUGIN(227, 227)
#endif

#ifdef USES_N228
  ADDNPLUGIN(228, 228)
#endif

#ifdef USES_N229
  ADDNPLUGIN(229, 229)
#endif

#ifdef USES_N230
  ADDNPLUGIN(230, 230)
#endif

#ifdef USES_N231
  ADDNPLUGIN(231, 231)
#endif

#ifdef USES_N232
  ADDNPLUGIN(232, 232)
#endif

#ifdef USES_N233
  ADDNPLUGIN(233, 233)
#endif

#ifdef USES_N234
  ADDNPLUGIN(234, 234)
#endif

#ifdef USES_N235
  ADDNPLUGIN(235, 235)
#endif

#ifdef USES_N236
  ADDNPLUGIN(236, 236)
#endif

#ifdef USES_N237
  ADDNPLUGIN(237, 237)
#endif

#ifdef USES_N238
  ADDNPLUGIN(238, 238)
#endif

#ifdef USES_N239
  ADDNPLUGIN(239, 239)
#endif

#ifdef USES_N240
  ADDNPLUGIN(240, 240)
#endif

#ifdef USES_N241
  ADDNPLUGIN(241, 241)
#endif

#ifdef USES_N242
  ADDNPLUGIN(242, 242)
#endif

#ifdef USES_N243
  ADDNPLUGIN(243, 243)
#endif

#ifdef USES_N244
  ADDNPLUGIN(244, 244)
#endif

#ifdef USES_N245
  ADDNPLUGIN(245, 245)
#endif

#ifdef USES_N246
  ADDNPLUGIN(246, 246)
#endif

#ifdef USES_N247
  ADDNPLUGIN(247, 247)
#endif

#ifdef USES_N248
  ADDNPLUGIN(248, 248)
#endif

#ifdef USES_N249
  ADDNPLUGIN(249, 249)
#endif

#ifdef USES_N250
  ADDNPLUGIN(250, 250)
#endif

#ifdef USES_N251
  ADDNPLUGIN(251, 251)
#endif

#ifdef USES_N252
  ADDNPLUGIN(252, 252)
#endif

#ifdef USES_N253
  ADDNPLUGIN(253, 253)
#endif

#ifdef USES_N254
  ADDNPLUGIN(254, 254)
#endif

#ifdef USES_N255
  ADDNPLUGIN(255, 255)
#endif

// When extending this, search for EXTEND_CONTROLLER_IDS 
// in the code to find all places that need to be updated too.

  #ifndef BUILD_NO_RAM_TRACKER
  logMemUsageAfter(F("ADDNPLUGIN(...)"));
  #endif

  NPluginCall(NPlugin::Function::NPLUGIN_PROTOCOL_ADD, 0);
  #ifndef BUILD_NO_RAM_TRACKER
  logMemUsageAfter(F("NPLUGIN_PROTOCOL_ADD"));
  #endif

}
#endif
#include "../Helpers/AdafruitGFX_helper.h"
#include "../../_Plugin_Helper.h"

#ifdef PLUGIN_USES_ADAFRUITGFX

# include "../Helpers/StringConverter.h"
# include "../WebServer/Markup_Forms.h"
# if FEATURE_SD && defined(ADAGFX_ENABLE_BMP_DISPLAY)
#  include <SD.h>
# endif // if FEATURE_SD && defined(ADAGFX_ENABLE_BMP_DISPLAY)

# if ADAGFX_FONTS_INCLUDED
#  include "src/Static/Fonts/Seven_Segment24pt7b.h"
#  include "src/Static/Fonts/Seven_Segment18pt7b.h"
#  include "src/Static/Fonts/FreeSans9pt7b.h"
#  ifdef ADAGFX_FONTS_EXTRA_8PT_INCLUDED
#   include "src/Static/Fonts/angelina8pt7b.h"
#   include "src/Static/Fonts/NovaMono8pt7b.h"
#   include "src/Static/Fonts/unispace8pt7b.h"
#   include "src/Static/Fonts/unispace_italic8pt7b.h"
#   include "src/Static/Fonts/whitrabt8pt7b.h"
#  endif // ifdef ADAGFX_FONTS_EXTRA_8PT_INCLUDED
#  ifdef ADAGFX_FONTS_EXTRA_12PT_INCLUDED
#   include "src/Static/Fonts/angelina12pt7b.h"
#   include "src/Static/Fonts/NovaMono12pt7b.h"
#   include "src/Static/Fonts/RepetitionScrolling12pt7b.h"
#   include "src/Static/Fonts/unispace12pt7b.h"
#   include "src/Static/Fonts/unispace_italic12pt7b.h"
#   include "src/Static/Fonts/whitrabt12pt7b.h"
#  endif // ifdef ADAGFX_FONTS_EXTRA_12PT_INCLUDED
#  ifdef ADAGFX_FONTS_EXTRA_16PT_INCLUDED
#   include "src/Static/Fonts/AmerikaSans16pt7b.h"
#   include "src/Static/Fonts/whitrabt16pt7b.h"
#  endif // ifdef ADAGFX_FONTS_EXTRA_16PT_INCLUDED
#  ifdef ADAGFX_FONTS_EXTRA_18PT_INCLUDED
#   include "src/Static/Fonts/whitrabt18pt7b.h"
#  endif // ifdef ADAGFX_FONTS_EXTRA_18PT_INCLUDED
#  ifdef ADAGFX_FONTS_EXTRA_20PT_INCLUDED
#   include "src/Static/Fonts/whitrabt20pt7b.h"
#  endif // ifdef ADAGFX_FONTS_EXTRA_20PT_INCLUDED
# endif  // if ADAGFX_FONTS_INCLUDED

/******************************************************************************************
 * get the display text for a 'text print mode' enum value
 *****************************************************************************************/
const __FlashStringHelper* toString(AdaGFXTextPrintMode mode) {
  switch (mode) {
    case AdaGFXTextPrintMode::ContinueToNextLine: return F("Continue to next line");
    case AdaGFXTextPrintMode::TruncateExceedingMessage: return F("Truncate exceeding message");
    case AdaGFXTextPrintMode::ClearThenTruncate: return F("Clear then truncate exceeding message");
    case AdaGFXTextPrintMode::MAX: break;
  }
  return F("None");
}

/******************************************************************************************
 * get the display text for a color depth enum value
 *****************************************************************************************/
const __FlashStringHelper* toString(AdaGFXColorDepth colorDepth) {
  switch (colorDepth) {
    case AdaGFXColorDepth::Monochrome: return F("Monochrome");
    case AdaGFXColorDepth::BlackWhiteRed: return F("Monochrome + 1 color");
    case AdaGFXColorDepth::BlackWhite2Greyscales: return F("Monochrome + 2 grey levels");
    # if ADAGFX_SUPPORT_7COLOR
    case AdaGFXColorDepth::SevenColor: return F("eInk - 7 colors");
    # endif // if ADAGFX_SUPPORT_7COLOR
    case AdaGFXColorDepth::EightColor: return F("TFT - 8 colors");
    case AdaGFXColorDepth::SixteenColor: return F("TFT - 16 colors");
    case AdaGFXColorDepth::FullColor: return F("Full color - 65535 colors");
  }
  return F("None");
}

/*****************************************************************************************
 * Show a selector for all available 'Text print mode' options, for use in PLUGIN_WEBFORM_LOAD
 ****************************************************************************************/
void AdaGFXFormTextPrintMode(const __FlashStringHelper *id,
                             uint8_t                    selectedIndex) {
  const int textModeCount                             = static_cast<int>(AdaGFXTextPrintMode::MAX);
  const __FlashStringHelper *textModes[textModeCount] = { // Be sure to use all available modes from enum!
    toString(AdaGFXTextPrintMode::ContinueToNextLine),
    toString(AdaGFXTextPrintMode::TruncateExceedingMessage),
    toString(AdaGFXTextPrintMode::ClearThenTruncate)
  };
  const int textModeOptions[textModeCount] = {
    static_cast<int>(AdaGFXTextPrintMode::ContinueToNextLine),
    static_cast<int>(AdaGFXTextPrintMode::TruncateExceedingMessage),
    static_cast<int>(AdaGFXTextPrintMode::ClearThenTruncate)
  };

  addFormSelector(F("Text print Mode"), id, textModeCount, textModes, textModeOptions, selectedIndex);
}

void AdaGFXFormColorDepth(const __FlashStringHelper *id,
                          uint16_t                   selectedIndex,
                          bool                       enabled) {
  # if ADAGFX_SUPPORT_7COLOR
  const int colorDepthCount = 7 + 1;
  # else // if ADAGFX_SUPPORT_7COLOR
  const int colorDepthCount = 6 + 1;
  # endif // if ADAGFX_SUPPORT_7COLOR
  const __FlashStringHelper *colorDepths[colorDepthCount] = { // Be sure to use all available modes from enum!
    toString(static_cast<AdaGFXColorDepth>(0)),               // include None
    toString(AdaGFXColorDepth::Monochrome),
    toString(AdaGFXColorDepth::BlackWhiteRed),
    toString(AdaGFXColorDepth::BlackWhite2Greyscales),
    # if ADAGFX_SUPPORT_7COLOR
    toString(AdaGFXColorDepth::SevenColor),
    # endif // if ADAGFX_SUPPORT_7COLOR
    toString(AdaGFXColorDepth::EightColor),
    toString(AdaGFXColorDepth::SixteenColor),
    toString(AdaGFXColorDepth::FullColor)
  };
  const int colorDepthOptions[colorDepthCount] = {
    0,
    static_cast<int>(AdaGFXColorDepth::Monochrome),
    static_cast<int>(AdaGFXColorDepth::BlackWhiteRed),
    static_cast<int>(AdaGFXColorDepth::BlackWhite2Greyscales),
    # if ADAGFX_SUPPORT_7COLOR
    static_cast<int>(AdaGFXColorDepth::SevenColor),
    # endif // if ADAGFX_SUPPORT_7COLOR
    static_cast<int>(AdaGFXColorDepth::EightColor),
    static_cast<int>(AdaGFXColorDepth::SixteenColor),
    static_cast<int>(AdaGFXColorDepth::FullColor)
  };

  addRowLabel_tr_id(F("Display Color-depth"), id);
  addSelector(id, colorDepthCount, colorDepths, colorDepthOptions, NULL, selectedIndex, false, enabled);
}

/*****************************************************************************************
 * Show a selector for Rotation options, supported by Adafruit_GFX
 ****************************************************************************************/
void AdaGFXFormRotation(const __FlashStringHelper *id,
                        uint8_t                    selectedIndex) {
  const __FlashStringHelper *rotationOptions[] = { F("Normal"), F("+90&deg;"), F("+180&deg;"), F("+270&deg;") };
  const int rotationOptionValues[]             = { 0, 1, 2, 3 };

  addFormSelector(F("Rotation"), id, 4, rotationOptions, rotationOptionValues, selectedIndex);
}

/*****************************************************************************************
 * Show a checkbox & note to disable background-fill for text
 ****************************************************************************************/
void AdaGFXFormTextBackgroundFill(const __FlashStringHelper *id,
                                  uint8_t                    selectedIndex) {
  addFormCheckBox(F("Background-fill for text"), id, selectedIndex);
  addFormNote(F("Fill entire line-height with background color."));
}

/*****************************************************************************************
 * Show a checkbox & note to enable col/row mode for txp, txz and txtfull subcommands
 ****************************************************************************************/
void AdaGFXFormTextColRowMode(const __FlashStringHelper *id,
                              bool                       selectedState) {
  addFormCheckBox(F("Text Coordinates in col/row"), id, selectedState);
  addFormNote(F("Unchecked: Coordinates in pixels. Applies only to 'txp', 'txz' and 'txtfull' subcommands."));
}

/*****************************************************************************************
 * Show a checkbox & note to enable -1 px compatibility mode for txp and txtfull subcommands
 ****************************************************************************************/
void AdaGFXFormOnePixelCompatibilityOption(const __FlashStringHelper *id,
                                           uint8_t                    selectedIndex) {
  addFormCheckBox(F("Use -1px offset for txp &amp; txtfull"), id, selectedIndex);
  addFormNote(F("This is for compatibility with the original plugin implementation."));
}

/*****************************************************************************************
 * Show 2 input fields for Foreground and Background color, translated to known color names or hex with # prefix
 ****************************************************************************************/
void AdaGFXFormForeAndBackColors(const __FlashStringHelper *foregroundId,
                                 uint16_t                   foregroundColor,
                                 const __FlashStringHelper *backgroundId,
                                 uint16_t                   backgroundColor,
                                 AdaGFXColorDepth           colorDepth) {
  String color;

  color = AdaGFXcolorToString(foregroundColor, colorDepth);
  addFormTextBox(F("Foreground color"), foregroundId, color, 11);
  color = AdaGFXcolorToString(backgroundColor, colorDepth);
  addFormTextBox(F("Background color"), backgroundId, color, 11);
  addFormNote(F("Use Color name, '#RGB565' (# + 1..4 hex nibbles) or '#RRGGBB' (# + 6 hex nibbles RGB color)."));
  addFormNote(F("NB: Colors stored as RGB565 value!"));
}

/*****************************************************************************************
 * Show a pin selector and percentage 1..100 for Backlight settings
 ****************************************************************************************/
void AdaGFXFormBacklight(const __FlashStringHelper *backlightPinId,
                         int8_t                     backlightPin,
                         const __FlashStringHelper *backlightPercentageId,
                         uint16_t                   backlightPercentage) {
  addFormPinSelect(PinSelectPurpose::Generic_output, formatGpioName_output_optional(F("Backlight ")), backlightPinId, backlightPin);

  addFormNumericBox(F("Backlight percentage"), backlightPercentageId, backlightPercentage, 1, 100);
  addUnit(F("1-100%"));
}

/*****************************************************************************************
 * Show pin selector, inverse option and timeout inputs for Displaybutton settings
 ****************************************************************************************/
void AdaGFXFormDisplayButton(const __FlashStringHelper *buttonPinId,
                             int8_t                     buttonPin,
                             const __FlashStringHelper *buttonInverseId,
                             bool                       buttonInverse,
                             const __FlashStringHelper *displayTimeoutId,
                             int                        displayTimeout) {
  addFormPinSelect(PinSelectPurpose::Generic_input, F("Display button"), buttonPinId, buttonPin);

  addFormCheckBox(F("Inversed Logic"), buttonInverseId, buttonInverse);

  addFormNumericBox(F("Display Timeout"), displayTimeoutId, displayTimeout, 0);
  addUnit(F("0 = off"));
}

/*****************************************************************************************
 * Show a numeric input 1..10 for Font scaling setting
 ****************************************************************************************/
void AdaGFXFormFontScaling(const __FlashStringHelper *fontScalingId,
                           uint8_t                    fontScaling,
                           uint8_t                    maxScale) {
  addFormNumericBox(F("Font scaling"), fontScalingId, fontScaling, 1, maxScale);
  String unit = F("1x..");

  unit += maxScale;
  unit += 'x';
  addUnit(unit);
}

/****************************************************************************
 * AdaGFXparseTemplate: Replace variables and adjust unicode special characters to Adafruit font
 ***************************************************************************/
String AdaGFXparseTemplate(String            & tmpString,
                           uint8_t             lineSize,
                           AdafruitGFX_helper *gfxHelper) {
  // Änderung WDS: Tabelle vorerst Abgeschaltet !!!!
  // Perform some specific changes for LCD display
  // https://www.letscontrolit.com/forum/viewtopic.php?t=2368
  # if ADAGFX_PARSE_SUBCOMMAND

  String result = tmpString;

  if (nullptr != gfxHelper) {
    String trigger = gfxHelper->getTrigger();

    if (!trigger.isEmpty()) {
      int16_t prefixTrigger  = result.indexOf(ADAGFX_PARSE_PREFIX);
      int16_t postfixTrigger = result.indexOf(ADAGFX_PARSE_POSTFIX, prefixTrigger + 1);

      while ((prefixTrigger > -1) && (postfixTrigger > -1) && (postfixTrigger > prefixTrigger)) { // Might be valid
        String subcommand = result.substring(prefixTrigger + ADAGFX_PARSE_POSTFIX_LEN, postfixTrigger);

        if (!subcommand.isEmpty()) {
          String command;
          command.reserve(trigger.length() + 1 + subcommand.length());
          command += trigger;
          command += ',';
          command += subcommand;

          #  ifndef BUILD_NO_DEBUG

          if (loglevelActiveFor(ADAGFX_LOG_LEVEL)) {
            String log;
            log.reserve(command.length() + 20);
            log += F("AdaGFX: inline cmd: ");
            log += command;
            addLogMove(ADAGFX_LOG_LEVEL, log);
          }
          #  endif // ifndef BUILD_NO_DEBUG

          if (gfxHelper->processCommand(command)) { // Execute command and remove from result incl. pre/postfix
            result.remove(prefixTrigger, (postfixTrigger - prefixTrigger) + ADAGFX_PARSE_POSTFIX_LEN);
            prefixTrigger  = result.indexOf(ADAGFX_PARSE_PREFIX);
            postfixTrigger = result.indexOf(ADAGFX_PARSE_POSTFIX, prefixTrigger + 1);
          } else { // If the command fails, exit further processing
            prefixTrigger  = -1;
            postfixTrigger = -1;
            #  ifndef BUILD_NO_DEBUG
            addLog(ADAGFX_LOG_LEVEL, F("AdaGFX: inline cmd: unknown"));
            #  endif // ifndef BUILD_NO_DEBUG
          }
        } else {
          prefixTrigger  = -1;
          postfixTrigger = -1;
        }
      }
    }
  }
  result = parseTemplate_padded(result, lineSize);
  # else // if ADAGFX_PARSE_SUBCOMMAND
  String result = parseTemplate_padded(tmpString, lineSize);
  # endif // if ADAGFX_PARSE_SUBCOMMAND

  const char euro[4]       = { 0xe2, 0x82, 0xac, 0 }; // Unicode euro symbol
  const char euro_ascii[2] = { 0xED, 0 };             // Euro symbol
  result.replace(euro, euro_ascii);

  char unicodePrefix = 0xc2;

  if (result.indexOf(unicodePrefix) != -1) {
    const char degree[3]       = { 0xc2, 0xb0, 0 };  // Unicode degree symbol
    const char degree_ascii[2] = { 0xf7, 0 };        // degree symbol
    result.replace(degree, degree_ascii);

    const char pound[3]       = { 0xc2, 0xa3, 0 };   // Unicode pound symbol
    const char pound_ascii[2] = { 0x9C, 0 };         // pound symbol
    result.replace(pound, pound_ascii);

    const char yen[3]       = { 0xc2, 0xa5, 0 };     // Unicode yen symbol
    const char yen_ascii[2] = { 0x9D, 0 };           // yen symbol
    result.replace(yen, yen_ascii);

    const char cent[3]       = { 0xc2, 0xa2, 0 };    // Unicode cent symbol
    const char cent_ascii[2] = { 0x9B, 0 };          // cent symbol
    result.replace(cent, cent_ascii);

    const char mu[3]       = { 0xc2, 0xb5, 0 };      // Unicode mu/micro (µ) symbol
    const char mu_ascii[2] = { 0xe5, 0 };            // mu/micro symbol
    result.replace(mu, mu_ascii);

    const char plusmin[3]       = { 0xc2, 0xb1, 0 }; // Unicode plusminus symbol
    const char plusmin_ascii[2] = { 0xf0, 0 };       // plusminus symbol
    result.replace(plusmin, plusmin_ascii);

    const char laquo[3]       = { 0xc2, 0xab, 0 };   // Unicode left aquo symbol
    const char laquo_ascii[2] = { 0xae, 0 };         // left aquo symbol
    result.replace(laquo, laquo_ascii);

    const char raquo[3]       = { 0xc2, 0xbb, 0 };   // Unicode right aquote symbol
    const char raquo_ascii[2] = { 0xaf, 0 };         // right aquote symbol
    result.replace(raquo, raquo_ascii);

    const char half[3]       = { 0xc2, 0xbd, 0 };    // Unicode half 1/2 symbol
    const char half_ascii[2] = { 0xab, 0 };          // half 1/2 symbol
    result.replace(half, half_ascii);

    const char quart[3]       = { 0xc2, 0xbc, 0 };   // Unicode quart 1/4 symbol
    const char quart_ascii[2] = { 0xac, 0 };         // quart 1/4 symbol
    result.replace(quart, quart_ascii);
    delay(0);
  }

  unicodePrefix = 0xc3;

  if (result.indexOf(unicodePrefix) != -1) {
    // See: https://github.com/letscontrolit/ESPEasy/issues/2081

    const char umlautAE_uni[3]   = { 0xc3, 0x84, 0 };  // Unicode Umlaute AE
    const char umlautAE_ascii[2] = { 0x8e, 0 };        // Umlaute A
    result.replace(umlautAE_uni, umlautAE_ascii);

    const char umlaut_ae_uni[3]  = { 0xc3, 0xa4, 0 };  // Unicode Umlaute ae
    const char umlautae_ascii[2] = { 0x84, 0 };        // Umlaute a
    result.replace(umlaut_ae_uni, umlautae_ascii);

    const char umlautOE_uni[3]   = { 0xc3, 0x96, 0 };  // Unicode Umlaute OE
    const char umlautOE_ascii[2] = { 0x99, 0 };        // Umlaute O
    result.replace(umlautOE_uni, umlautOE_ascii);

    const char umlaut_oe_uni[3]  = { 0xc3, 0xb6, 0 };  // Unicode Umlaute oe
    const char umlautoe_ascii[2] = { 0x94, 0 };        // Umlaute o
    result.replace(umlaut_oe_uni, umlautoe_ascii);

    const char umlautUE_uni[3]   = { 0xc3, 0x9c, 0 };  // Unicode Umlaute UE
    const char umlautUE_ascii[2] = { 0x9a, 0 };        // Umlaute U
    result.replace(umlautUE_uni, umlautUE_ascii);

    const char umlaut_ue_uni[3]  = { 0xc3, 0xbc, 0 };  // Unicode Umlaute ue
    const char umlautue_ascii[2] = { 0x81, 0 };        // Umlaute u
    result.replace(umlaut_ue_uni, umlautue_ascii);

    const char divide_uni[3]   = { 0xc3, 0xb7, 0 };    // Unicode divide symbol
    const char divide_ascii[2] = { 0xf5, 0 };          // Divide symbol
    result.replace(divide_uni, divide_ascii);

    const char umlaut_sz_uni[3]   = { 0xc3, 0x9f, 0 }; // Unicode Umlaute sz
    const char umlaut_sz_ascii[2] = { 0xe0, 0 };       // Umlaute
    result.replace(umlaut_sz_uni, umlaut_sz_ascii);
    delay(0);
  }

  for (uint16_t l = result.length(); l > 0 && isSpace(result[l - 1]); l--) { // Right-trim
    result.remove(l - 1);
  }

  # ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(ADAGFX_LOG_LEVEL)) {
    String log;
    log.reserve(result.length() + 24);
    log += F("AdaGFX: parse result: '");
    log += result;
    log += '\'';
    addLogMove(ADAGFX_LOG_LEVEL, log);
  }
  # endif // ifndef BUILD_NO_DEBUG
  return result;
}

// AdafruitGFX_helper class methods

/****************************************************************************
 * parameterized constructors
 ***************************************************************************/
AdafruitGFX_helper::AdafruitGFX_helper(Adafruit_GFX       *display,
                                       const String      & trigger,
                                       uint16_t            res_x,
                                       uint16_t            res_y,
                                       AdaGFXColorDepth    colorDepth,
                                       AdaGFXTextPrintMode textPrintMode,
                                       uint8_t             fontscaling,
                                       uint16_t            fgcolor,
                                       uint16_t            bgcolor,
                                       bool                useValidation,
                                       bool                textBackFill)
  : _display(display), _trigger(trigger), _res_x(res_x), _res_y(res_y), _colorDepth(colorDepth),
  _textPrintMode(textPrintMode), _fontscaling(fontscaling), _fgcolor(fgcolor), _bgcolor(bgcolor),
  _useValidation(useValidation), _textBackFill(textBackFill)
{
  addLog(LOG_LEVEL_INFO, F("AdaGFX_helper: GFX Init."));
  initialize();
}

# ifdef ADAGFX_ENABLE_BMP_DISPLAY
AdafruitGFX_helper::AdafruitGFX_helper(Adafruit_SPITFT    *display,
                                       const String      & trigger,
                                       uint16_t            res_x,
                                       uint16_t            res_y,
                                       AdaGFXColorDepth    colorDepth,
                                       AdaGFXTextPrintMode textPrintMode,
                                       uint8_t             fontscaling,
                                       uint16_t            fgcolor,
                                       uint16_t            bgcolor,
                                       bool                useValidation,
                                       bool                textBackFill)
  : _tft(display), _trigger(trigger), _res_x(res_x), _res_y(res_y), _colorDepth(colorDepth),
  _textPrintMode(textPrintMode), _fontscaling(fontscaling), _fgcolor(fgcolor), _bgcolor(bgcolor),
  _useValidation(useValidation), _textBackFill(textBackFill)
{
  _display = _tft;
  addLog(LOG_LEVEL_INFO, F("AdaGFX_helper: TFT Init."));
  initialize();
}

# endif // ifdef ADAGFX_ENABLE_BMP_DISPLAY

void AdafruitGFX_helper::initialize()
{
  _trigger.toLowerCase(); // store trigger in lowercase
  # ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(ADAGFX_LOG_LEVEL)) {
    String log;
    log.reserve(65);
    log += F("AdaGFX: Init, x: ");
    log += _res_x;
    log += F(", y: ");
    log += _res_y;
    log += F(", colors: ");
    log += static_cast<uint16_t>(_colorDepth);
    log += F(", trigger: ");
    log += _trigger;
    addLogMove(ADAGFX_LOG_LEVEL, log);
  }
  # endif // ifndef BUILD_NO_DEBUG

  _display_x = _res_x; // Store initial resolution
  _display_y = _res_y;

  if (_fontscaling < 1) { _fontscaling = 1; }

  if (nullptr != _display) {
    _display->setTextSize(_fontscaling);
    _display->setTextColor(_fgcolor, _bgcolor); // initialize text colors
    _display->setTextWrap(_textPrintMode == AdaGFXTextPrintMode::ContinueToNextLine);
  }
}

/****************************************************************************
 * getCursorXY: get the current (text) cursor coordinates, either in pixels or cols/rows, depending on related setting
 ***************************************************************************/
void AdafruitGFX_helper::getCursorXY(int16_t& currentX,
                                     int16_t& currentY) {
  _lastX = _display->getCursorX();
  _lastY = _display->getCursorY();

  if (_columnRowMode && (_lastX != 0)) { _lastX /= _fontwidth; }

  if (_columnRowMode && (_lastY != 0)) { _lastY /= _fontheight; }
  currentX = _lastX;
  currentY = _lastY;
}

/****************************************************************************
 * processCommand: Parse string to <command>,<subcommand>[,<arguments>...] and execute that command
 ***************************************************************************/
bool AdafruitGFX_helper::processCommand(const String& string) {
  bool success = false;

  if ((nullptr == _display) || _trigger.isEmpty()) { return success; }

  String cmd        = parseString(string, 1); // lower case
  String subcommand = parseString(string, 2);

  if (!cmd.equals(_trigger) || subcommand.isEmpty()) { return success; } // Only support own trigger, and at least a non=empty subcommand

  String log;
  String sParams[ADAGFX_PARSE_MAX_ARGS + 1];
  int    nParams[ADAGFX_PARSE_MAX_ARGS + 1];
  int    argCount = 0;
  bool   loop     = true;

  while (argCount <= ADAGFX_PARSE_MAX_ARGS && loop) {
    sParams[argCount] = parseStringKeepCase(string, argCount + 3); // 0-offset + 1st and 2nd argument used by trigger/subcommand
    validIntFromString(sParams[argCount], nParams[argCount]);
    loop = !sParams[argCount].isEmpty();

    # ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
      log  = ':';
      log += argCount;
      log += ' ';
      log += sParams[argCount];
      addLog(LOG_LEVEL_DEBUG_DEV, log);
    }
    # endif // ifndef BUILD_NO_DEBUG

    if (loop) { argCount++; }
  }
  success = true; // If we get this far, we'll flip the flag if something wrong is found

  # ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(ADAGFX_LOG_LEVEL)) {
    log.reserve(90);
    log.clear();
    log += F("AdaGFX: command: ");
    log += _trigger;
    log += F(" argCount: ");
    log += argCount;
    log += ':';
    log += string;
    addLog(ADAGFX_LOG_LEVEL, log);
  }
  # endif // ifndef BUILD_NO_DEBUG

  if (subcommand.equals(F("txt")))                          // txt: Print text at last cursor position, ends at next line!
  {
    _display->println(parseStringToEndKeepCase(string, 3)); // Print entire rest of provided line
  }
  else if (subcommand.equals(F("txp")) && (argCount == 2))  // txp: Text position
  {
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1], _columnRowMode)) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      if (_columnRowMode) {
        _display->setCursor(nParams[0] * _fontwidth, nParams[1] * _fontheight);
      } else {
        _display->setCursor(nParams[0] - _p095_compensation, nParams[1] - _p095_compensation);
      }
    }
  }
  else if (subcommand.equals(F("txz")) && (argCount >= 3)) // txz: Text at position
  {
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1], _columnRowMode)) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      if (_columnRowMode) {
        _display->setCursor(nParams[0] * _fontwidth, nParams[1] * _fontheight);
      } else {
        _display->setCursor(nParams[0], nParams[1]);
      }
      _display->println(parseStringToEndKeepCase(string, 5));                   // Print entire rest of provided line
    }
  }
  else if (subcommand.equals(F("txc")) && ((argCount == 1) || (argCount == 2))) // txc: Textcolor, fg and opt. bg colors
  {
    _fgcolor = AdaGFXparseColor(sParams[0], _colorDepth);

    if (argCount == 1) {
      _bgcolor = _fgcolor; // Transparent background
      _display->setTextColor(_fgcolor);
    } else {               // argCount=2
      _bgcolor = AdaGFXparseColor(sParams[1], _colorDepth);
      _display->setTextColor(_fgcolor, _bgcolor);
    }
  }
  else if (subcommand.equals(F("txs")) && (argCount == 1))
  {
    if ((nParams[0] >= 0) || (nParams[0] <= 10)) {
      _fontscaling = nParams[0];
      _display->setTextSize(_fontscaling);
      calculateTextMetrics(_fontwidth, _fontheight, _heightOffset, _isProportional);
    } else {
      success = false;
    }
  }
  else if (subcommand.equals(F("txtfull")) && (argCount >= 3) && (argCount <= 6)) { // txtfull: Text at position, with size and color
    switch (argCount) {
      case 3:                                                                       // single text

        # if ADAGFX_ARGUMENT_VALIDATION

        if (invalidCoordinates(nParams[0] - _p095_compensation, nParams[1] - _p095_compensation, _columnRowMode)) {
          success = false;
        } else
        # endif // if ADAGFX_ARGUMENT_VALIDATION
        {
          printText(sParams[2].c_str(),
                    nParams[0] - _p095_compensation,
                    nParams[1] - _p095_compensation,
                    _fontscaling,
                    _fgcolor,
                    _fgcolor); // transparent bg
        }
        break;
      case 4:                  // text + size

        # if ADAGFX_ARGUMENT_VALIDATION

        if (invalidCoordinates(nParams[0] - _p095_compensation, nParams[1] - _p095_compensation, _columnRowMode)) {
          success = false;
        } else
        # endif // if ADAGFX_ARGUMENT_VALIDATION
        {
          printText(sParams[3].c_str(),
                    nParams[0] - _p095_compensation,
                    nParams[1] - _p095_compensation,
                    nParams[2],
                    _fgcolor,
                    _fgcolor); // transparent bg
        }
        break;
      case 5:                  // text + size + color

        # if ADAGFX_ARGUMENT_VALIDATION

        if (invalidCoordinates(nParams[0] - _p095_compensation, nParams[1] - _p095_compensation, _columnRowMode)) {
          success = false;
        } else
        # endif // if ADAGFX_ARGUMENT_VALIDATION
        {
          uint16_t color = AdaGFXparseColor(sParams[3], _colorDepth);
          printText(sParams[4].c_str(),
                    nParams[0] - _p095_compensation,
                    nParams[1] - _p095_compensation,
                    nParams[2],
                    color,
                    color); // transparent bg
        }
        break;
      case 6:               // text + size + color + bkcolor

        # if ADAGFX_ARGUMENT_VALIDATION

        if (invalidCoordinates(nParams[0] - _p095_compensation, nParams[1] - _p095_compensation, _columnRowMode)) {
          success = false;
        } else
        # endif // if ADAGFX_ARGUMENT_VALIDATION
        {
          printText(sParams[5].c_str(),
                    nParams[0] - _p095_compensation,
                    nParams[1] - _p095_compensation,
                    nParams[2],
                    AdaGFXparseColor(sParams[3], _colorDepth),
                    AdaGFXparseColor(sParams[4], _colorDepth));
        }
        break;
      default:
        success = false;
        break;
    }
  }
  else if (subcommand.equals(F("clear"))) // Clear display
  {
    if (argCount >= 1) {
      _display->fillScreen(AdaGFXparseColor(sParams[0], _colorDepth));
    } else {
      _display->fillScreen(_bgcolor);
    }
  }
  else if (subcommand.equals(F("rot")) && (argCount == 1)) // Rotation
  {
    if ((nParams[0] < 0) || (nParams[0] > 3)) {
      success = false;
    } else {
      setRotation(nParams[0]);
    }
  }
  else if (subcommand.equals(F("tpm")) && (argCount == 1)) // Text Print Mode
  {
    if ((nParams[0] < 0) || (nParams[0] >= static_cast<int>(AdaGFXTextPrintMode::MAX))) {
      success = false;
    } else {
      _textPrintMode = static_cast<AdaGFXTextPrintMode>(nParams[0]);
      _display->setTextWrap(_textPrintMode == AdaGFXTextPrintMode::ContinueToNextLine);
    }
  }
  # if ADAGFX_USE_ASCIITABLE
  else if (subcommand.equals(F("asciitable"))) // Show ASCII table
  {
    String  line;
    int16_t start        = 0x80 + (argCount >= 1 && nParams[0] >= -4 && nParams[0] < 4 ? nParams[0] * 0x20 : 0);
    uint8_t scale        = (argCount == 2 && nParams[1] > 0 && nParams[1] <= 10 ? nParams[1] : 2);
    uint8_t currentScale = _fontscaling;

    if (_fontscaling != scale) { // Set fontscaling
      _fontscaling = scale;
      _display->setTextSize(_fontscaling);
      calculateTextMetrics(_fontwidth, _fontheight, _heightOffset, _isProportional);
    }
    line.reserve(_textcols);
    _display->setCursor(0, 0);
    int16_t row     = 0;
    bool    colMode = _columnRowMode;
    _columnRowMode = true;

    for (int16_t i = start; i <= 0xFF && row < _textrows; i++) {
      if ((i % 4 == 0) && (line.length() > (_textcols - 8u))) { // 8 = 4x space + char
        printText(line.c_str(), 0, row, _fontscaling, _fgcolor, _bgcolor);
        line.clear();
        row++;
      }

      if (line.isEmpty()) {
        line += F("0x");

        if (i < 0x10) { line += '0'; }
        line += String(i, HEX);
      }
      line += ' ';
      line += static_cast<char>(((i == 0x0A) || (i == 0x0D) ? 0x20 : i)); // Show a space instead of CR/LF
    }

    if (row < _textrows) {
      printText(line.c_str(), 0, row, _fontscaling, _fgcolor, _bgcolor);
    }

    _columnRowMode = colMode;           // Restore

    if (_fontscaling != currentScale) { // Restore if needed
      _fontscaling = currentScale;
      _display->setTextSize(_fontscaling);
      calculateTextMetrics(_fontwidth, _fontheight, _heightOffset, _isProportional);
    }
  }
  # endif // if ADAGFX_USE_ASCIITABLE
  else if (subcommand.equals(F("font")) && (argCount == 1)) { // font: Change font
    # if ADAGFX_FONTS_INCLUDED
    sParams[0].toLowerCase();

    if (sParams[0].equals(F("sevenseg24"))) {
      _display->setFont(&Seven_Segment24pt7b);
      calculateTextMetrics(21, 42, 35);
    } else if (sParams[0].equals(F("sevenseg18"))) {
      _display->setFont(&Seven_Segment18pt7b);
      calculateTextMetrics(16, 32, 25);
    } else if (sParams[0].equals(F("freesans"))) {
      _display->setFont(&FreeSans9pt7b);
      calculateTextMetrics(10, 16, 12);

      // Extra 8pt fonts:
    #  ifdef ADAGFX_FONTS_EXTRA_8PT_INCLUDED
    #   ifdef ADAGFX_FONTS_EXTRA_8PT_ANGELINA
    } else if (sParams[0].equals(F("angelina8prop"))) { // Proportional font!
      _display->setFont(&angelina8pt7b);
      calculateTextMetrics(6, 16, 12, true);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_8PT_ANGELINA
    #   ifdef ADAGFX_FONTS_EXTRA_8PT_NOVAMONO
    } else if (sParams[0].equals(F("novamono8pt"))) {
      _display->setFont(&NovaMono8pt7b);
      calculateTextMetrics(9, 16, 12);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_8PT_NOVAMONO
    #   ifdef ADAGFX_FONTS_EXTRA_8PT_UNISPACE
    } else if (sParams[0].equals(F("unispace8pt"))) {
      _display->setFont(&unispace8pt7b);
      calculateTextMetrics(13, 24, 20);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_8PT_UNISPACE
    #   ifdef ADAGFX_FONTS_EXTRA_8PT_UNISPACEITALIC
    } else if (sParams[0].equals(F("unispaceitalic8pt"))) {
      _display->setFont(&unispace_italic8pt7b);
      calculateTextMetrics(13, 24, 20);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_8PT_UNISPACEITALIC
    #   ifdef ADAGFX_FONTS_EXTRA_8PT_WHITERABBiT
    } else if (sParams[0].equals(F("whiterabbit8pt"))) {
      _display->setFont(&whitrabt8pt7b);
      calculateTextMetrics(10, 16, 12);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_8PT_WHITERABBiT
    #  endif  // ifdef ADAGFX_FONTS_EXTRA_8PT_INCLUDED
      // Extra 12pt fonts:
    #  ifdef ADAGFX_FONTS_EXTRA_12PT_INCLUDED
    #   ifdef ADAGFX_FONTS_EXTRA_12PT_ANGELINA
    } else if (sParams[0].equals(F("angelina12prop"))) { // Proportional font!
      _display->setFont(&angelina12pt7b);
      calculateTextMetrics(8, 22, 18, true);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_12PT_ANGELINA
    #   ifdef ADAGFX_FONTS_EXTRA_12PT_NOVAMONO
    } else if (sParams[0].equals(F("novamono12pt"))) {
      _display->setFont(&NovaMono12pt7b);
      calculateTextMetrics(13, 26, 22);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_12PT_NOVAMONO
    #   ifdef ADAGFX_FONTS_EXTRA_12PT_REPETITIONSCROLLiNG
    } else if (sParams[0].equals(F("repetitionscrolling12pt"))) {
      _display->setFont(&RepetitionScrolling12pt7b);
      calculateTextMetrics(13, 22, 18);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_12PT_REPETITIONSCROLLiNG
    #   ifdef ADAGFX_FONTS_EXTRA_12PT_UNISPACE
    } else if (sParams[0].equals(F("unispace12pt"))) {
      _display->setFont(&unispace12pt7b);
      calculateTextMetrics(18, 30, 26);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_12PT_UNISPACE
    #   ifdef ADAGFX_FONTS_EXTRA_12PT_UNISPACEITALIC
    } else if (sParams[0].equals(F("unispaceitalic12pt"))) {
      _display->setFont(&unispace_italic12pt7b);
      calculateTextMetrics(18, 30, 26);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_12PT_UNISPACEITALIC
    #   ifdef ADAGFX_FONTS_EXTRA_12PT_WHITERABBiT
    } else if (sParams[0].equals(F("whiterabbit12pt"))) {
      _display->setFont(&whitrabt12pt7b);
      calculateTextMetrics(13, 20, 16);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_12PT_WHITERABBiT
    #  endif  // ifdef ADAGFX_FONTS_EXTRA_12PT_INCLUDED
    #  ifdef ADAGFX_FONTS_EXTRA_16PT_INCLUDED
    #   ifdef ADAGFX_FONTS_EXTRA_16PT_AMERIKASANS
    } else if (sParams[0].equals(F("amerikasans16pt"))) { // Proportional font!
      _display->setFont(&AmerikaSans16pt7b);
      calculateTextMetrics(17, 30, 26, true);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_16PT_AMERIKASANS
    #   ifdef ADAGFX_FONTS_EXTRA_16PT_WHITERABBiT
    } else if (sParams[0].equals(F("whiterabbit16pt"))) {
      _display->setFont(&whitrabt16pt7b);
      calculateTextMetrics(18, 26, 22);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_16PT_WHITERABBiT
    #  endif  // ifdef ADAGFX_FONTS_EXTRA_16PT_INCLUDED
    #  ifdef ADAGFX_FONTS_EXTRA_18PT_INCLUDED
    #   ifdef ADAGFX_FONTS_EXTRA_18PT_WHITERABBiT
    } else if (sParams[0].equals(F("whiterabbit18pt"))) {
      _display->setFont(&whitrabt18pt7b);
      calculateTextMetrics(21, 30, 26);
      #   endif // ifdef ADAGFX_FONTS_EXTRA_18PT_WHITERABBiT
    #  endif    // ifdef ADAGFX_FONTS_EXTRA_18PT_WHITERABBiT
    #  ifdef ADAGFX_FONTS_EXTRA_20PT_INCLUDED
    #   ifdef ADAGFX_FONTS_EXTRA_20PT_WHITERABBiT
    } else if (sParams[0].equals(F("whiterabbit20pt"))) {
      _display->setFont(&whitrabt20pt7b);
      calculateTextMetrics(24, 32, 28);
    #   endif // ifdef ADAGFX_FONTS_EXTRA_20PT_WHITERABBiT
    #  endif  // ifdef ADAGFX_FONTS_EXTRA_20PT_INCLUDED
    } else if (sParams[0].equals(F("default"))) { // font,default is always available!
      _display->setFont();
      calculateTextMetrics(6, 10);
    } else {
      success = false;
    }
    # else // if ADAGFX_FONTS_INCLUDED
    success = false;
    # endif  // if ADAGFX_FONTS_INCLUDED
  }
  else if (subcommand.equals(F("l")) && (argCount == 5)) { // l: Line
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1]) ||
        invalidCoordinates(nParams[2], nParams[3])) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->drawLine(nParams[0], nParams[1], nParams[2], nParams[3], AdaGFXparseColor(sParams[4], _colorDepth));
    }
  }
  else if (subcommand.equals(F("lh")) && (argCount == 3)) { // lh: Horizontal line
    # if ADAGFX_ARGUMENT_VALIDATION

    if ((nParams[0] < 0) || (nParams[0] > _res_x)) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->drawFastHLine(0, nParams[0], nParams[1], AdaGFXparseColor(sParams[2], _colorDepth));
    }
  }
  else if (subcommand.equals(F("lv")) && (argCount == 3)) { // lv: Vertical line
    # if ADAGFX_ARGUMENT_VALIDATION

    if ((nParams[0] < 0) || (nParams[0] > _res_y)) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->drawFastVLine(nParams[0], 0, nParams[1], AdaGFXparseColor(sParams[2], _colorDepth));
    }
  }
  # if ADAGFX_ENABLE_EXTRA_CMDS
  else if ((subcommand.equals(F("lm")) || subcommand.equals(F("lmr"))) && (argCount >= 5)) { // lm/lmr: Multi-line, multiple coordinates
    uint16_t mcolor   = AdaGFXparseColor(sParams[0], _colorDepth);
    bool     mloop    = true;
    uint8_t  parCount = 0;
    uint8_t  optCount = 0;
    int  cx           = -1;
    int  cy           = -1;
    bool closeLine    = false;
    bool relativeMode = subcommand.equals(F("lmr")); // Use Relative mode
    #  ifndef BUILD_NO_DEBUG
    String log;
    log.reserve(40);
    #  endif // ifndef BUILD_NO_DEBUG

    while (mloop) {
      sParams[optCount] = parseString(string, parCount + 4);       // 0-offset + 1st and 2nd cmd-argument and 1 for color argument

      if (!validIntFromString(sParams[optCount], nParams[optCount]) && !sParams[optCount].isEmpty()) {
        mcolor = AdaGFXparseColor(sParams[optCount], _colorDepth); // Interpret as a color

        if (optCount > 0) { optCount--; }
      }
      mloop     = !sParams[optCount].isEmpty();
      closeLine = sParams[optCount].equals(F("c"));

      if (mloop) { parCount++; optCount++; } // Next argument

      if ((optCount == 4) || closeLine) { // 0..3 = 4th argument or close the line
        if (relativeMode) {
          nParams[2] += nParams[0];
          nParams[3] += nParams[1];
        }
        #  if ADAGFX_ARGUMENT_VALIDATION

        if (invalidCoordinates(nParams[0], nParams[1]) ||
            invalidCoordinates(nParams[2], nParams[3])) {
          success = false;
          mloop   = false; // break out
        } else
        #  endif // if ADAGFX_ARGUMENT_VALIDATION
        {
          if (closeLine) {
            nParams[2] = cx;
            nParams[3] = cy;
            mloop      = false; // Exit after closing the line
          }
          #  ifndef BUILD_NO_DEBUG
          log.clear();
          log += F("AdaGFX: cmd: lm x/y/x1/y1:");
          log += nParams[0];
          log += '/';
          log += nParams[1];
          log += '/';
          log += nParams[2];
          log += '/';
          log += nParams[3];
          log += F(" loop:");
          log += mloop ? 'T' : 'f';
          log += F(" color:");
          log += AdaGFXcolorToString(mcolor, _colorDepth);
          addLog(LOG_LEVEL_INFO, log);
          #  endif // ifndef BUILD_NO_DEBUG
          _display->drawLine(nParams[0], nParams[1], nParams[2], nParams[3], mcolor);

          if ((cx == -1) && (cy == -1)) {
            cx = nParams[0];
            cy = nParams[1];
          }
          nParams[0] = nParams[2]; // Move second set to first set
          nParams[1] = nParams[3];
          optCount   = 2;          // Get second set of arguments only
        }
      }
    }
  }
  # endif // if ADAGFX_ENABLE_EXTRA_CMDS
  else if (subcommand.equals(F("r")) && (argCount == 5)) { // r: Rectangle
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1]) ||
        invalidCoordinates(nParams[0] + nParams[2], nParams[1] + nParams[3])) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->drawRect(nParams[0], nParams[1], nParams[2], nParams[3], AdaGFXparseColor(sParams[4], _colorDepth));
    }
  }
  else if (subcommand.equals(F("rf")) && (argCount == 6)) { // rf: Rectangled, filled
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1]) ||
        invalidCoordinates(nParams[0] + nParams[2], nParams[1] + nParams[3])) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->fillRect(nParams[0], nParams[1], nParams[2], nParams[3], AdaGFXparseColor(sParams[5], _colorDepth));
      _display->drawRect(nParams[0], nParams[1], nParams[2], nParams[3], AdaGFXparseColor(sParams[4], _colorDepth));
    }
  }
  else if (subcommand.equals(F("c")) && (argCount == 4)) { // c: Circle
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1]) ||
        invalidCoordinates(nParams[2], 0)) { // Also check radius
      success = false;
    } else
    # endif  // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->drawCircle(nParams[0], nParams[1], nParams[2], AdaGFXparseColor(sParams[3], _colorDepth));
    }
  }
  else if (subcommand.equals(F("cf")) && (argCount == 5)) { // cf: Circle, filled
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1]) ||
        invalidCoordinates(nParams[2], 0)) { // Also check radius
      success = false;
    } else
    # endif  // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->fillCircle(nParams[0], nParams[1], nParams[2], AdaGFXparseColor(sParams[4], _colorDepth));
      _display->drawCircle(nParams[0], nParams[1], nParams[2], AdaGFXparseColor(sParams[3], _colorDepth));
    }
  }
  else if (subcommand.equals(F("t")) && (argCount == 7)) { // t: Triangle
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1]) ||
        invalidCoordinates(nParams[2], nParams[3]) ||
        invalidCoordinates(nParams[4], nParams[5])) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->drawTriangle(nParams[0], nParams[1], nParams[2], nParams[3], nParams[4], nParams[5],
                             AdaGFXparseColor(sParams[6], _colorDepth));
    }
  }
  else if (subcommand.equals(F("tf")) && (argCount == 8)) { // tf: Triangle, filled
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1]) ||
        invalidCoordinates(nParams[2], nParams[3]) ||
        invalidCoordinates(nParams[4], nParams[5])) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->fillTriangle(nParams[0],
                             nParams[1],
                             nParams[2],
                             nParams[3],
                             nParams[4],
                             nParams[5],
                             AdaGFXparseColor(sParams[7], _colorDepth));
      _display->drawTriangle(nParams[0],
                             nParams[1],
                             nParams[2],
                             nParams[3],
                             nParams[4],
                             nParams[5],
                             AdaGFXparseColor(sParams[6], _colorDepth));
    }
  }
  else if (subcommand.equals(F("rr")) && (argCount == 6)) { // rr: Rounded rectangle
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1]) ||
        invalidCoordinates(nParams[0] + nParams[2], nParams[1] + nParams[3]) ||
        invalidCoordinates(nParams[4],              0)) { // Also check radius
      success = false;
    } else
    # endif  // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->drawRoundRect(nParams[0], nParams[1], nParams[2], nParams[3], nParams[4], AdaGFXparseColor(sParams[5], _colorDepth));
    }
  }
  else if (subcommand.equals(F("rrf")) && (argCount == 7)) { // rrf: Rounded rectangle, filled
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1]) ||
        invalidCoordinates(nParams[0] + nParams[2], nParams[1] + nParams[3]) ||
        invalidCoordinates(nParams[4],              0)) { // Also check radius
      success = false;
    } else
    # endif  // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->fillRoundRect(nParams[0], nParams[1], nParams[2], nParams[3], nParams[4], AdaGFXparseColor(sParams[6], _colorDepth));
      _display->drawRoundRect(nParams[0], nParams[1], nParams[2], nParams[3], nParams[4], AdaGFXparseColor(sParams[5], _colorDepth));
    }
  }
  else if (subcommand.equals(F("px")) && (argCount == 3)) { // px: Pixel
    # if ADAGFX_ARGUMENT_VALIDATION

    if (invalidCoordinates(nParams[0], nParams[1])) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->drawPixel(nParams[0], nParams[1], AdaGFXparseColor(sParams[2], _colorDepth));
    }
  }
  else if ((subcommand.equals(F("pxh")) || subcommand.equals(F("pxv"))) && (argCount > 2)) { // pxh/pxv: Pixels, hor./vert. incremented
    # if ADAGFX_ARGUMENT_VALIDATION                                                          // merged loop is smaller than 2 separate loops

    if (invalidCoordinates(nParams[0], nParams[1])) {
      success = false;
    } else
    # endif // if ADAGFX_ARGUMENT_VALIDATION
    {
      _display->startWrite();
      _display->writePixel(nParams[0], nParams[1], AdaGFXparseColor(sParams[2], _colorDepth));
      loop = true;
      uint8_t h     = 0;
      uint8_t v     = 0;
      bool    isPxh = subcommand.equals(F("pxh"));

      if (isPxh) {
        h++;
      } else {
        v++;
      }

      while (loop) {
        String color = parseString(string, h + v + 5); // 5 = 2 + 3 already parsed merged loop is smaller than 2 separate loops

        if (color.isEmpty()
            # if ADAGFX_ARGUMENT_VALIDATION
            || invalidCoordinates(nParams[0] + h, nParams[1] + v)
            # endif // if ADAGFX_ARGUMENT_VALIDATION
            ) {
          loop = false;
        } else {
          _display->writePixel(nParams[0] + h, nParams[1] + v, AdaGFXparseColor(color, _colorDepth));

          if (isPxh) {
            h++;
          } else {
            v++;
          }
        }
        delay(0);
      }
      _display->endWrite();
    }
  }
  # if ADAGFX_ENABLE_BMP_DISPLAY
  else if (subcommand.equals(F("bmp")) && (argCount == 3)) { // bmp,x,y,filename.bmp : show bmp from file
    if (!sParams[2].isEmpty()) {
      success = showBmp(sParams[2], nParams[0], nParams[1]);
    } else {
      success = false;
    }
  }
  # endif // if ADAGFX_ENABLE_BMP_DISPLAY
  else {
    success = false;
  }

  return success;
}

/****************************************************************************
 * printText: Print text on display at a specific pixel or column/row location
 ***************************************************************************/
void AdafruitGFX_helper::printText(const char    *string,
                                   int            X,
                                   int            Y,
                                   unsigned int   textSize,
                                   unsigned short color,
                                   unsigned short bkcolor) {
  int16_t  _x = X;
  int16_t  _y = Y + (_heightOffset * textSize);
  uint16_t _w = 0;
  uint8_t  _h = 0;
  int16_t  x1, y1;
  uint16_t w0, w1, h1;

  if (_columnRowMode) {
    _x = X * (_fontwidth * textSize); // We need this multiple times
    _y = (Y * (_fontheight * textSize))  + (_heightOffset * textSize);
  }

  if (_textBackFill && (color != bkcolor)) { // Draw extra lines above text
    // Estimate used width
    _w = _textPrintMode == AdaGFXTextPrintMode::ContinueToNextLine ?
         strlen(string) * _fontwidth * textSize :
         ((_textcols + 1) * _fontwidth * textSize) - _x;

    do {
      _display->drawLine(_x, _y, _x + _w - 1, _y, bkcolor);
      _h++;
      _y++; // Shift down entire line
    } while (_h < textSize);
  }

  _display->setCursor(_x, _y);
  _display->setTextColor(color, bkcolor);
  _display->setTextSize(textSize);

  String newString = string;

  if (_textPrintMode != AdaGFXTextPrintMode::ContinueToNextLine) {
    if (_isProportional) {                                              // Proportional font. This is rather slow!
      _display->getTextBounds(newString, _x, _y, &x1, &y1, &w1, &h1);   // Count length

      while ((newString.length() > 0) && ((_x + w1) > _res_x)) {
        newString.remove(newString.length() - 1);                       // Cut last character off
        _display->getTextBounds(newString, _x, _y, &x1, &y1, &w1, &h1); // Re-count length
      }
    }
    else {                                                              // Fixed width font
      if (newString.length() > static_cast<uint16_t>(_textcols - (_x / (_fontwidth * textSize)))) {
        newString = newString.substring(0, _textcols - (_x / (_fontwidth * textSize)));
      }
    }
  }

  w1 = 0;

  _display->getTextBounds(F(" "), _x, _y, &x1, &y1, &w1, &h1);

  if (w1 == 0) { w1 = _fontwidth; } // Some fonts seem to have a 0-wide space, this is an endless loop protection

  if (_textPrintMode == AdaGFXTextPrintMode::ClearThenTruncate) { // Clear before print
    _display->setCursor(_x, _y);
    w0 = 0;
    uint16_t w2 = 0;

    _display->getTextBounds(newString, _x, _y, &x1, &y1, &w2, &h1); // Count length in pixels

    for (; (w0 < w2); w0 += w1) {                                   // Clear previously used text with spaces
      _display->print(' ');
    }
    delay(0);
  }

  _display->setCursor(_x, _y);
  _display->print(newString);

  _display->getTextBounds(newString, _x, _y, &x1, &y1, &w0, &h1); // Count length in pixels

  for (; ((_x + w0) < _res_x) && _textPrintMode != AdaGFXTextPrintMode::ContinueToNextLine; w0 += w1) {
    _display->print(' ');
  }

  if (_textBackFill && (color != bkcolor)) { // Draw extra lines below text
    _y += ((_fontheight - 1) * textSize);
    _h  = 0;

    do {
      _display->drawLine(_x, _y - _h, _x + _w - 1, _y - _h, bkcolor);
      _h++;
    } while (_h <= textSize);
  }
}

/****************************************************************************
 * color565: convert r, g, b colors to rgb565 (by bit-shifting)
 ***************************************************************************/
uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
  return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

/****************************************************************************
 * AdaGFXparseColor: translate color name, rgb565 hex #rGgb or rgb hex #RRGGBB to an RGB565 value,
 * also applies color reduction to mono(2), duo(3), quadro(4), septo(7), octo(8), quinto(16)-chrome colors
 ***************************************************************************/

// Parse color string to RGB565 color
// param [in] s : The color string (white, red, ...)
// Param [in] colorDepth: The requiresed color depth, default: FullColor
// param [in] defaultWhite: Return White color if empty, default: true
// return : color (default ADAGFX_WHITE)
uint16_t AdaGFXparseColor(String& s, AdaGFXColorDepth colorDepth, bool emptyIsBlack) {
  s.toLowerCase();
  int32_t result = -1; // No result yet

  if ((colorDepth == AdaGFXColorDepth::Monochrome) ||
      (colorDepth == AdaGFXColorDepth::BlackWhiteRed) ||
      (colorDepth == AdaGFXColorDepth::BlackWhite2Greyscales)) { // Only a limited set of colors is supported
    if (s.equals(F("black")))   { return static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_BLACK); }

    if (s.equals(F("white")))   { return static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_WHITE); }

    if (s.equals(F("inverse"))) { return static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_INVERSE); }

    if (s.equals(F("red")))     { return static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_RED); }

    // Synonym for red
    if (s.equals(F("yellow")))  { return static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_RED); }

    if (s.equals(F("dark")))    { return static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_DARK); }

    if (s.equals(F("light")))   { return static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_LIGHT); }

    // If we get this far, return the default
    return static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_WHITE);
  # if ADAGFX_SUPPORT_7COLOR
  } else if (colorDepth == AdaGFXColorDepth::SevenColor) {
    if (s.equals(F("black")))  { result = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLACK); }

    if (s.equals(F("white")))  { result = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_WHITE); }

    if (s.equals(F("green")))  { result = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_GREEN); }

    if (s.equals(F("blue")))   { result = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLUE); }

    if (s.equals(F("red")))    { result = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_RED); }

    if (s.equals(F("yellow"))) { result = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_YELLOW); }

    if (s.equals(F("orange"))) { result = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_ORANGE); }
  # endif // if ADAGFX_SUPPORT_7COLOR
  } else { // Some predefined colors
    if (s.equals(F("black")))       { result = ADAGFX_BLACK; }

    if (s.equals(F("navy")))        { result = ADAGFX_NAVY; }

    if (s.equals(F("darkgreen")))   { result = ADAGFX_DARKGREEN; }

    if (s.equals(F("darkcyan")))    { result = ADAGFX_DARKCYAN; }

    if (s.equals(F("maroon")))      { result = ADAGFX_MAROON; }

    if (s.equals(F("purple")))      { result = ADAGFX_PURPLE; }

    if (s.equals(F("olive")))       { result = ADAGFX_OLIVE; }

    if (s.equals(F("lightgrey")))   { result = ADAGFX_LIGHTGREY; }

    if (s.equals(F("darkgrey")))    { result = ADAGFX_DARKGREY; }

    if (s.equals(F("blue")))        { result = ADAGFX_BLUE; }

    if (s.equals(F("green")))       { result = ADAGFX_GREEN; }

    if (s.equals(F("cyan")))        { result = ADAGFX_CYAN; }

    if (s.equals(F("red")))         { result = ADAGFX_RED; }

    if (s.equals(F("magenta")))     { result = ADAGFX_MAGENTA; }

    if (s.equals(F("yellow")))      { result = ADAGFX_YELLOW; }

    if (s.equals(F("white")))       { result = ADAGFX_WHITE; }

    if (s.equals(F("orange")))      { result = ADAGFX_ORANGE; }

    if (s.equals(F("greenyellow"))) { result = ADAGFX_GREENYELLOW; }

    if (s.equals(F("pink")))        { result = ADAGFX_PINK; }
  }

  // Parse default hex #rgb565 (hex) string (1-4 hex nibbles accepted!)
  if ((result == -1) && (s.length() >= 2) && (s.length() <= 5) && (s[0] == '#')) {
    result = hexToUL(&s[1]);
  }

  // Parse default hex #RRGGBB string (must be 6 hex nibbles!)
  if ((result == -1) && (s.length() == 7) && (s[0] == '#')) {
    // convrt to long value in base16, then split up into r, g, b values
    uint32_t number = hexToUL(&s[1]);

    // uint32_t r = number >> 16 & 0xFF;
    // uint32_t g = number >> 8 & 0xFF;
    // uint32_t b = number & 0xFF;
    // convert to color565 (as used by adafruit lib)
    result = color565(number >> 16 & 0xFF, number >> 8 & 0xFF, number & 0xFF);
  }

  if ((result == -1) || (result == ADAGFX_WHITE)) { // Default & don't convert white
    if (
      # if ADAGFX_SUPPORT_7COLOR
      (colorDepth >= AdaGFXColorDepth::SevenColor) &&
      # endif // if ADAGFX_SUPPORT_7COLOR
      (colorDepth <= AdaGFXColorDepth::SixteenColor)) {
      result = static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_BLACK); // Monochrome fallback, compatible 7-color
    } else {
      if (emptyIsBlack) {
        result = ADAGFX_BLACK;
      } else {
        result = ADAGFX_WHITE; // Color fallback value
      }
    }
  } else {
    // Reduce colors?
    switch (colorDepth) {
      case AdaGFXColorDepth::Monochrome:
      case AdaGFXColorDepth::BlackWhiteRed:
      case AdaGFXColorDepth::BlackWhite2Greyscales:
        // Unsupported at this point, but compiler needs the cases because of the enum class
        break;
      # if ADAGFX_SUPPORT_7COLOR
      case AdaGFXColorDepth::SevenColor:
        result = AdaGFXrgb565ToColor7(result); // Convert
        break;
      # endif  // if ADAGFX_SUPPORT_7COLOR
      case AdaGFXColorDepth::EightColor:
        result = color565((result >> 11 & 0x1F) / 4, (result >> 5 & 0x3F) / 4, (result & 0x1F) / 4); // reduce colors factor 4
        break;
      case AdaGFXColorDepth::SixteenColor:
        result = color565((result >> 11 & 0x1F) / 2, (result >> 5 & 0x3F) / 2, (result & 0x1F) / 2); // reduce colors factor 2
        break;
      case AdaGFXColorDepth::FullColor:
        // No color reduction
        break;
    }
  }
  return result;
}

const __FlashStringHelper* AdaGFXcolorToString_internal(uint16_t         color,
                                                        AdaGFXColorDepth colorDepth,
                                                        bool             blackIsEmpty);

// Add a single optionvalue of a color to a datalist (internal/private)
void AdaGFXaddHtmlDataListColorOptionValue(uint16_t         color,
                                           AdaGFXColorDepth colorDepth) {
  const __FlashStringHelper *clr = AdaGFXcolorToString_internal(color, colorDepth, false);

  if (clr != F("*")) {
    addHtml(F("<option value=\""));
    addHtml(clr);
    addHtml(F("\">"));
    addHtml(clr);
    addHtml(F("</option>"));
  }
}

/*****************************************************************************************
 * Generate a html 'datalist' of the colors available for selected colorDepth, with id provided
 ****************************************************************************************/
void AdaGFXHtmlColorDepthDataList(const __FlashStringHelper *id,
                                  AdaGFXColorDepth           colorDepth) {
  addHtml(F("<datalist id=\""));
  addHtml(id);
  addHtml(F("\">"));

  switch (colorDepth) {
    case AdaGFXColorDepth::BlackWhiteRed:
    case AdaGFXColorDepth::BlackWhite2Greyscales:
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_RED),     colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_DARK),    colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_LIGHT),   colorDepth);
    case AdaGFXColorDepth::Monochrome:
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_BLACK),   colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_WHITE),   colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_INVERSE), colorDepth);
      break;
    # if ADAGFX_SUPPORT_7COLOR
    case AdaGFXColorDepth::SevenColor:
    {
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLACK),  colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_WHITE),  colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_GREEN),  colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLUE),   colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_RED),    colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_YELLOW), colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_ORANGE), colorDepth);
      break;
    }
    # endif // if ADAGFX_SUPPORT_7COLOR
    case AdaGFXColorDepth::EightColor: // TODO: Sort out the actual 8/16 color options
    case AdaGFXColorDepth::SixteenColor:
    case AdaGFXColorDepth::FullColor:
    {
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_BLACK,       colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_NAVY,        colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_DARKGREEN,   colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_DARKCYAN,    colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_MAROON,      colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_PURPLE,      colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_OLIVE,       colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_LIGHTGREY,   colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_DARKGREY,    colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_BLUE,        colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_GREEN,       colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_CYAN,        colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_RED,         colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_MAGENTA,     colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_YELLOW,      colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_WHITE,       colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_ORANGE,      colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_GREENYELLOW, colorDepth);
      AdaGFXaddHtmlDataListColorOptionValue(ADAGFX_PINK,        colorDepth);
      break;
    }
  }
  addHtml(F("</datalist>"));
}

/*****************************************************************************************
 * Convert an RGB565 color (number) to it's name or the #rgb565 hex string, based on depth
 ****************************************************************************************/
String AdaGFXcolorToString(uint16_t         color,
                           AdaGFXColorDepth colorDepth,
                           bool             blackIsEmpty) {
  String result = AdaGFXcolorToString_internal(color, colorDepth, blackIsEmpty);

  if (result.equals(F("*"))) {
    result  = '#';
    result += String(color, HEX);
    result.toUpperCase();
  }
  return result;
}

const __FlashStringHelper* AdaGFXcolorToString_internal(uint16_t         color,
                                                        AdaGFXColorDepth colorDepth,
                                                        bool             blackIsEmpty) {
  switch (colorDepth) {
    case AdaGFXColorDepth::Monochrome:
    case AdaGFXColorDepth::BlackWhiteRed:
    case AdaGFXColorDepth::BlackWhite2Greyscales:
    {
      switch (color) {
        case static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_BLACK): return blackIsEmpty ? F("") : F("black");
        case ADAGFX_WHITE: // Fall through
        case static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_WHITE): return F("white");
        case static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_INVERSE): return F("inverse");
        case static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_RED): return F("red");
        case static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_DARK): return F("dark");
        case static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_LIGHT): return F("light");
        default:
          break;
      }
      break;
    }
    # if ADAGFX_SUPPORT_7COLOR
    case AdaGFXColorDepth::SevenColor:
    {
      switch (color) {
        case static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLACK): return blackIsEmpty ? F("") : F("black");
        case ADAGFX_WHITE: // Fall through
        case static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_WHITE): return F("white");
        case static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_GREEN): return F("green");
        case static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLUE): return F("blue");
        case static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_RED): return F("red");
        case static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_YELLOW): return F("yellow");
        case static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_ORANGE): return F("orange");
        default:
          break;
      }
      break;
    }
    # endif // if ADAGFX_SUPPORT_7COLOR
    case AdaGFXColorDepth::EightColor:
    case AdaGFXColorDepth::SixteenColor:
    case AdaGFXColorDepth::FullColor:
    {
      switch (color) {
        case ADAGFX_BLACK:  return blackIsEmpty ? F("") : F("black");
        case ADAGFX_NAVY: return F("navy");
        case ADAGFX_DARKGREEN: return F("darkgreen");
        case ADAGFX_DARKCYAN: return F("darkcyan");
        case ADAGFX_MAROON: return F("maroon");
        case ADAGFX_PURPLE: return F("purple");
        case ADAGFX_OLIVE: return F("olive");
        case ADAGFX_LIGHTGREY: return F("lightgrey");
        case ADAGFX_DARKGREY: return F("darkgrey");
        case ADAGFX_BLUE: return F("blue");
        case ADAGFX_GREEN: return F("green");
        case ADAGFX_CYAN: return F("cyan");
        case ADAGFX_RED: return F("red");
        case ADAGFX_MAGENTA: return F("magenta");
        case ADAGFX_YELLOW: return F("yellow");
        case ADAGFX_WHITE: return F("white");
        case ADAGFX_ORANGE: return F("orange");
        case ADAGFX_GREENYELLOW: return F("greenyellow");
        case ADAGFX_PINK: return F("pink");
        default:
          break;
      }
      break;
    }
  }
  return F("*");
}

# if ADAGFX_SUPPORT_7COLOR

/****************************************************************************
 * AdaGFXrgb565ToColor7: Convert a rgb565 color to the 7 colors supported by 7-color eInk displays
 * Borrowed from https://github.com/ZinggJM/GxEPD2 color7() routine
 ***************************************************************************/
uint16_t AdaGFXrgb565ToColor7(uint16_t color) {
  uint16_t cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_WHITE); // Default = white

  uint16_t red   = (color & 0xF800);
  uint16_t green = (color & 0x07E0) << 5;
  uint16_t blue  = (color & 0x001F) << 11;

  if ((red < 0x8000) && (green < 0x8000) && (blue < 0x8000)) {
    cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLACK); // black
  }
  else if ((red >= 0x8000) && (green >= 0x8000) && (blue >= 0x8000)) {
    cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_WHITE); // white
  }
  else if ((red >= 0x8000) && (blue >= 0x8000)) {
    if (red > blue) {
      cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_RED);
    } else {
      cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLUE); // red, blue
    }
  }
  else if ((green >= 0x8000) && (blue >= 0x8000)) {
    if (green > blue) {
      cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_GREEN);
    } else {
      cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLUE); // green, blue
    }
  }
  else if ((red >= 0x8000) && (green >= 0x8000)) {
    static const uint16_t y2o_lim = ((ADAGFX_YELLOW - ADAGFX_ORANGE) / 2 + (ADAGFX_ORANGE & 0x07E0)) << 5;

    if (green > y2o_lim) {
      cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_YELLOW);
    } else {
      cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_ORANGE); // yellow, orange
    }
  }
  else if (red >= 0x8000) {
    cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_RED);   // red
  }
  else if (green >= 0x8000) {
    cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_GREEN); // green
  }
  else {
    cv7 = static_cast<uint16_t>(AdaGFX7Colors::ADAGFX7C_BLUE);  // blue
  }
  return cv7;
}

# endif // if ADAGFX_SUPPORT_7COLOR

/****************************************************************************
 * getTextMetrics: Returns the metrics related to current font
 ***************************************************************************/
void AdafruitGFX_helper::getTextMetrics(uint16_t& textcols,
                                        uint16_t& textrows,
                                        uint8_t & fontwidth,
                                        uint8_t & fontheight,
                                        uint8_t & fontscaling,
                                        uint8_t & heightOffset,
                                        uint16_t& xpix,
                                        uint16_t& ypix) {
  textcols     = _textcols;
  textrows     = _textrows;
  fontwidth    = _fontwidth;
  fontheight   = _fontheight;
  fontscaling  = _fontscaling;
  heightOffset = _heightOffset;
  xpix         = _res_x;
  ypix         = _res_y;
}

/****************************************************************************
 * getColors: Returns the current text colors
 ***************************************************************************/
void AdafruitGFX_helper::getColors(uint16_t& fgcolor,
                                   uint16_t& bgcolor) {
  fgcolor = _fgcolor;
  bgcolor = _bgcolor;
}

/****************************************************************************
 * calculateTextMetrics: Recalculate the text mertics based on supplied font parameters
 ***************************************************************************/
void AdafruitGFX_helper::calculateTextMetrics(uint8_t fontwidth,
                                              uint8_t fontheight,
                                              int8_t  heightOffset,
                                              bool    isProportional) {
  _fontwidth      = fontwidth;
  _fontheight     = fontheight;
  _heightOffset   = heightOffset;
  _isProportional = isProportional;
  _textcols       = _res_x / (_fontwidth * _fontscaling);
  _textrows       = _res_y / ((_fontheight + _heightOffset) * _fontscaling);

  # ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(ADAGFX_LOG_LEVEL)) {
    String log;
    log.reserve(60);
    log += F("AdaGFX:");

    if (!_trigger.isEmpty()) {
      log += F(" tr: ");
      log += _trigger;
    }
    log += F(" x: ");
    log += _res_x;
    log += F(", y: ");
    log += _res_y;
    log += F(", text columns: ");
    log += _textcols;
    log += F(" rows: ");
    log += _textrows;
    addLogMove(ADAGFX_LOG_LEVEL, log);
  }
  # endif // ifndef BUILD_NO_DEBUG
}

# if ADAGFX_ARGUMENT_VALIDATION

/****************************************************************************
 * invalidCoordinates: Check if X/Y coordinates stay within the limits of the display,
 * default pixel-mode, colRowMode true = character mode.
 * If Y == 0 then X is allowed the max. value of the display size.
 * *** Returns TRUE when invalid !! ***
 ***************************************************************************/
bool AdafruitGFX_helper::invalidCoordinates(int  X,
                                            int  Y,
                                            bool colRowMode) {
  #  ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(ADAGFX_LOG_LEVEL)) {
    String log;

    log.reserve(49);
    log += F("invalidCoordinates: X:");
    log += X;
    log += '/';
    log += (colRowMode ? _textcols : _res_x);
    log += F(" Y:");
    log += Y;
    log += '/';
    log += (colRowMode ? _textrows : _res_y);
    addLogMove(ADAGFX_LOG_LEVEL, log);
  }
  #  endif // ifndef BUILD_NO_DEBUG

  if (!_useValidation) { return false; }

  if (colRowMode) {
    return !((X >= 0) && (X <= _textcols) &&
             (Y >= 0) && (Y <= _textrows));
  } else {
    if (Y == 0) { // Y == 0: Accept largest x/y size value for x
      return !((X >= 0) && (X <= std::max(_res_x, _res_y)));
    } else {
      return !((X >= 0) && (X <= _res_x) &&
               (Y >= 0) && (Y <= _res_y));
    }
  }
}

# endif // if ADAGFX_ARGUMENT_VALIDATION

void AdafruitGFX_helper::setRotation(uint8_t m) {
  uint8_t rotation = m & 3;

  _display->setRotation(m); // Set rotation 0/1/2/3

  switch (rotation) {
    case 0:
    case 2:
      _res_x = _display_x;
      _res_y = _display_y;
      break;
    case 1:
    case 3:
      _res_x = _display_y;
      _res_y = _display_x;
      break;
  }
  calculateTextMetrics(_fontwidth, _fontheight, _heightOffset, _isProportional);
}

# if ADAGFX_ENABLE_BMP_DISPLAY

/**
 * CPA (Copy/paste/adapt) from Adafruit_ImageReader::coreBMP()
 * Changes:
 * - No 'load to memory' feature
 * - No special handling of SD Filesystem/FAT, but File only
 * - Adds support for non-SPI displays (like NeoPixel Matrix, and possibly I2C displays, once supported)
 */
bool AdafruitGFX_helper::showBmp(const String& filename,
                                 int16_t       x,
                                 int16_t       y) {
  bool transact = true;           // Enable transaction support to work proper with SD czrd, when enabled
  bool status   = false;          // IMAGE_SUCCESS on valid file
  uint16_t  tftbuf[BUFPIXELS];
  uint16_t *dest = tftbuf;        // TFT working buffer, or NULL if to canvas
  uint32_t  offset;               // Start of image data in file
  uint32_t  headerSize;           // Indicates BMP version
  uint8_t   planes;               // BMP planes
  uint8_t   depth;                // BMP bit depth
  uint32_t  compression = 0;      // BMP compression mode
  uint32_t  colors      = 0;      // Number of colors in palette
  uint16_t *quantized   = NULL;   // 16-bit 5/6/5 color palette
  uint32_t  rowSize;              // >bmpWidth if scanline padding
  uint8_t   sdbuf[3 * BUFPIXELS]; // BMP read buf (R+G+B/pixel)
  int bmpWidth, bmpHeight;        // BMP width & height in pixels

  #  if ((3 * BUFPIXELS) <= 255)
  uint8_t srcidx = sizeof sdbuf;  // Current position in sdbuf
  #  else // if ((3 * BUFPIXELS) <= 255)
  uint16_t srcidx = sizeof sdbuf;
  #  endif // if ((3 * BUFPIXELS) <= 255)
  uint32_t destidx = 0;
  bool     flip = true;      // BMP is stored bottom-to-top
  uint32_t bmpPos = 0;       // Next pixel position in file
  int loadWidth, loadHeight, // Region being loaded (clipped)
      loadX, loadY;          // "
  int row, col;              // Current pixel pos.
  uint8_t r, g, b;           // Current pixel color
  uint8_t bitIn = 0;         // Bit number for 1-bit data in
  int16_t drow  = 0;
  int16_t dcol  = 0;

  bool canTransact = (nullptr != _tft);

  // If BMP is being drawn off the right or bottom edge of the screen,
  // nothing to do here. NOT an error, just a trivial clip operation.
  if (_tft && ((x >= _tft->width()) || (y >= _tft->height()))) {
    addLog(LOG_LEVEL_INFO, F("showBmp: coordinates off display"));
    return false;
  }

  // Open requested file on storage
  // Search flash file system first, then SD if present
  file = tryOpenFile(filename, "r");
  #  if FEATURE_SD

  if (!file) {
    file = SD.open(filename.c_str(), "r");
  }
  #  endif // if FEATURE_SD

  if (!file) {
    addLog(LOG_LEVEL_ERROR, F("showBmp: file not found"));
    return false;
  }

  // Parse BMP header. 0x4D42 (ASCII 'BM') is the Windows BMP signature.
  // There are other values possible in a .BMP file but these are super
  // esoteric (e.g. OS/2 struct bitmap array) and NOT supported here!
  if (readLE16() == 0x4D42) { // BMP signature
    (void)readLE32();         // Read & ignore file size
    (void)readLE32();         // Read & ignore creator bytes
    offset = readLE32();      // Start of image data
    // Read DIB header
    headerSize = readLE32();
    bmpWidth   = readLE32();
    bmpHeight  = readLE32();

    // If bmpHeight is negative, image is in top-down order.
    // This is not canon but has been observed in the wild.
    if (bmpHeight < 0) {
      bmpHeight = -bmpHeight;
      flip      = false;
    }
    planes = readLE16();
    depth  = readLE16(); // Bits per pixel

    // Compression mode is present in later BMP versions (default = none)
    if (headerSize > 12) {
      compression = readLE32();
      (void)readLE32();    // Raw bitmap data size; ignore
      (void)readLE32();    // Horizontal resolution, ignore
      (void)readLE32();    // Vertical resolution, ignore
      colors = readLE32(); // Number of colors in palette, or 0 for 2^depth
      (void)readLE32();    // Number of colors used (ignore)
      // File position should now be at start of palette (if present)
    }

    if (!colors) {
      colors = 1 << depth;
    }
    #  ifndef BUILD_NO_DEBUG
    String log;

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log.reserve(80);
      log += F("showBmp: bitmap w:");
      log += bmpWidth;
      log += F(", h:");
      log += bmpHeight;
      log += F(", dpt:");
      log += depth;
      log += F(", colors:");
      log += colors;
      log += F(", cmp:");
      log += compression;
      log += F(", pl:");
      log += planes;
      log += F(", x:");
      log += x;
      log += F(", y:");
      log += y;
      addLog(LOG_LEVEL_INFO, log);
    }
    #  endif // ifndef BUILD_NO_DEBUG

    loadWidth  = bmpWidth;
    loadHeight = bmpHeight;
    loadX      = 0;
    loadY      = 0;

    if (_display) {
      // Crop area to be loaded (if destination is TFT)
      if (x < 0) {
        loadX      = -x;
        loadWidth += x;
        x          = 0;
      }

      if (y < 0) {
        loadY       = -y;
        loadHeight += y;
        y           = 0;
      }

      if ((x + loadWidth) > _display->width()) {
        loadWidth = _display->width() - x;
      }

      if ((y + loadHeight) > _display->height()) {
        loadHeight = _display->height() - y;
      }
    }
    #  ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log.clear();
      log += F("showBmp: x: ");
      log += x;
      log += F(", y:");
      log += y;
      log += F(", dw:");
      log += _display->width();
      log += F(", dh:");
      log += _display->height();
      addLogMove(LOG_LEVEL_INFO, log);
    }
    #  endif // ifndef BUILD_NO_DEBUG

    if ((planes == 1) && (compression == 0)) { // Only uncompressed is handled
      // BMP rows are padded (if needed) to 4-byte boundary
      rowSize = ((depth * bmpWidth + 31) / 32) * 4;

      if ((depth == 24) || (depth == 1)) {           // BGR or 1-bit bitmap format
        if (dest) {                                  // Supported format, alloc OK, etc.
          status = true;

          if ((loadWidth > 0) && (loadHeight > 0)) { // Clip top/left
            _display->startWrite();                  // Start SPI (regardless of transact)

            if (canTransact) {
              _tft->setAddrWindow(x, y, loadWidth, loadHeight);
            }

            if ((depth >= 16) ||
                (quantized = (uint16_t *)malloc(colors * sizeof(uint16_t)))) {
              if (depth < 16) {
                // Load and quantize color table
                for (uint16_t c = 0; c < colors; c++) {
                  b = file.read();
                  g = file.read();
                  r = file.read();
                  (void)file.read(); // Ignore 4th byte
                  quantized[c] =
                    ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
                }
              }

              for (row = 0; row < loadHeight; row++) { // For each scanline...
                delay(0);                              // Keep ESP8266 happy

                // Seek to start of scan line.  It might seem labor-intensive
                // to be doing this on every line, but this method covers a
                // lot of gritty details like cropping, flip and scanline
                // padding. Also, the seek only takes place if the file
                // position actually needs to change (avoids a lot of cluster
                // math in SD library).
                if (flip) { // Bitmap is stored bottom-to-top order (normal BMP)
                  bmpPos = offset + (bmpHeight - 1 - (row + loadY)) * rowSize;
                } else {    // Bitmap is stored top-to-bottom
                  bmpPos = offset + (row + loadY) * rowSize;
                }

                if (depth == 24) {
                  bmpPos += loadX * 3;
                } else {
                  bmpPos += loadX / 8;
                  bitIn   = 7 - (loadX & 7);
                }

                if (file.position() != bmpPos) {        // Need seek?
                  if (transact && canTransact) {
                    _tft->dmaWait();
                    _tft->endWrite();                   // End TFT SPI transaction
                  }
                  file.seek(bmpPos);                    // Seek = SD transaction
                  srcidx = sizeof sdbuf;                // Force buffer reload
                }

                for (col = 0; col < loadWidth; col++) { // For each pixel...
                  if (srcidx >= sizeof sdbuf) {         // Time to load more?
                    if (transact && canTransact) {
                      _tft->dmaWait();
                      _tft->endWrite();                 // End TFT SPI transact
                    }
                    file.read(sdbuf, sizeof sdbuf);     // Load from SD

                    if (transact && canTransact) {
                      _display->startWrite();           // Start TFT SPI transact
                    }

                    if (destidx) {                      // If buffered TFT data
                      // Non-blocking writes (DMA) have been temporarily
                      // disabled until this can be rewritten with two
                      // alternating 'dest' buffers (else the nonblocking
                      // data out is overwritten in the dest[] write below).
                      // tft->writePixels(dest, destidx, false); // Write it
                      delay(0);

                      if (canTransact) {
                        _tft->writePixels(dest, destidx, true); // Write it
                      } else {
                        // loop over buffer

                        for (uint16_t p = 0; p < destidx; p++) {
                          _display->drawPixel(x + p, y + drow, dest[p]);
                        }
                      }

                      if (col % 33 == 0) { delay(0); }
                      destidx = 0; // and reset dest index
                    }

                    srcidx = 0;    // Reset bmp buf index
                  }

                  if (depth == 24) {
                    // Convert each pixel from BMP to 565 format, save in dest
                    b               = sdbuf[srcidx++];
                    g               = sdbuf[srcidx++];
                    r               = sdbuf[srcidx++];
                    dest[destidx++] =
                      ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
                  } else {
                    // Extract 1-bit color index
                    uint8_t n = (sdbuf[srcidx] >> bitIn) & 1;

                    if (!bitIn) {
                      srcidx++;
                      bitIn = 7;
                    } else {
                      bitIn--;
                    }

                    // Look up in palette, store in tft dest buf
                    dest[destidx++] = quantized[n];
                  }
                  dcol++;
                }                                           // end pixel loop

                if (_tft) {                                 // Drawing to TFT?
                  delay(0);

                  if (destidx) {                            // Any remainders?
                    // See notes above re: DMA
                    _tft->writePixels(dest, destidx, true); // Write it
                    destidx = 0;                            // and reset dest index
                  }
                  _tft->dmaWait();
                  _tft->endWrite();                         // update display
                } else {
                  // loop over buffer
                  if (destidx) {
                    for (uint16_t p = 0; p < destidx; p++) {
                      _display->drawPixel(x + p, y + drow, dest[p]);

                      if (p % 100 == 0) { delay(0); }
                    }
                    destidx = 0; // and reset dest index
                  }
                }

                drow++;
                dcol = 0;
              } // end scanline loop

              if (quantized) {
                free(quantized);  // Palette no longer needed
              }
              delay(0);
            } // end depth>24 or quantized malloc OK
          }                       // end top/left clip
        }                         // end malloc check
      }       // end depth check
    } // end planes/compression check
  }   // end signature

  file.close();
  #  ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_INFO, F("showBmp: Done."));
  #  endif // ifndef BUILD_NO_DEBUG
  return status;

  // }
}

/*!
    @brief   Reads a little-endian 16-bit unsigned value from currently-
             open File, converting if necessary to the microcontroller's
             native endianism. (BMP files use little-endian values.)
    @return  Unsigned 16-bit value, native endianism.
 */
uint16_t AdafruitGFX_helper::readLE16(void) {
  // Big-endian or unknown. Byte-by-byte read will perform reversal if needed.
  return file.read() | ((uint16_t)file.read() << 8);
}

/*!
    @brief   Reads a little-endian 32-bit unsigned value from currently-
             open File, converting if necessary to the microcontroller's
             native endianism. (BMP files use little-endian values.)
    @return  Unsigned 32-bit value, native endianism.
 */
uint32_t AdafruitGFX_helper::readLE32(void) {
  // Big-endian or unknown. Byte-by-byte read will perform reversal if needed.
  return file.read() | ((uint32_t)file.read() << 8) |
         ((uint32_t)file.read() << 16) | ((uint32_t)file.read() << 24);
}

# endif // if ADAGFX_ENABLE_BMP_DISPLAY

#endif // ifdef PLUGIN_USES_ADAFRUITGFX

#include "../Helpers/_Internal_GPIO_pulseHelper.h"


#include "../ESPEasyCore/ESPEasyGPIO.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/ESPEasy_Scheduler.h"
#include "../Helpers/ESPEasy_time_calc.h"
#include "../WebServer/Markup_Forms.h"

#include "../../ESPEasy_common.h"

#include <Arduino.h>

#include <GPIO_Direct_Access.h>

#define GPIO_PLUGIN_ID  1


const __FlashStringHelper * Internal_GPIO_pulseHelper::toString(GPIOtriggerMode mode)
{
  switch (mode) {
    case GPIOtriggerMode::None: return F("None");
    case GPIOtriggerMode::Change: return F("Change");
    case GPIOtriggerMode::Rising: return F("Rising");
    case GPIOtriggerMode::Falling: return F("Falling");
    case GPIOtriggerMode::PulseLow: return F("PULSE Low");
    case GPIOtriggerMode::PulseHigh: return F("PULSE High");
    case GPIOtriggerMode::PulseChange: return F("PULSE Change");
  }
  return F("");
}

void Internal_GPIO_pulseHelper::addGPIOtriggerMode(const __FlashStringHelper *label,
                                                   const __FlashStringHelper *id,
                                                   GPIOtriggerMode            currentSelection)
{
  #define NR_TRIGGER_MODES  7
  const __FlashStringHelper *options[NR_TRIGGER_MODES];
  const int optionValues[NR_TRIGGER_MODES] = {
    static_cast<int>(GPIOtriggerMode::None),
    static_cast<int>(GPIOtriggerMode::Change),
    static_cast<int>(GPIOtriggerMode::Rising),
    static_cast<int>(GPIOtriggerMode::Falling),
    static_cast<int>(GPIOtriggerMode::PulseLow),
    static_cast<int>(GPIOtriggerMode::PulseHigh),
    static_cast<int>(GPIOtriggerMode::PulseChange)
  };

  for (int i = 0; i < NR_TRIGGER_MODES; ++i) {
    options[i] = Internal_GPIO_pulseHelper::toString(static_cast<Internal_GPIO_pulseHelper::GPIOtriggerMode>(optionValues[i]));
  }
  addFormSelector(label, id, NR_TRIGGER_MODES, options, optionValues, static_cast<int>(currentSelection));
}

Internal_GPIO_pulseHelper::Internal_GPIO_pulseHelper(Internal_GPIO_pulseHelper::pulseCounterConfig configuration)
  : config(configuration) {}

Internal_GPIO_pulseHelper::~Internal_GPIO_pulseHelper() {
  detachInterrupt(digitalPinToInterrupt(config.gpio));
}

bool Internal_GPIO_pulseHelper::init()
{
  if (checkValidPortRange(GPIO_PLUGIN_ID, config.gpio)) {
    pinMode(config.gpio, config.pullupPinMode);

    pulseModeData.currentStableState = config.interruptPinMode == GPIOtriggerMode::PulseLow ? HIGH : LOW;
    pulseModeData.lastCheckState     = HIGH;

    // initialize internal variables for PULSE mode handling
    #ifdef PULSE_STATISTIC
    resetStatsErrorVars();
    ISRdata.Step0counter         = ISRdata.pulseTotalCounter;
    pulseModeData.Step1counter   = ISRdata.pulseTotalCounter;
    pulseModeData.Step2OKcounter = ISRdata.pulseTotalCounter;
    pulseModeData.Step3OKcounter = ISRdata.pulseTotalCounter;
    #endif // ifdef PULSE_STATISTIC

    const int intPinMode = static_cast<int>(config.interruptPinMode) & MODE_INTERRUPT_MASK;
    attachInterruptArg(
      digitalPinToInterrupt(config.gpio),
      config.useEdgeMode() ?
        reinterpret_cast<void (*)(void *)>(ISR_edgeCheck) :
        reinterpret_cast<void (*)(void *)>(ISR_pulseCheck),
      this,
      intPinMode);
    return true;
  }
  return false;
}

void Internal_GPIO_pulseHelper::getPulseCounters(unsigned long& pulseCounter, unsigned long& pulseTotalCounter, float& pulseTime_msec)
{
  pulseCounter      = ISRdata.pulseCounter;
  pulseTotalCounter = ISRdata.pulseTotalCounter;
  pulseTime_msec    = static_cast<float>(ISRdata.pulseTime) / 1000.0f;
}

void Internal_GPIO_pulseHelper::setPulseCountTotal(unsigned long pulseTotalCounter)
{
  ISRdata.pulseTotalCounter = pulseTotalCounter;
}

void Internal_GPIO_pulseHelper::setPulseCounter(unsigned long pulseCounter, float pulseTime_msec)
{
  ISRdata.pulseCounter = pulseCounter;
  ISRdata.pulseTime    = static_cast<uint64_t>(pulseTime_msec * 1000.0f);
}

void Internal_GPIO_pulseHelper::resetPulseCounter()
{
  ISRdata.pulseCounter = 0;
  ISRdata.pulseTime    = 0;
}

void Internal_GPIO_pulseHelper::doPulseStepProcessing(int pStep)
{
  switch (pStep)
  {
    case GPIO_PULSE_HELPER_PROCESSING_STEP_0:
      // regularily called to check if the trigger has flagged the next signal edge
    {
      if (ISRdata.initStepsFlags)
      {
        // schedule step 1 in remaining milliseconds from debounce time
        long delayTime =
          static_cast<long>(
            (static_cast<int64_t>(config.debounceTime_micros)) -
            static_cast<int64_t>(us_Since_triggerTimestamp())
            ) / 1000L;

        if (delayTime < 0)
        {
          // if debounce time was too short or we were called too late by Scheduler
          #ifdef PULSE_STATISTIC
          pulseModeData.Step0ODcounter++; // count occurences
          pulseModeData.StepOverdueMax[pStep] = max(pulseModeData.StepOverdueMax[pStep], -delayTime);
          #endif // PULSE_STATISTIC
          delayTime = 0;
        }
        Scheduler.setPluginTaskTimer(delayTime, config.taskIndex, GPIO_PULSE_HELPER_PROCESSING_STEP_1);

        #ifdef PULSE_STATISTIC

        // FIXME TD-er: Why only correct this when statistics are collected?
        ISRdata.triggerTimestamp = getMicros64() + delayTime * 1000L;
        #endif // PULSE_STATISTIC

        // initialization done
        ISRdata.initStepsFlags = false;
      }
      break;
    }

    case GPIO_PULSE_HELPER_PROCESSING_STEP_1: // read pin status
    {
      #ifdef PULSE_STATISTIC
      pulseModeData.Step1counter++;
      pulseModeData.setStepOverdueMax(pStep, msec_Since_triggerTimestamp());
      #endif // PULSE_STATISTIC

      //  read current state from this tasks's GPIO
      pulseModeData.lastCheckState = DIRECT_pinRead(config.gpio);

      // after debounceTime/2, do step 2
      Scheduler.setPluginTaskTimer(config.debounceTime >> 1, config.taskIndex, GPIO_PULSE_HELPER_PROCESSING_STEP_2);

      #ifdef PULSE_STATISTIC

      // FIXME TD-er: Why only correct this when statistics are collected?
      ISRdata.triggerTimestamp = getMicros64() + (config.debounceTime >> 1) * 1000L;
      #endif // PULSE_STATISTIC
      break;
    }

    case GPIO_PULSE_HELPER_PROCESSING_STEP_2: // 1st validation of pin status
    {
      #ifdef PULSE_STATISTIC
      pulseModeData.setStepOverdueMax(pStep, msec_Since_triggerTimestamp());
      #endif // PULSE_STATISTIC

      //  read current state from this tasks's GPIO
      const int pinState = DIRECT_pinRead(config.gpio);

      if (pinState == pulseModeData.lastCheckState)

      // we found stable state
      {
        #ifdef PULSE_STATISTIC
        pulseModeData.Step2OKcounter++;
        #endif // PULSE_STATISTIC
        // after debounceTime/2, do step 3
        Scheduler.setPluginTaskTimer(config.debounceTime >> 1, config.taskIndex, GPIO_PULSE_HELPER_PROCESSING_STEP_3);
      }
      else

      // we found unexpected different pin state
      {
        #ifdef PULSE_STATISTIC
        pulseModeData.Step2NOKcounter++;
        #endif // PULSE_STATISTIC
        // lets ignore previous pin status. It might have been a spike. Try to detect stable signal
        pulseModeData.lastCheckState = pinState;    // now trust the new state
        // after debounceTime/2, do step 2 again
        Scheduler.setPluginTaskTimer(config.debounceTime >> 1, config.taskIndex, GPIO_PULSE_HELPER_PROCESSING_STEP_2);
      }

      #ifdef PULSE_STATISTIC

      // FIXME TD-er: Why only correct this when statistics are collected?
      ISRdata.triggerTimestamp = getMicros64() + (config.debounceTime >> 1) * 1000L;
      #endif // PULSE_STATISTIC
      break;
    }

    case GPIO_PULSE_HELPER_PROCESSING_STEP_3: // 2nd validation of pin status and counting
    {
      #ifdef PULSE_STATISTIC
      pulseModeData.setStepOverdueMax(pStep, msec_Since_triggerTimestamp());
      #endif // PULSE_STATISTIC

      // determine earliest effective start time of current stable pulse (= NOW - 2 * DebounceTime )
      const uint64_t pulseChangeTime = getMicros64() - (config.debounceTime_micros << 1);

      // determine how long the current stable pulse was lasting
      if (pulseModeData.currentStableState == HIGH) // pulse was HIGH
      {
        pulseModeData.pulseHighTime = pulseChangeTime - ISRdata.currentStableStartTime;
      }
      else // pulse was LOW
      {
        pulseModeData.pulseLowTime = pulseChangeTime - ISRdata.currentStableStartTime;
      }

      //  read current state from this tasks's GPIO
      const int pinState = DIRECT_pinRead(config.gpio);

      if (pinState == pulseModeData.lastCheckState)

      // we found the same state as in step 2. It is stable and valid.
      {
        processStablePulse(pinState, pulseChangeTime);
      }
      else

      // we found unexpected different pin state
      {
        #ifdef PULSE_STATISTIC
        pulseModeData.Step3NOKcounter++;
        #endif // PULSE_STATISTIC

        // ignore spike from previous step. It is regarded as spike within previous=current signal. Again try to detect stable signal
        pulseModeData.lastCheckState = pinState;    // now trust the previous=new state
        Scheduler.setPluginTaskTimer(config.debounceTime >> 1, config.taskIndex, GPIO_PULSE_HELPER_PROCESSING_STEP_2);

        #ifdef PULSE_STATISTIC

        // FIXME TD-er: Why only correct this when statistics are collected?
        ISRdata.triggerTimestamp = getMicros64() + (config.debounceTime >> 1) * 1000L;
        #endif // PULSE_STATISTIC
      }
      break;
    }
    default:
    {
      if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
        String log; log.reserve(48);
        log = F("_P003:PLUGIN_TIMER_IN: Invalid processingStep: "); log += pStep;
        addLogMove(LOG_LEVEL_ERROR, log);
      }
      break;
    }
  }
}

uint64_t Internal_GPIO_pulseHelper::us_Since_triggerTimestamp() const {
  return getMicros64() - ISRdata.triggerTimestamp;
}

long Internal_GPIO_pulseHelper::msec_Since_triggerTimestamp() const {
  return static_cast<long>(us_Since_triggerTimestamp()) / 1000L;
}

/*********************************************************************************************\
*  Processing for found stable pulse
\*********************************************************************************************/
void Internal_GPIO_pulseHelper::processStablePulse(int pinState, uint64_t pulseChangeTime)
{
  if (pinState != pulseModeData.currentStableState)

  // The state changed. Previous sable pulse ends, new starts
  {
    #ifdef PULSE_STATISTIC
    pulseModeData.Step3OKcounter++;
    #endif // PULSE_STATISTIC

    // lets terminate the previous pulse and setup start point for new stable one
    pulseModeData.currentStableState = !pulseModeData.currentStableState;
    ISRdata.currentStableStartTime   = pulseChangeTime;

    // now provide the counter result values for the ended pulse ( depending on mode type)
    switch (config.interruptPinMode)
    {
      case GPIOtriggerMode::PulseChange:
      {
        if (pulseModeData.currentStableState == LOW) { // HIGH had ended
          ISRdata.pulseTime = pulseModeData.pulseHighTime;
        }
        else {                                         // LOW has ended
          ISRdata.pulseTime = pulseModeData.pulseLowTime;
        }

        ISRdata.pulseCounter++;
        ISRdata.pulseTotalCounter++;
        break;
      }
      case GPIOtriggerMode::PulseHigh:
      {
        if (pulseModeData.currentStableState == LOW) // HIGH had ended (else do nothing)
        {
          ISRdata.pulseTime = pulseModeData.pulseLowTime + pulseModeData.pulseHighTime;
          ISRdata.pulseCounter++;
          ISRdata.pulseTotalCounter++;
        }
        break;
      }
      case GPIOtriggerMode::PulseLow:
      {
        if (pulseModeData.currentStableState == HIGH) // LOW had ended (else do nothing)
        {
          ISRdata.pulseTime = pulseModeData.pulseLowTime + pulseModeData.pulseHighTime;
          ISRdata.pulseCounter++;
          ISRdata.pulseTotalCounter++;
        }
        break;
      }
      default:
      {
        if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
          String log;
          log.reserve(48);
          log  = F("_P003:PLUGIN_TIMER_IN: Invalid modeType: ");
          log += static_cast<int>(config.interruptPinMode);
          addLogMove(LOG_LEVEL_ERROR, log);
        }
        break;
      }
    }
  }
  else

  // we found the same stable state as before
  {
    #ifdef PULSE_STATISTIC
    pulseModeData.Step3IGNcounter++;
    #endif // PULSE_STATISTIC
    // do nothing. Ignore interupt. previous stable state was confirmed probably after a spike
  }

  #ifdef PULSE_STATISTIC
  doStatisticLogging(pulseModeData.StatsLogLevel);
  #endif // PULSE_STATISTIC

  // allow next pulse check call from interrupt
  ISRdata.processingFlags = false;
}

void IRAM_ATTR Internal_GPIO_pulseHelper::ISR_edgeCheck(Internal_GPIO_pulseHelper *self)
{
  noInterrupts(); // s0170071: avoid nested interrups due to bouncing.

  // legacy edge Mode types
  // KP: we use here P003_currentStableStartTime[taskID] to persist the PulseTime (pulseTimePrevious)
  const uint64_t currentTime          = getMicros64();
  const uint64_t timeSinceLastTrigger = currentTime -
                                        self->ISRdata.currentStableStartTime;

  if (timeSinceLastTrigger > self->config.debounceTime_micros) // check with debounce time for this task
  {
    self->ISRdata.pulseCounter++;
    self->ISRdata.pulseTotalCounter++;
    self->ISRdata.pulseTime              = timeSinceLastTrigger;
    self->ISRdata.currentStableStartTime = currentTime; // reset when counted to determine interval between counted pulses
  }
  interrupts(); // enable interrupts again.
}

void IRAM_ATTR Internal_GPIO_pulseHelper::ISR_pulseCheck(Internal_GPIO_pulseHelper *self)
{
  noInterrupts(); // s0170071: avoid nested interrups due to bouncing.

  // processing for new PULSE mode types
  #ifdef PULSE_STATISTIC
  self->ISRdata.Step0counter++;
  #endif // PULSE_STATISTIC

  // check if processing is allowed (not blocked) for this task (taskID)
  if (!self->ISRdata.processingFlags)
  {
    // initiate processing
    self->ISRdata.processingFlags  = true; // block further initiations as long as async processing is taking place
    self->ISRdata.initStepsFlags   = true; // PLUGIN_FIFTY_PER_SECOND is polling for this flag set
    self->ISRdata.triggerTimestamp = getMicros64();
  }
  interrupts(); // enable interrupts again.
}

#ifdef PULSE_STATISTIC

void Internal_GPIO_pulseHelper::updateStatisticalCounters(int par1) {
  ISRdata.Step0counter         -= ISRdata.pulseTotalCounter - par1;
  pulseModeData.Step1counter   -= ISRdata.pulseTotalCounter - par1;
  pulseModeData.Step2OKcounter -= ISRdata.pulseTotalCounter - par1;
  pulseModeData.Step3OKcounter -= ISRdata.pulseTotalCounter - par1;
}

void Internal_GPIO_pulseHelper::setStatsLogLevel(uint8_t logLevel) {
  pulseModeData.StatsLogLevel = logLevel;
}

/*********************************************************************************************\
*  reset statistical error cunters and overview variables
\*********************************************************************************************/
void Internal_GPIO_pulseHelper::resetStatsErrorVars() {
  // initialize statistical step counters from TotalCounter and error counters with 0
  pulseModeData.Step2NOKcounter = 0;
  pulseModeData.Step3NOKcounter = 0;
  pulseModeData.Step3IGNcounter = 0;
  pulseModeData.Step0ODcounter  = 0;

  for (int pStep = 0; pStep <= P003_PSTEP_MAX; pStep++) {
    pulseModeData.StepOverdueMax[pStep] = 0;
  }
}

/*********************************************************************************************\
*  write statistic counters to logfile
\*********************************************************************************************/
void Internal_GPIO_pulseHelper::doStatisticLogging(uint8_t logLevel)
{
  if (loglevelActiveFor(logLevel)) {
    // Statistic to logfile. E.g: ... [123/1|111|100/5|80/3/4|40] [12243|3244]
    String log;

    if (log.reserve(125)) {
      log  = F("Pulse:");
      log += F("Stats (GPIO) [step0|1|2|3|tot(ok/nok/ign)] [lo|hi]= (");
      log += config.gpio;                       log += F(") [");
      log += ISRdata.Step0counter;              log += '|';
      log += pulseModeData.Step1counter;        log += '|';
      log += pulseModeData.Step2OKcounter;      log += '/';
      log += pulseModeData.Step2NOKcounter;     log += '|';
      log += pulseModeData.Step3OKcounter;      log += '/';
      log += pulseModeData.Step3NOKcounter;     log += '/';
      log += pulseModeData.Step3IGNcounter;     log += '|';
      log += ISRdata.pulseTotalCounter;         log += F("] [");
      log += pulseModeData.pulseLowTime / 1000L;  log += '|';
      log += pulseModeData.pulseHighTime / 1000L; log += ']';
      addLogMove(logLevel, log);
    }
  }
}

/*********************************************************************************************\
*  write collected timing values to logfile
\*********************************************************************************************/
void Internal_GPIO_pulseHelper::doTimingLogging(uint8_t logLevel)
{
  if (loglevelActiveFor(logLevel)) {
    // Timer to logfile. E.g: ... [4|12000|13444|12243|3244]
    String log;

    if (log.reserve(120)) {
      log  = F("Pulse:");
      log += F("OverDueStats (GPIO) [dbTim] {step0OdCnt} [maxOdTimeStep0|1|2|3]= (");
      log += config.gpio;  log += F(") [");
      log += config.debounceTime;  log += F("] {");
      log += pulseModeData.Step0ODcounter;  log += F("} [");

      for (int pStep = 0; pStep <= P003_PSTEP_MAX; pStep++) {
        log += pulseModeData.StepOverdueMax[pStep];

        if (pStep < P003_PSTEP_MAX) { log += '|'; }
      }
      log += ']';
      addLogMove(logLevel, log);
    }
  }
}

#endif // PULSE_STATISTIC

#include "../Helpers/Networking.h"

#include "../Commands/InternalCommands.h"
#include "../CustomBuild/CompiletimeDefines.h"
#include "../DataStructs/TimingStats.h"
#include "../DataTypes/EventValueSource.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasy_backgroundtasks.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../ESPEasyCore/ESPEasyWifi.h"
#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/ESPEasy_Scheduler.h"
#include "../Globals/EventQueue.h"
#include "../Globals/NetworkState.h"
#include "../Globals/Nodes.h"
#include "../Globals/Settings.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Network.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringProvider.h"

#include "../../ESPEasy-Globals.h"

#include <IPAddress.h>
#include <base64.h>
#include <MD5Builder.h>


// Generic Networking routines

// Syslog
// UDP system messaging
// SSDP
//  #if LWIP_VERSION_MAJOR == 2
#define IPADDR2STR(addr) (uint8_t)((uint32_t)addr &  0xFF), (uint8_t)(((uint32_t)addr >> 8) &  0xFF), \
  (uint8_t)(((uint32_t)addr >> 16) &  0xFF), (uint8_t)(((uint32_t)addr >> 24) &  0xFF)

//  #endif

#include <lwip/netif.h>


#ifdef SUPPORT_ARP
# include <lwip/etharp.h>

# ifdef ESP32
#  include <lwip/etharp.h>
#  include <lwip/tcpip.h>

void _etharp_gratuitous_func(struct netif *netif) {
  etharp_gratuitous(netif);
}

void etharp_gratuitous_r(struct netif *netif) {
  tcpip_callback_with_block((tcpip_callback_fn)_etharp_gratuitous_func, netif, 0);
}

# endif // ifdef ESP32

#endif  // ifdef SUPPORT_ARP

#if FEATURE_DOWNLOAD
# ifdef ESP8266
#  include <ESP8266HTTPClient.h>
# endif // ifdef ESP8266
# ifdef ESP32
#  include <HTTPClient.h>
#  include <Update.h>
# endif // ifdef ESP32
#endif  // if FEATURE_DOWNLOAD

#include <vector>

/*********************************************************************************************\
   Syslog client
\*********************************************************************************************/
void sendSyslog(uint8_t logLevel, const String& message)
{
  if ((Settings.Syslog_IP[0] != 0) && NetworkConnected())
  {
    IPAddress broadcastIP(Settings.Syslog_IP[0], Settings.Syslog_IP[1], Settings.Syslog_IP[2], Settings.Syslog_IP[3]);

    FeedSW_watchdog();

    if (portUDP.beginPacket(broadcastIP, Settings.SyslogPort) == 0) {
      // problem resolving the hostname or port
      return;
    }
    uint8_t prio = Settings.SyslogFacility * 8;

    if (logLevel == LOG_LEVEL_ERROR) {
      prio += 3; // syslog error
    }
    else if (logLevel == LOG_LEVEL_INFO) {
      prio += 5; // syslog notice
    }
    else {
      prio += 7;
    }

    // An RFC3164 compliant message must be formated like :  "<PRIO>[TimeStamp ]Hostname TaskName: Message"

    // Using Settings.Name as the Hostname (Hostname must NOT content space)
    {
      String header;
      String hostname = NetworkCreateRFCCompliantHostname(true);
      hostname.trim();
      hostname.replace(' ', '_');
      header.reserve(16 + hostname.length());
      char str[8] = { 0 };
      snprintf_P(str, sizeof(str), PSTR("<%u>"), prio);
      header  = str;
      header += hostname;
      header += F(" EspEasy: ");
      #ifdef ESP8266
      portUDP.write(header.c_str(),                                    header.length());
      #endif // ifdef ESP8266
      #ifdef ESP32
      portUDP.write(reinterpret_cast<const uint8_t *>(header.c_str()), header.length());
      #endif // ifdef ESP32
    }

    const size_t messageLength = message.length();

    for (size_t i = 0; i < messageLength; ++i) {
      #ifdef ESP8266
      portUDP.write(message[i]);
      #endif // ifdef ESP8266
      #ifdef ESP32
      portUDP.write((uint8_t)message[i]);
      #endif // ifdef ESP32
    }
    portUDP.endPacket();
    FeedSW_watchdog();
    delay(0);
  }
}

#if FEATURE_ESPEASY_P2P

/*********************************************************************************************\
   Send event using UDP message
\*********************************************************************************************/
void SendUDPCommand(uint8_t destUnit, const char *data, uint8_t dataLength)
{
  if (!NetworkConnected(10)) {
    return;
  }

  if (destUnit != 0)
  {
    sendUDP(destUnit, (const uint8_t *)data, dataLength);
    delay(10);
  } else {
    for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end(); ++it) {
      if (it->first != Settings.Unit) {
        sendUDP(it->first, (const uint8_t *)data, dataLength);
        delay(10);
      }
    }
  }
  delay(50);
}

/*********************************************************************************************\
   Send UDP message to specific unit (unit 255=broadcast)
\*********************************************************************************************/
void sendUDP(uint8_t unit, const uint8_t *data, uint8_t size)
{
  if (!NetworkConnected(10)) {
    return;
  }

  IPAddress remoteNodeIP = getIPAddressForUnit(unit);

  if (remoteNodeIP[0] == 0) {
    return;
  }

# ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG_MORE)) {
    String log = F("UDP  : Send UDP message to ");
    log += unit;
    addLogMove(LOG_LEVEL_DEBUG_MORE, log);
  }
# endif // ifndef BUILD_NO_DEBUG

  statusLED(true);
  FeedSW_watchdog();
  portUDP.beginPacket(remoteNodeIP, Settings.UDPPort);
  portUDP.write(data, size);
  portUDP.endPacket();
  FeedSW_watchdog();
  delay(0);
}

/*********************************************************************************************\
   Update UDP port (ESPEasy propiertary protocol)
\*********************************************************************************************/
void updateUDPport()
{
  static uint16_t lastUsedUDPPort = 0;

  if (Settings.UDPPort == lastUsedUDPPort) {
    return;
  }

  if (lastUsedUDPPort != 0) {
    portUDP.stop();
    lastUsedUDPPort = 0;
  }

  if (!NetworkConnected()) {
    return;
  }

  if (Settings.UDPPort != 0) {
    if (portUDP.begin(Settings.UDPPort) == 0) {
      if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
        String log = F("UDP : Cannot bind to ESPEasy p2p UDP port ");
        log += String(Settings.UDPPort);
        addLogMove(LOG_LEVEL_ERROR, log);
      }
    } else {
      lastUsedUDPPort = Settings.UDPPort;

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("UDP : Start listening on port ");
        log += String(Settings.UDPPort);
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }
  }
}

/*********************************************************************************************\
   Check UDP messages (ESPEasy propiertary protocol)
\*********************************************************************************************/
boolean runningUPDCheck = false;
void checkUDP()
{
  if (Settings.UDPPort == 0) {
    return;
  }

  if (runningUPDCheck) {
    return;
  }

  runningUPDCheck = true;

  // UDP events
  int packetSize = portUDP.parsePacket();

  if (packetSize > 0 /*&& portUDP.remotePort() == Settings.UDPPort*/)
  {
    statusLED(true);

    IPAddress remoteIP = portUDP.remoteIP();

    if (portUDP.remotePort() == 123)
    {
      // unexpected NTP reply, drop for now...
      runningUPDCheck = false;
      return;
    }

    // UDP_PACKETSIZE_MAX should be as small as possible but still enough to hold all
    // data for PLUGIN_UDP_IN or CPLUGIN_UDP_IN calls
    // This node may also receive other UDP packets which may be quite large
    // and then crash due to memory allocation failures
    if ((packetSize >= 2) && (packetSize < UDP_PACKETSIZE_MAX)) {
      // Allocate buffer to process packet.
      std::vector<char> packetBuffer;
      packetBuffer.resize(packetSize + 1);

      if (packetBuffer.size() >= static_cast<size_t>(packetSize)) {
        memset(&packetBuffer[0], 0, packetSize + 1);
        int len = portUDP.read(&packetBuffer[0], packetSize);

        if (len >= 2) {
          if (static_cast<uint8_t>(packetBuffer[0]) != 255)
          {
            packetBuffer[len] = 0;
            addLog(LOG_LEVEL_DEBUG, &packetBuffer[0]);
            ExecuteCommand_all(EventValueSource::Enum::VALUE_SOURCE_SYSTEM, &packetBuffer[0]);
          }
          else
          {
            // binary data!
            switch (packetBuffer[1])
            {
              case 1: // sysinfo message
              {
                if (len < 13) {
                  break;
                }
                uint8_t unit = packetBuffer[12];
# ifndef BUILD_NO_DEBUG
                MAC_address mac;
                uint8_t     ip[4];

                for (uint8_t x = 0; x < 6; x++) {
                  mac.mac[x] = packetBuffer[x + 2];
                }

                for (uint8_t x = 0; x < 4; x++) {
                  ip[x] = packetBuffer[x + 8];
                }
# endif // ifndef BUILD_NO_DEBUG
                {
                  # ifdef USE_SECOND_HEAP
                  HeapSelectIram ephemeral;

                  // TD-er: Disabled for now as it is suspect for crashes.
                  # endif // ifdef USE_SECOND_HEAP

                  Nodes[unit].age = 0; // Create a new element when not present
                }
                NodesMap::iterator it = Nodes.find(unit);

                if (it != Nodes.end()) {
                  for (uint8_t x = 0; x < 4; x++) {
                    it->second.ip[x] = packetBuffer[x + 8];
                  }
                  it->second.age = 0; // reset 'age counter'

                  if (len >= 41)      // extended packet size
                  {
                    it->second.build = makeWord(packetBuffer[14], packetBuffer[13]);
                    char tmpNodeName[26] = { 0 };
                    memcpy(&tmpNodeName[0], reinterpret_cast<uint8_t *>(&packetBuffer[15]), 25);
                    tmpNodeName[25] = 0;
                    {
                      # ifdef USE_SECOND_HEAP
                      HeapSelectIram ephemeral;
                      # endif // ifdef USE_SECOND_HEAP

                      it->second.nodeName = tmpNodeName;
                      it->second.nodeName.trim();
                    }
                    it->second.nodeType          = packetBuffer[40];
                    it->second.webgui_portnumber = 80;

                    if ((len >= 43) && (it->second.build >= 20107)) {
                      it->second.webgui_portnumber = makeWord(packetBuffer[42], packetBuffer[41]);
                    }
                  }
                }

# ifndef BUILD_NO_DEBUG

                if (loglevelActiveFor(LOG_LEVEL_DEBUG_MORE)) {
                  String log;
                  log += F("UDP  : ");
                  log += mac.toString();
                  log += ',';
                  log += formatIP(ip);
                  log += ',';
                  log += unit;
                  addLogMove(LOG_LEVEL_DEBUG_MORE, log);
                }
# endif // ifndef BUILD_NO_DEBUG
                break;
              }

              default:
              {
                struct EventStruct TempEvent;
                TempEvent.Data = reinterpret_cast<uint8_t *>(&packetBuffer[0]);
                TempEvent.Par1 = remoteIP[3];
                TempEvent.Par2 = len;
                String dummy;
                PluginCall(PLUGIN_UDP_IN, &TempEvent, dummy);
                CPluginCall(CPlugin::Function::CPLUGIN_UDP_IN, &TempEvent);
                break;
              }
            }
          }
        }
      }
    }
  }

  // Flush any remaining content of the packet.
  while (portUDP.available()) {
    // Do not call portUDP.flush() as that's meant to sending the packet (on ESP8266)
    portUDP.read();
  }
  runningUPDCheck = false;
}

/*********************************************************************************************\
   Get formatted IP address for unit
   formatcodes: 0 = default toString(), 1 = empty string when invalid, 2 = 0 when invalid
\*********************************************************************************************/
String formatUnitToIPAddress(uint8_t unit, uint8_t formatCode) {
  IPAddress unitIPAddress = getIPAddressForUnit(unit);

  if (unitIPAddress[0] == 0) { // Invalid?
    switch (formatCode) {
      case 1:                  // Return empty string
      {
        return EMPTY_STRING;
      }
      case 2: // Return "0"
      {
        return F("0");
      }
    }
  }
  return unitIPAddress.toString();
}

/*********************************************************************************************\
   Get IP address for unit
\*********************************************************************************************/
IPAddress getIPAddressForUnit(uint8_t unit) {
  IPAddress remoteNodeIP;

  if (unit == 255) {
    remoteNodeIP = { 255, 255, 255, 255 };
  }
  else {
    NodesMap::iterator it = Nodes.find(unit);

    if (it == Nodes.end()) {
      return remoteNodeIP;
    }

    if (it->second.ip[0] == 0) {
      return remoteNodeIP;
    }
    remoteNodeIP = it->second.ip;
  }
  return remoteNodeIP;
}

/*********************************************************************************************\
   Refresh aging for remote units, drop if too old...
\*********************************************************************************************/
void refreshNodeList()
{
  bool mustSendGratuitousARP = false;

  for (NodesMap::iterator it = Nodes.begin(); it != Nodes.end();) {
    bool mustRemove = true;

    if (it->second.ip[0] != 0) {
      if (it->second.age > 8) {
        // Increase frequency sending ARP requests for 2 minutes
        mustSendGratuitousARP = true;
      }

      if (it->second.age < 10) {
        it->second.age++;
        mustRemove = false;
        ++it;
      }
    }

    if (mustRemove) {
      it = Nodes.erase(it);
    }
  }

  if (mustSendGratuitousARP) {
    Scheduler.sendGratuitousARP_now();
  }
}

/*********************************************************************************************\
   Broadcast system info to other nodes. (to update node lists)
\*********************************************************************************************/
void sendSysInfoUDP(uint8_t repeats)
{
  if ((Settings.UDPPort == 0) || !NetworkConnected(10)) {
    return;
  }

  // TODO: make a nice struct of it and clean up
  // 1 uint8_t 'binary token 255'
  // 1 uint8_t id '1'
  // 6 uint8_t mac
  // 4 uint8_t ip
  // 1 uint8_t unit
  // 2 uint8_t build
  // 25 char name
  // 1 uint8_t node type id

  // send my info to the world...
# ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG_MORE, F("UDP  : Send Sysinfo message"));
# endif // ifndef BUILD_NO_DEBUG

  for (uint8_t counter = 0; counter < repeats; counter++)
  {
    uint8_t data[80] = { 0 };
    data[0] = 255;
    data[1] = 1;

    {
      const MAC_address macread = NetworkMacAddress();

      for (uint8_t x = 0; x < 6; x++) {
        data[x + 2] = macread.mac[x];
      }
    }

    {
      const IPAddress ip = NetworkLocalIP();

      for (uint8_t x = 0; x < 4; x++) {
        data[x + 8] = ip[x];
      }
    }
    data[12] = Settings.Unit;
    data[13] =  lowByte(Settings.Build);
    data[14] = highByte(Settings.Build);
    memcpy(reinterpret_cast<uint8_t *>(data) + 15, Settings.Name, 25);
    data[40] = NODE_TYPE_ID;
    data[41] =  lowByte(Settings.WebserverPort);
    data[42] = highByte(Settings.WebserverPort);
    statusLED(true);

    IPAddress broadcastIP(255, 255, 255, 255);
    FeedSW_watchdog();
    portUDP.beginPacket(broadcastIP, Settings.UDPPort);
    portUDP.write(data, 80);
    portUDP.endPacket();

    if (counter < (repeats - 1)) {
      // FIXME TD-er: Must use scheduler to send out messages, not using delay
      delay(100);
    }
  }

  {
    # ifdef USE_SECOND_HEAP

    // HeapSelectIram ephemeral;
    // TD-er: disabled for now as it is suspect for crashes.
    # endif // ifdef USE_SECOND_HEAP

    Nodes[Settings.Unit].age = 0; // Create new node when not already present.
  }

  // store my own info also in the list
  NodesMap::iterator it = Nodes.find(Settings.Unit);

  if (it != Nodes.end())
  {
    IPAddress ip = NetworkLocalIP();

    for (uint8_t x = 0; x < 4; x++) {
      it->second.ip[x] = ip[x];
    }
    it->second.age      = 0;
    it->second.build    = Settings.Build;
    it->second.nodeType = NODE_TYPE_ID;
  }
}

#endif // FEATURE_ESPEASY_P2P

#if defined(ESP8266)

# if FEATURE_SSDP

/********************************************************************************************\
   Respond to HTTP XML requests for SSDP information
 \*********************************************************************************************/
void SSDP_schema(WiFiClient& client) {
  if (!NetworkConnected(10)) {
    return;
  }

  const IPAddress ip     = NetworkLocalIP();
  const uint32_t  chipId = ESP.getChipId();
  char uuid[64];

  sprintf_P(uuid, PSTR("38323636-4558-4dda-9188-cda0e6%02x%02x%02x"),
            (uint16_t)((chipId >> 16) & 0xff),
            (uint16_t)((chipId >>  8) & 0xff),
            (uint16_t)chipId        & 0xff);

  client.print(F(
                 "HTTP/1.1 200 OK\r\n"
                 "Content-Type: text/xml\r\n"
                 "Connection: close\r\n"
                 "Access-Control-Allow-Origin: *\r\n"
                 "\r\n"
                 "<?xml version=\"1.0\"?>"
                 "<root xmlns=\"urn:schemas-upnp-org:device-1-0\">"
                 "<specVersion>"
                 "<major>1</major>"
                 "<minor>0</minor>"
                 "</specVersion>"
                 "<URLBase>http://"));

  client.print(formatIP(ip));
  client.print(F(":80/</URLBase>"
                 "<device>"
                 "<deviceType>urn:schemas-upnp-org:device:BinaryLight:1</deviceType>"
                 "<friendlyName>"));
  client.print(Settings.Name);
  client.print(F("</friendlyName>"
                 "<presentationURL>/</presentationURL>"
                 "<serialNumber>"));
  client.print(String(ESP.getChipId()));
  client.print(F("</serialNumber>"
                 "<modelName>ESP Easy</modelName>"
                 "<modelNumber>"));
  client.print(getValue(LabelType::GIT_BUILD));
  client.print(F("</modelNumber>"
                 "<modelURL>http://www.letscontrolit.com</modelURL>"
                 "<manufacturer>http://www.letscontrolit.com</manufacturer>"
                 "<manufacturerURL>http://www.letscontrolit.com</manufacturerURL>"
                 "<UDN>uuid:"));
  client.print(String(uuid));
  client.print(F("</UDN></device>"
                 "</root>\r\n"
                 "\r\n"));
}

/********************************************************************************************\
   Global SSDP stuff
 \*********************************************************************************************/

UdpContext *_server;

IPAddress _respondToAddr;
uint16_t  _respondToPort;

bool _pending;
unsigned short _delay;
unsigned long  _process_time;
unsigned long  _notify_time;

#  define SSDP_INTERVAL     1200
#  define SSDP_PORT         1900
#  define SSDP_METHOD_SIZE  10
#  define SSDP_URI_SIZE     2
#  define SSDP_BUFFER_SIZE  64
#  define SSDP_MULTICAST_TTL 2

static const IPAddress SSDP_MULTICAST_ADDR(239, 255, 255, 250);


/********************************************************************************************\
   Launch SSDP listener and send initial notify
 \*********************************************************************************************/
bool SSDP_begin() {
  _pending = false;

  if (_server != nullptr) {
    _server->unref();

    // FIXME TD-er: Shouldn't this also call delete _server ?

    _server = nullptr;
  }

  _server = new (std::nothrow) UdpContext;

  if (_server == nullptr) {
    return false;
  }
  _server->ref();

  ip_addr_t ifaddr;

  ifaddr.addr = NetworkLocalIP();
  ip_addr_t multicast_addr;

  multicast_addr.addr = (uint32_t)SSDP_MULTICAST_ADDR;

  if (igmp_joingroup(&ifaddr, &multicast_addr) != ERR_OK) {
    return false;
  }

#  ifdef CORE_POST_2_5_0

  // Core 2.5.0 changed the signature of some UdpContext function.
  if (!_server->listen(IP_ADDR_ANY, SSDP_PORT)) {
    return false;
  }

  _server->setMulticastInterface(&ifaddr);
  _server->setMulticastTTL(SSDP_MULTICAST_TTL);
  _server->onRx(&SSDP_update);

  if (!_server->connect(&multicast_addr, SSDP_PORT)) {
    return false;
  }
#  else // ifdef CORE_POST_2_5_0

  if (!_server->listen(*IP_ADDR_ANY, SSDP_PORT)) {
    return false;
  }

  _server->setMulticastInterface(ifaddr);
  _server->setMulticastTTL(SSDP_MULTICAST_TTL);
  _server->onRx(&SSDP_update);

  if (!_server->connect(multicast_addr, SSDP_PORT)) {
    return false;
  }
#  endif // ifdef CORE_POST_2_5_0

  SSDP_update();

  return true;
}

/********************************************************************************************\
   Send SSDP messages (notify & responses)
 \*********************************************************************************************/
void SSDP_send(uint8_t method) {
  uint32_t ip = NetworkLocalIP();

  // FIXME TD-er: Why create String objects of these flashstrings?
  String _ssdp_response_template = F(
    "HTTP/1.1 200 OK\r\n"
    "EXT:\r\n"
    "ST: upnp:rootdevice\r\n");

  String _ssdp_notify_template = F(
    "NOTIFY * HTTP/1.1\r\n"
    "HOST: 239.255.255.250:1900\r\n"
    "NT: upnp:rootdevice\r\n"
    "NTS: ssdp:alive\r\n");

  String _ssdp_packet_template = F(
    "%s"                                           // _ssdp_response_template / _ssdp_notify_template
    "CACHE-CONTROL: max-age=%u\r\n"                // SSDP_INTERVAL
    "SERVER: Arduino/1.0 UPNP/1.1 ESPEasy/%u\r\n"  // _modelNumber
    "USN: uuid:%s\r\n"                             // _uuid
    "LOCATION: http://%u.%u.%u.%u:80/ssdp.xml\r\n" // NetworkLocalIP(),
    "\r\n");
  {
    char uuid[64]   = { 0 };
    uint32_t chipId = ESP.getChipId();
    sprintf_P(uuid, PSTR("38323636-4558-4dda-9188-cda0e6%02x%02x%02x"),
              (uint16_t)((chipId >> 16) & 0xff),
              (uint16_t)((chipId >>  8) & 0xff),
              (uint16_t)chipId        & 0xff);

    char *buffer = new (std::nothrow) char[1460]();

    if (buffer == nullptr) { return; }
    int len = snprintf(buffer, 1460,
                       _ssdp_packet_template.c_str(),
                       (method == 0) ? _ssdp_response_template.c_str() : _ssdp_notify_template.c_str(),
                       SSDP_INTERVAL,
                       Settings.Build,
                       uuid,
                       IPADDR2STR(&ip)
                       );

    _server->append(buffer, len);
    delete[] buffer;
  }

  ip_addr_t remoteAddr;
  uint16_t  remotePort;

  if (method == 0) {
    remoteAddr.addr = _respondToAddr;
    remotePort      = _respondToPort;
  } else {
    remoteAddr.addr = SSDP_MULTICAST_ADDR;
    remotePort      = SSDP_PORT;
  }
  _server->send(&remoteAddr, remotePort);
  statusLED(true);
}

/********************************************************************************************\
   SSDP message processing
 \*********************************************************************************************/
void SSDP_update() {
  if (!_pending && _server->next()) {
    ssdp_method_t method = NONE;

    _respondToAddr = _server->getRemoteAddress();
    _respondToPort = _server->getRemotePort();

    typedef enum { METHOD, URI, PROTO, KEY, VALUE, ABORT } states;
    states state = METHOD;

    typedef enum { START, MAN, ST, MX } headers;
    headers header = START;

    uint8_t cursor = 0;
    uint8_t cr     = 0;

    char buffer[SSDP_BUFFER_SIZE] = { 0 };

    while (_server->getSize() > 0) {
      char c = _server->read();

      (c == '\r' || c == '\n') ? cr++ : cr = 0;

      switch (state) {
        case METHOD:

          if (c == ' ') {
            if (strcmp_P(buffer, PSTR("M-SEARCH")) == 0) { method = SEARCH; }
            else if (strcmp_P(buffer, PSTR("NOTIFY")) == 0) { method = NOTIFY; }

            if (method == NONE) { state = ABORT; }
            else { state = URI; }
            cursor = 0;
          } else if (cursor < SSDP_METHOD_SIZE - 1) {
            buffer[cursor++] = c;
            buffer[cursor]   = '\0';
          }
          break;
        case URI:

          if (c == ' ') {
            if (strcmp(buffer, "*")) { state = ABORT; }
            else { state = PROTO; }
            cursor = 0;
          } else if (cursor < SSDP_URI_SIZE - 1) {
            buffer[cursor++] = c;
            buffer[cursor]   = '\0';
          }
          break;
        case PROTO:

          if (cr == 2) {
            state  = KEY;
            cursor = 0;
          }
          break;
        case KEY:

          if (cr == 4) {
            _pending      = true;
            _process_time = millis();
          }
          else if (c == ' ') {
            cursor = 0;
            state  = VALUE;
          }
          else if ((c != '\r') && (c != '\n') && (c != ':') && (cursor < SSDP_BUFFER_SIZE - 1)) {
            buffer[cursor++] = c;
            buffer[cursor]   = '\0';
          }
          break;
        case VALUE:

          if (cr == 2) {
            switch (header) {
              case START:
                break;
              case MAN:
                break;
              case ST:

                if (strcmp_P(buffer, PSTR("ssdp:all"))) {
                  state = ABORT;
                }

                // if the search type matches our type, we should respond instead of ABORT
                if (strcmp_P(buffer, PSTR("urn:schemas-upnp-org:device:BinaryLight:1")) == 0) {
                  _pending      = true;
                  _process_time = millis();
                  state         = KEY;
                }
                break;
              case MX:
                _delay = random(0, atoi(buffer)) * 1000L;
                break;
            }

            if (state != ABORT) {
              state  = KEY;
              header = START;
              cursor = 0;
            }
          } else if ((c != '\r') && (c != '\n')) {
            if (header == START) {
              if (strncmp(buffer, "MA", 2) == 0) { header = MAN; }
              else if (strcmp(buffer, "ST") == 0) { header = ST; }
              else if (strcmp(buffer, "MX") == 0) { header = MX; }
            }

            if (cursor < SSDP_BUFFER_SIZE - 1) {
              buffer[cursor++] = c;
              buffer[cursor]   = '\0';
            }
          }
          break;
        case ABORT:
          _pending = false; _delay = 0;
          break;
      }
    }
  }

  if (_pending && timeOutReached(_process_time + _delay)) {
    _pending = false; _delay = 0;
    SSDP_send(NONE);
  } else if ((_notify_time == 0) || timeOutReached(_notify_time + (SSDP_INTERVAL * 1000L))) {
    _notify_time = millis();
    SSDP_send(NOTIFY);
  }

  if (_pending) {
    while (_server->next()) {
      _server->flush();
    }
  }
}

# endif // if FEATURE_SSDP
#endif  // if defined(ESP8266)


// ********************************************************************************
// Return subnet range of WiFi.
// ********************************************************************************
bool getSubnetRange(IPAddress& low, IPAddress& high)
{
  if (!WiFiEventData.WiFiGotIP()) {
    return false;
  }

  const IPAddress ip     = NetworkLocalIP();
  const IPAddress subnet = NetworkSubnetMask();

  low  = ip;
  high = ip;

  // Compute subnet range.
  for (uint8_t i = 0; i < 4; ++i) {
    if (subnet[i] != 255) {
      low[i]  = low[i] & subnet[i];
      high[i] = high[i] | ~subnet[i];
    }
  }
  return true;
}

// ********************************************************************************
// Functions to test and handle network/client connectivity.
// ********************************************************************************

#ifdef CORE_POST_2_5_0
# include <AddrList.h>
#endif // ifdef CORE_POST_2_5_0


bool hasIPaddr() {
  if (useStaticIP()) { return true; }

#ifdef CORE_POST_2_5_0
  bool configured = false;

  for (auto addr : addrList) {
    if ((configured = (!addr.isLocal() && (addr.ifnumber() == STATION_IF)))) {
      /*
         Serial.printf("STA: IF='%s' hostname='%s' addr= %s\n",
                    addr.ifname().c_str(),
                    addr.ifhostname(),
                    addr.toString().c_str());
       */
      break;
    }
  }
  return configured;
#else // ifdef CORE_POST_2_5_0
  return WiFi.isConnected();
#endif // ifdef CORE_POST_2_5_0
}

// Check connection. Maximum timeout 500 msec.
bool NetworkConnected(uint32_t timeout_ms) {
  uint32_t timer     = millis() + (timeout_ms > 500 ? 500 : timeout_ms);
  uint32_t min_delay = timeout_ms / 20;

  if (min_delay < 10) {
    delay(0); // Allow at least once time for backgroundtasks
    min_delay = 10;
  }

  // Apparently something needs network, perform check to see if it is ready now.
  while (!NetworkConnected()) {
    if (timeOutReached(timer)) {
      return false;
    }
    delay(min_delay); // Allow the backgroundtasks to continue procesing.
  }
  return true;
}

bool hostReachable(const IPAddress& ip) {
  if (!NetworkConnected()) { return false; }

  return true; // Disabled ping as requested here:
  // https://github.com/letscontrolit/ESPEasy/issues/1494#issuecomment-397872538

  /*
     // Only do 1 ping at a time to return early
     uint8_t retry = 3;
     while (retry > 0) {
   #if defined(ESP8266)
      if (Ping.ping(ip, 1)) return true;
   #endif
   #if defined(ESP32)
     if (ping_start(ip, 4, 0, 0, 5)) return true;
   #endif
      delay(50);
      --retry;
     }
     if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log = F("Host unreachable: ");
      log += formatIP(ip);
      addLog(LOG_LEVEL_ERROR, log);
     }
     if (ip[1] == 0 && ip[2] == 0 && ip[3] == 0) {
      // Work-around to fix connected but not able to communicate.
      addLog(LOG_LEVEL_ERROR, F("WiFi : Detected strange behavior, reconnect wifi."));
      WifiDisconnect();
     }
     logConnectionStatus();
     return false;
   */
}

bool connectClient(WiFiClient& client, const char *hostname, uint16_t port, uint32_t timeout_ms) {
  IPAddress ip;

  if (resolveHostByName(hostname, ip, timeout_ms)) {
    return connectClient(client, ip, port, timeout_ms);
  }
  return false;
}

bool connectClient(WiFiClient& client, IPAddress ip, uint16_t port, uint32_t timeout_ms)
{
  START_TIMER;

  if (!NetworkConnected()) {
    return false;
  }

  // In case of domain name resolution error result can be negative.
  // https://github.com/esp8266/Arduino/blob/18f643c7e2d6a0da9d26ff2b14c94e6536ab78c1/libraries/Ethernet/src/Dns.cpp#L44
  // Thus must match the result with 1.
  bool connected = (client.connect(ip, port) == 1);

  delay(0);

  if (!connected) {
    Scheduler.sendGratuitousARP_now();
  }
  STOP_TIMER(CONNECT_CLIENT_STATS);
#if defined(ESP32) || defined(ARDUINO_ESP8266_RELEASE_2_3_0) || defined(ARDUINO_ESP8266_RELEASE_2_4_0)
#else

  if (connected) {
    client.keepAlive(); // Use default keep alive values
  }
#endif // if defined(ESP32) || defined(ARDUINO_ESP8266_RELEASE_2_3_0) || defined(ARDUINO_ESP8266_RELEASE_2_4_0)
  return connected;
}

bool resolveHostByName(const char *aHostname, IPAddress& aResult, uint32_t timeout_ms) {
  START_TIMER;

  if (!NetworkConnected()) {
    return false;
  }

  FeedSW_watchdog();

#if defined(ARDUINO_ESP8266_RELEASE_2_3_0) || defined(ESP32)
  bool resolvedIP = WiFi.hostByName(aHostname, aResult) == 1;
#else // if defined(ARDUINO_ESP8266_RELEASE_2_3_0) || defined(ESP32)
  bool resolvedIP = WiFi.hostByName(aHostname, aResult, timeout_ms) == 1;
#endif // if defined(ARDUINO_ESP8266_RELEASE_2_3_0) || defined(ESP32)
  delay(0);
  FeedSW_watchdog();

  if (!resolvedIP) {
    Scheduler.sendGratuitousARP_now();
  }
  STOP_TIMER(HOST_BY_NAME_STATS);
  return resolvedIP;
}

bool hostReachable(const String& hostname) {
  IPAddress remote_addr;

  if (resolveHostByName(hostname.c_str(), remote_addr)) {
    return hostReachable(remote_addr);
  }

  if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
    String log = F("Hostname cannot be resolved: ");

    log += hostname;
    addLogMove(LOG_LEVEL_ERROR, log);
  }
  return false;
}

// Create a random port for the UDP connection.
// Return true when successful.
bool beginWiFiUDP_randomPort(WiFiUDP& udp) {
  if (!NetworkConnected()) {
    return false;
  }
  unsigned int attempts = 3;

  while (attempts > 0) {
    --attempts;
    long port = random(1025, 65535);

    if (udp.begin(port) != 0) {
      return true;
    }
  }
  return false;
}

void sendGratuitousARP() {
  if (!NetworkConnected()) {
    return;
  }
#ifdef SUPPORT_ARP

  // See https://github.com/letscontrolit/ESPEasy/issues/2374
  START_TIMER;
  netif *n = netif_list;

  while (n) {
    if ((n->hwaddr_len == ETH_HWADDR_LEN) &&
        (n->flags & NETIF_FLAG_ETHARP) &&
        ((n->flags & NETIF_FLAG_LINK_UP) && (n->flags & NETIF_FLAG_UP))) {
      # ifdef ESP32
      etharp_gratuitous_r(n);
      # else // ifdef ESP32
      etharp_gratuitous(n);
      # endif // ifdef ESP32
    }
    n = n->next;
  }
  STOP_TIMER(GRAT_ARP_STATS);
#endif // ifdef SUPPORT_ARP
}

bool splitHostPortString(const String& hostPortString, String& host, uint16_t& port) {
  port = 80; // Some default
  int index_colon = hostPortString.indexOf(':');

  if (index_colon >= 0) {
    int port_tmp;

    if (!validIntFromString(hostPortString.substring(index_colon + 1), port_tmp)) {
      return false;
    }

    if ((port_tmp < 0) || (port_tmp > 65535)) { return false; }
    port = port_tmp;
    host = hostPortString.substring(0, index_colon);
  } else {
    // No port nr defined.
    host = hostPortString;
  }
  return true;
}

bool splitUserPass_HostPortString(const String& hostPortString, String& user, String& pass, String& host, uint16_t& port)
{
  const int pos_at = hostPortString.indexOf('@');

  if (pos_at != -1) {
    user = hostPortString.substring(0, pos_at);
    const int pos_colon = user.indexOf(':');

    if (pos_colon != -1) {
      pass = user.substring(pos_colon + 1);
      user = user.substring(0, pos_colon);
    }
    return splitHostPortString(hostPortString.substring(pos_at + 1), host, port);
  }
  return splitHostPortString(hostPortString, host, port);
}

// Split a full URL like "http://hostname:port/path/file.htm"
// Return value is everything after the hostname:port section (including /)
String splitURL(const String& fullURL, String& user, String& pass, String& host, uint16_t& port, String& file) {
  int starthost = fullURL.indexOf(F("//"));

  if (starthost == -1) {
    starthost = 0;
  } else {
    starthost += 2;
  }
  int endhost = fullURL.indexOf('/', starthost);

  splitUserPass_HostPortString(fullURL.substring(starthost, endhost), user, pass, host, port);
  int startfile = fullURL.lastIndexOf('/');

  if (startfile >= 0) {
    file = fullURL.substring(startfile);
  }
  return fullURL.substring(endhost);
}

String get_user_agent_string() {
  static unsigned int agent_size = 20;
  String userAgent;

  userAgent.reserve(agent_size);
  userAgent += F("ESP Easy/");
  userAgent += BUILD;
  userAgent += '/';
  userAgent += get_build_date();
  userAgent += ' ';
  userAgent += get_build_time();
  agent_size = userAgent.length();
  return userAgent;
}

bool splitHeaders(int& strpos, const String& multiHeaders, String& name, String& value) {
  if (strpos < 0) {
    return false;
  }
  int colonPos = multiHeaders.indexOf(':', strpos);

  if (colonPos < 0) {
    return false;
  }
  name = multiHeaders.substring(strpos, colonPos);
  int valueEndPos = multiHeaders.indexOf('\n', colonPos + 1);

  if (valueEndPos < 0) {
    value  = multiHeaders.substring(colonPos + 1);
    strpos = -1;
  } else {
    value  = multiHeaders.substring(colonPos + 1, valueEndPos);
    strpos = valueEndPos + 1;
  }
  value.replace('\r', ' ');
  value.trim();
  return true;
}

String extractParam(const String& authReq, const String& param, const char delimit) {
  int _begin = authReq.indexOf(param);

  if (_begin == -1) { return EMPTY_STRING; }
  return authReq.substring(_begin + param.length(), authReq.indexOf(delimit, _begin + param.length()));
}

String getCNonce(const int len) {
  static const char alphanum[] = "0123456789"
                                 "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                 "abcdefghijklmnopqrstuvwxyz";
  String s;

  for (int i = 0; i < len; ++i) {
    s += alphanum[rand() % (sizeof(alphanum) - 1)];
  }

  return s;
}

String getDigestAuth(const String& authReq,
                     const String& username,
                     const String& password,
                     const String& method,
                     const String& uri,
                     unsigned int  counter) {
  // extracting required parameters for RFC 2069 simpler Digest
  const String realm  = extractParam(authReq, F("realm=\""), '"');
  const String nonce  = extractParam(authReq, F("nonce=\""), '"');
  const String cNonce = getCNonce(8);

  char nc[9];

  snprintf(nc, sizeof(nc), "%08x", counter);

  // parameters for the RFC 2617 newer Digest
  MD5Builder md5;

  md5.begin();
  md5.add(username + ':' + realm + ':' + password); // md5 of the user:realm:user
  md5.calculate();
  const String h1 = md5.toString();

  md5.begin();
  md5.add(method + ':' + uri);
  md5.calculate();
  const String h2 = md5.toString();

  md5.begin();
  md5.add(h1 + ':' + nonce + ':' + String(nc) + ':' + cNonce + F(":auth:") + h2);
  md5.calculate();
  const String response = md5.toString();

  const String authorization =
    String(F("Digest username=\"")) + username +
    F("\", realm=\"") + realm +
    F("\", nonce=\"") + nonce +
    F("\", uri=\"") + uri +
    F("\", algorithm=\"MD5\", qop=auth, nc=") + String(nc) +
    F(", cnonce=\"") + cNonce +
    F("\", response=\"") + response +
    '"';

  //  Serial.println(authorization);

  return authorization;
}

void log_http_result(const HTTPClient& http,
                     const String    & logIdentifier,
                     const String    & host,
                     const String    & HttpMethod,
                     int               httpCode,
                     const String    & response)
{
  uint8_t loglevel = LOG_LEVEL_ERROR;
  bool    success  = false;

  // HTTP codes:
  // 1xx Informational response
  // 2xx Success
  if ((httpCode >= 100) && (httpCode < 300)) {
    loglevel = LOG_LEVEL_INFO;
    success  = true;
  }

  if (loglevelActiveFor(loglevel)) {
    String log = F("HTTP : ");
    log += logIdentifier;
    log += ' ';
    log += host;
    log += ' ';
    log += HttpMethod;
    log += F("... ");

    if (!success) {
      log += F("failed ");
    }
    log += F("HTTP code: ");
    log += String(httpCode);

    if (!success) {
      log += ' ';
      log += http.errorToString(httpCode);
    }

    if (response.length() > 0) {
      log += F(" Received reply: ");
      log += response.substring(0, 100); // Returned string may be huge, so only log the first part.
    }
    addLogMove(loglevel, log);
  }
}

int http_authenticate(const String& logIdentifier,
                      WiFiClient  & client,
                      HTTPClient  & http,
                      uint16_t      timeout,
                      const String& user,
                      const String& pass,
                      const String& host,
                      uint16_t      port,
                      const String& uri,
                      const String& HttpMethod,
                      const String& header,
                      const String& postStr)
{
  int httpCode = 0;

  http.setAuthorization(user.c_str(), pass.c_str());
  http.setTimeout(timeout);
  http.setUserAgent(get_user_agent_string());

  if (Settings.SendToHTTP_follow_redirects()) {
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setRedirectLimit(2);
  }

  #ifdef MUSTFIX_CLIENT_TIMEOUT_IN_SECONDS

  // See: https://github.com/espressif/arduino-esp32/pull/6676
  client.setTimeout((timeout + 500) / 1000); // in seconds!!!!
  #else // ifdef MUSTFIX_CLIENT_TIMEOUT_IN_SECONDS
  client.setTimeout(timeout);                // in msec as it should be!
  #endif // ifdef MUSTFIX_CLIENT_TIMEOUT_IN_SECONDS

  // Add request header as fall back.
  // When adding another "accept" header, it may be interpreted as:
  // "if you have XXX, send it; or failing that, just give me what you've got."
  http.addHeader(F("Accept"), F("*/*;q=0.1"));

  delay(0);
#if defined(CORE_POST_2_6_0) || defined(ESP32)
  http.begin(client, host, port, uri, false); // HTTP
#else // if defined(CORE_POST_2_6_0) || defined(ESP32)
  http.begin(client, host, port, uri);
#endif // if defined(CORE_POST_2_6_0) || defined(ESP32)

  const char *keys[] = { "WWW-Authenticate" };
  http.collectHeaders(keys, 1);

  {
    int headerpos = 0;
    String name, value;

    while (splitHeaders(headerpos, header, name, value)) {
      http.addHeader(name, value);
    }
  }

  // start connection and send HTTP header (and body)
  if (HttpMethod.equals(F("HEAD")) || HttpMethod.equals(F("GET"))) {
    httpCode = http.sendRequest(HttpMethod.c_str());
  } else {
    httpCode = http.sendRequest(HttpMethod.c_str(), postStr);
  }

  // Check to see if we need to try digest auth
  if (httpCode == 401) {
    const String authReq = http.header(String(F("WWW-Authenticate")).c_str());

    if (authReq.indexOf(F("Digest")) != -1) {
      // Use Digest authorization
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        addLogMove(LOG_LEVEL_INFO, String(F("HTTP : Start Digest Authorization for ")) + host);
      }

      http.setAuthorization(""); // Clear Basic authorization
      const String authorization = getDigestAuth(authReq, user, pass, "GET", uri, 1);

      http.end();
#if defined(CORE_POST_2_6_0) || defined(ESP32)
      http.begin(client, host, port, uri, false); // HTTP, not HTTPS
#else // if defined(CORE_POST_2_6_0) || defined(ESP32)
      http.begin(client, host, port, uri);
#endif // if defined(CORE_POST_2_6_0) || defined(ESP32)

      http.addHeader(F("Authorization"), authorization);

      // start connection and send HTTP header (and body)
      if (HttpMethod.equals(F("HEAD")) || HttpMethod.equals(F("GET"))) {
        httpCode = http.sendRequest(HttpMethod.c_str());
      } else {
        httpCode = http.sendRequest(HttpMethod.c_str(), postStr);
      }
    }
  }

  if (Settings.UseRules) {
    // Generate event with the HTTP return code
    // e.g. http#hostname=401
    String event = F("http#");
    event += host;
    event += '=';
    event += httpCode;
    eventQueue.addMove(std::move(event));
  }
  log_http_result(http, logIdentifier, host, HttpMethod, httpCode, EMPTY_STRING);
  return httpCode;
}

String send_via_http(const String& logIdentifier,
                     WiFiClient  & client,
                     uint16_t      timeout,
                     const String& user,
                     const String& pass,
                     const String& host,
                     uint16_t      port,
                     const String& uri,
                     const String& HttpMethod,
                     const String& header,
                     const String& postStr,
                     int         & httpCode,
                     bool          must_check_reply) {
  HTTPClient http;

  httpCode = http_authenticate(
    logIdentifier,
    client,
    http,
    timeout,
    user,
    pass,
    host,
    port,
    uri,
    HttpMethod,
    header,
    postStr);

  String response;

  if ((httpCode > 0) && must_check_reply) {
    response = http.getString();

    if (!response.isEmpty()) {
      log_http_result(http, logIdentifier, host, HttpMethod, httpCode, response);
    }
  }
  http.end();
  return response;
}

#if FEATURE_DOWNLOAD

// FIXME TD-er: Must set the timeout somewhere
# ifndef DOWNLOAD_FILE_TIMEOUT
  #  define DOWNLOAD_FILE_TIMEOUT 2000
# endif // ifndef DOWNLOAD_FILE_TIMEOUT

// Download a file from a given URL and save to a local file named "file_save"
// If the URL ends with a /, the file part will be assumed the same as file_save.
// If file_save is empty, the file part from the URL will be used as local file name.
// Return true when successful.
bool downloadFile(const String& url, String file_save) {
  String error;

  return downloadFile(url, file_save, EMPTY_STRING, EMPTY_STRING, error);
}

// User and Pass may be updated if they occur in the hostname part.
// Thus have to be copied instead of const reference.
bool start_downloadFile(WiFiClient  & client,
                        HTTPClient  & http,
                        const String& url,
                        String      & file_save,
                        String        user,
                        String        pass,
                        String      & error) {
  String   host, file;
  uint16_t port;
  String   uri = splitURL(url, user, pass, host, port, file);

  if (file_save.isEmpty()) {
    file_save = file;
  } else if ((file.isEmpty()) && uri.endsWith("/")) {
    // file = file_save;
    uri += file_save;
  }
# ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("downloadFile: URL: ");
    log += url;
    log += F(" decoded: ");
    log += host;
    log += ':';
    log += port;
    log += uri;
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
# endif // ifndef BUILD_NO_DEBUG

  if (file_save.isEmpty()) {
    error = F("Empty filename");
    addLog(LOG_LEVEL_ERROR, error);
    return false;
  }

  const int httpCode = http_authenticate(
    F("DownloadFile"),
    client,
    http,
    DOWNLOAD_FILE_TIMEOUT,
    user,
    pass,
    host,
    port,
    uri,
    F("GET"),
    EMPTY_STRING, // header
    EMPTY_STRING  // postStr
    );

  if (httpCode != HTTP_CODE_OK) {
    error  = F("HTTP code: ");
    error += httpCode;
    error += ' ';
    error += url;

    addLog(LOG_LEVEL_ERROR, error);
    http.end();
    return false;
  }
  return true;
}

bool downloadFile(const String& url, String file_save, const String& user, const String& pass, String& error) {
  WiFiClient client;
  HTTPClient http;

  if (!start_downloadFile(client, http, url, file_save, user, pass, error)) {
    return false;
  }

  if (fileExists(file_save)) {
    error  = F("File exists: ");
    error += file_save;
    addLog(LOG_LEVEL_ERROR, error);
    return false;
  }

  long len   = http.getSize();
  fs::File f = tryOpenFile(file_save, "w");

  if (f) {
    const size_t downloadBuffSize = 256;
    uint8_t buff[downloadBuffSize];
    size_t  bytesWritten  = 0;
    unsigned long timeout = millis() + DOWNLOAD_FILE_TIMEOUT;

    // get tcp stream
    WiFiClient *stream = &client;

    // read all data from server
    while (http.connected() && (len > 0 || len == -1)) {
      // read up to downloadBuffSize at a time.
      size_t bytes_to_read = downloadBuffSize;

      if ((len > 0) && (len < static_cast<int>(bytes_to_read))) {
        bytes_to_read = len;
      }
      const size_t c = stream->readBytes(buff, bytes_to_read);

      if (c > 0) {
        timeout = millis() + DOWNLOAD_FILE_TIMEOUT;

        if (f.write(buff, c) != c) {
          error  = F("Error saving file: ");
          error += file_save;
          error += ' ';
          error += bytesWritten;
          error += F(" Bytes written");
          addLog(LOG_LEVEL_ERROR, error);
          http.end();
          return false;
        }
        bytesWritten += c;

        if (len > 0) { len -= c; }
      }

      if (timeOutReached(timeout)) {
        error  = F("Timeout: ");
        error += file_save;
        addLog(LOG_LEVEL_ERROR, error);
        delay(0);
        http.end();
        return false;
      }
      delay(0);
    }
    f.close();
    http.end();

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("downloadFile: ");
      log += file_save;
      log += F(" Success");
      addLogMove(LOG_LEVEL_INFO, log);
    }
    return true;
  }
  http.end();
  error  = F("Failed to open file for writing: ");
  error += file_save;
  addLog(LOG_LEVEL_ERROR, error);
  return false;
}

bool downloadFirmware(const String& url, String& error)
{
  String file_save;
  String user;
  String pass;
  WiFiClient client;
  HTTPClient http;

  if (!start_downloadFile(client, http, url, file_save, user, pass, error)) {
    return false;
  }

  int len = http.getSize();

  if (Update.begin(len, U_FLASH, Settings.Pin_status_led, Settings.Pin_status_led_Inversed ? LOW : HIGH)) {
    const size_t downloadBuffSize = 256;
    uint8_t buff[downloadBuffSize];
    size_t  bytesWritten  = 0;
    unsigned long timeout = millis() + DOWNLOAD_FILE_TIMEOUT;

    // get tcp stream
    WiFiClient *stream = &client;

    while (http.connected() && (len > 0 || len == -1)) {
      // read up to downloadBuffSize at a time.
      size_t bytes_to_read = downloadBuffSize;

      if ((len > 0) && (len < static_cast<int>(bytes_to_read))) {
        bytes_to_read = len;
      }
      const size_t c = stream->readBytes(buff, bytes_to_read);

      if (c > 0) {
        timeout = millis() + DOWNLOAD_FILE_TIMEOUT;

        if (Update.write(buff, c) != c) {
          error  = F("Error saving firmware update: ");
          error += file_save;
          error += ' ';
          error += bytesWritten;
          error += F(" Bytes written");
          addLog(LOG_LEVEL_ERROR, error);
          Update.end();
          http.end();
          return false;
        }
        bytesWritten += c;

        if (len > 0) { len -= c; }
      }

      if (timeOutReached(timeout)) {
        error  = F("Timeout: ");
        error += file_save;
        addLog(LOG_LEVEL_ERROR, error);
        delay(0);
        Update.end();
        http.end();
        return false;
      }

      if (!UseRTOSMultitasking) {
        // On ESP32 the schedule is executed on the 2nd core.
        Scheduler.handle_schedule();
      }
      backgroundtasks();
    }
    http.end();

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("downloadFile: ");
      log += file_save;
      log += F(" Success");
      addLogMove(LOG_LEVEL_INFO, log);
    }

    if (Update.end()) {
      if (Settings.UseRules) {
        String event = F("ProvisionFirmware#success=");
        event += file_save;
        eventQueue.addMove(std::move(event));
      }
    }
    return true;
  }
  http.end();
  Update.end();
  error  = F("Failed update firmware: ");
  error += file_save;
  addLog(LOG_LEVEL_ERROR, error);

  if (Settings.UseRules) {
    String event = F("ProvisionFirmware#failed=");
    event += file_save;
    eventQueue.addMove(std::move(event));
  }
  return false;
}

#endif // if FEATURE_DOWNLOAD

#include "../Helpers/ESPEasy_Storage.h"

#include "../../ESPEasy_common.h"

#include "../CustomBuild/StorageLayout.h"

#include "../DataStructs/TimingStats.h"

#include "../DataTypes/ESPEasyFileType.h"
#include "../DataTypes/SPI_options.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../ESPEasyCore/ESPEasyWifi.h"
#include "../ESPEasyCore/Serial.h"

#include "../Globals/CRCValues.h"
#include "../Globals/Cache.h"
#include "../Globals/Device.h"
#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/ESPEasy_Scheduler.h"
#include "../Globals/EventQueue.h"
#include "../Globals/ExtraTaskSettings.h"
#include "../Globals/Plugins.h"
#include "../Globals/RTC.h"
#include "../Globals/ResetFactoryDefaultPref.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/Settings.h"

#include "../Helpers/ESPEasyRTC.h"
#include "../Helpers/ESPEasy_checks.h"
#include "../Helpers/ESPEasy_FactoryDefault.h"
#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/FS_Helper.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/Memory.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Networking.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/PeriodicalActions.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringParser.h"


#ifdef ESP32
#include <MD5Builder.h>
#include <esp_partition.h>
#endif

#ifdef ESP32
String patch_fname(const String& fname) {
  if (fname.startsWith(F("/"))) {
    return fname;
  }
  return String('/') + fname;
}
#endif
#ifdef ESP8266
#define patch_fname(F) (F)
#endif

/********************************************************************************************\
   file system error handling
   Look here for error # reference: https://github.com/pellepl/spiffs/blob/master/src/spiffs.h
 \*********************************************************************************************/
String FileError(int line, const char *fname)
{
  String err = F("FS   : Error while reading/writing ");

  err += fname;
  err += F(" in ");
  err += line;
  addLogMove(LOG_LEVEL_ERROR, err);
  return err;
}

/********************************************************************************************\
   Keep track of number of flash writes.
 \*********************************************************************************************/
void flashCount()
{
  if (RTC.flashDayCounter <= MAX_FLASHWRITES_PER_DAY) {
    RTC.flashDayCounter++;
  }
  RTC.flashCounter++;
  saveToRTC();
}

String flashGuard()
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("flashGuard"));
  #endif

  if (RTC.flashDayCounter > MAX_FLASHWRITES_PER_DAY)
  {
    String log = F("FS   : Daily flash write rate exceeded! (powercycle or send command 'resetFlashWriteCounter' to reset this)");
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }
  flashCount();
  return String();
}

// use this in function that can return an error string. it automaticly returns with an error string if there where too many flash writes.
#define FLASH_GUARD() { String flashErr = flashGuard(); \
                        if (flashErr.length()) return flashErr; }


String appendLineToFile(const String& fname, const String& line) {
  return appendToFile(fname, reinterpret_cast<const uint8_t *>(line.c_str()), line.length());
}

String appendToFile(const String& fname, const uint8_t *data, unsigned int size) {
  fs::File f = tryOpenFile(fname, "a+");

  SPIFFS_CHECK(f,                   fname.c_str());
  SPIFFS_CHECK(f.write(data, size), fname.c_str());
  f.close();
  return "";
}

bool fileExists(const String& fname) {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  const String patched_fname = patch_fname(fname);
  auto search = Cache.fileExistsMap.find(patched_fname);
  if (search != Cache.fileExistsMap.end()) {
    return search->second;
  }
  bool res = ESPEASY_FS.exists(patched_fname);
  Cache.fileExistsMap[patched_fname] = res;
  return res;
}

fs::File tryOpenFile(const String& fname, const String& mode) {
  START_TIMER;
  fs::File f;
  if (fname.isEmpty() || fname.equals(F("/"))) {
    return f;
  }

  bool exists = fileExists(fname);

  if (!exists) {
    if (mode == F("r")) {
      return f;
    }
    Cache.fileExistsMap.clear();
  }
  f = ESPEASY_FS.open(patch_fname(fname), mode.c_str());
  STOP_TIMER(TRY_OPEN_FILE);
  return f;
}

bool tryRenameFile(const String& fname_old, const String& fname_new) {
  Cache.fileExistsMap.clear();
  if (fileExists(fname_old) && !fileExists(fname_new)) {
    clearAllCaches();
    return ESPEASY_FS.rename(patch_fname(fname_old), patch_fname(fname_new));
  }
  return false;
}

bool tryDeleteFile(const String& fname) {
  if (fname.length() > 0)
  {
    clearAllCaches();
    bool res = ESPEASY_FS.remove(patch_fname(fname));

    // A call to GarbageCollection() will at most erase a single block. (e.g. 8k block size)
    // A deleted file may have covered more than a single block, so try to clear multiple blocks.
    uint8_t retries = 3;

    while (retries > 0 && GarbageCollection()) {
      --retries;
    }
    return res;
  }
  return false;
}

/********************************************************************************************\
   Fix stuff to clear out differences between releases
 \*********************************************************************************************/
String BuildFixes()
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("BuildFixes"));
  #endif
  serialPrintln(F("\nBuild changed!"));

  if (Settings.Build < 145)
  {
    InitFile(SettingsType::SettingsFileEnum::FILE_NOTIFICATION_type);
  }

  if (Settings.Build < 20101)
  {
    serialPrintln(F("Fix reset Pin"));
    Settings.Pin_Reset = -1;
  }

  if (Settings.Build < 20102) {
    // Settings were 'mangled' by using older version
    // Have to patch settings to make sure no bogus data is being used.
    serialPrintln(F("Fix settings with uninitalized data or corrupted by switching between versions"));
    Settings.UseRTOSMultitasking       = false;
    Settings.Pin_Reset                 = -1;
    Settings.SyslogFacility            = DEFAULT_SYSLOG_FACILITY;
    Settings.MQTTUseUnitNameAsClientId_unused = DEFAULT_MQTT_USE_UNITNAME_AS_CLIENTID;
    Settings.StructSize                = sizeof(Settings);
  }

  if (Settings.Build < 20103) {
    Settings.ResetFactoryDefaultPreference = 0;
    Settings.OldRulesEngine(DEFAULT_RULES_OLDENGINE);
  }
  if (Settings.Build < 20105) {
    Settings.I2C_clockSpeed = DEFAULT_I2C_CLOCK_SPEED;
  }
  if (Settings.Build <= 20106) {
    // ClientID is now defined in the controller settings.
    #if FEATURE_MQTT
    controllerIndex_t controller_idx = firstEnabledMQTT_ControllerIndex();
    if (validControllerIndex(controller_idx)) {
      MakeControllerSettings(ControllerSettings); //-V522
      if (AllocatedControllerSettings()) {
        LoadControllerSettings(controller_idx, ControllerSettings);

        String clientid;
        if (Settings.MQTTUseUnitNameAsClientId_unused) {
          clientid = F("%sysname%");
          if (Settings.appendUnitToHostname()) {
            clientid += F("_%unit%");
          }
        }
        else {
          clientid  = F("ESPClient_%mac%");
        }
        safe_strncpy(ControllerSettings.ClientID, clientid, sizeof(ControllerSettings.ClientID));

        ControllerSettings.mqtt_uniqueMQTTclientIdReconnect(Settings.uniqueMQTTclientIdReconnect_unused());
        ControllerSettings.mqtt_retainFlag(Settings.MQTTRetainFlag_unused);
        SaveControllerSettings(controller_idx, ControllerSettings);
      }
    }
    #endif // if FEATURE_MQTT
  }
  if (Settings.Build < 20107) {
    Settings.WebserverPort = 80;
  }
  if (Settings.Build < 20108) {
    Settings.ETH_Phy_Addr   = DEFAULT_ETH_PHY_ADDR;
    Settings.ETH_Pin_mdc    = DEFAULT_ETH_PIN_MDC;
    Settings.ETH_Pin_mdio   = DEFAULT_ETH_PIN_MDIO;
    Settings.ETH_Pin_power  = DEFAULT_ETH_PIN_POWER;
    Settings.ETH_Phy_Type   = DEFAULT_ETH_PHY_TYPE;
    Settings.ETH_Clock_Mode = DEFAULT_ETH_CLOCK_MODE;
    Settings.NetworkMedium  = DEFAULT_NETWORK_MEDIUM;
  }
  if (Settings.Build < 20109) {
    Settings.SyslogPort = 514;
  }
  if (Settings.Build < 20110) {
    Settings.I2C_clockSpeed_Slow = DEFAULT_I2C_CLOCK_SPEED_SLOW;
    Settings.I2C_Multiplexer_Type = I2C_MULTIPLEXER_NONE;
    Settings.I2C_Multiplexer_Addr = -1;
    for (taskIndex_t x = 0; x < TASKS_MAX; x++) {
      Settings.I2C_Multiplexer_Channel[x] = -1;
    }
    Settings.I2C_Multiplexer_ResetPin = -1;
  }
  if (Settings.Build < 20111) {
    #ifdef ESP32
    constexpr uint8_t maxStatesesp32 = sizeof(Settings.PinBootStates_ESP32) / sizeof(Settings.PinBootStates_ESP32[0]);
    for (uint8_t i = 0; i < maxStatesesp32; ++i) {
      Settings.PinBootStates_ESP32[i] = 0;
    }
    #endif
  }
  if (Settings.Build < 20112) {
    Settings.WiFi_TX_power = 70; // 70 = 17.5dBm. unit: 0.25 dBm
    Settings.WiFi_sensitivity_margin = 3; // Margin in dBm on top of sensitivity.
  }
  if (Settings.Build < 20113) {
    Settings.NumberExtraWiFiScans = 0;
  }
  if (Settings.Build < 20114) {
    #ifdef USES_P003
    // P003_Pulse was always using the pull-up, now it is a setting.
    for (taskIndex_t taskIndex = 0; taskIndex < TASKS_MAX; ++taskIndex) {
      if (Settings.TaskDeviceNumber[taskIndex] == 3) {
        Settings.TaskDevicePin1PullUp[taskIndex] = true;
      }
    }
    #endif
  }
  if (Settings.Build < 20115) {
    if (Settings.InitSPI != static_cast<int>(SPI_Options_e::UserDefined)) { // User-defined SPI pins set to None
      Settings.SPI_SCLK_pin = -1;
      Settings.SPI_MISO_pin = -1;
      Settings.SPI_MOSI_pin = -1;
    }
  }
  #ifdef USES_P053
  if (Settings.Build < 20116) {
    // Added PWR button, init to "-none-"
    for (taskIndex_t taskIndex = 0; taskIndex < TASKS_MAX; ++taskIndex) {
      if (Settings.TaskDeviceNumber[taskIndex] == 53) {
        Settings.TaskDevicePluginConfig[taskIndex][3] = -1;
      }
    }
    // Remove PeriodicalScanWiFi
    // Reset to default 0 for future use.
    bitWrite(Settings.VariousBits1, 15, 0);
  }
  #endif


  Settings.Build = BUILD;
  return SaveSettings();
}

/********************************************************************************************\
   Mount FS and check config.dat
 \*********************************************************************************************/
void fileSystemCheck()
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("fileSystemCheck"));
  #endif
  addLog(LOG_LEVEL_INFO, F("FS   : Mounting..."));
#if defined(ESP32) && defined(USE_LITTLEFS)
  if (getPartionCount(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS) != 0 
      && ESPEASY_FS.begin())
#else
  if (ESPEASY_FS.begin())
#endif
  {
    clearAllCaches();
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("FS   : Mount successful, used ");
      log += SpiffsUsedBytes();
      log += F(" bytes of ");
      log += SpiffsTotalBytes();
      addLogMove(LOG_LEVEL_INFO, log);
    }

    // Run garbage collection before any file is open.
    uint8_t retries = 3;

    while (retries > 0 && GarbageCollection()) {
      --retries;
    }

    fs::File f = tryOpenFile(SettingsType::getSettingsFileName(SettingsType::Enum::BasicSettings_Type).c_str(), "r");
    if (f) { 
      f.close(); 
    } else {
      ResetFactory();
    }
  }
  else
  {
    const __FlashStringHelper * log = F("FS   : Mount failed");
    serialPrintln(log);
    addLogMove(LOG_LEVEL_ERROR, log);
    ResetFactory();
  }
}

bool FS_format() {
  #ifdef USE_LITTLEFS
    #ifdef ESP32
    const bool res = ESPEASY_FS.begin(true);
    ESPEASY_FS.end();
    return res;
    #else
    return ESPEASY_FS.format();
    #endif
  #else
  return ESPEASY_FS.format();
  #endif
}

#ifdef ESP32

# include <esp_partition.h>

int getPartionCount(uint8_t pType, uint8_t pSubType) {
  esp_partition_type_t partitionType       = static_cast<esp_partition_type_t>(pType);
  esp_partition_subtype_t subtype          = static_cast<esp_partition_subtype_t>(pSubType);
  esp_partition_iterator_t _mypartiterator = esp_partition_find(partitionType, subtype, NULL);
  int nrPartitions                         = 0;

  if (_mypartiterator) {
    do {
      ++nrPartitions;
    } while ((_mypartiterator = esp_partition_next(_mypartiterator)) != NULL);
  }
  esp_partition_iterator_release(_mypartiterator);
  return nrPartitions;
}


#endif

/********************************************************************************************\
   Garbage collection
 \*********************************************************************************************/
bool GarbageCollection() {
  #ifdef CORE_POST_2_6_0

  // Perform garbage collection
  START_TIMER;

  if (ESPEASY_FS.gc()) {
    addLog(LOG_LEVEL_INFO, F("FS   : Success garbage collection"));
    STOP_TIMER(FS_GC_SUCCESS);
    return true;
  }
  STOP_TIMER(FS_GC_FAIL);
  return false;
  #else // ifdef CORE_POST_2_6_0

  // Not supported, so nothing was removed.
  return false;
  #endif // ifdef CORE_POST_2_6_0
}

/********************************************************************************************\
   Save settings to file system
 \*********************************************************************************************/
String SaveSettings()
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("SaveSettings"));
  #endif
  String     err;
  {
    Settings.StructSize = sizeof(Settings);

    // FIXME @TD-er: As discussed in #1292, the CRC for the settings is now disabled.

    /*
      MD5Builder md5;
      uint8_t    tmp_md5[16] = { 0 };
      memcpy( Settings.ProgmemMd5, CRCValues.runTimeMD5, 16);
      md5.begin();
      md5.add(reinterpret_cast<const uint8_t *>(&Settings), sizeof(Settings)-16);
      md5.calculate();
      md5.getBytes(tmp_md5);
      if (memcmp(tmp_md5, Settings.md5, 16) != 0) {
        // Settings have changed, save to file.
        memcpy(Settings.md5, tmp_md5, 16);
    */
    Settings.validate();
    err = SaveToFile(SettingsType::getSettingsFileName(SettingsType::Enum::BasicSettings_Type).c_str(), 0, reinterpret_cast<const uint8_t *>(&Settings), sizeof(Settings));
  }

  if (err.length()) {
    return err;
  }

  // Must check this after saving, or else it is not possible to fix multiple
  // issues which can only corrected on different pages.
  if (!SettingsCheck(err)) { return err; }

  //  }

  err = SaveSecuritySettings();
  return err;
}

String SaveSecuritySettings() {
  MD5Builder md5;
  uint8_t    tmp_md5[16] = { 0 };
  String     err;

  SecuritySettings.validate();
  memcpy(SecuritySettings.ProgmemMd5, CRCValues.runTimeMD5, 16);
  md5.begin();
  md5.add(reinterpret_cast<uint8_t *>(&SecuritySettings), static_cast<uint16_t>(sizeof(SecuritySettings) - 16));
  md5.calculate();
  md5.getBytes(tmp_md5);

  if (memcmp(tmp_md5, SecuritySettings.md5, 16) != 0) {
    // Settings have changed, save to file.
    memcpy(SecuritySettings.md5, tmp_md5, 16);
    err = SaveToFile(SettingsType::getSettingsFileName(SettingsType::Enum::SecuritySettings_Type).c_str(), 0, reinterpret_cast<const uint8_t *>(&SecuritySettings), sizeof(SecuritySettings));

    if (WifiIsAP(WiFi.getMode())) {
      // Security settings are saved, may be update of WiFi settings or hostname.
      WiFiEventData.wifiSetupConnect         = true;
      WiFiEventData.wifiConnectAttemptNeeded = true;
    }
  }
  ExtendedControllerCredentials.save();
  afterloadSettings();
  return err;
}

void afterloadSettings() {
  ExtraTaskSettings.clear(); // make sure these will not contain old settings.

  // Load ResetFactoryDefaultPreference from provisioning.dat if available.
  uint32_t pref_temp = Settings.ResetFactoryDefaultPreference;
  #if FEATURE_CUSTOM_PROVISIONING
  if (fileExists(getFileName(FileType::PROVISIONING_DAT))) {
    MakeProvisioningSettings(ProvisioningSettings);
    if (AllocatedProvisioningSettings()) {
      loadProvisioningSettings(ProvisioningSettings);
      if (ProvisioningSettings.matchingFlashSize()) {
        pref_temp = ProvisioningSettings.ResetFactoryDefaultPreference.getPreference();
      }
    }
  }
  #endif

  // TODO TD-er: Try to get the information from more locations to make it more persistent
  // Maybe EEPROM location?


  ResetFactoryDefaultPreference_struct pref(pref_temp);
  if (modelMatchingFlashSize(pref.getDeviceModel())) {
    ResetFactoryDefaultPreference = pref_temp;
  }
  Scheduler.setEcoMode(Settings.EcoPowerMode());

  if (!Settings.UseRules) {
    eventQueue.clear();
  }
  CheckRunningServices(); // To update changes in hostname.
}

/********************************************************************************************\
   Load settings from file system
 \*********************************************************************************************/
String LoadSettings()
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("LoadSettings"));
  #endif
  String  err;
  uint8_t calculatedMd5[16];
  MD5Builder md5;

  err = LoadFromFile(SettingsType::getSettingsFileName(SettingsType::Enum::BasicSettings_Type).c_str(), 0, reinterpret_cast<uint8_t *>(&Settings), sizeof(SettingsStruct));

  if (err.length()) {
    return err;
  }
  Settings.validate();

  // FIXME @TD-er: As discussed in #1292, the CRC for the settings is now disabled.

  /*
     if (Settings.StructSize > 16) {
      md5.begin();
      md5.add(reinterpret_cast<const uint8_t *>(&Settings), Settings.StructSize -16);
      md5.calculate();
      md5.getBytes(calculatedMd5);
     }
     if (memcmp (calculatedMd5, Settings.md5,16)==0){
      addLog(LOG_LEVEL_INFO,  F("CRC  : Settings CRC           ...OK"));
      if (memcmp(Settings.ProgmemMd5, CRCValues.runTimeMD5, 16)!=0)
        addLog(LOG_LEVEL_INFO, F("CRC  : binary has changed since last save of Settings"));
     }
     else{
      addLog(LOG_LEVEL_ERROR, F("CRC  : Settings CRC           ...FAIL"));
     }
   */

  err = LoadFromFile(SettingsType::getSettingsFileName(SettingsType::Enum::SecuritySettings_Type).c_str(), 0, reinterpret_cast<uint8_t *>(&SecuritySettings), sizeof(SecurityStruct));

  md5.begin();
  md5.add(reinterpret_cast< uint8_t *>(&SecuritySettings), sizeof(SecuritySettings) - 16);
  md5.calculate();
  md5.getBytes(calculatedMd5);

  if (memcmp(calculatedMd5, SecuritySettings.md5, 16) == 0) {
    addLog(LOG_LEVEL_INFO, F("CRC  : SecuritySettings CRC   ...OK "));

    if (memcmp(SecuritySettings.ProgmemMd5, CRCValues.runTimeMD5, 16) != 0) {
      addLog(LOG_LEVEL_INFO, F("CRC  : binary has changed since last save of Settings"));
    }
  }
  else {
    addLog(LOG_LEVEL_ERROR, F("CRC  : SecuritySettings CRC   ...FAIL"));
  }

  ExtendedControllerCredentials.load();

  //  setupStaticIPconfig();
  // FIXME TD-er: Must check if static/dynamic IP was changed and trigger a reconnect? Or is a reboot better when changing those settings?
  afterloadSettings();
  SecuritySettings.validate();
  return err;
}



/********************************************************************************************\
   Disable Plugin, based on bootFailedCount
 \*********************************************************************************************/
uint8_t disablePlugin(uint8_t bootFailedCount) {
  for (taskIndex_t i = 0; i < TASKS_MAX && bootFailedCount > 0; ++i) {
    if (Settings.TaskDeviceEnabled[i]) {
      --bootFailedCount;

      if (bootFailedCount == 0) {
        Settings.TaskDeviceEnabled[i] = false;
      }
    }
  }
  return bootFailedCount;
}

uint8_t disableAllPlugins(uint8_t bootFailedCount) {
  if (bootFailedCount > 0) {
    --bootFailedCount;
    for (taskIndex_t i = 0; i < TASKS_MAX; ++i) {
        Settings.TaskDeviceEnabled[i] = false;
    }
  }
  return bootFailedCount;
}


/********************************************************************************************\
   Disable Controller, based on bootFailedCount
 \*********************************************************************************************/
uint8_t disableController(uint8_t bootFailedCount) {
  for (controllerIndex_t i = 0; i < CONTROLLER_MAX && bootFailedCount > 0; ++i) {
    if (Settings.ControllerEnabled[i]) {
      --bootFailedCount;

      if (bootFailedCount == 0) {
        Settings.ControllerEnabled[i] = false;
      }
    }
  }
  return bootFailedCount;
}

uint8_t disableAllControllers(uint8_t bootFailedCount) {
  if (bootFailedCount > 0) {
    --bootFailedCount;
    for (controllerIndex_t i = 0; i < CONTROLLER_MAX; ++i) {
      Settings.ControllerEnabled[i] = false;
    }
  }
  return bootFailedCount;
}


/********************************************************************************************\
   Disable Notification, based on bootFailedCount
 \*********************************************************************************************/
#if FEATURE_NOTIFIER
uint8_t disableNotification(uint8_t bootFailedCount) {
  for (uint8_t i = 0; i < NOTIFICATION_MAX && bootFailedCount > 0; ++i) {
    if (Settings.NotificationEnabled[i]) {
      --bootFailedCount;

      if (bootFailedCount == 0) {
        Settings.NotificationEnabled[i] = false;
      }
    }
  }
  return bootFailedCount;
}

uint8_t disableAllNotifications(uint8_t bootFailedCount) {
  if (bootFailedCount > 0) {
    --bootFailedCount;
    for (uint8_t i = 0; i < NOTIFICATION_MAX; ++i) {
        Settings.NotificationEnabled[i] = false;
    }
  }
  return bootFailedCount;
}
#endif

/********************************************************************************************\
   Disable Rules, based on bootFailedCount
 \*********************************************************************************************/
uint8_t disableRules(uint8_t bootFailedCount) {
  if (bootFailedCount > 0) {
    --bootFailedCount;
    Settings.UseRules = false;
  }
  return bootFailedCount;
}


bool getAndLogSettingsParameters(bool read, SettingsType::Enum settingsType, int index, int& offset, int& max_size) {
#ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
    String log = read ? F("Read") : F("Write");
    log += F(" settings: ");
    log += SettingsType::getSettingsTypeString(settingsType);
    log += F(" index: ");
    log += index;
    addLogMove(LOG_LEVEL_DEBUG_DEV, log);
  }
#endif // ifndef BUILD_NO_DEBUG
  return SettingsType::getSettingsParameters(settingsType, index, offset, max_size);
}


/********************************************************************************************\
   Load array of Strings from Custom settings
   Use maxStringLength = 0 to optimize for size (strings will be concatenated)
 \*********************************************************************************************/
String LoadStringArray(SettingsType::Enum settingsType, int index, String strings[], uint16_t nrStrings, uint16_t maxStringLength, uint32_t offset_in_block)
{
  int offset, max_size;
  if (!SettingsType::getSettingsParameters(settingsType, index, offset, max_size))
  {
    #ifndef BUILD_NO_DEBUG
    return F("Invalid index for custom settings");
    #else
    return F("Save error");
    #endif
  }

  const uint32_t bufferSize = 128;

  // FIXME TD-er: For now stack allocated, may need to be heap allocated?
  if (maxStringLength >= bufferSize) { return F("Max 128 chars allowed"); }

  char buffer[bufferSize] = {0};

  String   result;
  uint32_t readPos       = offset_in_block;
  uint32_t nextStringPos = readPos;
  uint32_t stringCount   = 0;

  const uint16_t estimatedStringSize = maxStringLength > 0 ? maxStringLength : bufferSize;
  String   tmpString;
  {
    #ifdef USE_SECOND_HEAP
    // Store each string in 2nd heap
    HeapSelectIram ephemeral;
    #endif
    tmpString.reserve(estimatedStringSize);
  }
  {
    while (stringCount < nrStrings && static_cast<int>(readPos) < max_size) {
      const uint32_t readSize = std::min(bufferSize, max_size - readPos);
      result += LoadFromFile(settingsType,
                            index,
                            reinterpret_cast<uint8_t *>(&buffer),
                            readSize,
                            readPos);

      for (uint32_t i = 0; i < readSize && stringCount < nrStrings; ++i) {
        const uint32_t curPos = readPos + i;

        if (curPos >= nextStringPos) {
          if (buffer[i] == 0) {
            if (maxStringLength != 0) {
              // Specific string length, so we have to set the next string position.
              nextStringPos += maxStringLength;
            }
            #ifdef USE_SECOND_HEAP
            // Store each string in 2nd heap
            HeapSelectIram ephemeral;
            #endif

            strings[stringCount] = tmpString;
            tmpString = String();
            tmpString.reserve(estimatedStringSize);
            ++stringCount;
          } else {
            tmpString += buffer[i];
          }
        }
      }
      readPos += bufferSize;
    }
  }

  if ((!tmpString.isEmpty()) && (stringCount < nrStrings)) {
    result              += F("Incomplete custom settings for index ");
    result              += (index + 1);
    strings[stringCount] = tmpString;
  }
  return result;
}

/********************************************************************************************\
   Save array of Strings from Custom settings
   Use maxStringLength = 0 to optimize for size (strings will be concatenated)
 \*********************************************************************************************/
String SaveStringArray(SettingsType::Enum settingsType, int index, const String strings[], uint16_t nrStrings, uint16_t maxStringLength, uint32_t posInBlock)
{
  int offset, max_size;
  if (!SettingsType::getSettingsParameters(settingsType, index, offset, max_size))
  {
    #ifndef BUILD_NO_DEBUG
    return F("Invalid index for custom settings");
    #else
    return F("Save error");
    #endif
  }

  #ifdef ESP8266
  uint16_t bufferSize = 256;
  #endif
  #ifdef ESP32
  uint16_t bufferSize = 1024;
  #endif
  if (bufferSize > max_size) {
    bufferSize = max_size;
  }

  std::vector<uint8_t> buffer;
  buffer.resize(bufferSize);

  String   result;
  int      writePos        = posInBlock;
  uint16_t stringCount     = 0;
  uint16_t stringReadPos   = 0;
  uint16_t nextStringPos   = writePos;
  uint16_t curStringLength = 0;

  if (maxStringLength != 0) {
    // Specified string length, check given strings
    for (uint16_t i = 0; i < nrStrings; ++i) {
      if (strings[i].length() >= maxStringLength) {
        result += getCustomTaskSettingsError(i);
      }
    }
  }

  while (stringCount < nrStrings && writePos < max_size) {
    for (size_t i = 0; i < buffer.size(); ++i) {
      buffer[i] = 0;
    }

    int bufpos = 0;
    for ( ; bufpos < bufferSize && stringCount < nrStrings; ++bufpos) {
      if (stringReadPos == 0) {
        // We're at the start of a string
        curStringLength = strings[stringCount].length();

        if (maxStringLength != 0) {
          if (curStringLength >= maxStringLength) {
            curStringLength = maxStringLength - 1;
          }
        }
      }

      const uint16_t curPos = writePos + bufpos;

      if (curPos >= nextStringPos) {
        if (stringReadPos < curStringLength) {
          buffer[bufpos] = strings[stringCount][stringReadPos];
          ++stringReadPos;
        } else {
          buffer[bufpos] = 0;
          stringReadPos  = 0;
          ++stringCount;

          if (maxStringLength == 0) {
            nextStringPos += curStringLength + 1;
          } else {
            nextStringPos += maxStringLength;
          }
        }
      }
    }

    // Buffer is filled, now write to flash
    // As we write in parts, only count as single write.
    if (RTC.flashDayCounter > 0) {
      RTC.flashDayCounter--;
    }
    result   += SaveToFile(settingsType, index, &(buffer[0]), bufpos, writePos);
    writePos += bufpos;
  }

  if ((writePos >= max_size) && (stringCount < nrStrings)) {
    result += F("Error: Not all strings fit in custom settings.");
  }
  return result;
}



/********************************************************************************************\
   Save Task settings to file system
 \*********************************************************************************************/
String SaveTaskSettings(taskIndex_t TaskIndex)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("SaveTaskSettings"));
  #endif

  if (ExtraTaskSettings.TaskIndex != TaskIndex) {
    #ifndef BUILD_NO_DEBUG
    return F("SaveTaskSettings taskIndex does not match");
    #else
    return F("Save error");
    #endif
  }
  String err = SaveToFile(SettingsType::Enum::TaskSettings_Type,
                          TaskIndex,
                          reinterpret_cast<const uint8_t *>(&ExtraTaskSettings),
                          sizeof(struct ExtraTaskSettingsStruct));

  if (err.isEmpty()) {
    err = checkTaskSettings(TaskIndex);
  }
  return err;
}

/********************************************************************************************\
   Load Task settings from file system
 \*********************************************************************************************/
String LoadTaskSettings(taskIndex_t TaskIndex)
{
  if (ExtraTaskSettings.TaskIndex == TaskIndex) {
    return String(); // already loaded
  }
  if (!validTaskIndex(TaskIndex)) {
    return String(); // Un-initialized task index.
  }
  ExtraTaskSettings.clear();
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("LoadTaskSettings"));
  #endif

  START_TIMER
  const String result = LoadFromFile(SettingsType::Enum::TaskSettings_Type, TaskIndex, reinterpret_cast<uint8_t *>(&ExtraTaskSettings), sizeof(struct ExtraTaskSettingsStruct));

  // After loading, some settings may need patching.
  ExtraTaskSettings.TaskIndex = TaskIndex; // Needed when an empty task was requested

  const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(TaskIndex);
  if (validDeviceIndex(DeviceIndex)) {
    if (!Device[DeviceIndex].configurableDecimals()) {
      // Nr of decimals cannot be configured, so set them to 0 just to be sure.
      for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
        ExtraTaskSettings.TaskDeviceValueDecimals[i] = 0;
      }      
    }
  }

  if (ExtraTaskSettings.TaskDeviceValueNames[0][0] == 0) {
    // if field set empty, reload defaults
    struct EventStruct TempEvent(TaskIndex);
    String tmp;

    // the plugin call should populate ExtraTaskSettings with its default values.
    PluginCall(PLUGIN_GET_DEVICEVALUENAMES, &TempEvent, tmp);
  }
  ExtraTaskSettings.validate();
  Cache.updateExtraTaskSettingsCache();
  STOP_TIMER(LOAD_TASK_SETTINGS);

  return result;
}

/********************************************************************************************\
   Save Custom Task settings to file system
 \*********************************************************************************************/
String SaveCustomTaskSettings(taskIndex_t TaskIndex, const uint8_t *memAddress, int datasize, uint32_t posInBlock)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("SaveCustomTaskSettings"));
  #endif
  return SaveToFile(SettingsType::Enum::CustomTaskSettings_Type, TaskIndex, memAddress, datasize, posInBlock);
}

/********************************************************************************************\
   Save array of Strings to Custom Task settings
   Use maxStringLength = 0 to optimize for size (strings will be concatenated)
 \*********************************************************************************************/
String SaveCustomTaskSettings(taskIndex_t TaskIndex, String strings[], uint16_t nrStrings, uint16_t maxStringLength, uint32_t posInBlock)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("SaveCustomTaskSettings"));
  #endif
  return SaveStringArray(
    SettingsType::Enum::CustomTaskSettings_Type, TaskIndex,
    strings, nrStrings, maxStringLength, posInBlock);
}

String getCustomTaskSettingsError(uint8_t varNr) {
  String error = F("Error: Text too long for line ");

  error += varNr + 1;
  error += '\n';
  return error;
}

/********************************************************************************************\
   Clear custom task settings
 \*********************************************************************************************/
String ClearCustomTaskSettings(taskIndex_t TaskIndex)
{
  // addLog(LOG_LEVEL_DEBUG, F("Clearing custom task settings"));
  return ClearInFile(SettingsType::Enum::CustomTaskSettings_Type, TaskIndex);
}

/********************************************************************************************\
   Load Custom Task settings from file system
 \*********************************************************************************************/
String LoadCustomTaskSettings(taskIndex_t TaskIndex, uint8_t *memAddress, int datasize, int offset_in_block)
{
  START_TIMER;
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("LoadCustomTaskSettings"));
  #endif
  String result = LoadFromFile(SettingsType::Enum::CustomTaskSettings_Type, TaskIndex, memAddress, datasize, offset_in_block);
  STOP_TIMER(LOAD_CUSTOM_TASK_STATS);
  return result;
}

/********************************************************************************************\
   Load array of Strings from Custom Task settings
   Use maxStringLength = 0 to optimize for size (strings will be concatenated)
 \*********************************************************************************************/
String LoadCustomTaskSettings(taskIndex_t TaskIndex, String strings[], uint16_t nrStrings, uint16_t maxStringLength, uint32_t offset_in_block)
{
  START_TIMER;
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("LoadCustomTaskSettings"));
  #endif
  String result = LoadStringArray(SettingsType::Enum::CustomTaskSettings_Type,
                           TaskIndex,
                           strings, nrStrings, maxStringLength, offset_in_block);
  STOP_TIMER(LOAD_CUSTOM_TASK_STATS);
  return result;
}

/********************************************************************************************\
   Save Controller settings to file system
 \*********************************************************************************************/
String SaveControllerSettings(controllerIndex_t ControllerIndex, ControllerSettingsStruct& controller_settings)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("SaveControllerSettings"));
  #endif
  controller_settings.validate(); // Make sure the saved controller settings have proper values.
  return SaveToFile(SettingsType::Enum::ControllerSettings_Type, ControllerIndex,
                    reinterpret_cast<const uint8_t *>(&controller_settings), sizeof(controller_settings));
}

/********************************************************************************************\
   Load Controller settings to file system
 \*********************************************************************************************/
String LoadControllerSettings(controllerIndex_t ControllerIndex, ControllerSettingsStruct& controller_settings) {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("LoadControllerSettings"));
  #endif
  String result =
    LoadFromFile(SettingsType::Enum::ControllerSettings_Type, ControllerIndex,
                 reinterpret_cast<uint8_t *>(&controller_settings), sizeof(controller_settings));
  controller_settings.validate(); // Make sure the loaded controller settings have proper values.
  return result;
}

/********************************************************************************************\
   Clear Custom Controller settings
 \*********************************************************************************************/
String ClearCustomControllerSettings(controllerIndex_t ControllerIndex)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("ClearCustomControllerSettings"));
  #endif

  // addLog(LOG_LEVEL_DEBUG, F("Clearing custom controller settings"));
  return ClearInFile(SettingsType::Enum::CustomControllerSettings_Type, ControllerIndex);
}

/********************************************************************************************\
   Save Custom Controller settings to file system
 \*********************************************************************************************/
String SaveCustomControllerSettings(controllerIndex_t ControllerIndex, const uint8_t *memAddress, int datasize)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("SaveCustomControllerSettings"));
  #endif
  return SaveToFile(SettingsType::Enum::CustomControllerSettings_Type, ControllerIndex, memAddress, datasize);
}

/********************************************************************************************\
   Load Custom Controller settings to file system
 \*********************************************************************************************/
String LoadCustomControllerSettings(controllerIndex_t ControllerIndex, uint8_t *memAddress, int datasize)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("LoadCustomControllerSettings"));
  #endif
  return LoadFromFile(SettingsType::Enum::CustomControllerSettings_Type, ControllerIndex, memAddress, datasize);
}


#if FEATURE_CUSTOM_PROVISIONING
/********************************************************************************************\
   Save Provisioning Settings
 \*********************************************************************************************/
String saveProvisioningSettings(ProvisioningStruct& ProvisioningSettings)
{
  MD5Builder md5;
  uint8_t    tmp_md5[16] = { 0 };
  String     err;

  ProvisioningSettings.validate();
  memcpy(ProvisioningSettings.ProgmemMd5, CRCValues.runTimeMD5, 16);
  md5.begin();
  md5.add((uint8_t *)&ProvisioningSettings + 16, sizeof(ProvisioningSettings) - 16);
  md5.calculate();
  md5.getBytes(tmp_md5);

  if (memcmp(tmp_md5, ProvisioningSettings.md5, 16) != 0) {
    // Settings have changed, save to file.
    memcpy(ProvisioningSettings.md5, tmp_md5, 16);
    err = SaveToFile_trunc(getFileName(FileType::PROVISIONING_DAT, 0).c_str(), 0, (uint8_t *)&ProvisioningSettings, sizeof(ProvisioningStruct));
  }
  return err;
}

/********************************************************************************************\
   Load Provisioning Settings
 \*********************************************************************************************/
String loadProvisioningSettings(ProvisioningStruct& ProvisioningSettings)
{
  uint8_t calculatedMd5[16] = { 0 };
  MD5Builder md5;

  String err = LoadFromFile(getFileName(FileType::PROVISIONING_DAT, 0).c_str(), 0, (uint8_t *)&ProvisioningSettings, sizeof(ProvisioningStruct));
  md5.begin();
  md5.add(((uint8_t *)&ProvisioningSettings) + 16, sizeof(ProvisioningSettings) - 16);
  md5.calculate();
  md5.getBytes(calculatedMd5);

  if (memcmp(calculatedMd5, ProvisioningSettings.md5, 16) == 0) {
    addLog(LOG_LEVEL_INFO, F("CRC  : ProvisioningSettings CRC   ...OK "));

    if (memcmp(ProvisioningSettings.ProgmemMd5, CRCValues.runTimeMD5, 16) != 0) {
      addLog(LOG_LEVEL_INFO, F("CRC  : binary has changed since last save of Settings"));
    }
  }
  else {
    addLog(LOG_LEVEL_ERROR, F("CRC  : ProvisioningSettings CRC   ...FAIL"));
  }
  ProvisioningSettings.validate();
  return err;
}

#endif

#if FEATURE_NOTIFIER
/********************************************************************************************\
   Save Controller settings to file system
 \*********************************************************************************************/
String SaveNotificationSettings(int NotificationIndex, const uint8_t *memAddress, int datasize)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("SaveNotificationSettings"));
  #endif
  return SaveToFile(SettingsType::Enum::NotificationSettings_Type, NotificationIndex, memAddress, datasize);
}

/********************************************************************************************\
   Load Controller settings to file system
 \*********************************************************************************************/
String LoadNotificationSettings(int NotificationIndex, uint8_t *memAddress, int datasize)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("LoadNotificationSettings"));
  #endif
  return LoadFromFile(SettingsType::Enum::NotificationSettings_Type, NotificationIndex, memAddress, datasize);
}
#endif
/********************************************************************************************\
   Init a file with zeros on file system
 \*********************************************************************************************/
String InitFile(const String& fname, int datasize)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("InitFile"));
  #endif
  FLASH_GUARD();

  fs::File f = tryOpenFile(fname, "w");

  if (f) {
    for (int x = 0; x < datasize; x++)
    {
      // See https://github.com/esp8266/Arduino/commit/b1da9eda467cc935307d553692fdde2e670db258#r32622483
      uint8_t zero_value = 0;
      SPIFFS_CHECK(f.write(&zero_value, 1), fname.c_str());
    }
    f.close();
  }

  // OK
  return String();
}

String InitFile(SettingsType::Enum settingsType)
{
  return InitFile(SettingsType::getSettingsFile(settingsType));
}

String InitFile(SettingsType::SettingsFileEnum file_type)
{
  return InitFile(SettingsType::getSettingsFileName(file_type), 
                  SettingsType::getInitFileSize(file_type));
}

/********************************************************************************************\
   Save data into config file on file system
 \*********************************************************************************************/
String SaveToFile(const char *fname, int index, const uint8_t *memAddress, int datasize)
{
  return doSaveToFile(fname, index, memAddress, datasize, "r+");
}

String SaveToFile_trunc(const char *fname, int index, const uint8_t *memAddress, int datasize)
{
  return doSaveToFile(fname, index, memAddress, datasize, "w+");
}

// See for mode description: https://github.com/esp8266/Arduino/blob/master/doc/filesystem.rst
String doSaveToFile(const char *fname, int index, const uint8_t *memAddress, int datasize, const char *mode)
{
#ifndef BUILD_NO_DEBUG
#ifndef ESP32

  if (allocatedOnStack(memAddress)) {
    String log = F("SaveToFile: ");
    log += fname;
    log += F(" ERROR, Data allocated on stack");
    addLog(LOG_LEVEL_ERROR, log);

    //    return log;  // FIXME TD-er: Should this be considered a breaking error?
  }
#endif // ifndef ESP32
#endif

  if (index < 0) {
    #ifndef BUILD_NO_DEBUG
    String log = F("SaveToFile: ");
    log += fname;
    log += F(" ERROR, invalid position in file");
    #else
    String log = F("Save error");
    #endif
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }
  START_TIMER;
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("SaveToFile"));
  #endif
  FLASH_GUARD();
  
  #ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("SaveToFile: free stack: ");
    log += getCurrentFreeStack();
    addLogMove(LOG_LEVEL_INFO, log);
  }
  #endif
  delay(1);
  unsigned long timer = millis() + 50;
  fs::File f          = tryOpenFile(fname, mode);

  if (f) {
    clearAllCaches();
    SPIFFS_CHECK(f,                          fname);
    SPIFFS_CHECK(f.seek(index, fs::SeekSet), fname);
    const uint8_t *pointerToByteToSave = memAddress;

    for (int x = 0; x < datasize; x++)
    {
      // See https://github.com/esp8266/Arduino/commit/b1da9eda467cc935307d553692fdde2e670db258#r32622483
      uint8_t byteToSave = *pointerToByteToSave;
      SPIFFS_CHECK(f.write(&byteToSave, 1), fname);
      pointerToByteToSave++;

      if (x % 256 == 0) {
        // one page written, do some background tasks
        timer = millis() + 50;
        delay(0);
      }

      if (timeOutReached(timer)) {
        timer += 50;
        delay(0);
      }
    }
    f.close();
    #ifndef BUILD_NO_DEBUG
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      log.reserve(48);
      log += F("FILE : Saved ");
      log += fname;
      log += F(" offset: ");
      log += index;
      log += F(" size: ");
      log += datasize;
      addLogMove(LOG_LEVEL_INFO, log);
    }
    #endif
  } else {
    #ifndef BUILD_NO_DEBUG
    String log = F("SaveToFile: ");
    log += fname;
    log += F(" ERROR, Cannot save to file");
    #else
    String log = F("Save error");
    #endif

    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }
  STOP_TIMER(SAVEFILE_STATS);
  #ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("SaveToFile: free stack after: ");
    log += getCurrentFreeStack();
    addLogMove(LOG_LEVEL_INFO, log);
  }
  #endif

  // OK
  return String();
}

/********************************************************************************************\
   Clear a certain area in a file (set to 0)
 \*********************************************************************************************/
String ClearInFile(const char *fname, int index, int datasize)
{
  if (index < 0) {
    #ifndef BUILD_NO_DEBUG
    String log = F("ClearInFile: ");
    log += fname;
    log += F(" ERROR, invalid position in file");
    #else
    String log = F("Save error");
    #endif

    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }

  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("ClearInFile"));
  #endif
  FLASH_GUARD();

  fs::File f = tryOpenFile(fname, "r+");

  if (f) {
    SPIFFS_CHECK(f.seek(index, fs::SeekSet), fname);

    for (int x = 0; x < datasize; x++)
    {
      // See https://github.com/esp8266/Arduino/commit/b1da9eda467cc935307d553692fdde2e670db258#r32622483
      uint8_t zero_value = 0;
      SPIFFS_CHECK(f.write(&zero_value, 1), fname);
    }
    f.close();
  } else {
    #ifndef BUILD_NO_DEBUG
    String log = F("ClearInFile: ");
    log += fname;
    log += F(" ERROR, Cannot save to file");
    #else
    String log = F("Save error");
    #endif
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }

  // OK
  return String();
}

/********************************************************************************************\
   Load data from config file on file system
 \*********************************************************************************************/
String LoadFromFile(const char *fname, int offset, uint8_t *memAddress, int datasize)
{
  if (offset < 0) {
    #ifndef BUILD_NO_DEBUG
    String log = F("LoadFromFile: ");
    log += fname;
    log += F(" ERROR, invalid position in file");
    #else
    String log = F("Load error");
    #endif
    addLog(LOG_LEVEL_ERROR, log);
    return log;
  }
  delay(0);
  START_TIMER;
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("LoadFromFile"));
  #endif
  fs::File f = tryOpenFile(fname, "r");
  SPIFFS_CHECK(f,                            fname);
  SPIFFS_CHECK(f.seek(offset, fs::SeekSet),  fname);
  SPIFFS_CHECK(f.read(memAddress, datasize), fname);
  f.close();

  STOP_TIMER(LOADFILE_STATS);
  delay(0);

  return String();
}

/********************************************************************************************\
   Wrapper functions to handle errors in accessing settings
 \*********************************************************************************************/
String getSettingsFileIndexRangeError(bool read, SettingsType::Enum settingsType, int index) {
  if (settingsType >= SettingsType::Enum::SettingsType_MAX) {
    String error = F("Unknown settingsType: ");
    error += static_cast<int>(settingsType);
    return error;
  }
  String error = read ? F("Load") : F("Save");
  #ifndef BUILD_NO_DEBUG
  error += SettingsType::getSettingsTypeString(settingsType);
  error += F(" index out of range: ");
  error += index;
  #else
  error += F(" error");
  #endif
  return error;
}

String getSettingsFileDatasizeError(bool read, SettingsType::Enum settingsType, int index, int datasize, int max_size) {
  String error = read ? F("Load") : F("Save");
  #ifndef BUILD_NO_DEBUG
  error += SettingsType::getSettingsTypeString(settingsType);
  error += '(';
  error += index;
  error += F(") datasize(");
  error += datasize;
  error += F(") > max_size(");
  error += max_size;
  error += ')';
  #else
  error += F(" error");
  #endif
  
  return error;
}

String LoadFromFile(SettingsType::Enum settingsType, int index, uint8_t *memAddress, int datasize, int offset_in_block) {
  bool read = true;
  int  offset, max_size;

  if (!getAndLogSettingsParameters(read, settingsType, index, offset, max_size)) {
    return getSettingsFileIndexRangeError(read, settingsType, index);
  }

  if ((datasize + offset_in_block) > max_size) {
    return getSettingsFileDatasizeError(read, settingsType, index, datasize, max_size);
  }
  const String fname = SettingsType::getSettingsFileName(settingsType);
  return LoadFromFile(fname.c_str(), (offset + offset_in_block), memAddress, datasize);
}

String SaveToFile(SettingsType::Enum settingsType, int index, const uint8_t *memAddress, int datasize, int posInBlock) {
  bool read = false;
  int  offset, max_size;

  if (!getAndLogSettingsParameters(read, settingsType, index, offset, max_size)) {
    return getSettingsFileIndexRangeError(read, settingsType, index);
  }

  if ((datasize > max_size) || ((posInBlock + datasize) > max_size)) {
    return getSettingsFileDatasizeError(read, settingsType, index, datasize, max_size);
  }
  const String fname = SettingsType::getSettingsFileName(settingsType);
  if (!fileExists(fname)) {
    InitFile(settingsType);
  }
  return SaveToFile(fname.c_str(), offset + posInBlock, memAddress, datasize);
}

String ClearInFile(SettingsType::Enum settingsType, int index) {
  bool read = false;
  int  offset, max_size;

  if (!getAndLogSettingsParameters(read, settingsType, index, offset, max_size)) {
    return getSettingsFileIndexRangeError(read, settingsType, index);
  }
  const String fname = SettingsType::getSettingsFileName(settingsType);
  return ClearInFile(fname.c_str(), offset, max_size);
}

/********************************************************************************************\
   Check file system area settings
 \*********************************************************************************************/
int SpiffsSectors()
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("SpiffsSectors"));
  #endif
  #if defined(ESP8266)
    # ifdef CORE_POST_2_6_0
  uint32_t _sectorStart = ((uint32_t)&_FS_start - 0x40200000) / SPI_FLASH_SEC_SIZE;
  uint32_t _sectorEnd   = ((uint32_t)&_FS_end - 0x40200000) / SPI_FLASH_SEC_SIZE;
    # else // ifdef CORE_POST_2_6_0
  uint32_t _sectorStart = ((uint32_t)&_SPIFFS_start - 0x40200000) / SPI_FLASH_SEC_SIZE;
  uint32_t _sectorEnd   = ((uint32_t)&_SPIFFS_end - 0x40200000) / SPI_FLASH_SEC_SIZE;
    # endif // ifdef CORE_POST_2_6_0

  return _sectorEnd - _sectorStart;
  #endif // if defined(ESP8266)
  #if defined(ESP32)
  return 32;
  #endif // if defined(ESP32)
}

size_t SpiffsUsedBytes() {
  size_t result = 1; // Do not output 0, this may be used in divisions.

  #ifdef ESP32
  result = ESPEASY_FS.usedBytes();
  #endif // ifdef ESP32
  #ifdef ESP8266
  fs::FSInfo fs_info;
  ESPEASY_FS.info(fs_info);
  result = fs_info.usedBytes;
  #endif // ifdef ESP8266
  return result;
}

size_t SpiffsTotalBytes() {
  static size_t result = 1; // Do not output 0, this may be used in divisions.
  if (result == 1) {
    #ifdef ESP32
    result = ESPEASY_FS.totalBytes();
    #endif // ifdef ESP32
    #ifdef ESP8266
    fs::FSInfo fs_info;
    ESPEASY_FS.info(fs_info);
    result = fs_info.totalBytes;
    #endif // ifdef ESP8266
  }
  return result;
}

size_t SpiffsBlocksize() {
  static size_t result = 1;
  if (result == 1) {
    #ifdef ESP32
    result = 8192;        // Just assume 8k, since we cannot query it
    #endif // ifdef ESP32
    #ifdef ESP8266
    fs::FSInfo fs_info;
    ESPEASY_FS.info(fs_info);
    result = fs_info.blockSize;
    #endif // ifdef ESP8266
  }
  return result;
}

size_t SpiffsPagesize() {
  static size_t result = 1;
  if (result == 1) {
    #ifdef ESP32
    result = 256;        // Just assume 256, since we cannot query it
    #endif // ifdef ESP32
    #ifdef ESP8266
    fs::FSInfo fs_info;
    ESPEASY_FS.info(fs_info);
    result = fs_info.pageSize;
    #endif // ifdef ESP8266
  }
  return result;
}

size_t SpiffsFreeSpace() {
  int freeSpace = SpiffsTotalBytes() - SpiffsUsedBytes();
  const size_t blocksize = SpiffsBlocksize();

  if (freeSpace < static_cast<int>(2 * blocksize)) {
    // Not enough free space left to store anything
    // There needs to be minimum of 2 free blocks.
    return 0;
  }
  return freeSpace - 2 * blocksize;
}

bool SpiffsFull() {
  return SpiffsFreeSpace() == 0;
}

/********************************************************************************************\
   Handling cached data
 \*********************************************************************************************/
String createCacheFilename(unsigned int count) {
  String fname;

  fname.reserve(16);
  #ifdef ESP32
  fname = '/';
  #endif // ifdef ESP32
  fname += F("cache_");
  fname += String(count);
  fname += F(".bin");
  return fname;
}

// Match string with an integer between '_' and ".bin"
int getCacheFileCountFromFilename(const String& fname) {
  int startpos = fname.indexOf('_');

  if (startpos < 0) { return -1; }
  int endpos = fname.indexOf(F(".bin"));

  if (endpos < 0) { return -1; }

  //  String digits = fname.substring(startpos + 1, endpos);
  int result;

  if (validIntFromString(fname.substring(startpos + 1, endpos), result)) {
    return result;
  }
  return -1;
}

// Look into the filesystem to see if there are any cache files present on the filesystem
// Return true if any found.
bool getCacheFileCounters(uint16_t& lowest, uint16_t& highest, size_t& filesizeHighest) {
  lowest          = 65535;
  highest         = 0;
  filesizeHighest = 0;
#ifdef ESP8266
  fs::Dir dir = ESPEASY_FS.openDir(F("cache"));

  while (dir.next()) {
    String filename = dir.fileName();
    int    count    = getCacheFileCountFromFilename(filename);

    if (count >= 0) {
      if (lowest > count) {
        lowest = count;
      }

      if (highest < count) {
        highest         = count;
        filesizeHighest = dir.fileSize();
      }
    }
  }
#endif // ESP8266
#ifdef ESP32
  fs::File root = ESPEASY_FS.open(F("/"));
  fs::File file = root.openNextFile();

  while (file)
  {
    if (!file.isDirectory()) {
      const String fname(file.name());
      if (fname.startsWith(F("/cache")) || fname.startsWith(F("cache"))) {
        int count = getCacheFileCountFromFilename(fname);

        if (count >= 0) {
          if (lowest > count) {
            lowest = count;
          }

          if (highest < count) {
            highest         = count;
            filesizeHighest = file.size();
          }
        } else {
          addLog(LOG_LEVEL_INFO, String(F("RTC  : Cannot get count from: ")) + fname);
        }
      }
    }
    file = root.openNextFile();
  }
#endif // ESP32

  if (lowest <= highest) {
    return true;
  }
  lowest  = 0;
  highest = 0;
  return false;
}

/********************************************************************************************\
   Get partition table information
 \*********************************************************************************************/
#ifdef ESP32
String getPartitionType(uint8_t pType, uint8_t pSubType) {
  esp_partition_type_t partitionType       = static_cast<esp_partition_type_t>(pType);
  esp_partition_subtype_t partitionSubType = static_cast<esp_partition_subtype_t>(pSubType);

  if (partitionType == ESP_PARTITION_TYPE_APP) {
    if ((partitionSubType >= ESP_PARTITION_SUBTYPE_APP_OTA_MIN) &&
        (partitionSubType < ESP_PARTITION_SUBTYPE_APP_OTA_MAX)) {
      String result = F("OTA partition ");
      result += (partitionSubType - ESP_PARTITION_SUBTYPE_APP_OTA_MIN);
      return result;
    }

    switch (partitionSubType) {
      case ESP_PARTITION_SUBTYPE_APP_FACTORY: return F("Factory app");
      case ESP_PARTITION_SUBTYPE_APP_TEST:    return F("Test app");
      default: break;
    }
  }

  if (partitionSubType == 0x99) {
    return F("EEPROM"); // Not defined in esp_partition_subtype_t
  }

  if (partitionType == ESP_PARTITION_TYPE_DATA) {
    switch (partitionSubType) {
      case ESP_PARTITION_SUBTYPE_DATA_OTA:      return F("OTA selection");
      case ESP_PARTITION_SUBTYPE_DATA_PHY:      return F("PHY init data");
      case ESP_PARTITION_SUBTYPE_DATA_NVS:      return F("NVS");
      case ESP_PARTITION_SUBTYPE_DATA_COREDUMP: return F("COREDUMP");
      case ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD: return F("ESPHTTPD");
      case ESP_PARTITION_SUBTYPE_DATA_FAT:      return F("FAT");
      case ESP_PARTITION_SUBTYPE_DATA_SPIFFS:   
        #ifdef USE_LITTLEFS
        return F("LittleFS");
        #else
        return F("SPIFFS");
        #endif
      default: break;
    }
  }
  String result = F("Unknown(");
  result += partitionSubType;
  result += ')';
  return result;
}

String getPartitionTableHeader(const String& itemSep, const String& lineEnd) {
  String result;

  result += F("Address");
  result += itemSep;
  result += F("Size");
  result += itemSep;
  result += F("Label");
  result += itemSep;
  result += F("Partition Type");
  result += itemSep;
  result += F("Encrypted");
  result += lineEnd;
  return result;
}

String getPartitionTable(uint8_t pType, const String& itemSep, const String& lineEnd) {
  esp_partition_type_t partitionType = static_cast<esp_partition_type_t>(pType);
  String result;
  esp_partition_iterator_t _mypartiterator = esp_partition_find(partitionType, ESP_PARTITION_SUBTYPE_ANY, nullptr);

  if (_mypartiterator) {
    do {
      const esp_partition_t *_mypart = esp_partition_get(_mypartiterator);
      result += formatToHex(_mypart->address);
      result += itemSep;
      result += formatToHex_decimal(_mypart->size, 1024);
      result += itemSep;
      result += _mypart->label;
      result += itemSep;
      result += getPartitionType(_mypart->type, _mypart->subtype);
      result += itemSep;
      result += (_mypart->encrypted ? F("Yes") : F("-"));
      result += lineEnd;
    } while ((_mypartiterator = esp_partition_next(_mypartiterator)) != nullptr);
  }
  esp_partition_iterator_release(_mypartiterator);
  return result;
}

#endif // ifdef ESP32

#if FEATURE_DOWNLOAD
String downloadFileType(const String& url, const String& user, const String& pass, FileType::Enum filetype, unsigned int filenr)
{
  if (!getDownloadFiletypeChecked(filetype, filenr)) {
    // Not selected, so not downloaded
    return F("Not Allowed");
  }

  String filename = getFileName(filetype, filenr);
  String fullUrl;

  fullUrl.reserve(url.length() + filename.length() + 1); // May need to add an extra slash
  fullUrl = url;
  fullUrl = parseTemplate(fullUrl, true);                // URL encode

  // URLEncode may also encode the '/' into "%2f"
  // FIXME TD-er: Can this really occur?
  fullUrl.replace(F("%2f"), F("/"));

  while (filename.startsWith(F("/"))) {
    filename = filename.substring(1);
  }

  if (!fullUrl.endsWith(F("/"))) {
    fullUrl += F("/");
  }
  fullUrl += filename;

  String error;

  if (ResetFactoryDefaultPreference.deleteFirst()) {
    if (fileExists(filename) && !tryDeleteFile(filename)) {
      return F("Could not delete existing file");
    }

    if (!downloadFile(fullUrl, filename, user, pass, error)) {
      return error;
    }
  } else {
    if (fileExists(filename)) {
      String filename_bak = filename;
      filename_bak += F("_bak");
      if (fileExists(filename_bak)) {
        return F("Could not rename to _bak");
      }

      // Must download it to a tmp file.
      String tmpfile = filename;
      tmpfile += F("_tmp");

      if (!downloadFile(fullUrl, tmpfile, user, pass, error)) {
        return error;
      }

      if (fileExists(filename) && !tryRenameFile(filename, filename_bak)) {
        return F("Could not rename to _bak");
      } else {
        // File does not exist (anymore)
        if (!tryRenameFile(tmpfile, filename)) {
          error = F("Could not rename tmp file");

          if (tryRenameFile(filename_bak, filename)) {
            error += F("... reverted");
          } else {
            error += F(" Not reverted!");
          }
          return error;
        }
      }
    } else {
      if (!downloadFile(fullUrl, filename, user, pass, error)) {
        return error;
      }
    }
  }
  return error;
}

#endif // if FEATURE_DOWNLOAD

#if FEATURE_CUSTOM_PROVISIONING

String downloadFileType(FileType::Enum filetype, unsigned int filenr)
{
  if (!ResetFactoryDefaultPreference.allowFetchByCommand()) {
    return F("Not Allowed");
  }
  String url, user, pass;

  {
    MakeProvisioningSettings(ProvisioningSettings);

    if (AllocatedProvisioningSettings()) {
      loadProvisioningSettings(ProvisioningSettings);
      url  = ProvisioningSettings.url;
      user = ProvisioningSettings.user;
      pass = ProvisioningSettings.pass;
    }
  }
  return downloadFileType(url, user, pass, filetype, filenr);
}

#endif // if FEATURE_CUSTOM_PROVISIONING

#include "../Helpers/StringProvider.h"

#if FEATURE_ETHERNET
# include <ETH.h>
#endif // if FEATURE_ETHERNET

#include "../../ESPEasy-Globals.h"

#include "../CustomBuild/CompiletimeDefines.h"

#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../ESPEasyCore/ESPEasyWifi.h"
#if FEATURE_ETHERNET
#include "../ESPEasyCore/ESPEasyEth.h"
#endif

#include "../Globals/ESPEasy_Scheduler.h"
#include "../Globals/ESPEasy_time.h"
#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/NetworkState.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/Settings.h"
#include "../Globals/WiFi_AP_Candidates.h"

#include "../Helpers/Convert.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Memory.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Scheduler.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringGenerator_System.h"
#include "../Helpers/StringGenerator_WiFi.h"

#include "../WebServer/JSON.h"
#include "../WebServer/AccessControl.h"


String getInternalLabel(LabelType::Enum label, char replaceSpace) {
  return to_internal_string(getLabel(label), replaceSpace);
}

const __FlashStringHelper * getLabel(LabelType::Enum label) {
  switch (label)
  {
    case LabelType::UNIT_NR:                return F("Unit Number");
    case LabelType::UNIT_NAME:              return F("Unit Name");
    case LabelType::HOST_NAME:              return F("Hostname");

    case LabelType::LOCAL_TIME:             return F("Local Time");
    case LabelType::TIME_SOURCE:            return F("Time Source");
    case LabelType::TIME_WANDER:            return F("Time Wander");
    case LabelType::UPTIME:                 return F("Uptime");
    case LabelType::LOAD_PCT:               return F("Load");
    case LabelType::LOOP_COUNT:             return F("Load LC");
    case LabelType::CPU_ECO_MODE:           return F("CPU Eco Mode");
#ifdef ESP8266 // TD-er: Disable setting TX power on ESP32 as it seems to cause issues on IDF4.4
    case LabelType::WIFI_TX_MAX_PWR:        return F("Max WiFi TX Power");
    case LabelType::WIFI_CUR_TX_PWR:        return F("Current WiFi TX Power");
    case LabelType::WIFI_SENS_MARGIN:       return F("WiFi Sensitivity Margin");
    case LabelType::WIFI_SEND_AT_MAX_TX_PWR:return F("Send With Max TX Power");
#endif
    case LabelType::WIFI_NR_EXTRA_SCANS:    return F("Extra WiFi scan loops");
    case LabelType::WIFI_USE_LAST_CONN_FROM_RTC: return F("Use Last Connected AP from RTC");

    case LabelType::FREE_MEM:               return F("Free RAM");
    case LabelType::FREE_STACK:             return F("Free Stack");
#ifdef USE_SECOND_HEAP
    case LabelType::FREE_HEAP_IRAM:         return F("Free 2nd Heap");
#endif

#if defined(CORE_POST_2_5_0) || defined(ESP32)
  #ifndef LIMIT_BUILD_SIZE
    case LabelType::HEAP_MAX_FREE_BLOCK:    return F("Heap Max Free Block");
  #endif
#endif // if defined(CORE_POST_2_5_0) || defined(ESP32)
#if defined(CORE_POST_2_5_0)
  #ifndef LIMIT_BUILD_SIZE
    case LabelType::HEAP_FRAGMENTATION:     return F("Heap Fragmentation");
  #endif
#endif // if defined(CORE_POST_2_5_0)

#ifdef ESP32
    case LabelType::HEAP_SIZE:              return F("Heap Size");
    case LabelType::HEAP_MIN_FREE:          return F("Heap Min Free");
    #ifdef BOARD_HAS_PSRAM
    case LabelType::PSRAM_SIZE:             return F("PSRAM Size");
    case LabelType::PSRAM_FREE:             return F("PSRAM Free");
    case LabelType::PSRAM_MIN_FREE:         return F("PSRAM Min Free");
    case LabelType::PSRAM_MAX_FREE_BLOCK:   return F("PSRAM Max Free Block");
    #endif // BOARD_HAS_PSRAM
#endif // ifdef ESP32

    case LabelType::JSON_BOOL_QUOTES:           return F("JSON bool output without quotes");
    case LabelType::ENABLE_TIMING_STATISTICS:   return F("Collect Timing Statistics");
    case LabelType::ENABLE_RULES_CACHING:       return F("Enable Rules Cache");
//    case LabelType::ENABLE_RULES_EVENT_REORDER: return F("Optimize Rules Cache Event Order"); // TD-er: Disabled for now
    case LabelType::TASKVALUESET_ALL_PLUGINS:   return F("Allow TaskValueSet on all plugins");
    case LabelType::ALLOW_OTA_UNLIMITED:        return F("Allow OTA without size-check");
    case LabelType::ENABLE_CLEAR_HUNG_I2C_BUS:  return F("Try clear I2C bus when stuck");
#ifndef BUILD_NO_RAM_TRACKER
    case LabelType::ENABLE_RAM_TRACKING:    return F("Enable RAM Tracker");
#endif

    case LabelType::BOOT_TYPE:              return F("Last Boot Cause");
    case LabelType::BOOT_COUNT:             return F("Boot Count");
    case LabelType::DEEP_SLEEP_ALTERNATIVE_CALL: return F("Deep Sleep Alternative");
    case LabelType::RESET_REASON:           return F("Reset Reason");
    case LabelType::LAST_TASK_BEFORE_REBOOT: return F("Last Action before Reboot");
    case LabelType::SW_WD_COUNT:            return F("SW WD count");


    case LabelType::WIFI_CONNECTION:        return F("WiFi Connection");
    case LabelType::WIFI_RSSI:              return F("RSSI");
    case LabelType::IP_CONFIG:              return F("IP Config");
    case LabelType::IP_CONFIG_STATIC:       return F("Static");
    case LabelType::IP_CONFIG_DYNAMIC:      return F("DHCP");
    case LabelType::IP_ADDRESS:             return F("IP Address");
    case LabelType::IP_SUBNET:              return F("IP Subnet");
    case LabelType::IP_ADDRESS_SUBNET:      return F("IP / Subnet");
    case LabelType::GATEWAY:                return F("Gateway");
    case LabelType::CLIENT_IP:              return F("Client IP");
    #if FEATURE_MDNS
    case LabelType::M_DNS:                  return F("mDNS");
    #endif // if FEATURE_MDNS
    case LabelType::DNS:                    return F("DNS");
    case LabelType::DNS_1:                  return F("DNS 1");
    case LabelType::DNS_2:                  return F("DNS 2");
    case LabelType::ALLOWED_IP_RANGE:       return F("Allowed IP Range");
    case LabelType::STA_MAC:                return F("STA MAC");
    case LabelType::AP_MAC:                 return F("AP MAC");
    case LabelType::SSID:                   return F("SSID");
    case LabelType::BSSID:                  return F("BSSID");
    case LabelType::CHANNEL:                return F("Channel");
    case LabelType::ENCRYPTION_TYPE_STA:    return F("Encryption Type");
    case LabelType::CONNECTED:              return F("Connected");
    case LabelType::CONNECTED_MSEC:         return F("Connected msec");
    case LabelType::LAST_DISCONNECT_REASON: return F("Last Disconnect Reason");
    case LabelType::LAST_DISC_REASON_STR:   return F("Last Disconnect Reason str");
    case LabelType::NUMBER_RECONNECTS:      return F("Number Reconnects");
    case LabelType::WIFI_STORED_SSID1:      return F("Configured SSID1");
    case LabelType::WIFI_STORED_SSID2:      return F("Configured SSID2");


    case LabelType::FORCE_WIFI_BG:          return F("Force WiFi B/G");
    case LabelType::RESTART_WIFI_LOST_CONN: return F("Restart WiFi Lost Conn");
    case LabelType::FORCE_WIFI_NOSLEEP:     return F("Force WiFi No Sleep");
    case LabelType::PERIODICAL_GRAT_ARP:    return F("Periodical send Gratuitous ARP");
    case LabelType::CONNECTION_FAIL_THRESH: return F("Connection Failure Threshold");

    case LabelType::BUILD_DESC:             return F("Build");
    case LabelType::GIT_BUILD:              return F("Git Build");
    case LabelType::SYSTEM_LIBRARIES:       return F("System Libraries");
    case LabelType::PLUGIN_COUNT:           return F("Plugin Count");
    case LabelType::PLUGIN_DESCRIPTION:     return F("Plugin Description");
    case LabelType::BUILD_TIME:             return F("Build Time");
    case LabelType::BINARY_FILENAME:        return F("Binary Filename");
    case LabelType::BUILD_PLATFORM:         return F("Build Platform");
    case LabelType::GIT_HEAD:               return F("Git HEAD");

    case LabelType::I2C_BUS_STATE:          return F("I2C Bus State");
    case LabelType::I2C_BUS_CLEARED_COUNT:  return F("I2C bus cleared count");

    case LabelType::SYSLOG_LOG_LEVEL:       return F("Syslog Log Level");
    case LabelType::SERIAL_LOG_LEVEL:       return F("Serial Log Level");
    case LabelType::WEB_LOG_LEVEL:          return F("Web Log Level");
  #if FEATURE_SD
    case LabelType::SD_LOG_LEVEL:           return F("SD Log Level");
  #endif // if FEATURE_SD

    case LabelType::ESP_CHIP_ID:            return F("ESP Chip ID");
    case LabelType::ESP_CHIP_FREQ:          return F("ESP Chip Frequency");
    case LabelType::ESP_CHIP_MODEL:         return F("ESP Chip Model");
    case LabelType::ESP_CHIP_REVISION:      return F("ESP Chip Revision");
    case LabelType::ESP_CHIP_CORES:         return F("ESP Chip Cores");

    case LabelType::ESP_BOARD_NAME:         return F("ESP Board Name");

    case LabelType::FLASH_CHIP_ID:          return F("Flash Chip ID");
    case LabelType::FLASH_CHIP_VENDOR:      return F("Flash Chip Vendor");
    case LabelType::FLASH_CHIP_MODEL:       return F("Flash Chip Model");
    case LabelType::FLASH_CHIP_REAL_SIZE:   return F("Flash Chip Real Size");
    case LabelType::FLASH_CHIP_SPEED:       return F("Flash Chip Speed");
    case LabelType::FLASH_IDE_SIZE:         return F("Flash IDE Size");
    case LabelType::FLASH_IDE_SPEED:        return F("Flash IDE Speed");
    case LabelType::FLASH_IDE_MODE:         return F("Flash IDE Mode");
    case LabelType::FLASH_WRITE_COUNT:      return F("Flash Writes");
    case LabelType::SKETCH_SIZE:            return F("Sketch Size");
    case LabelType::SKETCH_FREE:            return F("Sketch Free");
    #ifdef USE_LITTLEFS
    case LabelType::FS_SIZE:                return F("Little FS Size");
    case LabelType::FS_FREE:                return F("Little FS Free");
    #else // ifdef USE_LITTLEFS
    case LabelType::FS_SIZE:                return F("SPIFFS Size");
    case LabelType::FS_FREE:                return F("SPIFFS Free");
    #endif // ifdef USE_LITTLEFS
    case LabelType::MAX_OTA_SKETCH_SIZE:    return F("Max. OTA Sketch Size");
    case LabelType::OTA_2STEP:              return F("OTA 2-step Needed");
    case LabelType::OTA_POSSIBLE:           return F("OTA possible");
#if FEATURE_ETHERNET
    case LabelType::ETH_IP_ADDRESS:         return F("Eth IP Address");
    case LabelType::ETH_IP_SUBNET:          return F("Eth IP Subnet");
    case LabelType::ETH_IP_ADDRESS_SUBNET:  return F("Eth IP / Subnet");
    case LabelType::ETH_IP_GATEWAY:         return F("Eth Gateway");
    case LabelType::ETH_IP_DNS:             return F("Eth DNS");
    case LabelType::ETH_MAC:                return F("Eth MAC");
    case LabelType::ETH_DUPLEX:             return F("Eth Mode");
    case LabelType::ETH_SPEED:              return F("Eth Speed");
    case LabelType::ETH_STATE:              return F("Eth State");
    case LabelType::ETH_SPEED_STATE:        return F("Eth Speed State");
    case LabelType::ETH_CONNECTED:          return F("Eth connected");
#endif // if FEATURE_ETHERNET
    case LabelType::ETH_WIFI_MODE:          return F("Network Type");
    case LabelType::SUNRISE:                return F("Sunrise");
    case LabelType::SUNSET:                 return F("Sunset");
    case LabelType::SUNRISE_S:              return F("Sunrise sec.");
    case LabelType::SUNSET_S:               return F("Sunset sec.");
    case LabelType::SUNRISE_M:              return F("Sunrise min.");
    case LabelType::SUNSET_M:               return F("Sunset min.");
    case LabelType::ISNTP:                  return F("Use NTP");
    case LabelType::UPTIME_MS:              return F("Uptime (ms)");
    case LabelType::TIMEZONE_OFFSET:        return F("Timezone Offset");
    case LabelType::LATITUDE:               return F("Latitude");
    case LabelType::LONGITUDE:              return F("Longitude");

    case LabelType::MAX_LABEL:
      break;

  }
  return F("MissingString");
}

String getValue(LabelType::Enum label) {
  switch (label)
  {
    case LabelType::UNIT_NR:                return String(Settings.Unit);
    case LabelType::UNIT_NAME:              return String(Settings.Name); // Only return the set name, no appended unit.
    case LabelType::HOST_NAME:              return NetworkGetHostname();


    case LabelType::LOCAL_TIME:             return node_time.getDateTimeString('-', ':', ' ');
    case LabelType::TIME_SOURCE:            return toString(node_time.timeSource);
    case LabelType::TIME_WANDER:            return String(node_time.timeWander, 3);
    case LabelType::UPTIME:                 return String(getUptimeMinutes());
    case LabelType::LOAD_PCT:               return toString(getCPUload(), 2);
    case LabelType::LOOP_COUNT:             return String(getLoopCountPerSec());
    case LabelType::CPU_ECO_MODE:           return jsonBool(Settings.EcoPowerMode());
#ifdef ESP8266 // TD-er: Disable setting TX power on ESP32 as it seems to cause issues on IDF4.4
    case LabelType::WIFI_TX_MAX_PWR:        return toString(Settings.getWiFi_TX_power(), 2);
    case LabelType::WIFI_CUR_TX_PWR:        return toString(WiFiEventData.wifi_TX_pwr, 2);
    case LabelType::WIFI_SENS_MARGIN:       return String(Settings.WiFi_sensitivity_margin);
    case LabelType::WIFI_SEND_AT_MAX_TX_PWR:return jsonBool(Settings.UseMaxTXpowerForSending());
#endif
    case LabelType::WIFI_NR_EXTRA_SCANS:    return String(Settings.NumberExtraWiFiScans);
    case LabelType::WIFI_USE_LAST_CONN_FROM_RTC: return jsonBool(Settings.UseLastWiFiFromRTC());

    case LabelType::FREE_MEM:               return String(FreeMem());
    case LabelType::FREE_STACK:             return String(getCurrentFreeStack());

#ifdef USE_SECOND_HEAP
    case LabelType::FREE_HEAP_IRAM:         return String(FreeMem2ndHeap());
#endif

#if defined(CORE_POST_2_5_0)
  #ifndef LIMIT_BUILD_SIZE
    case LabelType::HEAP_MAX_FREE_BLOCK:    return String(ESP.getMaxFreeBlockSize());
  #endif
#endif // if defined(CORE_POST_2_5_0)
#if  defined(ESP32)
  #ifndef LIMIT_BUILD_SIZE
    case LabelType::HEAP_MAX_FREE_BLOCK:    return String(ESP.getMaxAllocHeap());
  #endif
#endif // if  defined(ESP32)
#if defined(CORE_POST_2_5_0)
  #ifndef LIMIT_BUILD_SIZE
    case LabelType::HEAP_FRAGMENTATION:     return String(ESP.getHeapFragmentation());
  #endif
#endif // if defined(CORE_POST_2_5_0)
#ifdef ESP32
    case LabelType::HEAP_SIZE:              return String(ESP.getHeapSize());
    case LabelType::HEAP_MIN_FREE:          return String(ESP.getMinFreeHeap());
    #ifdef BOARD_HAS_PSRAM
    case LabelType::PSRAM_SIZE:             return String(UsePSRAM() ? ESP.getPsramSize() : 0);
    case LabelType::PSRAM_FREE:             return String(UsePSRAM() ? ESP.getFreePsram() : 0);
    case LabelType::PSRAM_MIN_FREE:         return String(UsePSRAM() ? ESP.getMinFreePsram() : 0);
    case LabelType::PSRAM_MAX_FREE_BLOCK:   return String(UsePSRAM() ? ESP.getMaxAllocPsram() : 0);
    #endif // BOARD_HAS_PSRAM
#endif // ifdef ESP32


    case LabelType::JSON_BOOL_QUOTES:           return jsonBool(Settings.JSONBoolWithoutQuotes());
    case LabelType::ENABLE_TIMING_STATISTICS:   return jsonBool(Settings.EnableTimingStats());
    case LabelType::ENABLE_RULES_CACHING:       return jsonBool(Settings.EnableRulesCaching());
//    case LabelType::ENABLE_RULES_EVENT_REORDER: return jsonBool(Settings.EnableRulesEventReorder()); // TD-er: Disabled for now
    case LabelType::TASKVALUESET_ALL_PLUGINS:   return jsonBool(Settings.AllowTaskValueSetAllPlugins());
    case LabelType::ALLOW_OTA_UNLIMITED:        return jsonBool(Settings.AllowOTAUnlimited());
    case LabelType::ENABLE_CLEAR_HUNG_I2C_BUS:  return jsonBool(Settings.EnableClearHangingI2Cbus());
#ifndef BUILD_NO_RAM_TRACKER
    case LabelType::ENABLE_RAM_TRACKING:        return jsonBool(Settings.EnableRAMTracking());
#endif


    case LabelType::BOOT_TYPE:              return getLastBootCauseString();
    case LabelType::BOOT_COUNT:             break;
    case LabelType::DEEP_SLEEP_ALTERNATIVE_CALL: return jsonBool(Settings.UseAlternativeDeepSleep());
    case LabelType::RESET_REASON:           return getResetReasonString();
    case LabelType::LAST_TASK_BEFORE_REBOOT: return ESPEasy_Scheduler::decodeSchedulerId(lastMixedSchedulerId_beforereboot);
    case LabelType::SW_WD_COUNT:            return String(sw_watchdog_callback_count);

    case LabelType::WIFI_CONNECTION:        break;
    case LabelType::WIFI_RSSI:              return String(WiFi.RSSI());
    case LabelType::IP_CONFIG:              return useStaticIP() ? getLabel(LabelType::IP_CONFIG_STATIC) : getLabel(
        LabelType::IP_CONFIG_DYNAMIC);
    case LabelType::IP_CONFIG_STATIC:       break;
    case LabelType::IP_CONFIG_DYNAMIC:      break;
    case LabelType::IP_ADDRESS:             return NetworkLocalIP().toString();
    case LabelType::IP_SUBNET:              return NetworkSubnetMask().toString();
    case LabelType::IP_ADDRESS_SUBNET:      return String(getValue(LabelType::IP_ADDRESS) + F(" / ") + getValue(LabelType::IP_SUBNET));
    case LabelType::GATEWAY:                return NetworkGatewayIP().toString();
    case LabelType::CLIENT_IP:              return formatIP(web_server.client().remoteIP());

    #if FEATURE_MDNS
    case LabelType::M_DNS:                  return String(NetworkGetHostname()) + F(".local");
    #endif // if FEATURE_MDNS
    case LabelType::DNS:                    return String(getValue(LabelType::DNS_1) + F(" / ") + getValue(LabelType::DNS_2));
    case LabelType::DNS_1:                  return NetworkDnsIP(0).toString();
    case LabelType::DNS_2:                  return NetworkDnsIP(1).toString();
    case LabelType::ALLOWED_IP_RANGE:       return describeAllowedIPrange();
    case LabelType::STA_MAC:                return WifiSTAmacAddress();
    case LabelType::AP_MAC:                 return WifiSoftAPmacAddress();
    case LabelType::SSID:                   return WiFi.SSID();
    case LabelType::BSSID:                  return WiFi.BSSIDstr();
    case LabelType::CHANNEL:                return String(WiFi.channel());
    case LabelType::ENCRYPTION_TYPE_STA:    return // WiFi_AP_Candidates.getCurrent().encryption_type();
                                                   WiFi_encryptionType(WiFiEventData.auth_mode);
    case LabelType::CONNECTED:
      #if FEATURE_ETHERNET
      if(active_network_medium == NetworkMedium_t::Ethernet) {
        return format_msec_duration(EthEventData.lastConnectMoment.millisPassedSince());
      }
      #endif // if FEATURE_ETHERNET
      return format_msec_duration(WiFiEventData.lastConnectMoment.millisPassedSince());

    // Use only the nr of seconds to fit it in an int32, plus append '000' to have msec format again.
    case LabelType::CONNECTED_MSEC:         
      #if FEATURE_ETHERNET
      if(active_network_medium == NetworkMedium_t::Ethernet) {
        return String(static_cast<int32_t>(EthEventData.lastConnectMoment.millisPassedSince() / 1000ll)) + F("000"); 
      }
      #endif // if FEATURE_ETHERNET
      return String(static_cast<int32_t>(WiFiEventData.lastConnectMoment.millisPassedSince() / 1000ll)) + F("000"); 
    case LabelType::LAST_DISCONNECT_REASON: return String(WiFiEventData.lastDisconnectReason);
    case LabelType::LAST_DISC_REASON_STR:   return getLastDisconnectReason();
    case LabelType::NUMBER_RECONNECTS:      return String(WiFiEventData.wifi_reconnects);
    case LabelType::WIFI_STORED_SSID1:      return String(SecuritySettings.WifiSSID);
    case LabelType::WIFI_STORED_SSID2:      return String(SecuritySettings.WifiSSID2);


    case LabelType::FORCE_WIFI_BG:          return jsonBool(Settings.ForceWiFi_bg_mode());
    case LabelType::RESTART_WIFI_LOST_CONN: return jsonBool(Settings.WiFiRestart_connection_lost());
    case LabelType::FORCE_WIFI_NOSLEEP:     return jsonBool(Settings.WifiNoneSleep());
    case LabelType::PERIODICAL_GRAT_ARP:    return jsonBool(Settings.gratuitousARP());
    case LabelType::CONNECTION_FAIL_THRESH: return String(Settings.ConnectionFailuresThreshold);

    case LabelType::BUILD_DESC:             return String(BUILD);
    case LabelType::GIT_BUILD:              
      { 
        const String res(F(BUILD_GIT));
        if (!res.isEmpty()) return res;
        return get_git_head();
      }
    case LabelType::SYSTEM_LIBRARIES:       return getSystemLibraryString();
    case LabelType::PLUGIN_COUNT:           return String(deviceCount + 1);
    case LabelType::PLUGIN_DESCRIPTION:     return getPluginDescriptionString();
    case LabelType::BUILD_TIME:             return String(get_build_date()) + ' ' + get_build_time();
    case LabelType::BINARY_FILENAME:        return get_binary_filename();
    case LabelType::BUILD_PLATFORM:         return get_build_platform();
    case LabelType::GIT_HEAD:               return get_git_head();
    case LabelType::I2C_BUS_STATE:          return toString(I2C_state);
    case LabelType::I2C_BUS_CLEARED_COUNT:  return String(I2C_bus_cleared_count);
    case LabelType::SYSLOG_LOG_LEVEL:       return getLogLevelDisplayString(Settings.SyslogLevel);
    case LabelType::SERIAL_LOG_LEVEL:       return getLogLevelDisplayString(getSerialLogLevel());
    case LabelType::WEB_LOG_LEVEL:          return getLogLevelDisplayString(getWebLogLevel());
  #if FEATURE_SD
    case LabelType::SD_LOG_LEVEL:           return getLogLevelDisplayString(Settings.SDLogLevel);
  #endif // if FEATURE_SD

    case LabelType::ESP_CHIP_ID:            return formatToHex(getChipId(), 6);
    case LabelType::ESP_CHIP_FREQ:          return String(ESP.getCpuFreqMHz());
    case LabelType::ESP_CHIP_MODEL:         return getChipModel();
    case LabelType::ESP_CHIP_REVISION:      return String(getChipRevision());
    case LabelType::ESP_CHIP_CORES:         return String(getChipCores());
    case LabelType::ESP_BOARD_NAME:         return get_board_name();
    case LabelType::FLASH_CHIP_ID:          return String(getFlashChipId());
    case LabelType::FLASH_CHIP_VENDOR:      return formatToHex(getFlashChipId() & 0xFF, 2);
    case LabelType::FLASH_CHIP_MODEL:
    {
      const uint32_t flashChipId = getFlashChipId();
      const uint32_t flashDevice = (flashChipId & 0xFF00) | ((flashChipId >> 16) & 0xFF);
      return formatToHex(flashDevice, 4);
    }
    case LabelType::FLASH_CHIP_REAL_SIZE:   return String(getFlashRealSizeInBytes());
    case LabelType::FLASH_CHIP_SPEED:       return String(getFlashChipSpeed() / 1000000);
    case LabelType::FLASH_IDE_SIZE:         break;
    case LabelType::FLASH_IDE_SPEED:        break;
    case LabelType::FLASH_IDE_MODE:         break;
    case LabelType::FLASH_WRITE_COUNT:      break;
    case LabelType::SKETCH_SIZE:            break;
    case LabelType::SKETCH_FREE:            break;
    case LabelType::FS_SIZE:                return String(SpiffsTotalBytes());
    case LabelType::FS_FREE:                return String(SpiffsFreeSpace());
    case LabelType::MAX_OTA_SKETCH_SIZE:    break;
    case LabelType::OTA_2STEP:              break;
    case LabelType::OTA_POSSIBLE:           break;
#if FEATURE_ETHERNET
    case LabelType::ETH_IP_ADDRESS:         return NetworkLocalIP().toString();
    case LabelType::ETH_IP_SUBNET:          return NetworkSubnetMask().toString();
    case LabelType::ETH_IP_ADDRESS_SUBNET:  return String(getValue(LabelType::ETH_IP_ADDRESS) + F(" / ") +
                                                          getValue(LabelType::ETH_IP_SUBNET));
    case LabelType::ETH_IP_GATEWAY:         return NetworkGatewayIP().toString();
    case LabelType::ETH_IP_DNS:             return NetworkDnsIP(0).toString();
    case LabelType::ETH_MAC:                return NetworkMacAddress().toString();
    case LabelType::ETH_DUPLEX:             return EthLinkUp() ? (EthFullDuplex() ? F("Full Duplex") : F("Half Duplex")) : F("Link Down");
    case LabelType::ETH_SPEED:              return EthLinkUp() ? getEthSpeed() : F("Link Down");
    case LabelType::ETH_STATE:              return EthLinkUp() ? F("Link Up") : F("Link Down");
    case LabelType::ETH_SPEED_STATE:        return EthLinkUp() ? getEthLinkSpeedState() : F("Link Down");
    case LabelType::ETH_CONNECTED:          return ETHConnected() ? F("CONNECTED") : F("DISCONNECTED"); // 0=disconnected, 1=connected
#endif // if FEATURE_ETHERNET
    case LabelType::ETH_WIFI_MODE:          return toString(active_network_medium);
    case LabelType::SUNRISE:                return node_time.getSunriseTimeString(':');
    case LabelType::SUNSET:                 return node_time.getSunsetTimeString(':');
    case LabelType::SUNRISE_S:              return String(node_time.sunRise.tm_hour * 3600 + node_time.sunRise.tm_min * 60 + node_time.sunRise.tm_sec);
    case LabelType::SUNSET_S:               return String(node_time.sunSet.tm_hour * 3600 + node_time.sunSet.tm_min * 60 + node_time.sunSet.tm_sec);
    case LabelType::SUNRISE_M:              return String(node_time.sunRise.tm_hour * 60 + node_time.sunRise.tm_min);
    case LabelType::SUNSET_M:               return String(node_time.sunSet.tm_hour * 60 + node_time.sunSet.tm_min);
    case LabelType::ISNTP:                  return jsonBool(Settings.UseNTP());
    case LabelType::UPTIME_MS:              return ull2String(getMicros64() / 1000);
    case LabelType::TIMEZONE_OFFSET:        return String(Settings.TimeZone);
    case LabelType::LATITUDE:               return String(Settings.Latitude);
    case LabelType::LONGITUDE:              return String(Settings.Longitude);

    case LabelType::MAX_LABEL:
      break;
  }
  return F("MissingString");
}

#if FEATURE_ETHERNET
String getEthSpeed() {
  String result;

  result.reserve(7);
  result += EthLinkSpeed();
  result += F("Mbps");
  return result;
}

String getEthLinkSpeedState() {
  String result;

  result.reserve(29);

  if (EthLinkUp()) {
    result += getValue(LabelType::ETH_STATE);
    result += ' ';
    result += getValue(LabelType::ETH_DUPLEX);
    result += ' ';
    result += getEthSpeed();
  } else {
    result = getValue(LabelType::ETH_STATE);
  }
  return result;
}

#endif // if FEATURE_ETHERNET

String getExtendedValue(LabelType::Enum label) {
  switch (label)
  {
    case LabelType::UPTIME:
    {
      String result;
      result.reserve(40);
      int minutes = getUptimeMinutes();
      int days    = minutes / 1440;
      minutes = minutes % 1440;
      int hrs = minutes / 60;
      minutes = minutes % 60;

      result += days;
      result += F(" days ");
      result += hrs;
      result += F(" hours ");
      result += minutes;
      result += F(" minutes");
      return result;
    }

    default:
      break;
  }
  return "";
}

#include "../Helpers/Network.h"

#include "../../ESPEasy_common.h"

#include "../ESPEasyCore/ESPEasyWifi.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"

#include "../Globals/Settings.h"
#include "../Globals/Services.h"

#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/Misc.h"

/********************************************************************************************\
   Status LED
 \*********************************************************************************************/
void statusLED(bool traffic)
{
  static int gnStatusValueCurrent = -1;
  static long int gnLastUpdate    = millis();

  if (!validGpio(Settings.Pin_status_led)) {
    return;
  }

  if (gnStatusValueCurrent < 0) {
#ifndef ESP32
    pinMode(Settings.Pin_status_led, OUTPUT);
#endif // ESP32
  }

  int nStatusValue = gnStatusValueCurrent;

  if (traffic)
  {
    nStatusValue += STATUS_PWM_TRAFFICRISE; // ramp up fast
  }
  else
  {
    if (NetworkConnected())
    {
      long int delta = timePassedSince(gnLastUpdate);

      if ((delta > 0) || (delta < 0))
      {
        nStatusValue -= STATUS_PWM_NORMALFADE; // ramp down slowly
        nStatusValue  = std::max(nStatusValue, STATUS_PWM_NORMALVALUE);
        gnLastUpdate  = millis();
      }
    }

    // AP mode is active
    else if (WifiIsAP(WiFi.getMode()))
    {
      nStatusValue = ((millis() >> 1) & PWMRANGE_FULL) - (PWMRANGE_FULL >> 2); // ramp up for 2 sec, 3/4 luminosity
    }

    // Disconnected
    else
    {
      nStatusValue = (millis() >> 1) & (PWMRANGE_FULL >> 2); // ramp up for 1/2 sec, 1/4 luminosity
    }
  }

  nStatusValue = constrain(nStatusValue, 0, PWMRANGE_FULL);

  if (gnStatusValueCurrent != nStatusValue)
  {
    gnStatusValueCurrent = nStatusValue;

    long pwm = nStatusValue * nStatusValue; // simple gamma correction
    pwm >>= 10;

    if (Settings.Pin_status_led_Inversed) {
      pwm = PWMRANGE_FULL - pwm;
    }

    set_Gpio_PWM(Settings.Pin_status_led, pwm, 1000);
  }
}
#include "../Helpers/Numerical.h"

#include "../Globals/Settings.h"

/********************************************************************************************\
   Check if string is valid float
 \*********************************************************************************************/
bool isValidFloat(float f) {
  if (isnan(f)) { return false; // ("isnan");
  }

  if (isinf(f)) { return false; // ("isinf");
  }
  return true;
}

bool validIntFromString(const String& tBuf, int& result) {
  NumericalType detectedType;
  const String  numerical = getNumerical(tBuf, NumericalType::Integer, detectedType);

  if ((detectedType == NumericalType::BinaryUint) ||
      (detectedType == NumericalType::HexadecimalUInt)) {
    unsigned int tmp;
    bool isvalid = validUIntFromString(numerical, tmp);

    // FIXME TD-er: What to do here if the uint value > max_int ?
    result = static_cast<int>(tmp);
    return isvalid;
  }
  const bool isvalid = numerical.length() > 0;

  if (isvalid) {
    result = strtol(numerical.c_str(), nullptr, DEC);
  }
  return isvalid;
}

bool validInt64FromString(const String& tBuf, int64_t& result) {
  NumericalType detectedType;
  const String  numerical = getNumerical(tBuf, NumericalType::Integer, detectedType);

  if ((detectedType == NumericalType::BinaryUint) ||
      (detectedType == NumericalType::HexadecimalUInt)) {
    uint64_t tmp;
    bool     isvalid = validUInt64FromString(numerical, tmp);
    result = static_cast<int64_t>(tmp);
    return isvalid;
  }

  const bool isvalid = numerical.length() > 0;

  if (isvalid) {
    result = strtoll(numerical.c_str(), nullptr, DEC);
  }
  return isvalid;
}

bool validUIntFromString(const String& tBuf, unsigned int& result) {
  NumericalType detectedType;
  String numerical   = getNumerical(tBuf, NumericalType::HexadecimalUInt, detectedType);
  const bool isvalid = numerical.length() > 0;

  if (isvalid) {
    int base = DEC;

    if (detectedType == NumericalType::HexadecimalUInt) {
      numerical = numerical.substring(2);
      base      = HEX;
    } else if (detectedType == NumericalType::BinaryUint) {
      numerical = numerical.substring(2);
      base      = BIN;
    }
    result = strtoul(numerical.c_str(), nullptr, base);
  }
  return isvalid;
}

bool validUInt64FromString(const String& tBuf, uint64_t& result) {
  NumericalType detectedType;
  String numerical   = getNumerical(tBuf, NumericalType::HexadecimalUInt, detectedType);
  const bool isvalid = numerical.length() > 0;

  if (isvalid) {
    int base = DEC;

    if (detectedType == NumericalType::HexadecimalUInt) {
      numerical = numerical.substring(2);
      base      = HEX;
    } else if (detectedType == NumericalType::BinaryUint) {
      numerical = numerical.substring(2);
      base      = BIN;
    }
    result = strtoull(numerical.c_str(), nullptr, base);
  }
  return isvalid;
}

bool validFloatFromString(const String& tBuf, float& result) {
  // DO not call validDoubleFromString and then cast to float.
  // Working with double values is quite CPU intensive as it must be done in software
  // since the ESP does not have large enough registers for handling double values in hardware.
  NumericalType detectedType;
  const String  numerical = getNumerical(tBuf, NumericalType::FloatingPoint, detectedType);

  if ((detectedType == NumericalType::BinaryUint) ||
      (detectedType == NumericalType::HexadecimalUInt)) {
    unsigned int tmp;
    bool isvalid = validUIntFromString(tBuf, tmp);
    result = static_cast<float>(tmp);
    return isvalid;
  }

  const bool isvalid = numerical.length() > 0;

  if (isvalid) {
    result = numerical.toFloat();
  }
  return isvalid;
}

bool validDoubleFromString(const String& tBuf, double& result) {
  #if defined(CORE_POST_2_5_0) || defined(ESP32)

  // String.toDouble() is introduced in core 2.5.0
  NumericalType detectedType;
  const String  numerical = getNumerical(tBuf, NumericalType::FloatingPoint, detectedType);

  if ((detectedType == NumericalType::BinaryUint) ||
      (detectedType == NumericalType::HexadecimalUInt)) {
    uint64_t tmp;
    bool     isvalid = validUInt64FromString(tBuf, tmp);
    result = static_cast<double>(tmp);
    return isvalid;
  }

  const bool isvalid = numerical.length() > 0;

  if (isvalid) {
    result = numerical.toDouble();
  }
  return isvalid;
  #else // if defined(CORE_POST_2_5_0) || defined(ESP32)
  float tmp = static_cast<float>(result);
  bool  res = validFloatFromString(tBuf, tmp);
  result = static_cast<double>(tmp);
  return res;
  #endif // if defined(CORE_POST_2_5_0) || defined(ESP32)
}

bool mustConsiderAsString(NumericalType detectedType) {
  switch (detectedType) {
    case NumericalType::FloatingPoint:
    case NumericalType::Integer:
      break;
    case NumericalType::HexadecimalUInt:
    case NumericalType::BinaryUint: // Has '0x' or '0b' as prefix
    case NumericalType::Not_a_number:
      return true;
  }
  return false;
}

bool mustConsiderAsJSONString(const String& value) {
  if (value.isEmpty()) {
    // Empty string
    return true;
  }

  NumericalType detectedType;
  const bool    isNum  = isNumerical(value, detectedType);
  const bool    isBool = (Settings.JSONBoolWithoutQuotes() && ((value.equalsIgnoreCase(F("true")) || value.equalsIgnoreCase(F("false")))));

  return !isBool && (!isNum || value.isEmpty() || mustConsiderAsString(detectedType));
}

String getNumerical(const String& tBuf, NumericalType requestedType, NumericalType& detectedType) {
  const unsigned int bufLength = tBuf.length();
  unsigned int firstDec        = 0;
  String result;

  // Strip leading spaces
  while (firstDec < bufLength && tBuf.charAt(firstDec) == ' ') {
    ++firstDec;
  }

  // Strip leading zeroes until next char is a '.' or one of the "0x" and "0b" prefixes
  // or the zero remains as last character of the string
  // See: https://github.com/letscontrolit/ESPEasy/issues/4134
  bool doneStrippingZeroes = false;

  while ((firstDec + 1) < bufLength && tBuf.charAt(firstDec) == '0' && !doneStrippingZeroes) {
    const char nextChar = tBuf.charAt(firstDec + 1);

    if ((nextChar != '.') && 
        (nextChar != '+') && (nextChar != '-') && // +/-
        (nextChar != 'x') && (nextChar != 'X') && // Matching "0x"
        (nextChar != 'b') && (nextChar != 'B'))   // Matching "0b"
    {
      ++firstDec;
    }
    else { 
      doneStrippingZeroes = true;
    }
  }

  if (firstDec >= bufLength) {
    detectedType = NumericalType::Not_a_number;
    return result;
  }
  bool decPt = false;

  detectedType = NumericalType::Integer;
  char c = tBuf.charAt(firstDec);

  if ((c == '+') || (c == '-')) {
    if ((requestedType != NumericalType::HexadecimalUInt) &&
        (requestedType != NumericalType::BinaryUint)) {
      if (c == '-') {
        result += c;
      }
      ++firstDec;

      if (firstDec < bufLength) {
        c = tBuf.charAt(firstDec);
      }
    }
  }

  if (c == '0') {
    ++firstDec;
    result += c;

    if (firstDec < bufLength) {
      c = tBuf.charAt(firstDec);

      if ((c == 'x') || (c == 'X')) {
        ++firstDec;
        result      += c;
        detectedType = NumericalType::HexadecimalUInt;
      } else if ((c == 'b') || (c == 'B')) {
        ++firstDec;
        result      += c;
        detectedType = NumericalType::BinaryUint;
      } else if (NumericalType::Integer == requestedType) {
        // Allow leading zeroes in Integer types (e.g. in time notation)
        while (c == '0' && firstDec < bufLength) {
          // N.B. intentional "reverse order" of reading char and ++firstDec
          c = tBuf.charAt(firstDec);
          ++firstDec;
        }
      } else if ((NumericalType::FloatingPoint == requestedType) && (c == '.')) {
        // Only floating point numbers should start with '0.'
        // All other combinations are not valid.
        ++firstDec;
        result      += c;
        decPt        = true;
        detectedType = NumericalType::FloatingPoint;
      } else {
        if (result == F("-")) {
          detectedType = NumericalType::Not_a_number;
          return emptyString;
        }
        return result;
      }
    }
  } else {
    // Does not start with a 0 and already tested for +/-
    // Only allowed to have a '.' or digits.
    if ((c != '.') && !isdigit(c)) {
      detectedType = NumericalType::Not_a_number;
      return result;
    }
  }

  bool done = false;

  result.reserve(bufLength - firstDec + result.length());

  for (unsigned int x = firstDec; !done && x < bufLength; ++x) {
    c = tBuf.charAt(x);

    if (c == '.') {
      if (NumericalType::FloatingPoint != requestedType) { done = true; }

      // Only one decimal point allowed
      if (decPt) { done = true; }
      else {
        decPt        = true;
        detectedType = NumericalType::FloatingPoint;
      }
    } else {
      switch (detectedType) {
        case NumericalType::FloatingPoint:
        case NumericalType::Integer:

          if (!isdigit(c)) {
            done = true;
          }
          break;
        case NumericalType::HexadecimalUInt:

          if (!isxdigit(c)) {
            done = true;
          }
          break;
        case NumericalType::BinaryUint:

          if ((c != '0') && (c != '1')) {
            done = true;
          }
          break;
        case NumericalType::Not_a_number:
          done = true;
          break;
      }
    }

    if (!done) {
      result += c;
    }
  }

  if (result == F("-")) {
    detectedType = NumericalType::Not_a_number;
    return emptyString;
  }
  return result;
}

bool isNumerical(const String& tBuf, NumericalType& detectedType) {
  NumericalType requestedType = NumericalType::FloatingPoint;
  const String  result        = getNumerical(tBuf, requestedType, detectedType);

  if (detectedType == NumericalType::Not_a_number) { return false; }

  if (result.length() > 0)
  {
    String tmp(tBuf);
    tmp.trim(); // remove leading and trailing spaces

    // Resulting size should be the same size as the given string.
    // Not sure if it is possible to have a longer result, but better be sure to also allow for larger resulting strings.
    // For example ".123" -> "0.123"
    return result.length() >= tmp.length();
  }


  return result.length() > 0;
}

#include "../Helpers/msecTimerHandlerStruct.h"

#include <Arduino.h>

#include "../Helpers/ESPEasy_time_calc.h"


#define MAX_SCHEDULER_WAIT_TIME 5 // Max delay used in the scheduler for passing idle time.

  msecTimerHandlerStruct::msecTimerHandlerStruct() : get_called(0), get_called_ret_id(0), max_queue_length(0),
    last_exec_time_usec(0), total_idle_time_usec(0),  idle_time_pct(0.0f), is_idle(false), eco_mode(true)
  {
    last_log_start_time = millis();
  }

  void msecTimerHandlerStruct::setEcoMode(bool enabled) {
    eco_mode = enabled;
  }

  void msecTimerHandlerStruct::registerAt(unsigned long id, unsigned long timer) {
    timer_id_couple item(id, timer);

    insert(item);
  }

  // Check if timeout has been reached and also return its set timer.
  // Return 0 if no item has reached timeout moment.
  unsigned long msecTimerHandlerStruct::getNextId(unsigned long& timer) {
    ++get_called;

    if (_timer_ids.empty()) {
      recordIdle();

      if (eco_mode) {
        delay(MAX_SCHEDULER_WAIT_TIME); // Nothing to do, try save some power.
      }
      return 0;
    }
    timer_id_couple item = _timer_ids.front();
    const long passed    = timePassedSince(item._timer);

    if (passed < 0) {
      // No timeOutReached
      recordIdle();

      if (eco_mode) {
        long waitTime = (-1 * passed) - 1; // will be non negative

        if (waitTime > MAX_SCHEDULER_WAIT_TIME) {
          waitTime = MAX_SCHEDULER_WAIT_TIME;
        } else if (waitTime < 0) {  //-V547
          // Should not happen, but just to be sure we will not wait forever.
          waitTime = 0;
        }
        delay(waitTime);
      }
      return 0;
    }
    recordRunning();
    unsigned long size = _timer_ids.size();

    if (size > max_queue_length) { max_queue_length = size; }
    _timer_ids.pop_front();
    timer = item._timer;
    ++get_called_ret_id;
    return item._id;
  }


  bool msecTimerHandlerStruct::getTimerForId(unsigned long id, unsigned long& timer) const {
    for (auto it = _timer_ids.begin(); it != _timer_ids.end(); ++it) {
      if (it->_id == id) {
        timer = it->_timer;
        return true;
      }
    }
    return false;
  }

  String msecTimerHandlerStruct::getQueueStats() {
    String result;

    result           += get_called;
    result           += '/';
    result           += get_called_ret_id;
    result           += '/';
    result           += max_queue_length;
    result           += '/';
    result           += idle_time_pct;
    get_called        = 0;
    get_called_ret_id = 0;

    // max_queue_length = 0;
    return result;
  }

  void msecTimerHandlerStruct::updateIdleTimeStats() {
    const long duration = timePassedSince(last_log_start_time);

    last_log_start_time  = millis();
    idle_time_pct        = static_cast<float>(total_idle_time_usec) / duration / 10.0f;
    total_idle_time_usec = 0;
  }

  float msecTimerHandlerStruct::getIdleTimePct() const {
    return idle_time_pct;
  }

  struct match_id {
    match_id(unsigned long id) : _id(id) {}

    bool operator()(const timer_id_couple& item) {
      return _id == item._id;
    }

    unsigned long _id;
  };

  void msecTimerHandlerStruct::insert(const timer_id_couple& item) {
    if (item._id == 0) { return; }

    // Make sure only one is present with the same id.
    _timer_ids.remove_if(match_id(item._id));
    const bool mustSort = !_timer_ids.empty();
    _timer_ids.push_front(item);

    if (mustSort) {
      _timer_ids.sort(); // TD-er: Must check if this is an expensive operation.
    }

    // It should be a relative light operation, to insert into a sorted list.
    // Perhaps it is better to use std::set ????
    // Keep in mind: order is based on timer, uniqueness is based on id.
  }

  void msecTimerHandlerStruct::recordIdle() {
    if (is_idle) { return; }
    last_exec_time_usec = getMicros64();
    is_idle             = true;
    delay(0); // Nothing to do, so leave time for backgroundtasks
  }

  void msecTimerHandlerStruct::recordRunning() {
    if (!is_idle) { return; }
    is_idle               = false;
    total_idle_time_usec += usecPassedSince(last_exec_time_usec);
  }

#include "../Helpers/_Plugin_Helper_serial.h"


#include "../../_Plugin_Helper.h"

#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../Globals/Cache.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringGenerator_GPIO.h"

#include "../WebServer/HTML_wrappers.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Forms.h"

#include <ESPEasySerialType.h>


String serialHelper_getSerialTypeLabel(ESPEasySerialPort serType) {
  return ESPEasySerialPort_toString(serType);
}

void serialHelper_log_GpioDescription(ESPEasySerialPort typeHint, int config_pin1, int config_pin2) {
  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("Serial : ");
    log += serialHelper_getGpioDescription(typeHint, config_pin1, config_pin2, " ");
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
}

String serialHelper_getGpioDescription(ESPEasySerialPort typeHint, int config_pin1, int config_pin2, const String& newline) {
  String result;

  result.reserve(20);

  switch (ESPeasySerialType::getSerialType(typeHint, config_pin1, config_pin2)) {
    case ESPEasySerialPort::sc16is752:
    {
      result += formatToHex(config_pin1);
      result += newline;
      result += F(" ch: ");
      result += config_pin2 == 0 ? F("A") : F("B");
      return result;
    }
    case ESPEasySerialPort::software:
    case ESPEasySerialPort::serial0_swap:
    case ESPEasySerialPort::serial0:
    case ESPEasySerialPort::serial1:
    case ESPEasySerialPort::serial2:
    {
      result += F("RX: ");
      result += formatGpioLabel(config_pin1, false);
      result += newline;
      result += F("TX: ");
      result += formatGpioLabel(config_pin2, false);
      break;
    }
    default:
      break;
  }
  return result;
}

void serialHelper_getGpioNames(struct EventStruct *event, bool rxOptional, bool txOptional) {
  event->String1 = formatGpioName_RX(rxOptional);
  event->String2 = formatGpioName_TX(txOptional);
}

int8_t serialHelper_getRxPin(struct EventStruct *event) {
  return CONFIG_PIN1;
}

int8_t serialHelper_getTxPin(struct EventStruct *event) {
  return CONFIG_PIN2;
}

ESPEasySerialPort serialHelper_getSerialType(struct EventStruct *event) {
  ESPEasySerialPort serialType = static_cast<ESPEasySerialPort>(CONFIG_PORT);

  if (serialType != ESPEasySerialPort::not_set) {
    return serialType;
  }

  return ESPeasySerialType::getSerialType(
    serialType,
    serialHelper_getRxPin(event),
    serialHelper_getTxPin(event));
}

String serialHelper_getSerialTypeLabel(struct EventStruct *event) {
  return serialHelper_getSerialTypeLabel(serialHelper_getSerialType(event));
}

#ifndef DISABLE_SC16IS752_Serial
void serialHelper_addI2CuartSelectors(int address, int channel) {
  #define     SC16IS752_I2C_ADDRESSES             16
  #define     SC16IS752_I2C_BASE_ADDR             (0x90 >> 1)
  #define     SC16IS752_CHANNELS                  2
  #define     SC16IS752_CHANNEL_A                 0x00
  #define     SC16IS752_CHANNEL_B                 0x01
  {
    String id = F("i2cuart_addr");
    addRowLabel_tr_id(F("I2C Address"), id);
    do_addSelector_Head(id, EMPTY_STRING, EMPTY_STRING, false);

    if ((address < SC16IS752_I2C_BASE_ADDR) || (address >= (SC16IS752_I2C_BASE_ADDR + SC16IS752_I2C_ADDRESSES))) {
      // selected address is not in range
      address = SC16IS752_I2C_BASE_ADDR;
    }

    for (int i = 0; i < SC16IS752_I2C_ADDRESSES; i++)
    {
      int addr = SC16IS752_I2C_BASE_ADDR + i;
      String option;
      option.reserve(24);
      option  = formatToHex(addr);
      option += F(" (datasheet: ");
      option += formatToHex(addr * 2);
      option += ')';
      addSelector_Item(option, addr, addr == address);
    }
    addSelector_Foot();
  }
  {
    if ((channel != SC16IS752_CHANNEL_A) && (channel != SC16IS752_CHANNEL_B)) {
      channel = SC16IS752_CHANNEL_A;
    }
    const __FlashStringHelper * chOptions[SC16IS752_CHANNELS];
    int    chValues[SC16IS752_CHANNELS];
    chValues[0]  = SC16IS752_CHANNEL_A;
    chValues[1]  = SC16IS752_CHANNEL_B;
    chOptions[0] = F("A");
    chOptions[1] = F("B");
    addFormSelector(F("Channel"), F("i2cuart_ch"), SC16IS752_CHANNELS, chOptions, chValues, channel);
  }
}
#endif

void serialHelper_webformLoad(struct EventStruct *event) {
  serialHelper_webformLoad(event, true);
}

// These helper functions were made to create a generic interface to setup serial port config.
// See issue #2343 and Pull request https://github.com/letscontrolit/ESPEasy/pull/2352
// For now P020 and P044 have been reverted to make them work again.
void serialHelper_webformLoad(struct EventStruct *event, bool allowSoftwareSerial) {
  serialHelper_webformLoad(static_cast<ESPEasySerialPort>(CONFIG_PORT),
                           serialHelper_getRxPin(event),
                           serialHelper_getTxPin(event),
                           allowSoftwareSerial);
}

void serialHelper_webformLoad(ESPEasySerialPort port, int rxPinDef, int txPinDef, bool allowSoftwareSerial) {
  // Field for I2C addr & RX are shared
  // Field for channel and TX are shared
  #ifdef ESP8266

  // Script to show GPIO pins for SoftwareSerial or I2C addresses for the I2C to UART bridge
  html_add_script(F(
                    "function serialPortChanged(elem) {var style = elem.value == 6 ? '' : 'none';var i2cstyle = elem.value == 1 ? '' : 'none';document.getElementById('tr_taskdevicepin1').style.display = style;document.getElementById('tr_taskdevicepin2').style.display = style;document.getElementById('tr_i2cuart_addr').style.display = i2cstyle;document.getElementById('tr_i2cuart_ch').style.display = i2cstyle;}"),
                  false);
  #endif // ifdef ESP8266
  #ifdef ESP32

  // Script to show GPIO pins for HW serial ports or I2C addresses for the I2C to UART bridge
  html_add_script(F(
                    "function serialPortChanged(elem) {var style = (elem.value == 2 || elem.value == 4 || elem.value == 5) ? '' : 'none';var i2cstyle = elem.value == 1 ? '' : 'none';document.getElementById('tr_taskdevicepin1').style.display = style;document.getElementById('tr_taskdevicepin2').style.display = style;document.getElementById('tr_i2cuart_addr').style.display = i2cstyle;document.getElementById('tr_i2cuart_ch').style.display = i2cstyle;}"),
                  false);
  #endif // ifdef ESP32

  String options[NR_ESPEASY_SERIAL_TYPES];
  int    ids[NR_ESPEASY_SERIAL_TYPES];
  String attr[NR_ESPEASY_SERIAL_TYPES];

  int index = 0;

  for (int i = 0; (index < NR_ESPEASY_SERIAL_TYPES) && (i < static_cast<int>(ESPEasySerialPort::MAX_SERIAL_TYPE)); ++i) {
    int rxPin, txPin;
    ESPEasySerialPort serType = static_cast<ESPEasySerialPort>(i);

    if (ESPeasySerialType::getSerialTypePins(serType, rxPin, txPin)) {
      String option;
      option.reserve(48);
      option = serialHelper_getSerialTypeLabel(serType);

      switch (serType) {
        case ESPEasySerialPort::software:
        {
          if (!allowSoftwareSerial) {
            attr[index] = F("disabled");
          }
          break;
        }
        case ESPEasySerialPort::sc16is752:
        {
          break;
        }
        case ESPEasySerialPort::serial0:
        case ESPEasySerialPort::serial0_swap:
        case ESPEasySerialPort::serial1:
        case ESPEasySerialPort::serial2:
        {
          #ifdef ESP8266

          // Show pins for ports with fixed pins
          option += F(": ");
          option += formatGpioLabel(rxPin, false);
          option += ' ';
          option += formatGpioDirection(gpio_input);
          option += F("TX / ");
          option += formatGpioLabel(txPin, false);
          option += ' ';
          option += formatGpioDirection(gpio_output);
          option += F("RX");
          #endif // ifdef ESP8266
          break;
        }

        default:
          break;
      }
      options[index] = option;
      ids[index]     = i;
      ++index;
    }
  }
  addFormSelector_script(F("Serial Port"), F("serPort"), NR_ESPEASY_SERIAL_TYPES,
                         options, ids, nullptr,
                         static_cast<int>(ESPeasySerialType::getSerialType(port, rxPinDef, txPinDef)),
                         F("serialPortChanged(this)")); // Script to toggle GPIO visibility when changing selection.
#ifndef DISABLE_SC16IS752_Serial
  serialHelper_addI2CuartSelectors(rxPinDef, txPinDef);
#endif

#ifdef ESP8266
  if ((rxPinDef == 15) || (txPinDef == 15)) {
    addFormNote(F("GPIO-15 (D8) requires a Buffer Circuit (PNP transistor) or ESP boot may fail."));
  }
#endif
}

void serialHelper_webformSave(uint8_t& port, int8_t& rxPin, int8_t& txPin) {
  int serialPortSelected = getFormItemInt(F("serPort"), -1);

  if (serialPortSelected < 0) { return; }

  ESPEasySerialPort serType = static_cast<ESPEasySerialPort>(serialPortSelected);

  port = serialPortSelected;

  switch (serType) {
    case ESPEasySerialPort::software:
      break;
    #ifndef DISABLE_SC16IS752_Serial
    case ESPEasySerialPort::sc16is752:
      rxPin = getFormItemInt(F("i2cuart_addr"), rxPin);
      txPin = getFormItemInt(F("i2cuart_ch"), txPin);
      break;
    #endif
    case ESPEasySerialPort::serial0:
    case ESPEasySerialPort::serial0_swap:
    case ESPEasySerialPort::serial1:
    case ESPEasySerialPort::serial2:
    {
      #ifdef ESP8266

      // Ports with a fixed pin layout, so load the defaults.
      int tmprxPin, tmptxPin;

      if (ESPeasySerialType::getSerialTypePins(serType, tmprxPin, tmptxPin)) {
        rxPin = tmprxPin;
        txPin = tmptxPin;
      }
      #endif // ifdef ESP8266
      break;
    }
    default:
      break;
  }
}

void serialHelper_webformSave(struct EventStruct *event) {
  serialHelper_webformSave(CONFIG_PORT, CONFIG_PIN1, CONFIG_PIN2);
}

bool serialHelper_isValid_serialconfig(uint8_t serialconfig) {
  if ((serialconfig >= 0x10) && (serialconfig <= 0x3f)) {
    return true;
  }
  return false;
}

void serialHelper_serialconfig_webformLoad(struct EventStruct *event, uint8_t currentSelection) {
  // nrOptions = 4 * 3 * 2  = 24  (bits 5..8 , parity N/E/O  , stopbits 1/2)
  String id = F("serConf");

  addRowLabel_tr_id(F("Serial Config"), id);
  do_addSelector_Head(id, EMPTY_STRING, EMPTY_STRING, false);

  if (currentSelection == 0) {
    // Must truncate it to 1 uint8_t, since ESP32 uses a 32-bit value. We add these high bits later for ESP32.
    currentSelection = static_cast<uint8_t>(SERIAL_8N1 & 0xFF); // Some default
  }

  for (uint8_t parity = 0; parity < 3; ++parity) {
    for (uint8_t stopBits = 1; stopBits <= 2; ++stopBits) {
      for (uint8_t bits = 5; bits <= 8; ++bits) {
        String label;
        label.reserve(36);
        label  = String(bits);
        label += F(" bit / parity: ");
        int value = ((bits - 5) << 2);

        switch (parity) {
          case 0: label += F("None"); break;
          case 1: label += F("Even"); value += 2; break;
          case 2: label += F("Odd");  value += 3; break;
        }
        label += F(" / stop bits: ");
        label += String(stopBits);

        // There are also values for 0 and "1.5" stop bit, not used now.
        switch (stopBits) {
          case 1:  value += 0x10; break;
          case 2:  value += 0x30; break;
        }
        addSelector_Item(label, value, value == currentSelection);
      }
    }
  }
  addSelector_Foot();
}

uint8_t serialHelper_serialconfig_webformSave() {
  int serialConfSelected = getFormItemInt(F("serConf"), 0);

  if (serialHelper_isValid_serialconfig(serialConfSelected)) {
    return serialConfSelected;
  }

  // Must truncate it to 1 uint8_t, since ESP32 uses a 32-bit value. We add these high bits later for ESP32.
  return static_cast<uint8_t>(SERIAL_8N1 & 0xFF); // Some default
}

// Used by some plugins, which used several TaskDevicePluginConfigLong
uint8_t serialHelper_convertOldSerialConfig(uint8_t newLocationConfig) {
  if (serialHelper_isValid_serialconfig(newLocationConfig)) {
    return newLocationConfig;
  }
  uint8_t serialconfig = 0x10;                                                   // Default stopbits = 1

  serialconfig += ExtraTaskSettings.TaskDevicePluginConfigLong[3];            // Parity
  serialconfig += (ExtraTaskSettings.TaskDevicePluginConfigLong[2] - 5) << 2; // databits

  if (ExtraTaskSettings.TaskDevicePluginConfigLong[4] == 2) {
    serialconfig += 0x20;                                                     // Stopbits = 2
  }

  if (serialHelper_isValid_serialconfig(serialconfig)) {
    return serialconfig;
  }

  // Must truncate it to 1 uint8_t, since ESP32 uses a 32-bit value. We add these high bits later for ESP32.
  return static_cast<uint8_t>(SERIAL_8N1 & 0xFF); // Some default
}

#include "../Helpers/ESPEasy_checks.h"


#include "../../ESPEasy_common.h"

#include "../DataStructs/CRCStruct.h"
#include "../DataStructs/ControllerSettingsStruct.h"
#include "../DataStructs/DeviceStruct.h"
#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../DataStructs/ExtraTaskSettingsStruct.h"
#include "../DataStructs/FactoryDefaultPref.h"
#include "../DataStructs/GpioFactorySettingsStruct.h"
#include "../DataStructs/LogStruct.h"
#include "../DataStructs/NodeStruct.h"
#include "../DataStructs/PortStatusStruct.h"
#include "../DataStructs/ProtocolStruct.h"
#if FEATURE_CUSTOM_PROVISIONING
#include "../DataStructs/ProvisioningStruct.h"
#endif
#include "../DataStructs/RTCStruct.h"
#include "../DataStructs/SecurityStruct.h"
#include "../DataStructs/SettingsStruct.h"
#include "../DataStructs/SystemTimerStruct.h"

#include "../Globals/ExtraTaskSettings.h"
#include "../Globals/Settings.h"

#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/StringConverter.h"

#include <cstddef>

#ifdef USES_C013
#include "../DataStructs/C013_p2p_dataStructs.h"
#endif

#ifdef USES_C016
#include "../ControllerQueue/C016_queue_element.h"
#endif

#if FEATURE_NOTIFIER
#include "../DataStructs/NotificationStruct.h"
#include "../DataStructs/NotificationSettingsStruct.h"
#endif // if FEATURE_NOTIFIER


// ********************************************************************************
// Check struct sizes at compile time
// Usage:
//   struct foo
//   {
//     char bla[16];
//   };
//
//   check_size<foo, 8>();
// ********************************************************************************
template <typename ToCheck, std::size_t ExpectedSize, std::size_t RealSize = sizeof(ToCheck)>
void check_size() {
  static_assert(ExpectedSize == RealSize, "");
}



// ********************************************************************************
// Check struct sizes at compile time
// Usage:
//   struct X { int a, b, c, d; }
//   static_assert(ExpectedSize == offsetof(X, c), "");
// ********************************************************************************

void run_compiletime_checks() {
  #ifndef LIMIT_BUILD_SIZE
  check_size<CRCStruct,                             204u>();
  check_size<SecurityStruct,                        593u>();
  #ifdef ESP32
  const unsigned int SettingsStructSize = (316 + 84 * TASKS_MAX);
  #endif
  #ifdef ESP8266
  const unsigned int SettingsStructSize = (292 + 84 * TASKS_MAX);
  #endif
  #if FEATURE_CUSTOM_PROVISIONING
  check_size<ProvisioningStruct,                    256u>();  
  #endif
  check_size<SettingsStruct,                        SettingsStructSize>();
  check_size<ControllerSettingsStruct,              820u>();
  #if FEATURE_NOTIFIER
  check_size<NotificationSettingsStruct,            996u>();
  #endif // if FEATURE_NOTIFIER
  check_size<ExtraTaskSettingsStruct,               536u>();
  #if ESP_IDF_VERSION_MAJOR > 3
  // String class has increased with 4 bytes
  check_size<EventStruct,                           116u>(); // Is not stored
  #else
  check_size<EventStruct,                           96u>(); // Is not stored
  #endif


  // LogStruct is mainly dependent on the number of lines.
  // Has to be round up to multiple of 4.
  #if ESP_IDF_VERSION_MAJOR > 3
  // String class has increased with 4 bytes
  const unsigned int LogStructSize = ((13u + 21 * LOG_STRUCT_MESSAGE_LINES) + 3) & ~3;
  #else
  const unsigned int LogStructSize = ((13u + 17 * LOG_STRUCT_MESSAGE_LINES) + 3) & ~3;
  #endif
  check_size<LogStruct,                             LogStructSize>(); // Is not stored
  check_size<DeviceStruct,                          8u>(); // Is not stored
  check_size<ProtocolStruct,                        6u>();
  #if FEATURE_NOTIFIER
  check_size<NotificationStruct,                    3u>();
  #endif // if FEATURE_NOTIFIER
  #if FEATURE_ESPEASY_P2P
  #if ESP_IDF_VERSION_MAJOR > 3
  // String class has increased with 4 bytes
  check_size<NodeStruct,                            32u>();
  #else
  check_size<NodeStruct,                            28u>();
  #endif
  #endif
  check_size<systemTimerStruct,                     24u>();
  check_size<RTCStruct,                             32u>();
  check_size<portStatusStruct,                      6u>();
  check_size<ResetFactoryDefaultPreference_struct,  4u>();
  check_size<GpioFactorySettingsStruct,             18u>();
  #ifdef USES_C013
  check_size<C013_SensorInfoStruct,                 137u>();
  check_size<C013_SensorDataStruct,                 24u>();
  #endif
  #ifdef USES_C016
  check_size<C016_queue_element,                    24u>();
  #endif


  #if FEATURE_NON_STANDARD_24_TASKS && defined(ESP8266)
    static_assert(TASKS_MAX == 24, "TASKS_MAX invalid size");
  #endif

  // Check for alignment issues at compile time
  {
    const unsigned int ControllerUser_offset = 256u;
    static_assert(ControllerUser_offset == offsetof(SecurityStruct, ControllerUser), "");

    const unsigned int ControllerPassword_offset = 256u + (CONTROLLER_MAX * 26);
    static_assert(ControllerPassword_offset == offsetof(SecurityStruct, ControllerPassword), "");

    const unsigned int Password_offset = ControllerPassword_offset + (CONTROLLER_MAX * 64);
    static_assert(Password_offset == offsetof(SecurityStruct, Password), "");

    const unsigned int AllowedIPrangeLow_offset = Password_offset + 26;
    static_assert(AllowedIPrangeLow_offset == offsetof(SecurityStruct, AllowedIPrangeLow), "");

    const unsigned int IPblockLevel_offset = AllowedIPrangeLow_offset + 8;
    static_assert(IPblockLevel_offset == offsetof(SecurityStruct, IPblockLevel), "");

    const unsigned int ProgmemMd5_offset = IPblockLevel_offset + 1;
    static_assert(ProgmemMd5_offset == offsetof(SecurityStruct, ProgmemMd5), "");

    const unsigned int md5_offset = ProgmemMd5_offset + 16;
    static_assert(md5_offset == offsetof(SecurityStruct, md5), "");
  }


  static_assert(192u == offsetof(SettingsStruct, Protocol), "");
  static_assert(195u == offsetof(SettingsStruct, Notification), "CONTROLLER_MAX has changed?");
  static_assert(198u == offsetof(SettingsStruct, TaskDeviceNumber), "NOTIFICATION_MAX has changed?");

  // All settings related to N_TASKS
  static_assert((200 + TASKS_MAX) == offsetof(SettingsStruct, OLD_TaskDeviceID), ""); // 32-bit alignment, so offset of 2 bytes.
  static_assert((200 + (67 * TASKS_MAX)) == offsetof(SettingsStruct, ControllerEnabled), ""); 

  // Used to compute true offset.
  //const size_t offset = offsetof(SettingsStruct, ControllerEnabled);
  //check_size<SettingsStruct, offset>();

  #endif
}

#ifndef LIMIT_BUILD_SIZE
String ReportOffsetErrorInStruct(const String& structname, size_t offset) {
  String error;
  if (error.reserve(48 + structname.length())) {
    error  = F("Error: Incorrect offset in struct: ");
    error += structname;
    error += wrap_braces(String(offset));
  }
  return error;
}
#endif

/*********************************************************************************************\
*  Analyze SettingsStruct and report inconsistencies
*  Not a member function to be able to use the F-macro
\*********************************************************************************************/
bool SettingsCheck(String& error) {
  error = String();
  #ifndef LIMIT_BUILD_SIZE
#ifdef esp8266
  size_t offset = offsetof(SettingsStruct, ResetFactoryDefaultPreference);

  if (offset != 1224) {
    error = ReportOffsetErrorInStruct(F("SettingsStruct"), offset);
  }
#endif // ifdef esp8266

  if (!Settings.networkSettingsEmpty()) {
    if ((Settings.IP[0] == 0) || (Settings.Gateway[0] == 0) || (Settings.Subnet[0] == 0) || (Settings.DNS[0] == 0)) {
      error += F("Error: Either fill all IP settings fields or leave all empty");
    }
  }

  #endif

  return error.isEmpty();
}

#include "../Helpers/Numerical.h"

String checkTaskSettings(taskIndex_t taskIndex) {
  String err = LoadTaskSettings(taskIndex);
  #ifndef LIMIT_BUILD_SIZE
  if (err.length() > 0) return err;
  if (!ExtraTaskSettings.checkUniqueValueNames()) {
    return F("Use unique value names");
  }
  if (!ExtraTaskSettings.checkInvalidCharInNames()) {
    return F("Invalid character in name. Do not use ',-+/*=^%!#[]{}()' or space.");
  }
  String deviceName = ExtraTaskSettings.TaskDeviceName;
  NumericalType detectedType;
  if (isNumerical(deviceName, detectedType)) {
    return F("Invalid name. Should not be numeric.");
  }
  if (deviceName.isEmpty()) {
    if (Settings.TaskDeviceEnabled[taskIndex]) {
      // Decide what to do here, for now give a warning when task is enabled.
      return F("Warning: Task Device Name is empty. It is adviced to give tasks an unique name");
    }
  }
  // Do not use the cached function findTaskIndexByName since that one does rely on the fact names should be unique.
  for (taskIndex_t i = 0; i < TASKS_MAX; ++i) {
    if (i != taskIndex && Settings.TaskDeviceEnabled[i]) {
      LoadTaskSettings(i);
      if (ExtraTaskSettings.TaskDeviceName[0] != 0) {
        if (strcasecmp(ExtraTaskSettings.TaskDeviceName, deviceName.c_str()) == 0) {
          err = F("Task Device Name is not unique, conflicts with task ID #");
          err += (i+1);
//          return err;
        }
      }
    }
  }

  err += LoadTaskSettings(taskIndex);
  #endif
  return err;
}
#include "../Helpers/LongTermTimer.h"

LongTermTimer::LongTermTimer(const LongTermTimer& rhs) {
    _timer_usec = rhs.get();
}

LongTermTimer::LongTermTimer(bool usenow) : _timer_usec(0ull) {
    if (usenow) setNow();
}


  LongTermTimer& LongTermTimer::operator=(const LongTermTimer& rhs)
  {
    _timer_usec = rhs.get();
    return *this;
  }

  void LongTermTimer::setMillisFromNow(uint32_t millisFromNow) {
    _timer_usec = getMicros64() + (millisFromNow * 1000ull);
  }

  void LongTermTimer::setNow() {
    _timer_usec = getMicros64();
  }

  LongTermTimer::Duration LongTermTimer::millisPassedSince() const {
    return usecPassedSince() / 1000ll;
  }

  bool LongTermTimer::timeoutReached(uint32_t millisTimeout) const {
    return getMicros64() > (_timer_usec + (millisTimeout * 1000ull));
  }

#include "../Helpers/ESPEasyRTC.h"

#include "../Globals/RTC.h"
#include "../DataStructs/RTCStruct.h"
#include "../DataStructs/RTCCacheStruct.h"
#include "../DataStructs/RTC_cache_handler_struct.h"
#include "../DataStructs/TimingStats.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/Plugins.h"
#include "../Globals/RuntimeData.h"
#include "../Globals/Settings.h"
#include "../Helpers/CRC_functions.h"
#include "../../ESPEasy_common.h"

#ifdef ESP8266
#include <user_interface.h>
#endif

#ifdef ESP32
#include <soc/rtc.h>
#endif

/*********************************************************************************************\
* RTC memory stored values
\*********************************************************************************************/

// ESP8266 RTC layout:
/*
   During deep sleep, only RTC still working, so maybe we need to save some user data in RTC memory.
   Only user data area can be used by user.
 |<--------system data--------->|<-----------------user data--------------->|
 | 256 bytes                    | 512 bytes                                 |
   Note:
   RTC memory is 4 bytes aligned for read and write operations.
   Address parameter refers to block number(4 bytes per block).
   So, if we want to access some data at the beginning of user data area,
   address: 256/4 = 64
   data   : data pointer
   size   : data length, uint8_t

   Prototype:
    bool system_rtc_mem_read (
      uint32 src_addr,
      void * data,
      uint32 save_size
    )

    bool system_rtc_mem_write (
      uint32 des_addr,
      void * data,
      uint32 save_size
    )
 */

// RTC layout ESPeasy:
// these offsets are in blocks, bytes = blocks * 4
// 64   RTCStruct  max 40 bytes: ( 74 - 64 ) * 4
// 74   UserVar
// 122  UserVar checksum:  RTC_BASE_USERVAR + UserVar.getNrElements()
// 128  Cache (C016) metadata  4 blocks
// 132  Cache (C016) data  6 blocks per sample => max 10 samples



// ESP32 has 2 types of RTC memory:
// RTC SLOW:
//   - 8 kB which can be accessed by both CPU cores and ULP core
//   - Persistent even after reset (not power loss)
//   - Needs RTC_NOINIT_ATTR attribute
// RTC FAST:
//   - 8 kB, only accessed by the "PRO_CPU" 
//   - Persistent during sleep, but not after reset
//   - Needs RTC_DATA_ATTR attribute

// Important to realize:
// Since allocation to RTC memory on ESP32 is done by the compiler, there is no
// guarantee the addresses will be the same after a recompile.
// Thus the data stored in RTC may not survive flashing a new build.

// Structs stored in RTC SLOW:
//   - RTCStruct to keep information on reboot reason, last used WiFi, etc.
//   - UserVar   to keep task values persistent just like on ESP8266





//#define RTC_STRUCT_DEBUG



#ifdef ESP32
constexpr size_t UserVar_nrelements = VARS_PER_TASK * TASKS_MAX;


// Since the global UserVar and RTC objects are defined "extern", they cannot be located in the RTC memory.
// Thus we have to keep a copy here.
RTC_NOINIT_ATTR RTCStruct RTC_tmp;
RTC_NOINIT_ATTR float UserVar_RTC[UserVar_nrelements];
RTC_NOINIT_ATTR uint32_t UserVar_checksum;
#endif


/********************************************************************************************\
   Save RTC struct to RTC memory
 \*********************************************************************************************/
bool saveToRTC()
{
  // ESP8266 has the RTC struct stored in memory which we must actively fetch
  // ESP32 can use a compiler flag to mark a struct to be located in RTC_SLOW memory
  #if defined(ESP32)
  RTC_tmp = RTC;
  return true;
  #else // if defined(ESP32)

  START_TIMER
  if (!system_rtc_mem_write(RTC_BASE_STRUCT, reinterpret_cast<const uint8_t *>(&RTC), sizeof(RTC)) || !readFromRTC())
  {
      # ifdef RTC_STRUCT_DEBUG
    addLog(LOG_LEVEL_ERROR, F("RTC  : Error while writing to RTC"));
      # endif // ifdef RTC_STRUCT_DEBUG
    return false;
  }
  else
  {
    STOP_TIMER(SAVE_TO_RTC);
    return true;
  }
  #endif // if defined(ESP32)
}

/********************************************************************************************\
   Initialize RTC memory
 \*********************************************************************************************/
void initRTC()
{
  RTC.init();
  saveToRTC();

  for (size_t i = 0; i < UserVar.getNrElements(); ++i) {
    UserVar[i] = 0.0f;
  }
  saveUserVarToRTC();
}

/********************************************************************************************\
   Read RTC struct from RTC memory
 \*********************************************************************************************/
bool readFromRTC()
{
  // ESP8266 has the RTC struct stored in memory which we must actively fetch
  // ESP32 can use a compiler flag to mark a struct to be located in RTC_SLOW memory
  #ifdef ESP32
  RTC = RTC_tmp;
  #endif
  #ifdef ESP8266
  if (!system_rtc_mem_read(RTC_BASE_STRUCT, reinterpret_cast<uint8_t *>(&RTC), sizeof(RTC))) {
    return false;
  }
  #endif
  return RTC.ID1 == 0xAA && RTC.ID2 == 0x55;
}

/********************************************************************************************\
   Save values to RTC memory
 \*********************************************************************************************/
bool saveUserVarToRTC()
{
  // ESP8266 has the RTC struct stored in memory which we must actively fetch
  // ESP32   Uses a temp structure which is mapped to the RTC address range.
  #if defined(ESP32)
  for (size_t i = 0; i < UserVar_nrelements; ++i) {
    UserVar_RTC[i] = UserVar[i];
  }
  UserVar_checksum = calc_CRC32(reinterpret_cast<const uint8_t *>(&UserVar[0]), UserVar_nrelements * sizeof(float)); 
  return true;
  #endif

  #ifdef ESP8266
  // addLog(LOG_LEVEL_DEBUG, F("RTCMEM: saveUserVarToRTC"));
  uint8_t    *buffer = UserVar.get();
  size_t   size   = UserVar.getNrElements() * sizeof(float);
  uint32_t sum    = calc_CRC32(buffer, size);
  bool  ret    = system_rtc_mem_write(RTC_BASE_USERVAR, buffer, size);
  ret &= system_rtc_mem_write(RTC_BASE_USERVAR + (size >> 2), reinterpret_cast<const uint8_t *>(&sum), 4);
  return ret;
  #endif
}

/********************************************************************************************\
   Read RTC struct from RTC memory
 \*********************************************************************************************/
bool readUserVarFromRTC()
{
  // ESP8266 has the RTC struct stored in memory which we must actively fetch
  // ESP32   Uses a temp structure which is mapped to the RTC address range.
  #if defined(ESP32)
  if (calc_CRC32(reinterpret_cast<const uint8_t *>(&UserVar_RTC[0]), UserVar_nrelements * sizeof(float)) == UserVar_checksum) {
    for (size_t i = 0; i < UserVar_nrelements; ++i) {
      UserVar[i] = UserVar_RTC[i];
    }
    return true;
  }
  return false;
  #endif

  #ifdef ESP8266
  // addLog(LOG_LEVEL_DEBUG, F("RTCMEM: readUserVarFromRTC"));
  uint8_t    *buffer = UserVar.get();
  size_t   size   = UserVar.getNrElements() * sizeof(float);
  bool  ret    = system_rtc_mem_read(RTC_BASE_USERVAR, buffer, size);
  uint32_t sumRAM = calc_CRC32(buffer, size);
  uint32_t sumRTC = 0;
  ret &= system_rtc_mem_read(RTC_BASE_USERVAR + (size >> 2), reinterpret_cast<uint8_t *>(&sumRTC), 4);

  if (!ret || (sumRTC != sumRAM))
  {
      # ifdef RTC_STRUCT_DEBUG
    addLog(LOG_LEVEL_ERROR, F("RTC  : Checksum error on reading RTC user var"));
      # endif // ifdef RTC_STRUCT_DEBUG
    memset(buffer, 0, size);
  }
  return ret;
  #endif 
}


#include "../Helpers/Dallas1WireHelper.h"

#include "../../_Plugin_Helper.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Misc.h"

#include "../WebServer/JSON.h"



// DEBUG code using logic analyzer for timings
// #define DEBUG_LOGIC_ANALYZER_PIN  27 
// #define DEBUG_LOGIC_ANALYZER_PIN_ERROR  26


// Macros to perform direct access on GPIOs
// Macros written by Paul Stoffregen
// See: https://github.com/PaulStoffregen/OneWire/blob/master/util/
#include <GPIO_Direct_Access.h>


// ESP8266 does work fine without the IRAM attribute
// But ESP32 may benefit from having the code always loaded in RAM.
#ifdef ESP8266
# define DALLAS_IRAM_ATTR
#endif // ifdef ESP8266
#ifdef ESP32
# define DALLAS_IRAM_ATTR IRAM_ATTR
#endif // ifdef ESP32


#include <vector>

unsigned char ROM_NO[8];
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
uint8_t LastDeviceFlag;

int64_t usec_release   = 0;
int64_t presence_start = 0;
int64_t presence_end   = 0;


// References to 1-wire family codes:
// http://owfs.sourceforge.net/simple_family.html
// https://github.com/owfs/owfs-doc/wiki/1Wire-Device-List
String Dallas_getModel(uint8_t family) {
  String model;

  switch (family) {
    case 0x28: model = F("DS18B20"); break;
    case 0x3b: model = F("DS1825");  break;
    case 0x22: model = F("DS1822");  break;
    case 0x10: model = F("DS1820 / DS18S20");  break;
    case 0x1D: model = F("DS2423");  break; // 4k RAM with counter
    case 0x01: model = F("DS1990A"); break; // Serial Number iButton
  }
  return model;
}

String Dallas_format_address(const uint8_t addr[]) {
  String result;

  result.reserve(40);

  for (uint8_t j = 0; j < 8; j++)
  {
    if (addr[j] < 0x10) {
      result += '0';
    }
    result += String(addr[j], HEX);

    if (j < 7) { result += '-'; }
  }
  result += F(" [");
  result += Dallas_getModel(addr[0]);
  result += ']';

  return result;
}

uint64_t Dallas_addr_to_uint64(const uint8_t addr[]) {
  uint64_t tmpAddr_64 = 0;

  for (uint8_t i = 0; i < 8; ++i) {
    tmpAddr_64 *= 256;
    tmpAddr_64 += addr[i];
  }
  return tmpAddr_64;
}

void Dallas_uint64_to_addr(uint64_t value, uint8_t addr[]) {
  uint8_t i = 8;

  while (i > 0) {
    --i;
    addr[i] = static_cast<uint8_t>(value & 0xFF);
    value  /= 256;
  }
}

void Dallas_addr_selector_webform_load(taskIndex_t TaskIndex, int8_t gpio_pin_rx, int8_t gpio_pin_tx, uint8_t nrVariables) {
  if ((gpio_pin_rx == -1) || (gpio_pin_tx == -1)) {
    return;
  }

  if (nrVariables >= VARS_PER_TASK) {
    nrVariables = VARS_PER_TASK;
  }

  if (!validTaskIndex(TaskIndex)) {
    return;
  }

  std::map<uint64_t, String> addr_task_map;

  for (taskIndex_t task = 0; validTaskIndex(task); ++task) {
    if (Dallas_plugin(Settings.TaskDeviceNumber[task])) {
      uint8_t tmpAddress[8] = { 0 };

      for (uint8_t var_index = 0; var_index < VARS_PER_TASK; ++var_index) {
        Dallas_plugin_get_addr(tmpAddress, task, var_index);
        uint64_t tmpAddr_64 = Dallas_addr_to_uint64(tmpAddress);

        if (tmpAddr_64 != 0) {
          String label;
          label.reserve(32);
          label  = F(" (task ");
          label += String(task + 1);
          label += F(" [");
          label += getTaskDeviceName(task);
          label += '#';
          label += getTaskValueName(task, var_index);
          label += F("])");

          addr_task_map[tmpAddr_64] = label;
        }
      }
    }
  }

  // find all suitable devices
  std::vector<uint64_t> scan_res;

  Dallas_reset(gpio_pin_rx, gpio_pin_tx);
  Dallas_reset_search();
  uint8_t tmpAddress[8];

  while (Dallas_search(tmpAddress, gpio_pin_rx, gpio_pin_tx))
  {
    scan_res.push_back(Dallas_addr_to_uint64(tmpAddress));
  }

  for (uint8_t var_index = 0; var_index < nrVariables; ++var_index) {
    String id = F("dallas_addr");
    id += String(var_index);
    String rowLabel = F("Device Address");

    if (nrVariables > 1) {
      rowLabel += ' ';
      rowLabel += String(var_index + 1);
    }
    addRowLabel(rowLabel);
    addSelector_Head(id);
    addSelector_Item(F("- None -"), -1, false); // Empty choice
    uint8_t tmpAddress[8];

    // get currently saved address
    uint8_t savedAddress[8];

    for (uint8_t index = 0; index < scan_res.size(); ++index) {
      Dallas_plugin_get_addr(savedAddress, TaskIndex, var_index);
      Dallas_uint64_to_addr(scan_res[index], tmpAddress);
      String option = Dallas_format_address(tmpAddress);
      auto   it     = addr_task_map.find(Dallas_addr_to_uint64(tmpAddress));

      if (it != addr_task_map.end()) {
        option += it->second;
      }

      bool selected = (memcmp(tmpAddress, savedAddress, 8) == 0) ? true : false;
      addSelector_Item(option, index, selected);
    }
    addSelector_Foot();
  }
}

void Dallas_show_sensor_stats_webform_load(const Dallas_SensorData& sensor_data)
{
  if (sensor_data.addr == 0) {
    return;
  }
  addRowLabel(F("Address"));
  addHtml(sensor_data.get_formatted_address());

  addRowLabel(F("Resolution"));
  addHtmlInt(sensor_data.actual_res);

  addRowLabel(F("Parasite Powered"));
  addHtml(jsonBool(sensor_data.parasitePowered));

  addRowLabel(F("Samples Read Success"));
  addHtmlInt(sensor_data.read_success);

  addRowLabel(F("Samples Read Init Failed"));
  addHtmlInt(sensor_data.start_read_failed);

  addRowLabel(F("Samples Read Retry"));
  addHtmlInt(sensor_data.read_retry);

  addRowLabel(F("Samples Read Failed"));
  addHtmlInt(sensor_data.read_failed);
}

void Dallas_addr_selector_webform_save(taskIndex_t TaskIndex, int8_t gpio_pin_rx, int8_t gpio_pin_tx, uint8_t nrVariables)
{
  if (gpio_pin_rx == -1) {
    return;
  }

  if (gpio_pin_tx == -1) {
    return;
  }

  if (nrVariables >= VARS_PER_TASK) {
    nrVariables = VARS_PER_TASK;
  }

  if (!validTaskIndex(TaskIndex)) {
    return;
  }
  uint8_t addr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  for (uint8_t var_index = 0; var_index < nrVariables; ++var_index) {
    String id = F("dallas_addr");
    id += String(var_index);
    int selection = getFormItemInt(id, -1);

    if (selection != -1) {
      Dallas_scan(getFormItemInt(id), addr, gpio_pin_rx, gpio_pin_tx);
      Dallas_plugin_set_addr(addr, TaskIndex, var_index);
    }
  }
}

bool Dallas_plugin(pluginID_t pluginID)
{
  switch (pluginID) {
    case 4:
    case 80:
    case 100:
      return true;
  }
  return false;
}

void Dallas_plugin_get_addr(uint8_t addr[], taskIndex_t TaskIndex, uint8_t var_index)
{
  if (var_index >= 4) {
    return;
  }

  // Load ROM address from tasksettings
  LoadTaskSettings(TaskIndex);

  for (uint8_t x = 0; x < 8; x++) {
    uint32_t value = (uint32_t)ExtraTaskSettings.TaskDevicePluginConfigLong[x];
    addr[x] = static_cast<uint8_t>((value >> (var_index * 8)) & 0xFF);
  }
}

void Dallas_plugin_set_addr(uint8_t addr[], taskIndex_t TaskIndex, uint8_t var_index)
{
  if (var_index >= 4) {
    return;
  }
  LoadTaskSettings(TaskIndex);
  const uint32_t mask = ~(0xFF << (var_index * 8));

  for (uint8_t x = 0; x < 8; x++) {
    uint32_t value = (uint32_t)ExtraTaskSettings.TaskDevicePluginConfigLong[x];
    value                                          &= mask;
    value                                          += (static_cast<uint32_t>(addr[x]) << (var_index * 8));
    ExtraTaskSettings.TaskDevicePluginConfigLong[x] = (long)value;
  }
}

/*********************************************************************************************\
   Dallas Scan bus
\*********************************************************************************************/
uint8_t Dallas_scan(uint8_t getDeviceROM, uint8_t *ROM, int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  uint8_t tmpaddr[8];
  uint8_t devCount = 0;

  Dallas_reset(gpio_pin_rx, gpio_pin_tx);

  Dallas_reset_search();

  while (Dallas_search(tmpaddr, gpio_pin_rx, gpio_pin_tx))
  {
    if (getDeviceROM == devCount) {
      for (uint8_t  i = 0; i < 8; i++) {
        ROM[i] = tmpaddr[i];
      }
    }
    devCount++;
  }
  return devCount;
}

// read power supply
bool Dallas_is_parasite(const uint8_t ROM[8], int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  if (!Dallas_address_ROM(ROM, gpio_pin_rx, gpio_pin_tx)) {
    return false;
  }
  Dallas_write(0xB4, gpio_pin_rx, gpio_pin_tx); // read power supply
  return !Dallas_read_bit(gpio_pin_rx, gpio_pin_tx);
}

void Dallas_startConversion(const uint8_t ROM[8], int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  Dallas_reset(gpio_pin_rx, gpio_pin_tx);
  Dallas_write(0x55, gpio_pin_rx, gpio_pin_tx); // Choose ROM

  for (uint8_t i = 0; i < 8; i++) {
    Dallas_write(ROM[i], gpio_pin_rx, gpio_pin_tx);
  }
  Dallas_write(0x44, gpio_pin_rx, gpio_pin_tx);
}

/*********************************************************************************************\
*  Dallas Read temperature from scratchpad
\*********************************************************************************************/
bool Dallas_readTemp(const uint8_t ROM[8], float *value, int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  int16_t DSTemp;
  uint8_t ScratchPad[12];

  if (!Dallas_address_ROM(ROM, gpio_pin_rx, gpio_pin_tx)) {
    return false;
  }
  Dallas_write(0xBE, gpio_pin_rx, gpio_pin_tx); // Read scratchpad

  for (uint8_t i = 0; i < 9; i++) {             // read 9 bytes
    ScratchPad[i] = Dallas_read(gpio_pin_rx, gpio_pin_tx);
  }

  bool crc_ok = Dallas_crc8(ScratchPad);

  #ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("DS: SP: ");

    for (uint8_t x = 0; x < 9; x++)
    {
      if (x != 0) {
        log += ',';
      }
      log += String(ScratchPad[x], HEX);
    }

    if (crc_ok) {
      log += F(",OK");
    }

    if (Dallas_is_parasite(ROM, gpio_pin_rx, gpio_pin_tx)) {
      log += F(",P");
    }
    log += ',';
    log += ll2String(usec_release, DEC);
    log += ',';
    log += ll2String(presence_start, DEC);
    log += ',';
    log += ll2String(presence_end, DEC);
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
  #endif // ifndef BUILD_NO_DEBUG

  if (!crc_ok)
  {
#ifdef DEBUG_LOGIC_ANALYZER_PIN_ERROR
  // Toggle the CRC error pin to make it better visible in the logic analyzer trace
  static bool error_pin_toggle = false;
  error_pin_toggle = !error_pin_toggle;
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN_ERROR, error_pin_toggle ? 1 : 0);
#endif


    *value = 0;
    return false;
  }

  if ((ROM[0] == 0x28) || (ROM[0] == 0x3b) || (ROM[0] == 0x22)) // DS18B20 or DS1825 or DS1822
  {
    DSTemp = (ScratchPad[1] << 8) + ScratchPad[0];

    if (DSTemp == 0x550) { // power-on reset value
      return false;
    }
    *value = (float(DSTemp) * 0.0625f);
  }
  else if (ROM[0] == 0x10)       // DS1820 DS18S20
  {
    if (ScratchPad[0] == 0xaa) { // power-on reset value
      return false;
    }
    DSTemp = (ScratchPad[1] << 11) | ScratchPad[0] << 3;
    DSTemp = ((DSTemp & 0xfff0) << 3) - 16 +
             (((ScratchPad[7] - ScratchPad[6]) << 7) / ScratchPad[7]);
    *value = float(DSTemp) * 0.0078125f;
  }
  return true;
}

bool Dallas_readiButton(const uint8_t addr[8], int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  // maybe this is needed to trigger the reading
  //    uint8_t ScratchPad[12];

  Dallas_reset(gpio_pin_rx, gpio_pin_tx);
  Dallas_write(0x55, gpio_pin_rx, gpio_pin_tx); // Choose ROM

  for (uint8_t i = 0; i < 8; i++) {
    Dallas_write(addr[i], gpio_pin_rx, gpio_pin_tx);
  }

  Dallas_write(0xBE, gpio_pin_rx, gpio_pin_tx); // Read scratchpad

  //    for (uint8_t i = 0; i < 9; i++) // read 9 bytes
  //        ScratchPad[i] = Dallas_read();
  // end maybe this is needed to trigger the reading

  uint8_t tmpaddr[8];
  bool    found = false;

  Dallas_reset(gpio_pin_rx, gpio_pin_tx);
  String log;

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    log  = F("DS   : iButton searching for address: ");
    log += Dallas_format_address(addr);
    log += F(" found: ");
  }
  Dallas_reset_search();

  while (Dallas_search(tmpaddr, gpio_pin_rx, gpio_pin_tx))
  {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log += Dallas_format_address(tmpaddr);
      log += ',';
    }

    if (memcmp(addr, tmpaddr, 8) == 0)
    {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        log += F("Success. Button was found");
      }
      found = true;
    }
  }
  addLogMove(LOG_LEVEL_INFO, log);
  return found;
}

/*********************************************************************************************\
   Dallas read DS2423 counter
   Taken from https://github.com/jbechter/arduino-onewire-DS2423
\*********************************************************************************************/
#define DS2423_READ_MEMORY_COMMAND 0xa5
#define DS2423_PAGE_ONE 0xc0
#define DS2423_PAGE_TWO 0xe0

bool Dallas_readCounter(const uint8_t ROM[8], float *value, int8_t gpio_pin_rx, int8_t gpio_pin_tx, uint8_t counter)
{
  uint8_t data[45];

  data[0] = DS2423_READ_MEMORY_COMMAND;
  data[1] = (counter == 0 ? DS2423_PAGE_ONE : DS2423_PAGE_TWO);
  data[2] = 0x01;

  if (!Dallas_address_ROM(ROM, gpio_pin_rx, gpio_pin_tx)) {
    return false;
  }

  Dallas_write(data[0], gpio_pin_rx, gpio_pin_tx);
  Dallas_write(data[1], gpio_pin_rx, gpio_pin_tx);
  Dallas_write(data[2], gpio_pin_rx, gpio_pin_tx);

  for (int j = 3; j < 45; j++) {
    data[j] = Dallas_read(gpio_pin_rx, gpio_pin_tx);
  }

  Dallas_reset(gpio_pin_rx, gpio_pin_tx);

  uint32_t count = (uint32_t)data[38];

  for (int j = 37; j >= 35; j--) {
    count = (count << 8) + (uint32_t)data[j];
  }

  uint16_t crc            = Dallas_crc16(data, 43, 0);
  const uint8_t *crcBytes = reinterpret_cast<const uint8_t *>(&crc);
  uint8_t crcLo           = ~data[43];
  uint8_t crcHi           = ~data[44];
  bool    error           = (crcLo != crcBytes[0]) || (crcHi != crcBytes[1]);

  if (!error)
  {
    *value = count;
    return true;
  }
  else
  {
    *value = 0;
    return false;
  }
}

/*********************************************************************************************\
* Dallas Get Resolution
\*********************************************************************************************/
uint8_t Dallas_getResolution(const uint8_t ROM[8], int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  // DS1820 and DS18S20 have no resolution configuration register
  if (ROM[0] == 0x10) { return 12; }

  uint8_t ScratchPad[12];

  if (!Dallas_address_ROM(ROM, gpio_pin_rx, gpio_pin_tx)) {
    return 0;
  }
  Dallas_write(0xBE, gpio_pin_rx, gpio_pin_tx); // Read scratchpad

  for (uint8_t i = 0; i < 9; i++) {             // read 9 bytes
    ScratchPad[i] = Dallas_read(gpio_pin_rx, gpio_pin_tx);
  }

  if (!Dallas_crc8(ScratchPad)) {
    return 0;
  }
  else
  {
    switch (ScratchPad[4])
    {
      case 0x7F: // 12 bit
        return 12;

      case 0x5F: // 11 bit
        return 11;

      case 0x3F: // 10 bit
        return 10;

      case 0x1F: //  9 bit
      default:
        return 9;
    }
  }
  return 0;
}

/*********************************************************************************************\
* Dallas Get Resolution
\*********************************************************************************************/
bool Dallas_setResolution(const uint8_t ROM[8], uint8_t res, int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  // DS1820 and DS18S20 have no resolution configuration register
  if (ROM[0] == 0x10) { return true; }

  uint8_t ScratchPad[12];

  if (!Dallas_address_ROM(ROM, gpio_pin_rx, gpio_pin_tx)) {
    return false;
  }
  Dallas_write(0xBE, gpio_pin_rx, gpio_pin_tx); // Read scratchpad

  for (uint8_t i = 0; i < 9; i++) {             // read 9 bytes
    ScratchPad[i] = Dallas_read(gpio_pin_rx, gpio_pin_tx);
  }

  if (!Dallas_crc8(ScratchPad)) {
    addLog(LOG_LEVEL_ERROR, F("DS   : Cannot set resolution"));
    return false;
  }
  else
  {
    uint8_t old_configuration = ScratchPad[4];

    switch (res)
    {
      case 12:
        ScratchPad[4] = 0x7F; // 12 bits
        break;
      case 11:
        ScratchPad[4] = 0x5F; // 11 bits
        break;
      case 10:
        ScratchPad[4] = 0x3F; // 10 bits
        break;
      case 9:
      default:
        ScratchPad[4] = 0x1F; //  9 bits
        break;
    }

    if (ScratchPad[4] == old_configuration) {
      return true;
    }

    if (!Dallas_address_ROM(ROM, gpio_pin_rx, gpio_pin_tx)) return false;
    Dallas_write(0x4E,          gpio_pin_rx, gpio_pin_tx); // Write to EEPROM
    Dallas_write(ScratchPad[2], gpio_pin_rx, gpio_pin_tx); // high alarm temp
    Dallas_write(ScratchPad[3], gpio_pin_rx, gpio_pin_tx); // low alarm temp
    Dallas_write(ScratchPad[4], gpio_pin_rx, gpio_pin_tx); // configuration register

    if (!Dallas_address_ROM(ROM, gpio_pin_rx, gpio_pin_tx)) return false;

    // save the newly written values to eeprom
    Dallas_write(0x48, gpio_pin_rx, gpio_pin_tx);
    delay(100);  // <--- added 20ms delay to allow 10ms long EEPROM write operation (as specified by datasheet)
    Dallas_reset(gpio_pin_rx, gpio_pin_tx);

    return true; // new value set
  }
}

/*********************************************************************************************\
*  Dallas Reset
\*********************************************************************************************/
uint8_t Dallas_reset(int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  uint8_t retries = 125;

  noInterrupts();

#ifdef DEBUG_LOGIC_ANALYZER_PIN
  // DEBUG code using logic analyzer for timings
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 1);
#endif

  if (gpio_pin_rx == gpio_pin_tx) {
    DIRECT_PINMODE_INPUT(gpio_pin_rx);
  } else {
    DIRECT_pinWrite(gpio_pin_tx, 1);
  }
#ifdef DEBUG_LOGIC_ANALYZER_PIN
  // DEBUG code using logic analyzer for timings
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 0);
#endif
  bool success = true;

  do // wait until the wire is high... just in case
  {
    if (--retries == 0) {
      success = false;
    }
    delayMicroseconds(2);
  }
  while (!DIRECT_pinRead(gpio_pin_rx) && success);

  usec_release   = 0;
  presence_start = 0;
  presence_end   = 0;

  if (success) {
#ifdef DEBUG_LOGIC_ANALYZER_PIN
  // DEBUG code using logic analyzer for timings
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 1);
#endif
    // The master starts a transmission with a reset pulse,
    // which pulls the wire to 0 volts for at least 480 µs.
    // This resets every slave device on the bus.
    DIRECT_pinWrite(gpio_pin_tx, 0);

    if (gpio_pin_rx == gpio_pin_tx) {
      DIRECT_PINMODE_OUTPUT(gpio_pin_rx);
    }

    delayMicroseconds(480);

    if (gpio_pin_rx == gpio_pin_tx) {
      DIRECT_PINMODE_INPUT(gpio_pin_rx);
    } else {
      DIRECT_pinWrite(gpio_pin_tx, 1);
    }
#ifdef DEBUG_LOGIC_ANALYZER_PIN
  // DEBUG code using logic analyzer for timings
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 0);
#endif


    // After that, any slave device, if present, shows that it exists with a "presence" pulse:
    // it holds the bus low for at least 60 µs after the master releases the bus.
    // This may take about 30 usec after release for present sensors to pull the line low.
    // Sequence:
    // - Release => pin high
    // - Presence condition start (typ: 30 usec after release)
    // - Presence condition end   (minimal duration 60 usec, typ: 100 usec)
    // - Wait till 480 usec after release.
    const uint64_t start = getMicros64();
    int64_t usec_passed  = 0;

    bool waiting_for_presence = true;

    while ((usec_passed < 480) && waiting_for_presence) {
      usec_passed = usecPassedSince(start);

      const bool pin_state = !!DIRECT_pinRead(gpio_pin_rx);

      if (usec_release == 0) {
        if (pin_state) {
          // Pin has been released
          usec_release = usec_passed;
        }
      } else if (presence_start == 0) {
        if (!pin_state) {
          // Presence condition started
          presence_start = usec_passed;
        }
      } else if (presence_end == 0) {
        if (pin_state) {
          // Presence condition ended
          presence_end = usec_passed;
        }
      } else {
        // Set the pin high so we have a clear starting level on the next write.
        DIRECT_pinWrite(gpio_pin_tx, 1);
        if (gpio_pin_rx == gpio_pin_tx) {
          DIRECT_PINMODE_OUTPUT(gpio_pin_rx);
        }
        waiting_for_presence = false;
        delayMicroseconds(4);
      }
      delayMicroseconds(2);
    }
  }
  interrupts();

  if (presence_end != 0) {
    const long presence_duration = presence_end - presence_start;

    if (presence_duration > 60) { return 1; }
  }
  return 0;
}

#define FALSE 0
#define TRUE  1

/*********************************************************************************************\
*  Dallas Reset Search
\*********************************************************************************************/
void Dallas_reset_search()
{
  // reset the search state
  LastDiscrepancy       = 0;
  LastDeviceFlag        = FALSE;
  LastFamilyDiscrepancy = 0;

  for (uint8_t i = 0; i < 8; i++) {
    ROM_NO[i] = 0;
  }
}

/*********************************************************************************************\
*  Dallas Search bus
\*********************************************************************************************/
uint8_t Dallas_search(uint8_t *newAddr, int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  uint8_t id_bit_number;
  uint8_t last_zero, rom_byte_number, search_result;
  uint8_t id_bit, cmp_id_bit;
  unsigned char rom_byte_mask, search_direction;

  // initialize for search
  id_bit_number   = 1;
  last_zero       = 0;
  rom_byte_number = 0;
  rom_byte_mask   = 1;
  search_result   = 0;

  // if the last call was not the last one
  if (!LastDeviceFlag)
  {
    // 1-Wire reset
    if (!Dallas_reset(gpio_pin_rx, gpio_pin_tx))
    {
      // reset the search
      LastDiscrepancy       = 0;
      LastDeviceFlag        = FALSE;
      LastFamilyDiscrepancy = 0;
      return FALSE;
    }

    // issue the search command
    Dallas_write(0xF0, gpio_pin_rx, gpio_pin_tx);

    // loop to do the search
    do
    {
      // read a bit and its complement
      id_bit     = Dallas_read_bit(gpio_pin_rx, gpio_pin_tx);
      cmp_id_bit = Dallas_read_bit(gpio_pin_rx, gpio_pin_tx);

      // check for no devices on 1-wire
      if ((id_bit == 1) && (cmp_id_bit == 1)) {
        break;
      }
      else
      {
        // all devices coupled have 0 or 1
        if (id_bit != cmp_id_bit) {
          search_direction = id_bit; // bit write value for search
        }
        else
        {
          // if this discrepancy if before the Last Discrepancy
          // on a previous next then pick the same as last time
          if (id_bit_number < LastDiscrepancy) {
            search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
          }
          else {
            // if equal to last pick 1, if not then pick 0
            search_direction = (id_bit_number == LastDiscrepancy);
          }

          // if 0 was picked then record its position in LastZero
          if (search_direction == 0)
          {
            last_zero = id_bit_number;

            // check for Last discrepancy in family
            if (last_zero < 9) {
              LastFamilyDiscrepancy = last_zero;
            }
          }
        }

        // set or clear the bit in the ROM byte rom_byte_number
        // with mask rom_byte_mask
        if (search_direction == 1) {
          ROM_NO[rom_byte_number] |= rom_byte_mask;
        }
        else {
          ROM_NO[rom_byte_number] &= ~rom_byte_mask;
        }

        DIRECT_pinWrite(gpio_pin_tx, 1);
        if (gpio_pin_rx == gpio_pin_tx) {
          DIRECT_PINMODE_OUTPUT(gpio_pin_rx);
        }

        // serial number search direction write bit
        Dallas_write_bit(search_direction, gpio_pin_rx, gpio_pin_tx);

        // increment the byte counter id_bit_number
        // and shift the mask rom_byte_mask
        id_bit_number++;
        rom_byte_mask <<= 1;

        // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
        if (rom_byte_mask == 0)
        {
          rom_byte_number++;
          rom_byte_mask = 1;
        }
      }
    }
    while (rom_byte_number < 8); // loop until through all ROM bytes 0-7

    // if the search was successful then
    if (!(id_bit_number < 65))
    {
      // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
      LastDiscrepancy = last_zero;

      // check for last device
      if (LastDiscrepancy == 0) {
        LastDeviceFlag = TRUE;
      }

      search_result = TRUE;
    }
  }

  // if no device found then reset counters so next 'search' will be like a first
  if (!search_result || !ROM_NO[0])
  {
    LastDiscrepancy       = 0;
    LastDeviceFlag        = FALSE;
    LastFamilyDiscrepancy = 0;
    search_result         = FALSE;
  }

  for (int i = 0; i < 8; i++) {
    newAddr[i] = ROM_NO[i];
  }

  return search_result;
}

#undef FALSE
#undef TRUE

/*********************************************************************************************\
*  Dallas Read byte
\*********************************************************************************************/
uint8_t Dallas_read(int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  uint8_t bitMask;
  uint8_t r = 0;

  for (bitMask = 0x01; bitMask; bitMask <<= 1) {
    if (Dallas_read_bit(gpio_pin_rx, gpio_pin_tx)) {
      r |= bitMask;
    }
  }

  return r;
}

/*********************************************************************************************\
*  Dallas Write byte
\*********************************************************************************************/
void Dallas_write(uint8_t ByteToWrite, int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  uint8_t bitMask;

  DIRECT_pinWrite(gpio_pin_tx, 1);
  if (gpio_pin_rx == gpio_pin_tx) {
    DIRECT_PINMODE_OUTPUT(gpio_pin_rx);
  }
#ifdef DEBUG_LOGIC_ANALYZER_PIN
  // DEBUG code using logic analyzer for timings
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 1);
#endif
  for (bitMask = 0x01; bitMask; bitMask <<= 1) {
    Dallas_write_bit((bitMask & ByteToWrite) ? 1 : 0, gpio_pin_rx, gpio_pin_tx);
  }
#ifdef DEBUG_LOGIC_ANALYZER_PIN
  // DEBUG code using logic analyzer for timings
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 0);
#endif

}

/*********************************************************************************************\
*  Dallas Read bit
\*********************************************************************************************/
uint8_t Dallas_read_bit(int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  if (gpio_pin_rx == -1) { return 0; }

  if (gpio_pin_tx == -1) { return 0; }
  uint64_t start = 0;
  uint8_t r            = Dallas_read_bit_ISR(gpio_pin_rx, gpio_pin_tx, start);

  while (usecPassedSince(start) < 70ll) {
    // Wait for another 55 usec
    // Complete read cycle:
    // LOW: 6 usec
    // Float: 9 msec
    // Read value. Typically the sensor keeps the level low for 27 usec.
    // Wait for 55 usec => complete cycle = 6 + 9 + 55 = 70 usec.
  }
  return r;
}

uint8_t DALLAS_IRAM_ATTR Dallas_read_bit_ISR(
  int8_t        gpio_pin_rx,
  int8_t        gpio_pin_tx,
  uint64_t& start)
{
  uint8_t r;

  {
    noInterrupts();
    start = getMicros64();
    DIRECT_pinWrite(gpio_pin_tx, 0);
    if (gpio_pin_rx == gpio_pin_tx) {
      DIRECT_PINMODE_OUTPUT(gpio_pin_rx);
    }

    while (usecPassedSince(start) < 6) {
      // Wait for 6 usec
    }
    const uint64_t startwait = getMicros64();

    if (gpio_pin_rx == gpio_pin_tx) {
      DIRECT_PINMODE_INPUT(gpio_pin_rx); // let pin float, pull up will raise
    } else {
      DIRECT_pinWrite(gpio_pin_tx, 1);
    }

    while (usecPassedSince(startwait) < 9ll) {
      // Wait for another 9 usec
    }
    r = DIRECT_pinRead(gpio_pin_rx);

    interrupts();
  }
  return r;
}

/*********************************************************************************************\
*  Dallas Write bit
\*********************************************************************************************/
void Dallas_write_bit(uint8_t v, int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  if (gpio_pin_tx == -1) { return; }

  // Determine times in usec for high and low
  // write 1: low 6 usec, high 64 usec
  // write 0: low 60 usec, high 10 usec
  const long low_time  = (v & 1) ? 6 : 60;
  const long high_time = (v & 1) ? 64 : 10;
  uint64_t   start     = 0;

  Dallas_write_bit_ISR(v, gpio_pin_rx, gpio_pin_tx, low_time, high_time, start);

  while (usecPassedSince(start) < high_time) {
    // output remains high
  }
}

void DALLAS_IRAM_ATTR Dallas_write_bit_ISR(uint8_t   v,
                                           int8_t    gpio_pin_rx,
                                           int8_t    gpio_pin_tx,
                                           long      low_time,
                                           long      high_time,
                                           uint64_t& start)
{
  noInterrupts();
  start     = getMicros64();
  DIRECT_pinWrite(gpio_pin_tx, 0);

  while (usecPassedSince(start) < low_time) {
    // output remains low
  }
  start = getMicros64();
  DIRECT_pinWrite(gpio_pin_tx, 1);
  interrupts();
}

/*********************************************************************************************\
*  Standard function to initiate addressing a sensor.
\*********************************************************************************************/
bool Dallas_address_ROM(const uint8_t ROM[8], int8_t gpio_pin_rx, int8_t gpio_pin_tx)
{
  if (!Dallas_reset(gpio_pin_rx, gpio_pin_tx)) { return false; }
  Dallas_write(0x55, gpio_pin_rx, gpio_pin_tx); // Choose ROM

  for (uint8_t i = 0; i < 8; i++) {
    Dallas_write(ROM[i], gpio_pin_rx, gpio_pin_tx);
  }
  return true;
}

/*********************************************************************************************\
*  Dallas Calculate CRC8 and compare it of addr[0-7] and compares it to addr[8]
\*********************************************************************************************/
bool Dallas_crc8(const uint8_t *addr)
{
  uint8_t crc = 0;
  uint8_t len = 8;

  while (len--)
  {
    uint8_t inbyte = *addr++; // from 0 to 7

    for (uint8_t i = 8; i; i--)
    {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;

      if (mix) { crc ^= 0x8C; }
      inbyte >>= 1;
    }
  }
  return crc == *addr; // addr 8
}

/*********************************************************************************************\
*  Dallas Calculate CRC16
\*********************************************************************************************/
uint16_t Dallas_crc16(const uint8_t *input, uint16_t len, uint16_t crc)
{
  static const uint8_t oddparity[16] =
  { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

  for (uint16_t i = 0; i < len; i++) {
    // Even though we're just copying a byte from the input,
    // we'll be doing 16-bit computation with it.
    uint16_t cdata = input[i];
    cdata = (cdata ^ crc) & 0xff;
    crc >>= 8;

    if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4]) {
      crc ^= 0xC001;
    }

    cdata <<= 6;
    crc    ^= cdata;
    cdata <<= 1;
    crc    ^= cdata;
  }

  return crc;
}

void Dallas_SensorData::set_measurement_inactive() {
  measurementActive = false;
  value             = 0.0f;
  valueRead         = false;
}

bool Dallas_SensorData::initiate_read(int8_t gpio_rx, int8_t gpio_tx, int8_t res) {
  if (addr == 0) { return false; }
  uint8_t tmpaddr[8];

  Dallas_uint64_to_addr(addr, tmpaddr);

  if (lastReadError) {
    if (!check_sensor(gpio_rx, gpio_tx, res)) {
      return false;
    }
    lastReadError = false;
  }

  if (!Dallas_address_ROM(tmpaddr, gpio_rx, gpio_tx)) {
    ++start_read_retry;
    if (!Dallas_address_ROM(tmpaddr, gpio_rx, gpio_tx)) {
      ++start_read_failed;
      lastReadError = true;
      return false;
    }
  }
  Dallas_write(0x44, gpio_rx, gpio_tx); // Take temperature measurement
  return true;
}

bool Dallas_SensorData::collect_value(int8_t gpio_rx, int8_t gpio_tx) {
  if ((addr != 0) && measurementActive) {
    uint8_t tmpaddr[8];
    Dallas_uint64_to_addr(addr, tmpaddr);

    if (!Dallas_readTemp(tmpaddr, &value, gpio_rx, gpio_tx)) {
      ++read_retry;
      if (!Dallas_readTemp(tmpaddr, &value, gpio_rx, gpio_tx)) {
        ++read_failed;
        lastReadError = true;
        return false;
      }
    }

    ++read_success;
    lastReadError = false;
    valueRead     = true;
    return true;
  }
  return false;
}

String Dallas_SensorData::get_formatted_address() const {
  if (addr == 0) { return EMPTY_STRING; }

  uint8_t tmpaddr[8];

  Dallas_uint64_to_addr(addr, tmpaddr);
  return Dallas_format_address(tmpaddr);
}

bool Dallas_SensorData::check_sensor(int8_t gpio_rx, int8_t gpio_tx, int8_t res) {
  if (addr == 0) { return false; }
  uint8_t tmpaddr[8];

  Dallas_uint64_to_addr(addr, tmpaddr);

  actual_res = Dallas_getResolution(tmpaddr, gpio_rx, gpio_tx);

  if (actual_res == 0) {
    ++read_failed;
    lastReadError = true;
    return false;
  }

  if (res != actual_res) {
    if (!Dallas_setResolution(tmpaddr, res, gpio_rx, gpio_tx)) {
      return false;
    }
  }

  parasitePowered = Dallas_is_parasite(tmpaddr, gpio_rx, gpio_tx);
  return true;
}

#include "../Helpers/DeepSleep.h"

#include "../../ESPEasy_common.h"
#include "../../ESPEasy-Globals.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyEth.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../ESPEasyCore/ESPEasyWifi.h"
#include "../ESPEasyCore/ESPEasyRules.h"

#include "../Globals/EventQueue.h"
#include "../Globals/RTC.h"
#include "../Globals/Settings.h"
#include "../Globals/Statistics.h"

#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/Misc.h"
#include "../Helpers/PeriodicalActions.h"

#include <limits.h>


/**********************************************************
*                                                         *
* Deep Sleep related functions                            *
*                                                         *
**********************************************************/
int getDeepSleepMax()
{
  #ifdef ESP8266
  int dsmax = 4294; // About 71 minutes, limited by hardware
  #endif // ifdef ESP8266
  #ifdef ESP32
  int dsmax = 281474976; // / 3600 (hour) / 24 (day) / 365 (year) = ~8 years. (max. 48 bits in microseconds)
  #endif // ifdef ESp32

#if defined(CORE_POST_2_5_0)
  dsmax = INT_MAX;

  // Convert to sec and add 5% margin.
  // See: https://github.com/esp8266/Arduino/pull/4936#issuecomment-410435875
  const uint64_t sdk_dsmax_sec = ESP.deepSleepMax() / 1050000ULL;

  if (sdk_dsmax_sec <= static_cast<uint64_t>(INT_MAX)) {
    dsmax = sdk_dsmax_sec;
  }
#endif // if defined(CORE_POST_2_5_0)
  return dsmax;
}

bool isDeepSleepEnabled()
{
  if (!Settings.deepSleep_wakeTime) {
    return false;
  }

  // cancel deep sleep loop by pulling the pin GPIO16(D0) to GND
  // recommended wiring: 3-pin-header with 1=RST, 2=D0, 3=GND
  //                    short 1-2 for normal deep sleep / wakeup loop
  //                    short 2-3 to cancel sleep loop for modifying settings

#ifndef ESP32 // pinMode() crashes the ESP32 when PSRAM is enabled and available. So don't check for ESP32.
  pinMode(16, INPUT_PULLUP);

  if (!digitalRead(16))
  {
    return false;
  }
#endif
  return true;
}

bool readyForSleep()
{
  if (!isDeepSleepEnabled()) {
    return false;
  }

  if (!NetworkConnected()) {
    // Allow 12 seconds to establish connections
    return timeOutReached(timerAwakeFromDeepSleep + 12000);
  }
  return timeOutReached(timerAwakeFromDeepSleep + 1000 * Settings.deepSleep_wakeTime);
}

void prepare_deepSleep(int dsdelay)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("prepare_deepSleep"));
  #endif

  if (!isDeepSleepEnabled())
  {
    // Deep sleep canceled by GPIO16(D0)=LOW
    return;
  }

  // first time deep sleep? offer a way to escape
  if (lastBootCause != BOOT_CAUSE_DEEP_SLEEP)
  {
    addLog(LOG_LEVEL_INFO, F("SLEEP: Entering deep sleep in 30 seconds."));

    if (Settings.UseRules && isDeepSleepEnabled())
    {
      eventQueue.add(F("System#NoSleep=30"));
      while (processNextEvent()) {
        delay(1);
      }
    }
    delayBackground(30000);

    // disabled?
    if (!isDeepSleepEnabled())
    {
      addLog(LOG_LEVEL_INFO, F("SLEEP: Deep sleep cancelled (GPIO16 connected to GND)"));
      return;
    }
  }
  deepSleepStart(dsdelay); // Call deepSleepStart function after these checks
}

void deepSleepStart(int dsdelay)
{
  // separate function that is called from above function or directly from rules, usign deepSleep_wakeTime as a one-shot
  if (Settings.UseRules)
  {
    eventQueue.add(F("System#Sleep"));
    while (processNextEvent()) {
      delay(1);
    }
  }

  addLog(LOG_LEVEL_INFO, F("SLEEP: Powering down to deepsleep..."));
  RTC.deepSleepState = 1;
  prepareShutdown(ESPEasy_Scheduler::IntendedRebootReason_e::DeepSleep);

  #if defined(ESP8266)
    # if defined(CORE_POST_2_5_0)
  if ((dsdelay < 0) || dsdelay > getDeepSleepMax()) {
    dsdelay = getDeepSleepMax();
  }
  uint64_t deepSleep_usec = dsdelay * 1000000ULL;

  if (Settings.UseAlternativeDeepSleep()) {
    // See: https://github.com/esp8266/Arduino/issues/6318#issuecomment-711389479
    #include <c_types.h>
    // system_phy_set_powerup_option:
    // 1 = RF initialization only calibrate VDD33 and Tx power which will take about 18 ms
    // 2 = RF initialization only calibrate VDD33 which will take about 2 ms
    system_phy_set_powerup_option(2); // calibrate only 2ms;
    system_deep_sleep_set_option(static_cast<int>(WAKE_RF_DEFAULT));
    uint32_t*RT= (uint32_t *)0x60000700;
    uint32 t_us = 1.31 * deepSleep_usec;
    {
      RT[4] = 0;
      *RT = 0;
      RT[1]=100;
      RT[3] = 0x10010;
      RT[6] = 8;
      RT[17] = 4;
      RT[2] = 1<<20;
      ets_delay_us(10);
      RT[1]=t_us>>3;
      RT[3] = 0x640C8;
      RT[4]= 0;
      RT[6] = 0x18;
      RT[16] = 0x7F;
      RT[17] = 0x20;
      RT[39] = 0x11;
      RT[40] = 0x03;
      RT[2] |= 1<<20;
      __asm volatile ("waiti 0");
    }
    yield();
  } else {
    ESP.deepSleep(deepSleep_usec, WAKE_RF_DEFAULT);
  }
    # else // if defined(CORE_POST_2_5_0)

  if ((dsdelay > 4294) || (dsdelay < 0)) {
    dsdelay = 4294; // max sleep time ~71 minutes
  }
  ESP.deepSleep((uint32_t)dsdelay * 1000000, WAKE_RF_DEFAULT);
    # endif // if defined(CORE_POST_2_5_0)
  #endif // if defined(ESP8266)
  #if defined(ESP32)
  if ((dsdelay > getDeepSleepMax()) || (dsdelay < 0)) {
    dsdelay = getDeepSleepMax(); // Max sleep time ~8 years! Reference: ESP32 technical reference manual, ULP Coprocessor, RTC Timer: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#ulp
  }
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(dsdelay) * 1000000ULL); // max 48 bits used, value in microseconds: 0xFFFFFFFFFFFF (281474976710655 dec.) / 1000000 [usec](281474976 seconds) / 3600 (hours) / 24 (day) / 365 (year) = ~8
  esp_deep_sleep_start();
  #endif // if defined(ESP32)
}


#include "../Helpers/OTA.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/Serial.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/Services.h"
#include "../Globals/Settings.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/Misc.h"

#if FEATURE_ARDUINO_OTA
  //enable Arduino OTA updating.
  //Note: This adds around 10kb to the firmware size, and 1kb extra ram.
  #include <ArduinoOTA.h>
#endif


bool OTA_possible(uint32_t& maxSketchSize, bool& use2step) {
#if defined(ESP8266)

  // Compute the current free space and sketch size, rounded to 4k blocks.
  // These block bounaries are needed for erasing a full block on flash.
  const uint32_t freeSketchSpace            = (getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  const uint32_t currentSketchSize          = (getSketchSize() + 0x1000) & 0xFFFFF000;
  const uint32_t smallestOtaImageSizeNeeded = (((SMALLEST_OTA_IMAGE + 16) + 0x1000) & 0xFFFFF000);
  const bool     otaPossible                = freeSketchSpace >= smallestOtaImageSizeNeeded;
  use2step = freeSketchSpace < currentSketchSize; // Assume the new image has the same size.

  if (use2step) {
    const uint32_t totalSketchSpace = freeSketchSpace + currentSketchSize;
    maxSketchSize = totalSketchSpace - smallestOtaImageSizeNeeded;
  } else {
    maxSketchSize = freeSketchSpace;
  }
  maxSketchSize -= 16; // Must leave 16 bytes at the end.

  if (maxSketchSize > MAX_SKETCH_SIZE) { maxSketchSize = MAX_SKETCH_SIZE; }
  return otaPossible;
#elif defined(ESP32)
  // ESP32 writes an OTA image to the "other" app partition.
  // Thus what is reported as "free" sketch space is the size of the not used app partition.
  maxSketchSize = getFreeSketchSpace();
  use2step      = false;
  return true;
#else // if defined(ESP8266)
  return false;
#endif // if defined(ESP8266)
}

#if FEATURE_ARDUINO_OTA

/********************************************************************************************\
   Allow updating via the Arduino OTA-protocol. (this allows you to upload directly from platformio)
 \*********************************************************************************************/
void ArduinoOTAInit()
{
  # ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("ArduinoOTAInit"));
  # endif // ifndef BUILD_NO_RAM_TRACKER

  ArduinoOTA.setPort(ARDUINO_OTA_PORT);
  ArduinoOTA.setHostname(Settings.getHostname().c_str());

  if (SecuritySettings.Password[0] != 0) {
    ArduinoOTA.setPassword(SecuritySettings.Password);
  }

  ArduinoOTA.onStart([]() {
    serialPrintln(F("OTA  : Start upload"));
    ArduinoOTAtriggered = true;
    ESPEASY_FS.end(); // important, otherwise it fails
  });

  ArduinoOTA.onEnd([]() {
    serialPrintln(F("\nOTA  : End"));

    // "dangerous": if you reset during flash you have to reflash via serial
    // so dont touch device until restart is complete
    serialPrintln(F("\nOTA  : DO NOT RESET OR POWER OFF UNTIL BOOT+FLASH IS COMPLETE."));

    // delay(100);
    // reboot(); //Not needed, node reboots automaticall after calling onEnd and succesfully flashing
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (Settings.UseSerial) {
      Serial.printf("OTA  : Progress %u%%\r", (progress / (total / 100)));
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    serialPrint(F("\nOTA  : Error (will reboot): "));

    if (error == OTA_AUTH_ERROR) { serialPrintln(F("Auth Failed")); }
    else if (error == OTA_BEGIN_ERROR) { serialPrintln(F("Begin Failed")); }
    else if (error == OTA_CONNECT_ERROR) { serialPrintln(F("Connect Failed")); }
    else if (error == OTA_RECEIVE_ERROR) { serialPrintln(F("Receive Failed")); }
    else if (error == OTA_END_ERROR) { serialPrintln(F("End Failed")); }

    delay(100);
    reboot(ESPEasy_Scheduler::IntendedRebootReason_e::OTA_error);
  });

  #if defined(ESP8266) && FEATURE_MDNS
  ArduinoOTA.begin(true);
  #else
  ArduinoOTA.begin();
  #endif

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("OTA  : Arduino OTA enabled on port ");
    log += ARDUINO_OTA_PORT;
    addLogMove(LOG_LEVEL_INFO, log);
  }
}

void ArduinoOTA_handle()
{
  ArduinoOTA.handle();
}

#endif // if FEATURE_ARDUINO_OTA

#include "../Helpers/_Plugin_init.h"

#include "../../ESPEasy_common.h"

#include "../Globals/Plugins.h"
#include "../Globals/Settings.h"

#include "../Helpers/Misc.h"


// ********************************************************************************
// Initialize all plugins that where defined earlier
// and initialize the function call pointer into the plugin array
// ********************************************************************************


// Uncrustify must not be used on macros, so turn it off.
// *INDENT-OFF*
#define ADDPLUGIN(NNN, ID) if (addPlugin(ID, x)) Plugin_ptr[x++] = &Plugin_##NNN;
// Uncrustify must not be used on macros, but we're now done, so turn Uncrustify on again.
// *INDENT-ON*

void PluginInit()
{
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  DeviceIndex_to_Plugin_id[PLUGIN_MAX] = INVALID_PLUGIN_ID;
  // Clear pointer table for all plugins
  for (deviceIndex_t x = 0; x < PLUGIN_MAX; x++)
  {
    Plugin_ptr[x] = nullptr;
    DeviceIndex_to_Plugin_id[x] = INVALID_PLUGIN_ID;
    // Do not initialize Plugin_id_to_DeviceIndex[x] to an invalid value. (it is map)
  }
  uint32_t x = 0; // Used in ADDPLUGIN macro

#ifdef USES_P001
  ADDPLUGIN(001, 1)
#endif

#ifdef USES_P002
  ADDPLUGIN(002, 2)
#endif

#ifdef USES_P003
  ADDPLUGIN(003, 3)
#endif

#ifdef USES_P004
  ADDPLUGIN(004, 4)
#endif

#ifdef USES_P005
  ADDPLUGIN(005, 5)
#endif

#ifdef USES_P006
  ADDPLUGIN(006, 6)
#endif

#ifdef USES_P007
  ADDPLUGIN(007, 7)
#endif

#ifdef USES_P008
  ADDPLUGIN(008, 8)
#endif

#ifdef USES_P009
  ADDPLUGIN(009, 9)
#endif

#ifdef USES_P010
  ADDPLUGIN(010, 10)
#endif

#ifdef USES_P011
  ADDPLUGIN(011, 11)
#endif

#ifdef USES_P012
  ADDPLUGIN(012, 12)
#endif

#ifdef USES_P013
  ADDPLUGIN(013, 13)
#endif

#ifdef USES_P014
  ADDPLUGIN(014, 14)
#endif

#ifdef USES_P015
  ADDPLUGIN(015, 15)
#endif

#ifdef USES_P016
  ADDPLUGIN(016, 16)
#endif

#ifdef USES_P017
  ADDPLUGIN(017, 17)
#endif

#ifdef USES_P018
  ADDPLUGIN(018, 18)
#endif

#ifdef USES_P019
  ADDPLUGIN(019, 19)
#endif

#ifdef USES_P020
  ADDPLUGIN(020, 20)
#endif

#ifdef USES_P021
  ADDPLUGIN(021, 21)
#endif

#ifdef USES_P022
  ADDPLUGIN(022, 22)
#endif

#ifdef USES_P023
  ADDPLUGIN(023, 23)
#endif

#ifdef USES_P024
  ADDPLUGIN(024, 24)
#endif

#ifdef USES_P025
  ADDPLUGIN(025, 25)
#endif

#ifdef USES_P026
  ADDPLUGIN(026, 26)
#endif

#ifdef USES_P027
  ADDPLUGIN(027, 27)
#endif

#ifdef USES_P028
  ADDPLUGIN(028, 28)
#endif

#ifdef USES_P029
  ADDPLUGIN(029, 29)
#endif

#ifdef USES_P030
  ADDPLUGIN(030, 30)
#endif

#ifdef USES_P031
  ADDPLUGIN(031, 31)
#endif

#ifdef USES_P032
  ADDPLUGIN(032, 32)
#endif

#ifdef USES_P033
  ADDPLUGIN(033, 33)
#endif

#ifdef USES_P034
  ADDPLUGIN(034, 34)
#endif

#ifdef USES_P035
  ADDPLUGIN(035, 35)
#endif

#ifdef USES_P036
  ADDPLUGIN(036, 36)
#endif

#ifdef USES_P037
  ADDPLUGIN(037, 37)
#endif

#ifdef USES_P038
  ADDPLUGIN(038, 38)
#endif

#ifdef USES_P039
  ADDPLUGIN(039, 39)
#endif

#ifdef USES_P040
  ADDPLUGIN(040, 40)
#endif

#ifdef USES_P041
  ADDPLUGIN(041, 41)
#endif

#ifdef USES_P042
  ADDPLUGIN(042, 42)
#endif

#ifdef USES_P043
  ADDPLUGIN(043, 43)
#endif

#ifdef USES_P044
  ADDPLUGIN(044, 44)
#endif

#ifdef USES_P045
  ADDPLUGIN(045, 45)
#endif

#ifdef USES_P046
  ADDPLUGIN(046, 46)
#endif

#ifdef USES_P047
  ADDPLUGIN(047, 47)
#endif

#ifdef USES_P048
  ADDPLUGIN(048, 48)
#endif

#ifdef USES_P049
  ADDPLUGIN(049, 49)
#endif

#ifdef USES_P050
  ADDPLUGIN(050, 50)
#endif

#ifdef USES_P051
  ADDPLUGIN(051, 51)
#endif

#ifdef USES_P052
  ADDPLUGIN(052, 52)
#endif

#ifdef USES_P053
  ADDPLUGIN(053, 53)
#endif

#ifdef USES_P054
  ADDPLUGIN(054, 54)
#endif

#ifdef USES_P055
  ADDPLUGIN(055, 55)
#endif

#ifdef USES_P056
  ADDPLUGIN(056, 56)
#endif

#ifdef USES_P057
  ADDPLUGIN(057, 57)
#endif

#ifdef USES_P058
  ADDPLUGIN(058, 58)
#endif

#ifdef USES_P059
  ADDPLUGIN(059, 59)
#endif

#ifdef USES_P060
  ADDPLUGIN(060, 60)
#endif

#ifdef USES_P061
  ADDPLUGIN(061, 61)
#endif

#ifdef USES_P062
  ADDPLUGIN(062, 62)
#endif

#ifdef USES_P063
  ADDPLUGIN(063, 63)
#endif

#ifdef USES_P064
  ADDPLUGIN(064, 64)
#endif

#ifdef USES_P065
  ADDPLUGIN(065, 65)
#endif

#ifdef USES_P066
  ADDPLUGIN(066, 66)
#endif

#ifdef USES_P067
  ADDPLUGIN(067, 67)
#endif

#ifdef USES_P068
  ADDPLUGIN(068, 68)
#endif

#ifdef USES_P069
  ADDPLUGIN(069, 69)
#endif

#ifdef USES_P070
  ADDPLUGIN(070, 70)
#endif

#ifdef USES_P071
  ADDPLUGIN(071, 71)
#endif

#ifdef USES_P072
  ADDPLUGIN(072, 72)
#endif

#ifdef USES_P073
  ADDPLUGIN(073, 73)
#endif

#ifdef USES_P074
  ADDPLUGIN(074, 74)
#endif

#ifdef USES_P075
  ADDPLUGIN(075, 75)
#endif

#ifdef USES_P076
  ADDPLUGIN(076, 76)
#endif

#ifdef USES_P077
  ADDPLUGIN(077, 77)
#endif

#ifdef USES_P078
  ADDPLUGIN(078, 78)
#endif

#ifdef USES_P079
  ADDPLUGIN(079, 79)
#endif

#ifdef USES_P080
  ADDPLUGIN(080, 80)
#endif

#ifdef USES_P081
  ADDPLUGIN(081, 81)
#endif

#ifdef USES_P082
  ADDPLUGIN(082, 82)
#endif

#ifdef USES_P083
  ADDPLUGIN(083, 83)
#endif

#ifdef USES_P084
  ADDPLUGIN(084, 84)
#endif

#ifdef USES_P085
  ADDPLUGIN(085, 85)
#endif

#ifdef USES_P086
  ADDPLUGIN(086, 86)
#endif

#ifdef USES_P087
  ADDPLUGIN(087, 87)
#endif

#ifdef USES_P088
  ADDPLUGIN(088, 88)
#endif

#ifdef USES_P089
  #ifdef ESP8266
  // FIXME TD-er: Support Ping plugin for ESP32
  ADDPLUGIN(089, 89)
  #endif
#endif

#ifdef USES_P090
  ADDPLUGIN(090, 90)
#endif

#ifdef USES_P091
  ADDPLUGIN(091, 91)
#endif

#ifdef USES_P092
  ADDPLUGIN(092, 92)
#endif

#ifdef USES_P093
  ADDPLUGIN(093, 93)
#endif

#ifdef USES_P094
  ADDPLUGIN(094, 94)
#endif

#ifdef USES_P095
  ADDPLUGIN(095, 95)
#endif

#ifdef USES_P096
  ADDPLUGIN(096, 96)
#endif

#ifdef USES_P097
  #ifdef ESP32
  ADDPLUGIN(097, 97) // Touch (ESP32)
  #endif
#endif

#ifdef USES_P098
  ADDPLUGIN(098, 98)
#endif

#ifdef USES_P099
  ADDPLUGIN(099, 99)
#endif

#ifdef USES_P100
  ADDPLUGIN(100, 100)
#endif

#ifdef USES_P101
  ADDPLUGIN(101, 101)
#endif

#ifdef USES_P102
  ADDPLUGIN(102, 102)
#endif

#ifdef USES_P103
  ADDPLUGIN(103, 103)
#endif

#ifdef USES_P104
  ADDPLUGIN(104, 104)
#endif

#ifdef USES_P105
  ADDPLUGIN(105, 105)
#endif

#ifdef USES_P106
  ADDPLUGIN(106, 106)
#endif

#ifdef USES_P107
  ADDPLUGIN(107, 107)
#endif

#ifdef USES_P108
  ADDPLUGIN(108, 108)
#endif

#ifdef USES_P109
  ADDPLUGIN(109, 109)
#endif

#ifdef USES_P110
  ADDPLUGIN(110, 110)
#endif

#ifdef USES_P111
  ADDPLUGIN(111, 111)
#endif

#ifdef USES_P112
  ADDPLUGIN(112, 112)
#endif

#ifdef USES_P113
  ADDPLUGIN(113, 113)
#endif

#ifdef USES_P114
  ADDPLUGIN(114, 114)
#endif

#ifdef USES_P115
  ADDPLUGIN(115, 115)
#endif

#ifdef USES_P116
  ADDPLUGIN(116, 116)
#endif

#ifdef USES_P117
  ADDPLUGIN(117, 117)
#endif

#ifdef USES_P118
  ADDPLUGIN(118, 118)
#endif

#ifdef USES_P119
  ADDPLUGIN(119, 119)
#endif

#ifdef USES_P120
  ADDPLUGIN(120, 120)
#endif

#ifdef USES_P121
  ADDPLUGIN(121, 121)
#endif

#ifdef USES_P122
  ADDPLUGIN(122, 122)
#endif

#ifdef USES_P123
  ADDPLUGIN(123, 123)
#endif

#ifdef USES_P124
  ADDPLUGIN(124, 124)
#endif

#ifdef USES_P125
  ADDPLUGIN(125, 125)
#endif

#ifdef USES_P126
  ADDPLUGIN(126, 126)
#endif

#ifdef USES_P127
  ADDPLUGIN(127, 127)
#endif

#ifdef USES_P128
  ADDPLUGIN(128, 128)
#endif

#ifdef USES_P129
  ADDPLUGIN(129, 129)
#endif

#ifdef USES_P130
  ADDPLUGIN(130, 130)
#endif

#ifdef USES_P131
  ADDPLUGIN(131, 131)
#endif

#ifdef USES_P132
  ADDPLUGIN(132, 132)
#endif

#ifdef USES_P133
  ADDPLUGIN(133, 133)
#endif

#ifdef USES_P134
  ADDPLUGIN(134, 134)
#endif

#ifdef USES_P135
  ADDPLUGIN(135, 135)
#endif

#ifdef USES_P136
  ADDPLUGIN(136, 136)
#endif

#ifdef USES_P137
  ADDPLUGIN(137, 137)
#endif

#ifdef USES_P138
  ADDPLUGIN(138, 138)
#endif

#ifdef USES_P139
  ADDPLUGIN(139, 139)
#endif

#ifdef USES_P140
  ADDPLUGIN(140, 140)
#endif

#ifdef USES_P141
  ADDPLUGIN(141, 141)
#endif

#ifdef USES_P142
  ADDPLUGIN(142, 142)
#endif

#ifdef USES_P143
  ADDPLUGIN(143, 143)
#endif

#ifdef USES_P144
  ADDPLUGIN(144, 144)
#endif

#ifdef USES_P145
  ADDPLUGIN(145, 145)
#endif

#ifdef USES_P146
  ADDPLUGIN(146, 146)
#endif

#ifdef USES_P147
  ADDPLUGIN(147, 147)
#endif

#ifdef USES_P148
  ADDPLUGIN(148, 148)
#endif

#ifdef USES_P149
  ADDPLUGIN(149, 149)
#endif

#ifdef USES_P150
  ADDPLUGIN(150, 150)
#endif

#ifdef USES_P151
  ADDPLUGIN(151, 151)
#endif

#ifdef USES_P152
  ADDPLUGIN(152, 152)
#endif

#ifdef USES_P153
  ADDPLUGIN(153, 153)
#endif

#ifdef USES_P154
  ADDPLUGIN(154, 154)
#endif

#ifdef USES_P155
  ADDPLUGIN(155, 155)
#endif

#ifdef USES_P156
  ADDPLUGIN(156, 156)
#endif

#ifdef USES_P157
  ADDPLUGIN(157, 157)
#endif

#ifdef USES_P158
  ADDPLUGIN(158, 158)
#endif

#ifdef USES_P159
  ADDPLUGIN(159, 159)
#endif

#ifdef USES_P160
  ADDPLUGIN(160, 160)
#endif

#ifdef USES_P161
  ADDPLUGIN(161, 161)
#endif

#ifdef USES_P162
  ADDPLUGIN(162, 162)
#endif

#ifdef USES_P163
  ADDPLUGIN(163, 163)
#endif

#ifdef USES_P164
  ADDPLUGIN(164, 164)
#endif

#ifdef USES_P165
  ADDPLUGIN(165, 165)
#endif

#ifdef USES_P166
  ADDPLUGIN(166, 166)
#endif

#ifdef USES_P167
  ADDPLUGIN(167, 167)
#endif

#ifdef USES_P168
  ADDPLUGIN(168, 168)
#endif

#ifdef USES_P169
  ADDPLUGIN(169, 169)
#endif

#ifdef USES_P170
  ADDPLUGIN(170, 170)
#endif

#ifdef USES_P171
  ADDPLUGIN(171, 171)
#endif

#ifdef USES_P172
  ADDPLUGIN(172, 172)
#endif

#ifdef USES_P173
  ADDPLUGIN(173, 173)
#endif

#ifdef USES_P174
  ADDPLUGIN(174, 174)
#endif

#ifdef USES_P175
  ADDPLUGIN(175, 175)
#endif

#ifdef USES_P176
  ADDPLUGIN(176, 176)
#endif

#ifdef USES_P177
  ADDPLUGIN(177, 177)
#endif

#ifdef USES_P178
  ADDPLUGIN(178, 178)
#endif

#ifdef USES_P179
  ADDPLUGIN(179, 179)
#endif

#ifdef USES_P180
  ADDPLUGIN(180, 180)
#endif

#ifdef USES_P181
  ADDPLUGIN(181, 181)
#endif

#ifdef USES_P182
  ADDPLUGIN(182, 182)
#endif

#ifdef USES_P183
  ADDPLUGIN(183, 183)
#endif

#ifdef USES_P184
  ADDPLUGIN(184, 184)
#endif

#ifdef USES_P185
  ADDPLUGIN(185, 185)
#endif

#ifdef USES_P186
  ADDPLUGIN(186, 186)
#endif

#ifdef USES_P187
  ADDPLUGIN(187, 187)
#endif

#ifdef USES_P188
  ADDPLUGIN(188, 188)
#endif

#ifdef USES_P189
  ADDPLUGIN(189, 189)
#endif

#ifdef USES_P190
  ADDPLUGIN(190, 190)
#endif

#ifdef USES_P191
  ADDPLUGIN(191, 191)
#endif

#ifdef USES_P192
  ADDPLUGIN(192, 192)
#endif

#ifdef USES_P193
  ADDPLUGIN(193, 193)
#endif

#ifdef USES_P194
  ADDPLUGIN(194, 194)
#endif

#ifdef USES_P195
  ADDPLUGIN(195, 195)
#endif

#ifdef USES_P196
  ADDPLUGIN(196, 196)
#endif

#ifdef USES_P197
  ADDPLUGIN(197, 197)
#endif

#ifdef USES_P198
  ADDPLUGIN(198, 198)
#endif

#ifdef USES_P199
  ADDPLUGIN(199, 199)
#endif

#ifdef USES_P200
  ADDPLUGIN(200, 200)
#endif

#ifdef USES_P201
  ADDPLUGIN(201, 201)
#endif

#ifdef USES_P202
  ADDPLUGIN(202, 202)
#endif

#ifdef USES_P203
  ADDPLUGIN(203, 203)
#endif

#ifdef USES_P204
  ADDPLUGIN(204, 204)
#endif

#ifdef USES_P205
  ADDPLUGIN(205, 205)
#endif

#ifdef USES_P206
  ADDPLUGIN(206, 206)
#endif

#ifdef USES_P207
  ADDPLUGIN(207, 207)
#endif

#ifdef USES_P208
  ADDPLUGIN(208, 208)
#endif

#ifdef USES_P209
  ADDPLUGIN(209, 209)
#endif

#ifdef USES_P210
  ADDPLUGIN(210, 210)
#endif

#ifdef USES_P211
  ADDPLUGIN(211, 211)
#endif

#ifdef USES_P212
  ADDPLUGIN(212, 212)
#endif

#ifdef USES_P213
  ADDPLUGIN(213, 213)
#endif

#ifdef USES_P214
  ADDPLUGIN(214, 214)
#endif

#ifdef USES_P215
  ADDPLUGIN(215, 215)
#endif

#ifdef USES_P216
  ADDPLUGIN(216, 216)
#endif

#ifdef USES_P217
  ADDPLUGIN(217, 217)
#endif

#ifdef USES_P218
  ADDPLUGIN(218, 218)
#endif

#ifdef USES_P219
  ADDPLUGIN(219, 219)
#endif

#ifdef USES_P220
  ADDPLUGIN(220, 220)
#endif

#ifdef USES_P221
  ADDPLUGIN(221, 221)
#endif

#ifdef USES_P222
  ADDPLUGIN(222, 222)
#endif

#ifdef USES_P223
  ADDPLUGIN(223, 223)
#endif

#ifdef USES_P224
  ADDPLUGIN(224, 224)
#endif

#ifdef USES_P225
  ADDPLUGIN(225, 225)
#endif

#ifdef USES_P226
  ADDPLUGIN(226, 226)
#endif

#ifdef USES_P227
  ADDPLUGIN(227, 227)
#endif

#ifdef USES_P228
  ADDPLUGIN(228, 228)
#endif

#ifdef USES_P229
  ADDPLUGIN(229, 229)
#endif

#ifdef USES_P230
  ADDPLUGIN(230, 230)
#endif

#ifdef USES_P231
  ADDPLUGIN(231, 231)
#endif

#ifdef USES_P232
  ADDPLUGIN(232, 232)
#endif

#ifdef USES_P233
  ADDPLUGIN(233, 233)
#endif

#ifdef USES_P234
  ADDPLUGIN(234, 234)
#endif

#ifdef USES_P235
  ADDPLUGIN(235, 235)
#endif

#ifdef USES_P236
  ADDPLUGIN(236, 236)
#endif

#ifdef USES_P237
  ADDPLUGIN(237, 237)
#endif

#ifdef USES_P238
  ADDPLUGIN(238, 238)
#endif

#ifdef USES_P239
  ADDPLUGIN(239, 239)
#endif

#ifdef USES_P240
  ADDPLUGIN(240, 240)
#endif

#ifdef USES_P241
  ADDPLUGIN(241, 241)
#endif

#ifdef USES_P242
  ADDPLUGIN(242, 242)
#endif

#ifdef USES_P243
  ADDPLUGIN(243, 243)
#endif

#ifdef USES_P244
  ADDPLUGIN(244, 244)
#endif

#ifdef USES_P245
  ADDPLUGIN(245, 245)
#endif

#ifdef USES_P246
  ADDPLUGIN(246, 246)
#endif

#ifdef USES_P247
  ADDPLUGIN(247, 247)
#endif

#ifdef USES_P248
  ADDPLUGIN(248, 248)
#endif

#ifdef USES_P249
  ADDPLUGIN(249, 249)
#endif

#ifdef USES_P250
  ADDPLUGIN(250, 250)
#endif

#ifdef USES_P251
  ADDPLUGIN(251, 251)
#endif

#ifdef USES_P252
  ADDPLUGIN(252, 252)
#endif

#ifdef USES_P253
  ADDPLUGIN(253, 253)
#endif

#ifdef USES_P254
  ADDPLUGIN(254, 254)
#endif

#ifdef USES_P255
  ADDPLUGIN(255, 255)
#endif

#ifndef BUILD_NO_RAM_TRACKER
  logMemUsageAfter(F("ADDPLUGIN(...)"));
#endif

  String dummy;
  PluginCall(PLUGIN_DEVICE_ADD, nullptr, dummy);
    // Set all not supported plugins to disabled.
  for (taskIndex_t taskIndex = 0; taskIndex < TASKS_MAX; ++taskIndex) {
    if (!supportedPluginID(Settings.TaskDeviceNumber[taskIndex])) {
      Settings.TaskDeviceEnabled[taskIndex] = false;
    }
  }

#ifndef BUILD_NO_RAM_TRACKER
  logMemUsageAfter(F("PLUGIN_DEVICE_ADD"));
#endif

  PluginCall(PLUGIN_INIT_ALL, nullptr, dummy);
#ifndef BUILD_NO_RAM_TRACKER
  logMemUsageAfter(F("PLUGIN_INIT_ALL"));
#endif

  sortDeviceIndexArray(); // Used in device selector dropdown.
}


#include "../Helpers/ESPEasy_time_calc.h"

#include <Arduino.h>
#include <limits.h>

#include "../Globals/ESPEasy_time.h"
#include "../Helpers/StringConverter.h"


#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)


bool isLeapYear(int year) {
  return ((year > 0) && !(year % 4) && ((year % 100) || !(year % 400)));
}

uint8_t getMonthDays(int year, uint8_t month) {
  const uint8_t monthDays[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  if (month == 1 && isLeapYear(year)) {
    return 29;
  }
  if (month > 11) {
    return 0;
  }
  return monthDays[month];
}

/********************************************************************************************\
   Unix Time computations
 \*********************************************************************************************/

uint32_t makeTime(const struct tm& tm) {
  // assemble time elements into uint32_t
  // note year argument is offset from 1970 (see macros in time.h to convert to other formats)
  // previous version used full four digit year (or digits since 2000),i.e. 2009 was 2009 or 9
  const int tm_year = tm.tm_year + 1900;

  // seconds from 1970 till 1 jan 00:00:00 of the given year
  // tm_year starts at 1900
  uint32_t seconds = 1577836800; // 01/01/2020 @ 12:00am (UTC)
  int year = 2020;
  if (tm_year < year) {
    // Just in case this function is called on old dates
    year = 1970;
    seconds = 0;
  }

  for (; year < tm_year; ++year) {
    seconds += SECS_PER_DAY * 365;
    if (isLeapYear(year)) {
      seconds += SECS_PER_DAY; // add extra days for leap years
    }
  }

  // add days for this year, months start from 0
  for (int i = 0; i < tm.tm_mon; i++) {
    seconds += SECS_PER_DAY * getMonthDays(tm_year, i);
  }
  seconds += (tm.tm_mday - 1) * SECS_PER_DAY;
  seconds += tm.tm_hour * SECS_PER_HOUR;
  seconds += tm.tm_min * SECS_PER_MIN;
  seconds += tm.tm_sec;
  return seconds;
}


/********************************************************************************************\
   Time computations for rules.
 \*********************************************************************************************/

String timeLong2String(unsigned long lngTime)
{
  unsigned long x = 0;
  String time;

  x = (lngTime >> 16) & 0xf;

  if (x == 0x0f) {
    x = 0;
  }
  String weekDays = F("AllSunMonTueWedThuFriSatWrkWkd");
  time  = weekDays.substring(x * 3, x * 3 + 3);
  time += ',';

  x = (lngTime >> 12) & 0xf;

  if (x == 0xf) {
    time += '*';
  }
  else if (x == 0xe) {
    time += '-';
  }
  else {
    time += x;
  }

  x = (lngTime >> 8) & 0xf;

  if (x == 0xf) {
    time += '*';
  }
  else if (x == 0xe) {
    time += '-';
  }
  else {
    time += x;
  }

  time += ':';

  x = (lngTime >> 4) & 0xf;

  if (x == 0xf) {
    time += '*';
  }
  else if (x == 0xe) {
    time += '-';
  }
  else {
    time += x;
  }

  x = (lngTime) & 0xf;

  if (x == 0xf) {
    time += '*';
  }
  else if (x == 0xe) {
    time += '-';
  }
  else {
    time += x;
  }

  return time;
}


unsigned long string2TimeLong(const String& str)
{
  // format 0000WWWWAAAABBBBCCCCDDDD
  // WWWW=weekday, AAAA=hours tens digit, BBBB=hours, CCCC=minutes tens digit DDDD=minutes

  char command[20];
  int  w, x, y;
  unsigned long a;
  {
    // Within a scope so the tmpString is only used for copy.
    String tmpString(str);
    tmpString.toLowerCase();
    tmpString.toCharArray(command, 20);
  }
  unsigned long lngTime = 0;
  String TmpStr1;

  if (GetArgv(command, TmpStr1, 1))
  {
    String day      = TmpStr1;
    String weekDays = F("allsunmontuewedthufrisatwrkwkd");
    y = weekDays.indexOf(TmpStr1) / 3;

    if (y == 0) {
      y = 0xf; // wildcard is 0xf
    }
    lngTime |= (unsigned long)y << 16;
  }

  if (GetArgv(command, TmpStr1, 2))
  {
    y = 0;

    for (x = TmpStr1.length() - 1; x >= 0; x--)
    {
      w = TmpStr1[x];

      if (isDigit(w) || (w == '*'))
      {
        a        = 0xffffffff  ^ (0xfUL << y); // create mask to clean nibble position y
        lngTime &= a;                          // maak nibble leeg

        if (w == '*') {
          lngTime |= (0xFUL << y);             // fill nibble with wildcard value
        }
        else {
          lngTime |= (w - '0') << y;           // fill nibble with token
        }
        y += 4;
      }
      else
      if (w == ':') {}
      else
      {
        break;
      }
    }
  }
  #undef TmpStr1Length
  return lngTime;
}



/********************************************************************************************\
   Match clock event
 \*********************************************************************************************/
bool matchClockEvent(unsigned long clockEvent, unsigned long clockSet)
{
  unsigned long Mask;

  for (uint8_t y = 0; y < 8; y++)
  {
    if (((clockSet >> (y * 4)) & 0xf) == 0xf)         // if nibble y has the wildcard value 0xf
    {
      Mask        = 0xffffffff  ^ (0xFUL << (y * 4)); // Mask to wipe nibble position y.
      clockEvent &= Mask;                             // clear nibble
      clockEvent |= (0xFUL << (y * 4));               // fill with wildcard value 0xf
    }
  }

  if (((clockSet >> (16)) & 0xf) == 0x8) {         // if weekday nibble has the wildcard value 0x8 (workdays)
    if (node_time.weekday() >= 2 and node_time.weekday() <= 6)         // and we have a working day today...
    {
      Mask        = 0xffffffff  ^ (0xFUL << (16)); // Mask to wipe nibble position.
      clockEvent &= Mask;                          // clear nibble
      clockEvent |= (0x8UL << (16));               // fill with wildcard value 0x8
    }
  }

  if (((clockSet >> (16)) & 0xf) == 0x9) {         // if weekday nibble has the wildcard value 0x9 (weekends)
    if (node_time.weekday() == 1 or node_time.weekday() == 7)          // and we have a weekend day today...
    {
      Mask        = 0xffffffff  ^ (0xFUL << (16)); // Mask to wipe nibble position.
      clockEvent &= Mask;                          // clear nibble
      clockEvent |= (0x9UL << (16));               // fill with wildcard value 0x9
    }
  }

  if (clockEvent == clockSet) {
    return true;
  }
  return false;
}

#include "../Helpers/_CPlugin_LoRa_TTN_helper.h"

#include "../DataStructs/ESPEasy_EventStruct.h"

#include "../Globals/Settings.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../../_Plugin_Helper.h"

// #######################################################################################################
// #  Helper functions to encode data for use on LoRa/TTN network.
// #######################################################################################################

#if FEATURE_PACKED_RAW_DATA


String getPackedFromPlugin(struct EventStruct *event, uint8_t sampleSetCount)
{
  uint8_t   value_count = getValueCountForTask(event->TaskIndex);
  String raw_packed;

  if (PluginCall(PLUGIN_GET_PACKED_RAW_DATA, event, raw_packed)) {
    value_count = event->Par1;
  }
  String packed;
  packed.reserve(32);
  packed += LoRa_addInt(Settings.TaskDeviceNumber[event->TaskIndex], PackedData_uint8);
  packed += LoRa_addInt(event->idx, PackedData_uint16);
  packed += LoRa_addInt(sampleSetCount, PackedData_uint8);
  packed += LoRa_addInt(value_count, PackedData_uint8);
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("packed header: ");
    log += packed;
    if (raw_packed.length() > 0) {
      log += F(" RAW: ");
      log += raw_packed;
    }
    addLogMove(LOG_LEVEL_INFO, log);
  }

  if (raw_packed.length() > 0) {
    packed += raw_packed;
  } else {
    switch (event->getSensorType())
    {
      case Sensor_VType::SENSOR_TYPE_LONG:
      {
        unsigned long longval = UserVar.getSensorTypeLong(event->TaskIndex);
        packed += LoRa_addInt(longval, PackedData_uint32);
        break;
      }

      default:

        for (uint8_t i = 0; i < value_count && i < VARS_PER_TASK; ++i) {
          // For now, just store the floats as an int32 by multiplying the value with 10000.
          packed += LoRa_addFloat(UserVar[event->BaseVarIndex + i], PackedData_int32_1e4);
        }
        break;
    }
  }
  return packed;
}

float getLoRaAirTime(uint8_t pl, uint8_t sf, uint16_t bw, uint8_t cr, uint8_t n_preamble, bool header, bool crc)
{
  if (sf > 12) {
    sf = 12;
  } else if (sf < 7) {
    sf = 7;
  }

  if (cr > 4) {
    cr = 4;
  } else if (cr < 1) {
    cr = 1;
  }

  // Symbols in frame
  int payload_length = 8;
  {
    int beta_offset = 28;

    if (crc) { beta_offset += 16; }

    if (!header) { beta_offset -= 20; }
    float beta_f                  = 8.0f * pl - 4.0f * sf + beta_offset;
    bool  lowDataRateOptimization = (bw == 125 && sf >= 11);

    if (lowDataRateOptimization) {
      beta_f = beta_f / (4.0f * (sf - 2));
    } else {
      beta_f = beta_f / (4.0f * sf);
    }
    int beta = static_cast<int>(beta_f + 1.0f); // ceil

    if (beta > 0) {
      payload_length += (beta * (cr + 4));
    }
  }

  // t_symbol and t_air in msec
  float t_symbol = static_cast<float>(1 << sf) / bw;
  float t_air    = ((n_preamble + 4.25f) + payload_length) * t_symbol;
  return t_air;
}

#endif // if FEATURE_PACKED_RAW_DATA

#include "../Helpers/WiFi_AP_CandidatesList.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/RTC.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/Settings.h"
#include "../Helpers/Misc.h"


#define WIFI_CUSTOM_DEPLOYMENT_KEY_INDEX     3
#define WIFI_CUSTOM_SUPPORT_KEY_INDEX        4
#define WIFI_CREDENTIALS_FALLBACK_SSID_INDEX 5

WiFi_AP_CandidatesList::WiFi_AP_CandidatesList() {
  known.clear();
  candidates.clear();
  known_it = known.begin();
  load_knownCredentials();
}

void WiFi_AP_CandidatesList::load_knownCredentials() {
  if (!_mustLoadCredentials) { return; }
  _mustLoadCredentials = false;
  known.clear();
  candidates.clear();

  {
    // Add the known SSIDs
    String ssid, key;
    uint8_t   index = 1; // Index 0 is the "unset" value

    bool done = false;

    while (!done) {
      if (get_SSID_key(index, ssid, key)) {
        known.emplace_back(index, ssid, key);
        if (SettingsIndexMatchCustomCredentials(index)) {
          if (SettingsIndexMatchEmergencyFallback(index)) {
            known.back().isEmergencyFallback = true;
          } else {
            known.back().lowPriority = true;
          }
        }
        ++index;
      } else {
        if (SettingsIndexMatchCustomCredentials(index)) {
          ++index;
        } else {
          done = true;
        }
      }
    }
  }
  loadCandidatesFromScanned();
  addFromRTC();
}

void WiFi_AP_CandidatesList::clearCache() {
  _mustLoadCredentials = true;
  known.clear();
  known_it = known.begin();
}


void WiFi_AP_CandidatesList::force_reload() {
  clearCache();
  RTC.clearLastWiFi(); // Invalidate the RTC WiFi data.
  candidates.clear();
  loadCandidatesFromScanned();
}

void WiFi_AP_CandidatesList::begin_sync_scan() {
  candidates.clear();
}

void WiFi_AP_CandidatesList::purge_expired() {
  for (auto it = scanned.begin(); it != scanned.end(); ) {
    if (it->expired()) {
      it = scanned.erase(it);
    } else {
      ++it;
    }
  }
}

void WiFi_AP_CandidatesList::process_WiFiscan(uint8_t scancount) {
  // Append or update found APs from scan.
  for (uint8_t i = 0; i < scancount; ++i) {
    const WiFi_AP_Candidate tmp(i);

    scanned_new.push_back(tmp);
  }

  after_process_WiFiscan();
}

#ifdef ESP8266
void WiFi_AP_CandidatesList::process_WiFiscan(const bss_info& ap) {
  WiFi_AP_Candidate tmp(ap);
  scanned_new.push_back(tmp);
}
#endif

void WiFi_AP_CandidatesList::after_process_WiFiscan() {
  scanned_new.sort();
  scanned_new.unique();
  _mustLoadCredentials = true;
  WiFi.scanDelete();
}

bool WiFi_AP_CandidatesList::getNext(bool scanAllowed) {
  load_knownCredentials();

  if (candidates.empty()) { 
    if (scanAllowed) {
      return false;
    }
    loadCandidatesFromScanned();
    if (candidates.empty()) { return false; }
  }

  bool mustPop = true;

  currentCandidate = candidates.front();

  if (currentCandidate.isHidden) {
    // Iterate over the known credentials to try them all
    // Hidden SSID stations do not broadcast their SSID, so we must fill it in ourselves.
    if (known_it != known.end()) {
      currentCandidate.ssid  = known_it->ssid;
      currentCandidate.key   = known_it->key;
      currentCandidate.index = known_it->index;
      ++known_it;
    }

    if (known_it != known.end()) {
      mustPop = false;
    }
  }

  if (mustPop) {
    if (currentCandidate.isHidden) {
      // We tried to connect to hidden SSIDs in 1 run, so pop all hidden candidates.
      for (auto cand_it = candidates.begin(); cand_it != candidates.end() && cand_it->isHidden; ) {
        cand_it = candidates.erase(cand_it);
      }
    } else {
      if (!candidates.empty()) {
        candidates.pop_front();
      }
    }

    known_it = known.begin();
  }
  return currentCandidate.usable();
}

const WiFi_AP_Candidate& WiFi_AP_CandidatesList::getCurrent() const {
  return currentCandidate;
}

WiFi_AP_Candidate WiFi_AP_CandidatesList::getBestCandidate() const {
  for (auto it = candidates.begin(); it != candidates.end(); ++it) {
    if (it->rssi < -1) { return *it; }
  }
  return WiFi_AP_Candidate();
}

bool WiFi_AP_CandidatesList::hasKnownCredentials() {
  load_knownCredentials();
  return !known.empty();
}

void WiFi_AP_CandidatesList::markCurrentConnectionStable() {
  clearCache();
  if (currentCandidate.enc_type == 0) {
    bool matchfound = false;
    for (auto it = candidates.begin(); !matchfound && it != candidates.end(); ++it) {
      if (currentCandidate == *it) {
        // We may have gotten the enc_type of the active used candidate
        // Make sure to store the enc type before clearing the candidates list
        currentCandidate.enc_type = it->enc_type;
        matchfound = true;
      }
    }
  }
  if (currentCandidate.usable()) {
    // Store in RTC
    RTC.lastWiFiChannel = currentCandidate.channel;
    currentCandidate.bssid.get(RTC.lastBSSID);
    RTC.lastWiFiSettingsIndex = currentCandidate.index;
  }

  candidates.clear();
  addFromRTC(); // Store the current one from RTC as the first candidate for a reconnect.
}

int8_t WiFi_AP_CandidatesList::scanComplete() const {
  size_t found = 0;
  for (auto scan = scanned.begin(); scan != scanned.end(); ++scan) {
    if (!scan->expired()) {
      ++found;
    }
  }
  if (found > 0) {    
    return found;
  }
  const int8_t scanCompleteStatus = WiFi.scanComplete();
  if (scanCompleteStatus <= 0) {
    return scanCompleteStatus;
  }
  return 0;
}

bool WiFi_AP_CandidatesList::SettingsIndexMatchCustomCredentials(uint8_t index)
{
  return (WIFI_CUSTOM_DEPLOYMENT_KEY_INDEX     == index ||
          WIFI_CUSTOM_SUPPORT_KEY_INDEX        == index ||
          SettingsIndexMatchEmergencyFallback(index));
}

bool WiFi_AP_CandidatesList::SettingsIndexMatchEmergencyFallback(uint8_t index)
{
  return (WIFI_CREDENTIALS_FALLBACK_SSID_INDEX == index);
}


void WiFi_AP_CandidatesList::loadCandidatesFromScanned() {
  if (scanned_new.size() > 0) {
    // We have new scans to process.
    #ifdef USE_SECOND_HEAP
    HeapSelectIram ephemeral;
    // TD-er: Disabled for now as it is suspect for crashes
    #endif
    purge_expired();
    for (auto scan = scanned_new.begin(); scan != scanned_new.end();) {
      #ifndef BUILD_NO_DEBUG
      if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
        String log = F("WiFi : Scan result: ");
        log += scan->toString();
        addLogMove(LOG_LEVEL_DEBUG, log);
      }
      #endif // ifndef BUILD_NO_DEBUG

      // Check to see if it is already present, if so, remove existing one.
      for (auto tmp = scanned.begin(); tmp != scanned.end();) {
        if (*tmp == *scan) {
          tmp = scanned.erase(tmp);
        } else {
          ++tmp;
        }
      }

      // We copy instead of move, to make sure it is stored on the 2nd heap.
      scanned.push_back(*scan);
      scan = scanned_new.erase(scan);
    }
    scanned.sort();
    scanned.unique();
  }

  if (candidates.size() > 1) {
    // Do not mess with the current candidates order if > 1 present
    return;
  }
  // Purge unusable from known list.
  for (auto it = known.begin(); it != known.end();) {
    if (it->usable()) {
      ++it;
    } else {
      it = known.erase(it);
    }
  }
  known.sort();
  known.unique();
  known_it = known.begin();

  for (auto scan = scanned.begin(); scan != scanned.end();) {
    if (scan->expired()) {
      scan = scanned.erase(scan);
    } else {
      if (scan->isHidden) {
        if (Settings.IncludeHiddenSSID()) {
          if (SecuritySettings.hasWiFiCredentials()) {
            candidates.push_back(*scan);
          }
        }
      } else if (scan->ssid.length() > 0) {
        for (auto kn_it = known.begin(); kn_it != known.end(); ++kn_it) {
          if (scan->ssid.equals(kn_it->ssid)) {
            WiFi_AP_Candidate tmp = *scan;
            tmp.key   = kn_it->key;
            tmp.index = kn_it->index;
            tmp.lowPriority = kn_it->lowPriority;
            tmp.isEmergencyFallback = kn_it->isEmergencyFallback;

            if (tmp.usable()) {
              candidates.push_back(tmp);

              // Check all knowns as we may have several AP's with the same SSID and different passwords.
            }
          }
        }
      }
      ++scan;
    }
  }
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    const WiFi_AP_Candidate bestCandidate = getBestCandidate();
    if (bestCandidate.usable()) {
      String log = F("WiFi : Best AP candidate: ");
      log += bestCandidate.toString();
      addLogMove(LOG_LEVEL_INFO, log);
    }
  }
  candidates.sort();
  candidates.unique();
  addFromRTC();
  purge_unusable();
}

void WiFi_AP_CandidatesList::addFromRTC() {
  if (!Settings.UseLastWiFiFromRTC() || !RTC.lastWiFi_set()) { return; }

  if (SettingsIndexMatchCustomCredentials(RTC.lastWiFiSettingsIndex)) 
  { 
    return;
  }

  String ssid, key;

  if (!get_SSID_key(RTC.lastWiFiSettingsIndex, ssid, key)) {
    return;
  }

  WiFi_AP_Candidate fromRTC(RTC.lastWiFiSettingsIndex, ssid, key);
  fromRTC.bssid   = RTC.lastBSSID;
  fromRTC.channel = RTC.lastWiFiChannel;

  if (!fromRTC.usable()) {
    return;
  }

  if (candidates.size() > 0 && candidates.front().ssid.equals(fromRTC.ssid)) {
    // Front candidate was already from RTC.
    candidates.pop_front();
  }

  // See if we may have a better candidate for the current network, with a significant better RSSI.
  auto bestMatch = candidates.end();
  auto lastUsed  = bestMatch;
  for (auto it = candidates.begin(); lastUsed == candidates.end() && it != candidates.end(); ++it) {
    if (it->usable() && it->ssid.equals(fromRTC.ssid)) {
      const bool foundLastUsed = fromRTC.bssid_match(it->bssid);
      if (foundLastUsed) {
        lastUsed = it;
      } else if (bestMatch == candidates.end()) {
        bestMatch = it;
      }
    }
  }
  bool matchAdded = false;
  if (bestMatch != candidates.end()) {
    // Found a best match, possibly better than the last used.
    if (lastUsed == candidates.end() || (bestMatch->rssi > (lastUsed->rssi + 10))) {
      // Last used was not found or
      // Other candidate has significant better RSSI
      matchAdded = true;
      candidates.push_front(*bestMatch);
    }
  } else if (lastUsed != candidates.end()) {
    matchAdded = true;
    candidates.push_front(*lastUsed);
  }
  if (!matchAdded) {
    candidates.push_front(fromRTC);
    // This is not taken from a scan, so no idea of the used encryption.
    // Try to find a matching BSSID to get the encryption.
    for (auto it = candidates.begin(); it != candidates.end(); ++it) {
      if ((it->rssi != -1) && candidates.front() == *it) {
        candidates.front().enc_type = it->enc_type;
        return;
      }
    }
  }

  candidates.front().rssi = -1; // Set to best possible RSSI so it is tried first.

  if (!candidates.front().usable() || !candidates.front().allowQuickConnect()) {
    candidates.pop_front();
    return;
  }

  if (currentCandidate == candidates.front()) {
    candidates.front().enc_type = currentCandidate.enc_type;
  }
}

void WiFi_AP_CandidatesList::purge_unusable() {
  for (auto it = candidates.begin(); it != candidates.end();) {
    if (it->usable()) {
      ++it;
    } else {
      it = candidates.erase(it);
    }
  }
  candidates.sort();
  candidates.unique();
}

bool WiFi_AP_CandidatesList::get_SSID_key(uint8_t index, String& ssid, String& key) const {
  switch (index) {
    case 1:
      ssid = SecuritySettings.WifiSSID;
      key  = SecuritySettings.WifiKey;
      break;
    case 2:
      ssid = SecuritySettings.WifiSSID2;
      key  = SecuritySettings.WifiKey2;
      break;
    case WIFI_CUSTOM_DEPLOYMENT_KEY_INDEX:
      #if !defined(CUSTOM_DEPLOYMENT_SSID) || !defined(CUSTOM_DEPLOYMENT_KEY)
      return false;
      #else
      ssid = F(CUSTOM_DEPLOYMENT_SSID);
      key  = F(CUSTOM_DEPLOYMENT_KEY);
      #endif
      break;
    case WIFI_CUSTOM_SUPPORT_KEY_INDEX:
      #if !defined(CUSTOM_SUPPORT_SSID) || !defined(CUSTOM_SUPPORT_KEY)
      return false;
      #else
      ssid = F(CUSTOM_SUPPORT_SSID);
      key  = F(CUSTOM_SUPPORT_KEY);
      #endif
      break;
    case WIFI_CREDENTIALS_FALLBACK_SSID_INDEX:
    {
      #if !defined(CUSTOM_EMERGENCY_FALLBACK_SSID) || !defined(CUSTOM_EMERGENCY_FALLBACK_KEY)
      return false;
      #else
      int allowedUptimeMinutes = 10;
      #ifdef CUSTOM_EMERGENCY_FALLBACK_ALLOW_MINUTES_UPTIME
      allowedUptimeMinutes = CUSTOM_EMERGENCY_FALLBACK_ALLOW_MINUTES_UPTIME;
      #endif
      if (getUptimeMinutes() < allowedUptimeMinutes && SecuritySettings.hasWiFiCredentials()) {
        ssid = F(CUSTOM_EMERGENCY_FALLBACK_SSID);
        key  = F(CUSTOM_EMERGENCY_FALLBACK_KEY);
      } else {
        return false;
      }
      #endif
      break;
    }
    default:
      return false;
  }

  // TODO TD-er: Read other credentials from extra file.



  // Spaces are allowed in both SSID and pass phrase, so make sure to not trim the ssid and key.
  return true;
}

#include "../Helpers/StringParser.h"

#include "../../ESPEasy_common.h"

#include "../../_Plugin_Helper.h"

#include "../Commands/GPIO.h"

#include "../DataStructs/TimingStats.h"

#include "../ESPEasyCore/ESPEasyRules.h"

#include "../Globals/Cache.h"
#include "../Globals/Plugins_other.h"
#include "../Globals/RuntimeData.h"

#include "../Helpers/ESPEasy_math.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/Rules_calculate.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringGenerator_GPIO.h"

#include <Arduino.h>

/********************************************************************************************\
   Parse string template
 \*********************************************************************************************/
String parseTemplate(String& tmpString)
{
  return parseTemplate(tmpString, false);
}

String parseTemplate(String& tmpString, bool useURLencode)
{
  return parseTemplate_padded(tmpString, 0, useURLencode);
}

String parseTemplate_padded(String& tmpString, uint8_t minimal_lineSize)
{
  return parseTemplate_padded(tmpString, minimal_lineSize, false);
}

String parseTemplate_padded(String& tmpString, uint8_t minimal_lineSize, bool useURLencode)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("parseTemplate_padded"));
  #endif // ifndef BUILD_NO_RAM_TRACKER
  START_TIMER;

  // Keep current loaded taskSettings to restore at the end.
  uint8_t   currentTaskIndex = ExtraTaskSettings.TaskIndex;
  String newString;

  newString.reserve(minimal_lineSize); // Our best guess of the new size.


  if (parseTemplate_CallBack_ptr != nullptr) {
    parseTemplate_CallBack_ptr(tmpString, useURLencode);
  }
  parseSystemVariables(tmpString, useURLencode);


  int startpos = 0;
  int lastStartpos = 0;
  int endpos = 0;
  {
    String deviceName, valueName, format;

    while (findNextDevValNameInString(tmpString, startpos, endpos, deviceName, valueName, format)) {
      // First copy all upto the start of the [...#...] part to be replaced.
      newString += tmpString.substring(lastStartpos, startpos);

      // deviceName is lower case, so we can compare literal string (no need for equalsIgnoreCase)
      const bool devNameEqInt = deviceName.equals(F("int"));
      if (devNameEqInt || deviceName.equals(F("var")))
      {
        // Address an internal variable either as float or as int
        // For example: Let,10,[VAR#9]
        unsigned int varNum;

        if (validUIntFromString(valueName, varNum)) {
          unsigned char nr_decimals = maxNrDecimals_double(getCustomFloatVar(varNum));
          bool trimTrailingZeros    = true;

          if (devNameEqInt) {
            nr_decimals = 0;
          } else if (!format.isEmpty())
          {
            // There is some formatting here, so do not throw away decimals
            trimTrailingZeros = false;
          }
          String value = doubleToString(getCustomFloatVar(varNum), nr_decimals, trimTrailingZeros);
          transformValue(
            newString, 
            minimal_lineSize, 
            std::move(value), 
            format, 
            tmpString);
        }
      }
      else if (deviceName.equals(F("plugin")))
      {
        // Handle a plugin request.
        // For example: "[Plugin#GPIO#Pinstate#N]"
        // The command is stored in valueName & format
        String command;
        command.reserve(valueName.length() + format.length() + 1);
        command  = valueName;
        command += '#';
        command += format;
        command.replace('#', ',');

        if (getGPIOPinStateValues(command)) {
          newString += command;
        }
  /* @giig1967g
        if (PluginCall(PLUGIN_REQUEST, 0, command))
        {
          // Do not call transformValue here.
          // The "format" is not empty so must not call the formatter function.
          newString += command;
        }
  */
      }
      else
      {
        // Address a value from a plugin.
        // For example: "[bme#temp]"
        // If value name is unknown, run a PLUGIN_GET_CONFIG_VALUE command.
        // For example: "[<taskname>#getLevel]"
        taskIndex_t taskIndex = findTaskIndexByName(deviceName);

        if (validTaskIndex(taskIndex) && Settings.TaskDeviceEnabled[taskIndex]) {
          uint8_t valueNr = findDeviceValueIndexByName(valueName, taskIndex);

          if (valueNr != VARS_PER_TASK) {
            // here we know the task and value, so find the uservar
            // Try to format and transform the values
            bool   isvalid;
            String value = formatUserVar(taskIndex, valueNr, isvalid);

            if (isvalid) {
              transformValue(newString, minimal_lineSize, std::move(value), format, tmpString);
            }
          } else {
            // try if this is a get config request
            struct EventStruct TempEvent(taskIndex);
            String tmpName = valueName;

            if (PluginCall(PLUGIN_GET_CONFIG_VALUE, &TempEvent, tmpName))
            {
              transformValue(newString, minimal_lineSize, std::move(tmpName), format, tmpString);
            }
          }
        }
      }


      // Conversion is done (or impossible) for the found "[...#...]"
      // Continue with the next one.
      lastStartpos = endpos + 1;
      startpos     = endpos + 1;

      // This may have taken some time, so call delay()
      delay(0);
    }
  }

  // Copy the rest of the string (or all if no replacements were done)
  newString += tmpString.substring(lastStartpos);
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("parseTemplate2"));
  #endif // ifndef BUILD_NO_RAM_TRACKER

  // Restore previous loaded taskSettings
  if (currentTaskIndex != 255)
  {
    LoadTaskSettings(currentTaskIndex);
  }

  parseStandardConversions(newString, useURLencode);

  // process other markups as well
  parse_string_commands(newString);

  // padding spaces
  while (newString.length() < minimal_lineSize) {
    newString += ' ';
  }

  STOP_TIMER(PARSE_TEMPLATE_PADDED);
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("parseTemplate3"));
  #endif // ifndef BUILD_NO_RAM_TRACKER
  return newString;
}

/********************************************************************************************\
   Transform values
 \*********************************************************************************************/

// Syntax: [task#value#transformation#justification]
// valueFormat="transformation#justification"
void transformValue(
  String      & newString,
  uint8_t          lineSize,
  String        value,
  String      & valueFormat,
  const String& tmpString)
{
  // FIXME TD-er: This function does append to newString and uses its length to perform right aling.
  // Is this the way it is intended to use?
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("transformValue"));
  #endif // ifndef BUILD_NO_RAM_TRACKER

  // start changes by giig1967g - 2018-04-20
  // Syntax: [task#value#transformation#justification]
  // valueFormat="transformation#justification"
  if (valueFormat.length() > 0) // do the checks only if a Format is defined to optimize loop
  {
    String valueJust;

    int hashtagIndex = valueFormat.indexOf('#');

    if (hashtagIndex >= 0)
    {
      valueJust   = valueFormat.substring(hashtagIndex + 1); // Justification part
      valueFormat = valueFormat.substring(0, hashtagIndex);  // Transformation part
    }

    // valueFormat="transformation"
    // valueJust="justification"
    if (valueFormat.length() > 0) // do the checks only if a Format is defined to optimize loop
    {
      int logicVal    = 0;
      double valFloat = 0.0;

      if (validDoubleFromString(value, valFloat))
      {
        // to be used for binary values (0 or 1)
        logicVal = static_cast<int>(roundf(valFloat)) == 0 ? 0 : 1;
      } else {
        if (value.length() > 0) {
          logicVal = 1;
        }
      }
      String tempValueFormat = valueFormat;
      {
        const int invertedIndex = tempValueFormat.indexOf('!');

        if (invertedIndex != -1) {
          // We must invert the value.
          logicVal = (logicVal == 0) ? 1 : 0;

          // Remove the '!' from the string.
          tempValueFormat.remove(invertedIndex, 1);
        }
      }

      const int  rightJustifyIndex = tempValueFormat.indexOf('R');
      const bool rightJustify      = rightJustifyIndex >= 0 ? 1 : 0;

      if (rightJustify) {
        tempValueFormat.remove(rightJustifyIndex, 1);
      }

      const int tempValueFormatLength = tempValueFormat.length();

      // Check Transformation syntax
      if (tempValueFormatLength > 0)
      {
        switch (tempValueFormat[0])
        {
          case 'V': // value = value without transformations
            break;
          case 'p': // Password hide using asterisks or custom character: pc
          {
            char maskChar = '*';

            if (tempValueFormatLength > 1)
            {
              maskChar = tempValueFormat[1];
            }

            if (value == F("0")) {
              value = String();
            } else {
              const int valueLength = value.length();

              for (int i = 0; i < valueLength; i++) {
                value[i] = maskChar;
              }
            }
            break;
          }
          case 'O':
            value = logicVal == 0 ? F("OFF") : F(" ON"); // (equivalent to XOR operator)
            break;
          case 'C':
            value = logicVal == 0 ? F("CLOSE") : F(" OPEN");
            break;
          case 'c':
            value = logicVal == 0 ? F("CLOSED") : F("  OPEN");
            break;
          case 'M':
            value = logicVal == 0 ? F("AUTO") : F(" MAN");
            break;
          case 'm':
            value = logicVal == 0 ? 'A' : 'M';
            break;
          case 'H':
            value = logicVal == 0 ? F("COLD") : F(" HOT");
            break;
          case 'U':
            value = logicVal == 0 ? F("DOWN") : F("  UP");
            break;
          case 'u':
            value = logicVal == 0 ? 'D' : 'U';
            break;
          case 'Y':
            value = logicVal == 0 ? F(" NO") : F("YES");
            break;
          case 'y':
            value = logicVal == 0 ? 'N' : 'Y';
            break;
          case 'X':
            value = logicVal == 0 ? 'O' : 'X';
            break;
          case 'I':
            value = logicVal == 0 ? F("OUT") : F(" IN");
            break;
          case 'L':
            value = logicVal == 0 ? F(" LEFT") : F("RIGHT");
            break;
          case 'l':
            value = logicVal == 0 ? 'L' : 'R';
            break;
          case 'Z': // return "0" or "1"
            value = logicVal == 0 ? '0' : '1';
            break;
          case 'D': // Dx.y min 'x' digits zero filled & 'y' decimal fixed digits
          case 'd': // like above but with spaces padding
          {
            int x;
            int y;
            x = 0;
            y = 0;

            switch (tempValueFormatLength)
            {
              case 2: // Dx

                if (isDigit(tempValueFormat[1]))
                {
                  x = static_cast<int>(tempValueFormat[1]) - '0';
                }
                break;
              case 3: // D.y

                if ((tempValueFormat[1] == '.') && isDigit(tempValueFormat[2]))
                {
                  y = static_cast<int>(tempValueFormat[2]) - '0';
                }
                break;
              case 4: // Dx.y

                if (isDigit(tempValueFormat[1]) && (tempValueFormat[2] == '.') && isDigit(tempValueFormat[3]))
                {
                  x = static_cast<int>(tempValueFormat[1]) - '0';
                  y = static_cast<int>(tempValueFormat[3]) - '0';
                }
                break;
              case 1:  // D
              default: // any other combination x=0; y=0;
                break;
            }
            bool trimTrailingZeros = false;
            value = doubleToString(valFloat, y, trimTrailingZeros);
            int indexDot = value.indexOf('.');

            if (indexDot == -1) {
              indexDot = value.length();
            }

            for (uint8_t f = 0; f < (x - indexDot); f++) {
              value = (tempValueFormat[0] == 'd' ? ' ' : '0') + value;
            }
            break;
          }
          case 'F': // FLOOR (round down)
            value = static_cast<int>(floorf(valFloat));
            break;
          case 'E': // CEILING (round up)
            value = static_cast<int>(ceilf(valFloat));
            break;
          default:
            value = F("ERR");
            break;
        }

        // Check Justification syntax
        const int valueJustLength = valueJust.length();

        if (valueJustLength > 0) // do the checks only if a Justification is defined to optimize loop
        {
          value.trim();          // remove right justification spaces for backward compatibility

          switch (valueJust[0])
          {
            case 'P': // Prefix Fill with n spaces: Pn

              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1]))                          // Check Pn where n is between 0 and 9
                {
                  int filler = valueJust[1] - value.length() - '0'; // char '0' = 48; char '9' = 58

                  for (uint8_t f = 0; f < filler; f++) {
                    newString += ' ';
                  }
                }
              }
              break;
            case 'S': // Suffix Fill with n spaces: Sn

              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1]))                          // Check Sn where n is between 0 and 9
                {
                  int filler = valueJust[1] - value.length() - '0'; // 48

                  for (uint8_t f = 0; f < filler; f++) {
                    value += ' ';
                  }
                }
              }
              break;
            case 'L': // left part of the string

              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1])) // Check n where n is between 0 and 9
                {
                  value = value.substring(0, static_cast<int>(valueJust[1]) - '0');
                }
              }
              break;
            case 'R': // Right part of the string

              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1])) // Check n where n is between 0 and 9
                {
                  value = value.substring(std::max(0, static_cast<int>(value.length()) - (static_cast<int>(valueJust[1]) - '0')));
                }
              }
              break;
            case 'U': // Substring Ux.y where x=firstChar and y=number of characters

              if (valueJustLength > 1)
              {
                if (isDigit(valueJust[1]) && (valueJust[2] == '.') && isDigit(valueJust[3]) && (valueJust[1] > '0') && (valueJust[3] > '0'))
                {
                  value = value.substring(std::min(static_cast<int>(value.length()), static_cast<int>(valueJust[1]) - '0' - 1),
                                          static_cast<int>(valueJust[1]) - '0' - 1 + static_cast<int>(valueJust[3]) - '0');
                }
                else
                {
                  newString += F("ERR");
                }
              }
              break;
            case 'C': // Capitalize First Word-Character value (space/period are checked)

              if (value.length() > 0) {
                value.toLowerCase();
                bool nextCapital = true;

                for (uint8_t i = 0; i < value.length(); i++) {
                  if (nextCapital) {
                    value[i] = toupper(value[i]);
                  }
                  nextCapital = (value[i] == ' ' || value[i] == '.'); // Very simple, capitalize-first-after-space/period
                }
              }
              break;
            case 'u': // Uppercase
              value.toUpperCase();
              break;
            case 'l': // Lowercase
              value.toLowerCase();
              break;
            default:
              newString += F("ERR");
              break;
          }
        }
      }

      if (rightJustify)
      {
        int filler = lineSize - newString.length() - value.length() - tmpString.length();

        for (uint8_t f = 0; f < filler; f++) {
          newString += ' ';
        }
      }
      {
#ifndef BUILD_NO_DEBUG

        if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
          String logFormatted = F("DEBUG: Formatted String='");
          logFormatted += newString;
          logFormatted += value;
          logFormatted += '\'';
          addLogMove(LOG_LEVEL_DEBUG, logFormatted);
        }
#endif // ifndef BUILD_NO_DEBUG
      }
    }
  }

  // end of changes by giig1967g - 2018-04-18

  newString += value;
  {
#ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
      String logParsed = F("DEBUG DEV: Parsed String='");
      logParsed += newString;
      logParsed += '\'';
      addLogMove(LOG_LEVEL_DEBUG_DEV, logParsed);
    }
#endif // ifndef BUILD_NO_DEBUG
  }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("transformValue2"));
  #endif // ifndef BUILD_NO_RAM_TRACKER
}

// Find the first (enabled) task with given name
// Return INVALID_TASK_INDEX when not found, else return taskIndex
taskIndex_t findTaskIndexByName(const String& deviceName)
{
  // cache this, since LoadTaskSettings does take some time.
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  auto result = Cache.taskIndexName.find(String(deviceName));
  #else
  auto result = Cache.taskIndexName.find(deviceName);
  #endif


  if (result != Cache.taskIndexName.end()) {
    return result->second;
  }

  for (taskIndex_t taskIndex = 0; taskIndex < TASKS_MAX; taskIndex++)
  {
    if (Settings.TaskDeviceEnabled[taskIndex]) {
      String taskDeviceName = getTaskDeviceName(taskIndex);

      if (!taskDeviceName.isEmpty())
      {
        // Use entered taskDeviceName can have any case, so compare case insensitive.
        if (deviceName.equalsIgnoreCase(taskDeviceName))
        {
          #ifdef USE_SECOND_HEAP
          Cache.taskIndexName[String(deviceName)] = taskIndex;
          #else
          Cache.taskIndexName[deviceName] = taskIndex;
          #endif
          return taskIndex;
        }
      }
    }
  }
  return INVALID_TASK_INDEX;
}

// Find the first device value index of a taskIndex.
// Return VARS_PER_TASK if none found.
uint8_t findDeviceValueIndexByName(const String& valueName, taskIndex_t taskIndex)
{
  const deviceIndex_t deviceIndex = getDeviceIndex_from_TaskIndex(taskIndex);

  if (!validDeviceIndex(deviceIndex)) { return VARS_PER_TASK; }

  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif


  // cache this, since LoadTaskSettings does take some time.
  // We need to use a cache search key including the taskIndex,
  // to allow several tasks to have the same value names.
  String cache_valueName;

  cache_valueName.reserve(valueName.length() + 4);
  cache_valueName  = valueName;
  cache_valueName += '#';        // The '#' cannot exist in a value name, use it in the cache key.
  cache_valueName += taskIndex;
  cache_valueName.toLowerCase(); // No need to store multiple versions of the same entry with only different case.

  auto result = Cache.taskIndexValueName.find(cache_valueName);

  if (result != Cache.taskIndexValueName.end()) {
    return result->second;
  }
  const uint8_t valCount = getValueCountForTask(taskIndex);

  for (uint8_t valueNr = 0; valueNr < valCount; valueNr++)
  {
    // Check case insensitive, since the user entered value name can have any case.
    if (valueName.equalsIgnoreCase(getTaskValueName(taskIndex, valueNr)))
    {
      Cache.taskIndexValueName[cache_valueName] = valueNr;
      return valueNr;
    }
  }
  return VARS_PER_TASK;
}

// Find positions of [...#...] in the given string.
// Only update pos values on success.
// Return true when found.
bool findNextValMarkInString(const String& input, int& startpos, int& hashpos, int& endpos) {
  int tmpStartpos = input.indexOf('[', startpos);

  if (tmpStartpos == -1) { return false; }
  const int tmpHashpos = input.indexOf('#', tmpStartpos);

  if (tmpHashpos == -1) { return false; }

  // We found a hash position, check if there is another '[' inbetween.
  for (int i = tmpStartpos; i < tmpHashpos; ++i) {
    if (input[i] == '[') {
      tmpStartpos = i;
    }
  }

  const int tmpEndpos = input.indexOf(']', tmpStartpos);

  if (tmpEndpos == -1) { return false; }

  if (tmpHashpos >= tmpEndpos) {
    return false;
  }

  hashpos  = tmpHashpos;
  startpos = tmpStartpos;
  endpos   = tmpEndpos;
  return true;
}

// Find [deviceName#valueName] or [deviceName#valueName#format]
// DeviceName and valueName will be returned in lower case.
// Format may contain case sensitive formatting syntax.
bool findNextDevValNameInString(const String& input, int& startpos, int& endpos, String& deviceName, String& valueName, String& format) {
  int hashpos;

  if (!findNextValMarkInString(input, startpos, hashpos, endpos)) { return false; }
  deviceName = input.substring(startpos + 1, hashpos);
  valueName  = input.substring(hashpos + 1, endpos);
  hashpos    = valueName.indexOf('#');

  if (hashpos != -1) {
    // Found an extra '#' in the valueName, will split valueName and format.
    format    = valueName.substring(hashpos + 1);
    valueName = valueName.substring(0, hashpos);
  } else {
    format = String();
  }
  deviceName.toLowerCase();
  valueName.toLowerCase();
  return true;
}

/********************************************************************************************\
   Check to see if a given argument is a valid taskIndex (argc = 0 => command)
 \*********************************************************************************************/
taskIndex_t parseCommandArgumentTaskIndex(const String& string, unsigned int argc)
{
  taskIndex_t taskIndex = INVALID_TASK_INDEX;
  const int   ti        = parseCommandArgumentInt(string, argc);

  if (ti > 0) {
    // Task Index used as argument in commands start at 1.
    taskIndex = static_cast<taskIndex_t>(ti - 1);
  }
  return taskIndex;
}

/********************************************************************************************\
   Get int from command argument (argc = 0 => command)
 \*********************************************************************************************/
int parseCommandArgumentInt(const String& string, unsigned int argc)
{
  int value = 0;

  if (argc > 0) {
    // No need to check for the command (argc == 0)
    String TmpStr;

    if (GetArgv(string.c_str(), TmpStr, argc + 1)) {
      value = CalculateParam(TmpStr);
    }
  }
  return value;
}

/********************************************************************************************\
   Parse a command string to event struct
 \*********************************************************************************************/
void parseCommandString(struct EventStruct *event, const String& string)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("parseCommandString"));
  #endif // ifndef BUILD_NO_RAM_TRACKER
  event->Par1 = parseCommandArgumentInt(string, 1);
  event->Par2 = parseCommandArgumentInt(string, 2);
  event->Par3 = parseCommandArgumentInt(string, 3);
  event->Par4 = parseCommandArgumentInt(string, 4);
  event->Par5 = parseCommandArgumentInt(string, 5);
}

#include "../Helpers/RulesMatcher.h"

#include "../Helpers/ESPEasy_math.h"
#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/StringParser.h"


bool ruleMatch(String event, String rule) {
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("ruleMatch"));
  #endif // ifndef BUILD_NO_RAM_TRACKER

  if (rule.equals("*")) {
    // wildcard, always process
    return true;
  }

  rule.trim();
  parseTemplate(rule);

  event.trim();

  if (event.equalsIgnoreCase(rule)) {
    return true;
  }

  // clock events need different handling...
  if (event.substring(0, 10).equalsIgnoreCase(F("Clock#Time")))
  {
    int pos1 = event.indexOf('=');
    int pos2 = rule.indexOf('=');

    if ((pos1 > 0) && (pos2 > 0)) {
      if (event.substring(0, pos1).equalsIgnoreCase(rule.substring(0, pos2))) // if this is a clock rule
      {
        unsigned long clockEvent = string2TimeLong(event.substring(pos1 + 1));
        unsigned long clockSet   = string2TimeLong(rule.substring(pos2 + 1));

        return matchClockEvent(clockEvent, clockSet);
      }
    } else {
      // Not supported yet, see: https://github.com/letscontrolit/ESPEasy/issues/2640
      return false;
    }
  }

  // Handling wildcard in event
  {
    const int asterisk_pos = rule.indexOf('*');

    if (asterisk_pos != -1) // a * sign in rule, so use a 'wildcard' match on message
    {
      return event.substring(0, asterisk_pos).equalsIgnoreCase(rule.substring(0, asterisk_pos));
    }
  }

  // Special handling of literal string events, they should start with '!'
  if (event.charAt(0) == '!') {
    const bool pound_char_found = rule.indexOf('#') != -1;

    if (!pound_char_found)
    {
      // no # sign in rule, use 'wildcard' match on event 'source'
      return event.substring(0, rule.length()).equalsIgnoreCase(rule);
    }
    return event.equalsIgnoreCase(rule);
  }


  // parse event into verb and value
  double value = 0;
  int    equal_pos   = event.indexOf('=');

  if (equal_pos >= 0) {
    if (!validDoubleFromString(event.substring(equal_pos + 1), value)) {
      return false;

      // FIXME TD-er: What to do when trying to match NaN values?
    }
    event = event.substring(0, equal_pos);
  }

  // parse rule
  int  posStart, posEnd;
  char compare;

  if (!findCompareCondition(rule, compare, posStart, posEnd)) {
    // No compare condition found, so just check if the event- and rule string match.
    return event.equalsIgnoreCase(rule);
  }

  const bool stringMatch = event.equalsIgnoreCase(rule.substring(0, posStart));
  double     ruleValue   = 0;

  if (!validDoubleFromString(rule.substring(posEnd), ruleValue)) {
    return false;

    // FIXME TD-er: What to do when trying to match NaN values?
  }

  bool match = false;

  if (stringMatch) {
    match = compareDoubleValues(compare, value, ruleValue);
  }
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("ruleMatch2"));
  #endif // ifndef BUILD_NO_RAM_TRACKER
  return match;
}

bool compareIntValues(char compare, const int& Value1, const int& Value2)
{
  switch (compare) {
    case '>' + '=': return Value1 >= Value2;
    case '<' + '=': return Value1 <= Value2;
    case '<' + '>': return Value1 != Value2;
    case '>':       return Value1 > Value2;
    case '<':       return Value1 < Value2;
    case '=':       return Value1 == Value2;
  }
  return false;
}

bool compareDoubleValues(char compare, const double& Value1, const double& Value2)
{
  switch (compare) {
    case '>' + '=': return !definitelyLessThan(Value1, Value2);
    case '<' + '=': return !definitelyGreaterThan(Value1, Value2);
    case '<' + '>': return !essentiallyEqual(Value1, Value2);
    case '>':       return definitelyGreaterThan(Value1, Value2);
    case '<':       return definitelyLessThan(Value1, Value2);
    case '=':       return essentiallyEqual(Value1, Value2);
  }
  return false;
}

// Find the compare condition.
// @param posStart = first position of the compare condition in the string
// @param posEnd   = first position rest of the string, right after the compare condition.
bool findCompareCondition(const String& check, char& compare, int& posStart, int& posEnd)
{
  posStart = check.length();
  posEnd   = posStart;
  int  comparePos = 0;
  bool found      = false;

  if (((comparePos = check.indexOf(F("!="))) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd   = posStart + 2;
    compare  = '<' + '>';
    found    = true;
  }

  if (((comparePos = check.indexOf(F("<>"))) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd   = posStart + 2;
    compare  = '<' + '>';
    found    = true;
  }

  if (((comparePos = check.indexOf(F(">="))) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd   = posStart + 2;
    compare  = '>' + '=';
    found    = true;
  }

  if (((comparePos = check.indexOf(F("<="))) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd   = posStart + 2;
    compare  = '<' + '=';
    found    = true;
  }

  if (((comparePos = check.indexOf(F("=="))) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd   = posStart + 2;
    compare  = '=';
    found    = true;
  }

  if (((comparePos = check.indexOf('<')) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd   = posStart + 1;
    compare  = '<';
    found    = true;
  }

  if (((comparePos = check.indexOf('>')) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd   = posStart + 1;
    compare  = '>';
    found    = true;
  }

  if (((comparePos = check.indexOf('=')) > 0) && (comparePos < posStart)) {
    posStart = comparePos;
    posEnd   = posStart + 1;
    compare  = '=';
    found    = true;
  }
  return found;
}

bool getEventFromRulesLine(const String& line, String& event, String& action)
{
  if (line.length() == 0) {
    return false;
  }

  if (!line.substring(0, 3).equalsIgnoreCase(F("on "))) {
    return false;
  }

  // Make sure to skip any number of spaces between 'on' and the event.
  int startpos_event = 2;
  while (static_cast<int>(line.length()) > startpos_event && line[startpos_event] == ' ') {
    ++startpos_event;
  }

  // Need to look for the " do" part starting after "on " (3 chars)
  // and the event needs to be at least 1 char.
  String line_lc = line;

  line_lc.toLowerCase();
  const int pos_do = line_lc.indexOf(F(" do"), startpos_event + 1);

  if (pos_do == -1) {
    return false;
  }

  // event: The part between on ... do
  event = line.substring(startpos_event, pos_do);
  event.trim();

  // Ignore escape char
  event.replace(F("["), EMPTY_STRING);
  event.replace(F("]"), EMPTY_STRING);

  // action: The optional part after the " do"
  action = line.substring(pos_do + 3);
  action.trim();
  // Remove optional "endon"
  if (action.endsWith(F("endon"))) {
    action = action.substring(0, action.lastIndexOf(F("endon")));
  }
  return true;
}

#include "../Helpers/Hardware.h"

#include "../CustomBuild/ESPEasyLimits.h"
#include "../DataTypes/SPI_options.h"
#include "../ESPEasyCore/ESPEasyGPIO.h"
#include "../ESPEasyCore/ESPEasy_Log.h"

#include "../Globals/Device.h"
#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/ExtraTaskSettings.h"
#include "../Globals/Settings.h"
#include "../Globals/Statistics.h"
#include "../Globals/GlobalMapPortStatus.h"

#include "../Helpers/ESPEasy_FactoryDefault.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Misc.h"
#include "../Helpers/PortStatus.h"
#include "../Helpers/StringConverter.h"


// #include "../../ESPEasy-Globals.h"

#ifdef ESP32
  # include <soc/soc.h>
  # include <soc/efuse_reg.h>

  # if ESP_IDF_VERSION_MAJOR > 3      // IDF 4+
    #  if CONFIG_IDF_TARGET_ESP32     // ESP32/PICO-D4
      #   include <esp32/rom/spi_flash.h>
      #   include <esp32/spiram.h>
    #  elif CONFIG_IDF_TARGET_ESP32S2 // ESP32-S2
      #   include <esp32s2/rom/spi_flash.h>
      #   include <esp32s2/spiram.h>
    #  elif CONFIG_IDF_TARGET_ESP32C3 // ESP32-C3
      #   include <esp32c3/rom/spi_flash.h>
      #   include <esp32c3/spiram.h>
    #  else // if CONFIG_IDF_TARGET_ESP32
      #   error Target CONFIG_IDF_TARGET is not supported
    #  endif // if CONFIG_IDF_TARGET_ESP32
  # else // ESP32 Before IDF 4.0
    #  include <rom/spi_flash.h>
  # endif // if ESP_IDF_VERSION_MAJOR > 3

#endif // ifdef ESP32


#if FEATURE_SD
# include <SD.h>
#endif // if FEATURE_SD

/********************************************************************************************\
 * Initialize specific hardware settings (only global ones, others are set through devices)
 \*********************************************************************************************/
void hardwareInit()
{
  // set GPIO pins state if not set to default
  bool hasPullUp, hasPullDown;

  for (int gpio = 0; gpio <= MAX_GPIO; ++gpio) {
    const bool serialPinConflict = (Settings.UseSerial && (gpio == 1 || gpio == 3));

    if (!serialPinConflict) {
      const uint32_t key = createKey(1, gpio);
      #ifdef ESP32
      checkAndClearPWM(key);
      #endif // ifdef ESP32

      if (getGpioPullResistor(gpio, hasPullUp, hasPullDown)) {
        PinBootState bootState = Settings.getPinBootState(gpio);
      #if FEATURE_ETHERNET

        if (Settings.ETH_Pin_power == gpio)
        {
          if (loglevelActiveFor(LOG_LEVEL_INFO)) {
            String log = F("ETH  : Reset ETH module on pin ");
            log += Settings.ETH_Pin_power;
            addLog(LOG_LEVEL_INFO, log);
          }
          bootState = PinBootState::Output_low;
        }

      #endif // if FEATURE_ETHERNET

        if (bootState != PinBootState::Default_state) {
          int8_t  state = -1;
          uint8_t mode  = PIN_MODE_UNDEFINED;
          int8_t  init  = 0;

          switch (bootState)
          {
            case PinBootState::Default_state:
              // At startup, pins are configured as INPUT
              break;
            case PinBootState::Output_low:
              pinMode(gpio, OUTPUT);
              digitalWrite(gpio, LOW);
              state = LOW;
              mode  = PIN_MODE_OUTPUT;
              init  = 1;

              // setPinState(1, gpio, PIN_MODE_OUTPUT, LOW);
              break;
            case PinBootState::Output_high:
              pinMode(gpio, OUTPUT);
              digitalWrite(gpio, HIGH);
              state = HIGH;
              mode  = PIN_MODE_OUTPUT;
              init  = 1;

              // setPinState(1, gpio, PIN_MODE_OUTPUT, HIGH);
              break;
            case PinBootState::Input_pullup:

              if (hasPullUp) {
                pinMode(gpio, INPUT_PULLUP);
                state = 0;
                mode  = PIN_MODE_INPUT_PULLUP;
                init  = 1;
              }
              break;
            case PinBootState::Input_pulldown:

              if (hasPullDown) {
                #ifdef ESP8266

                if (gpio == 16) {
                  pinMode(gpio, INPUT_PULLDOWN_16);
                }
                #endif // ifdef ESP8266
                #ifdef ESP32
                pinMode(gpio, INPUT_PULLDOWN);
                #endif // ifdef ESP32
                state = 0;
                mode  = PIN_MODE_INPUT_PULLDOWN;
                init  = 1;
              }
              break;
            case PinBootState::Input:
              pinMode(gpio, INPUT);
              state = 0;
              mode  = PIN_MODE_INPUT;
              init  = 1;
              break;
          }

          if (init == 1) {
            globalMapPortStatus[key].state = state;
            globalMapPortStatus[key].mode  = mode;
            globalMapPortStatus[key].init  = init;
          }
        }
      }
    }
  }

  if (getGpioPullResistor(Settings.Pin_Reset, hasPullUp, hasPullDown)) {
    if (hasPullUp) {
      pinMode(Settings.Pin_Reset, INPUT_PULLUP);
    }
  }

  initI2C();

  // SPI Init
  if (Settings.isSPI_valid())
  {
    SPI.setHwCs(false);

    // MFD: for ESP32 enable the SPI on HSPI as the default is VSPI
    #ifdef ESP32

    const SPI_Options_e SPI_selection = static_cast<SPI_Options_e>(Settings.InitSPI);

    switch (SPI_selection) {
      case SPI_Options_e::Hspi:
      {
        # define HSPI_MISO   12
        # define HSPI_MOSI   13
        # define HSPI_SCLK   14
        # define HSPI_SS     15
        SPI.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI); // HSPI
        break;
      }
      case SPI_Options_e::UserDefined:
      {
        SPI.begin(Settings.SPI_SCLK_pin,
                  Settings.SPI_MISO_pin,
                  Settings.SPI_MOSI_pin); // User-defined SPI
        break;
      }
      case SPI_Options_e::Vspi:
      {
        SPI.begin(); // VSPI
        break;
      }
      case SPI_Options_e::None:
        break;
    }
    #else // ifdef ESP32
    SPI.begin();
    #endif // ifdef ESP32
    addLog(LOG_LEVEL_INFO, F("INIT : SPI Init (without CS)"));
  }
  else
  {
    addLog(LOG_LEVEL_INFO, F("INIT : SPI not enabled"));
  }

#if FEATURE_SD

  if (Settings.Pin_sd_cs >= 0)
  {
    if (SD.begin(Settings.Pin_sd_cs))
    {
      addLog(LOG_LEVEL_INFO, F("SD   : Init OK"));
    }
    else
    {
      SD.end();
      addLog(LOG_LEVEL_ERROR, F("SD   : Init failed"));
    }
  }
#endif // if FEATURE_SD
}

void initI2C() {
  // configure hardware pins according to eeprom settings.
  if (!Settings.isI2CEnabled())
  {
    return;
  }
  addLog(LOG_LEVEL_INFO, F("INIT : I2C"));
  I2CSelectHighClockSpeed(); // Set normal clock speed

  if (Settings.WireClockStretchLimit)
  {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("INIT : I2C custom clockstretchlimit:");
      log += Settings.WireClockStretchLimit;
      addLogMove(LOG_LEVEL_INFO, log);
    }
      #if defined(ESP8266)
    Wire.setClockStretchLimit(Settings.WireClockStretchLimit);
      #endif // if defined(ESP8266)
      #ifdef ESP32S2
    Wire.setTimeOut(Settings.WireClockStretchLimit);
      #endif // ifdef ESP32S2
  }

  #if FEATURE_I2CMULTIPLEXER

  if (validGpio(Settings.I2C_Multiplexer_ResetPin)) { // Initialize Reset pin to High if configured
    pinMode(Settings.I2C_Multiplexer_ResetPin, OUTPUT);
    digitalWrite(Settings.I2C_Multiplexer_ResetPin, HIGH);
  }
  #endif // if FEATURE_I2CMULTIPLEXER

  // I2C Watchdog boot status check
  if (Settings.WDI2CAddress != 0)
  {
    delay(500);
    Wire.beginTransmission(Settings.WDI2CAddress);
    Wire.write(0x83); // command to set pointer
    Wire.write(17);   // pointer value to status uint8_t
    Wire.endTransmission();

    Wire.requestFrom(Settings.WDI2CAddress, (uint8_t)1);

    if (Wire.available())
    {
      uint8_t status = Wire.read();

      if (status & 0x1)
      {
        addLog(LOG_LEVEL_ERROR, F("INIT : Reset by WD!"));
        lastBootCause = BOOT_CAUSE_EXT_WD;
      }
    }
  }
}

void I2CSelectHighClockSpeed() {
  I2CSelectClockSpeed(Settings.I2C_clockSpeed);
}

void I2CSelectLowClockSpeed() {
  I2CSelectClockSpeed(Settings.I2C_clockSpeed_Slow);
}

void I2CSelect_Max100kHz_ClockSpeed() {
  if (Settings.I2C_clockSpeed <= 100000) {
    I2CSelectHighClockSpeed();
  } else if (Settings.I2C_clockSpeed_Slow <= 100000) {
    I2CSelectLowClockSpeed();
  } else {
    I2CSelectClockSpeed(100000);
  }
}

void I2CSelectClockSpeed(uint32_t clockFreq) {
  I2CBegin(Settings.Pin_i2c_sda, Settings.Pin_i2c_scl, clockFreq);
}

void I2CForceResetBus_swap_pins(uint8_t address) {
  if (!Settings.EnableClearHangingI2Cbus()) { return; }

  // As a final work-around, we temporary swap SDA and SCL, perform a scan and return pin order.
  I2CBegin(Settings.Pin_i2c_scl, Settings.Pin_i2c_sda, 100000);
  Wire.beginTransmission(address);
  Wire.endTransmission();
  delay(1);

  // Now we switch back to the correct pins
  I2CSelectClockSpeed(100000);
}

void I2CBegin(int8_t sda, int8_t scl, uint32_t clockFreq) {
  static uint32_t lastI2CClockSpeed = 0;
  static int8_t last_sda            = -1;
  static int8_t last_scl            = -1;

  if ((clockFreq == lastI2CClockSpeed) && (sda == last_sda) && (scl == last_scl)) {
    // No need to change the clock speed.
    return;
  }
  lastI2CClockSpeed = clockFreq;
  last_scl          = scl;
  last_sda          = sda;

  #ifdef ESP32
  Wire.begin(sda, scl, clockFreq);
  #else // ifdef ESP32
  Wire.begin(sda, scl);
  Wire.setClock(clockFreq);
  #endif // ifdef ESP32
}

#if FEATURE_I2CMULTIPLEXER

// Check if the I2C Multiplexer is enabled
bool isI2CMultiplexerEnabled() {
  return Settings.I2C_Multiplexer_Type != I2C_MULTIPLEXER_NONE
         && Settings.I2C_Multiplexer_Addr != -1;
}

// Reset the I2C Multiplexer, if a pin is assigned for that. Pulled to low to force a reset.
void I2CMultiplexerReset() {
  if (Settings.I2C_Multiplexer_ResetPin != -1) {
    digitalWrite(Settings.I2C_Multiplexer_ResetPin, LOW);
    delay(1); // minimum requirement of low for a proper reset seems to be about 6 nsec, so 1 msec should be more than sufficient
    digitalWrite(Settings.I2C_Multiplexer_ResetPin, HIGH);
  }
}

// Shift the bit in the right position when selecting a single channel
uint8_t I2CMultiplexerShiftBit(uint8_t i) {
  uint8_t toWrite = 0;

  switch (Settings.I2C_Multiplexer_Type) {
    case I2C_MULTIPLEXER_TCA9543A: // TCA9543/6/8 addressing
    case I2C_MULTIPLEXER_TCA9546A:
    case I2C_MULTIPLEXER_TCA9548A:
      toWrite = (1 << i);
      break;
    case I2C_MULTIPLEXER_PCA9540: // PCA9540 needs bit 2 set to write the channel
      toWrite = 0b00000100;

      if (i == 1) {
        toWrite |= 0b00000010; // And bit 0 not set when selecting channel 0...
      }
      break;
  }
  return toWrite;
}

// As initially constructed by krikk in PR#254, quite adapted
// utility method for the I2C multiplexer
// select the multiplexer port given as parameter, if taskIndex < 0 then take that abs value as the port to select (to allow I2C scanner)
void I2CMultiplexerSelectByTaskIndex(taskIndex_t taskIndex) {
  if (!validTaskIndex(taskIndex)) { return; }

  if (!I2CMultiplexerPortSelectedForTask(taskIndex)) { return; }

  uint8_t toWrite = 0;

  if (!bitRead(Settings.I2C_Flags[taskIndex], I2C_FLAGS_MUX_MULTICHANNEL)) {
    uint8_t i = Settings.I2C_Multiplexer_Channel[taskIndex];

    if (i > 7) { return; }
    toWrite = I2CMultiplexerShiftBit(i);
  } else {
    toWrite = Settings.I2C_Multiplexer_Channel[taskIndex]; // Bitpattern is already correctly stored
  }

  SetI2CMultiplexer(toWrite);
}

void I2CMultiplexerSelect(uint8_t i) {
  if (i > 7) { return; }

  uint8_t toWrite = I2CMultiplexerShiftBit(i);

  SetI2CMultiplexer(toWrite);
}

void I2CMultiplexerOff() {
  SetI2CMultiplexer(0); // no channel selected
}

void SetI2CMultiplexer(uint8_t toWrite) {
  if (isI2CMultiplexerEnabled()) {
    // FIXME TD-er: Must check to see if we can cache the value so only change it when needed.
    Wire.beginTransmission(Settings.I2C_Multiplexer_Addr);
    Wire.write(toWrite);
    Wire.endTransmission();

    // FIXME TD-er: We must check if the chip needs some time to set the output. (delay?)
  }
}

uint8_t I2CMultiplexerMaxChannels() {
  uint channels = 0;

  switch (Settings.I2C_Multiplexer_Type) {
    case I2C_MULTIPLEXER_TCA9548A:  channels = 8; break; // TCA9548A has 8 channels
    case I2C_MULTIPLEXER_TCA9546A:  channels = 4; break; // TCA9546A has 4 channels
    case I2C_MULTIPLEXER_PCA9540:   channels = 2; break; // PCA9540 has 2 channels
    case I2C_MULTIPLEXER_TCA9543A:  channels = 2; break; // TCA9543A has 2 channels
  }
  return channels;
}

// Has this taskIndex a channel selected? Checks for both Single channel and Multiple channel mode
// taskIndex must already be validated! (0..MAX_TASKS)
bool I2CMultiplexerPortSelectedForTask(taskIndex_t taskIndex) {
  if (!validTaskIndex(taskIndex)) { return false; }

  if (!isI2CMultiplexerEnabled()) { return false; }
  return (!bitRead(Settings.I2C_Flags[taskIndex], I2C_FLAGS_MUX_MULTICHANNEL) && Settings.I2C_Multiplexer_Channel[taskIndex] != -1)
         || (bitRead(Settings.I2C_Flags[taskIndex], I2C_FLAGS_MUX_MULTICHANNEL) && Settings.I2C_Multiplexer_Channel[taskIndex] !=  0);
}

#endif // if FEATURE_I2CMULTIPLEXER

void checkResetFactoryPin() {
  static uint8_t factoryResetCounter = 0;

  if (Settings.Pin_Reset == -1) {
    return;
  }

  if (digitalRead(Settings.Pin_Reset) == 0) { // active low reset pin
    factoryResetCounter++;                    // just count every second
  }
  else
  {                                           // reset pin released
    if (factoryResetCounter > 9) {
      // factory reset and reboot
      ResetFactory();
    }

    if (factoryResetCounter > 3) {
      // normal reboot
      reboot(ESPEasy_Scheduler::IntendedRebootReason_e::ResetFactoryPinActive);
    }
    factoryResetCounter = 0; // count was < 3, reset counter
  }
}

#ifdef ESP8266
int lastADCvalue = 0;

int espeasy_analogRead(int pin) {
  if (!WiFiEventData.wifiConnectInProgress) {
    lastADCvalue = analogRead(A0);
  }
  return lastADCvalue;
}

#endif // ifdef ESP8266

#ifdef ESP32

// ESP32 ADC calibration datatypes.
esp_adc_cal_value_t adc1_calibration_type = ESP_ADC_CAL_VAL_NOT_SUPPORTED;
esp_adc_cal_characteristics_t adc_chars[ADC_ATTEN_MAX];

void initADC() {
  # ifndef DEFAULT_VREF
  #  define DEFAULT_VREF 1100
  # endif // ifndef DEFAULT_VREF
  const adc_bits_width_t adc_bit_width = static_cast<adc_bits_width_t>(ADC_WIDTH_MAX - 1);
  for (size_t atten = 0; atten < ADC_ATTEN_MAX; ++atten) {
    adc1_calibration_type = esp_adc_cal_characterize(ADC_UNIT_1, static_cast<adc_atten_t>(atten), adc_bit_width, DEFAULT_VREF, &adc_chars[atten]);
  }
}

bool hasADC_factory_calibration() {
  return esp_adc_cal_check_efuse(adc1_calibration_type) == ESP_OK;
}

const __FlashStringHelper* getADC_factory_calibration_type() {
  switch (adc1_calibration_type) {
    case ESP_ADC_CAL_VAL_EFUSE_VREF:   return F("V_ref in eFuse");
    case ESP_ADC_CAL_VAL_EFUSE_TP:     return F("Two Point values in eFuse");
    case ESP_ADC_CAL_VAL_DEFAULT_VREF: return F("Default reference voltage");
    case ESP_ADC_CAL_VAL_EFUSE_TP_FIT: return F("Two Point values and fitting curve in eFuse");
    case ESP_ADC_CAL_VAL_NOT_SUPPORTED:
      break;
  }
  return F("Unknown");
}

int getADC_num_for_gpio(int pin) {
  int adc, ch, t;

  if (getADC_gpio_info(pin, adc, ch, t)) {
    return adc;
  }
  return -1;
}

int espeasy_analogRead(int pin, bool readAsTouch) {
  int value = 0;
  int adc, ch, t;

  if (getADC_gpio_info(pin, adc, ch, t)) {
    bool canread = false;

    switch (adc) {
      case 0:
      # ifndef ESP32S2
        value = hallRead();
      # endif // ifndef ESP32S2
        break;
      case 1:
        canread = true;
        break;
      case 2:

        if (WiFi.getMode() == WIFI_OFF) {
          // See:
          // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#configuration-and-reading-adc
          // ADC2 is shared with WiFi, so don't read ADC2 when WiFi is on.
          canread = true;
        }
        break;
    }

    if (canread) {
      if (readAsTouch && (t >= 0)) {
        value = touchRead(pin);
      } else {
        value = analogRead(pin);
      }
    }
  }
  return value;
}

#endif // ifdef ESP32


/********************************************************************************************\
   Hardware information
 \*********************************************************************************************/
uint32_t getFlashChipId() {
  // Cache since size does not change
  static uint32_t flashChipId = 0;

  if (flashChipId == 0) {
  #ifdef ESP32
    uint32_t tmp = g_rom_flashchip.device_id;

    for (int i = 0; i < 3; ++i) {
      flashChipId  = flashChipId << 8;
      flashChipId |= (tmp & 0xFF);
      tmp          = tmp >> 8;
    }

    //    esp_flash_read_id(nullptr, &flashChipId);
  #elif defined(ESP8266)
    flashChipId = ESP.getFlashChipId();
  #endif // ifdef ESP32
  }
  return flashChipId;
}

uint32_t getFlashRealSizeInBytes() {
  // Cache since size does not change
  static uint32_t res = 0;

  if (res == 0) {
    #if defined(ESP32)
    res = (1 << ((getFlashChipId() >> 16) & 0xFF));
    #else // if defined(ESP32)
    res = ESP.getFlashChipRealSize(); // ESP.getFlashChipSize();
    #endif // if defined(ESP32)
  }
  return res;
}

uint32_t getFlashChipSpeed() {
  return ESP.getFlashChipSpeed();
}

bool puyaSupport() {
  bool supported = false;

#ifdef PUYA_SUPPORT

  // New support starting core 2.5.0
  if (PUYA_SUPPORT) { supported = true; }
#endif // ifdef PUYA_SUPPORT
#ifdef PUYASUPPORT

  // Old patch
  supported = true;
#endif // ifdef PUYASUPPORT
  return supported;
}

uint8_t getFlashChipVendorId() {
#ifdef PUYA_SUPPORT
  return ESP.getFlashChipVendorId();
#else // ifdef PUYA_SUPPORT
  # if defined(ESP8266)

  // Cache since size does not change
  static uint32_t flashChipId = ESP.getFlashChipId();
  return flashChipId & 0x000000ff;
  # elif defined(ESP32)
  return 0xFF; // Not an existing function for ESP32
  # endif // if defined(ESP8266)
#endif // ifdef PUYA_SUPPORT
}

bool flashChipVendorPuya() {
  const uint8_t vendorId = getFlashChipVendorId();

  return vendorId == 0x85; // 0x146085 PUYA
}

uint32_t getChipId() {
  uint32_t chipId = 0;

#ifdef ESP8266
  chipId = ESP.getChipId();
#endif // ifdef ESP8266
#ifdef ESP32

  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
#endif // ifdef ESP32

  return chipId;
}

uint8_t getChipCores() {
  #ifdef ESP8266
  return 1;
  #else // ifdef ESP8266
  static uint8_t cores = 0;

  if (cores == 0) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    cores = chip_info.cores;
  }
  return cores;
  #endif // ifdef ESP8266
}

const __FlashStringHelper* getChipModel() {
#ifdef ESP32

  // https://www.espressif.com/en/products/socs
  // https://github.com/arendst/Tasmota/blob/1e6b78a957be538cf494f0e2dc49060d1cb0fe8b/tasmota/support_esp.ino#L579

  /*
     Source: esp-idf esp_system.h and esptool
     typedef enum {
      CHIP_ESP32   = 1,  //!< ESP32
      CHIP_ESP32S2 = 2,  //!< ESP32-S2
      CHIP_ESP32S3 = 4,  //!< ESP32-S3
      CHIP_ESP32C3 = 5,  //!< ESP32-C3
     } esp_chip_model_t;
     // Chip feature flags, used in esp_chip_info_t
   #define CHIP_FEATURE_EMB_FLASH      BIT(0)      //!< Chip has embedded flash memory
   #define CHIP_FEATURE_WIFI_BGN       BIT(1)      //!< Chip has 2.4GHz WiFi
   #define CHIP_FEATURE_BLE            BIT(4)      //!< Chip has Bluetooth LE
   #define CHIP_FEATURE_BT             BIT(5)      //!< Chip has Bluetooth Classic
     // The structure represents information about the chip
     typedef struct {
      esp_chip_model_t model;  //!< chip model, one of esp_chip_model_t
      uint32_t features;       //!< bit mask of CHIP_FEATURE_x feature flags
      uint8_t cores;           //!< number of CPU cores
      uint8_t revision;        //!< chip revision number
     } esp_chip_info_t;
   */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  uint32_t chip_model    = chip_info.model;
  uint32_t chip_revision = chip_info.revision;

  //  uint32_t chip_revision = ESP.getChipRevision();
  bool rev3 = (3 == chip_revision);

  //  bool single_core = (1 == ESP.getChipCores());
  bool single_core = (1 == chip_info.cores);

  if (chip_model < 2) { // ESP32
# ifdef CONFIG_IDF_TARGET_ESP32

    /* esptool:
        def get_pkg_version(self):
            word3 = self.read_efuse(3)
            pkg_version = (word3 >> 9) & 0x07
            pkg_version += ((word3 >> 2) & 0x1) << 3
            return pkg_version
     */
    uint32_t chip_ver    = REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_VER_PKG);
    uint32_t pkg_version = chip_ver & 0x7;

    //    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision,
    // chip_info.cores, chip_ver);

    switch (pkg_version) {
      case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ6:

        if (single_core) { return F("ESP32-S0WDQ6"); }   // Max 240MHz, Single core, QFN 6*6
        else if (rev3)   { return F("ESP32-D0WDQ6-V3"); }  // Max 240MHz, Dual core, QFN 6*6
        else {             return F("ESP32-D0WDQ6"); }   // Max 240MHz, Dual core, QFN 6*6
      case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ5:

        if (single_core) { return F("ESP32-S0WD"); }     // Max 160MHz, Single core, QFN 5*5, ESP32-SOLO-1, ESP32-DevKitC
        else if (rev3)   { return F("ESP32-D0WDQ5-V3"); }  // Max 240MHz, Dual core, QFN 5*5, ESP32-WROOM-32E, ESP32_WROVER-E, ESP32-DevKitC
        else {             return F("ESP32-D0WDQ5"); }   // Max 240MHz, Dual core, QFN 5*5, ESP32-WROOM-32D, ESP32_WROVER-B, ESP32-DevKitC
      case EFUSE_RD_CHIP_VER_PKG_ESP32D2WDQ5:
        return F("ESP32-D2WDQ5");                        // Max 160MHz, Dual core, QFN 5*5, 2MB embedded flash
      case 3:

        if (single_core) { return F("ESP32-S0WD-OEM"); } // Max 160MHz, Single core, QFN 5*5, Xiaomi Yeelight
        else {             return F("ESP32-D0WD-OEM"); } // Max 240MHz, Dual core, QFN 5*5
      case EFUSE_RD_CHIP_VER_PKG_ESP32U4WDH:
        return F("ESP32-U4WDH");                         // Max 160MHz, Single core, QFN 5*5, 4MB embedded flash, ESP32-MINI-1,
                                                         // ESP32-DevKitM-1
      case EFUSE_RD_CHIP_VER_PKG_ESP32PICOD4:

        if (rev3)        { return F("ESP32-PICO-V3"); }  // Max 240MHz, Dual core, LGA 7*7, ESP32-PICO-V3-ZERO, ESP32-PICO-V3-ZERO-DevKit
        else {             return F("ESP32-PICO-D4"); }  // Max 240MHz, Dual core, LGA 7*7, 4MB embedded flash, ESP32-PICO-KIT
      case EFUSE_RD_CHIP_VER_PKG_ESP32PICOV302:
        return F("ESP32-PICO-V3-02");                    // Max 240MHz, Dual core, LGA 7*7, 8MB embedded flash, 2MB embedded PSRAM,
                                                         // ESP32-PICO-MINI-02, ESP32-PICO-DevKitM-2
    }
# endif // CONFIG_IDF_TARGET_ESP32
    return F("ESP32");
  }
  else if (2 == chip_model) { // ESP32-S2
# ifdef CONFIG_IDF_TARGET_ESP32S2

    /* esptool:
        def get_pkg_version(self):
            num_word = 3
            block1_addr = self.EFUSE_BASE + 0x044
            word3 = self.read_reg(block1_addr + (4 * num_word))
            pkg_version = (word3 >> 21) & 0x0F
            return pkg_version
     */
    uint32_t chip_ver    = REG_GET_FIELD(EFUSE_RD_MAC_SPI_SYS_3_REG, EFUSE_PKG_VERSION);
    uint32_t pkg_version = chip_ver & 0x7;

    //    uint32_t pkg_version = esp_efuse_get_pkg_ver();

    //    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision,
    // chip_info.cores, chip_ver);

    switch (pkg_version) {
      case 0:              return F("ESP32-S2");      // Max 240MHz, Single core, QFN 7*7, ESP32-S2-WROOM, ESP32-S2-WROVER,
                                                      // ESP32-S2-Saola-1, ESP32-S2-Kaluga-1
      case 1:              return F("ESP32-S2FH2");   // Max 240MHz, Single core, QFN 7*7, 2MB embedded flash, ESP32-S2-MINI-1,
                                                      // ESP32-S2-DevKitM-1
      case 2:              return F("ESP32-S2FH4");   // Max 240MHz, Single core, QFN 7*7, 4MB embedded flash
      case 3:              return F("ESP32-S2FN4R2"); // Max 240MHz, Single core, QFN 7*7, 4MB embedded flash, 2MB embedded PSRAM, ,
                                                      // ESP32-S2-MINI-1U, ESP32-S2-DevKitM-1U
    }
# endif // CONFIG_IDF_TARGET_ESP32S2
    return F("ESP32-S2");
  }
  else if (4 == chip_model) { // ESP32-S3
    return F("ESP32-S3");     // Max 240MHz, Dual core, QFN 7*7, ESP32-S3-WROOM-1, ESP32-S3-DevKitC-1
  }
  else if (5 == chip_model) { // ESP32-C3
# ifdef CONFIG_IDF_TARGET_ESP32C3

    /* esptool:
        def get_pkg_version(self):
            num_word = 3
            block1_addr = self.EFUSE_BASE + 0x044
            word3 = self.read_reg(block1_addr + (4 * num_word))
            pkg_version = (word3 >> 21) & 0x0F
            return pkg_version
     */
    uint32_t chip_ver    = REG_GET_FIELD(EFUSE_RD_MAC_SPI_SYS_3_REG, EFUSE_PKG_VERSION);
    uint32_t pkg_version = chip_ver & 0x7;

    //    uint32_t pkg_version = esp_efuse_get_pkg_ver();

    //    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision,
    // chip_info.cores, chip_ver);

    switch (pkg_version) {
      case 0:              return F("ESP32-C3");    // Max 160MHz, Single core, QFN 5*5, ESP32-C3-WROOM-02, ESP32-C3-DevKitC-02
      case 1:              return F("ESP32-C3FH4"); // Max 160MHz, Single core, QFN 5*5, 4MB embedded flash, ESP32-C3-MINI-1,
                                                    // ESP32-C3-DevKitM-1
    }
# endif // CONFIG_IDF_TARGET_ESP32C3
    return F("ESP32-C3");
  }
  else if (6 == chip_model) { // ESP32-S3(beta3)
    return F("ESP32-S3");
  }
  else if (7 == chip_model) { // ESP32-C6(beta)
# ifdef CONFIG_IDF_TARGET_ESP32C6

    /* esptool:
        def get_pkg_version(self):
            num_word = 3
            block1_addr = self.EFUSE_BASE + 0x044
            word3 = self.read_reg(block1_addr + (4 * num_word))
            pkg_version = (word3 >> 21) & 0x0F
            return pkg_version
     */
    uint32_t chip_ver    = REG_GET_FIELD(EFUSE_RD_MAC_SPI_SYS_3_REG, EFUSE_PKG_VERSION);
    uint32_t pkg_version = chip_ver & 0x7;

    //    uint32_t pkg_version = esp_efuse_get_pkg_ver();

    //    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision,
    // chip_info.cores, chip_ver);

    switch (pkg_version) {
      case 0:              return F("ESP32-C6");
    }
# endif // CONFIG_IDF_TARGET_ESP32C6
    return F("ESP32-C6");
  }
  else if (10 == chip_model) {  // ESP32-H2
# ifdef CONFIG_IDF_TARGET_ESP32H2

    /* esptool:
        def get_pkg_version(self):
            num_word = 3
            block1_addr = self.EFUSE_BASE + 0x044
            word3 = self.read_reg(block1_addr + (4 * num_word))
            pkg_version = (word3 >> 21) & 0x0F
            return pkg_version
     */
    uint32_t chip_ver    = REG_GET_FIELD(EFUSE_RD_MAC_SPI_SYS_3_REG, EFUSE_PKG_VERSION);
    uint32_t pkg_version = chip_ver & 0x7;

    //    uint32_t pkg_version = esp_efuse_get_pkg_ver();

    //    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("HDW: ESP32 Model %d, Revision %d, Core %d, Package %d"), chip_info.model, chip_revision,
    // chip_info.cores, chip_ver);

    switch (pkg_version) {
      case 0:              return F("ESP32-H2");
    }
# endif // CONFIG_IDF_TARGET_ESP32H2
    return F("ESP32-H2");
  }
  return F("ESP32");
#elif defined(ESP8266)
  return isESP8285() ? F("ESP8285") : F("ESP8266");
#endif
  return F("Unknown");
}

bool isESP8285() {
  #ifdef ESP8266
  const uint32_t efuse_blocks[4] {
    READ_PERI_REG(0x3ff00050),
    READ_PERI_REG(0x3ff00054),
    READ_PERI_REG(0x3ff00058),
    READ_PERI_REG(0x3ff0005c)
  };

  return (
      (efuse_blocks[0] & (1 << 4))
      || (efuse_blocks[2] & (1 << 16))
  );
  #else
  return false;
  #endif
}

uint8_t getChipRevision() {
  uint8_t rev = 0;

  #ifdef ESP32
  rev = ESP.getChipRevision();
  #endif // ifdef ESP32
  return rev;
}

uint32_t getSketchSize() {
  // Cache the value as this never changes during run time.
  static uint32_t res = ESP.getSketchSize();

  return res;
}

uint32_t getFreeSketchSpace() {
  // Cache the value as this never changes during run time.
  static uint32_t res = ESP.getFreeSketchSpace();

  return res;
}

/********************************************************************************************\
   PSRAM support
 \*********************************************************************************************/

#ifdef ESP32

// this function is a replacement for `psramFound()`.
// `psramFound()` can return true even if no PSRAM is actually installed
// This new version also checks `esp_spiram_is_initialized` to know if the PSRAM is initialized
// Original Tasmota:
// https://github.com/arendst/Tasmota/blob/1e6b78a957be538cf494f0e2dc49060d1cb0fe8b/tasmota/support_esp.ino#L470
bool FoundPSRAM() {
# if CONFIG_IDF_TARGET_ESP32C3
  return psramFound();
# else // if CONFIG_IDF_TARGET_ESP32C3
  return psramFound() && esp_spiram_is_initialized();
# endif // if CONFIG_IDF_TARGET_ESP32C3
}

// new function to check whether PSRAM is present and supported (i.e. required pacthes are present)
bool UsePSRAM() {
  static bool can_use_psram = CanUsePSRAM();

  return FoundPSRAM() && can_use_psram;
}

/*
 * ESP32 v1 and v2 needs some special patches to use PSRAM.
 * Original function used from Tasmota:
 * https://github.com/arendst/Tasmota/blob/1e6b78a957be538cf494f0e2dc49060d1cb0fe8b/tasmota/support_esp.ino#L762
 *
 * If using ESP32 v1, please add: `-mfix-esp32-psram-cache-issue -lc-psram-workaround -lm-psram-workaround`
 *
 * This function returns true if the chip supports PSRAM natively (v3) or if the
 * patches are present.
 */
bool CanUsePSRAM() {
  if (!FoundPSRAM()) { return false; }
# ifdef HAS_PSRAM_FIX
  return true;
# endif // ifdef HAS_PSRAM_FIX
# ifdef CONFIG_IDF_TARGET_ESP32
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  if ((CHIP_ESP32 == chip_info.model) && (chip_info.revision < 3)) {
    return false;
  }
#  if ESP_IDF_VERSION_MAJOR < 4
  uint32_t chip_ver    = REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_VER_PKG);
  uint32_t pkg_version = chip_ver & 0x7;

  if ((CHIP_ESP32 == chip_info.model) && (pkg_version >= 6)) {
    return false; // support for embedded PSRAM of ESP32-PICO-V3-02 requires esp-idf 4.4
  }
#  endif // ESP_IDF_VERSION_MAJOR < 4

# endif // CONFIG_IDF_TARGET_ESP32
  return true;
}

#endif // ESP32


/*********************************************************************************************\
* High entropy hardware random generator
* Thanks to DigitalAlchemist
\*********************************************************************************************/

// Based on code from https://raw.githubusercontent.com/espressif/esp-idf/master/components/esp32/hw_random.c
// https://github.com/arendst/Tasmota/blob/1e6b78a957be538cf494f0e2dc49060d1cb0fe8b/tasmota/support_esp.ino#L805
uint32_t HwRandom() {
#if ESP8266

  // https://web.archive.org/web/20160922031242/http://esp8266-re.foogod.com/wiki/Random_Number_Generator
  # define _RAND_ADDR 0x3FF20E44UL
#endif // ESP8266
#ifdef ESP32
  # define _RAND_ADDR 0x3FF75144UL
#endif // ESP32
  static uint32_t last_ccount = 0;
  uint32_t ccount;
  uint32_t result = 0;

  do {
    ccount  = ESP.getCycleCount();
    result ^= *(volatile uint32_t *)_RAND_ADDR;     // -V566
  } while (ccount - last_ccount < 64);
  last_ccount = ccount;
  return result ^ *(volatile uint32_t *)_RAND_ADDR; // -V566
#undef _RAND_ADDR
}

#ifdef ESP8266
void readBootCause() {
  lastBootCause = BOOT_CAUSE_MANUAL_REBOOT;
  const rst_info *resetInfo = ESP.getResetInfoPtr();

  if (resetInfo != nullptr) {
    switch (resetInfo->reason) {
      // normal startup by power on
      case REASON_DEFAULT_RST:      lastBootCause = BOOT_CAUSE_COLD_BOOT; break;

      // hardware watch dog reset
      case REASON_WDT_RST:          lastBootCause = BOOT_CAUSE_EXT_WD; break;

      // exception reset, GPIO status won’t change
      case REASON_EXCEPTION_RST:    lastBootCause = BOOT_CAUSE_EXCEPTION; break;

      // software watch dog reset, GPIO status won’t change
      case REASON_SOFT_WDT_RST:     lastBootCause = BOOT_CAUSE_SW_WATCHDOG; break;

      // software restart ,system_restart , GPIO status won’t change
      case REASON_SOFT_RESTART:     lastBootCause = BOOT_CAUSE_SOFT_RESTART; break;

      // wake up from deep-sleep
      case REASON_DEEP_SLEEP_AWAKE: lastBootCause = BOOT_CAUSE_DEEP_SLEEP; break;

      // external system reset
      case REASON_EXT_SYS_RST:      lastBootCause = BOOT_CAUSE_MANUAL_REBOOT; break;
      default:
        break;
    }
  }
}

#endif // ifdef ESP8266

#ifdef ESP32
void readBootCause() {
  lastBootCause = BOOT_CAUSE_MANUAL_REBOOT;

  switch (rtc_get_reset_reason(0)) {
    case NO_MEAN:           break;
    case POWERON_RESET:     lastBootCause = BOOT_CAUSE_MANUAL_REBOOT; break;
    # ifndef ESP32S2
    case SW_RESET:          lastBootCause = BOOT_CAUSE_SOFT_RESTART; break;
    case OWDT_RESET:        lastBootCause = BOOT_CAUSE_SW_WATCHDOG; break;
    # endif // ifndef ESP32S2
    case DEEPSLEEP_RESET:   lastBootCause = BOOT_CAUSE_DEEP_SLEEP; break;
    # ifndef ESP32S2
    case SDIO_RESET:        lastBootCause = BOOT_CAUSE_MANUAL_REBOOT; break;
    # endif // ifndef ESP32S2
    case TG0WDT_SYS_RESET:
    case TG1WDT_SYS_RESET:
    case RTCWDT_SYS_RESET:  lastBootCause = BOOT_CAUSE_EXT_WD; break;
    # ifndef ESP32S2
    case SW_CPU_RESET:
    case TGWDT_CPU_RESET:
    # endif // ifndef ESP32S2
    case INTRUSION_RESET:   lastBootCause = BOOT_CAUSE_SOFT_RESTART; break;  // Both call to ESP.reset() and on exception crash
    case RTCWDT_CPU_RESET:  lastBootCause = BOOT_CAUSE_EXT_WD; break;
    # ifndef ESP32S2
    case EXT_CPU_RESET:     lastBootCause = BOOT_CAUSE_MANUAL_REBOOT; break; // reset button or cold boot, only for core 1
    # endif // ifndef ESP32S2
    case RTCWDT_BROWN_OUT_RESET: lastBootCause = BOOT_CAUSE_POWER_UNSTABLE; break;
    case RTCWDT_RTC_RESET:  lastBootCause      = BOOT_CAUSE_COLD_BOOT; break;
    # ifdef ESP32S2
    case RTC_SW_SYS_RESET: lastBootCause  = BOOT_CAUSE_SOFT_RESTART; break;
    case TG0WDT_CPU_RESET: lastBootCause  = BOOT_CAUSE_EXT_WD; break;
    case RTC_SW_CPU_RESET: lastBootCause  = BOOT_CAUSE_SOFT_RESTART; break;
    case TG1WDT_CPU_RESET: lastBootCause  = BOOT_CAUSE_EXT_WD; break;
    case SUPER_WDT_RESET:   lastBootCause = BOOT_CAUSE_EXT_WD; break;
    case GLITCH_RTC_RESET:  lastBootCause = BOOT_CAUSE_POWER_UNSTABLE; break; // FIXME TD-er: Does this need a different reason?
    case EFUSE_RESET:       break; // FIXME TD-er: No idea what may cause this reset reason.
    # endif // ifdef ESP32S2
  }
}

#endif // ifdef ESP32


/********************************************************************************************\
   Hardware specific configurations
 \*********************************************************************************************/
const __FlashStringHelper* getDeviceModelBrandString(DeviceModel model) {
  switch (model) {
    case DeviceModel::DeviceModel_Sonoff_Basic:
    case DeviceModel::DeviceModel_Sonoff_TH1x:
    case DeviceModel::DeviceModel_Sonoff_S2x:
    case DeviceModel::DeviceModel_Sonoff_TouchT1:
    case DeviceModel::DeviceModel_Sonoff_TouchT2:
    case DeviceModel::DeviceModel_Sonoff_TouchT3:
    case DeviceModel::DeviceModel_Sonoff_4ch:
    case DeviceModel::DeviceModel_Sonoff_POW:
    case DeviceModel::DeviceModel_Sonoff_POWr2:   return F("Sonoff");
    case DeviceModel::DeviceModel_Shelly1:
    case DeviceModel::DeviceModel_ShellyPLUG_S:   return F("Shelly");
    case DeviceModel::DeviceModel_Olimex_ESP32_PoE:
    case DeviceModel::DeviceModel_Olimex_ESP32_EVB:
    case DeviceModel::DeviceModel_Olimex_ESP32_GATEWAY:
    #ifdef ESP32
      return F("Olimex");
    #endif // ifdef ESP32
    case DeviceModel::DeviceModel_wESP32:
    #ifdef ESP32
      return F("wESP32");
    #endif // ifdef ESP32
    case DeviceModel::DeviceModel_WT32_ETH01:
    #ifdef ESP32
      return F("WT32-ETH01");
    #endif // ifdef ESP32
    case DeviceModel::DeviceModel_default:
    case DeviceModel::DeviceModel_MAX:      break;

      // Do not use default: as this allows the compiler to detect any missing cases.
  }
  return F("");
}

const __FlashStringHelper* getDeviceModelTypeString(DeviceModel model)
{
  switch (model) {
#if defined(ESP8266) && !defined(LIMIT_BUILD_SIZE)
    case DeviceModel::DeviceModel_Sonoff_Basic:   return F(" Basic");
    case DeviceModel::DeviceModel_Sonoff_TH1x:    return F(" TH1x");
    case DeviceModel::DeviceModel_Sonoff_S2x:     return F(" S2x");
    case DeviceModel::DeviceModel_Sonoff_TouchT1: return F(" TouchT1");
    case DeviceModel::DeviceModel_Sonoff_TouchT2: return F(" TouchT2");
    case DeviceModel::DeviceModel_Sonoff_TouchT3: return F(" TouchT3");
    case DeviceModel::DeviceModel_Sonoff_4ch:     return F(" 4ch");
    case DeviceModel::DeviceModel_Sonoff_POW:     return F(" POW");
    case DeviceModel::DeviceModel_Sonoff_POWr2:   return F(" POW-r2");
    case DeviceModel::DeviceModel_Shelly1:        return F("1");
    case DeviceModel::DeviceModel_ShellyPLUG_S:   return F(" PLUG S");
#else // if defined(ESP8266) && !defined(LIMIT_BUILD_SIZE)
    case DeviceModel::DeviceModel_Sonoff_Basic:
    case DeviceModel::DeviceModel_Sonoff_TH1x:
    case DeviceModel::DeviceModel_Sonoff_S2x:
    case DeviceModel::DeviceModel_Sonoff_TouchT1:
    case DeviceModel::DeviceModel_Sonoff_TouchT2:
    case DeviceModel::DeviceModel_Sonoff_TouchT3:
    case DeviceModel::DeviceModel_Sonoff_4ch:
    case DeviceModel::DeviceModel_Sonoff_POW:
    case DeviceModel::DeviceModel_Sonoff_POWr2:
    case DeviceModel::DeviceModel_Shelly1:
    case DeviceModel::DeviceModel_ShellyPLUG_S:
      return F("default");
#endif // if defined(ESP8266) && !defined(LIMIT_BUILD_SIZE)
#ifdef ESP32
    case DeviceModel::DeviceModel_Olimex_ESP32_PoE:      return F(" ESP32-PoE");
    case DeviceModel::DeviceModel_Olimex_ESP32_EVB:      return F(" ESP32-EVB");
    case DeviceModel::DeviceModel_Olimex_ESP32_GATEWAY:  return F(" ESP32-GATEWAY");
    case DeviceModel::DeviceModel_wESP32:                break;
    case DeviceModel::DeviceModel_WT32_ETH01:            return F(" add-on");
#else // ifdef ESP32
    case DeviceModel::DeviceModel_Olimex_ESP32_PoE:
    case DeviceModel::DeviceModel_Olimex_ESP32_EVB:
    case DeviceModel::DeviceModel_Olimex_ESP32_GATEWAY:
    case DeviceModel::DeviceModel_wESP32:
    case DeviceModel::DeviceModel_WT32_ETH01:
#endif // ifdef ESP32

    case DeviceModel::DeviceModel_default:
    case DeviceModel::DeviceModel_MAX:             return F("default");

      // Do not use default: as this allows the compiler to detect any missing cases.
  }
  return F("");
}

String getDeviceModelString(DeviceModel model) {
  String result = getDeviceModelBrandString(model);

  result += getDeviceModelTypeString(model);
  return result;
}

bool modelMatchingFlashSize(DeviceModel model) {
#if defined(ESP8266) || (defined(ESP32) && FEATURE_ETHERNET)
  const uint32_t size_MB = getFlashRealSizeInBytes() >> 20;
#endif // if defined(ESP8266) || (defined(ESP32) && FEATURE_ETHERNET)

  // TD-er: This also checks for ESP8266/ESP8285/ESP32
  switch (model) {
    case DeviceModel::DeviceModel_Sonoff_Basic:
    case DeviceModel::DeviceModel_Sonoff_TH1x:
    case DeviceModel::DeviceModel_Sonoff_S2x:
    case DeviceModel::DeviceModel_Sonoff_TouchT1:
    case DeviceModel::DeviceModel_Sonoff_TouchT2:
    case DeviceModel::DeviceModel_Sonoff_TouchT3:
    case DeviceModel::DeviceModel_Sonoff_4ch:
#ifdef ESP8266
      return size_MB == 1;
#else // ifdef ESP8266
      return false;
#endif // ifdef ESP8266

    case DeviceModel::DeviceModel_Sonoff_POW:
    case DeviceModel::DeviceModel_Sonoff_POWr2:
#ifdef ESP8266
      return size_MB == 4;
#else // ifdef ESP8266
      return false;
#endif // ifdef ESP8266

    case DeviceModel::DeviceModel_Shelly1:
    case DeviceModel::DeviceModel_ShellyPLUG_S:
#ifdef ESP8266
      return size_MB == 2;
#else // ifdef ESP8266
      return false;
#endif // ifdef ESP8266

    // These Olimex boards all have Ethernet
    case DeviceModel::DeviceModel_Olimex_ESP32_PoE:
    case DeviceModel::DeviceModel_Olimex_ESP32_EVB:
    case DeviceModel::DeviceModel_Olimex_ESP32_GATEWAY:
    case DeviceModel::DeviceModel_wESP32:
    case DeviceModel::DeviceModel_WT32_ETH01:
#if  defined(ESP32) && FEATURE_ETHERNET
      return size_MB == 4;
#else // if  defined(ESP32) && FEATURE_ETHERNET
      return false;
#endif // if  defined(ESP32) && FEATURE_ETHERNET

    case DeviceModel::DeviceModel_default:
    case DeviceModel::DeviceModel_MAX:
      return true;

      // Do not use default: as this allows the compiler to detect any missing cases.
  }
  return true;
}

void setFactoryDefault(DeviceModel model) {
  ResetFactoryDefaultPreference.setDeviceModel(model);
}

/********************************************************************************************\
   Add pre defined plugins and rules.
 \*********************************************************************************************/
void addSwitchPlugin(taskIndex_t taskIndex, int gpio, const String& name, bool activeLow) {
  setTaskDevice_to_TaskIndex(1, taskIndex);
  setBasicTaskValues(
    taskIndex,
    0,    // taskdevicetimer
    true, // enabled
    name, // name
    gpio, // pin1
    -1,   // pin2
    -1);  // pin3
  Settings.TaskDevicePin1PullUp[taskIndex] = true;

  if (activeLow) {
    Settings.TaskDevicePluginConfig[taskIndex][2] = 1; // PLUGIN_001_BUTTON_TYPE_PUSH_ACTIVE_LOW;
  }
  Settings.TaskDevicePluginConfig[taskIndex][3] = 1;   // "Send Boot state" checked.
}

void addPredefinedPlugins(const GpioFactorySettingsStruct& gpio_settings) {
  taskIndex_t taskIndex = 0;

  for (int i = 0; i < 4; ++i) {
    if (gpio_settings.button[i] >= 0) {
      String label = F("Button");
      label += (i + 1);
      addSwitchPlugin(taskIndex, gpio_settings.button[i], label, true);
      ++taskIndex;
    }

    if (gpio_settings.relais[i] >= 0) {
      String label = F("Relay");
      label += (i + 1);
      addSwitchPlugin(taskIndex, gpio_settings.relais[i], label, false);
      ++taskIndex;
    }
  }
}

void addButtonRelayRule(uint8_t buttonNumber, int relay_gpio) {
  Settings.UseRules = true;
  String fileName;

  #if defined(ESP32)
  fileName += '/';
  #endif // if defined(ESP32)
  fileName += F("rules1.txt");
  String rule = F("on ButtonBNR#state do\n  if [RelayBNR#state]=0\n    gpio,GNR,1\n  else\n    gpio,GNR,0\n  endif\nendon\n");
  rule.replace(F("BNR"), String(buttonNumber));
  rule.replace(F("GNR"), String(relay_gpio));
  String result = appendLineToFile(fileName, rule);

  if (result.length() > 0) {
    addLogMove(LOG_LEVEL_ERROR, result);
  }
}

void addPredefinedRules(const GpioFactorySettingsStruct& gpio_settings) {
  for (int i = 0; i < 4; ++i) {
    if ((gpio_settings.button[i] >= 0) && (gpio_settings.relais[i] >= 0)) {
      addButtonRelayRule((i + 1), gpio_settings.relais[i]);
    }
  }
}

#ifdef ESP32

// ********************************************************************************
// Get info of a specific GPIO pin
// ********************************************************************************
bool getGpioInfo(int gpio, int& pinnr, bool& input, bool& output, bool& warning) {
  pinnr = -1; // ESP32 does not label the pins, they just use the GPIO number.

# ifdef ESP32S2

  // Input GPIOs:  0-21, 26, 33-46
  // Output GPIOs: 0-21, 26, 33-45
  input  = gpio <= 46;
  output = gpio <= 45;

  if ((gpio < 0) || ((gpio > 21) && (gpio < 26)) || ((gpio > 26) && (gpio < 33))) {
    input  = false;
    output = false;
  }

  if (gpio == 26) {
    // Pin shared with the flash memory and/or PSRAM.
    // Cannot be used as regular GPIO
    input   = false;
    output  = false;
    warning = true;
  }

  if ((gpio > 26) && (gpio < 33)) {
    // SPIHD, SPIWP, SPICS0, SPICLK, SPIQ, SPID pins of ESP32-S2FH2 and ESP32-S2FH4
    // are connected to embedded flash and not recommended for other uses.
    warning = true;
  }


  if ((input == false) && (output == false)) {
    return false;
  }

  if (gpio == 45) {
    // VDD_SPI can work as the power supply for the external device at either
    // 1.8 V (when GPIO45 is 1 during boot), or
    // 3.3 V (when GPIO45 is 0 and at default state during boot).
    warning = true;
  }

  // GPIO 0  State during boot determines boot mode.
  warning = gpio == 0;


  if (gpio == 46) {
    // Part of the boot strapping pins.
    warning = true;
  }

  /*
   # if FEATURE_ETHERNET

     // Check pins used for RMII Ethernet PHY
     if (NetworkMedium_t::Ethernet == Settings.NetworkMedium) {
      switch (gpio) {
        case 0:
        case 21:
        case 19:
        case 22:
        case 25:
        case 26:
        case 27:
          warning = true;
          break;
      }


      // FIXME TD-er: Must we also check for pins used for MDC/MDIO and Eth PHY power?
     }


   # endif // if FEATURE_ETHERNET

   */
# else // ifdef ESP32S2

  // ESP32 classic

  // Input GPIOs:  0-19, 21-23, 25-27, 32-39
  // Output GPIOs: 0-19, 21-23, 25-27, 32-33
  input  = gpio <= 39;
  output = gpio <= 33;

  if ((gpio < 0) || (gpio == 20) || (gpio == 24) || ((gpio > 27) && (gpio < 32))) {
    input  = false;
    output = false;
  }

  if ((gpio == 37) || (gpio == 38)) {
    // Pins are not present on the ESP32
    input  = true;
    output = false;
  }

  if ((gpio >= 6) && (gpio <= 11)) {
    // Connected to the integrated SPI flash.
    input   = false;
    output  = false;
    warning = true;
  }

  if ((input == false) && (output == false)) {
    return false;
  }

  // GPIO 0 & 2 can't be used as an input. State during boot is dependent on boot mode.
  warning = (gpio == 0 || gpio == 2);

  if (gpio == 12) {
    // If driven High, flash voltage (VDD_SDIO) is 1.8V not default 3.3V.
    // Has internal pull-down, so unconnected = Low = 3.3V.
    // May prevent flashing and/or booting if 3.3V flash is used and this pin is
    // pulled high, causing the flash to brownout.
    // See the ESP32 datasheet for more details.
    warning = true;
  }

  if (gpio == 15) {
    // If driven Low, silences boot messages printed by the ROM bootloader.
    // Has an internal pull-up, so unconnected = High = normal output.
    warning = true;
  }

  #  if FEATURE_ETHERNET

  // Check pins used for RMII Ethernet PHY
  if (NetworkMedium_t::Ethernet == Settings.NetworkMedium) {
    switch (gpio) {
      case 0:
      case 21:
      case 19:
      case 22:
      case 25:
      case 26:
      case 27:
        warning = true;
        break;
    }


    // FIXME TD-er: Must we also check for pins used for MDC/MDIO and Eth PHY power?
  }


  #  endif // if FEATURE_ETHERNET

# endif // ifdef ESP32S2

  if (UsePSRAM()) {
    // PSRAM can use GPIO 16 and 17
    // There will be a high frequency signal on those pins (flash frequency)
    // which makes them unusable for other purposes.
    // WROVER does not even have these pins made available on the outside.
    switch (gpio) {
      case 16:
      case 17:
        warning = true;
        break;
    }
  }
  return true;
}

bool getGpioPullResistor(int gpio, bool& hasPullUp, bool& hasPullDown) {
  hasPullDown = false;
  hasPullUp   = false;

  int pinnr;
  bool input;
  bool output;
  bool warning;

  if (!getGpioInfo(gpio, pinnr, input, output, warning)) {
    return false;
  }

# ifdef ESP32S2

  if (gpio <= 45) {
    hasPullUp   = true;
    hasPullDown = true;
  }
# else // ifdef ESP32S2

  // ESP32 classic
  if (gpio >= 34) {
    // For GPIO 34 .. 39, no pull-up nor pull-down.
  } else if (gpio == 12) {
    // No Pull-up on GPIO12
    // compatible with the SDIO protocol.
    // Just connect GPIO12 to VDD via a 10 kOhm resistor.
  } else {
    hasPullUp   = true;
    hasPullDown = true;
  }

# endif // ifdef ESP32S2
  return true;
}

#endif // ifdef ESP32

#ifdef ESP8266

// return true when pin can be used.
bool getGpioInfo(int gpio, int& pinnr, bool& input, bool& output, bool& warning) {
  pinnr  = -1;
  input  = true;
  output = true;

  // GPIO 0, 2 & 15 can't be used as an input. State during boot is dependent on boot mode.
  warning = (gpio == 0 || gpio == 2 || gpio == 15);

  switch (gpio) {
    case  0: pinnr =  3; break;
    case  1: pinnr = 10; break;
    case  2: pinnr =  4; break;
    case  3: pinnr =  9; break;
    case  4: pinnr =  2; break;
    case  5: pinnr =  1; break;
    case  6:                    // GPIO 6 .. 8  is used for flash
    case  7:
    case  8: pinnr = -1; break;
    case  9: pinnr = 11; break; // On ESP8266 used for flash
    case 10: pinnr = 12; break; // On ESP8266 used for flash
    case 11: pinnr = -1; break;
    case 12: pinnr =  6; break;
    case 13: pinnr =  7; break;
    case 14: pinnr =  5; break;

    // GPIO-15 Can't be used as an input. There is an external pull-down on this pin.
    case 15: pinnr =  8; input = false; break;
    case 16: pinnr =  0; break; // This is used by the deep-sleep mechanism
  }

  if (isFlashInterfacePin(gpio)) {
    if (isESP8285()) {
      if ((gpio == 9) || (gpio == 10)) {
        // Usable on ESP8285
      } else {
        warning = true;
      }
    } else {
      warning = true;
      // On ESP8266 GPIO 9 & 10 are only usable if not connected to flash 
      if (gpio == 9) {
        // GPIO9 is internally used to control the flash memory.
        input  = false;
        output = false;
      } else if (gpio == 10) {
        // GPIO10 can be used as input only.
        output = false;
      }
    }
  }

  if ((pinnr < 0) || (pinnr > 16)) {
    input  = false;
    output = false;
  }
  return input || output;
}

bool getGpioPullResistor(int gpio, bool& hasPullUp, bool& hasPullDown) {
  hasPullDown = false;
  hasPullUp   = false;

  if (!validGpio(gpio)) {
    return false;
  }

  if (gpio == 16) {
    hasPullDown = true;
  } else {
    hasPullUp = true;
  }
  return true;
}

#endif // ifdef ESP8266

bool validGpio(int gpio) {
  if ((gpio < 0) || (gpio > MAX_GPIO)) { return false; }
  int pinnr;
  bool input;
  bool output;
  bool warning;

  return getGpioInfo(gpio, pinnr, input, output, warning);
}

#ifdef ESP32

// Get ADC related info for a given GPIO pin
// @param gpio_pin   GPIO pin number
// @param adc        Number of ADC unit (0 == Hall effect)
// @param ch         Channel number on ADC unit
// @param t          index of touch pad ID
bool getADC_gpio_info(int gpio_pin, int& adc, int& ch, int& t)
{
  adc = -1;
  ch  = -1;
  t   = -1;

# ifdef ESP32S2

  switch (gpio_pin) {
    case 1: adc  = 1; ch = 0; t = 1; break;
    case 2: adc  = 1; ch = 1; t = 2; break;
    case 3: adc  = 1; ch = 2; t = 3; break;
    case 4: adc  = 1; ch = 3; t = 4; break;
    case 5: adc  = 1; ch = 4; t = 5; break;
    case 6: adc  = 1; ch = 5; t = 6; break;
    case 7: adc  = 1; ch = 6; t = 7; break;
    case 8: adc  = 1; ch = 7; t = 8; break;
    case 9: adc  = 1; ch = 8; t = 9; break;
    case 10: adc = 1; ch = 9; t = 10; break;
    case 11: adc = 2; ch = 0; t = 11; break;
    case 12: adc = 2; ch = 1; t = 12; break;
    case 13: adc = 2; ch = 2; t = 13; break;
    case 14: adc = 2; ch = 3; t = 14; break;
    case 15: adc = 2; ch = 4;  break;
    case 16: adc = 2; ch = 5;  break;
    case 17: adc = 2; ch = 6;  break;
    case 18: adc = 2; ch = 7;  break;
    case 19: adc = 2; ch = 8;  break;
    case 20: adc = 2; ch = 9;  break;
    default:
      return false;
  }
# else // ifdef ESP32S2

  // Classic ESP32
  switch (gpio_pin) {
    case -1: adc = 0; break; // Hall effect Sensor
    case 36: adc = 1; ch = 0; break;
    case 37: adc = 1; ch = 1; break;
    case 38: adc = 1; ch = 2; break;
    case 39: adc = 1; ch = 3; break;
    case 32: adc = 1; ch = 4; t = 9; break;
    case 33: adc = 1; ch = 5; t = 8; break;
    case 34: adc = 1; ch = 6; break;
    case 35: adc = 1; ch = 7; break;
    case 4:  adc = 2; ch = 0; t = 0; break;
    case 0:  adc = 2; ch = 1; t = 1; break;
    case 2:  adc = 2; ch = 2; t = 2; break;
    case 15: adc = 2; ch = 3; t = 3; break;
    case 13: adc = 2; ch = 4; t = 4; break;
    case 12: adc = 2; ch = 5; t = 5; break;
    case 14: adc = 2; ch = 6; t = 6; break;
    case 27: adc = 2; ch = 7; t = 7; break;
    case 25: adc = 2; ch = 8; break;
    case 26: adc = 2; ch = 9; break;
    default:
      return false;
  }
# endif // ifdef ESP32S2
  return true;
}

int touchPinToGpio(int touch_pin)
{
# ifdef ESP32S2

  switch (touch_pin) {
    case 1: return T1;
    case 2: return T2;
    case 3: return T3;
    case 4: return T4;
    case 5: return T5;
    case 6: return T6;
    case 7: return T7;
    case 8: return T8;
    case 9: return T9;
    case 10: return T10;
    case 11: return T11;
    case 12: return T12;
    case 13: return T13;
    case 14: return T14;
    default:
      break;
  }
# else // ifdef ESP32S2

  // ESP32 classic
  switch (touch_pin) {
    case 0: return T0;
    case 1: return T1;
    case 2: return T2;
    case 3: return T3;
    case 4: return T4;
    case 5: return T5;
    case 6: return T6;
    case 7: return T7;
    case 8: return T8;
    case 9: return T9;
    default:
      break;
  }
# endif // ifdef ESP32S2
  return -1;
}

#endif // ifdef ESP32

// ********************************************************************************
// Manage PWM state of GPIO pins.
// ********************************************************************************
void initAnalogWrite()
{
  #if defined(ESP32)

  for (uint8_t x = 0; x < 16; x++) {
    ledChannelPin[x]  = -1;
    ledChannelFreq[x] = ledcSetup(x, 1000, 10); // Clear the channel
  }
  #endif // if defined(ESP32)
  #ifdef ESP8266

  // See https://github.com/esp8266/Arduino/commit/a67986915512c5304bd7c161cf0d9c65f66e0892
  analogWriteRange(1023);
  #endif // ifdef ESP8266
}

#if defined(ESP32)
int8_t ledChannelPin[16];
uint32_t ledChannelFreq[16] = { 0 };


int8_t attachLedChannel(int pin, uint32_t frequency)
{
  static bool initialized = false;

  if (!initialized) {
    for (uint8_t x = 0; x < 16; x++) {
      ledChannelPin[x]  = -1;
      ledChannelFreq[x] = 0;
    }
    initialized = true;
  }


  // find existing channel if this pin has been used before
  int8_t ledChannel = -1;
  bool mustSetup    = false;

  for (uint8_t x = 0; x < 16; x++) {
    if (ledChannelPin[x] == pin) {
      ledChannel = x;
    }
  }

  if (ledChannel == -1)                                    // no channel set for this pin
  {
    for (uint8_t x = 0; x < 16 && ledChannel == -1; ++x) { // find free channel
      if (ledChannelPin[x] == -1)
      {
        if (static_cast<uint32_t>(ledcReadFreq(x)) == ledChannelFreq[x]) {
          // Channel is not used by some other piece of code.
          ledChannel = x;
          mustSetup  = true;
          break;
        }
      }
    }
  }

  if (ledChannel == -1) { return ledChannel; }

  if (frequency != 0) {
    if (ledChannelFreq[ledChannel] != frequency)
    {
      // Frequency is given and has changed
      mustSetup = true;
    }
    ledChannelFreq[ledChannel] = frequency;
  } else if (ledChannelFreq[ledChannel] == 0) {
    mustSetup = true;

    // Set some default frequency
    ledChannelFreq[ledChannel] = 1000;
  }

  if (mustSetup) {
    // setup channel to 10 bit and set frequency.
    ledChannelFreq[ledChannel] = ledcSetup(ledChannel, ledChannelFreq[ledChannel], 10);
    ledChannelPin[ledChannel]  = pin; // store pin nr
    ledcAttachPin(pin, ledChannel);   // attach to this pin
    //    pinMode(pin, OUTPUT);
  }

  return ledChannel;
}

void detachLedChannel(int pin)
{
  int8_t ledChannel = -1;

  for (uint8_t x = 0; x < 16; x++) {
    if (ledChannelPin[x] == pin) {
      ledChannel = x;
    }
  }

  if (ledChannel != -1) {
    ledcWrite(ledChannel, 0);
    ledcDetachPin(pin);
    ledChannelPin[ledChannel]  = -1;
    ledChannelFreq[ledChannel] = 0;
  }
}

uint32_t analogWriteESP32(int pin, int value, uint32_t frequency)
{
  if (value == 0) {
    detachLedChannel(pin);
    return 0;
  }

  // find existing channel if this pin has been used before
  int8_t ledChannel = attachLedChannel(pin, frequency);

  if (ledChannel != -1) {
    ledcWrite(ledChannel, value);
    return ledChannelFreq[ledChannel];
  }
  return 0;
}

#endif // if defined(ESP32)

bool set_Gpio_PWM_pct(int gpio, float dutyCycle_f, uint32_t frequency) {
  uint32_t dutyCycle = dutyCycle_f * 10.23f;

  return set_Gpio_PWM(gpio, dutyCycle, frequency);
}

bool set_Gpio_PWM(int gpio, uint32_t dutyCycle, uint32_t frequency) {
  uint32_t key;

  return set_Gpio_PWM(gpio, dutyCycle, 0, frequency, key);
}

bool set_Gpio_PWM(int gpio, uint32_t dutyCycle, uint32_t fadeDuration_ms, uint32_t& frequency, uint32_t& key)
{
  // For now, we only support the internal GPIO pins.
  uint8_t   pluginID = PLUGIN_GPIO;

  if (!checkValidPortRange(pluginID, gpio)) {
    return false;
  }
  portStatusStruct tempStatus;

  // FIXME TD-er: PWM values cannot be stored very well in the portStatusStruct.
  key = createKey(pluginID, gpio);

  // WARNING: operator [] creates an entry in the map if key does not exist
  // So the next command should be part of each command:
  tempStatus = globalMapPortStatus[key];

        #if defined(ESP8266)
  pinMode(gpio, OUTPUT);
        #endif // if defined(ESP8266)

  #if defined(ESP8266)

  if ((frequency > 0) && (frequency <= 40000)) {
    analogWriteFreq(frequency);
  }
  #endif // if defined(ESP8266)

  if (fadeDuration_ms != 0)
  {
    const int32_t resolution_factor = (1 << 12);
    const uint8_t prev_mode         = tempStatus.mode;
    int32_t   prev_value            = tempStatus.getDutyCycle();

    // getPinState(pluginID, gpio, &prev_mode, &prev_value);
    if (prev_mode != PIN_MODE_PWM) {
      prev_value = 0;
    }

    const int32_t step_value = ((static_cast<int32_t>(dutyCycle) - prev_value) * resolution_factor) / static_cast<int32_t>(fadeDuration_ms);
    int32_t curr_value       = prev_value * resolution_factor;

    int i = fadeDuration_ms;

    while (i--) {
      curr_value += step_value;
      const int16_t new_value = curr_value / resolution_factor;
            #if defined(ESP8266)
      analogWrite(gpio, new_value);
            #endif // if defined(ESP8266)
            #if defined(ESP32)
      frequency = analogWriteESP32(gpio, new_value, frequency);
            #endif // if defined(ESP32)
      delay(1);
    }
  }

        #if defined(ESP8266)
  analogWrite(gpio, dutyCycle);
        #endif // if defined(ESP8266)
        #if defined(ESP32)
  frequency = analogWriteESP32(gpio, dutyCycle, frequency);
        #endif // if defined(ESP32)

  // setPinState(pluginID, gpio, PIN_MODE_PWM, dutyCycle);
  tempStatus.mode      = PIN_MODE_PWM;
  tempStatus.dutyCycle = dutyCycle;
  tempStatus.command   = 1; // set to 1 in order to display the status in the PinStatus page

  savePortStatus(key, tempStatus);
  return true;
}

// ********************************************************************************
// change of device: cleanup old device and reset default settings
// ********************************************************************************
void setTaskDevice_to_TaskIndex(pluginID_t taskdevicenumber, taskIndex_t taskIndex) {
  struct EventStruct TempEvent(taskIndex);
  String dummy;

  // let the plugin do its cleanup by calling PLUGIN_EXIT with this TaskIndex
  PluginCall(PLUGIN_EXIT, &TempEvent, dummy);
  taskClear(taskIndex, false); // clear settings, but do not save
  ClearCustomTaskSettings(taskIndex);

  Settings.TaskDeviceNumber[taskIndex] = taskdevicenumber;

  if (validPluginID_fullcheck(taskdevicenumber)) // set default values if a new device has been selected
  {
    // FIXME TD-er: Must check if this is working (e.g. need to set nr. decimals?)
    ExtraTaskSettings.clear();
    ExtraTaskSettings.TaskIndex = taskIndex;

    // NOTE: do not enable task by default. allow user to enter sensible valus first and let him enable it when ready.
    PluginCall(PLUGIN_SET_DEFAULTS,         &TempEvent, dummy);
    PluginCall(PLUGIN_GET_DEVICEVALUENAMES, &TempEvent, dummy); // the plugin should populate ExtraTaskSettings with its default values.
  } else {
    // New task is empty task, thus save config now.
    taskClear(taskIndex, true);                                 // clear settings, and save
  }
}

// ********************************************************************************
// Initialize task with some default values applicable for almost all tasks
// ********************************************************************************
void setBasicTaskValues(taskIndex_t taskIndex, unsigned long taskdevicetimer,
                        bool enabled, const String& name, int pin1, int pin2, int pin3) {
  if (!validTaskIndex(taskIndex)) { return; }
  const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(taskIndex);

  if (!validDeviceIndex(DeviceIndex)) { return; }

  LoadTaskSettings(taskIndex); // Make sure ExtraTaskSettings are up-to-date

  if (taskdevicetimer > 0) {
    Settings.TaskDeviceTimer[taskIndex] = taskdevicetimer;
  } else {
    if (!Device[DeviceIndex].TimerOptional) { // Set default delay, unless it's optional...
      Settings.TaskDeviceTimer[taskIndex] = Settings.Delay;
    }
    else {
      Settings.TaskDeviceTimer[taskIndex] = 0;
    }
  }
  Settings.TaskDeviceEnabled[taskIndex] = enabled;
  safe_strncpy(ExtraTaskSettings.TaskDeviceName, name.c_str(), sizeof(ExtraTaskSettings.TaskDeviceName));

  // FIXME TD-er: Check for valid GPIO pin (and  -1 for "not set")
  Settings.TaskDevicePin1[taskIndex] = pin1;
  Settings.TaskDevicePin2[taskIndex] = pin2;
  Settings.TaskDevicePin3[taskIndex] = pin3;
}

#include "../Helpers/StringGenerator_GPIO.h"

#include "../Globals/Settings.h"
#include "../Helpers/Hardware.h"
#include "../../ESPEasy_common.h"

/*********************************************************************************************\
   Device GPIO name functions to share flash strings
\*********************************************************************************************/
const __FlashStringHelper* formatGpioDirection(gpio_direction direction) {
  switch (direction) {
    case gpio_input:         return F("&larr; ");
    case gpio_output:        return F("&rarr; ");
    case gpio_bidirectional: return F("&#8644; ");
  }
  return F("");
}

String formatGpioLabel(int gpio, bool includeWarning) {
  int  pinnr = -1;
  bool input, output, warning;

  if (getGpioInfo(gpio, pinnr, input, output, warning)) {
    if (!includeWarning) {
      return createGPIO_label(gpio, pinnr, true, true, false);
    }
    return createGPIO_label(gpio, pinnr, input, output, warning);
  }
  return F("-");
}

String formatGpioName(const __FlashStringHelper * label, gpio_direction direction, bool optional) {
  int reserveLength = 5 /* "GPIO " */ + 8 /* "&#8644; " */ + strlen_P((PGM_P)label);

  if (optional) {
    reserveLength += 11;
  }
  String result;

  result.reserve(reserveLength);
  result += F("GPIO ");
  result += formatGpioDirection(direction);
  result += label;

  if (optional) {
    result += F("(optional)");
  }
  return result;
}

String formatGpioName_input(const __FlashStringHelper * label) {
  return formatGpioName(label, gpio_input, false);
}

String formatGpioName_output(const __FlashStringHelper * label) {
  return formatGpioName(label, gpio_output, false);
}

String formatGpioName_bidirectional(const __FlashStringHelper * label) {
  return formatGpioName(label, gpio_bidirectional, false);
}

String formatGpioName_input_optional(const __FlashStringHelper * label) {
  return formatGpioName(label, gpio_input, true);
}

String formatGpioName_output_optional(const __FlashStringHelper * label) {
  return formatGpioName(label, gpio_output, true);
}

// RX/TX are the only signals which are crossed, so they must be labelled like this:
// "GPIO <-- TX" and "GPIO --> RX"
String formatGpioName_TX(bool optional) {
  return formatGpioName(F("RX"), gpio_output, optional);
}

String formatGpioName_RX(bool optional) {
  return formatGpioName(F("TX"), gpio_input, optional);
}

String formatGpioName_TX_HW(bool optional) {
  return formatGpioName(F("RX (HW)"), gpio_output, optional);
}

String formatGpioName_RX_HW(bool optional) {
  return formatGpioName(F("TX (HW)"), gpio_input, optional);
}

#ifdef ESP32

String formatGpioName_ADC(int gpio_pin) {
  int adc, ch, t;

  if (getADC_gpio_info(gpio_pin, adc, ch, t)) {
    if (adc == 0) {
      return F("Hall Effect");
    }
    String res = F("ADC# ch?");
    res.replace(F("#"), String(adc));
    res.replace(F("?"), String(ch));

    if (t >= 0) {
      res += F(" (T");
      res += t;
      res += ')';
    }
    return res;
  }
  return "";
}

#endif // ifdef ESP32

// ********************************************************************************
// Add a GPIO pin select dropdown list for 8266, 8285 or ESP32
// ********************************************************************************
String createGPIO_label(int gpio, int pinnr, bool input, bool output, bool warning) {
  if (gpio < 0) { return F("- None -"); }
  String result;

  result.reserve(24);
  result  = F("GPIO-");
  result += gpio;

  if (pinnr >= 0) {
    result += F(" (D");
    result += pinnr;
    result += ')';
  }

  if (input != output) {
    result += ' ';
    result += input ? F(HTML_SYMBOL_INPUT) : F(HTML_SYMBOL_OUTPUT);
  }

  if (warning) {
    result += ' ';
    result += F(HTML_SYMBOL_WARNING);
  }
  return result;
}

const __FlashStringHelper* getConflictingUse(int gpio, PinSelectPurpose purpose)
{
  if (Settings.UseSerial) {
    if (gpio == 1) { return F("TX0"); }

    if (gpio == 3) { return F("RX0"); }
  }
  bool includeI2C = true;
  bool includeSPI = true;

  #if FEATURE_ETHERNET
  bool includeEthernet = true;
  #endif // if FEATURE_ETHERNET

  switch (purpose) {
    case PinSelectPurpose::I2C:
      includeI2C = false;
      break;
    case PinSelectPurpose::SPI:
      includeSPI = false;
      break;
    case PinSelectPurpose::Ethernet:
      #if FEATURE_ETHERNET
      includeEthernet = false;
      #endif // if FEATURE_ETHERNET
      break;
    case PinSelectPurpose::Generic:
    case PinSelectPurpose::Generic_input:
    case PinSelectPurpose::Generic_output:
    case PinSelectPurpose::Generic_bidir:
      break;
  }

  if (includeI2C && Settings.isI2C_pin(gpio)) {
    return (Settings.Pin_i2c_sda == gpio) ?  F("I2C SDA") : F("I2C SCL");
  }

  if (includeSPI && Settings.isSPI_pin(gpio)) {
    return F("SPI");
  }
  #if FEATURE_ETHERNET

  if (Settings.isEthernetPin(gpio)) {
    return F("Eth");
  }

  if (includeEthernet && Settings.isEthernetPinOptional(gpio)) {
    if (Settings.ETH_Pin_mdc == gpio) { return F("Eth MDC"); }

    if (Settings.ETH_Pin_mdio == gpio) { return F("Eth MDIO"); }

    if (Settings.ETH_Pin_power == gpio) { return F("Eth Pwr"); }

    return F("Eth");
  }
  #endif // if FEATURE_ETHERNET

#ifdef ESP32
  if (UsePSRAM()) {
    // PSRAM can use GPIO 16 and 17
    switch (gpio) {
      case 16:
      case 17:
        return F("PSRAM");
    }
  }
#endif

  return F("");
}

String getConflictingUse_wrapped(int gpio, PinSelectPurpose purpose)
{
  String conflict = getConflictingUse(gpio, purpose);

  if (conflict.isEmpty()) { return conflict; }
  String res = F(" [");

  res += conflict;
  res += ']';
  return res;
}

#include "../Helpers/ESPEasy_time_zone.h"

#include "../DataStructs/TimeChangeRule.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/ESPEasy_time.h"
#include "../Globals/Settings.h"

#include "../Helpers/ESPEasy_time_calc.h"

#include <time.h>


#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)



void ESPEasy_time_zone::getDefaultDst_flash_values(uint16_t& start, uint16_t& end) {
  // DST start: Last Sunday March    2am => 3am
  // DST end:   Last Sunday October  3am => 2am
  TimeChangeRule CEST(TimeChangeRule::Last, TimeChangeRule::Sun, TimeChangeRule::Mar, 2, Settings.TimeZone); // Summer Time
  TimeChangeRule CET(TimeChangeRule::Last, TimeChangeRule::Sun, TimeChangeRule::Oct, 3, Settings.TimeZone);  // Standard Time

  start = CEST.toFlashStoredValue();
  end   = CET.toFlashStoredValue();
}

void ESPEasy_time_zone::applyTimeZone(uint32_t curTime) {
  int dst_offset = Settings.DST ? 60 : 0;
  uint16_t tmpStart(Settings.DST_Start);
  uint16_t tmpEnd(Settings.DST_End);

  for (int i = 0; i < 2; ++i) {
    TimeChangeRule start(tmpStart, Settings.TimeZone + dst_offset); // Summer Time
    TimeChangeRule end(tmpEnd, Settings.TimeZone);                  // Standard Time

    if (start.isValid() && end.isValid()) {
      setTimeZone(start, end, curTime);
      return;
    }
    getDefaultDst_flash_values(tmpStart, tmpEnd);
  }
}

void ESPEasy_time_zone::setTimeZone(const TimeChangeRule& dstStart, const TimeChangeRule& stdStart, uint32_t curTime) {
  m_dst = dstStart;
  m_std = stdStart;

  if (calcTimeChanges(ESPEasy_time::year(curTime))) {
    logTimeZoneInfo();
  }
}

void ESPEasy_time_zone::logTimeZoneInfo() {
  if (!loglevelActiveFor(LOG_LEVEL_INFO)) return;
  String log = F("Current Time Zone: ");

  if (m_std.offset != m_dst.offset) {
    // Summer time
    log += F(" DST time start: ");

    if (m_dstLoc != 0) {
      struct tm tmp;
      ESPEasy_time::breakTime(m_dstLoc, tmp);
      log += ESPEasy_time::getDateTimeString(tmp, '-', ':', ' ', false);
    }
    log += F(" offset: ");
    log += m_dst.offset;
    log += F(" min ");
  }

  // Standard/Winter time.
  log += F("STD time start: ");

  if (m_stdLoc != 0) {
    struct tm tmp;
    ESPEasy_time::breakTime(m_stdLoc, tmp);
    log += ESPEasy_time::getDateTimeString(tmp, '-', ':', ' ', false);
  }
  log += F(" offset: ");
  log += m_std.offset;
  log += F(" min");
  addLogMove(LOG_LEVEL_INFO, log);
}


///*----------------------------------------------------------------------*
// * Convert the given time change rule to a uint32_t value                 *
// * for the given year.                                                  *
// *----------------------------------------------------------------------*/
uint32_t ESPEasy_time_zone::calcTimeChangeForRule(const TimeChangeRule& r, int yr)
{
  uint8_t m = r.month;
  uint8_t w = r.week;

  if (w == 0)          // is this a "Last week" rule?
  {
    if (++m > 12)      // yes, for "Last", go to the next month
    {
      m = 1;
      ++yr;
    }
    w = 1; // and treat as first week of next month, subtract 7 days later
  }

  // calculate first day of the month, or for "Last" rules, first day of the next month
  struct tm tm;
  tm.tm_hour = r.hour;
  tm.tm_min  = 0;
  tm.tm_sec  = 0;
  tm.tm_mday = 1;
  tm.tm_mon  = m - 1; // TimeChangeRule month starts at 1
  tm.tm_year = yr - 1900;
  uint32_t t = makeTime(tm);

  // add offset from the first of the month to r.dow, and offset for the given week
  t += ((r.dow - ESPEasy_time::weekday(t) + 7) % 7 + (w - 1) * 7) * SECS_PER_DAY;

  // back up a week if this is a "Last" rule
  if (r.week == 0) { t -= 7 * SECS_PER_DAY; }
  return t;
}

/*----------------------------------------------------------------------*
* Calculate the DST and standard time change points for the given      *
* given year as local and UTC uint32_t values.                           *
*----------------------------------------------------------------------*/
bool ESPEasy_time_zone::calcTimeChanges(int yr)
{
  uint32_t dstLoc  = calcTimeChangeForRule(m_dst, yr);
  uint32_t stdLoc  = calcTimeChangeForRule(m_std, yr);
  bool     changed = (m_dstLoc != dstLoc) || (m_stdLoc != stdLoc);

  m_dstLoc = dstLoc;
  m_stdLoc = stdLoc;
  m_dstUTC = m_dstLoc - m_std.offset * SECS_PER_MIN;
  m_stdUTC = m_stdLoc - m_dst.offset * SECS_PER_MIN;
  return changed;
}

/*----------------------------------------------------------------------*
* Convert the given UTC time to local time, standard or                *
* daylight time, as appropriate.                                       *
*----------------------------------------------------------------------*/
uint32_t ESPEasy_time_zone::toLocal(uint32_t utc)
{
  // recalculate the time change points if needed
  if (ESPEasy_time::year(utc) != ESPEasy_time::year(m_dstUTC)) { calcTimeChanges(ESPEasy_time::year(utc)); }

  if (utcIsDST(utc)) {
    return utc + m_dst.offset * SECS_PER_MIN;
  }
  else {
    return utc + m_std.offset * SECS_PER_MIN;
  }
}

/*----------------------------------------------------------------------*
* Determine whether the given UTC uint32_t is within the DST interval    *
* or the Standard time interval.                                       *
*----------------------------------------------------------------------*/
bool ESPEasy_time_zone::utcIsDST(uint32_t utc)
{
  // recalculate the time change points if needed
  if (ESPEasy_time::year(utc) != ESPEasy_time::year(m_dstUTC)) { calcTimeChanges(ESPEasy_time::year(utc)); }

  if (m_stdUTC == m_dstUTC) {     // daylight time not observed in this tz
    return false;
  }
  else if (m_stdUTC > m_dstUTC) { // northern hemisphere
    return utc >= m_dstUTC && utc < m_stdUTC;
  }
  else {                          // southern hemisphere
    return !(utc >= m_stdUTC && utc < m_dstUTC);
  }
}

/*----------------------------------------------------------------------*
* Determine whether the given Local uint32_t is within the DST interval  *
* or the Standard time interval.                                       *
*----------------------------------------------------------------------*/
bool ESPEasy_time_zone::locIsDST(uint32_t local)
{
  // recalculate the time change points if needed
  if (ESPEasy_time::year(local) != ESPEasy_time::year(m_dstLoc)) { calcTimeChanges(ESPEasy_time::year(local)); }

  if (m_stdUTC == m_dstUTC) {     // daylight time not observed in this tz
    return false;
  }
  else if (m_stdLoc > m_dstLoc) { // northern hemisphere
    return local >= m_dstLoc && local < m_stdLoc;
  }
  else {                          // southern hemisphere
    return !(local >= m_stdLoc && local < m_dstLoc);
  }
}



#include "../Helpers/_CPlugin_Helper.h"

#include "../../ESPEasy_common.h"

#include "../CustomBuild/CompiletimeDefines.h"
#include "../CustomBuild/ESPEasyLimits.h"

#include "../DataStructs/SecurityStruct.h"
#include "../DataStructs/SettingsStruct.h"

#include "../DataStructs/ControllerSettingsStruct.h"
#include "../DataStructs/TimingStats.h"

#include "../ESPEasyCore/ESPEasy_backgroundtasks.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyEth.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../ESPEasyCore/ESPEasyWifi.h"

#include "../Globals/Settings.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/ESPEasyWiFiEvent.h"

#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Network.h"
#include "../Helpers/Networking.h"
#include "../Helpers/StringConverter.h"

#include <WiFiClient.h>
#include <WiFiUdp.h>


bool safeReadStringUntil(Stream     & input,
                         String     & str,
                         char         terminator,
                         unsigned int maxSize,
                         unsigned int timeout)
{
  int c;
  const unsigned long start           = millis();
  const unsigned long timer           = start + timeout;
  unsigned long backgroundtasks_timer = start + 10;

  str = String();

  do {
    // read character
    if (input.available()) {
      c = input.read();

      if (c >= 0) {
        // found terminator, we're ok
        if (c == terminator) {
          return true;
        }

        // found character, add to string
        str += char(c);

        // string at max size?
        if (str.length() >= maxSize) {
          addLog(LOG_LEVEL_ERROR, F("Not enough bufferspace to read all input data!"));
          return false;
        }
      }

      // We must run the backgroundtasks every now and then.
      if (timeOutReached(backgroundtasks_timer)) {
        backgroundtasks_timer += 10;
        backgroundtasks();
      } else {
        delay(0);
      }
    } else {
      delay(0);
    }
  } while (!timeOutReached(timer));

  addLog(LOG_LEVEL_ERROR, F("Timeout while reading input data!"));
  return false;
}

#ifndef BUILD_NO_DEBUG
void log_connecting_to(const __FlashStringHelper *prefix, int controller_number, ControllerSettingsStruct& ControllerSettings) {
  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = prefix;
    log += get_formatted_Controller_number(controller_number);
    log += F(" connecting to ");
    log += ControllerSettings.getHostPortString();
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
}

#endif // ifndef BUILD_NO_DEBUG

void log_connecting_fail(const __FlashStringHelper *prefix, int controller_number) {
  if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
    String log = prefix;
    log += get_formatted_Controller_number(controller_number);
    log += F(" connection failed (");
    log += WiFiEventData.connectionFailures;
    log += F("/");
    log += Settings.ConnectionFailuresThreshold;
    log += F(")");
    addLogMove(LOG_LEVEL_ERROR, log);
  }
}

bool count_connection_results(bool success, const __FlashStringHelper *prefix, int controller_number) {
  if (!success)
  {
    ++WiFiEventData.connectionFailures;
    log_connecting_fail(prefix, controller_number);
    return false;
  }
  statusLED(true);

  if (WiFiEventData.connectionFailures > 0) {
    --WiFiEventData.connectionFailures;
  }
  return true;
}

bool try_connect_host(int controller_number, WiFiUDP& client, ControllerSettingsStruct& ControllerSettings) {
  START_TIMER;

  if (!NetworkConnected()) { return false; }
  #ifdef MUSTFIX_CLIENT_TIMEOUT_IN_SECONDS

  // See: https://github.com/espressif/arduino-esp32/pull/6676
  client.setTimeout((ControllerSettings.ClientTimeout + 500) / 1000); // in seconds!!!!
  #else // ifdef MUSTFIX_CLIENT_TIMEOUT_IN_SECONDS
  client.setTimeout(ControllerSettings.ClientTimeout);                // in msec as it should be!
  #endif // ifdef MUSTFIX_CLIENT_TIMEOUT_IN_SECONDS
  delay(0);
#ifndef BUILD_NO_DEBUG
  log_connecting_to(F("UDP  : "), controller_number, ControllerSettings);
#endif // ifndef BUILD_NO_DEBUG
  bool success      = ControllerSettings.beginPacket(client);
  const bool result = count_connection_results(
    success,
    F("UDP  : "), controller_number);
  STOP_TIMER(TRY_CONNECT_HOST_UDP);
  return result;
}

bool try_connect_host(int controller_number, WiFiClient& client, ControllerSettingsStruct& ControllerSettings) {
  return try_connect_host(controller_number, client, ControllerSettings, F("HTTP : "));
}

bool try_connect_host(int                        controller_number,
                      WiFiClient               & client,
                      ControllerSettingsStruct & ControllerSettings,
                      const __FlashStringHelper *loglabel) {
  START_TIMER;

  if (!NetworkConnected()) { return false; }

  // Use WiFiClient class to create TCP connections
  delay(0);
  #ifdef MUSTFIX_CLIENT_TIMEOUT_IN_SECONDS

  // See: https://github.com/espressif/arduino-esp32/pull/6676
  client.setTimeout((ControllerSettings.ClientTimeout + 500) / 1000); // in seconds!!!!
  #else // ifdef MUSTFIX_CLIENT_TIMEOUT_IN_SECONDS
  client.setTimeout(ControllerSettings.ClientTimeout);                // in msec as it should be!
  #endif // ifdef MUSTFIX_CLIENT_TIMEOUT_IN_SECONDS

#ifndef BUILD_NO_DEBUG
  log_connecting_to(loglabel, controller_number, ControllerSettings);
#endif // ifndef BUILD_NO_DEBUG
  const bool success = ControllerSettings.connectToHost(client);
  const bool result  = count_connection_results(
    success,
    loglabel, controller_number);
  STOP_TIMER(TRY_CONNECT_HOST_TCP);
  return result;
}

// Use "client.available() || client.connected()" to read all lines from slow servers.
// See: https://github.com/esp8266/Arduino/pull/5113
//      https://github.com/esp8266/Arduino/pull/1829
bool client_available(WiFiClient& client) {
  delay(0);
  return (client.available() != 0) || (client.connected() != 0);
}

String send_via_http(int                             controller_number,
                     const ControllerSettingsStruct& ControllerSettings,
                     controllerIndex_t               controller_idx,
                     WiFiClient                    & client,
                     const String                  & uri,
                     const String                  & HttpMethod,
                     const String                  & header,
                     const String                  & postStr,
                     int                           & httpCode) {
  const String result = send_via_http(
    get_formatted_Controller_number(controller_number),
    client,
    ControllerSettings.ClientTimeout,
    getControllerUser(controller_idx, ControllerSettings),
    getControllerPass(controller_idx, ControllerSettings),
    ControllerSettings.getHost(),
    ControllerSettings.Port,
    uri,
    HttpMethod,
    header,
    postStr,
    httpCode,
    ControllerSettings.MustCheckReply);

  // FIXME TD-er: Shouldn't this be: success = (httpCode >= 100) && (httpCode < 300)
  // or is reachability of the host the important factor here?
  const bool success = httpCode > 0;

  count_connection_results(
    success,
    F("HTTP  : "),
    controller_number);

  return result;
}





String getControllerUser(controllerIndex_t controller_idx, const ControllerSettingsStruct& ControllerSettings)
{
  if (!validControllerIndex(controller_idx)) { return EMPTY_STRING; }

  if (ControllerSettings.useExtendedCredentials()) {
    return ExtendedControllerCredentials.getControllerUser(controller_idx);
  }
  return SecuritySettings.ControllerUser[controller_idx];
}

String getControllerPass(controllerIndex_t controller_idx, const ControllerSettingsStruct& ControllerSettings)
{
  if (!validControllerIndex(controller_idx)) { return EMPTY_STRING; }

  if (ControllerSettings.useExtendedCredentials()) {
    return ExtendedControllerCredentials.getControllerPass(controller_idx);
  }
  return SecuritySettings.ControllerPassword[controller_idx];
}

void setControllerUser(controllerIndex_t controller_idx, const ControllerSettingsStruct& ControllerSettings, const String& value)
{
  if (!validControllerIndex(controller_idx)) { return; }

  if (ControllerSettings.useExtendedCredentials()) {
    ExtendedControllerCredentials.setControllerUser(controller_idx, value);
  } else {
    safe_strncpy(SecuritySettings.ControllerUser[controller_idx], value, sizeof(SecuritySettings.ControllerUser[0]));
  }
}

void setControllerPass(controllerIndex_t controller_idx, const ControllerSettingsStruct& ControllerSettings, const String& value)
{
  if (!validControllerIndex(controller_idx)) { return; }

  if (ControllerSettings.useExtendedCredentials()) {
    ExtendedControllerCredentials.setControllerPass(controller_idx, value);
  } else {
    safe_strncpy(SecuritySettings.ControllerPassword[controller_idx], value, sizeof(SecuritySettings.ControllerPassword[0]));
  }
}

bool hasControllerCredentialsSet(controllerIndex_t controller_idx, const ControllerSettingsStruct& ControllerSettings)
{
  return !getControllerUser(controller_idx, ControllerSettings).isEmpty() &&
         !getControllerPass(controller_idx, ControllerSettings).isEmpty();
}

#include "../Helpers/StringConverter.h"


#include "../../_Plugin_Helper.h"

#include "../DataStructs/ESPEasy_EventStruct.h"

#include "../ESPEasyCore/ESPEasy_Log.h"

#include "../Globals/Cache.h"
#include "../Globals/CRCValues.h"
#include "../Globals/Device.h"
#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/ESPEasy_time.h"
#include "../Globals/MQTT.h"
#include "../Globals/Plugins.h"
#include "../Globals/Settings.h"

#include "../Helpers/Convert.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Networking.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/StringParser.h"
#include "../Helpers/SystemVariables.h"
#include "../Helpers/_Plugin_SensorTypeHelper.h"

// -V::569

/********************************************************************************************\
   Convert a char string to integer
 \*********************************************************************************************/

// FIXME: change original code so it uses String and String.toInt()
unsigned long str2int(const char *string)
{
  unsigned int temp = 0;

  validUIntFromString(string, temp);

  return static_cast<unsigned long>(temp);
}

String ull2String(uint64_t value, uint8_t base) {
  String res;

  if (value == 0) {
    res = '0';
    return res;
  }

  while (value > 0) {
    res   += String(static_cast<uint32_t>(value % base), base);
    value /= base;
  }

  int endpos   = res.length() - 1;
  int beginpos = 0;

  while (endpos > beginpos) {
    const char c = res[beginpos];
    res[beginpos] = res[endpos];
    res[endpos]   = c;
    ++beginpos;
    --endpos;
  }

  return res;
}

String ll2String(int64_t value, uint8_t  base) {
  if (value < 0) {
    String res;
    res = '-';
    res += ull2String(value * -1ll, base);
    return res;
  } else {
    return ull2String(value, base);
  }
}


/********************************************************************************************\
   Check if valid float and convert string to float.
 \*********************************************************************************************/
bool string2float(const String& string, float& floatvalue) {
  return validFloatFromString(string, floatvalue);
}

/********************************************************************************************\
   Convert a char string to IP uint8_t array
 \*********************************************************************************************/
bool isIP(const String& string) {
  IPAddress tmpip;
  return (tmpip.fromString(string));
}

bool str2ip(const String& string, uint8_t *IP) {
  return str2ip(string.c_str(), IP);
}

bool str2ip(const char *string, uint8_t *IP)
{
  IPAddress tmpip; // Default constructor => set to 0.0.0.0

  if ((*string == 0) || tmpip.fromString(string)) {
    // Eiher empty string or a valid IP addres, so copy value.
    for (uint8_t i = 0; i < 4; ++i) {
      IP[i] = tmpip[i];
    }
    return true;
  }
  return false;
}

String formatIP(const IPAddress& ip) {
#if defined(ARDUINO_ESP8266_RELEASE_2_3_0)
  IPAddress tmp(ip);
  return tmp.toString();
#else // if defined(ARDUINO_ESP8266_RELEASE_2_3_0)
  return ip.toString();
#endif // if defined(ARDUINO_ESP8266_RELEASE_2_3_0)
}


/********************************************************************************************\
   Handling HEX strings
 \*********************************************************************************************/

// Convert max. 8 hex decimals to unsigned long
unsigned long hexToUL(const String& input_c, size_t nrHexDecimals) {
  const unsigned long long resULL = hexToULL(input_c, nrHexDecimals);
  return static_cast<unsigned long>(resULL & 0xFFFFFFFFull);
}

unsigned long hexToUL(const String& input_c) {
  return hexToUL(input_c, input_c.length());
}

unsigned long hexToUL(const String& input_c, size_t startpos, size_t nrHexDecimals) {
  return hexToUL(input_c.substring(startpos, startpos + nrHexDecimals), nrHexDecimals);
}

// Convert max. 16 hex decimals to unsigned long long (aka uint64_t)
unsigned long long hexToULL(const String& input_c, size_t nrHexDecimals) {
  size_t nr_decimals = nrHexDecimals;

  if (nr_decimals > 16) {
    nr_decimals = 16;
  }
  const size_t inputLength = input_c.length();

  if (nr_decimals > inputLength) {
    nr_decimals = inputLength;
  } else if (input_c.startsWith(F("0x"))) { // strtoull handles that prefix nicely
    nr_decimals += 2;
  }
  return strtoull(input_c.substring(0, nr_decimals).c_str(), 0, 16);
}

unsigned long long hexToULL(const String& input_c) {
  return hexToULL(input_c, input_c.length());
}

unsigned long long hexToULL(const String& input_c, size_t startpos, size_t nrHexDecimals) {
  return hexToULL(input_c.substring(startpos, startpos + nrHexDecimals), nrHexDecimals);
}

String formatToHex(unsigned long value, 
                   const __FlashStringHelper * prefix,
                   unsigned int minimal_hex_digits) {
  String result = prefix;
  String hex(value, HEX);

  hex.toUpperCase();
  if (hex.length() < minimal_hex_digits) {
    const size_t leading_zeros = minimal_hex_digits - hex.length();
    for (size_t i = 0; i < leading_zeros; ++i) {
      result += '0';
    }
  }
  result += hex;
  return result;
}

String formatToHex(unsigned long value,
                   const __FlashStringHelper * prefix) {
  return formatToHex(value, prefix, 0);
}

String formatToHex(unsigned long value, unsigned int minimal_hex_digits) {
  return formatToHex(value, F("0x"), minimal_hex_digits);
}

String formatToHex_no_prefix(unsigned long value, unsigned int minimal_hex_digits) {
  return formatToHex(value, F(""), minimal_hex_digits);
}

String formatHumanReadable(unsigned long value, unsigned long factor) {
  String result = formatHumanReadable(value, factor, 2);

  result.replace(F(".00"), EMPTY_STRING);
  return result;
}

String formatHumanReadable(unsigned long value, unsigned long factor, int NrDecimals) {
  float floatValue(value);
  uint8_t  steps = 0;

  while (value >= factor) {
    value /= factor;
    ++steps;
    floatValue /= float(factor);
  }
  String result = toString(floatValue, NrDecimals);

  switch (steps) {
    case 0: break;
    case 1: result += 'k'; break;
    case 2: result += 'M'; break;
    case 3: result += 'G'; break;
    case 4: result += 'T'; break;
    default:
      result += '*';
      result += factor;
      result += '^';
      result += steps;
      break;
  }
  return result;
}

String formatToHex_decimal(unsigned long value) {
  return formatToHex_decimal(value, 1);
}

String formatToHex_decimal(unsigned long value, unsigned long factor) {
  String result = formatToHex(value);

  result += F(" (");

  if (factor > 1) {
    result += formatHumanReadable(value, factor);
  } else {
    result += value;
  }
  result += ')';
  return result;
}

const __FlashStringHelper * boolToString(bool value) {
  return value ? F("true") : F("false");
}

/*********************************************************************************************\
   Typical string replace functions.
\*********************************************************************************************/
void removeExtraNewLine(String& line) {
  while (line.endsWith(F("\r\n\r\n"))) {
    line.remove(line.length() - 2);
  }
}

void addNewLine(String& line) {
  line += F("\r\n");
}

size_t UTF8_charLength(uint8_t firstByte) {
  if (firstByte <= 0x7f) {
    return 1;
  }
  // First Byte  Second Byte Third Byte  Fourth Byte
  // [0x00,0x7F]         
  // [0xC2,0xDF] [0x80,0xBF]     
  // 0xE0        [0xA0,0xBF] [0x80,0xBF] 
  // [0xE1,0xEC] [0x80,0xBF] [0x80,0xBF] 
  // 0xED        [0x80,0x9F] [0x80,0xBF] 
  // [0xEE,0xEF] [0x80,0xBF] [0x80,0xBF] 
  // 0xF0        [0x90,0xBF] [0x80,0xBF] [0x80,0xBF]
  // [0xF1,0xF3] [0x80,0xBF] [0x80,0xBF] [0x80,0xBF]
  // 0xF4        [0x80,0x8F] [0x80,0xBF] [0x80,0xBF]
  // See: https://lemire.me/blog/2018/05/09/how-quickly-can-you-check-that-a-string-is-valid-unicode-utf-8/

  size_t charLength = 2;
  if (firstByte > 0xEF) {
    charLength = 4;
  } else if (firstByte > 0xDF) {
    charLength = 3;
  }
  return charLength;
}

void replaceUnicodeByChar(String& line, char replChar) {
  size_t pos = 0;
  while (pos < line.length()) {
    const size_t charLength = UTF8_charLength((uint8_t)line[pos]);

    if (charLength > 1) {
      // Is unicode char in UTF-8 format
      // Need to find how many characters we need to replace.
      const size_t charsLeft = line.length() - pos;
      if (charsLeft >= charLength) {
        line.replace(line.substring(pos, pos + charLength), String(replChar));
      }
    }
    ++pos;
  }
}

/*********************************************************************************************\
   Format a value to the set number of decimals
\*********************************************************************************************/
String doFormatUserVar(struct EventStruct *event, uint8_t rel_index, bool mustCheck, bool& isvalid) {
  if (event == nullptr) return EMPTY_STRING;
  isvalid = true;

  const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(event->TaskIndex);

  if (!validDeviceIndex(DeviceIndex)) {
    isvalid = false;
    return EMPTY_STRING;
  }

  {
    // First try to format using the plugin specific formatting.
    String result;
    EventStruct tempEvent;
    tempEvent.deep_copy(event);
    tempEvent.idx = rel_index;
    PluginCall(PLUGIN_FORMAT_USERVAR, &tempEvent, result);
    if (result.length() > 0) {
      return result;
    }
  }


  const uint8_t   valueCount = getValueCountForTask(event->TaskIndex);
  Sensor_VType sensorType = event->getSensorType();

  if (valueCount <= rel_index) {
    isvalid = false;

    #ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log = F("No sensor value for TaskIndex: ");
      log += event->TaskIndex + 1;
      log += F(" varnumber: ");
      log += rel_index + 1;
      log += F(" type: ");
      log += getSensorTypeLabel(sensorType);
      addLogMove(LOG_LEVEL_ERROR, log);
    }
    #endif // ifndef BUILD_NO_DEBUG
    return EMPTY_STRING;
  }

  switch (sensorType) {
    case Sensor_VType::SENSOR_TYPE_LONG:
      return String(UserVar.getSensorTypeLong(event->TaskIndex));
    case Sensor_VType::SENSOR_TYPE_STRING:
      return event->String2;

    default:
      break;
  }

  float f(UserVar[event->BaseVarIndex + rel_index]);

  if (mustCheck && !isValidFloat(f)) {
    isvalid = false;
#ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log = F("Invalid float value for TaskIndex: ");
      log += event->TaskIndex;
      log += F(" varnumber: ");
      log += rel_index;
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
#endif // ifndef BUILD_NO_DEBUG
    f = 0;
  }

  uint8_t nrDecimals = 0;
  if (Device[DeviceIndex].configurableDecimals()) {
    nrDecimals = Cache.getTaskDeviceValueDecimals(event->TaskIndex, rel_index);
  }

  String result = toString(f, nrDecimals);
  result.trim();
  return result;
}

String formatUserVarNoCheck(taskIndex_t TaskIndex, uint8_t rel_index) {
  bool isvalid;

  // FIXME TD-er: calls to this function cannot handle Sensor_VType::SENSOR_TYPE_STRING
  struct EventStruct TempEvent(TaskIndex);

  return doFormatUserVar(&TempEvent, rel_index, false, isvalid);
}

String formatUserVar(taskIndex_t TaskIndex, uint8_t rel_index, bool& isvalid) {
  // FIXME TD-er: calls to this function cannot handle Sensor_VType::SENSOR_TYPE_STRING
  struct EventStruct TempEvent(TaskIndex);

  return doFormatUserVar(&TempEvent, rel_index, true, isvalid);
}

String formatUserVarNoCheck(struct EventStruct *event, uint8_t rel_index)
{
  bool isvalid;

  return doFormatUserVar(event, rel_index, false, isvalid);
}

String formatUserVar(struct EventStruct *event, uint8_t rel_index, bool& isvalid)
{
  return doFormatUserVar(event, rel_index, true, isvalid);
}

String get_formatted_Controller_number(cpluginID_t cpluginID) {
  if (!validCPluginID(cpluginID)) {
    return F("C---");
  }
  String result;
  result += 'C';

  if (cpluginID < 100) { result += '0'; }

  if (cpluginID < 10) { result += '0'; }
  result += cpluginID;
  return result;
}

/*********************************************************************************************\
   Wrap a string with given pre- and postfix string.
\*********************************************************************************************/
String wrap_braces(const String& string) {
  return wrap_String(string, '(', ')');
}

String wrap_String(const String& string, char wrap) {
  String result;
  result.reserve(string.length() + 2);
  result += wrap;
  result += string;
  result += wrap;
  return result;
}

String wrap_String(const String& string, char char1, char char2) {
  String result;
  result.reserve(string.length() + 2);
  result += char1;
  result += string;
  result += char2;
  return result;
}

String wrapIfContains(const String& value, char contains, char wrap) {
  if (value.indexOf(contains) != -1) {
    return wrap_String(value, wrap, wrap);
  }
  return value;
}

String wrapWithQuotes(const String& text) {
  if (isWrappedWithQuotes(text)) {
    return text;
  }
  // Try to find unused quote char and wrap
  char quotechar = '_';
  if (!findUnusedQuoteChar(text, quotechar)) {
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log = F("No unused quote to wrap: _");
      log += text;
      log += '_';
      addLogMove(LOG_LEVEL_ERROR, log);
    }
  }
  return wrap_String(text, quotechar);
}

String wrapWithQuotesIfContainsParameterSeparatorChar(const String& text) {
  if (isWrappedWithQuotes(text)) {
    return text;
  }
  if (stringContainsSeparatorChar(text)) {
    return wrapWithQuotes(text);
  }
  return text;
}

/*********************************************************************************************\
   Format an object value pair for use in JSON.
\*********************************************************************************************/
String to_json_object_value(const __FlashStringHelper * object,
                            const __FlashStringHelper * value,
                            bool wrapInQuotes) 
{
  return to_json_object_value(String(object), String(value), wrapInQuotes);
}


String to_json_object_value(const __FlashStringHelper * object,
                            const String& value,
                            bool wrapInQuotes) 
{
  return to_json_object_value(String(object), value, wrapInQuotes);
}

String to_json_object_value(const __FlashStringHelper * object,
                            String&& value,
                            bool wrapInQuotes) 
{
  return to_json_object_value(String(object), value, wrapInQuotes);
}

String to_json_object_value(const String& object, const String& value, bool wrapInQuotes) {
  String result;
  result.reserve(object.length() + value.length() + 6);
  result = wrap_String(object, '"');
  result += ':';
  result += to_json_value(value, wrapInQuotes);
  return result;
}

String to_json_value(const String& value, bool wrapInQuotes) {
  if (value.isEmpty()) {
    // Empty string
    return F("\"\"");
  }
  if (wrapInQuotes || mustConsiderAsJSONString(value)) {
    // Is not a numerical value, or BIN/HEX notation, thus wrap with quotes
    if ((value.indexOf('\n') != -1) || (value.indexOf('\r') != -1) || (value.indexOf('"') != -1)) {
      // Must replace characters, so make a deepcopy
      String tmpValue(value);
      tmpValue.replace('\n', '^');
      tmpValue.replace('\r', '^');
      tmpValue.replace('"',  '\'');
      return wrap_String(tmpValue, '"');
    } else {
      return wrap_String(value, '"');
    }
  } 
  // It is a numerical
  return value;
}


/*********************************************************************************************\
   Strip wrapping chars (e.g. quotes)
\*********************************************************************************************/
String stripWrappingChar(const String& text, char wrappingChar) {
  const unsigned int length = text.length();

  if ((length >= 2) && stringWrappedWithChar(text, wrappingChar)) {
    return text.substring(1, length - 1);
  }
  return text;
}

bool stringWrappedWithChar(const String& text, char wrappingChar) {
  const unsigned int length = text.length();

  if (length < 2) { return false; }
  return (text.charAt(0) == wrappingChar) && 
         (text.charAt(length - 1) == wrappingChar);
}

bool isQuoteChar(char c) {
  return c == '\'' || c == '"' || c == '`';
}

bool findUnusedQuoteChar(const String& text, char& quotechar) {
  quotechar = '_';
  if (text.indexOf('\'') == -1) quotechar = '\'';
  else if (text.indexOf('"') == -1) quotechar = '"';
  else if (text.indexOf('`') == -1) quotechar = '`';
  
  return isQuoteChar(quotechar);
}

bool isParameterSeparatorChar(char c) {
  return c == ',' || c == ' ';
}

bool stringContainsSeparatorChar(const String& text) {
  return text.indexOf(',') != -1 || text.indexOf(' ') != -1;
}

bool isWrappedWithQuotes(const String& text) {
  if (text.length() < 2) {
    return false;
  }
  const char quoteChar = text[0];
  return isQuoteChar(quoteChar) && stringWrappedWithChar(text, quoteChar);
}

String stripQuotes(const String& text) {
  if (text.length() >= 2) {
    char c = text.charAt(0);

    if (isQuoteChar(c)) {
      return stripWrappingChar(text, c);
    }
  }
  return text;
}

bool safe_strncpy(char         *dest,
                  const __FlashStringHelper * source,
                  size_t        max_size) 
{
  return safe_strncpy(dest, String(source), max_size);
}

bool safe_strncpy(char *dest, const String& source, size_t max_size) {
  return safe_strncpy(dest, source.c_str(), max_size);
}

bool safe_strncpy(char *dest, const char *source, size_t max_size) {
  if (max_size < 1) { return false; }

  if (dest == nullptr) { return false; }

  if (source == nullptr) { return false; }
  bool result = true;

  memset(dest, 0, max_size);
  size_t str_length = strlen_P(source);

  if (str_length >= max_size) {
    str_length = max_size;
    result     = false;
  }
  strncpy_P(dest, source, str_length);
  dest[max_size - 1] = 0;
  return result;
}

// Convert a string to lower case and replace spaces with underscores.
String to_internal_string(const String& input, char replaceSpace) {
  String result = input;

  result.trim();
  result.toLowerCase();
  result.replace(' ', replaceSpace);
  return result;
}

/*********************************************************************************************\
   Parse a string and get the xth command or parameter
   IndexFind = 1 => command.
    // FIXME TD-er: parseString* should use index starting at 0.
\*********************************************************************************************/
String parseString(const String& string, uint8_t indexFind, char separator, bool trimResult) {
  String result = parseStringKeepCase(string, indexFind, separator, trimResult);

  result.toLowerCase();
  return result;
}

String parseStringKeepCase(const String& string, uint8_t indexFind, char separator, bool trimResult) {
  String result;

  if (!GetArgv(string.c_str(), result, indexFind, separator)) {
    return EMPTY_STRING;
  }
  if (trimResult) {
    result.trim();
  }
  return stripQuotes(result);
}

String parseStringToEnd(const String& string, uint8_t indexFind, char separator, bool trimResult) {
  String result = parseStringToEndKeepCase(string, indexFind, separator, trimResult);

  result.toLowerCase();
  return result;
}

String parseStringToEndKeepCase(const String& string, uint8_t indexFind, char separator, bool trimResult) {
  // Loop over the arguments to find the first and last pos of the arguments.
  int  pos_begin = string.length();
  int  pos_end = pos_begin;
  int  tmppos_begin, tmppos_end = -1;
  uint8_t nextArgument = indexFind;
  bool hasArgument  = false;

  while (GetArgvBeginEnd(string.c_str(), nextArgument, tmppos_begin, tmppos_end, separator))
  {
    hasArgument = true;

    if ((tmppos_begin < pos_begin) && (tmppos_begin >= 0)) {
      pos_begin = tmppos_begin;
    }

    if ((tmppos_end >= 0)) {
      pos_end = tmppos_end;
    }
    ++nextArgument;
  }

  if (!hasArgument || (pos_begin < 0) || (pos_begin == pos_end)) {
    return EMPTY_STRING;
  }
  String result = string.substring(pos_begin, pos_end);

  if (trimResult) {
    result.trim();
  }
  return stripQuotes(result);
}

String tolerantParseStringKeepCase(const char * string,
                                   uint8_t      indexFind,
                                   char         separator,
                                   bool         trimResult)
{
  return tolerantParseStringKeepCase(String(string), indexFind, separator, trimResult);
}


String tolerantParseStringKeepCase(const String& string, uint8_t indexFind, char separator, bool trimResult)
{
  if (Settings.TolerantLastArgParse()) {
    return parseStringToEndKeepCase(string, indexFind, separator, trimResult);
  }
  return parseStringKeepCase(string, indexFind, separator, trimResult);
}

// escapes special characters in strings for use in html-forms
bool htmlEscapeChar(char c, String& esc)
{
  const __FlashStringHelper * escaped = F("");
  switch (c)
  {
    case '&':  escaped = F("&amp;");  break;
    case '\"': escaped = F("&quot;"); break;
    case '\'': escaped = F("&#039;"); break;
    case '<':  escaped = F("&lt;");   break;
    case '>':  escaped = F("&gt;");   break;
    case '/':  escaped = F("&#047;"); break;
    default:
      return false;
  }

  esc = String(escaped);  
  return true;
}

void htmlEscape(String& html, char c)
{
  String repl;

  if (htmlEscapeChar(c, repl)) {
    html.replace(String(c), repl);
  }
}

void htmlEscape(String& html)
{
  htmlEscape(html, '&');
  htmlEscape(html, '\"');
  htmlEscape(html, '\'');
  htmlEscape(html, '<');
  htmlEscape(html, '>');
  htmlEscape(html, '/');
}

void htmlStrongEscape(String& html)
{
  String escaped;

  escaped.reserve(html.length());

  for (unsigned i = 0; i < html.length(); ++i)
  {
    if (isAlphaNumeric(html[i]))
    {
      escaped += html[i];
    }
    else
    {
      char s[4] = {0};
      sprintf_P(s, PSTR("%03d"), static_cast<int>(html[i]));
      escaped += '&';
      escaped += '#';
      escaped += s;
      escaped += ';';
    }
  }
  html = escaped;
}

// ********************************************************************************
// URNEncode char string to string object
// ********************************************************************************
String URLEncode(const String& msg)
{
  const char *hex = "0123456789abcdef";
  String encodedMsg;

  const size_t msg_length = msg.length();

  encodedMsg.reserve(msg_length);

  for (size_t i = 0; i < msg_length; ++i) {
    const char ch = msg[i];
    if (isAlphaNumeric(ch)
        || ('-' == ch) || ('_' == ch)
        || ('.' == ch) || ('~' == ch)) {
      encodedMsg += ch;
    } else {
      encodedMsg += '%';
      encodedMsg += hex[ch >> 4];
      encodedMsg += hex[ch & 15];
    }
  }
  return encodedMsg;
}

void repl(const __FlashStringHelper * key,
            const String& val,
            String      & s,
            bool       useURLencode)
{
  repl(String(key), val, s, useURLencode);
}

void repl(const __FlashStringHelper * key,
          const char* val,
          String      & s,
          bool       useURLencode)
{
  repl(String(key), String(val), s, useURLencode);
}

void repl(const __FlashStringHelper * key1,
           const __FlashStringHelper * key2,
           const char* val,
           String      & s,
           bool       useURLencode)
{
  repl(key1, val, s, useURLencode);
  repl(key2, val, s, useURLencode);
}


void repl(const String& key, const String& val, String& s, bool useURLencode)
{
  if (useURLencode) {
    // URLEncode does take resources, so check first if needed.
    if (s.indexOf(key) == -1) { return; }
    s.replace(key, URLEncode(val));
  } else {
    s.replace(key, val);
  }
}

void parseSpecialCharacters(String& s, bool useURLencode)
{
  const bool no_accolades   = s.indexOf('{') == -1 || s.indexOf('}') == -1;
  const bool no_html_entity = s.indexOf('&') == -1 || s.indexOf(';') == -1;

  if (no_accolades && no_html_entity) {
    return; // Nothing to replace
  }
  {
    // Degree
    const char degree[3]   = { 0xc2, 0xb0, 0 };       // Unicode degree symbol
    const char degreeC[4]  = { 0xe2, 0x84, 0x83, 0 }; // Unicode degreeC symbol
    const char degree_C[4] = { 0xc2, 0xb0, 'C', 0 };  // Unicode degree symbol + captial C
    repl(F("{D}"),   degree,   s, useURLencode);
    repl(F("&deg;"), degree,   s, useURLencode);
    repl(degreeC,    degree_C, s, useURLencode);
  }
  // Degree symbol is often used on displays, so still support that one.
#ifndef BUILD_NO_SPECIAL_CHARACTERS_STRINGCONVERTER
  {
    // Angle quotes
    const char laquo[3] = { 0xc2, 0xab, 0 }; // Unicode left angle quotes symbol
    const char raquo[3] = { 0xc2, 0xbb, 0 }; // Unicode right angle quotes symbol
    repl(F("{<<}"), F("&laquo;"), laquo, s, useURLencode);
    repl(F("{>>}"), F("&raquo;"), raquo, s, useURLencode);
  }
  {
    // Greek letter Mu
    const char mu[3] = { 0xc2, 0xb5, 0 }; // Unicode greek letter mu
    repl(F("{u}"), F("&micro;"), mu, s, useURLencode);
  }
  {
    // Currency
    const char euro[4]  = { 0xe2, 0x82, 0xac, 0 }; // Unicode euro symbol
    const char yen[3]   = { 0xc2, 0xa5, 0 };       // Unicode yen symbol
    const char pound[3] = { 0xc2, 0xa3, 0 };       // Unicode pound symbol
    const char cent[3]  = { 0xc2, 0xa2, 0 };       // Unicode cent symbol
    repl(F("{E}"), F("&euro;"),  euro,  s, useURLencode);
    repl(F("{Y}"), F("&yen;"),   yen,   s, useURLencode);
    repl(F("{P}"), F("&pound;"), pound, s, useURLencode);
    repl(F("{c}"), F("&cent;"),  cent,  s, useURLencode);
  }
  {
    // Math symbols
    const char sup1[3]   = { 0xc2, 0xb9, 0 }; // Unicode sup1 symbol
    const char sup2[3]   = { 0xc2, 0xb2, 0 }; // Unicode sup2 symbol
    const char sup3[3]   = { 0xc2, 0xb3, 0 }; // Unicode sup3 symbol
    const char frac14[3] = { 0xc2, 0xbc, 0 }; // Unicode frac14 symbol
    const char frac12[3] = { 0xc2, 0xbd, 0 }; // Unicode frac12 symbol
    const char frac34[3] = { 0xc2, 0xbe, 0 }; // Unicode frac34 symbol
    const char plusmn[3] = { 0xc2, 0xb1, 0 }; // Unicode plusmn symbol
    const char times[3]  = { 0xc3, 0x97, 0 }; // Unicode times symbol
    const char divide[3] = { 0xc3, 0xb7, 0 }; // Unicode divide symbol
    repl(F("{^1}"),  F("&sup1;"),   sup1,   s, useURLencode);
    repl(F("{^2}"),  F("&sup2;"),   sup2,   s, useURLencode);
    repl(F("{^3}"),  F("&sup3;"),   sup3,   s, useURLencode);
    repl(F("{1_4}"), F("&frac14;"), frac14, s, useURLencode);
    repl(F("{1_2}"), F("&frac12;"), frac12, s, useURLencode);
    repl(F("{3_4}"), F("&frac34;"), frac34, s, useURLencode);
    repl(F("{+-}"),  F("&plusmn;"), plusmn, s, useURLencode);
    repl(F("{x}"),   F("&times;"),  times,  s, useURLencode);
    repl(F("{..}"),  F("&divide;"), divide, s, useURLencode);
  }
#endif // ifndef BUILD_NO_SPECIAL_CHARACTERS_STRINGCONVERTER
}


/********************************************************************************************\
   replace other system variables like %sysname%, %systime%, %ip%
 \*********************************************************************************************/
void parseControllerVariables(String& s, struct EventStruct *event, bool useURLencode) {
  s = parseTemplate(s, useURLencode);
  parseEventVariables(s, event, useURLencode);
}

void parseSingleControllerVariable(String            & s,
                                   struct EventStruct *event,
                                   uint8_t                taskValueIndex,
                                   bool             useURLencode) {
  if (validTaskIndex(event->TaskIndex)) {
    repl(F("%valname%"), getTaskValueName(event->TaskIndex, taskValueIndex), s, useURLencode);
  } else {
    repl(F("%valname%"), EMPTY_STRING, s, useURLencode);
  }
}

// FIXME TD-er: These macros really increase build size.
// Simple macro to create the replacement string only when needed.
#define SMART_REPL(T, S) \
  if (s.indexOf(T) != -1) { repl((T), (S), s, useURLencode); }
void parseSystemVariables(String& s, bool useURLencode)
{
  parseSpecialCharacters(s, useURLencode);

  SystemVariables::parseSystemVariables(s, useURLencode);
}

void parseEventVariables(String& s, struct EventStruct *event, bool useURLencode)
{
  repl(F("%id%"), String(event->idx), s, useURLencode);

  if (validTaskIndex(event->TaskIndex)) {
    if (s.indexOf(F("%val")) != -1) {
      if (event->getSensorType() == Sensor_VType::SENSOR_TYPE_LONG) {
        SMART_REPL(F("%val1%"), String(UserVar.getSensorTypeLong(event->TaskIndex)))
      } else {
        for (uint8_t i = 0; i < getValueCountForTask(event->TaskIndex); ++i) {
          String valstr = F("%val");
          valstr += (i + 1);
          valstr += '%';
          SMART_REPL(valstr, formatUserVarNoCheck(event, i));
        }
      }
    }
  }

  if (validTaskIndex(event->TaskIndex)) {
    repl(F("%tskname%"), getTaskDeviceName(event->TaskIndex), s, useURLencode);
  } else {
    repl(F("%tskname%"), EMPTY_STRING, s, useURLencode);
  }

  const bool vname_found = s.indexOf(F("%vname")) != -1;

  if (vname_found) {
    for (uint8_t i = 0; i < 4; ++i) {
      String vname = F("%vname");
      vname += (i + 1);
      vname += '%';

      if (validTaskIndex(event->TaskIndex)) {
        repl(vname, getTaskValueName(event->TaskIndex, i), s, useURLencode);
      } else {
        repl(vname, EMPTY_STRING, s, useURLencode);
      }
    }
  }
}

#undef SMART_REPL

bool getConvertArgument(const __FlashStringHelper * marker, const String& s, float& argument, int& startIndex, int& endIndex) {
  String argumentString;

  if (getConvertArgumentString(marker, s, argumentString, startIndex, endIndex)) {
    return validFloatFromString(argumentString, argument);
  }
  return false;
}

bool getConvertArgument2(const __FlashStringHelper * marker, const String& s, float& arg1, float& arg2, int& startIndex, int& endIndex) {
  String argumentString;

  if (getConvertArgumentString(marker, s, argumentString, startIndex, endIndex)) {
    const int pos_comma = argumentString.indexOf(',');

    if (pos_comma == -1) { return false; }

    if (validFloatFromString(argumentString.substring(0, pos_comma), arg1)) {
      return validFloatFromString(argumentString.substring(pos_comma + 1), arg2);
    }
  }
  return false;
}

bool getConvertArgumentString(const __FlashStringHelper * marker, const String& s, String& argumentString, int& startIndex, int& endIndex) {
  return getConvertArgumentString(String(marker), s, argumentString, startIndex, endIndex);
}

bool getConvertArgumentString(const String& marker,
                              const String& s,
                              String      & argumentString,
                              int         & startIndex,
                              int         & endIndex) {


  startIndex = s.indexOf(marker);

  if (startIndex == -1) { return false; }

  int startIndexArgument = startIndex + marker.length();

  if (s.charAt(startIndexArgument) != '(') {
    return false;
  }
  ++startIndexArgument;
  endIndex = s.indexOf(')', startIndexArgument);

  if (endIndex == -1) { return false; }

  argumentString = s.substring(startIndexArgument, endIndex);

  if (argumentString.isEmpty()) { return false; }
  ++endIndex; // Must also strip ')' from the original string.
  return true;
}


// FIXME TD-er: These macros really increase build size
struct ConvertArgumentData {
  ConvertArgumentData(String& s, bool useURLencode) 
    : str(s), arg1(0.0f), arg2(0.0f), startIndex(0), endIndex(0),
      URLencode(useURLencode) {}

  ConvertArgumentData() = delete;

  String& str;
  float arg1, arg2;
  int   startIndex;
  int   endIndex;
  bool  URLencode;
};

void repl(ConvertArgumentData& data, const String& repl_str) {
  repl(data.str.substring(data.startIndex, data.endIndex), repl_str, data.str, data.URLencode);
}

bool getConvertArgument(const __FlashStringHelper * marker, ConvertArgumentData& data) {
  return getConvertArgument(marker, data.str, data.arg1, data.startIndex, data.endIndex);
}

bool getConvertArgument2(const __FlashStringHelper * marker, ConvertArgumentData& data) {
  return getConvertArgument2(marker, data.str, data.arg1, data.arg2, data.startIndex, data.endIndex);
}

// Parse conversions marked with "%conv_marker%(float)"
// Must be called last, since all sensor values must be converted, processed, etc.
void parseStandardConversions(String& s, bool useURLencode) {
  if (s.indexOf(F("%c_")) == -1) {
    return; // Nothing to replace
  }

  ConvertArgumentData data(s, useURLencode);

  // These replacements should be done in a while loop per marker,
  // since they also replace the numerical parameter.
  // The marker may occur more than once per string, but with different parameters.
  #define SMART_CONV(T, FUN) \
  while (getConvertArgument((T), data)) { repl(data, (FUN)); }
  SMART_CONV(F("%c_w_dir%"),  getBearing(data.arg1))
  SMART_CONV(F("%c_c2f%"),    toString(CelsiusToFahrenheit(data.arg1), 2))
  SMART_CONV(F("%c_ms2Bft%"), String(m_secToBeaufort(data.arg1)))
  SMART_CONV(F("%c_cm2imp%"), centimeterToImperialLength(data.arg1))
  SMART_CONV(F("%c_mm2imp%"), millimeterToImperialLength(data.arg1))
  SMART_CONV(F("%c_m2day%"),  toString(minutesToDay(data.arg1), 2))
  SMART_CONV(F("%c_m2dh%"),   minutesToDayHour(data.arg1))
  SMART_CONV(F("%c_m2dhm%"),  minutesToDayHourMinute(data.arg1))
  SMART_CONV(F("%c_s2dhms%"), secondsToDayHourMinuteSecond(data.arg1))
  SMART_CONV(F("%c_2hex%"),   formatToHex_no_prefix(data.arg1))
  #undef SMART_CONV

  // Conversions with 2 parameters
  #define SMART_CONV(T, FUN) \
  while (getConvertArgument2((T), data)) { repl(data, (FUN)); }
  SMART_CONV(F("%c_dew_th%"), toString(compute_dew_point_temp(data.arg1, data.arg2), 2))
  #if FEATURE_ESPEASY_P2P
  SMART_CONV(F("%c_u2ip%"),   formatUnitToIPAddress(data.arg1, data.arg2))
  #endif
  SMART_CONV(F("%c_alt_pres_sea%"), toString(altitudeFromPressure(data.arg1, data.arg2), 2))
  SMART_CONV(F("%c_sea_pres_alt%"), toString(pressureElevation(data.arg1, data.arg2), 2))
  #undef SMART_CONV
}

/********************************************************************************************\
   Find positional parameter in a char string
 \*********************************************************************************************/
bool HasArgv(const char *string, unsigned int argc) {
  String argvString;

  return GetArgv(string, argvString, argc);
}

bool GetArgv(const char *string, String& argvString, unsigned int argc, char separator) {
  int  pos_begin, pos_end;
  bool hasArgument = GetArgvBeginEnd(string, argc, pos_begin, pos_end, separator);

  argvString = String();

  if (!hasArgument) { return false; }

  if ((pos_begin >= 0) && (pos_end >= 0) && (pos_end > pos_begin)) {
    argvString.reserve(pos_end - pos_begin);

    for (int i = pos_begin; i < pos_end; ++i) {
      argvString += string[i];
    }
    argvString.trim();
    argvString = stripQuotes(argvString);
  }
  return true;
}

bool GetArgvBeginEnd(const char *string, const unsigned int argc, int& pos_begin, int& pos_end, char separator) {
  pos_begin = -1;
  pos_end   = -1;
  size_t string_len = strlen(string);
  unsigned int string_pos = 0, argc_pos = 0;
  bool parenthesis          = false;
  char matching_parenthesis = '"';

  while (string_pos < string_len)
  {
    char c, d, e; // c = current char, d,e = next char (if available)
    c = string[string_pos];
    d = 0;
    e = 0;

    if ((string_pos + 1) < string_len) {
      d = string[string_pos + 1];
    }
    if ((string_pos + 2) < string_len) {
      e = string[string_pos + 2];
    }

    if  (!parenthesis && (((c == ' ') && (d == ' ')) || 
                          ((c == separator) && (d == ' ')))) {
      // Consider multiple consequitive spaces as one.
    }
    else if  (!parenthesis && ((d == ' ') && (e == separator))) {
      // Skip the space.      
    }
    else
    {
      // Found the start of the new argument.
      if (pos_begin == -1 && !parenthesis && !((c == separator) || isParameterSeparatorChar(c))) {
        pos_begin = string_pos;
        pos_end   = string_pos;
      }
      if (pos_end != -1) {
        ++pos_end;
      }

      // Check if we're in a set of parenthesis (any quote char or [])
      if (!parenthesis && (isQuoteChar(c) || (c == '['))) {
        parenthesis          = true;
        matching_parenthesis = c;

        if (c == '[') {
          matching_parenthesis = ']';
        }
      } else if (parenthesis && (c == matching_parenthesis)) {
        parenthesis = false;
      }

      if (!parenthesis && (isParameterSeparatorChar(d) || (d == separator) || (d == 0))) // end of word
      {
        argc_pos++;
        if (argc_pos == argc)
        {
          return true;
        }
        // new Argument separator found
        pos_begin = -1;
        pos_end   = -1;
      }
    }
    string_pos++;
  }
  return false;
}

#include "../Helpers/Scheduler.h"


#include "../../ESPEasy-Globals.h"

#include "../../_Plugin_Helper.h"

#include "../ControllerQueue/DelayQueueElements.h"
#include "../ESPEasyCore/ESPEasyGPIO.h"
#include "../ESPEasyCore/ESPEasyRules.h"
#include "../Globals/GlobalMapPortStatus.h"
#include "../Globals/RTC.h"
#include "../Globals/NPlugins.h"
#include "../Helpers/DeepSleep.h"
#include "../Helpers/ESPEasyRTC.h"
#include "../Helpers/Networking.h"
#include "../Helpers/PeriodicalActions.h"
#include "../Helpers/PortStatus.h"


#define TIMER_ID_SHIFT       28 // Must be decreased as soon as timers below reach 15


#ifndef BUILD_NO_DEBUG
const __FlashStringHelper * toString_f(ESPEasy_Scheduler::IntervalTimer_e timer) {
  switch (timer) {
    case ESPEasy_Scheduler::IntervalTimer_e::TIMER_20MSEC:           return F("TIMER_20MSEC");
    case ESPEasy_Scheduler::IntervalTimer_e::TIMER_100MSEC:          return F("TIMER_100MSEC");
    case ESPEasy_Scheduler::IntervalTimer_e::TIMER_1SEC:             return F("TIMER_1SEC");
    case ESPEasy_Scheduler::IntervalTimer_e::TIMER_30SEC:            return F("TIMER_30SEC");
    case ESPEasy_Scheduler::IntervalTimer_e::TIMER_MQTT:             return F("TIMER_MQTT");
    case ESPEasy_Scheduler::IntervalTimer_e::TIMER_STATISTICS:       return F("TIMER_STATISTICS");
    case ESPEasy_Scheduler::IntervalTimer_e::TIMER_GRATUITOUS_ARP:   return F("TIMER_GRATUITOUS_ARP");
    case ESPEasy_Scheduler::IntervalTimer_e::TIMER_MQTT_DELAY_QUEUE: return F("TIMER_MQTT_DELAY_QUEUE");
    default: 
      break;
  }
  return F("unknown");
}
#endif

String ESPEasy_Scheduler::toString(ESPEasy_Scheduler::IntervalTimer_e timer) {
#ifdef BUILD_NO_DEBUG
  return String(static_cast<int>(timer));
#else // ifdef BUILD_NO_DEBUG

  if (timer >= IntervalTimer_e::TIMER_C001_DELAY_QUEUE && 
      timer <= IntervalTimer_e::TIMER_C025_DELAY_QUEUE) 
  {
    String res;
    res.reserve(24);
    res = F("TIMER_C0");
    const int id = static_cast<int>(timer) - static_cast<int>(IntervalTimer_e::TIMER_C001_DELAY_QUEUE) + 1;

    if (id < 10) { res += '0'; }
    res += id;
    res += F("_DELAY_QUEUE");
    return res;
  }
  return toString_f(timer);
#endif // ifdef BUILD_NO_DEBUG
}

const __FlashStringHelper * ESPEasy_Scheduler::toString(ESPEasy_Scheduler::SchedulerTimerType_e timerType) {
  switch (timerType) {
    case SchedulerTimerType_e::SystemEventQueue:       return F("SystemEventQueue");
    case SchedulerTimerType_e::ConstIntervalTimer:     return F("Const Interval");
    case SchedulerTimerType_e::PLUGIN_TIMER_IN_e:      return F("PLUGIN_TIMER_IN");
    case SchedulerTimerType_e::TaskDeviceTimer:        return F("PLUGIN_READ");
    case SchedulerTimerType_e::GPIO_timer:             return F("GPIO_timer");
    case SchedulerTimerType_e::PLUGIN_ONLY_TIMER_IN_e: return F("PLUGIN_ONLY_TIMER_IN");
    case SchedulerTimerType_e::RulesTimer:             return F("Rules#Timer");
    case SchedulerTimerType_e::IntendedReboot:         return F("Intended Reboot");
  }
  return F("unknown");
}

const __FlashStringHelper * ESPEasy_Scheduler::toString(ESPEasy_Scheduler::PluginPtrType pluginType) {
  switch (pluginType) {
    case PluginPtrType::TaskPlugin:         return F("Plugin");
    case PluginPtrType::ControllerPlugin:   return F("Controller");
#if FEATURE_NOTIFIER
    case PluginPtrType::NotificationPlugin: return F("Notification");
#endif
  }
  return F("unknown");
}

const __FlashStringHelper * toString_f(ESPEasy_Scheduler::IntendedRebootReason_e reason) {
  switch (reason) {
    case ESPEasy_Scheduler::IntendedRebootReason_e::DeepSleep:              return F("DeepSleep");
    case ESPEasy_Scheduler::IntendedRebootReason_e::DelayedReboot:          return F("DelayedReboot");
    case ESPEasy_Scheduler::IntendedRebootReason_e::ResetFactory:           return F("ResetFactory");
    case ESPEasy_Scheduler::IntendedRebootReason_e::ResetFactoryPinActive:  return F("ResetFactoryPinActive");
    case ESPEasy_Scheduler::IntendedRebootReason_e::ResetFactoryCommand:    return F("ResetFactoryCommand");
    case ESPEasy_Scheduler::IntendedRebootReason_e::CommandReboot:          return F("CommandReboot");
    case ESPEasy_Scheduler::IntendedRebootReason_e::RestoreSettings:        return F("RestoreSettings");
    case ESPEasy_Scheduler::IntendedRebootReason_e::OTA_error:              return F("OTA_error");
    case ESPEasy_Scheduler::IntendedRebootReason_e::ConnectionFailuresThreshold: return F("ConnectionFailuresThreshold");
  }
  return F("");
}

String ESPEasy_Scheduler::toString(ESPEasy_Scheduler::IntendedRebootReason_e reason) {
  String res = toString_f(reason);
  if (res.isEmpty()) return String(static_cast<int>(reason));
  return res;
}

void ESPEasy_Scheduler::markIntendedReboot(ESPEasy_Scheduler::IntendedRebootReason_e reason) {
  const unsigned long mixed_id = getMixedId(SchedulerTimerType_e::IntendedReboot, static_cast<unsigned long>(reason));

  RTC.lastMixedSchedulerId = mixed_id;
  saveToRTC();
}

/*********************************************************************************************\
* Generic Timer functions.
\*********************************************************************************************/
void ESPEasy_Scheduler::setNewTimerAt(unsigned long id, unsigned long timer) {
  START_TIMER;
  msecTimerHandler.registerAt(id, timer);
  STOP_TIMER(SET_NEW_TIMER);
}

// Mix timer type int with an ID describing the scheduled job.
unsigned long ESPEasy_Scheduler::getMixedId(SchedulerTimerType_e timerType, unsigned long id) {
  return (static_cast<unsigned long>(timerType) << TIMER_ID_SHIFT) + id;
}

unsigned long ESPEasy_Scheduler::decodeSchedulerId(unsigned long mixed_id, SchedulerTimerType_e& timerType) {
  timerType = static_cast<SchedulerTimerType_e>(mixed_id >> TIMER_ID_SHIFT);
  const unsigned long mask = (1 << TIMER_ID_SHIFT) - 1;

  return mixed_id & mask;
}

String ESPEasy_Scheduler::decodeSchedulerId(unsigned long mixed_id) {
  if (mixed_id == 0) {
    return F("Background Task");
  }
  SchedulerTimerType_e timerType = SchedulerTimerType_e::SystemEventQueue;
  const unsigned long  id        = decodeSchedulerId(mixed_id, timerType);
  String idStr                   = String(id);
  String result                  = toString(timerType);

  result += F(": ");

#ifndef BUILD_NO_DEBUG
  result.reserve(64);

  switch (timerType) {
    case SchedulerTimerType_e::SystemEventQueue:
    {
      const PluginPtrType ptr_type = static_cast<PluginPtrType>((id >> 16) & 0xFF);
      const uint8_t index          = (id >> 8) & 0xFF;
      const uint8_t function       = id & 0xFF;
      result += toString(ptr_type);
      result += ',';
      result += (index + 1); // TaskIndex / ControllerIndex / NotificationIndex
      result += ',';
      result += function;

      return result;
    }
    case SchedulerTimerType_e::ConstIntervalTimer:
      result +=  toString(static_cast<ESPEasy_Scheduler::IntervalTimer_e>(id));
      return result;
    case SchedulerTimerType_e::PLUGIN_TIMER_IN_e:
    {
      const deviceIndex_t deviceIndex = ((1 << 8) - 1) & id;

      if (validDeviceIndex(deviceIndex)) {
        idStr = getPluginNameFromDeviceIndex(deviceIndex);
      }
      result += idStr;
      return result;
    }
    case SchedulerTimerType_e::PLUGIN_ONLY_TIMER_IN_e:
    {
      const deviceIndex_t deviceIndex = ((1 << 8) - 1) & id;

      if (validDeviceIndex(deviceIndex)) {
        idStr = getPluginNameFromDeviceIndex(deviceIndex);
      }
      result += idStr;
      return result;
    }
    case SchedulerTimerType_e::TaskDeviceTimer:
    {
      result = F("Task ");

      // Id is taskIndex
      result += (id + 1);
      return result;
    }
    case SchedulerTimerType_e::GPIO_timer:
    {
      uint8_t GPIOType      = static_cast<uint8_t>((id) & 0xFF);
      uint8_t pinNumber     = static_cast<uint8_t>((id >> 8) & 0xFF);
      uint8_t pinStateValue = static_cast<uint8_t>((id >> 16) & 0xFF);

      switch (GPIOType)
      {
        case GPIO_TYPE_INTERNAL:
          result += F("int");
          break;
        case GPIO_TYPE_MCP:
          result += F("MCP");
          break;
        case GPIO_TYPE_PCF:
          result += F("PCF");
          break;
        default:
          result += F("?");
          break;
      }
      result += F(" pin: ");
      result += pinNumber;
      result += F(" state: ");
      result += pinStateValue;
      return result;
    }
    case SchedulerTimerType_e::RulesTimer:
    {
      result = F("Rules#Timer=");
      const unsigned long mask    = (1 << TIMER_ID_SHIFT) - 1;
      const unsigned long timerID = id & mask;
      result += timerID;
      return result;
    }
    case SchedulerTimerType_e::IntendedReboot:
    {
      result += toString(static_cast<ESPEasy_Scheduler::IntendedRebootReason_e>(id));
      return result;
    }
  }
#endif // ifndef BUILD_NO_DEBUG
  result += F(" timer, id: ");
  result += idStr;
  return result;
}

/*********************************************************************************************\
* Handle scheduled timers.
\*********************************************************************************************/
void ESPEasy_Scheduler::handle_schedule() {
  START_TIMER
  unsigned long timer    = 0;
  unsigned long mixed_id = 0;

  if (timePassedSince(last_system_event_run) < 500) {
    // Make sure system event queue will be looked at every now and then.
    mixed_id = msecTimerHandler.getNextId(timer);
  }

  if (RTC.lastMixedSchedulerId != mixed_id) {
    RTC.lastMixedSchedulerId = mixed_id;
    saveToRTC();
  }

  if (mixed_id == 0) {
    // No id ready to run right now.
    // Events are not that important to run immediately.
    // Make sure normal scheduled jobs run at higher priority.
    // backgroundtasks();
    process_system_event_queue();

    // System events may have added one or more rule events, try to process those
    processNextEvent();
    last_system_event_run = millis();
    STOP_TIMER(HANDLE_SCHEDULER_IDLE);
    return;
  }

  SchedulerTimerType_e timerType = SchedulerTimerType_e::SystemEventQueue;
  const unsigned long  id        = decodeSchedulerId(mixed_id, timerType);

  delay(0); // See: https://github.com/letscontrolit/ESPEasy/issues/1818#issuecomment-425351328

  switch (timerType) {
    case SchedulerTimerType_e::ConstIntervalTimer:
      process_interval_timer(static_cast<ESPEasy_Scheduler::IntervalTimer_e>(id), timer);
      break;
    case SchedulerTimerType_e::PLUGIN_TIMER_IN_e:
      process_plugin_task_timer(id);
      break;
    case SchedulerTimerType_e::PLUGIN_ONLY_TIMER_IN_e:
      process_plugin_timer(id);
      break;
    case SchedulerTimerType_e::RulesTimer:
      process_rules_timer(id, timer);
      break;
    case SchedulerTimerType_e::TaskDeviceTimer:
      process_task_device_timer(id, timer);
      break;
    case SchedulerTimerType_e::GPIO_timer:
      process_gpio_timer(id);
      break;

    case SchedulerTimerType_e::SystemEventQueue:
    case SchedulerTimerType_e::IntendedReboot:
      // TD-er: Not really something that needs to be processed here.
      // - SystemEventQueue has its own ScheduledEventQueue which isn't time based.
      // - IntendedReboot is just used to mark the intended reboot reason in RTC.
      break;
  }
  STOP_TIMER(HANDLE_SCHEDULER_TASK);
}

/*********************************************************************************************\
* Interval Timer
* These timers set a new scheduled timer, based on the old value.
* This will make their interval as constant as possible.
\*********************************************************************************************/
void ESPEasy_Scheduler::setNextTimeInterval(unsigned long& timer, const unsigned long step) {
  timer += step;
  const long passed = timePassedSince(timer);

  if (passed < 0) {
    // Event has not yet happened, which is fine.
    return;
  }

  if (static_cast<unsigned long>(passed) > step) {
    // No need to keep running behind, start again.
    timer = millis() + step;
    return;
  }

  // Try to get in sync again.
  timer = millis() + (step - passed);
}

void ESPEasy_Scheduler::setIntervalTimer(IntervalTimer_e id) {
  setIntervalTimer(id, millis());
}

void ESPEasy_Scheduler::setIntervalTimerAt(IntervalTimer_e id, unsigned long newtimer) {
  setNewTimerAt(getMixedId(SchedulerTimerType_e::ConstIntervalTimer, static_cast<unsigned long>(id)), newtimer);
}

void ESPEasy_Scheduler::setIntervalTimerOverride(IntervalTimer_e id, unsigned long msecFromNow) {
  unsigned long timer = millis();

  setNextTimeInterval(timer, msecFromNow);
  setNewTimerAt(getMixedId(SchedulerTimerType_e::ConstIntervalTimer, static_cast<unsigned long>(id)), timer);
}

void ESPEasy_Scheduler::scheduleNextDelayQueue(IntervalTimer_e id, unsigned long nextTime) {
  if (nextTime != 0) {
    // Schedule for next process run.
    setIntervalTimerAt(id, nextTime);
  }
}

void ESPEasy_Scheduler::setIntervalTimer(IntervalTimer_e id, unsigned long lasttimer) {
  // Set the initial timers for the regular runs
  unsigned long interval = 0;

  switch (id) {
    case IntervalTimer_e::TIMER_20MSEC:         interval = 20; break;
    case IntervalTimer_e::TIMER_100MSEC:        interval = 100; break;
    case IntervalTimer_e::TIMER_1SEC:           interval = 1000; break;
    case IntervalTimer_e::TIMER_30SEC:
    case IntervalTimer_e::TIMER_STATISTICS:     interval = 30000; break;
    case IntervalTimer_e::TIMER_MQTT:           interval = timermqtt_interval; break;
    case IntervalTimer_e::TIMER_GRATUITOUS_ARP: interval = timer_gratuitous_arp_interval; break;

    // Fall-through for all DelayQueue, which are just the fall-back timers.
    // The timers for all delay queues will be set according to their own settings as long as there is something to process.
    case IntervalTimer_e::TIMER_MQTT_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C001_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C003_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C004_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C007_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C008_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C009_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C010_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C011_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C012_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C013_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C014_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C015_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C016_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C017_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C018_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C019_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C020_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C021_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C022_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C023_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C024_DELAY_QUEUE:
    case IntervalTimer_e::TIMER_C025_DELAY_QUEUE:
      // When extending this, search for EXTEND_CONTROLLER_IDS
      // in the code to find all places that need to be updated too.
      interval = 1000; break;
  }
  unsigned long timer = lasttimer;

  setNextTimeInterval(timer, interval);
  setNewTimerAt(getMixedId(SchedulerTimerType_e::ConstIntervalTimer, static_cast<unsigned long>(id)), timer);
}

void ESPEasy_Scheduler::sendGratuitousARP_now() {
  sendGratuitousARP();

  if (Settings.gratuitousARP()) {
    timer_gratuitous_arp_interval = 100;
    setIntervalTimer(ESPEasy_Scheduler::IntervalTimer_e::TIMER_GRATUITOUS_ARP);
  }
}

void ESPEasy_Scheduler::process_interval_timer(IntervalTimer_e id, unsigned long lasttimer) {
  // Set the interval timer now, it may be altered by the commands below.
  // This is the default next-run-time.
  setIntervalTimer(id, lasttimer);

  switch (id) {
    case IntervalTimer_e::TIMER_20MSEC:         run50TimesPerSecond(); break;
    case IntervalTimer_e::TIMER_100MSEC:

      if (!UseRTOSMultitasking) {
        run10TimesPerSecond();
      }
      break;
    case IntervalTimer_e::TIMER_1SEC:             runOncePerSecond();      break;
    case IntervalTimer_e::TIMER_30SEC:            runEach30Seconds();      break;
    case IntervalTimer_e::TIMER_MQTT:
#if FEATURE_MQTT
      runPeriodicalMQTT();
#endif // if FEATURE_MQTT
      break;
    case IntervalTimer_e::TIMER_STATISTICS:       logTimerStatistics();    break;
    case IntervalTimer_e::TIMER_GRATUITOUS_ARP:

      // Slowly increase the interval timer.
      timer_gratuitous_arp_interval = 2 * timer_gratuitous_arp_interval;

      if (timer_gratuitous_arp_interval > TIMER_GRATUITOUS_ARP_MAX) {
        timer_gratuitous_arp_interval = TIMER_GRATUITOUS_ARP_MAX;
      }

      if (Settings.gratuitousARP()) {
        sendGratuitousARP();
      }
      break;
    case IntervalTimer_e::TIMER_MQTT_DELAY_QUEUE:
#if FEATURE_MQTT
      processMQTTdelayQueue();
#endif // if FEATURE_MQTT
      break;
    case IntervalTimer_e::TIMER_C001_DELAY_QUEUE:
  #ifdef USES_C001
      process_c001_delay_queue();
  #endif // ifdef USES_C001
      break;
    case IntervalTimer_e::TIMER_C003_DELAY_QUEUE:
  #ifdef USES_C003
      process_c003_delay_queue();
  #endif // ifdef USES_C003
      break;
    case IntervalTimer_e::TIMER_C004_DELAY_QUEUE:
  #ifdef USES_C004
      process_c004_delay_queue();
  #endif // ifdef USES_C004
      break;
    case IntervalTimer_e::TIMER_C007_DELAY_QUEUE:
  #ifdef USES_C007
      process_c007_delay_queue();
  #endif // ifdef USES_C007
      break;
    case IntervalTimer_e::TIMER_C008_DELAY_QUEUE:
  #ifdef USES_C008
      process_c008_delay_queue();
  #endif // ifdef USES_C008
      break;
    case IntervalTimer_e::TIMER_C009_DELAY_QUEUE:
  #ifdef USES_C009
      process_c009_delay_queue();
  #endif // ifdef USES_C009
      break;
    case IntervalTimer_e::TIMER_C010_DELAY_QUEUE:
  #ifdef USES_C010
      process_c010_delay_queue();
  #endif // ifdef USES_C010
      break;
    case IntervalTimer_e::TIMER_C011_DELAY_QUEUE:
  #ifdef USES_C011
      process_c011_delay_queue();
  #endif // ifdef USES_C011
      break;
    case IntervalTimer_e::TIMER_C012_DELAY_QUEUE:
  #ifdef USES_C012
      process_c012_delay_queue();
  #endif // ifdef USES_C012
      break;

    case IntervalTimer_e::TIMER_C013_DELAY_QUEUE:
      /*
       #ifdef USES_C013
            process_c013_delay_queue();
       #endif
       */
      break;

    case IntervalTimer_e::TIMER_C014_DELAY_QUEUE:
      /*
       #ifdef USES_C014
            process_c014_delay_queue();
       #endif
       */
      break;

    case IntervalTimer_e::TIMER_C015_DELAY_QUEUE:
  #ifdef USES_C015
      process_c015_delay_queue();
  #endif // ifdef USES_C015
      break;
    case IntervalTimer_e::TIMER_C016_DELAY_QUEUE:
  #ifdef USES_C016
      process_c016_delay_queue();
  #endif // ifdef USES_C016
      break;

    case IntervalTimer_e::TIMER_C017_DELAY_QUEUE:
  #ifdef USES_C017
      process_c017_delay_queue();
  #endif // ifdef USES_C017
      break;

    case IntervalTimer_e::TIMER_C018_DELAY_QUEUE:
  #ifdef USES_C018
      process_c018_delay_queue();
  #endif // ifdef USES_C018
      break;

    case IntervalTimer_e::TIMER_C019_DELAY_QUEUE:
      /*
       #ifdef USES_C019
            process_c019_delay_queue();
       #endif
       */
      break;

    case IntervalTimer_e::TIMER_C020_DELAY_QUEUE:
      /*
       #ifdef USES_C020
            process_c020_delay_queue();
       #endif
       */
      break;

    case IntervalTimer_e::TIMER_C021_DELAY_QUEUE:
      /*
       #ifdef USES_C021
            process_c021_delay_queue();
       #endif
       */
      break;

    case IntervalTimer_e::TIMER_C022_DELAY_QUEUE:
      /*
       #ifdef USES_C022
            process_c022_delay_queue();
       #endif
       */
      break;

    case IntervalTimer_e::TIMER_C023_DELAY_QUEUE:
      /*
       #ifdef USES_C023
            process_c023_delay_queue();
       #endif
       */
      break;

    case IntervalTimer_e::TIMER_C024_DELAY_QUEUE:
      /*
       #ifdef USES_C024
            process_c024_delay_queue();
       #endif
       */
      break;

    case IntervalTimer_e::TIMER_C025_DELAY_QUEUE:
      /*
       #ifdef USES_C025
            process_c025_delay_queue();
       #endif
       */
      break;

      // When extending this, search for EXTEND_CONTROLLER_IDS
      // in the code to find all places that need to be updated too.
  }
}

/*********************************************************************************************\
* Plugin Task Timer
\*********************************************************************************************/
unsigned long ESPEasy_Scheduler::createPluginTaskTimerId(deviceIndex_t deviceIndex, int Par1) {
  const unsigned long mask  = (1 << TIMER_ID_SHIFT) - 1;
  const unsigned long mixed = (Par1 << 8) + deviceIndex;

  return mixed & mask;
}

void ESPEasy_Scheduler::setPluginTaskTimer(unsigned long msecFromNow, taskIndex_t taskIndex, int Par1, int Par2, int Par3, int Par4, int Par5)
{
  // plugin number and par1 form a unique key that can be used to restart a timer
  // Use deviceIndex instead of pluginID, since the deviceIndex uses less bits.
  const deviceIndex_t deviceIndex = getDeviceIndex_from_TaskIndex(taskIndex);

  if (!validDeviceIndex(deviceIndex)) { return; }

  const unsigned long mixedTimerId = getMixedId(SchedulerTimerType_e::PLUGIN_TIMER_IN_e, createPluginTaskTimerId(deviceIndex, Par1));

  systemTimerStruct timer_data;

  timer_data.TaskIndex       = taskIndex;
  timer_data.Par1            = Par1;
  timer_data.Par2            = Par2;
  timer_data.Par3            = Par3;
  timer_data.Par4            = Par4;
  timer_data.Par5            = Par5;
  systemTimers[mixedTimerId] = timer_data;
  setNewTimerAt(mixedTimerId, millis() + msecFromNow);
}

void ESPEasy_Scheduler::process_plugin_task_timer(unsigned long id) {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  START_TIMER;

  const unsigned long mixedTimerId = getMixedId(SchedulerTimerType_e::PLUGIN_TIMER_IN_e, id);
  auto it                          = systemTimers.find(mixedTimerId);

  if (it == systemTimers.end()) { return; }

  const deviceIndex_t deviceIndex = getDeviceIndex_from_TaskIndex(it->second.TaskIndex);

  struct EventStruct TempEvent(it->second.TaskIndex);

  TempEvent.Par1 = it->second.Par1;
  TempEvent.Par2 = it->second.Par2;
  TempEvent.Par3 = it->second.Par3;
  TempEvent.Par4 = it->second.Par4;
  TempEvent.Par5 = it->second.Par5;

  // TD-er: Not sure if we have to keep original source for notifications.
  TempEvent.Source = EventValueSource::Enum::VALUE_SOURCE_SYSTEM;


  /*
     String log = F("proc_system_timer: Pluginid: ");
     log += deviceIndex;
     log += F(" taskIndex: ");
     log += it->second.TaskIndex;
     log += F(" sysTimerID: ");
     log += id;
     addLog(LOG_LEVEL_INFO, log);
   */
  systemTimers.erase(mixedTimerId);

  if (validDeviceIndex(deviceIndex)) {
    if (validUserVarIndex(TempEvent.BaseVarIndex)) {
      // checkDeviceVTypeForTask(&TempEvent);
      String dummy;
      Plugin_ptr[deviceIndex](PLUGIN_TIMER_IN, &TempEvent, dummy);
    }
  }
  STOP_TIMER(PROC_SYS_TIMER);
}

/*********************************************************************************************\
* Rules Timer
\*********************************************************************************************/
unsigned long ESPEasy_Scheduler::createRulesTimerId(unsigned int timerIndex) {
  const unsigned long mask  = (1 << TIMER_ID_SHIFT) - 1;
  const unsigned long mixed = timerIndex;

  return mixed & mask;
}

static bool checkRulesTimerIndex(unsigned int timerIndex) {
  if ((timerIndex > RULES_TIMER_MAX) || (timerIndex == 0)) {
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log = F("TIMER: invalid timer number ");
      log += timerIndex;
      addLogMove(LOG_LEVEL_ERROR, log);
    }
    return false;
  }
  return true;
}

bool ESPEasy_Scheduler::setRulesTimer(unsigned long msecFromNow, unsigned int timerIndex, int recurringCount) {
  if (!checkRulesTimerIndex(timerIndex)) { return false; }

  const unsigned long mixedTimerId = getMixedId(SchedulerTimerType_e::RulesTimer, createRulesTimerId(timerIndex));
  const systemTimerStruct timer_data(recurringCount, msecFromNow, timerIndex);

  systemTimers[mixedTimerId] = timer_data;
  setNewTimerAt(mixedTimerId, millis() + msecFromNow);
  return true;
}

void ESPEasy_Scheduler::process_rules_timer(unsigned long id, unsigned long lasttimer) {
  const unsigned long mixedTimerId = getMixedId(SchedulerTimerType_e::RulesTimer, id);

  auto it = systemTimers.find(mixedTimerId);

  if (it == systemTimers.end()) { return; }

  if (it->second.isPaused()) {
    // Timer is paused.
    // Must keep this timer 'active' in the scheduler.
    // However it does not need to be looked at frequently as the resume function re-schedules it when needed.
    // Look for its state every second.
    setNewTimerAt(mixedTimerId, millis() + 1000);
    return;
  }

  // Create a deep copy of the timer data as we may delete it from the map before sending the event.
  const int loopCount  = it->second.getLoopCount();
  const int timerIndex = it->second.getTimerIndex();

  // Reschedule before sending the event, as it may get rescheduled in handling the timer event.
  if (it->second.isRecurring()) {
    // Recurring timer
    unsigned long newTimer = lasttimer;
    setNextTimeInterval(newTimer, it->second.getInterval());
    setNewTimerAt(mixedTimerId, newTimer);
    it->second.markNextRecurring();
  } else {
    systemTimers.erase(mixedTimerId);
  }

  if (loopCount > 0) {
    // Should be executed
    if (Settings.UseRules) {
      String event = F("Rules#Timer=");
      event += timerIndex;

      // Add count as 2nd eventvalue
      event += ',';
      event += loopCount;
      rulesProcessing(event); // TD-er: Do not add to the eventQueue, but execute right now.
    }
  }
}

bool ESPEasy_Scheduler::pause_rules_timer(unsigned long timerIndex) {
  if (!checkRulesTimerIndex(timerIndex)) { return false; }
  const unsigned long mixedTimerId = getMixedId(SchedulerTimerType_e::RulesTimer, createRulesTimerId(timerIndex));
  auto it                          = systemTimers.find(mixedTimerId);

  if (it == systemTimers.end()) {
    addLog(LOG_LEVEL_INFO, F("TIMER: no timer set"));
    return false;
  }

  unsigned long timer;

  if (msecTimerHandler.getTimerForId(mixedTimerId, timer)) {
    if (it->second.isPaused()) {
      addLog(LOG_LEVEL_INFO, F("TIMER: already paused"));
    } else {
      // Store remainder of interval
      const long timeLeft = timePassedSince(timer) * -1;

      if (timeLeft > 0) {
        it->second.setRemainder(timeLeft);
        return true;
      }
    }
  }
  return false;
}

bool ESPEasy_Scheduler::resume_rules_timer(unsigned long timerIndex) {
  if (!checkRulesTimerIndex(timerIndex)) { return false; }
  const unsigned long mixedTimerId = getMixedId(SchedulerTimerType_e::RulesTimer, createRulesTimerId(timerIndex));
  auto it                          = systemTimers.find(mixedTimerId);

  if (it == systemTimers.end()) { return false; }

  unsigned long timer;

  if (msecTimerHandler.getTimerForId(mixedTimerId, timer)) {
    if (it->second.isPaused()) {
      // Reschedule timer with remainder of interval
      setNewTimerAt(mixedTimerId, millis() + it->second.getRemainder());
      it->second.setRemainder(0);
      return true;
    }
  }
  return false;
}

/*********************************************************************************************\
* Plugin Timer
* Essentially calling PLUGIN_ONLY_TIMER_IN
* Similar to PLUGIN_TIMER_IN, addressed to a plugin instead of a task.
\*********************************************************************************************/
unsigned long ESPEasy_Scheduler::createPluginTimerId(deviceIndex_t deviceIndex, int Par1) {
  const unsigned long mask  = (1 << TIMER_ID_SHIFT) - 1;
  const unsigned long mixed = (Par1 << 8) + deviceIndex;

  return mixed & mask;
}

void ESPEasy_Scheduler::setPluginTimer(unsigned long msecFromNow, pluginID_t pluginID, int Par1, int Par2, int Par3, int Par4, int Par5)
{
  // plugin number and par1 form a unique key that can be used to restart a timer
  // Use deviceIndex instead of pluginID, since the deviceIndex uses less bits.
  const deviceIndex_t deviceIndex = getDeviceIndex(pluginID);

  if (!validDeviceIndex(deviceIndex)) { return; }

  const unsigned long mixedTimerId = getMixedId(SchedulerTimerType_e::PLUGIN_ONLY_TIMER_IN_e, createPluginTimerId(deviceIndex, Par1));
  systemTimerStruct   timer_data;

  // PLUGIN_ONLY_TIMER_IN does not address a task, so don't set TaskIndex
  timer_data.Par1            = Par1;
  timer_data.Par2            = Par2;
  timer_data.Par3            = Par3;
  timer_data.Par4            = Par4;
  timer_data.Par5            = Par5;
  systemTimers[mixedTimerId] = timer_data;
  setNewTimerAt(mixedTimerId, millis() + msecFromNow);
}

void ESPEasy_Scheduler::process_plugin_timer(unsigned long id) {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  START_TIMER;
  const unsigned long mixedTimerId = getMixedId(SchedulerTimerType_e::PLUGIN_ONLY_TIMER_IN_e, id);
  auto it                          = systemTimers.find(mixedTimerId);

  if (it == systemTimers.end()) { return; }

  struct EventStruct TempEvent;

  // PLUGIN_ONLY_TIMER_IN does not address a task, so don't set TaskIndex

  // extract deviceID from timer id:
  const deviceIndex_t deviceIndex = ((1 << 8) - 1) & id;

  TempEvent.Par1 = it->second.Par1;
  TempEvent.Par2 = it->second.Par2;
  TempEvent.Par3 = it->second.Par3;
  TempEvent.Par4 = it->second.Par4;
  TempEvent.Par5 = it->second.Par5;

  // TD-er: Not sure if we have to keep original source for notifications.
  TempEvent.Source = EventValueSource::Enum::VALUE_SOURCE_SYSTEM;

  //  const deviceIndex_t deviceIndex = getDeviceIndex_from_TaskIndex(it->second.TaskIndex);

  /*
     String log = F("proc_system_timer: Pluginid: ");
     log += deviceIndex;
     log += F(" taskIndex: ");
     log += it->second.TaskIndex;
     log += F(" sysTimerID: ");
     log += id;
     addLog(LOG_LEVEL_INFO, log);
   */
  systemTimers.erase(mixedTimerId);

  if (validDeviceIndex(deviceIndex)) {
    String dummy;
    Plugin_ptr[deviceIndex](PLUGIN_ONLY_TIMER_IN, &TempEvent, dummy);
  }
  STOP_TIMER(PROC_SYS_TIMER);
}

/*********************************************************************************************\
* GPIO Timer
* Special timer to handle timed GPIO actions
\*********************************************************************************************/
unsigned long ESPEasy_Scheduler::createGPIOTimerId(uint8_t GPIOType, uint8_t pinNumber, int Par1) {
  const unsigned long mask = (1 << TIMER_ID_SHIFT) - 1;

  //  const unsigned long mixed = (Par1 << 8) + pinNumber;
  const unsigned long mixed = (Par1 << 16) + (pinNumber << 8) + GPIOType;

  return mixed & mask;
}

void ESPEasy_Scheduler::setGPIOTimer(unsigned long msecFromNow, pluginID_t pluginID, int Par1, int Par2, int Par3, int Par4, int Par5)
{
  uint8_t GPIOType = GPIO_TYPE_INVALID;

  switch (pluginID) {
    case PLUGIN_GPIO:
      GPIOType = GPIO_TYPE_INTERNAL;
      break;
    case PLUGIN_PCF:
      GPIOType = GPIO_TYPE_PCF;
      break;
    case PLUGIN_MCP:
      GPIOType = GPIO_TYPE_MCP;
      break;
  }

  if (GPIOType != GPIO_TYPE_INVALID) {
    // Par1 & Par2 & GPIOType form a unique key
    const unsigned long mixedTimerId = getMixedId(SchedulerTimerType_e::GPIO_timer, createGPIOTimerId(GPIOType, Par1, Par2));
    setNewTimerAt(mixedTimerId, millis() + msecFromNow);
  }
}

void ESPEasy_Scheduler::process_gpio_timer(unsigned long id) {
  uint8_t GPIOType      = static_cast<uint8_t>((id) & 0xFF);
  uint8_t pinNumber     = static_cast<uint8_t>((id >> 8) & 0xFF);
  uint8_t pinStateValue = static_cast<uint8_t>((id >> 16) & 0xFF);

  bool success = true;

  uint8_t pluginID;

  switch (GPIOType)
  {
    case GPIO_TYPE_INTERNAL:
      GPIO_Internal_Write(pinNumber, pinStateValue);
      pluginID = PLUGIN_GPIO;
      break;
    case GPIO_TYPE_MCP:
      GPIO_MCP_Write(pinNumber, pinStateValue);
      pluginID = PLUGIN_MCP;
      break;
    case GPIO_TYPE_PCF:
      GPIO_PCF_Write(pinNumber, pinStateValue);
      pluginID = PLUGIN_PCF;
      break;
    default:
      success = false;
  }

  if (success) {
    const uint32_t key = createKey(pluginID, pinNumber);

    // WARNING: operator [] creates an entry in the map if key does not exist
    portStatusStruct tempStatus = globalMapPortStatus[key];

    tempStatus.mode    = PIN_MODE_OUTPUT;
    tempStatus.command = 1; // set to 1 in order to display the status in the PinStatus page

    if (tempStatus.state != pinStateValue) {
      tempStatus.state        = pinStateValue;
      tempStatus.output       = pinStateValue;
      tempStatus.forceEvent   = 1;
      tempStatus.forceMonitor = 1;
    }
    savePortStatus(key, tempStatus);
  }
}

/*********************************************************************************************\
* Task Device Timer
* This is the interval set in a plugin to get a new reading.
* These timers will re-schedule themselves as long as the plugin task is enabled.
* When the plugin task is initialized, a call to schedule_task_device_timer_at_init
* will bootstrap this sequence.
\*********************************************************************************************/
void ESPEasy_Scheduler::schedule_task_device_timer_at_init(unsigned long task_index) {
  unsigned long runAt = millis();

  if (!isDeepSleepEnabled()) {
    // Deepsleep is not enabled, add some offset based on the task index
    // to make sure not all are run at the same time.
    // This scheduled time may be overriden by the plugin's own init.
    runAt += (task_index * 37) + 100;
  } else {
    runAt += (task_index * 11) + 10;
  }
  schedule_task_device_timer(task_index, runAt);
}

// Typical use case is to run this when all needed connections are made.
void ESPEasy_Scheduler::schedule_all_task_device_timers() {
  for (taskIndex_t task = 0; task < TASKS_MAX; task++) {
    schedule_task_device_timer_at_init(task);
  }
}

void ESPEasy_Scheduler::schedule_task_device_timer(unsigned long task_index, unsigned long runAt) {
  /*
     String log = F("schedule_task_device_timer: task: ");
     log += task_index;
     log += F(" @ ");
     log += runAt;
     if (Settings.TaskDeviceEnabled[task_index]) {
      log += F(" (enabled)");
     }
     addLog(LOG_LEVEL_INFO, log);
   */

  if (!validTaskIndex(task_index)) { return; }

  const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(task_index);

  if (!validDeviceIndex(DeviceIndex)) { return; }

  // TD-er: Tasks without a timer or optional timer set to 0 should still be able to call PLUGIN_READ
  // For example to schedule a read from the PLUGIN_TEN_PER_SECOND when a new value is ready.

  /*
     if (!Device[DeviceIndex].TimerOption) { return; }

     if (Device[DeviceIndex].TimerOptional && (Settings.TaskDeviceTimer[task_index] == 0)) {
      return;
     }
   */

  if (Settings.TaskDeviceEnabled[task_index]) {
    setNewTimerAt(getMixedId(SchedulerTimerType_e::TaskDeviceTimer, task_index), runAt);
  }
}

void ESPEasy_Scheduler::reschedule_task_device_timer(unsigned long task_index, unsigned long lasttimer) {
  if (!validTaskIndex(task_index)) { return; }
  unsigned long newtimer = Settings.TaskDeviceTimer[task_index];

  if (newtimer != 0) {
    newtimer = lasttimer + (newtimer * 1000);
    schedule_task_device_timer(task_index, newtimer);
  }
}

void ESPEasy_Scheduler::process_task_device_timer(unsigned long task_index, unsigned long lasttimer) {
  if (!validTaskIndex(task_index)) { return; }
  reschedule_task_device_timer(task_index, lasttimer);
  START_TIMER;
  SensorSendTask(task_index);
  STOP_TIMER(SENSOR_SEND_TASK);
}

/*********************************************************************************************\
* System Event Timer
* Handling of these events will be asynchronous and being called from the loop().
* Thus only use these when the result is not needed immediately.
* Proper use case is calling from a callback function, since those cannot use yield() or delay()
\*********************************************************************************************/
void ESPEasy_Scheduler::schedule_plugin_task_event_timer(deviceIndex_t DeviceIndex, uint8_t Function, struct EventStruct&& event) {
  if (validDeviceIndex(DeviceIndex)) {
    schedule_event_timer(PluginPtrType::TaskPlugin, DeviceIndex, Function, std::move(event));
  }
}

#if FEATURE_MQTT
void ESPEasy_Scheduler::schedule_mqtt_plugin_import_event_timer(deviceIndex_t DeviceIndex,
                                                                taskIndex_t   TaskIndex,
                                                                uint8_t       Function,
                                                                char         *c_topic,
                                                                uint8_t      *b_payload,
                                                                unsigned int  length) {
  if (validDeviceIndex(DeviceIndex)) {
    const unsigned long mixedId = createSystemEventMixedId(PluginPtrType::TaskPlugin, DeviceIndex, static_cast<uint8_t>(Function));
    EventStruct  event(TaskIndex);
    const size_t topic_length = strlen_P(c_topic);

    if (!(event.String1.reserve(topic_length) && event.String2.reserve(length))) {
      addLog(LOG_LEVEL_ERROR, F("MQTT : Out of Memory! Cannot process MQTT message"));
      return;
    }

    for (size_t i = 0; i < topic_length; ++i) {
      event.String1 += c_topic[i];
    }

    for (unsigned int i = 0; i < length; ++i) {
      const char c = static_cast<char>(*(b_payload + i));
      event.String2 += c;
    }

    // Emplace using move.
    // This makes sure the relatively large event will not be in memory twice.
    ScheduledEventQueue.emplace_back(mixedId, std::move(event));
  }
}
#endif

void ESPEasy_Scheduler::schedule_controller_event_timer(protocolIndex_t ProtocolIndex, uint8_t Function, struct EventStruct&& event) {
  if (validProtocolIndex(ProtocolIndex)) {
    schedule_event_timer(PluginPtrType::ControllerPlugin, ProtocolIndex, Function, std::move(event));
  }
}

unsigned long ESPEasy_Scheduler::createSystemEventMixedId(PluginPtrType ptr_type, uint8_t Index, uint8_t Function) {
  unsigned long subId = static_cast<unsigned long>(ptr_type);

  subId = (subId << 8) + Index;
  subId = (subId << 8) + Function;
  return getMixedId(SchedulerTimerType_e::SystemEventQueue, subId);
}

#if FEATURE_MQTT
void ESPEasy_Scheduler::schedule_mqtt_controller_event_timer(protocolIndex_t   ProtocolIndex,
                                                             CPlugin::Function Function,
                                                             char             *c_topic,
                                                             uint8_t          *b_payload,
                                                             unsigned int      length) {
  if (validProtocolIndex(ProtocolIndex)) {
    // Emplace empty event in the queue first and the fill it.
    // This makes sure the relatively large event will not be in memory twice.
    const unsigned long mixedId = createSystemEventMixedId(PluginPtrType::ControllerPlugin, ProtocolIndex, static_cast<uint8_t>(Function));
    ScheduledEventQueue.emplace_back(mixedId, EventStruct());
    ScheduledEventQueue.back().event.String1 = c_topic;

    String& payload = ScheduledEventQueue.back().event.String2;

    if (!payload.reserve(length)) {
      addLog(LOG_LEVEL_ERROR, F("MQTT : Out of Memory! Cannot process MQTT message"));
    }

    for (unsigned int i = 0; i < length; ++i) {
      char c = static_cast<char>(*(b_payload + i));
      payload += c;
    }
  }
}
#endif

#if FEATURE_NOTIFIER
void ESPEasy_Scheduler::schedule_notification_event_timer(uint8_t              NotificationProtocolIndex,
                                                          NPlugin::Function    Function,
                                                          struct EventStruct&& event) {
  schedule_event_timer(PluginPtrType::NotificationPlugin, NotificationProtocolIndex, static_cast<uint8_t>(Function), std::move(event));
}
#endif

void ESPEasy_Scheduler::schedule_event_timer(PluginPtrType ptr_type, uint8_t Index, uint8_t Function, struct EventStruct&& event) {
  const unsigned long mixedId = createSystemEventMixedId(ptr_type, Index, Function);

  //  EventStructCommandWrapper eventWrapper(mixedId, *event);
  //  ScheduledEventQueue.push_back(eventWrapper);
  ScheduledEventQueue.emplace_back(mixedId, std::move(event));
}

void ESPEasy_Scheduler::process_system_event_queue() {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  if (ScheduledEventQueue.size() == 0) { return; }

  START_TIMER

  const unsigned long id = ScheduledEventQueue.front().id;

  if (RTC.lastMixedSchedulerId != id) {
    RTC.lastMixedSchedulerId = id;
    saveToRTC();
  }

  uint8_t Function       = id & 0xFF;
  uint8_t Index          = (id >> 8) & 0xFF;
  PluginPtrType ptr_type = static_cast<PluginPtrType>((id >> 16) & 0xFF);

  // At this moment, the String is not being used in the plugin calls, so just supply a dummy String.
  // Also since these events will be processed asynchronous, the resulting
  //   output in the String is probably of no use elsewhere.
  // Else the line string could be used.
  String tmpString;

  switch (ptr_type) {
    case PluginPtrType::TaskPlugin:

      if (validDeviceIndex(Index)) {
        if (Function != PLUGIN_READ || Device[Index].ErrorStateValues) {
          LoadTaskSettings(ScheduledEventQueue.front().event.TaskIndex);
        }
        Plugin_ptr[Index](Function, &ScheduledEventQueue.front().event, tmpString);
      }
      break;
    case PluginPtrType::ControllerPlugin:
      CPluginCall(Index, static_cast<CPlugin::Function>(Function), &ScheduledEventQueue.front().event, tmpString);
      break;
#if FEATURE_NOTIFIER
    case PluginPtrType::NotificationPlugin:
      NPlugin_ptr[Index](static_cast<NPlugin::Function>(Function), &ScheduledEventQueue.front().event, tmpString);
      break;
#endif
  }
  ScheduledEventQueue.pop_front();
  STOP_TIMER(PROCESS_SYSTEM_EVENT_QUEUE);
}

String ESPEasy_Scheduler::getQueueStats() {
  return msecTimerHandler.getQueueStats();
}

void ESPEasy_Scheduler::updateIdleTimeStats() {
  msecTimerHandler.updateIdleTimeStats();
}

float ESPEasy_Scheduler::getIdleTimePct() const {
  return msecTimerHandler.getIdleTimePct();
}

void ESPEasy_Scheduler::setEcoMode(bool enabled) {
  msecTimerHandler.setEcoMode(enabled);
}

#include "../Helpers/_Plugin_Helper_webform.h"

#include "../Globals/GlobalMapPortStatus.h"

#include "../Helpers/PortStatus.h"

#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Forms.h"


#define SWITCH_DOUBLECLICK_MAX_INTERVAL      3000
#define SWITCH_LONGPRESS_MAX_INTERVAL        5000
#define SWITCH_DC_DISABLED                   0
#define SWITCH_DC_LOW                        1
#define SWITCH_DC_HIGH                       2
#define SWITCH_DC_BOTH                       3
#define SWITCH_LONGPRESS_DISABLED            0
#define SWITCH_LONGPRESS_LOW                 1
#define SWITCH_LONGPRESS_HIGH                2
#define SWITCH_LONGPRESS_BOTH                3


void SwitchWebformLoad(
  const int16_t& bootState,
  const float  & debounce_ms,
  const int16_t& doubleClickEvent,
  float        & doubleClickMaxInterval,
  const int16_t& longPressEvent,
  float        & longPressMinInterval_ms,
  const float  & useSafeButton)
{
  addFormCheckBox(F("Send Boot state"), F("sw_boot"),
                  bootState);

  addFormSubHeader(F("Advanced event management"));

  addFormNumericBox(F("De-bounce (ms)"), F("sw_debounce"), round(debounce_ms), 0, 250);

  // set minimum value for doubleclick MIN max speed
  if (doubleClickMaxInterval < SWITCH_DOUBLECLICK_MIN_INTERVAL) {
    doubleClickMaxInterval = SWITCH_DOUBLECLICK_MIN_INTERVAL;
  }

  {
    uint8_t choiceDC                       = doubleClickEvent;
    const __FlashStringHelper *buttonDC[4] = {
      F("Disabled"),
      F("Active only on LOW (EVENT=3)"),
      F("Active only on HIGH (EVENT=3)"),
      F("Active on LOW & HIGH (EVENT=3)")
    };
    int buttonDCValues[4] = { SWITCH_DC_DISABLED, SWITCH_DC_LOW, SWITCH_DC_HIGH, SWITCH_DC_BOTH };

    addFormSelector(F("Doubleclick event"), F("sw_dc"), 4, buttonDC, buttonDCValues, choiceDC);
  }

  addFormNumericBox(F("Doubleclick max. interval (ms)"),
                    F("sw_dcmaxinterval"),
                    round(doubleClickMaxInterval),
                    SWITCH_DOUBLECLICK_MIN_INTERVAL,
                    SWITCH_DOUBLECLICK_MAX_INTERVAL);

  // set minimum value for longpress MIN max speed
  if (longPressMinInterval_ms < SWITCH_LONGPRESS_MIN_INTERVAL) {
    longPressMinInterval_ms = SWITCH_LONGPRESS_MIN_INTERVAL;
  }

  {
    uint8_t choiceLP                       = longPressEvent;
    const __FlashStringHelper *buttonLP[4] = {
      F("Disabled"),
      F("Active only on LOW (EVENT= 10 [NORMAL] or 11 [INVERSED])"),
      F("Active only on HIGH (EVENT= 11 [NORMAL] or 10 [INVERSED])"),
      F("Active on LOW & HIGH (EVENT= 10 or 11)")
    };
    int buttonLPValues[4] =
    { SWITCH_LONGPRESS_DISABLED, SWITCH_LONGPRESS_LOW, SWITCH_LONGPRESS_HIGH, SWITCH_LONGPRESS_BOTH };
    addFormSelector(F("Longpress event"), F("sw_lp"), 4, buttonLP, buttonLPValues, choiceLP);
  }

  addFormNumericBox(F("Longpress min. interval (ms)"),
                    F("sw_lpmininterval"),
                    round(longPressMinInterval_ms),
                    SWITCH_LONGPRESS_MIN_INTERVAL,
                    SWITCH_LONGPRESS_MAX_INTERVAL);

  addFormCheckBox(F("Use Safe Button (slower)"), F("sw_sb"), round(useSafeButton));

  // TO-DO: add Extra-Long Press event
  // addFormCheckBox(F("Extra-Longpress event (20 & 21)"), F("sw_elp"), PCONFIG_LONG(1));
  // addFormNumericBox(F("Extra-Longpress min. interval (ms)"), F("sw_elpmininterval"), PCONFIG_LONG(2), 500, 2000);
}

void SwitchWebformSave(
  taskIndex_t TaskIndex,
  pluginID_t  pluginID,
  int16_t   & bootState,
  float     & debounce_ms,
  int16_t   & doubleClickEvent,
  float     & doubleClickMaxInterval,
  int16_t   & longPressEvent,
  float     & longPressMinInterval_ms,
  float     & useSafeButton)
{
  bootState = isFormItemChecked(F("sw_boot"));

  debounce_ms = getFormItemInt(F("sw_debounce"));

  doubleClickEvent       = getFormItemInt(F("sw_dc"));
  doubleClickMaxInterval = getFormItemInt(F("sw_dcmaxinterval"));

  longPressEvent          = getFormItemInt(F("sw_lp"));
  longPressMinInterval_ms = getFormItemInt(F("sw_lpmininterval"));

  useSafeButton = isFormItemChecked(F("sw_sb"));

  // TO-DO: add Extra-Long Press event
  // PCONFIG_LONG(1) = isFormItemChecked(F("sw_elp"));
  // PCONFIG_LONG(2) = getFormItemInt(F("sw_elpmininterval"));

  // check if a task has been edited and remove 'task' bit from the previous pin
  for (std::map<uint32_t, portStatusStruct>::iterator it = globalMapPortStatus.begin(); it != globalMapPortStatus.end(); ++it) {
    if ((it->second.previousTask == TaskIndex) && (getPluginFromKey(it->first) == pluginID)) {
      globalMapPortStatus[it->first].previousTask = -1;
      removeTaskFromPort(it->first);
      break;
    }
  }
}

#include "../Helpers/PortStatus.h"

#include "../DataStructs/PinMode.h"
#include "../Globals/GlobalMapPortStatus.h"

#include "../../ESPEasy-Globals.h"


#ifdef ESP32

#include "../Helpers/Hardware.h"

void checkAndClearPWM(uint32_t key) {
  if (existPortStatus(key)) {
    switch (globalMapPortStatus[key].mode) {
      case PIN_MODE_PWM:
      case PIN_MODE_SERVO:
        {
          const uint16_t port = getPortFromKey(key);
          analogWriteESP32(port, 0);
        }
        break;
    }
  }
}

#endif


/**********************************************************
*                                                         *
* Helper Functions for managing the status data structure *
*                                                         *
**********************************************************/
void savePortStatus(uint32_t key, struct portStatusStruct& tempStatus) {
  // FIXME TD-er: task and monitor are unsigned, should we only check for == ????
  if ((tempStatus.task <= 0) && (tempStatus.monitor <= 0) && (tempStatus.command <= 0)) {
    #ifdef ESP32
    checkAndClearPWM(key);
    #endif

    globalMapPortStatus.erase(key);
  }
  else {
    #ifdef ESP32
    switch (tempStatus.mode) {
      case PIN_MODE_PWM:
      case PIN_MODE_SERVO:
        break;
      default:
        checkAndClearPWM(key);
        break;
    }
    #endif

    globalMapPortStatus[key] = tempStatus;
  }
}

bool existPortStatus(uint32_t key) {
  return globalMapPortStatus.find(key) != globalMapPortStatus.end();
}

void removeTaskFromPort(uint32_t key) {
  const auto it = globalMapPortStatus.find(key);
  if (it != globalMapPortStatus.end()) {
    (it->second.task > 0) ? it->second.task-- : it->second.task = 0;

    if ((it->second.task <= 0) && (it->second.monitor <= 0) && (it->second.command <= 0) &&
        (it->second.init <= 0)) {
      // erase using the key, so the iterator can be const
      #ifdef ESP32
      checkAndClearPWM(key);
      #endif

      globalMapPortStatus.erase(key);
    }
  }
}

void removeMonitorFromPort(uint32_t key) {
  const auto it = globalMapPortStatus.find(key);
  if (it != globalMapPortStatus.end()) {
    it->second.monitor = 0;

    if ((it->second.task <= 0) && (it->second.monitor <= 0) && (it->second.command <= 0) &&
        (it->second.init <= 0)) {
      // erase using the key, so the iterator can be const
      #ifdef ESP32
      checkAndClearPWM(key);
      #endif

      globalMapPortStatus.erase(key);
    }
  }
}

void addMonitorToPort(uint32_t key) {
  globalMapPortStatus[key].monitor = 1;
}

uint32_t createKey(uint16_t pluginNumber, uint16_t portNumber) {
  return (uint32_t)pluginNumber << 16 | portNumber;
}

pluginID_t getPluginFromKey(uint32_t key) {
  return static_cast<pluginID_t>((key >> 16) & 0xFFFF);
}

uint16_t getPortFromKey(uint32_t key) {
  return static_cast<uint16_t>(key & 0xFFFF);
}




/*********************************************************************************************\
   set pin mode & state (info table)
\*********************************************************************************************/
/*
   void setPinState(uint8_t plugin, uint8_t index, uint8_t mode, uint16_t value)
   {
   // plugin number and index form a unique key
   // first check if this pin is already known
   bool reUse = false;
   for (uint8_t x = 0; x < PINSTATE_TABLE_MAX; x++)
    if ((pinStates[x].plugin == plugin) && (pinStates[x].index == index))
    {
      pinStates[x].mode = mode;
      pinStates[x].value = value;
      reUse = true;
      break;
    }

   if (!reUse)
   {
    for (uint8_t x = 0; x < PINSTATE_TABLE_MAX; x++)
      if (pinStates[x].plugin == 0)
      {
        pinStates[x].plugin = plugin;
        pinStates[x].index = index;
        pinStates[x].mode = mode;
        pinStates[x].value = value;
        break;
      }
   }
   }
 */

/*********************************************************************************************\
   get pin mode & state (info table)
\*********************************************************************************************/

/*
   bool getPinState(uint8_t plugin, uint8_t index, uint8_t *mode, uint16_t *value)
   {
   for (uint8_t x = 0; x < PINSTATE_TABLE_MAX; x++)
    if ((pinStates[x].plugin == plugin) && (pinStates[x].index == index))
    {
 * mode = pinStates[x].mode;
 * value = pinStates[x].value;
      return true;
    }
   return false;
   }

 */
/*********************************************************************************************\
   check if pin mode & state is known (info table)
\*********************************************************************************************/
/*
   bool hasPinState(uint8_t plugin, uint8_t index)
   {
   for (uint8_t x = 0; x < PINSTATE_TABLE_MAX; x++)
    if ((pinStates[x].plugin == plugin) && (pinStates[x].index == index))
    {
      return true;
    }
   return false;
   }

 */


/*********************************************************************************************\
   report pin mode & state (info table) using json
\*********************************************************************************************/
String getPinStateJSON(bool search, uint32_t key, const String& log, int16_t noSearchValue)
{
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("getPinStateJSON"));
  #endif
  printToWebJSON = true;
  uint8_t mode     = PIN_MODE_INPUT;
  int16_t value = noSearchValue;
  bool    found = false;

  if (search) {
    const auto it = globalMapPortStatus.find(key);
    if (it != globalMapPortStatus.end()) {
      mode  = it->second.mode;
      value = it->second.getValue();
      found = true;
    }
  }

  if (!search || found)
  {
    String reply;
    reply.reserve(128);
    reply += F("{\n\"log\": \"");
    {
      // truncate to 25 chars, max MQTT message size = 128 including header...
      int colonPos = 1 + log.indexOf(':');

      String tmp = log.substring(colonPos, colonPos + 25);
      tmp.trim();      
      reply += tmp;
    }
    reply += F("\",\n\"plugin\": ");
    reply += getPluginFromKey(key);
    reply += F(",\n\"pin\": ");
    reply += getPortFromKey(key);
    reply += F(",\n\"mode\": \"");
    reply += getPinModeString(mode);
    reply += F("\",\n\"state\": ");
    reply += value;
    reply += F("\n}\n");
    return reply;
  }
  return "";
}

const __FlashStringHelper * getPinModeString(uint8_t mode) {
  switch (mode)
  {
    case PIN_MODE_UNDEFINED:    return F("undefined");
    case PIN_MODE_INPUT:        return F("input");
    case PIN_MODE_INPUT_PULLUP: return F("input pullup");
    case PIN_MODE_INPUT_PULLDOWN: return F("input pulldown");
    case PIN_MODE_OFFLINE:      return F("offline");
    case PIN_MODE_OUTPUT:       return F("output");
    case PIN_MODE_PWM:          return F("PWM");
    case PIN_MODE_SERVO:        return F("servo");
    default:
      break;
  }
  return F("ERROR: Not Defined");
}
#include "../Helpers/MDNS_Helper.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyEth.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"

#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/NetworkState.h"
#include "../Globals/Services.h"
#include "../Globals/Settings.h"

#include "../Helpers/StringProvider.h"

void set_mDNS() {
  #if FEATURE_MDNS

  if (!WiFiEventData.WiFiServicesInitialized()) { return; }

  if (webserverRunning) {
    if (!mDNS_init) {
      addLog(LOG_LEVEL_INFO, F("WIFI : Starting mDNS..."));
      mDNS_init = MDNS.begin(NetworkGetHostname().c_str());
      MDNS.setInstanceName(NetworkGetHostname()); // Needed for when the hostname has changed.

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("WIFI : ");

        if (mDNS_init) {
          log += F("mDNS started, with name: ");
          log += getValue(LabelType::M_DNS);
        }
        else {
          log += F("mDNS failed");
        }
        addLogMove(LOG_LEVEL_INFO, log);
      }
      if (mDNS_init) {
        MDNS.addService(F("http"), F("tcp"), Settings.WebserverPort);
      }
    }
  } else {
    #ifdef ESP8266
    if (mDNS_init) {
      MDNS.close();
    }
    mDNS_init = false;
    #endif
  }
  #endif // if FEATURE_MDNS
}

#include "../Helpers/SystemVariables.h"


#include "../../ESPEasy_common.h"

#include "../../ESPEasy-Globals.h"

#include "../CustomBuild/CompiletimeDefines.h"

#include "../DataStructs/TimingStats.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"

#include "../Globals/CRCValues.h"
#include "../Globals/ESPEasy_time.h"
#include "../Globals/ESPEasyWiFiEvent.h"
#if FEATURE_MQTT
# include "../Globals/MQTT.h"
#endif // if FEATURE_MQTT
#include "../Globals/NetworkState.h"
#include "../Globals/RuntimeData.h"
#include "../Globals/Settings.h"
#include "../Globals/Statistics.h"

#include "../Helpers/Convert.h"
#include "../Helpers/Hardware.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Numerical.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringProvider.h"


String getReplacementString(const String& format, String& s) {
  int startpos = s.indexOf(format);
  int endpos   = s.indexOf('%', startpos + 1);
  String R     = s.substring(startpos, endpos + 1);

#ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("ReplacementString SunTime: ");
    log += R;
    log += F(" offset: ");
    log += ESPEasy_time::getSecOffset(R);
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
#endif // ifndef BUILD_NO_DEBUG
  return R;
}

void replSunRiseTimeString(const String& format, String& s, boolean useURLencode) {
  String R = getReplacementString(format, s);

  repl(R, node_time.getSunriseTimeString(':', ESPEasy_time::getSecOffset(R)), s, useURLencode);
}

void replSunSetTimeString(const String& format, String& s, boolean useURLencode) {
  String R = getReplacementString(format, s);

  repl(R, node_time.getSunsetTimeString(':', ESPEasy_time::getSecOffset(R)), s, useURLencode);
}

String timeReplacement_leadZero(int value)
{
  char valueString[5] = { 0 };

  sprintf_P(valueString, PSTR("%02d"), value);
  return valueString;
}

// FIXME TD-er: Try to match these with  StringProvider::getValue
LabelType::Enum SystemVariables2LabelType(SystemVariables::Enum enumval) {
  LabelType::Enum label = LabelType::MAX_LABEL;
  switch (enumval)
  {
    case SystemVariables::IP:                label = LabelType::IP_ADDRESS; break;
    case SystemVariables::SUBNET:            label = LabelType::IP_SUBNET; break;
    case SystemVariables::DNS:               label = LabelType::DNS; break;
    case SystemVariables::DNS_1:             label = LabelType::DNS_1; break;
    case SystemVariables::DNS_2:             label = LabelType::DNS_2; break;
    case SystemVariables::GATEWAY:           label = LabelType::GATEWAY; break;
    case SystemVariables::CLIENTIP:          label = LabelType::CLIENT_IP; break;

    #if FEATURE_ETHERNET

    case SystemVariables::ETHWIFIMODE:       label = LabelType::ETH_WIFI_MODE; break; // 0=WIFI, 1=ETH
    case SystemVariables::ETHCONNECTED:      label = LabelType::ETH_CONNECTED; break; // 0=disconnected, 1=connected
    case SystemVariables::ETHDUPLEX:         label = LabelType::ETH_DUPLEX; break;
    case SystemVariables::ETHSPEED:          label = LabelType::ETH_SPEED; break;
    case SystemVariables::ETHSTATE:          label = LabelType::ETH_STATE; break;
    case SystemVariables::ETHSPEEDSTATE:     label = LabelType::ETH_SPEED_STATE; break;
    #endif // if FEATURE_ETHERNET
    case SystemVariables::LCLTIME:           label = LabelType::LOCAL_TIME; break;
    case SystemVariables::MAC:               label = LabelType::STA_MAC; break;
    case SystemVariables::RSSI:              label = LabelType::WIFI_RSSI; break;
    case SystemVariables::SUNRISE_S:         label = LabelType::SUNRISE_S; break;
    case SystemVariables::SUNSET_S:          label = LabelType::SUNSET_S; break;
    case SystemVariables::SUNRISE_M:         label = LabelType::SUNRISE_M; break;
    case SystemVariables::SUNSET_M:          label = LabelType::SUNSET_M; break;
    case SystemVariables::SYSBUILD_DESCR:    label = LabelType::BUILD_DESC; break;
    case SystemVariables::SYSBUILD_FILENAME: label = LabelType::BINARY_FILENAME; break;
    case SystemVariables::SYSBUILD_GIT:      label = LabelType::GIT_BUILD; break;
    case SystemVariables::SYSSTACK:          label = LabelType::FREE_STACK; break;
    case SystemVariables::UNIT_sysvar:       label = LabelType::UNIT_NR; break;
    case SystemVariables::FLASH_FREQ:        label = LabelType::FLASH_CHIP_SPEED; break;
    case SystemVariables::FLASH_SIZE:        label = LabelType::FLASH_CHIP_REAL_SIZE; break;
    case SystemVariables::FLASH_CHIP_VENDOR: label = LabelType::FLASH_CHIP_VENDOR; break;
    case SystemVariables::FLASH_CHIP_MODEL:  label = LabelType::FLASH_CHIP_MODEL; break;
    case SystemVariables::FS_SIZE:           label = LabelType::FS_SIZE; break;
    case SystemVariables::FS_FREE:           label = LabelType::FS_FREE; break;

    case SystemVariables::ESP_CHIP_ID:       label = LabelType::ESP_CHIP_ID; break;
    case SystemVariables::ESP_CHIP_FREQ:     label = LabelType::ESP_CHIP_FREQ; break;
    case SystemVariables::ESP_CHIP_MODEL:    label = LabelType::ESP_CHIP_MODEL; break;
    case SystemVariables::ESP_CHIP_REVISION: label = LabelType::ESP_CHIP_REVISION; break;
    case SystemVariables::ESP_CHIP_CORES:    label = LabelType::ESP_CHIP_CORES; break;
    case SystemVariables::ESP_BOARD_NAME:    label = LabelType::ESP_BOARD_NAME; break;

    default: 
      // No matching LabelType yet.
      break;
  }
  return label;
}


String SystemVariables::getSystemVariable(SystemVariables::Enum enumval) {
  const LabelType::Enum label = SystemVariables2LabelType(enumval);

  if (LabelType::MAX_LABEL != label) {
    return getValue(label);
  }

  switch (enumval)
  {
    case BOOT_CAUSE:        return String(lastBootCause);                         // Integer value to be used in rules
    case BSSID:             return String((WiFiEventData.WiFiDisconnected()) ? MAC_address().toString() : WiFi.BSSIDstr());
    case CR:                return String('\r');
    case IP4:               return String(static_cast<int>(NetworkLocalIP()[3])); // 4th IP octet
    #if FEATURE_MQTT
    case ISMQTT:            return String(MQTTclient_connected ? 1 : 0);
    #else // if FEATURE_MQTT
    case ISMQTT:            return String('0');
    #endif // if FEATURE_MQTT

    #ifdef USES_P037
    case ISMQTTIMP:         return String(P037_MQTTImport_connected ? 1 : 0);
    #else // ifdef USES_P037
    case ISMQTTIMP:         return String('0');
    #endif // USES_P037


    case ISNTP:             return String(statusNTPInitialized ? 1 : 0);
    case ISWIFI:            return String(WiFiEventData.wifiStatus); // 0=disconnected, 1=connected, 2=got ip, 4=services
    // initialized
    case LCLTIME_AM:        return node_time.getDateTimeString_ampm('-', ':', ' ');
    case LF:                return String('\n');
    case MAC_INT:           return String(getChipId()); // Last 24 bit of MAC address as integer, to be used in rules.
    case SPACE:             return String(' ');
    case SSID:              return (WiFiEventData.WiFiDisconnected()) ? F("--") : WiFi.SSID();
    case SYSBUILD_DATE:     return get_build_date();
    case SYSBUILD_TIME:     return get_build_time();
    case SYSDAY:            return String(node_time.day());
    case SYSDAY_0:          return timeReplacement_leadZero(node_time.day());
    case SYSHEAP:           return String(ESP.getFreeHeap());
    case SYSHOUR:           return String(node_time.hour());
    case SYSHOUR_0:         return timeReplacement_leadZero(node_time.hour());
    case SYSLOAD:           return String(getCPUload(), 2);
    case SYSMIN:            return String(node_time.minute());
    case SYSMIN_0:          return timeReplacement_leadZero(node_time.minute());
    case SYSMONTH:          return String(node_time.month());
    case SYSNAME:           return Settings.getHostname();
    case SYSSEC:            return String(node_time.second());
    case SYSSEC_0:          return timeReplacement_leadZero(node_time.second());
    case SYSSEC_D:          return String(((node_time.hour() * 60) + node_time.minute()) * 60 + node_time.second());
    case SYSTIME:           return node_time.getTimeString(':');
    case SYSTIME_AM:        return node_time.getTimeString_ampm(':');
    case SYSTIME_AM_0:      return node_time.getTimeString_ampm(':', true, '0');
    case SYSTIME_AM_SP:     return node_time.getTimeString_ampm(':', true, ' ');
    case SYSTM_HM:          return node_time.getTimeString(':', false);
    case SYSTM_HM_0:        return node_time.getTimeString(':', false, '0');
    case SYSTM_HM_SP:       return node_time.getTimeString(':', false, ' ');
    case SYSTM_HM_AM:       return node_time.getTimeString_ampm(':', false);
    case SYSTM_HM_AM_0:     return node_time.getTimeString_ampm(':', false, '0');
    case SYSTM_HM_AM_SP:    return node_time.getTimeString_ampm(':', false, ' ');
    case SYSWEEKDAY:        return String(node_time.weekday());
    case SYSWEEKDAY_S:      return node_time.weekday_str();
    case SYSYEAR_0:
    case SYSYEAR:           return String(node_time.year());
    case SYSYEARS:          return timeReplacement_leadZero(node_time.year() % 100);
    case SYS_MONTH_0:       return timeReplacement_leadZero(node_time.month());
    case S_CR:              return F("\\r");
    case S_LF:              return F("\\n");
    case UNIXDAY:           return String(node_time.getUnixTime() / 86400);
    case UNIXDAY_SEC:       return String(node_time.getUnixTime() % 86400);
    case UNIXTIME:          return String(node_time.getUnixTime());
    case UPTIME:            return String(getUptimeMinutes());
    case UPTIME_MS:         return ull2String(getMicros64() / 1000);
    #if FEATURE_ADC_VCC
    case VCC:               return String(vcc);
    #else // if FEATURE_ADC_VCC
    case VCC:               return String(-1);
    #endif // if FEATURE_ADC_VCC
    case WI_CH:             return String((WiFiEventData.WiFiDisconnected()) ? 0 : WiFi.channel());

    default:
      // Already handled above.
      return EMPTY_STRING;
  }
  return EMPTY_STRING;
}

#define SMART_REPL_T(T, S) \
  if (s.indexOf(T) != -1) { (S((T), s, useURLencode)); }

void SystemVariables::parseSystemVariables(String& s, boolean useURLencode)
{
  START_TIMER

  if (s.indexOf('%') == -1) {
    STOP_TIMER(PARSE_SYSVAR_NOCHANGE);
    return;
  }

  SystemVariables::Enum enumval = static_cast<SystemVariables::Enum>(0);

  do {
    enumval = SystemVariables::nextReplacementEnum(s, enumval);

    switch (enumval)
    {
      case SUNRISE:           SMART_REPL_T(SystemVariables::toString(enumval), replSunRiseTimeString); break;
      case SUNSET:            SMART_REPL_T(SystemVariables::toString(enumval), replSunSetTimeString); break;
      case UNKNOWN:

        // Do not replace
        break;
      default:
      {
        const String value = getSystemVariable(enumval);
        repl(SystemVariables::toString(enumval), value, s, useURLencode);
        break;
      }
    }
  }
  while (enumval != SystemVariables::Enum::UNKNOWN);

  int v_index = s.indexOf(F("%v"));

  while ((v_index != -1)) {
    unsigned int i;

    if (validUIntFromString(s.substring(v_index + 2), i)) {
      String key = F("%v");
      key += i;
      key += '%';

      if (s.indexOf(key) != -1) {
        const bool   trimTrailingZeros = true;
        const String value             = doubleToString(getCustomFloatVar(i), 6, trimTrailingZeros);
        repl(key, value, s, useURLencode);
      }
    }
    v_index = s.indexOf(F("%v"), v_index + 1); // Find next occurance
  }

  STOP_TIMER(PARSE_SYSVAR);
}

#undef SMART_REPL_T


SystemVariables::Enum SystemVariables::nextReplacementEnum(const String& str, SystemVariables::Enum last_tested)
{
  if (str.indexOf('%') == -1) {
    return Enum::UNKNOWN;
  }

  SystemVariables::Enum nextTested = static_cast<SystemVariables::Enum>(0);

  if (last_tested > nextTested) {
    nextTested = static_cast<SystemVariables::Enum>(last_tested + 1);
  }

  if (nextTested >= Enum::UNKNOWN) {
    return Enum::UNKNOWN;
  }

  String str_prefix        = String(SystemVariables::toString(nextTested)).substring(0, 2);
  bool   str_prefix_exists = str.indexOf(str_prefix) != -1;

  for (int i = nextTested; i < Enum::UNKNOWN; ++i) {
    SystemVariables::Enum enumval = static_cast<SystemVariables::Enum>(i);
    const String new_str_prefix   = String(SystemVariables::toString(enumval)).substring(0, 2);

    if ((str_prefix == new_str_prefix) && !str_prefix_exists) {
      // Just continue
    } else {
      str_prefix        = new_str_prefix;
      str_prefix_exists = str.indexOf(str_prefix) != -1;

      if (str_prefix_exists) {
        if (str.indexOf(SystemVariables::toString(enumval)) != -1) {
          return enumval;
        }
      }
    }
  }

  return Enum::UNKNOWN;
}

const __FlashStringHelper * SystemVariables::toString(SystemVariables::Enum enumval)
{
  switch (enumval) {
    case Enum::BOOT_CAUSE:         return F("%bootcause%");
    case Enum::BSSID:              return F("%bssid%");
    case Enum::CR:                 return F("%CR%");
    case Enum::IP4:                return F("%ip4%");
    case Enum::IP:                 return F("%ip%");
    case Enum::SUBNET:             return F("%subnet%");
    case Enum::DNS:                return F("%dns%");
    case Enum::DNS_1:              return F("%dns1%");
    case Enum::DNS_2:              return F("%dns2%");
    case Enum::GATEWAY:            return F("%gateway%");
    case Enum::CLIENTIP:           return F("%clientip%");
    case Enum::ISMQTT:             return F("%ismqtt%");
    case Enum::ISMQTTIMP:          return F("%ismqttimp%");
    case Enum::ISNTP:              return F("%isntp%");
    case Enum::ISWIFI:             return F("%iswifi%");
    #if FEATURE_ETHERNET
    case Enum::ETHWIFIMODE:        return F("%ethwifimode%");
    case Enum::ETHCONNECTED:       return F("%ethconnected%");
    case Enum::ETHDUPLEX:          return F("%ethduplex%");
    case Enum::ETHSPEED:           return F("%ethspeed%");
    case Enum::ETHSTATE:           return F("%ethstate%");
    case Enum::ETHSPEEDSTATE:      return F("%ethspeedstate%");
    #endif // if FEATURE_ETHERNET
    case Enum::LCLTIME:            return F("%lcltime%");
    case Enum::LCLTIME_AM:         return F("%lcltime_am%");
    case Enum::LF:                 return F("%LF%");
    case Enum::MAC:                return F("%mac%");
    case Enum::MAC_INT:            return F("%mac_int%");
    case Enum::RSSI:               return F("%rssi%");
    case Enum::SPACE:              return F("%SP%");
    case Enum::SSID:               return F("%ssid%");
    case Enum::SUNRISE:            return F("%sunrise");
    case Enum::SUNSET:             return F("%sunset");
    case Enum::SUNRISE_S:          return F("%s_sunrise%");
    case Enum::SUNSET_S:           return F("%s_sunset%");
    case Enum::SUNRISE_M:          return F("%m_sunrise%");
    case Enum::SUNSET_M:           return F("%m_sunset%");
    case Enum::SYSBUILD_DATE:      return F("%sysbuild_date%");
    case Enum::SYSBUILD_DESCR:     return F("%sysbuild_desc%");
    case Enum::SYSBUILD_FILENAME:  return F("%sysbuild_filename%");
    case Enum::SYSBUILD_GIT:       return F("%sysbuild_git%");
    case Enum::SYSBUILD_TIME:      return F("%sysbuild_time%");
    case Enum::SYSDAY:             return F("%sysday%");
    case Enum::SYSDAY_0:           return F("%sysday_0%");
    case Enum::SYSHEAP:            return F("%sysheap%");
    case Enum::SYSHOUR:            return F("%syshour%");
    case Enum::SYSHOUR_0:          return F("%syshour_0%");
    case Enum::SYSLOAD:            return F("%sysload%");
    case Enum::SYSMIN:             return F("%sysmin%");
    case Enum::SYSMIN_0:           return F("%sysmin_0%");
    case Enum::SYSMONTH:           return F("%sysmonth%");
    case Enum::SYSNAME:            return F("%sysname%");
    case Enum::SYSSEC:             return F("%syssec%");
    case Enum::SYSSEC_0:           return F("%syssec_0%");
    case Enum::SYSSEC_D:           return F("%syssec_d%");
    case Enum::SYSSTACK:           return F("%sysstack%");
    case Enum::SYSTIME:            return F("%systime%");
    case Enum::SYSTIME_AM:         return F("%systime_am%");
    case Enum::SYSTIME_AM_0:       return F("%systime_am_0%");
    case Enum::SYSTIME_AM_SP:      return F("%systime_am_sp%");
    case Enum::SYSTM_HM:           return F("%systm_hm%");
    case Enum::SYSTM_HM_0:         return F("%systm_hm_0%");
    case Enum::SYSTM_HM_SP:        return F("%systm_hm_sp%");
    case Enum::SYSTM_HM_AM:        return F("%systm_hm_am%");
    case Enum::SYSTM_HM_AM_0:      return F("%systm_hm_am_0%");
    case Enum::SYSTM_HM_AM_SP:     return F("%systm_hm_am_sp%");
    case Enum::SYSWEEKDAY:         return F("%sysweekday%");
    case Enum::SYSWEEKDAY_S:       return F("%sysweekday_s%");
    case Enum::SYSYEAR:            return F("%sysyear%");
    case Enum::SYSYEARS:           return F("%sysyears%");
    case Enum::SYSYEAR_0:          return F("%sysyear_0%");
    case Enum::SYS_MONTH_0:        return F("%sysmonth_0%");
    case Enum::S_CR:               return F("%R%");
    case Enum::S_LF:               return F("%N%");
    case Enum::UNIT_sysvar:        return F("%unit%");
    case Enum::UNIXDAY:            return F("%unixday%");
    case Enum::UNIXDAY_SEC:        return F("%unixday_sec%");
    case Enum::UNIXTIME:           return F("%unixtime%");
    case Enum::UPTIME:             return F("%uptime%");
    case Enum::UPTIME_MS:          return F("%uptime_ms%");
    case Enum::VCC:                return F("%vcc%");
    case Enum::WI_CH:              return F("%wi_ch%");
    case Enum::FLASH_FREQ:         return F("%flash_freq%");
    case Enum::FLASH_SIZE:         return F("%flash_size%");
    case Enum::FLASH_CHIP_VENDOR:  return F("%flash_chip_vendor%");
    case Enum::FLASH_CHIP_MODEL:   return F("%flash_chip_model%");
    case Enum::FS_FREE:            return F("%fs_free%");
    case Enum::FS_SIZE:            return F("%fs_size%");
    case Enum::ESP_CHIP_ID:        return F("%cpu_id%");
    case Enum::ESP_CHIP_FREQ:      return F("%cpu_freq%");
    case Enum::ESP_CHIP_MODEL:     return F("%cpu_model%");
    case Enum::ESP_CHIP_REVISION:  return F("%cpu_rev%");
    case Enum::ESP_CHIP_CORES:     return F("%cpu_cores%");
    case Enum::ESP_BOARD_NAME:     return F("%board_name%");

    case Enum::UNKNOWN: break;
  }
  return F("Unknown");
}

#include "../Helpers/WebServer_commandHelper.h"

#include "../../ESPEasy-Globals.h"
#include "../Commands/InternalCommands.h"
#include "../Globals/EventQueue.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringParser.h"
#include "../WebServer/AccessControl.h"


HandledWebCommand_result handle_command_from_web(EventValueSource::Enum source, String& webrequest)
{
  if (!clientIPallowed()) { return HandledWebCommand_result::IP_not_allowed; }
  webrequest.trim();
  if (webrequest.isEmpty()) { return HandledWebCommand_result::NoCommand; }

  addLog(LOG_LEVEL_INFO,  String(F("HTTP: ")) + webrequest);
  webrequest = parseTemplate(webrequest);
#ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_DEBUG, String(F("HTTP after parseTemplate: ")) + webrequest);
#endif // ifndef BUILD_NO_DEBUG

  bool handledCmd = false;
  bool sendOK     = false;
  printWebString = String();
  printToWeb     = false;
  printToWebJSON = false;

  // in case of event, store to buffer and return...
  String command = parseString(webrequest, 1);

  if ((command == F("event")) || (command == F("asyncevent")))
  {
    eventQueue.addMove(parseStringToEndKeepCase(webrequest, 2));
    handledCmd = true;
    sendOK     = true;
  } else if (command.equals(F("taskrun")) ||
             command.equals(F("taskvalueset")) ||
             command.equals(F("taskvaluesetandrun")) ||
             command.equals(F("taskvaluetoggle")) ||
             command.equals(F("let")) ||
             command.equals(F("logportstatus")) ||
             command.equals(F("jsonportstatus")) ||
             command.equals(F("rules"))) {
    printToWeb = true;
    handledCmd = ExecuteCommand_internal(source, webrequest.c_str());
    sendOK     = true;

    // handledCmd = true;
  } else {
    printToWeb     = true;
    handledCmd     = ExecuteCommand_all_config(source, webrequest.c_str());
    sendOK         = false;
  }

  if (handledCmd) {
    if (sendOK) {
      String reply = printWebString.isEmpty() ? F("OK") : printWebString;
      reply.replace(F("\n"), EMPTY_STRING); // Don't use newline in JSON.
      if (printToWebJSON) {
        // Format "OK" to JSON format
        printWebString = F("{\"return\": \"");
        printWebString += reply;
        printWebString += F("\",\"command\": \"");
        printWebString += webrequest;
        printWebString += F("\"}");
      } else {
        printWebString = reply;
      }
    }
    return HandledWebCommand_result::CommandHandled;
  }

  if (printToWebJSON) {
    // Format error to JSON format
    printWebString = F("{\"return\": \"");
    printWebString += F("Unknown or restricted command");
    printWebString += F("\",\"command\": \"");
    printWebString += webrequest;
    printWebString += F("\"}");
  }
  return HandledWebCommand_result::Unknown_or_restricted_command;
}

#include "../Helpers/I2C_access.h"

#include "../Globals/I2Cdev.h"
#include "../Helpers/ESPEasy_time_calc.h"

enum class I2C_clear_bus_state {
  Start,
  Wait_SCL_become_high,     // Wait for 2.5 seconds for SCL to become high after enabling pull-up resistors
  Wait_SDA_become_high,
  Wait_SCL_SDA_become_high, // SDA is low, try to toggle SCL and wait for it to be freed.
};


// Code to clear I2C bus as described here:
// http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
// Changed into a state machine for use in a non blocking way to be used in ESPEasy.
I2C_bus_state I2C_check_bus(int8_t scl, int8_t sda) {
  static I2C_clear_bus_state clearing_state = I2C_clear_bus_state::Start;
  static unsigned long last_state_change    = 0;
  static int clockCount                     = 20; // > 2x9 clock


  switch (clearing_state) {
    case I2C_clear_bus_state::Start:
    {
      // FIXME TD-er: Check for proper I2C pins
      if ((sda < 0) || (scl < 0)) { 
        last_state_change = 0;
        return I2C_bus_state::NotConfigured; 
      }

      if ((digitalRead(scl) == HIGH) && (digitalRead(sda) == HIGH)) { 
        last_state_change = 0;
        return I2C_bus_state::OK; 
      }

      pinMode(sda, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
      pinMode(scl, INPUT_PULLUP);

      clockCount        = 20;
      clearing_state    = I2C_clear_bus_state::Wait_SCL_become_high;
      last_state_change = millis();
      break;
    }

    case I2C_clear_bus_state::Wait_SCL_become_high:
    {
      // Wait 2.5 secs. This is strictly only necessary on the first power
      // up of the DS3231 module to allow it to initialize properly,
      // but is also assists in reliable programming of FioV3 boards as it gives the
      // IDE a chance to start uploaded the program
      // before existing sketch confuses the IDE by sending Serial data.
      if (digitalRead(scl) == LOW) {
        if (timePassedSince(last_state_change) > 2500) {
          clearing_state = I2C_clear_bus_state::Start;
          return I2C_bus_state::SCL_Low; // I2C bus error. Could not clear SCL clock line held low
        }
        return I2C_bus_state::ClearingProcessActive;
      }
      clearing_state    = I2C_clear_bus_state::Wait_SDA_become_high;
      last_state_change = millis();
      break;
    }

    case I2C_clear_bus_state::Wait_SDA_become_high:
    {
      boolean SDA_LOW = (digitalRead(sda) == LOW); // vi. Check SDA input.

      while (SDA_LOW && (clockCount > 0)) {        //  vii. If SDA is Low,
        clockCount--;

        // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
        pinMode(scl, INPUT);        // release SCL pullup so that when made output it will be LOW
        pinMode(scl, OUTPUT);       // then clock SCL Low
        delayMicroseconds(10);      //  for >5uS
        pinMode(scl, INPUT);        // release SCL LOW
        pinMode(scl, INPUT_PULLUP); // turn on pullup resistors again
        // do not force high as slave may be holding it low for clock stretching.
        delayMicroseconds(10);      //  for >5uS

        // The >5uS is so that even the slowest I2C devices are handled.
        if (digitalRead(scl) == LOW) {
          //  loop waiting for SCL to become High only wait 2sec.
          clearing_state    = I2C_clear_bus_state::Wait_SCL_SDA_become_high;
          last_state_change = millis();
          return I2C_bus_state::ClearingProcessActive;
        }
        SDA_LOW = (digitalRead(sda) == LOW);     //   and check SDA input again and loop
      }

      if (SDA_LOW) {                             // still low
        clearing_state = I2C_clear_bus_state::Start;
        return I2C_bus_state::SDA_Low_20_clocks; // I2C bus error. Could not clear. SDA data line held low
      }

      // else pull SDA line low for Start or Repeated Start
      pinMode(sda, INPUT);              // remove pullup.
      pinMode(sda, OUTPUT);             // and then make it LOW i.e. send an I2C Start or Repeated start control.
      // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
      // A Repeat Start is a Start occurring after a Start with no intervening Stop.
      delayMicroseconds(10);            // wait >5uS
      pinMode(sda, INPUT);              // remove output low
      pinMode(sda, INPUT_PULLUP);       // and make SDA high i.e. send I2C STOP control.
      delayMicroseconds(10);            // x. wait >5uS
      pinMode(sda, INPUT);              // and reset pins as tri-state inputs which is the default state on reset
      pinMode(scl, INPUT);
      clearing_state = I2C_clear_bus_state::Start;
      return I2C_bus_state::BusCleared; // all ok
    }

    case I2C_clear_bus_state::Wait_SCL_SDA_become_high:
    {
      if (digitalRead(scl) == LOW) {
        if (timePassedSince(last_state_change) > 2000) {
          // I2C bus error. Could not clear.
          // SCL clock line held low by slave clock stretch for >2sec
          clearing_state = I2C_clear_bus_state::Start;
          return I2C_bus_state::SDA_Low_over_2_sec;
        }
        return I2C_bus_state::ClearingProcessActive;
      }
      clearing_state    = I2C_clear_bus_state::Wait_SDA_become_high;
      last_state_change = millis();
      break;
    }
  }

  if (timePassedSince(last_state_change) > 5000) {
    // Just to prevent infinite loop
    // Should not be needed.
    clearing_state    = I2C_clear_bus_state::Start;
    last_state_change = millis();
  }
  return I2C_bus_state::ClearingProcessActive;
}

// **************************************************************************/
// Central functions for I2C data transfers
// **************************************************************************/
bool I2C_read_bytes(uint8_t i2caddr, I2Cdata_bytes& data) {
  const uint8_t size = data.getSize();

  return size == i2cdev.readBytes(i2caddr, data.getRegister(), size, data.get());
}

bool I2C_read_words(uint8_t i2caddr, I2Cdata_words& data) {
  const uint8_t size = data.getSize();

  return size == i2cdev.readWords(i2caddr, data.getRegister(), size, data.get());
}

// See https://github.com/platformio/platform-espressif32/issues/126
#ifdef ESP32

// ESP32: uint8_t TwoWire::endTransmission(bool sendStop)
  # define END_TRANSMISSION_FLAG true
#else // ifdef ESP32
// ESP8266: uint8_t TwoWire::endTransmission(uint8_t sendStop)
  # define END_TRANSMISSION_FLAG 0
#endif // ifdef ESP32

// **************************************************************************/
// Wake up I2C device
// **************************************************************************/
unsigned char I2C_wakeup(uint8_t i2caddr) {
  Wire.beginTransmission(i2caddr);
  return Wire.endTransmission();
}

// **************************************************************************/
// Writes an 8 bit value over I2C
// **************************************************************************/
bool I2C_write8(uint8_t i2caddr, uint8_t value) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)value);
  return Wire.endTransmission() == 0;
}

// **************************************************************************/
// Writes an 8 bit value over I2C to a register
// **************************************************************************/
bool I2C_write8_reg(uint8_t i2caddr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  return Wire.endTransmission() == 0;
}

// **************************************************************************/
// Writes an 16 bit value over I2C to a register
// **************************************************************************/
bool I2C_write16_reg(uint8_t i2caddr, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)value);
  return Wire.endTransmission() == 0;
}

// **************************************************************************/
// Writes an 16 bit value over I2C to a register
// **************************************************************************/
bool I2C_write16_LE_reg(uint8_t i2caddr, uint8_t reg, uint16_t value) {
  return I2C_write16_reg(i2caddr, reg, (value << 8) | (value >> 8));
}

// **************************************************************************/
// Reads an 8 bit value over I2C
// **************************************************************************/
uint8_t I2C_read8(uint8_t i2caddr, bool *is_ok) {
  uint8_t value;

  uint8_t count = Wire.requestFrom(i2caddr, (uint8_t)1);

  if (is_ok != nullptr) {
    *is_ok = (count == 1);
  }

  value = Wire.read();


  return value;
}

// **************************************************************************/
// Reads an 8 bit value from a register over I2C
// **************************************************************************/
uint8_t I2C_read8_reg(uint8_t i2caddr, uint8_t reg, bool *is_ok) {
  uint8_t value;

  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);

  if (Wire.endTransmission(END_TRANSMISSION_FLAG) != 0) {
    /*
       0:success
       1:data too long to fit in transmit buffer
       2:received NACK on transmit of address
       3:received NACK on transmit of data
       4:other error
       See https://www.arduino.cc/en/Reference/WireEndTransmission
     */
    if (is_ok != nullptr) {
      *is_ok = false;
    }
  }
  uint8_t count = Wire.requestFrom(i2caddr, (uint8_t)1);

  if (is_ok != nullptr) {
    *is_ok = (count == 1);
  }
  value = Wire.read();

  return value;
}

// **************************************************************************/
// Reads a 16 bit value starting at a given register over I2C
// **************************************************************************/
uint16_t I2C_read16_reg(uint8_t i2caddr, uint8_t reg) {
  uint16_t value(0);

  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(END_TRANSMISSION_FLAG);
  Wire.requestFrom(i2caddr, (uint8_t)2);
  value = (Wire.read() << 8) | Wire.read();

  return value;
}

// **************************************************************************/
// Reads a 24 bit value starting at a given register over I2C
// **************************************************************************/
int32_t I2C_read24_reg(uint8_t i2caddr, uint8_t reg) {
  int32_t value;

  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(END_TRANSMISSION_FLAG);
  Wire.requestFrom(i2caddr, (uint8_t)3);
  value = (((int32_t)Wire.read()) << 16) | (Wire.read() << 8) | Wire.read();

  return value;
}

// **************************************************************************/
// Reads a 32 bit value starting at a given register over I2C
// **************************************************************************/
int32_t I2C_read32_reg(uint8_t i2caddr, uint8_t reg) {
  int32_t value;

  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(END_TRANSMISSION_FLAG);
  Wire.requestFrom(i2caddr, (uint8_t)4);
  value = (((int32_t)Wire.read()) << 24) | (((uint32_t)Wire.read()) << 16) | (Wire.read() << 8) | Wire.read();

  return value;
}

// **************************************************************************/
// Reads a 16 bit value starting at a given register over I2C
// **************************************************************************/
uint16_t I2C_read16_LE_reg(uint8_t i2caddr, uint8_t reg) {
  uint16_t temp = I2C_read16_reg(i2caddr, reg);

  return (temp >> 8) | (temp << 8);
}

// **************************************************************************/
// Reads a signed 16 bit value starting at a given register over I2C
// **************************************************************************/
int16_t I2C_readS16_reg(uint8_t i2caddr, uint8_t reg) {
  return (int16_t)I2C_read16_reg(i2caddr, reg);
}

int16_t I2C_readS16_LE_reg(uint8_t i2caddr, uint8_t reg) {
  return (int16_t)I2C_read16_LE_reg(i2caddr, reg);
}

#undef END_TRANSMISSION_FLAG

#include "../Helpers/_CPlugin_Helper_webform.h"

#include "../../ESPEasy_common.h"

#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../DataTypes/ESPEasy_plugin_functions.h"
#include "../Globals/CPlugins.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/Settings.h"
#include "../Helpers/_CPlugin_Helper.h"
#include "../Helpers/Networking.h"
#include "../WebServer/WebServer.h"
#include "../WebServer/Markup.h"
#include "../WebServer/Markup_Forms.h"

/*********************************************************************************************\
* Functions to load and store controller settings on the web page.
\*********************************************************************************************/
const __FlashStringHelper * toString(ControllerSettingsStruct::VarType parameterIdx, bool displayName)
{
  switch (parameterIdx) {
    case ControllerSettingsStruct::CONTROLLER_USE_DNS:                  return  F("Locate Controller");      
    case ControllerSettingsStruct::CONTROLLER_HOSTNAME:                 return  F("Controller Hostname");    
    case ControllerSettingsStruct::CONTROLLER_IP:                       return  F("Controller IP");          
    case ControllerSettingsStruct::CONTROLLER_PORT:                     return  F("Controller Port");        
    case ControllerSettingsStruct::CONTROLLER_USER:                     return  F("Controller User");        
    case ControllerSettingsStruct::CONTROLLER_PASS:                     return  F("Controller Password");    

    case ControllerSettingsStruct::CONTROLLER_MIN_SEND_INTERVAL:        return  F("Minimum Send Interval");  
    case ControllerSettingsStruct::CONTROLLER_MAX_QUEUE_DEPTH:          return  F("Max Queue Depth");        
    case ControllerSettingsStruct::CONTROLLER_MAX_RETRIES:              return  F("Max Retries");            
    case ControllerSettingsStruct::CONTROLLER_FULL_QUEUE_ACTION:        return  F("Full Queue Action");      
    case ControllerSettingsStruct::CONTROLLER_ALLOW_EXPIRE:             return  F("Allow Expire");           
    case ControllerSettingsStruct::CONTROLLER_DEDUPLICATE:              return  F("De-duplicate");           
    case ControllerSettingsStruct::CONTROLLER_USE_LOCAL_SYSTEM_TIME:    return  F("Use Local System Time");
    
    case ControllerSettingsStruct::CONTROLLER_CHECK_REPLY:              return  F("Check Reply");            

    case ControllerSettingsStruct::CONTROLLER_CLIENT_ID:                return  F("Controller Client ID");   
    case ControllerSettingsStruct::CONTROLLER_UNIQUE_CLIENT_ID_RECONNECT: return  F("Unique Client ID on Reconnect");   
    case ControllerSettingsStruct::CONTROLLER_RETAINFLAG:               return  F("Publish Retain Flag");    
    case ControllerSettingsStruct::CONTROLLER_SUBSCRIBE:                return  F("Controller Subscribe");   
    case ControllerSettingsStruct::CONTROLLER_PUBLISH:                  return  F("Controller Publish");     
    case ControllerSettingsStruct::CONTROLLER_LWT_TOPIC:                return  F("Controller LWT Topic");   
    case ControllerSettingsStruct::CONTROLLER_LWT_CONNECT_MESSAGE:      return  F("LWT Connect Message");    
    case ControllerSettingsStruct::CONTROLLER_LWT_DISCONNECT_MESSAGE:   return  F("LWT Disconnect Message"); 
    case ControllerSettingsStruct::CONTROLLER_SEND_LWT:                 return  F("Send LWT to broker");     
    case ControllerSettingsStruct::CONTROLLER_WILL_RETAIN:              return  F("Will Retain");            
    case ControllerSettingsStruct::CONTROLLER_CLEAN_SESSION:            return  F("Clean Session");          
    case ControllerSettingsStruct::CONTROLLER_USE_EXTENDED_CREDENTIALS: return  F("Use Extended Credentials");  
    case ControllerSettingsStruct::CONTROLLER_SEND_BINARY:              return  F("Send Binary");            
    case ControllerSettingsStruct::CONTROLLER_TIMEOUT:                  return  F("Client Timeout");         
    case ControllerSettingsStruct::CONTROLLER_SAMPLE_SET_INITIATOR:     return  F("Sample Set Initiator");   

    case ControllerSettingsStruct::CONTROLLER_ENABLED:

      if (displayName) { return  F("Enabled"); }
      else {             return  F("controllerenabled"); }
      

    default:
      return  F("Undefined");
  }
}

String getControllerParameterName(protocolIndex_t                   ProtocolIndex,
                                  ControllerSettingsStruct::VarType parameterIdx,
                                  bool                              displayName,
                                  bool                            & isAlternative) {
  String name;

  if (displayName) {
    EventStruct tmpEvent;
    tmpEvent.idx = parameterIdx;

    if (CPluginCall(ProtocolIndex, CPlugin::Function::CPLUGIN_GET_PROTOCOL_DISPLAY_NAME, &tmpEvent, name)) {
      // Found an alternative name for it.
      isAlternative = true;
      return name;
    }
  }
  isAlternative = false;

  name = toString(parameterIdx, displayName);

  if (!displayName) {
    // Change name to lower case and remove spaces to make it an internal name.
    name.toLowerCase();
    name.replace(F(" "), EMPTY_STRING);
  }
  return name;
}

String getControllerParameterInternalName(protocolIndex_t ProtocolIndex, ControllerSettingsStruct::VarType parameterIdx) {
  bool isAlternative; // Dummy, not needed for internal name
  bool displayName = false;

  return getControllerParameterName(ProtocolIndex, parameterIdx, displayName, isAlternative);
}

String getControllerParameterDisplayName(protocolIndex_t ProtocolIndex, ControllerSettingsStruct::VarType parameterIdx, bool& isAlternative) {
  bool displayName = true;

  return getControllerParameterName(ProtocolIndex, parameterIdx, displayName, isAlternative);
}

void addControllerEnabledForm(controllerIndex_t controllerindex) {
  protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(controllerindex);

  if (!validProtocolIndex(ProtocolIndex)) {
    return;
  }

  ControllerSettingsStruct::VarType varType = ControllerSettingsStruct::CONTROLLER_ENABLED;

  bool   isAlternativeDisplayName = false;
  const String displayName        = getControllerParameterDisplayName(ProtocolIndex, varType, isAlternativeDisplayName);
  const String internalName       = getControllerParameterInternalName(ProtocolIndex, varType);
  addFormCheckBox(displayName, internalName, Settings.ControllerEnabled[controllerindex]);
}

void addControllerParameterForm(const ControllerSettingsStruct& ControllerSettings, controllerIndex_t controllerindex, ControllerSettingsStruct::VarType varType) {
  protocolIndex_t  ProtocolIndex  = getProtocolIndex_from_ControllerIndex(controllerindex);
  if (!validProtocolIndex(ProtocolIndex)) {
    return;
  }

  bool   isAlternativeDisplayName = false;
  const String displayName        = getControllerParameterDisplayName(ProtocolIndex, varType, isAlternativeDisplayName);
  const String internalName       = getControllerParameterInternalName(ProtocolIndex, varType);

  switch (varType) {
    case ControllerSettingsStruct::CONTROLLER_USE_DNS:
    {
      uint8_t   choice = ControllerSettings.UseDNS;
      const __FlashStringHelper * options[2] = {
        F("Use IP address"),
        F("Use Hostname")
      };
      addFormSelector(displayName, internalName, 2, options, nullptr, nullptr, choice, true);
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_HOSTNAME:
    {
      addFormTextBox(displayName, internalName, ControllerSettings.HostName, sizeof(ControllerSettings.HostName) - 1);
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_IP:
    {
      addFormIPBox(displayName, internalName, ControllerSettings.IP);
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_PORT:
    {
      addFormNumericBox(displayName, internalName, ControllerSettings.Port, 1, 65535);
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_USER:
    {
      const size_t fieldMaxLength =
        ControllerSettings.useExtendedCredentials() ? EXT_SECURITY_MAX_USER_LENGTH : sizeof(SecuritySettings.ControllerUser[0]) - 1;
      addFormTextBox(displayName,
                     internalName,
                     getControllerUser(controllerindex, ControllerSettings),
                     fieldMaxLength);
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_PASS:
    {
      const size_t fieldMaxLength = ControllerSettings.useExtendedCredentials() ? EXT_SECURITY_MAX_PASS_LENGTH : sizeof(SecuritySettings.ControllerPassword[0]) - 1;
      if (isAlternativeDisplayName) {
        // It is not a regular password, thus use normal text field.
        addFormTextBox(displayName, internalName, 
                       getControllerPass(controllerindex, ControllerSettings),
                       fieldMaxLength);
      } else {
        addFormPasswordBox(displayName, internalName,
                           getControllerPass(controllerindex, ControllerSettings),
                           fieldMaxLength);
      }
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_MIN_SEND_INTERVAL:
    {
      addFormNumericBox(displayName, internalName, ControllerSettings.MinimalTimeBetweenMessages, 1, CONTROLLER_DELAY_QUEUE_DELAY_MAX);
      addUnit(F("ms"));
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_MAX_QUEUE_DEPTH:
    {
      addFormNumericBox(displayName, internalName, ControllerSettings.MaxQueueDepth, 1, CONTROLLER_DELAY_QUEUE_DEPTH_MAX);
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_MAX_RETRIES:
    {
      addFormNumericBox(displayName, internalName, ControllerSettings.MaxRetry, 1, CONTROLLER_DELAY_QUEUE_RETRY_MAX);
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_FULL_QUEUE_ACTION:
    {
      const __FlashStringHelper * options[2] {
        F("Ignore New"),
        F("Delete Oldest")
      };
      addFormSelector(displayName, internalName, 2, options, nullptr, nullptr, ControllerSettings.DeleteOldest, false);
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_ALLOW_EXPIRE:
      addFormCheckBox(displayName, internalName, ControllerSettings.allowExpire());
      break;
    case ControllerSettingsStruct::CONTROLLER_DEDUPLICATE:
      addFormCheckBox(displayName, internalName, ControllerSettings.deduplicate());
      break;
    case ControllerSettingsStruct::CONTROLLER_USE_LOCAL_SYSTEM_TIME:
      addFormCheckBox(displayName, internalName, ControllerSettings.useLocalSystemTime());
      break;      
    case ControllerSettingsStruct::CONTROLLER_CHECK_REPLY:
    {
      const __FlashStringHelper * options[2] = {
        F("Ignore Acknowledgement"),
        F("Check Acknowledgement")
      };
      addFormSelector(displayName, internalName, 2, options, nullptr, nullptr, ControllerSettings.MustCheckReply, false);
      break;
    }
    case ControllerSettingsStruct::CONTROLLER_CLIENT_ID:
      addFormTextBox(displayName, internalName, ControllerSettings.ClientID, sizeof(ControllerSettings.ClientID) - 1);
      break;
    case ControllerSettingsStruct::CONTROLLER_UNIQUE_CLIENT_ID_RECONNECT:
      addFormCheckBox(displayName, internalName, ControllerSettings.mqtt_uniqueMQTTclientIdReconnect());
      break;
    case ControllerSettingsStruct::CONTROLLER_RETAINFLAG:
      addFormCheckBox(displayName, internalName, ControllerSettings.mqtt_retainFlag());
      break;
    case ControllerSettingsStruct::CONTROLLER_SUBSCRIBE:
      addFormTextBox(displayName, internalName, ControllerSettings.Subscribe,            sizeof(ControllerSettings.Subscribe) - 1);
      break;
    case ControllerSettingsStruct::CONTROLLER_PUBLISH:
      addFormTextBox(displayName, internalName, ControllerSettings.Publish,              sizeof(ControllerSettings.Publish) - 1);
      break;
    case ControllerSettingsStruct::CONTROLLER_LWT_TOPIC:
      addFormTextBox(displayName, internalName, ControllerSettings.MQTTLwtTopic,         sizeof(ControllerSettings.MQTTLwtTopic) - 1);
      break;
    case ControllerSettingsStruct::CONTROLLER_LWT_CONNECT_MESSAGE:
      addFormTextBox(displayName, internalName, ControllerSettings.LWTMessageConnect,    sizeof(ControllerSettings.LWTMessageConnect) - 1);
      break;
    case ControllerSettingsStruct::CONTROLLER_LWT_DISCONNECT_MESSAGE:
      addFormTextBox(displayName, internalName, ControllerSettings.LWTMessageDisconnect, sizeof(ControllerSettings.LWTMessageDisconnect) - 1);
      break;
    case ControllerSettingsStruct::CONTROLLER_SEND_LWT:
      addFormCheckBox(displayName, internalName, ControllerSettings.mqtt_sendLWT());
      break;
    case ControllerSettingsStruct::CONTROLLER_WILL_RETAIN:
      addFormCheckBox(displayName, internalName, ControllerSettings.mqtt_willRetain());
      break;
    case ControllerSettingsStruct::CONTROLLER_CLEAN_SESSION:
      addFormCheckBox(displayName, internalName, ControllerSettings.mqtt_cleanSession());
      break;
    case ControllerSettingsStruct::CONTROLLER_USE_EXTENDED_CREDENTIALS:
      addFormCheckBox(displayName, internalName, ControllerSettings.useExtendedCredentials());
      break;
    case ControllerSettingsStruct::CONTROLLER_SEND_BINARY:
      addFormCheckBox(displayName, internalName, ControllerSettings.sendBinary());
      break;
    case ControllerSettingsStruct::CONTROLLER_TIMEOUT:
      addFormNumericBox(displayName, internalName, ControllerSettings.ClientTimeout, 10, CONTROLLER_CLIENTTIMEOUT_MAX);
      addUnit(F("ms"));
      break;
    case ControllerSettingsStruct::CONTROLLER_SAMPLE_SET_INITIATOR:
      addTaskSelectBox(displayName, internalName, ControllerSettings.SampleSetInitiator);
      break;
    case ControllerSettingsStruct::CONTROLLER_ENABLED:
      addFormCheckBox(displayName, internalName, Settings.ControllerEnabled[controllerindex]);
      break;
  }
}

void saveControllerParameterForm(ControllerSettingsStruct        & ControllerSettings,
                                 controllerIndex_t                 controllerindex,
                                 ControllerSettingsStruct::VarType varType) {
  protocolIndex_t ProtocolIndex = getProtocolIndex_from_ControllerIndex(controllerindex);

  if (!validProtocolIndex(ProtocolIndex)) {
    return;
  }
  String internalName = getControllerParameterInternalName(ProtocolIndex, varType);

  switch (varType) {
    case ControllerSettingsStruct::CONTROLLER_USE_DNS:  ControllerSettings.UseDNS = getFormItemInt(internalName); break;
    case ControllerSettingsStruct::CONTROLLER_HOSTNAME:

      if (ControllerSettings.UseDNS)
      {
        strncpy_webserver_arg(ControllerSettings.HostName, internalName);
        IPAddress IP;
        resolveHostByName(ControllerSettings.HostName, IP, ControllerSettings.ClientTimeout);

        for (uint8_t x = 0; x < 4; x++) {
          ControllerSettings.IP[x] = IP[x];
        }
      }
      break;
    case ControllerSettingsStruct::CONTROLLER_IP:

      if (!ControllerSettings.UseDNS)
      {
        String controllerip = webArg(internalName);
        str2ip(controllerip, ControllerSettings.IP);
      }
      break;
    case ControllerSettingsStruct::CONTROLLER_PORT:
      ControllerSettings.Port = getFormItemInt(internalName, ControllerSettings.Port);
      break;
    case ControllerSettingsStruct::CONTROLLER_USER:
      setControllerUser(controllerindex, ControllerSettings, webArg(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_PASS:
    {
      String password;

      if (getFormPassword(internalName, password)) {
        setControllerPass(controllerindex, ControllerSettings, password);
      }
      break;
    }

    case ControllerSettingsStruct::CONTROLLER_MIN_SEND_INTERVAL:
      ControllerSettings.MinimalTimeBetweenMessages = getFormItemInt(internalName, ControllerSettings.MinimalTimeBetweenMessages);
      break;
    case ControllerSettingsStruct::CONTROLLER_MAX_QUEUE_DEPTH:
      ControllerSettings.MaxQueueDepth = getFormItemInt(internalName, ControllerSettings.MaxQueueDepth);
      break;
    case ControllerSettingsStruct::CONTROLLER_MAX_RETRIES:
      ControllerSettings.MaxRetry = getFormItemInt(internalName, ControllerSettings.MaxRetry);
      break;
    case ControllerSettingsStruct::CONTROLLER_FULL_QUEUE_ACTION:
      ControllerSettings.DeleteOldest = getFormItemInt(internalName, ControllerSettings.DeleteOldest);
      break;
    case ControllerSettingsStruct::CONTROLLER_ALLOW_EXPIRE:
      ControllerSettings.allowExpire(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_DEDUPLICATE:
      ControllerSettings.deduplicate(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_USE_LOCAL_SYSTEM_TIME:
      ControllerSettings.useLocalSystemTime(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_CHECK_REPLY:
      ControllerSettings.MustCheckReply = getFormItemInt(internalName, ControllerSettings.MustCheckReply);
      break;
    case ControllerSettingsStruct::CONTROLLER_CLIENT_ID:
      strncpy_webserver_arg(ControllerSettings.ClientID, internalName);
      break;
    case ControllerSettingsStruct::CONTROLLER_UNIQUE_CLIENT_ID_RECONNECT:
      ControllerSettings.mqtt_uniqueMQTTclientIdReconnect(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_RETAINFLAG:
      ControllerSettings.mqtt_retainFlag(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_SUBSCRIBE:
      strncpy_webserver_arg(ControllerSettings.Subscribe,            internalName);
      break;
    case ControllerSettingsStruct::CONTROLLER_PUBLISH:
      strncpy_webserver_arg(ControllerSettings.Publish,              internalName);
      break;
    case ControllerSettingsStruct::CONTROLLER_LWT_TOPIC:
      strncpy_webserver_arg(ControllerSettings.MQTTLwtTopic,         internalName);
      break;
    case ControllerSettingsStruct::CONTROLLER_LWT_CONNECT_MESSAGE:
      strncpy_webserver_arg(ControllerSettings.LWTMessageConnect,    internalName);
      break;
    case ControllerSettingsStruct::CONTROLLER_LWT_DISCONNECT_MESSAGE:
      strncpy_webserver_arg(ControllerSettings.LWTMessageDisconnect, internalName);
      break;
    case ControllerSettingsStruct::CONTROLLER_SEND_LWT:
      ControllerSettings.mqtt_sendLWT(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_WILL_RETAIN:
      ControllerSettings.mqtt_willRetain(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_CLEAN_SESSION:
      ControllerSettings.mqtt_cleanSession(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_USE_EXTENDED_CREDENTIALS:
      ControllerSettings.useExtendedCredentials(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_SEND_BINARY:
      ControllerSettings.sendBinary(isFormItemChecked(internalName));
      break;
    case ControllerSettingsStruct::CONTROLLER_TIMEOUT:
      ControllerSettings.ClientTimeout = getFormItemInt(internalName, ControllerSettings.ClientTimeout);
      break;
    case ControllerSettingsStruct::CONTROLLER_SAMPLE_SET_INITIATOR:
      ControllerSettings.SampleSetInitiator = getFormItemInt(internalName, ControllerSettings.SampleSetInitiator);
      break;
    case ControllerSettingsStruct::CONTROLLER_ENABLED:
      Settings.ControllerEnabled[controllerindex] = isFormItemChecked(internalName);
      break;
  }
}

#include "../Helpers/Modbus_RTU.h"


#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/StringConverter.h"


ModbusRTU_struct::ModbusRTU_struct() : easySerial(nullptr) {
  reset();
}

ModbusRTU_struct::~ModbusRTU_struct() {
  reset();
}

void ModbusRTU_struct::reset() {
  if (easySerial != nullptr) {
    delete easySerial;
    easySerial = nullptr;
  }
  detected_device_description = String();

  for (int i = 0; i < 8; ++i) {
    _sendframe[i] = 0;
  }
  _sendframe_used = 0;

  for (int i = 0; i < MODBUS_RECEIVE_BUFFER; ++i) {
    _recv_buf[i] = 0xff;
  }
  _recv_buf_used    = 0;
  _modbus_address   = MODBUS_BROADCAST_ADDRESS;
  _reads_pass       = 0;
  _reads_crc_failed = 0;
  _reads_nodata     = 0;
}

bool ModbusRTU_struct::init(const ESPEasySerialPort port, const int16_t serial_rx, const int16_t serial_tx, int16_t baudrate, uint8_t address) {
  return init(port, serial_rx, serial_tx, baudrate, address, -1);
}

bool ModbusRTU_struct::init(const ESPEasySerialPort port, const int16_t serial_rx, const int16_t serial_tx, int16_t baudrate, uint8_t address, int8_t dere_pin) {
  if ((serial_rx < 0) || (serial_tx < 0)) {
    return false;
  }
  reset();
  easySerial = new (std::nothrow) ESPeasySerial(port, serial_rx, serial_tx);
  if (easySerial == nullptr) { return false; }
  easySerial->begin(baudrate);

  if (!isInitialized()) { return false; }
  _modbus_address = address;
  _dere_pin       = dere_pin;

  if (_dere_pin != -1) { // set output pin mode for DE/RE pin when used (for control MAX485)
    pinMode(_dere_pin, OUTPUT);
  }

  detected_device_description = getDevice_description(_modbus_address);

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log; // = F("Modbus detected: ");
    log += detected_device_description;
    addLogMove(LOG_LEVEL_INFO, log);
    modbus_log_MEI(_modbus_address);
  }
  return true;
}

bool ModbusRTU_struct::isInitialized() const {
  return easySerial != nullptr;
}

void ModbusRTU_struct::getStatistics(uint32_t& pass, uint32_t& fail, uint32_t& nodata) const {
  pass   = _reads_pass;
  fail   = _reads_crc_failed;
  nodata = _reads_nodata;
}

void ModbusRTU_struct::setModbusTimeout(uint16_t timeout) {
  _modbus_timeout = timeout;
}

uint16_t ModbusRTU_struct::getModbusTimeout() const {
  return _modbus_timeout;
}

String ModbusRTU_struct::getDevice_description(uint8_t slaveAddress) {
  bool more_follows     = true;
  uint8_t next_object_id   = 0;
  uint8_t conformity_level = 0;
  unsigned int object_value_int;
  String description;
  String obj_text;

  for (uint8_t object_id = 0; object_id < 0x84; ++object_id) {
    if (object_id == 6) {
      object_id = 0x82; // Skip to the serialnr/sensor type
    }
    int result = modbus_get_MEI(slaveAddress, object_id, obj_text,
                                object_value_int, next_object_id,
                                more_follows, conformity_level);
    String label;

    switch (object_id) {
      case 0x01:

        if (result == 0) { label = F("Pcode"); }
        break;
      case 0x02:

        if (result == 0) { label = F("Rev"); }
        break;
      case 0x82:
      {
        if (result != 0) {
          uint32_t sensorId = readSensorId();
          obj_text = String(sensorId, HEX);
          result   = 0;
        }

        label = F("S/N");
        break;
      }
      case 0x83:
      {
        if (result != 0) {
          uint32_t sensorId = readTypeId();
          obj_text = String(sensorId, HEX);
          result   = 0;
        }

        label = F("Type");
        break;
      }
      default:
        break;
    }

    if (result == 0) {
      if (label.length() > 0) {
        // description += MEI_objectid_to_name(object_id);
        description += label;
        description += F(": ");
      }

      if (obj_text.length() > 0) {
        description += obj_text;
        description += F(" - ");
      }
    }
  }
  return description;
}

// Read from RAM or EEPROM
void ModbusRTU_struct::buildRead_RAM_EEPROM(uint8_t slaveAddress, uint8_t functionCode,
                                            short startAddress, uint8_t number_bytes) {
  _sendframe[0]   = slaveAddress;
  _sendframe[1]   = functionCode;
  _sendframe[2]   = (uint8_t)(startAddress >> 8);
  _sendframe[3]   = (uint8_t)(startAddress & 0xFF);
  _sendframe[4]   = number_bytes;
  _sendframe_used = 5;
}

// Write to the Special Control Register (SCR)
void ModbusRTU_struct::buildWriteCommandRegister(uint8_t slaveAddress, uint8_t value) {
  _sendframe[0]   = slaveAddress;
  _sendframe[1]   = MODBUS_CMD_WRITE_RAM;
  _sendframe[2]   = 0;    // Address-Hi SCR  (0x0060)
  _sendframe[3]   = 0x60; // Address-Lo SCR
  _sendframe[4]   = 1;    // Count
  _sendframe[5]   = value;
  _sendframe_used = 6;
}

void ModbusRTU_struct::buildWriteMult16bRegister(uint8_t slaveAddress, uint16_t startAddress, uint16_t value) {
  _sendframe[0]   = slaveAddress;
  _sendframe[1]   = MODBUS_WRITE_MULTIPLE_REGISTERS;
  _sendframe[2]   = (uint8_t)(startAddress >> 8);
  _sendframe[3]   = (uint8_t)(startAddress & 0xFF);
  _sendframe[4]   = 0; // nr reg hi
  _sendframe[5]   = 1; // nr reg lo
  _sendframe[6]   = 2; // nr bytes to follow (2 bytes per register)
  _sendframe[7]   = (uint8_t)(value >> 8);
  _sendframe[8]   = (uint8_t)(value & 0xFF);
  _sendframe_used = 9;
}

void ModbusRTU_struct::buildFrame(uint8_t slaveAddress, uint8_t functionCode,
                                  short startAddress, short parameter) {
  _sendframe[0]   = slaveAddress;
  _sendframe[1]   = functionCode;
  _sendframe[2]   = (uint8_t)(startAddress >> 8);
  _sendframe[3]   = (uint8_t)(startAddress & 0xFF);
  _sendframe[4]   = (uint8_t)(parameter >> 8);
  _sendframe[5]   = (uint8_t)(parameter & 0xFF);
  _sendframe_used = 6;
}

void ModbusRTU_struct::build_modbus_MEI_frame(uint8_t slaveAddress, uint8_t device_id,
                                              uint8_t object_id) {
  _sendframe[0] = slaveAddress;
  _sendframe[1] = 0x2B;
  _sendframe[2] = 0x0E;

  // The parameter "Read Device ID code" allows to define four access types :
  // 01: request to get the basic device identification (stream access)
  // 02: request to get the regular device identification (stream access)
  // 03: request to get the extended device identification (stream access)
  // 04: request to get one specific identification object (individual access)
  _sendframe[3]   = device_id;
  _sendframe[4]   = object_id;
  _sendframe_used = 5;
}

String ModbusRTU_struct::MEI_objectid_to_name(uint8_t object_id) {
  String result;

  switch (object_id) {
    case 0:    result = F("VendorName");          break;
    case 1:    result = F("ProductCode");         break;
    case 2:    result = F("MajorMinorRevision");  break;
    case 3:    result = F("VendorUrl");           break;
    case 4:    result = F("ProductName");         break;
    case 5:    result = F("ModelName");           break;
    case 6:    result = F("UserApplicationName"); break;
    case 0x80: result = F("MemoryMapVersion");    break;
    case 0x81: result = F("Firmware Rev.");       break;
    case 0x82: result = F("Sensor S/N");          break;
    case 0x83: result = F("Sensor type");         break;
    default:
      result = formatToHex(object_id);
      break;
  }
  return result;
}

String ModbusRTU_struct::parse_modbus_MEI_response(unsigned int& object_value_int,
                                                   uint8_t        & next_object_id,
                                                   bool        & more_follows,
                                                   uint8_t        & conformity_level) {
  String result;

  if (_recv_buf_used < 8) {
    // Too small.
    addLog(LOG_LEVEL_INFO,
           String(F("MEI response too small: ")) + _recv_buf_used);
    next_object_id = 0xFF;
    more_follows   = false;
    return result;
  }
  int pos = 4; // Data skipped: slave_address, FunctionCode, MEI type, ReadDevId
  // See http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
  // Page 45
  conformity_level = _recv_buf[pos++];
  more_follows     = _recv_buf[pos++] != 0;
  next_object_id   = _recv_buf[pos++];
  const uint8_t number_objects = _recv_buf[pos++];
  uint8_t object_id            = 0;

  for (int i = 0; i < number_objects; ++i) {
    if ((pos + 3) < _recv_buf_used) {
      object_id = _recv_buf[pos++];
      const uint8_t object_length = _recv_buf[pos++];

      if ((pos + object_length) < _recv_buf_used) {
        String object_value;

        if (object_id < 0x80) {
          // Parse as type String
          object_value.reserve(object_length);
          object_value_int = static_cast<unsigned int>(-1);

          for (int c = 0; c < object_length; ++c) {
            object_value += char(_recv_buf[pos++]);
          }
        } else {
          object_value.reserve(2 * object_length + 2);
          object_value_int = 0;

          for (int c = 0; c < object_length; ++c) {
            object_value_int =
              object_value_int << 8 | _recv_buf[pos++];
          }
          object_value += formatToHex(object_value_int);
        }

        if (i != 0) {
          // Append to existing description
          result += F(", ");
        }
        result += object_value;
      }
    }
  }
  return result;
}

void ModbusRTU_struct::logModbusException(uint8_t value) {
  if (value == 0) {
    return;
  }

  /*
     // Exception Response, see:
     // http://digital.ni.com/public.nsf/allkb/E40CA0CFA0029B2286256A9900758E06?OpenDocument
     String log = F("Modbus Exception - ");
     switch (value) {
     case MODBUS_EXCEPTION_ILLEGAL_FUNCTION: {
      // The function code received in the query is not an allowable action for
      // the slave.
      // If a Poll Program Complete command was issued, this code indicates that
      // no program function preceded it.
      log += F("Illegal Function (not allowed by client)");
      break;
     }
     case MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS: {
      // The data address received in the query is not an allowable address for
      // the slave.
      log += F("Illegal Data Address");
      break;
     }
     case MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE: {
      // A value contained in the query data field is not an allowable value for
      // the slave
      log += F("Illegal Data Value");
      break;
     }
     case MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE: {
      // An unrecoverable error occurred while the slave was attempting to perform
      // the requested action
      log += F("Slave Device Failure");
      break;
     }
     case MODBUS_EXCEPTION_ACKNOWLEDGE: {
      // The slave has accepted the request and is processing it, but a long
      // duration of time will be
      // required to do so. This response is returned to prevent a timeout error
      // from occurring in the master.
      // The master can next issue a Poll Program Complete message to determine if
      // processing is completed.
      log += F("Acknowledge");
      break; // Is this an error?
     }
     case MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY: {
      // The slave is engaged in processing a long-duration program command.
      // The master should retransmit the message later when the slave is free.
      log += F("Slave Device Busy");
      break;
     }
     case MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE:
      log += F("Negative acknowledge");
      break;
     case MODBUS_EXCEPTION_MEMORY_PARITY:
      log += F("Memory parity error");
      break;
     case MODBUS_EXCEPTION_GATEWAY_PATH:
      log += F("Gateway path unavailable");
      break;
     case MODBUS_EXCEPTION_GATEWAY_TARGET:
      log += F("Target device failed to respond");
      break;
     case MODBUS_BADCRC:
      log += F("Invalid CRC");
      break;
     case MODBUS_BADDATA:
      log += F("Invalid data");
      break;
     case MODBUS_BADEXC:
      log += F("Invalid exception code");
      break;
     case MODBUS_MDATA:
      log += F("Too many data");
      break;
     case MODBUS_BADSLAVE:
      log += F("Response not from requested slave");
      break;
     case MODBUS_TIMEOUT:
      log += F("Modbus Timeout");
      break;
     case MODBUS_NODATA:
      log += F("Modbus No Data");
      break;
     default:
      log += String(F("Unknown Exception code: ")) + value;
      break;
     }
     log += F(" - sent: ");
     log += log_buffer(_sendframe, _sendframe_used);
     log += F(" - received: ");
     log += log_buffer(_recv_buf, _recv_buf_used);
     addLog(LOG_LEVEL_DEBUG_MORE, log);
   */
}

/*
   String log_buffer(uint8_t *buffer, int length) {
    String log;
    log.reserve(3 * length + 5);
    for (int i = 0; i < length; ++i) {
      String hexvalue(buffer[i], HEX);
      hexvalue.toUpperCase();
      log += hexvalue;
      log += ' ';
    }
    log += '(';
    log += length;
    log += ')';
    return log;
   }
 */
uint8_t ModbusRTU_struct::processCommand() {
  // CRC-calculation
  unsigned int crc =
    ModRTU_CRC(_sendframe, _sendframe_used);

  // Note, this number has low and high bytes swapped, so use it accordingly (or
  // swap bytes)
  uint8_t checksumHi = (uint8_t)((crc >> 8) & 0xFF);
  uint8_t checksumLo = (uint8_t)(crc & 0xFF);

  _sendframe[_sendframe_used++] = checksumLo;
  _sendframe[_sendframe_used++] = checksumHi;

  int  nrRetriesLeft = 2;
  uint8_t return_value  = 0;

  while (nrRetriesLeft > 0) {
    return_value = 0;

    // Send the uint8_t array
    startWrite();
    easySerial->write(_sendframe, _sendframe_used);

    // sent all data from buffer
    easySerial->flush();
    startRead();

    // Read answer from sensor
    _recv_buf_used = 0;
    unsigned long timeout    = millis() + _modbus_timeout;
    bool validPacket         = false;
    bool invalidDueToTimeout = false;

    //  idx:    0,   1,   2,   3,   4,   5,   6,   7
    // send: 0x02,0x03,0x00,0x00,0x00,0x01,0x39,0x84
    // recv: 0x02,0x03,0x02,0x01,0x57,0xBC,0x2A

    while (!validPacket && !invalidDueToTimeout && _recv_buf_used < MODBUS_RECEIVE_BUFFER) {
      if (timeOutReached(timeout)) {
        invalidDueToTimeout = true;
      }

      while (!invalidDueToTimeout && easySerial->available() && _recv_buf_used < MODBUS_RECEIVE_BUFFER) {
        if (timeOutReached(timeout)) {
          invalidDueToTimeout = true;
        }
        _recv_buf[_recv_buf_used++] = easySerial->read();
      }

      if (_recv_buf_used > 2) {                                         // got length
        if (_recv_buf_used >= (3 + _recv_buf[2] + 2)) {                 // got whole pkt
          crc          = ModRTU_CRC(_recv_buf, _recv_buf_used);         // crc16 is 0 for whole valid pkt
          validPacket  = (crc == 0) && (_recv_buf[0] == _sendframe[0]); // check crc and address
          return_value = 0;                                             // reset return value
        }
      }
      delay(0);
    }

    // Check for MODBUS exception
    if (invalidDueToTimeout) {
      ++_reads_nodata;

      if (_recv_buf_used == 0) {
        return_value = MODBUS_NODATA;
      } else {
        return_value = MODBUS_TIMEOUT;
      }
    } else if (!validPacket) {
      ++_reads_crc_failed;
      return_value = MODBUS_BADCRC;
    } else {
      const uint8_t received_functionCode = _recv_buf[1];

      if ((received_functionCode & 0x80) != 0) {
        return_value = _recv_buf[2];
      }
      ++_reads_pass;
      _reads_nodata = 0;
    }

    switch (return_value) {
      case MODBUS_EXCEPTION_ACKNOWLEDGE:
      case MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY:
      case MODBUS_BADCRC:
      case MODBUS_TIMEOUT:

        // Bad communication, makes sense to retry.
        break;
      default:
        nrRetriesLeft = 0; // When not supported, does not make sense to retry.
        break;
    }
    --nrRetriesLeft;
  }
  _last_error = return_value;
  return return_value;
}

uint32_t ModbusRTU_struct::read_32b_InputRegister(short address) {
  uint32_t result = 0;
  uint8_t     errorcode;
  int idHigh = readInputRegister(address, errorcode);

  if (errorcode != 0) { return result; }
  int idLow = readInputRegister(address + 1, errorcode);

  if (errorcode == 0) {
    result  = idHigh;
    result  = result << 16;
    result += idLow;
  }
  return result;
}

uint32_t ModbusRTU_struct::read_32b_HoldingRegister(short address) {
  uint32_t result = 0;

  process_32b_register(_modbus_address, MODBUS_READ_HOLDING_REGISTERS, address, result);
  return result;
}

float ModbusRTU_struct::read_float_HoldingRegister(short address) {
  union {
    uint32_t ival;
    float    fval;
  } conversion;

  conversion.ival = read_32b_HoldingRegister(address);
  return conversion.fval;

  //    uint32_t ival = read_32b_HoldingRegister(address);
  //    float fval = *reinterpret_cast<float*>(&ival);
  //    return fval;
}

int ModbusRTU_struct::readInputRegister(short address, uint8_t& errorcode) {
  // Only read 1 register
  return process_16b_register(_modbus_address, MODBUS_READ_INPUT_REGISTERS, address, 1, errorcode);
}

int ModbusRTU_struct::readHoldingRegister(short address, uint8_t& errorcode) {
  // Only read 1 register
  return process_16b_register(
    _modbus_address, MODBUS_READ_HOLDING_REGISTERS, address, 1, errorcode);
}

// Write to holding register.
int ModbusRTU_struct::writeSingleRegister(short address, short value) {
  // No check for the specific error code.
  uint8_t errorcode = 0;

  return writeSingleRegister(address, value, errorcode);
}

int ModbusRTU_struct::writeSingleRegister(short address, short value, uint8_t& errorcode) {
  // GN: Untested, will probably not work
  return process_16b_register(
    _modbus_address, MODBUS_WRITE_SINGLE_REGISTER, address, value, errorcode);
}

// Function 16 (0x10) "Write Multiple Registers" to write to a single holding register
int ModbusRTU_struct::writeMultipleRegisters(short address, short value) {
  return preset_mult16b_register(
    _modbus_address, address, value);
}

uint8_t ModbusRTU_struct::modbus_get_MEI(uint8_t slaveAddress, uint8_t object_id,
                                      String& result, unsigned int& object_value_int,
                                      uint8_t& next_object_id, bool& more_follows,
                                      uint8_t& conformity_level) {
  // Force device_id to 4 = individual access (reading one ID object per call)
  build_modbus_MEI_frame(slaveAddress, 4, object_id);
  const uint8_t process_result = processCommand();

  if (process_result == 0) {
    result = parse_modbus_MEI_response(object_value_int,
                                       next_object_id, more_follows,
                                       conformity_level);
  } else {
    more_follows = false;
  }
  return process_result;
}

void ModbusRTU_struct::modbus_log_MEI(uint8_t slaveAddress) {
  // Iterate over all Device identification items, using
  // Modbus command (0x2B / 0x0E) Read Device Identification
  // And add to log.
  bool more_follows     = true;
  uint8_t conformity_level = 0;
  uint8_t object_id        = 0;
  uint8_t next_object_id   = 0;

  while (more_follows) {
    String result;
    unsigned int object_value_int;
    const uint8_t   process_result = modbus_get_MEI(
      slaveAddress, object_id, result, object_value_int, next_object_id,
      more_follows, conformity_level);

    if (process_result == 0) {
      if (result.length() > 0) {
        String log = MEI_objectid_to_name(object_id);
        log += F(": ");
        log += result;
        addLogMove(LOG_LEVEL_INFO, log);
      }
    } else {
      switch (process_result) {
        case MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS:

          // No need to log this exception when scanning.
          break;
        default:
          logModbusException(process_result);
          break;
      }
    }

    // If more parts are needed, collect them or iterate over the known list.
    // For example with "individual access" a new request has to be sent for each single item
    if (more_follows) {
      object_id = next_object_id;
    } else if (object_id < 0x84) {
      // Allow for scanning only the usual object ID's
      // This range is vendor specific
      more_follows = true;
      object_id++;

      if (object_id == 7) {
        // Skip range 0x07...0x7F
        object_id = 0x80;
      }
    }
  }
}

int ModbusRTU_struct::process_16b_register(uint8_t slaveAddress, uint8_t functionCode,
                                           short startAddress, short parameter,
                                           uint8_t& errorcode) {
  buildFrame(slaveAddress, functionCode, startAddress, parameter);
  errorcode = processCommand();

  if (errorcode == 0) {
    return (_recv_buf[3] << 8) | (_recv_buf[4]);
  }
  logModbusException(errorcode);
  return -1;
}

// Still writing single register, but calling it using "Preset Multiple Registers" function (FC=16)
int ModbusRTU_struct::preset_mult16b_register(uint8_t slaveAddress, uint16_t startAddress, uint16_t value) {
  buildWriteMult16bRegister(slaveAddress, startAddress, value);
  const uint8_t process_result = processCommand();

  if (process_result == 0) {
    return (_recv_buf[4] << 8) | (_recv_buf[5]);
  }
  logModbusException(process_result);
  return -1 * process_result;
}

bool ModbusRTU_struct::process_32b_register(uint8_t slaveAddress, uint8_t functionCode,
                                            short startAddress, uint32_t& result) {
  buildFrame(slaveAddress, functionCode, startAddress, 2);
  const uint8_t process_result = processCommand();

  if (process_result == 0) {
    result = 0;

    for (uint8_t i = 0; i < 4; ++i) {
      result  = result << 8;
      result += _recv_buf[i + 3];
    }
    return true;
  }
  logModbusException(process_result);
  return false;
}

int ModbusRTU_struct::writeSpecialCommandRegister(uint8_t command) {
  buildWriteCommandRegister(_modbus_address, command);
  const uint8_t process_result = processCommand();

  if (process_result == 0) {
    return 0;
  }
  logModbusException(process_result);
  return -1 * process_result;
}

unsigned int ModbusRTU_struct::read_RAM_EEPROM(uint8_t command, uint8_t startAddress,
                                               uint8_t nrBytes,
                                               uint8_t& errorcode) {
  buildRead_RAM_EEPROM(_modbus_address, command,
                       startAddress, nrBytes);
  errorcode = processCommand();

  if (errorcode == 0) {
    unsigned int result = 0;

    for (int i = 0; i < _recv_buf[2]; ++i) {
      // Most significant uint8_t at lower address
      result = (result << 8) | _recv_buf[i + 3];
    }
    return result;
  }
  logModbusException(errorcode);
  return 0;
}

// Compute the MODBUS RTU CRC
unsigned int ModbusRTU_struct::ModRTU_CRC(uint8_t *buf, int len) {
  unsigned int crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (unsigned int)buf[pos]; // XOR uint8_t into least sig. uint8_t of crc

    for (int i = 8; i != 0; i--) { // Loop over each bit
      if ((crc & 0x0001) != 0) {   // If the LSB is set
        crc >>= 1;                 // Shift right and XOR 0xA001
        crc  ^= 0xA001;
      } else {                     // Else LSB is not set
        crc >>= 1;                 // Just shift right
      }
    }
  }
  return crc;
}

uint32_t ModbusRTU_struct::readTypeId() {
  return read_32b_InputRegister(25);
}

uint32_t ModbusRTU_struct::readSensorId() {
  return read_32b_InputRegister(29);
}

uint8_t ModbusRTU_struct::getLastError() const {
  return _last_error;
}

uint32_t ModbusRTU_struct::getFailedReadsSinceLastValid() const {
  return _reads_nodata;
}

void ModbusRTU_struct::startWrite() {
  // transmit to device  -> DE Enable, /RE Disable (for control MAX485)
  if ((_dere_pin == -1) || !isInitialized()) { return; }
  digitalWrite(_dere_pin, HIGH);
  delay(2); // Switching may take some time
}

void ModbusRTU_struct::startRead() {
  if (!isInitialized()) { return; }
  easySerial->flush(); // clear out tx buffer

  // receive from device -> DE Disable, /RE Enable (for control MAX485)
  if (_dere_pin != -1) {
    digitalWrite(_dere_pin, LOW);
  }
}

