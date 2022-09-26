#include "../DataStructs/RulesEventCache.h"

#include "../DataStructs/TimingStats.h"
#include "../Helpers/RulesMatcher.h"


void RulesEventCache::clear()
{
  _eventCache.clear();
  _initialized = false;
}

void RulesEventCache::initialize()
{
  _initialized = true;
}

bool RulesEventCache::addLine(const String& line, const String& filename, size_t pos)
{
  String event, action;

  if (getEventFromRulesLine(line, event, action)) {
    _eventCache.emplace_back(filename, pos, std::move(event), std::move(action));
    return true;
  }
  return false;
}

RulesEventCache_vector::const_iterator RulesEventCache::findMatchingRule(const String& event, bool optimize)
{
  RulesEventCache_vector::iterator it   = _eventCache.begin();
//  RulesEventCache_vector::iterator prev = _eventCache.end();

  // FIXME TD-er: Disable optimize as it has some side effects.
  // For example, matching a specific event first and then a more generic one is perfectly normal to do.
  // But this optimization will then put the generic one in front as it will be matched more often.
  // Thus it will never match the more specific one anymore.


  for (; it != _eventCache.end(); ++it)
  {
    START_TIMER
    const bool match = ruleMatch(event, it->_event);
    STOP_TIMER(RULES_MATCH);

    if (match) {
      /*
      if (optimize) {
        it->_nrTimesMatched++;

        if (prev != _eventCache.end()) {
          // Check to see if we need to place this one more to the front of the vector
          // to speed up parsing.
          if (prev->_nrTimesMatched < it->_nrTimesMatched) {
            std::swap(*prev, *it);
            return prev;
          }
        }
      }
      */
      return it;
    }
/*
    if (optimize) {
      if (prev == _eventCache.end()) {
        prev = it;
      }
      else if (prev->_nrTimesMatched > it->_nrTimesMatched) {
        // Found one that's having a lower match rate
        prev = it;
      }
    }
    */
  }
  return it;
}

#include "../DataStructs/FactoryDefaultPref.h"

#include "../../ESPEasy_common.h"

ResetFactoryDefaultPreference_struct::ResetFactoryDefaultPreference_struct(uint32_t preference) : _preference(preference) {}

DeviceModel ResetFactoryDefaultPreference_struct::getDeviceModel() const {
  return static_cast<DeviceModel>(_preference & 0xFF);
}

void ResetFactoryDefaultPreference_struct::setDeviceModel(DeviceModel model) {
  _preference &= ~(0xFF); // set DeviceModel bits to 0
  _preference |= static_cast<uint32_t>(model);
}

bool ResetFactoryDefaultPreference_struct::keepWiFi() const {
  return bitRead(_preference, 9);
}

void ResetFactoryDefaultPreference_struct::keepWiFi(bool keep) {
  bitWrite(_preference, 9, keep);
}

bool ResetFactoryDefaultPreference_struct::keepNTP() const {
  return bitRead(_preference, 10);
}

void ResetFactoryDefaultPreference_struct::keepNTP(bool keep) {
  bitWrite(_preference, 10, keep);
}

bool ResetFactoryDefaultPreference_struct::keepNetwork() const {
  return bitRead(_preference, 11);
}

void ResetFactoryDefaultPreference_struct::keepNetwork(bool keep) {
  bitWrite(_preference, 11, keep);
}

bool ResetFactoryDefaultPreference_struct::keepLogSettings() const {
  return bitRead(_preference, 12);
}

void ResetFactoryDefaultPreference_struct::keepLogSettings(bool keep) {
  bitWrite(_preference, 12, keep);
}

bool ResetFactoryDefaultPreference_struct::keepUnitName() const {
  return bitRead(_preference, 13);
}

void ResetFactoryDefaultPreference_struct::keepUnitName(bool keep) {
  bitWrite(_preference, 13, keep);
}

// filenr = 0...3 for files rules1.txt ... rules4.txt
bool ResetFactoryDefaultPreference_struct::fetchRulesTXT(int filenr) const {
  return bitRead(_preference, 14 + filenr);
}

void ResetFactoryDefaultPreference_struct::fetchRulesTXT(int filenr, bool fetch) {
  bitWrite(_preference, 14 + filenr, fetch);
}

bool ResetFactoryDefaultPreference_struct::fetchNotificationDat() const {
  return bitRead(_preference, 18);
}

void ResetFactoryDefaultPreference_struct::fetchNotificationDat(bool fetch) {
  bitWrite(_preference, 18, fetch);
}

bool ResetFactoryDefaultPreference_struct::fetchSecurityDat() const {
  return bitRead(_preference, 19);
}

void ResetFactoryDefaultPreference_struct::fetchSecurityDat(bool fetch) {
  bitWrite(_preference, 19, fetch);
}

bool ResetFactoryDefaultPreference_struct::fetchConfigDat() const {
  return bitRead(_preference, 20);
}

void ResetFactoryDefaultPreference_struct::fetchConfigDat(bool fetch) {
  bitWrite(_preference, 20, fetch);
}

bool ResetFactoryDefaultPreference_struct::deleteFirst() const {
  return bitRead(_preference, 21);
}

void ResetFactoryDefaultPreference_struct::deleteFirst(bool checked) {
  bitWrite(_preference, 21, checked);
}

bool ResetFactoryDefaultPreference_struct::saveURL() const {
  return bitRead(_preference, 22);
}

void ResetFactoryDefaultPreference_struct::saveURL(bool checked) {
  bitWrite(_preference, 22, checked);
}

bool ResetFactoryDefaultPreference_struct::allowFetchByCommand() const {
  return bitRead(_preference, 23);
}

void ResetFactoryDefaultPreference_struct::allowFetchByCommand(bool checked) {
  bitWrite(_preference, 23, checked);
}

bool ResetFactoryDefaultPreference_struct::storeCredentials() const {
  return bitRead(_preference, 24);
}

void ResetFactoryDefaultPreference_struct::storeCredentials(bool checked) {
  bitWrite(_preference, 24, checked);
}

bool ResetFactoryDefaultPreference_struct::fetchProvisioningDat() const {
  return bitRead(_preference, 25);
}

void ResetFactoryDefaultPreference_struct::fetchProvisioningDat(bool checked) {
  bitWrite(_preference, 25, checked);
}


uint32_t ResetFactoryDefaultPreference_struct::getPreference() {
  return _preference;
}

#include "../DataStructs/ESPEasy_EventStruct.h"

#include "../CustomBuild/ESPEasyLimits.h"
#include "../DataTypes/EventValueSource.h"
#include "../Globals/Plugins.h"
#include "../Globals/CPlugins.h"
#include "../Globals/NPlugins.h"

#include "../../_Plugin_Helper.h"

EventStruct::EventStruct(taskIndex_t taskIndex) :
  TaskIndex(taskIndex), BaseVarIndex(taskIndex * VARS_PER_TASK) 
{}

void EventStruct::deep_copy(const struct EventStruct& other) {
  this->operator=(other);
}

void EventStruct::deep_copy(const struct EventStruct* other) {
  if (other != nullptr) {
    deep_copy(*other);
  }
}

void EventStruct::setTaskIndex(taskIndex_t taskIndex) {
  TaskIndex    = taskIndex;
  BaseVarIndex = taskIndex * VARS_PER_TASK;
  sensorType   = Sensor_VType::SENSOR_TYPE_NOT_SET;
}

void EventStruct::clear() {
  *this = EventStruct();
}

Sensor_VType EventStruct::getSensorType() {
  const int tmp_idx = idx;
  checkDeviceVTypeForTask(this);
  idx = tmp_idx;
  return sensorType;
}

#include "../DataStructs/Modbus.h"

#if FEATURE_MODBUS

#include "../DataStructs/ControllerSettingsStruct.h"
#include "../ESPEasyCore/ESPEasy_Log.h"


Modbus::Modbus() : ModbusClient(nullptr), errcnt(0), timeout(0),
  TXRXstate(MODBUS_IDLE), RXavailable(0), payLoad(0) {}

Modbus::~Modbus() {
  if (ModbusClient) {
    ModbusClient->flush();
    ModbusClient->stop();
    delete (ModbusClient);
    delay(1);
    ModbusClient = nullptr;
  }
}

bool Modbus::begin(uint8_t function, uint8_t ModbusID, uint16_t ModbusRegister,  MODBUS_registerTypes_t type, char *IPaddress)
{
  currentRegister = ModbusRegister;
  currentFunction = function;
  incomingValue   = type;
  resultReceived  = false;
  ModbusClient    = new (std::nothrow) WiFiClient();
  if (ModbusClient == nullptr) {
    return false;
  }
  ModbusClient->setNoDelay(true);
  ModbusClient->setTimeout(CONTROLLER_CLIENTTIMEOUT_DFLT);
  timeout = millis();
  ModbusClient->flush();

  if (ModbusClient->connected()) {
    #ifndef BUILD_NO_DEBUG
    LogString += F(" already connected. ");
    #endif
  } else {
    #ifndef BUILD_NO_DEBUG
    LogString += F("connect: ");
    LogString += IPaddress;
    #endif

    if (ModbusClient->connect(IPaddress, 502) != 1) {
      #ifndef BUILD_NO_DEBUG
      LogString += F(" fail. ");
      #endif
      TXRXstate  = MODBUS_IDLE;
      errcnt++;

      #ifndef BUILD_NO_DEBUG
      if (LogString.length() > 1) { addLog(LOG_LEVEL_DEBUG, LogString); }
      #endif
      return false;
    }
  }
  #ifndef BUILD_NO_DEBUG
  LogString += F(" OK, sending read request: ");
  #endif

  sendBuffer[6] = ModbusID;
  sendBuffer[7] = function;
  sendBuffer[8] = (ModbusRegister >> 8);
  sendBuffer[9] = (ModbusRegister & 0x00ff);

  if ((incomingValue == signed64) || (incomingValue == unsigned64)) {
    sendBuffer[11] = 4;
  }

  if ((incomingValue == signed32) || (incomingValue == unsigned32)) {
    sendBuffer[11] = 2;
  }

  if ((incomingValue == signed16) || (incomingValue == unsigned16)) {
    sendBuffer[11] = 1;
  }
  ModbusClient->flush();
  ModbusClient->write(&sendBuffer[0], sizeof(sendBuffer));

  #ifndef BUILD_NO_DEBUG
  for (unsigned int i = 0; i < sizeof(sendBuffer); i++) {
    LogString += ((unsigned int)(sendBuffer[i]));
    LogString += ' ';
  }
  #endif
  TXRXstate = MODBUS_RECEIVE;
  
  #ifndef BUILD_NO_DEBUG
  if (LogString.length() > 1) { addLog(LOG_LEVEL_DEBUG, LogString); }
  #endif
  return true;
}

bool Modbus::handle() {
  unsigned int RXavailable = 0;

  #ifndef BUILD_NO_DEBUG
  LogString = String();
  #endif
  int64_t rxValue = 0;

  switch (TXRXstate) {
    case MODBUS_IDLE:

      // clean up;
      if (ModbusClient) {
        ModbusClient->flush();
        ModbusClient->stop();
        delete (ModbusClient);
        delay(1);
        ModbusClient = nullptr;
      }
      break;

    case MODBUS_RECEIVE:

      if  (hasTimeout()) { break; }

      if  (ModbusClient->available() < 9) { break; }

      #ifndef BUILD_NO_DEBUG
      LogString += F("reading bytes: ");
      #endif

      for (int a = 0; a < 9; a++) {
        payLoad    = ModbusClient->read();
        #ifndef BUILD_NO_DEBUG
        LogString += (payLoad);
        LogString += ' ';
        #endif
      }
      #ifndef BUILD_NO_DEBUG
      LogString += F("> ");
      #endif

      if (payLoad > 8) {
        #ifndef BUILD_NO_DEBUG
        LogString += F("Payload too large !? ");
        #endif
        errcnt++;
        TXRXstate = MODBUS_IDLE;
      }
      // FIXME TD-er: Missing break?

    case MODBUS_RECEIVE_PAYLOAD:

      if  (hasTimeout()) { break; }
      RXavailable = ModbusClient->available();

      if (payLoad != RXavailable) {
        TXRXstate = MODBUS_RECEIVE_PAYLOAD;
        break;
      }

      for (unsigned int i = 0; i < RXavailable; i++) {
        rxValue = rxValue << 8;
        char a = ModbusClient->read();
        rxValue    = rxValue | a;
        #ifndef BUILD_NO_DEBUG
        LogString += static_cast<int>(a);  
        LogString += ' ';
        #endif
      }

      switch (incomingValue) {
        case signed16:
          result = (int16_t)rxValue;
          break;
        case unsigned16:
          result = (uint16_t)rxValue;
          break;
        case signed32:
          result = (int32_t)rxValue;
          break;
        case unsigned32:
          result = (uint32_t)rxValue;
          break;
        case signed64:
          result = (int64_t)rxValue;
          break;
        case unsigned64:
          result = (uint64_t)rxValue;
          break;
      }

      #ifndef BUILD_NO_DEBUG
      LogString += F("value: "); 
      LogString += result;
      #endif

      // if ((systemTimePresent()) && (hour() == 0)) errcnt = 0;

      TXRXstate = MODBUS_IDLE;

      resultReceived = true;
      break;

    default:
      #ifndef BUILD_NO_DEBUG
      LogString += F("default. ");
      #endif
      TXRXstate  = MODBUS_IDLE;
      break;
  }
  #ifndef BUILD_NO_DEBUG
  if (LogString.length() > 1) { addLog(LOG_LEVEL_DEBUG, LogString); }
  #endif
  return true;
}

bool Modbus::hasTimeout()
{
  if   ((millis() - timeout) > 10000) { // too many bytes or timeout
    #ifndef BUILD_NO_DEBUG
    LogString += F("Modbus RX timeout. "); 
    LogString += String(ModbusClient->available());
    #endif
    errcnt++;
    TXRXstate = MODBUS_IDLE;
    return true;
  }
  return false;
}

// tryread can be called in a round robin fashion. It will initiate a read if Modbus is idle and update the result once it is available.
// subsequent calls (if Modbus is busy etc. ) will return false and not update the result.
// Use to read multiple values non blocking in an re-entrant function. Not tested yet.
bool Modbus::tryRead(uint8_t ModbusID, uint16_t M_register,  MODBUS_registerTypes_t type, char *IPaddress, double& result) {
  if (isBusy()) { return false; // not done yet
  }

  if (available()) {
    if ((currentFunction == MODBUS_FUNCTION_READ) && (currentRegister == M_register)) {
      result = read(); // result belongs to this request.
      return true;
    }
  } else {
    begin(MODBUS_FUNCTION_READ, ModbusID, M_register, type, IPaddress); // idle and no result -> begin read request
  }
  return false;
}

#endif // FEATURE_MODBUS

#include "../DataStructs/tcp_cleanup.h"



#if defined(ESP8266)

struct tcp_pcb;
extern struct tcp_pcb* tcp_tw_pcbs;
extern "C" void tcp_abort (struct tcp_pcb* pcb);

void tcpCleanup()
{

     while(tcp_tw_pcbs!=nullptr)
    {
      tcp_abort(tcp_tw_pcbs);
    }

 }
#endif
#include "../DataStructs/ProvisioningStruct.h"

#if FEATURE_CUSTOM_PROVISIONING

# include "../Helpers/StringConverter.h"
# include "../Helpers/Hardware.h"

ProvisioningStruct::ProvisioningStruct() {
  ZERO_FILL(user);
  ZERO_FILL(pass);
  ZERO_FILL(url);
}

void ProvisioningStruct::validate() {
  ZERO_TERMINATE(user);
  ZERO_TERMINATE(pass);
  ZERO_TERMINATE(url);
}

bool ProvisioningStruct::matchingFlashSize() const
{
  return modelMatchingFlashSize(ResetFactoryDefaultPreference.getDeviceModel());
}

bool ProvisioningStruct::setUser(const String& username)
{
  return safe_strncpy(user, username, sizeof(user));
}

bool ProvisioningStruct::setPass(const String& password)
{
  return safe_strncpy(pass, password, sizeof(pass));
}

bool ProvisioningStruct::setUrl(const String& url_str)
{
  return safe_strncpy(url, url_str, sizeof(url));
}

#endif // if FEATURE_CUSTOM_PROVISIONING

#include "../DataStructs/PortStatusStruct.h"

#include "../DataStructs/PinMode.h"

portStatusStruct::portStatusStruct() : state(-1), output(-1), command(0), init(0), not_used(0), mode(0), task(0), monitor(0), forceMonitor(0),
  forceEvent(0), previousTask(-1), x(INVALID_DEVICE_INDEX) {}

uint16_t portStatusStruct::getDutyCycle() const
{
  if (mode == PIN_MODE_PWM) {
    return dutyCycle;
  }
  return 0;
}

int16_t portStatusStruct::getValue() const
{
  switch (mode) {
    case PIN_MODE_PWM:
    case PIN_MODE_SERVO:
      return dutyCycle;
    default:
      break;
  }
  return state;
}
#include "../DataStructs/SystemTimerStruct.h"

#include "../ESPEasyCore/ESPEasy_Log.h"


// Rules Timer use
// ***************
systemTimerStruct::systemTimerStruct(int recurringCount, unsigned long msecFromNow, unsigned int timerIndex) :
  Par1(recurringCount), Par2(msecFromNow), Par3(timerIndex), Par4(0), Par5(1)
{
  if (recurringCount > 0) {
    // Will run with Par1 == 0, so must subtract one when setting the value.
    Par1--;
  }

  if (msecFromNow == 0) {
    // Create a new timer which should be "scheduled" now to clear up any data
    Par1 = 0; // Do not reschedule
    Par5 = 0; // Do not execute
    addLog(LOG_LEVEL_INFO, F("TIMER: disable timer"));
  }
}

bool systemTimerStruct::isRecurring() const {
  return Par1 != 0;
}

void systemTimerStruct::markNextRecurring() {
  if (Par1 > 0) {
    // This is a timer with a limited number of runs, so decrease its value.
    Par1--;
  }

  if (Par5 > 0) {
    // This one should be executed, so increase the count.
    Par5++;
  }
}

unsigned long systemTimerStruct::getInterval() const {
  return Par2;
}

unsigned int systemTimerStruct::getTimerIndex() const {
  return Par3;
}

bool systemTimerStruct::isPaused() const {
  return Par4 != 0;
}

int systemTimerStruct::getRemainder() const {
  return Par4;
}

void systemTimerStruct::setRemainder(int timeLeft) {
  Par4 = timeLeft;
}

int systemTimerStruct::getLoopCount() const {
  return Par5;
}

#include "../DataStructs/UserVarStruct.h"

#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/Plugins.h"

UserVarStruct::UserVarStruct()
{
  _data.resize(VARS_PER_TASK * TASKS_MAX);
  for (size_t i = 0; i < (VARS_PER_TASK * TASKS_MAX); ++i) {
    _data[i] = 0.0f;
  }
}

// Implementation of [] operator.  This function must return a
// reference as array element can be put on left side
float& UserVarStruct::operator[](unsigned int index)
{
  if (index >= _data.size()) {
    static float errorvalue = NAN;
    addLog(LOG_LEVEL_ERROR, F("UserVar index out of range"));
    return errorvalue;
  }
  return _data.at(index);
}

const float& UserVarStruct::operator[](unsigned int index) const
{
  if (index >= _data.size()) {
    static float errorvalue = NAN;
    addLog(LOG_LEVEL_ERROR, F("UserVar index out of range"));
    return errorvalue;
  }
  return _data.at(index);
}

unsigned long UserVarStruct::getSensorTypeLong(taskIndex_t taskIndex) const
{
  unsigned long value = 0;

  if (validTaskIndex(taskIndex)) {
    const unsigned int baseVarIndex = taskIndex * VARS_PER_TASK;
    value = static_cast<unsigned long>(_data[baseVarIndex]) + (static_cast<unsigned long>(_data[baseVarIndex + 1]) << 16);
  }
  return value;
}

void UserVarStruct::setSensorTypeLong(taskIndex_t taskIndex, unsigned long value)
{
  if (!validTaskIndex(taskIndex)) {
    return;
  }
  const unsigned int baseVarIndex = taskIndex * VARS_PER_TASK;

  _data[baseVarIndex]     = value & 0xFFFF;
  _data[baseVarIndex + 1] = (value >> 16) & 0xFFFF;
}

uint32_t UserVarStruct::getUint32(taskIndex_t taskIndex, uint8_t varNr) const
{
  if (!validTaskIndex(taskIndex) || (varNr >= VARS_PER_TASK)) {
    addLog(LOG_LEVEL_ERROR, F("UserVar index out of range"));
    return 0;
  }
  const unsigned int baseVarIndex = taskIndex * VARS_PER_TASK;
  uint32_t res;
  memcpy(&res, &_data[baseVarIndex + varNr], sizeof(float));
  return res;
}

void UserVarStruct::setUint32(taskIndex_t taskIndex, uint8_t varNr, uint32_t value)
{
  if (!validTaskIndex(taskIndex) || (varNr >= VARS_PER_TASK)) {
    addLog(LOG_LEVEL_ERROR, F("UserVar index out of range"));
    return;
  }
  const unsigned int baseVarIndex = taskIndex * VARS_PER_TASK;

  // Store in a new variable to prevent
  // warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
  float tmp;
  memcpy(&tmp, &value, sizeof(float));
  _data[baseVarIndex + varNr] = tmp;
}

size_t UserVarStruct::getNrElements() const
{
  return _data.size();
}

uint8_t * UserVarStruct::get()
{
  return reinterpret_cast<uint8_t *>(&_data[0]);
}

#include "../DataStructs/RTCStruct.h"

  void RTCStruct::init() {
    ID1 = 0xAA;
    ID2 = 0x55;
    clearLastWiFi();
    factoryResetCounter = 0;
    deepSleepState = 0;
    bootFailedCount = 0;
    flashDayCounter = 0;
    lastWiFiSettingsIndex = 0;
    flashCounter = 0;
    bootCounter = 0;
    lastMixedSchedulerId = 0;
    unused1 = 0;
    unused2 = 0;
    lastSysTime = 0;
  }

  void RTCStruct::clearLastWiFi() {
    for (uint8_t i = 0; i < 6; ++i) {
      lastBSSID[i] = 0;
    }
    lastWiFiChannel = 0;
    lastWiFiSettingsIndex = 0;
  }

  bool RTCStruct::lastWiFi_set() const {
    return lastBSSID[0] != 0 && lastWiFiChannel != 0 && lastWiFiSettingsIndex != 0;
  }

#include "../DataStructs/ExtraTaskSettingsStruct.h"

#include "../../ESPEasy_common.h"

#define EXTRA_TASK_SETTINGS_VERSION 1

ExtraTaskSettingsStruct::ExtraTaskSettingsStruct() : TaskIndex(INVALID_TASK_INDEX) {
  clear();
}

void ExtraTaskSettingsStruct::clear() {
  TaskIndex = INVALID_TASK_INDEX;
  ZERO_FILL(TaskDeviceName);

  clearUnusedValueNames(0);

  for (uint8_t i = 0; i < PLUGIN_EXTRACONFIGVAR_MAX; ++i) {
    TaskDevicePluginConfigLong[i] = 0;
    TaskDevicePluginConfig[i]     = 0;
  }
}

void ExtraTaskSettingsStruct::validate() {
  ZERO_TERMINATE(TaskDeviceName);

  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    ZERO_TERMINATE(TaskDeviceFormula[i]);
    ZERO_TERMINATE(TaskDeviceValueNames[i]);
  }

  if (dummy1 != 0) {
    // FIXME TD-er: This check was added to add the version for allowing to make transitions on the data.
    // If we've been using this for a while, we no longer need to check for the value of this dummy and we can re-use it for something else.
    dummy1  = 0;
    version = 0;
  }

  if (version != EXTRA_TASK_SETTINGS_VERSION) {
    if (version < 1) {
      // Need to initialize the newly added fields
      for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
        setIgnoreRangeCheck(i);
        TaskDeviceErrorValue[i] = 0.0f;
        VariousBits[i]          = 0u;
      }
    }
    version = EXTRA_TASK_SETTINGS_VERSION;
  }
}

bool ExtraTaskSettingsStruct::checkUniqueValueNames() const {
  for (int i = 0; i < (VARS_PER_TASK - 1); ++i) {
    for (int j = i; j < VARS_PER_TASK; ++j) {
      if ((i != j) && (TaskDeviceValueNames[i][0] != 0)) {
        if (strcasecmp(TaskDeviceValueNames[i], TaskDeviceValueNames[j]) == 0) {
          return false;
        }
      }
    }
  }
  return true;
}

void ExtraTaskSettingsStruct::clearUnusedValueNames(uint8_t usedVars) {
  for (uint8_t i = usedVars; i < VARS_PER_TASK; ++i) {
    ZERO_FILL(TaskDeviceFormula[i]);
    ZERO_FILL(TaskDeviceValueNames[i]);
    TaskDeviceValueDecimals[i] = 2;
    setIgnoreRangeCheck(i);
    TaskDeviceErrorValue[i] = 0.0f;
    VariousBits[i] = 0;
  }
}

bool ExtraTaskSettingsStruct::checkInvalidCharInNames(const char *name) const {
  int pos = 0;

  while (*(name + pos) != 0) {
    if (!validCharForNames(*(name + pos))) { return false; }
    ++pos;
  }
  return true;
}

bool ExtraTaskSettingsStruct::checkInvalidCharInNames() const {
  if (!checkInvalidCharInNames(&TaskDeviceName[0])) { return false; }

  for (int i = 0; i < VARS_PER_TASK; ++i) {
    if (!checkInvalidCharInNames(&TaskDeviceValueNames[i][0])) { return false; }
  }
  return true;
}

bool ExtraTaskSettingsStruct::validCharForNames(char c) {
  // Smal optimization to check these chars as they are in sequence in the ASCII table

  /*
     case '(': // 40
     case ')': // 41
     case '*': // 42
     case '+': // 43
     case ',': // 44
     case '-': // 45
   */

  if ((c >= '(') && (c <= '-')) { return false; }

  if (
    (c == ' ') ||
    (c == '!') ||
    (c == '#') ||
    (c == '%') ||
    (c == '/') ||
    (c == '=') ||
    (c == '[') ||
    (c == ']') ||
    (c == '^') ||
    (c == '{') ||
    (c == '}')) {
    return false;
  }
  return true;
}

void ExtraTaskSettingsStruct::setAllowedRange(taskVarIndex_t taskVarIndex, const float& minValue, const float& maxValue)
{
  if (validTaskVarIndex(taskVarIndex)) {
    if (minValue > maxValue) {
      TaskDeviceMinValue[taskVarIndex] = maxValue;
      TaskDeviceMaxValue[taskVarIndex] = minValue;
    } else {
      TaskDeviceMinValue[taskVarIndex] = minValue;
      TaskDeviceMaxValue[taskVarIndex] = maxValue;
    }
  }
}

void ExtraTaskSettingsStruct::setIgnoreRangeCheck(taskVarIndex_t taskVarIndex)
{
  if (validTaskVarIndex(taskVarIndex)) {
    // Clear range to indicate no range check should be done.
    TaskDeviceMinValue[taskVarIndex] = 0.0f;
    TaskDeviceMaxValue[taskVarIndex] = 0.0f;
  }
}

bool ExtraTaskSettingsStruct::ignoreRangeCheck(taskVarIndex_t taskVarIndex) const
{
  if (validTaskVarIndex(taskVarIndex)) {
    return essentiallyEqual(TaskDeviceMinValue[taskVarIndex], TaskDeviceMaxValue[taskVarIndex]);
  }
  return true;
}

bool ExtraTaskSettingsStruct::valueInAllowedRange(taskVarIndex_t taskVarIndex, const float& value) const
{
  if (ignoreRangeCheck(taskVarIndex)) { return true; }

  if (validTaskVarIndex(taskVarIndex)) {
    return definitelyLessThan(value, TaskDeviceMaxValue[taskVarIndex]) ||
           definitelyGreaterThan(value, TaskDeviceMinValue[taskVarIndex]);
  }
  #ifndef BUILD_NO_DEBUG
  addLog(LOG_LEVEL_ERROR, F("Programming error: invalid taskVarIndex"));
  #endif // ifndef BUILD_NO_DEBUG
  return false;
}

float ExtraTaskSettingsStruct::checkAllowedRange(taskVarIndex_t taskVarIndex, const float& value) const
{
  if (!valueInAllowedRange(taskVarIndex, value)) {
    if (validTaskVarIndex(taskVarIndex)) {
      return TaskDeviceErrorValue[taskVarIndex];
    }
  }
  return value;
}

bool ExtraTaskSettingsStruct::enabledPluginStats(taskVarIndex_t taskVarIndex) const
{
  if (!validTaskVarIndex(taskVarIndex)) { return false; }
  return bitRead(VariousBits[taskVarIndex], 0);
}

void ExtraTaskSettingsStruct::enablePluginStats(taskVarIndex_t taskVarIndex, bool enabled)
{
  if (validTaskVarIndex(taskVarIndex)) {
    bitWrite(VariousBits[taskVarIndex], 0, enabled);
  }
}

bool ExtraTaskSettingsStruct::anyEnabledPluginStats() const
{
  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    if (enabledPluginStats(i)) return true;
  }
  return false;
}

#include "../DataStructs/ProtocolStruct.h"

ProtocolStruct::ProtocolStruct() :
    defaultPort(0), Number(0), usesMQTT(false), usesAccount(false), usesPassword(false),
    usesTemplate(false), usesID(false), Custom(false), usesHost(true), usesPort(true),
    usesQueue(true), usesCheckReply(true), usesTimeout(true), usesSampleSets(false), 
    usesExtCreds(false), needsNetwork(true), allowsExpire(true), allowLocalSystemTime(false) {}

bool ProtocolStruct::useCredentials() const {
  return usesAccount || usesPassword;
}

bool ProtocolStruct::useExtendedCredentials() const {
  if (usesExtCreds) {
    return useCredentials();
  }
  return false;
}
#include "../DataStructs/WiFiEventData.h"

#include "../ESPEasyCore/ESPEasy_Log.h"

#include "../Globals/RTC.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/WiFi_AP_Candidates.h"

#include "../Helpers/ESPEasy_Storage.h"


// Bit numbers for WiFi status
#define ESPEASY_WIFI_CONNECTED               0
#define ESPEASY_WIFI_GOT_IP                  1
#define ESPEASY_WIFI_SERVICES_INITIALIZED    2

#define WIFI_RECONNECT_WAIT                  20000  // in milliSeconds
#define WIFI_PROCESS_EVENTS_TIMEOUT          10000  // in milliSeconds

bool WiFiEventData_t::WiFiConnectAllowed() const {
  if (!wifiConnectAttemptNeeded) return false;
  if (wifiSetupConnect) return true;
  if (lastDisconnectMoment.isSet()) {
    // TODO TD-er: Make this time more dynamic.
    if (!lastDisconnectMoment.timeoutReached(1000)) {
      return false;
    }
  }
  return true;
}

bool WiFiEventData_t::unprocessedWifiEvents() const {
  if (processedConnect && processedDisconnect && processedGotIP && processedDHCPTimeout)
  {
    return false;
  }
  if (!processedConnect) {
    if (lastConnectMoment.isSet() && lastConnectMoment.timeoutReached(WIFI_PROCESS_EVENTS_TIMEOUT)) {
      return false;
    }
  }
  if (!processedGotIP) {
    if (lastGetIPmoment.isSet() && lastGetIPmoment.timeoutReached(WIFI_PROCESS_EVENTS_TIMEOUT)) {
      return false;
    }
  }
  if (!processedDisconnect) {
    if (lastDisconnectMoment.isSet() && lastDisconnectMoment.timeoutReached(WIFI_PROCESS_EVENTS_TIMEOUT)) {
      return false;
    }
  }
  if (!processedDHCPTimeout) {
    return false;
  }
  return true;
}

void WiFiEventData_t::clearAll() {
  markWiFiTurnOn();
  lastGetScanMoment.clear();
  last_wifi_connect_attempt_moment.clear();
  timerAPstart.clear();

  lastWiFiResetMoment.setNow();
  wifi_TX_pwr = 0;
  usedChannel = 0;
}

void WiFiEventData_t::markWiFiTurnOn() {
  setWiFiDisconnected();
  lastDisconnectMoment.clear();
  lastConnectMoment.clear();
  lastGetIPmoment.clear();
  wifi_considered_stable    = false;

  // Mark all flags to default to prevent handling old events.
  processedConnect          = true;
  processedDisconnect       = true;
  processedGotIP            = true;
  processedDHCPTimeout      = true;
  processedConnectAPmode    = true;
  processedDisconnectAPmode = true;
  processedScanDone         = true;
  wifiConnectAttemptNeeded  = true;
  wifiConnectInProgress     = false;
  processingDisconnect.clear();
}

void WiFiEventData_t::markWiFiBegin() {
  markWiFiTurnOn();
  last_wifi_connect_attempt_moment.setNow();
  wifiConnectInProgress  = true;
  usedChannel = 0;
  ++wifi_connect_attempt;
  if (!timerAPstart.isSet()) {
    timerAPstart.setMillisFromNow(WIFI_RECONNECT_WAIT);
  }
}

bool WiFiEventData_t::WiFiDisconnected() const {
  return wifiStatus == ESPEASY_WIFI_DISCONNECTED;
}

bool WiFiEventData_t::WiFiGotIP() const {
  return bitRead(wifiStatus, ESPEASY_WIFI_GOT_IP);
}

bool WiFiEventData_t::WiFiConnected() const {
  return bitRead(wifiStatus, ESPEASY_WIFI_CONNECTED);
}

bool WiFiEventData_t::WiFiServicesInitialized() const {
  return bitRead(wifiStatus, ESPEASY_WIFI_SERVICES_INITIALIZED);
}

void WiFiEventData_t::setWiFiDisconnected() {
  wifiConnectInProgress = false;
  wifiStatus            = ESPEASY_WIFI_DISCONNECTED;
}

void WiFiEventData_t::setWiFiGotIP() {
  bitSet(wifiStatus, ESPEASY_WIFI_GOT_IP);
}

void WiFiEventData_t::setWiFiConnected() {
  bitSet(wifiStatus, ESPEASY_WIFI_CONNECTED);
}

void WiFiEventData_t::setWiFiServicesInitialized() {
  if (!unprocessedWifiEvents() && WiFiConnected() && WiFiGotIP()) {
    addLog(LOG_LEVEL_DEBUG, F("WiFi : WiFi services initialized"));
    bitSet(wifiStatus, ESPEASY_WIFI_SERVICES_INITIALIZED);
    wifiConnectInProgress = false;
  }
}

void WiFiEventData_t::markGotIP() {
  lastGetIPmoment.setNow();

  // Create the 'got IP event' so mark the wifiStatus to not have the got IP flag set
  // This also implies the services are not fully initialized.
  bitClear(wifiStatus, ESPEASY_WIFI_GOT_IP);
  bitClear(wifiStatus, ESPEASY_WIFI_SERVICES_INITIALIZED);
  processedGotIP = false;
}

void WiFiEventData_t::markLostIP() {
  bitClear(wifiStatus, ESPEASY_WIFI_GOT_IP);
  bitClear(wifiStatus, ESPEASY_WIFI_SERVICES_INITIALIZED);
}

void WiFiEventData_t::markDisconnect(WiFiDisconnectReason reason) {
  lastDisconnectMoment.setNow();
  usedChannel = 0;

  if (last_wifi_connect_attempt_moment.isSet() && !lastConnectMoment.isSet()) {
    // There was an unsuccessful connection attempt
    lastConnectedDuration_us = last_wifi_connect_attempt_moment.timeDiff(lastDisconnectMoment);
  } else {
    lastConnectedDuration_us = lastConnectMoment.timeDiff(lastDisconnectMoment);
  }
  lastDisconnectReason = reason;
  processedDisconnect  = false;
  wifiConnectInProgress = false;
}

void WiFiEventData_t::markConnected(const String& ssid, const uint8_t bssid[6], uint8_t channel) {
  usedChannel = channel;
  lastConnectMoment.setNow();
  processedConnect    = false;
  channel_changed     = RTC.lastWiFiChannel != channel;
  last_ssid           = ssid;
  bssid_changed       = false;
  auth_mode           = WiFi_AP_Candidates.getCurrent().enc_type;

  RTC.lastWiFiChannel = channel;
  for (uint8_t i = 0; i < 6; ++i) {
    if (RTC.lastBSSID[i] != bssid[i]) {
      bssid_changed    = true;
      RTC.lastBSSID[i] = bssid[i];
    }
  }
}

void WiFiEventData_t::markConnectedAPmode(const uint8_t mac[6]) {
  lastMacConnectedAPmode = mac;
  processedConnectAPmode = false;
}

void WiFiEventData_t::markDisconnectedAPmode(const uint8_t mac[6]) {
  lastMacDisconnectedAPmode = mac;
  processedDisconnectAPmode = false;
}

void WiFiEventData_t::setAuthMode(uint8_t newMode) {
  auth_mode = newMode;
}


String WiFiEventData_t::ESPeasyWifiStatusToString() const {
  String log;
  if (WiFiDisconnected()) {
    log = F("DISCONNECTED");
  } else {
    if (WiFiConnected()) {
      log += F("Conn. ");
    }
    if (WiFiGotIP()) {
      log += F("IP ");
    }
    if (WiFiServicesInitialized()) {
      log += F("Init");
    }
  }
  return log;
}

#include "../DataStructs/ControllerSettingsStruct.h"

#include "../../ESPEasy_common.h"

#include "../CustomBuild/ESPEasyLimits.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../Helpers/Misc.h"
#include "../Helpers/Networking.h"
#include "../Helpers/StringConverter.h"


#include <IPAddress.h>
#include <WString.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

ControllerSettingsStruct::ControllerSettingsStruct()
{
  reset();
}

void ControllerSettingsStruct::reset() {
  UseDNS                     = DEFAULT_SERVER_USEDNS;
  Port                       = DEFAULT_PORT;
  MinimalTimeBetweenMessages = CONTROLLER_DELAY_QUEUE_DELAY_DFLT;
  MaxQueueDepth              = CONTROLLER_DELAY_QUEUE_DEPTH_DFLT;
  MaxRetry                   = CONTROLLER_DELAY_QUEUE_RETRY_DFLT;
  DeleteOldest               = DEFAULT_CONTROLLER_DELETE_OLDEST;
  ClientTimeout              = CONTROLLER_CLIENTTIMEOUT_DFLT;
  MustCheckReply             = DEFAULT_CONTROLLER_MUST_CHECK_REPLY ;
  SampleSetInitiator         = INVALID_TASK_INDEX;
  VariousFlags               = 0;

  for (uint8_t i = 0; i < 4; ++i) {
    IP[i] = 0;
  }
  ZERO_FILL(HostName);
  ZERO_FILL(ClientID);
  ZERO_FILL(Publish);
  ZERO_FILL(Subscribe);
  ZERO_FILL(MQTTLwtTopic);
  ZERO_FILL(LWTMessageConnect);
  ZERO_FILL(LWTMessageDisconnect);
  safe_strncpy(ClientID, F(CONTROLLER_DEFAULT_CLIENTID), sizeof(ClientID));
}

bool ControllerSettingsStruct::isSet() const {
  if (UseDNS) {
    return HostName[0] != 0;
  }
  return ipSet();
}

void ControllerSettingsStruct::validate() {
  if (Port > 65535) { Port = 0; }

  if ((MinimalTimeBetweenMessages < 1) ||  (MinimalTimeBetweenMessages > CONTROLLER_DELAY_QUEUE_DELAY_MAX)) {
    MinimalTimeBetweenMessages = CONTROLLER_DELAY_QUEUE_DELAY_DFLT;
  }

  if (MaxQueueDepth > CONTROLLER_DELAY_QUEUE_DEPTH_MAX) { MaxQueueDepth = CONTROLLER_DELAY_QUEUE_DEPTH_DFLT; }

  if (MaxRetry > CONTROLLER_DELAY_QUEUE_RETRY_MAX) { MaxRetry = CONTROLLER_DELAY_QUEUE_RETRY_MAX; }

  if (MaxQueueDepth == 0) { MaxQueueDepth = CONTROLLER_DELAY_QUEUE_DEPTH_DFLT; }

  if (MaxRetry == 0) { MaxRetry = CONTROLLER_DELAY_QUEUE_RETRY_DFLT; }

  if ((ClientTimeout < 10) || (ClientTimeout > CONTROLLER_CLIENTTIMEOUT_MAX)) {
    ClientTimeout = CONTROLLER_CLIENTTIMEOUT_DFLT;
  }
  ZERO_TERMINATE(HostName);
  ZERO_TERMINATE(Publish);
  ZERO_TERMINATE(Subscribe);
  ZERO_TERMINATE(MQTTLwtTopic);
  ZERO_TERMINATE(LWTMessageConnect);
  ZERO_TERMINATE(LWTMessageDisconnect);
}

IPAddress ControllerSettingsStruct::getIP() const {
  IPAddress host(IP[0], IP[1], IP[2], IP[3]);

  return host;
}

String ControllerSettingsStruct::getHost() const {
  if (UseDNS) {
    return HostName;
  }
  return getIP().toString();
}

void ControllerSettingsStruct::setHostname(const String& controllerhostname) {
  safe_strncpy(HostName, controllerhostname.c_str(), sizeof(HostName));
  updateIPcache();
}

bool ControllerSettingsStruct::checkHostReachable(bool quick) {
  if (!isSet()) {
    // No IP/hostname set
    return false;
  }
  if (!NetworkConnected(10)) {
    return false; // Not connected, so no use in wasting time to connect to a host.
  }
  delay(0);       // Make sure the Watchdog will not trigger a reset.

  if (quick && ipSet()) { return true; }

  if (UseDNS) {
    if (!updateIPcache()) {
      return false;
    }
  }
  return hostReachable(getIP());
}

bool ControllerSettingsStruct::connectToHost(WiFiClient& client) {
  if (!checkHostReachable(true)) {
    return false; // Host not reachable
  }
  uint8_t retry     = 2;
  bool connected = false;

  while (retry > 0 && !connected) {
    --retry;
    connected = connectClient(client, getIP(), Port, ClientTimeout);

    if (connected) { return true; }

    if (!checkHostReachable(false)) {
      return false;
    }
  }
  return false;
}

bool ControllerSettingsStruct::beginPacket(WiFiUDP& client) {
  if (!checkHostReachable(true)) {
    return false; // Host not reachable
  }
  uint8_t retry     = 2;
  while (retry > 0) {
    --retry;
    FeedSW_watchdog();
    if (client.beginPacket(getIP(), Port) == 1) {
      return true;
    }

    if (!checkHostReachable(false)) {
      return false;
    }
    delay(10);
  }
  return false;
}

String ControllerSettingsStruct::getHostPortString() const {
  String result = getHost();

  result += ':';
  result += Port;
  return result;
}

bool ControllerSettingsStruct::ipSet() const {
  for (uint8_t i = 0; i < 4; ++i) {
    if (IP[i] != 0) { return true; }
  }
  return false;
}

bool ControllerSettingsStruct::updateIPcache() {
  if (!UseDNS) {
    return true;
  }

  if (!NetworkConnected()) { return false; }
  IPAddress tmpIP;

  if (resolveHostByName(HostName, tmpIP, ClientTimeout)) {
    for (uint8_t x = 0; x < 4; x++) {
      IP[x] = tmpIP[x];
    }
    return true;
  }
  return false;
}

bool ControllerSettingsStruct::mqtt_cleanSession() const
{
  return bitRead(VariousFlags, 1);
}

void ControllerSettingsStruct::mqtt_cleanSession(bool value)
{
  bitWrite(VariousFlags, 1, value);
}

bool ControllerSettingsStruct::mqtt_sendLWT() const
{
  return !bitRead(VariousFlags, 2);
}

void ControllerSettingsStruct::mqtt_sendLWT(bool value)
{
  bitWrite(VariousFlags, 2, !value);
}

bool ControllerSettingsStruct::mqtt_willRetain() const
{
  return !bitRead(VariousFlags, 3);
}

void ControllerSettingsStruct::mqtt_willRetain(bool value)
{
  bitWrite(VariousFlags, 3, !value);
}

bool ControllerSettingsStruct::mqtt_uniqueMQTTclientIdReconnect() const
{
  return bitRead(VariousFlags, 4);
}

void ControllerSettingsStruct::mqtt_uniqueMQTTclientIdReconnect(bool value)
{
  bitWrite(VariousFlags, 4, value);
}

bool ControllerSettingsStruct::mqtt_retainFlag() const
{
  return bitRead(VariousFlags, 5);
}

void ControllerSettingsStruct::mqtt_retainFlag(bool value)
{
  bitWrite(VariousFlags, 5, value);
}

bool ControllerSettingsStruct::useExtendedCredentials() const
{
  return bitRead(VariousFlags, 6);
}

void ControllerSettingsStruct::useExtendedCredentials(bool value)
{
  bitWrite(VariousFlags, 6, value);
}

bool ControllerSettingsStruct::sendBinary() const
{
  return bitRead(VariousFlags, 7);
}

void ControllerSettingsStruct::sendBinary(bool value)
{
  bitWrite(VariousFlags, 7, value);
}

bool ControllerSettingsStruct::allowExpire() const
{
  return bitRead(VariousFlags, 9);
}

void ControllerSettingsStruct::allowExpire(bool value)
{
  bitWrite(VariousFlags, 9, value);
}

bool ControllerSettingsStruct::deduplicate() const
{
  return bitRead(VariousFlags, 10);
}

void ControllerSettingsStruct::deduplicate(bool value)
{
  bitWrite(VariousFlags, 10, value);
}

bool ControllerSettingsStruct::useLocalSystemTime() const
{
  return bitRead(VariousFlags, 11);
}

void ControllerSettingsStruct::useLocalSystemTime(bool value)
{
  bitWrite(VariousFlags, 11, value);
}

#include "../DataStructs/PluginTaskData_base.h"

#include "../DataStructs/ESPEasy_EventStruct.h"

#include "../Globals/RuntimeData.h"

#include "../Helpers/StringConverter.h"

#include "../WebServer/Chart_JS.h"
#include "../WebServer/HTML_wrappers.h"

PluginTaskData_base::PluginTaskData_base()
  : _taskdata_pluginID(INVALID_PLUGIN_ID)
#if FEATURE_PLUGIN_STATS
  , _plugin_stats_array(nullptr)
#endif // if FEATURE_PLUGIN_STATS
{}

PluginTaskData_base::~PluginTaskData_base() {
#if FEATURE_PLUGIN_STATS
  delete _plugin_stats_array;
  _plugin_stats_array = nullptr;
#endif // if FEATURE_PLUGIN_STATS
}

bool PluginTaskData_base::hasPluginStats() const
{
#if FEATURE_PLUGIN_STATS

  if (_plugin_stats_array != nullptr) {
    return _plugin_stats_array->hasStats();
  }
#endif // if FEATURE_PLUGIN_STATS
  return false;
}

bool PluginTaskData_base::hasPeaks() const
{
#if FEATURE_PLUGIN_STATS

  if (_plugin_stats_array != nullptr) {
    return _plugin_stats_array->hasPeaks();
  }
#endif // if FEATURE_PLUGIN_STATS
  return false;
}

uint8_t PluginTaskData_base::nrSamplesPresent() const
{
#if FEATURE_PLUGIN_STATS

  if (_plugin_stats_array != nullptr) {
    return _plugin_stats_array->nrSamplesPresent();
  }
#endif // if FEATURE_PLUGIN_STATS
  return 0;
}

#if FEATURE_PLUGIN_STATS
void PluginTaskData_base::initPluginStats(taskVarIndex_t taskVarIndex)
{
  if (taskVarIndex < VARS_PER_TASK) {
    if (_plugin_stats_array == nullptr) {
      _plugin_stats_array = new (std::nothrow) PluginStats_array();
    }

    if (_plugin_stats_array != nullptr) {
      _plugin_stats_array->initPluginStats(taskVarIndex);
    }
  }
}

void PluginTaskData_base::clearPluginStats(taskVarIndex_t taskVarIndex)
{
  if ((taskVarIndex < VARS_PER_TASK) && _plugin_stats_array) {
    _plugin_stats_array->clearPluginStats(taskVarIndex);

    if (!_plugin_stats_array->hasStats()) {
      delete _plugin_stats_array;
      _plugin_stats_array = nullptr;
    }
  }
}

#endif // if FEATURE_PLUGIN_STATS
void PluginTaskData_base::pushPluginStatsValues(struct EventStruct *event, bool trackPeaks)
{
#if FEATURE_PLUGIN_STATS

  if (_plugin_stats_array != nullptr) {
    _plugin_stats_array->pushPluginStatsValues(event, trackPeaks);
  }
#endif // if FEATURE_PLUGIN_STATS
}

bool PluginTaskData_base::plugin_get_config_value_base(struct EventStruct *event,
                                                       String            & string) const
{
#if FEATURE_PLUGIN_STATS

  if (_plugin_stats_array != nullptr) {
    return _plugin_stats_array->plugin_get_config_value_base(event, string);
  }
#endif // if FEATURE_PLUGIN_STATS
  return false;
}

bool PluginTaskData_base::plugin_write_base(struct EventStruct *event, const String& string)
{
#if FEATURE_PLUGIN_STATS

  if (_plugin_stats_array != nullptr) {
    return _plugin_stats_array->plugin_write_base(event, string);
  }
#endif // if FEATURE_PLUGIN_STATS
  return false;
}

#if FEATURE_PLUGIN_STATS
bool PluginTaskData_base::webformLoad_show_stats(struct EventStruct *event) const
{
  if (_plugin_stats_array != nullptr) {
    return _plugin_stats_array->webformLoad_show_stats(event);
  }
  return false;
}

# if FEATURE_CHART_JS
void PluginTaskData_base::plot_ChartJS() const
{
  if (_plugin_stats_array != nullptr) {
    _plugin_stats_array->plot_ChartJS();
  }
}

# endif // if FEATURE_CHART_JS

PluginStats * PluginTaskData_base::getPluginStats(taskVarIndex_t taskVarIndex) const
{
  if (_plugin_stats_array != nullptr) {
    return _plugin_stats_array->getPluginStats(taskVarIndex);
  }
  return nullptr;
}

PluginStats * PluginTaskData_base::getPluginStats(taskVarIndex_t taskVarIndex)
{
  if (_plugin_stats_array != nullptr) {
    return _plugin_stats_array->getPluginStats(taskVarIndex);
  }
  return nullptr;
}

#endif // if FEATURE_PLUGIN_STATS

#include "../DataStructs/Caches.h"

#include "../Globals/Device.h"
#include "../Globals/ExtraTaskSettings.h"
#include "../Globals/Settings.h"
#include "../Globals/WiFi_AP_Candidates.h"

#include "../Helpers/ESPEasy_Storage.h"


#include <ESPeasySerial.h>


void Caches::clearAllCaches()
{
  fileExistsMap.clear();
  clearTaskCaches();
  WiFi_AP_Candidates.clearCache();
  rulesHelper.closeAllFiles();
}

void Caches::clearTaskCaches() {
  taskIndexName.clear();
  taskIndexValueName.clear();
  extraTaskSettings_cache.clear();
  updateActiveTaskUseSerial0();
}

void Caches::updateActiveTaskUseSerial0() {
  activeTaskUseSerial0 = false;

  // Check to see if a task is enabled and using the pins we also use for receiving commands.
  // We're now receiving only from Serial0, so check if an enabled task is also using it.
  for (taskIndex_t task = 0; validTaskIndex(task); ++task)
  {
    const deviceIndex_t DeviceIndex = getDeviceIndex_from_TaskIndex(task);

    if (Settings.TaskDeviceEnabled[task] && validDeviceIndex(DeviceIndex)) {
      if ((Device[DeviceIndex].Type == DEVICE_TYPE_SERIAL) ||
          (Device[DeviceIndex].Type == DEVICE_TYPE_SERIAL_PLUS1)) {
        switch (ESPeasySerialType::getSerialType(
                  ESPEasySerialPort::not_set,
                  Settings.TaskDevicePin1[task],
                  Settings.TaskDevicePin2[task]))
        {
          case ESPEasySerialPort::serial0_swap:
          case ESPEasySerialPort::serial0:
            activeTaskUseSerial0 = true;
          default:
            break;
        }
      }
    }
  }
}

uint8_t Caches::getTaskDeviceValueDecimals(taskIndex_t TaskIndex, uint8_t rel_index)
{
  if (validTaskIndex(TaskIndex) && (rel_index < VARS_PER_TASK)) {
    auto it = getExtraTaskSettings(TaskIndex);

    if (it != extraTaskSettings_cache.end()) {
      return it->second.decimals[rel_index];
    }
  }
  return 0;
}

String Caches::getTaskDeviceName(taskIndex_t TaskIndex)
{
  if (validTaskIndex(TaskIndex)) {
  #ifdef ESP8266

    // ESP8266 uses SPIFFS, which is fast enough to read the task settings
    // Also RAM is limited, so don't waste it on caching where it is not needed.
    LoadTaskSettings(TaskIndex);
    return ExtraTaskSettings.TaskDeviceName;
  #endif // ifdef ESP8266
  #ifdef ESP32

    auto it = getExtraTaskSettings(TaskIndex);

    if (it != extraTaskSettings_cache.end()) {
      return it->second.TaskDeviceName;
    }
  #endif // ifdef ESP32
  }
  return EMPTY_STRING;
}

String Caches::getTaskDeviceValueName(taskIndex_t TaskIndex, uint8_t rel_index)
{
  if (validTaskIndex(TaskIndex) && (rel_index < VARS_PER_TASK)) {
  #ifdef ESP8266

    // ESP8266 uses SPIFFS, which is fast enough to read the task settings
    // Also RAM is limited, so don't waste it on caching where it is not needed.
    LoadTaskSettings(TaskIndex);
    return ExtraTaskSettings.TaskDeviceValueNames[rel_index];
  #endif // ifdef ESP8266
  #ifdef ESP32

    auto it = getExtraTaskSettings(TaskIndex);

    if (it != extraTaskSettings_cache.end()) {
      return it->second.TaskDeviceValueNames[rel_index];
    }
    #endif // ifdef ESP32
  }

  return EMPTY_STRING;
}

String Caches::getTaskDeviceFormula(taskIndex_t TaskIndex, uint8_t rel_index)
{
  if (validTaskIndex(TaskIndex) && (rel_index < VARS_PER_TASK)) {
    {
      // Just a quick test to see if we do have a formula present.
      // Task Formula are not used very often, so we will probably almost always have to return an empty string.
      auto it = getExtraTaskSettings(TaskIndex);

      if (it != extraTaskSettings_cache.end()) {
        if (!it->second.hasFormula) { return EMPTY_STRING; }
      }
    }

    LoadTaskSettings(TaskIndex);
    return ExtraTaskSettings.TaskDeviceFormula[rel_index];
  }
  return EMPTY_STRING;
}

void Caches::updateExtraTaskSettingsCache()
{
  const taskIndex_t TaskIndex = ExtraTaskSettings.TaskIndex;

  if (validTaskIndex(TaskIndex)) {
    auto it = extraTaskSettings_cache.find(TaskIndex);

    if (it != extraTaskSettings_cache.end()) {
      extraTaskSettings_cache.erase(it);
    }

    ExtraTaskSettings_cache_t tmp;
      #ifdef ESP32
    tmp.TaskDeviceName = ExtraTaskSettings.TaskDeviceName;
      #endif // ifdef ESP32

    for (size_t i = 0; i < VARS_PER_TASK; ++i) {
        #ifdef ESP32
      tmp.TaskDeviceValueNames[i] = ExtraTaskSettings.TaskDeviceValueNames[i];
        #endif // ifdef ESP32

      if (ExtraTaskSettings.TaskDeviceFormula[i][0] != 0) {
        tmp.hasFormula = true;
      }
      tmp.decimals[i] = ExtraTaskSettings.TaskDeviceValueDecimals[i];
    }

    extraTaskSettings_cache[TaskIndex] = tmp;
  }
}

ExtraTaskSettingsMap::const_iterator Caches::getExtraTaskSettings(taskIndex_t TaskIndex)
{
  if (validTaskIndex(TaskIndex)) {
    auto it = extraTaskSettings_cache.find(TaskIndex);

    if (it == extraTaskSettings_cache.end()) {
      ExtraTaskSettings.clear(); // Force reload so the cache is filled
      LoadTaskSettings(TaskIndex);
      it = extraTaskSettings_cache.find(TaskIndex);
    }
    return it;
  }
  return extraTaskSettings_cache.end();
}

#include "../DataStructs/ESPEasyControllerCache.h"


ControllerCache_struct::~ControllerCache_struct() {
  if (_RTC_cache_handler != nullptr) {
    delete _RTC_cache_handler;
    _RTC_cache_handler = nullptr;
  }
}

// Write a single sample set to the buffer
bool ControllerCache_struct::write(const uint8_t *data, unsigned int size) {
  if (_RTC_cache_handler == nullptr) {
    return false;
  }
  return _RTC_cache_handler->write(data, size);
}

// Read a single sample set, either from file or buffer.
// May delete a file if it is all read and not written to.
bool ControllerCache_struct::read(uint8_t *data, unsigned int size) {
  return true;
}

// Dump whatever is in the buffer to the filesystem
bool ControllerCache_struct::flush() {
  if (_RTC_cache_handler == nullptr) {
    return false;
  }
  return _RTC_cache_handler->flush();
}

void ControllerCache_struct::init() {
  if (_RTC_cache_handler == nullptr) {
    _RTC_cache_handler = new (std::nothrow) RTC_cache_handler_struct;
  }
}

bool ControllerCache_struct::isInitialized() {
  return _RTC_cache_handler != nullptr;
}

// Clear all caches
void ControllerCache_struct::clearCache() {}

bool ControllerCache_struct::deleteOldestCacheBlock() {
  if (_RTC_cache_handler != nullptr) {
    return _RTC_cache_handler->deleteOldestCacheBlock();
  }
  return false;
}

void ControllerCache_struct::resetpeek() {
  if (_RTC_cache_handler != nullptr) {
    _RTC_cache_handler->resetpeek();
  }
}

// Read data without marking it as being read.
bool ControllerCache_struct::peek(uint8_t *data, unsigned int size) {
  if (_RTC_cache_handler == nullptr) {
    return false;
  }
  return _RTC_cache_handler->peek(data, size);
}

String ControllerCache_struct::getPeekCacheFileName(bool& islast) {
  if (_RTC_cache_handler == nullptr) {
    return "";
  }
  return _RTC_cache_handler->getPeekCacheFileName(islast);
}

#include "../DataStructs/RTC_cache_handler_struct.h"

#include "../../ESPEasy_common.h"
#include "../DataStructs/RTCStruct.h"
#include "../Helpers/CRC_functions.h"
#include "../Helpers/ESPEasy_Storage.h"

#include "../ESPEasyCore/ESPEasy_Log.h"

#ifdef ESP8266
#include <user_interface.h>
#endif

#ifdef ESP32
  #include <soc/rtc.h>

  // For ESP32 the RTC mapped structure may not be a member of an object, 
  // but must be declared 'static'
  // This also means we can only have a single instance of this 
  // RTC_cache_handler_struct.
  RTC_NOINIT_ATTR RTC_cache_struct    RTC_cache;
  RTC_NOINIT_ATTR uint8_t RTC_cache_data[RTC_CACHE_DATA_SIZE];
#endif




/********************************************************************************************\
   RTC located cache
 \*********************************************************************************************/
RTC_cache_handler_struct::RTC_cache_handler_struct() {
  bool success = loadMetaData() && loadData();

  if (!success) {
      #ifdef RTC_STRUCT_DEBUG
    addLog(LOG_LEVEL_INFO, F("RTC  : Error reading cache data"));
      #endif // ifdef RTC_STRUCT_DEBUG
    RTC_cache.init();
    flush();
  } else {
      #ifdef RTC_STRUCT_DEBUG
    rtc_debug_log(F("Read from RTC cache"), RTC_cache.writePos);
      #endif // ifdef RTC_STRUCT_DEBUG
  }
}

unsigned int RTC_cache_handler_struct::getFreeSpace() {
  if (RTC_cache.writePos >= RTC_CACHE_DATA_SIZE) {
    return 0;
  }
  return RTC_CACHE_DATA_SIZE - RTC_cache.writePos;
}

void RTC_cache_handler_struct::resetpeek() {
  if (fp) {
    fp.close();
  }
  peekfilenr  = 0;
  peekreadpos = 0;
}

bool RTC_cache_handler_struct::peek(uint8_t *data, unsigned int size) {
  int retries = 2;

  while (retries > 0) {
    --retries;

    if (!fp) {
      int tmppos;
      String fname;

      if (peekfilenr == 0) {
        fname      = getReadCacheFileName(tmppos);
        peekfilenr = getCacheFileCountFromFilename(fname);
      } else {
        ++peekfilenr;
        fname = createCacheFilename(peekfilenr);
      }

      if (fname.isEmpty()) { return false; }
      fp = tryOpenFile(fname, "r");
    }

    if (!fp) { return false; }

    if (fp.read(data, size)) {
      return true;
    }
    fp.close();
  }
  return true;
}

// Write a single sample set to the buffer
bool RTC_cache_handler_struct::write(const uint8_t *data, unsigned int size) {
    #ifdef RTC_STRUCT_DEBUG
  rtc_debug_log(F("write RTC cache data"), size);
    #endif // ifdef RTC_STRUCT_DEBUG

  if (getFreeSpace() < size) {
    if (!flush()) {
      return false;
    }
  }

  // First store it in the buffer
  for (unsigned int i = 0; i < size; ++i) {
    RTC_cache_data[RTC_cache.writePos] = data[i];
    ++RTC_cache.writePos;
  }

  // Now store the updated part of the buffer to the RTC memory.
  // Pad some extra bytes around it to allow sample sizes not multiple of 4 bytes.
  int startOffset = RTC_cache.writePos - size;
  startOffset -= startOffset % 4;

  if (startOffset < 0) {
    startOffset = 0;
  }
  int nrBytes = RTC_cache.writePos - startOffset;

  if (nrBytes % 4 != 0) {
    nrBytes -= nrBytes % 4;
    nrBytes += 4;
  }

  if ((nrBytes + startOffset) >  RTC_CACHE_DATA_SIZE) {
    // Can this happen?
    nrBytes = RTC_CACHE_DATA_SIZE - startOffset;
  }
  return saveRTCcache(startOffset, nrBytes);
}

// Mark all content as being processed and empty buffer.
bool RTC_cache_handler_struct::flush() {
  if (prepareFileForWrite()) {
    if (RTC_cache.writePos > 0) {
      #ifdef RTC_STRUCT_DEBUG
      size_t filesize    = fw.size();
      #endif
      int    bytesWriten = fw.write(&RTC_cache_data[0], RTC_cache.writePos);

      delay(0);
      fw.flush();
        #ifdef RTC_STRUCT_DEBUG
      addLog(LOG_LEVEL_INFO, F("RTC  : flush RTC cache"));
        #endif // ifdef RTC_STRUCT_DEBUG


      if ((bytesWriten < RTC_cache.writePos) /*|| (fw.size() == filesize)*/) {
          #ifdef RTC_STRUCT_DEBUG
          if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
            String log = F("RTC  : error writing file. Size before: ");
            log += filesize;
            log += F(" after: ");
            log += fw.size();
            log += F(" writen: ");
            log += bytesWriten;
            addLogMove(LOG_LEVEL_ERROR, log);
          }
          #endif // ifdef RTC_STRUCT_DEBUG
        fw.close();

        if (!GarbageCollection()) {
          // Garbage collection was not able to remove anything
          writeerror = true;
        }
        return false;
      }
      initRTCcache_data();
      clearRTCcacheData();
      saveRTCcache();
      return true;
    }
  }
  return false;
}

// Return usable filename for reading.
// Will be empty if there is no file to process.
String RTC_cache_handler_struct::getReadCacheFileName(int& readPos) {
  initRTCcache_data();

  for (int i = 0; i < 2; ++i) {
    String fname = createCacheFilename(RTC_cache.readFileNr);

    if (fileExists(fname)) {
      if (i != 0) {
        // First attempt failed, so stored read position is not valid
        RTC_cache.readPos = 0;
      }
      readPos = RTC_cache.readPos;
      return fname;
    }

    if (i == 0) {
      updateRTC_filenameCounters();
    }
  }

  // No file found
  RTC_cache.readPos = 0;
  readPos           = RTC_cache.readPos;
  return "";
}

String RTC_cache_handler_struct::getPeekCacheFileName(bool& islast) {
  int tmppos;
  String fname;

  if (peekfilenr == 0) {
    fname      = getReadCacheFileName(tmppos);
    peekfilenr = getCacheFileCountFromFilename(fname);
  } else {
    ++peekfilenr;
    fname = createCacheFilename(peekfilenr);
  }
  islast = peekfilenr > RTC_cache.writeFileNr;

  if (fileExists(fname)) {
    return fname;
  }
  return "";
}

bool RTC_cache_handler_struct::deleteOldestCacheBlock() {
  if (updateRTC_filenameCounters()) {
    const int nrCacheFiles = RTC_cache.writeFileNr - RTC_cache.readFileNr;

    if (nrCacheFiles > 1) {
      // read and write file nr are not the same file, remove the read file nr.
      String fname = createCacheFilename(RTC_cache.readFileNr);

      writeerror = false;
      if (tryDeleteFile(fname)) {
          #ifdef RTC_STRUCT_DEBUG
          if (loglevelActiveFor(LOG_LEVEL_INFO)) {
            String log = F("RTC  : Removed file from FS: ");
            log += fname;
            addLogMove(LOG_LEVEL_INFO, log);
          }
          #endif // ifdef RTC_STRUCT_DEBUG
        updateRTC_filenameCounters();
        return true;
      }
    }
  }
  return false;
}

bool RTC_cache_handler_struct::loadMetaData()
{
  // No need to load on ESP32, as the data is already allocated to the RTC memory by the compiler

  #ifdef ESP8266
  if (!system_rtc_mem_read(RTC_BASE_CACHE, reinterpret_cast<uint8_t *>(&RTC_cache), sizeof(RTC_cache))) {
    return false;
  }
  #endif

  return RTC_cache.checksumMetadata == calc_CRC32(reinterpret_cast<const uint8_t *>(&RTC_cache), sizeof(RTC_cache) - sizeof(uint32_t));
}

bool RTC_cache_handler_struct::loadData()
{
  initRTCcache_data();

  // No need to load on ESP32, as the data is already allocated to the RTC memory by the compiler
  #ifdef ESP8266
  if (!system_rtc_mem_read(RTC_BASE_CACHE + (sizeof(RTC_cache) / 4), reinterpret_cast<uint8_t *>(&RTC_cache_data[0]), RTC_CACHE_DATA_SIZE)) {
    return false;
  }
  #endif

  if (RTC_cache.checksumData != getDataChecksum()) {
        # ifdef RTC_STRUCT_DEBUG
    addLog(LOG_LEVEL_ERROR, F("RTC  : Checksum error reading RTC cache data"));
        # endif // ifdef RTC_STRUCT_DEBUG
    return false;
  }
  return RTC_cache.checksumData == getDataChecksum();
}

bool RTC_cache_handler_struct::saveRTCcache() {
  return saveRTCcache(0, RTC_CACHE_DATA_SIZE);
}

bool RTC_cache_handler_struct::saveRTCcache(unsigned int startOffset, size_t nrBytes)
{
  RTC_cache.checksumData     = getDataChecksum();
  RTC_cache.checksumMetadata = calc_CRC32(reinterpret_cast<const uint8_t *>(&RTC_cache), sizeof(RTC_cache) - sizeof(uint32_t));
  #ifdef ESP32
  return true;
  #endif

  #ifdef ESP8266
  if (!system_rtc_mem_write(RTC_BASE_CACHE, reinterpret_cast<const uint8_t *>(&RTC_cache), sizeof(RTC_cache)) || !loadMetaData())
  {
        # ifdef RTC_STRUCT_DEBUG
    addLog(LOG_LEVEL_ERROR, F("RTC  : Error while writing cache metadata to RTC"));
        # endif // ifdef RTC_STRUCT_DEBUG
    return false;
  }
  delay(0);

  if (nrBytes > 0) { // Check needed?
    const size_t address = RTC_BASE_CACHE + ((sizeof(RTC_cache) + startOffset) / 4);

    if (!system_rtc_mem_write(address, reinterpret_cast<const uint8_t *>(&RTC_cache_data[startOffset]), nrBytes))
    {
          # ifdef RTC_STRUCT_DEBUG
      addLog(LOG_LEVEL_ERROR, F("RTC  : Error while writing cache data to RTC"));
          # endif // ifdef RTC_STRUCT_DEBUG
      return false;
    }
        # ifdef RTC_STRUCT_DEBUG
    rtc_debug_log(F("Write cache data to RTC"), nrBytes);
        # endif // ifdef RTC_STRUCT_DEBUG
  }
  return true;
  #endif
}

uint32_t RTC_cache_handler_struct::getDataChecksum() {
  initRTCcache_data();
  /*
  size_t dataLength = RTC_cache.writePos;

  if (dataLength > RTC_CACHE_DATA_SIZE) {
    // Is this allowed to happen?
    dataLength = RTC_CACHE_DATA_SIZE;
  }
  */

  // Only compute the checksum over the number of samples stored.
  return calc_CRC32(reinterpret_cast<const uint8_t *>(&RTC_cache_data[0]), /*dataLength*/ RTC_CACHE_DATA_SIZE);
}

void RTC_cache_handler_struct::initRTCcache_data() {
  #ifdef ESP8266
  if (RTC_cache_data.size() != RTC_CACHE_DATA_SIZE) {
    RTC_cache_data.resize(RTC_CACHE_DATA_SIZE);
  }
  #endif

  if (RTC_cache.writeFileNr == 0) {
    // RTC value not reliable
    updateRTC_filenameCounters();
  }
}

void RTC_cache_handler_struct::clearRTCcacheData() {
  for (size_t i = 0; i < RTC_CACHE_DATA_SIZE; ++i) {
    RTC_cache_data[i] = 0;
  }
  RTC_cache.writePos = 0;
}

// Return true if any cache file found
bool RTC_cache_handler_struct::updateRTC_filenameCounters() {
  size_t filesizeHighest;

  if (getCacheFileCounters(RTC_cache.readFileNr, RTC_cache.writeFileNr, filesizeHighest)) {
    if (filesizeHighest >= CACHE_FILE_MAX_SIZE) {
      // Start new file
      ++RTC_cache.writeFileNr;
    }
    return true;
  } else {
    // Do not use 0, since that will be the cleared content of the struct, indicating invalid RTC data.
    RTC_cache.writeFileNr = 1;
  }
  return false;
}

bool RTC_cache_handler_struct::prepareFileForWrite() {
  //    if (storageLocation != CACHE_STORAGE_SPIFFS) {
  //      return false;
  //    }
  if (SpiffsFull()) {
      #ifdef RTC_STRUCT_DEBUG
    addLog(LOG_LEVEL_ERROR, F("RTC  : FS full"));
      #endif // ifdef RTC_STRUCT_DEBUG
    return false;
  }
  unsigned int retries = 3;

  while (retries > 0) {
    --retries;

    if (fw && (fw.size() >= CACHE_FILE_MAX_SIZE)) {
      fw.close();
      GarbageCollection();
    }

    if (!fw) {
      // Open file to write
      initRTCcache_data();

      if (updateRTC_filenameCounters()) {
        if (writeerror || (SpiffsFreeSpace() < ((2 * CACHE_FILE_MAX_SIZE) + SpiffsBlocksize()))) {
          // Not enough room for another file, remove the oldest one.
          deleteOldestCacheBlock();
        }
      }

      String fname = createCacheFilename(RTC_cache.writeFileNr);
      fw = tryOpenFile(fname, "a+");

      if (!fw) {
          #ifdef RTC_STRUCT_DEBUG
        addLog(LOG_LEVEL_ERROR, F("RTC  : error opening file"));
          #endif // ifdef RTC_STRUCT_DEBUG
      } else {
          #ifdef RTC_STRUCT_DEBUG

        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log = F("Write to ");
          log += fname;
          log += F(" size");
          rtc_debug_log(log, fw.size());
        }
          #endif // ifdef RTC_STRUCT_DEBUG
      }
    }
    delay(0);

    if (fw && (fw.size() < CACHE_FILE_MAX_SIZE)) {
      return true;
    }
  }
    #ifdef RTC_STRUCT_DEBUG
  addLog(LOG_LEVEL_ERROR, F("RTC  : prepareFileForWrite failed"));
    #endif // ifdef RTC_STRUCT_DEBUG
  return false;
}

#ifdef RTC_STRUCT_DEBUG
void RTC_cache_handler_struct::rtc_debug_log(const String& description, size_t nrBytes) {
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log;
    if (log.reserve(18 + description.length())) {
      log  = F("RTC  : ");
      log += description;
      log += ' ';
      log += nrBytes;
      log += F(" bytes");
      addLogMove(LOG_LEVEL_INFO, log);
    }
  }
}

#endif // ifdef RTC_STRUCT_DEBUG

#include "../DataStructs/NotificationSettingsStruct.h"

#if FEATURE_NOTIFIER

NotificationSettingsStruct::NotificationSettingsStruct() : Port(0), Pin1(-1), Pin2(-1) {
    ZERO_FILL(Server);
    ZERO_FILL(Domain);
    ZERO_FILL(Sender);
    ZERO_FILL(Receiver);
    ZERO_FILL(Subject);
    ZERO_FILL(Body);
    ZERO_FILL(User);
    ZERO_FILL(Pass);
  }

  void NotificationSettingsStruct::validate() {
    ZERO_TERMINATE(Server);
    ZERO_TERMINATE(Domain);
    ZERO_TERMINATE(Sender);
    ZERO_TERMINATE(Receiver);
    ZERO_TERMINATE(Subject);
    ZERO_TERMINATE(Body);
    ZERO_TERMINATE(User);
    ZERO_TERMINATE(Pass);
  }

#endif
#include "../DataStructs/DeviceStruct.h"

DeviceStruct::DeviceStruct() :
  Number(0), Type(0), VType(Sensor_VType::SENSOR_TYPE_NONE), Ports(0), ValueCount(0),
  OutputDataType(Output_Data_type_t::Default),
  PullUpOption(false), InverseLogicOption(false), FormulaOption(false),
  Custom(false), SendDataOption(false), GlobalSyncOption(false),
  TimerOption(false), TimerOptional(false), DecimalsOnly(false),
  ExitTaskBeforeSave(true), ErrorStateValues(false), 
  PluginStats(false), PluginLogsPeaks(false) {}

bool DeviceStruct::connectedToGPIOpins() const {
  switch(Type) {
    case DEVICE_TYPE_SINGLE:  // Single GPIO
    case DEVICE_TYPE_SPI:
    case DEVICE_TYPE_CUSTOM1:

    case DEVICE_TYPE_DUAL:    // Dual GPIOs
    case DEVICE_TYPE_SERIAL:
    case DEVICE_TYPE_SPI2:
    case DEVICE_TYPE_CUSTOM2:

    case DEVICE_TYPE_TRIPLE:  // Triple GPIOs
    case DEVICE_TYPE_SERIAL_PLUS1:
    case DEVICE_TYPE_SPI3:
    case DEVICE_TYPE_CUSTOM3:    
      return true;
    default:
      return false;
  }
}

bool DeviceStruct::usesTaskDevicePin(int pin) const {
  switch (pin) {
    case 1:
      return connectedToGPIOpins();
    case 2:
      return connectedToGPIOpins() && 
            !(Type == DEVICE_TYPE_SINGLE  ||
              Type == DEVICE_TYPE_SPI ||
              Type == DEVICE_TYPE_CUSTOM1);
    case 3:
      return Type == DEVICE_TYPE_TRIPLE || 
             Type == DEVICE_TYPE_SERIAL_PLUS1 || 
             Type == DEVICE_TYPE_SPI3 ||
             Type == DEVICE_TYPE_CUSTOM3;
  }
  return false;
}


bool DeviceStruct::configurableDecimals() const {
  return FormulaOption || DecimalsOnly;
}

bool DeviceStruct::isSerial() const {
  return (Type == DEVICE_TYPE_SERIAL) || 
         (Type == DEVICE_TYPE_SERIAL_PLUS1);
}

bool DeviceStruct::isSPI() const {
  return (Type == DEVICE_TYPE_SPI) || 
         (Type == DEVICE_TYPE_SPI2) || 
         (Type == DEVICE_TYPE_SPI3);
}

bool DeviceStruct::isCustom() const {
  return (Type == DEVICE_TYPE_CUSTOM0) || 
         (Type == DEVICE_TYPE_CUSTOM1) || 
         (Type == DEVICE_TYPE_CUSTOM2) || 
         (Type == DEVICE_TYPE_CUSTOM3);
}
#include "../DataStructs/PluginStats.h"

#if FEATURE_PLUGIN_STATS
# include "../../_Plugin_Helper.h"

# include "../Helpers/ESPEasy_math.h"

# include "../WebServer/Chart_JS.h"

PluginStats::PluginStats(uint8_t nrDecimals, float errorValue) :
  _errorValue(errorValue),
  _nrDecimals(nrDecimals)

{
  resetPeaks();
}

bool PluginStats::push(float value)
{
  return _samples.push(value);
}

void PluginStats::trackPeak(float value)
{
  if (value > _maxValue) { _maxValue = value; }

  if (value < _minValue) { _minValue = value; }
}

void PluginStats::resetPeaks()
{
  _minValue = std::numeric_limits<float>::max();
  _maxValue = std::numeric_limits<float>::min();
}

float PluginStats::getSampleAvg(PluginStatsBuffer_t::index_t lastNrSamples) const
{
  if (_samples.size() == 0) { return _errorValue; }
  float sum = 0.0f;

  PluginStatsBuffer_t::index_t i = 0;

  if (lastNrSamples < _samples.size()) {
    i = _samples.size() - lastNrSamples;
  }
  PluginStatsBuffer_t::index_t samplesUsed = 0;

  const bool errorValueIsNaN = isnan(_errorValue);

  for (; i < _samples.size(); ++i) {
    if (!isnan(_samples[i])) {
      if (errorValueIsNaN || !essentiallyEqual(_errorValue, _samples[i])) {
        ++samplesUsed;
        sum += _samples[i];
      }
    }
  }

  if (samplesUsed == 0) { return _errorValue; }
  return sum / samplesUsed;
}

float PluginStats::operator[](PluginStatsBuffer_t::index_t index) const
{
  if (index < _samples.size()) { return _samples[index]; }
  return _errorValue;
}

bool PluginStats::plugin_get_config_value_base(struct EventStruct *event, String& string) const
{
  bool success = false;

  // Full value name is something like "taskvaluename.avg"
  const String fullValueName = parseString(string, 1);
  const String command       = parseString(fullValueName, 2, '.');

  float value;

  if (command == F("min")) {        // [taskname#valuename.min] Lowest value seen since value reset
    value   = getPeakLow();
    success = true;
  } else if (command == F("max")) { // [taskname#valuename.max] Highest value seen since value reset
    value   = getPeakHigh();
    success = true;
  } else if (command.startsWith(F("avg"))) {
    if (command == F("avg")) { // [taskname#valuename.avg] Average value of the last N kept samples
      value   = getSampleAvg();
      success = true;
    } else {
      // Check for "avgN", where N is the number of most recent samples to use.
      int nrSamples = 0;

      if (validIntFromString(command.substring(3), nrSamples)) {
        if (nrSamples > 0) {
          // [taskname#valuename.avgN] Average over N most recent samples
          value   = getSampleAvg(nrSamples);
          success = true;
        }
      }
    }
  }

  if (success) {
    string = toString(value, _nrDecimals);
  }
  return success;
}

bool PluginStats::webformLoad_show_stats(struct EventStruct *event) const
{
  bool somethingAdded = false;

  if (webformLoad_show_avg(event)) { somethingAdded = true; }

  if (webformLoad_show_peaks(event)) { somethingAdded = true; }

  if (somethingAdded) {
    addFormSeparator(4);
  }

  return somethingAdded;
}

bool PluginStats::webformLoad_show_avg(struct EventStruct *event) const
{
  if (getNrSamples() > 0) {
    addRowLabel(getLabel() +  F(" Average"));
    addHtmlFloat(getSampleAvg(), _nrDecimals);
    addHtml(' ', '(');
    addHtmlInt(getNrSamples());
    addHtml(F(" samples)"));
    return true;
  }
  return false;
}

bool PluginStats::webformLoad_show_peaks(struct EventStruct *event) const
{
  if (hasPeaks()) {
    addRowLabel(getLabel() +  F(" Peak Low/High"));
    addHtmlFloat(getPeakLow(), _nrDecimals);
    addHtml('/');
    addHtmlFloat(getPeakHigh(), _nrDecimals);
    return true;
  }
  return false;
}

# if FEATURE_CHART_JS
void PluginStats::plot_ChartJS_dataset() const
{
  add_ChartJS_dataset_header(getLabel(), _ChartJS_dataset_config.color);

  PluginStatsBuffer_t::index_t i = 0;

  for (; i < _samples.size(); ++i) {
    if (i != 0) {
      addHtml(',');
    }

    if (!isnan(_samples[i])) {
      addHtmlFloat(_samples[i], _nrDecimals);
    }
    else {
      addHtml(F("null"));
    }
  }
  add_ChartJS_dataset_footer(_ChartJS_dataset_config.hidden);
}

# endif // if FEATURE_CHART_JS

PluginStats_array::PluginStats_array()
{
  for (size_t i = 0; i < VARS_PER_TASK; ++i) {
    _plugin_stats[i] = nullptr;
  }
}

PluginStats_array::~PluginStats_array()
{
  for (size_t i = 0; i < VARS_PER_TASK; ++i) {
    if (_plugin_stats[i] != nullptr) {
      delete _plugin_stats[i];
      _plugin_stats[i] = nullptr;
    }
  }
}

void PluginStats_array::initPluginStats(taskVarIndex_t taskVarIndex)
{
  if (taskVarIndex < VARS_PER_TASK) {
    delete _plugin_stats[taskVarIndex];
    _plugin_stats[taskVarIndex] = nullptr;

    if (ExtraTaskSettings.enabledPluginStats(taskVarIndex)) {
      _plugin_stats[taskVarIndex] = new (std::nothrow) PluginStats(
        ExtraTaskSettings.TaskDeviceValueDecimals[taskVarIndex],
        ExtraTaskSettings.TaskDeviceErrorValue[taskVarIndex]);

      if (_plugin_stats[taskVarIndex] != nullptr) {
        _plugin_stats[taskVarIndex]->setLabel(ExtraTaskSettings.TaskDeviceValueNames[taskVarIndex]);
        # if FEATURE_CHART_JS
        const __FlashStringHelper *colors[] = { F("#A52422"), F("#BEA57D"), F("#EFF2C0"), F("#A4BAB7") };
        _plugin_stats[taskVarIndex]->_ChartJS_dataset_config.color = colors[taskVarIndex];
        # endif // if FEATURE_CHART_JS
      }
    }
  }
}

void PluginStats_array::clearPluginStats(taskVarIndex_t taskVarIndex)
{
  if (taskVarIndex < VARS_PER_TASK) {
    if (_plugin_stats[taskVarIndex] != nullptr) {
      delete _plugin_stats[taskVarIndex];
      _plugin_stats[taskVarIndex] = nullptr;
    }
  }
}

bool PluginStats_array::hasStats() const
{
  for (size_t i = 0; i < VARS_PER_TASK; ++i) {
    if (_plugin_stats[i] != nullptr) { return true; }
  }
  return false;
}

bool PluginStats_array::hasPeaks() const
{
  for (size_t i = 0; i < VARS_PER_TASK; ++i) {
    if ((_plugin_stats[i] != nullptr) && _plugin_stats[i]->hasPeaks()) {
      return true;
    }
  }
  return false;
}

uint8_t PluginStats_array::nrSamplesPresent() const
{
  for (size_t i = 0; i < VARS_PER_TASK; ++i) {
    if (_plugin_stats[i] != nullptr) {
      return _plugin_stats[i]->getNrSamples();
    }
  }
  return 0;
}

void PluginStats_array::pushPluginStatsValues(struct EventStruct *event, bool trackPeaks)
{
  for (size_t i = 0; i < VARS_PER_TASK; ++i) {
    if (_plugin_stats[i] != nullptr) {
      _plugin_stats[i]->push(UserVar[event->BaseVarIndex + i]);

      if (trackPeaks) {
        _plugin_stats[i]->trackPeak(UserVar[event->BaseVarIndex + i]);
      }
    }
  }
}

bool PluginStats_array::plugin_get_config_value_base(struct EventStruct *event,
                                                     String            & string) const
{
  // Full value name is something like "taskvaluename.avg"
  const String fullValueName = parseString(string, 1);
  const String valueName     = parseString(fullValueName, 1, '.');

  for (uint8_t i = 0; i < VARS_PER_TASK; i++)
  {
    if (_plugin_stats[i] != nullptr) {
      // Check case insensitive, since the user entered value name can have any case.
      if (valueName.equalsIgnoreCase(getTaskValueName(event->TaskIndex, i)))
      {
        return _plugin_stats[i]->plugin_get_config_value_base(event, string);
      }
    }
  }
  return false;
}

bool PluginStats_array::plugin_write_base(struct EventStruct *event, const String& string)
{
  bool success     = false;
  const String cmd = parseString(string, 1);               // command

  const bool resetPeaks   = cmd.equals(F("resetpeaks"));   // Command: "taskname.resetPeaks"
  const bool clearSamples = cmd.equals(F("clearsamples")); // Command: "taskname.clearSamples"

  if (resetPeaks || clearSamples) {
    success = true;

    for (size_t i = 0; i < VARS_PER_TASK; ++i) {
      if (_plugin_stats[i] != nullptr) {
        if (resetPeaks) {
          _plugin_stats[i]->resetPeaks();
        }

        if (clearSamples) {
          _plugin_stats[i]->clearSamples();
        }
      }
    }
  }
  return success;
}

bool PluginStats_array::webformLoad_show_stats(struct EventStruct *event) const
{
  bool somethingAdded = false;

  for (size_t i = 0; i < VARS_PER_TASK; ++i) {
    if (_plugin_stats[i] != nullptr) {
      if (_plugin_stats[i]->webformLoad_show_stats(event)) { somethingAdded = true; }
    }
  }
  return somethingAdded;
}

# if FEATURE_CHART_JS
void PluginStats_array::plot_ChartJS() const
{
  const uint8_t nrSamples = nrSamplesPresent();

  if (nrSamples == 0) { return; }

  // Chart Header
  add_ChartJS_chart_header(F("line"), F("TaskStatsChart"), F(""), 500, 500);

  // Add labels
  for (size_t i = 0; i < nrSamples; ++i) {
    if (i != 0) {
      addHtml(',');
    }
    addHtmlInt(i);
  }
  addHtml(F("],datasets: ["));


  // Data sets
  for (size_t i = 0; i < VARS_PER_TASK; ++i) {
    if (_plugin_stats[i] != nullptr) {
      _plugin_stats[i]->plot_ChartJS_dataset();
    }
  }
  add_ChartJS_chart_footer();
}

# endif // if FEATURE_CHART_JS


PluginStats * PluginStats_array::getPluginStats(taskVarIndex_t taskVarIndex) const
{
  if ((taskVarIndex < VARS_PER_TASK)) {
    return _plugin_stats[taskVarIndex];
  }
  return nullptr;
}

PluginStats * PluginStats_array::getPluginStats(taskVarIndex_t taskVarIndex)
{
  if ((taskVarIndex < VARS_PER_TASK)) {
    return _plugin_stats[taskVarIndex];
  }
  return nullptr;
}

#endif // if FEATURE_PLUGIN_STATS

#include "../DataStructs/EthernetEventData.h"

#if FEATURE_ETHERNET

#include "../ESPEasyCore/ESPEasy_Log.h"

// Bit numbers for Eth status
#define ESPEASY_ETH_CONNECTED               0
#define ESPEASY_ETH_GOT_IP                  1
#define ESPEASY_ETH_SERVICES_INITIALIZED    2

bool EthernetEventData_t::EthConnectAllowed() const {
  if (!ethConnectAttemptNeeded) return false;
  if (last_eth_connect_attempt_moment.isSet()) {
    // TODO TD-er: Make this time more dynamic.
    if (!last_eth_connect_attempt_moment.timeoutReached(10000)) {
      return false;
    }
  }
  return true;
}

bool EthernetEventData_t::unprocessedEthEvents() const {
  if (processedConnect && processedDisconnect && processedGotIP && processedDHCPTimeout)
  {
    return false;
  }
  return true;
}

void EthernetEventData_t::clearAll() {
  lastDisconnectMoment.clear();
  lastConnectMoment.clear();
  lastGetIPmoment.clear();
  last_eth_connect_attempt_moment.clear();

  lastEthResetMoment.setNow();
  eth_considered_stable = false;

  // Mark all flags to default to prevent handling old events.
  processedConnect          = true;
  processedDisconnect       = true;
  processedGotIP            = true;
  processedDHCPTimeout      = true;
  ethConnectAttemptNeeded  = true;
}

void EthernetEventData_t::markEthBegin() {
  lastDisconnectMoment.clear();
  lastConnectMoment.clear();
  lastGetIPmoment.clear();
  last_eth_connect_attempt_moment.setNow();
  eth_considered_stable = false;
  ethConnectInProgress  = true;
  ++eth_connect_attempt;
}

bool EthernetEventData_t::EthDisconnected() const {
  return ethStatus == ESPEASY_ETH_DISCONNECTED;
}

bool EthernetEventData_t::EthGotIP() const {

  return bitRead(ethStatus, ESPEASY_ETH_GOT_IP);
}

bool EthernetEventData_t::EthConnected() const {
  return bitRead(ethStatus, ESPEASY_ETH_CONNECTED);
}

bool EthernetEventData_t::EthServicesInitialized() const {
  return bitRead(ethStatus, ESPEASY_ETH_SERVICES_INITIALIZED);
}

void EthernetEventData_t::setEthDisconnected() {
  processedConnect          = true;
  processedDisconnect       = true;
  processedGotIP            = true;
  processedDHCPTimeout      = true;

  ethStatus = ESPEASY_ETH_DISCONNECTED;
}

void EthernetEventData_t::setEthGotIP() {
  bitSet(ethStatus, ESPEASY_ETH_GOT_IP);
  setEthServicesInitialized();
}

void EthernetEventData_t::setEthConnected() {
  bitSet(ethStatus, ESPEASY_ETH_CONNECTED);
  setEthServicesInitialized();
}

void EthernetEventData_t::setEthServicesInitialized() {
  if (!unprocessedEthEvents() && !EthServicesInitialized()) {
    if (EthGotIP() && EthConnected()) {
      #ifndef BUILD_NO_DEBUG
      addLog(LOG_LEVEL_DEBUG, F("Eth : Eth services initialized"));
      #endif
      bitSet(ethStatus, ESPEASY_ETH_SERVICES_INITIALIZED);
      ethConnectInProgress = false;
    }
  }
}

void EthernetEventData_t::markGotIP() {
  lastGetIPmoment.setNow();

  // Create the 'got IP event' so mark the ethStatus to not have the got IP flag set
  // This also implies the services are not fully initialized.
  bitClear(ethStatus, ESPEASY_ETH_GOT_IP);
  bitClear(ethStatus, ESPEASY_ETH_SERVICES_INITIALIZED);
  processedGotIP = false;
}

void EthernetEventData_t::markLostIP() {
  bitClear(ethStatus, ESPEASY_ETH_GOT_IP);
  bitClear(ethStatus, ESPEASY_ETH_SERVICES_INITIALIZED);
  lastGetIPmoment.clear();
}

void EthernetEventData_t::markDisconnect() {
  lastDisconnectMoment.setNow();

  if (last_eth_connect_attempt_moment.isSet() && !lastConnectMoment.isSet()) {
    // There was an unsuccessful connection attempt
    lastConnectedDuration_us = last_eth_connect_attempt_moment.timeDiff(lastDisconnectMoment);
  } else {
    lastConnectedDuration_us = lastConnectMoment.timeDiff(lastDisconnectMoment);
  }
  lastConnectMoment.clear();
  processedDisconnect  = false;
}

void EthernetEventData_t::markConnected() {
  lastConnectMoment.setNow();
  processedConnect    = false;
}

String EthernetEventData_t::ESPEasyEthStatusToString() const {
  String log;
  if (EthDisconnected()) {
    log = F("DISCONNECTED");
  } else {
    if (EthConnected()) {
      log += F("Conn. ");
    }
    if (EthGotIP()) {
      log += F("IP ");
    }
    if (EthServicesInitialized()) {
      log += F("Init");
    }
  }
  return log;

}

#endif
#include "../DataStructs/Web_StreamingBuffer.h"

#include "../DataStructs/tcp_cleanup.h"
#include "../DataTypes/ESPEasyTimeSource.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"

// FIXME TD-er: Should keep a pointer to the webserver as a member, not use the global defined one.
#include "../Globals/Services.h"

#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/Convert.h"
#include "../Helpers/StringConverter.h"

#include "../../ESPEasy_common.h"

#ifdef ESP8266
#define CHUNKED_BUFFER_SIZE         512
#else 
#define CHUNKED_BUFFER_SIZE         1360
#endif

Web_StreamingBuffer::Web_StreamingBuffer(void) : lowMemorySkip(false),
  initialRam(0), beforeTXRam(0), duringTXRam(0), finalRam(0), maxCoreUsage(0),
  maxServerUsage(0), sentBytes(0), flashStringCalls(0), flashStringData(0)
{
  buf.reserve(CHUNKED_BUFFER_SIZE + 50);
  buf.clear();
}

Web_StreamingBuffer& Web_StreamingBuffer::operator+=(char a)                   {
  if (this->buf.length() >= CHUNKED_BUFFER_SIZE) {
    flush();
  }
  this->buf += a;
  return *this;
}

Web_StreamingBuffer& Web_StreamingBuffer::operator+=(uint64_t a) {
  return addString(ull2String(a));
}

Web_StreamingBuffer& Web_StreamingBuffer::operator+=(int64_t a) {
  return addString(ll2String(a));
}

Web_StreamingBuffer& Web_StreamingBuffer::operator+=(const float& a)           {
  return addString(toString(a, 2));
}

Web_StreamingBuffer& Web_StreamingBuffer::operator+=(const double& a)          {
  return addString(doubleToString(a));
}

Web_StreamingBuffer& Web_StreamingBuffer::operator+=(const String& a)          {
  return addString(a);
}

Web_StreamingBuffer& Web_StreamingBuffer::operator+=(PGM_P str) {
  return addFlashString(str);
}

Web_StreamingBuffer& Web_StreamingBuffer::operator+=(const __FlashStringHelper* str) {
  return addFlashString((PGM_P)str);
}

Web_StreamingBuffer& Web_StreamingBuffer::addFlashString(PGM_P str) {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif


  if (!str) { 
    return *this; // return if the pointer is void
  }

  #ifdef USE_SECOND_HEAP
  if (mmu_is_iram(str)) {
    // Have to copy the string using mmu_get functions
    // This is not a flash string.
    bool done = false;
    const char* cur_char = str;
    while (!done) {
      const uint8_t ch = mmu_get_uint8(cur_char++);
      if (ch == 0) return *this;
      if (this->buf.length() >= CHUNKED_BUFFER_SIZE) {
        flush();
      }
      this->buf += (char)ch;
    }
  }
  #endif

  ++flashStringCalls;

  if (lowMemorySkip) { return *this; }
  const unsigned int length = strlen_P((PGM_P)str);

  if (length == 0) { return *this; }
  flashStringData += length;

  checkFull();

  int flush_step = CHUNKED_BUFFER_SIZE - this->buf.length();
  if (flush_step < 1) { flush_step = 0; }
/*
  // This part does act strange on 1 heap builds
  // See: https://github.com/letscontrolit/ESPEasy/pull/3680#issuecomment-1031716163
  if (length < static_cast<unsigned int>(flush_step)) {
    // Just use the faster String operator to copy flash strings.
    this->buf += str;
    return *this;
  }
*/
  // FIXME TD-er: Not sure what happens, but streaming large flash chunks does cause allocation issues.
  const bool stream_P = ESP.getFreeHeap() > 4000 && 
                        length > (CHUNKED_BUFFER_SIZE >> 2) &&
                        length < (2 * CHUNKED_BUFFER_SIZE);

  if (stream_P && ((this->buf.length() + length) > CHUNKED_BUFFER_SIZE)) {
    // Do not copy to the internal buffer, but stream immediately.
    flush();
    web_server.sendContent_P(str);
  } else {
    // Copy to internal buffer and send in chunks
    unsigned int pos          = 0;
    while (pos < length) {
      if (flush_step == 0) {
        flush();
        flush_step = CHUNKED_BUFFER_SIZE;
      }
      this->buf += (char)pgm_read_byte(&str[pos]);
      ++pos;
      --flush_step;
    }
  }
  return *this;
}

Web_StreamingBuffer& Web_StreamingBuffer::addString(const String& a) {
  if (lowMemorySkip) { return *this; }
  const unsigned int length = a.length();
  if (length == 0) { return *this; }

  checkFull();
  int flush_step = CHUNKED_BUFFER_SIZE - this->buf.length();

  if (flush_step < 1) { flush_step = 0; }

  if (length < static_cast<unsigned int>(flush_step)) {
    // Just use the faster String operator to copy flash strings.
    this->buf += a;
    return *this;
  }

  unsigned int pos = 0;
  while (pos < length) {
    if (flush_step == 0) {
      flush();
      flush_step = CHUNKED_BUFFER_SIZE;
    } else {
      // Just copy per byte instead of using substring as substring needs to allocate memory.
      this->buf += a[pos];
      ++pos;
      --flush_step;
    }
  }
  return *this;
}

void Web_StreamingBuffer::flush() {
  if (lowMemorySkip) {
    this->buf.clear();
  } else {
    if (this->buf.length() > 0) {
      sendContentBlocking(this->buf);
    }
  }
}

void Web_StreamingBuffer::checkFull() {
  if (lowMemorySkip) { this->buf.clear(); }

  if (this->buf.length() >= CHUNKED_BUFFER_SIZE) {
    trackTotalMem();
    flush();
  }
}

void Web_StreamingBuffer::startStream() {
  startStream(false, F("text/html"), F(""));
}

void Web_StreamingBuffer::startStream(const __FlashStringHelper * origin) {
  startStream(false, F("text/html"), origin);
}

void Web_StreamingBuffer::startStream(const __FlashStringHelper * content_type, const __FlashStringHelper * origin) {
  startStream(false, content_type, origin);
}


void Web_StreamingBuffer::startJsonStream() {
  startStream(true, F("application/json"), F("*"));
}

void Web_StreamingBuffer::startStream(bool allowOriginAll, 
                                      const __FlashStringHelper * content_type, 
                                      const __FlashStringHelper * origin) {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  maxCoreUsage = maxServerUsage = 0;
  initialRam   = ESP.getFreeHeap();
  beforeTXRam  = initialRam;
  sentBytes    = 0;
  buf.clear();
  buf.reserve(CHUNKED_BUFFER_SIZE);
  
  if (beforeTXRam < 3000) {
    lowMemorySkip = true;
    web_server.send(200, F("text/plain"), F("Low memory. Cannot display webpage :-("));
      #if defined(ESP8266)
    tcpCleanup();
      #endif // if defined(ESP8266)
    return;
  } else {
    sendHeaderBlocking(allowOriginAll, content_type, origin);
  }
}

void Web_StreamingBuffer::trackTotalMem() {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  beforeTXRam = ESP.getFreeHeap();

  if ((initialRam - beforeTXRam) > maxServerUsage) {
    maxServerUsage = initialRam - beforeTXRam;
  }
}

void Web_StreamingBuffer::trackCoreMem() {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  duringTXRam = ESP.getFreeHeap();

  if ((initialRam - duringTXRam) > maxCoreUsage) {
    maxCoreUsage = (initialRam - duringTXRam);
  }
}

void Web_StreamingBuffer::endStream() {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  if (!lowMemorySkip) {
    if (buf.length() > 0) { sendContentBlocking(buf); }
    buf.clear();
    sendContentBlocking(buf);
    #ifdef ESP8266
    web_server.client().flush(100);
    #endif
    #ifdef ESP32
    web_server.client().flush();
    #endif
    finalRam = ESP.getFreeHeap();

    /*
        if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
        String log = String("Ram usage: Webserver only: ") + maxServerUsage +
                    " including Core: " + maxCoreUsage +
                    " flashStringCalls: " + flashStringCalls +
                    " flashStringData: " + flashStringData;
        addLog(LOG_LEVEL_DEBUG, log);
        }
      */
  } else {
    addLog(LOG_LEVEL_ERROR, String("Webpage skipped: low memory: ") + finalRam);
    lowMemorySkip = false;
  }
}




void Web_StreamingBuffer::sendContentBlocking(String& data) {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  delay(0); // Try to prevent WDT reboots

  const uint32_t length   = data.length();
#ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG_DEV)) {
    String log;
    log += F("sendcontent free: ");
    log += ESP.getFreeHeap();
    log += F(" chunk size:");
    log += length;
    addLogMove(LOG_LEVEL_DEBUG_DEV, log);
  }
#endif // ifndef BUILD_NO_DEBUG
  const uint32_t freeBeforeSend = ESP.getFreeHeap();
  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("sendContentBlocking"));
  #endif

  if (beforeTXRam > freeBeforeSend) {
    beforeTXRam = freeBeforeSend;
  }
  duringTXRam = freeBeforeSend;
  
#if defined(ESP8266) && defined(ARDUINO_ESP8266_RELEASE_2_3_0)
  String size = formatToHex(length) + "\r\n";

  // do chunked transfer encoding ourselves (WebServer doesn't support it)
  web_server.sendContent(size);

  if (length > 0) { web_server.sendContent(data); }
  web_server.sendContent("\r\n");
#else // ESP8266 2.4.0rc2 and higher and the ESP32 webserver supports chunked http transfer
  unsigned int timeout = 1;

  if (freeBeforeSend < 5000) { timeout = 100; }

  if (freeBeforeSend < 4000) { timeout = 300; }
  web_server.sendContent(data);

  data.clear();
  const uint32_t beginWait = millis();
  while ((!data.reserve(CHUNKED_BUFFER_SIZE) || (ESP.getFreeHeap() < 4000 /*freeBeforeSend*/ )) &&
         !timeOutReached(beginWait + timeout)) {
    if (ESP.getFreeHeap() < duringTXRam) {
      duringTXRam = ESP.getFreeHeap();
    }
    trackCoreMem();
    #ifndef BUILD_NO_RAM_TRACKER
    checkRAM(F("duringDataTX"));
    #endif

    delay(1);
  }
#endif // if defined(ESP8266) && defined(ARDUINO_ESP8266_RELEASE_2_3_0)

  sentBytes += length;
  delay(0);
}

void Web_StreamingBuffer::sendHeaderBlocking(bool allowOriginAll, 
                                             const String& content_type, 
                                             const String& origin) {
  #ifdef USE_SECOND_HEAP
  HeapSelectDram ephemeral;
  #endif

  #ifndef BUILD_NO_RAM_TRACKER
  checkRAM(F("sendHeaderBlocking"));
  #endif
  
  web_server.client().flush();

#if defined(ESP8266) && defined(ARDUINO_ESP8266_RELEASE_2_3_0)
  web_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  web_server.sendHeader(F("Accept-Ranges"),     F("none"));
  web_server.sendHeader(F("Cache-Control"),     F("no-cache"));
  web_server.sendHeader(F("Transfer-Encoding"), F("chunked"));

  if (allowOriginAll) {
    web_server.sendHeader(F("Access-Control-Allow-Origin"), "*");
  }
  web_server.send(200, content_type, EMPTY_STRING);
#else // if defined(ESP8266) && defined(ARDUINO_ESP8266_RELEASE_2_3_0)
  unsigned int timeout          = 0;
  const uint32_t freeBeforeSend = ESP.getFreeHeap();

  if (freeBeforeSend < 5000) { timeout = 100; }

  if (freeBeforeSend < 4000) { timeout = 1000; }
  const uint32_t beginWait = millis();
  web_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  web_server.sendHeader(F("Cache-Control"), F("no-cache"));

  if (origin.length() > 0) {
    web_server.sendHeader(F("Access-Control-Allow-Origin"), origin);
  }
  web_server.send(200, content_type, EMPTY_STRING);

  // dont wait on 2.3.0. Memory returns just too slow.
  while ((ESP.getFreeHeap() < freeBeforeSend) &&
         !timeOutReached(beginWait + timeout)) {
    #ifndef BUILD_NO_RAM_TRACKER
    checkRAM(F("duringHeaderTX"));
    #endif
    delay(1);
  }
#endif // if defined(ESP8266) && defined(ARDUINO_ESP8266_RELEASE_2_3_0)
  delay(0);
}

#include "../DataStructs/TimingStats.h"

#if FEATURE_TIMING_STATS

#include "../DataTypes/ESPEasy_plugin_functions.h"
#include "../Globals/CPlugins.h"
#include "../Helpers/_CPlugin_Helper.h"
#include "../Helpers/StringConverter.h"


std::map<int, TimingStats> pluginStats;
std::map<int, TimingStats> controllerStats;
std::map<int, TimingStats> miscStats;
unsigned long timingstats_last_reset(0);




TimingStats::TimingStats() : _timeTotal(0.0f), _count(0), _maxVal(0), _minVal(4294967295) {}

void TimingStats::add(int64_t time) {
  _timeTotal += static_cast<float>(time);
  ++_count;

  if (time > static_cast<int64_t>(_maxVal)) { _maxVal = time; }

  if (time < static_cast<int64_t>(_minVal)) { _minVal = time; }
}

void TimingStats::reset() {
  _timeTotal = 0.0f;
  _count     = 0;
  _maxVal    = 0;
  _minVal    = 4294967295;
}

bool TimingStats::isEmpty() const {
  return _count == 0;
}

float TimingStats::getAvg() const {
  if (_count == 0) { return 0.0f; }
  return _timeTotal / static_cast<float>(_count);
}

uint32_t TimingStats::getMinMax(uint64_t& minVal, uint64_t& maxVal) const {
  if (_count == 0) {
    minVal = 0;
    maxVal = 0;
    return 0;
  }
  minVal = _minVal;
  maxVal = _maxVal;
  return _count;
}

bool TimingStats::thresholdExceeded(const uint64_t& threshold) const {
  if (_count == 0) {
    return false;
  }
  return _maxVal > threshold;
}

/********************************************************************************************\
   Functions used for displaying timing stats
 \*********************************************************************************************/
const __FlashStringHelper * getPluginFunctionName(int function) {
  switch (function) {
    case PLUGIN_INIT_ALL:              return F("INIT_ALL");
    case PLUGIN_INIT:                  return F("INIT");
    case PLUGIN_READ:                  return F("READ");
    case PLUGIN_ONCE_A_SECOND:         return F("ONCE_A_SECOND");
    case PLUGIN_TEN_PER_SECOND:        return F("TEN_PER_SECOND");
    case PLUGIN_DEVICE_ADD:            return F("DEVICE_ADD");
    case PLUGIN_EVENTLIST_ADD:         return F("EVENTLIST_ADD");
    case PLUGIN_WEBFORM_SAVE:          return F("WEBFORM_SAVE");
    case PLUGIN_WEBFORM_LOAD:          return F("WEBFORM_LOAD");
    case PLUGIN_WEBFORM_SHOW_VALUES:   return F("WEBFORM_SHOW_VALUES");
    case PLUGIN_FORMAT_USERVAR:        return F("FORMAT_USERVAR");
    case PLUGIN_GET_DEVICENAME:        return F("GET_DEVICENAME");
    case PLUGIN_GET_DEVICEVALUENAMES:  return F("GET_DEVICEVALUENAMES");
    case PLUGIN_WRITE:                 return F("WRITE");
    case PLUGIN_EVENT_OUT:             return F("EVENT_OUT");
    case PLUGIN_WEBFORM_SHOW_CONFIG:   return F("WEBFORM_SHOW_CONFIG");
    case PLUGIN_SERIAL_IN:             return F("SERIAL_IN");
    case PLUGIN_UDP_IN:                return F("UDP_IN");
    case PLUGIN_CLOCK_IN:              return F("CLOCK_IN");
    case PLUGIN_TIMER_IN:              return F("TIMER_IN");
    case PLUGIN_FIFTY_PER_SECOND:      return F("FIFTY_PER_SECOND");
    case PLUGIN_SET_CONFIG:            return F("SET_CONFIG");
    case PLUGIN_GET_DEVICEGPIONAMES:   return F("GET_DEVICEGPIONAMES");
    case PLUGIN_EXIT:                  return F("EXIT");
    case PLUGIN_GET_CONFIG_VALUE:      return F("GET_CONFIG");
    case PLUGIN_UNCONDITIONAL_POLL:    return F("UNCONDITIONAL_POLL");
    case PLUGIN_REQUEST:               return F("REQUEST");
  }
  return F("Unknown");
}

bool mustLogFunction(int function) {
  if (!Settings.EnableTimingStats()) return false;
  switch (function) {
    case PLUGIN_INIT_ALL:              return false;
    case PLUGIN_INIT:                  return false;
    case PLUGIN_READ:                  return true;
    case PLUGIN_ONCE_A_SECOND:         return true;
    case PLUGIN_TEN_PER_SECOND:        return true;
    case PLUGIN_DEVICE_ADD:            return false;
    case PLUGIN_EVENTLIST_ADD:         return false;
    case PLUGIN_WEBFORM_SAVE:          return false;
    case PLUGIN_WEBFORM_LOAD:          return false;
    case PLUGIN_WEBFORM_SHOW_VALUES:   return false;
    case PLUGIN_FORMAT_USERVAR:        return false;
    case PLUGIN_GET_DEVICENAME:        return false;
    case PLUGIN_GET_DEVICEVALUENAMES:  return false;
    case PLUGIN_WRITE:                 return true;
    case PLUGIN_EVENT_OUT:             return true;
    case PLUGIN_WEBFORM_SHOW_CONFIG:   return false;
    case PLUGIN_SERIAL_IN:             return true;
    case PLUGIN_UDP_IN:                return true;
    case PLUGIN_CLOCK_IN:              return false;
    case PLUGIN_TIMER_IN:              return true;
    case PLUGIN_FIFTY_PER_SECOND:      return true;
    case PLUGIN_SET_CONFIG:            return false;
    case PLUGIN_GET_DEVICEGPIONAMES:   return false;
    case PLUGIN_EXIT:                  return false;
    case PLUGIN_GET_CONFIG_VALUE:      return false;
    case PLUGIN_UNCONDITIONAL_POLL:    return false;
    case PLUGIN_REQUEST:               return true;
  }
  return false;
}

const __FlashStringHelper * getCPluginCFunctionName(CPlugin::Function function) {
  switch (function) {
    case CPlugin::Function::CPLUGIN_PROTOCOL_ADD:              return F("CPLUGIN_PROTOCOL_ADD");
    case CPlugin::Function::CPLUGIN_PROTOCOL_TEMPLATE:         return F("CPLUGIN_PROTOCOL_TEMPLATE");
    case CPlugin::Function::CPLUGIN_PROTOCOL_SEND:             return F("CPLUGIN_PROTOCOL_SEND");
    case CPlugin::Function::CPLUGIN_PROTOCOL_RECV:             return F("CPLUGIN_PROTOCOL_RECV");
    case CPlugin::Function::CPLUGIN_GET_DEVICENAME:            return F("CPLUGIN_GET_DEVICENAME");
    case CPlugin::Function::CPLUGIN_WEBFORM_SAVE:              return F("CPLUGIN_WEBFORM_SAVE");
    case CPlugin::Function::CPLUGIN_WEBFORM_LOAD:              return F("CPLUGIN_WEBFORM_LOAD");
    case CPlugin::Function::CPLUGIN_GET_PROTOCOL_DISPLAY_NAME: return F("CPLUGIN_GET_PROTOCOL_DISPLAY_NAME");
    case CPlugin::Function::CPLUGIN_TASK_CHANGE_NOTIFICATION:  return F("CPLUGIN_TASK_CHANGE_NOTIFICATION");
    case CPlugin::Function::CPLUGIN_INIT:                      return F("CPLUGIN_INIT");
    case CPlugin::Function::CPLUGIN_UDP_IN:                    return F("CPLUGIN_UDP_IN");
    case CPlugin::Function::CPLUGIN_FLUSH:                     return F("CPLUGIN_FLUSH");
    case CPlugin::Function::CPLUGIN_TEN_PER_SECOND:            return F("CPLUGIN_TEN_PER_SECOND");
    case CPlugin::Function::CPLUGIN_FIFTY_PER_SECOND:          return F("CPLUGIN_FIFTY_PER_SECOND");
    case CPlugin::Function::CPLUGIN_INIT_ALL:                  return F("CPLUGIN_INIT_ALL");
    case CPlugin::Function::CPLUGIN_EXIT:                      return F("CPLUGIN_EXIT");

    case CPlugin::Function::CPLUGIN_GOT_CONNECTED:
    case CPlugin::Function::CPLUGIN_GOT_INVALID:
    case CPlugin::Function::CPLUGIN_INTERVAL:
    case CPlugin::Function::CPLUGIN_ACKNOWLEDGE:
    case CPlugin::Function::CPLUGIN_WEBFORM_SHOW_HOST_CONFIG:
      break;

  }
  return F("Unknown");
}

bool mustLogCFunction(CPlugin::Function function) {
  if (!Settings.EnableTimingStats()) return false;
  switch (function) {
    case CPlugin::Function::CPLUGIN_PROTOCOL_ADD:              return false;
    case CPlugin::Function::CPLUGIN_PROTOCOL_TEMPLATE:         return false;
    case CPlugin::Function::CPLUGIN_PROTOCOL_SEND:             return true;
    case CPlugin::Function::CPLUGIN_PROTOCOL_RECV:             return true;
    case CPlugin::Function::CPLUGIN_GET_DEVICENAME:            return false;
    case CPlugin::Function::CPLUGIN_WEBFORM_SAVE:              return false;
    case CPlugin::Function::CPLUGIN_WEBFORM_LOAD:              return false;
    case CPlugin::Function::CPLUGIN_GET_PROTOCOL_DISPLAY_NAME: return false;
    case CPlugin::Function::CPLUGIN_TASK_CHANGE_NOTIFICATION:  return false;
    case CPlugin::Function::CPLUGIN_INIT:                      return false;
    case CPlugin::Function::CPLUGIN_UDP_IN:                    return true;
    case CPlugin::Function::CPLUGIN_FLUSH:                     return false;
    case CPlugin::Function::CPLUGIN_TEN_PER_SECOND:            return true;
    case CPlugin::Function::CPLUGIN_FIFTY_PER_SECOND:          return true;
    case CPlugin::Function::CPLUGIN_INIT_ALL:                  return false;
    case CPlugin::Function::CPLUGIN_EXIT:                      return false;

    case CPlugin::Function::CPLUGIN_GOT_CONNECTED:
    case CPlugin::Function::CPLUGIN_GOT_INVALID:
    case CPlugin::Function::CPLUGIN_INTERVAL:
    case CPlugin::Function::CPLUGIN_ACKNOWLEDGE:
    case CPlugin::Function::CPLUGIN_WEBFORM_SHOW_HOST_CONFIG:
      break;

  }
  return false;
}

// Return flash string type to reduce bin size
const __FlashStringHelper * getMiscStatsName_F(int stat) {
  switch (stat) {
    case LOADFILE_STATS:          return F("Load File");
    case SAVEFILE_STATS:          return F("Save File");
    case LOOP_STATS:              return F("Loop");
    case PLUGIN_CALL_50PS:        return F("Plugin call 50 p/s");
    case PLUGIN_CALL_10PS:        return F("Plugin call 10 p/s");
    case PLUGIN_CALL_10PSU:       return F("Plugin call 10 p/s U");
    case PLUGIN_CALL_1PS:         return F("Plugin call  1 p/s");
    case CPLUGIN_CALL_50PS:       return F("CPlugin call 50 p/s");
    case CPLUGIN_CALL_10PS:       return F("CPlugin call 10 p/s");
    case SENSOR_SEND_TASK:        return F("SensorSendTask()");
    case SEND_DATA_STATS:         return F("sendData()");
    case COMPUTE_FORMULA_STATS:   return F("Compute formula");
    case PROC_SYS_TIMER:          return F("proc_system_timer()");
    case SET_NEW_TIMER:           return F("setNewTimerAt()");
    case TIME_DIFF_COMPUTE:       return F("timeDiff()");
    case MQTT_DELAY_QUEUE:        return F("Delay queue MQTT");
    case TRY_CONNECT_HOST_TCP:    return F("try_connect_host() (TCP)");
    case TRY_CONNECT_HOST_UDP:    return F("try_connect_host() (UDP)");
    case HOST_BY_NAME_STATS:      return F("hostByName()");
    case CONNECT_CLIENT_STATS:    return F("connectClient()");
    case LOAD_CUSTOM_TASK_STATS:  return F("LoadCustomTaskSettings()");
    case WIFI_ISCONNECTED_STATS:  return F("WiFi.isConnected()");
    case WIFI_NOTCONNECTED_STATS: return F("WiFi.isConnected() (fail)");
    case LOAD_TASK_SETTINGS:      return F("LoadTaskSettings()");
    case TRY_OPEN_FILE:           return F("TryOpenFile()");
    case FS_GC_SUCCESS:           return F("ESPEASY_FS GC success");
    case FS_GC_FAIL:              return F("ESPEASY_FS GC fail");
    case RULES_PROCESSING:        return F("rulesProcessing()");
    case RULES_PARSE_LINE:        return F("parseCompleteNonCommentLine()");
    case RULES_PROCESS_MATCHED:   return F("processMatchedRule()");
    case RULES_MATCH:             return F("rulesMatch()");
    case GRAT_ARP_STATS:          return F("sendGratuitousARP()");
    case SAVE_TO_RTC:             return F("saveToRTC()");
    case BACKGROUND_TASKS:        return F("backgroundtasks()");
    case PROCESS_SYSTEM_EVENT_QUEUE: return F("process_system_event_queue()");
    case HANDLE_SCHEDULER_IDLE:   return F("handle_schedule() idle");
    case HANDLE_SCHEDULER_TASK:   return F("handle_schedule() task");
    case PARSE_TEMPLATE_PADDED:   return F("parseTemplate_padded()");
    case PARSE_SYSVAR:            return F("parseSystemVariables()");
    case PARSE_SYSVAR_NOCHANGE:   return F("parseSystemVariables() No change");
    case HANDLE_SERVING_WEBPAGE:  return F("handle webpage");
    case WIFI_SCAN_ASYNC:         return F("WiFi Scan Async");
    case WIFI_SCAN_SYNC:          return F("WiFi Scan Sync (blocking)");
    case C018_AIR_TIME:           return F("C018 LoRa TTN - Air Time");
  }
  return F("Unknown");
}

String getMiscStatsName(int stat) {
  if (stat >= C001_DELAY_QUEUE && stat <= C025_DELAY_QUEUE) {
    String result;
    result.reserve(16);
    result  = F("Delay queue ");
    result += get_formatted_Controller_number(static_cast<cpluginID_t>(stat - C001_DELAY_QUEUE + 1));
    return result;
  }
  return getMiscStatsName_F(stat);
}

#endif // if FEATURE_TIMING_STATS
#include "../DataStructs/LogStruct.h"

#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/StringConverter.h"

void LogStruct::add_end(const uint8_t loglevel) {
  timeStamp[write_idx] = millis();
  log_level[write_idx] = loglevel;
  if (isFull()) {
    read_idx  = nextIndex(read_idx);
  }
  write_idx = nextIndex(write_idx);
  is_full = (write_idx == read_idx);
}

void LogStruct::add(const uint8_t loglevel, const String& line) {
  if (line.length() > 0)
  {
    #ifdef USE_SECOND_HEAP
    // Allow to store the logs in 2nd heap if present.
    HeapSelectIram ephemeral;
    #endif

    if (line.length() > LOG_STRUCT_MESSAGE_SIZE - 1) {
      Message[write_idx] = std::move(line.substring(0, LOG_STRUCT_MESSAGE_SIZE - 1));
    } else {
      Message[write_idx] = line;
    }
    add_end(loglevel);
  }
}

void LogStruct::add(const uint8_t loglevel, String&& line) {
  if (line.length() > 0)
  {
    if (line.length() > LOG_STRUCT_MESSAGE_SIZE - 1) {
      #ifdef USE_SECOND_HEAP
      // Need to make a substring, which is a new allocation, on the 2nd heap
      HeapSelectIram ephemeral;
      #endif
      Message[write_idx] = std::move(line.substring(0, LOG_STRUCT_MESSAGE_SIZE - 1));
    } else {
      #ifdef USE_SECOND_HEAP
      // Allow to store the logs in 2nd heap if present.
      HeapSelectIram ephemeral;

      if (!mmu_is_iram(&(line[0]))) {
        // The log entry was not allocated on the 2nd heap, so copy instead of move
        Message[write_idx] = line;
      } else {
        Message[write_idx] = std::move(line);
      }
      #else
      Message[write_idx] = std::move(line);
      #endif
    }
    add_end(loglevel);
  }
}


bool LogStruct::getNext(bool& logLinesAvailable, unsigned long& timestamp, String& message, uint8_t& loglevel) {
  lastReadTimeStamp = millis();
  logLinesAvailable = false;

  if (isEmpty()) {
    return false;
  }
  timestamp = timeStamp[read_idx];
  message = std::move(Message[read_idx]);
  loglevel = log_level[read_idx];
  clearOldest();
  if (!isEmpty()) { 
    logLinesAvailable = true;
  }
  return true;
}

bool LogStruct::isEmpty() const {
  return !is_full && (write_idx == read_idx);
}

bool LogStruct::isFull() const {
  return is_full;
}

bool LogStruct::logActiveRead() {
  clearExpiredEntries();
  return timePassedSince(lastReadTimeStamp) < LOG_BUFFER_EXPIRE;
}

void LogStruct::clearExpiredEntries() {
  unsigned int maxLoops = LOG_STRUCT_MESSAGE_LINES;
  while (maxLoops > 0) {
    --maxLoops;
    if (isEmpty() ||  // Nothing left
        (timePassedSince(timeStamp[read_idx]) < LOG_BUFFER_EXPIRE)) // Expired
    {
      return;
    }
    clearOldest();
  }
}

void LogStruct::clearOldest() {
  if (!isEmpty()) {
    is_full = false;
    Message[read_idx] = String();
    timeStamp[read_idx] = 0;
    log_level[read_idx] = 0;
    read_idx  = nextIndex(read_idx);
  }
}
#include "../DataStructs/WiFi_AP_Candidate.h"

#include "../Globals/ESPEasyWiFiEvent.h"
#include "../Globals/SecuritySettings.h"
#include "../Globals/Statistics.h"
#include "../Helpers/ESPEasy_time_calc.h"
#include "../Helpers/Misc.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/StringGenerator_WiFi.h"
#include "../../ESPEasy_common.h"




#define WIFI_AP_CANDIDATE_MAX_AGE   300000  // 5 minutes in msec


WiFi_AP_Candidate::WiFi_AP_Candidate(uint8_t index_c, const String& ssid_c, const String& pass) :
  rssi(0), channel(0), index(index_c), isHidden(false)
{
  const size_t ssid_length = ssid_c.length();

  if ((ssid_length == 0) || ssid_c.equals(F("ssid"))) {
    return;
  }

  if (ssid_length > 32) { return; }

  if (pass.length() > 64) { return; }

  ssid = ssid_c;
  key  = pass;
}

WiFi_AP_Candidate::WiFi_AP_Candidate(uint8_t networkItem) : index(0) {
  ssid    = WiFi.SSID(networkItem);
  rssi    = WiFi.RSSI(networkItem);
  channel = WiFi.channel(networkItem);
  bssid   = WiFi.BSSID(networkItem);
  enc_type = WiFi.encryptionType(networkItem);
  #ifdef ESP8266
  isHidden = WiFi.isHidden(networkItem);
  #endif // ifdef ESP8266
  #ifdef ESP32
  isHidden = ssid.isEmpty();
  #endif // ifdef ESP32
  last_seen = millis();
}

#ifdef ESP8266
WiFi_AP_Candidate::WiFi_AP_Candidate(const bss_info& ap) :
  rssi(ap.rssi), channel(ap.channel), bssid(ap.bssid), 
  index(0), enc_type(ap.authmode), isHidden(ap.is_hidden)
{
  last_seen = millis();
  ssid.reserve(ap.ssid_len);
  for (int i = 0; i < ap.ssid_len; ++i) {
    ssid += ap.ssid[i];
  }

}
#endif


bool WiFi_AP_Candidate::operator<(const WiFi_AP_Candidate& other) const {
  if (isEmergencyFallback != other.isEmergencyFallback) {
    return isEmergencyFallback;
  }
  if (lowPriority != other.lowPriority) {
    return !lowPriority;
  }
  // Prefer non hidden over hidden.
  if (isHidden != other.isHidden) {
    return !isHidden;
  }

  // RSSI values >= 0 are invalid
  if (rssi >= 0) { return false; }

  if (other.rssi >= 0) { return true; }

  // RSSI values are negative, so the larger value is the better one.
  return rssi > other.rssi;
}

bool WiFi_AP_Candidate::operator==(const WiFi_AP_Candidate& other) const {
  return bssid_match(other.bssid) && ssid.equals(other.ssid);// && key.equals(other.key);
}

bool WiFi_AP_Candidate::usable() const {
  // Allow for empty pass
  // if (key.isEmpty()) return false;
  if (isEmergencyFallback) {
    int allowedUptimeMinutes = 10;
    #ifdef CUSTOM_EMERGENCY_FALLBACK_ALLOW_MINUTES_UPTIME
    allowedUptimeMinutes = CUSTOM_EMERGENCY_FALLBACK_ALLOW_MINUTES_UPTIME;
    #endif
    if (getUptimeMinutes() > allowedUptimeMinutes || 
        !SecuritySettings.hasWiFiCredentials() || 
        WiFiEventData.performedClearWiFiCredentials ||
        lastBootCause != BOOT_CAUSE_COLD_BOOT) {
      return false;
    }
  }
  if (!isHidden && (ssid.isEmpty())) { return false; }
  return true;
}

bool WiFi_AP_Candidate::expired() const {
  if (last_seen == 0) {
    // Not set, so cannot expire
    return false;
  }
  return timePassedSince(last_seen) > WIFI_AP_CANDIDATE_MAX_AGE;
}

bool WiFi_AP_Candidate::allowQuickConnect() const {
  if (channel == 0) { return false; }
  return bssid_set();
}

bool WiFi_AP_Candidate::bssid_set() const {
  return !bssid.all_zero();
}

bool WiFi_AP_Candidate::bssid_match(const uint8_t bssid_c[6]) const {
  return bssid == bssid_c;
}

bool WiFi_AP_Candidate::bssid_match(const MAC_address& other) const {
  return bssid == other;
}

String WiFi_AP_Candidate::toString(const String& separator) const {
  String result = ssid;

  htmlEscape(result);
  if (isHidden) {
    result += F("#Hidden#");
  }
  result += separator;
  result += bssid.toString();
  result += separator;
  result += F("Ch:");
  result += channel;

  if (rssi == -1) {
    result += F(" (RTC) ");
  } else {
    result += F(" (");
    result += rssi;
    result += F("dBm) ");
  }

  result += encryption_type();
  return result;
}

String WiFi_AP_Candidate::encryption_type() const {
  return WiFi_encryptionType(enc_type);
}

#include "../DataStructs/ESPEasy_packed_raw_data.h"


uint8_t getPackedDataTypeSize(PackedData_enum dtype, float& factor, float& offset) {
  offset = 0;
  factor = 1;
  if (dtype > 0x1000 && dtype < 0x12FF) {
    const uint32_t exponent = dtype & 0xF;
    switch(exponent) {
      case 0: factor = 1; break;
      case 1: factor = 1e1; break;
      case 2: factor = 1e2; break;
      case 3: factor = 1e3; break;
      case 4: factor = 1e4; break;
      case 5: factor = 1e5; break;
      case 6: factor = 1e6; break;
    }
    const uint8_t size = (dtype >> 4) & 0xF;
    return size;
  }
  switch (dtype) {
    case PackedData_pluginid:    factor = 1;         return 1;
    case PackedData_latLng:      factor = 46600;     return 3; // 2^23 / 180
    case PackedData_hdop:        factor = 10;        return 1;
    case PackedData_altitude:    factor = 4;     offset = 1000; return 2; // -1000 .. 15383.75 meter
    case PackedData_vcc:         factor = 41.83; offset = 1;    return 1; // -1 .. 5.12V
    case PackedData_pct_8:       factor = 2.56;                 return 1; // 0 .. 100%
    default:
      break;
  }

  // Unknown type
  factor = 1;
  return 0;
}

void LoRa_uintToBytes(uint64_t value, uint8_t byteSize, uint8_t *data, uint8_t& cursor) {
  // Clip values to upper limit
  const uint64_t upperlimit = (1ull << (8*byteSize)) - 1;
  if (value > upperlimit) { value = upperlimit; }
  for (uint8_t x = 0; x < byteSize; x++) {
    uint8_t next = 0;
    if (sizeof(value) > x) {
      next = static_cast<uint8_t>((value >> (x * 8)) & 0xFF);
    }
    data[cursor] = next;
    ++cursor;
  }
}

void LoRa_intToBytes(int64_t value, uint8_t byteSize, uint8_t *data, uint8_t& cursor) {
  // Clip values to lower limit
  const int64_t lowerlimit = (1ull << ((8*byteSize) - 1)) * -1;
  if (value < lowerlimit) { value = lowerlimit; }
  if (value < 0) {
    value += (1ull << (8*byteSize));
  }
  LoRa_uintToBytes(value, byteSize, data, cursor);
}

String LoRa_base16Encode(uint8_t *data, size_t size) {
  String output;
  output.reserve(size * 2);
  char buffer[3];
  for (unsigned i=0; i<size; i++)
  {
    sprintf_P(buffer, PSTR("%02X"), data[i]);
    output += buffer[0];
    output += buffer[1];
  }
  return output;
}

String LoRa_addInt(uint64_t value, PackedData_enum datatype) {
  float factor, offset;
  uint8_t byteSize = getPackedDataTypeSize(datatype, factor, offset);
  uint8_t data[4] = {0};
  uint8_t cursor = 0;
  LoRa_uintToBytes((value + offset) * factor, byteSize, &data[0], cursor);
  return LoRa_base16Encode(data, cursor);
}


String LoRa_addFloat(float value, PackedData_enum datatype) {
  float factor, offset;
  uint8_t byteSize = getPackedDataTypeSize(datatype, factor, offset);
  uint8_t data[4] = {0};
  uint8_t cursor = 0;
  LoRa_intToBytes((value + offset) * factor, byteSize, &data[0], cursor);
  return LoRa_base16Encode(data, cursor);
}

#include "../DataStructs/TimeChangeRule.h"



TimeChangeRule::TimeChangeRule() :  week(0), dow(1), month(1), hour(0), offset(0) {}

TimeChangeRule::TimeChangeRule(uint8_t weeknr, uint8_t downr, uint8_t m, uint8_t h, int16_t minutesoffset) :
  week(weeknr), dow(downr), month(m), hour(h), offset(minutesoffset) {}

// Construct time change rule from stored values optimized for minimum space.
TimeChangeRule::TimeChangeRule(uint16_t flash_stored_value, int16_t minutesoffset) : offset(minutesoffset) {
  hour  = flash_stored_value & 0x001f;
  month = (flash_stored_value >> 5) & 0x000f;
  dow   = (flash_stored_value >> 9) & 0x0007;
  week  = (flash_stored_value >> 12) & 0x0007;
}

uint16_t TimeChangeRule::toFlashStoredValue() const {
  uint16_t value = hour;

  value = value | (month << 5);
  value = value | (dow << 9);
  value = value | (week << 12);
  return value;
}

bool TimeChangeRule::isValid() const {
  return (week <= 4) && (dow != 0) && (dow <= 7) &&
          (month != 0) && (month <= 12) && (hour <= 23) &&
          (offset > -720) && (offset < 900); // UTC-12h ... UTC+14h + 1h DSToffset
}

#include "../DataStructs/EventQueue.h"

#include "../../ESPEasy_common.h"


void EventQueueStruct::add(const String& event, bool deduplicate)
{
  #ifdef USE_SECOND_HEAP
  HeapSelectIram ephemeral;
  #endif // ifdef USE_SECOND_HEAP

  if (!deduplicate || !isDuplicate(event)) {
    _eventQueue.push_back(event);
  }
}

void EventQueueStruct::add(const __FlashStringHelper *event, bool deduplicate)
{
  #ifdef USE_SECOND_HEAP
  HeapSelectIram ephemeral;
  #endif // ifdef USE_SECOND_HEAP

  // Wrap in String() constructor to make sure it is using the 2nd heap allocator if present.
  if (!deduplicate || !isDuplicate(event)) {
    _eventQueue.push_back(String(event));
  }
}

void EventQueueStruct::addMove(String&& event, bool deduplicate)
{
  if (!event.length()) { return; }
  #ifdef USE_SECOND_HEAP
  HeapSelectIram ephemeral;

  if (!mmu_is_iram(&(event[0]))) {
    // Wrap in String constructor to make sure it is stored in the 2nd heap.
    if (!deduplicate || !isDuplicate(event)) {
      _eventQueue.push_back(String(event));
    }
    return;
  }
  #endif // ifdef USE_SECOND_HEAP

  if (!deduplicate || !isDuplicate(event)) {
    _eventQueue.emplace_back(std::move(event));
  }
}

bool EventQueueStruct::getNext(String& event)
{
  if (_eventQueue.empty()) {
    return false;
  }
  #ifdef USE_SECOND_HEAP
  {
    // Fetch the event and make sure it is allocated on the DRAM heap, not the 2nd heap
    // Otherwise checks like strnlen_P may crash on it.
    HeapSelectDram ephemeral;
    event = std::move(String(_eventQueue.front()));
  }
  #else // ifdef USE_SECOND_HEAP
  event = std::move(_eventQueue.front());
  #endif // ifdef USE_SECOND_HEAP
  _eventQueue.pop_front();
  return true;
}

void EventQueueStruct::clear()
{
  _eventQueue.clear();
}

bool EventQueueStruct::isEmpty() const
{
  return _eventQueue.empty();
}

bool EventQueueStruct::isDuplicate(const String& event) {
  return std::find(_eventQueue.begin(), _eventQueue.end(), event) != _eventQueue.end();
}

#include "../DataStructs/SecurityStruct.h"

#include "../../ESPEasy_common.h"
#include "../CustomBuild/ESPEasyLimits.h"
#include "../ESPEasyCore/ESPEasy_Log.h"
#include "../Globals/CPlugins.h"

SecurityStruct::SecurityStruct() {
  ZERO_FILL(WifiSSID);
  ZERO_FILL(WifiKey);
  ZERO_FILL(WifiSSID2);
  ZERO_FILL(WifiKey2);
  ZERO_FILL(WifiAPKey);

  for (controllerIndex_t i = 0; i < CONTROLLER_MAX; ++i) {
    ZERO_FILL(ControllerUser[i]);
    ZERO_FILL(ControllerPassword[i]);
  }
  ZERO_FILL(Password);
}

void SecurityStruct::validate() {
  ZERO_TERMINATE(WifiSSID);
  ZERO_TERMINATE(WifiKey);
  ZERO_TERMINATE(WifiSSID2);
  ZERO_TERMINATE(WifiKey2);
  ZERO_TERMINATE(WifiAPKey);

  for (controllerIndex_t i = 0; i < CONTROLLER_MAX; ++i) {
    ZERO_TERMINATE(ControllerUser[i]);
    ZERO_TERMINATE(ControllerPassword[i]);
  }
  ZERO_TERMINATE(Password);
}

void SecurityStruct::clearWiFiCredentials() {
  ZERO_FILL(WifiSSID);
  ZERO_FILL(WifiKey);
  ZERO_FILL(WifiSSID2);
  ZERO_FILL(WifiKey2);
  addLog(LOG_LEVEL_INFO, F("WiFi : Clear WiFi credentials from settings"));
}

void SecurityStruct::clearWiFiCredentials(SecurityStruct::WiFiCredentialsSlot slot) {
  switch (slot) {
    case SecurityStruct::WiFiCredentialsSlot::first:
      ZERO_FILL(WifiSSID);
      ZERO_FILL(WifiKey);
      break;
    case SecurityStruct::WiFiCredentialsSlot::second:
      ZERO_FILL(WifiSSID2);
      ZERO_FILL(WifiKey2);
      break;
  }
}

bool SecurityStruct::hasWiFiCredentials() const {
  return hasWiFiCredentials(SecurityStruct::WiFiCredentialsSlot::first) || 
         hasWiFiCredentials(SecurityStruct::WiFiCredentialsSlot::second);
}

bool SecurityStruct::hasWiFiCredentials(SecurityStruct::WiFiCredentialsSlot slot) const {
  switch (slot) {
    case SecurityStruct::WiFiCredentialsSlot::first:
      return (WifiSSID[0] != 0 && !String(WifiSSID).equalsIgnoreCase(F("ssid")));
    case SecurityStruct::WiFiCredentialsSlot::second:
      return (WifiSSID2[0] != 0 && !String(WifiSSID2).equalsIgnoreCase(F("ssid")));
  }
  return false;
}

String SecurityStruct::getPassword() const {
  String res;
  const size_t passLength = strnlen(Password, sizeof(Password));
  res.reserve(passLength);
  for (size_t i = 0; i < passLength; ++i) {
    res += Password[i];
  }
  return res;
}
#include "../DataStructs/ExtendedControllerCredentialsStruct.h"

#include "../Helpers/ESPEasy_Storage.h"

#define EXT_CONTR_CRED_USER_OFFSET 0
#define EXT_CONTR_CRED_PASS_OFFSET 1


ExtendedControllerCredentialsStruct::ExtendedControllerCredentialsStruct() {}


String ExtendedControllerCredentialsStruct::load()
{
  return LoadStringArray(SettingsType::Enum::ExtdControllerCredentials_Type,
                           0,
                           _strings, CONTROLLER_MAX * 2, 0);
}

String ExtendedControllerCredentialsStruct::save() const
{
  return SaveStringArray(SettingsType::Enum::ExtdControllerCredentials_Type,
                           0,
                           _strings, CONTROLLER_MAX * 2, 0);
}


String ExtendedControllerCredentialsStruct::getControllerUser(controllerIndex_t controller_idx) const
{
  if (validControllerIndex(controller_idx)) {
    return _strings[controller_idx * 2 + EXT_CONTR_CRED_USER_OFFSET];
  }
  return "";
}

String ExtendedControllerCredentialsStruct::getControllerPass(controllerIndex_t controller_idx) const
{
  if (validControllerIndex(controller_idx)) {
    return _strings[controller_idx * 2 + EXT_CONTR_CRED_PASS_OFFSET];
  }
  return "";
}

void ExtendedControllerCredentialsStruct::setControllerUser(controllerIndex_t controller_idx, const String& user)
{
  if (validControllerIndex(controller_idx)) {
    _strings[controller_idx * 2 + EXT_CONTR_CRED_USER_OFFSET] = user;
  }
}

void ExtendedControllerCredentialsStruct::setControllerPass(controllerIndex_t controller_idx, const String& pass)
{
  if (validControllerIndex(controller_idx)) {
    _strings[controller_idx * 2 + EXT_CONTR_CRED_PASS_OFFSET] = pass;
  }
}

#include "../DataStructs/ChartJS_dataset_config.h"
#include "../DataStructs/UnitMessageCount.h"

bool UnitLastMessageCount_map::isNew(const UnitMessageCount_t *count) const {
  if (count == nullptr) { return true; }
  auto it = _map.find(count->unit);

  if (it != _map.end()) {
    return it->second != count->count;
  }
  return true;
}

void UnitLastMessageCount_map::add(const UnitMessageCount_t *count) {
  if (count == nullptr) { return; }

  if ((count->unit != 0) && (count->unit != 255)) {
    _map[count->unit] = count->count;
  }
}

#include "../DataStructs/MAC_address.h"

#include "../../ESPEasy_common.h"

MAC_address::MAC_address()
{}

MAC_address::MAC_address(const uint8_t new_mac[6])
{
  memcpy(mac, new_mac, 6);
}

MAC_address::MAC_address(const MAC_address& other)
{
  for (int i = 0; i < 6; ++i) {
    mac[i] = other.mac[i];
  }
}

MAC_address& MAC_address::operator=(const MAC_address& other)
{
  for (int i = 0; i < 6; ++i) {
    mac[i] = other.mac[i];
  }
  return *this;
}

bool MAC_address::set(const char *string)
{
  unsigned u[6];
  int c = sscanf(string, "%x:%x:%x:%x:%x:%x", u, u + 1, u + 2, u + 3, u + 4, u + 5);

  if (c != 6) {
    return false;
  }

  for (int i = 0; i < 6; ++i) {
    mac[i] = static_cast<uint8_t>(u[i]);
  }
  return true;
}

void MAC_address::set(const uint8_t other[6])
{
  memcpy(mac, other, 6);
}

void MAC_address::get(uint8_t mac_out[6]) const
{
  memcpy(mac_out, mac, 6);
}

bool MAC_address::all_zero() const
{
  for (int i = 0; i < 6; ++i) {
    if (mac[i] != 0) {
      return false;
    }
  }
  return true;
}

bool MAC_address::all_one() const
{
  for (int i = 0; i < 6; ++i) {
    if (mac[i] != 0xFF) {
      return false;
    }
  }
  return true;
}

String MAC_address::toString() const
{
  char str[18] = { 0 };
  sprintf_P(str, PSTR("%02X:%02X:%02X:%02X:%02X:%02X"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(str);
}

bool MAC_address::mac_addr_cmp(const uint8_t other[6]) const
{
  for (int i = 0; i < 6; ++i) {
    if (mac[i] != other[i]) {
      return false;
    }
  }
  return true;
}
#include "../DataStructs/C013_p2p_dataStructs.h"

#include "../Globals/Plugins.h"

#ifdef USES_C013

C013_SensorInfoStruct::C013_SensorInfoStruct()
{
  ZERO_FILL(taskName);

  for (int i = 0; i < VARS_PER_TASK; ++i) {
    ZERO_FILL(ValueNames[i]);
  }
}

bool C013_SensorInfoStruct::isValid() const
{
  if ((header != 255) || (ID != 3)) { return false; }

  if (!validTaskIndex(sourceTaskIndex) ||
      !validTaskIndex(destTaskIndex) ||
      !validPluginID(deviceNumber)) {
    return false;
  }


  return true;
}

C013_SensorDataStruct::C013_SensorDataStruct()
{
  for (int i = 0; i < VARS_PER_TASK; ++i) {
    Values[i] = 0.0f;
  }
}

bool C013_SensorDataStruct::isValid() const
{
  if ((header != 255) || (ID != 5)) { return false; }

  if (!validTaskIndex(sourceTaskIndex) ||
      !validTaskIndex(destTaskIndex)) {
    return false;
  }
  return true;
}

#endif
#include "../DataStructs/timer_id_couple.h"

#include "../Helpers/ESPEasy_time_calc.h"

  timer_id_couple::timer_id_couple(unsigned long id, unsigned long newtimer) : _id(id), _timer(newtimer) {}

  timer_id_couple::timer_id_couple(unsigned long id) : _id(id) {
    _timer = millis();
  }

  bool timer_id_couple::operator<(const timer_id_couple& other) {
    const unsigned long now(millis());

    // timediff > 0, means timer has already passed
    return timeDiff(_timer, now) > timeDiff(other._timer, now);
  }

#include "../DataStructs/NodeStruct.h"

const __FlashStringHelper * getNodeTypeDisplayString(uint8_t nodeType) {
  switch (nodeType)
  {
    case NODE_TYPE_ID_ESP_EASY_STD:     return F("ESP Easy");
    case NODE_TYPE_ID_ESP_EASYM_STD:    return F("ESP Easy Mega");
    case NODE_TYPE_ID_ESP_EASY32_STD:   return F("ESP Easy 32");
    case NODE_TYPE_ID_ESP_EASY32S2_STD: return F("ESP Easy 32-s2");
    case NODE_TYPE_ID_ESP_EASY32C3_STD: return F("ESP Easy 32-c3");
    case NODE_TYPE_ID_RPI_EASY_STD:     return F("RPI Easy");
    case NODE_TYPE_ID_ARDUINO_EASY_STD: return F("Arduino Easy");
    case NODE_TYPE_ID_NANO_EASY_STD:    return F("Nano Easy");
  }
  return F("");
}

#if FEATURE_ESPEASY_P2P
  NodeStruct::NodeStruct() :
    build(0), age(0), nodeType(0), webgui_portnumber(0)
  {
    for (uint8_t i = 0; i < 4; ++i) { ip[i] = 0; }
  }
#endif
#include "../DataStructs/GpioFactorySettingsStruct.h"

#include "../CustomBuild/ESPEasyDefaults.h"

GpioFactorySettingsStruct::GpioFactorySettingsStruct(DeviceModel model)
  :
  status_led(DEFAULT_PIN_STATUS_LED),
  i2c_sda(DEFAULT_PIN_I2C_SDA),
  i2c_scl(DEFAULT_PIN_I2C_SCL),
  eth_phyaddr(DEFAULT_ETH_PHY_ADDR),
  eth_phytype(DEFAULT_ETH_PHY_TYPE),
  eth_mdc(DEFAULT_ETH_PIN_MDC),
  eth_mdio(DEFAULT_ETH_PIN_MDIO),
  eth_power(DEFAULT_ETH_PIN_POWER),
  eth_clock_mode(DEFAULT_ETH_CLOCK_MODE),
  network_medium(DEFAULT_NETWORK_MEDIUM)

{
  for (int i = 0; i < 4; ++i) {
    button[i] = -1;
    relais[i] = -1;
  }

#ifndef LIMIT_BUILD_SIZE

  switch (model) {
#if defined(ESP8266)
    case DeviceModel::DeviceModel_Sonoff_Basic:
    case DeviceModel::DeviceModel_Sonoff_TH1x:
    case DeviceModel::DeviceModel_Sonoff_S2x:
    case DeviceModel::DeviceModel_Sonoff_TouchT1:
    case DeviceModel::DeviceModel_Sonoff_POWr2:
      button[0]  = 0;  // Single Button
      relais[0]  = 12; // Red Led and Relay (0 = Off, 1 = On)
      status_led = 13; // Green/Blue Led (0 = On, 1 = Off)
      i2c_sda    = -1;
      i2c_scl    = -1;
      break;
    case DeviceModel::DeviceModel_Sonoff_POW:
      button[0]  = 0;  // Single Button
      relais[0]  = 12; // Red Led and Relay (0 = Off, 1 = On)
      status_led = 15; // Blue Led (0 = On, 1 = Off)
      i2c_sda    = -1;
      i2c_scl    = -1; // GPIO5 conflicts with HLW8012 Sel output
      break;
    case DeviceModel::DeviceModel_Sonoff_TouchT2:
      button[0]  = 0;  // Button 1
      button[1]  = 9;  // Button 2
      relais[0]  = 12; // Led and Relay1 (0 = Off, 1 = On)
      relais[1]  = 4;  // Led and Relay2 (0 = Off, 1 = On)
      status_led = 13; // Blue Led (0 = On, 1 = Off)
      i2c_sda    = -1; // GPIO4 conflicts with GPIO_REL3
      i2c_scl    = -1; // GPIO5 conflicts with GPIO_REL2
      break;
    case DeviceModel::DeviceModel_Sonoff_TouchT3:
      button[0]  = 0;  // Button 1
      button[1]  = 10; // Button 2
      button[2]  = 9;  // Button 3
      relais[0]  = 12; // Led and Relay1 (0 = Off, 1 = On)
      relais[1]  = 5;  // Led and Relay2 (0 = Off, 1 = On)
      relais[2]  = 4;  // Led and Relay3 (0 = Off, 1 = On)
      status_led = 13; // Blue Led (0 = On, 1 = Off)
      i2c_sda    = -1; // GPIO4 conflicts with GPIO_REL3
      i2c_scl    = -1; // GPIO5 conflicts with GPIO_REL2
      break;

    case DeviceModel::DeviceModel_Sonoff_4ch:
      button[0]  = 0;             // Button 1
      button[1]  = 9;             // Button 2
      button[2]  = 10;            // Button 3
      button[3]  = 14;            // Button 4
      relais[0]  = 12;            // Red Led and Relay1 (0 = Off, 1 = On)
      relais[1]  = 5;             // Red Led and Relay2 (0 = Off, 1 = On)
      relais[2]  = 4;             // Red Led and Relay3 (0 = Off, 1 = On)
      relais[3]  = 15;            // Red Led and Relay4 (0 = Off, 1 = On)
      status_led = 13;            // Blue Led (0 = On, 1 = Off)
      i2c_sda    = -1;            // GPIO4 conflicts with GPIO_REL3
      i2c_scl    = -1;            // GPIO5 conflicts with GPIO_REL2
      break;
    case DeviceModel::DeviceModel_Shelly1:
      button[0]  = 5;             // Single Button
      relais[0]  = 4;             // Red Led and Relay (0 = Off, 1 = On)
      status_led = 15;            // Blue Led (0 = On, 1 = Off)
      i2c_sda    = -1;            // GPIO4 conflicts with relay control.
      i2c_scl    = -1;            // GPIO5 conflicts with SW input
      break;
    case DeviceModel::DeviceModel_ShellyPLUG_S:
      button[0]  = 13;            // Single Button
      relais[0]  = 15;            // Red Led and Relay (0 = Off, 1 = On)
      status_led = 2;             // Blue Led (0 = On, 1 = Off)
      i2c_sda    = -1;            // GPIO4 conflicts with relay control.
      i2c_scl    = -1;            // GPIO5 conflicts with SW input
      break;
#else
    case DeviceModel::DeviceModel_Sonoff_Basic:
    case DeviceModel::DeviceModel_Sonoff_TH1x:
    case DeviceModel::DeviceModel_Sonoff_S2x:
    case DeviceModel::DeviceModel_Sonoff_TouchT1:
    case DeviceModel::DeviceModel_Sonoff_POWr2:
    case DeviceModel::DeviceModel_Sonoff_POW:
    case DeviceModel::DeviceModel_Sonoff_TouchT2:
    case DeviceModel::DeviceModel_Sonoff_TouchT3:
    case DeviceModel::DeviceModel_Sonoff_4ch:
    case DeviceModel::DeviceModel_Shelly1:
    case DeviceModel::DeviceModel_ShellyPLUG_S:
      break;
#endif


#ifdef ESP32
    case DeviceModel::DeviceModel_Olimex_ESP32_PoE:
      button[0]             = 34; // BUT1 Button
      relais[0]             = -1; // No LED's or relays on board
      status_led            = -1;
      i2c_sda               = 13;
      i2c_scl               = 16;
      eth_phyaddr           = 0;
      eth_phytype           = EthPhyType_t::LAN8710;
      eth_mdc               = 23;
      eth_mdio              = 18;
      eth_power             = 12;
      eth_clock_mode        = EthClockMode_t::Int_50MHz_GPIO_17_inv;
      network_medium = NetworkMedium_t::Ethernet;
      break;
    case DeviceModel::DeviceModel_Olimex_ESP32_EVB:
      button[0] = 34; // BUT1 Button
      relais[0] = 32; // LED1 + Relay1 (0 = Off, 1 = On)
      relais[1] = 33; // LED2 + Relay2 (0 = Off, 1 = On)

      status_led            = -1;
      i2c_sda               = 13;
      i2c_scl               = 16;
      eth_phyaddr           = 0;
      eth_phytype           = EthPhyType_t::LAN8710;
      eth_mdc               = 23;
      eth_mdio              = 18;
      eth_power             = -1; // No Ethernet power pin
      eth_clock_mode        = EthClockMode_t::Ext_crystal_osc;
      network_medium = NetworkMedium_t::Ethernet;
      break;

    case DeviceModel::DeviceModel_Olimex_ESP32_GATEWAY:
      button[0]             = 34; // BUT1 Button
      relais[0]             = -1; // No LED's or relays on board
      status_led            = 33;
      i2c_sda               = -1;
      i2c_scl               = -1;
      eth_phyaddr           = 0;
      eth_phytype           = EthPhyType_t::LAN8710;
      eth_mdc               = 23;
      eth_mdio              = 18;
      eth_power             = 5;
      eth_clock_mode        = EthClockMode_t::Int_50MHz_GPIO_17_inv;
      network_medium = NetworkMedium_t::Ethernet;
      // Rev A to E:
      // GPIO 5, 17 can be used only if Ethernet functionality is not used
      // GPIO 6, 7, 8, 9, 10, 11 used for internal flash and SD card
      // GPIO 33 - Status LED
      // GPIO 34 - User button
      // GPIO 16, 32, 35, 36, 39 free to use

      // Rev F and up:
      // GPIO 5, 17 can be used only if Ethernet functionality is not used
      // GPIO 2, 14, 15 are used for SD card, they are free to use if SD is not used.
      // GPIO 33 - Status LED
      // GPIO 34 - User button
      // GPIO 4, 12, 13, 32, 35, 36, 39 free to use

      // ESPEasy default setting:
      // No GPIO pins selected in profile for I2C.
      // Since there are none free to use in all revisions capable of input/output.
      // N.B. GPIO 35 and up are input only.

      break;

    case DeviceModel::DeviceModel_wESP32:
      status_led            = -1;
      i2c_sda               = 15;
      i2c_scl               = 4;
      eth_phyaddr           = 0;
      eth_phytype           = EthPhyType_t::LAN8710;
      eth_mdc               = 16;
      eth_mdio              = 17;
      eth_power             = -1;
      eth_clock_mode        = EthClockMode_t::Ext_crystal_osc;
      network_medium = NetworkMedium_t::Ethernet;
      break;

    case DeviceModel::DeviceModel_WT32_ETH01:
      status_led            = -1;
      i2c_sda               = 21;
      i2c_scl               = 22;
      eth_phyaddr           = 1;
      eth_phytype           = EthPhyType_t::LAN8710;
      eth_mdc               = 23;
      eth_mdio              = 18;
      eth_power             = 12;  // TODO TD-er: Better to use GPIO-16? as shown here: https://letscontrolit.com/forum/viewtopic.php?p=50133#p50133
      eth_clock_mode        = EthClockMode_t::Ext_crystal_osc;
      network_medium = NetworkMedium_t::Ethernet;
      break;

  #else
      case DeviceModel::DeviceModel_Olimex_ESP32_PoE:
      case DeviceModel::DeviceModel_Olimex_ESP32_EVB:
      case DeviceModel::DeviceModel_Olimex_ESP32_GATEWAY:
      case DeviceModel::DeviceModel_wESP32:
      case DeviceModel::DeviceModel_WT32_ETH01:
  #endif

    case DeviceModel::DeviceModel_default:
    case DeviceModel::DeviceModel_MAX:
      break;

      // Do not use default: as this allows the compiler to detect any missing cases.
      // default: break;
  }
  #endif
}

#include "../DataStructs/I2CTypes.h"

const __FlashStringHelper* toString(I2C_bus_state state) {
  switch (state) {
    case I2C_bus_state::BusCleared:            return F("Bus Cleared");
    case I2C_bus_state::OK:                    return F("OK");
    case I2C_bus_state::NotConfigured:         return F("Not Configured");
    case I2C_bus_state::ClearingProcessActive: return F("Clearing Process Active");
    case I2C_bus_state::SCL_Low:               return F("SCL Low");
    case I2C_bus_state::SDA_Low_over_2_sec:    return F("SCL Low by I2C device clock stretch > 2 sec");
    case I2C_bus_state::SDA_Low_20_clocks:     return F("SDA Low");
  }
  return F("");
}

#include "../DataStructs/CRCStruct.h"

#include <string.h>

bool CRCStruct::checkPassed() const
{
  return memcmp(compileTimeMD5, runTimeMD5, 16) == 0;
}

