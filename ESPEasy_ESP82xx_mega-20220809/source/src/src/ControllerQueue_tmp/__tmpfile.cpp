#include "../ControllerQueue/C016_queue_element.h"

#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../Globals/Plugins.h"
#include "../Globals/RuntimeData.h"
#include "../Helpers/ESPEasy_math.h"

#ifdef USES_C016

C016_queue_element::C016_queue_element() : _timestamp(0), TaskIndex(INVALID_TASK_INDEX), controller_idx(0), sensorType(
    Sensor_VType::SENSOR_TYPE_NONE) {}

C016_queue_element::C016_queue_element(C016_queue_element&& other)
  : _timestamp(other._timestamp)
  , TaskIndex(other.TaskIndex)
  , controller_idx(other.controller_idx)
  , sensorType(other.sensorType)
  , valueCount(other.valueCount)
{
  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    values[i] = other.values[i];
  }
}

C016_queue_element::C016_queue_element(const struct EventStruct *event, uint8_t value_count, unsigned long unixTime) :
  _timestamp(unixTime),
  TaskIndex(event->TaskIndex),
  controller_idx(event->ControllerIndex),
  sensorType(event->sensorType),
  valueCount(value_count)
{
  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    if (i < value_count) {
      values[i] = UserVar[event->BaseVarIndex + i];
    } else {
      values[i] = 0.0f;
    }
  }
}

C016_queue_element& C016_queue_element::operator=(C016_queue_element&& other) {
  _timestamp = other._timestamp;
  TaskIndex = other.TaskIndex;
  controller_idx = other.controller_idx;
  sensorType = other.sensorType;
  valueCount = other.valueCount;
  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    values[i] = other.values[i];
  }
  return *this;
}

size_t C016_queue_element::getSize() const {
  return sizeof(*this);
}

bool C016_queue_element::isDuplicate(const C016_queue_element& other) const {
  if ((other.controller_idx != controller_idx) ||
      (other.TaskIndex != TaskIndex) ||
      (other.sensorType != sensorType) ||
      (other.valueCount != valueCount)) {
    return false;
  }

  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    if (!essentiallyEqual(other.values[i] , values[i])) {
      return false;
    }
  }
  return true;
}

#endif // ifdef USES_C016

#include "../ControllerQueue/SimpleQueueElement_formatted_Strings.h"

#include "../DataStructs/ESPEasy_EventStruct.h"
#include "../Helpers/StringConverter.h"

#include "../../_Plugin_Helper.h"


SimpleQueueElement_formatted_Strings::SimpleQueueElement_formatted_Strings(struct EventStruct *event) :
  idx(event->idx),
  TaskIndex(event->TaskIndex),
  controller_idx(event->ControllerIndex),
  sensorType(event->sensorType),
  valuesSent(0) 
{
  #ifdef USE_SECOND_HEAP
  HeapSelectIram ephemeral;
  #endif

  valueCount = getValueCountForTask(TaskIndex);

  for (uint8_t i = 0; i < valueCount; ++i) {
    txt[i] = formatUserVarNoCheck(event, i);
  }
}

SimpleQueueElement_formatted_Strings::SimpleQueueElement_formatted_Strings(const struct EventStruct *event, uint8_t value_count) :
  idx(event->idx),
  TaskIndex(event->TaskIndex),
  controller_idx(event->ControllerIndex),
  sensorType(event->sensorType),
  valuesSent(0),
  valueCount(value_count) {}

SimpleQueueElement_formatted_Strings::SimpleQueueElement_formatted_Strings(SimpleQueueElement_formatted_Strings&& rval)
  : idx(rval.idx), _timestamp(rval._timestamp),  TaskIndex(rval.TaskIndex),
  controller_idx(rval.controller_idx), sensorType(rval.sensorType),
  valuesSent(rval.valuesSent), valueCount(rval.valueCount)
{
  #ifdef USE_SECOND_HEAP
  HeapSelectIram ephemeral;
  #endif

  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    txt[i] = std::move(rval.txt[i]);
  }
}

SimpleQueueElement_formatted_Strings& SimpleQueueElement_formatted_Strings::operator=(SimpleQueueElement_formatted_Strings&& rval) {
  idx            = rval.idx;
  _timestamp     = rval._timestamp;
  TaskIndex      = rval.TaskIndex;
  controller_idx = rval.controller_idx;
  sensorType     = rval.sensorType;
  valuesSent     = rval.valuesSent;
  valueCount     = rval.valueCount;

  for (size_t i = 0; i < VARS_PER_TASK; ++i) {
    #ifdef USE_SECOND_HEAP
    HeapSelectIram ephemeral;
    if (rval.txt[i].length() && !mmu_is_iram(&(rval.txt[i][0]))) {
      txt[i] = rval.txt[i];
    } else {
      txt[i] = std::move(rval.txt[i]);
    }
    #else
    txt[i] = std::move(rval.txt[i]);
    #endif // ifdef USE_SECOND_HEAP
  }
  return *this;
}

bool SimpleQueueElement_formatted_Strings::checkDone(bool succesfull) const {
  if (succesfull) { ++valuesSent; }
  return valuesSent >= valueCount || valuesSent >= VARS_PER_TASK;
}

size_t SimpleQueueElement_formatted_Strings::getSize() const {
  size_t total = sizeof(*this);

  for (int i = 0; i < VARS_PER_TASK; ++i) {
    total += txt[i].length();
  }
  return total;
}

bool SimpleQueueElement_formatted_Strings::isDuplicate(const SimpleQueueElement_formatted_Strings& rval) const {
  if ((rval.controller_idx != controller_idx) ||
      (rval.TaskIndex != TaskIndex) ||
      (rval.sensorType != sensorType) ||
      (rval.valueCount != valueCount) ||
      (rval.idx != idx)) {
    return false;
  }

  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    if (rval.txt[i] != txt[i]) {
      return false;
    }
  }
  return true;
}

#include "../ControllerQueue/DelayQueueElements.h"

#include "../DataStructs/ControllerSettingsStruct.h"
#include "../DataStructs/TimingStats.h"
#include "../Globals/ESPEasy_Scheduler.h"
#include "../Helpers/PeriodicalActions.h"

#if FEATURE_MQTT
ControllerDelayHandlerStruct<MQTT_queue_element> *MQTTDelayHandler = nullptr;

bool init_mqtt_delay_queue(controllerIndex_t ControllerIndex, String& pubname, bool& retainFlag) {
  MakeControllerSettings(ControllerSettings); //-V522
  if (!AllocatedControllerSettings()) {
    return false;
  }
  LoadControllerSettings(ControllerIndex, ControllerSettings);
  if (MQTTDelayHandler == nullptr) {
    #ifdef USE_SECOND_HEAP
    HeapSelectIram ephemeral;
    #endif

    MQTTDelayHandler = new (std::nothrow) ControllerDelayHandlerStruct<MQTT_queue_element>;
  }
  if (MQTTDelayHandler == nullptr) {
    return false;
  }
  MQTTDelayHandler->configureControllerSettings(ControllerSettings);
  pubname = ControllerSettings.Publish;
  retainFlag = ControllerSettings.mqtt_retainFlag();
  Scheduler.setIntervalTimerOverride(ESPEasy_Scheduler::IntervalTimer_e::TIMER_MQTT, 10); // Make sure the MQTT is being processed as soon as possible.
  scheduleNextMQTTdelayQueue();
  return true;
}

void exit_mqtt_delay_queue() {
  if (MQTTDelayHandler != nullptr) {
    delete MQTTDelayHandler;
    MQTTDelayHandler = nullptr;
  }
}

#endif // if FEATURE_MQTT


/*********************************************************************************************\
* C001_queue_element for queueing requests for C001.
\*********************************************************************************************/
#ifdef USES_C001
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(00,  1)  // -V522
#endif // ifdef USES_C001

/*********************************************************************************************\
* C003_queue_element for queueing requests for C003 Nodo Telnet.
\*********************************************************************************************/
#ifdef USES_C003
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(00,  3)  // -V522
#endif // ifdef USES_C003

#ifdef USES_C004
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(00,  4)  // -V522
#endif // ifdef USES_C004

#ifdef USES_C007
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(00,  7)  // -V522
#endif // ifdef USES_C007



/*********************************************************************************************\
* C008_queue_element for queueing requests for 008: Generic HTTP
* Using SimpleQueueElement_formatted_Strings
\*********************************************************************************************/
#ifdef USES_C008
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(00,  8)  // -V522
#endif // ifdef USES_C008

#ifdef USES_C009
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(00,  9)  // -V522
#endif // ifdef USES_C009


/*********************************************************************************************\
* C010_queue_element for queueing requests for 010: Generic UDP
* Using SimpleQueueElement_formatted_Strings
\*********************************************************************************************/
#ifdef USES_C010
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP( 0, 10)  // -V522
#endif // ifdef USES_C010



/*********************************************************************************************\
* C011_queue_element for queueing requests for 011: Generic HTTP Advanced
\*********************************************************************************************/
#ifdef USES_C011
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP( 0, 11)  // -V522
#endif // ifdef USES_C011


/*********************************************************************************************\
* C012_queue_element for queueing requests for 012: Blynk
* Using SimpleQueueElement_formatted_Strings
\*********************************************************************************************/
#ifdef USES_C012
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP( 0, 12)  // -V522
#endif // ifdef USES_C012

/*
 #ifdef USES_C013
   DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 13)  // -V522
 #endif
 */

/*
 #ifdef USES_C014
   DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 14)  // -V522
 #endif
 */


#ifdef USES_C015
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 15)  // -V522
#endif // ifdef USES_C015



#ifdef USES_C016
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 16)  // -V522
#endif // ifdef USES_C016


#ifdef USES_C017
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 17)  // -V522
#endif // ifdef USES_C017

#ifdef USES_C018
DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 18)  // -V522
#endif // ifdef USES_C018


/*
 #ifdef USES_C019
   DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 19)  // -V522
 #endif
 */

/*
 #ifdef USES_C020
   DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 20)  // -V522
 #endif
 */

/*
 #ifdef USES_C021
   DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 21)  // -V522
 #endif
 */

/*
 #ifdef USES_C022
   DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 22)  // -V522
 #endif
 */

/*
 #ifdef USES_C023
   DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 23)  // -V522
 #endif
 */

/*
 #ifdef USES_C024
   DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 24)  // -V522
 #endif
 */

/*
 #ifdef USES_C025
   DEFINE_Cxxx_DELAY_QUEUE_MACRO_CPP(0, 25)  // -V522
 #endif
 */



// When extending this, search for EXTEND_CONTROLLER_IDS 
// in the code to find all places that need to be updated too.

#include "../ControllerQueue/C011_queue_element.h"

#include "../DataStructs/ESPEasy_EventStruct.h"

#ifdef USES_C011


C011_queue_element::C011_queue_element(const struct EventStruct *event) :
  idx(event->idx),
  TaskIndex(event->TaskIndex),
  controller_idx(event->ControllerIndex),
  sensorType(event->sensorType) {}

size_t C011_queue_element::getSize() const {
  size_t total = sizeof(*this);

  total += uri.length();
  total += HttpMethod.length();
  total += header.length();
  total += postStr.length();
  return total;
}

bool C011_queue_element::isDuplicate(const C011_queue_element& other) const {
  if ((other.controller_idx != controller_idx) ||
      (other.TaskIndex != TaskIndex) ||
      (other.sensorType != sensorType) ||
      (other.idx != idx)) {
    return false;
  }
  return other.uri.equals(uri) &&
         other.HttpMethod.equals(HttpMethod) &&
         other.header.equals(header) &&
         other.postStr.equals(postStr);
}

#endif // ifdef USES_C011

#include "../ControllerQueue/C018_queue_element.h"

#ifdef USES_C018

#include "../DataStructs/ESPEasy_EventStruct.h"

#include "../ESPEasyCore/ESPEasy_Log.h"

#include "../Helpers/_CPlugin_LoRa_TTN_helper.h"

C018_queue_element::C018_queue_element(struct EventStruct *event, uint8_t sampleSetCount) :
  TaskIndex(event->TaskIndex),
  controller_idx(event->ControllerIndex)
{
  # if FEATURE_PACKED_RAW_DATA
    #ifdef USE_SECOND_HEAP
//    HeapSelectIram ephemeral;
    #endif

    packed = getPackedFromPlugin(event, sampleSetCount);

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("C018 queue element: ");
      log += packed;
      addLogMove(LOG_LEVEL_INFO, log);
    }
  # endif // if FEATURE_PACKED_RAW_DATA
}

size_t C018_queue_element::getSize() const {
  return sizeof(*this) + packed.length();
}

bool C018_queue_element::isDuplicate(const C018_queue_element& other) const {
  if ((other.controller_idx != controller_idx) ||
      (other.TaskIndex != TaskIndex) ||
      (other.packed != packed)) {
    return false;
  }
  return true;
}

#endif // ifdef USES_C018

#include "../ControllerQueue/MQTT_queue_element.h"

#if FEATURE_MQTT

MQTT_queue_element::MQTT_queue_element(int ctrl_idx,
                                       taskIndex_t TaskIndex,
                                       const String& topic, const String& payload, bool retained) :
  TaskIndex(TaskIndex), controller_idx(ctrl_idx), _retained(retained)
{
  #ifdef USE_SECOND_HEAP
  HeapSelectIram ephemeral;
  #endif
  // Copy in the scope of the constructor, so we might store it in the 2nd heap
  _topic = topic;
  _payload = payload;

  removeEmptyTopics();
}

MQTT_queue_element::MQTT_queue_element(int         ctrl_idx,
                                       taskIndex_t TaskIndex,
                                       String   && topic,
                                       String   && payload,
                                       bool        retained)
  : TaskIndex(TaskIndex), controller_idx(ctrl_idx), _retained(retained)
{
  // Copy in the scope of the constructor, so we might store it in the 2nd heap
  #ifdef USE_SECOND_HEAP
  HeapSelectIram ephemeral;
  if (topic.length() && !mmu_is_iram(&(topic[0]))) {
    _topic = topic;
  } else {
    _topic = std::move(topic);  
  }
  if (payload.length() && !mmu_is_iram(&(payload[0]))) {
    _payload = payload;
  } else {
    _payload = std::move(payload);
  }
  #else
  _topic = std::move(topic);
  _payload = std::move(payload);
  #endif

  removeEmptyTopics();
}

size_t MQTT_queue_element::getSize() const {
  return sizeof(*this) + _topic.length() + _payload.length();
}

bool MQTT_queue_element::isDuplicate(const MQTT_queue_element& other) const {
  if ((other.controller_idx != controller_idx) ||
      (other._retained != _retained) ||
      other._topic != _topic ||
      other._payload != _payload) {
    return false;
  }
  return true;
}

void MQTT_queue_element::removeEmptyTopics() {
  // some parts of the topic may have been replaced by empty strings,
  // or "/status" may have been appended to a topic ending with a "/"
  // Get rid of "//"
  while (_topic.indexOf(F("//")) != -1) {
    _topic.replace(F("//"), F("/"));
  }
}
#endif // if FEATURE_MQTT

#include "../ControllerQueue/C015_queue_element.h"

#include "../DataStructs/ESPEasy_EventStruct.h"

#ifdef USES_C015

C015_queue_element::C015_queue_element(C015_queue_element&& other)
  : idx(other.idx), _timestamp(other._timestamp), TaskIndex(other.TaskIndex)
  , controller_idx(other.controller_idx), valuesSent(other.valuesSent)
  , valueCount(other.valueCount)
{
  #ifdef USE_SECOND_HEAP
  HeapSelectIram ephemeral;
  #endif

  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    txt[i]  = std::move(other.txt[i]);
    vPin[i] = other.vPin[i];
  }
}

C015_queue_element::C015_queue_element(const struct EventStruct *event, uint8_t value_count) :
  idx(event->idx),
  TaskIndex(event->TaskIndex),
  controller_idx(event->ControllerIndex),
  valuesSent(0),
  valueCount(value_count) {}

C015_queue_element& C015_queue_element::operator=(C015_queue_element&& other) {
  idx = other.idx;
  _timestamp = other._timestamp;
  TaskIndex  = other.TaskIndex;
  controller_idx = other.controller_idx;
  valuesSent = other.valuesSent;
  valueCount = other.valueCount;
  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    txt[i]  = std::move(other.txt[i]);
    vPin[i] = other.vPin[i];
  }
  return *this;
}

bool C015_queue_element::checkDone(bool succesfull) const {
  if (succesfull) { ++valuesSent; }
  return valuesSent >= valueCount || valuesSent >= VARS_PER_TASK;
}

size_t C015_queue_element::getSize() const {
  size_t total = sizeof(*this);

  for (int i = 0; i < VARS_PER_TASK; ++i) {
    total += txt[i].length();
  }
  return total;
}

bool C015_queue_element::isDuplicate(const C015_queue_element& other) const {
  if ((other.controller_idx != controller_idx) ||
      (other.TaskIndex != TaskIndex) ||
      (other.valueCount != valueCount) ||
      (other.idx != idx)) {
    return false;
  }

  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    if (other.txt[i] != txt[i]) {
      return false;
    }

    if (other.vPin[i] != vPin[i]) {
      return false;
    }
  }
  return true;
}

#endif // ifdef USES_C015

#include "../ControllerQueue/SimpleQueueElement_string_only.h"


simple_queue_element_string_only::simple_queue_element_string_only(int ctrl_idx, taskIndex_t TaskIndex,  String&& req) :
  TaskIndex(TaskIndex), controller_idx(ctrl_idx)
{
  #ifdef USE_SECOND_HEAP
  HeapSelectIram ephemeral;
  if (req.length() > 0 && !mmu_is_iram(&(req[0]))) {
    // The string was not allocated on the 2nd heap, so copy instead of move
    txt = req;
  } else {
    txt = std::move(req);
  }
  #else
  txt = std::move(req);
  #endif
}

size_t simple_queue_element_string_only::getSize() const {
  return sizeof(*this) + txt.length();
}

bool simple_queue_element_string_only::isDuplicate(const simple_queue_element_string_only& other) const {
  if ((other.controller_idx != controller_idx) ||
      (other.TaskIndex != TaskIndex) ||
      (other.txt != txt)) {
    return false;
  }
  return true;
}

