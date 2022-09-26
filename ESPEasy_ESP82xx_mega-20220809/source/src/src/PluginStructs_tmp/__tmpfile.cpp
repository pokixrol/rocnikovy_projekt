#include "../PluginStructs/P132_data_struct.h"

#ifdef USES_P132

// **************************************************************************/
// Constructor
// **************************************************************************/
P132_data_struct::P132_data_struct(struct EventStruct *event) {
  _i2c_address = P132_I2C_ADDR;
  setCalibration_INA3221(event);
}

// **************************************************************************/
// Destructor
// **************************************************************************/
P132_data_struct::~P132_data_struct() {}

// **************************************************************************/
// Gets the raw bus voltage  (7FF8 / 32760) LSB 8mV
// **************************************************************************/
int16_t P132_data_struct::getBusVoltage_raw(byte reg) {
  uint16_t value = I2C_read16_reg(_i2c_address, reg);

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB 8 mV
  # ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("INA3221: get raw bus ");
    log += value;
    log += F(" reg - ");
    log += reg;
    addLog(LOG_LEVEL_DEBUG, log);
  }
  # endif // ifndef BUILD_NO_DEBUG
  return (int16_t)((value >> 3) * 8);
}

// **************************************************************************/
// Gets the raw shunt voltage (integer, so +-32760) LSB 40 uV
// **************************************************************************/
int16_t P132_data_struct::getShuntVoltage_raw(byte reg) {
  uint16_t value = I2C_read16_reg(_i2c_address, reg);

  # ifndef BUILD_NO_DEBUG
  String log = F("INA3221: get raw shunt voltage ");
  log += value;
  log += F(" value2 - ");
  # endif // ifndef BUILD_NO_DEBUG

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  if (value > 32767) {              // check value is negative
    //		value = 0;  // no negative measure
    value = ((value >> 3) | 57344); // correct int16_t value
    # ifndef BUILD_NO_DEBUG
    log += F(" value_neg - ");
    log += value;
    # endif // ifndef BUILD_NO_DEBUG
  } else {
    value = (value >> 3);
    # ifndef BUILD_NO_DEBUG
    log += F(" value_pos - ");
    log += value;
    # endif // ifndef BUILD_NO_DEBUG
  }
  # ifndef BUILD_NO_DEBUG
  log += F(" reg - ");
  log += reg;
  addLog(LOG_LEVEL_DEBUG, log);
  # endif // ifndef BUILD_NO_DEBUG
  return value;
}

// **************************************************************************/
// Gets the shunt voltage in mV (32760 so +-163.8 mV) 7ff8 LSB 40uV
// **************************************************************************/
float P132_data_struct::getShuntVoltage_mV(byte reg) {
  int16_t value = getShuntVoltage_raw(reg);

  # ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("INA3221: shunt voltage in mV * 0.04 ");
    log += value;
    log += F(" reg - ");
    log += reg;
    addLog(LOG_LEVEL_DEBUG, log);
  }
  # endif // ifndef BUILD_NO_DEBUG
  return value * 0.04f;
}

// **************************************************************************/
// Gets the Bus voltage in volts
// **************************************************************************/
float P132_data_struct::getBusVoltage_V(byte reg) {
  int16_t value = getBusVoltage_raw(reg);

  # ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("INA3221: get bus voltage ");
    log += value;
    log += F(" reg - ");
    log += reg;
    addLog(LOG_LEVEL_DEBUG, log);
  }
  # endif // ifndef BUILD_NO_DEBUG
  return value * 0.001f;
}

// **************************************************************************/
// Configures to INA3221
// **************************************************************************/
void P132_data_struct::setCalibration_INA3221(struct EventStruct *event) {
  // Set Config register
  uint32_t config = I2C_read16_reg(_i2c_address, 0x00); // read, chip default: 0x7127
  uint16_t mfgid  = I2C_read16_reg(_i2c_address, 0xFE); // read manufacturer ID, should be 0x5449

  set3BitToUL(config, INA3221_AVERAGE_BIT,          P132_GET_AVERAGE);
  set3BitToUL(config, INA3221_CONVERSION_BUS_BIT,   P132_GET_CONVERSION_B);
  set3BitToUL(config, INA3221_CONVERSION_SHUNT_BIT, P132_GET_CONVERSION_S);

  # ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("INA3221: init I2C: 0x");
    log += String(_i2c_address, HEX);
    log += F(" mfg: 0x");
    log += String(mfgid, HEX);
    log += F(", config: 0x");
    log += String(config, HEX);
    log += F(", 0b");
    log += String(config, BIN);
    addLog(LOG_LEVEL_INFO, log);
  }
  # endif // ifndef BUILD_NO_DEBUG

  if (mfgid != 0x5449) {
    addLogMove(LOG_LEVEL_ERROR, F("INA3221: Invalid Manufacturer ID! (0x5449)"));
  }

  I2C_write16_reg(_i2c_address, 0x00, static_cast<uint16_t>(config));
}

#endif // ifdef USES_P132

#include "../PluginStructs/P053_data_struct.h"

#ifdef USES_P053

const __FlashStringHelper* toString(PMSx003_type sensorType) {
  switch (sensorType) {
    case PMSx003_type::PMS1003_5003_7003: return F("PMS1003 / PMS5003 / PMS7003");
    # ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
    case PMSx003_type::PMS2003_3003: return F("PMS2003 / PMS3003");
    case PMSx003_type::PMS5003_S:    return F("PMS5003S");
    case PMSx003_type::PMS5003_T:    return F("PMS5003T");
    case PMSx003_type::PMS5003_ST:   return F("PMS5003ST");
    # endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  }
  return F("Unknown");
}

const __FlashStringHelper* toString(PMSx003_output_selection selection) {
  switch (selection) {
    case PMSx003_output_selection::Particles_ug_m3: return F("Particles &micro;g/m3: pm1.0, pm2.5, pm10");
    case PMSx003_output_selection::PM2_5_TempHum_Formaldehyde:  return F("Particles &micro;g/m3: pm2.5; Other: Temp, Humi, HCHO (PMS5003ST)");
    case PMSx003_output_selection::ParticlesCount_100ml_cnt1_0_cnt2_5_cnt10:
      return F("Particles count/0.1L: cnt1.0, cnt2.5, cnt5, cnt10 (PMS1003/5003(ST)/7003)");
    case PMSx003_output_selection::ParticlesCount_100ml_cnt0_3__cnt_2_5:
      return F("Particles count/0.1L: cnt0.3, cnt0.5, cnt1.0, cnt2.5 (PMS1003/5003(ST)/7003)");
  }
  return F("Unknown");
}

const __FlashStringHelper* toString(PMSx003_event_datatype selection) {
  switch (selection) {
    case PMSx003_event_datatype::Event_None:       return F("None");
    case PMSx003_event_datatype::Event_PMxx_TempHum_Formaldehyde:  return F("Particles &micro;g/m3 and Temp/Humi/HCHO");
    case PMSx003_event_datatype::Event_All:  return F("Particles &micro;g/m3, Temp/Humi/HCHO and Particles count/0.1L");
    case PMSx003_event_datatype::Event_All_count_bins: return F("Particles count/0.1L");
  }
  return F("Unknown");
}

P053_data_struct::P053_data_struct(
  taskIndex_t             TaskIndex,
  int8_t                  rxPin,
  int8_t                  txPin,
  const ESPEasySerialPort port,
  int8_t                  resetPin,
  int8_t                  pwrPin,
  PMSx003_type            sensortype,
  uint32_t                delay_read_after_wakeup_ms
  # ifdef                 PLUGIN_053_ENABLE_EXTRA_SENSORS
  , bool                  oversample
  , bool                  splitCntBins
  # endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  )
  : _taskIndex(TaskIndex),
  _sensortype(sensortype),
  # ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  _oversample(oversample),
  _splitCntBins(splitCntBins),
  # endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  _delay_read_after_wakeup_ms(delay_read_after_wakeup_ms),
  _resetPin(resetPin), _pwrPin(pwrPin)
{
  # ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log;
    log.reserve(25);
    log  = F("PMSx003 : config ");
    log += rxPin;
    log += ' ';
    log += txPin;
    log += ' ';
    log += _resetPin;
    log += ' ';
    log += _pwrPin;
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
  # endif // ifndef BUILD_NO_DEBUG

  if (port == ESPEasySerialPort::software) {
    addLog(LOG_LEVEL_INFO, F("PMSx003 : using software serial"));
  } else {
    addLog(LOG_LEVEL_INFO, F("PMSx003 : using hardware serial"));
  }
  _easySerial = new (std::nothrow) ESPeasySerial(port, rxPin, txPin, false, 96); // 96 Bytes buffer, enough for up to 3 packets.

  if (_easySerial != nullptr) {
    _easySerial->begin(9600);
    _easySerial->flush();

    wakeSensor();

    resetSensor();

    // Make sure to set the mode to active reading mode.
    // This is the default, but not sure if passive mode can be set persistant.
    setActiveReadingMode();
  }
  clearReceivedData();
}

P053_data_struct::~P053_data_struct() {
  if (_easySerial != nullptr) {
    delete _easySerial;
    _easySerial = nullptr;
  }
}

bool P053_data_struct::initialized() const
{
  return _easySerial != nullptr;
}

// Read 2 bytes from serial and make an uint16 of it. Additionally calculate
// checksum for PMSx003. Assumption is that there is data available, otherwise
// this function is blocking.
void P053_data_struct::SerialRead16(uint16_t& value, uint16_t *checksum)
{
  if (!initialized()) { return; }
  const uint8_t data_high = _easySerial->read();
  const uint8_t data_low  = _easySerial->read();

  value  = data_low;
  value |= (data_high << 8);

  if (checksum != nullptr)
  {
    *checksum += data_high;
    *checksum += data_low;
  }

  # ifdef P053_LOW_LEVEL_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    // Low-level logging to see data from sensor
    String log = F("PMSx003 : uint8_t high=0x");
    log += String(data_high, HEX);
    log += F(" uint8_t low=0x");
    log += String(data_low, HEX);
    log += F(" result=0x");
    log += String(value, HEX);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // ifdef P053_LOW_LEVEL_DEBUG
}

void P053_data_struct::SerialFlush() {
  if (_easySerial != nullptr) {
    _easySerial->flush();
  }
}

uint8_t P053_data_struct::packetSize() const {
  switch (_sensortype) {
    case PMSx003_type::PMS1003_5003_7003:    return PMS1003_5003_7003_SIZE;
    # ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
    case PMSx003_type::PMS5003_S:    return PMS5003_S_SIZE;
    case PMSx003_type::PMS5003_T:    return PMS5003_T_SIZE;
    case PMSx003_type::PMS5003_ST:   return PMS5003_ST_SIZE;
    case PMSx003_type::PMS2003_3003: return PMS2003_3003_SIZE;
    # endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  }
  return 0u;
}

bool P053_data_struct::packetAvailable()
{
  if (_easySerial != nullptr)
  {
    // When there is enough data in the buffer, search through the buffer to
    // find header (buffer may be out of sync)
    if (!_easySerial->available()) { return false; }

    while ((_easySerial->peek() != PMSx003_SIG1) && _easySerial->available()) {
      _easySerial->read(); // Read until the buffer starts with the
      // first uint8_t of a message, or buffer
      // empty.
    }

    if (_easySerial->available() < packetSize()) {
      return false; // Not enough yet for a complete packet
    }
  }
  return true;
}

# ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
void P053_data_struct::sendEvent(const String& baseEvent,
                                 uint8_t       index) {
  float value = 0.0f;

  if (!getValue(index, value)) { return; }


  String valueEvent;

  valueEvent.reserve(32);
  const unsigned char nrDecimals = getNrDecimals(index, _oversample);

  // Temperature
  valueEvent  = baseEvent;
  valueEvent += getEventString(index);
  valueEvent += '=';
  valueEvent += toString(value, nrDecimals);
  eventQueue.addMove(std::move(valueEvent));
}

bool P053_data_struct::hasFormaldehyde() const {
  #  ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  return _sensortype == PMSx003_type::PMS5003_S ||
         _sensortype == PMSx003_type::PMS5003_ST;
  #  else // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  return false;
  #  endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
}

bool P053_data_struct::hasTempHum() const {
  #  ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  return _sensortype == PMSx003_type::PMS5003_T ||
         _sensortype == PMSx003_type::PMS5003_ST;
  #  else // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  return false;
  #  endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
}

# endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS

bool P053_data_struct::processData(struct EventStruct *event) {
  uint16_t checksum = 0, checksum2 = 0;
  uint16_t framelength   = 0;
  uint16_t packet_header = 0;

  SerialRead16(packet_header, &checksum); // read PMSx003_SIG1 + PMSx003_SIG2

  if (packet_header != ((PMSx003_SIG1 << 8) | PMSx003_SIG2)) {
    // Not the start of the packet, stop reading.
    return false;
  }

  SerialRead16(framelength, &checksum);

  if ((framelength + 4) != packetSize()) {
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log;
      log.reserve(34);
      log  = F("PMSx003 : invalid framelength - ");
      log += framelength;
      addLogMove(LOG_LEVEL_ERROR, log);
    }
    return false;
  }

  uint8_t frameData = packetSize();

  if (frameData > 0u) {
    frameData /= 2;                               // Each value is 16 bits
    frameData -= 3;                               // start markers, length, checksum
  }
  uint16_t data[PMS_RECEIVE_BUFFER_SIZE] = { 0 }; // uint8_t data_low, data_high;

  for (uint8_t i = 0; i < frameData; i++) {
    SerialRead16(data[i], &checksum);
  }

  # ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS

  if (GET_PLUGIN_053_SENSOR_MODEL_SELECTOR == PMSx003_type::PMS5003_T) {
    data[PMS_Temp_C]  = data[PMS_T_Temp_C]; // Move data to the 'usual' location for Temp/Hum
    data[PMS_Hum_pct] = data[PMS_T_Hum_pct];
  }
  # endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS

  # ifndef BUILD_NO_DEBUG
  #  ifdef P053_LOW_LEVEL_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) { // Available on all supported sensor models
    String log;
    if (log.reserve(87)) {
      log  = F("PMSx003 : pm1.0=");
      log += data[PMS_PM1_0_ug_m3_factory];
      log += F(", pm2.5=");
      log += data[PMS_PM2_5_ug_m3_factory];
      log += F(", pm10=");
      log += data[PMS_PM10_0_ug_m3_factory];
      log += F(", pm1.0a=");
      log += data[PMS_PM1_0_ug_m3_normal];
      log += F(", pm2.5a=");
      log += data[PMS_PM2_5_ug_m3_normal];
      log += F(", pm10a=");
      log += data[PMS_PM10_0_ug_m3_normal];
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
  }

  #   ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)
      && (GET_PLUGIN_053_SENSOR_MODEL_SELECTOR != PMSx003_type::PMS2003_3003)) { // 'Count' values not available on
    // PMS2003/PMS3003 models
    // (handled as 1 model in code)
    String log;
    if (log.reserve(96)) {
      log  = F("PMSx003 : count/0.1L : 0.3um=");
      log += data[PMS_cnt0_3_100ml];
      log += F(", 0.5um=");
      log += data[PMS_cnt0_5_100ml];
      log += F(", 1.0um=");
      log += data[PMS_cnt1_0_100ml];
      log += F(", 2.5um=");
      log += data[PMS_cnt2_5_100ml];
      log += F(", 5.0um=");
      log += data[PMS_cnt5_0_100ml];
      log += F(", 10um=");
      log += data[PMS_cnt10_0_100ml];
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
  }


  if (loglevelActiveFor(LOG_LEVEL_DEBUG)
      && ((GET_PLUGIN_053_SENSOR_MODEL_SELECTOR == PMSx003_type::PMS5003_ST)
          || (GET_PLUGIN_053_SENSOR_MODEL_SELECTOR == PMSx003_type::PMS5003_T))) { // Values only available on PMS5003ST & PMS5003T
    String log;
    if (log.reserve(45)) {
      log  = F("PMSx003 : temp=");
      log += static_cast<float>(data[PMS_Temp_C]) / 10.0f;
      log += F(", humi=");
      log += static_cast<float>(data[PMS_Hum_pct]) / 10.0f;

      if (GET_PLUGIN_053_SENSOR_MODEL_SELECTOR == PMSx003_type::PMS5003_ST) {
        log += F(", hcho=");
        log += static_cast<float>(data[PMS_Formaldehyde_mg_m3]) / 1000.0f;
      }
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
  }
  #   endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  #  endif     // ifdef P053_LOW_LEVEL_DEBUG
  # endif      // ifndef BUILD_NO_DEBUG

  // Compare checksums
  SerialRead16(checksum2, nullptr);
  SerialFlush(); // Make sure no data is lost due to full buffer.

  if (checksum != checksum2) {
    addLog(LOG_LEVEL_ERROR, F("PMSx003 : Checksum error"));
    return false;
  }

  if (_last_wakeup_moment.isSet() && !_last_wakeup_moment.timeReached()) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      if (log.reserve(80)) {
        log = F("PMSx003 : Less than ");
        log += _delay_read_after_wakeup_ms / 1000ul;
        log += F(" sec since sensor wakeup => Ignoring sample");
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }
    return false;
  }

  if (checksum == _last_checksum) {
    // Duplicate message
      # ifndef BUILD_NO_DEBUG

    addLog(LOG_LEVEL_DEBUG, F("PMSx003 : Duplicate message"));
      # endif // ifndef BUILD_NO_DEBUG
    return false;
  }
  # ifndef PLUGIN_053_ENABLE_EXTRA_SENSORS

  // Data is checked and good, fill in output
  UserVar[event->BaseVarIndex]     = data[PMS_PM1_0_ug_m3_normal];
  UserVar[event->BaseVarIndex + 1] = data[PMS_PM2_5_ug_m3_normal];
  UserVar[event->BaseVarIndex + 2] = data[PMS_PM10_0_ug_m3_normal];
  _values_received                 = 1;
  # else // ifndef PLUGIN_053_ENABLE_EXTRA_SENSORS

  // Store in the averaging buffer to process later
  if (!_oversample) {
    clearReceivedData();
  }

  for (uint8_t i = 0; i < PMS_RECEIVE_BUFFER_SIZE; ++i) {
    _data[i] += data[i];
  }
  ++_values_received;
  # endif // ifndef PLUGIN_053_ENABLE_EXTRA_SENSORS


  // Store new checksum, to help detect duplicates.
  _last_checksum = checksum;

  return true;
}

bool P053_data_struct::checkAndClearValuesReceived(struct EventStruct *event) {
  if (_values_received == 0) { return false; }

  # ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS

  // Data is checked and good, fill in output

  _value_mask = 0;

  switch (GET_PLUGIN_053_OUTPUT_SELECTOR) {
    case PMSx003_output_selection::Particles_ug_m3:
    {
      UserVar[event->BaseVarIndex]     = getValue(PMS_PM1_0_ug_m3_normal);
      UserVar[event->BaseVarIndex + 1] = getValue(PMS_PM2_5_ug_m3_normal);
      UserVar[event->BaseVarIndex + 2] = getValue(PMS_PM10_0_ug_m3_normal);
      UserVar[event->BaseVarIndex + 3] = 0.0f;
      break;
    }
    case PMSx003_output_selection::PM2_5_TempHum_Formaldehyde:
    {
      UserVar[event->BaseVarIndex]     = getValue(PMS_PM2_5_ug_m3_normal);
      UserVar[event->BaseVarIndex + 1] = getValue(PMS_Temp_C);
      UserVar[event->BaseVarIndex + 2] = getValue(PMS_Hum_pct);
      UserVar[event->BaseVarIndex + 3] = getValue(PMS_Formaldehyde_mg_m3);
      break;
    }
    case PMSx003_output_selection::ParticlesCount_100ml_cnt0_3__cnt_2_5:
    {
      UserVar[event->BaseVarIndex]     = getValue(PMS_cnt0_3_100ml);
      UserVar[event->BaseVarIndex + 1] = getValue(PMS_cnt0_5_100ml);
      UserVar[event->BaseVarIndex + 2] = getValue(PMS_cnt1_0_100ml);
      UserVar[event->BaseVarIndex + 3] = getValue(PMS_cnt2_5_100ml);
      break;
    }
    case PMSx003_output_selection::ParticlesCount_100ml_cnt1_0_cnt2_5_cnt10:
    {
      UserVar[event->BaseVarIndex]     = getValue(PMS_cnt1_0_100ml);
      UserVar[event->BaseVarIndex + 1] = getValue(PMS_cnt2_5_100ml);
      UserVar[event->BaseVarIndex + 2] = getValue(PMS_cnt5_0_100ml);
      UserVar[event->BaseVarIndex + 3] = getValue(PMS_cnt10_0_100ml);
      break;
    }
  }

  if (Settings.UseRules
      && (GET_PLUGIN_053_EVENT_OUT_SELECTOR != PMSx003_event_datatype::Event_None)
      && (GET_PLUGIN_053_SENSOR_MODEL_SELECTOR != PMSx003_type::PMS2003_3003)) {
    // Events not applicable to PMS2003 & PMS3003 models
    String baseEvent;
    baseEvent.reserve(21);
    baseEvent  = getTaskDeviceName(event->TaskIndex);
    baseEvent += '#';


    // Send out events for those values not present in the task output
    switch (GET_PLUGIN_053_EVENT_OUT_SELECTOR) {
      case PMSx003_event_datatype::Event_None: break;
      case PMSx003_event_datatype::Event_All:
      {
        // Send all remaining
        for (uint8_t i = PMS_PM1_0_ug_m3_normal; i < PMS_RECEIVE_BUFFER_SIZE; ++i) {
          sendEvent(baseEvent, i);
        }
        break;
      }
      case PMSx003_event_datatype::Event_PMxx_TempHum_Formaldehyde:
      {
        const uint8_t indices[] =
        {
          PMS_PM1_0_ug_m3_normal,
          PMS_PM2_5_ug_m3_normal,
          PMS_PM10_0_ug_m3_normal,
          PMS_Temp_C,
          PMS_Hum_pct,
          PMS_Formaldehyde_mg_m3
        };

        for (uint8_t i = 0; i < 6; ++i) {
          sendEvent(baseEvent, indices[i]);
        }
        break;
      }
      case PMSx003_event_datatype::Event_All_count_bins:
      {
        // Thexe values are sequential, so just use a simple for loop.
        for (uint8_t i = PMS_cnt0_3_100ml; i <= PMS_cnt10_0_100ml; ++i) {
          sendEvent(baseEvent, i);
        }
        break;
      }
    }
  }

  if (_oversample) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("PMSx003: Oversampling using ");
      log += _values_received;
      log += F(" samples");
      addLogMove(LOG_LEVEL_INFO, log);
    }
  }
  # endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS

  clearReceivedData();
  return true;
}

bool P053_data_struct::resetSensor() {
  if (_resetPin >= 0) { // Reset if pin is configured
    // Toggle 'reset' to assure we start reading header
    addLog(LOG_LEVEL_INFO, F("PMSx003: resetting module"));
    pinMode(_resetPin, OUTPUT);
    digitalWrite(_resetPin, LOW);
    delay(250);
    digitalWrite(_resetPin, HIGH);
    pinMode(_resetPin, INPUT_PULLUP);
    return true;
  }
  return false;
}

bool P053_data_struct::wakeSensor() {
  if (!initialized()) {
    return false;
  }
  addLog(LOG_LEVEL_INFO, F("PMSx003: Wake sensor"));

  if (_pwrPin >= 0) {
    // Make sure the sensor is "on"
    pinMode(_pwrPin, OUTPUT);
    digitalWrite(_pwrPin, HIGH);
    pinMode(_pwrPin, INPUT_PULLUP);
  } else {
    const uint8_t command[7] = { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74
    };
    _easySerial->write(command, 7);
  }

  // No idea how old this data is, so clear it.
  // Otherwise oversampling may give weird results
  clearReceivedData();

  if (_delay_read_after_wakeup_ms != 0) {
    _last_wakeup_moment.setMillisFromNow(_delay_read_after_wakeup_ms);
  } else {
    _last_wakeup_moment.clear();
  }
  return true;
}

bool P053_data_struct::sleepSensor() {
  if (!initialized()) {
    return false;
  }

  // Put the sensor to sleep
  addLog(LOG_LEVEL_INFO, F("PMSx003: Sleep sensor"));

  if (_pwrPin >= 0) {
    pinMode(_pwrPin, OUTPUT);
    digitalWrite(_pwrPin, LOW);
  } else {
    const uint8_t command[7] = { 0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };
    _easySerial->write(command, 7);
  }

  if (_values_received > 0) {
    // Going to sleep, so flush whatever is read.
    Scheduler.schedule_task_device_timer(_taskIndex, millis() + 10);
  }
  return true;
}

void P053_data_struct::setActiveReadingMode() {
  if (initialized()) {
    const uint8_t command[7] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 };
    _easySerial->write(command, 7);
    _activeReadingModeEnabled = true;
  }
}

void P053_data_struct::setPassiveReadingMode() {
  if (initialized()) {
    const uint8_t command[7] = { 0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70 };
    _easySerial->write(command, 7);
    _activeReadingModeEnabled = false;
  }
}

void P053_data_struct::requestData() {
  if (initialized() && !_activeReadingModeEnabled) {
    const uint8_t command[7] = { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 };
    _easySerial->write(command, 7);
  }
}

# ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
float P053_data_struct::getValue(uint8_t index) {
  float value = 0.0f;

  getValue(index, value);
  return value;
}

bool P053_data_struct::getValue(uint8_t index, float& value) {
  if (bitRead(_value_mask, index) || (index >= PMS_RECEIVE_BUFFER_SIZE)) {
    // Already read
    return false;
  }

  switch (index) {
    case PMS_PM1_0_ug_m3_factory:
    case PMS_PM2_5_ug_m3_factory:
    case PMS_PM10_0_ug_m3_factory:
    case PMS_FW_rev_error:
    case PMS_Reserved:
      return false;

    case PMS_Temp_C:
    case PMS_Hum_pct:

      if (!hasTempHum()) { return false; }
      value = _data[index] / 10.0f;
      break;
    case PMS_Formaldehyde_mg_m3:

      if (!hasFormaldehyde()) { return false; }
      value = _data[index] / 1000.0f;
      break;
    case PMS_cnt5_0_100ml:
    case PMS_cnt10_0_100ml: // this option was missing :-|

      if (_sensortype == PMSx003_type::PMS5003_T) { return false; } // else: fall through
    case PMS_cnt0_3_100ml:
    case PMS_cnt0_5_100ml:
    case PMS_cnt1_0_100ml:
    case PMS_cnt2_5_100ml:
      value = _data[index];

      if (_splitCntBins) {
        value -= _data[index + 1];

        if (value < 0.0f) { value = 0.0f; }
      }
      break;

    default:
      value = _data[index];
      break;
  }

  if (_values_received > 1) {
    value /= static_cast<float>(_values_received);
  }

  bitSet(_value_mask, index);
  return true;
}

# endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS

void P053_data_struct::clearReceivedData() {
  # ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS

  for (uint8_t i = 0; i < PMS_RECEIVE_BUFFER_SIZE; ++i) {
    _data[i] = 0.0f;
  }
  # endif // ifdef PLUGIN_053_ENABLE_EXTRA_SENSORS
  _values_received = 0;
}

const __FlashStringHelper * P053_data_struct::getEventString(uint8_t index) {
  switch (index) {
    // case PMS_PM1_0_ug_m3_factory   : return F("");
    // case PMS_PM2_5_ug_m3_factory   : return F("");
    // case PMS_PM10_0_ug_m3_factory  : return F("");
    case PMS_PM1_0_ug_m3_normal: return F("pm1.0");
    case PMS_PM2_5_ug_m3_normal: return F("pm2.5");
    case PMS_PM10_0_ug_m3_normal: return F("pm10");
    case PMS_cnt0_3_100ml: return F("cnt0.3");
    case PMS_cnt0_5_100ml: return F("cnt0.5");
    case PMS_cnt1_0_100ml: return F("cnt1.0");
    case PMS_cnt2_5_100ml: return F("cnt2.5");
    case PMS_cnt5_0_100ml: return F("cnt5.0");
    case PMS_cnt10_0_100ml: return F("cnt10");
    case PMS_Formaldehyde_mg_m3: return F("HCHO");
    case PMS_Temp_C: return F("Temp");
    case PMS_Hum_pct: return F("Humi");

      // case PMS_FW_rev_error          : return F("");
  }
  return F("Unknown");
}

void P053_data_struct::setTaskValueNames(ExtraTaskSettingsStruct& settings, const uint8_t indices[], uint8_t nrElements,
                                         bool oversample) {
  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    settings.TaskDeviceValueDecimals[i] = 0;

    if (i < nrElements) {
      safe_strncpy(
        settings.TaskDeviceValueNames[i],
        P053_data_struct::getEventString(indices[i]),
        sizeof(settings.TaskDeviceValueNames[i]));

      settings.TaskDeviceValueDecimals[i] = getNrDecimals(indices[i], oversample);
    } else {
      ZERO_FILL(settings.TaskDeviceValueNames[i]);
    }
  }
}

unsigned char P053_data_struct::getNrDecimals(uint8_t index, bool oversample) {
  switch (index) {
    case PMS_Temp_C:
    case PMS_Hum_pct:
      return 1;
    case PMS_Formaldehyde_mg_m3:
      return 3;
  }

  if (oversample) { return 1; }
  return 0;
}

#endif // ifdef USES_P053

#include "../PluginStructs/P014_data_struct.h"

#ifdef USES_P014
P014_data_struct::P014_data_struct(uint8_t resolution) : res(resolution) {
  reset();
}

void P014_data_struct::reset()
{
  state         = SI7021_state::Uninitialized; // Force device setup next time
  timeStartRead = 0;
}

bool P014_data_struct::init()
{
  state = SI7021_state::Uninitialized;

  // Set the resolution we want
  const uint8_t ret = setResolution(res);

  if (ret == 0) {
    state = SI7021_state::Initialized;
    return true;
  }

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("SI7021 : Res=0x");
    log += String(res, HEX);
    log += F(" => Error 0x");
    log += String(ret, HEX);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  return false;
}

bool P014_data_struct::loop() {
  switch (state)
  {
    case SI7021_state::Initialized:
    {
      // Start conversion for humidity
      startConv(SI7021_MEASURE_HUM);
      timeStartRead = millis();

      // change state of sensor
      state = SI7021_state::Wait_for_temperature_samples;
      break;
    }

    case SI7021_state::Wait_for_temperature_samples:
    {
      // Check if conversion is finished
      if (readValues(SI7021_MEASURE_HUM, res) == 0) {
        // Start conversion for temperature
        startConv(SI7021_MEASURE_TEMP);

        // change state of sensor
        state = SI7021_state::Wait_for_humidity_samples;
      }
      break;
    }

    case SI7021_state::Wait_for_humidity_samples:
    {
      // Check if conversion is finished
      if (readValues(SI7021_MEASURE_TEMP, res) == 0) {
        // change state of sensor
        state         = SI7021_state::New_values;
        timeStartRead = 0;
      }
      break;
    }

    default:
      break;
  }

  if (timeStartRead != 0) {
    // Apparently we're waiting for some reading.
    if (timePassedSince(timeStartRead) > SI7021_TIMEOUT) {
      reset();
    }
  }
  return SI7021_state::New_values == state;
}

bool P014_data_struct::getReadValue(float& temperature, float& humidity) {
  bool success = false;

  switch (state) {
    case SI7021_state::Uninitialized:
    {
      addLog(LOG_LEVEL_INFO, F("SI7021 : sensor not initialized !"));
      init();
      break;
    }
    case SI7021_state::Initialized:
    {
      // No call made to start a new reading
      // Should be handled in the loop()
      addLog(LOG_LEVEL_INFO, F("SI7021 : No read started !"));
      break;
    }
    case SI7021_state::Wait_for_temperature_samples:
    case SI7021_state::Wait_for_humidity_samples:
    {
      // Still waiting for data
      // Should be handled in the loop()
      addLog(LOG_LEVEL_ERROR, F("SI7021 : Read Error !"));
      break;
    }

    case SI7021_state::New_values:
    {
      temperature = si7021_temperature / 100.0f;
      humidity    = si7021_humidity / 10.0f;
      state       = SI7021_state::Values_read;
      success     = true;

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("SI7021 : Temperature: ");
        log += temperature;
        addLogMove(LOG_LEVEL_INFO, log);
        log  = F("SI7021 : Humidity: ");
        log += humidity;
        addLogMove(LOG_LEVEL_INFO, log);
      }
      break;
    }
    case SI7021_state::Values_read:
    {
      // This must be done in a separate call to
      // make sure we only start reading when the plugin wants us to perform a reading.
      // Change state of sensor for non blocking reading
      state = SI7021_state::Initialized;
      break;
    }
  }
  return success;
}

/* ======================================================================
   Function: checkCRC
   Purpose : check the CRC of received data
   Input   : value read from sensor
   Output  : CRC read from sensor
   Comments: 0 if okay
   ====================================================================== */
uint8_t P014_data_struct::checkCRC(uint16_t data, uint8_t check)
{
  uint32_t remainder, divisor;

  // Pad with 8 bits because we have to add in the check value
  remainder = (uint32_t)data << 8;

  // From: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
  // POLYNOMIAL = 0x0131 = x^8 + x^5 + x^4 + 1 : http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
  // 0x988000 is the 0x0131 polynomial shifted to farthest left of three bytes
  divisor = (uint32_t)0x988000;

  // Add the check value
  remainder |= check;

  // Operate on only 16 positions of max 24.
  // The remaining 8 are our remainder and should be zero when we're done.
  for (uint8_t i = 0; i < 16; i++) {
    // Check if there is a one in the left position
    if (remainder & (uint32_t)1 << (23 - i)) {
      remainder ^= divisor;
    }

    // Rotate the divisor max 16 times so that we have 8 bits left of a remainder
    divisor >>= 1;
  }
  return (uint8_t)remainder;
}

/* ======================================================================
   Function: si7021_readRegister
   Purpose : read the user register from the sensor
   Input   : user register value filled by function
   Output  : 0 if okay
   Comments: -
   ====================================================================== */
int8_t P014_data_struct::readRegister(uint8_t *value)
{
  // Request user register
  Wire.beginTransmission(SI7021_I2C_ADDRESS);
  Wire.write(SI7021_READ_REG);
  Wire.endTransmission();

  // request 1 uint8_t result
  Wire.requestFrom(SI7021_I2C_ADDRESS, 1);

  if (Wire.available() >= 1) {
    *value = Wire.read();
    return 0;
  }

  return 1;
}

/* ======================================================================
   Function: startConv
   Purpose : return temperature or humidity measured
   Input   : data type SI7021_READ_HUM or SI7021_READ_TEMP
   Output  : 0 if okay
   Comments: -
   ====================================================================== */
uint8_t P014_data_struct::startConv(uint8_t datatype)
{
  // Request a reading
  Wire.beginTransmission(SI7021_I2C_ADDRESS);
  Wire.write(datatype);
  Wire.endTransmission();

  return 0;
}

/* ======================================================================
   Function: readValues
   Purpose : read temperature and humidity from SI7021 sensor
   Input   : current config resolution
   Output  : 0 if okay
   Comments: -
   ====================================================================== */
int8_t P014_data_struct::readValues(uint8_t datatype, uint8_t resolution)
{
  long data;
  uint16_t raw;
  uint8_t  checksum;

  if (Wire.requestFrom(SI7021_I2C_ADDRESS, 3) != 3) {
    return -1;
  }

  // Comes back in three bytes, data(MSB) / data(LSB) / Checksum
  raw      = ((uint16_t)Wire.read()) << 8;
  raw     |= Wire.read();
  checksum = Wire.read();

  // Check CRC of data received
  if (checkCRC(raw, checksum) != 0) {
    addLog(LOG_LEVEL_ERROR, F("SI7021 : checksum error!"));
    return -1;
  }

  // Humidity
  if ((datatype == SI7021_MEASURE_HUM) || (datatype == SI7021_MEASURE_HUM_HM)) {
    // Convert value to Himidity percent
    // pm-cz: it is possible to enable decimal places for humidity as well by multiplying the value in formula by 100
    data = ((1250 * (long)raw) >> 16) - 60;

    // Datasheet says doing this check
    if (data > 1000) { data = 1000; }

    if (data < 0) { data = 0; }

    // pm-cz: Let us make sure we have enough precision due to ADC bits
    if (resolution == SI7021_RESOLUTION_12T_08RH) {
      data  = (data + 5) / 10;
      data *= 10;
    }

    // save value
    si7021_humidity = (uint16_t)data;

    // Temperature
  } else if ((datatype == SI7021_MEASURE_TEMP) || (datatype == SI7021_MEASURE_TEMP_HM) || (datatype == SI7021_MEASURE_TEMP_HUM)) {
    // Convert value to Temperature (*100)
    // for 23.45C value will be 2345
    data =  ((17572 * (long)raw) >> 16) - 4685;

    /*
       // pm-cz: We should probably check for precision here as well
       if (resolution != SI7021_RESOLUTION_14T_12RH) {
       if (data > 0) {
        data = (data + 5) / 10;
       } else {
        data = (data - 5) / 10;
       }
       data *= 10;
       }
     */

    // save value
    si7021_temperature = (int16_t)data;
  }

  return 0;
}

/* ======================================================================
   Function: setResolution
   Purpose : Sets the sensor resolution to one of four levels
   Input   : see #define default is SI7021_RESOLUTION_14T_12RH
   Output  : 0 if okay
   Comments: -
   ====================================================================== */
int8_t P014_data_struct::setResolution(uint8_t res)
{
  // Get the current register value
  uint8_t reg   = 0;
  uint8_t error = readRegister(&reg);

  if (error == 0) {
    // remove resolution bits
    reg &= SI7021_RESOLUTION_MASK;

    // Prepare to write to the register value
    Wire.beginTransmission(SI7021_I2C_ADDRESS);
    Wire.write(SI7021_WRITE_REG);

    // Write the new resolution bits but clear unused before
    Wire.write(reg | (res &= ~SI7021_RESOLUTION_MASK));
    return (int8_t)Wire.endTransmission();
  }

  return error;
}

#endif // ifdef USES_P014

#include "../PluginStructs/P104_data_struct.h"

#ifdef USES_P104

# include "../Helpers/ESPEasy_Storage.h"
# include "../Helpers/Numerical.h"
# include "../WebServer/Markup_Forms.h"
# include "../WebServer/WebServer.h"
# include "../WebServer/Markup.h"
# include "../WebServer/HTML_wrappers.h"
# include "../ESPEasyCore/ESPEasyRules.h"
# include "../Globals/ESPEasy_time.h"
# include "../Globals/RTC.h"

# include <vector>
# include <MD_Parola.h>
# include <MD_MAX72xx.h>

// Needed also here for PlatformIO's library finder as the .h file
// is in a directory which is excluded in the src_filter

# if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
void createHString(String& string); // Forward definition
# endif // if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
void reverseStr(String& str);       // Forward definition

/****************************************************************
 * Constructor
 ***************************************************************/
P104_data_struct::P104_data_struct(MD_MAX72XX::moduleType_t _mod,
                                   taskIndex_t              _taskIndex,
                                   int8_t                   _cs_pin,
                                   uint8_t                  _modules,
                                   uint8_t                  _zonesCount)
  : mod(_mod), taskIndex(_taskIndex), cs_pin(_cs_pin), modules(_modules), expectedZones(_zonesCount) {
  if (Settings.isSPI_valid()) {
    P = new (std::nothrow) MD_Parola(mod, cs_pin, modules);
  } else {
    addLog(LOG_LEVEL_ERROR, F("DOTMATRIX: Required SPI not enabled. Initialization aborted!"));
  }
}

/*******************************
 * Destructor
 ******************************/
P104_data_struct::~P104_data_struct() {
  # ifdef P104_USE_BAR_GRAPH

  if (nullptr != pM) {
    pM = nullptr; // Not created here, only reset
  }
  # endif // ifdef P104_USE_BAR_GRAPH

  if (nullptr != P) {
    // P->~MD_Parola(); // Call destructor directly, as delete of the object fails miserably
    // do not: delete P; // Warning: the MD_Parola object doesn't have a virtual destructor, and when changed,
    // a reboot uccurs when the object is deleted here!
    P = nullptr; // Reset only
  }
}

/*******************************
 * Initializer/starter
 ******************************/
bool P104_data_struct::begin() {
  if (!initialized) {
    loadSettings();
    initialized = true;
  }

  if ((P != nullptr) && (cs_pin > -1)) {
    # ifdef P104_DEBUG
    addLog(LOG_LEVEL_INFO, F("dotmatrix: begin() called"));
    # endif // ifdef P104_DEBUG
    P->begin(expectedZones);
    # ifdef P104_USE_BAR_GRAPH
    pM = P->getGraphicObject();
    # endif // ifdef P104_USE_BAR_GRAPH
    return true;
  }
  return false;
}

# define P104_ZONE_SEP   '\x02'
# define P104_FIELD_SEP  '\x01'
# define P104_ZONE_DISP  ';'
# define P104_FIELD_DISP ','

# define P104_CONFIG_VERSION_V2  0xF000 // Marker in first uint16_t to to indicate second version config settings, anything else if first
                                        // version.
                                        // Any third version or later could use 0xE000, etc. The 'version' is stored in the first uint16_t
                                        // stored in the custom settings

/*
   Settings layout:
   Version 1:
   - uint16_t : size of the next blob holding all settings
   - char[x]  : Blob with settings, with csv-like strings, using P104_FIELD_SEP and P104_ZONE_SEP separators
   Version 2:
   - uint16_t : marker with content P104_CONFIG_VERSION_V2
   - uint16_t : size of next blob holding 1 zone settings string
   - char[y]  : Blob holding 1 zone settings string, with csv like string, using P104_FIELD_SEP separators
   - uint16_t : next size, if 0 then no more blobs
   - char[x]  : Blob
   - ...
   - Max. allowed total custom settings size = 1024
 */
/**************************************
 * loadSettings
 *************************************/
void P104_data_struct::loadSettings() {
  uint16_t bufferSize;
  char    *settingsBuffer;

  if (taskIndex < TASKS_MAX) {
    int loadOffset = 0;

    // Read size of the used buffer, could be the settings-version marker
    LoadFromFile(SettingsType::Enum::CustomTaskSettings_Type, taskIndex, (uint8_t *)&bufferSize, sizeof(bufferSize), loadOffset);
    bool settingsVersionV2  = (bufferSize == P104_CONFIG_VERSION_V2) || (bufferSize == 0u);
    uint16_t structDataSize = 0;
    uint16_t reservedBuffer = 0;

    if (!settingsVersionV2) {
      reservedBuffer = bufferSize + 1;              // just add 1 for storing a string-terminator
      addLog(LOG_LEVEL_INFO, F("dotmatrix: Reading Settings V1, will be stored as Settings V2."));
    } else {
      reservedBuffer = P104_SETTINGS_BUFFER_V2 + 1; // just add 1 for storing a string-terminator
    }
    reservedBuffer++;                               // Add 1 for 0..size use
    settingsBuffer = new char[reservedBuffer]();    // Allocate buffer and reset to all zeroes
    loadOffset    += sizeof(bufferSize);

    if (settingsVersionV2) {
      LoadFromFile(SettingsType::Enum::CustomTaskSettings_Type, taskIndex, (uint8_t *)&bufferSize, sizeof(bufferSize), loadOffset);
      loadOffset += sizeof(bufferSize); // Skip the size
    }
    structDataSize = bufferSize;
    # ifdef P104_DEBUG_DEV
    {
      String log;

      if (loglevelActiveFor(LOG_LEVEL_INFO) &&
          log.reserve(54)) {
        log  = F("P104: loadSettings stored Size: ");
        log += structDataSize;
        log += F(" taskindex: ");
        log += taskIndex;
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }
    # endif // ifdef P104_DEBUG_DEV

    // Read actual data
    if (structDataSize > 0) { // Reading 0 bytes logs an error, so lets avoid that
      LoadFromFile(SettingsType::Enum::CustomTaskSettings_Type, taskIndex, (uint8_t *)settingsBuffer, structDataSize, loadOffset);
    }
    settingsBuffer[bufferSize + 1] = '\0'; // Terminate string

    uint8_t zoneIndex = 0;

    {
      String buffer;
      buffer = String(settingsBuffer);
      # ifdef P104_DEBUG_DEV

      String log;
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        log  = F("P104: loadSettings bufferSize: ");
        log += bufferSize;
        log += F(" untrimmed: ");
        log += buffer.length();
      }
      # endif // ifdef P104_DEBUG_DEV
      buffer.trim();
      # ifdef P104_DEBUG_DEV

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        log += F(" trimmed: ");
        log += buffer.length();
        addLogMove(LOG_LEVEL_INFO, log);
      }
      # endif // ifdef P104_DEBUG_DEV

      if (zones.size() > 0) {
        zones.clear();
      }
      zones.reserve(P104_MAX_ZONES);
      numDevices = 0;

      String   tmp;
      String   fld;
      int      tmp_int;
      uint16_t prev2   = 0;
      int16_t  offset2 = buffer.indexOf(P104_ZONE_SEP);

      if ((offset2 == -1) && (buffer.length() > 0)) {
        offset2 = buffer.length();
      }

      while (offset2 > -1) {
        tmp = buffer.substring(prev2, offset2);
        # ifdef P104_DEBUG_DEV

        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          log  = F("P104: reading string: ");
          log += tmp;
          log.replace(P104_FIELD_SEP, P104_FIELD_DISP);
          addLogMove(LOG_LEVEL_INFO, log);
        }
        # endif // ifdef P104_DEBUG_DEV

        zones.push_back(P104_zone_struct(zoneIndex + 1));

        tmp_int = 0;

        // WARNING: Order of parsing these values should match the numeric order of P104_OFFSET_* values

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_SIZE, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].size = tmp_int;
        }

        zones[zoneIndex].text = parseStringKeepCase(tmp, 1 + P104_OFFSET_TEXT, P104_FIELD_SEP);

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_ALIGNMENT, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].alignment = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_ANIM_IN, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].animationIn = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_SPEED, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].speed = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_ANIM_OUT, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].animationOut = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_PAUSE, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].pause = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_FONT, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].font = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_CONTENT, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].content = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_LAYOUT, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].layout = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_SPEC_EFFECT, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].specialEffect = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_OFFSET, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].offset = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_BRIGHTNESS, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].brightness = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_REPEATDELAY, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].repeatDelay = tmp_int;
        }

        if (validIntFromString(parseString(tmp, 1 + P104_OFFSET_INVERTED, P104_FIELD_SEP), tmp_int)) {
          zones[zoneIndex].inverted = tmp_int;
        }

        delay(0);

        numDevices += zones[zoneIndex].size + zones[zoneIndex].offset;

        if (!settingsVersionV2) {
          prev2   = offset2 + 1;
          offset2 = buffer.indexOf(P104_ZONE_SEP, prev2);
        } else {
          loadOffset    += bufferSize;
          structDataSize = sizeof(bufferSize);
          LoadFromFile(SettingsType::Enum::CustomTaskSettings_Type, taskIndex, (uint8_t *)&bufferSize, structDataSize, loadOffset);
          offset2 = bufferSize;  // Length

          if (bufferSize == 0) { // End of zones reached
            offset2 = -1;        // fall out of while loop
          } else {
            structDataSize = bufferSize;
            loadOffset    += sizeof(bufferSize);
            LoadFromFile(SettingsType::Enum::CustomTaskSettings_Type, taskIndex, (uint8_t *)settingsBuffer, structDataSize, loadOffset);
            settingsBuffer[bufferSize + 1] = '\0'; // Terminate string
            buffer = String(settingsBuffer);
          }
        }
        zoneIndex++;

        # ifdef P104_DEBUG

        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log;
          log  = F("dotmatrix: parsed zone: ");
          log += zoneIndex;
          addLogMove(LOG_LEVEL_INFO, log);
        }
        # endif // ifdef P104_DEBUG
      }

      buffer = String();        // Free some memory
    }

    delete[] settingsBuffer; // Release allocated buffer
    # ifdef P104_DEBUG_DEV

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log  = F("P104: read zones from config: ");
      log += zoneIndex;

      // log += F(" struct size: ");
      // log += sizeof(P104_zone_struct);
      addLogMove(LOG_LEVEL_INFO, log);
    }
    # endif // ifdef P104_DEBUG_DEV

    if (expectedZones == -1) { expectedZones = zoneIndex; }

    if (expectedZones == 0) { expectedZones++; } // Guarantee at least 1 zone to be displayed

    while (zoneIndex < expectedZones) {
      zones.push_back(P104_zone_struct(zoneIndex + 1));

      if (zones[zoneIndex].text == F("\"\"")) { // Special case
        zones[zoneIndex].text.clear();
      }

      zoneIndex++;
      delay(0);
    }
    # ifdef P104_DEBUG_DEV

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log  = F("P104: total zones initialized: ");
      log += zoneIndex;
      log += F(" expected: ");
      log += expectedZones;
      addLogMove(LOG_LEVEL_INFO, log);
    }
    # endif // ifdef P104_DEBUG_DEV
  }
}

/****************************************************
 * configureZones: initialize Zones setup
 ***************************************************/
void P104_data_struct::configureZones() {
  if (!initialized) {
    loadSettings();
    initialized = true;
  }

  uint8_t currentZone = 0;
  uint8_t zoneOffset  = 0;

  # ifdef P104_DEBUG_DEV
  String log;

  if (loglevelActiveFor(LOG_LEVEL_INFO) &&
      log.reserve(45)) {
    log  = F("P104: configureZones to do: ");
    log += zones.size();
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // ifdef P104_DEBUG_DEV

  if (nullptr == P) { return; }

  P->displayClear();

  for (auto it = zones.begin(); it != zones.end(); ++it) {
    if (it->zone <= expectedZones) {
      zoneOffset += it->offset;
      P->setZone(currentZone, zoneOffset, zoneOffset + it->size - 1);
      # ifdef P104_USE_BAR_GRAPH
      it->_startModule = zoneOffset;
      P->getDisplayExtent(currentZone, it->_lower, it->_upper);
      # endif // ifdef P104_USE_BAR_GRAPH
      zoneOffset += it->size;

      switch (it->font) {
        # ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
        case P104_DOUBLE_HEIGHT_FONT_ID: {
          P->setFont(currentZone, numeric7SegDouble);
          P->setCharSpacing(currentZone, P->getCharSpacing() * 2); // double spacing as well
          break;
        }
        # endif // ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
        # ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
        case P104_FULL_DOUBLEHEIGHT_FONT_ID: {
          P->setFont(currentZone, BigFont);
          P->setCharSpacing(currentZone, P->getCharSpacing() * 2); // double spacing as well
          break;
        }
        # endif // ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
        # ifdef P104_USE_VERTICAL_FONT
        case P104_VERTICAL_FONT_ID: {
          P->setFont(currentZone, _fontVertical);
          break;
        }
        # endif // ifdef P104_USE_VERTICAL_FONT
        # ifdef P104_USE_EXT_ASCII_FONT
        case P104_EXT_ASCII_FONT_ID: {
          P->setFont(currentZone, ExtASCII);
          break;
        }
        # endif // ifdef P104_USE_EXT_ASCII_FONT
        # ifdef P104_USE_ARABIC_FONT
        case P104_ARABIC_FONT_ID: {
          P->setFont(currentZone, fontArabic);
          break;
        }
        # endif // ifdef P104_USE_ARABIC_FONT
        # ifdef P104_USE_GREEK_FONT
        case P104_GREEK_FONT_ID: {
          P->setFont(currentZone, fontGreek);
          break;
        }
        # endif // ifdef P104_USE_GREEK_FONT
        # ifdef P104_USE_KATAKANA_FONT
        case P104_KATAKANA_FONT_ID: {
          P->setFont(currentZone, fontKatakana);
          break;
        }
        # endif // ifdef P104_USE_KATAKANA_FONT

        // Extend above this comment with more fonts if/when available,
        // case P104_DEFAULT_FONT_ID: and default: clauses should be the last options.
        // This should also make sure the default font is set if a no longer available font was selected
        case P104_DEFAULT_FONT_ID:
        default: {
          P->setFont(currentZone, nullptr); // default font
          break;
        }
      }

      // Inverted
      P->setInvert(currentZone, it->inverted);

      // Special Effects
      P->setZoneEffect(currentZone, (it->specialEffect & P104_SPECIAL_EFFECT_UP_DOWN) == P104_SPECIAL_EFFECT_UP_DOWN,       PA_FLIP_UD);
      P->setZoneEffect(currentZone, (it->specialEffect & P104_SPECIAL_EFFECT_LEFT_RIGHT) == P104_SPECIAL_EFFECT_LEFT_RIGHT, PA_FLIP_LR);

      // Brightness
      P->setIntensity(currentZone, it->brightness);

      # ifdef P104_DEBUG_DEV

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        log  = F("P104: configureZones #");
        log += (currentZone + 1);
        log += '/';
        log += expectedZones;
        log += F(" offset: ");
        log += zoneOffset;
        addLogMove(LOG_LEVEL_INFO, log);
      }
      # endif // ifdef P104_DEBUG_DEV

      delay(0);

      // Content == text && text != ""
      if (((it->content == P104_CONTENT_TEXT) ||
           (it->content == P104_CONTENT_TEXT_REV))
          && (!it->text.isEmpty())) {
        displayOneZoneText(currentZone, *it, it->text);
      }

      # ifdef P104_USE_BAR_GRAPH

      // Content == Bar-graph && text != ""
      if ((it->content == P104_CONTENT_BAR_GRAPH)
          && (!it->text.isEmpty())) {
        displayBarGraph(currentZone, *it, it->text);
      }
      # endif // ifdef P104_USE_BAR_GRAPH

      if (it->repeatDelay > -1) {
        it->_repeatTimer = millis();
      }
      currentZone++;
      delay(0);
    }
  }

  // Synchronize the start
  P->synchZoneStart();
}

/**********************************************************
 * Display the text with attributes for a specific zone
 *********************************************************/
void P104_data_struct::displayOneZoneText(uint8_t                 zone,
                                          const P104_zone_struct& zstruct,
                                          const String          & text) {
  if ((nullptr == P) || (zone >= P104_MAX_ZONES)) { return; } // double check
  sZoneInitial[zone].reserve(text.length());
  sZoneInitial[zone] = text; // Keep the original string for future use
  sZoneBuffers[zone].reserve(text.length());
  sZoneBuffers[zone] = text; // We explicitly want a copy here so it can be modified by parseTemplate()

  sZoneBuffers[zone] = parseTemplate(sZoneBuffers[zone]);

  # if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)

  if (zstruct.layout == P104_LAYOUT_DOUBLE_UPPER) {
    createHString(sZoneBuffers[zone]);
  }
  # endif // if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)

  if (zstruct.content == P104_CONTENT_TEXT_REV) {
    reverseStr(sZoneBuffers[zone]);
  }

  String log;

  if (loglevelActiveFor(LOG_LEVEL_INFO) &&
      logAllText &&
      log.reserve(28 + text.length() + sZoneBuffers[zone].length())) {
    log  = F("dotmatrix: ZoneText: ");
    log += zone + 1; // UI-number
    log += F(", '");
    log += text;
    log += F("' -> '");
    log += sZoneBuffers[zone];
    log += '\'';
    addLogMove(LOG_LEVEL_INFO, log);
  }

  P->displayZoneText(zone,
                     sZoneBuffers[zone].c_str(),
                     static_cast<textPosition_t>(zstruct.alignment),
                     zstruct.speed,
                     zstruct.pause,
                     static_cast<textEffect_t>(zstruct.animationIn),
                     static_cast<textEffect_t>(zstruct.animationOut));
}

/*********************************************
 * Update all or the specified zone
 ********************************************/
void P104_data_struct::updateZone(uint8_t                 zone,
                                  const P104_zone_struct& zstruct) {
  if (nullptr == P) { return; }

  if (zone == 0) {
    for (auto it = zones.begin(); it != zones.end(); ++it) {
      if ((it->zone > 0) &&
          ((it->content == P104_CONTENT_TEXT) ||
           (it->content == P104_CONTENT_TEXT_REV))) {
        displayOneZoneText(it->zone - 1, *it, sZoneInitial[it->zone - 1]); // Re-send last displayed text
        P->displayReset(it->zone - 1);
      }
      # ifdef P104_USE_BAR_GRAPH

      if ((it->zone > 0) &&
          (it->content == P104_CONTENT_BAR_GRAPH)) {
        displayBarGraph(it->zone - 1, *it, sZoneInitial[it->zone - 1]); // Re-send last displayed bar graph
      }
      # endif // ifdef P104_USE_BAR_GRAPH

      if ((zstruct.content == P104_CONTENT_TEXT)
          || zstruct.content == P104_CONTENT_TEXT_REV
          # ifdef P104_USE_BAR_GRAPH
          || zstruct.content == P104_CONTENT_BAR_GRAPH
          # endif // ifdef P104_USE_BAR_GRAPH
          ) {
        if (it->repeatDelay > -1) { // Restart repeat timer
          it->_repeatTimer = millis();
        }
      }
    }
  } else {
    if ((zstruct.zone > 0) &&
        ((zstruct.content == P104_CONTENT_TEXT) ||
         (zstruct.content == P104_CONTENT_TEXT_REV))) {
      displayOneZoneText(zstruct.zone - 1, zstruct, sZoneInitial[zstruct.zone - 1]); // Re-send last displayed text
      P->displayReset(zstruct.zone - 1);
    }
    # ifdef P104_USE_BAR_GRAPH

    if ((zstruct.zone > 0) &&
        (zstruct.content == P104_CONTENT_BAR_GRAPH)) {
      displayBarGraph(zstruct.zone - 1, zstruct, sZoneInitial[zstruct.zone - 1]); // Re-send last displayed bar graph
    }
    # endif // ifdef P104_USE_BAR_GRAPH

    // Repeat timer is/should be started elsewhere
  }
}

# ifdef P104_USE_BAR_GRAPH

/***********************************************
 * Enable/Disable updating a range of modules
 **********************************************/
void P104_data_struct::modulesOnOff(uint8_t start, uint8_t end, MD_MAX72XX::controlValue_t on_off) {
  for (uint8_t m = start; m <= end; m++) {
    pM->control(m, MD_MAX72XX::UPDATE, on_off);
  }
}

/********************************************************
 * draw a single bar-graph, arguments already adjusted for direction
 *******************************************************/
void P104_data_struct::drawOneBarGraph(uint16_t lower,
                                       uint16_t upper,
                                       int16_t  pixBottom,
                                       int16_t  pixTop,
                                       uint16_t zeroPoint,
                                       uint8_t  barWidth,
                                       uint8_t  barType,
                                       uint8_t  row) {
  bool on_off;

  for (uint8_t r = 0; r < barWidth; r++) {
    for (uint8_t col = lower; col <= upper; col++) {
      on_off = (col >= pixBottom && col <= pixTop); // valid area

      if ((zeroPoint != 0) &&
          (barType == P104_BARTYPE_STANDARD) &&
          (barWidth > 2) &&
          ((r == 0) || (r == barWidth - 1)) &&
          (col == lower + zeroPoint)) {
        on_off = false; // when bar wider than 2, turn off zeropoint top and bottom led
      }

      if ((barType == P104_BARTYPE_SINGLE) && (r > 0)) {
        on_off = false; // barType 1 = only a single line is drawn, independent of the width
      }

      if ((barType == P104_BARTYPE_ALT_DOT) && (barWidth > 1) && on_off) {
        on_off = ((r % 2) == (col % 2)); // barType 2 = dotted line when bar is wider than 1 pixel
      }
      pM->setPoint(row + r, col, on_off);

      if (col % 16 == 0) { delay(0); }
    }
    delay(0); // Leave some breathingroom
  }
}

/********************************************************************
 * Process a graph-string to display in a zone, format:
 * value,max-value,min-value,direction,bartype|...
 *******************************************************************/
void P104_data_struct::displayBarGraph(uint8_t                 zone,
                                       const P104_zone_struct& zstruct,
                                       const String          & graph) {
  if ((nullptr == P) || (nullptr == pM) || graph.isEmpty()) { return; }
  sZoneInitial[zone] = graph; // Keep the original string for future use

  #  define NOT_A_COMMA 0x02  // Something else than a comma, or the parseString function will get confused
  String parsedGraph = graph; // Extra copy created so we don't mess up the incoming String
  parsedGraph = parseTemplate(parsedGraph);
  parsedGraph.replace(',', NOT_A_COMMA);

  std::vector<P104_bargraph_struct> barGraphs;
  uint8_t currentBar = 0;
  bool    loop       = true;

  // Parse the graph-string
  while (loop && currentBar < 8) { // Maximum 8 valuesets possible
    String graphpart = parseString(parsedGraph, currentBar + 1, '|');
    graphpart.trim();
    graphpart.replace(NOT_A_COMMA, ',');

    if (graphpart.isEmpty()) {
      loop = false;
    } else {
      barGraphs.push_back(P104_bargraph_struct(currentBar));
    }

    if (loop && validDoubleFromString(parseString(graphpart, 1), barGraphs[currentBar].value)) { // value
      String datapart = parseString(graphpart, 2);                                               // max (default: 100.0)

      if (datapart.isEmpty()) {
        barGraphs[currentBar].max = 100.0;
      } else {
        validDoubleFromString(datapart, barGraphs[currentBar].max);
      }
      datapart = parseString(graphpart, 3); // min (default: 0.0)

      if (datapart.isEmpty()) {
        barGraphs[currentBar].min = 0.0;
      } else {
        validDoubleFromString(datapart, barGraphs[currentBar].min);
      }
      datapart = parseString(graphpart, 4); // direction

      if (datapart.isEmpty()) {
        barGraphs[currentBar].direction = 0;
      } else {
        int value = 0;
        validIntFromString(datapart, value);
        barGraphs[currentBar].direction = value;
      }
      datapart = parseString(graphpart, 5); // barType

      if (datapart.isEmpty()) {
        barGraphs[currentBar].barType = 0;
      } else {
        int value = 0;
        validIntFromString(datapart, value);
        barGraphs[currentBar].barType = value;
      }

      if (definitelyGreaterThan(barGraphs[currentBar].min, barGraphs[currentBar].max)) {
        std::swap(barGraphs[currentBar].min, barGraphs[currentBar].max);
      }
    }
    #  ifdef P104_DEBUG

    if (logAllText && loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      if (log.reserve(70)) {
        log = F("dotmatrix: Bar-graph: ");

        if (loop) {
          log += currentBar;
          log += F(" in: ");
          log += graphpart;
          log += F(" value: ");
          log += barGraphs[currentBar].value;
          log += F(" max: ");
          log += barGraphs[currentBar].max;
          log += F(" min: ");
          log += barGraphs[currentBar].min;
          log += F(" dir: ");
          log += barGraphs[currentBar].direction;
          log += F(" typ: ");
          log += barGraphs[currentBar].barType;
        } else {
          log += F(" bsize: ");
          log += barGraphs.size();
        }
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }
    #  endif // ifdef P104_DEBUG
    currentBar++; // next
    delay(0); // Leave some breathingroom
  }
  #  undef NOT_A_COMMA

  if (barGraphs.size() > 0) {
    uint8_t  barWidth = 8 / barGraphs.size(); // Divide the 8 pixel width per number of bars to show
    int16_t  pixTop, pixBottom;
    uint16_t zeroPoint;
    #  ifdef P104_DEBUG
    String log;

    if (logAllText &&
        loglevelActiveFor(LOG_LEVEL_INFO) &&
        log.reserve(64)) {
      log  = F("dotmatrix: bar Width: ");
      log += barWidth;
      log += F(" low: ");
      log += zstruct._lower;
      log += F(" high: ");
      log += zstruct._upper;
    }
    #  endif // ifdef P104_DEBUG
    modulesOnOff(zstruct._startModule, zstruct._startModule + zstruct.size - 1, MD_MAX72XX::MD_OFF); // Stop updates on modules
    P->setIntensity(zstruct.zone - 1, zstruct.brightness); // don't forget to set the brightness
    uint8_t row = 0;

    if ((barGraphs.size() == 3) || (barGraphs.size() == 5) || (barGraphs.size() == 6)) { // Center within the rows a bit
      for (; row < (barGraphs.size() == 5 ? 2 : 1); row++) {
        for (uint8_t col = zstruct._lower; col <= zstruct._upper; col++) {
          pM->setPoint(row, col, false);                                                 // all off

          if (col % 16 == 0) { delay(0); }
        }
        delay(0); // Leave some breathingroom
      }
    }

    for (auto it = barGraphs.begin(); it != barGraphs.end(); ++it) {
      if (essentiallyEqual(it->min, 0.0)) {
        pixTop    = zstruct._lower - 1 + (((zstruct._upper + 1) - zstruct._lower) / it->max) * it->value;
        pixBottom = zstruct._lower - 1;
        zeroPoint = 0;
      } else {
        if (definitelyLessThan(it->min, 0.0) &&
            definitelyGreaterThan(it->max,           0.0) &&
            definitelyGreaterThan(it->max - it->min, 0.01)) { // Zero-point is used
          zeroPoint = (it->min * -1.0) / ((it->max - it->min) / (1.0 * ((zstruct._upper + 1) - zstruct._lower)));
        } else {
          zeroPoint = 0;
        }
        pixTop    = zstruct._lower + zeroPoint + (((zstruct._upper + 1) - zstruct._lower) / (it->max - it->min)) * it->value;
        pixBottom = zstruct._lower + zeroPoint;

        if (definitelyLessThan(it->value, 0.0)) {
          std::swap(pixTop, pixBottom);
        }
      }

      if (it->direction == 1) { // Left to right display: Flip values within the lower/upper range
        pixBottom = zstruct._upper - (pixBottom - zstruct._lower);
        pixTop    = zstruct._lower + (zstruct._upper - pixTop);
        std::swap(pixBottom, pixTop);
        zeroPoint = zstruct._upper - zstruct._lower - zeroPoint + (zeroPoint == 0 ? 1 : 0);
      }
      #  ifdef P104_DEBUG_DEV

      if (logAllText && loglevelActiveFor(LOG_LEVEL_INFO)) {
        log += F(" B: ");
        log += pixBottom;
        log += F(" T: ");
        log += pixTop;
        log += F(" Z: ");
        log += zeroPoint;
      }
      #  endif // ifdef P104_DEBUG_DEV
      drawOneBarGraph(zstruct._lower, zstruct._upper, pixBottom, pixTop, zeroPoint, barWidth, it->barType, row);
      row += barWidth; // Next set of rows
      delay(0); // Leave some breathingroom
    }

    for (; row < 8; row++) {           // Clear unused rows
      for (uint8_t col = zstruct._lower; col <= zstruct._upper; col++) {
        pM->setPoint(row, col, false); // all off

        if (col % 16 == 0) { delay(0); }
      }
      delay(0); // Leave some breathingroom
    }
    #  ifdef P104_DEBUG

    if (logAllText && loglevelActiveFor(LOG_LEVEL_INFO)) {
      addLogMove(LOG_LEVEL_INFO, log);
    }
    #  endif // ifdef P104_DEBUG
    modulesOnOff(zstruct._startModule, zstruct._startModule + zstruct.size - 1, MD_MAX72XX::MD_ON);  // Continue updates on modules
  }
}

# endif // ifdef P104_USE_BAR_GRAPH

/**************************************************
 * Check if an animation is available in the current build
 *************************************************/
bool isAnimationAvailable(uint8_t animation, bool noneIsAllowed = false) {
  textEffect_t selection = static_cast<textEffect_t>(animation);

  switch (selection) {
    case PA_NO_EFFECT:
    {
      return noneIsAllowed;
    }
    case PA_PRINT:
    case PA_SCROLL_UP:
    case PA_SCROLL_DOWN:
    case PA_SCROLL_LEFT:
    case PA_SCROLL_RIGHT:
    {
      return true;
    }
    # if ENA_SPRITE
    case PA_SPRITE:
    {
      return true;
    }
    # endif // ENA_SPRITE
    # if ENA_MISC
    case PA_SLICE:
    case PA_MESH:
    case PA_FADE:
    case PA_DISSOLVE:
    case PA_BLINDS:
    case PA_RANDOM:
    {
      return true;
    }
    # endif // ENA_MISC
    # if ENA_WIPE
    case PA_WIPE:
    case PA_WIPE_CURSOR:
    {
      return true;
    }
    # endif // ENA_WIPE
    # if ENA_SCAN
    case PA_SCAN_HORIZ:
    case PA_SCAN_HORIZX:
    case PA_SCAN_VERT:
    case PA_SCAN_VERTX:
    {
      return true;
    }
    # endif // ENA_SCAN
    # if ENA_OPNCLS
    case PA_OPENING:
    case PA_OPENING_CURSOR:
    case PA_CLOSING:
    case PA_CLOSING_CURSOR:
    {
      return true;
    }
    # endif // ENA_OPNCLS
    # if ENA_SCR_DIA
    case PA_SCROLL_UP_LEFT:
    case PA_SCROLL_UP_RIGHT:
    case PA_SCROLL_DOWN_LEFT:
    case PA_SCROLL_DOWN_RIGHT:
    {
      return true;
    }
    # endif // ENA_SCR_DIA
    # if ENA_GROW
    case PA_GROW_UP:
    case PA_GROW_DOWN:
    {
      return true;
    }
    # endif // ENA_GROW
    default:
      return false;
  }
}

/*******************************************************
 * handlePluginWrite : process commands
 ******************************************************/
bool P104_data_struct::handlePluginWrite(taskIndex_t   taskIndex,
                                         const String& string) {
  # ifdef P104_USE_COMMANDS
  bool reconfigure = false;
  # endif // ifdef P104_USE_COMMANDS
  bool   success = false;
  String command = parseString(string, 1);

  if ((nullptr != P) && command.equals(F("dotmatrix"))) { // main command: dotmatrix
    String sub = parseString(string, 2);

    int zoneIndex;
    String string4 = parseStringKeepCase(string, 4);
    # ifdef P104_USE_COMMANDS
    int value4;
    validIntFromString(string4, value4);
    # endif // ifdef P104_USE_COMMANDS

    // Global subcommands
    if (sub.equals(F("clear")) && // subcommand: clear[,all]
        (string4.isEmpty() ||
         string4.equalsIgnoreCase(F("all")))) {
      P->displayClear();
      success = true;
    }

    if (sub.equals(F("update")) && // subcommand: update[,all]
        (string4.isEmpty() ||
         string4.equalsIgnoreCase(F("all")))) {
      updateZone(0, P104_zone_struct(0));
      success = true;
    }

    // Zone-specific subcommands
    if (validIntFromString(parseString(string, 3), zoneIndex) &&
        (zoneIndex > 0) &&
        (static_cast<unsigned int>(zoneIndex) <= zones.size())) {
      // subcommands are processed in the same order as they are presented in the UI
      for (auto it = zones.begin(); it != zones.end() && !success; ++it) {
        if ((it->zone == zoneIndex)) {  // This zone
          if (sub.equals(F("clear"))) { // subcommand: clear,<zone>
            P->displayClear(zoneIndex - 1);
            success = true;
            break;
          }

          if (sub.equals(F("update"))) { // subcommand: update,<zone>
            updateZone(zoneIndex, *it);
            success = true;
            break;
          }

          # ifdef P104_USE_COMMANDS

          if (sub.equals(F("size")) && // subcommand: size,<zone>,<size> (1..)
              (value4 > 0) &&
              (value4 <= P104_MAX_MODULES_PER_ZONE)) {
            reconfigure = (it->size != value4);
            it->size    = value4;
            success     = true;
            break;
          }
          # endif // ifdef P104_USE_COMMANDS

          if ((sub.equals(F("txt")) ||                                                          // subcommand: [set]txt,<zone>,<text> (only
               sub.equals(F("settxt"))) &&                                                      // allowed for zones with Text content)
              ((it->content == P104_CONTENT_TEXT) || (it->content == P104_CONTENT_TEXT_REV))) { // no length check, so longer than the UI
                                                                                                // allows is made possible
            if (sub.equals(F("settxt")) &&                                                      // subcommand: settxt,<zone>,<text> (stores
                (string4.length() <= P104_MAX_TEXT_LENGTH_PER_ZONE)) {                          // the text in the settings, is not saved)
              it->text = string4;                                                               // Only if not too long, could 'blow up' the
            }                                                                                   // settings when saved
            displayOneZoneText(zoneIndex - 1, *it, string4);
            success = true;
            break;
          }

          # ifdef P104_USE_COMMANDS

          if (sub.equals(F("content")) && // subcommand: content,<zone>,<contenttype> (0..<P104_CONTENT_count>-1)
              (value4 >= 0) &&
              (value4 < P104_CONTENT_count)) {
            reconfigure = (it->content != value4);
            it->content = value4;
            success     = true;
            break;
          }

          if (sub.equals(F("alignment")) &&                             // subcommand: alignment,<zone>,<alignment> (0..3)
              (value4 >= 0) &&
              (value4 <= static_cast<int>(textPosition_t::PA_RIGHT))) { // last item in the enum
            it->alignment = value4;
            success       = true;
            break;
          }

          if (sub.equals(F("anim.in")) && // subcommand: anim.in,<zone>,<animation> (1..)
              isAnimationAvailable(value4)) {
            it->animationIn = value4;
            success         = true;
            break;
          }

          if (sub.equals(F("speed")) && // subcommand: speed,<zone>,<speed_ms> (0..P104_MAX_SPEED_PAUSE_VALUE)
              (value4 >= 0) &&
              (value4 <= P104_MAX_SPEED_PAUSE_VALUE)) {
            it->speed = value4;
            success   = true;
            break;
          }

          if (sub.equals(F("anim.out")) && // subcommand: anim.out,<zone>,<animation> (0..)
              isAnimationAvailable(value4, true)) {
            it->animationOut = value4;
            success          = true;
            break;
          }

          if (sub.equals(F("pause")) && // subcommand: pause,<zone>,<pause_ms> (0..P104_MAX_SPEED_PAUSE_VALUE)
              (value4 >= 0) &&
              (value4 <= P104_MAX_SPEED_PAUSE_VALUE)) {
            it->pause = value4;
            success   = true;
            break;
          }

          if (sub.equals(F("font")) && // subcommand: font,<zone>,<font id> (only for incuded font id's)
              (
                (value4 == 0)
                #  ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
                || (value4 == P104_DOUBLE_HEIGHT_FONT_ID)
                #  endif // ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
                #  ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
                || (value4 == P104_FULL_DOUBLEHEIGHT_FONT_ID)
                #  endif // ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
                #  ifdef P104_USE_VERTICAL_FONT
                || (value4 == P104_VERTICAL_FONT_ID)
                #  endif // ifdef P104_USE_VERTICAL_FONT
                #  ifdef P104_USE_EXT_ASCII_FONT
                || (value4 == P104_EXT_ASCII_FONT_ID)
                #  endif // ifdef P104_USE_EXT_ASCII_FONT
                #  ifdef P104_USE_ARABIC_FONT
                || (value4 == P104_ARABIC_FONT_ID)
                #  endif // ifdef P104_USE_ARABIC_FONT
                #  ifdef P104_USE_GREEK_FONT
                || (value4 == P104_GREEK_FONT_ID)
                #  endif // ifdef P104_USE_GREEK_FONT
                #  ifdef P104_USE_KATAKANA_FONT
                || (value4 == P104_KATAKANA_FONT_ID)
                #  endif // ifdef P104_USE_KATAKANA_FONT
              )
              ) {
            reconfigure = (it->font != value4);
            it->font    = value4;
            success     = true;
            break;
          }

          if (sub.equals(F("inverted")) && // subcommand: inverted,<zone>,<invertedstate> (disable/enable)
              (value4 >= 0) &&
              (value4 <= 1)) {
            reconfigure  = (it->inverted != value4);
            it->inverted = value4;
            success      = true;
            break;
          }

          #  if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)

          if (sub.equals(F("layout")) && // subcommand: layout,<zone>,<layout> (0..2), only when double-height font is available
              (value4 >= 0) &&
              (value4 <= P104_LAYOUT_DOUBLE_LOWER)) {
            reconfigure = (it->layout != value4);
            it->layout  = value4;
            success     = true;
            break;
          }
          #  endif // if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)

          if (sub.equals(F("specialeffect")) && // subcommand: specialeffect,<zone>,<effect> (0..3)
              (value4 >= 0) &&
              (value4 <= P104_SPECIAL_EFFECT_BOTH)) {
            reconfigure       = (it->specialEffect != value4);
            it->specialEffect = value4;
            success           = true;
            break;
          }

          if (sub.equals(F("offset")) && // subcommand: offset,<zone>,<size> (0..<size>-1)
              (value4 >= 0) &&
              (value4 < P104_MAX_MODULES_PER_ZONE) &&
              (value4 < it->size)) {
            reconfigure = (it->offset != value4);
            it->offset  = value4;
            success     = true;
            break;
          }

          if (sub.equals(F("brightness")) && // subcommand: brightness,<zone>,<brightness> (0..15)
              (value4 >= 0) &&
              (value4 <= P104_BRIGHTNESS_MAX)) {
            it->brightness = value4;
            P->setIntensity(zoneIndex - 1, it->brightness); // Change brightness directly
            success = true;
            break;
          }

          if (sub.equals(F("repeat")) && // subcommand: repeaat,<zone>,<repeat_sec> (-1..86400 = 24h)
              (value4 >= -1) &&
              (value4 <= P104_MAX_REPEATDELAY_VALUE)) {
            it->repeatDelay = value4;
            success         = true;

            if (it->repeatDelay > -1) {
              it->_repeatTimer = millis();
            }
            break;
          }
          # else // ifdef P104_USE_COMMANDS
          {
            String validCommands = F(
              "|size|content|alignment|anim.in|speed|anim.out|pause|font|inverted|layout|specialeffect|offset|brightness|repeat|");
            String testSub = '|';
            testSub += sub;
            testSub += '|';

            if (validCommands.indexOf(testSub) > -1) {
              addLog(LOG_LEVEL_ERROR, F("dotmatrix: subcommand not included in build."));
            }
          }
          # endif // ifdef P104_USE_COMMANDS

          # ifdef P104_USE_BAR_GRAPH

          if ((sub.equals(F("bar")) ||                                 // subcommand: [set]bar,<zone>,<graph-string> (only allowed for zones
               sub.equals(F("setbar"))) &&                             // with Bargraph content) no length check, so longer than the UI
              (it->content == P104_CONTENT_BAR_GRAPH)) {               // allows is made possible
            if (sub.equals(F("setbar")) &&                             // subcommand: setbar,<zone>,<graph-string> (stores the graph-string
                (string4.length() <= P104_MAX_TEXT_LENGTH_PER_ZONE)) { // in the settings, is not saved)
              it->text = string4;                                      // Only if not too long, could 'blow up' the settings when saved
            }
            displayBarGraph(zoneIndex - 1, *it, string4);
            success = true;
            break;
          }
          # endif // ifdef P104_USE_BAR_GRAPH

          // FIXME TD-er: success is always false here. Maybe this must be done outside the for-loop?
          if (success) { // Reset the repeat timer
            if (it->repeatDelay > -1) {
              it->_repeatTimer = millis();
            }
          }
        }
      }
    }
  }

  # ifdef P104_USE_COMMANDS

  if (reconfigure) {
    configureZones(); // Re-initialize
    success = true;   // Successful
  }
  # endif // ifdef P104_USE_COMMANDS

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log;

    if (log.reserve(34 + string.length())) {
      log = F("dotmatrix: command ");

      if (!success) { log += F("NOT "); }
      log += F("succesful: ");
      log += string;
      addLogMove(LOG_LEVEL_INFO, log);
    }
  }

  return success; // Default: unknown command
}

int8_t getTime(char *psz,
               bool  seconds  = false,
               bool  colon    = true,
               bool  time12h  = false,
               bool  timeAmpm = false) {
  uint16_t h, M, s;
  String   ampm;

  # ifdef P104_USE_DATETIME_OPTIONS

  if (time12h) {
    if (timeAmpm) {
      ampm = (node_time.hour() >= 12 ? F("p") : F("a"));
    }
    h = node_time.hour() % 12;

    if (h == 0) { h = 12; }
  } else
  # endif // ifdef P104_USE_DATETIME_OPTIONS
  {
    h = node_time.hour();
  }
  M = node_time.minute();

  if (!seconds) {
    sprintf_P(psz, PSTR("%02d%c%02d%s"), h, (colon ? ':' : ' '), M, ampm.c_str());
  } else {
    s = node_time.second();
    sprintf_P(psz, PSTR("%02d%c%02d %02d%s"), h, (colon ? ':' : ' '), M, s, ampm.c_str());
  }
  return M;
}

void getDate(char           *psz,
             bool            showYear = true,
             bool            fourDgt  = false
             # ifdef         P104_USE_DATETIME_OPTIONS
             , const uint8_t dateFmt = 0
             , const uint8_t dateSep = 0
             # endif // ifdef P104_USE_DATETIME_OPTIONS
             ) {
  uint16_t d, m, y;
  const uint16_t year = node_time.year() - (fourDgt ? 0 : 2000);

  # ifdef P104_USE_DATETIME_OPTIONS
  const String separators = F(" /-.");
  const char   sep        = separators[dateSep];
  # else // ifdef P104_USE_DATETIME_OPTIONS
  const char sep = ' ';
  # endif // ifdef P104_USE_DATETIME_OPTIONS

  d = node_time.day();
  m = node_time.month();
  y = year;
  # ifdef P104_USE_DATETIME_OPTIONS

  if (showYear) {
    switch (dateFmt) {
      case P104_DATE_FORMAT_US:
        d = node_time.month();
        m = node_time.day();
        y = year;
        break;
      case P104_DATE_FORMAT_JP:
        d = year;
        m = node_time.month();
        y = node_time.day();
        break;
    }
  } else {
    if ((dateFmt == P104_DATE_FORMAT_US) ||
        (dateFmt == P104_DATE_FORMAT_JP)) {
      std::swap(d, m);
    }
  }
  # endif // ifdef P104_USE_DATETIME_OPTIONS

  if (showYear) {
    sprintf_P(psz, PSTR("%02d%c%02d%c%02d"), d, sep, m, sep, y);     // %02d will expand to 04 when needed
  } else {
    sprintf_P(psz, PSTR("%02d%c%02d"), d, sep, m);
  }
}

uint8_t getDateTime(char           *psz,
                    bool            colon    = true,
                    bool            time12h  = false,
                    bool            timeAmpm = false,
                    bool            fourDgt  = false
                    # ifdef         P104_USE_DATETIME_OPTIONS
                    , const uint8_t dateFmt = 0
                    , const uint8_t dateSep = 0
                    # endif // ifdef P104_USE_DATETIME_OPTIONS
                    ) {
  String   ampm;
  uint16_t d, M, y;
  uint8_t  h, m;
  const uint16_t year = node_time.year() - (fourDgt ? 0 : 2000);

  # ifdef P104_USE_DATETIME_OPTIONS
  const String separators = F(" /-.");
  const char   sep        = separators[dateSep];
  # else // ifdef P104_USE_DATETIME_OPTIONS
  const char sep = ' ';
  # endif // ifdef P104_USE_DATETIME_OPTIONS

  # ifdef P104_USE_DATETIME_OPTIONS

  if (time12h) {
    if (timeAmpm) {
      ampm = (node_time.hour() >= 12 ? F("p") : F("a"));
    }
    h = node_time.hour() % 12;

    if (h == 0) { h = 12; }
  } else
  # endif // ifdef P104_USE_DATETIME_OPTIONS
  {
    h = node_time.hour();
  }
  M = node_time.minute();

  # ifdef P104_USE_DATETIME_OPTIONS

  switch (dateFmt) {
    case P104_DATE_FORMAT_US:
      d = node_time.month();
      m = node_time.day();
      y = year;
      break;
    case P104_DATE_FORMAT_JP:
      d = year;
      m = node_time.month();
      y = node_time.day();
      break;
    default:
  # endif // ifdef P104_USE_DATETIME_OPTIONS
  d = node_time.day();
  m = node_time.month();
  y = year;
  # ifdef P104_USE_DATETIME_OPTIONS
}

  # endif // ifdef P104_USE_DATETIME_OPTIONS
  sprintf_P(psz, PSTR("%02d%c%02d%c%02d %02d%c%02d%s"), d, sep, m, sep, y, h, (colon ? ':' : ' '), M, ampm.c_str()); // %02d will expand to
                                                                                                                     // 04 when needed
  return M;
}

# if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
void createHString(String& string) {
  const uint16_t stringLen = string.length();

  for (uint16_t i = 0; i < stringLen; i++) {
    string[i] |= 0x80; // use 'high' part of the font, by adding 0x80
  }
}

# endif // if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)

void reverseStr(String& str) {
  const uint16_t n = str.length();

  // Swap characters starting from two corners
  for (uint16_t i = 0; i < n / 2; i++) {
    std::swap(str[i], str[n - i - 1]);
  }
}

/************************************************************************
 * execute all PLUGIN_ONE_PER_SECOND tasks
 ***********************************************************************/
bool P104_data_struct::handlePluginOncePerSecond(struct EventStruct *event) {
  if (nullptr == P) { return false; }
  bool redisplay = false;
  bool success   = false;

  # ifdef P104_USE_DATETIME_OPTIONS
  bool useFlasher = !bitRead(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_FLASH);
  bool time12h    = bitRead(P104_CONFIG_DATETIME,  P104_CONFIG_DATETIME_12H);
  bool timeAmpm   = bitRead(P104_CONFIG_DATETIME,  P104_CONFIG_DATETIME_AMPM);
  bool year4dgt   = bitRead(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_YEAR4DGT);
  # else // ifdef P104_USE_DATETIME_OPTIONS
  bool useFlasher = true;
  bool time12h    = false;
  bool timeAmpm   = false;
  bool year4dgt   = false;
  # endif // ifdef P104_USE_DATETIME_OPTIONS
  bool newFlasher = !flasher && useFlasher;

  for (auto it = zones.begin(); it != zones.end(); ++it) {
    redisplay = false;

    if (P->getZoneStatus(it->zone - 1)) { // Animations done?
      switch (it->content) {
        case P104_CONTENT_TIME:           // time
        case P104_CONTENT_TIME_SEC:       // time sec
        {
          bool   useSeconds = (it->content == P104_CONTENT_TIME_SEC);
          int8_t m          = getTime(szTimeL, useSeconds, flasher || !useFlasher, time12h, timeAmpm);
          flasher          = newFlasher;
          redisplay        = useFlasher || useSeconds || (it->_lastChecked != m);
          it->_lastChecked = m;
          break;
        }
        case P104_CONTENT_DATE4: // date/4
        case P104_CONTENT_DATE6: // date/6
        {
          if (it->_lastChecked != node_time.day()) {
            getDate(szTimeL,
                    it->content != P104_CONTENT_DATE4,
                    year4dgt
                    # ifdef P104_USE_DATETIME_OPTIONS
                    , get4BitFromUL(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_FORMAT)
                    , get4BitFromUL(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_SEP_CHAR)
                    # endif // ifdef P104_USE_DATETIME_OPTIONS
                    );
            redisplay        = true;
            it->_lastChecked = node_time.day();
          }
          break;
        }
        case P104_CONTENT_DATE_TIME: // date-time/9
        {
          int8_t m = getDateTime(szTimeL,
                                 flasher || !useFlasher,
                                 time12h,
                                 timeAmpm,
                                 year4dgt
                                 # ifdef P104_USE_DATETIME_OPTIONS
                                 , get4BitFromUL(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_FORMAT)
                                 , get4BitFromUL(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_SEP_CHAR)
                                 # endif // ifdef P104_USE_DATETIME_OPTIONS
                                 );
          flasher          = newFlasher;
          redisplay        = useFlasher || (it->_lastChecked != m);
          it->_lastChecked = m;
          break;
        }
        default:
          break;
      }

      if (redisplay) {
        displayOneZoneText(it->zone - 1, *it, String(szTimeL));
        P->displayReset(it->zone - 1);

        if (it->repeatDelay > -1) {
          it->_repeatTimer = millis();
        }
      }
    }
    delay(0); // Leave some breathingroom
  }

  if (redisplay) {
    // synchronise the start
    P->synchZoneStart();
  }
  return redisplay || success;
}

/***************************************************
 * restart a zone if the repeat delay (if any) has passed
 **************************************************/
void P104_data_struct::checkRepeatTimer(uint8_t z) {
  if (nullptr == P) { return; }
  bool handled = false;

  for (auto it = zones.begin(); it != zones.end() && !handled; ++it) {
    if (it->zone == z + 1) {
      handled = true;

      if ((it->repeatDelay > -1) && (timePassedSince(it->_repeatTimer) >= (it->repeatDelay - 1) * 1000)) { // Compensated for the '1' in
                                                                                                           // PLUGIN_ONE_PER_SECOND
        # ifdef P104_DEBUG

        if (logAllText && loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log;
          log.reserve(51);
          log  = F("dotmatrix: Repeat zone: ");
          log += it->zone;
          log += F(" delay: ");
          log += it->repeatDelay;
          log += F(" (");
          log += (timePassedSince(it->_repeatTimer) / 1000.0f); // Decimals can be useful here
          log += ')';
          addLogMove(LOG_LEVEL_INFO, log);
        }
        # endif // ifdef P104_DEBUG

        if ((it->content == P104_CONTENT_TEXT) ||
            (it->content == P104_CONTENT_TEXT_REV)) {
          displayOneZoneText(it->zone - 1, *it, sZoneInitial[it->zone - 1]); // Re-send last displayed text
          P->displayReset(it->zone - 1);
        }

        if ((it->content == P104_CONTENT_TIME) ||
            (it->content == P104_CONTENT_TIME_SEC) ||
            (it->content == P104_CONTENT_DATE4) ||
            (it->content == P104_CONTENT_DATE6) ||
            (it->content == P104_CONTENT_DATE_TIME)) {
          it->_lastChecked = -1; // Invalidate so next run will re-display the date/time
        }
        # ifdef P104_USE_BAR_GRAPH

        if (it->content == P104_CONTENT_BAR_GRAPH) {
          displayBarGraph(it->zone - 1, *it, sZoneInitial[it->zone - 1]); // Re-send last displayed bar graph
        }
        # endif // ifdef P104_USE_BAR_GRAPH
        it->_repeatTimer = millis();
      }
    }
    delay(0); // Leave some breathingroom
  }
}


/***************************************
 * saveSettings gather the zones data from the UI and store in customsettings
 **************************************/
bool P104_data_struct::saveSettings() {
  error = String(); // Clear
  String zbuffer;

  # ifdef P104_DEBUG_DEV
  {
    String log;

    if (loglevelActiveFor(LOG_LEVEL_INFO) &&
        log.reserve(64)) {
      log  = F("P104: saving zones, count: ");
      log += expectedZones;
      addLogMove(LOG_LEVEL_INFO, log);
    }
  }
  # endif // ifdef P104_DEBUG_DEV

  uint8_t index      = 0;
  uint8_t action     = P104_ACTION_NONE;
  uint8_t zoneIndex  = 0;
  int8_t  zoneOffset = 0;

  zones.clear(); // Start afresh

  for (uint8_t zCounter = 0; zCounter < expectedZones; zCounter++) {
    # ifdef P104_USE_ZONE_ACTIONS
    action = getFormItemInt(getPluginCustomArgName(index + P104_OFFSET_ACTION));

    if (((action == P104_ACTION_ADD_ABOVE) && (zoneOrder == 0)) ||
        ((action == P104_ACTION_ADD_BELOW) && (zoneOrder == 1))) {
      zones.push_back(P104_zone_struct(0));
      zoneOffset++;
      #  ifdef P104_DEBUG_DEV

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log  = F("P104: insert before zone: ");
        log += (zoneIndex + 1);
        addLogMove(LOG_LEVEL_INFO, log);
      }
      #  endif // ifdef P104_DEBUG_DEV
    }
    # endif    // ifdef P104_USE_ZONE_ACTIONS
    zoneIndex = zCounter + zoneOffset;

    if (action == P104_ACTION_DELETE) {
      zoneOffset--;
    } else {
      # ifdef P104_DEBUG_DEV

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log  = F("P104: read zone: ");
        log += (zoneIndex + 1);
        addLogMove(LOG_LEVEL_INFO, log);
      }
      # endif // ifdef P104_DEBUG_DEV
      zones.push_back(P104_zone_struct(zoneIndex + 1));

      zones[zoneIndex].size          = getFormItemIntCustomArgName(index + P104_OFFSET_SIZE);
      zones[zoneIndex].text          = wrapWithQuotes(webArg(getPluginCustomArgName(index + P104_OFFSET_TEXT)));
      zones[zoneIndex].content       = getFormItemIntCustomArgName(index + P104_OFFSET_CONTENT);
      zones[zoneIndex].alignment     = getFormItemIntCustomArgName(index + P104_OFFSET_ALIGNMENT);
      zones[zoneIndex].animationIn   = getFormItemIntCustomArgName(index + P104_OFFSET_ANIM_IN);
      zones[zoneIndex].speed         = getFormItemIntCustomArgName(index + P104_OFFSET_SPEED);
      zones[zoneIndex].animationOut  = getFormItemIntCustomArgName(index + P104_OFFSET_ANIM_OUT);
      zones[zoneIndex].pause         = getFormItemIntCustomArgName(index + P104_OFFSET_PAUSE);
      zones[zoneIndex].font          = getFormItemIntCustomArgName(index + P104_OFFSET_FONT);
      zones[zoneIndex].layout        = getFormItemIntCustomArgName(index + P104_OFFSET_LAYOUT);
      zones[zoneIndex].specialEffect = getFormItemIntCustomArgName(index + P104_OFFSET_SPEC_EFFECT);
      zones[zoneIndex].offset        = getFormItemIntCustomArgName(index + P104_OFFSET_OFFSET);
      zones[zoneIndex].inverted      = getFormItemIntCustomArgName(index + P104_OFFSET_INVERTED);

      if (zones[zoneIndex].size != 0) { // for newly added zone, use defaults
        zones[zoneIndex].brightness  = getFormItemIntCustomArgName(index + P104_OFFSET_BRIGHTNESS);
        zones[zoneIndex].repeatDelay = getFormItemIntCustomArgName(index + P104_OFFSET_REPEATDELAY);
      }
    }
    # ifdef P104_DEBUG_DEV

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log  = F("P104: add zone: ");
      log += (zoneIndex + 1);
      addLogMove(LOG_LEVEL_INFO, log);
    }
    # endif // ifdef P104_DEBUG_DEV

    # ifdef P104_USE_ZONE_ACTIONS

    if (((action == P104_ACTION_ADD_BELOW) && (zoneOrder == 0)) ||
        ((action == P104_ACTION_ADD_ABOVE) && (zoneOrder == 1))) {
      zones.push_back(P104_zone_struct(0));
      zoneOffset++;
      #  ifdef P104_DEBUG_DEV

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log  = F("P104: insert after zone: ");
        log += (zoneIndex + 2);
        addLogMove(LOG_LEVEL_INFO, log);
      }
      #  endif // ifdef P104_DEBUG_DEV
    }
    # endif    // ifdef P104_USE_ZONE_ACTIONS

    index += P104_OFFSET_COUNT;
    delay(0);
  }

  uint16_t bufferSize;
  int saveOffset = 0;

  numDevices = 0;                      // Count the number of connected display units

  bufferSize = P104_CONFIG_VERSION_V2; // Save special marker that we're using V2 settings
  // This write is counting
  error      += SaveToFile(SettingsType::Enum::CustomTaskSettings_Type, taskIndex, (uint8_t *)&bufferSize, sizeof(bufferSize), saveOffset);
  saveOffset += sizeof(bufferSize);

  if (zbuffer.reserve(P104_SETTINGS_BUFFER_V2 + 2)) {
    for (auto it = zones.begin(); it != zones.end() && error.length() == 0; ++it) {
      zbuffer.clear();

      // WARNING: Order of values should match the numeric order of P104_OFFSET_* values
      zbuffer += it->size;          // 2
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->text;          // 2 + ~15
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->content;       // 1
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->alignment;     // 1
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->animationIn;   // 2
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->speed;         // 5
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->animationOut;  // 2
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->pause;         // 5
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->font;          // 1
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->layout;        // 1
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->specialEffect; // 1
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->offset;        // 2
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->brightness;    // 2
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->repeatDelay;   // 4
      zbuffer += P104_FIELD_SEP;    // 1
      zbuffer += it->inverted;      // 1
      zbuffer += P104_FIELD_SEP;    // 1

      // 47 total + (max) 100 characters for it->text requires a buffer of ~150 (P104_SETTINGS_BUFFER_V2), but only the required length is
      // stored with the length prefixed

      numDevices += (it->size != 0 ? it->size : 1) + it->offset;                                // Count corrected for newly added zones

      if (saveOffset + zbuffer.length() + (sizeof(bufferSize) * 2) > (DAT_TASKS_CUSTOM_SIZE)) { // Detect ourselves if we've reached the
        error.reserve(55);                                                                      // high-water mark
        error += F("Total combination of Zones & text too long to store.\n");
        addLog(LOG_LEVEL_ERROR, error);
      } else {
        // Store length of buffer
        bufferSize = zbuffer.length();

        // As we write in parts, only count as single write.
        if (RTC.flashDayCounter > 0) {
          RTC.flashDayCounter--;
        }
        error += SaveToFile(SettingsType::Enum::CustomTaskSettings_Type,
                            taskIndex,
                            (uint8_t *)&bufferSize,
                            sizeof(bufferSize),
                            saveOffset);
        saveOffset += sizeof(bufferSize);

        // As we write in parts, only count as single write.
        if (RTC.flashDayCounter > 0) {
          RTC.flashDayCounter--;
        }
        error += SaveToFile(SettingsType::Enum::CustomTaskSettings_Type,
                            taskIndex,
                            (uint8_t *)zbuffer.c_str(),
                            bufferSize,
                            saveOffset);
        saveOffset += bufferSize;

        # ifdef P104_DEBUG_DEV

        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log;
          log.reserve(64);
          log  = F("P104: saveSettings zone: ");
          log += it->zone;
          log += F(" bufferSize: ");
          log += bufferSize;
          log += F(" offset: ");
          log += saveOffset;
          addLogMove(LOG_LEVEL_INFO, log);
          zbuffer.replace(P104_FIELD_SEP, P104_FIELD_DISP);
          addLog(LOG_LEVEL_INFO, zbuffer);
        }
        # endif // ifdef P104_DEBUG_DEV
      }

      delay(0);
    }

    // Store an End-of-settings marker == 0
    bufferSize = 0u;

    // This write is counting
    SaveToFile(SettingsType::Enum::CustomTaskSettings_Type, taskIndex, (uint8_t *)&bufferSize, sizeof(bufferSize), saveOffset);

    if (numDevices > 255) {
      error += F("More than 255 modules configured (");
      error += numDevices;
      error += ')';
      error += '\n';
    }
  } else {
    addLog(LOG_LEVEL_ERROR, F("DOTMATRIX: Can't allocate string for saving settings, insufficient memory!"));
    return false; // Don't continue
  }

  return error.isEmpty();
}

/**************************************************************
* webform_load
**************************************************************/
bool P104_data_struct::webform_load(struct EventStruct *event) {
  addFormSubHeader(F("Device settings"));

  {                                       // Hardware types
    # define P104_hardwareTypeCount 8
    const __FlashStringHelper *hardwareTypes[P104_hardwareTypeCount] = {
      F("Generic (DR:0, CR:1, RR:0)"),    // 010
      F("Parola (DR:1, CR:1, RR:0)"),     // 110
      F("FC16 (DR:1, CR:0, RR:0)"),       // 100
      F("IC Station (DR:1, CR:1, RR:1)"), // 111
      F("Other 1 (DR:0, CR:0, RR:0)"),    // 000
      F("Other 2 (DR:0, CR:0, RR:1)"),    // 001
      F("Other 3 (DR:0, CR:1, RR:1)"),    // 011
      F("Other 4 (DR:1, CR:0, RR:1)")     // 101
    };
    const int hardwareOptions[P104_hardwareTypeCount] = {
      static_cast<int>(MD_MAX72XX::moduleType_t::GENERIC_HW),
      static_cast<int>(MD_MAX72XX::moduleType_t::PAROLA_HW),
      static_cast<int>(MD_MAX72XX::moduleType_t::FC16_HW),
      static_cast<int>(MD_MAX72XX::moduleType_t::ICSTATION_HW),
      static_cast<int>(MD_MAX72XX::moduleType_t::DR0CR0RR0_HW),
      static_cast<int>(MD_MAX72XX::moduleType_t::DR0CR0RR1_HW),
      static_cast<int>(MD_MAX72XX::moduleType_t::DR0CR1RR1_HW),
      static_cast<int>(MD_MAX72XX::moduleType_t::DR1CR0RR1_HW)
    };
    addFormSelector(F("Hardware type"),
                    F("plugin_104_hardware"),
                    P104_hardwareTypeCount,
                    hardwareTypes,
                    hardwareOptions,
                    P104_CONFIG_HARDWARETYPE);
    # ifdef P104_ADD_SETTINGS_NOTES
    addFormNote(F("DR = Digits as Rows, CR = Column Reversed, RR = Row Reversed; 0 = no, 1 = yes."));
    # endif // ifdef P104_ADD_SETTINGS_NOTES
  }

  {
    addFormCheckBox(F("Clear display on disable"), F("plugin_104_cleardisable"),
                    bitRead(P104_CONFIG_FLAGS, P104_CONFIG_FLAG_CLEAR_DISABLE));

    addFormCheckBox(F("Log all displayed text (info)"),
                    F("plugin_104_logalltext"),
                    bitRead(P104_CONFIG_FLAGS, P104_CONFIG_FLAG_LOG_ALL_TEXT));
  }

  # ifdef P104_USE_DATETIME_OPTIONS
  {
    addFormSubHeader(F("Content options"));

    addFormCheckBox(F("Clock with flashing colon"), F("plugin_104_clockflash"), !bitRead(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_FLASH));
    addFormCheckBox(F("Clock 12h display"),         F("plugin_104_clock12h"),   bitRead(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_12H));
    addFormCheckBox(F("Clock 12h AM/PM indicator"), F("plugin_104_clockampm"),  bitRead(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_AMPM));
  }
  { // Date format
    const __FlashStringHelper *dateFormats[] = {
      F("Day Month [Year]"),
      F("Month Day [Year] (US-style)"),
      F("[Year] Month Day (Japanese-style)")
    };
    const int dateFormatOptions[] = {
      P104_DATE_FORMAT_EU,
      P104_DATE_FORMAT_US,
      P104_DATE_FORMAT_JP
    };
    addFormSelector(F("Date format"), F("plugin_104_dateformat"),
                    3,
                    dateFormats, dateFormatOptions,
                    get4BitFromUL(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_FORMAT));
  }
  { // Date separator
    const __FlashStringHelper *dateSeparators[] = {
      F("Space"),
      F("Slash /"),
      F("Dash -"),
      F("Dot <b>.</b>")
    };
    const int dateSeparatorOptions[] = {
      P104_DATE_SEPARATOR_SPACE,
      P104_DATE_SEPARATOR_SLASH,
      P104_DATE_SEPARATOR_DASH,
      P104_DATE_SEPARATOR_DOT
    };
    addFormSelector(F("Date separator"), F("plugin_104_dateseparator"),
                    4,
                    dateSeparators, dateSeparatorOptions,
                    get4BitFromUL(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_SEP_CHAR));

    addFormCheckBox(F("Year uses 4 digits"), F("plugin_104_year4dgt"), bitRead(P104_CONFIG_DATETIME, P104_CONFIG_DATETIME_YEAR4DGT));
  }
  # endif // ifdef P104_USE_DATETIME_OPTIONS

  addFormSubHeader(F("Zones"));

  { // Zones
    String zonesList[P104_MAX_ZONES];
    int    zonesOptions[P104_MAX_ZONES];

    for (uint8_t i = 0; i < P104_MAX_ZONES; i++) {
      zonesList[i]    = i + 1;
      zonesOptions[i] = i + 1; // No 0 needed or wanted
    }
    # if defined(P104_USE_TOOLTIPS) || defined(P104_ADD_SETTINGS_NOTES)
    String zonetip;

    if (zonetip.reserve(90)) {
      zonetip  = F("Select between 1 and ");
      zonetip += P104_MAX_ZONES;
      zonetip += F(" zones, changing");
      #  ifdef P104_USE_ZONE_ORDERING
      zonetip += F(" Zones or Zone order");
      #  endif // ifdef P104_USE_ZONE_ORDERING
      zonetip += F(" will save and reload the page.");
    }
    # endif    // if defined(P104_USE_TOOLTIPS) || defined(P104_ADD_SETTINGS_NOTES)
    addFormSelector(F("Zones"), F("plugin_104_zonecount"), P104_MAX_ZONES, zonesList, zonesOptions, nullptr, P104_CONFIG_ZONE_COUNT, true
                    # ifdef P104_USE_TOOLTIPS
                    , zonetip
                    # endif // ifdef P104_USE_TOOLTIPS
                    );

    # ifdef P104_USE_ZONE_ORDERING
    const String orderTypes[] = {
      F("Numeric order (1..n)"),
      F("Display order (n..1)")
    };
    const int    orderOptions[] = { 0, 1 };
    addFormSelector(F("Zone order"), F("plugin_104_zoneorder"), 2, orderTypes, orderOptions, nullptr,
                    bitRead(P104_CONFIG_FLAGS, P104_CONFIG_FLAG_ZONE_ORDER) ? 1 : 0, true
                    #  ifdef P104_USE_TOOLTIPS
                    , zonetip
                    #  endif // ifdef P104_USE_TOOLTIPS
                    );
    # endif                  // ifdef P104_USE_ZONE_ORDERING
    # ifdef P104_ADD_SETTINGS_NOTES
    addFormNote(zonetip);
    # endif // ifdef P104_ADD_SETTINGS_NOTES
  }
  expectedZones = P104_CONFIG_ZONE_COUNT;

  if (expectedZones == 0) { expectedZones++; } // Minimum of 1 zone

  { // Optionlists and zones table
    const __FlashStringHelper *alignmentTypes[3] = {
      F("Left"),
      F("Center"),
      F("Right")
    };
    const int alignmentOptions[3] = {
      static_cast<int>(textPosition_t::PA_LEFT),
      static_cast<int>(textPosition_t::PA_CENTER),
      static_cast<int>(textPosition_t::PA_RIGHT)
    };
    int animationCount = 6;
    # if ENA_SPRITE
    animationCount += 1;
    # endif // ENA_SPRITE
    # if ENA_MISC
    animationCount += 6;
    # endif // ENA_MISC
    # if ENA_WIPE
    animationCount += 2;
    # endif // ENA_WIPE
    # if ENA_SCAN
    animationCount += 4;
    # endif // ENA_SCAN
    # if ENA_OPNCLS
    animationCount += 4;
    # endif // ENA_OPNCLS
    # if ENA_SCR_DIA
    animationCount += 4;
    # endif // ENA_SCR_DIA
    # if ENA_GROW
    animationCount += 2;
    # endif // ENA_GROW
    String animationTypes[] {
      F("None")
      , F("Print")
      , F("Scroll up")
      , F("Scroll down")
      , F("Scroll left *")
      , F("Scroll right *")
    # if ENA_SPRITE
      , F("Sprite")
    # endif // ENA_SPRITE
    # if ENA_MISC
      , F("Slice *")
      , F("Mesh")
      , F("Fade")
      , F("Dissolve")
      , F("Blinds")
      , F("Random")
    # endif // ENA_MISC
    # if ENA_WIPE
      , F("Wipe")
      , F("Wipe w. cursor")
    # endif // ENA_WIPE
    # if ENA_SCAN
      , F("Scan horiz.")
      , F("Scan horiz. cursor")
      , F("Scan vert.")
      , F("Scan vert. cursor")
    # endif // ENA_SCAN
    # if ENA_OPNCLS
      , F("Opening")
      , F("Opening w. cursor")
      , F("Closing")
      , F("Closing w. cursor")
    # endif // ENA_OPNCLS
    # if ENA_SCR_DIA
      , F("Scroll up left *")
      , F("Scroll up right *")
      , F("Scroll down left *")
      , F("Scroll down right *")
    # endif // ENA_SCR_DIA
    # if ENA_GROW
      , F("Grow up")
      , F("Grow down")
    # endif // ENA_GROW
    };
    const int animationOptions[] = {
      static_cast<int>(textEffect_t::PA_NO_EFFECT)
      , static_cast<int>(textEffect_t::PA_PRINT)
      , static_cast<int>(textEffect_t::PA_SCROLL_UP)
      , static_cast<int>(textEffect_t::PA_SCROLL_DOWN)
      , static_cast<int>(textEffect_t::PA_SCROLL_LEFT)
      , static_cast<int>(textEffect_t::PA_SCROLL_RIGHT)
    # if ENA_SPRITE
      , static_cast<int>(textEffect_t::PA_SPRITE)
    # endif // ENA_SPRITE
    # if ENA_MISC
      , static_cast<int>(textEffect_t::PA_SLICE)
      , static_cast<int>(textEffect_t::PA_MESH)
      , static_cast<int>(textEffect_t::PA_FADE)
      , static_cast<int>(textEffect_t::PA_DISSOLVE)
      , static_cast<int>(textEffect_t::PA_BLINDS)
      , static_cast<int>(textEffect_t::PA_RANDOM)
    # endif // ENA_MISC
    # if ENA_WIPE
      , static_cast<int>(textEffect_t::PA_WIPE)
      , static_cast<int>(textEffect_t::PA_WIPE_CURSOR)
    # endif // ENA_WIPE
    # if ENA_SCAN
      , static_cast<int>(textEffect_t::PA_SCAN_HORIZ)
      , static_cast<int>(textEffect_t::PA_SCAN_HORIZX)
      , static_cast<int>(textEffect_t::PA_SCAN_VERT)
      , static_cast<int>(textEffect_t::PA_SCAN_VERTX)
    # endif // ENA_SCAN
    # if ENA_OPNCLS
      , static_cast<int>(textEffect_t::PA_OPENING)
      , static_cast<int>(textEffect_t::PA_OPENING_CURSOR)
      , static_cast<int>(textEffect_t::PA_CLOSING)
      , static_cast<int>(textEffect_t::PA_CLOSING_CURSOR)
    # endif // ENA_OPNCLS
    # if ENA_SCR_DIA
      , static_cast<int>(textEffect_t::PA_SCROLL_UP_LEFT)
      , static_cast<int>(textEffect_t::PA_SCROLL_UP_RIGHT)
      , static_cast<int>(textEffect_t::PA_SCROLL_DOWN_LEFT)
      , static_cast<int>(textEffect_t::PA_SCROLL_DOWN_RIGHT)
    # endif // ENA_SCR_DIA
    # if ENA_GROW
      , static_cast<int>(textEffect_t::PA_GROW_UP)
      , static_cast<int>(textEffect_t::PA_GROW_DOWN)
    # endif // ENA_GROW
    };

    // Append the numeric value as a reference for the 'anim.in' and 'anim.out' subcommands
    for (uint8_t a = 0; a < animationCount; a++) {
      animationTypes[a] += F(" (");
      animationTypes[a] += animationOptions[a];
      animationTypes[a] += ')';
    }
    delay(0);

    int fontCount = 1;
    # ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
    fontCount++;
    # endif // ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
    # ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
    fontCount++;
    # endif // ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
    # ifdef P104_USE_VERTICAL_FONT
    fontCount++;
    # endif // ifdef P104_USE_VERTICAL_FONT
    # ifdef P104_USE_EXT_ASCII_FONT
    fontCount++;
    # endif // ifdef P104_USE_EXT_ASCII_FONT
    # ifdef P104_USE_ARABIC_FONT
    fontCount++;
    # endif // ifdef P104_USE_ARABIC_FONT
    # ifdef P104_USE_GREEK_FONT
    fontCount++;
    # endif // ifdef P104_USE_GREEK_FONT
    # ifdef P104_USE_KATAKANA_FONT
    fontCount++;
    # endif // ifdef P104_USE_KATAKANA_FONT
    const __FlashStringHelper *fontTypes[] = {
      F("Default (0)")
    # ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
      , F("Numeric, double height (1)")
    # endif   // ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
    # ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
      , F("Full, double height (2)")
    # endif   // ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
    # ifdef P104_USE_VERTICAL_FONT
      , F("Vertical (3)")
    # endif   // ifdef P104_USE_VERTICAL_FONT
    # ifdef P104_USE_EXT_ASCII_FONT
      , F("Extended ASCII (4)")
      # endif // ifdef P104_USE_EXT_ASCII_FONT
    # ifdef P104_USE_ARABIC_FONT
      , F("Arabic (5)")
    # endif   // ifdef P104_USE_ARABIC_FONT
    # ifdef P104_USE_GREEK_FONT
      , F("Greek (6)")
    # endif   // ifdef P104_USE_GREEK_FONT
    # ifdef P104_USE_KATAKANA_FONT
      , F("Katakana (7)")
    # endif   // ifdef P104_USE_KATAKANA_FONT
    };
    const int fontOptions[] = {
      P104_DEFAULT_FONT_ID
    # ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
      , P104_DOUBLE_HEIGHT_FONT_ID
    # endif   // ifdef P104_USE_NUMERIC_DOUBLEHEIGHT_FONT
    # ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
      , P104_FULL_DOUBLEHEIGHT_FONT_ID
    # endif   // ifdef P104_USE_FULL_DOUBLEHEIGHT_FONT
    # ifdef P104_USE_VERTICAL_FONT
      , P104_VERTICAL_FONT_ID
    # endif   // ifdef P104_USE_VERTICAL_FONT
    # ifdef P104_USE_EXT_ASCII_FONT
      , P104_EXT_ASCII_FONT_ID
      # endif // ifdef P104_USE_EXT_ASCII_FONT
    # ifdef P104_USE_ARABIC_FONT
      , P104_ARABIC_FONT_ID
    # endif   // ifdef P104_USE_ARABIC_FONT
    # ifdef P104_USE_GREEK_FONT
      , P104_GREEK_FONT_ID
    # endif   // ifdef P104_USE_GREEK_FONT
    # ifdef P104_USE_KATAKANA_FONT
      , P104_KATAKANA_FONT_ID
    # endif   // ifdef P104_USE_KATAKANA_FONT
    };

    int layoutCount = 1;
    # if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
    layoutCount += 2;
    # endif // if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
    const __FlashStringHelper *layoutTypes[] = {
      F("Standard")
    # if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
      , F("Double, upper")
      , F("Double, lower")
    # endif // if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
    };
    const int layoutOptions[] = {
      P104_LAYOUT_STANDARD
    # if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
      , P104_LAYOUT_DOUBLE_UPPER
      , P104_LAYOUT_DOUBLE_LOWER
    # endif // if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) || defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
    };

    const int specialEffectCount                    = 4;
    const __FlashStringHelper *specialEffectTypes[] = {
      F("None"),
      F("Flip up/down"),
      F("Flip left/right *"),
      F("Flip u/d &amp; l/r *")
    };
    const int specialEffectOptions[] = {
      P104_SPECIAL_EFFECT_NONE,
      P104_SPECIAL_EFFECT_UP_DOWN,
      P104_SPECIAL_EFFECT_LEFT_RIGHT,
      P104_SPECIAL_EFFECT_BOTH
    };

    const __FlashStringHelper *contentTypes[] = {
      F("Text"),
      F("Text reverse"),
      F("Clock (4 mod)"),
      F("Clock sec (6 mod)"),
      F("Date (4 mod)"),
      F("Date yr (6/7 mod)"),
      F("Date/time (9/13 mod)")
      # ifdef P104_USE_BAR_GRAPH
      , F("Bar graph")
      # endif // ifdef P104_USE_BAR_GRAPH
    };
    const int contentOptions[] {
      P104_CONTENT_TEXT,
      P104_CONTENT_TEXT_REV,
      P104_CONTENT_TIME,
      P104_CONTENT_TIME_SEC,
      P104_CONTENT_DATE4,
      P104_CONTENT_DATE6,
      P104_CONTENT_DATE_TIME
      # ifdef P104_USE_BAR_GRAPH
      , P104_CONTENT_BAR_GRAPH
      # endif // ifdef P104_USE_BAR_GRAPH
    };
    const __FlashStringHelper *invertedTypes[3] = {
      F("Normal"),
      F("Inverted")
    };
    const int invertedOptions[] = {
      0,
      1
    };
    # ifdef P104_USE_ZONE_ACTIONS
    uint8_t actionCount = 0;
    const __FlashStringHelper *actionTypes[4];
    int actionOptions[4];
    actionTypes[actionCount]   = F("None");
    actionOptions[actionCount] = P104_ACTION_NONE;
    actionCount++;

    if (zones.size() < P104_MAX_ZONES) {
      actionTypes[actionCount]   = F("New above");
      actionOptions[actionCount] = P104_ACTION_ADD_ABOVE;
      actionCount++;
      actionTypes[actionCount]   = F("New below");
      actionOptions[actionCount] = P104_ACTION_ADD_BELOW;
      actionCount++;
    }
    actionTypes[actionCount]   = F("Delete");
    actionOptions[actionCount] = P104_ACTION_DELETE;
    actionCount++;
    # endif // ifdef P104_USE_ZONE_ACTIONS

    delay(0);

    addFormSubHeader(F("Zone configuration"));

    {
      html_table(EMPTY_STRING); // Sub-table
      html_table_header(F("Zone #&nbsp;"));
      html_table_header(F("Modules"));
      html_table_header(F("Text"), 180);
      html_table_header(F("Content"));
      html_table_header(F("Alignment"));
      html_table_header(F("Animation In/Out"));               // 1st and 2nd row title
      html_table_header(F("Speed/Pause"));                    // 1st and 2nd row title
      html_table_header(F("Font/Layout"));                    // 1st and 2nd row title
      html_table_header(F("Inverted/ Special&nbsp;Effects")); // 1st and 2nd row title
      html_table_header(F("Offset"));
      html_table_header(F("Brightness"));
      html_table_header(F("Repeat (sec)"));
      # ifdef P104_USE_ZONE_ACTIONS
      html_table_header(F(""),       15); // Spacer
      html_table_header(F("Action"), 45);
      # endif // ifdef P104_USE_ZONE_ACTIONS
    }

    uint16_t index;
    int16_t  startZone, endZone;
    int8_t   incrZone = 1;
    # ifdef P104_USE_ZONE_ACTIONS
    uint8_t currentRow = 0;
    # endif // ifdef P104_USE_ZONE_ACTIONS

    # ifdef P104_USE_ZONE_ORDERING

    if (bitRead(P104_CONFIG_FLAGS, P104_CONFIG_FLAG_ZONE_ORDER)) {
      startZone = zones.size() - 1;
      endZone   = -1;
      incrZone  = -1;
    } else
    # endif // ifdef P104_USE_ZONE_ORDERING
    {
      startZone = 0;
      endZone   = zones.size();
    }

    for (int8_t zone = startZone; zone != endZone; zone += incrZone) {
      if (zones[zone].zone <= expectedZones) {
        index = (zones[zone].zone - 1) * P104_OFFSET_COUNT;

        html_TR_TD(); // All columns use max. width available
        addHtml(F("&nbsp;"));
        addHtmlInt(zones[zone].zone);

        html_TD(); // Modules
        addNumericBox(getPluginCustomArgName(index + P104_OFFSET_SIZE), zones[zone].size, 1, P104_MAX_MODULES_PER_ZONE);

        html_TD(); // Text
        addTextBox(getPluginCustomArgName(index + P104_OFFSET_TEXT),
                   zones[zone].text,
                   P104_MAX_TEXT_LENGTH_PER_ZONE,
                   false,
                   false,
                   EMPTY_STRING,
                   EMPTY_STRING);

        html_TD(); // Content
        addSelector(getPluginCustomArgName(index + P104_OFFSET_CONTENT),
                    P104_CONTENT_count,
                    contentTypes,
                    contentOptions,
                    nullptr,
                    zones[zone].content,
                    false,
                    true,
                    EMPTY_STRING);

        html_TD(); // Alignment
        addSelector(getPluginCustomArgName(index + P104_OFFSET_ALIGNMENT),
                    3,
                    alignmentTypes,
                    alignmentOptions,
                    nullptr,
                    zones[zone].alignment,
                    false,
                    true,
                    EMPTY_STRING);

        {
          html_TD(); // Animation In (without None by passing the second element index)
          addSelector(getPluginCustomArgName(index + P104_OFFSET_ANIM_IN),
                      animationCount - 1,
                      &animationTypes[1],
                      &animationOptions[1],
                      nullptr,
                      zones[zone].animationIn,
                      false,
                      true,
                      F("")
                      # ifdef P104_USE_TOOLTIPS
                      , F("Animation In")
                      # endif // ifdef P104_USE_TOOLTIPS
                      );
        }

        html_TD();                   // Speed In
        addNumericBox(getPluginCustomArgName(index + P104_OFFSET_SPEED), zones[zone].speed, 0, P104_MAX_SPEED_PAUSE_VALUE
                      # ifdef P104_USE_TOOLTIPS
                      , EMPTY_STRING // classname
                      , F("Speed")   // title
                      # endif // ifdef P104_USE_TOOLTIPS
                      );

        html_TD(); // Font
        addSelector(getPluginCustomArgName(index + P104_OFFSET_FONT),
                    fontCount,
                    fontTypes,
                    fontOptions,
                    nullptr,
                    zones[zone].font,
                    false,
                    true,
                    EMPTY_STRING
                    # ifdef P104_USE_TOOLTIPS
                    , F("Font") // title
                    # endif // ifdef P104_USE_TOOLTIPS
                    );

        html_TD(); // Inverted
        addSelector(getPluginCustomArgName(index + P104_OFFSET_INVERTED),
                    2,
                    invertedTypes,
                    invertedOptions,
                    nullptr,
                    zones[zone].inverted,
                    false,
                    true,
                    EMPTY_STRING
                    # ifdef P104_USE_TOOLTIPS
                    , F("Inverted") // title
                    # endif // ifdef P104_USE_TOOLTIPS
                    );

        html_TD(3); // Fill columns
        # ifdef P104_USE_ZONE_ACTIONS

        html_TD();  // Spacer
        addHtml('|');

        if (currentRow < 2) {
          addHtml(F("<TD style=\"text-align:center;font-size:90%\">")); // Action column, text centered and font-size 90%
        } else {
          html_TD();
        }

        if (currentRow == 0) {
          addHtml(F("(applied immediately!)"));
        } else if (currentRow == 1) {
          addHtml(F("(Delete can't be undone!)"));
        }
        currentRow++;
        # endif // ifdef P104_USE_ZONE_ACTIONS

        // Split here
        html_TR_TD(); // Start new row
        html_TD(4);  // Start with some blank columns

        {
          html_TD(); // Animation Out
          addSelector(getPluginCustomArgName(index + P104_OFFSET_ANIM_OUT),
                      animationCount,
                      animationTypes,
                      animationOptions,
                      nullptr,
                      zones[zone].animationOut,
                      false,
                      true,
                      EMPTY_STRING
                      # ifdef P104_USE_TOOLTIPS
                      , F("Animation Out")
                      # endif // ifdef P104_USE_TOOLTIPS
                      );
        }

        html_TD();                   // Pause after Animation In
        addNumericBox(getPluginCustomArgName(index + P104_OFFSET_PAUSE), zones[zone].pause, 0, P104_MAX_SPEED_PAUSE_VALUE
                      # ifdef P104_USE_TOOLTIPS
                      , EMPTY_STRING // classname
                      , F("Pause")   // title
                      # endif // ifdef P104_USE_TOOLTIPS
                      );

        html_TD(); // Layout
        addSelector(getPluginCustomArgName(index + P104_OFFSET_LAYOUT),
                    layoutCount,
                    layoutTypes,
                    layoutOptions,
                    nullptr,
                    zones[zone].layout,
                    false,
                    true,
                    EMPTY_STRING
                    # ifdef P104_USE_TOOLTIPS
                    , F("Layout") // title
                    # endif // ifdef P104_USE_TOOLTIPS
                    );

        html_TD(); // Special effects
        addSelector(getPluginCustomArgName(index + P104_OFFSET_SPEC_EFFECT),
                    specialEffectCount,
                    specialEffectTypes,
                    specialEffectOptions,
                    nullptr,
                    zones[zone].specialEffect,
                    false,
                    true,
                    EMPTY_STRING
                    # ifdef P104_USE_TOOLTIPS
                    , F("Special Effects") // title
                    # endif // ifdef P104_USE_TOOLTIPS
                    );

        html_TD(); // Offset
        addNumericBox(getPluginCustomArgName(index + P104_OFFSET_OFFSET), zones[zone].offset, 0, 254);

        html_TD(); // Brightness

        if (zones[zone].brightness == -1) { zones[zone].brightness = P104_BRIGHTNESS_DEFAULT; }
        addNumericBox(getPluginCustomArgName(index + P104_OFFSET_BRIGHTNESS), zones[zone].brightness, 0, P104_BRIGHTNESS_MAX);

        html_TD(); // Repeat (sec)
        addNumericBox(getPluginCustomArgName(index + P104_OFFSET_REPEATDELAY),
                      zones[zone].repeatDelay,
                      -1,
                      P104_MAX_REPEATDELAY_VALUE                     // max delay 86400 sec. = 24 hours
                      # ifdef P104_USE_TOOLTIPS
                      , EMPTY_STRING                                 // classname
                      , F("Repeat after this delay (sec), -1 = off") // tooltip
                      # endif // ifdef P104_USE_TOOLTIPS
                      );

        # ifdef P104_USE_ZONE_ACTIONS
        html_TD(); // Spacer
        addHtml('|');

        html_TD(); // Action
        addSelector(getPluginCustomArgName(index + P104_OFFSET_ACTION),
                    actionCount,
                    actionTypes,
                    actionOptions,
                    nullptr,
                    P104_ACTION_NONE, // Always start with None
                    true,
                    true,
                    EMPTY_STRING);
        # endif // ifdef P104_USE_ZONE_ACTIONS

        delay(0);
      }
    }
    html_end_table();
  }

  # ifdef P104_ADD_SETTINGS_NOTES
  String devicesMsg;

  if (devicesMsg.reserve(80)) {
    devicesMsg  = F("- Maximum nr. of modules possible (Zones * Size + Offset) = 255. Last saved: ");
    devicesMsg += numDevices;
    addFormNote(devicesMsg);
  }
  addFormNote(F("- 'Animation In' or 'Animation Out' and 'Special Effects' marked with <b>*</b> should <b>not</b> be combined in a Zone."));
  #  if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) && !defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
  addFormNote(F("- 'Layout' 'Double upper' and 'Double lower' are only supported for numeric 'Content' types like 'Clock' and 'Date'."));
  #  endif // if defined(P104_USE_NUMERIC_DOUBLEHEIGHT_FONT) && !defined(P104_USE_FULL_DOUBLEHEIGHT_FONT)
  # endif    // ifdef P104_ADD_SETTINGS_NOTES

  return true;
}

/**************************************************************
* webform_save
**************************************************************/
bool P104_data_struct::webform_save(struct EventStruct *event) {
  P104_CONFIG_ZONE_COUNT   = getFormItemInt(F("plugin_104_zonecount"));
  P104_CONFIG_HARDWARETYPE = getFormItemInt(F("plugin_104_hardware"));

  bitWrite(P104_CONFIG_FLAGS, P104_CONFIG_FLAG_CLEAR_DISABLE, isFormItemChecked(F("plugin_104_cleardisable")));
  bitWrite(P104_CONFIG_FLAGS, P104_CONFIG_FLAG_LOG_ALL_TEXT,  isFormItemChecked(F("plugin_104_logalltext")));

  # ifdef P104_USE_ZONE_ORDERING
  zoneOrder = getFormItemInt(F("plugin_104_zoneorder")); // Is used in saveSettings()
  bitWrite(P104_CONFIG_FLAGS, P104_CONFIG_FLAG_ZONE_ORDER, zoneOrder == 1);
  # endif // ifdef P104_USE_ZONE_ORDERING

  # ifdef P104_USE_DATETIME_OPTIONS
  uint32_t ulDateTime = 0;
  bitWrite(ulDateTime, P104_CONFIG_DATETIME_FLASH,    !isFormItemChecked(F("plugin_104_clockflash"))); // Inverted flag
  bitWrite(ulDateTime, P104_CONFIG_DATETIME_12H,      isFormItemChecked(F("plugin_104_clock12h")));
  bitWrite(ulDateTime, P104_CONFIG_DATETIME_AMPM,     isFormItemChecked(F("plugin_104_clockampm")));
  bitWrite(ulDateTime, P104_CONFIG_DATETIME_YEAR4DGT, isFormItemChecked(F("plugin_104_year4dgt")));
  set4BitToUL(ulDateTime, P104_CONFIG_DATETIME_FORMAT,   getFormItemInt(F("plugin_104_dateformat")));
  set4BitToUL(ulDateTime, P104_CONFIG_DATETIME_SEP_CHAR, getFormItemInt(F("plugin_104_dateseparator")));
  P104_CONFIG_DATETIME = ulDateTime;
  # endif // ifdef P104_USE_DATETIME_OPTIONS

  previousZones = expectedZones;
  expectedZones = P104_CONFIG_ZONE_COUNT;

  bool result = saveSettings();         // Determines numDevices and re-fills zones list

  P104_CONFIG_ZONE_COUNT  = zones.size();
  P104_CONFIG_TOTAL_UNITS = numDevices; // Store counted number of devices

  zones.clear();                        // Free some memory (temporarily)

  return result;
}

#endif // ifdef USES_P104

#include "../PluginStructs/_StatsOnly_data_struct.h"


#if FEATURE_PLUGIN_STATS

_StatsOnly_data_struct::_StatsOnly_data_struct()
{

}

#endif // if FEATURE_PLUGIN_STATS
#include "../PluginStructs/P077_data_struct.h"

#ifdef USES_P077

bool P077_data_struct::processCseReceived(struct EventStruct *event) {
  uint8_t header = serial_in_buffer[0];

  if ((header & 0xFC) == 0xFC) {
    //  Abnormal hardware
    return false;
  }

  // Get chip calibration data (coefficients) and use as initial defaults
  if (HLW_UREF_PULSE == PCONFIG(0)) {
    long voltage_coefficient = 191200; // uSec

    if (CSE_NOT_CALIBRATED != header) {
      voltage_coefficient = serial_in_buffer[2] << 16 |
                            serial_in_buffer[3] << 8 |
                            serial_in_buffer[4];
    }
    PCONFIG(0) = voltage_coefficient / CSE_UREF;
  }

  if (HLW_IREF_PULSE == PCONFIG(1)) {
    long current_coefficient = 16140; // uSec

    if (CSE_NOT_CALIBRATED != header) {
      current_coefficient = serial_in_buffer[8] << 16 |
                            serial_in_buffer[9] << 8 |
                            serial_in_buffer[10];
    }
    PCONFIG(1) = current_coefficient;
  }

  if (HLW_PREF_PULSE == PCONFIG(2)) {
    long power_coefficient = 5364000; // uSec

    if (CSE_NOT_CALIBRATED != header) {
      power_coefficient = serial_in_buffer[14] << 16 |
                          serial_in_buffer[15] << 8 |
                          serial_in_buffer[16];
    }
    PCONFIG(2) = power_coefficient / CSE_PREF;
  }

  adjustment    = serial_in_buffer[20];
  voltage_cycle = serial_in_buffer[5] << 16 |
                  serial_in_buffer[6] << 8 |
                  serial_in_buffer[7];
  current_cycle = serial_in_buffer[11] << 16 |
                  serial_in_buffer[12] << 8 |
                  serial_in_buffer[13];
  power_cycle = serial_in_buffer[17] << 16 |
                serial_in_buffer[18] << 8 |
                serial_in_buffer[19];
  cf_pulses = serial_in_buffer[21] << 8 |
              serial_in_buffer[22];

  //  if (energy_power_on) {  // Powered on

  if (adjustment & 0x40) { // Voltage valid
    energy_voltage = static_cast<float>(PCONFIG(0) * CSE_UREF) / static_cast<float>(voltage_cycle);
  }

  if (adjustment & 0x10) {         // Power valid
    if ((header & 0xF2) == 0xF2) { // Power cycle exceeds range
      energy_power = 0;
    } else {
      if (0 == power_cycle_first) {
        power_cycle_first = power_cycle; // Skip first incomplete power_cycle
      }

      if (power_cycle_first != power_cycle) {
        power_cycle_first = -1;
        energy_power      = static_cast<float>(PCONFIG(2) * CSE_PREF) / static_cast<float>(power_cycle);
      } else {
        energy_power = 0;
      }
    }
  } else {
    power_cycle_first = 0;
    energy_power      = 0; // Powered on but no load
  }

  if (adjustment & 0x20) { // Current valid
    if (0 == energy_power) {
      energy_current = 0;
    } else {
      energy_current = static_cast<float>(PCONFIG(1)) / static_cast<float>(current_cycle);
    }
  }

  // } else {  // Powered off
  //    power_cycle_first = 0;
  //    energy_voltage = 0;
  //    energy_power = 0;
  //    energy_current = 0;
  //  }


  return true;
}

bool P077_data_struct::processSerialData() {
  long t_start = millis();
  bool found   = false;

  while (Serial.available() > 0 && !found) {
    uint8_t serial_in_byte = Serial.read();
    count_bytes++;
    checksum -= serial_in_buffer[2];             // substract from checksum data to be removed
    memmove(serial_in_buffer, serial_in_buffer + 1,
            sizeof(serial_in_buffer) - 1);       // scroll buffer
    serial_in_buffer[25] = serial_in_byte;       // add new data
    checksum            += serial_in_buffer[22]; // add online checksum

    if ((checksum == serial_in_buffer[23]) &&
        (serial_in_buffer[1] == 0x5A)) {
      count_pkt++;
      found = true;
    }
  }
  long t_diff = timePassedSince(t_start);

  t_all += t_diff;

  if (count_pkt > 10) { // bypass first 10 pkts
    t_max = max(t_max, t_diff);
  }

  if (found) {
    count_max = max(count_max, count_bytes);
    t_pkt     = t_start - t_pkt_tmp;
    t_pkt_tmp = t_start;
  }

  return found;
}

#endif // ifdef USES_P077

#include "../PluginStructs/P114_data_struct.h"

#ifdef USES_P114

// **************************************************************************/
// Constructor
// **************************************************************************/
P114_data_struct::P114_data_struct(uint8_t i2c_addr, uint8_t integration_time, bool highDensity)
  : i2cAddress(i2c_addr), IT(integration_time), HD(highDensity) {}


// **************************************************************************/
// Initialize sensor and read data from VEML6075
// **************************************************************************/
bool P114_data_struct::read_sensor(float& _UVA, float& _UVB, float& _UVIndex) {
  if (!initialised) {
    initialised = init_sensor(); // Check id device is present
  }


  #ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log;
    log.reserve(40);
    log  = F("VEML6075: i2caddress: 0x");
    log += String(i2cAddress, HEX);
    addLogMove(LOG_LEVEL_DEBUG, log);
    log  = F("VEML6075: initialized: ");
    log += String(initialised ? F("true") : F("false"));
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
  #endif

  if (initialised) {
    for (int j = 0; j < 5; j++) {
      UVData[j] = I2C_read16_LE_reg(i2cAddress, VEML6075_UVA_DATA + j);
    }

    // Calculate the UV Index, valid in open air not behind glass!
    UVAComp  = (UVData[0] - UVData[1]) - ACoef * (UVData[3] - UVData[1]) - BCoef * (UVData[4] - UVData[1]);
    UVBComp  = (UVData[2] - UVData[1]) - CCoef * (UVData[3] - UVData[1]) - DCoef * (UVData[4] - UVData[1]);
    _UVIndex = ((UVBComp * UVBresponsivity) +  (UVAComp * UVAresponsivity)) / 2.0f;

    _UVA = static_cast<float>(UVData[0]) / static_cast<float>(1 << (IT - 1)); // UVA light sensitivity increases linear with integration time
    _UVB = static_cast<float>(UVData[2]) / static_cast<float>(1 << (IT - 1)); // UVB light sensitivity increases linear with integration time

    // float UVASensitivity = 0.93/(static_cast<float>(IT + 1)); // UVA light sensitivity increases with integration time
    // float UVBSensitivity = 2.10/(static_cast<float>(IT + 1)); // UVB light sensitivity increases with integration time
    #ifndef BUILD_NO_DEBUG
    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log  = F("VEML6075: IT raw: 0x");
      log += String(IT + 1, HEX);
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
    #endif
    return true;
  }
  return false;
}

// **************************************************************************/
// Check VEML6075 presence and initialize
// **************************************************************************/
bool P114_data_struct::init_sensor() {
  uint16_t deviceID = I2C_readS16_LE_reg(i2cAddress, VEML6075_UV_ID);

  #ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log;
    log.reserve(60);
    log  = F("VEML6075: ID: 0x");
    log += String(deviceID, HEX);
    log += F(" / checked Address: 0x");
    log += String(i2cAddress, HEX);
    log += F(" / 0x");
    log += String(VEML6075_UV_ID, HEX);
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
  #endif

  if (deviceID != 0x26) {
    if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
      String log;
      log.reserve(60);
      log  = F("VEML6075: wrong deviceID: ");
      log += String(deviceID, HEX);
      addLogMove(LOG_LEVEL_ERROR, log);
    }
    return false;
  } else {

    // log  = F("VEML6075: found deviceID: 0x");
    // log += String(deviceID, HEX);

    if (!I2C_write16_LE_reg(i2cAddress, VEML6075_UV_CONF, (IT << 4) | (HD << 3))) { // Bit 3 must be 0, bit 0 is 0 for run and 1 for
      // shutdown, LS Byte
      if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
        String log;
        log.reserve(60);
        log  = F("VEML6075: setup failed!!");
        log += F(" / CONF: ");
        log += String(static_cast<uint16_t>(IT << 4) | (HD << 3), BIN);
        addLogMove(LOG_LEVEL_ERROR, log);
      }
      return false;
    } else if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      log  = F("VEML6075: sensor initialised / CONF: ");
      log += String((uint16_t)(IT << 4) | (HD << 3), BIN);
      addLogMove(LOG_LEVEL_INFO, log);
    }
  }
  return true;
}

#endif // ifdef USES_P114

#include "../PluginStructs/P096_data_struct.h"

#ifdef USES_P096

# include "../Helpers/Hardware.h"

/****************************************************************************
 * EPD_type_toString: Display-value for the device selected
 ***************************************************************************/
const __FlashStringHelper* EPD_type_toString(EPD_type_e device) {
  switch (device) {
    case EPD_type_e::EPD_IL3897: return F("IL3897 (Lolin 250 x 122px)");
    case EPD_type_e::EPD_UC8151D: return F("UC8151D (212 x 104px)");
    case EPD_type_e::EPD_SSD1680: return F("SSD1680 (250 x 212px)");
    # if P096_USE_WAVESHARE_2IN7
    case EPD_type_e::EPD_WS2IN7: return F("Waveshare 2.7\" (264 x 176px)");
    # endif // if P096_USE_WAVESHARE_2IN7
    case EPD_type_e::EPD_MAX: break;
  }
  return F("Unsupported type!");
}

/****************************************************************************
 * P096_CommandTrigger_toString: return the command string selected
 ***************************************************************************/
const __FlashStringHelper* P096_CommandTrigger_toString(P096_CommandTrigger cmd) {
  switch (cmd) {
    case P096_CommandTrigger::eInk: return F("eink");
    case P096_CommandTrigger::ePaper: return F("epaper");
    case P096_CommandTrigger::il3897: return F("il3897");
    case P096_CommandTrigger::uc8151d: return F("uc8151d");
    case P096_CommandTrigger::ssd1680: return F("ssd1680");
    # if P096_USE_WAVESHARE_2IN7
    case P096_CommandTrigger::ws2in7: return F("ws2in7");
    # endif // if P096_USE_WAVESHARE_2IN7
    case P096_CommandTrigger::MAX: return F("None");
    case P096_CommandTrigger::epd: break;
  }
  return F("epd"); // Default command trigger
}

/****************************************************************************
 * EPD_type_toResolution: X and Y resolution for the selected type
 ***************************************************************************/
void EPD_type_toResolution(EPD_type_e device, uint16_t& x, uint16_t& y) {
  switch (device) {
    case EPD_type_e::EPD_IL3897:
    case EPD_type_e::EPD_SSD1680:
      x = 250;
      y = 122;
      break;
    case EPD_type_e::EPD_UC8151D:
      x = 212;
      y = 104;
      break;
    # if P096_USE_WAVESHARE_2IN7
    case EPD_type_e::EPD_WS2IN7:
      x = 264;
      y = 176;
      break;
    # endif // if P096_USE_WAVESHARE_2IN7
    case EPD_type_e::EPD_MAX:
      break;
  }
}

/****************************************************************************
 * Constructor
 ***************************************************************************/
P096_data_struct::P096_data_struct(EPD_type_e          display,
                                   # if !P096_USE_EXTENDED_SETTINGS
                                   uint16_t            width,
                                   uint16_t            height,
                                   # endif // if P096_USE_EXTENDED_SETTINGS
                                   uint8_t             rotation,
                                   uint8_t             fontscaling,
                                   AdaGFXTextPrintMode textmode,
                                   String              commandTrigger,
                                   uint16_t            fgcolor,
                                   uint16_t            bgcolor,
                                   AdaGFXColorDepth    colorDepth,
                                   bool                textBackFill)
  : _display(display),
  # if !P096_USE_EXTENDED_SETTINGS
  _xpix(width), _ypix(height),
  # endif // if !P096_USE_EXTENDED_SETTINGS
  _rotation(rotation), _fontscaling(fontscaling), _textmode(textmode), _commandTrigger(commandTrigger),
  _fgcolor(fgcolor), _bgcolor(bgcolor), _colorDepth(colorDepth), _textBackFill(textBackFill)
{
  # if P096_USE_EXTENDED_SETTINGS

  EPD_type_toResolution(_display, _xpix, _ypix);
  # endif // if P096_USE_EXTENDED_SETTINGS

  updateFontMetrics();
  _commandTrigger.toLowerCase();
  _commandTriggerCmd  = _commandTrigger;
  _commandTriggerCmd += F("cmd");
}

/****************************************************************************
 * Destructor
 ***************************************************************************/
P096_data_struct::~P096_data_struct() {
  if (nullptr != gfxHelper) {
    delete gfxHelper;
    gfxHelper = nullptr;
  }

  if (nullptr != eInkScreen) {
    delete eInkScreen;
    eInkScreen = nullptr;
  }
}

/****************************************************************************
 * plugin_init: Initialize display
 ***************************************************************************/
bool P096_data_struct::plugin_init(struct EventStruct *event) {
  bool success = false;

  if (nullptr == eInkScreen) {
    addLog(LOG_LEVEL_INFO, F("EPD  : Init start."));

    switch (_display) {
      case EPD_type_e::EPD_IL3897:
        eInkScreen = new (std::nothrow) LOLIN_IL3897(_xpix, _ypix, PIN(1), PIN(2), PIN(0), PIN(3));  // HSPI
        break;
      case EPD_type_e::EPD_UC8151D:
        eInkScreen = new (std::nothrow) LOLIN_UC8151D(_xpix, _ypix, PIN(1), PIN(2), PIN(0), PIN(3)); // HSPI
        break;
      case EPD_type_e::EPD_SSD1680:
        eInkScreen = new (std::nothrow) LOLIN_SSD1680(_xpix, _ypix, PIN(1), PIN(2), PIN(0), PIN(3)); // HSPI
        break;
      # if P096_USE_WAVESHARE_2IN7
      case EPD_type_e::EPD_WS2IN7:
        eInkScreen = new (std::nothrow) Waveshare_2in7(_xpix, _ypix, PIN(1), PIN(2), PIN(0), PIN(3)); // HSPI
        break;
      # endif // if P096_USE_WAVESHARE_2IN7
      case EPD_type_e::EPD_MAX:
        break;
    }
    plugin_096_sequence_in_progress = false;
    # ifdef P096_USE_ADA_GRAPHICS

    if (nullptr != eInkScreen) {
      gfxHelper = new (std::nothrow) AdafruitGFX_helper(eInkScreen,
                                                        _commandTrigger,
                                                        _xpix,
                                                        _ypix,
                                                        _colorDepth,
                                                        _textmode,
                                                        _fontscaling,
                                                        _fgcolor,
                                                        _bgcolor,
                                                        true,
                                                        _textBackFill);
      #  if P096_USE_EXTENDED_SETTINGS

      if (nullptr != gfxHelper) {
        gfxHelper->setRotation(_rotation);
        gfxHelper->setColumnRowMode(bitRead(P096_CONFIG_FLAGS, P096_CONFIG_FLAG_USE_COL_ROW));
        gfxHelper->setTxtfullCompensation(!bitRead(P096_CONFIG_FLAGS, P096_CONFIG_FLAG_COMPAT_P096) ? 0 : 1); // Inverted
      }
      #  endif // if P096_USE_EXTENDED_SETTINGS
    }
    updateFontMetrics();
    # endif // ifdef P096_USE_ADA_GRAPHICS

    # ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      log.reserve(50);
      log += F("EPD  : Init done, address: 0x");
      log += String(reinterpret_cast<ulong>(eInkScreen), HEX);
      log += ' ';

      if (nullptr == eInkScreen) {
        log += F("in");
      }
      log += F("valid, commands: ");
      log += _commandTrigger;
      log += F(", display: ");
      log += EPD_type_toString(_display);
      addLog(LOG_LEVEL_INFO, log);
      log.clear();
      log += F("EPD  : Foreground: 0x");
      log += String(_fgcolor, HEX);
      log += F(", background: 0x");
      log += String(_bgcolor, HEX);
      addLogMove(LOG_LEVEL_INFO, log);
    }
    # endif // ifndef BUILD_NO_DEBUG

    if (nullptr != eInkScreen) {
      eInkScreen->begin(); // Start the device
      eInkScreen->clearBuffer();

      eInkScreen->setRotation(_rotation);
      eInkScreen->setTextColor(_fgcolor);
      eInkScreen->setTextSize(_fontscaling); // Handles 0 properly, text size, default 1 = very small
      eInkScreen->setCursor(0, 0);           // move cursor to position (0, 0) pixel
    }

    success = true;
  } else {
    addLog(LOG_LEVEL_INFO, F("EPD  : No init?"));
  }

  return success;
}

/****************************************************************************
 * updateFontMetrics: recalculate x and y columns, based on font size and font scale
 ***************************************************************************/
void P096_data_struct::updateFontMetrics() {
  if (nullptr != gfxHelper) {
    gfxHelper->getTextMetrics(_textcols, _textrows, _fontwidth, _fontheight, _fontscaling, _heightOffset, _xpix, _ypix);
    gfxHelper->getColors(_fgcolor, _bgcolor);
  } else {
    if (_fontscaling == 0) { _fontscaling = 1; }
    _textcols = _xpix / (_fontwidth * _fontscaling);
    _textrows = _ypix / (_fontheight * _fontscaling);
  }
}

/****************************************************************************
 * plugin_exit: De-initialize before destruction
 ***************************************************************************/
bool P096_data_struct::plugin_exit(struct EventStruct *event) {
  addLog(LOG_LEVEL_INFO, F("EPD  : Exit."));

  # if P096_USE_EXTENDED_SETTINGS

  if (nullptr != gfxHelper) { delete gfxHelper; }
  gfxHelper = nullptr;

  # endif // if P096_USE_EXTENDED_SETTINGS

  if (nullptr != eInkScreen) { delete eInkScreen; }
  eInkScreen = nullptr; // Is used as a proxy only
  return true;
}

/****************************************************************************
 * plugin_read: Re-draw the default content
 ***************************************************************************/
bool P096_data_struct::plugin_read(struct EventStruct *event) {
  # if P096_USE_EXTENDED_SETTINGS

  if (nullptr != eInkScreen) {
    String strings[P096_Nlines];
    LoadCustomTaskSettings(event->TaskIndex, strings, P096_Nlines, 0);

    bool hasContent = false;

    for (uint8_t x = 0; x < P096_Nlines && !hasContent; x++) {
      hasContent = !strings[x].isEmpty();
    }

    if (hasContent) {
      gfxHelper->setColumnRowMode(false); // Turn off column mode

      eInkScreen->clearBuffer();

      int yPos = 0;

      for (uint8_t x = 0; x < P096_Nlines; x++) {
        String newString = AdaGFXparseTemplate(strings[x], _textcols, gfxHelper);

        #  if ADAGFX_PARSE_SUBCOMMAND
        updateFontMetrics();
        #  endif // if ADAGFX_PARSE_SUBCOMMAND

        if (yPos < _ypix) {
          gfxHelper->printText(newString.c_str(), 0, yPos, _fontscaling, _fgcolor, _bgcolor);
        }
        delay(0);
        yPos += (_fontheight * _fontscaling);
      }
      gfxHelper->setColumnRowMode(bitRead(P096_CONFIG_FLAGS, P096_CONFIG_FLAG_USE_COL_ROW)); // Restore column mode
      int16_t curX, curY;
      gfxHelper->getCursorXY(curX, curY);                                                    // Get current X and Y coordinates,
      UserVar[event->BaseVarIndex]     = curX;                                               // and put into Values
      UserVar[event->BaseVarIndex + 1] = curY;

      eInkScreen->display();
      eInkScreen->clearBuffer();
    }
  }
  # endif // if P096_USE_EXTENDED_SETTINGS
  return false; // Always return false, so no attempt to send to Controllers or generate events is started
}

/****************************************************************************
 * plugin_write: Handle commands
 ***************************************************************************/
bool P096_data_struct::plugin_write(struct EventStruct *event, const String& string) {
  bool   success = false;
  String cmd     = parseString(string, 1);

  if ((nullptr != eInkScreen) && cmd.equals(_commandTriggerCmd)) {
    String arg1 = parseString(string, 2);

    if (arg1.equals(F("off"))) { // Not supported 'on' and 'off' as commands
      success = false;
    }
    else if (arg1.equals(F("on"))) {
      success = false;
    }
    else if (arg1.equals(F("clear"))) {
      String arg2 = parseString(string, 3);

      eInkScreen->clearBuffer();

      if (!arg2.isEmpty()) {
        eInkScreen->fillScreen(AdaGFXparseColor(arg2, _colorDepth));
      } else {
        eInkScreen->fillScreen(_bgcolor);
      }
      eInkScreen->display();
      eInkScreen->clearBuffer();
      success = true;
    }
    else if (arg1.equals(F("backlight"))) { // not supported
      success = false;
    }
    else if (arg1.equals(F("deepsleep"))) {
      eInkScreen->deepSleep();
    }
    else if (arg1.equals(F("seq_start"))) {
      String arg2 = parseString(string, 3);

      eInkScreen->clearBuffer();
      const uint16_t fillColor =
        (arg2.isEmpty() ? static_cast<uint16_t>(AdaGFXMonoRedGreyscaleColors::ADAGFXEPD_BLACK)
        : AdaGFXparseColor(arg2, _colorDepth));
      eInkScreen->fillScreen(fillColor);
      plugin_096_sequence_in_progress = true;
      success                         = true;
    }
    else if (arg1.equals(F("seq_end"))) {
      // # ifndef BUILD_NO_DEBUG
      //             TimingStats s;
      //             const unsigned statisticsTimerStart(micros());
      // # endif // ifndef BUILD_NO_DEBUG
      eInkScreen->display();

      // # ifndef BUILD_NO_DEBUG
      //             s.add(usecPassedSince(statisticsTimerStart));
      //             tmpString += "<br/> Display timings = " + String(s.getAvg());
      // # endif // ifndef BUILD_NO_DEBUG
      eInkScreen->clearBuffer();
      plugin_096_sequence_in_progress = false;
      success                         = true;
    }
    else if (arg1.equals(F("inv"))) {
      String arg2 = parseString(string, 3);
      int    nArg2;

      if (validIntFromString(arg2, nArg2) &&
          (nArg2 >= 0) &&
          (nArg2 <= 1)) {
        eInkScreen->invertDisplay(nArg2);
        eInkScreen->display();
        success = true;
      }
    }
    else if (arg1.equals(F("rot"))) {
      ///control?cmd=epdcmd,rot,0
      // not working to verify
      String arg2 = parseString(string, 3);
      int    nArg2;

      if (validIntFromString(arg2, nArg2) &&
          (nArg2 >= 0)) {
        eInkScreen->setRotation(nArg2 % 4);
        eInkScreen->display();
        success = true;
      }
    } else {
      success = false;
    }
  }
  else if (eInkScreen && (cmd.equals(_commandTrigger) ||
                          (gfxHelper && gfxHelper->isAdaGFXTrigger(cmd)))) {
    success = true;

    // if (!bitRead(P096_CONFIG_FLAGS, P096_CONFIG_FLAG_NO_WAKE)) { // Wake display?
    //   displayOnOff(true, P096_CONFIG_BACKLIGHT_PIN, P096_CONFIG_BACKLIGHT_PERCENT, P096_CONFIG_DISPLAY_TIMEOUT);
    // }

    if (nullptr != gfxHelper) {
      String tmp = string;

      if (!plugin_096_sequence_in_progress) {
        eInkScreen->clearBuffer();
        eInkScreen->fillScreen(EPD_WHITE);
      }

      // Hand it over after replacing variables
      success = gfxHelper->processCommand(AdaGFXparseTemplate(tmp, _textcols, gfxHelper));

      if (success && !plugin_096_sequence_in_progress) {
        eInkScreen->display();

        // eInkScreen->clearBuffer();
      }

      updateFontMetrics(); // Font or color may have changed

      if (success) {
        int16_t curX, curY;
        gfxHelper->getCursorXY(curX, curY); // Get current X and Y coordinates, and put into Values
        UserVar[event->BaseVarIndex]     = curX;
        UserVar[event->BaseVarIndex + 1] = curY;
      }
    }
  }
  return success;
}

#endif // ifdef USES_P096

#include "../PluginStructs/P061_data_struct.h"

#if defined(USES_P061)

bool P061_data_struct::plugin_init(struct EventStruct *event) {
  switch (P061_CONFIG_KEYPAD_TYPE) {
    case 0: MCP23017_KeyPadMatrixInit(_i2c_addr); break;
    case 1: PCF8574_KeyPadMatrixInit(_i2c_addr); break;
    case 2: PCF8574_KeyPadDirectInit(_i2c_addr); break;
    case 3: MCP23017_KeyPadDirectInit(_i2c_addr); break;
    # ifdef P061_ENABLE_PCF8575
    case 4: PCF8575_KeyPadMatrixInit(_i2c_addr); break;
    case 5: PCF8575_KeyPadDirectInit(_i2c_addr); break;
    # endif // ifdef P061_ENABLE_PCF8575
  }

  return true;
}

bool P061_data_struct::plugin_fifty_per_second(struct EventStruct *event) {
  uint8_t actScanCode = 0;

  switch (P061_CONFIG_KEYPAD_TYPE) {
    case 0: actScanCode = MCP23017_KeyPadMatrixScan(_i2c_addr); break;
    case 1: actScanCode = PCF8574_KeyPadMatrixScan(_i2c_addr); break;
    case 2: actScanCode = PCF8574_KeyPadDirectScan(_i2c_addr); break;
    case 3: actScanCode = MCP23017_KeyPadDirectScan(_i2c_addr); break;
    # ifdef P061_ENABLE_PCF8575
    case 4: actScanCode = PCF8575_KeyPadMatrixScan(_i2c_addr); break;
    case 5: actScanCode = PCF8575_KeyPadDirectScan(_i2c_addr); break;
    # endif // ifdef P061_ENABLE_PCF8575
  }

  if (lastScanCode == actScanCode) {   // debounced? - two times the same value?
    if (sentScanCode != actScanCode) { // any change to last sent data?
      UserVar[event->BaseVarIndex] = actScanCode;
      event->sensorType            = Sensor_VType::SENSOR_TYPE_SWITCH;

      String log = F("KPad : ScanCode=0x");
      log += String(actScanCode, HEX);
      addLogMove(LOG_LEVEL_INFO, log);

      sendData(event);

      sentScanCode = actScanCode;
    }
  } else {
    lastScanCode = actScanCode;
  }

  return true;
}

void P061_data_struct::MCP23017_setReg(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t P061_data_struct::MCP23017_getReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)0x1);

  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

void P061_data_struct::MCP23017_KeyPadMatrixInit(uint8_t addr) {
  MCP23017_setReg(addr, MCP23017_IODIRA, 0x00); // port A to output
  MCP23017_setReg(addr, MCP23017_GPIOA,  0x00); // port A to low
  MCP23017_setReg(addr, MCP23017_IODIRB, 0xFF); // port B to input
  MCP23017_setReg(addr, MCP23017_GPPUA,  0xFF); // port A pullup on
  MCP23017_setReg(addr, MCP23017_GPPUB,  0xFF); // port B pullup on
}

void P061_data_struct::MCP23017_KeyPadDirectInit(uint8_t addr) {
  MCP23017_setReg(addr, MCP23017_IODIRA, 0xFF); // port A to input
  MCP23017_setReg(addr, MCP23017_IODIRB, 0xFF); // port B to input
  MCP23017_setReg(addr, MCP23017_GPPUA,  0xFF); // port A pullup on
  MCP23017_setReg(addr, MCP23017_GPPUB,  0xFF); // port B pullup on
}

uint8_t P061_data_struct::MCP23017_KeyPadMatrixScan(uint8_t addr) {
  uint8_t rowMask = 1;
  uint8_t colData;

  colData = MCP23017_getReg(addr, MCP23017_GPIOB);
  # if P061_DEBUG_LOG

  if (loglevelActiveFor(LOG_LEVEL_INFO) && (millis() % 1000 < 10)) {
    String log = F("P061 MCP23017 matrix, read data: 0x");
    log += String(colData, HEX);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // if P061_DEBUG_LOG

  if (colData == 0xFF) { // no key pressed?
    return 0;            // no key pressed!
  }

  for (uint8_t row = 0; row <= 8; row++) {
    if (row == 0) {
      MCP23017_setReg(addr, MCP23017_IODIRA, 0xFF);     // no bit of port A to output
    } else {
      MCP23017_setReg(addr, MCP23017_IODIRA, ~rowMask); // one bit of port A to output 0
      rowMask <<= 1;
    }

    colData = MCP23017_getReg(addr, MCP23017_GPIOB);

    if (colData != 0xFF) { // any key pressed?
      uint8_t colMask = 1;

      for (uint8_t col = 1; col <= 8; col++) {
        if ((colData & colMask) == 0) {                 // this key pressed?
          MCP23017_setReg(addr, MCP23017_IODIRA, 0x00); // port A to output 0
          return (row << 4) | col;
        }
        colMask <<= 1;
      }
    }
  }

  MCP23017_setReg(addr, MCP23017_IODIRA, 0x00); // port A to output 0
  return 0;                                     // no key pressed!
}

uint8_t P061_data_struct::MCP23017_KeyPadDirectScan(uint8_t addr) {
  uint16_t colData;

  colData  = (MCP23017_getReg(addr, MCP23017_GPIOB) << 8);
  colData |= MCP23017_getReg(addr, MCP23017_GPIOA);
  # if P061_DEBUG_LOG

  if (loglevelActiveFor(LOG_LEVEL_INFO) && (millis() % 1000 < 10)) {
    String log = F("P061 MCP23017 direct, read data: 0x");
    log += String(colData, HEX);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // if P061_DEBUG_LOG

  if (colData == 0xFFFF) { // no key pressed?
    return 0;              // no key pressed!
  }
  uint16_t colMask = 0x01;

  for (uint8_t col = 1; col <= 16; col++) {
    if ((colData & colMask) == 0) { // this key pressed?
      return col;
    }
    colMask <<= 1;
  }

  return 0; // no key pressed!
}

// PCF8574 Matrix //////////////////////////////////////////////////////////////

void P061_data_struct::PCF8574_setReg(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t P061_data_struct::PCF8574_getReg(uint8_t addr) {
  Wire.requestFrom(addr, (uint8_t)0x1);

  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

void P061_data_struct::PCF8574_KeyPadMatrixInit(uint8_t addr) {
  PCF8574_setReg(addr, 0xF0); // low nibble to output 0
}

uint8_t P061_data_struct::PCF8574_KeyPadMatrixScan(uint8_t addr) {
  uint8_t rowMask = 1;
  uint8_t colData;

  colData = PCF8574_getReg(addr) & 0xF0;
  # if P061_DEBUG_LOG

  if (loglevelActiveFor(LOG_LEVEL_INFO) && (millis() % 1000 < 10)) {
    String log = F("P061 PCF8574 matrix, read data: 0x");
    log += String(colData, HEX);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // if P061_DEBUG_LOG

  if (colData == 0xF0) { // no key pressed?
    return 0;            // no key pressed!
  }

  for (uint8_t row = 0; row <= 4; row++) {
    if (row == 0) {
      PCF8574_setReg(addr, 0xFF);     // no bit of port A to output
    } else {
      PCF8574_setReg(addr, ~rowMask); // one bit of port A to output 0
      rowMask <<= 1;
    }

    colData = PCF8574_getReg(addr) & 0xF0;

    if (colData != 0xF0) { // any key pressed?
      uint8_t colMask = 0x10;

      for (uint8_t col = 1; col <= 4; col++) {
        if ((colData & colMask) == 0) { // this key pressed?
          PCF8574_setReg(addr, 0xF0);   // low nibble to output 0
          return (row << 4) | col;
        }
        colMask <<= 1;
      }
    }
  }

  PCF8574_setReg(addr, 0xF0); // low nibble to output 0
  return 0;                   // no key pressed!
}

// PCF8574 Direct //////////////////////////////////////////////////////////////

void P061_data_struct::PCF8574_KeyPadDirectInit(uint8_t addr) {
  PCF8574_setReg(addr, 0xFF); // all to input
}

uint8_t P061_data_struct::PCF8574_KeyPadDirectScan(uint8_t addr) {
  uint8_t colData;

  colData = PCF8574_getReg(addr);
  # if P061_DEBUG_LOG

  if (loglevelActiveFor(LOG_LEVEL_INFO) && (millis() % 1000 < 10)) {
    String log = F("P061 PCF8574 direct, read data: 0x");
    log += String(colData, HEX);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // if P061_DEBUG_LOG

  if (colData == 0xFF) { // no key pressed?
    return 0;            // no key pressed!
  }
  uint8_t colMask = 0x01;

  for (uint8_t col = 1; col <= 8; col++) {
    if ((colData & colMask) == 0) { // this key pressed?
      return col;
    }
    colMask <<= 1;
  }

  return 0; // no key pressed!
}

# ifdef P061_ENABLE_PCF8575

// PCF8575 Matrix /////////////////////////////////////////////////////////////

void P061_data_struct::PCF8575_setReg(uint8_t addr, uint16_t data) {
  Wire.beginTransmission(addr);
  Wire.write(lowByte(data));
  Wire.write(highByte(data));
  Wire.endTransmission();
}

uint16_t P061_data_struct::PCF8575_getReg(uint8_t addr) {
  uint16_t data;

  Wire.beginTransmission(addr);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)2u);

  if (Wire.available()) {
    data  = Wire.read();        // Low byte
    data |= (Wire.read() << 8); // High byte
    return data;
  }
  return 0xFFFF;
}

void P061_data_struct::PCF8575_KeyPadMatrixInit(uint8_t addr) {
  PCF8575_setReg(addr, 0xFF00); // low byte to output 00
}

uint8_t P061_data_struct::PCF8575_KeyPadMatrixScan(uint8_t addr) {
  uint16_t rowMask = 1;
  uint16_t colData;

  PCF8575_setReg(addr, 0xFF00); // P1x all to input
  colData = PCF8575_getReg(addr) & 0xFF00;
  #  if P061_DEBUG_LOG

  if (loglevelActiveFor(LOG_LEVEL_INFO) && (millis() % 1000 < 10)) {
    String log = F("P061 PCF8575 matrix, read data: 0x");
    log += String(colData, HEX);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  #  endif // if P061_DEBUG_LOG

  if (colData == 0xFF00) { // no key pressed?
    return 0;              // no key pressed!
  }

  for (uint8_t row = 0; row <= 8; row++) {
    if (row == 0) {
      PCF8575_setReg(addr, 0xFFFF);   // no bit of port A to output
    } else {
      PCF8575_setReg(addr, ~rowMask); // one bit of port A to output 0
      rowMask <<= 1;
    }

    colData = PCF8575_getReg(addr) & 0xFF00;

    if (colData != 0xFF00) { // any key pressed?
      uint16_t colMask = 0x0100;

      for (uint8_t col = 1; col <= 8; col++) {
        if ((colData & colMask) == 0) { // this key pressed?
          PCF8575_setReg(addr, 0xFF00); // low byte to output 00
          return (row << 4) | col;
        }
        colMask <<= 1;
      }
    }
  }

  PCF8575_setReg(addr, 0xFF00); // low byte to output 00
  return 0;                     // no key pressed!
}

// PCF8575 Direct //////////////////////////////////////////////////////////////

void P061_data_struct::PCF8575_KeyPadDirectInit(uint8_t addr) {
  PCF8575_setReg(addr, 0xFFFF); // all to input
}

uint8_t P061_data_struct::PCF8575_KeyPadDirectScan(uint8_t addr) {
  uint16_t colData;

  PCF8575_setReg(addr, 0xFFFF);   // all to input
  colData = PCF8575_getReg(addr); // Read the actual state
  #  if P061_DEBUG_LOG

  if (loglevelActiveFor(LOG_LEVEL_INFO) && (millis() % 1000 < 10)) {
    String log = F("P061 PCF8575 direct, read data: 0x");
    log += String(colData, HEX);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  #  endif // if P061_DEBUG_LOG

  if (colData == 0xFFFF) { // no key pressed?
    return 0;              // no key pressed!
  }
  uint16_t colMask = 0x01;

  for (uint8_t col = 1; col <= 16; col++) {
    if ((colData & colMask) == 0) { // this key pressed?
      return col;
    }
    colMask <<= 1;
  }

  return 0; // no key pressed!
}

# endif // ifdef P061_ENABLE_PCF8575

#endif // if defined(USES_P061)

#include "../PluginStructs/P016_data_struct.h"

#ifdef USES_P016

# include "../Commands/InternalCommands.h"
# include "../Helpers/ESPEasy_Storage.h"
# include <IRutils.h>

# ifdef P16_SETTINGS_V1

// Conversion constructor
tCommandLinesV2::tCommandLinesV2(const tCommandLinesV1& lineV1, uint8_t i)
{
  String log;

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    #  ifndef PLUGIN_016_DEBUG
    log.reserve(20); // less space needed
    #  else // ifndef PLUGIN_016_DEBUG
    log.reserve(80);
    #  endif // ifndef PLUGIN_016_DEBUG
  }

  log  = F("P016: converting "); // Still a little logging
  log += i;
  #  ifdef PLUGIN_016_DEBUG
  log += ':';
  #  endif // ifdef PLUGIN_016_DEBUG

  memcpy(Command, lineV1.Command, P16_Nchars);
  const uint32_t oldCode = lineV1.Code;

  if (oldCode > 0) {
    CodeDecodeType = static_cast<decode_type_t>((oldCode >> 24));                // decode_type
    bitWrite(CodeFlags, P16_FLAGS_REPEAT, oldCode & (0x1 << P16_CMDBIT_REPEAT)); // Repeat flag
    Code = oldCode & 0x7FFFFF;                                                   // Only keep lowest 23 bits
    #  ifdef PLUGIN_016_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log += F(" type ");
      log += typeToString(CodeDecodeType, (oldCode & (0x1 << P16_CMDBIT_REPEAT)) != 0);
      log += F(" code 0x");
      log += uint64ToString(oldCode, 16);
    }
    #  endif // ifdef PLUGIN_016_DEBUG
  }

  const uint32_t oldAlternativeCode = lineV1.AlternativeCode;

  if (oldAlternativeCode > 0) {
    AlternativeCodeDecodeType = static_cast<decode_type_t>((oldAlternativeCode >> 24));                // decode_type
    bitWrite(AlternativeCodeFlags, P16_FLAGS_REPEAT, oldAlternativeCode & (0x1 << P16_CMDBIT_REPEAT)); // Repeat flag
    AlternativeCode = oldAlternativeCode & 0x7FFFFF;                                                   // Only keep lowest 23 bits
    #  ifdef PLUGIN_016_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log += F(" alt.type ");
      log += typeToString(AlternativeCodeDecodeType, (oldAlternativeCode & (0x1 << P16_CMDBIT_REPEAT)) != 0);
      log += F(" alt.code 0x");
      log += uint64ToString(oldAlternativeCode, 16);
    }
    #  endif // ifdef PLUGIN_016_DEBUG
  }
  addLogMove(LOG_LEVEL_INFO, log);
}

# endif // ifdef P16_SETTINGS_V1

P016_data_struct::P016_data_struct() {}

void P016_data_struct::init(struct EventStruct *event, uint16_t CmdInhibitTime) {
  loadCommandLines(event);
  iCmdInhibitTime = CmdInhibitTime;
  iLastCmd        = 0;
  iLastCmdTime    = 0;
}

void P016_data_struct::loadCommandLines(struct EventStruct *event) {
  # ifdef P16_SETTINGS_V1

  // Convert the settings if both versions are defined and PCONFIG(7) != latest version
  if (PCONFIG(7) != P16_SETTINGS_LATEST) {
    addLog(LOG_LEVEL_ERROR, F("P016 IR: Settings conversion, save task settings to store in new format."));
  }
  # endif // ifdef P16_SETTINGS_V1
  CommandLines.clear(); // Start fresh

  for (uint8_t i = 0; i < P16_Nlines; i++) {
    CommandLines.push_back(tCommandLinesV2());
    loadCommandLine(event, CommandLines[i], i);
  }
}

void P016_data_struct::saveCommandLines(struct EventStruct *event) {
  for (uint8_t i = 0; i < P16_Nlines; i++) {
    saveCommandLine(event, CommandLines[i], i);
  }
}

void P016_data_struct::loadCommandLine(struct EventStruct *event, tCommandLinesV2& line, uint8_t lineNr)
{
  # ifdef P16_SETTINGS_V1

  if (PCONFIG(7) != P16_SETTINGS_LATEST) {
    loadCommandLinev1(event, line, lineNr);
    return;
  }
  # endif // ifdef P16_SETTINGS_V1

  const int loadOffset = lineNr * sizeof(tCommandLinesV2);
  LoadFromFile(SettingsType::Enum::CustomTaskSettings_Type,
               event->TaskIndex,
               reinterpret_cast<uint8_t *>(&line),
               sizeof(tCommandLinesV2),
               loadOffset);
  line.Command[P16_Nchars - 1] = 0; // Terminate in case of uninitalized data
}

# ifdef P16_SETTINGS_V1
void P016_data_struct::loadCommandLinev1(struct EventStruct *event, tCommandLinesV2& line, uint8_t lineNr)
{
  tCommandLinesV1 lineV1;

  {
    const int loadOffsetV1 = lineNr * sizeof(tCommandLinesV1);
    LoadFromFile(SettingsType::Enum::CustomTaskSettings_Type,
                 event->TaskIndex,
                 reinterpret_cast<uint8_t *>(&lineV1),
                 sizeof(tCommandLinesV2),
                 loadOffsetV1);
  }
  line                         = tCommandLinesV2(lineV1, lineNr);
  line.Command[P16_Nchars - 1] = 0;
}

# endif // ifdef P16_SETTINGS_V1

void P016_data_struct::saveCommandLine(struct EventStruct *event, const tCommandLinesV2& line, uint8_t lineNr)
{
  const int saveOffset = lineNr * sizeof(tCommandLinesV2);

  SaveToFile(SettingsType::Enum::CustomTaskSettings_Type,
             event->TaskIndex,
             reinterpret_cast<const uint8_t *>(&line),
             sizeof(tCommandLinesV2),
             saveOffset);
}

void P016_data_struct::AddCode(uint64_t Code, decode_type_t DecodeType, uint16_t CodeFlags) {
  // add received code
  int _index = P16_Nlines;

  if (Code == 0) {
    return;
  }

  for (int i = 0; i < P16_Nlines; ++i) {
    if (validateCode(i, Code, DecodeType, CodeFlags)) {
      // code already saved
      return;
    }

    if (CommandLines[i].Code == 0) {
      // get first index to add the code
      _index = std::min(i, _index);
    }
  }

  if (_index == P16_Nlines) {
    // no free index
    return;
  }
  CommandLines[_index].Code           = Code;
  CommandLines[_index].CodeDecodeType = DecodeType;
  CommandLines[_index].CodeFlags      = CodeFlags;
  bCodeChanged                        = true;
  # ifdef PLUGIN_016_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log;
    if (log.reserve(80)) { // estimated
      log  = F("[P36] AddCode: ");
      log += typeToString(DecodeType, bitRead(CodeFlags, P16_FLAGS_REPEAT));
      log += F(" code: 0x");
      log += uint64ToString(Code, 16);
      log += F(" to index ");
      log += _index;
      addLogMove(LOG_LEVEL_INFO, log);
    }
  }
  # endif // PLUGIN_016_DEBUG
}

void P016_data_struct::ExecuteCode(uint64_t Code, decode_type_t DecodeType, uint16_t CodeFlags) {
  if (Code == 0) {
    return;
  }

  if ((iLastCmd == Code) && (iLastDecodeType == DecodeType)) {
    // same code as before
    if (iCmdInhibitTime > timePassedSince(iLastCmdTime)) {
      // inhibit time not ellapsed
      return;
    }
  }

  for (int i = 0; i < P16_Nlines; ++i) {
    if (validateCode(i, Code, DecodeType, CodeFlags)) {
      // code already saved
      iLastCmd        = Code;
      iLastDecodeType = DecodeType;
      iLastCodeFlags  = CodeFlags;
      iLastCmdTime    = millis();

      if (CommandLines[i].Command[0] != 0) {
        # ifdef PLUGIN_016_DEBUG
        bool _success =
        # endif // ifdef PLUGIN_016_DEBUG
        ExecuteCommand_all(EventValueSource::Enum::VALUE_SOURCE_SYSTEM, CommandLines[i].Command);
        # ifdef PLUGIN_016_DEBUG

        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log;
          if (log.reserve(128)) { // estimated
            log  = F("[P36] Execute: ");
            log += typeToString(DecodeType, bitRead(CodeFlags, P16_FLAGS_REPEAT));
            log += F(" Code: 0x");
            log += uint64ToString(Code, 16);
            log += F(" with command ");
            log += (i + 1);
            log += F(": {");
            log += String(CommandLines[i].Command);
            log += '}';

            if (!_success) {
              log += F(" FAILED!");
            }
            addLogMove(LOG_LEVEL_INFO, log);
          }
        }
        # endif // PLUGIN_016_DEBUG
      }
      return;
    }
    # ifdef PLUGIN_016_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      if (log.reserve(128)) { // estimated
        log  = F("[P36] ValidateCode failed: ");
        log += typeToString(DecodeType, bitRead(CodeFlags, P16_FLAGS_REPEAT));
        log += F(" Code: 0x");
        log += uint64ToString(Code, 16);
        log += F(" / [");
        log += (i + 1);
        log += F("] = {");
        log += typeToString(CommandLines[i].CodeDecodeType, bitRead(CommandLines[i].CodeFlags, P16_FLAGS_REPEAT));
        log += F(" Code: 0x");
        log += uint64ToString(CommandLines[i].Code, 16);
        log += '}';
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }
    # endif // PLUGIN_016_DEBUG
  }
}

bool P016_data_struct::validateCode(int i, uint64_t Code, decode_type_t DecodeType, uint16_t CodeFlags) {
  return ((CommandLines[i].Code == Code)
          && (CommandLines[i].CodeDecodeType == DecodeType)
          && (CommandLines[i].CodeFlags == CodeFlags))
         || ((CommandLines[i].AlternativeCode == Code)
             && (CommandLines[i].AlternativeCodeDecodeType == DecodeType)
             && (CommandLines[i].AlternativeCodeFlags == CodeFlags));
}

#endif // ifdef USES_P016

#include "../PluginStructs/P113_data_struct.h"

#ifdef USES_P113

P113_data_struct::P113_data_struct(uint8_t i2c_addr, int timing, bool range) : i2cAddress(i2c_addr), timing(timing), range(range) {}

// **************************************************************************/
// Initialize VL53L1X
// **************************************************************************/
bool P113_data_struct::begin() {
  initState = true;

  sensor.setI2CAddress(i2cAddress); // Initialize for configured address

  if (sensor.begin() != 0) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("VL53L1X: Sensor not found, init failed for 0x");
      log += String(i2cAddress, HEX);
      addLogMove(LOG_LEVEL_INFO, log);
    }
    initState = false;
    return initState;
  }

  sensor.setTimingBudgetInMs(timing);

  if (range) {
    sensor.setDistanceModeLong();
  } else {
    sensor.setDistanceModeShort();
  }

  return initState;
}

bool P113_data_struct::startRead() {
  if (initState && !readActive) {
    sensor.startRanging();
    readActive = true;
    distance   = -1;
  }
  return readActive;
}

bool P113_data_struct::readAvailable() {
  bool ready = sensor.checkForDataReady();

  if (ready) {
    distance = sensor.getDistance();
    sensor.clearInterrupt();
    sensor.stopRanging();

    // readActive = false;
  }
  return ready;
}

uint16_t P113_data_struct::readDistance() {
  success = false;

  # if defined(P113_DEBUG) || defined(P113_DEBUG_DEBUG)
  String log;
  # endif // if defined(P113_DEBUG) || defined(P113_DEBUG_DEBUG)
  # ifdef P113_DEBUG_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    log  = F("VL53L1X  : idx: 0x");
    log += String(i2cAddress, HEX);
    log += F(" init: ");
    log += String(initState, BIN);
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
  # endif // P113_DEBUG_DEBUG

  success    = true;
  readActive = false;

  if (distance >= 8190) {
    # ifdef P113_DEBUG_DEBUG
    addLog(LOG_LEVEL_DEBUG, "VL53L1X: NO MEASUREMENT");
    # endif // P113_DEBUG_DEBUG
    success = false;
  }

  # ifdef P113_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    log  = F("VL53L1X: Address: 0x");
    log += String(i2cAddress, HEX);
    log += F(" / Timing: ");
    log += String(timing, DEC);
    log += F(" / Long Range: ");
    log += String(range, BIN);
    log += F(" / Distance: ");
    log += distance;
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // P113_DEBUG

  return distance;
}

uint16_t P113_data_struct::readAmbient() {
  return sensor.getAmbientRate();
}

bool P113_data_struct::isReadSuccessful() {
  return success;
}

#endif // ifdef USES_P113

#include "../PluginStructs/P117_data_struct.h"

#ifdef USES_P117

// **************************************************************************/
// Constructor
// **************************************************************************/
P117_data_struct::P117_data_struct(uint16_t altitude,
                                   float    temperatureOffset,
                                   bool     autoCalibration,
                                   uint16_t interval)
  : _altitude(altitude), _temperatureOffset(temperatureOffset), _autoCalibration(autoCalibration), _interval(interval) {}


// **************************************************************************/
// Initialize sensor and read data from SCD30
// **************************************************************************/
uint32_t P117_data_struct::read_sensor(uint16_t *scd30_CO2, uint16_t *scd30_CO2EAvg, float *scd30_Temp, float *scd30_Humid) {
  if (!initialised) {
    initialised = init_sensor(); // Check id device is present
  }

  if (initialised) {
    return scd30.readMeasurement(scd30_CO2, scd30_CO2EAvg, scd30_Temp, scd30_Humid);
  }
  return ERROR_SCD30_NO_DATA;
}

// **************************************************************************/
// Check SCD30 presence and initialize
// **************************************************************************/
bool P117_data_struct::softReset() {
  if (initialised) {
    scd30.softReset();
  }
  return initialised;
}

// **************************************************************************/
// Check SCD30 presence and initialize
// **************************************************************************/
bool P117_data_struct::init_sensor() {
  if (!initialised) {
    scd30.begin();

    scd30.setCalibrationType(_autoCalibration);
    scd30.setMeasurementInterval(_interval);

    scd30.beginMeasuring();
    scd30.setAltitudeCompensation(_altitude);
    scd30.setTemperatureOffset(_temperatureOffset);
    return true;
  }

  return initialised;
}

int P117_data_struct::setCalibrationMode(bool isAuto) {
  if (initialised) {
    _autoCalibration = isAuto;
    return scd30.setCalibrationType(isAuto);
  }
  return ERROR_SCD30_NOT_FOUND_ERROR;
}

int P117_data_struct::setForcedRecalibrationFactor(uint16_t co2_ppm) {
  if (initialised) {
    setCalibrationMode(false); // Force to manual mode
    return scd30.setForcedRecalibrationFactor(co2_ppm);
  }
  return ERROR_SCD30_NOT_FOUND_ERROR;
}

int P117_data_struct::setMeasurementInterval(uint16_t interval) {
  if (initialised) {
    if ((interval >= 2) && (interval <= 1800)) {
      return scd30.setMeasurementInterval(interval);
    } else {
      return ERROR_SCD30_INVALID_VALUE;
    }
  }
  return ERROR_SCD30_NOT_FOUND_ERROR;
}

#endif // ifdef USES_P117

#include "../PluginStructs/P090_data_struct.h"

#ifdef USES_P090

#include "../Globals/I2Cdev.h"

// Register addresses
# define CSS811_STATUS          0x00
# define CSS811_MEAS_MODE       0x01
# define CSS811_ALG_RESULT_DATA 0x02
# define CSS811_RAW_DATA        0x03
# define CSS811_ENV_DATA        0x05
# define CSS811_NTC             0x06
# define CSS811_THRESHOLDS      0x10
# define CSS811_BASELINE        0x11
# define CSS811_HW_ID           0x20
# define CSS811_HW_VERSION      0x21
# define CSS811_FW_BOOT_VERSION 0x23
# define CSS811_FW_APP_VERSION  0x24
# define CSS811_ERROR_ID        0xE0
# define CSS811_APP_START       0xF4
# define CSS811_SW_RESET        0xFF

// ****************************************************************************//
//
//  LIS3DHCore functions
//
//  For I2C, construct LIS3DHCore myIMU(<address>);
//
//  Default <address> is 0x5B.
//
// ****************************************************************************//
CCS811Core::CCS811Core(uint8_t inputArg) : I2CAddress(inputArg)
{}

void CCS811Core::setAddress(uint8_t address)
{
  I2CAddress = address;
}

CCS811Core::status CCS811Core::beginCore(void)
{
  CCS811Core::status returnError = SENSOR_SUCCESS;

  // Wire.begin(); // not necessary

    # ifdef __AVR__
    # else
    # endif

    # ifdef __MK20DX256__
    # else
    # endif

    # ifdef ARDUINO_ARCH_ESP8266
    # else
    # endif

  // Spin for a few ms
  volatile uint8_t temp = 0;

  for (uint16_t i = 0; i < 10000; i++)
  {
    temp++;
  }

  while (Wire.available()) // Clear wire as a precaution
  {
    Wire.read();
  }

  // Check the ID register to determine if the operation was a success.
  uint8_t readCheck;
  readCheck   = 0;
  returnError = readRegister(CSS811_HW_ID, &readCheck);

  if (returnError != SENSOR_SUCCESS)
  {
    return returnError;
  }

  if (readCheck != 0x81)
  {
    returnError = SENSOR_ID_ERROR;
  }

  return returnError;
} // CCS811Core::beginCore

// ****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    offset -- register to read
//    *outputPointer -- Pass &variable (address of) to save read data to
//
// ****************************************************************************//
CCS811Core::status CCS811Core::readRegister(uint8_t offset, uint8_t *outputPointer)
{
  bool wire_status = false;

  *outputPointer = I2C_read8_reg(I2CAddress, offset, &wire_status);

  if (wire_status) {
    return SENSOR_SUCCESS;
  }
  return SENSOR_I2C_ERROR;
}

// ****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
// ****************************************************************************//
CCS811Core::status CCS811Core::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
  if (I2C_write8_reg(I2CAddress, offset, dataToWrite)) {
    return SENSOR_SUCCESS;
  }
  return SENSOR_I2C_ERROR;
}

// ****************************************************************************//
//
//  multiReadRegister
//
//  Parameters:
//    offset -- register to read
//    *inputPointer -- Pass &variable (base address of) to save read data to
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
// ****************************************************************************//
CCS811Core::status CCS811Core::multiWriteRegister(uint8_t offset, uint8_t *inputPointer, uint8_t length)
{
  CCS811Core::status returnError = SENSOR_SUCCESS;

  // define pointer that will point to the external space
  uint8_t i = 0;

  // Set the address
  Wire.beginTransmission(I2CAddress);
  Wire.write(offset);

  while (i < length)           // send data bytes
  {
    Wire.write(*inputPointer); // receive a uint8_t as character
    inputPointer++;
    i++;
  }

  if (Wire.endTransmission() != 0)
  {
    returnError = SENSOR_I2C_ERROR;
  }

  return returnError;
}

// ****************************************************************************//
//
//  Main user class -- wrapper for the core class + maths
//
//  Construct with same rules as the core ( uint8_t busType, uint8_t inputArg )
//
// ****************************************************************************//
CCS811::CCS811(uint8_t inputArg) : CCS811Core(inputArg)
{
  refResistance = 10000;
  resistance    = 0;
  _temperature  = 0;
  tVOC          = 0;
  CO2           = 0;
}

// ****************************************************************************//
//
//  Begin
//
//  This starts the lower level begin, then applies settings
//
// ****************************************************************************//
CCS811Core::status CCS811::begin(void)
{
  uint8_t data[4] = { 0x11, 0xE5, 0x72, 0x8A };    // Reset key

  CCS811Core::status returnError = SENSOR_SUCCESS; // Default error state

  // restart the core
  returnError = beginCore();

  if (returnError != SENSOR_SUCCESS)
  {
    return returnError;
  }

  // Reset the device
  multiWriteRegister(CSS811_SW_RESET, data, 4);
  delay(1);

  if (checkForStatusError() == true)
  {
    return SENSOR_INTERNAL_ERROR;
  }

  if (appValid() == false)
  {
    return SENSOR_INTERNAL_ERROR;
  }

  // Write 0 bytes to this register to start app
  Wire.beginTransmission(I2CAddress);
  Wire.write(CSS811_APP_START);

  if (Wire.endTransmission() != 0)
  {
    return SENSOR_I2C_ERROR;
  }

  delay(200);

  // returnError = setDriveMode(1); //Read every second
  //    Serial.println();

  return returnError;
} // CCS811::begin

// ****************************************************************************//
//
//  Sensor functions
//
// ****************************************************************************//
// Updates the total voltatile organic compounds (TVOC) in parts per billion (PPB)
// and the CO2 value
// Returns nothing
CCS811Core::status CCS811::readAlgorithmResults(void)
{
  I2Cdata_bytes data(4, CSS811_ALG_RESULT_DATA);
  bool allDataRead = I2C_read_bytes(I2CAddress, data);

  if (!allDataRead) {
    return SENSOR_I2C_ERROR;
  }

  // Data ordered:
  // co2MSB, co2LSB, tvocMSB, tvocLSB

  CO2  = ((uint16_t)data[CSS811_ALG_RESULT_DATA + 0] << 8) | data[CSS811_ALG_RESULT_DATA + 1];
  tVOC = ((uint16_t)data[CSS811_ALG_RESULT_DATA + 2] << 8) | data[CSS811_ALG_RESULT_DATA + 3];
  return SENSOR_SUCCESS;
}

// Checks to see if error bit is set
bool CCS811::checkForStatusError(void)
{
  uint8_t value;

  // return the status bit
  readRegister(CSS811_STATUS, &value);
  return value & (1 << 0);
}

// Checks to see if DATA_READ flag is set in the status register
bool CCS811::dataAvailable(void)
{
  uint8_t value;

  CCS811Core::status returnError = readRegister(CSS811_STATUS, &value);

  if (returnError != SENSOR_SUCCESS)
  {
    return false;
  }
  else
  {
    return value & (1 << 3);
  }
}

// Checks to see if APP_VALID flag is set in the status register
bool CCS811::appValid(void)
{
  uint8_t value;

  CCS811Core::status returnError = readRegister(CSS811_STATUS, &value);

  if (returnError != SENSOR_SUCCESS)
  {
    return false;
  }
  else
  {
    return value & (1 << 4);
  }
}

uint8_t CCS811::getErrorRegister(void)
{
  uint8_t value;

  CCS811Core::status returnError = readRegister(CSS811_ERROR_ID, &value);

  if (returnError != SENSOR_SUCCESS)
  {
    return 0xFF;
  }
  else
  {
    return value; // Send all errors in the event of communication error
  }
}

// Returns the baseline value
// Used for telling sensor what 'clean' air is
// You must put the sensor in clean air and record this value
uint16_t CCS811::getBaseline(void)
{
  return I2C_read16_reg(I2CAddress, CSS811_BASELINE);
}

CCS811Core::status CCS811::setBaseline(uint16_t input)
{
  if (I2C_write16_reg(I2CAddress, CSS811_BASELINE, input)) {
    return SENSOR_SUCCESS;
  }
  return SENSOR_I2C_ERROR;
}

// Enable the nINT signal
CCS811Core::status CCS811::enableInterrupts(void)
{
  uint8_t value;
  CCS811Core::status returnError = readRegister(CSS811_MEAS_MODE, &value); // Read what's currently there

  if (returnError != SENSOR_SUCCESS)
  {
    return returnError;
  }

  //    Serial.println(value, HEX);
  value |= (1 << 3); // Set INTERRUPT bit
  writeRegister(CSS811_MEAS_MODE, value);

  //    Serial.println(value, HEX);
  return returnError;
}

// Disable the nINT signal
CCS811Core::status CCS811::disableInterrupts(void)
{
  uint8_t value;

  CCS811Core::status returnError = readRegister(CSS811_MEAS_MODE, &value); // Read what's currently there

  if (returnError != SENSOR_SUCCESS)
  {
    return returnError;
  }

  value      &= ~(1 << 3); // Clear INTERRUPT bit
  returnError = writeRegister(CSS811_MEAS_MODE, value);
  return returnError;
}

// Mode 0 = Idle
// Mode 1 = read every 1s
// Mode 2 = every 10s
// Mode 3 = every 60s
// Mode 4 = RAW mode
CCS811Core::status CCS811::setDriveMode(uint8_t mode)
{
  if (mode > 4)
  {
    mode = 4; // sanitize input
  }

  uint8_t value;
  CCS811Core::status returnError = readRegister(CSS811_MEAS_MODE, &value); // Read what's currently there

  if (returnError != SENSOR_SUCCESS)
  {
    return returnError;
  }

  value      &= ~(0b00000111 << 4); // Clear DRIVE_MODE bits
  value      |= (mode << 4);        // Mask in mode
  returnError = writeRegister(CSS811_MEAS_MODE, value);
  return returnError;
}

// Given a temp and humidity, write this data to the CSS811 for better compensation
// This function expects the humidity and temp to come in as floats
CCS811Core::status CCS811::setEnvironmentalData(float relativeHumidity, float temperature)
{
  // Check for invalid temperatures
  if ((temperature < -25) || (temperature > 50))
  {
    return SENSOR_GENERIC_ERROR;
  }

  // Check for invalid humidity
  if ((relativeHumidity < 0) || (relativeHumidity > 100))
  {
    return SENSOR_GENERIC_ERROR;
  }

  uint32_t rH   = relativeHumidity * 1000; // 42.348 becomes 42348
  uint32_t temp = temperature * 1000;      // 23.2 becomes 23200

  uint8_t envData[4];

  //Split value into 7-bit integer and 9-bit fractional

  //Incorrect way from datasheet.
  //envData[0] = ((rH % 1000) / 100) > 7 ? (rH / 1000 + 1) << 1 : (rH / 1000) << 1;
  //envData[1] = 0; //CCS811 only supports increments of 0.5 so bits 7-0 will always be zero
  //if (((rH % 1000) / 100) > 2 && (((rH % 1000) / 100) < 8))
  //{
  //	envData[0] |= 1; //Set 9th bit of fractional to indicate 0.5%
  //}

  //Correct rounding. See issue 8: https://github.com/sparkfun/Qwiic_BME280_CCS811_Combo/issues/8
  envData[0] = (rH + 250) / 500;
  envData[1] = 0; //CCS811 only supports increments of 0.5 so bits 7-0 will always be zero

  temp += 25000; //Add the 25C offset
  //Split value into 7-bit integer and 9-bit fractional
  //envData[2] = ((temp % 1000) / 100) > 7 ? (temp / 1000 + 1) << 1 : (temp / 1000) << 1;
  //envData[3] = 0;
  //if (((temp % 1000) / 100) > 2 && (((temp % 1000) / 100) < 8))
  //{
  //	envData[2] |= 1;  //Set 9th bit of fractional to indicate 0.5C
  //}

  //Correct rounding
  envData[2] = (temp + 250) / 500;
  envData[3] = 0;

  CCS811Core::status returnError = multiWriteRegister(CSS811_ENV_DATA, envData, 4);

  return returnError;
} // CCS811::setEnvironmentalData

void CCS811::setRefResistance(float input)
{
  refResistance = input;
}

CCS811Core::status CCS811::readNTC(void)
{
  I2Cdata_bytes data(4, CSS811_NTC);
  bool allDataRead = I2C_read_bytes(I2CAddress, data);

  if (!allDataRead) {
    return SENSOR_I2C_ERROR;
  }

  vrefCounts = (static_cast<uint16_t>(data[CSS811_NTC + 0]) << 8) | data[CSS811_NTC + 1];

  // Serial.print("vrefCounts: ");
  // Serial.println(vrefCounts);
  ntcCounts = (static_cast<uint16_t>(data[CSS811_NTC + 2]) << 8) | data[CSS811_NTC + 3];

  // Serial.print("ntcCounts: ");
  // Serial.println(ntcCounts);
  // Serial.print("sum: ");
  // Serial.println(ntcCounts + vrefCounts);
  resistance = (static_cast<float>(ntcCounts) * refResistance / static_cast<float>(vrefCounts));

  // Code from Milan Malesevic and Zoran Stupic, 2011,
  // Modified by Max Mayfield,
  _temperature = log(static_cast<long>(resistance));
  _temperature = 1  / (0.001129148f + (0.000234125f * _temperature) + (0.0000000876741f * _temperature * _temperature * _temperature));
  _temperature = _temperature - 273.15f; // Convert Kelvin to Celsius

  return SENSOR_SUCCESS;
}

uint16_t CCS811::getTVOC(void)
{
  return tVOC;
}

uint16_t CCS811::getCO2(void)
{
  return CO2;
}

float CCS811::getResistance(void)
{
  return resistance;
}

float CCS811::getTemperature(void)
{
  return _temperature;
}

// getDriverError decodes the CCS811Core::status type and prints the
// type of error to the serial terminal.
//
// Save the return value of any function of type CCS811Core::status, then pass
// to this function to see what the output was.
const __FlashStringHelper * CCS811::getDriverError(CCS811Core::status errorCode)
{
  switch (errorCode)
  {
    case CCS811Core::SENSOR_SUCCESS:
      return F("SUCCESS");

    case CCS811Core::SENSOR_ID_ERROR:
      return F("ID_ERROR");

    case CCS811Core::SENSOR_I2C_ERROR:
      return F("I2C_ERROR");

    case CCS811Core::SENSOR_INTERNAL_ERROR:
      return F("INTERNAL_ERROR");

    case CCS811Core::SENSOR_GENERIC_ERROR:
      return F("GENERIC_ERROR");

    default:
      return F("Unknown");
  }
}

// getSensorError gets, clears, then prints the errors
// saved within the error register.
const __FlashStringHelper * CCS811::getSensorError()
{
  uint8_t error = getErrorRegister();

  if (error == 0xFF)
  {
    return F("Failed to get ERROR_ID register.");
  }
  else
  {
    if (error & 1 << 5)
    {
      return F("HeaterSupply");
    }

    if (error & 1 << 4)
    {
      return F("HeaterFault");
    }

    if (error & 1 << 3)
    {
      return F("MaxResistance");
    }

    if (error & 1 << 2)
    {
      return F("MeasModeInvalid");
    }

    if (error & 1 << 1)
    {
      return F("ReadRegInvalid");
    }

    if (error & 1 << 0)
    {
      return F("MsgInvalid");
    }
  }
  return F("");
}

P090_data_struct::P090_data_struct(uint8_t i2cAddr) :
  myCCS811(0x5B) // start with default, but will update later on with user settings
{
  myCCS811.setAddress(i2cAddr);
}

#endif // ifdef USES_P090

#include "../PluginStructs/P052_data_struct.h"

#ifdef USES_P052

P052_data_struct::~P052_data_struct() {
  reset();
}

void P052_data_struct::reset() {
  modbus.reset();
}

bool P052_data_struct::init(const ESPEasySerialPort port, const int16_t serial_rx, const int16_t serial_tx) {
  return modbus.init(port, serial_rx, serial_tx, 9600, P052_MODBUS_SLAVE_ADDRESS);
}

bool P052_data_struct::isInitialized() const {
  return modbus.isInitialized();
}

const __FlashStringHelper * P052_data_struct::Plugin_052_valuename(uint8_t value_nr, bool displayString) {
  switch (value_nr) {
    case 0:  return displayString ? F("Empty") : F("");
    case 1:  return displayString ? F("Carbon Dioxide") : F("co2");
    case 2:  return displayString ? F("Temperature") : F("T");
    case 3:  return displayString ? F("Humidity") : F("H");
    case 4:  return displayString ? F("Relay Status") : F("rel");
    case 5:  return displayString ? F("Temperature Adjustment") : F("Tadj");
    case 6:  return displayString ? F("ABC period") : F("abc_per");
    case 7:  return displayString ? F("Error Status") : F("err");
    default:
      break;
  }
  return F("");
}

#endif // ifdef USES_P052

#include "../PluginStructs/P083_data_struct.h"


// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter
#include <Adafruit_SGP30.h>


#ifdef USES_P083

P083_data_struct::P083_data_struct() {
  initialized = sgp.begin();
  init_time   = millis();
}

#endif // ifdef USES_P083

#include "../PluginStructs/P099_data_struct.h"

#ifdef USES_P099
#include "../ESPEasyCore/ESPEasyNetwork.h"
#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Scheduler.h"
#include "../Helpers/StringConverter.h"
#include "../Helpers/SystemVariables.h"



#include <XPT2046_Touchscreen.h>

P099_data_struct::P099_data_struct() : touchscreen(nullptr) {}

P099_data_struct::~P099_data_struct() {
  reset();
}

/**
 * Proper reset and cleanup.
 */
void P099_data_struct::reset() {
  if (touchscreen != nullptr) {
    delete touchscreen;
    touchscreen = nullptr;
  }
#ifdef PLUGIN_099_DEBUG
  addLog(LOG_LEVEL_INFO, F("P099 DEBUG Touchscreen reset."));
#endif // PLUGIN_099_DEBUG
}

/**
 * Initialize data and set up the touchscreen.
 */
bool P099_data_struct::init(taskIndex_t taskIndex,
                            uint8_t     cs,
                            uint8_t     rotation,
                            bool        flipped,
                            uint8_t     z_treshold,
                            bool        send_xy,
                            bool        send_z,
                            bool        useCalibration,
                            uint16_t    ts_x_res,
                            uint16_t    ts_y_res) {
  reset();

  _address_ts_cs  = cs;
  _z_treshold     = z_treshold;
  _rotation       = rotation;
  _flipped        = flipped;
  _send_xy        = send_xy;
  _send_z         = send_z;
  _useCalibration = useCalibration;
  _ts_x_res       = ts_x_res;
  _ts_y_res       = ts_y_res;

  touchscreen = new (std::nothrow) XPT2046_Touchscreen(_address_ts_cs);
  if (touchscreen != nullptr) {
    touchscreen->setRotation(_rotation);
    touchscreen->setRotationFlipped(_flipped);
    touchscreen->begin();
    loadTouchObjects(taskIndex);
#ifdef PLUGIN_099_DEBUG
    addLog(LOG_LEVEL_INFO, F("P099 DEBUG Plugin & touchscreen initialized."));
   } else {
    addLog(LOG_LEVEL_INFO, F("P099 DEBUG Touchscreen initialisation FAILED."));
#endif // PLUGIN_099_DEBUG
  }
  return isInitialized();
}

/**
 * Properly initialized? then true
 */
bool P099_data_struct::isInitialized() const {
  return touchscreen != nullptr;
}

/**
 * Load the touch objects from the settings, and initialize then properly where needed.
 */
void P099_data_struct::loadTouchObjects(taskIndex_t taskIndex) {
#ifdef PLUGIN_099_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("P099 DEBUG loadTouchObjects size: ");
    log += sizeof(StoredSettings);
    addLogMove(LOG_LEVEL_INFO, log);
  }
#endif // PLUGIN_099_DEBUG
  LoadCustomTaskSettings(taskIndex, reinterpret_cast<uint8_t *>(&StoredSettings), sizeof(StoredSettings));

  for (int i = 0; i < P099_MaxObjectCount; i++) {
    StoredSettings.TouchObjects[i].objectname[P099_MaxObjectNameLength - 1] = 0; // Terminate strings in case of uninitialized data
    SurfaceAreas[i] = 0; // Reset
    TouchTimers[i]  = 0;
    TouchStates[i]  = false;
  }
}

/**
 * Check if the screen is touched.
 */
bool P099_data_struct::touched() {
  if (isInitialized()) {
    return touchscreen->touched();
  } 
  return false;
}

/**
 * Read the raw data if the touchscreen is initialized.
 */
void P099_data_struct::readData(uint16_t *x, uint16_t *y, uint8_t *z) {
  if (isInitialized()) {
    touchscreen->readData(x, y, z);
#ifdef PLUGIN_099_DEBUG
    addLog(LOG_LEVEL_INFO, F("P099 DEBUG readData"));
#endif // PLUGIN_099_DEBUG
  }
}

/**
 * Only set rotation if the touchscreen is initialized.
 */
void P099_data_struct::setRotation(uint8_t n) {
  if (isInitialized()) {
    touchscreen->setRotation(n);
#ifdef PLUGIN_099_DEBUG
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("P099 DEBUG Rotation set: ");
      log += n;
      addLogMove(LOG_LEVEL_INFO, log);
    }
#endif // PLUGIN_099_DEBUG
  }
}

/**
 * Only set rotationFlipped if the touchscreen is initialized.
 */
void P099_data_struct::setRotationFlipped(bool flipped) {
  if (isInitialized()) {
    touchscreen->setRotationFlipped(flipped);
#ifdef PLUGIN_099_DEBUG
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("P099 DEBUG RotationFlipped set: ");
      log += flipped;
      addLogMove(LOG_LEVEL_INFO, log);
    }
#endif // PLUGIN_099_DEBUG
  }
}

/**
 * Determine if calibration is enabled and usable.
 */
bool P099_data_struct::isCalibrationActive() {
  return    _useCalibration
         && StoredSettings.Calibration.top_left.x > 0
         && StoredSettings.Calibration.top_left.y > 0
         && StoredSettings.Calibration.bottom_right.x > 0
         && StoredSettings.Calibration.bottom_right.y > 0; // Enabled and all values != 0 => Active
}

/**
 * Check within the list of defined objects if we touched one of them.
 * The smallest matching surface is selected if multiple objects overlap.
 * Returns state, and sets selectedObjectName to the best matching object
 */
bool P099_data_struct::isValidAndTouchedTouchObject(uint16_t x, uint16_t y, String &selectedObjectName, int &selectedObjectIndex, uint8_t checkObjectCount) {
  uint32_t lastObjectArea = 0;
  bool     selected = false;
  for (uint8_t objectNr = 0; objectNr < checkObjectCount; objectNr++) {
    String objectName = String(StoredSettings.TouchObjects[objectNr].objectname);
    if ( objectName.length() > 0
      && objectName.substring(0,1 ) != F("_")         // Ignore if name starts with an underscore
      && StoredSettings.TouchObjects[objectNr].bottom_right.x > 0
      && StoredSettings.TouchObjects[objectNr].bottom_right.y > 0) { // Not initial could be valid

      if (SurfaceAreas[objectNr] == 0) { // Need to calculate the surface area
        SurfaceAreas[objectNr] = (StoredSettings.TouchObjects[objectNr].bottom_right.x - StoredSettings.TouchObjects[objectNr].top_left.x) * (StoredSettings.TouchObjects[objectNr].bottom_right.y - StoredSettings.TouchObjects[objectNr].top_left.y);
      }

      if ( StoredSettings.TouchObjects[objectNr].top_left.x <= x
        && StoredSettings.TouchObjects[objectNr].top_left.y <= y
        && StoredSettings.TouchObjects[objectNr].bottom_right.x >= x
        && StoredSettings.TouchObjects[objectNr].bottom_right.y >= y
        && (lastObjectArea == 0 
          || SurfaceAreas[objectNr] < lastObjectArea)) { // Select smallest area that fits the coordinates
        selectedObjectName  = objectName;
        selectedObjectIndex = objectNr;
        lastObjectArea      = SurfaceAreas[objectNr];
        selected            = true;
      }
#ifdef PLUGIN_099_DEBUG
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("P099 DEBUG Touched: obj: ");
        log += objectName;
        log += ',';
        log += StoredSettings.TouchObjects[objectNr].top_left.x;
        log += ',';
        log += StoredSettings.TouchObjects[objectNr].top_left.y;
        log += ',';
        log += StoredSettings.TouchObjects[objectNr].bottom_right.x;
        log += ',';
        log += StoredSettings.TouchObjects[objectNr].bottom_right.y;
        log += F(" surface:");
        log += SurfaceAreas[objectNr];
        log += F(" x,y:");
        log += x;
        log += ',';
        log += y;
        log += F(" sel:");
        log += selectedObjectName;
        log += '/';
        log += selectedObjectIndex;
        addLogMove(LOG_LEVEL_INFO, log);
      }
#endif // PLUGIN_099_DEBUG
    }
  }
  return selected;
}

/**
 * Set the enabled/disabled state by inserting or deleting an underscore '_' as the first character of the object name.
 * Checks if the name doesn't exceed the max. length.
 */
bool P099_data_struct::setTouchObjectState(const String& touchObject, bool state, uint8_t checkObjectCount) {
  if (touchObject.isEmpty() || touchObject.substring(0, 1) == F("_")) return false;
  String findObject = (state ? F("_") : F("")); // When enabling, try to find a disabled object
  findObject += touchObject;
  String thisObject;
  bool   success = false;
  thisObject.reserve(P099_MaxObjectNameLength);
  for (uint8_t objectNr = 0; objectNr < checkObjectCount; objectNr++) {
    thisObject = String(StoredSettings.TouchObjects[objectNr].objectname);
    if (thisObject.length() > 0 && findObject.equalsIgnoreCase(thisObject)) {
      if (state) {
        success = safe_strncpy(StoredSettings.TouchObjects[objectNr].objectname, thisObject.substring(1), P099_MaxObjectNameLength); // Keep original character casing
      } else {
        if (thisObject.length() < P099_MaxObjectNameLength - 2) { // Leave room for the underscore and the terminating 0.
          String disabledObject = F("_");
          disabledObject += thisObject;
          success = safe_strncpy(StoredSettings.TouchObjects[objectNr].objectname, disabledObject, P099_MaxObjectNameLength);
        }
      }
      StoredSettings.TouchObjects[objectNr].objectname[P099_MaxObjectNameLength - 1] = 0; // Just to be safe
#ifdef PLUGIN_099_DEBUG
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("P099 setTouchObjectState: obj: ");
        log += thisObject;
        if (success) {
          log += F(", new state: ");
          log += (state ? F("en") : F("dis"));
          log += F("abled.");
        } else {
          log += F("failed!");
        }
        addLogMove(LOG_LEVEL_INFO, log);
      }
#endif // PLUGIN_099_DEBUG
      // break; // Only first one found is processed
    }
  }
  return success;
}

/**
 * Scale the provided raw coordinates to screen-resolution coordinates if calibration is enabled/configured
 */
void P099_data_struct::scaleRawToCalibrated(uint16_t &x, uint16_t &y) {
  if (isCalibrationActive()) {
    uint16_t lx = x - StoredSettings.Calibration.top_left.x;
    if (lx <= 0) {
      x = 0;
    } else {
      if (lx > StoredSettings.Calibration.bottom_right.x) {
        lx = StoredSettings.Calibration.bottom_right.x;
      }
      float x_fact = static_cast<float>(StoredSettings.Calibration.bottom_right.x - StoredSettings.Calibration.top_left.x) / static_cast<float>(_ts_x_res);
      x = static_cast<uint16_t>(round(lx / x_fact));
    }
    uint16_t ly = y - StoredSettings.Calibration.top_left.y;
    if (ly <= 0) {
      y = 0;
    } else {
      if (ly > StoredSettings.Calibration.bottom_right.y) {
        ly = StoredSettings.Calibration.bottom_right.y;
      }
      float y_fact = (StoredSettings.Calibration.bottom_right.y - StoredSettings.Calibration.top_left.y) / _ts_y_res;
      y = static_cast<uint16_t>(round(ly / y_fact));
    }
  }
}

#endif  // ifdef USES_P099
#include "../PluginStructs/P031_data_struct.h"

#ifdef USES_P031

uint8_t P031_data_struct::init(uint8_t data_pin, uint8_t clock_pin, bool pullUp, uint8_t clockdelay) {
  _dataPin    = data_pin;
  _clockPin   = clock_pin;
  _clockdelay = clockdelay;

  if (_clockdelay > P031_MAX_CLOCK_DELAY) {
    _clockdelay = P031_MAX_CLOCK_DELAY;
  }
  input_mode = pullUp ? INPUT_PULLUP : INPUT;
  state      = P031_IDLE;

  pinMode(_dataPin,  input_mode); /* Keep Hi-Z except when sending data */
  pinMode(_clockPin, OUTPUT);
  resetSensor();
  return readStatus();
}

bool P031_data_struct::process() {
  switch (state) {
    case P031_IDLE: return false; // Nothing changed, nothing to do
    case P031_WAIT_TEMP: {
      if (digitalRead(_dataPin) == LOW) {
        float tempRaw = readData(16);

        // Temperature conversion coefficients from SHT1X datasheet for version 4
        const float d1 = -39.7f; // 3.5V
        const float d2 = 0.01f;  // 14-bit
        tempC = d1 + (tempRaw * d2);
        state = P031_WAIT_HUM;   // Wait for humidity
        sendCommand(SHT1X_CMD_MEASURE_RH);
      }
      break;
    }
    case P031_WAIT_HUM:
    {
      if (digitalRead(_dataPin) == LOW) {
        float raw = readData(16);

        // Temperature conversion coefficients from SHT1X datasheet for version 4
        const float c1 = -2.0468f;
        const float c2 = 0.0367f;
        const float c3 = -1.5955E-6f;
        const float t1 = 0.01f;
        const float t2 = 0.00008f;

        const float rhLinear = c1 + c2 * raw + c3 * raw * raw;
        rhTrue = (tempC - 25) * (t1 + t2 * raw) + rhLinear;

        /*
            String log = F("SHT1X : Read humidity (raw): ");
            log += String(raw);
            log += F(" (Linear): ");
            log += String(rhLinear);
            log += F(" (True): ");
            log += String(rhTrue);
            addLog(LOG_LEVEL_DEBUG, log);
         */
        state = P031_MEAS_READY; // Measurement ready
        return true;
      }
      break;
    }
    case P031_MEAS_READY: return true;
    default:
      // It is already an error state, just return.
      return false;
  }

  // Compute timeout
  if (timePassedSince(sendCommandTime) > 320) {
    state = P031_NO_DATA; // No data after 320 msec
  }
  return false;
}

void P031_data_struct::startMeasurement() {
  state = P031_WAIT_TEMP; // Wait for temperature
  sendCommand(SHT1X_CMD_MEASURE_TEMP);
}

void P031_data_struct::resetSensor()
{
  state = P031_IDLE;
  delay(11);

  for (int i = 0; i < 9; i++) {
    digitalWrite(_clockPin, HIGH);
    digitalWrite(_clockPin, LOW);
  }
  sendCommand(SHT1X_CMD_SOFT_RESET);
  delay(11);
}

uint8_t P031_data_struct::readStatus()
{
  sendCommand(SHT1X_CMD_READ_STATUS);
  return readData(8);
}

void P031_data_struct::sendCommand(const uint8_t cmd)
{
  sendCommandTime = millis();

  // Transmission Start sequence
  digitalWrite(_dataPin, HIGH);
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_clockPin, HIGH);
  P031_DELAY_LONGER_CABLES
    digitalWrite(_dataPin, LOW);

  digitalWrite(_clockPin, LOW);
  P031_DELAY_LONGER_CABLES
    digitalWrite(_clockPin, HIGH);
  P031_DELAY_LONGER_CABLES
    digitalWrite(_dataPin, HIGH);

  digitalWrite(_clockPin, LOW);
  P031_DELAY_LONGER_CABLES

  // Send the command (address must be 000b)
    shiftOut(_dataPin, _clockPin, MSBFIRST, cmd);

  // Wait for ACK
  pinMode(_dataPin, input_mode);
  bool ackerror = false;

  digitalWrite(_clockPin, HIGH);
  P031_DELAY_LONGER_CABLES

  if (digitalRead(_dataPin) != LOW) { ackerror = true; }
  digitalWrite(_clockPin, LOW);
  P031_DELAY_LONGER_CABLES

  if ((cmd == SHT1X_CMD_MEASURE_TEMP) || (cmd == SHT1X_CMD_MEASURE_RH)) {
    delayMicroseconds(1); /* Give the sensor time to release the data line */

    if (digitalRead(_dataPin) != HIGH) { ackerror = true; }
  }

  if (ackerror) {
    state = P031_COMMAND_NO_ACK;
  }
}

int P031_data_struct::readData(const int bits) const
{
  int val = 0;

  if (bits == 16) {
    // Read most significant uint8_t
    val   = shiftIn(_dataPin, _clockPin, MSBFIRST);
    val <<= 8;

    // Send ACK
    pinMode(_dataPin, OUTPUT);
    digitalWrite(_dataPin,  LOW);
    digitalWrite(_clockPin, HIGH);
    P031_DELAY_LONGER_CABLES
      digitalWrite(_clockPin, LOW);
    P031_DELAY_LONGER_CABLES
      pinMode(_dataPin, input_mode);
  }

  // Read least significant uint8_t
  val |= shiftIn(_dataPin, _clockPin, MSBFIRST);

  // Keep DATA pin high to skip CRC
  digitalWrite(_clockPin, HIGH);
  P031_DELAY_LONGER_CABLES
    digitalWrite(_clockPin, LOW);

  P031_DELAY_LONGER_CABLES

  return val;
}

uint8_t P031_data_struct::shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) const
{
  uint8_t value = 0;
  uint8_t i;

  for (i = 0; i < 8; ++i) {
    digitalWrite(clockPin, HIGH);
    P031_DELAY_LONGER_CABLES

    if (bitOrder == LSBFIRST) {
      value |= digitalRead(dataPin) << i;
    }
    else {
      value |= digitalRead(dataPin) << (7 - i);
    }
    digitalWrite(clockPin, LOW);
    P031_DELAY_LONGER_CABLES
  }
  return value;
}

void P031_data_struct::shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val) const
{
  uint8_t i;

  for (i = 0; i < 8; i++)  {
    if (bitOrder == LSBFIRST) {
      digitalWrite(dataPin, !!(val & (1 << i)));
    }
    else {
      digitalWrite(dataPin, !!(val & (1 << (7 - i))));
    }

    digitalWrite(clockPin, HIGH);
    P031_DELAY_LONGER_CABLES
      digitalWrite(clockPin, LOW);
    P031_DELAY_LONGER_CABLES
  }
}

#endif // ifdef USES_P031

#include "../PluginStructs/P006_data_struct.h"

#ifdef USES_P006


# define BMP085_I2CADDR           0x77
# define BMP085_CAL_AC1           0xAA // R   Calibration data (16 bits)
# define BMP085_CAL_AC2           0xAC // R   Calibration data (16 bits)
# define BMP085_CAL_AC3           0xAE // R   Calibration data (16 bits)
# define BMP085_CAL_AC4           0xB0 // R   Calibration data (16 bits)
# define BMP085_CAL_AC5           0xB2 // R   Calibration data (16 bits)
# define BMP085_CAL_AC6           0xB4 // R   Calibration data (16 bits)
# define BMP085_CAL_B1            0xB6 // R   Calibration data (16 bits)
# define BMP085_CAL_B2            0xB8 // R   Calibration data (16 bits)
# define BMP085_CAL_MB            0xBA // R   Calibration data (16 bits)
# define BMP085_CAL_MC            0xBC // R   Calibration data (16 bits)
# define BMP085_CAL_MD            0xBE // R   Calibration data (16 bits)
# define BMP085_CONTROL           0xF4
# define BMP085_TEMPDATA          0xF6
# define BMP085_PRESSUREDATA      0xF6
# define BMP085_READTEMPCMD       0x2E
# define BMP085_READPRESSURECMD   0x34


bool P006_data_struct::begin()
{
  if (!initialized) {
    if (I2C_read8_reg(BMP085_I2CADDR, 0xD0) != 0x55) { return false; }

    /* read calibration data */
    ac1 = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_AC1);
    ac2 = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_AC2);
    ac3 = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_AC3);
    ac4 = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_AC4);
    ac5 = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_AC5);
    ac6 = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_AC6);

    b1 = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_B1);
    b2 = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_B2);

    mb = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_MB);
    mc = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_MC);
    md = I2C_read16_reg(BMP085_I2CADDR, BMP085_CAL_MD);

    initialized = true;
  }

  return true;
}

uint16_t P006_data_struct::readRawTemperature(void)
{
  I2C_write8_reg(BMP085_I2CADDR, BMP085_CONTROL, BMP085_READTEMPCMD);
  delay(5);
  return I2C_read16_reg(BMP085_I2CADDR, BMP085_TEMPDATA);
}

uint32_t P006_data_struct::readRawPressure(void)
{
  uint32_t raw;

  I2C_write8_reg(BMP085_I2CADDR, BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  delay(26);

  raw   = I2C_read16_reg(BMP085_I2CADDR, BMP085_PRESSUREDATA);
  raw <<= 8;
  raw  |= I2C_read8_reg(BMP085_I2CADDR, BMP085_PRESSUREDATA + 2);
  raw >>= (8 - oversampling);

  return raw;
}

int32_t P006_data_struct::readPressure(void)
{
  int32_t  UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = readRawTemperature();
  UP = readRawPressure();

  // do temperature calculations
  X1 = (UT - (int32_t)(ac6)) * ((int32_t)(ac5)) / pow(2, 15);
  X2 = ((int32_t)mc * pow(2, 11)) / (X1 + (int32_t)md);
  B5 = X1 + X2;

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1 * 4 + X3) << oversampling) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> oversampling);

  if (B7 < 0x80000000)
  {
    p = (B7 * 2) / B4;
  }
  else
  {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791) >> 4);
  return p;
}

float P006_data_struct::readTemperature(void)
{
  int32_t UT, X1, X2, B5; // following ds convention
  float   temp;

  UT = readRawTemperature();

  // step 1
  X1    = (UT - (int32_t)ac6) * ((int32_t)ac5) / pow(2, 15);
  X2    = ((int32_t)mc * pow(2, 11)) / (X1 + (int32_t)md);
  B5    = X1 + X2;
  temp  = (B5 + 8) / pow(2, 4);
  temp /= 10;

  return temp;
}

#endif // ifdef USES_P006

#include "../PluginStructs/P024_data_struct.h"

#ifdef USES_P024

P024_data_struct::P024_data_struct(uint8_t i2c_addr) : i2cAddress(i2c_addr) {}

float P024_data_struct::readTemperature(uint8_t reg)
{
  float temp = readRegister024(reg);

  temp *= .02f;
  temp -= 273.15f;
  return temp;
}

uint16_t P024_data_struct::readRegister024(uint8_t reg) {
  uint16_t ret;

  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, (uint8_t)3);
  ret  = Wire.read();      // receive DATA
  ret |= Wire.read() << 8; // receive DATA
  Wire.read();
  return ret;
}

#endif // ifdef USES_P024

#include "../PluginStructs/P069_data_struct.h"

#ifdef USES_P069

#include "../Globals/I2Cdev.h"


# define LM75A_BASE_ADDRESS        0x48
# define LM75A_DEGREES_RESOLUTION  0.125f
# define LM75A_REG_ADDR_TEMP     0


P069_data_struct::P069_data_struct(bool A0_value, bool A1_value, bool A2_value)
{
  _i2c_device_address = LM75A_BASE_ADDRESS;

  if (A0_value) {
    _i2c_device_address += 1;
  }

  if (A1_value) {
    _i2c_device_address += 2;
  }

  if (A2_value) {
    _i2c_device_address += 4;
  }
}

P069_data_struct::P069_data_struct(uint8_t addr)
{
  _i2c_device_address = addr;
}

void P069_data_struct::setAddress(uint8_t addr)
{
  _i2c_device_address = addr;
}

float P069_data_struct::getTemperatureInDegrees() const
{
  float   real_result = NAN;
  int16_t value       = 0;

  // Go to temperature data register
  Wire.beginTransmission(_i2c_device_address);
  Wire.write(LM75A_REG_ADDR_TEMP);

  if (Wire.endTransmission())
  {
    // Transmission error
    return real_result;
  }

  // Get content
  Wire.requestFrom(_i2c_device_address, (uint8_t)2);

  if (Wire.available() == 2)
  {
    value = (Wire.read() << 8) | Wire.read();
  }
  else
  {
    // Can't read temperature
    return real_result;
  }

  // Shift data (left-aligned)
  value >>= 5;

  // Relocate negative bit (11th bit to 16th bit)
  if (value & 0x0400) // negative?
  {
    value |= 0xFC00;  // expand to 16 bit
  }

  // Real value can be calculated with sensor resolution
  real_result = static_cast<float>(value) * LM75A_DEGREES_RESOLUTION;

  return real_result;
}

#endif // ifdef USES_P069

#include "../PluginStructs/P106_data_struct.h"

#ifdef USES_P106


// Needed also here for PlatformIO's library finder as the .h file
// is in a directory which is excluded in the src_filter
# include <Adafruit_Sensor.h>
# include <Adafruit_BME680.h>


bool P106_data_struct::begin(uint8_t addr, bool initSettings)
{
  if (!initialized) {
    initialized = bme.begin(addr, initSettings);

    if (initialized) {
      // Set up oversampling and filter initialization
      bme.setTemperatureOversampling(BME680_OS_8X);
      bme.setHumidityOversampling(BME680_OS_2X);
      bme.setPressureOversampling(BME680_OS_4X);
      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme.setGasHeater(320, 150); // 320*C for 150 ms
    }
  }

  return initialized;
}

#endif // ifdef USES_P106

#include "../PluginStructs/P094_data_struct.h"

#ifdef USES_P094

// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter
#include <ESPeasySerial.h>

#include <Regexp.h>

#include "../Globals/ESPEasy_time.h"
#include "../Helpers/StringConverter.h"


P094_data_struct::P094_data_struct() :  easySerial(nullptr) {}

P094_data_struct::~P094_data_struct() {
  reset();
}

void P094_data_struct::reset() {
  if (easySerial != nullptr) {
    delete easySerial;
    easySerial = nullptr;
  }
}

bool P094_data_struct::init(ESPEasySerialPort port, 
                            const int16_t serial_rx, 
                            const int16_t serial_tx, 
                            unsigned long baudrate) {
  if ((serial_rx < 0) && (serial_tx < 0)) {
    return false;
  }
  reset();
  easySerial = new (std::nothrow) ESPeasySerial(port, serial_rx, serial_tx);

  if (isInitialized()) {
    easySerial->begin(baudrate);
    return true;
  }
  return false;
}

void P094_data_struct::post_init() {
  for (uint8_t i = 0; i < P094_FILTER_VALUE_Type_NR_ELEMENTS; ++i) {
    valueType_used[i] = false;
  }

  for (uint8_t i = 0; i < P094_NR_FILTERS; ++i) {
    size_t lines_baseindex            = P094_Get_filter_base_index(i);
    int    index                      = _lines[lines_baseindex].toInt();
    int    tmp_filter_comp            = _lines[lines_baseindex + 2].toInt();
    const bool filter_string_notempty = _lines[lines_baseindex + 3].length() > 0;
    const bool valid_index            = index >= 0 && index < P094_FILTER_VALUE_Type_NR_ELEMENTS;
    const bool valid_filter_comp      = tmp_filter_comp >= 0 && tmp_filter_comp < P094_FILTER_COMP_NR_ELEMENTS;

    valueType_index[i] = P094_not_used;

    if (valid_index && valid_filter_comp && filter_string_notempty) {
      valueType_used[index] = true;
      valueType_index[i]    = static_cast<P094_Filter_Value_Type>(index);
      filter_comp[i]        = static_cast<P094_Filter_Comp>(tmp_filter_comp);
    }
  }
}

bool P094_data_struct::isInitialized() const {
  return easySerial != nullptr;
}

void P094_data_struct::sendString(const String& data) {
  if (isInitialized()) {
    if (data.length() > 0) {
      setDisableFilterWindowTimer();
      easySerial->write(data.c_str());

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("Proxy: Sending: ");
        log += data;
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }
  }
}

bool P094_data_struct::loop() {
  if (!isInitialized()) {
    return false;
  }
  bool fullSentenceReceived = false;

  if (easySerial != nullptr) {
    int available = easySerial->available();

    unsigned long timeout = millis() + 10;

    while (available > 0 && !fullSentenceReceived) {
      // Look for end marker
      char c = easySerial->read();
      --available;

      if (available == 0) {
        if (!timeOutReached(timeout)) {
          available = easySerial->available();
        }
        delay(0);
      }

      switch (c) {
        case 13:
        {
          const size_t length = sentence_part.length();
          bool valid          = length > 0;

          for (size_t i = 0; i < length && valid; ++i) {
            if ((sentence_part[i] > 127) || (sentence_part[i] < 32)) {
              sentence_part = String();
              ++sentences_received_error;
              valid = false;
            }
          }

          if (valid) {
            fullSentenceReceived = true;
          }
          break;
        }
        case 10:

          // Ignore LF
          break;
        default:
          sentence_part += c;
          break;
      }

      if (max_length_reached()) { fullSentenceReceived = true; }
    }
  }

  if (fullSentenceReceived) {
    ++sentences_received;
    length_last_received = sentence_part.length();
  }
  return fullSentenceReceived;
}

const String& P094_data_struct::peekSentence() const {
  return sentence_part;
}

void P094_data_struct::getSentence(String& string, bool appendSysTime) {
  string = std::move(sentence_part);
  sentence_part = String(); // FIXME TD-er: Should not be needed as move already cleared it.
  if (appendSysTime) {
    // Unix timestamp = 10 decimals + separator
    if (string.reserve(sentence_part.length() + 11)) {
      string += ';';
      string += node_time.getUnixTime();
    }
  }
  sentence_part.reserve(string.length());
}

void P094_data_struct::getSentencesReceived(uint32_t& succes, uint32_t& error, uint32_t& length_last) const {
  succes      = sentences_received;
  error       = sentences_received_error;
  length_last = length_last_received;
}

void P094_data_struct::setMaxLength(uint16_t maxlenght) {
  max_length = maxlenght;
}

void P094_data_struct::setLine(uint8_t varNr, const String& line) {
  if (varNr < P94_Nlines) {
    _lines[varNr] = line;
  }
}

uint32_t P094_data_struct::getFilterOffWindowTime() const {
  return _lines[P094_FILTER_OFF_WINDOW_POS].toInt();
}

P094_Match_Type P094_data_struct::getMatchType() const {
  return static_cast<P094_Match_Type>(_lines[P094_MATCH_TYPE_POS].toInt());
}

bool P094_data_struct::invertMatch() const {
  switch (getMatchType()) {
    case P094_Regular_Match:
      break;
    case P094_Regular_Match_inverted:
      return true;
    case P094_Filter_Disabled:
      break;
  }
  return false;
}

bool P094_data_struct::filterUsed(uint8_t lineNr) const
{
  if (valueType_index[lineNr] == P094_Filter_Value_Type::P094_not_used) { return false; }
  uint8_t varNr = P094_Get_filter_base_index(lineNr);
  return _lines[varNr + 3].length() > 0;
}

String P094_data_struct::getFilter(uint8_t lineNr, P094_Filter_Value_Type& filterValueType, uint32_t& optional,
                                   P094_Filter_Comp& comparator) const
{
  uint8_t varNr = P094_Get_filter_base_index(lineNr);

  filterValueType = P094_Filter_Value_Type::P094_not_used;

  if ((varNr + 3) >= P94_Nlines) { return ""; }
  optional        = _lines[varNr + 1].toInt();
  filterValueType = valueType_index[lineNr];
  comparator      = filter_comp[lineNr];

  //  filterValueType = static_cast<P094_Filter_Value_Type>(_lines[varNr].toInt());
  //  comparator      = static_cast<P094_Filter_Comp>(_lines[varNr + 2].toInt());
  return _lines[varNr + 3];
}

void P094_data_struct::setDisableFilterWindowTimer() {
  if (getFilterOffWindowTime() == 0) {
    disable_filter_window = 0;
  }
  else {
    disable_filter_window = millis() + getFilterOffWindowTime();
  }
}

bool P094_data_struct::disableFilterWindowActive() const {
  if (disable_filter_window != 0) {
    if (!timeOutReached(disable_filter_window)) {
      // We're still in the window where filtering is disabled
      return true;
    }
  }
  return false;
}

bool P094_data_struct::parsePacket(const String& received) const {
  size_t strlength = received.length();

  if (strlength == 0) {
    return false;
  }


  if (getMatchType() == P094_Filter_Disabled) {
    return true;
  }

  bool match_result = false;

  // FIXME TD-er: For now added '$' to test with GPS.
  if ((received[0] == 'b') || (received[0] == '$')) {
    // Received a data packet in CUL format.
    if (strlength < 21) {
      return false;
    }

    // Decoded packet

    unsigned long packet_header[P094_FILTER_VALUE_Type_NR_ELEMENTS];
    packet_header[P094_packet_length] = hexToUL(received, 1, 2);
    packet_header[P094_unknown1]      = hexToUL(received, 3, 2);
    packet_header[P094_manufacturer]  = hexToUL(received, 5, 4);
    packet_header[P094_serial_number] = hexToUL(received, 9, 8);
    packet_header[P094_unknown2]      = hexToUL(received, 17, 2);
    packet_header[P094_meter_type]    = hexToUL(received, 19, 2);

    // FIXME TD-er: Is this also correct?
    packet_header[P094_rssi] = hexToUL(received, strlength - 4, 4);

    // FIXME TD-er: Is this correct?
    // match_result = packet_length == (strlength - 21) / 2;

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      if (log.reserve(128)) {
        log  = F("CUL Reader: ");
        log += F(" length: ");
        log += packet_header[P094_packet_length];
        log += F(" (header: ");
        log += strlength - (packet_header[P094_packet_length] * 2);
        log += F(") manu: ");
        log += formatToHex_decimal(packet_header[P094_manufacturer]);
        log += F(" serial: ");
        log += formatToHex_decimal(packet_header[P094_serial_number]);
        log += F(" mType: ");
        log += formatToHex_decimal(packet_header[P094_meter_type]);
        log += F(" RSSI: ");
        log += formatToHex_decimal(packet_header[P094_rssi]);
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }

    bool filter_matches[P094_NR_FILTERS];

    for (unsigned int f = 0; f < P094_NR_FILTERS; ++f) {
      filter_matches[f] = false;
    }

    // Do not check for "not used" (0)
    for (unsigned int i = 1; i < P094_FILTER_VALUE_Type_NR_ELEMENTS; ++i) {
      if (valueType_used[i]) {
        for (unsigned int f = 0; f < P094_NR_FILTERS; ++f) {
          if (valueType_index[f] == i) {
            // Have a matching filter

            uint32_t optional;
            P094_Filter_Value_Type filterValueType;
            P094_Filter_Comp comparator;
            bool   match = false;
            String inputString;
            String valueString;

            if (i == P094_Filter_Value_Type::P094_position) {
              valueString = getFilter(f, filterValueType, optional, comparator);

              if (received.length() >= (optional + valueString.length())) {
                // received string is long enough to fit the expression.
                inputString = received.substring(optional, optional + valueString.length());
                match = inputString.equalsIgnoreCase(valueString);
              }
            } else {
              unsigned long value = hexToUL(getFilter(f, filterValueType, optional, comparator));
              match       = (value == packet_header[i]);
              inputString = formatToHex_decimal(packet_header[i]);
              valueString = formatToHex_decimal(value);
            }


            if (loglevelActiveFor(LOG_LEVEL_INFO)) {
              String log;
              if (log.reserve(64)) {
                log  = F("CUL Reader: ");
                log += P094_FilterValueType_toString(valueType_index[f]);
                log += F(":  in:");
                log += inputString;
                log += ' ';
                log += P094_FilterComp_toString(comparator);
                log += ' ';
                log += valueString;

                switch (comparator) {
                  case P094_Filter_Comp::P094_Equal_OR:
                  case P094_Filter_Comp::P094_Equal_MUST:

                    if (match) { log += F(" expected MATCH"); } 
                    break;
                  case P094_Filter_Comp::P094_NotEqual_OR:
                  case P094_Filter_Comp::P094_NotEqual_MUST:

                    if (!match) { log += F(" expected NO MATCH"); }
                    break;
                }
                addLogMove(LOG_LEVEL_INFO, log);
              }
            }

            switch (comparator) {
              case P094_Filter_Comp::P094_Equal_OR:

                if (match) { filter_matches[f] = true; }
                break;
              case P094_Filter_Comp::P094_NotEqual_OR:

                if (!match) { filter_matches[f] = true; }
                break;

              case P094_Filter_Comp::P094_Equal_MUST:

                if (!match) { return false; }
                break;

              case P094_Filter_Comp::P094_NotEqual_MUST:

                if (match) { return false; }
                break;
            }
          }
        }
      }
    }

    // Now we have to check if all rows per filter line in filter_matches[f] are true or not used.
    int nrMatches = 0;
    int nrNotUsed = 0;

    for (unsigned int f = 0; !match_result && f < P094_NR_FILTERS; ++f) {
      if (f % P094_AND_FILTER_BLOCK == 0) {
        if ((nrMatches > 0) && ((nrMatches + nrNotUsed) == P094_AND_FILTER_BLOCK)) {
          match_result = true;
        }
        nrMatches = 0;
        nrNotUsed = 0;
      }

      if (filter_matches[f]) {
        ++nrMatches;
      } else {
        if (!filterUsed(f)) {
          ++nrNotUsed;
        }
      }
    }
  } else {
    switch (received[0]) {
      case 'C': // CMODE
      case 'S': // SMODE
      case 'T': // TMODE
      case 'O': // OFF
      case 'V': // Version info

        // FIXME TD-er: Must test the result of the other possible answers.
        match_result = true;
        break;
    }
  }

  return match_result;
}

const __FlashStringHelper * P094_data_struct::MatchType_toString(P094_Match_Type matchType) {
  switch (matchType)
  {
    case P094_Match_Type::P094_Regular_Match:          return F("Regular Match");
    case P094_Match_Type::P094_Regular_Match_inverted: return F("Regular Match inverted");
    case P094_Match_Type::P094_Filter_Disabled:        return F("Filter Disabled");
  }
  return F("");
}

const __FlashStringHelper * P094_data_struct::P094_FilterValueType_toString(P094_Filter_Value_Type valueType)
{
  switch (valueType) {
    case P094_Filter_Value_Type::P094_not_used:      return F("---");
    case P094_Filter_Value_Type::P094_packet_length: return F("Packet Length");
    case P094_Filter_Value_Type::P094_unknown1:      return F("unknown1");
    case P094_Filter_Value_Type::P094_manufacturer:  return F("Manufacturer");
    case P094_Filter_Value_Type::P094_serial_number: return F("Serial Number");
    case P094_Filter_Value_Type::P094_unknown2:      return F("unknown2");
    case P094_Filter_Value_Type::P094_meter_type:    return F("Meter Type");
    case P094_Filter_Value_Type::P094_rssi:          return F("RSSI");
    case P094_Filter_Value_Type::P094_position:      return F("Position");

      //    default: break;
  }
  return F("unknown");
}

const __FlashStringHelper * P094_data_struct::P094_FilterComp_toString(P094_Filter_Comp comparator)
{
  switch (comparator) {
    case P094_Filter_Comp::P094_Equal_OR:      return F("==");
    case P094_Filter_Comp::P094_NotEqual_OR:   return F("!=");
    case P094_Filter_Comp::P094_Equal_MUST:    return F("== (must)");
    case P094_Filter_Comp::P094_NotEqual_MUST: return F("!= (must)");
  }
  return F("");
}

bool P094_data_struct::max_length_reached() const {
  if (max_length == 0) { return false; }
  return sentence_part.length() >= max_length;
}

size_t P094_data_struct::P094_Get_filter_base_index(size_t filterLine) {
  return filterLine * P094_ITEMS_PER_FILTER + P094_FIRST_FILTER_POS;
}

uint32_t P094_data_struct::getDebugCounter() {
  return debug_counter++;
}

#endif // USES_P094
#include "../PluginStructs/P093_data_struct.h"

#ifdef USES_P093


# define PLUGIN_093_DEBUG

/*
 *
 * Bi-directional communication with the heat pump.
 *
 * Should support all Mitsubishi HP units with CN105 connector.
 *
 * Plugin is based on "Arduino library to control Mitsubishi Heat Pumps" from
 * https://github.com/SwiCago/HeatPump.
 *
 */

P093_data_struct::P093_data_struct(const ESPEasySerialPort port, const int16_t serialRx, const int16_t serialTx, bool includeStatus) :
  _serial(port, serialRx, serialTx),
  _state(NotConnected),
  _fastBaudRate(false),
  _readPos(0),
  _writeTimeout(0),
  _infoModeIndex(0),
  _statusUpdateTimeout(0),
  _tempMode(false),
  _wideVaneAdj(false),
  _valuesInitialized(false),
  _includeStatus(includeStatus) {
  setState(Connecting);
}

bool P093_data_struct::sync() {
  if (_statusUpdateTimeout != 0) {
    if (_writeStatus.isDirty()) {
      cancelWaitingAndTransitTo(ApplyingSettings);
      return false;
    }

    if (timeOutReached(_statusUpdateTimeout)) {
      cancelWaitingAndTransitTo(UpdatingStatus);
      return false;
    }
  }

  return readIncommingBytes();
}

bool P093_data_struct::read(String& result) const {
  if (_valuesInitialized == false) {
    return false;
  }

  result.reserve(150);

  // FIXME TD-er: See if this macro can be simpler as it does expand to quite some code which is not changing.
    # define map_list(x, list) findByValue(x, list, sizeof(list) / sizeof(Tuple))

  result  = F("{\"roomTemperature\":");
  result += toString(_currentValues.roomTemperature, 1);
  result += F(",\"wideVane\":\"");
  result += map_list(_currentValues.wideVane, _mappings.wideVane);
  result += F("\",\"power\":\"");
  result += map_list(_currentValues.power, _mappings.power);
  result += F("\",\"mode\":\"");
  result += map_list(_currentValues.mode, _mappings.mode);
  result += F("\",\"fan\":\"");
  result += map_list(_currentValues.fan, _mappings.fan);
  result += F("\",\"vane\":\"");
  result += map_list(_currentValues.vane, _mappings.vane);
  result += F("\",\"iSee\":");
  result += boolToString(_currentValues.iSee);

  if (_includeStatus) {
    result += F(",\"operating\":");
    result += boolToString(_currentValues.operating);
    result += F(",\"compressorFrequency\":");
    result += _currentValues.compressorFrequency;
  }
  result += F(",\"temperature\":");
  result += toString(_currentValues.temperature, 1) + '}';

    # undef map_list

  return true;
}

void P093_data_struct::write(const String& command, const String& value) {
    # define lookup(x, list, placeholder) findByMapping(x, list, sizeof(list) / sizeof(Tuple), placeholder)

  if (command == F("temperature")) {
    float temperature = 0;

    if (string2float(value, temperature) && (temperature >= 16) && (temperature <= 31)) {
      _wantedSettings.temperature = temperature;
      _writeStatus.set(Temperature);
    }
  } else if ((command == F("power")) && lookup(value, _mappings.power, _wantedSettings.power)) {
    _writeStatus.set(Power);
  } else if ((command == F("mode")) && lookup(value, _mappings.mode, _wantedSettings.mode)) {
    _writeStatus.set(Mode);
  } else if ((command == F("fan")) && lookup(value, _mappings.fan, _wantedSettings.fan)) {
    _writeStatus.set(Fan);
  } else if ((command == F("vane")) && lookup(value, _mappings.vane, _wantedSettings.vane)) {
    _writeStatus.set(Vane);
  } else if ((command == F("widevane")) && lookup(value, _mappings.wideVane, _wantedSettings.wideVane)) {
    _writeStatus.set(WideVane);
  }

    # undef lookup
}

void P093_data_struct::setState(P093_data_struct::State newState) {
  if ((newState != _state) && shouldTransition(_state, newState)) {
    State currentState = _state;
    _state = newState;
    didTransition(currentState, newState);
  } else {
# ifdef PLUGIN_093_DEBUG
    addLog(LOG_LEVEL_DEBUG, String(F("M-AC: SS - ignoring ")) +
           stateToString(_state) + F(" -> ") + stateToString(newState));
# endif // ifdef PLUGIN_093_DEBUG
  }
}

bool P093_data_struct::shouldTransition(P093_data_struct::State from, P093_data_struct::State to) {
  switch (to) {
    case Connecting:
      return from == NotConnected || from == ReadTimeout;

    case Connected:
      return from == Connecting;

    case UpdatingStatus:
      return from == Connected || from == StatusUpdated || from == WaitingForScheduledStatusUpdate;

    case StatusUpdated:
      return from == UpdatingStatus;

    case ScheduleNextStatusUpdate:
      return from == StatusUpdated || from == SettingsApplied;

    case WaitingForScheduledStatusUpdate:
      return from == ScheduleNextStatusUpdate;

    case ApplyingSettings:
      return from == WaitingForScheduledStatusUpdate;

    case SettingsApplied:
      return from == ApplyingSettings;

    case ReadTimeout:
      return from == UpdatingStatus || from == ApplyingSettings || from == Connecting;

    default:
      return false;
  }
}

void P093_data_struct::didTransition(P093_data_struct::State from, P093_data_struct::State to) {
# ifdef PLUGIN_093_DEBUG
  addLog(LOG_LEVEL_DEBUG, String(F("M-AC: didTransition: ")) +
         stateToString(from) + " -> " + stateToString(to));
# endif // ifdef PLUGIN_093_DEBUG

  switch (to) {
    case ReadTimeout:

      // Try to connect using different boud rate if we don't get response while connecting.
      if (from == Connecting) {
        _fastBaudRate = !_fastBaudRate;
      }
      setState(Connecting);
      break;

    case Connecting:
      connect();
      break;

    case Connected:
      responseReceived();
      _infoModeIndex = 0;
      setState(UpdatingStatus);
      break;

    case UpdatingStatus:
      updateStatus();
      break;

    case StatusUpdated: {
      responseReceived();
      const int infoModeCount = _includeStatus ? sizeof(INFOMODE) : sizeof(INFOMODE) - 1;
      _infoModeIndex = (_infoModeIndex + 1) % infoModeCount;

      if (_infoModeIndex != 0) {
        setState(UpdatingStatus);
      } else {
        _valuesInitialized = true;
        setState(ScheduleNextStatusUpdate);
      }
      break;
    }

    case ScheduleNextStatusUpdate:
      _statusUpdateTimeout = millis() + 1000;
      setState(WaitingForScheduledStatusUpdate);
      break;

    case ApplyingSettings:
      // Let's be optimistic and apply local changes immediately so potential
      // read operation will fetch latest settings even if they are in the
      // process of being applied. If settings will fail to apply for some reason
      // then next status update will re-set correct values (values from AC unit).
      applySettingsLocally();
      applySettings();
      _writeStatus.clear();
      break;

    case SettingsApplied:
      responseReceived();
      setState(ScheduleNextStatusUpdate);
      break;

    default:
      break;
  }
}

void P093_data_struct::applySettingsLocally() {
  if (_writeStatus.isDirty(Power)) {
    _currentValues.power = _wantedSettings.power;
  }

  if (_writeStatus.isDirty(Mode)) {
    _currentValues.mode = _wantedSettings.mode;
  }

  if (_writeStatus.isDirty(Temperature)) {
    _currentValues.temperature = _wantedSettings.temperature;
  }

  if (_writeStatus.isDirty(Fan)) {
    _currentValues.fan = _wantedSettings.fan;
  }

  if (_writeStatus.isDirty(Vane)) {
    _currentValues.vane = _wantedSettings.vane;
  }

  if (_writeStatus.isDirty(WideVane)) {
    _currentValues.wideVane = _wantedSettings.wideVane;
  }
}

void P093_data_struct::cancelWaitingAndTransitTo(P093_data_struct::State state) {
  _statusUpdateTimeout = 0;
  setState(state);
}

void P093_data_struct::responseReceived() {
  _writeTimeout = 0;
}

void P093_data_struct::updateStatus() {
  addLog(LOG_LEVEL_DEBUG, String(F("M-AC: US: ")) + _infoModeIndex);

  uint8_t packet[PACKET_LEN] = { 0xfc, 0x42, 0x01, 0x30, 0x10 };

  packet[5] = INFOMODE[_infoModeIndex];
  memset(packet + 6, 0, 15);
  packet[21] = checkSum(packet, 21);

  sendPacket(packet, PACKET_LEN);
}

void P093_data_struct::applySettings() {
  uint8_t packet[PACKET_LEN] = { 0xfc, 0x41, 0x01, 0x30, 0x10, 0x01 };

  memset(packet + 6, 0, 15);

  if (_writeStatus.isDirty(Power)) {
    packet[8]  = _wantedSettings.power;
    packet[6] |= 0x01;
  }

  if (_writeStatus.isDirty(Mode)) {
    packet[9]  = _wantedSettings.mode;
    packet[6] |= 0x02;
  }

  if (_writeStatus.isDirty(Temperature)) {
    packet[6] |= 0x04;

    if (_tempMode) {
      packet[19] = static_cast<uint8_t>(_wantedSettings.temperature * 2.0f + 128.0f);
    } else {
      packet[10] = 31 - _wantedSettings.temperature;
    }
  }

  if (_writeStatus.isDirty(Fan)) {
    packet[11] = _wantedSettings.fan;
    packet[6] |= 0x08;
  }

  if (_writeStatus.isDirty(Vane)) {
    packet[12] = _wantedSettings.vane;
    packet[6] |= 0x10;
  }

  if (_writeStatus.isDirty(WideVane)) {
    packet[18] = _wantedSettings.wideVane | (_wideVaneAdj ? 0x80 : 0x00);
    packet[7] |= 0x01;
  }

  packet[21] = checkSum(packet, 21);

  sendPacket(packet, PACKET_LEN);
}

void P093_data_struct::connect() {
  addLog(LOG_LEVEL_DEBUG, String(F("M-AC: Connect ")) + getBaudRate());

  _serial.begin(getBaudRate(), SERIAL_8E1);
  const uint8_t buffer[] = { 0xfc, 0x5a, 0x01, 0x30, 0x02, 0xca, 0x01, 0xa8 };

  sendPacket(buffer, sizeof(buffer));
}

unsigned long P093_data_struct::getBaudRate() const {
  return _fastBaudRate ? 9600 : 2400;
}

void P093_data_struct::sendPacket(const uint8_t *packet, size_t size) {
# ifdef PLUGIN_093_DEBUG
  addLog(LOG_LEVEL_DEBUG_MORE, dumpOutgoingPacket(packet, size));
# endif // ifdef PLUGIN_093_DEBUG

  _serial.write(packet, size);
  _writeTimeout = millis() + 2000;
}

void P093_data_struct::addByteToReadBuffer(uint8_t value) {
  if (_readPos < READ_BUFFER_LEN) {
    _readBuffer[_readPos] = value;
    ++_readPos;
  } else {
    addLog(LOG_LEVEL_DEBUG, F("M-AC: ABTRB(0)"));
    _readPos = 0;
  }
}

bool P093_data_struct::readIncommingBytes() {
  if ((_writeTimeout != 0) && timeOutReached(_writeTimeout)) {
    _writeTimeout = 0;
    _readPos      = 0;
    setState(ReadTimeout);
    return false;
  }

  static const uint8_t DATA_LEN_INDEX = 4;

  while (_serial.available() > 0) {
    uint8_t value = _serial.read();

    if (_readPos == 0) {
      // Wait for start uint8_t.
      if (value == 0xfc) {
        addByteToReadBuffer(value);
      } else {
        addLog(LOG_LEVEL_DEBUG, String(F("M-AC: RIB(0) ")) + formatToHex(value));
      }
    } else if ((_readPos <= DATA_LEN_INDEX) || (_readPos <= DATA_LEN_INDEX + _readBuffer[DATA_LEN_INDEX])) {
      // Read header + data part - data length is at index 4.
      addByteToReadBuffer(value);
    } else {
      // Done, last uint8_t is checksum.
      uint8_t length = _readPos;
      _readPos = 0;
      return processIncomingPacket(_readBuffer, length, value);
    }
  }

  return false;
}

bool P093_data_struct::processIncomingPacket(const uint8_t *packet, uint8_t length, uint8_t checksum) {
  P093_data_struct::State state = checkIncomingPacket(packet, length, checksum);

  if (state == StatusUpdated) {
    static const uint8_t dataPartOffset = 5;

    if (length <= dataPartOffset) {
      return false;
    }

    Values values = _currentValues;

    if (parseValues(_readBuffer + dataPartOffset, length - dataPartOffset)) {
      setState(StatusUpdated);
      return values != _currentValues;
    }
  } else if (state != Invalid) {
    setState(state);
  }

  return false;
}

bool P093_data_struct::parseValues(const uint8_t *data, size_t length) {
  if (length == 0) {
    addLog(LOG_LEVEL_DEBUG, F("M-AC: PV(0)"));
    return false;
  }

  switch (data[0]) {
    case 0x02:

      if (length > 11) {
        _currentValues.power = data[3];
        _currentValues.iSee  = data[4] > 0x08 ? true : false;
        _currentValues.mode  = _currentValues.iSee ? (data[4] - 0x08) : data[4];

        if (data[11] != 0x00) {
          _currentValues.temperature = (static_cast<float>(data[11]) - 128.0f) / 2.0f;
          _tempMode                  = true;
        } else {
          _currentValues.temperature = 31 - data[5];
        }

        _currentValues.fan      = data[6];
        _currentValues.vane     = data[7];
        _currentValues.wideVane = data[10] & 0x0F;
        _wideVaneAdj            = (data[10] & 0xF0) == 0x80 ? true : false;

        return true;
      }
      break;

    case 0x03:

      if (length > 6) {
        if (data[6] != 0x00) {
          _currentValues.roomTemperature = (static_cast<float>(data[6]) - 128.0f) / 2.0f;
        } else {
          _currentValues.roomTemperature = data[3] + 10;
        }
        return true;
      }
      break;

    case 0x06:

      if (length > 4) {
        _currentValues.operating           = data[4];
        _currentValues.compressorFrequency = data[3];
        return true;
      }
      break;
  }

  addLog(LOG_LEVEL_DEBUG, F("M-AC: PV(1)"));
  return false;
}

P093_data_struct::State P093_data_struct::checkIncomingPacket(const uint8_t *packet, uint8_t length, uint8_t checksum) {
# ifdef PLUGIN_093_DEBUG
  addLog(LOG_LEVEL_DEBUG_MORE, dumpIncomingPacket(packet, length));
# endif // ifdef PLUGIN_093_DEBUG

  if ((packet[2] != 0x01) || (packet[3] != 0x30)) {
    addLog(LOG_LEVEL_DEBUG, F("M-AC: CIP(0)"));
    return Invalid;
  }

  uint8_t calculatedChecksum = checkSum(packet, length);

  if (calculatedChecksum != checksum) {
    addLog(LOG_LEVEL_DEBUG, String(F("M-AC: CIP(1) ")) + calculatedChecksum);
    return Invalid;
  }

  switch (packet[1]) {
    case 0x61:
      return SettingsApplied;
    case 0x62:
      return StatusUpdated;
    case 0x7a:
      return Connected;
    default:
      return Invalid;
  }
}

uint8_t P093_data_struct::checkSum(const uint8_t *bytes, size_t length) {
  uint8_t sum = 0;

  for (size_t i = 0; i < length; ++i) {
    sum += bytes[i];
  }
  return (0xfc - sum) & 0xff;
}

const __FlashStringHelper * P093_data_struct::findByValue(uint8_t value, const Tuple list[], size_t count) {
  for (size_t index = 0; index < count; ++index) {
    const Tuple& tuple = list[index];

    if (value == tuple.value) {
      return tuple.mapping;
    }
  }
  return list[0].mapping;
}

bool P093_data_struct::findByMapping(const String& mapping, const Tuple list[], size_t count, uint8_t& value) {
  for (size_t index = 0; index < count; ++index) {
    const Tuple& tuple = list[index];

    if (mapping == tuple.mapping) {
      value = tuple.value;
      return true;
    }
  }
  return false;
}

  # ifdef PLUGIN_093_DEBUG
const __FlashStringHelper * P093_data_struct::stateToString_f(P093_data_struct::State state) {
  switch (state) {
    case Invalid: return F("Invalid");
    case NotConnected: return F("NotConnected");
    case Connecting: return F("Connecting");
    case Connected: return F("Connected");
    case UpdatingStatus: return F("UpdatingStatus");
    case StatusUpdated: return F("StatusUpdated");
    case ReadTimeout: return F("ReadTimeout");
    case ScheduleNextStatusUpdate: return F("ScheduleNextStatusUpdate");
    case WaitingForScheduledStatusUpdate: return F("WaitingForScheduledStatusUpdate");
    case ApplyingSettings: return F("ApplyingSettings");
    case SettingsApplied: return F("SettingsApplied");
  }
  return F("");
}

String P093_data_struct::stateToString(P093_data_struct::State state) {
  String res = stateToString_f(state);

  if (res.isEmpty()) {
    return String(F("<unknown> ")) + state;
  }
  return res;
}

void P093_data_struct::dumpPacket(const uint8_t *packet, size_t length, String& result) {
  for (size_t idx = 0; idx < length; ++idx) {
    result += formatToHex(packet[idx], F(""));
    result += ' ';
  }
}

String P093_data_struct::dumpOutgoingPacket(const uint8_t *packet, size_t length) {
  String message = F("M-AC - OUT: ");

  dumpPacket(packet, length, message);
  return message;
}

String P093_data_struct::dumpIncomingPacket(const uint8_t *packet, int length) {
  String message = F("M-AC - IN: ");

  dumpPacket(packet, length, message);
  return message;
}

  # endif // ifdef PLUGIN_093_DEBUG


#endif // ifdef USES_P093

#include "../PluginStructs/P038_data_struct.h"

#ifdef USES_P038

// **************************************************************************/
// Constructor
// **************************************************************************/
P038_data_struct::P038_data_struct(int8_t gpioPin, uint16_t ledCount, uint8_t stripType)
  : _gpioPin(gpioPin), _maxPixels(ledCount), _stripType(stripType) {}

// **************************************************************************/
// Destructor
// **************************************************************************/
P038_data_struct::~P038_data_struct() {
  if (isInitialized()) {
    delete Plugin_038_pixels;
    Plugin_038_pixels = nullptr;
  }
}

bool P038_data_struct::plugin_init(struct EventStruct *event) {
  bool success = false;

  if (!isInitialized()) {
    Plugin_038_pixels = new (std::nothrow) Adafruit_NeoPixel(_maxPixels,
                                                             _gpioPin,
                                                             (_stripType == P038_STRIP_TYPE_RGBW ? NEO_GRBW : NEO_GRB) + NEO_KHZ800);

    if (Plugin_038_pixels != nullptr) {
      Plugin_038_pixels->begin(); // This initializes the NeoPixel library.
      success = true;
    }
  }

  return success;
}

bool P038_data_struct::plugin_exit(struct EventStruct *event) {
  if (isInitialized()) {
    delete Plugin_038_pixels;
    Plugin_038_pixels = nullptr;
  }
  return true;
}

bool P038_data_struct::plugin_write(struct EventStruct *event, const String& string) {
  bool success = false;

  if (isInitialized()) {
    const String cmd = parseString(string, 1);
    if (!cmd.startsWith(F("neo"))) {
      return success;
    }

    {
      String log;

      if (loglevelActiveFor(LOG_LEVEL_INFO) &&
          log.reserve(64)) {
        log += F("P038 : write - ");
        log += string;
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }

    if (cmd.equals(F("neopixel"))) { // NeoPixel
      Plugin_038_pixels->setPixelColor(event->Par1 - 1, Plugin_038_pixels->Color(event->Par2, event->Par3, event->Par4, event->Par5));
      Plugin_038_pixels->show();     // This sends the updated pixel color to the hardware.
      success = true;
    }

    // extra function to receive HSV values (i.e. homie controler)
    if (cmd.equals(F("neopixelhsv"))) { // NeoPixelHSV
      int rgbw[4];
      rgbw[3] = 0;

      HSV2RGBWorRGBandLog(event->Par2, event->Par3, event->Par4, rgbw);

      Plugin_038_pixels->setPixelColor(event->Par1 - 1, Plugin_038_pixels->Color(rgbw[0], rgbw[1], rgbw[2], rgbw[3]));
      Plugin_038_pixels->show(); // This sends the updated pixel color to the hardware.
      success = true;
    }

    if (cmd.equals(F("neopixelall"))) { // NeoPixelAll
      for (int i = 0; i < _maxPixels; i++) {
        Plugin_038_pixels->setPixelColor(i, Plugin_038_pixels->Color(event->Par1, event->Par2, event->Par3, event->Par4));
      }
      Plugin_038_pixels->show();
      success = true;
    }

    if (cmd.equals(F("neopixelallhsv"))) { // NeoPixelAllHSV
      int rgbw[4];
      rgbw[3] = 0;

      HSV2RGBWorRGBandLog(event->Par1, event->Par2, event->Par3, rgbw);

      for (int i = 0; i < _maxPixels; i++) {
        Plugin_038_pixels->setPixelColor(i, Plugin_038_pixels->Color(rgbw[0], rgbw[1], rgbw[2], rgbw[3]));
      }
      Plugin_038_pixels->show();
      success = true;
    }

    if (cmd.equals(F("neopixelline"))) {                      // NeoPixelLine
      int brightness = 0;
      validIntFromString(parseString(string, 7), brightness); // Get 7th argument aka Par6

      for (int i = event->Par1 - 1; i < event->Par2; i++) {
        Plugin_038_pixels->setPixelColor(i, Plugin_038_pixels->Color(event->Par3, event->Par4, event->Par5, brightness));
      }
      Plugin_038_pixels->show();
      success = true;
    }

    if (cmd.equals(F("neopixellinehsv"))) { // NeoPixelLineHSV
      int rgbw[4];
      rgbw[3] = 0;

      HSV2RGBWorRGBandLog(event->Par3, event->Par4, event->Par5, rgbw);

      for (int i = event->Par1 - 1; i < event->Par2; i++) {
        Plugin_038_pixels->setPixelColor(i, Plugin_038_pixels->Color(rgbw[0], rgbw[1], rgbw[2], rgbw[3]));
      }
      Plugin_038_pixels->show();
      success = true;
    }
  }
  return success;
}

void P038_data_struct::HSV2RGBWorRGBandLog(float H, float S, float V, int rgbw[4]) {
  if (_stripType == P038_STRIP_TYPE_RGBW) { // RGBW
    HSV2RGBW(H, S, V, rgbw);
  } else {                                  // RGB
    HSV2RGB(H, S, V, rgbw);
  }
  String log;

  if (loglevelActiveFor(LOG_LEVEL_INFO) &&
      log.reserve(48)) {
    log += F("P038 HSV converted to RGB(W):");
    log += rgbw[0];
    log += ',';
    log += rgbw[1];
    log += ',';
    log += rgbw[2];
    log += ',';
    log += rgbw[3];
    addLogMove(LOG_LEVEL_INFO, log);
  }
}

#endif // ifdef USES_P038


#ifdef USES_P127
#include "../PluginStructs/P127_data_struct.h"

// #######################################################################################################
// ################################## Plugin 127 I2C CDM7160 CO2 sensor ##################################
// #######################################################################################################
//
//

P127_data_struct::P127_data_struct(int8_t i2caddr)
  : _i2cAddress(i2caddr) {}

// Do all required initialization
bool P127_data_struct::init(uint16_t alt) {
  setPowerDown();

  // delay required to store config byte to EEPROM, device pulls SCL low
  delay(100);

  uint8_t elev = alt / 10; // Altitude is as 'finegrained' per 10 meter

  if (elev) {
    setAltitude(elev);
  } else {
    clearAltitude();
  }

  // delay required to store config byte to EEPROM, device pulls SCL low
  delay(100);

  // Start reading
  setContinuous();
  return true;
}

// Check status and read data if not busy
bool P127_data_struct::checkData() {
  uint8_t status = getStatus();

  if (!(status & CDM7160_FLAG_BUSY)) {
    _co2 = getCO2();
  }
  return true;
}

// Return the last measured value
uint16_t P127_data_struct::readData() {
  return _co2;
}

uint8_t P127_data_struct::getAltitude() {
  return I2C_read8_ST_reg(_i2cAddress, CDM7160_REG_HIT);
}

uint8_t P127_data_struct::getCompensation() {
  return I2C_read8_ST_reg(_i2cAddress, CDM7160_REG_FUNC);
}

bool P127_data_struct::setPowerDown(void)

// Set power down mode CDM7160 to enable correct settings modification
// Returns true (1) if successful, false (0) if there was an I2C error
{
  // Write 0x06 to control register
  return I2C_write8_reg(_i2cAddress, CDM7160_REG_CTL, CDM7160_FLAG_DOWN);
}

bool P127_data_struct::setContinuous(void)

// Set continuous operation mode CDM7160 to start measurements
// Returns true (1) if successful, false (0) if there was an I2C error
{
  // Write 0x00 to control register
  return I2C_write8_reg(_i2cAddress, CDM7160_REG_CTL, CDM7160_FLAG_CONT);
}

bool P127_data_struct::setReset(void)

// Reset CDM7160
// Returns true (1) if successful, false (0) if there was an I2C error
{
  // Write 0x01 to reset register
  return I2C_write8_reg(_i2cAddress, CDM7160_REG_RESET, CDM7160_FLAG_REST);
}

bool P127_data_struct::setAltitude(uint8_t alt)

// Set altitude compensation on CDM7160
// Returns true (1) if successful, false (0) if there was an I2C error
{
  // Write altitude and enable compensation
  I2C_write8_reg(_i2cAddress, CDM7160_REG_HIT, alt);
  return I2C_write8_reg(_i2cAddress, CDM7160_REG_FUNC, (CDM7160_FLAG_HPAE | CDM7160_FLAG_PWME));
}

bool P127_data_struct::clearAltitude(void)

// Disable altitude compensation on CDM7160
// Returns true (1) if successful, false (0) if there was an I2C error
{
  // Clear altitude and disable compensation
  I2C_write8_reg(_i2cAddress, CDM7160_REG_HIT, 0);
  return I2C_write8_reg(_i2cAddress, CDM7160_REG_FUNC, CDM7160_FLAG_PWME);
}

uint8_t P127_data_struct::getStatus()

// Retrieve CO2 data in ppm
// Returns true (1) if successful, false (0) if there was an I2C error
{
  // Get content of status register
  return I2C_read8_ST_reg(_i2cAddress, CDM7160_REG_STATUS);
}

uint16_t P127_data_struct::getCO2()

// Retrieve CO2 data in ppm
// Returns the value
{
  // Get co2 ppm data out of result registers
  return I2C_read16_LE_ST_reg(_i2cAddress, CDM7160_REG_DATA);
}

// Reads an 8 bit value from a register over I2C, no repeated start
uint8_t P127_data_struct::I2C_read8_ST_reg(uint8_t i2caddr, byte reg) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(i2caddr, (byte)1);
  return Wire.read();
}

// Reads a 16 bit value starting at a given register over I2C, no repeated start
uint16_t P127_data_struct::I2C_read16_LE_ST_reg(uint8_t i2caddr, byte reg) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(i2caddr, (byte)2);

  return (Wire.read()) | Wire.read() << 8;
}

#endif // ifdef USES_P127

#include "../PluginStructs/P105_data_struct.h"

#ifdef USES_P105

# include "../Helpers/Convert.h"

struct AHTx_Status {
  inline AHTx_Status(uint8_t stat) : status(stat) {}

  inline bool valid() const {
    return status != 0xFF;
  }

  inline bool calibrated() const {
    return (status & (1 << 3)) != 0;
  }

  inline bool busy() const {
    return (status & (1 << 7)) != 0;
  }

  const uint8_t status;
};

AHTx_Device::AHTx_Device(uint8_t addr, AHTx_device_type type) :
  i2cAddress(addr),
  device_type(type),
  last_hum_val(0.0f),
  last_temp_val(0.0f) {}

const __FlashStringHelper * AHTx_Device::getDeviceName() const {
  switch (device_type) {
    case AHTx_device_type::AHT10_DEVICE: return F("AHT10");
    case AHTx_device_type::AHT20_DEVICE: return F("AHT20");
    case AHTx_device_type::AHT21_DEVICE: return F("AHT21");
  }
  return F("AHTx");
}

bool AHTx_Device::initialize() {
  const uint8_t cmd_init = (device_type == AHTx_device_type::AHT10_DEVICE) ? 0xE1 : 0xBE;

  return I2C_write16_reg(i2cAddress, cmd_init, 0x0800);
}

bool AHTx_Device::triggerMeasurement() {
  return I2C_write16_reg(i2cAddress, 0xAC, 0x3300); // measurement time takes over 80 msec
}

bool AHTx_Device::softReset() {
  return I2C_write8(i2cAddress, 0xBA); // soft reset takes less than 20 msec
}

uint8_t AHTx_Device::readStatus() {
  return I2C_read8(i2cAddress, nullptr);
}

bool AHTx_Device::readData() {
  const uint8_t data_len = 6;

  // I2C_read8 len
  if (Wire.requestFrom(i2cAddress, data_len) == 0) {
    return false;
  }

  uint8_t data[data_len];

  for (uint8_t i = 0; i < data_len; ++i) {
    data[i] = Wire.read();
  }

  // check status
  AHTx_Status status = data[0];

  if (!(status.valid() && status.calibrated())) {
    return false;
  }

  // 20 bits humidity value
  uint32_t value = data[1];

  value        = (value << 8) | data[2];
  value        = (value << 4) | (data[3] >> 4);
  last_hum_val = (static_cast<float>(value) / (1 << 20)) * 100.0f;

  // 20 bits temperature value
  value         = data[3] & 0x0F;
  value         = (value << 8) | data[4];
  value         = (value << 8) | data[5];
  last_temp_val = ((static_cast<float>(value) / (1 << 20)) * 200.0f) - 50.0f;

  return true;
}

P105_data_struct::P105_data_struct(uint8_t addr, AHTx_device_type dev) :
  device(addr, dev),
  state(AHTx_state::AHTx_Uninitialized),
  last_measurement(0),
  trigger_time(0) {}

bool P105_data_struct::initialized() const {
  return state != AHTx_state::AHTx_Uninitialized;
}

void P105_data_struct::setUninitialized() {
  state = AHTx_state::AHTx_Uninitialized;
}

// Perform the measurements with interval
bool P105_data_struct::updateMeasurements(taskIndex_t task_index) {
  const unsigned long current_time = millis();

  if (!initialized()) {
    String log;
    log.reserve(30);

    if (!device.initialize()) {
      log += getDeviceName();
      log += F(" : unable to initialize");
      addLogMove(LOG_LEVEL_ERROR, log);
      return false;
    }
    log  = getDeviceName();
    log += F(" : initialized");
    addLogMove(LOG_LEVEL_INFO, log);

    trigger_time = current_time;
    state        = AHTx_state::AHTx_Trigger_measurement;
    return false;
  }

  if ((state != AHTx_state::AHTx_Wait_for_samples) && (state != AHTx_state::AHTx_Trigger_measurement)) {
    if (!timeOutReached(last_measurement + (Settings.TaskDeviceTimer[task_index] * 1000))) {
      // Timeout has not yet been reached.
      return false;
    }
    trigger_time = current_time;
    state        = AHTx_state::AHTx_Trigger_measurement;
  }

  // state: AHTx_Wait_for_samples or AHTx_Trigger_measurement
  AHTx_Status status = device.readStatus();

  if (status.valid() && status.calibrated() && !status.busy()) {
    if (state == AHTx_state::AHTx_Trigger_measurement) {
      device.triggerMeasurement();

      trigger_time = current_time;
      state        = AHTx_state::AHTx_Wait_for_samples;
      return false;
    }

    // state: AHTx_Wait_for_samples
    if (!device.readData()) {
      return false;
    }

    last_measurement = current_time;
    state            = AHTx_state::AHTx_New_values;

    #ifndef BUILD_NO_DEBUG
    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) { // Log raw measuerd values only on level DEBUG
      String log;
      log.reserve(50);                        // Prevent re-allocation
      log += getDeviceName();
      log += F(" : humidity ");
      log += device.getHumidity();
      log += F("% temperature ");
      log += device.getTemperature();
      log += 'C';
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
    #endif

    return true;
  }

  if (timePassedSince(trigger_time) > 1000) {
    // should not happen
    String log;
    log.reserve(15); // Prevent re-allocation
    log += getDeviceName();
    log += F(" : reset");
    addLogMove(LOG_LEVEL_ERROR, log);
    device.softReset();

    state = AHTx_state::AHTx_Uninitialized;
  }

  return false;
}

#endif // ifdef USES_P105

#include "../PluginStructs/P107_data_struct.h"

#ifdef USES_P107

// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter
# include <Adafruit_SI1145.h>

bool P107_data_struct::begin()
{
  return uv.begin();
}

#endif // ifdef USES_P107

#include "../PluginStructs/P049_data_struct.h"

#ifdef USES_P049


// 9 uint8_t commands:
// mhzCmdReadPPM[]              = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
// mhzCmdCalibrateZero[]        = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};
// mhzCmdABCEnable[]            = {0xFF,0x01,0x79,0xA0,0x00,0x00,0x00,0x00,0xE6};
// mhzCmdABCDisable[]           = {0xFF,0x01,0x79,0x00,0x00,0x00,0x00,0x00,0x86};
// mhzCmdReset[]                = {0xFF,0x01,0x8d,0x00,0x00,0x00,0x00,0x00,0x72};

/* It seems the offsets [3]..[4] for the detection range setting (command uint8_t 0x99) are wrong in the latest
 * online data sheet: http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf
 * According to the MH-Z19B datasheet version 1.2, valid from: 2017.03.22 (received 2018-03-07)
 * the offset should be [6]..[7] instead.
 * 0x99 - Detection range setting, send command:
 * /---------+---------+---------+---------+---------+---------+---------+---------+---------\
 * | Byte 0  | Byte 1  | Byte 2  | Byte 3  | Byte 4  | Byte 5  | Byte 6  | Byte 7  | Byte 8  |
 * |---------+---------+---------+---------+---------+---------+---------+---------+---------|
 * | Start   | Reserved| Command | Reserved|Detection|Detection|Detection|Detection| Checksum|
 * | Byte    |         |         |         |range    |range    |range    |range    |         |
 * |         |         |         |         |24~32 bit|16~23 bit|8~15 bit |0~7 bit  |         |
 * |---------+---------+---------+---------+---------+---------+---------+---------+---------|
 * | 0xFF    | 0x01    | 0x99    | 0x00    | Data 1  | Data 2  | Data 3  | Data 4  | Checksum|
 * \---------+---------+---------+---------+---------+---------+---------+---------+---------/
 * Note: Detection range should be 0~2000, 0~5000, 0~10000 ppm.
 * For example: set 0~2000 ppm  detection range, send command: FF 01 99 00 00 00 07 D0 8F
 *              set 0~10000 ppm detection range, send command: FF 01 99 00 00 00 27 10 8F
 * The latter, updated version above is implemented here.
 */

// mhzCmdMeasurementRange1000[] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x03,0xE8,0x7B};
// mhzCmdMeasurementRange2000[] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x07,0xD0,0x8F};
// mhzCmdMeasurementRange3000[] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x0B,0xB8,0xA3};
// mhzCmdMeasurementRange5000[] = {0xFF,0x01,0x99,0x00,0x00,0x00,0x13,0x88,0xCB};
// Removing redundant data, just keeping offsets [2], [6]..[7]:
const PROGMEM uint8_t mhzCmdData[][3] = {
  { 0x86, 0x00, 0x00 },
  { 0x87, 0x00, 0x00 },
  { 0x79, 0xA0, 0x00 },
  { 0x79, 0x00, 0x00 },
  { 0x8d, 0x00, 0x00 },
# ifdef ENABLE_DETECTION_RANGE_COMMANDS
  { 0x99, 0x03, 0xE8 },
  { 0x99, 0x07, 0xD0 },
  { 0x99, 0x0B, 0xB8 },
  { 0x99, 0x13, 0x88 }
# endif // ifdef ENABLE_DETECTION_RANGE_COMMANDS
};


P049_data_struct::P049_data_struct() {
  reset();
  sensorResets = 0;
}

P049_data_struct::~P049_data_struct() {
  reset();
}

void P049_data_struct::reset() {
  if (easySerial != nullptr) {
    delete easySerial;
    easySerial = nullptr;
  }
  linesHandled       = 0;
  checksumFailed     = 0;
  nrUnknownResponses = 0;
  ++sensorResets;

  // Default of the sensor is to run ABC
  ABC_Disable     = false;
  ABC_MustApply   = false;
  modelA_detected = false;
}

bool P049_data_struct::init(ESPEasySerialPort port, const int16_t serial_rx, const int16_t serial_tx, bool setABCdisabled) {
  if ((serial_rx < 0) || (serial_tx < 0)) {
    return false;
  }
  reset();
  easySerial = new (std::nothrow) ESPeasySerial(port, serial_rx, serial_tx);

  if (easySerial == nullptr) {
    return false;
  }
  easySerial->begin(9600);
  ABC_Disable = setABCdisabled;

  if (ABC_Disable) {
    // No guarantee the correct state is active on the sensor after reboot.
    ABC_MustApply = true;
  }
  lastInitTimestamp = millis();
  initTimePassed    = false;
  return isInitialized();
}

bool P049_data_struct::plugin_write(struct EventStruct *event, const String& string)
{
  String command = parseString(string, 1);

  if (command == F("mhzcalibratezero"))
  {
    send_mhzCmd(mhzCmdCalibrateZero);
    addLog(LOG_LEVEL_INFO, F("MHZ19: Calibrated zero point!"));
    return true;
  }
  else if (command == F("mhzreset"))
  {
    send_mhzCmd(mhzCmdReset);
    addLog(LOG_LEVEL_INFO, F("MHZ19: Sent sensor reset!"));
    return true;
  }
  else if (command == F("mhzabcenable"))
  {
    send_mhzCmd(mhzCmdABCEnable);
    addLog(LOG_LEVEL_INFO, F("MHZ19: Sent sensor ABC Enable!"));
    return true;
  }
  else if (command == F("mhzabcdisable"))
  {
    send_mhzCmd(mhzCmdABCDisable);
    addLog(LOG_LEVEL_INFO, F("MHZ19: Sent sensor ABC Disable!"));
    return true;
  }

# ifdef ENABLE_DETECTION_RANGE_COMMANDS
  else if (command.startsWith(F("mhzmeasurementrange"))) {
    if (command == F("mhzmeasurementrange1000"))
    {
      send_mhzCmd(mhzCmdMeasurementRange1000);
      addLog(LOG_LEVEL_INFO, F("MHZ19: Sent measurement range 0-1000PPM!"));
      return true;
    }
    else if (command == F("mhzmeasurementrange2000"))
    {
      send_mhzCmd(mhzCmdMeasurementRange2000);
      addLog(LOG_LEVEL_INFO, F("MHZ19: Sent measurement range 0-2000PPM!"));
      return true;
    }
    else if (command == F("mhzmeasurementrange3000"))
    {
      send_mhzCmd(mhzCmdMeasurementRange3000);
      addLog(LOG_LEVEL_INFO, F("MHZ19: Sent measurement range 0-3000PPM!"));
      return true;
    }
    else if (command == F("mhzmeasurementrange5000"))
    {
      send_mhzCmd(mhzCmdMeasurementRange5000);
      addLog(LOG_LEVEL_INFO, F("MHZ19: Sent measurement range 0-5000PPM!"));
      return true;
    }
  }
# endif // ENABLE_DETECTION_RANGE_COMMANDS
  return false;
}

void P049_data_struct::setABCmode(int abcDisableSetting) {
  const bool new_ABC_disable = (abcDisableSetting == P049_ABC_disabled);

  if (ABC_Disable != new_ABC_disable) {
    // Setting changed in the webform.
    ABC_MustApply = true;
    ABC_Disable   = new_ABC_disable;
  }
}

uint8_t P049_data_struct::calculateChecksum() const {
  uint8_t checksum = 0;

  for (uint8_t i = 1; i < 8; i++) {
    checksum += mhzResp[i];
  }
  checksum = 0xFF - checksum;
  return checksum + 1;
}

size_t P049_data_struct::send_mhzCmd(uint8_t CommandId)
{
  if (!isInitialized()) { return 0; }

  // The receive buffer "mhzResp" is re-used to send a command here:
  mhzResp[0] = 0xFF; // Start uint8_t, fixed
  mhzResp[1] = 0x01; // Sensor number, 0x01 by default
  memcpy_P(&mhzResp[2], mhzCmdData[CommandId], sizeof(mhzCmdData[0]));
  mhzResp[6] = mhzResp[3]; mhzResp[7] = mhzResp[4];
  mhzResp[3] = mhzResp[4] = mhzResp[5] = 0x00;
  mhzResp[8] = calculateChecksum();

  if (!initTimePassed) {
    // Allow for 3 minutes of init time.
    initTimePassed = timePassedSince(lastInitTimestamp) > 180000;
  }

  return easySerial->write(mhzResp, sizeof(mhzResp));
}

bool P049_data_struct::read_ppm(unsigned int& ppm, signed int& temp, unsigned int& s, float& u) {
  if (!isInitialized()) { return false; }

  // send read PPM command
  uint8_t nbBytesSent = send_mhzCmd(mhzCmdReadPPM);

  if (nbBytesSent != 9) {
    return false;
  }

  // get response
  memset(mhzResp, 0, sizeof(mhzResp));

  long timer   = millis() + PLUGIN_READ_TIMEOUT;
  int  counter = 0;

  while (!timeOutReached(timer) && (counter < 9)) {
    if (easySerial->available() > 0) {
      uint8_t value = easySerial->read();

      if (((counter == 0) && (value == 0xFF)) || (counter > 0)) {
        mhzResp[counter++] = value;
      }
    } else {
      delay(10);
    }
  }

  if (counter < 9) {
    // Timeout
    return false;
  }
  ++linesHandled;

  if (!(mhzResp[8] == calculateChecksum())) {
    ++checksumFailed;
    return false;
  }

  if ((mhzResp[0] == 0xFF) && (mhzResp[1] == 0x86)) {
    // calculate CO2 PPM
    ppm = (static_cast<unsigned int>(mhzResp[2]) << 8) + mhzResp[3];

    // set temperature (offset 40)
    unsigned int mhzRespTemp = (unsigned int)mhzResp[4];
    temp = mhzRespTemp - 40;

    // set 's' (stability) value
    s = mhzResp[5];

    if (s != 0) {
      modelA_detected = true;
    }

    // calculate 'u' value
    u = (static_cast<unsigned int>(mhzResp[6]) << 8) + mhzResp[7];
    return true;
  }
  return false;
}

bool P049_data_struct::receivedCommandAcknowledgement(bool& expectReset) {
  expectReset = false;

  if (mhzResp[0] == 0xFF)  {
    switch (mhzResp[1]) {
      case 0x86: // Read CO2 concentration
      case 0x79: // ON/OFF Auto Calibration
        break;
      case 0x87: // Calibrate Zero Point (ZERO)
      case 0x88: // Calibrate Span Point (SPAN)
      case 0x99: // Detection range setting
        expectReset = true;
        break;
      default:
        ++nrUnknownResponses;
        return false;
    }
    uint8_t checksum = calculateChecksum();
    return mhzResp[8] == checksum;
  }
  ++nrUnknownResponses;
  return false;
}

String P049_data_struct::getBufferHexDump() const {
  String result;

  result.reserve(27);

  for (int i = 0; i < 9; ++i) {
    result += ' ';
    result += String(mhzResp[i], HEX);
  }
  return result;
}

MHZ19Types P049_data_struct::getDetectedDevice() const {
  if (linesHandled > checksumFailed) {
    return modelA_detected ? MHZ19_A : MHZ19_B;
  }
  return MHZ19_notDetected;
}

bool Plugin_049_Check_and_ApplyFilter(unsigned int prevVal, unsigned int& newVal, uint32_t s, const int filterValue, String& log) {
  if (s == 1) {
    // S==1 => "A" version sensor bootup, do not use values.
    return false;
  }

  if ((prevVal < 400) || (prevVal > 5000)) {
    // Prevent unrealistic values during start-up with filtering enabled.
    // Just assume the entered value is correct.
    return true;
  }
  bool filterApplied = filterValue > PLUGIN_049_FILTER_OFF_ALLSAMPLES;
  int32_t difference = newVal - prevVal;

  if ((s > 0) && (s < 64) && (filterValue != PLUGIN_049_FILTER_OFF)) {
    // Not the "B" version of the sensor, S value is used.
    // S==0 => "B" version, else "A" version
    // The S value is an indication of the stability of the reading.
    // S == 64 represents a stable reading and any lower value indicates (unusual) fast change.
    // Now we increase the delay filter for low values of S and increase response time when the
    // value is more stable.
    // This will make the reading useful in more turbulent environments,
    // where the sensor would report more rapid change of measured values.
    difference    = difference * s;
    difference   /= 64;
    log          += F("Compensate Unstable ");
    filterApplied = true;
  }

  switch (filterValue) {
    case PLUGIN_049_FILTER_OFF: {
      if ((s != 0) && (s != 64)) {
        log += F("Skip Unstable ");
        return false;
      }
      filterApplied = false;
      break;
    }

    // #Samples to reach >= 75% of step response
    case PLUGIN_049_FILTER_OFF_ALLSAMPLES: filterApplied = false; break; // No Delay
    case PLUGIN_049_FILTER_FAST:    difference          /= 2; break;     // Delay: 2 samples
    case PLUGIN_049_FILTER_MEDIUM:  difference          /= 4; break;     // Delay: 5 samples
    case PLUGIN_049_FILTER_SLOW:    difference          /= 8; break;     // Delay: 11 samples
  }

  if (filterApplied) {
    log += F("Raw PPM: ");
    log += newVal;
    log += F(" Filtered ");
  }
  newVal = static_cast<unsigned int>(prevVal + difference);
  return true;
}

void P049_html_show_stats(struct EventStruct *event) {
  P049_data_struct *P049_data =
    static_cast<P049_data_struct *>(getPluginTaskData(event->TaskIndex));

  if (nullptr == P049_data) {
    return;
  }

  addRowLabel(F("Checksum (pass/fail/reset)"));
  addHtmlInt(P049_data->linesHandled);
  addHtml('/');
  addHtmlInt(P049_data->checksumFailed);
  addHtml('/');
  addHtmlInt(P049_data->sensorResets);

  addRowLabel(F("Detected"));

  switch (P049_data->getDetectedDevice()) {
    case MHZ19_A: addHtml(F("MH-Z19A")); break;
    case MHZ19_B: addHtml(F("MH-Z19B")); break;
    default: addHtml(F("---")); break;
  }
}

bool P049_perform_init(struct EventStruct *event) {
  bool success                 = false;
  const int16_t serial_rx      = CONFIG_PIN1;
  const int16_t serial_tx      = CONFIG_PIN2;
  const ESPEasySerialPort port = static_cast<ESPEasySerialPort>(CONFIG_PORT);
  P049_data_struct *P049_data  =
    static_cast<P049_data_struct *>(getPluginTaskData(event->TaskIndex));

  if (nullptr != P049_data) {
    if (P049_data->init(port, serial_rx, serial_tx, (PCONFIG(0) == P049_ABC_disabled))) {
      success = true;
      addLog(LOG_LEVEL_INFO, F("MHZ19: Init OK "));

      // delay first read, because hardware needs to initialize on cold boot
      // otherwise we get a weird value or read error
      Scheduler.schedule_task_device_timer(event->TaskIndex, millis() + 15000);
    }
  }
  return success;
}

#endif // ifdef USES_P049

#include "../PluginStructs/P110_data_struct.h"

#ifdef USES_P110

P110_data_struct::P110_data_struct(uint8_t i2c_addr, int timing, bool range) : i2cAddress(i2c_addr), timing(timing), range(range) {}

//**************************************************************************/
// Initialize VL53L0X
//**************************************************************************/
bool P110_data_struct::begin() {

  initState = true;

  sensor.setAddress(i2cAddress); // Initialize for configured address

  if (!sensor.init()) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("VL53L0X: Sensor not found, init failed for 0x");
      log += String(i2cAddress, HEX);
      addLogMove(LOG_LEVEL_INFO, log);
      addLog(LOG_LEVEL_INFO, sensor.getInitResult());
    }
    initState = false;
    return initState;
  }

  sensor.setTimeout(500);

  if (range) {
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  }

  sensor.setMeasurementTimingBudget(timing * 1000);

  delay(timing + 50);

  return initState;
}


long P110_data_struct::readDistance() {

  long dist = -1; // Invalid

#if defined(P110_DEBUG) || defined (P110_DEBUG_DEBUG)
  String log;
#endif
#ifdef P110_DEBUG_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    log  = F("VL53L0X  : idx: 0x");
    log += String(i2cAddress, HEX);
    log += F(" init: ");
    log += String(initState, BIN);
    addLogMove(LOG_LEVEL_DEBUG, log);
  }
#endif // P110_DEBUG_DEBUG

  if (initState) {
    success = true;
    dist = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) {
      #ifdef P110_DEBUG_DEBUG
      addLog(LOG_LEVEL_DEBUG, F("VL53L0X: TIMEOUT"));
      #endif // P110_DEBUG_DEBUG
      success = false;
    } else if ( dist >= 8190 ) {
      #ifdef P110_DEBUG_DEBUG
      addLog(LOG_LEVEL_DEBUG, F("VL53L0X: NO MEASUREMENT"));
      #endif // P110_DEBUG_DEBUG
      success = false;
    }

#ifdef P110_DEBUG
    log = F("VL53L0X: Address: 0x");
    log += String(i2cAddress, HEX);
    log += F(" / Timing: ");
    log += String(timing, DEC);
    log += F(" / Long Range: ");
    log += String(range, BIN);
    log += F(" / Distance: ");
    log += dist;
    addLogMove(LOG_LEVEL_INFO, log);
#endif // P110_DEBUG
  }
  return dist;
};

bool P110_data_struct::isReadSuccessful() {
  return success;
};

#endif // ifdef USES_P110

#include "../PluginStructs/P092_data_struct.h"

#ifdef USES_P092

//
// DLBus reads and decodes the DL-Bus.
// The DL-Bus is used in heating control units e.g. sold by Technische Alternative (www.ta.co.at).
// Author uwekaditz

// #define DLbus_DEBUG

// Flags for pulse width (bit 0 is the content!)
#define DLbus_FlagSingleWidth                 0x02
#define DLbus_FlagDoubleWidth                 0x04
#define DLbus_FlagShorterThanSingleWidth      0x10
#define DLbus_FlagBetweenDoubleSingleWidth    0x20
#define DLbus_FlagLongerThanDoubleWidth       0x40
#define DLbus_FlagLongerThanTwiceDoubleWidth  0x80
#define DLbus_FlagsWrongTiming                (DLbus_FlagLongerThanTwiceDoubleWidth | DLbus_FlagLongerThanDoubleWidth | \
                                               DLbus_FlagBetweenDoubleSingleWidth | DLbus_FlagShorterThanSingleWidth)

// Helper for ISR call
DLBus *DLBus::__instance                         = nullptr;
volatile  static uint8_t *ISR_PtrChangeBitStream = nullptr; // pointer to received bit change stream

DLBus::DLBus()
{
  if (__instance == nullptr)
  {
    __instance             = this;
    ISR_PtrChangeBitStream = DLbus_ChangeBitStream;
    addLog(LOG_LEVEL_INFO, F("Class DLBus created"));
  }
}

DLBus::~DLBus()
{
  if (__instance == this)
  {
    __instance             = nullptr;
    ISR_PtrChangeBitStream = nullptr;
    addLog(LOG_LEVEL_INFO, F("Class DLBus destroyed"));
  }
}

void DLBus::AddToInfoLog(const String& string)
{
  if ((IsLogLevelInfo) && (LogLevelInfo != 0xff)) {
    addLog(LogLevelInfo, string);
  }
}

void DLBus::AddToErrorLog(const String& string)
{
  if (LogLevelError != 0xff) {
    addLog(LogLevelError, string);
  }
}

void DLBus::attachDLBusInterrupt(void)
{
  ISR_Receiving = false;
  IsISRset = true;
  IsNoData = false;
  attachInterrupt(digitalPinToInterrupt(ISR_DLB_Pin), ISR, CHANGE);
}

void DLBus::StartReceiving(void)
{
  noInterrupts(); // make sure we don't get interrupted before we are ready
  ISR_PulseCount      = 0;
  ISR_Receiving       = (ISR_PtrChangeBitStream != nullptr);
  ISR_AllBitsReceived = false;
  interrupts(); // interrupts allowed now, next instruction WILL be executed
}

void IRAM_ATTR DLBus::ISR(void)
{
  if (__instance)
  {
    __instance->ISR_PinChanged();
  }
}

void IRAM_ATTR DLBus::ISR_PinChanged(void)
{
//  long TimeDiff = usecPassedSince(ISR_TimeLastBitChange); // time difference to previous pulse in Âµs
  uint32_t _now = micros();
  int32_t TimeDiff = (int32_t)(_now - ISR_TimeLastBitChange);

//  ISR_TimeLastBitChange = micros();                           // save last pin change time
  ISR_TimeLastBitChange = _now;                           // save last pin change time

  if (ISR_Receiving) {
    uint8_t val = digitalRead(ISR_DLB_Pin);               // read state

    // check pulse width
    if (TimeDiff >= 2 * ISR_MinDoublePulseWidth) {
      val |= DLbus_FlagLongerThanTwiceDoubleWidth; // longer then 2x double pulse width
    }
    else if (TimeDiff > ISR_MaxDoublePulseWidth) {
      val |= DLbus_FlagLongerThanDoubleWidth;      // longer then double pulse width
    }
    else if (TimeDiff >= ISR_MinDoublePulseWidth) {
      val |= DLbus_FlagDoubleWidth;                // double pulse width
    }
    else if (TimeDiff > ISR_MaxPulseWidth) {
      val |= DLbus_FlagBetweenDoubleSingleWidth;   // between double and single pulse width
    }
    else if (TimeDiff < ISR_MinPulseWidth) {
      val |= DLbus_FlagShorterThanSingleWidth;     // shorter then single pulse width
    }
    else {
      val |= DLbus_FlagSingleWidth;                // single pulse width
    }

    if (ISR_PulseCount < 2) {
      // check if sync is received
      if (val & DLbus_FlagLongerThanTwiceDoubleWidth) {
        // sync received
        *ISR_PtrChangeBitStream       = !(val & 0x01);
        *(ISR_PtrChangeBitStream + 1) = val;
        ISR_PulseCount                = 2;
      }
      else {
        ISR_PulseCount = 1; // flag that interrupt is receiving
      }
    }
    else {
      *(ISR_PtrChangeBitStream + ISR_PulseCount) = val;         // set bit
      ISR_PulseCount++;
      ISR_Receiving       = (ISR_PulseCount < ISR_PulseNumber); // stop P092_receiving when data frame is complete
      ISR_AllBitsReceived = !ISR_Receiving;
    }
  }
}

boolean DLBus::CheckTimings(void) {
  uint8_t rawval, val;
  uint8_t WrongTimeCnt = 0;
  int     i;

#ifdef DLbus_DEBUG
  uint16_t WrongTimingArray[5][6];
#endif // DLbus_DEBUG

  //  AddToInfoLog(F("Receive stopped."));

  ISR_PulseCount = 0;

  for (i = 0; i <= ISR_PulseNumber; i++) {
    // store DLbus_ChangeBitStream into ByteStream
    rawval = *(ISR_PtrChangeBitStream + i);

    if (rawval & DLbus_FlagsWrongTiming) {
      // wrong DLbus_time_diff
      if (ISR_PulseCount > 0) {
#ifdef DLbus_DEBUG
        WrongTimingArray[WrongTimeCnt][0] = i;
        WrongTimingArray[WrongTimeCnt][1] = ISR_PulseCount;
        WrongTimingArray[WrongTimeCnt][2] = BitNumber;
        WrongTimingArray[WrongTimeCnt][3] = rawval;
#endif // DLbus_DEBUG

        if ((rawval == DLbus_FlagLongerThanTwiceDoubleWidth) && (*(ISR_PtrChangeBitStream + i - 1) == (DLbus_FlagDoubleWidth | 0x01))) {
          // Add two additional short pulses (low and high), previous bit is High and contains DLbus_FlagDoubleWidth
          ProcessBit(0);
          ProcessBit(1);
#ifdef DLbus_DEBUG
          WrongTimingArray[WrongTimeCnt][4] = DLbus_FlagSingleWidth;
          WrongTimingArray[WrongTimeCnt][5] = DLbus_FlagSingleWidth + 1;
#endif // DLbus_DEBUG
        }
#ifdef DLbus_DEBUG
        else {
          WrongTimingArray[WrongTimeCnt][4] = 0xff;
          WrongTimingArray[WrongTimeCnt][5] = 0xff;
        }
#endif // DLbus_DEBUG
        WrongTimeCnt++;

        if (WrongTimeCnt >= 5) {
          return false;
        }
      }
    }
    else {
      val = rawval & 0x01;

      if ((rawval & DLbus_FlagDoubleWidth) == DLbus_FlagDoubleWidth) {
        // double pulse width
        ProcessBit(!val);
        ProcessBit(val);
      }
      else {
        // single pulse width
        ProcessBit(val);
      }
    }
  }

  //  AddToInfoLog(F("DLbus_ChangeBitStream copied."));

#ifdef DLbus_DEBUG

  if (WrongTimeCnt > 0) {
    if (IsLogLevelInfo) {
      String log = F("Wrong Timings: ");
      AddToInfoLog(log);

      for (i = 0; i < WrongTimeCnt; i++) {
        log  = i + 1;
        log += F(": PulseCount:");
        log += WrongTimingArray[i][1];
        log += F(": BitCount:");
        log += WrongTimingArray[i][2];
        log += F(" Value:0x");
        log += String(WrongTimingArray[i][3], HEX);
        log += F(" ValueBefore:0x");
        log += String(*(ISR_PtrChangeBitStream + WrongTimingArray[i][0] - 1), HEX);
        log += F(" ValueAfter:0x");
        log += String(*(ISR_PtrChangeBitStream + WrongTimingArray[i][0] + 1), HEX);

        if (WrongTimingArray[i][4] != 0xff) {
          log += F(" Added:0x");
          log += String(WrongTimingArray[i][4], HEX);
        }

        if (WrongTimingArray[i][5] != 0xff) {
          log += F(" Added:0x");
          log += String(WrongTimingArray[i][5], HEX);
        }
        AddToInfoLog(log);
      }
    }
  }
#endif // DLbus_DEBUG
  return true;
}

void DLBus::ProcessBit(uint8_t b) {
  // ignore first pulse
  ISR_PulseCount++;

  if (ISR_PulseCount % 2) {
    return;
  }
  BitNumber = (ISR_PulseCount / 2);

  if (b) {
    ByteStream[BitNumber / 8] |= (1 << (BitNumber % 8));  // set bit
  }
  else {
    ByteStream[BitNumber / 8] &= ~(1 << (BitNumber % 8)); // clear bit
  }
}

boolean DLBus::Processing(void) {
  boolean inverted = false;
  int16_t StartBit; // first bit of data frame (-1 not recognized)
  String  log;

  AddToInfoLog(F("Processing..."));
  StartBit = Analyze(); // find the data frame's beginning

  // inverted signal?
  while (StartBit == -1) {
    if (inverted) {
      AddToErrorLog(F("Error: Already inverted!"));
      return false;
    }
    Invert(); // invert again
    inverted = true;
    StartBit = Analyze();

    if (StartBit == -1) {
      AddToErrorLog(F("Error: No data frame available!"));
      return false;
    }
    uint16_t RequiredBitStreamLength = (ISR_PulseNumber - DLBus_ReserveBytes) / DLBus_BitChangeFactor;

    if ((BitNumber - StartBit) < RequiredBitStreamLength) {
      // no complete data frame available (difference between start_bit and received bits is < RequiredBitStreamLength)
      AddToErrorLog(F("Start bit too close to end of stream!"));

      if (IsLogLevelInfo) {
        log  = F("# Required bits: ");
        log += RequiredBitStreamLength;
        log += F(" StartBit: ");
        log += StartBit;
        log += F(" / EndBit: ");
        log += BitNumber;
        AddToInfoLog(log);
      }
      return false;
    }
  }

  if (IsLogLevelInfo) {
    log  = F("StartBit: ");
    log += StartBit;
    log += F(" / EndBit: ");
    log += BitNumber;
    AddToInfoLog(log);
  }
  Trim(StartBit);      // remove start and stop bits

  if (CheckDevice()) { // check connected device
    return true;
  }
  else {
    AddToErrorLog(F("Error: Device not found!"));
    return false;
  }
}

int DLBus::Analyze(void) {
  uint8_t sync = 0;

  // find SYNC (16 * sequential 1)
  for (int i = 0; i < BitNumber; i++) {
    if (ReadBit(i)) {
      sync++;
    }
    else {
      sync = 0;
    }

    if (sync == DLBus_SyncBits) {
      // finde erste 0 // find first 0
      while (ReadBit(i) == 1) {
        i++;
      }
      return i; // beginning of data frame
    }
  }

  // no data frame available. check signal?
  return -1;
}

void DLBus::Invert(void) {
  AddToInfoLog(F("Invert bit stream..."));

  for (int i = 0; i < BitNumber; i++) {
    WriteBit(i, ReadBit(i) ? 0 : 1); // invert every bit
  }
}

uint8_t DLBus::ReadBit(int pos) {
  int row = pos / 8;                          // detect position in bitmap
  int col = pos % 8;

  return ((ByteStream[row]) >> (col)) & 0x01; // return bit
}

void DLBus::WriteBit(int pos, uint8_t set) {
  int row = pos / 8; // detect position in bitmap
  int col = pos % 8;

  if (set) {
    ByteStream[row] |= 1 << col;    // set bit
  }
  else {
    ByteStream[row] &= ~(1 << col); // clear bit
  }
}

void DLBus::Trim(int start_bit) {
  for (int i = start_bit, bit = 0; i < BitNumber; i++) {
    int offset = i - start_bit;

    // ignore start and stop bits:
    // start bits: 0 10 20 30, also  x    % 10 == 0
    // stop bits:  9 19 29 39, also (x+1) % 10 == 0
    if (offset % 10 && (offset + 1) % 10) {
      WriteBit(bit, ReadBit(i));
      bit++;
    }
  }
}

boolean DLBus::CheckDevice(void) {
  // Data frame of a device?
  if (ByteStream[0] == DeviceBytes[0]) {
    if ((DeviceBytes[1] == 0) || (ByteStream[1] == DeviceBytes[1])) {
      return true;
    }
  }

  if (IsLogLevelInfo) {
    String log = F("# Received DeviceByte(s): 0x");
    log += String(ByteStream[0], HEX);

    if (DeviceBytes[1] != 0) {
      log += String(ByteStream[1], HEX);
    }
    log += F(" Requested: 0x");
    log += String(DeviceBytes[0], HEX);

    if (DeviceBytes[1] != 0) {
      log += String(DeviceBytes[1], HEX);
    }
    AddToInfoLog(log);
  }
  return false;
}

boolean DLBus::CheckCRC(uint8_t IdxCRC) {
  // CRC check sum
  if (IdxCRC == 0) {
    return true;
  }
  AddToInfoLog(F("Check CRC..."));
  uint16_t dataSum = 0;

  for (int i = 0; i < IdxCRC; i++) {
    dataSum = dataSum + ByteStream[i];
  }
  dataSum = dataSum & 0xff;

  if (dataSum == ByteStream[IdxCRC]) {
    return true;
  }
  AddToErrorLog(F("Check CRC failed!"));

  if (IsLogLevelInfo) {
    String log = F("# Calculated CRC: 0x");
    log += String(dataSum, HEX);
    log += F(" Received: 0x");
    log += String(ByteStream[IdxCRC], HEX);
    AddToInfoLog(log);
  }
  return false;
}

// sensor types
# define DLbus_UNUSED              0b000
# define DLbus_Sensor_DIGITAL      0b001
# define DLbus_Sensor_TEMP         0b010
# define DLbus_Sensor_VOLUME_FLOW  0b011
# define DLbus_Sensor_RAYS         0b110
# define DLbus_Sensor_ROOM         0b111

// room sensor modes
# define DLbus_RSM_AUTO            0b00
# define DLbus_RSM_NORMAL          0b01
# define DLbus_RSM_LOWER           0b10
# define DLbus_RSM_STANDBY         0b11


P092_data_struct::P092_data_struct() {}

P092_data_struct::~P092_data_struct() {
  if (DLbus_Data != nullptr) {
    if (DLbus_Data->ISR_DLB_Pin != 0xFF) {
      detachInterrupt(digitalPinToInterrupt(DLbus_Data->ISR_DLB_Pin));
    }
    delete DLbus_Data;
    DLbus_Data = nullptr;
  }

}

bool P092_data_struct::init(int8_t pin1, int P092DeviceIndex, eP092pinmode P092pinmode) {
  DLbus_Data = new (std::nothrow) DLBus;

  if (DLbus_Data == nullptr) {
    return false;
  }
  DLbus_Data->LogLevelInfo   = LOG_LEVEL_INFO;
  DLbus_Data->LogLevelError  = LOG_LEVEL_ERROR;
  DLbus_Data->IsLogLevelInfo = loglevelActiveFor(LOG_LEVEL_INFO);
  DLbus_Data->ISR_DLB_Pin    = pin1;

  //interrupt is detached in PLUGIN_WEBFORM_SAVE and attached in PLUGIN_ONCE_A_SECOND
  //to ensure that new interrupt is attached after new pin is configured, setting
  //IsISRset to false is done here.
  DLbus_Data->IsISRset       = false;

  switch (P092pinmode) {
    case eP092pinmode::ePPM_InputPullUp:
      addLog(LOG_LEVEL_INFO, F("P092_init: Set input pin with pullup"));
      pinMode(pin1, INPUT_PULLUP);
    break;
#ifdef INPUT_PULLDOWN
    case eP092pinmode::ePPM_InputPullDown:
      addLog(LOG_LEVEL_INFO, F("P092_init: Set input pin with pulldown"));
      pinMode(pin1, INPUT_PULLDOWN);
    break;
#endif
    default:
      addLog(LOG_LEVEL_INFO, F("P092_init: Set input pin"));
      pinMode(pin1, INPUT);
  }

// on a CHANGE on the data pin P092_Pin_changed is called
//DLbus_Data->attachDLBusInterrupt();
  return true;
}

void P092_data_struct::Plugin_092_SetIndices(int P092DeviceIndex) {
  // Set the indices for the DL bus packet
  int iDeviceBytes, iDontCareBytes, iTimeStampBytes;

  // default settings for ESR21
  P092_DataSettings.DataBytes                 = 31;
  P092_DataSettings.DLbus_MinPulseWidth       = P092_min_width_488;
  P092_DataSettings.DLbus_MaxPulseWidth       = P092_max_width_488;
  P092_DataSettings.DLbus_MinDoublePulseWidth = P092_double_min_width_488;
  P092_DataSettings.DLbus_MaxDoublePulseWidth = P092_double_max_width_488;

  P092_DataSettings.DeviceByte0    = 0x70;
  P092_DataSettings.DeviceByte1    = 0x8F;
  iDeviceBytes                     = 2;
  iDontCareBytes                   = 0;
  iTimeStampBytes                  = 0;
  P092_DataSettings.MaxSensors     = 3;
  P092_DataSettings.MaxExtSensors  = 6;
  P092_DataSettings.OutputBytes    = 1;
  P092_DataSettings.SpeedBytes     = 1;
  P092_DataSettings.MaxAnalogOuts  = 1;
  P092_DataSettings.AnalogBytes    = 1;
  P092_DataSettings.VolumeBytes    = 0;
  P092_DataSettings.MaxHeatMeters  = 1;
  P092_DataSettings.CurrentHmBytes = 2;
  P092_DataSettings.MWhBytes       = 2;
  P092_DataSettings.IdxCRC         = 30;

  switch (P092DeviceIndex) {
    case 31: // UVR31
      P092_DataSettings.DataBytes                 = 8;
      P092_DataSettings.DLbus_MinPulseWidth       = P092_min_width_50;
      P092_DataSettings.DLbus_MaxPulseWidth       = P092_max_width_50;
      P092_DataSettings.DLbus_MinDoublePulseWidth = P092_double_min_width_50;
      P092_DataSettings.DLbus_MaxDoublePulseWidth = P092_double_max_width_50;

      P092_DataSettings.DeviceByte0    = 0x30;
      P092_DataSettings.DeviceByte1    = 0;
      iDeviceBytes                     = 1;
      P092_DataSettings.MaxExtSensors  = 0;
      P092_DataSettings.SpeedBytes     = 0;
      P092_DataSettings.AnalogBytes    = 0;
      P092_DataSettings.MaxAnalogOuts  = 0;
      P092_DataSettings.MaxHeatMeters  = 0;
      P092_DataSettings.CurrentHmBytes = 0;
      P092_DataSettings.MWhBytes       = 0;
      P092_DataSettings.IdxCRC         = 0;
      break;
    case 1611: // UVR1611
      P092_DataSettings.DataBytes = 64;

      P092_DataSettings.DeviceByte0    = 0x80;
      P092_DataSettings.DeviceByte1    = 0x7F;
      iDontCareBytes                   = 1;
      iTimeStampBytes                  = 5;
      P092_DataSettings.MaxSensors     = 16;
      P092_DataSettings.MaxExtSensors  = 0;
      P092_DataSettings.OutputBytes    = 2;
      P092_DataSettings.SpeedBytes     = 4;
      P092_DataSettings.AnalogBytes    = 0;
      P092_DataSettings.MaxAnalogOuts  = 0;
      P092_DataSettings.MaxHeatMeters  = 2;
      P092_DataSettings.CurrentHmBytes = 4;
      P092_DataSettings.IdxCRC         = P092_DataSettings.DataBytes - 1;

      break;
    case 6132: // UVR 61-3 (up to V8.2)
      P092_DataSettings.DataBytes = 35;

      P092_DataSettings.DeviceByte0   = 0x90;
      P092_DataSettings.DeviceByte1   = 0x6F;
      iDontCareBytes                  = 1;
      iTimeStampBytes                 = 5;
      P092_DataSettings.MaxSensors    = 6;
      P092_DataSettings.MaxExtSensors = 0;
      P092_DataSettings.MaxAnalogOuts = 1;
      P092_DataSettings.VolumeBytes   = 2;
      P092_DataSettings.MWhBytes      = 4;
      P092_DataSettings.IdxCRC        = P092_DataSettings.DataBytes - 1;

      break;
    case 6133: // UVR 61-3 (from V8.3)
      P092_DataSettings.DataBytes = 62;

      P092_DataSettings.DeviceByte0   = 0x90;
      P092_DataSettings.DeviceByte1   = 0x9F;
      iDontCareBytes                  = 1;
      iTimeStampBytes                 = 5;
      P092_DataSettings.MaxSensors    = 6;
      P092_DataSettings.MaxExtSensors = 9;
      P092_DataSettings.MaxAnalogOuts = 2;
      P092_DataSettings.MaxHeatMeters = 3;
      P092_DataSettings.IdxCRC        = P092_DataSettings.DataBytes - 1;

      break;
  }
  P092_DataSettings.IdxSensor     = iDeviceBytes + iDontCareBytes + iTimeStampBytes;
  P092_DataSettings.IdxExtSensor  = P092_DataSettings.IdxSensor + 2 * P092_DataSettings.MaxSensors;
  P092_DataSettings.IdxOutput     = P092_DataSettings.IdxExtSensor + 2 * P092_DataSettings.MaxExtSensors;
  P092_DataSettings.IdxDrehzahl   = P092_DataSettings.IdxOutput + P092_DataSettings.OutputBytes;
  P092_DataSettings.IdxAnalog     = P092_DataSettings.IdxDrehzahl + P092_DataSettings.SpeedBytes;
  P092_DataSettings.IdxHmRegister = P092_DataSettings.IdxAnalog + (P092_DataSettings.AnalogBytes * P092_DataSettings.MaxAnalogOuts);
  P092_DataSettings.IdxVolume     = P092_DataSettings.IdxHmRegister + 1;
  P092_DataSettings.IdxHeatMeter1 = P092_DataSettings.IdxVolume + P092_DataSettings.VolumeBytes;
  P092_DataSettings.IdxkWh1       = P092_DataSettings.IdxHeatMeter1 + P092_DataSettings.CurrentHmBytes;
  P092_DataSettings.IdxMWh1       = P092_DataSettings.IdxkWh1 + 2;
  P092_DataSettings.IdxHeatMeter2 = P092_DataSettings.IdxMWh1 + P092_DataSettings.MWhBytes;
  P092_DataSettings.IdxkWh2       = P092_DataSettings.IdxHeatMeter2 + P092_DataSettings.CurrentHmBytes;
  P092_DataSettings.IdxMWh2       = P092_DataSettings.IdxkWh2 + 2;
  P092_DataSettings.IdxHeatMeter3 = P092_DataSettings.IdxMWh2 + P092_DataSettings.MWhBytes;
  P092_DataSettings.IdxkWh3       = P092_DataSettings.IdxHeatMeter3 + P092_DataSettings.CurrentHmBytes;
  P092_DataSettings.IdxMWh3       = P092_DataSettings.IdxkWh3 + 2;
}

/****************\
   DLBus P092_receiving
\****************/
void P092_data_struct::Plugin_092_StartReceiving(taskIndex_t taskindex) {
  DLbus_Data->ISR_Receiving   = false;
  DLbus_Data->DeviceBytes[0]  = P092_DataSettings.DeviceByte0;
  DLbus_Data->DeviceBytes[1]  = P092_DataSettings.DeviceByte1;
  DLbus_Data->ISR_PulseNumber =
    (((P092_DataSettings.DataBytes + DLbus_AdditionalRecBytes) * (DLbus_StartBits + 8 +  DLbus_StopBits) + DLBus_SyncBits) *
     DLBus_BitChangeFactor) + DLBus_ReserveBytes;
  DLbus_Data->ISR_MinPulseWidth       = P092_DataSettings.DLbus_MinPulseWidth;
  DLbus_Data->ISR_MaxPulseWidth       = P092_DataSettings.DLbus_MaxPulseWidth;
  DLbus_Data->ISR_MinDoublePulseWidth = P092_DataSettings.DLbus_MinDoublePulseWidth;
  DLbus_Data->ISR_MaxDoublePulseWidth = P092_DataSettings.DLbus_MaxDoublePulseWidth;
  DLbus_Data->StartReceiving();
  uint32_t start = millis();

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("P092_receiving ... TaskIndex:");
    log += taskindex;
    addLogMove(LOG_LEVEL_INFO, log);
  }

  while ((timePassedSince(start) < 100) && (DLbus_Data->ISR_PulseCount == 0)) {
    // wait for first pulse received (timeout 100ms)
    delay(0);
  }

  if (DLbus_Data->ISR_PulseCount == 0) {
    // nothing received
    DLbus_Data->ISR_Receiving = false;
    DLbus_Data->IsNoData = true;  // stop receiving until next PLUGIN_092_READ
    addLog(LOG_LEVEL_ERROR, F("## StartReceiving: Error: Nothing received! No DL bus connected!"));
  }
}

/****************\
   DLBus get data
\****************/
boolean P092_data_struct::P092_GetData(int OptionIdx, int CurIdx, sP092_ReadData *ReadData) {
  String  log;
  boolean result = false;

  switch (OptionIdx) {
    case 1: // F("Sensor")
      log  = F("Get Sensor");
      log += CurIdx;

      if (CurIdx > P092_DataSettings.MaxSensors) {
        result = false;
        break;
      }
      ReadData->Idx = P092_DataSettings.IdxSensor;
      result        = P092_fetch_sensor(CurIdx, ReadData);
      break;
    case 2: // F("Sensor")
      log  = F("Get ExtSensor");
      log += CurIdx;

      if (CurIdx > P092_DataSettings.MaxExtSensors) {
        result = false;
        break;
      }
      ReadData->Idx = P092_DataSettings.IdxExtSensor;
      result        = P092_fetch_sensor(CurIdx, ReadData);
      break;
    case 3: // F("Digital output")
      log  = F("Get DigitalOutput");
      log += CurIdx;

      if (CurIdx > (8 * P092_DataSettings.OutputBytes)) {
        result = false;
        break;
      }
      result = P092_fetch_output(CurIdx, ReadData);
      break;
    case 4: // F("Speed step")
      log  = F("Get SpeedStep");
      log += CurIdx;

      if (CurIdx > P092_DataSettings.SpeedBytes) {
        result = false;
        break;
      }
      result = P092_fetch_speed(CurIdx, ReadData);
      break;
    case 5: // F("Analog output")
      log  = F("Get AnalogOutput");
      log += CurIdx;

      if (CurIdx > P092_DataSettings.AnalogBytes) {
        result = false;
        break;
      }
      result = P092_fetch_analog(CurIdx, ReadData);
      break;
    case 6: // F("Heat power (kW)")
      log  = F("Get HeatPower");
      log += CurIdx;

      if (CurIdx > P092_DataSettings.MaxHeatMeters) {
        result = false;
        break;
      }
      result = P092_fetch_heatpower(CurIdx, ReadData);
      break;
    case 7: // F("Heat meter (MWh)"
      log  = F("Get HeatMeter");
      log += CurIdx;

      if (CurIdx > P092_DataSettings.MaxHeatMeters) {
        result = false;
        break;
      }
      result = P092_fetch_heatmeter(CurIdx, ReadData);
      break;
  }

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    log += F(": ");

    if (result) {
      log += String(ReadData->value, 1);
    }
    else {
      log += F("nan");
    }
    addLogMove(LOG_LEVEL_INFO, log);
  }
  return result;
}

boolean P092_data_struct::P092_fetch_sensor(int number, sP092_ReadData *ReadData) {
  float value = 0.0f;

  ReadData->mode = -1;
  number         = ReadData->Idx + (number - 1) * 2;
  int32_t sensorvalue = (DLbus_Data->ByteStream[number + 1] << 8) | DLbus_Data->ByteStream[number];

  if (sensorvalue == 0) {
    return false;
  }
  uint8_t sensortype = (sensorvalue & 0x7000) >> 12;

  if (!(sensorvalue & 0x8000)) { // sign positive
    sensorvalue &= 0xfff;

    // calculations for different sensor types
    switch (sensortype) {
      case DLbus_Sensor_DIGITAL:
        value = false;
        break;
      case DLbus_Sensor_TEMP:
        value = static_cast<float>(sensorvalue) * 0.1f;
        break;
      case DLbus_Sensor_RAYS:
        value = sensorvalue;
        break;
      case DLbus_Sensor_VOLUME_FLOW:
        value = sensorvalue * 4;
        break;
      case DLbus_Sensor_ROOM:
        ReadData->mode = (sensorvalue & 0x600) >> 9;
        value          = static_cast<float>(sensorvalue & 0x1ff) * 0.1f;
        break;
      default:
        return false;
    }
  }
  else { // sign negative
    sensorvalue |= 0xf000;

    // calculations for different sensor types
    switch (sensortype) {
      case DLbus_Sensor_DIGITAL:
        value = true;
        break;
      case DLbus_Sensor_TEMP:
        value = static_cast<float>(sensorvalue - 0x10000) * 0.1f;
        break;
      case DLbus_Sensor_RAYS:
        value = sensorvalue - 0x10000;
        break;
      case DLbus_Sensor_VOLUME_FLOW:
        value = (sensorvalue - 0x10000) * 4;
        break;
      case DLbus_Sensor_ROOM:
        ReadData->mode = (sensorvalue & 0x600) >> 9;
        value          = static_cast<float>((sensorvalue & 0x1ff) - 0x10000) * 0.1f;
        break;
      default:
        return false;
    }
  }
  ReadData->value = value;
  return true;
}

boolean P092_data_struct::P092_fetch_output(int number, sP092_ReadData *ReadData) {
  int32_t outputs;

  if (P092_DataSettings.OutputBytes > 1) {
    outputs = (DLbus_Data->ByteStream[P092_DataSettings.IdxOutput + 1] << 8) | DLbus_Data->ByteStream[P092_DataSettings.IdxOutput];
  }
  else {
    outputs = DLbus_Data->ByteStream[P092_DataSettings.IdxOutput];
  }

  if (outputs & (1 << (number - 1))) {
    ReadData->value = 1;
  }
  else {
    ReadData->value = 0;
  }
  return true;
}

boolean P092_data_struct::P092_fetch_speed(int number, sP092_ReadData *ReadData) {
  uint8_t speedbyte;

  if ((P092_DataSettings.IdxDrehzahl + (number - 1)) >= P092_DataSettings.IdxAnalog) {
    // wrong index for speed, overlapping next index (IdxAnalog)
    return false;
  }
  speedbyte = DLbus_Data->ByteStream[P092_DataSettings.IdxDrehzahl + (number - 1)];

  if (speedbyte & 0x80) {
    return false;
  }
  ReadData->value = (speedbyte & 0x1f);
  return true;
}

boolean P092_data_struct::P092_fetch_analog(int number, sP092_ReadData *ReadData) {
  uint8_t analogbyte;

  if ((P092_DataSettings.IdxAnalog + (number - 1)) >= P092_DataSettings.IdxHmRegister) {
    // wrong index for analog, overlapping next index (IdxHmRegister)
    return false;
  }
  analogbyte = DLbus_Data->ByteStream[P092_DataSettings.IdxAnalog + (number - 1)];

  if (analogbyte & 0x80) {
    return false;
  }
  ReadData->value = (analogbyte * 0.1);
  return true;
}

P092_data_struct::sDLbus_HMindex P092_data_struct::P092_CheckHmRegister(int number) {
  sDLbus_HMindex result;

  result.IndexIsValid = 0;

  switch (number) {
    case 1:

      if ((DLbus_Data->ByteStream[P092_DataSettings.IdxHmRegister] & 0x1) == 0) {
        return result;
      }
      result.power_index = P092_DataSettings.IdxHeatMeter1;
      result.kwh_index   = P092_DataSettings.IdxkWh1;
      result.mwh_index   = P092_DataSettings.IdxMWh1;
      break;
    case 2:

      if ((DLbus_Data->ByteStream[P092_DataSettings.IdxHmRegister] & 0x2) == 0) {
        return result;
      }
      result.power_index = P092_DataSettings.IdxHeatMeter2;
      result.kwh_index   = P092_DataSettings.IdxkWh2;
      result.mwh_index   = P092_DataSettings.IdxMWh2;
      break;
    case 3:

      if ((DLbus_Data->ByteStream[P092_DataSettings.IdxHmRegister] & 0x4) == 0) {
        return result;
      }
      result.power_index = P092_DataSettings.IdxHeatMeter3;
      result.kwh_index   = P092_DataSettings.IdxkWh3;
      result.mwh_index   = P092_DataSettings.IdxMWh3;
      break;
    default:
      return result;
  }
  result.IndexIsValid = 1;
  return result;
}

boolean P092_data_struct::P092_fetch_heatpower(int number, sP092_ReadData *ReadData) {
  // current power
  int32_t high;
  sDLbus_HMindex HMindex = P092_CheckHmRegister(number);

  if (HMindex.IndexIsValid == 0) {
    return false;
  }
  uint8_t b1 = DLbus_Data->ByteStream[HMindex.power_index];
  uint8_t b2 = DLbus_Data->ByteStream[HMindex.power_index + 1];

  if (P092_DataSettings.CurrentHmBytes > 2) {
    uint8_t b3 = DLbus_Data->ByteStream[HMindex.power_index + 2];
    uint8_t b4 = DLbus_Data->ByteStream[HMindex.power_index + 3];
    high = 0x10000 * b4 + 0x100 * b3 + b2;
    int low = (b1 * 10) / 0x100;

    if (!(b4 & 0x80)) { // sign positive
      ReadData->value = static_cast<float>(10 * high + low) / 100.0f;
    }
    else {              // sign negative
      ReadData->value = static_cast<float>(10 * (high - 0x10000) - low) / 100.0f;
    }
  }
  else {
    high = (b2 << 8) | b1;

    if ((b2 & 0x80) == 0) { // sign positive
      ReadData->value = static_cast<float>(high) / 10.0f;
    }
    else {                  // sign negative
      ReadData->value = static_cast<float>(high - 0x10000) / 10.0f;
    }
  }
  return true;
}

boolean P092_data_struct::P092_fetch_heatmeter(int number, sP092_ReadData *ReadData) {
  // heat meter
  int32_t heat_meter;
  float   heat_meter_mwh;

  sDLbus_HMindex HMindex = P092_CheckHmRegister(number);

  if (HMindex.IndexIsValid == 0) {
    return false;
  }
  heat_meter     = (DLbus_Data->ByteStream[HMindex.kwh_index + 1] << 8) | DLbus_Data->ByteStream[HMindex.kwh_index];
  heat_meter_mwh = (heat_meter * 0.1f) / 1000.0f; // in MWh

  if (heat_meter_mwh > 1.0f) {
    // in kWh
    heat_meter      = heat_meter_mwh;
    heat_meter_mwh -= heat_meter;
  }

  // MWh
  heat_meter      = (DLbus_Data->ByteStream[HMindex.mwh_index + 1] << 8) | DLbus_Data->ByteStream[HMindex.mwh_index];
  ReadData->value = heat_meter_mwh + heat_meter;
  return true;
}

#endif // ifdef USES_P092
#include "../PluginStructs/P062_data_struct.h"

// Needed also here for PlatformIO's library finder as the .h file
// is in a directory which is excluded in the src_filter
#ifdef USES_P062

# include <Adafruit_MPR121.h>
# include "../Helpers/ESPEasy_Storage.h"

P062_data_struct::P062_data_struct() {
  # ifdef PLUGIN_062_DEBUG
  addLog(LOG_LEVEL_INFO, F("P062_data_struct constructor"));
  # endif // ifdef PLUGIN_062_DEBUG
  clearCalibrationData(); // Reset
}

P062_data_struct::~P062_data_struct() {
  if (keypad != nullptr) {
    delete keypad;
    keypad = nullptr;
  }
}

bool P062_data_struct::init(taskIndex_t taskIndex,
                            uint8_t     i2c_addr,
                            bool        scancode,
                            bool        keepCalibrationData,
                            uint8_t     sensitivity) {
  # ifdef PLUGIN_062_DEBUG
  addLog(LOG_LEVEL_INFO, F("P062_data_struct init()"));
  # endif // ifdef PLUGIN_062_DEBUG
  _i2c_addr            = i2c_addr;
  _use_scancode        = scancode;
  _keepCalibrationData = keepCalibrationData;
  _sensitivity         = sensitivity;

  if (!keypad) {
    keypad = new (std::nothrow) Adafruit_MPR121();
  }

  if (keypad) {
    keypad->begin(_i2c_addr, _sensitivity);
    loadTouchObjects(taskIndex);
    return true;
  }
  return false;
}

void P062_data_struct::updateCalibration(uint8_t t) {
  if (t >= P062_MaxTouchObjects) { return; }

  if (_keepCalibrationData) {
    uint16_t current = keypad->filteredData(t);
    CalibrationData.CalibrationValues[t].current = current;

    if ((CalibrationData.CalibrationValues[t].min == 0) || (current < CalibrationData.CalibrationValues[t].min)) {
      CalibrationData.CalibrationValues[t].min = current;
    }

    if ((CalibrationData.CalibrationValues[t].max == 0) || (current > CalibrationData.CalibrationValues[t].max)) {
      CalibrationData.CalibrationValues[t].max = current;
    }
  }
}

bool P062_data_struct::readKey(uint16_t& key) {
  if (!keypad) { return false; }
  key = keypad->touched();

  if (key)
  {
    uint16_t colMask = 0x01;

    for (uint8_t col = 1; col <= 12; col++)
    {
      if (key & colMask) // this key pressed?
      {
        updateCalibration(col - 1);

        if (_use_scancode) {
          key = col;
          break;
        }
      }
      colMask <<= 1;
    }
  }

  if (keyLast != key)
  {
    keyLast = key;
    return true;
  }
  return false;
}

/**
 * Set all tresholds at once
 */
void P062_data_struct::setThresholds(uint8_t touch, uint8_t release) {
  keypad->setThresholds(touch, release);
}

/**
 * Set a single treshold
 */
void P062_data_struct::setThreshold(uint8_t t, uint8_t touch, uint8_t release) {
  keypad->setThreshold(t, touch, release);
}

/**
 * Load the touch objects from the settings, and initialize then properly where needed.
 */
void P062_data_struct::loadTouchObjects(taskIndex_t taskIndex) {
  # ifdef PLUGIN_062_DEBUG
  String log = F("P062 DEBUG loadTouchObjects size: ");
  log += sizeof(StoredSettings);
  addLogMove(LOG_LEVEL_INFO, log);
  # endif // PLUGIN_062_DEBUG
  LoadCustomTaskSettings(taskIndex, reinterpret_cast<uint8_t *>(&StoredSettings), sizeof(StoredSettings));
}

/**
 * Get the Calibration data for 1 touch object, return false if all zeroes or invalid input for t.
 */
bool P062_data_struct::getCalibrationData(uint8_t t, uint16_t *current, uint16_t *min, uint16_t *max) {
  if (t >= P062_MaxTouchObjects) { return false; }
  *current = CalibrationData.CalibrationValues[t].current;
  *min     = CalibrationData.CalibrationValues[t].min;
  *max     = CalibrationData.CalibrationValues[t].max;
  return (*current + *min + *max) > 0;
}

/**
 * Reset the touch data.
 */
void P062_data_struct::clearCalibrationData() {
  for (uint8_t t = 0; t < P062_MaxTouchObjects; t++) {
    CalibrationData.CalibrationValues[t].current = 0;
    CalibrationData.CalibrationValues[t].min     = 0;
    CalibrationData.CalibrationValues[t].max     = 0;
  }
}

#endif // ifdef USES_P062

#include "../PluginStructs/P058_data_struct.h"

// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter
# include <HT16K33.h>


#ifdef USES_P058

P058_data_struct::P058_data_struct(uint8_t i2c_addr) {
  keyPad.Init(i2c_addr);
}

bool P058_data_struct::readKey(uint8_t& key) {
  key = keyPad.ReadKeys();

  if (keyLast != key)
  {
    keyLast = key;
    return true;
  }
  return false;
}

#endif // ifdef USES_P058

#include "../PluginStructs/P035_data_struct.h"

#ifdef USES_P035

// **************************************************************************/
// Constructor
// **************************************************************************/
P035_data_struct::P035_data_struct(int8_t gpioPin)
  : _gpioPin(gpioPin) {}

// **************************************************************************/
// Destructor
// **************************************************************************/
P035_data_struct::~P035_data_struct() {
  if (Plugin_035_irSender != nullptr) {
    delete Plugin_035_irSender;
    Plugin_035_irSender = nullptr;
  }
  # ifdef P016_P035_Extended_AC

  if (Plugin_035_commonAc != nullptr) {
    delete Plugin_035_commonAc;
    Plugin_035_commonAc = nullptr;
  }
  # endif // ifdef P016_P035_Extended_AC
}

bool P035_data_struct::plugin_init(struct EventStruct *event) {
  bool success = false;

  if ((Plugin_035_irSender == nullptr) && validGpio(_gpioPin)) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      addLog(LOG_LEVEL_INFO, F("INIT: IR TX"));
      addLog(LOG_LEVEL_INFO, F("IR lib Version: " _IRREMOTEESP8266_VERSION_STR));
      # ifdef P035_DEBUG_LOG
      addLog(LOG_LEVEL_INFO, String(F("Supported Protocols by IRSEND: ")) + listProtocols());
      # endif // ifdef P035_DEBUG_LOG
    }
    Plugin_035_irSender = new (std::nothrow) IRsend(_gpioPin);

    if (Plugin_035_irSender != nullptr) {
      Plugin_035_irSender->begin(); // Start the sender
      success = true;
    }
  }

  if ((Plugin_035_irSender != nullptr) && (_gpioPin == -1)) {
    addLog(LOG_LEVEL_INFO, F("INIT: IR TX Removed"));
    delete Plugin_035_irSender;
    Plugin_035_irSender = nullptr;
    success             = false;
  }

  # ifdef P016_P035_Extended_AC

  if (success && (Plugin_035_commonAc == nullptr) && validGpio(_gpioPin)) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      addLog(LOG_LEVEL_INFO, F("INIT AC: IR TX"));
      #  ifdef P035_DEBUG_LOG
      addLog(LOG_LEVEL_INFO, String(F("Supported Protocols by IRSENDAC: ")) + listACProtocols());
      #  endif // ifdef P035_DEBUG_LOG
    }
    Plugin_035_commonAc = new (std::nothrow) IRac(_gpioPin);
  }

  if ((Plugin_035_commonAc != nullptr) && (_gpioPin == -1)) {
    addLog(LOG_LEVEL_INFO, F("INIT AC: IR TX Removed"));
    delete Plugin_035_commonAc;
    Plugin_035_commonAc = nullptr;
    success             = false;
  }
  # endif // ifdef P016_P035_Extended_AC
  return success;
}

bool P035_data_struct::plugin_exit(struct EventStruct *event) {
  if (Plugin_035_irSender != nullptr) {
    delete Plugin_035_irSender;
    Plugin_035_irSender = nullptr;
  }
  # ifdef P016_P035_Extended_AC

  if (Plugin_035_commonAc != nullptr) {
    delete Plugin_035_commonAc;
    Plugin_035_commonAc = nullptr;
  }
  # endif // ifdef P016_P035_Extended_AC
  return true;
}

bool P035_data_struct::plugin_write(struct EventStruct *event, const String& string) {
  bool success = false;

  String cmdCode = parseString(string, 1);

  if (cmdCode.equalsIgnoreCase(F("IRSEND")) && (Plugin_035_irSender != nullptr)) {
    success = true;
    enableIR_RX(false);

    # ifdef P016_P035_USE_RAW_RAW2
    handleRawRaw2Encoding(string);
    # endif // P016_P035_USE_RAW_RAW2

    handleIRremote(string);
  }

  # ifdef P016_P035_Extended_AC
  else if (cmdCode.equalsIgnoreCase(F("IRSENDAC")) && (Plugin_035_commonAc != nullptr)) {
    success = true;
    enableIR_RX(false);
    handle_AC_IRremote(parseStringToEnd(string, 2));
  }
  # endif // P016_P035_Extended_AC

  enableIR_RX(true);

  return success;
}

bool P035_data_struct::handleIRremote(const String& cmd) {
  String IrType;
  String TmpStr1;

  uint64_t IrCode   = 0;
  uint16_t IrBits   = 0;
  uint16_t IrRepeat = 0;
  String   ircodestr;

  StaticJsonDocument<200> docTemp;
  DeserializationError    error = deserializeJson(docTemp, parseStringToEnd(cmd, 2));

  if (!error) {                                  // If the command is in JSON format
    IrType    =  docTemp[F("protocol")].as<String>();
    ircodestr = docTemp[F("data")].as<String>(); // JSON does not support hex values, thus we use command representation
    IrCode    = strtoull(ircodestr.c_str(), nullptr, 16);
    IrBits    = docTemp[F("bits")] | 0;
    IrRepeat  = docTemp[F("repeats")] | 0;
  } else { // If the command is NOT in JSON format (legacy)
    IrType = parseString(cmd, 2);

    if (IrType.length() > 0) {
      ircodestr = parseString(cmd, 3);

      if (ircodestr.length() > 0) {
        IrCode = strtoull(ircodestr.c_str(), nullptr, 16);
      }
      IrBits   = parseString(cmd, 4).toInt(); // Number of bits to be sent. USE 0 for default protocol bits
      IrRepeat = parseString(cmd, 5).toInt(); // Nr. of times the message is to be repeated
    }
  }

  bool IRsent = sendIRCode(strToDecodeType(IrType.c_str()), IrCode, ircodestr.c_str(), IrBits, IrRepeat); // Send the IR command

  if (IRsent) {
    printToLog(IrType, ircodestr, IrBits, IrRepeat);
  }
  return IRsent;
}

# ifdef P016_P035_Extended_AC
bool P035_data_struct::handle_AC_IRremote(const String& irData) {
  StaticJsonDocument<JSON_OBJECT_SIZE(18) + 190> doc;
  DeserializationError error = deserializeJson(doc, irData); // Deserialize the JSON document

  if (error) {                                               // Test if parsing succeeds.
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      addLog(LOG_LEVEL_INFO, String(F("IRTX: Deserialize Json failed: ")) + error.c_str());
    }
    return false; // do not continue with sending the signal.
  }
  String sprotocol = doc[F("protocol")];

  st.protocol = strToDecodeType(sprotocol.c_str());

  if (!IRac::isProtocolSupported(st.protocol)) { // Check if we support the protocol
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      addLog(LOG_LEVEL_INFO, String(F("IRTX: Protocol not supported:")) + sprotocol);
    }
    return false; // do not continue with sending of the signal.
  }

  String tempstr;

  tempstr  = doc[F("model")].as<String>();
  st.model = IRac::strToModel(tempstr.c_str(), -1);                             // The specific model of A/C if applicable. //strToModel();.
                                                                                // Defaults to -1 (unknown) if missing from JSON
  tempstr    = doc[F("power")].as<String>();
  st.power   = IRac::strToBool(tempstr.c_str(), false);                         // POWER ON or OFF. Defaults to false if missing from JSON
  st.degrees = doc[F("temp")] | 22.0f;                                          // What temperature should the unit be set to?. Defaults to
                                                                                // 22c if missing from JSON
  tempstr    = doc[F("use_celsius")].as<String>();
  st.celsius = IRac::strToBool(tempstr.c_str(), true);                          // Use degreees Celsius, otherwise Fahrenheit. Defaults to
                                                                                // true if missing from JSON
  tempstr = doc[F("mode")].as<String>();
  st.mode = IRac::strToOpmode(tempstr.c_str(), stdAc::opmode_t::kAuto);         // What operating mode should the unit perform? e.g. Cool.
                                                                                // Defaults to auto if missing from JSON
  tempstr     = doc[F("fanspeed")].as<String>();
  st.fanspeed = IRac::strToFanspeed(tempstr.c_str(), stdAc::fanspeed_t::kAuto); // Fan Speed setting. Defaults to auto if missing from JSON
  tempstr     = doc[F("swingv")].as<String>();
  st.swingv   = IRac::strToSwingV(tempstr.c_str(), stdAc::swingv_t::kAuto);     // Vertical swing setting. Defaults to auto if missing from
                                                                                // JSON
  tempstr   = doc[F("swingh")].as<String>();
  st.swingh = IRac::strToSwingH(tempstr.c_str(), stdAc::swingh_t::kAuto);       // Horizontal Swing setting. Defaults to auto if missing
                                                                                // from JSON
  tempstr  = doc[F("quiet")].as<String>();
  st.quiet = IRac::strToBool(tempstr.c_str(), false);                           // Quiet setting ON or OFF. Defaults to false if missing
                                                                                // from JSON
  tempstr  = doc[F("turbo")].as<String>();
  st.turbo = IRac::strToBool(tempstr.c_str(), false);                           // Turbo setting ON or OFF. Defaults to false if missing
                                                                                // from JSON
  tempstr  = doc[F("econo")].as<String>();
  st.econo = IRac::strToBool(tempstr.c_str(), false);                           // Economy setting ON or OFF. Defaults to false if missing
                                                                                // from JSON
  tempstr  = doc[F("light")].as<String>();
  st.light = IRac::strToBool(tempstr.c_str(), true);                            // Light setting ON or OFF. Defaults to true if missing from
                                                                                // JSON
  tempstr   = doc[F("filter")].as<String>();
  st.filter = IRac::strToBool(tempstr.c_str(), false);                          // Filter setting ON or OFF. Defaults to false if missing
                                                                                // from JSON
  tempstr  = doc[F("clean")].as<String>();
  st.clean = IRac::strToBool(tempstr.c_str(), false);                           // Clean setting ON or OFF. Defaults to false if missing
                                                                                // from JSON
  tempstr = doc[F("beep")].as<String>();
  st.beep = IRac::strToBool(tempstr.c_str(), false);                            // Beep setting ON or OFF. Defaults to false if missing from
                                                                                // JSON
  st.sleep = doc[F("sleep")] | -1;                                              // Nr. of mins of sleep mode, or use sleep mode. (<= 0 means
                                                                                // off.). Defaults to -1 if missing from JSON
  st.clock = doc[F("clock")] | -1;                                              // Nr. of mins past midnight to set the clock to. (< 0 means
                                                                                // off.). Defaults to -1 if missing from JSON

  // Send the IR command
  bool IRsent = Plugin_035_commonAc->sendAc(st, &prev);

  if (IRsent) {
    printToLog(typeToString(st.protocol), irData, 0, 0);
  }
  return IRsent;
}

# endif // ifdef P016_P035_Extended_AC


bool P035_data_struct::handleRawRaw2Encoding(const String& cmd) {
  bool   raw    = true;
  String IrType = parseString(cmd, 2);

  if (IrType.isEmpty()) { return false; }

  if (IrType.equalsIgnoreCase(F("RAW"))) {
    raw = true;
  } else if (IrType.equalsIgnoreCase(F("RAW2"))) {
    raw = false;
  }

  String   IrRaw      = parseString(cmd, 3);         // Get the "Base32" encoded/compressed Ir signal
  uint16_t IrHz       = parseString(cmd, 4).toInt(); // Get the base freguency of the signal (allways 38)
  unsigned int IrPLen = parseString(cmd, 5).toInt(); // Get the Pulse Length in ms
  unsigned int IrBLen = parseString(cmd, 6).toInt(); // Get the Blank Pulse Length in ms

  uint16_t  idx = 0;                                 // If this goes above the buf.size then the esp will throw a 28 EXCCAUSE
  uint16_t *buf;

  buf = new (std::nothrow) uint16_t[P35_Ntimings];   // The Raw Timings that we can buffer.

  if (buf == nullptr) {                              // error assigning memory.
    return false;
  }


  if (raw) {
    unsigned int c0 = 0; // count consecutives 0s
    unsigned int c1 = 0; // count consecutives 1s

    // printWebString += F("Interpreted RAW Code: ");  //print the number of 1s and 0s just for debugging/info purposes
    // Loop throught every char in RAW string
    for (unsigned int i = 0; i < IrRaw.length(); i++) {
      // Get the decimal value from base32 table
      // See: https://en.wikipedia.org/wiki/Base32#base32hex
      char c = from_32hex(IrRaw[i]);

      // Loop through 5 LSB (bits 16, 8, 4, 2, 1)
      for (unsigned int shft = 1; shft < 6; shft++) {
        // if bit is 1 (5th position - 00010000 = 16)
        if ((c & 16) != 0) {
          // add 1 to counter c1
          c1++;

          // if we already have any 0s in counting (the previous
          // bit was 0)
          if (c0 > 0) {
            // add the total ms into the buffer (number of 0s multiplied
            // by defined blank length ms)
            buf[idx++] = c0 * IrBLen;

            // print the number of 0s just for debuging/info purpouses
            // for (uint t = 0; t < c0; t++)
            // printWebString += '0';
          }

          // So, as we receive a "1", and processed the counted 0s
          // sending them as a ms timing into the buffer, we clear
          // the 0s counter
          c0 = 0;
        } else {
          // So, bit is 0

          // On first call, ignore 0s (suppress left-most 0s)
          if (c0 + c1 != 0) {
            // add 1 to counter c0
            c0++;

            // if we already have any 1s in counting (the previous
            // bit was 1)
            if (c1 > 0) {
              // add the total ms into the buffer (number of 1s
              // multiplied by defined pulse length ms)
              buf[idx++] = c1 * IrPLen;

              // print the number of 1s just for debugging/info purposes
              //                          for (uint t = 0; t < c1; t++)
              //                            printWebString += '1';
            }

            // So, as we receive a "0", and processed the counted 1s
            // sending them as a ms timing into the buffer, we clear
            // the 1s counter
            c1 = 0;
          }
        }

        // shift to left the "c" variable to process the next bit that is
        // in 5th position (00010000 = 16)
        c <<= 1;
      }
    }

    // Finally, we need to process the last counted bit that we were
    // processing

    // If we have pendings 0s
    if (c0 > 0) {
      buf[idx++] = c0 * IrBLen;

      // for (uint t = 0; t < c0; t++)
      // printWebString += '0';
    }

    // If we have pendings 1s
    if (c1 > 0) {
      buf[idx++] = c1 * IrPLen;

      // for (uint t = 0; t < c1; t++)
      // printWebString += '1';
    }

    // printWebString += F("<BR>");
  }

  if (!raw) { // RAW2
    for (unsigned int i = 0, total = IrRaw.length(), gotRep = 0, rep = 0; i < total;) {
      char c = IrRaw[i++];

      if (c == '*') {
        if (((i + 2) >= total) || ((idx + (rep = from_32hex(IrRaw[i++])) * 2) > (sizeof(buf[0]) * P35_Ntimings))) {
          delete[] buf;
          buf = nullptr;
          return addErrorTrue();
        }
        gotRep = 2;
      } else {
        if (((c == '^') && ((i + 1u) >= total)) || ((idx + 2u) >= (sizeof(buf[0]) * P35_Ntimings))) {
          delete[] buf;
          buf = nullptr;
          return addErrorTrue();
        }

        uint16_t irLen = (idx & 1) ? IrBLen : IrPLen;

        if (c == '^') {
          buf[idx++] = (from_32hex(IrRaw[i]) * 32 + from_32hex(IrRaw[i + 1])) * irLen;
          i         += 2;
        } else {
          buf[idx++] = from_32hex(c) * irLen;
        }

        if (--gotRep == 0) {
          while (--rep) {
            buf[idx]     = buf[idx - 2];
            buf[idx + 1] = buf[idx - 1];
            idx         += 2;
          }
        }
      }
    }
  } // End RAW2

  Plugin_035_irSender->sendRaw(buf, idx, IrHz);

  // printWebString += IrType + String(F(": Base32Hex RAW Code: ")) + IrRaw + String(F("<BR>kHz: ")) + IrHz + String(F("<BR>Pulse Len: ")) +
  // IrPLen + String(F("<BR>Blank Len: ")) + IrBLen + String(F("<BR>"));
  printToLog(F(": Base32Hex RAW Code Send "), IrRaw, 0, 0);

  // printWebString += String(F(": Base32Hex RAW Code Send "));
  delete[] buf;
  buf = nullptr;
  return true;
}

void P035_data_struct::printToLog(const String& protocol, const String& data, int bits, int repeats) {
  if (!loglevelActiveFor(LOG_LEVEL_INFO) && !printToWeb) {
    return;
  }
  String tmp = F("IRTX: IR Code Sent: ");

  tmp += protocol;
  tmp += F(" Data: ");
  tmp += data;

  if (bits > 0) {
    tmp += F(" Bits: ");
    tmp += bits;
  }

  if (repeats > 0) {
    tmp += F(" Repeats: ");
    tmp += repeats;
  }
  if (printToWeb) {
    printWebString = tmp;
  }
  addLogMove(LOG_LEVEL_INFO, tmp);
}

# ifdef P035_DEBUG_LOG
String P035_data_struct::listProtocols() {
  String temp;

  for (uint32_t i = 0; i <= kLastDecodeType; i++) {
    if (IRsend::defaultBits((decode_type_t)i) > 0) {
      temp += typeToString((decode_type_t)i) + ' ';
    }
  }
  return temp;
}

# endif // ifdef P035_DEBUG_LOG

# if defined(P016_P035_Extended_AC) && defined(P035_DEBUG_LOG)
String P035_data_struct::listACProtocols() {
  String temp;

  for (uint32_t i = 0; i <= kLastDecodeType; i++) {
    if (hasACState((decode_type_t)i)) {
      temp += typeToString((decode_type_t)i) + ' ';
    }
  }
  return temp;
}

# endif // if defined(P016_P035_Extended_AC) && defined(P035_DEBUG_LOG)

bool P035_data_struct::addErrorTrue() {
  addLog(LOG_LEVEL_ERROR, F("RAW2: Invalid encoding!"));
  return true;
}

// A lot of the following code has been taken directly (with permission) from the IRMQTTServer.ino example code
// of the IRremoteESP8266 library. (https://github.com/markszabo/IRremoteESP8266)

// Transmit the given IR message.
//
// Args:
//   irsend:   A pointer to a IRsend object to transmit via.
//   irtype:  enum of the protocol to be sent.
//   code:     Numeric payload of the IR message. Most protocols use this.
//   code_str: The unparsed code to be sent. Used by complex protocol encodings.
//   bits:     Nr. of bits in the protocol. 0 means use the protocol's default.
//   repeat:   Nr. of times the message is to be repeated. (Not all protcols.)
// Returns:
//   bool: Successfully sent or not.
bool P035_data_struct::sendIRCode(int const irtype,
                                  uint64_t const code, char const *code_str, uint16_t bits,
                                  uint16_t repeat) {
  decode_type_t irType = (decode_type_t)irtype;
  bool success         = true; // Assume success.

  repeat = std::max(IRsend::minRepeats(irType), repeat);

  if (bits == 0) {
    bits = IRsend::defaultBits(irType);
  }

  // send the IR message.

  if (hasACState(irType)) { // protocols with > 64 bits
    success = parseStringAndSendAirCon(irType, code_str);
  } else {                  // protocols with <= 64 bits
    success = Plugin_035_irSender->send(irType, code, bits, repeat);
  }

  return success;
}

// Parse an Air Conditioner A/C Hex String/code and send it.
// Args:
//   irtype: Nr. of the protocol we need to send.
//   str: A hexadecimal string containing the state to be sent.
bool P035_data_struct::parseStringAndSendAirCon(const int irtype, const String str) {
  decode_type_t irType          = (decode_type_t)irtype;
  uint8_t  strOffset            = 0;
  uint8_t  state[kStateSizeMax] = { 0 }; // All array elements are set to 0.
  uint16_t stateSize            = 0;

  if (str.startsWith("0x") || str.startsWith("0X")) {
    strOffset = 2;
  }

  // Calculate how many hexadecimal characters there are.
  uint16_t inputLength = str.length() - strOffset;

  if (inputLength == 0) {
    // debug("Zero length AirCon code encountered. Ignored.");
    return false;   // No input. Abort.
  }

  switch (irType) { // Get the correct state size for the protocol.
    case DAIKIN:
      // Daikin has 2 different possible size states.
      // (The correct size, and a legacy shorter size.)
      // Guess which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/uint8_t size.
      // This should provide backward compatiblity with legacy messages.
      stateSize = inputLength / 2; // Every two hex chars is a uint8_t.
      // Use at least the minimum size.
      stateSize = std::max(stateSize, kDaikinStateLengthShort);

      // If we think it isn't a "short" message.
      if (stateSize > kDaikinStateLengthShort) {
        // Then it has to be at least the version of the "normal" size.
        stateSize = std::max(stateSize, kDaikinStateLength);
      }

      // Lastly, it should never exceed the "normal" size.
      stateSize = std::min(stateSize, kDaikinStateLength);
      break;
    case FUJITSU_AC:
      // Fujitsu has four distinct & different size states, so make a best guess
      // which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/uint8_t size.
      stateSize = inputLength / 2; // Every two hex chars is a uint8_t.
      // Use at least the minimum size.
      stateSize = std::max(stateSize,
                           (uint16_t)(kFujitsuAcStateLengthShort - 1));

      // If we think it isn't a "short" message.
      if (stateSize > kFujitsuAcStateLengthShort) {
        // Then it has to be at least the smaller version of the "normal" size.
        stateSize = std::max(stateSize, (uint16_t)(kFujitsuAcStateLength - 1));
      }

      // Lastly, it should never exceed the maximum "normal" size.
      stateSize = std::min(stateSize, kFujitsuAcStateLength);
      break;
    case HITACHI_AC3:
      // HitachiAc3 has two distinct & different size states, so make a best
      // guess which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/uint8_t size.
      stateSize = inputLength / 2; // Every two hex chars is a uint8_t.
      // Use at least the minimum size.
      stateSize = std::max(stateSize,
                           (uint16_t)(kHitachiAc3MinStateLength));

      // If we think it isn't a "short" message.
      if (stateSize > kHitachiAc3MinStateLength) {
        // Then it probably the "normal" size.
        stateSize = std::max(stateSize,
                             (uint16_t)(kHitachiAc3StateLength));
      }

      // Lastly, it should never exceed the maximum "normal" size.
      stateSize = std::min(stateSize, kHitachiAc3StateLength);
      break;
    case MWM:
      // MWM has variable size states, so make a best guess
      // which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/uint8_t size.
      stateSize = inputLength / 2; // Every two hex chars is a uint8_t.
      // Use at least the minimum size.
      stateSize = std::max(stateSize, (uint16_t)3);

      // Cap the maximum size.
      stateSize = std::min(stateSize, kStateSizeMax);
      break;
    case SAMSUNG_AC:
      // Samsung has two distinct & different size states, so make a best guess
      // which one we are being presented with based on the number of
      // hexadecimal digits provided. i.e. Zero-pad if you need to to get
      // the correct length/uint8_t size.
      stateSize = inputLength / 2; // Every two hex chars is a uint8_t.
      // Use at least the minimum size.
      stateSize = std::max(stateSize, (uint16_t)(kSamsungAcStateLength));

      // If we think it isn't a "normal" message.
      if (stateSize > kSamsungAcStateLength) {
        // Then it probably the extended size.
        stateSize = std::max(stateSize,
                             (uint16_t)(kSamsungAcExtendedStateLength));
      }

      // Lastly, it should never exceed the maximum "extended" size.
      stateSize = std::min(stateSize, kSamsungAcExtendedStateLength);
      break;
    default: // Everything else.
      stateSize = (IRsend::defaultBits(irType) + 7) / 8;

      if (!stateSize || !hasACState(irType)) {
        // Not a protocol we expected. Abort.
        // debug("Unexpected AirCon protocol detected. Ignoring.");
        return false;
      }
  }

  if (inputLength > stateSize * 2) {
    // debug("AirCon code to large for the given protocol.");
    return false;
  }

  // Ptr to the least significant uint8_t of the resulting state for this protocol.
  uint8_t *statePtr = &state[stateSize - 1];

  // Convert the string into a state array of the correct length.
  for (uint16_t i = 0; i < inputLength; i++) {
    // Grab the next least sigificant hexadecimal digit from the string.
    uint8_t c = tolower(str[inputLength + strOffset - i - 1]);

    if (isxdigit(c)) {
      if (isdigit(c)) {
        c -= '0';
      } else {
        c = c - 'a' + 10;
      }
    } else {
      // debug("Aborting! Non-hexadecimal char found in AirCon state:");
      // debug(str.c_str());
      return false;
    }

    if (i % 2 == 1) { // Odd: Upper half of the uint8_t.
      *statePtr += (c << 4);
      statePtr--;     // Advance up to the next least significant uint8_t of state.
    } else {          // Even: Lower half of the uint8_t.
      *statePtr = c;
    }
  }

  if (!Plugin_035_irSender->send(irType, state, stateSize)) {
    // debug("Unexpected AirCon type in send request. Not sent.");
    return false;
  }
  return true; // We were successful as far as we can tell.
}

#endif // ifdef USES_P035

#include "../PluginStructs/P073_data_struct.h"

#ifdef USES_P073

P073_data_struct::P073_data_struct()
  : dotpos(-1), pin1(-1), pin2(-1), pin3(-1), displayModel(0), output(0),
  brightness(0), timesep(false), shift(false), periods(false), hideDegree(false),
  rightAlignTempMAX7219(false), fontset(0)
  # ifdef P073_7DBIN_COMMAND
  , binaryData(false)
  # endif // P073_7DBIN_COMMAND
  # ifdef P073_SCROLL_TEXT
  , txtScrolling(false), scrollCount(0), scrollPos(0), scrollFull(false)
  , _scrollSpeed(0)
  # endif // P073_SCROLL_TEXT
{
  ClearBuffer();
}

void P073_data_struct::FillBufferWithTime(boolean sevendgt_now,
                                          uint8_t sevendgt_hours,
                                          uint8_t sevendgt_minutes,
                                          uint8_t sevendgt_seconds,
                                          boolean flag12h) {
  ClearBuffer();

  if (sevendgt_now) {
    sevendgt_hours   = node_time.hour();
    sevendgt_minutes = node_time.minute();
    sevendgt_seconds = node_time.second();
  }

  if (flag12h && (sevendgt_hours > 12)) {
    sevendgt_hours -= 12; // if flag 12h is TRUE and h>12 adjust subtracting 12
  }

  if (flag12h && (sevendgt_hours == 0)) {
    sevendgt_hours = 12; // if flag 12h is TRUE and h=0  adjust to h=12
  }
  showbuffer[0] = static_cast<uint8_t>(sevendgt_hours / 10);
  showbuffer[1] = sevendgt_hours % 10;
  showbuffer[2] = static_cast<uint8_t>(sevendgt_minutes / 10);
  showbuffer[3] = sevendgt_minutes % 10;
  showbuffer[4] = static_cast<uint8_t>(sevendgt_seconds / 10);
  showbuffer[5] = sevendgt_seconds % 10;
}

void P073_data_struct::FillBufferWithDate(boolean sevendgt_now,
                                          uint8_t sevendgt_day,
                                          uint8_t sevendgt_month,
                                          int     sevendgt_year) {
  ClearBuffer();
  int sevendgt_year0 = sevendgt_year;

  if (sevendgt_now) {
    sevendgt_day   = node_time.day();
    sevendgt_month = node_time.month();
    sevendgt_year0 = node_time.year();
  } else if (sevendgt_year0 < 100) {
    sevendgt_year0 += 2000;
  }
  const uint8_t sevendgt_year1 = static_cast<uint8_t>(sevendgt_year0 / 100);
  const uint8_t sevendgt_year2 = static_cast<uint8_t>(sevendgt_year0 % 100);

  showbuffer[0] = static_cast<uint8_t>(sevendgt_day / 10);
  showbuffer[1] = sevendgt_day % 10;
  showbuffer[2] = static_cast<uint8_t>(sevendgt_month / 10);
  showbuffer[3] = sevendgt_month % 10;
  showbuffer[4] = static_cast<uint8_t>(sevendgt_year1 / 10);
  showbuffer[5] = sevendgt_year1 % 10;
  showbuffer[6] = static_cast<uint8_t>(sevendgt_year2 / 10);
  showbuffer[7] = sevendgt_year2 % 10;
}

void P073_data_struct::FillBufferWithNumber(const String& number) {
  ClearBuffer();
  uint8_t p073_index = 7;

  dotpos = -1; // -1 means no dot to display

  for (int i = number.length() - 1; i >= 0 && p073_index >= 0; --i) {
    const char p073_tmpchar = number.charAt(i);

    if (p073_tmpchar == '.') { // dot
      dotpos = p073_index;
    } else {
      showbuffer[p073_index] = mapCharToFontPosition(p073_tmpchar, fontset);
      p073_index--;
    }
  }
}

void P073_data_struct::FillBufferWithTemp(long temperature) {
  ClearBuffer();
  char p073_digit[8];
  bool between10and0 = ((temperature < 10) && (temperature >= 0));      // To have a zero prefix (0.x and -0.x) display between 0.9 and
                                                                        // -0.9
  bool between0andMinus10 = ((temperature < 0) && (temperature > -10)); // degrees,as all display types use 1 digit for temperatures
                                                                        // between 10.0 and -10.0
  String format;

  if (hideDegree) {
    format = (between10and0 ? F("      %02d") : (between0andMinus10 ? F("     %03d") : F("%8d")));
  } else {
    format = (between10and0 ? F("     %02d") : (between0andMinus10 ? F("    %03d") : F("%7d")));
  }
  sprintf_P(p073_digit, format.c_str(), static_cast<int>(temperature));
  int p073_numlenght = strlen(p073_digit);

  for (int i = 0; i < p073_numlenght; i++) {
    showbuffer[i] = mapCharToFontPosition(p073_digit[i], fontset);
  }

  if (!hideDegree) {
    showbuffer[7] = 12; // degree "°"
  }
}

# ifdef P073_7DDT_COMMAND

/**
 * FillBufferWithDualTemp()
 * leftTemperature or rightTempareature < -100.0 then shows dashes
 */
void P073_data_struct::FillBufferWithDualTemp(long leftTemperature,
                                              bool leftWithDecimal,
                                              long rightTemperature,
                                              bool rightWithDecimal) {
  ClearBuffer();
  char   p073_digit[8];
  String format;
  bool   leftBetween10and0 = (leftWithDecimal && (leftTemperature < 10) && (leftTemperature >= 0));

  // To have a zero prefix (0.x and -0.x) display between 0.9 and -0.9 degrees,
  // as all display types use 1 digit for temperatures between 10.0 and -10.0
  bool leftBetween0andMinus10 = (leftWithDecimal && (leftTemperature < 0) && (leftTemperature > -10));

  if (hideDegree) {
    // Include a space for compensation of the degree symbol
    format = (leftBetween10and0 ? F("  %02d") : (leftBetween0andMinus10 ? F(" %03d") : leftTemperature < -1000 ? F("----") : F("%4d")));
  } else {
    // Include a space for compensation of the degree symbol
    format = (leftBetween10and0 ? F(" %02d ") : (leftBetween0andMinus10 ? F("%03d ") : leftTemperature < -100 ? F("----") : F("%3d ")));
  }
  bool rightBetween10and0 = (rightWithDecimal && (rightTemperature < 10) && (rightTemperature >= 0));

  // To have a zero prefix (0.x and -0.x) display between 0.9 and -0.9 degrees,
  // as all display types use 1 digit for temperatures between 10.0 and -10.0
  bool rightBetween0andMinus10 = (rightWithDecimal && (rightTemperature < 0) && (rightTemperature > -10));

  if (hideDegree) {
    format += (rightBetween10and0 ? F("  %02d") : (rightBetween0andMinus10 ? F(" %03d") : rightTemperature < -1000 ? F("----") : F("%4d")));
  } else {
    format += (rightBetween10and0 ? F(" %02d") : (rightBetween0andMinus10 ? F("%03d") : rightTemperature < -100 ? F("----") : F("%3d")));
  }
  sprintf_P(p073_digit, format.c_str(), static_cast<int>(leftTemperature), static_cast<int>(rightTemperature));
  const int p073_numlenght = strlen(p073_digit);

  for (int i = 0; i < p073_numlenght; i++) {
    showbuffer[i] = mapCharToFontPosition(p073_digit[i], fontset);
  }

  if (!hideDegree) {
    if (leftTemperature  > -100.0) {
      showbuffer[3] = 12; // degree "°"
    }

    if (rightTemperature > -100.0) {
      showbuffer[7] = 12; // degree "°"
    }
  }

  // addLog(LOG_LEVEL_INFO, String(F("7dgt format")) + format);
}

# endif // ifdef P073_7DDT_COMMAND

void P073_data_struct::FillBufferWithString(const String& textToShow,
                                            bool          useBinaryData) {
  # ifdef P073_7DBIN_COMMAND
  binaryData = useBinaryData;
  # endif // P073_7DBIN_COMMAND
  ClearBuffer();
  const int p073_txtlength = textToShow.length();

  int p = 0;

  for (int i = 0; i < p073_txtlength && p <= 8; i++) { // p <= 8 to allow a period after last digit
    if (periods
        && textToShow.charAt(i) == '.'
        # ifdef P073_7DBIN_COMMAND
        && !binaryData
        # endif // P073_7DBIN_COMMAND
        ) {   // If setting periods true
      if (p == 0) {   // Text starts with a period, becomes a space with a dot
        showperiods[p] = true;
        p++;
      } else {
        // if (p > 0) {
        showperiods[p - 1] = true;                        // The period displays as a dot on the previous digit!
      }

      if ((i > 0) && (textToShow.charAt(i - 1) == '.')) { // Handle consecutive periods
        p++;

        if ((p - 1) < 8) {
          showperiods[p - 1] = true; // The period displays as a dot on the previous digit!
        }
      }
    } else if (p < 8) {
      # ifdef P073_7DBIN_COMMAND
      showbuffer[p] = useBinaryData ? textToShow.charAt(i) : mapCharToFontPosition(textToShow.charAt(i), fontset);
      # else // P073_7DBIN_COMMAND
      showbuffer[p] = mapCharToFontPosition(textToShow.charAt(i), fontset);
      # endif // P073_7DBIN_COMMAND
      p++;
    }
  }
  # ifdef P073_DEBUG
  LogBufferContent(F("7dtext"));
  # endif // ifdef P073_DEBUG
}

# ifdef P073_SCROLL_TEXT
uint8_t P073_data_struct::getBufferLength(uint8_t displayModel) {
  uint8_t bufLen = 0;

  switch (displayModel) {
    case P073_TM1637_4DGTCOLON:
    case P073_TM1637_4DGTDOTS:
      bufLen = 4;
      break;
    case P073_TM1637_6DGT:
      bufLen = 6;
      break;
    case P073_MAX7219_8DGT:
      bufLen = 8;
      break;
  }
  return bufLen;
}

int P073_data_struct::getEffectiveTextLength(const String& text) {
  const int textLength = text.length();
  int p                = 0;

  for (int i = 0; i < textLength; i++) {
    if (periods && (text.charAt(i) == '.')) { // If setting periods true
      if (p == 0) {                           // Text starts with a period, becomes a space with a dot
        p++;
      }

      if ((i > 0) && (text.charAt(i - 1) == '.')) { // Handle consecutive periods
        p++;
      }
    } else {
      p++;
    }
  }
  return p;
}

void P073_data_struct::NextScroll() {
  if (txtScrolling && (!_textToScroll.isEmpty())) {
    if ((scrollCount > 0) && (scrollCount < 0xFFFF)) { scrollCount--; }

    if (scrollCount == 0) {
      scrollCount = 0xFFFF; // Max value to avoid interference when scrolling long texts
      const int bufToFill      = getBufferLength(displayModel);
      const int p073_txtlength = _textToScroll.length();
      ClearBuffer();

      int p = 0;

      for (int i = scrollPos; i < p073_txtlength && p <= bufToFill; i++) { // p <= bufToFill to allow a period after last digit
        if (periods
            && _textToScroll.charAt(i) == '.'
            #  ifdef P073_7DBIN_COMMAND
            && !binaryData
            #  endif // P073_7DBIN_COMMAND
            ) {   // If setting periods true
          if (p == 0) {   // Text starts with a period, becomes a space with a dot
            showperiods[p] = true;
            p++;
          } else {
            showperiods[p - 1] = true;                                   // The period displays as a dot on the previous digit!
          }

          if ((i > scrollPos) && (_textToScroll.charAt(i - 1) == '.')) { // Handle consecutive periods
            showperiods[p - 1] = true;                                   // The period displays as a dot on the previous digit!
            p++;
          }
        } else if (p < bufToFill) {
          #  ifdef P073_7DBIN_COMMAND
          showbuffer[p] = binaryData ?
                          _textToScroll.charAt(i) :
                          mapCharToFontPosition(_textToScroll.charAt(i), fontset);
          #  else // P073_7DBIN_COMMAND
          showbuffer[p] = mapCharToFontPosition(_textToScroll.charAt(i), fontset);
          #  endif // P073_7DBIN_COMMAND
          p++;
        }
      }
      scrollPos++;

      if (scrollPos > _textToScroll.length() - bufToFill) {
        scrollPos = 0;            // Restart when all text displayed
      }
      scrollCount = _scrollSpeed; // Restart countdown
      #  ifdef P073_DEBUG
      LogBufferContent(F("nextScroll"));
      #  endif // P073_DEBUG
    }
  }
}

void P073_data_struct::setTextToScroll(const String& text) {
  _textToScroll = String();

  if (!text.isEmpty()) {
    const int bufToFill = getBufferLength(displayModel);
    _textToScroll.reserve(text.length() + bufToFill + (scrollFull ? bufToFill : 0));

    for (int i = 0; scrollFull && i < bufToFill; i++) { // Scroll text in from the right, so start with all spaces
      _textToScroll +=
        #  ifdef P073_7DBIN_COMMAND
        binaryData ? (char)0x00 :
        #  endif // P073_7DBIN_COMMAND
        ' ';
    }
    _textToScroll += text;

    for (int i = 0; i < bufToFill; i++) { // Scroll text off completely before restarting
      _textToScroll +=
        #  ifdef P073_7DBIN_COMMAND
        binaryData ? (char)0x00 :
        #  endif // P073_7DBIN_COMMAND
        ' ';
    }
  }
  scrollCount = _scrollSpeed;
  scrollPos   = 0;
  #  ifdef P073_7DBIN_COMMAND
  binaryData = false;
  #  endif // P073_7DBIN_COMMAND
}

void P073_data_struct::setScrollSpeed(uint8_t speed) {
  _scrollSpeed = speed;
  scrollCount  = _scrollSpeed;
  scrollPos    = 0;
}

# endif // P073_SCROLL_TEXT

# ifdef P073_7DBIN_COMMAND
void P073_data_struct::setBinaryData(const String& data) {
  binaryData = true;
  #  ifdef P073_SCROLL_TEXT
  setTextToScroll(data);
  binaryData  = true; // is reset in setTextToScroll
  scrollCount = _scrollSpeed;
  scrollPos   = 0;
  #  else // P073_SCROLL_TEXT
  _textToScroll = data;
  #  endif // P073_SCROLL_TEXT
}

# endif      // P073_7DBIN_COMMAND

# ifdef P073_DEBUG
void P073_data_struct::LogBufferContent(String prefix) {
  String log;

  if (loglevelActiveFor(LOG_LEVEL_INFO) &&
      log.reserve(48)) {
    log  = prefix;
    log += F(" buffer: periods: ");
    log += periods ? 't' : 'f';
    log += ' ';

    for (uint8_t i = 0; i < 8; i++) {
      if (i > 0) { log += ','; }
      log += F("0x");
      log += String(showbuffer[i], HEX);
      log += ',';
      log += showperiods[i] ? F(".") : F("");
    }
    addLogMove(LOG_LEVEL_INFO, log);
  }
}

# endif // P073_DEBUG

// in case of error show all dashes
void P073_data_struct::FillBufferWithDash() {
  memset(showbuffer, 11, sizeof(showbuffer));
}

void P073_data_struct::ClearBuffer() {
  memset(showbuffer,
         # ifdef P073_7DBIN_COMMAND
         binaryData ? 0 :
         # endif // P073_7DBIN_COMMAND
         10, sizeof(showbuffer));

  for (uint8_t i = 0; i < 8; i++) {
    showperiods[i] = false;
  }
}

uint8_t P073_data_struct::mapCharToFontPosition(char    character,
                                                uint8_t fontset) {
  uint8_t position = 10;

  # ifdef P073_EXTRA_FONTS
  String specialChars = F(" -^=/_%@.,;:+*#!?'\"<>\\()|");
  String chnorux      = F("CHNORUX");

  switch (fontset) {
    case 1: // Siekoo
    case 2: // Siekoo with uppercase 'CHNORUX'

      if ((fontset == 2) && (chnorux.indexOf(character) > -1)) {
        position = chnorux.indexOf(character) + 35;
      } else if (isDigit(character)) {
        position = character - '0';
      } else if (isAlpha(character)) {
        position = character - (isLowerCase(character) ? 'a' : 'A') + 42;
      } else {
        int idx = specialChars.indexOf(character);

        if (idx > -1) {
          position = idx + 10;
        }
      }
      break;
    case 3:  // dSEG7 (same table size as 7Dgt)
    default: // Original fontset (7Dgt)
  # endif // P073_EXTRA_FONTS

  if (isDigit(character)) {
    position = character - '0';
  } else if (isAlpha(character)) {
    position = character - (isLowerCase(character) ? 'a' : 'A') + 16;
  } else {
    switch (character) {
      case ' ': position = 10; break;
      case '-': position = 11; break;
      case '^': position = 12; break; // degree
      case '=': position = 13; break;
      case '/': position = 14; break;
      case '_': position = 15; break;
    }
  }
  # ifdef P073_EXTRA_FONTS
}

  # endif // P073_EXTRA_FONTS
  return position;
}

uint8_t P073_data_struct::mapMAX7219FontToTM1673Font(uint8_t character) {
  uint8_t newCharacter = character & 0x80; // Keep dot-bit if passed in

  for (int b = 0; b < 7; b++) {
    if (character & (0x01 << b)) {
      newCharacter |= (0x40 >> b);
    }
  }
  return newCharacter;
}

uint8_t P073_data_struct::tm1637_getFontChar(uint8_t index,
                                             uint8_t fontset) {
  # ifdef P073_EXTRA_FONTS

  switch (fontset) {
    case 1:                                                                        // Siekoo
    case 2:                                                                        // Siekoo uppercase CHNORUX
      return mapMAX7219FontToTM1673Font(pgm_read_byte(&(SiekooCharTable[index]))); // SiekooTableTM1637[index];
    case 3:                                                                        // dSEG7
      return mapMAX7219FontToTM1673Font(pgm_read_byte(&(Dseg7CharTable[index])));  // Dseg7TableTM1637[index];
    default:                                                                       // Standard fontset
  # endif // P073_EXTRA_FONTS
  return mapMAX7219FontToTM1673Font(pgm_read_byte(&(DefaultCharTable[index])));    // CharTableTM1637[index];
  # ifdef P073_EXTRA_FONTS
} // Out of wack because of the conditional compilation ifdef's

  # endif // P073_EXTRA_FONTS
}

#endif    // ifdef USES_P073

#include "../PluginStructs/P020_data_struct.h"

#ifdef USES_P020

# include "../ESPEasyCore/Serial.h"
# include "../ESPEasyCore/ESPEasyNetwork.h"

# include "../Globals/EventQueue.h"

# include "../Helpers/ESPEasy_Storage.h"
# include "../Helpers/Misc.h"

# define P020_RX_WAIT              PCONFIG(4)
# define P020_RX_BUFFER            PCONFIG(7)


P020_Task::P020_Task(taskIndex_t taskIndex) : _taskIndex(taskIndex) {
  clearBuffer();
}

P020_Task::~P020_Task() {
  stopServer();
  serialEnd();
}

bool P020_Task::serverActive(WiFiServer *server) {
# if defined(ESP8266)
  return nullptr != server && server->status() != CLOSED;
# elif defined(ESP32)
  return nullptr != server && *server;
# endif // if defined(ESP8266)
}

void P020_Task::startServer(uint16_t portnumber) {
  if ((gatewayPort == portnumber) && serverActive(ser2netServer)) {
    // server is already listening on this port
    return;
  }
  stopServer();
  gatewayPort   = portnumber;
  ser2netServer = new (std::nothrow) WiFiServer(portnumber);

  if ((nullptr != ser2netServer) && NetworkConnected()) {
    ser2netServer->begin();

    if (serverActive(ser2netServer)) {
      addLog(LOG_LEVEL_INFO, String(F("Ser2Net  : WiFi server started at port ")) + portnumber);
    } else {
      addLog(LOG_LEVEL_ERROR, String(F("Ser2Net   : WiFi server start failed at port ")) +
             portnumber + String(F(", retrying...")));
    }
  }
}

void P020_Task::checkServer() {
  if ((nullptr != ser2netServer) && !serverActive(ser2netServer) && NetworkConnected()) {
    ser2netServer->close();
    ser2netServer->begin();

    if (serverActive(ser2netServer)) {
      addLog(LOG_LEVEL_INFO, F("Ser2net   : WiFi server started"));
    }
  }
}

void P020_Task::stopServer() {
  if (nullptr != ser2netServer) {
    if (ser2netClient) { ser2netClient.stop(); }
    clientConnected = false;
    ser2netServer->close();
    addLog(LOG_LEVEL_INFO, F("Ser2net   : WiFi server closed"));
    delete ser2netServer;
    ser2netServer = nullptr;
  }
}

bool P020_Task::hasClientConnected() {
  if ((nullptr != ser2netServer) && ser2netServer->hasClient())
  {
    if (ser2netClient) { ser2netClient.stop(); }
    ser2netClient = ser2netServer->available();
    ser2netClient.setTimeout(CONTROLLER_CLIENTTIMEOUT_DFLT);
    sendConnectedEvent(true);
    addLog(LOG_LEVEL_INFO, F("Ser2Net   : Client connected!"));
  }

  if (ser2netClient.connected())
  {
    clientConnected = true;
  }
  else
  {
    if (clientConnected) // there was a client connected before...
    {
      clientConnected = false;
      sendConnectedEvent(false);
      addLog(LOG_LEVEL_INFO, F("Ser2net   : Client disconnected!"));
    }
  }
  return clientConnected;
}

void P020_Task::discardClientIn() {
  // flush all data received from the WiFi gateway
  // as a P1 meter does not receive data
  while (ser2netClient.available()) {
    ser2netClient.read();
  }
}

void P020_Task::clearBuffer() {
  serial_buffer = String();
  serial_buffer.reserve(P020_DATAGRAM_MAX_SIZE);
}

void P020_Task::serialBegin(const ESPEasySerialPort port, int16_t rxPin, int16_t txPin, unsigned long baud, uint8_t config) {
  serialEnd();

  if (rxPin >= 0) {
    ser2netSerial = new (std::nothrow) ESPeasySerial(port, rxPin, txPin);

    if (nullptr != ser2netSerial) {
      # if defined(ESP8266)
      ser2netSerial->begin(baud, (SerialConfig)config);
      # elif defined(ESP32)
      ser2netSerial->begin(baud, config);
      # endif // if defined(ESP8266)
      # ifndef BUILD_NO_DEBUG
      addLog(LOG_LEVEL_DEBUG, F("Ser2net   : Serial opened"));
      # endif // ifndef BUILD_NO_DEBUG
    }
  }
}

void P020_Task::serialEnd() {
  if (nullptr != ser2netSerial) {
    delete ser2netSerial;
    clearBuffer();
    ser2netSerial = nullptr;
    # ifndef BUILD_NO_DEBUG
    addLog(LOG_LEVEL_DEBUG, F("Ser2net   : Serial closed"));
    # endif // ifndef BUILD_NO_DEBUG
  }
}

void P020_Task::handleClientIn(struct EventStruct *event) {
  int count      = ser2netClient.available();
  int bytes_read = 0;
  uint8_t net_buf[P020_DATAGRAM_MAX_SIZE];

  if (count > 0) {
    if (count > P020_DATAGRAM_MAX_SIZE) { count = P020_DATAGRAM_MAX_SIZE; }
    bytes_read = ser2netClient.read(net_buf, count);
    ser2netSerial->write(net_buf, bytes_read);
    ser2netSerial->flush();             // Waits for the transmission of outgoing serial data to

    while (ser2netClient.available()) { // flush overflow data if available
      ser2netClient.read();
    }
  }
}

void P020_Task::handleSerialIn(struct EventStruct *event) {
  if (nullptr == ser2netSerial) { return; }
  int RXWait  = P020_RX_WAIT;
  int timeOut = RXWait;

  do {
    if (ser2netSerial->available()) {
      if (serial_buffer.length() > static_cast<size_t>(P020_RX_BUFFER)) {
        # ifndef BUILD_NO_DEBUG
        addLog(LOG_LEVEL_DEBUG, F("Ser2Net   : Error: Buffer overflow, discarded input."));
        # endif // ifndef BUILD_NO_DEBUG
        ser2netSerial->read();
      }
      else { serial_buffer += (char)ser2netSerial->read(); }
      timeOut = RXWait; // if serial received, reset timeout counter
    } else {
      if (timeOut <= 0) { break; }
      delay(1);
      --timeOut;
    }
  } while (true);

  if (serial_buffer.length() > 0) {
    if (ser2netClient.connected()) { // Only send out if a client is connected
      ser2netClient.print(serial_buffer);
    }
    rulesEngine(serial_buffer);
    ser2netClient.flush();
    clearBuffer();
    # ifndef BUILD_NO_DEBUG
    addLog(LOG_LEVEL_DEBUG, F("Ser2Net   : data send!"));
    # endif // ifndef BUILD_NO_DEBUG
  } // done
}

void P020_Task::discardSerialIn() {
  if (nullptr != ser2netSerial) {
    while (ser2netSerial->available()) {
      ser2netSerial->read();
    }
  }
}

// We can also use the rules engine for local control!
void P020_Task::rulesEngine(const String& message) {
  if (!Settings.UseRules || message.isEmpty()) { return; }
  int NewLinePos    = 0;
  uint16_t StartPos = 0;

  NewLinePos = message.indexOf('\n', StartPos);

  do {
    if (NewLinePos < 0) {
      NewLinePos = message.length();
    }

    String eventString;

    if ((NewLinePos - StartPos) + 10 > 12) {
      eventString.reserve((NewLinePos - StartPos) + 10); // Include the prefix
    }

    // Remove preceeding CR also
    if ((message[NewLinePos] == '\n') && (message[NewLinePos - 1] == '\r')) {
      NewLinePos--;
    }

    switch (serial_processing) {
      case 0: { break; }
      case 1: { // Generic
        if (NewLinePos > StartPos) {
          eventString  = F("!Serial#");
          eventString += message.substring(StartPos, NewLinePos);
        }
        break;
      }
      case 2: {                          // RFLink
        StartPos += 6;                   // RFLink, strip 20;xx; from incoming message

        if (message.substring(StartPos, NewLinePos)
            .startsWith(F("ESPEASY"))) { // Special treatment for gpio values, strip unneeded parts...
          StartPos   += 8;               // Strip "ESPEASY;"
          eventString = F("RFLink#");
        } else {
          eventString = F("!RFLink#");   // default event as it comes in, literal match needed in rules, using '!'
        }

        if (NewLinePos > StartPos) {
          eventString += message.substring(StartPos, NewLinePos);
        }
        break;
      }
    } // switch

    // Skip CR/LF
    StartPos = NewLinePos; // Continue after what was already handled

    while (StartPos < message.length() && (message[StartPos] == '\n' || message[StartPos] == '\r')) {
      StartPos++;
    }

    if (!eventString.isEmpty()) {
      eventQueue.add(eventString);
    }
    NewLinePos = message.indexOf('\n', StartPos);

    if (handleMultiLine && (NewLinePos < 0)) {
      NewLinePos = message.length();
    }
  } while (handleMultiLine && NewLinePos > StartPos);
}

bool P020_Task::isInit() const {
  return nullptr != ser2netServer && nullptr != ser2netSerial;
}

void P020_Task::sendConnectedEvent(bool connected)
{
  if (Settings.UseRules)
  {
    String RuleEvent;
    RuleEvent += getTaskDeviceName(_taskIndex);
    RuleEvent += '#';
    RuleEvent += F("Client");
    RuleEvent += '=';
    RuleEvent += (connected ? 1 : 0);
    eventQueue.addMove(std::move(RuleEvent));
  }
}

#endif // ifdef USES_P020

#include "../PluginStructs/P111_data_struct.h"

#ifdef USES_P111

#include "../PluginStructs/P111_data_struct.h"
// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter

#include <MFRC522.h>

P111_data_struct::P111_data_struct(uint8_t csPin, uint8_t rstPin) : mfrc522(nullptr), _csPin(csPin), _rstPin(rstPin)
{}

P111_data_struct::~P111_data_struct() {
  if (mfrc522 != nullptr) {
    delete mfrc522;
    mfrc522 = nullptr;
  }
}

void P111_data_struct::init() {
  if (mfrc522 != nullptr) {
    delete mfrc522;
    mfrc522 = nullptr;
  }
  mfrc522 = new (std::nothrow) MFRC522(_csPin, _rstPin);   // Instantiate a MFRC522
  if (mfrc522 != nullptr) {
    mfrc522->PCD_Init();  // Initialize MFRC522 reader
  }
}

/**
 * read status and tag
 */
uint8_t P111_data_struct::readCardStatus(unsigned long *key, bool *removedTag) {

  uint8_t error = 0;

  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
  uint8_t uidLength;

  error = readPassiveTargetID(uid, &uidLength);

  switch(error) {
    case 1: // Read error
    {
      errorCount++;
      removedState = false;
      if (loglevelActiveFor(LOG_LEVEL_ERROR)) {
        String log = F("MFRC522: Read error: ");
        log += errorCount;
        addLogMove(LOG_LEVEL_ERROR, log);
      }
      break;
    } 
    case 2: // No tag found
      if (!removedState) {
        removedState = true;
        *removedTag  = true;
        error = 0; // pass through that removal just once
      }
      errorCount = 0;
      break;
    default:  // Read a tag correctly
      errorCount = 0;
      removedState = false; // No longer removed
      break;
  }

  if (errorCount > 2) { // if three consecutive read errors, reset MFRC522
    reset(_csPin,_rstPin);
  }
  unsigned long tmpKey = uid[0];
  for (uint8_t i = 1; i < 4; i++) {
    tmpKey <<= 8;
    tmpKey += uid[i];
  }
  *key = tmpKey;

  return error;
}

/**
 * Returns last read card (type) name
 */
String P111_data_struct::getCardName() {
  return mfrc522->PICC_GetTypeName(mfrc522->PICC_GetType(mfrc522->uid.sak));
}

/*********************************************************************************************\
 * MFRC522 init
\*********************************************************************************************/
bool P111_data_struct::reset(int8_t csPin, int8_t resetPin) {
  if (resetPin != -1) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("MFRC522: Reset on pin: ");
      log += resetPin;
      addLogMove(LOG_LEVEL_INFO, log);
    }
    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, LOW);
    delay(100);
    digitalWrite(resetPin, HIGH);
    pinMode(resetPin, INPUT_PULLUP);
    delay(10);
  }

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, LOW);
    
  mfrc522->PCD_Init(csPin, resetPin);   // Init MFRC522 module

  //If you set Antenna Gain to Max it will increase reading distance
  mfrc522->PCD_SetAntennaGain(mfrc522->RxGain_max);
  
  bool result = mfrc522->PCD_PerformSelfTest(); // perform the test
  
  if (result) {
    //String log = F("RC522: Found");
    // Get the MFRC522 software version
    uint8_t v = mfrc522->PCD_ReadRegister(mfrc522->VersionReg);
    
    // When 0x00 or 0xFF is returned, communication probably failed
    if ((v == 0x00) || (v == 0xFF)) {
      addLog(LOG_LEVEL_ERROR, F("MFRC522: Communication failure, is the MFRC522 properly connected?"));
      return false;
    } else {
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log=F("MFRC522: Software Version: ");
        if (v == 0x91)
          log+=F(" = v1.0");
        else if (v == 0x92)
          log+=F(" = v2.0");
        else
          log+=F(" (unknown),probably a chinese clone?");
              
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }
    return true;
  }
  return false;
}

/*********************************************************************************************\
 * RC522 read tag ID
\*********************************************************************************************/
uint8_t P111_data_struct::readPassiveTargetID(uint8_t *uid, uint8_t *uidLength) { //needed ? see above (not PN532)
  // Getting ready for Reading PICCs
  if ( ! mfrc522->PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
    return 2;
  }
  addLog(LOG_LEVEL_INFO, F("MFRC522: New Card Detected"));
  if ( ! mfrc522->PICC_ReadCardSerial()) {   //Since a PICC placed get Serial and continue
    return 1;
  }
  
  // There are Mifare PICCs which have 4 uint8_t or 7 uint8_t UID care if you use 7 uint8_t PICC
  // I think we should assume every PICC as they have 4 uint8_t UID
  // Until we support 7 uint8_t PICCs
  addLog(LOG_LEVEL_INFO, F("MFRC522: Scanned PICC's UID"));
  for (uint8_t i = 0; i < 4; i++) {  //
    uid[i] = mfrc522->uid.uidByte[i];
  }
  *uidLength = 4;
  mfrc522->PICC_HaltA(); // Stop reading
  return 0;
}


#endif // ifdef USES_P111

#include "../PluginStructs/P064_data_struct.h"

// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter
#include <SparkFun_APDS9960.h> // Lib is modified to work with ESP

#ifdef USES_P064

P064_data_struct::P064_data_struct() {}


#endif // ifdef USES_P064

#include "../PluginStructs/P082_data_struct.h"

#ifdef USES_P082


// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter
# include <TinyGPS++.h>
# include <ESPeasySerial.h>


const __FlashStringHelper * Plugin_082_valuename(P082_query value_nr, bool displayString) {
  switch (value_nr) {
    case P082_query::P082_QUERY_LONG:        return displayString ? F("Longitude")          : F("long");
    case P082_query::P082_QUERY_LAT:         return displayString ? F("Latitude")           : F("lat");
    case P082_query::P082_QUERY_ALT:         return displayString ? F("Altitude")           : F("alt");
    case P082_query::P082_QUERY_SPD:         return displayString ? F("Speed (m/s)")        : F("spd");
    case P082_query::P082_QUERY_SATVIS:      return displayString ? F("Satellites Visible") : F("sat_vis");
    case P082_query::P082_QUERY_SATUSE:      return displayString ? F("Satellites Tracked") : F("sat_tr");
    case P082_query::P082_QUERY_HDOP:        return displayString ? F("HDOP")               : F("hdop");
    case P082_query::P082_QUERY_FIXQ:        return displayString ? F("Fix Quality")        : F("fix_qual");
    case P082_query::P082_QUERY_DB_MAX:      return displayString ? F("Max SNR in dBHz")    : F("snr_max");
    case P082_query::P082_QUERY_CHKSUM_FAIL: return displayString ? F("Checksum Fail")      : F("chksum_fail");
    case P082_query::P082_QUERY_DISTANCE:    return displayString ? F("Distance (ODO)")     : F("dist");
    case P082_query::P082_QUERY_DIST_REF:    return displayString ? F("Distance from Reference Point") : F("dist_ref");
    case P082_query::P082_NR_OUTPUT_OPTIONS: break;
  }
  return F("");
}

P082_query Plugin_082_from_valuename(const String& valuename)
{
  for (uint8_t query = 0; query < static_cast<uint8_t>(P082_query::P082_NR_OUTPUT_OPTIONS); ++query) {
    if (valuename.equalsIgnoreCase(Plugin_082_valuename(static_cast<P082_query>(query), false))) {
      return static_cast<P082_query>(query);
    }
  }
  return P082_query::P082_NR_OUTPUT_OPTIONS;
}

const __FlashStringHelper* toString(P082_PowerMode mode) {
  switch (mode) {
    case P082_PowerMode::Max_Performance: return F("Max Performance");
    case P082_PowerMode::Power_Save:      return F("Power Save");
    case P082_PowerMode::Eco:             return F("ECO");
  }
  return F("");
}

const __FlashStringHelper* toString(P082_DynamicModel model) {
  switch (model) {
    case P082_DynamicModel::Portable:    return F("Portable");
    case P082_DynamicModel::Stationary:  return F("Stationary");
    case P082_DynamicModel::Pedestrian:  return F("Pedestrian");
    case P082_DynamicModel::Automotive:  return F("Automotive");
    case P082_DynamicModel::Sea:         return F("Sea");
    case P082_DynamicModel::Airborne_1g: return F("Airborne_1g");
    case P082_DynamicModel::Airborne_2g: return F("Airborne_2g");
    case P082_DynamicModel::Airborne_4g: return F("Airborne_4g");
    case P082_DynamicModel::Wrist:       return F("Wrist");
    case P082_DynamicModel::Bike:        return F("Bike");
  }
  return F("");
}

P082_data_struct::P082_data_struct() : gps(nullptr), easySerial(nullptr) {}

P082_data_struct::~P082_data_struct() {
  powerDown();
  reset();
}

void P082_data_struct::reset() {
  if (gps != nullptr) {
    delete gps;
    gps = nullptr;
  }

  if (easySerial != nullptr) {
    delete easySerial;
    easySerial = nullptr;
  }
}

bool P082_data_struct::init(ESPEasySerialPort port, const int16_t serial_rx, const int16_t serial_tx) {
  if (serial_rx < 0) {
    return false;
  }
  reset();
  gps        = new (std::nothrow) TinyGPSPlus();
  easySerial = new (std::nothrow) ESPeasySerial(port, serial_rx, serial_tx);

  if (easySerial != nullptr) {
    easySerial->begin(9600);
    wakeUp();
  }
  return isInitialized();
}

bool P082_data_struct::isInitialized() const {
  return gps != nullptr && easySerial != nullptr;
}

bool P082_data_struct::loop() {
  if (!isInitialized()) {
    return false;
  }
  bool completeSentence = false;

  if (easySerial != nullptr) {
    int available           = easySerial->available();
    unsigned long startLoop = millis();

    while (available > 0 && timePassedSince(startLoop) < 10) {
      --available;
      int c = easySerial->read();
      if (c >= 0) {
# ifdef P082_SEND_GPS_TO_LOG
        if (_currentSentence.length() <= 80) {
          // No need to capture more than 80 bytes as a NMEA message is never that long.
          if (c != 0) {
            _currentSentence += static_cast<char>(c);
          }
        }
# endif // ifdef P082_SEND_GPS_TO_LOG

        if (c == 0x85) {
          // Found possible start of u-blox message
          unsigned long timeout = millis() + 200;
          unsigned int bytesRead = 0;
          bool done = false;
          bool ack_nak_read = false;
          while (!timeOutReached(timeout) && !done)
          {
            if (available == 0) {
              available = easySerial->available();
            } else {
              const int c = easySerial->read();
              if (c >= 0) {
                switch (bytesRead) {
                  case 0:
                    if (c != 0x62) {
                      done = true;
                    }
                    ++bytesRead;
                    break;
                  case 1:
                    if (c != 0x05) {
                      done = true;
                    }
                    ++bytesRead;
                    break;
                  case 2:
                    if (c == 0x01) {
                      ack_nak_read = true;
                      addLog(LOG_LEVEL_INFO, F("GPS  : ACK-ACK"));
                    } else if (c == 0x00) {
                      ack_nak_read = true;
                      addLog(LOG_LEVEL_ERROR, F("GPS  : ACK-NAK"));
                    }
                    done = true;
                    break;
                  default:
                    done = true;                    
                    break;
                }
              }
            }
          }
          if (!done) {
            addLog(LOG_LEVEL_ERROR, F("GPS  : Ack/Nack timeout"));
          } else if (!ack_nak_read) {
            addLog(LOG_LEVEL_ERROR, F("GPS  : Unexpected reply"));
          }
        }

        if (gps->encode(c)) {
          // Full sentence received
# ifdef P082_SEND_GPS_TO_LOG
          _lastSentence    = _currentSentence;
          _currentSentence = String();
# endif // ifdef P082_SEND_GPS_TO_LOG
          completeSentence = true;
        } else {
          if (available == 0) {
            available = easySerial->available();
          }
        }
      }
    }
  }
  return completeSentence;
}

bool P082_data_struct::hasFix(unsigned int maxAge_msec) {
  if (!isInitialized()) {
    return false;
  }
  return gps->location.isValid() && gps->location.age() < maxAge_msec;
}

bool P082_data_struct::storeCurPos(unsigned int maxAge_msec) {
  if (!hasFix(maxAge_msec)) {
    return false;
  }

  _distance += distanceSinceLast(maxAge_msec);
  _last_lat = gps->location.lat();
  _last_lng = gps->location.lng();
  return true;
}

// Return the distance in meters compared to last stored position.
// @retval  -1 when no fix.
double P082_data_struct::distanceSinceLast(unsigned int maxAge_msec) {
  if (!hasFix(maxAge_msec)) {
    return -1.0;
  }

  if (((_last_lat < 0.0001) && (_last_lat > -0.0001)) || ((_last_lng < 0.0001) && (_last_lng > -0.0001))) {
    return -1.0;
  }
  return gps->distanceBetween(_last_lat, _last_lng, gps->location.lat(), gps->location.lng());
}

// Return the GPS time stamp, which is in UTC.
// @param age is the time in msec since the last update of the time +
// additional centiseconds given by the GPS.
bool P082_data_struct::getDateTime(struct tm& dateTime, uint32_t& age, bool& pps_sync) {
  if (!isInitialized()) {
    return false;
  }

  if (_pps_time != 0) {
    age      = timePassedSince(_pps_time);
    _pps_time = 0;
    pps_sync = true;

    if ((age > 1000) || (gps->time.age() > age)) {
      return false;
    }
  } else {
    age      = gps->time.age();
    pps_sync = false;
  }

  if (age > P082_TIMESTAMP_AGE) {
    return false;
  }

  if (gps->date.age() > P082_TIMESTAMP_AGE) {
    return false;
  }

  if (!gps->date.isValid() || !gps->time.isValid()) {
    return false;
  }
  dateTime.tm_year = gps->date.year() - 1900;
  dateTime.tm_mon  = gps->date.month() - 1; // GPS month starts at 1, tm_mon at 0
  dateTime.tm_mday = gps->date.day();

  dateTime.tm_hour = gps->time.hour();
  dateTime.tm_min  = gps->time.minute();
  dateTime.tm_sec  = gps->time.second();

  // FIXME TD-er: Must the offset in centisecond be added when pps_sync active?
  if (!pps_sync) {
    age += (gps->time.centisecond() * 10);
  }
  return true;
}

bool P082_data_struct::powerDown() {
  const uint8_t UBLOX_GPSStandby[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B}; 
  return writeToGPS(UBLOX_GPSStandby, sizeof(UBLOX_GPSStandby));
}

bool P082_data_struct::wakeUp() {
  if (isInitialized()) {
    if (easySerial->isTxEnabled()) {
      easySerial->println();   // Send some character to wake it up.
    }
  }
  return false;
}

#ifdef P082_USE_U_BLOX_SPECIFIC
bool P082_data_struct::setPowerMode(P082_PowerMode mode) {
  switch (mode) {
    case P082_PowerMode::Max_Performance: 
    {
      const uint8_t UBLOX_command[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91}; 
      return writeToGPS(UBLOX_command, sizeof(UBLOX_command));
    }
    case P082_PowerMode::Power_Save:      
    {
      const uint8_t UBLOX_command[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92}; 
      return writeToGPS(UBLOX_command, sizeof(UBLOX_command));
    }
    case P082_PowerMode::Eco:             
    {
      const uint8_t UBLOX_command[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x04, 0x25, 0x95}; 
      return writeToGPS(UBLOX_command, sizeof(UBLOX_command));
    }
  }
  return false;
}

bool P082_data_struct::setDynamicModel(P082_DynamicModel model) {

  const uint8_t dynModel = static_cast<uint8_t>(model);
  if (dynModel == 1 || dynModel > 10) {
    return false;
  }

  uint8_t UBLOX_command[] = {
    0xB5, 0x62, // header
    0x06, // class
    0x24, // ID, UBX-CFG-NAV5
    0x24, 0x00, // length
    0x01, 0x00, // mask
    dynModel, // dynModel
    0x03, // fixMode auto 2D/3D
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  setUbloxChecksum(UBLOX_command, sizeof(UBLOX_command));
  return writeToGPS(UBLOX_command, sizeof(UBLOX_command));
}
#endif

#ifdef P082_USE_U_BLOX_SPECIFIC
void P082_data_struct::computeUbloxChecksum(const uint8_t* data, size_t size, uint8_t & CK_A, uint8_t & CK_B) {
  CK_A = 0;
  CK_B = 0;
  for (size_t i = 0; i < size; ++i) {
    CK_A = CK_A + data[i];
    CK_B = CK_B + CK_A;
  }
}

void P082_data_struct::setUbloxChecksum(uint8_t* data, size_t size) {
  uint8_t CK_A;
  uint8_t CK_B;
  computeUbloxChecksum(data + 2, size - 4, CK_A, CK_B);
  data[size - 2] = CK_A;
  data[size - 1] = CK_B;
}
#endif

bool P082_data_struct::writeToGPS(const uint8_t* data, size_t size) {
  if (isInitialized()) {
    if (easySerial->isTxEnabled()) {
      if (size != easySerial->write(data, size)) {
        addLog(LOG_LEVEL_ERROR, F("GPS  : Written less bytes than expected"));
        return false;
      } 
      return true;
    }
  }
  addLog(LOG_LEVEL_ERROR, F("GPS  : Cannot send to GPS"));
  return false;
}

#endif // ifdef USES_P082

#include "../PluginStructs/P085_data_struct.h"

#ifdef USES_P085

P085_data_struct::~P085_data_struct() {
  reset();
}

void P085_data_struct::reset() {
  modbus.reset();
}

bool P085_data_struct::init(ESPEasySerialPort port, const int16_t serial_rx, const int16_t serial_tx, int8_t dere_pin,
                            unsigned int baudrate, uint8_t modbusAddress) {
  return modbus.init(port, serial_rx, serial_tx, baudrate, modbusAddress, dere_pin);
}

bool P085_data_struct::isInitialized() const {
  return modbus.isInitialized();
}

const __FlashStringHelper* Plugin_085_valuename(uint8_t value_nr, bool displayString) {
  switch (value_nr) {
    case P085_QUERY_V:      return displayString ? F("Voltage (V)") : F("V");
    case P085_QUERY_A:      return displayString ? F("Current (A)") : F("A");
    case P085_QUERY_W:      return displayString ? F("Power (W)") : F("W");
    case P085_QUERY_Wh_imp: return displayString ? F("Import Energy (Wh)") : F("Wh_imp");
    case P085_QUERY_Wh_exp: return displayString ? F("Export Energy (Wh)") : F("Wh_exp");
    case P085_QUERY_Wh_tot: return displayString ? F("Total Energy (Wh)") : F("Wh_tot");
    case P085_QUERY_Wh_net: return displayString ? F("Net Energy (Wh)") : F("Wh_net");
    case P085_QUERY_h_tot:  return displayString ? F("Meter Running Time (h)") : F("h_tot");
    case P085_QUERY_h_load: return displayString ? F("Load Running Time (h)") : F("h_load");
  }
  return F("");
}

int p085_storageValueToBaudrate(uint8_t baudrate_setting) {
  switch (baudrate_setting) {
    case 0:
      return 1200;
    case 1:
      return 2400;
    case 2:
      return 4800;
    case 3:
      return 9600;
    case 4:
      return 19200;
    case 5:
      return 38500;
  }
  return 19200;
}

float p085_readValue(uint8_t query, struct EventStruct *event) {
  P085_data_struct *P085_data =
    static_cast<P085_data_struct *>(getPluginTaskData(event->TaskIndex));

  if ((nullptr != P085_data) && P085_data->isInitialized()) {
    switch (query) {
      case P085_QUERY_V:
        return P085_data->modbus.read_float_HoldingRegister(0x200);
      case P085_QUERY_A:
        return P085_data->modbus.read_float_HoldingRegister(0x202);
      case P085_QUERY_W:
        return P085_data->modbus.read_float_HoldingRegister(0x204) * 1000.0f; // power (kW => W)
      case P085_QUERY_Wh_imp:
        return P085_data->modbus.read_32b_HoldingRegister(0x300) * 10.0f;     // 0.01 kWh => Wh
      case P085_QUERY_Wh_exp:
        return P085_data->modbus.read_32b_HoldingRegister(0x302) * 10.0f;     // 0.01 kWh => Wh
      case P085_QUERY_Wh_tot:
        return P085_data->modbus.read_32b_HoldingRegister(0x304) * 10.0f;     // 0.01 kWh => Wh
      case P085_QUERY_Wh_net:
      {
        int64_t intvalue = P085_data->modbus.read_32b_HoldingRegister(0x306);

        if (intvalue >= 2147483648ll) {
          intvalue = 4294967296ll - intvalue;
        }
        float value = static_cast<float>(intvalue);
        value *= 10.0f; // 0.01 kWh => Wh
        return value;
      }
      case P085_QUERY_h_tot:
        return P085_data->modbus.read_32b_HoldingRegister(0x280) / 100.0f;
      case P085_QUERY_h_load:
        return P085_data->modbus.read_32b_HoldingRegister(0x282) / 100.0f;
    }
  }
  return 0.0f;
}

void p085_showValueLoadPage(uint8_t query, struct EventStruct *event) {
  addRowLabel(Plugin_085_valuename(query, true));
  addHtml(String(p085_readValue(query, event)));
}

#endif // ifdef USES_P085

#include "../PluginStructs/P095_data_struct.h"

#ifdef USES_P095

# include "../Helpers/Hardware.h"

/****************************************************************************
 * P095_CommandTrigger_toString: return the command string selected
 ***************************************************************************/
const __FlashStringHelper* P095_CommandTrigger_toString(P095_CommandTrigger cmd) {
  switch (cmd) {
    case P095_CommandTrigger::tft: return F("tft");
    case P095_CommandTrigger::ili9341: return F("ili9341");
    case P095_CommandTrigger::MAX: return F("None");
  }
  return F("ili9341"); // Default command trigger
}

/****************************************************************************
 * Constructor
 ***************************************************************************/
P095_data_struct::P095_data_struct(uint8_t             rotation,
                                   uint8_t             fontscaling,
                                   AdaGFXTextPrintMode textmode,
                                   int8_t              backlightPin,
                                   uint8_t             backlightPercentage,
                                   uint32_t            displayTimer,
                                   String              commandTrigger,
                                   uint16_t            fgcolor,
                                   uint16_t            bgcolor,
                                   bool                textBackFill)
  : _rotation(rotation), _fontscaling(fontscaling), _textmode(textmode), _backlightPin(backlightPin),
  _backlightPercentage(backlightPercentage), _displayTimer(displayTimer), _displayTimeout(displayTimer),
  _commandTrigger(commandTrigger), _fgcolor(fgcolor), _bgcolor(bgcolor), _textBackFill(textBackFill)
{
  _xpix = 240;
  _ypix = 320;

  updateFontMetrics();
  _commandTrigger.toLowerCase();
  _commandTriggerCmd  = _commandTrigger;
  _commandTriggerCmd += F("cmd");
}

/****************************************************************************
 * Destructor
 ***************************************************************************/
P095_data_struct::~P095_data_struct() {
  if (nullptr != gfxHelper) {
    delete gfxHelper;
    gfxHelper = nullptr;
  }

  if (nullptr != tft) {
    delete tft;
    tft = nullptr;
  }
}

/****************************************************************************
 * plugin_init: Initialize display
 ***************************************************************************/
bool P095_data_struct::plugin_init(struct EventStruct *event) {
  bool success = false;

  if (nullptr == tft) {
    addLog(LOG_LEVEL_INFO, F("ILI9341: Init start."));

    tft = new (std::nothrow) Adafruit_ILI9341(PIN(0), PIN(1), PIN(2));

    # ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      log.reserve(65);
      log += F("ILI9341: Init done, address: 0x");
      log += String(reinterpret_cast<ulong>(tft), HEX);
      log += ' ';

      if (nullptr == tft) {
        log += F("in");
      }
      log += F("valid, commands: ");
      log += _commandTrigger;
      log += '/';
      log += _commandTriggerCmd;
      addLogMove(LOG_LEVEL_INFO, log);
    }
    # endif // ifndef BUILD_NO_DEBUG
  } else {
    addLog(LOG_LEVEL_INFO, F("ILI9341: No init?"));
  }

  if (nullptr != tft) {
    gfxHelper = new (std::nothrow) AdafruitGFX_helper(tft,
                                                      _commandTrigger,
                                                      _xpix,
                                                      _ypix,
                                                      AdaGFXColorDepth::FullColor,
                                                      _textmode,
                                                      _fontscaling,
                                                      _fgcolor,
                                                      _bgcolor,
                                                      true,
                                                      _textBackFill);

    if (nullptr != gfxHelper) {
      gfxHelper->setRotation(_rotation);
      gfxHelper->setColumnRowMode(bitRead(P095_CONFIG_FLAGS, P095_CONFIG_FLAG_USE_COL_ROW));
      gfxHelper->setTxtfullCompensation(!bitRead(P095_CONFIG_FLAGS, P095_CONFIG_FLAG_COMPAT_P095) ? 0 : 1);
    }
    updateFontMetrics();
    tft->fillScreen(_bgcolor);             // fill screen with black color
    tft->setTextColor(_fgcolor, _bgcolor); // set text color to white and black background
    tft->setTextSize(_fontscaling);        // Handles 0 properly, text size, default 1 = very small
    tft->setCursor(0, 0);                  // move cursor to position (0, 0) pixel
    displayOnOff(true);
    # ifdef P095_SHOW_SPLASH
    uint16_t yPos = 0;
    gfxHelper->printText(String(F("ESPEasy")).c_str(), 0, yPos, 3, ST77XX_WHITE, ST77XX_BLUE);
    yPos += (3 * _fontheight);
    gfxHelper->printText(String(F("ILI9341")).c_str(), 0, yPos, 2, ST77XX_BLUE,  ST77XX_WHITE);
    delay(100); // Splash
    # endif // ifdef P095_SHOW_SPLASH
    updateFontMetrics();


    if (P095_CONFIG_BUTTON_PIN != -1) {
      pinMode(P095_CONFIG_BUTTON_PIN, INPUT_PULLUP);
    }
    success = true;
  }
  return success;
}

/****************************************************************************
 * updateFontMetrics: recalculate x and y columns, based on font size and font scale
 ***************************************************************************/
void P095_data_struct::updateFontMetrics() {
  if (nullptr != gfxHelper) {
    gfxHelper->getTextMetrics(_textcols, _textrows, _fontwidth, _fontheight, _fontscaling, _heightOffset, _xpix, _ypix);
    gfxHelper->getColors(_fgcolor, _bgcolor);
  } else {
    _textcols = _xpix / (_fontwidth * _fontscaling);
    _textrows = _ypix / (_fontheight * _fontscaling);
  }
}

/****************************************************************************
 * plugin_exit: De-initialize before destruction
 ***************************************************************************/
bool P095_data_struct::plugin_exit(struct EventStruct *event) {
  addLog(LOG_LEVEL_INFO, F("ILI9341: Exit."));

  if ((nullptr != tft) && bitRead(P095_CONFIG_FLAGS, P095_CONFIG_FLAG_CLEAR_ON_EXIT)) {
    tft->fillScreen(ADAGFX_BLACK); // fill screen with black color
    displayOnOff(false);
  }

  if (nullptr != gfxHelper) { delete gfxHelper; }
  gfxHelper = nullptr;

  if (nullptr != tft) {
    // delete tft; // Library is not properly inherited so no destructor called
    free(tft); // Free up some memory without calling the destructor chain
  }
  tft = nullptr;
  return true;
}

/****************************************************************************
 * plugin_read: Re-draw the default content
 ***************************************************************************/
bool P095_data_struct::plugin_read(struct EventStruct *event) {
  if (nullptr != tft) {
    String strings[P095_Nlines];
    LoadCustomTaskSettings(event->TaskIndex, strings, P095_Nlines, 0);

    bool hasContent = false;

    for (uint8_t x = 0; x < P095_Nlines && !hasContent; x++) {
      hasContent = !strings[x].isEmpty();
    }

    if (hasContent) {
      gfxHelper->setColumnRowMode(false); // Turn off column mode

      int yPos = 0;

      for (uint8_t x = 0; x < P095_Nlines; x++) {
        String newString = AdaGFXparseTemplate(strings[x], _textcols, gfxHelper);

        # if ADAGFX_PARSE_SUBCOMMAND
        updateFontMetrics();
        # endif // if ADAGFX_PARSE_SUBCOMMAND

        if (yPos < _ypix) {
          gfxHelper->printText(newString.c_str(), 0, yPos, _fontscaling, _fgcolor, _bgcolor);
        }
        delay(0);
        yPos += (_fontheight * _fontscaling);
      }
      gfxHelper->setColumnRowMode(bitRead(P095_CONFIG_FLAGS, P095_CONFIG_FLAG_USE_COL_ROW)); // Restore column mode
      int16_t curX, curY;
      gfxHelper->getCursorXY(curX, curY);                                                    // Get current X and Y coordinates,
      UserVar[event->BaseVarIndex]     = curX;                                               // and put into Values
      UserVar[event->BaseVarIndex + 1] = curY;
    }
  }
  return false; // Always return false, so no attempt to send to
                // Controllers or generate events is started
}

/****************************************************************************
 * plugin_ten_per_second: check button, if any, that wakes up the display
 ***************************************************************************/
bool P095_data_struct::plugin_ten_per_second(struct EventStruct *event) {
  if ((P095_CONFIG_BUTTON_PIN != -1) && (getButtonState()) && (nullptr != tft)) {
    displayOnOff(true);
    markButtonStateProcessed();
  }
  return true;
}

/****************************************************************************
 * plugin_once_a_second: Count down display timer, if any, and turn display off if countdown reached
 ***************************************************************************/
bool P095_data_struct::plugin_once_a_second(struct EventStruct *event) {
  if (_displayTimer > 0) {
    _displayTimer--;

    if ((nullptr != tft) && (_displayTimer == 0)) {
      displayOnOff(false);
    }
  }
  return true;
}

/****************************************************************************
 * plugin_write: Handle commands
 ***************************************************************************/
bool P095_data_struct::plugin_write(struct EventStruct *event, const String& string) {
  bool   success = false;
  String cmd     = parseString(string, 1);

  if ((nullptr != tft) && cmd.equals(_commandTriggerCmd)) {
    String arg1 = parseString(string, 2);
    success = true;

    if (arg1.equals(F("off"))) {
      displayOnOff(false);
    }
    else if (arg1.equals(F("on"))) {
      displayOnOff(true);
    }
    else if (arg1.equals(F("clear")))
    {
      String arg2 = parseString(string, 3);

      if (!arg2.isEmpty()) {
        tft->fillScreen(AdaGFXparseColor(arg2));
      } else {
        tft->fillScreen(_bgcolor);
      }
    }
    else if (arg1.equals(F("backlight"))) {
      String arg2 = parseString(string, 3);
      int    nArg2;

      if ((P095_CONFIG_BACKLIGHT_PIN != -1) && // All is valid?
          validIntFromString(arg2, nArg2) &&
          (nArg2 > 0) &&
          (nArg2 <= 100)) {
        P095_CONFIG_BACKLIGHT_PERCENT = nArg2; // Set but don't store
        displayOnOff(true);
      } else {
        success = false;
      }
    }
    else if (arg1.equals(F("inv")))
    {
      String arg2 = parseString(string, 3);
      int    nArg2;

      if (validIntFromString(arg2, nArg2) &&
          (nArg2 >= 0) &&
          (nArg2 <= 1)) {
        tft->invertDisplay(nArg2);
      } else {
        success = false;
      }
    }
    else if (arg1.equals(F("rot")))
    {
      ///control?cmd=tftcmd,rot,0
      // not working to verify
      String arg2 = parseString(string, 3);
      int    nArg2;

      if (validIntFromString(arg2, nArg2) &&
          (nArg2 >= 0)) {
        tft->setRotation(nArg2 % 4);
      } else {
        success = false;
      }
    } else {
      success = false;
    }
  }
  else if (tft && (cmd.equals(_commandTrigger) ||
                   (gfxHelper && gfxHelper->isAdaGFXTrigger(cmd)))) {
    success = true;

    if (!bitRead(P095_CONFIG_FLAGS, P095_CONFIG_FLAG_NO_WAKE)) { // Wake display?
      displayOnOff(true);
    }

    if (nullptr != gfxHelper) {
      String tmp = string;

      // Hand it over after replacing variables
      success = gfxHelper->processCommand(AdaGFXparseTemplate(tmp, _textcols, gfxHelper));

      updateFontMetrics(); // Font or color may have changed

      if (success) {
        int16_t curX, curY;
        gfxHelper->getCursorXY(curX, curY); // Get current X and Y coordinates, and put into Values
        UserVar[event->BaseVarIndex]     = curX;
        UserVar[event->BaseVarIndex + 1] = curY;
      }
    }
  }
  return success;
}

/****************************************************************************
 * displayOnOff: Turn display on or off
 ***************************************************************************/
void P095_data_struct::displayOnOff(bool state) {
  if (_backlightPin != -1) {
    # if defined(ESP8266)
    analogWrite(_backlightPin, state ? ((1024 / 100) * _backlightPercentage) : 0);
    # endif // if defined(ESP8266)
    # if defined(ESP32)
    analogWriteESP32(_backlightPin, state ? ((1024 / 100) * _backlightPercentage) : 0, 0);
    # endif // if defined(ESP32)
  }

  if (state) {
    tft->sendCommand(ILI9341_DISPON);
  } else {
    tft->sendCommand(ILI9341_DISPOFF);
  }
  _displayTimer = (state ? _displayTimeout : 0);
}

/****************************************************************************
 * registerButtonState: the button has been pressed, apply some debouncing
 ***************************************************************************/
void P095_data_struct::registerButtonState(uint8_t newButtonState, bool bPin3Invers) {
  if ((ButtonLastState == 0xFF) || (bPin3Invers != (!!newButtonState))) {
    ButtonLastState = newButtonState;
    DebounceCounter++;
  } else {
    ButtonLastState = 0xFF; // Reset
    DebounceCounter = 0;
    ButtonState     = false;
  }

  if ((ButtonLastState == newButtonState) &&
      (DebounceCounter >= P095_DebounceTreshold)) {
    ButtonState = true;
  }
}

/****************************************************************************
 * markButtonStateProcessed: reset the button state
 ***************************************************************************/
void P095_data_struct::markButtonStateProcessed() {
  ButtonState     = false;
  DebounceCounter = 0;
}

#endif // ifdef USES_P095

#include "../../_Plugin_Helper.h"
#ifdef USES_P128

# include "../PluginStructs/P128_data_struct.h"

// ***************************************************************/
// Constructor
// ***************************************************************/
P128_data_struct::P128_data_struct(int8_t   _gpioPin,
                                   uint16_t _pixelCount,
                                   uint8_t  _maxBright)
  : gpioPin(_gpioPin), pixelCount(_pixelCount), maxBright(_maxBright) {
  if (!Plugin_128_pixels) {
    # ifdef ESP8266
    Plugin_128_pixels = new (std::nothrow) NEOPIXEL_LIB<FEATURE, METHOD>(min(pixelCount, static_cast<uint16_t>(ARRAYSIZE)));
    # endif // ifdef ESP8266
    # ifdef ESP32
    Plugin_128_pixels = new (std::nothrow) NEOPIXEL_LIB<FEATURE, METHOD>(min(pixelCount, static_cast<uint16_t>(ARRAYSIZE)),
                                                                         _gpioPin);
    # endif // ifdef ESP32

    if (nullptr != Plugin_128_pixels) {
      Plugin_128_pixels->Begin(); // This initializes the NeoPixelBus library.
      Plugin_128_pixels->SetBrightness(maxBright);
    }
  }
}

// ***************************************************************/
// Destructor
// ***************************************************************/
P128_data_struct::~P128_data_struct() {
  delete Plugin_128_pixels;
  Plugin_128_pixels = nullptr;
}

bool P128_data_struct::plugin_read(struct EventStruct *event) {
  // there is no need to read them, just use current values
  UserVar[event->BaseVarIndex]     = static_cast<int>(mode);
  UserVar[event->BaseVarIndex + 1] = static_cast<int>(savemode);
  UserVar[event->BaseVarIndex + 2] = fadetime;
  UserVar[event->BaseVarIndex + 3] = fadedelay;

  # ifndef LIMIT_BUILD_SIZE

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log;
    log.reserve(64);
    log  = F("Lights: mode: ");
    log += P128_modeType_toString(mode);
    log += F(" lastmode: ");
    log += P128_modeType_toString(savemode);
    log += F(" fadetime: ");
    log += (int)UserVar[event->BaseVarIndex + 2];
    log += F(" fadedelay: ");
    log += (int)UserVar[event->BaseVarIndex + 3];
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // ifndef LIMIT_BUILD_SIZE
  return true;
}

bool P128_data_struct::plugin_write(struct EventStruct *event,
                                    const String      & string) {
  bool success = false;

  const String command = parseString(string, 1);

  if ((command == F("neopixelfx")) || (command == F("nfx"))) {
    const String subCommand = parseString(string, 2);

    const String  str3  = parseString(string, 3);
    const int32_t str3i = event->Par2;
    const String  str4  = parseString(string, 4);
    const int32_t str4i = event->Par3;
    const String  str5  = parseString(string, 5);
    const int32_t str5i = event->Par4;
    const String  str6  = parseString(string, 6);
    const int32_t str6i = event->Par5;
    const String  str7  = parseString(string, 7);
    const int32_t str7i = str7.toInt();

    if (subCommand == F("fadetime")) {
      success  = true;
      fadetime = str3i;
    }

    else if (subCommand == F("fadedelay")) {
      success   = true;
      fadedelay = str3i;
    }

    else if (subCommand == F("speed")) {
      success      = true;
      defaultspeed = str3i;
      speed        = defaultspeed;
    }

    else if (subCommand == F("bgcolor")) {
      success = true;
      hex2rrggbb(str3);
    }

    else if (subCommand == F("count")) {
      success = true;
      count   = str3i;
    }

    else if ((subCommand == F("on")) || (subCommand == F("off"))) {
      success   = true;
      fadetime  = 1000;
      fadedelay = 0;

      fadetime = str3.isEmpty()
          ? fadetime
          : str3i;
      fadedelay = str4.isEmpty()
          ? fadedelay
          : str4i;

      for (int pixel = 0; pixel < pixelCount; pixel++) {
        r_pixel = (fadedelay < 0)
            ? pixelCount - pixel - 1
            : pixel;

        starttime[r_pixel] = counter20ms + (pixel * abs(fadedelay) / 20);

        if ((subCommand == F("on")) && (mode == P128_modetype::Off)) { // switch on
          rgb_target[pixel] = rgb_old[pixel];
          rgb_old[pixel]    = Plugin_128_pixels->GetPixelColor(pixel);
        } else if (subCommand == F("off")) {                           // switch off
          rgb_old[pixel]    = Plugin_128_pixels->GetPixelColor(pixel);
          rgb_target[pixel] = RgbColor(0);
        }
      }

      if ((subCommand == F("on")) && (mode == P128_modetype::Off)) { // switch on
        mode = (savemode == P128_modetype::On) ? P128_modetype::Fade : savemode;
      } else if (subCommand == F("off")) {                           // switch off
        savemode = mode;
        mode     = P128_modetype::Fade;
      }

      maxtime = starttime[r_pixel] + (fadetime / 20);
    }

    else if (subCommand == F("dim")) {
      if ((str3i >= 0) && (str3i <= maxBright)) { // Safety check
        success = true;
        Plugin_128_pixels->SetBrightness(str3i);
      }
    }

    else if (subCommand == F("line")) {
      success = true;
      mode    = P128_modetype::On;

      hex2rgb(str5);

      for (int i = 0; i <= (str4i - str3i + pixelCount) % pixelCount; i++) {
        Plugin_128_pixels->SetPixelColor((i + str3i - 1) % pixelCount, rgb);
      }
    }

    else if (subCommand == F("tick")) {
      success = true;
      mode    = P128_modetype::On;

      hex2rgb(str4);

      //          for (int i = 0; i < pixelCount ; i = i + (pixelCount / parseString(string, 3).toInt())) {
      for (int i = 0; i < str3i; i++) {
        Plugin_128_pixels->SetPixelColor(i * pixelCount / str3i, rgb);
      }
    }

    else if (subCommand == F("one")) {
      success = true;
      mode    = P128_modetype::On;

      const uint16_t pixnum = str3i - 1;
      hex2rgb(str4);

      Plugin_128_pixels->SetPixelColor(pixnum, rgb);
    }

    else if ((subCommand == F("fade")) || (subCommand == F("all")) || (subCommand == F("rgb"))) {
      success = true;
      mode    = P128_modetype::Fade;

      if ((subCommand == F("all")) || (subCommand == F("rgb"))) {
        fadedelay = 0;
      }

      hex2rgb(str3);
      hex2rgb_pixel(str3);

      fadetime = str4.isEmpty()
          ? fadetime
          : str4i;
      fadedelay = str5.isEmpty()
          ? fadedelay
          : str5i;

      for (int pixel = 0; pixel < pixelCount; pixel++) {
        r_pixel = (fadedelay < 0)
            ? pixelCount - pixel - 1
            : pixel;

        starttime[r_pixel] = counter20ms + (pixel * abs(fadedelay) / 20);

        rgb_old[pixel] = Plugin_128_pixels->GetPixelColor(pixel);
      }
      maxtime = starttime[r_pixel] + (fadetime / 20);
    }

    else if (subCommand == F("hsv")) {
      success   = true;
      mode      = P128_modetype::Fade;
      fadedelay = 0;
      rgb       =
        RgbColor(HsbColor(str3.toFloat() / 360.0f, str4.toFloat() / 100.0f,
                          str5.toFloat() / 100.0f));

      rgb2colorStr();

      hex2rgb_pixel(colorStr);

      fadetime = str6.isEmpty()
          ? fadetime
          : str6i;
      fadedelay = str7.isEmpty()
          ? fadedelay
          : str7i;

      for (int pixel = 0; pixel < pixelCount; pixel++) {
        r_pixel = (fadedelay < 0)
            ? pixelCount - pixel - 1
            : pixel;

        starttime[r_pixel] = counter20ms + (pixel * abs(fadedelay) / 20);

        rgb_old[pixel] = Plugin_128_pixels->GetPixelColor(pixel);
      }
      maxtime = starttime[r_pixel] + (fadetime / 20);
    }

    else if (subCommand == F("hsvone")) {
      success = true;
      mode    = P128_modetype::On;
      rgb     =
        RgbColor(HsbColor(str4.toFloat() / 360.0f, str5.toFloat() / 100.0f,
                          str6.toFloat() / 100.0f));

      rgb2colorStr();

      hex2rgb(colorStr);
      const uint16_t pixnum = str3i - 1;
      Plugin_128_pixels->SetPixelColor(pixnum, rgb);
    }

    else if (subCommand == F("hsvline")) {
      success = true;
      mode    = P128_modetype::On;

      rgb =
        RgbColor(HsbColor(str5.toFloat() / 360.0f, str6.toFloat() / 100.0f,
                          str7.toFloat() / 100.0f));

      rgb2colorStr();

      hex2rgb(colorStr);

      for (int i = 0; i <= (str4i - str3i + pixelCount) % pixelCount; i++) {
        Plugin_128_pixels->SetPixelColor((i + str3i - 1) % pixelCount, rgb);
      }
    }

    else if (subCommand == F("rainbow")) {
      success     = true;
      fadeIn      = (mode == P128_modetype::Off) ? true : false;
      mode        = P128_modetype::Rainbow;
      starttimerb = counter20ms;

      rainbowspeed = str3.isEmpty()
          ? speed
          : str3i;

      fadetime = str4.isEmpty()
          ? fadetime
          : str4i;
    }

    else if (subCommand == F("colorfade")) {
      success = true;
      mode    = P128_modetype::ColorFade;

      hex2rgb(str3);

      if (!str4.isEmpty()) { hex2rrggbb(str4); }

      startpixel = str5.isEmpty()
          ? 0
          : str5i - 1;
      endpixel = str6.isEmpty()
          ? pixelCount - 1
          : str6i - 1;
    }

    else if (subCommand == F("kitt")) {
      success = true;
      mode    = P128_modetype::Kitt;

      _counter_mode_step = 0;

      hex2rgb(str3);

      speed = str4.isEmpty()
          ? defaultspeed
          : str4i;
    }

    else if (subCommand == F("comet")) {
      success = true;
      mode    = P128_modetype::Comet;

      _counter_mode_step = 0;

      hex2rgb(str3);

      speed = str4.isEmpty()
          ? defaultspeed
          : str4i;
    }

    else if (subCommand == F("theatre")) {
      success = true;
      mode    = P128_modetype::Theatre;

      hex2rgb(str3);

      if (!str4.isEmpty()) { hex2rrggbb(str4); }

      count = str5.isEmpty()
          ? count
          : str5i;

      speed = str6.isEmpty()
          ? defaultspeed
          : str6i;

      for (int i = 0; i < pixelCount; i++) {
        if ((i / count) % 2 == 0) {
          Plugin_128_pixels->SetPixelColor(i, rgb);
        } else {
          Plugin_128_pixels->SetPixelColor(i, rrggbb);
        }
      }
    }

    else if (subCommand == F("scan")) {
      success = true;
      mode    = P128_modetype::Scan;

      _counter_mode_step = 0;

      hex2rgb(str3);

      if (!str4.isEmpty()) { hex2rrggbb(str4); }

      speed = str5.isEmpty()
          ? defaultspeed
          : str5i;
    }

    else if (subCommand == F("dualscan")) {
      success = true;
      mode    = P128_modetype::Dualscan;

      _counter_mode_step = 0;

      hex2rgb(str3);

      if (!str4.isEmpty()) { hex2rrggbb(str4); }

      speed = str5.isEmpty()
          ? defaultspeed
          : str5i;
    }

    else if (subCommand == F("twinkle")) {
      success = true;
      mode    = P128_modetype::Twinkle;

      _counter_mode_step = 0;

      hex2rgb(str3);

      if (!str4.isEmpty()) { hex2rrggbb(str4); }

      speed = str5.isEmpty()
          ? defaultspeed
          : str5i;
    }

    else if (subCommand == F("twinklefade")) {
      success = true;
      mode    = P128_modetype::TwinkleFade;

      hex2rgb(str3);

      count = str4.isEmpty()
          ? count
          : str4i;

      speed = str5.isEmpty()
          ? defaultspeed
          : str5i;
    }

    else if (subCommand == F("sparkle")) {
      success = true;
      mode    = P128_modetype::Sparkle;

      _counter_mode_step = 0;

      hex2rgb(str3);
      hex2rrggbb(str4);

      speed = str5.isEmpty()
          ? defaultspeed
          : str5i;
    }

    else if (subCommand == F("wipe")) {
      success = true;
      mode    = P128_modetype::Wipe;

      _counter_mode_step = 0;

      hex2rgb(str3);

      if (!str4.isEmpty()) {
        hex2rrggbb(str4);
      } else {
        hex2rrggbb("000000");
      }

      speed = str5.isEmpty()
          ? defaultspeed
          : str5i;
    }

    else if (subCommand == F("dualwipe")) {
      success = true;
      mode    = P128_modetype::Dualwipe;

      _counter_mode_step = 0;

      hex2rgb(str3);

      if (!str4.isEmpty()) {
        hex2rrggbb(str4);
      } else {
        hex2rrggbb("000000");
      }

      speed = str5.isEmpty()
          ? defaultspeed
          : str5i;
    }

    # if P128_ENABLE_FAKETV
    else if (subCommand == F("faketv")) {
      success            = true;
      mode               = P128_modetype::FakeTV;
      _counter_mode_step = 0;

      randomSeed(analogRead(A0));
      pixelNum = random(NUMPixels); // Begin at random point

      startpixel = str3.isEmpty()
          ? 0
          : str3i - 1;
      endpixel = str4.isEmpty()
          ? pixelCount
          : str4i;
    }
    # endif // if P128_ENABLE_FAKETV

    else if (subCommand == F("fire")) {
      success = true;
      mode    = P128_modetype::Fire;

      fps = str3.isEmpty()
          ? fps
          : str3i;

      fps = (fps == 0 || fps > 50) ? 50 : fps;

      brightness = str4.isEmpty()
          ? brightness
          : str4.toFloat();
      cooling = str5.isEmpty()
          ? cooling
          : str5.toFloat();
      sparking = str6.isEmpty()
          ? sparking
          : str6.toFloat();
    }

    else if (subCommand == F("fireflicker")) {
      success = true;
      mode    = P128_modetype::FireFlicker;

      rev_intensity = str3.isEmpty()
          ? rev_intensity
          : str3i;

      speed = str4.isEmpty()
          ? defaultspeed
          : str4i;
    }

    else if (subCommand == F("simpleclock")) {
      success = true;
      mode    = P128_modetype::SimpleClock;

      # if defined(RGBW) || defined(GRBW)

      if (!str3.isEmpty()) {
        const uint32_t hcolorui = rgbStr2Num(str3);

        if (str3.length() <= 6) {
          rgb_tick_s = RgbwColor(hcolorui >> 16, hcolorui >> 8,
                                 hcolorui);
        } else {
          rgb_tick_s = RgbwColor(hcolorui >> 24,
                                 hcolorui >> 16,
                                 hcolorui >> 8,
                                 hcolorui);
        }
      }

      if (!str4.isEmpty()) {
        const uint32_t hcolorui = rgbStr2Num(str4);

        if (str4.length() <= 6) {
          rgb_tick_b = RgbwColor(hcolorui >> 16, hcolorui >> 8,
                                 hcolorui);
        } else {
          rgb_tick_b = RgbwColor(hcolorui >> 24,
                                 hcolorui >> 16,
                                 hcolorui >> 8,
                                 hcolorui);
        }
      }

      if (!str5.isEmpty()) {
        const uint32_t hcolorui = rgbStr2Num(str5);

        if (str5.length() <= 6) {
          rgb_h = RgbwColor(hcolorui >> 16, hcolorui >> 8,
                            hcolorui);
        } else {
          rgb_h = RgbwColor(hcolorui >> 24,
                            hcolorui >> 16,
                            hcolorui >> 8,
                            hcolorui);
        }
      }

      if (!str6.isEmpty()) {
        const uint32_t hcolorui = rgbStr2Num(str6);

        if (str6.length() <= 6) {
          rgb_m = RgbwColor(hcolorui >> 16, hcolorui >> 8,
                            hcolorui);
        } else {
          rgb_m = RgbwColor(hcolorui >> 24,
                            hcolorui >> 16,
                            hcolorui >> 8,
                            hcolorui);
        }
      }

      if (!str7.isEmpty()) {
        if (str7 == F("off")) {
          rgb_s_off = true;
        } else if (str7.length() <= 6) {
          const uint32_t hcolorui = rgbStr2Num(str7);
          rgb_s_off = false;
          rgb_s     = RgbwColor(hcolorui >> 16, hcolorui >> 8,
                                hcolorui);
        } else {
          const uint32_t hcolorui = rgbStr2Num(str7);
          rgb_s_off = false;
          rgb_s     = RgbwColor(hcolorui >> 24,
                                hcolorui >> 16,
                                hcolorui >> 8,
                                hcolorui);
        }
      }

      # else // if defined(RGBW) || defined(GRBW)

      if (!str3.isEmpty()) {
        const uint32_t hcolorui = rgbStr2Num(str3);
        rgb_tick_s = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui);
      }

      if (!str4.isEmpty()) {
        const uint32_t hcolorui = rgbStr2Num(str4);
        rgb_tick_b = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui);
      }

      if (!str5.isEmpty()) {
        const uint32_t hcolorui = rgbStr2Num(str5);
        rgb_h = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui);
      }

      if (!str6.isEmpty()) {
        const uint32_t hcolorui = rgbStr2Num(str6);
        rgb_m = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui);
      }

      if (!str7.isEmpty()) {
        if (str7 == F("off")) {
          rgb_s_off = true;
        } else {
          const uint32_t hcolorui = rgbStr2Num(str7);
          rgb_s_off = false;
          rgb_s     = RgbColor(hcolorui >> 16, hcolorui >> 8,
                               hcolorui);
        }
      }

      # endif // if defined(RGBW) || defined(GRBW)

      if (!parseString(string, 8).isEmpty()) {
        hex2rrggbb(parseString(string, 8));
      }
    }

    else if (subCommand == F("stop")) {
      success = true;
      mode    = P128_modetype::On;
    }

    else if (subCommand == F("statusrequest")) {
      success = true;
    }

    if (!success) {
      success = true; // Fake the command to be successful, to get this custom error message out
      String log = F("NeoPixelBus: unknown subcommand: ");
      log += subCommand;
      addLogMove(LOG_LEVEL_INFO, log);

      String json;
      printToWebJSON = true;

      json += '{'; json += '\n';
      json += to_json_object_value(F("plugin"), F("128"));
      json += ','; json += '\n';
      String subjson = F("NeoPixelBus: unknown command: ");
      subjson.reserve(subCommand.length() + 30);
      subjson += subCommand;
      json    += to_json_object_value(F("log"), subjson);
      json    += '\n'; json += '}'; json += '\n';

      // event->Source=EventValueSource::Enum::VALUE_SOURCE_HTTP;
      SendStatus(event, json); // send http response to controller (JSON format)
      printToWeb = false;
    }
    NeoPixelSendStatus(event);

    if (speed == 0) {
      mode = P128_modetype::On; // speed = 0 = stop mode
    }

    // avoid invalid values
    if ((speed > SPEED_MAX) || (speed < -SPEED_MAX)) {
      speed = defaultspeed;
    }

    if (fadetime <= 0) {
      fadetime = 20;
    }
  } // command neopixel

  return success;
}

void P128_data_struct::rgb2colorStr() {
  colorStr.clear();
  colorStr += formatToHex_no_prefix(rgb.R, 2);
  colorStr += formatToHex_no_prefix(rgb.G, 2);
  colorStr += formatToHex_no_prefix(rgb.B, 2);
}

bool P128_data_struct::plugin_fifty_per_second(struct EventStruct *event) {
  counter20ms++;
  lastmode = mode;

  switch (mode) {
    case P128_modetype::Fade:
      fade();
      break;

    case P128_modetype::ColorFade:
      colorfade();
      break;

    case P128_modetype::Rainbow:
      rainbow();
      break;

    case P128_modetype::Kitt:
      kitt();
      break;

    case P128_modetype::Comet:
      comet();
      break;

    case P128_modetype::Theatre:
      theatre();
      break;

    case P128_modetype::Scan:
      scan();
      break;

    case P128_modetype::Dualscan:
      dualscan();
      break;

    case P128_modetype::Twinkle:
      twinkle();
      break;

    case P128_modetype::TwinkleFade:
      twinklefade();
      break;

    case P128_modetype::Sparkle:
      sparkle();
      break;

    case P128_modetype::Fire:
      fire();
      break;

    case P128_modetype::FireFlicker:
      fire_flicker();
      break;

    case P128_modetype::Wipe:
      wipe();
      break;

    case P128_modetype::Dualwipe:
      dualwipe();
      break;

    # if P128_ENABLE_FAKETV
    case P128_modetype::FakeTV:
      faketv();
      break;
    # endif // if P128_ENABLE_FAKETV

    case P128_modetype::SimpleClock:
      Plugin_128_simpleclock();
      break;

    default:
      break;
  } // switch mode

  Plugin_128_pixels->Show();

  if (mode != lastmode) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("NeoPixelBus: Mode Change: ");
      log += P128_modeType_toString(mode);
      addLogMove(LOG_LEVEL_INFO, log);
    }
    NeoPixelSendStatus(event);
  }
  return true;
}

void P128_data_struct::fade(void) {
  for (int pixel = 0; pixel < pixelCount; pixel++) {
    long  counter  = 20 * (counter20ms - starttime[pixel]);
    float progress = (float)counter / (float)fadetime;
    progress = (progress < 0) ? 0 : progress;
    progress = (progress > 1) ? 1 : progress;

    # if defined(RGBW) || defined(GRBW)
    RgbwColor updatedColor = RgbwColor::LinearBlend(
      rgb_old[pixel], rgb_target[pixel],
      progress);
    # else // if defined(RGBW) || defined(GRBW)
    RgbColor updatedColor = RgbColor::LinearBlend(
      rgb_old[pixel], rgb_target[pixel],
      progress);
    # endif // if defined(RGBW) || defined(GRBW)

    if ((counter20ms > maxtime) && (Plugin_128_pixels->GetPixelColor(pixel).CalculateBrightness() == 0)) {
      mode = P128_modetype::Off;
    } else if (counter20ms > maxtime) {
      mode = P128_modetype::On;
    }

    Plugin_128_pixels->SetPixelColor(pixel, updatedColor);
  }
}

void P128_data_struct::colorfade(void) {
  float progress = 0;

  difference = (endpixel - startpixel + pixelCount) % pixelCount;

  for (uint16_t i = 0; i <= difference; i++)
  {
    progress = (float)i / (difference - 1);
    progress = (progress >= 1) ? 1 : progress;
    progress = (progress <= 0) ? 0 : progress;

    # if defined(RGBW) || defined(GRBW)
    RgbwColor updatedColor = RgbwColor::LinearBlend(
      rgb, rrggbb,
      progress);
    # else // if defined(RGBW) || defined(GRBW)
    RgbColor updatedColor = RgbColor::LinearBlend(
      rgb, rrggbb,
      progress);
    # endif // if defined(RGBW) || defined(GRBW)

    Plugin_128_pixels->SetPixelColor((i + startpixel) % pixelCount, updatedColor);
  }
  mode = P128_modetype::On;
}

void P128_data_struct::wipe(void) {
  if (counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) {
    if (speed > 0) {
      Plugin_128_pixels->SetPixelColor(_counter_mode_step, rrggbb);

      if (_counter_mode_step > 0) { Plugin_128_pixels->SetPixelColor(_counter_mode_step - 1, rgb); }
    } else {
      Plugin_128_pixels->SetPixelColor(pixelCount - _counter_mode_step - 1, rrggbb);

      if (_counter_mode_step > 0) { Plugin_128_pixels->SetPixelColor(pixelCount - _counter_mode_step, rgb); }
    }

    if (_counter_mode_step == pixelCount) { mode = P128_modetype::On; }
    _counter_mode_step++;
  }
}

void P128_data_struct::dualwipe(void) {
  if (counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) {
    if (speed > 0) {
      int i = _counter_mode_step - pixelCount;
      i = abs(i);
      Plugin_128_pixels->SetPixelColor(_counter_mode_step, rrggbb);
      Plugin_128_pixels->SetPixelColor(i,                  rgb);

      if (_counter_mode_step > 0) {
        Plugin_128_pixels->SetPixelColor(_counter_mode_step - 1, rgb);
        Plugin_128_pixels->SetPixelColor(i - 1,                  rrggbb);
      }
    } else {
      int i = (pixelCount / 2) - _counter_mode_step;
      i = abs(i);
      Plugin_128_pixels->SetPixelColor(_counter_mode_step + (pixelCount / 2), rrggbb);
      Plugin_128_pixels->SetPixelColor(i,                                     rgb);

      if (_counter_mode_step > 0) {
        Plugin_128_pixels->SetPixelColor(_counter_mode_step + (pixelCount / 2) - 1, rgb);
        Plugin_128_pixels->SetPixelColor(i - 1,                                     rrggbb);
      }
    }

    if (_counter_mode_step >= pixelCount / 2) {
      mode = P128_modetype::On;
      Plugin_128_pixels->SetPixelColor(_counter_mode_step - 1, rgb);
    }
    _counter_mode_step++;
  }
}

# if P128_ENABLE_FAKETV
void P128_data_struct::faketv(void) {
  if (counter20ms >= ftv_holdTime) {
    difference = abs(endpixel - startpixel);

    if (ftv_elapsed >= ftv_fadeTime) {
      // Read next 16-bit (5/6/5) color
      ftv_hi = pgm_read_byte(&ftv_colors[pixelNum * 2]);
      ftv_lo = pgm_read_byte(&ftv_colors[pixelNum * 2 + 1]);

      if (++pixelNum >= NUMPixels) { pixelNum = 0; }

      // Expand to 24-bit (8/8/8)
      ftv_r8 = (ftv_hi & 0xF8) | (ftv_hi >> 5);
      ftv_g8 = (ftv_hi << 5) | ((ftv_lo & 0xE0) >> 3) | ((ftv_hi & 0x06) >> 1);
      ftv_b8 = (ftv_lo << 3) | ((ftv_lo & 0x1F) >> 2);

      // Apply gamma correction, further expand to 16/16/16
      ftv_nr = (uint8_t)pgm_read_byte(&ftv_gamma8[ftv_r8]) * 257; // New R/G/B
      ftv_ng = (uint8_t)pgm_read_byte(&ftv_gamma8[ftv_g8]) * 257;
      ftv_nb = (uint8_t)pgm_read_byte(&ftv_gamma8[ftv_b8]) * 257;

      ftv_totalTime = random(12, 125);                            // Semi-random pixel-to-pixel time
      ftv_fadeTime  = random(0, ftv_totalTime);                   // Pixel-to-pixel transition time

      if (random(10) < 3) { ftv_fadeTime = 0; }                   // Force scene cut 30% of time
      ftv_holdTime  = counter20ms + ftv_totalTime - ftv_fadeTime; // Non-transition time
      ftv_startTime = counter20ms;
    }

    ftv_elapsed = counter20ms - ftv_startTime;

    if (ftv_fadeTime) {
      ftv_r = map(ftv_elapsed, 0, ftv_fadeTime, ftv_pr, ftv_nr); // 16-bit interp
      ftv_g = map(ftv_elapsed, 0, ftv_fadeTime, ftv_pg, ftv_ng);
      ftv_b = map(ftv_elapsed, 0, ftv_fadeTime, ftv_pb, ftv_nb);
    } else {                                                     // Avoid divide-by-ftv_fraczero in map()
      ftv_r = ftv_nr;
      ftv_g = ftv_ng;
      ftv_b = ftv_nb;
    }

    for (ftv_i = 0; ftv_i < difference; ftv_i++) {
      ftv_r8   = ftv_r >> 8;                 // Quantize to 8-bit
      ftv_g8   = ftv_g >> 8;
      ftv_b8   = ftv_b >> 8;
      ftv_frac = (ftv_i << 16) / difference; // LED index scaled to 0-65535 (16Bit)

      if ((ftv_r8 < 255) && ((ftv_r & 0xFF) >= ftv_frac)) { ftv_r8++; } // Boost some fraction

      if ((ftv_g8 < 255) && ((ftv_g & 0xFF) >= ftv_frac)) { ftv_g8++; } // of LEDs to handle

      if ((ftv_b8 < 255) && ((ftv_b & 0xFF) >= ftv_frac)) { ftv_b8++; } // interp > 8bit

      Plugin_128_pixels->SetPixelColor(ftv_i + startpixel, RgbColor(ftv_r8, ftv_g8, ftv_b8));
    }

    ftv_pr = ftv_nr; // Prev RGB = new RGB
    ftv_pg = ftv_ng;
    ftv_pb = ftv_nb;
  }
}

# endif // if P128_ENABLE_FAKETV

/*
 * Cycles a rainbow over the entire string of LEDs.
 */
void P128_data_struct::rainbow(void) {
  long  counter  = 20 * (counter20ms - starttimerb);
  float progress = (float)counter / (float)fadetime;

  if (fadeIn == true) {
    Plugin_128_pixels->SetBrightness(progress * maxBright); // Safety check
    fadeIn = (progress == 1) ? false : true;
  }

  for (int i = 0; i < pixelCount; i++) {
    uint8_t r1 = (Wheel(((i * 256 / pixelCount) + counter20ms * rainbowspeed / 10) & 255) >> 16);
    uint8_t g1 = (Wheel(((i * 256 / pixelCount) + counter20ms * rainbowspeed / 10) & 255) >> 8);
    uint8_t b1 = (Wheel(((i * 256 / pixelCount) + counter20ms * rainbowspeed / 10) & 255));
    Plugin_128_pixels->SetPixelColor(i, RgbColor(r1, g1, b1));
  }
  mode = (rainbowspeed == 0) ? P128_modetype::On : P128_modetype::Rainbow;
}

/*
 * Put a value 0 to 255 in to get a color value.
 * The colours are a transition r -> g -> b -> back to r
 * Inspired by the Adafruit examples.
 */
uint32_t P128_data_struct::Wheel(uint8_t pos) {
  pos = 255 - pos;

  if (pos < 85) {
    return ((uint32_t)(255 - pos * 3) << 16) | ((uint32_t)(0) << 8) | (pos * 3);
  } else if (pos < 170) {
    pos -= 85;
    return ((uint32_t)(0) << 16) | ((uint32_t)(pos * 3) << 8) | (255 - pos * 3);
  } else {
    pos -= 170;
    return ((uint32_t)(pos * 3) << 16) | ((uint32_t)(255 - pos * 3) << 8) | (0);
  }
}

// Larson Scanner K.I.T.T.
void P128_data_struct::kitt(void) {
  if (counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) {
    for (uint16_t i = 0; i < pixelCount; i++) {
      # if defined(RGBW) || defined(GRBW)
      RgbwColor px_rgb = Plugin_128_pixels->GetPixelColor(i);

      // fade out (divide by 2)
      px_rgb.R = px_rgb.R >> 1;
      px_rgb.G = px_rgb.G >> 1;
      px_rgb.B = px_rgb.B >> 1;
      px_rgb.W = px_rgb.W >> 1;

      # else // if defined(RGBW) || defined(GRBW)

      RgbColor px_rgb = Plugin_128_pixels->GetPixelColor(i);

      // fade out (divide by 2)
      px_rgb.R = px_rgb.R >> 1;
      px_rgb.G = px_rgb.G >> 1;
      px_rgb.B = px_rgb.B >> 1;
      # endif // if defined(RGBW) || defined(GRBW)

      Plugin_128_pixels->SetPixelColor(i, px_rgb);
    }

    uint16_t pos = 0;

    if (_counter_mode_step < pixelCount) {
      pos = _counter_mode_step;
    } else {
      pos = (pixelCount * 2) - _counter_mode_step - 2;
    }

    Plugin_128_pixels->SetPixelColor(pos, rgb);

    _counter_mode_step = (_counter_mode_step + 1) % ((pixelCount * 2) - 2);
  }
}

// Firing comets from one end.
void P128_data_struct::comet(void) {
  if (counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) {
    for (uint16_t i = 0; i < pixelCount; i++) {
      if (speed > 0) {
        # if defined(RGBW) || defined(GRBW)
        RgbwColor px_rgb = Plugin_128_pixels->GetPixelColor(i);

        // fade out (divide by 2)
        px_rgb.R = px_rgb.R >> 1;
        px_rgb.G = px_rgb.G >> 1;
        px_rgb.B = px_rgb.B >> 1;
        px_rgb.W = px_rgb.W >> 1;

        # else // if defined(RGBW) || defined(GRBW)

        RgbColor px_rgb = Plugin_128_pixels->GetPixelColor(i);

        // fade out (divide by 2)
        px_rgb.R = px_rgb.R >> 1;
        px_rgb.G = px_rgb.G >> 1;
        px_rgb.B = px_rgb.B >> 1;
        # endif // if defined(RGBW) || defined(GRBW)

        Plugin_128_pixels->SetPixelColor(i, px_rgb);
      } else {
        # if defined(RGBW) || defined(GRBW)
        RgbwColor px_rgb = Plugin_128_pixels->GetPixelColor(pixelCount - i - 1);

        // fade out (divide by 2)
        px_rgb.R = px_rgb.R >> 1;
        px_rgb.G = px_rgb.G >> 1;
        px_rgb.B = px_rgb.B >> 1;
        px_rgb.W = px_rgb.W >> 1;

        # else // if defined(RGBW) || defined(GRBW)

        RgbColor px_rgb = Plugin_128_pixels->GetPixelColor(pixelCount - i - 1);

        // fade out (divide by 2)
        px_rgb.R = px_rgb.R >> 1;
        px_rgb.G = px_rgb.G >> 1;
        px_rgb.B = px_rgb.B >> 1;
        # endif // if defined(RGBW) || defined(GRBW)

        Plugin_128_pixels->SetPixelColor(pixelCount - i - 1, px_rgb);
      }
    }

    if (speed > 0) {
      Plugin_128_pixels->SetPixelColor(_counter_mode_step, rgb);
    } else {
      Plugin_128_pixels->SetPixelColor(pixelCount - _counter_mode_step - 1, rgb);
    }

    _counter_mode_step = (_counter_mode_step + 1) % pixelCount;
  }
}

// Theatre lights
void P128_data_struct::theatre(void) {
  if ((counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) && (speed != 0)) {
    if (speed > 0) {
      Plugin_128_pixels->RotateLeft(1, 0, (pixelCount / count) * count - 1);
    } else {
      Plugin_128_pixels->RotateRight(1, 0, (pixelCount / count) * count - 1);
    }
  }
}

/*
 * Runs a single pixel back and forth.
 */
void P128_data_struct::scan(void) {
  if ((counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) && (speed != 0)) {
    if (_counter_mode_step > uint16_t((pixelCount * 2) - 2)) {
      _counter_mode_step = 0;
    }
    _counter_mode_step++;

    int i = _counter_mode_step - (pixelCount - 1);
    i = abs(i);

    // Plugin_128_pixels->ClearTo(rrggbb);
    for (int i = 0; i < pixelCount; i++) {
      Plugin_128_pixels->SetPixelColor(i, rrggbb);
    }
    Plugin_128_pixels->SetPixelColor(abs(i), rgb);
  }
}

/*
 * Runs two pixel back and forth in opposite directions.
 */
void P128_data_struct::dualscan(void) {
  if ((counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) && (speed != 0)) {
    if (_counter_mode_step > uint16_t((pixelCount * 2) - 2)) {
      _counter_mode_step = 0;
    }

    _counter_mode_step++;

    int i = _counter_mode_step - (pixelCount - 1);
    i = abs(i);

    // Plugin_128_pixels->ClearTo(rrggbb);
    for (int i = 0; i < pixelCount; i++) {
      Plugin_128_pixels->SetPixelColor(i, rrggbb);
    }
    Plugin_128_pixels->SetPixelColor(abs(i),               rgb);
    Plugin_128_pixels->SetPixelColor(pixelCount - (i + 1), rgb);
  }
}

/*
 * Blink several LEDs on, reset, repeat.
 * Inspired by www.tweaking4all.com/hardware/arduino/arduino-led-strip-effects/
 */
void P128_data_struct::twinkle(void) {
  if ((counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) && (speed != 0)) {
    if (_counter_mode_step == 0) {
      // Plugin_128_pixels->ClearTo(rrggbb);
      for (int i = 0; i < pixelCount; i++) {
        Plugin_128_pixels->SetPixelColor(i, rrggbb);
      }
      uint16_t min_leds = _max(1, pixelCount / 5); // make sure, at least one LED is on
      uint16_t max_leds = _max(1, pixelCount / 2); // make sure, at least one LED is on
      _counter_mode_step = random(min_leds, max_leds);
    }

    Plugin_128_pixels->SetPixelColor(random(pixelCount), rgb);

    _counter_mode_step--;
  }
}

/*
 * Blink several LEDs on, fading out.
 */
void P128_data_struct::twinklefade(void) {
  if ((counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) && (speed != 0)) {
    for (uint16_t i = 0; i < pixelCount; i++) {
      # if defined(RGBW) || defined(GRBW)
      RgbwColor px_rgb = Plugin_128_pixels->GetPixelColor(pixelCount - i - 1);

      // fade out (divide by 2)
      px_rgb.R = px_rgb.R >> 1;
      px_rgb.G = px_rgb.G >> 1;
      px_rgb.B = px_rgb.B >> 1;
      px_rgb.W = px_rgb.W >> 1;

      # else // if defined(RGBW) || defined(GRBW)

      RgbColor px_rgb = Plugin_128_pixels->GetPixelColor(pixelCount - i - 1);

      // fade out (divide by 2)
      px_rgb.R = px_rgb.R >> 1;
      px_rgb.G = px_rgb.G >> 1;
      px_rgb.B = px_rgb.B >> 1;
      # endif // if defined(RGBW) || defined(GRBW)

      Plugin_128_pixels->SetPixelColor(i, px_rgb);
    }

    if (random(count) < 50) {
      Plugin_128_pixels->SetPixelColor(random(pixelCount), rgb);
    }
  }
}

/*
 * Blinks one LED at a time.
 * Inspired by www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/
 */
void P128_data_struct::sparkle(void) {
  if ((counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) && (speed != 0)) {
    // Plugin_128_pixels->ClearTo(rrggbb);
    for (int i = 0; i < pixelCount; i++) {
      Plugin_128_pixels->SetPixelColor(i, rrggbb);
    }
    Plugin_128_pixels->SetPixelColor(random(pixelCount), rgb);
  }
}

void P128_data_struct::fire(void) {
  if (counter20ms > fireTimer + 50 / fps) {
    fireTimer = counter20ms;
    Fire2012();
    RgbColor pixel;

    for (int i = 0; i < pixelCount; i++) {
      pixel = leds[i];
      pixel = RgbColor::LinearBlend(pixel, RgbColor(0, 0, 0), (255 - brightness) / 255.0);
      Plugin_128_pixels->SetPixelColor(i, pixel);
    }
  }
}

/// Generate an 8-bit random number
uint8_t P128_data_struct::random8() {
  rand16seed = (rand16seed * ((uint16_t)(2053))) + ((uint16_t)(13849));

  // return the sum of the high and low bytes, for better
  //  mixing and non-sequential correlation
  return (uint8_t)(((uint8_t)(rand16seed & 0xFF)) +
                   ((uint8_t)(rand16seed >> 8)));
}

/// Generate an 8-bit random number between 0 and lim
/// @param lim the upper bound for the result
uint8_t P128_data_struct::random8(uint8_t lim) {
  uint8_t r = random8();

  r = (r * lim) >> 8;
  return r;
}

/// Generate an 8-bit random number in the given range
/// @param min the lower bound for the random number
/// @param lim the upper bound for the random number
uint8_t P128_data_struct::random8(uint8_t min, uint8_t lim) {
  uint8_t delta = lim - min;
  uint8_t r     = random8(delta) + min;

  return r;
}

/// subtract one byte from another, saturating at 0x00
/// @returns i - j with a floor of 0
uint8_t P128_data_struct::qsub8(uint8_t i, uint8_t j) {
  int t = i - j;

  if (t < 0) { t = 0; }
  return t;
}

/// add one byte to another, saturating at 0xFF
/// @param i - first byte to add
/// @param j - second byte to add
/// @returns the sum of i & j, capped at 0xFF
uint8_t P128_data_struct::qadd8(uint8_t i, uint8_t j) {
  unsigned int t = i + j;

  if (t > 255) { t = 255; }
  return t;
}

///  The "video" version of scale8 guarantees that the output will
///  be only be zero if one or both of the inputs are zero.  If both
///  inputs are non-zero, the output is guaranteed to be non-zero.
///  This makes for better 'video'/LED dimming, at the cost of
///  several additional cycles.
uint8_t P128_data_struct::scale8_video(uint8_t i, uint8_t scale) {
  uint8_t j = (((int)i * (int)scale) >> 8) + ((i && scale) ? 1 : 0);

  // uint8_t nonzeroscale = (scale != 0) ? 1 : 0;
  // uint8_t j = (i == 0) ? 0 : (((int)i * (int)(scale) ) >> 8) + nonzeroscale;
  return j;
}

void P128_data_struct::Fire2012(void) {
  // Step 1.  Cool down every cell a little
  for (int i = 0; i < pixelCount; i++) {
    heat[i] = qsub8(heat[i],  random8(0, ((cooling * 10) / pixelCount) + 2));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for (int k = pixelCount - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if (random8() < sparking) {
    int y = random8(7);
    heat[y] = qadd8(heat[y], random8(160, 255));
  }

  // Step 4.  Map from heat cells to LED colors
  for (int j = 0; j < pixelCount; j++) {
    RgbColor heatcolor;

    // Scale 'heat' down from 0-255 to 0-191,
    // which can then be easily divided into three
    // equal 'thirds' of 64 units each.
    uint8_t t192 = scale8_video(heat[j], 191);

    // calculate a value that ramps up from
    // zero to 255 in each 'third' of the scale.
    uint8_t heatramp = t192 & 0x3F; // 0..63
    heatramp <<= 2;                 // scale up to 0..252

    // now figure out which third of the spectrum we're in:
    if (t192 & 0x80) {
      // we're in the hottest third
      heatcolor.R = 255;      // full red
      heatcolor.G = 255;      // full green
      heatcolor.B = heatramp; // ramp up blue
    } else if (t192 & 0x40) {
      // we're in the middle third
      heatcolor.R = 255;      // full red
      heatcolor.G = heatramp; // ramp up green
      heatcolor.B = 0;        // no blue
    } else {
      // we're in the coolest third
      heatcolor.R = heatramp; // ramp up red
      heatcolor.G = 0;        // no green
      heatcolor.B = 0;        // no blue
    }

    int pixelnumber;

    if (gReverseDirection) {
      pixelnumber = (pixelCount - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = heatcolor;
  }
}

/*
 * Fire flicker function
 */
void P128_data_struct::fire_flicker() {
  if ((counter20ms % (unsigned long)(SPEED_MAX / abs(speed)) == 0) && (speed != 0)) {
    byte w   = 0;   // (SEGMENT.colors[0] >> 24) & 0xFF;
    byte r   = 255; // (SEGMENT.colors[0] >> 16) & 0xFF;
    byte g   = 96;  // (SEGMENT.colors[0] >>  8) & 0xFF;
    byte b   = 12;  // (SEGMENT.colors[0]        & 0xFF);
    byte lum = max(w, max(r, max(g, b))) / rev_intensity;

    for (uint16_t i = 0; i <= NUMPixels - 1; i++) {
      int flicker = random8(lum);

      # if defined(RGBW) || defined(GRBW)
      Plugin_128_pixels->SetPixelColor(i, RgbwColor(max(r - flicker, 0), max(g - flicker, 0), max(b - flicker, 0), max(w - flicker, 0)));
      # else // if defined(RGBW) || defined(GRBW)
      Plugin_128_pixels->SetPixelColor(i, RgbColor(max(r - flicker, 0), max(g - flicker, 0), max(b - flicker, 0)));
      # endif  // if defined(RGBW) || defined(GRBW)
    }
  }
}

void P128_data_struct::Plugin_128_simpleclock() {
  byte Hours      = node_time.hour() % 12;
  byte Minutes    = node_time.minute();
  byte Seconds    = node_time.second();
  byte big_tick   = 15;
  byte small_tick = 5;

  // hack for sub-second calculations.... reset when first time new second begins..
  if (cooling != Seconds) { maxtime = counter20ms; }
  cooling = Seconds;
  Plugin_128_pixels->ClearTo(rrggbb);

  for (int i = 0; i < (60 / small_tick); i++) {
    if (i % (big_tick / small_tick) == 0) {
      Plugin_128_pixels->SetPixelColor((i * pixelCount * small_tick / 60) % pixelCount, rgb_tick_b);
    } else {
      Plugin_128_pixels->SetPixelColor((i * pixelCount * small_tick / 60) % pixelCount, rgb_tick_s);
    }
  }


  for (int i = 0; i < pixelCount; i++) {
    if (round((((float)Seconds + ((float)counter20ms - (float)maxtime) / 50.0) * (float)pixelCount) / 60.0) == i) {
      if (rgb_s_off  == false) {
        Plugin_128_pixels->SetPixelColor(i, rgb_s);
      }
    }
    else if (round((((float)Minutes * 60.0) + (float)Seconds) / 60.0 * (float)pixelCount / 60.0) == i) {
      Plugin_128_pixels->SetPixelColor(i, rgb_m);
    }
    else if (round(((float)Hours + (float)Minutes / 60) * (float)pixelCount / 12.0)  == i) {
      Plugin_128_pixels->SetPixelColor(i,                                 rgb_h);
      Plugin_128_pixels->SetPixelColor((i + 1) % pixelCount,              rgb_h);
      Plugin_128_pixels->SetPixelColor((i - 1 + pixelCount) % pixelCount, rgb_h);
    }
  }
}

uint32_t P128_data_struct::rgbStr2Num(String rgbStr) {
  uint32_t rgbDec = static_cast<uint32_t>(strtoul(&rgbStr[0], NULL, 16));

  return rgbDec;
}

void P128_data_struct::hex2rgb(const String& hexcolor) {
  colorStr = hexcolor;
  const uint32_t hcolorui = rgbStr2Num(hexcolor);

  # if defined(RGBW) || defined(GRBW)
  hexcolor.length() <= 6
    ? rgb = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui)
    : rgb = RgbwColor(hcolorui >> 24, hcolorui >> 16, hcolorui >> 8, hcolorui);
  # else // if defined(RGBW) || defined(GRBW)
  rgb = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui);
  # endif // if defined(RGBW) || defined(GRBW)
}

void P128_data_struct::hex2rrggbb(const String& hexcolor) {
  backgroundcolorStr = hexcolor;
  const uint32_t hcolorui = rgbStr2Num(hexcolor);

  # if defined(RGBW) || defined(GRBW)
  hexcolor.length() <= 6
    ? rrggbb = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui)
    : rrggbb = RgbwColor(hcolorui >> 24, hcolorui >> 16, hcolorui >> 8, hcolorui);
  # else // if defined(RGBW) || defined(GRBW)
  rrggbb = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui);
  # endif // if defined(RGBW) || defined(GRBW)
}

void P128_data_struct::hex2rgb_pixel(const String& hexcolor) {
  colorStr = hexcolor;
  const uint32_t hcolorui = rgbStr2Num(hexcolor);

  for (int i = 0; i < pixelCount; i++) {
    # if defined(RGBW) || defined(GRBW)
    hexcolor.length() <= 6
      ? rgb_target[i] = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui)
      : rgb_target[i] = RgbwColor(hcolorui >> 24, hcolorui >> 16, hcolorui >> 8, hcolorui);
    # else // if defined(RGBW) || defined(GRBW)
    rgb_target[i] = RgbColor(hcolorui >> 16, hcolorui >> 8, hcolorui);
    # endif // if defined(RGBW) || defined(GRBW)
  }
}

// ---------------------------------------------------------------------------------
// ------------------------------ JsonResponse -------------------------------------
// ---------------------------------------------------------------------------------
void P128_data_struct::NeoPixelSendStatus(struct EventStruct *eventSource) {
  String log = F("NeoPixelBusFX: Set ");

  log += rgb.R;
  log += '/';
  log += rgb.G;
  log += '/';
  log += rgb.B;

  addLogMove(LOG_LEVEL_INFO, log);

  String json;

  json.reserve(285); // Awfully long string :-|

  printToWebJSON = true;

  json += '{'; json += '\n';                                                               // 2
  json += to_json_object_value(F("plugin"), F("128"));                                     // 12
  json += ','; json += '\n';
  json += to_json_object_value(F("mode"), P128_modeType_toString(mode));                   // 14..23
  json += ','; json += '\n';
  json += to_json_object_value(F("lastmode"), P128_modeType_toString(savemode));           // 18..27
  json += ','; json += '\n';
  json += to_json_object_value(F("fadetime"), toString(fadetime, 0));                      // 15..19
  json += ','; json += '\n';
  json += to_json_object_value(F("fadedelay"), toString(fadedelay, 0));                    // 15..19
  json += ','; json += '\n';
  json += to_json_object_value(F("dim"), toString(Plugin_128_pixels->GetBrightness(), 0)); // 8..10
  json += ','; json += '\n';
  json += to_json_object_value(F("rgb"), colorStr, true);                                  // 15..17
  json += ','; json += '\n';

  HsbColor hsbColor = HsbColor(RgbColor(rgb.R, rgb.G, rgb.B));                             // Calculate only once

  json += to_json_object_value(F("hue"), toString(hsbColor.H * 360.0f, 0));                // 17
  json += ','; json += '\n';
  json += to_json_object_value(F("saturation"), toString(hsbColor.S * 100.0f, 0));         // 26?
  json += ','; json += '\n';
  json += to_json_object_value(F("brightness"), toString(hsbColor.B * 100.0f, 0));         // 26?
  json += ','; json += '\n';
  json += to_json_object_value(F("bgcolor"), backgroundcolorStr, true);                    // 21..23
  json += ','; json += '\n';
  json += to_json_object_value(F("count"), toString(count, 0));                            // 12..15
  json += ','; json += '\n';
  json += to_json_object_value(F("speed"), toString(speed, 0));                            // 12..14
  json += ','; json += '\n';
  json += to_json_object_value(F("pixelcount"), toString(pixelCount, 0));                  // 17..19
  json += '\n'; json += '}'; json += '\n';                                                 // 4

  SendStatus(eventSource, json);                                                           // send http response to controller (JSON format)
  printToWeb = false;
}

const __FlashStringHelper * P128_data_struct::P128_modeType_toString(P128_modetype modeType) {
  switch (modeType) {
    case P128_modetype::Off: return F("off");
    case P128_modetype::On: return F("on");
    case P128_modetype::Fade: return F("fade");
    case P128_modetype::ColorFade: return F("colorfade");
    case P128_modetype::Rainbow: return F("rainbow");
    case P128_modetype::Kitt: return F("kitt");
    case P128_modetype::Comet: return F("comet");
    case P128_modetype::Theatre: return F("theatre");
    case P128_modetype::Scan: return F("scan");
    case P128_modetype::Dualscan: return F("dualscan");
    case P128_modetype::Twinkle: return F("twinkle");
    case P128_modetype::TwinkleFade: return F("twinklefade");
    case P128_modetype::Sparkle: return F("sparkle");
    case P128_modetype::Fire: return F("fire");
    case P128_modetype::FireFlicker: return F("fireflicker");
    case P128_modetype::Wipe: return F("wipe");
    case P128_modetype::Dualwipe: return F("dualwipe");
    # if P128_ENABLE_FAKETV
    case P128_modetype::FakeTV: return F("faketv");
    # endif // if P128_ENABLE_FAKETV
    case P128_modetype::SimpleClock: return F("simpleclock");
  }
  return F("*unknown*");
}

#endif // ifdef USES_P128

#include "../PluginStructs/P087_data_struct.h"

#ifdef USES_P087


// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter
#include <ESPeasySerial.h>
#include <Regexp.h>


#include <vector>

P087_data_struct::P087_data_struct() :  easySerial(nullptr) {}

P087_data_struct::~P087_data_struct() {
  reset();
}

void P087_data_struct::reset() {
  if (easySerial != nullptr) {
    delete easySerial;
    easySerial = nullptr;
  }
}

bool P087_data_struct::init(ESPEasySerialPort port, const int16_t serial_rx, const int16_t serial_tx, unsigned long baudrate, uint8_t config) {
  if ((serial_rx < 0) && (serial_tx < 0)) {
    return false;
  }
  reset();
  easySerial = new (std::nothrow) ESPeasySerial(port, serial_rx, serial_tx);

  if (isInitialized()) {
    # if defined(ESP8266)
    easySerial->begin(baudrate, (SerialConfig)config);
    # elif defined(ESP32)
    easySerial->begin(baudrate, config);
    # endif // if defined(ESP8266)
    return true;
  }
  return false;
}

void P087_data_struct::post_init() {
  for (uint8_t i = 0; i < P87_MAX_CAPTURE_INDEX; ++i) {
    capture_index_used[i] = false;
  }
  regex_empty = _lines[P087_REGEX_POS].isEmpty();
  String log = F("P087_post_init:");

  for (uint8_t i = 0; i < P087_NR_FILTERS; ++i) {
    // Create some quick lookup table to see if we have a filter for the specific index
    capture_index_must_not_match[i] = _lines[i * 3 + P087_FIRST_FILTER_POS + 1].toInt() == P087_Filter_Comp::NotEqual;
    int index = _lines[i * 3 + P087_FIRST_FILTER_POS].toInt();

    // Index is negative when not used.
    if ((index >= 0) && (index < P87_MAX_CAPTURE_INDEX) && (_lines[i * 3 + P087_FIRST_FILTER_POS + 2].length() > 0)) {
      log                      += ' ';
      log                      += String(i);
      log                      += ':';
      log                      += String(index);
      capture_index[i]          = index;
      capture_index_used[index] = true;
    }
  }
  addLogMove(LOG_LEVEL_DEBUG, log);
}

bool P087_data_struct::isInitialized() const {
  return easySerial != nullptr;
}

void P087_data_struct::sendString(const String& data) {
  if (isInitialized()) {
    if (data.length() > 0) {
      setDisableFilterWindowTimer();
      easySerial->write(data.c_str());

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("Proxy: Sending: ");
        log += data;
        addLogMove(LOG_LEVEL_INFO, log);
      }
    }
  }
}

bool P087_data_struct::loop() {
  if (!isInitialized()) {
    return false;
  }
  bool fullSentenceReceived = false;

  if (easySerial != nullptr) {
    int available = easySerial->available();

    while (available > 0 && !fullSentenceReceived) {
      // Look for end marker
      char c = easySerial->read();
      --available;

      if (available == 0) {
        available = easySerial->available();
        delay(0);
      }

      switch (c) {
        case 13:
        {
          const size_t length = sentence_part.length();
          bool valid          = length > 0;

          for (size_t i = 0; i < length && valid; ++i) {
            if ((sentence_part[i] > 127) || (sentence_part[i] < 32)) {
              sentence_part = String();
              ++sentences_received_error;
              valid = false;
            }
          }

          if (valid) {
            fullSentenceReceived = true;
            last_sentence = sentence_part;
            sentence_part = String();
          }
          break;
        }
        case 10:

          // Ignore LF
          break;
        default:
          sentence_part += c;
          break;
      }

      if (max_length_reached()) { fullSentenceReceived = true; }
    }
  }

  if (fullSentenceReceived) {
    ++sentences_received;
    length_last_received = last_sentence.length();
  }
  return fullSentenceReceived;
}

bool P087_data_struct::getSentence(String& string) {
  string        = last_sentence;
  if (string.isEmpty()) {
    return false;
  }
  last_sentence = String();
  return true;
}

void P087_data_struct::getSentencesReceived(uint32_t& succes, uint32_t& error, uint32_t& length_last) const {
  succes      = sentences_received;
  error       = sentences_received_error;
  length_last = length_last_received;
}

void P087_data_struct::setMaxLength(uint16_t maxlenght) {
  max_length = maxlenght;
}

void P087_data_struct::setLine(uint8_t varNr, const String& line) {
  if (varNr < P87_Nlines) {
    _lines[varNr] = line;
  }
}

String P087_data_struct::getRegEx() const {
  return _lines[P087_REGEX_POS];
}

uint16_t P087_data_struct::getRegExpMatchLength() const {
  return _lines[P087_NR_CHAR_USE_POS].toInt();
}

uint32_t P087_data_struct::getFilterOffWindowTime() const {
  return _lines[P087_FILTER_OFF_WINDOW_POS].toInt();
}

P087_Match_Type P087_data_struct::getMatchType() const {
  return static_cast<P087_Match_Type>(_lines[P087_MATCH_TYPE_POS].toInt());
}

bool P087_data_struct::invertMatch() const {
  switch (getMatchType()) {
    case Regular_Match:          // fallthrough
    case Global_Match:
      break;
    case Regular_Match_inverted: // fallthrough
    case Global_Match_inverted:
      return true;
    case Filter_Disabled:
      break;
  }
  return false;
}

bool P087_data_struct::globalMatch() const {
  switch (getMatchType()) {
    case Regular_Match: // fallthrough
    case Regular_Match_inverted:
      break;
    case Global_Match:  // fallthrough
    case Global_Match_inverted:
      return true;
    case Filter_Disabled:
      break;
  }
  return false;
}

String P087_data_struct::getFilter(uint8_t lineNr, uint8_t& capture, P087_Filter_Comp& comparator) const
{
  uint8_t varNr = lineNr * 3 + P087_FIRST_FILTER_POS;

  if ((varNr + 3) > P87_Nlines) { return ""; }

  capture    = _lines[varNr++].toInt();
  comparator = _lines[varNr++] == "1" ? P087_Filter_Comp::NotEqual : P087_Filter_Comp::Equal;
  return _lines[varNr];
}

void P087_data_struct::setDisableFilterWindowTimer() {
  if (getFilterOffWindowTime() == 0) {
    disable_filter_window = 0;
  }
  else {
    disable_filter_window = millis() + getFilterOffWindowTime();
  }
}

bool P087_data_struct::disableFilterWindowActive() const {
  if (disable_filter_window != 0) {
    if (!timeOutReached(disable_filter_window)) {
      // We're still in the window where filtering is disabled
      return true;
    }
  }
  return false;
}

typedef std::pair<uint8_t, String> capture_tuple;
static std::vector<capture_tuple> capture_vector;


// called for each match
void P087_data_struct::match_callback(const char *match, const unsigned int length, const MatchState& ms)
{
  for (uint8_t i = 0; i < ms.level; i++)
  {
    capture_tuple tuple;
    tuple.first  = i;
    tuple.second = ms.GetCapture(i);
    capture_vector.push_back(tuple);
  } // end of for each capture
}

bool P087_data_struct::matchRegexp(String& received) const {
  size_t strlength = received.length();

  if (strlength == 0) {
    return false;
  }
  if (regex_empty || getMatchType() == Filter_Disabled) {
    return true;
  }


  uint32_t regexp_match_length = getRegExpMatchLength();

  if ((regexp_match_length > 0) && (strlength > regexp_match_length)) {
    strlength = regexp_match_length;
  }

  // We need to do a const_cast here, but this only is valid as long as we
  // don't call a replace function from regexp.
  MatchState ms(const_cast<char *>(received.c_str()), strlength);

  bool match_result = false;
  if (globalMatch()) {
    capture_vector.clear();
    ms.GlobalMatch(_lines[P087_REGEX_POS].c_str(), match_callback);
    const uint8_t vectorlength = capture_vector.size();

    for (uint8_t i = 0; i < vectorlength; ++i) {
      if ((capture_vector[i].first < P87_MAX_CAPTURE_INDEX) && capture_index_used[capture_vector[i].first]) {
        for (uint8_t n = 0; n < P087_NR_FILTERS; ++n) {
          unsigned int lines_index = n * 3 + P087_FIRST_FILTER_POS + 2;

          if ((capture_index[n] == capture_vector[i].first) && !(_lines[lines_index].isEmpty())) {
            String log;
            log.reserve(32);
            log  = F("P087: Index: ");
            log += capture_vector[i].first;
            log += F(" Found ");
            log += capture_vector[i].second;

            // Found a Capture Filter with this capture index.
            if (capture_vector[i].second == _lines[lines_index]) {
              log += F(" Matches");

              // Found a match. Now check if it is supposed to be one or not.
              if (capture_index_must_not_match[n]) {
                log += F(" (!=)");
                addLogMove(LOG_LEVEL_INFO, log);
                return false;
              } else {
                match_result = true;
                log         += F(" (==)");
              }
            } else {
              log += F(" No Match");

              if (capture_index_must_not_match[n]) {
                log += F(" (!=) ");
              } else {
                log += F(" (==) ");
              }
              log += _lines[lines_index];
            }
            addLogMove(LOG_LEVEL_INFO, log);
          }
        }
      }
    }
    capture_vector.clear();
  } else {
    char result = ms.Match(_lines[P087_REGEX_POS].c_str());

    if (result == REGEXP_MATCHED) {
      #ifndef BUILD_NO_DEBUG
      if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
        String log = F("Match at: ");
        log += ms.MatchStart;
        log += F(" Match Length: ");
        log += ms.MatchLength;
        addLogMove(LOG_LEVEL_DEBUG, log);
      }
      #endif
      match_result = true;
    }
  }
  return match_result;
}

const __FlashStringHelper * P087_data_struct::MatchType_toString(P087_Match_Type matchType) {
  switch (matchType)
  {
    case P087_Match_Type::Regular_Match:          return F("Regular Match");
    case P087_Match_Type::Regular_Match_inverted: return F("Regular Match inverted");
    case P087_Match_Type::Global_Match:           return F("Global Match");
    case P087_Match_Type::Global_Match_inverted:  return F("Global Match inverted");
    case P087_Match_Type::Filter_Disabled:        return F("Filter Disabled");
  }
  return F("");
}

bool P087_data_struct::max_length_reached() const {
  if (max_length == 0) { return false; }
  return sentence_part.length() >= max_length;
}

#endif // USES_P087

#include "../PluginStructs/P098_data_struct.h"

#ifdef USES_P098

# include "../ESPEasyCore/ESPEasyGPIO.h"

# include "../Commands/GPIO.h" // FIXME TD-er: Only needed till we can call GPIO commands from the ESPEasy core.

# include "../Helpers/Hardware.h"

# define GPIO_PLUGIN_ID  1


const __FlashStringHelper * P098_config_struct::toString(P098_config_struct::PWM_mode_type PWM_mode) {
  switch (PWM_mode) {
    case P098_config_struct::PWM_mode_type::NoPWM:  return F("No PWM");
    case P098_config_struct::PWM_mode_type::PWM:    return F("PWM");

    case P098_config_struct::PWM_mode_type::MAX_TYPE: break;
  }

  return F("");
}

P098_data_struct::P098_data_struct(const P098_config_struct& config) : _config(config) {}

P098_data_struct::~P098_data_struct() {
  if (initialized) {
    detachInterrupt(digitalPinToInterrupt(_config.limitA.gpio));
    detachInterrupt(digitalPinToInterrupt(_config.limitB.gpio));
    detachInterrupt(digitalPinToInterrupt(_config.encoder.gpio));
  }
}

bool P098_data_struct::begin(int pos, int limitApos, int limitBpos)
{
  if (!initialized) {
    initialized = true;

    const bool switchPosSet = pos != 0 && limitApos != 0;

    limitA.switchposSet = switchPosSet;
    limitA.switchpos    = switchPosSet ? limitApos : 0;
    limitB.switchpos    = limitBpos;
    position            = pos + limitA.switchpos;
    stop();
    state = P098_data_struct::State::Idle;

    if (setPinMode(_config.limitA)) {
      attachInterruptArg(
        digitalPinToInterrupt(_config.limitA.gpio),
        reinterpret_cast<void (*)(void *)>(ISRlimitA),
        this, CHANGE);
    }

    if (setPinMode(_config.limitB)) {
      attachInterruptArg(
        digitalPinToInterrupt(_config.limitB.gpio),
        reinterpret_cast<void (*)(void *)>(ISRlimitB),
        this, CHANGE);
    }

    if (setPinMode(_config.encoder)) {
      attachInterruptArg(
        digitalPinToInterrupt(_config.encoder.gpio),
        reinterpret_cast<void (*)(void *)>(ISRencoder),
        this, CHANGE); // Act on 'CHANGE', not on rising/falling
    }
  }

  return true;
}

bool P098_data_struct::loop()
{
  const State old_state(state);

  check_limit_switch(_config.limitA, limitA);
  check_limit_switch(_config.limitB, limitB);

  if (check_encoder_timeout(_config.encoder)) {
    stop();
    state = P098_data_struct::State::StopEncoderTimeout;  
    return old_state == state;
  }

  switch (state) {
    case P098_data_struct::State::Idle:
      return true;
    case P098_data_struct::State::RunFwd:
    {
      checkLimit(limitB);
      break;
    }
    case P098_data_struct::State::RunRev:
    {
      checkLimit(limitA);
      break;
    }
    case P098_data_struct::State::StopEncoderTimeout:
    case P098_data_struct::State::StopLimitSw:
    case P098_data_struct::State::StopPosReached:
      // Still in a state that needs inspection
      return false;
  }

  // Must check when state has changed from running to some other state
  return old_state == state;
}

bool P098_data_struct::homePosSet() const
{
  return limitA.switchposSet;
}

bool P098_data_struct::canRun()
{
  if (!homePosSet()) { return false; }

  switch (state) {
    case P098_data_struct::State::Idle:
    case P098_data_struct::State::RunFwd:
    case P098_data_struct::State::RunRev:
      return true;
    case P098_data_struct::State::StopLimitSw:
    case P098_data_struct::State::StopPosReached:
    case P098_data_struct::State::StopEncoderTimeout:
      // Still in a state that needs inspection
      return false;
  }
  return false;
}

void P098_data_struct::findHome()
{
  pos_dest            = INT_MIN;
  limitA.switchposSet = false;
  startMoving();
}

void P098_data_struct::moveForward(int steps)
{
  if (steps <= 0) {
    pos_dest            = INT_MAX;
    limitB.switchposSet = false;
  } else {
    pos_dest = position + steps;
  }
  startMoving();
}

void P098_data_struct::moveReverse(int steps)
{
  if (steps > 0) {
    pos_dest = position - steps;
    startMoving();
  }
}

bool P098_data_struct::moveToPos(int pos)
{
  if (!canRun()) { return false; }
  const int offset = pos - getPosition();

  pos_dest = position + offset;
  startMoving();
  return true;
}

void P098_data_struct::stop()
{
  setPinState(_config.motorFwd, 0);
  setPinState(_config.motorRev, 0);
}

int P098_data_struct::getPosition() const
{
  if (limitA.switchposSet) {
    return position - limitA.switchpos;
  }
  return position;
}

void P098_data_struct::getLimitSwitchStates(bool& limitA_triggered, bool& limitB_triggered) const
{
  limitA_triggered = limitA.state == P098_limit_switch_state::State::High ? 1 : 0;
  limitB_triggered = limitB.state == P098_limit_switch_state::State::High ? 1 : 0;

  /*
     limitA_triggered = _config.limitA.readState();
     limitB_triggered = _config.limitB.readState();
   */
}

void P098_data_struct::getLimitSwitchPositions(int& limitApos, int& limitBpos) const
{
  limitApos = limitA.switchposSet ? limitA.switchpos : 0;
  limitBpos = limitB.switchposSet ? limitB.switchpos : 0;
}

void P098_data_struct::startMoving()
{
  // Stop first, to make sure both outputs will not be set high
  stop();

  if (pos_dest > position) {
    state = P098_data_struct::State::RunFwd;
    setPinState(_config.motorFwd, 1);
  } else {
    state = P098_data_struct::State::RunRev;
    setPinState(_config.motorRev, 1);
  }
  // Touch the timer, so it will not immediately timeout.
  enc_lastChanged_us = getMicros64();
}

void P098_data_struct::checkLimit(volatile P098_limit_switch_state& switch_state)
{
  if (switch_state.state == P098_limit_switch_state::State::High) {
    stop();
    state = P098_data_struct::State::StopLimitSw;
    return;
  }
  checkPosition();
}

void P098_data_struct::checkPosition()
{
  bool mustStop = false;

  switch (state) {
    case P098_data_struct::State::RunFwd:

      mustStop = ((position + pos_overshoot) >= pos_dest);
      break;
    case P098_data_struct::State::RunRev:

      mustStop = ((position - pos_overshoot) <= pos_dest);
      break;
    default:
      return;
  }

  if (mustStop) {
    stop();
    pos_overshoot = 0;
    state = P098_data_struct::State::StopPosReached;

    /*
       // Correct for position error
       if (std::abs(position - pos_dest) > 10) {
       startMoving();
       }
     */
  }
}

void P098_data_struct::setPinState(const P098_GPIO_config& gpio_config, int8_t state)
{
  // FIXME TD-er: Must move this code to the ESPEasy core code.
  uint8_t mode = PIN_MODE_OUTPUT;

  state = state == 0 ? gpio_config.low() : gpio_config.high();
  uint32_t key = createKey(GPIO_PLUGIN_ID, gpio_config.gpio);

  if (globalMapPortStatus[key].mode != PIN_MODE_OFFLINE)
  {
    int8_t currentState;
    GPIO_Read(GPIO_PLUGIN_ID, gpio_config.gpio, currentState);

    if (currentState == -1) {
      mode  = PIN_MODE_OFFLINE;
      state = -1;
    }

    switch (_config.PWM_mode) {
      case P098_config_struct::PWM_mode_type::NoPWM:
        if (mode == PIN_MODE_OUTPUT)  {
          createAndSetPortStatus_Mode_State(key, mode, state);
          GPIO_Write(
            GPIO_PLUGIN_ID,
            gpio_config.gpio,
            state,
            mode);
        }
        break;
      case P098_config_struct::PWM_mode_type::PWM:
      {
        const uint32_t dutycycle = state == 0 ? 0 : _config.pwm_duty_cycle;
        const uint32_t fade_duration = _config.pwm_soft_startstop ? 
              100 /* (_config.encoder.timer_us / 1000) */
              : 0;
        uint32_t frequency = _config.pwm_freq;
        set_Gpio_PWM(
          gpio_config.gpio,
          dutycycle,
          fade_duration,
          frequency,
          key);
        if (state == 0) {
          // Turn off PWM mode too
          createAndSetPortStatus_Mode_State(key, PIN_MODE_OUTPUT, state);
          GPIO_Write(
            GPIO_PLUGIN_ID,
            gpio_config.gpio,
            state,
            PIN_MODE_OUTPUT);
        }
      }
      break;
      case P098_config_struct::PWM_mode_type::MAX_TYPE:
        break;
    }
  }
}

bool P098_data_struct::setPinMode(const P098_GPIO_config& gpio_config)
{
  if (checkValidPortRange(GPIO_PLUGIN_ID, gpio_config.gpio)) {
    pinMode(gpio_config.gpio, gpio_config.pullUp ? INPUT_PULLUP : INPUT);
    return true;
  }
  return false;
}

void P098_data_struct::check_limit_switch(
  const P098_GPIO_config          & gpio_config,
  volatile P098_limit_switch_state& switch_state)
{
  if (gpio_config.gpio == -1) {
    return;
  }
  // State is changed first in ISR, but compared after values are copied.
  const int triggerpos          = switch_state.triggerpos;
  const uint64_t lastChanged_us = switch_state.lastChanged_us;

  if (switch_state.state == P098_limit_switch_state::State::TriggerWaitBounce) {
    if (lastChanged_us != 0) {
      const uint64_t timeSinceLastTrigger = getMicros64() - lastChanged_us;

      if (timeSinceLastTrigger > gpio_config.timer_us) {
        mark_limit_switch_state(triggerpos, switch_state);
      }
    }
  }
}

void P098_data_struct::mark_limit_switch_state(
    int triggerpos, 
    volatile P098_limit_switch_state& switch_state)
{
  if (!switch_state.switchposSet) {
    switch_state.switchpos    = triggerpos;
    switch_state.switchposSet = true;
  }

  // Perform an extra check here on the state as it may have changed in the ISR call
  if (switch_state.state == P098_limit_switch_state::State::TriggerWaitBounce) {
    switch_state.state = P098_limit_switch_state::State::High;
  }
}

bool P098_data_struct::check_encoder_timeout(const P098_GPIO_config & gpio_config)
{
  if (gpio_config.gpio == -1) {
    return false;
  }
  if (enc_lastChanged_us == 0) {
    return false;
  }
  const bool expired = usecPassedSince(enc_lastChanged_us) > static_cast<int64_t>(_config.encoder.timer_us);
  if (!expired) {
    return false;
  }
  switch (state) {
    case P098_data_struct::State::RunFwd:
    {
      mark_limit_switch_state(position, limitB);
      break;
    }
    case P098_data_struct::State::RunRev:
    {
      mark_limit_switch_state(position, limitA);
      break;
    }
    default:
      return false;
  }
  return true;
}

void ICACHE_RAM_ATTR P098_data_struct::process_limit_switch(
  const P098_GPIO_config          & gpio_config,
  volatile P098_limit_switch_state& switch_state,
  int                               position)
{
  noInterrupts();
  {
    // Don't call gpio_config.readState() here
    const bool pinState        = gpio_config.inverted ? digitalRead(gpio_config.gpio) == 0 : digitalRead(gpio_config.gpio) != 0;
    const uint64_t currentTime = getMicros64();


    switch (switch_state.state) {
      case P098_limit_switch_state::State::Low:

        if (pinState) {
          switch_state.state          = P098_limit_switch_state::State::TriggerWaitBounce;
          switch_state.lastChanged_us = currentTime;
          switch_state.triggerpos     = position;
        }
        break;
      case P098_limit_switch_state::State::TriggerWaitBounce:
      {
        // Do not evaluate the debounce time here, evaluate in the loop
        if (pinState) {
          // Only situation we can get here is when we missed a low state interrupt.
          switch_state.lastChanged_us = currentTime;
          switch_state.triggerpos     = position;
        } else {
          switch_state.state          = P098_limit_switch_state::State::Low;
          switch_state.lastChanged_us = 0;
          switch_state.triggerpos     = 0;
        }
        break;
      }
      case P098_limit_switch_state::State::High:

        if (!pinState) {
          switch_state.state          = P098_limit_switch_state::State::Low;
          switch_state.lastChanged_us = 0;
          switch_state.triggerpos     = 0;
        }
        break;
    }
  }
  interrupts(); // enable interrupts again.
}

void ICACHE_RAM_ATTR P098_data_struct::ISRlimitA(P098_data_struct *self)
{
  process_limit_switch(self->_config.limitA, self->limitA, self->position);
}

void ICACHE_RAM_ATTR P098_data_struct::ISRlimitB(P098_data_struct *self)
{
  process_limit_switch(self->_config.limitB, self->limitB, self->position);
}

void ICACHE_RAM_ATTR P098_data_struct::ISRencoder(P098_data_struct *self)
{
  noInterrupts();

  switch (self->state) {
    case P098_data_struct::State::RunFwd:
      ++(self->position);
      break;
    case P098_data_struct::State::RunRev:
      --(self->position);
      break;
    default:
      interrupts(); // enable interrupts again.
      return;
  }
  self->enc_lastChanged_us = getMicros64();
  interrupts(); // enable interrupts again.
}

#endif // ifdef USES_P098

#include "../PluginStructs/P075_data_struct.h"

#ifdef USES_P075


P075_data_struct::P075_data_struct(ESPEasySerialPort port, int rx, int tx, uint32_t baud) : rxPin(rx), txPin(tx), baudrate(baud) {
  if ((baudrate < 9600) || (baudrate > 115200)) {
    baudrate = 9600;
  }
  easySerial = new (std::nothrow) ESPeasySerial(port, rx, tx, false, RXBUFFSZ);

  if (easySerial != nullptr) {
    easySerial->begin(baudrate);
    easySerial->flush();
  }
}

P075_data_struct::~P075_data_struct() {
  if (easySerial != nullptr) {
    easySerial->flush();
    delete easySerial;
    easySerial = nullptr;
  }
}

void P075_data_struct::loadDisplayLines(taskIndex_t taskIndex) {
  LoadCustomTaskSettings(taskIndex, displayLines, P75_Nlines, P75_Nchars);
}

String P075_data_struct::getLogString() const {
  String result;

  if (easySerial != nullptr) {
    result = easySerial->getLogString();
  }
  return result;
}

#endif // ifdef USES_P075

#include "../PluginStructs/P015_data_struct.h"
#ifdef USES_P015

#include "../Helpers/Misc.h"



# define TSL2561_CMD           0x80
# define TSL2561_REG_CONTROL   0x00
# define TSL2561_REG_TIMING    0x01
# define TSL2561_REG_DATA_0    0x0C
# define TSL2561_REG_DATA_1    0x0E


P015_data_struct::P015_data_struct(uint8_t i2caddr, unsigned int gain, uint8_t integration) :
  _gain(gain),
  _i2cAddr(i2caddr),
  _integration(integration)
{
  // If gain = false (0), device is set to low gain (1X)
  // If gain = high (1), device is set to high gain (16X)

  _gain16xActive = gain == 1;

  if (!useAutoGain()) {
    _gain16xActive = gain == 1;
  }
}

bool P015_data_struct::performRead(float& luxVal,
                                   float& infraredVal,
                                   float& broadbandVal,
                                   float& ir_broadband_ratio)
{
  bool success = false;
  int  attempt = useAutoGain() ? 2 : 1;

  while (!success && attempt > 0) {
    --attempt;

    float ms; // Integration ("shutter") time in milliseconds

    // If time = 0, integration will be 13.7ms
    // If time = 1, integration will be 101ms
    // If time = 2, integration will be 402ms
    unsigned char time = _integration;

    plugin_015_setTiming(_gain16xActive, time, ms);
    setPowerUp();
    delayBackground(ms); // FIXME TD-er: Do not use delayBackground but collect data later.
    unsigned int data0, data1;

    if (getData(data0, data1))
    {
      float lux;       // Resulting lux value
      float infrared;  // Resulting infrared value
      float broadband; // Resulting broadband value


      // Perform lux calculation:
      success = !ADC_saturated(time,  data0) && !ADC_saturated(time, data1);
      getLux(_gain16xActive, ms, data0, data1, lux, infrared, broadband);

      if (useAutoGain()) {
        if (_gain16xActive) {
          // Last reading was using 16x gain
          // Check using some margin to see if gain is still needed
          if (ADC_saturated(time,  data0 * 16)) {
            _gain16xActive = false;
          }
        } else {
          // Check using some margin to see if gain will improve reading resolution
          if (lux < 40) {
            _gain16xActive = true;
          }
        }
      }

      if (success) {
        if (broadband > 0.0f) {
          // Store the ratio in an unused user var. (should we make it available?)
          // Only store/update it when not close to the limits of both ADC ranges.
          // When using this value to compute extended ranges, it must not be using a ratio taken from a
          // heated sensor, since then the IR part may be off quite a bit resulting in very unrealistic values.
          if (!ADC_saturated(time,  data0 * 2) && !ADC_saturated(time, data1 * 2)) {
            ir_broadband_ratio = infrared / broadband;
          }
        }
      } else {
        // Use last known ratio to reconstruct the broadband value
        // If IR is saturated, output the max value based on the last known ratio.
        if ((ir_broadband_ratio > 0.0f) && (_gain == P015_EXT_AUTO_GAIN)) {
          data0 = static_cast<float>(data1) / ir_broadband_ratio;
          getLux(_gain16xActive, ms, data0, data1, lux, infrared, broadband);
          success = true;
        }
      }
      luxVal       = lux;
      infraredVal  = infrared;
      broadbandVal = broadband;
    }
    else
    {
      // getData() returned false because of an I2C error, inform the user.
      addLog(LOG_LEVEL_ERROR, F("TSL2561: i2c error"));
      success = false;
      attempt = 0;
    }
  }
  return success;
}

bool P015_data_struct::useAutoGain() const
{
  const bool autoGain = _gain == P015_AUTO_GAIN || _gain == P015_EXT_AUTO_GAIN;

  return autoGain;
}

bool P015_data_struct::begin()
{
  // Wire.begin();   called in ESPEasy framework
  return true;
}

bool P015_data_struct::readByte(unsigned char address, unsigned char& value)

// Reads a byte from a TSL2561 address
// Address: TSL2561 address (0 to 15)
// Value will be set to stored byte
// Returns true (1) if successful, false (0) if there was an I2C error
{
  // Set up command byte for read
  Wire.beginTransmission(_i2cAddr);
  Wire.write((address & 0x0F) | TSL2561_CMD);
  _error = Wire.endTransmission();

  // Read requested byte
  if (_error == 0)
  {
    Wire.requestFrom(_i2cAddr, (uint8_t)1);

    if (Wire.available() == 1)
    {
      value = Wire.read();
      return true;
    }
  }
  return false;
}

bool P015_data_struct::writeByte(unsigned char address, unsigned char value)

// Write a byte to a TSL2561 address
// Address: TSL2561 address (0 to 15)
// Value: byte to write to address
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() above)
{
  // Set up command byte for write
  Wire.beginTransmission(_i2cAddr);
  Wire.write((address & 0x0F) | TSL2561_CMD);

  // Write byte
  Wire.write(value);
  _error = Wire.endTransmission();

  if (_error == 0) {
    return true;
  }

  return false;
}

bool P015_data_struct::readUInt(unsigned char address, unsigned int& value)

// Reads an unsigned integer (16 bits) from a TSL2561 address (low byte first)
// Address: TSL2561 address (0 to 15), low byte first
// Value will be set to stored unsigned integer
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() above)
{
  // Set up command byte for read
  Wire.beginTransmission(_i2cAddr);
  Wire.write((address & 0x0F) | TSL2561_CMD);
  _error = Wire.endTransmission();

  // Read two bytes (low and high)
  if (_error == 0)
  {
    Wire.requestFrom(_i2cAddr, (uint8_t)2);

    if (Wire.available() == 2)
    {
      char high, low;
      low  = Wire.read();
      high = Wire.read();

      // Combine bytes into unsigned int
      value = word(high, low);
      return true;
    }
  }
  return false;
}

bool P015_data_struct::writeUInt(unsigned char address, unsigned int value)

// Write an unsigned integer (16 bits) to a TSL2561 address (low byte first)
// Address: TSL2561 address (0 to 15), low byte first
// Value: unsigned int to write to address
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() above)
{
  // Split int into lower and upper bytes, write each byte
  if (writeByte(address, lowByte(value))
      && writeByte(address + 1, highByte(value))) {
    return true;
  }

  return false;
}

bool P015_data_struct::plugin_015_setTiming(bool gain, unsigned char time)

// If gain = false (0), device is set to low gain (1X)
// If gain = high (1), device is set to high gain (16X)
// If time = 0, integration will be 13.7ms
// If time = 1, integration will be 101ms
// If time = 2, integration will be 402ms
// If time = 3, use manual start / stop
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)
{
  unsigned char timing;

  // Get timing byte
  if (readByte(TSL2561_REG_TIMING, timing))
  {
    // Set gain (0 or 1)
    if (gain) {
      timing |= 0x10;
    }
    else {
      timing &= ~0x10;
    }

    // Set integration time (0 to 3)
    timing &= ~0x03;
    timing |= (time & 0x03);

    // Write modified timing byte back to device
    if (writeByte(TSL2561_REG_TIMING, timing)) {
      return true;
    }
  }
  return false;
}

bool P015_data_struct::plugin_015_setTiming(bool gain, unsigned char time, float& ms)

// If gain = false (0), device is set to low gain (1X)
// If gain = high (1), device is set to high gain (16X)
// If time = 0, integration will be 13.7ms
// If time = 1, integration will be 101ms
// If time = 2, integration will be 402ms
// If time = 3, use manual start / stop (ms = 0)
// ms will be set to integration time
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)
{
  // Calculate ms for user
  switch (time)
  {
    case 0: ms  = 13.7f; break;
    case 1: ms  = 101; break;
    case 2: ms  = 402; break;
    default: ms = 402; // used in a division, so do not use 0
  }

  // Set integration using base function
  return plugin_015_setTiming(gain, time);
}

// Determine if either sensor saturated (max depends on clock freq. and integration time)
// If so, abandon ship (calculation will not be accurate)
bool P015_data_struct::ADC_saturated(unsigned char time, unsigned int value) {
  unsigned int max_ADC_count = 65535;

  switch (time)
  {
    case 0: max_ADC_count = 5047; break;
    case 1: max_ADC_count = 37177; break;
    case 2:
    default: break;
  }
  return value >= max_ADC_count;
}

bool P015_data_struct::setPowerUp(void)

// Turn on TSL2561, begin integrations
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)
{
  // Write 0x03 to command byte (power on)
  return writeByte(TSL2561_REG_CONTROL, 0x03);
}

bool P015_data_struct::setPowerDown(void)

// Turn off TSL2561
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)
{
  // Clear command byte (power off)
  return writeByte(TSL2561_REG_CONTROL, 0x00);
}

bool P015_data_struct::getData(unsigned int& data0, unsigned int& data1)

// Retrieve raw integration results
// data0 and data1 will be set to integration results
// Returns true (1) if successful, false (0) if there was an I2C error
// (Also see getError() below)
{
  // Get data0 and data1 out of result registers
  if (readUInt(TSL2561_REG_DATA_0, data0) && readUInt(TSL2561_REG_DATA_1, data1)) {
    return true;
  }

  return false;
}

void P015_data_struct::getLux(unsigned char gain,
                              float         ms,
                              unsigned int  CH0,
                              unsigned int  CH1,
                              float       & lux,
                              float       & infrared,
                              float       & broadband)

// Convert raw data to lux
// gain: 0 (1X) or 1 (16X), see setTiming()
// ms: integration time in ms, from setTiming() or from manual integration
// CH0, CH1: results from getData()
// lux will be set to resulting lux calculation
// returns true (1) if calculation was successful
// RETURNS false (0) AND lux = 0.0 IF EITHER SENSOR WAS SATURATED (0XFFFF)
{
  float ratio, d0, d1;

  // Convert from unsigned integer to floating point
  d0 = CH0; d1 = CH1;

  // We will need the ratio for subsequent calculations
  ratio = d1 / d0;

  // save original values
  infrared  = d1;
  broadband = d0;

  // Normalize for integration time
  d0 *= (402.0f / ms);
  d1 *= (402.0f / ms);

  // Normalize for gain
  if (!gain)
  {
    d0 *= 16;
    d1 *= 16;
  }

  // Determine lux per datasheet equations:
  if (ratio < 0.5f)
  {
    lux = 0.0304f * d0 - 0.062f * d0 * powf(ratio, 1.4f);
  } else if (ratio < 0.61f)
  {
    lux = 0.0224f * d0 - 0.031f * d1;
  } else if (ratio < 0.80f)
  {
    lux = 0.0128f * d0 - 0.0153f * d1;
  } else if (ratio < 1.30f)
  {
    lux = 0.00146f * d0 - 0.00112f * d1;
  } else {
    // ratio >= 1.30
    lux = 0.0f;
  }
}

#endif // ifdef USES_P015

#include "../PluginStructs/P025_data_struct.h"

#ifdef USES_P025

P025_data_struct::P025_data_struct(uint8_t i2c_addr, uint8_t _pga, uint8_t _mux) : pga(_pga), mux(_mux), i2cAddress(i2c_addr) {}

int16_t P025_data_struct::read() {
  uint16_t config = (0x0003)    | // Disable the comparator (default val)
                    (0x0000)    | // Non-latching (default val)
                    (0x0000)    | // Alert/Rdy active low   (default val)
                    (0x0000)    | // Traditional comparator (default val)
                    (0x0080)    | // 128 samples per second (default)
                    (0x0100);     // Single-shot mode (default)

  config |= static_cast<uint16_t>(pga) << 9;
  config |= static_cast<uint16_t>(mux) << 12;
  config |= (0x8000); // Start a single conversion

  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)(0x01));
  Wire.write((uint8_t)(config >> 8));
  Wire.write((uint8_t)(config & 0xFF));
  Wire.endTransmission();

  delay(9); // See https://github.com/letscontrolit/ESPEasy/issues/3159#issuecomment-660546091
  return readRegister025(0x00);
}

uint16_t P025_data_struct::readRegister025(uint8_t reg) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((0x00));
  Wire.endTransmission();

  if (Wire.requestFrom(i2cAddress, (uint8_t)2) != 2) {
    return 0x8000;
  }
  return (Wire.read() << 8) | Wire.read();
}

#endif // ifdef USES_P025

#include "../PluginStructs/P036_data_struct.h"

#ifdef USES_P036

# include <Arduino.h>

# include "../ESPEasyCore/ESPEasyNetwork.h"
# include "../Helpers/ESPEasy_Storage.h"
# include "../Helpers/Misc.h"
# include "../Helpers/Scheduler.h"
# include "../Helpers/StringConverter.h"
# include "../Helpers/StringParser.h"
# include "../Helpers/SystemVariables.h"

# include <Dialog_Plain_12_font.h>
# include <OLED_SSD1306_SH1106_images.h>

P036_data_struct::P036_data_struct() : display(nullptr) {}

P036_data_struct::~P036_data_struct() {
  reset();
}

void P036_data_struct::reset() {
  if (display != nullptr) {
    display->displayOff();
    display->end();
    delete display;
    display = nullptr;
  }
}

// FIXME TD-er: with using functions to get the font, this object is stored in .dram0.data
// The same as when using the DRAM_ATTR attribute used for interrupt code.
// This is very precious memory, so we must find something other way to define this.
const tFontSizes FontSizes[P36_MaxFontCount] = {
  { getArialMT_Plain_24(), 24,    28                 }, // 9643
  { getArialMT_Plain_16(), 16,    19                 }, // 5049
  { getDialog_plain_12(),  13,    15                 }, // 3707
  { getArialMT_Plain_10(), 10,    13                 }, // 2731
};

const tSizeSettings SizeSettings[P36_MaxSizesCount] = {
  { P36_MaxDisplayWidth, P36_MaxDisplayHeight, 0,  // 128x64
    4,                                             // max. line count
    113, 15                                        // WiFi indicator
  },
  { P36_MaxDisplayWidth, 32,                   0,  // 128x32
    2,                                             // max. line count
    113, 15                                        // WiFi indicator
  },
  { 64,                  48,                   32, // 64x48
    3,                                             // max. line count
    32,  10                                        // WiFi indicator
  }
};


const tSizeSettings& P036_data_struct::getDisplaySizeSettings(p036_resolution disp_resolution) {
  int index = static_cast<int>(disp_resolution);

  if ((index < 0) || (index >= P36_MaxSizesCount)) { index = 0; }

  return SizeSettings[index];
}

bool P036_data_struct::init(taskIndex_t     taskIndex,
                            uint8_t         LoadVersion,
                            uint8_t         Type,
                            uint8_t         Address,
                            uint8_t         Sda,
                            uint8_t         Scl,
                            p036_resolution Disp_resolution,
                            bool            Rotated,
                            uint8_t         Contrast,
                            uint8_t         DisplayTimer,
                            uint8_t         NrLines) {
  reset();

  lastWiFiState       = P36_WIFI_STATE_UNSET;
  disp_resolution     = Disp_resolution;
  bAlternativHeader   = false; // start with first header content
  HeaderCount         = 0;
  bPageScrollDisabled = true;  // first page after INIT without scrolling
  TopLineOffset       = 0;     // Offset for top line, used for rotated image while using displays < P36_MaxDisplayHeight lines

  HeaderContent            = eHeaderContent::eSysName;
  HeaderContentAlternative = eHeaderContent::eSysName;
  MaxFramesToDisplay       = 0xFF;
  currentFrameToDisplay    = 0;
  nextFrameToDisplay       = 0xFF; // next frame because content changed in PLUGIN_WRITE

  ButtonState     = false;         // button not touched
  ButtonLastState = 0xFF;          // Last state checked (debouncing in progress)
  DebounceCounter = 0;             // debounce counter
  RepeatCounter   = 0;             // Repeat delay counter when holding button pressed

  displayTimer          = DisplayTimer;
  frameCounter          = 0;       // need to keep track of framecounter from call to call
  disableFrameChangeCnt = 0;       // counter to disable frame change after JumpToPage in case PLUGIN_READ already scheduled

  switch (Type) {
    case 1:
      display = new (std::nothrow) SSD1306Wire(Address, Sda, Scl);
      break;
    case 2:
      display = new (std::nothrow) SH1106Wire(Address, Sda, Scl);
      break;
    default:
      return false;
  }

  if (display != nullptr) {
    display->init(); // call to local override of init function

    // disp_resolution = Disp_resolution;
    bHideFooter |= !(getDisplaySizeSettings(disp_resolution).Height == P36_MaxDisplayHeight);

    if (disp_resolution == p036_resolution::pix128x32) {
      display->displayOff();
      display->SetComPins(0x02); // according to the adafruit lib, sometimes this may need to be 0x02
      bHideFooter = true;
    }

    display->displayOn();
    loadDisplayLines(taskIndex, LoadVersion);

    // Flip screen if required
    setOrientationRotated(Rotated);

    setContrast(Contrast);

    //      Display the device name, logo, time and wifi
    display_logo();
    update_display();

    //    Initialize frame counter
    frameCounter                 = 0;
    currentFrameToDisplay        = 0;
    nextFrameToDisplay           = 0;
    bPageScrollDisabled          = true;  // first page after INIT without scrolling
    ScrollingPages.linesPerFrame = NrLines;
    bLineScrollEnabled           = false; // start without line scrolling

    //    Clear scrolling line data
    for (uint8_t i = 0; i < P36_MAX_LinesPerPage; i++) {
      ScrollingLines.Line[i].Width     = 0;
      ScrollingLines.Line[i].LastWidth = 0;
    }

    //    prepare font and positions for page and line scrolling
    prepare_pagescrolling();
  }

  return isInitialized();
}

bool P036_data_struct::isInitialized() const {
  return display != nullptr;
}

void P036_data_struct::loadDisplayLines(taskIndex_t taskIndex, uint8_t LoadVersion) {
  if (LoadVersion == 0) {
    // read data of version 0 (up to 22.11.2019)
    String DisplayLinesV0[P36_Nlines];                                           // used to load the CustomTaskSettings for V0
    LoadCustomTaskSettings(taskIndex, DisplayLinesV0, P36_Nlines, P36_NcharsV0); // max. length 1024 Byte  (DAT_TASKS_CUSTOM_SIZE)

    for (int i = 0; i < P36_Nlines; ++i) {
      safe_strncpy(DisplayLinesV1[i].Content, DisplayLinesV0[i], P36_NcharsV1);
      DisplayLinesV1[i].Content[P36_NcharsV1 - 1] = 0; // Terminate in case of uninitalized data
      DisplayLinesV1[i].FontType                  = 0xff;
      DisplayLinesV1[i].FontHeight                = 0xff;
      DisplayLinesV1[i].FontSpace                 = 0xff;
      DisplayLinesV1[i].reserved                  = 0xff;
    }
  } else {
    // read data of version 1 (beginning from 22.11.2019)
    LoadCustomTaskSettings(taskIndex, reinterpret_cast<uint8_t *>(&DisplayLinesV1), sizeof(DisplayLinesV1));

    for (int i = 0; i < P36_Nlines; ++i) {
      DisplayLinesV1[i].Content[P36_NcharsV1 - 1] = 0; // Terminate in case of uninitalized data
    }
  }
}

void P036_data_struct::setContrast(uint8_t OLED_contrast) {
  if (!isInitialized()) {
    return;
  }
  char contrast  = 100;
  char precharge = 241;
  char comdetect = 64;

  switch (OLED_contrast) {
    case P36_CONTRAST_OFF:
      display->displayOff();
      return;
    case P36_CONTRAST_LOW:
      contrast = 10; precharge = 5; comdetect = 0;
      break;
    case P36_CONTRAST_MED:
      contrast = P36_CONTRAST_MED; precharge = 0x1F; // comdetect = 64;
      break;
    case P36_CONTRAST_HIGH:
    default:
      contrast = P36_CONTRAST_HIGH; precharge = 241; // comdetect = 64;
      break;
  }
  display->displayOn();
  display->setContrast(contrast, precharge, comdetect);
}

void P036_data_struct::setOrientationRotated(bool rotated) {
  if (rotated) {
    display->flipScreenVertically();
    TopLineOffset = P36_MaxDisplayHeight - getDisplaySizeSettings(disp_resolution).Height;
  } else {
    TopLineOffset = 0;
  }
}

# ifdef P036_ENABLE_LINECOUNT
void P036_data_struct::setNrLines(uint8_t NrLines) {
  if ((NrLines >= 1) && (NrLines <= 4)) {
    ScrollingPages.linesPerFrame = NrLines;
    prepare_pagescrolling();   // Recalculate font
    MaxFramesToDisplay = 0xFF; // Recalculate page indicator
    nextFrameToDisplay = 0;    // Reset to first page
  }
}

# endif // P036_ENABLE_LINECOUNT


void P036_data_struct::display_header() {
  if (!isInitialized()) {
    return;
  }

  if (bHideHeader) { //  hide header
    return;
  }

  eHeaderContent iHeaderContent;
  String newString, strHeader;

  if ((HeaderContentAlternative == HeaderContent) || !bAlternativHeader) {
    iHeaderContent = HeaderContent;
  } else {
    iHeaderContent = HeaderContentAlternative;
  }

  switch (iHeaderContent) {
    case eHeaderContent::eSSID:

      if (NetworkConnected()) {
        strHeader = WiFi.SSID();
      }
      else {
        newString = F("%sysname%");
      }
      break;
    case eHeaderContent::eSysName:
      newString = F("%sysname%");
      break;
    case eHeaderContent::eTime:
      newString = F("%systime%");
      break;
    case eHeaderContent::eDate:
      newString = F("%sysday_0%.%sysmonth_0%.%sysyear%");
      break;
    case eHeaderContent::eIP:
      newString = F("%ip%");
      break;
    case eHeaderContent::eMAC:
      newString = F("%mac%");
      break;
    case eHeaderContent::eRSSI:
      newString = F("%rssi%dBm");
      break;
    case eHeaderContent::eBSSID:
      newString = F("%bssid%");
      break;
    case eHeaderContent::eWiFiCh:
      newString = F("Channel: %wi_ch%");
      break;
    case eHeaderContent::eUnit:
      newString = F("Unit: %unit%");
      break;
    case eHeaderContent::eSysLoad:
      newString = F("Load: %sysload%%");
      break;
    case eHeaderContent::eSysHeap:
      newString = F("Mem: %sysheap%");
      break;
    case eHeaderContent::eSysStack:
      newString = F("Stack: %sysstack%");
      break;
    case eHeaderContent::ePageNo:
      strHeader  = F("page ");
      strHeader += (currentFrameToDisplay + 1);

      if (MaxFramesToDisplay != 0xFF) {
        strHeader += F("/");
        strHeader += (MaxFramesToDisplay + 1);
      }
      break;
    default:
      return;
  }

  if (newString.length() > 0) {
    // Right now only systemvariables have been used, so we don't have to call the parseTemplate.
    parseSystemVariables(newString, false);
    strHeader = newString;
  }

  strHeader.trim();
  display_title(strHeader);

  // Display time and wifibars both clear area below, so paint them after the title.
  if (getDisplaySizeSettings(disp_resolution).Width == P36_MaxDisplayWidth) {
    display_time(); // only for 128pix wide displays
  }
  display_wifibars();
}

void P036_data_struct::display_time() {
  if (!isInitialized()) {
    return;
  }

  String dtime = F("%systime%");

  parseSystemVariables(dtime, false);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(getArialMT_Plain_10());
  display->setColor(BLACK);
  display->fillRect(0, TopLineOffset, 28, GetHeaderHeight() - 2);
  display->setColor(WHITE);
  display->drawString(0, TopLineOffset, dtime.substring(0, 5));
}

void P036_data_struct::display_title(const String& title) {
  if (!isInitialized()) {
    return;
  }
  display->setFont(getArialMT_Plain_10());
  display->setColor(BLACK);
  display->fillRect(0, TopLineOffset, P36_MaxDisplayWidth, GetHeaderHeight()); // don't clear line under title.
  display->setColor(WHITE);

  if (getDisplaySizeSettings(disp_resolution).Width == P36_MaxDisplayWidth) {
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(P36_DisplayCentre, TopLineOffset, title);
  } else {
    display->setTextAlignment(TEXT_ALIGN_LEFT); // Display right of WiFi bars
    display->drawString(getDisplaySizeSettings(disp_resolution).PixLeft + getDisplaySizeSettings(disp_resolution).WiFiIndicatorWidth + 3,
                        TopLineOffset,
                        title);
  }
}

void P036_data_struct::display_logo() {
  if (!isInitialized()) {
    return;
  }
  # ifdef PLUGIN_036_DEBUG
  addLog(LOG_LEVEL_INFO, F("P036_DisplayLogo"));
  # endif // PLUGIN_036_DEBUG

  int left = 24;
  int top;
  tFontSettings iFontsettings = CalculateFontSettings(2); // get font with max. height for displaying "ESP Easy"

  bDisplayingLogo = true;                                 // next time the display must be cleared completely
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(iFontsettings.fontData);
  display->clear();                                       // resets all pixels to black
  display->setColor(WHITE);
  display->drawString(65, iFontsettings.Top + TopLineOffset,                                              F("ESP"));
  display->drawString(65, iFontsettings.Top + iFontsettings.Height + iFontsettings.Space + TopLineOffset, F("Easy"));

  if (getDisplaySizeSettings(disp_resolution).PixLeft < left) { left = getDisplaySizeSettings(disp_resolution).PixLeft; }
  top = (getDisplaySizeSettings(disp_resolution).Height - espeasy_logo_height) / 2;

  if (top < 0) { top = 0; }
  display->drawXbm(left,
                   top + TopLineOffset,
                   espeasy_logo_width,
                   espeasy_logo_height,
                   espeasy_logo_bits); // espeasy_logo_width=espeasy_logo_height=36
}

// Draw the frame position

void P036_data_struct::display_indicator() {
  if (!isInitialized()) {
    return;
  }

  if (bHideFooter) { //  hide footer
    return;
  }

  int frameCount = MaxFramesToDisplay + 1;

  //  Erase Indicator Area
  display->setColor(BLACK);
  display->fillRect(0, P036_IndicatorTop + TopLineOffset, P36_MaxDisplayWidth, P036_IndicatorHeight);

  // Only display when there is something to display.
  if ((frameCount <= 1) || (frameCount > P36_Nlines)) { return; }

  display->setColor(WHITE);

  // Display chars as required
  for (uint8_t i = 0; i < frameCount; i++) {
    const char *image;

    if (currentFrameToDisplay == i) {
      image = activeSymbole;
    } else {
      image = inactiveSymbole;
    }

    int x, y;

    y = P036_IndicatorTop + TopLineOffset; // 2022-01-31 Removed unneeded offset '+ 2'

    // I would like a margin of 20 pixels on each side of the indicator.
    // Therefore the width of the indicator should be 128-40=88 and so space between indicator dots is 88/(framecount-1)
    // The width of the display is 128 and so the first dot must be at x=20 if it is to be centred at 64
    const int number_spaces = frameCount - 1;

    if (number_spaces <= 0) {
      return;
    }
    int margin  = 20;
    int spacing = (P36_MaxDisplayWidth - 2 * margin) / number_spaces;

    // Now apply some max of 30 pixels between the indicators and center the row of indicators.
    if (spacing > 30) {
      spacing = 30;
      margin  = (P36_MaxDisplayWidth - number_spaces * spacing) / 2;
    }

    x = margin + (spacing * i);
    display->drawXbm(x, y, 8, 8, image);
  }
}

int16_t P036_data_struct::GetHeaderHeight() {
  if (bHideHeader) {
    // no header
    return 0;
  }
  return P36_HeaderHeight;
}

int16_t P036_data_struct::GetIndicatorTop() {
  if (bHideFooter) {
    // no footer (indicator) -> returm max. display height
    return getDisplaySizeSettings(disp_resolution).Height;
  }
  return P036_IndicatorTop;
}

tFontSettings P036_data_struct::CalculateFontSettings(uint8_t lDefaultLines) {
  tFontSettings result;
  int iHeight;
  int8_t  iFontIndex = -1;
  uint8_t iMaxHeightForFont;
  uint8_t iLinesPerFrame;

  if (lDefaultLines == 0) {
    // number of lines can be reduced if no font fits the setting
    iLinesPerFrame = ScrollingPages.linesPerFrame;
    iHeight        = GetIndicatorTop() - GetHeaderHeight();
  } else {
    // number of lines is fixed (e.g. for splash screen)
    iLinesPerFrame = lDefaultLines;
    iHeight        = getDisplaySizeSettings(disp_resolution).Height;
  }

  # ifdef P036_FONT_CALC_LOG

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log;
    log.reserve(80);
    log += F("P036 CalculateFontSettings lines: ");
    log += iLinesPerFrame;
    log += F(", height: ");
    log += iHeight;
    log += F(", header: ");
    log += boolToString(!bHideHeader);
    log += F(", footer: ");
    log += boolToString(!bHideFooter);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  String log;
  # endif // ifdef P036_FONT_CALC_LOG

  while (iFontIndex < 0) {
    iMaxHeightForFont = round(iHeight / (iLinesPerFrame * 1.0f)); // no extra space between lines
    // Fonts already have their own extra space, no need to add an extra pixel space

    # ifdef P036_FONT_CALC_LOG
    uint8_t prLines = iLinesPerFrame;
    log.reserve(80);
    log.clear();
    log += F("CalculateFontSettings prLines: ");
    log += prLines;
    log += F(", max: ");
    log += iMaxHeightForFont;
    # endif // ifdef P036_FONT_CALC_LOG

    for (uint8_t i = P36_MaxFontCount; i > 0; i--) {
      // check available fonts for the line setting
      # ifdef P036_FONT_CALC_LOG
      log += F(", i: ");
      log += i;
      log += F(", h: ");
      log += FontSizes[i - 1].Height;
      # endif // ifdef P036_FONT_CALC_LOG

      if (FontSizes[i - 1].Height > iMaxHeightForFont) {
        // height of font is too big
        break;
      }
      iFontIndex = i - 1; // save the current index
      # ifdef P036_FONT_CALC_LOG
      log += F(", f: ");
      log += iFontIndex;
      # endif // ifdef P036_FONT_CALC_LOG

      if (FontSizes[iFontIndex].Height == iMaxHeightForFont) {
        // height of font just fits the line setting
        break;
      }
    }

    if (iFontIndex < 0) {
      // FIXED: tonhuisman Reduce number of lines feature disabled for now, by tweaking the font metrics for 4 lines, see below
      // no font fits -> reduce number of lines per page
      // iLinesPerFrame--;
      # ifdef P036_FONT_CALC_LOG
      log += F(", L: ");
      log += iLinesPerFrame;
      # endif // ifdef P036_FONT_CALC_LOG

      // if (iLinesPerFrame == 0) {
      // lines per frame is at minimum
      break;

      // }
    }
    # ifdef P036_FONT_CALC_LOG
    log += F(", font: ");
    log += iFontIndex;
    addLogMove(LOG_LEVEL_INFO, log);
    # endif // ifdef P036_FONT_CALC_LOG
  }

  if (iFontIndex >= 0) {
    // font found -> calculate top position and space between lines
    iMaxHeightForFont = FontSizes[iFontIndex].Height * iLinesPerFrame;

    if (iLinesPerFrame > 1) {
      // more than one lines per frame -> calculate space inbetween
      result.Space = (iHeight - iMaxHeightForFont) / iLinesPerFrame;
    } else {
      // just one lines per frame -> no space inbetween
      result.Space = 0;
    }
    result.Top = (iHeight - (iMaxHeightForFont + (result.Space * (iLinesPerFrame - 1)))) / 2;
  } else {
    // no font found -> return font with shortest height
    // FIXED: tonhuisman Tweaked to match the 13 pix font to fit for 4 lines display
    // iLinesPerFrame = 1; // Disabled to show all 4 lines
    result.Top   = 0;
    result.Space = bHideFooter ? 0 : -2;
    iFontIndex   = P36_MaxFontCount - 1;
  }
  result.fontData = FontSizes[iFontIndex].fontData;
  result.Height   = FontSizes[iFontIndex].Height;

  # ifdef PLUGIN_036_DEBUG
  String log;

  if (loglevelActiveFor(LOG_LEVEL_INFO) &&
      log.reserve(128)) { // estimated
    log += F("CalculateFontSettings: FontIndex:");
    log += iFontIndex;
    log += F(" Top:");
    log += result.Top;
    log += F(" FontHeight:");
    log += result.Height;
    log += F(" Space:");
    log += result.Space;
    log += F(" Height:");
    log += iHeight;
    log += F(" LinesPerFrame:");
    log += iLinesPerFrame;
    log += F(" DefaultLines:");
    log += lDefaultLines;
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // PLUGIN_036_DEBUG

  if (lDefaultLines == 0) {
    ScrollingPages.linesPerFrame = iLinesPerFrame;
  }
  return result;
}

void P036_data_struct::prepare_pagescrolling() {
  if (!isInitialized()) {
    return;
  }

  tFontSettings iFontsettings = CalculateFontSettings(0);

  ScrollingPages.Font    = iFontsettings.fontData;
  ScrollingPages.ypos[0] = iFontsettings.Top + GetHeaderHeight() + TopLineOffset;
  ScrollingPages.ypos[1] = ScrollingPages.ypos[0] + iFontsettings.Height + iFontsettings.Space;
  ScrollingPages.ypos[2] = ScrollingPages.ypos[1] + iFontsettings.Height + iFontsettings.Space;
  ScrollingPages.ypos[3] = ScrollingPages.ypos[2] + iFontsettings.Height + iFontsettings.Space;

  ScrollingLines.Font  = ScrollingPages.Font;
  ScrollingLines.Space = iFontsettings.Height + iFontsettings.Space + 1;

  for (uint8_t i = 0; i < P36_MAX_LinesPerPage; i++) {
    ScrollingLines.Line[i].ypos = ScrollingPages.ypos[i];
  }
}

uint8_t P036_data_struct::display_scroll(ePageScrollSpeed lscrollspeed, int lTaskTimer)
{
  if (!isInitialized()) {
    return 0;
  }

  // LineOut[] contain the outgoing strings in this frame
  // LineIn[] contain the incoming strings in this frame

  int iPageScrollTime;
  int iCharToRemove;

  # ifdef PLUGIN_036_DEBUG
  String log;

  if (loglevelActiveFor(LOG_LEVEL_INFO) &&
      log.reserve(32)) {
    log += F("Start Scrolling: Speed: ");
    log += static_cast<int>(lscrollspeed);
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // PLUGIN_036_DEBUG

  display->setFont(ScrollingPages.Font);

  ScrollingLines.wait = 0;

  // calculate total page scrolling time
  if (lscrollspeed == ePageScrollSpeed::ePSS_Instant) {
    // no scrolling, just the handling time to build the new page
    iPageScrollTime = P36_PageScrollTick - P36_PageScrollTimer;
  } else {
    iPageScrollTime = (P36_MaxDisplayWidth / (P36_PageScrollPix * static_cast<int>(lscrollspeed))) * P36_PageScrollTick;
  }
  int iScrollTime = static_cast<float>(lTaskTimer * 1000 - iPageScrollTime - 2 * P36_WaitScrollLines * 100) / 100; // scrollTime in ms

  # ifdef PLUGIN_036_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log;
    log.reserve(32);
    log += F("PageScrollTime: ");
    log += iPageScrollTime;
    addLogMove(LOG_LEVEL_INFO, log);
  }
  # endif // PLUGIN_036_DEBUG

  uint16_t MaxPixWidthForPageScrolling = P36_MaxDisplayWidth;

  if (bLineScrollEnabled) {
    // Reduced scrolling width because line is displayed left or right aligned
    MaxPixWidthForPageScrolling -= getDisplaySizeSettings(disp_resolution).PixLeft;
  }

  for (uint8_t j = 0; j < ScrollingPages.linesPerFrame; j++) {
    // default no line scrolling and strings are centered
    ScrollingLines.Line[j].LastWidth = 0;
    ScrollingLines.Line[j].Width     = 0;

    // get last and new line width
    uint16_t PixLengthLineOut = display->getStringWidth(ScrollingPages.LineOut[j]);
    uint16_t PixLengthLineIn  = display->getStringWidth(ScrollingPages.LineIn[j]);

    if (PixLengthLineIn > 255) {
      // shorten string because OLED controller can not handle such long strings
      int   strlen         = ScrollingPages.LineIn[j].length();
      float fAvgPixPerChar = static_cast<float>(PixLengthLineIn) / strlen;
      iCharToRemove            = ceil((static_cast<float>(PixLengthLineIn - 255)) / fAvgPixPerChar);
      ScrollingPages.LineIn[j] = ScrollingPages.LineIn[j].substring(0, strlen - iCharToRemove);
      PixLengthLineIn          = display->getStringWidth(ScrollingPages.LineIn[j]);
    }

    if (PixLengthLineOut > 255) {
      // shorten string because OLED controller can not handle such long strings
      int   strlen         = ScrollingPages.LineOut[j].length();
      float fAvgPixPerChar = static_cast<float>(PixLengthLineOut) / strlen;
      iCharToRemove             = ceil((static_cast<float>(PixLengthLineOut - 255)) / fAvgPixPerChar);
      ScrollingPages.LineOut[j] = ScrollingPages.LineOut[j].substring(0, strlen - iCharToRemove);
      PixLengthLineOut          = display->getStringWidth(ScrollingPages.LineOut[j]);
    }

    if (bLineScrollEnabled) {
      // settings for following line scrolling
      if (PixLengthLineOut > getDisplaySizeSettings(disp_resolution).Width) {
        ScrollingLines.Line[j].LastWidth = PixLengthLineOut; // while page scrolling this line is right aligned
      }

      if ((PixLengthLineIn > getDisplaySizeSettings(disp_resolution).Width) &&
          (iScrollTime > 0)) {
        // width of the line > display width -> scroll line
        ScrollingLines.Line[j].LineContent = ScrollingPages.LineIn[j];
        ScrollingLines.Line[j].Width       = PixLengthLineIn; // while page scrolling this line is left aligned
        ScrollingLines.Line[j].CurrentLeft = getDisplaySizeSettings(disp_resolution).PixLeft;
        ScrollingLines.Line[j].fPixSum     = getDisplaySizeSettings(disp_resolution).PixLeft;

        // pix change per scrolling line tick
        ScrollingLines.Line[j].dPix = (static_cast<float>(PixLengthLineIn - getDisplaySizeSettings(disp_resolution).Width)) / iScrollTime;

        # ifdef PLUGIN_036_DEBUG

        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log;
          log.reserve(32);
          log += F("Line: ");
          log += (j + 1);
          log += F(" width: ");
          log += ScrollingLines.Line[j].Width;
          log += F(" dPix: ");
          log += ScrollingLines.Line[j].dPix;
          addLogMove(LOG_LEVEL_INFO, log);
        }
        # endif // PLUGIN_036_DEBUG
      }
    }

    // reduce line content for page scrolling to max width
    if (PixLengthLineIn > MaxPixWidthForPageScrolling) {
      int strlen = ScrollingPages.LineIn[j].length();
      # ifdef PLUGIN_036_DEBUG
      String LineInStr = ScrollingPages.LineIn[j];
      # endif // PLUGIN_036_DEBUG
      float fAvgPixPerChar = static_cast<float>(PixLengthLineIn) / strlen;

      if (bLineScrollEnabled) {
        // shorten string on right side because line is displayed left aligned while scrolling
        // using floor() because otherwise empty space on right side
        iCharToRemove            = floor((static_cast<float>(PixLengthLineIn - MaxPixWidthForPageScrolling)) / fAvgPixPerChar);
        ScrollingPages.LineIn[j] = ScrollingPages.LineIn[j].substring(0, strlen - iCharToRemove);
      } else {
        // shorten string on both sides because line is displayed centered
        // using floor() because otherwise empty space on both sides
        iCharToRemove            = floor((static_cast<float>(PixLengthLineIn - MaxPixWidthForPageScrolling)) / (2 * fAvgPixPerChar));
        ScrollingPages.LineIn[j] = ScrollingPages.LineIn[j].substring(0, strlen - iCharToRemove);
        ScrollingPages.LineIn[j] = ScrollingPages.LineIn[j].substring(iCharToRemove);
      }
      # ifdef PLUGIN_036_DEBUG
      String log;

      if (loglevelActiveFor(LOG_LEVEL_INFO) &&
          log.reserve(128)) {
        log += F("Line: "); log += (j + 1);
        log += F(" LineIn: "); log += LineInStr;
        log += F(" Length: "); log += strlen;
        log += F(" PixLength: "); log += PixLengthLineIn;
        log += F(" AvgPixPerChar: "); log += fAvgPixPerChar;
        log += F(" CharsRemoved: "); log += iCharToRemove;
        addLog(LOG_LEVEL_INFO, log);
        log.clear();
        log += F(" -> Changed to: "); log += ScrollingPages.LineIn[j];
        log += F(" Length: "); log += ScrollingPages.LineIn[j].length();
        log += F(" PixLength: "); log += display->getStringWidth(ScrollingPages.LineIn[j]);
        addLogMove(LOG_LEVEL_INFO, log);
      }
      # endif // PLUGIN_036_DEBUG
    }

    // reduce line content for page scrolling to max width
    if (PixLengthLineOut > MaxPixWidthForPageScrolling) {
      int strlen = ScrollingPages.LineOut[j].length();
      # ifdef PLUGIN_036_DEBUG
      String LineOutStr = ScrollingPages.LineOut[j];
      # endif // PLUGIN_036_DEBUG
      float fAvgPixPerChar = static_cast<float>(PixLengthLineOut) / strlen;

      if (bLineScrollEnabled) {
        // shorten string on left side because line is displayed right aligned while scrolling
        // using ceil() because otherwise overlapping the new text
        iCharToRemove             = ceil((static_cast<float>(PixLengthLineOut - MaxPixWidthForPageScrolling)) / fAvgPixPerChar);
        ScrollingPages.LineOut[j] = ScrollingPages.LineOut[j].substring(iCharToRemove);

        if (display->getStringWidth(ScrollingPages.LineOut[j]) > MaxPixWidthForPageScrolling) {
          // remove one more character because still overlapping the new text
          ScrollingPages.LineOut[j] = ScrollingPages.LineOut[j].substring(1, iCharToRemove - 1);
        }
      } else {
        // shorten string on both sides because line is displayed centered
        // using ceil() because otherwise overlapping the new text
        iCharToRemove             = ceil((static_cast<float>(PixLengthLineOut - MaxPixWidthForPageScrolling)) / (2 * fAvgPixPerChar));
        ScrollingPages.LineOut[j] = ScrollingPages.LineOut[j].substring(0, strlen - iCharToRemove);
        ScrollingPages.LineOut[j] = ScrollingPages.LineOut[j].substring(iCharToRemove);
      }
      # ifdef PLUGIN_036_DEBUG
      String log;

      if (loglevelActiveFor(LOG_LEVEL_INFO) &&
          log.reserve(128)) {
        log += F("Line: "); log += (j + 1);
        log += F(" LineOut: "); log += LineOutStr;
        log += F(" Length: "); log += strlen;
        log += F(" PixLength: "); log += PixLengthLineOut;
        log += F(" AvgPixPerChar: "); log += fAvgPixPerChar;
        log += F(" CharsRemoved: "); log += iCharToRemove;
        addLog(LOG_LEVEL_INFO, log);
        log.clear();
        log += F(" -> Changed to: "); log += ScrollingPages.LineOut[j];
        log += F(" Length: "); log += ScrollingPages.LineOut[j].length();
        log += F(" PixLength: "); log += display->getStringWidth(ScrollingPages.LineOut[j]);
        addLogMove(LOG_LEVEL_INFO, log);
      }
      # endif // PLUGIN_036_DEBUG
    }
  }

  ScrollingPages.dPix    = P36_PageScrollPix * static_cast<int>(lscrollspeed); // pix change per scrolling page tick
  ScrollingPages.dPixSum = ScrollingPages.dPix;

  display_scroll_timer(true, lscrollspeed);                                    // Initial display of the page

  # ifdef PLUGIN_036_DEBUG
  addLog(LOG_LEVEL_INFO, F("Scrolling finished"));
  # endif // PLUGIN_036_DEBUG
  return ScrollingPages.Scrolling;
}

uint8_t P036_data_struct::display_scroll_timer(bool             initialScroll,
                                               ePageScrollSpeed lscrollspeed) {
  if (!isInitialized()) {
    return 0;
  }

  // page scrolling (using PLUGIN_TIMER_IN)
  display->setColor(BLACK);

  // We allow 12 pixels (including underline) at the top because otherwise the wifi indicator gets too squashed!!
  // scrolling window is 44 pixels high - ie 64 less margin of 12 at top and 8 at bottom
  display->fillRect(0, GetHeaderHeight() + (initialScroll ? 0 : 1) + TopLineOffset, P36_MaxDisplayWidth,
                    GetIndicatorTop() - GetHeaderHeight());
  display->setColor(WHITE);

  if (initialScroll) {
    if (!bHideHeader) {
      display->drawLine(0,
                        GetHeaderHeight() + TopLineOffset,
                        P36_MaxDisplayWidth,
                        GetHeaderHeight() + TopLineOffset); // line below title
    }
  } else {
    display->setFont(ScrollingPages.Font);
  }

  for (uint8_t j = 0; j < ScrollingPages.linesPerFrame; j++) {
    if ((initialScroll && (lscrollspeed < ePageScrollSpeed::ePSS_Instant)) ||
        !initialScroll) { // scrolling
      if (ScrollingLines.Line[j].LastWidth > 0) {
        // width of LineOut[j] > display width -> line is right aligned while scrolling page
        display->setTextAlignment(TEXT_ALIGN_RIGHT);
        display->drawString(P36_MaxDisplayWidth - getDisplaySizeSettings(disp_resolution).PixLeft + ScrollingPages.dPixSum,
                            ScrollingPages.ypos[j],
                            ScrollingPages.LineOut[j]);
      } else {
        // line is centered while scrolling page
        display->setTextAlignment(textAlignment);
        display->drawString(textLeftMargin + ScrollingPages.dPixSum,
                            ScrollingPages.ypos[j],
                            ScrollingPages.LineOut[j]);
      }
    }

    if (ScrollingLines.Line[j].Width > 0) {
      // width of LineIn[j] > display width -> line is left aligned while scrolling page
      display->setTextAlignment(TEXT_ALIGN_LEFT);
      display->drawString(-P36_MaxDisplayWidth + getDisplaySizeSettings(disp_resolution).PixLeft + ScrollingPages.dPixSum,
                          ScrollingPages.ypos[j],
                          ScrollingPages.LineIn[j]);
    } else {
      // line is centered while scrolling page
      display->setTextAlignment(textAlignment);
      display->drawString((-1 * getDisplaySizeSettings(disp_resolution).Width) + textLeftMargin + ScrollingPages.dPixSum,
                          ScrollingPages.ypos[j],
                          ScrollingPages.LineIn[j]);
    }
  }

  update_display();

  if ((initialScroll && (lscrollspeed < ePageScrollSpeed::ePSS_Instant)) ||
      (!initialScroll && (ScrollingPages.dPixSum < P36_MaxDisplayWidth))) { // scrolling
    // page still scrolling
    ScrollingPages.dPixSum += ScrollingPages.dPix;
  } else {
    // page scrolling finished
    ScrollingPages.Scrolling = 0; // allow following line scrolling
  }
  return ScrollingPages.Scrolling;
}

// Draw scrolling line (1pix/s)
void P036_data_struct::display_scrolling_lines() {
  if (!isInitialized()) {
    return;
  }

  // line scrolling (using PLUGIN_TEN_PER_SECOND)

  int  i;
  bool bscroll       = false;
  bool updateDisplay = false;
  int  iCurrentLeft;

  for (i = 0; i < ScrollingPages.linesPerFrame; i++) {
    if (ScrollingLines.Line[i].Width != 0) {
      display->setFont(ScrollingLines.Font);
      bscroll = true;
      break;
    }
  }

  if (bscroll) {
    ScrollingLines.wait++;

    if (ScrollingLines.wait < P36_WaitScrollLines) {
      return; // wait before scrolling line not finished
    }

    for (i = 0; i < ScrollingPages.linesPerFrame; i++) {
      if (ScrollingLines.Line[i].Width != 0) {
        // scroll this line
        ScrollingLines.Line[i].fPixSum -= ScrollingLines.Line[i].dPix;
        iCurrentLeft                    = round(ScrollingLines.Line[i].fPixSum);

        if (iCurrentLeft != ScrollingLines.Line[i].CurrentLeft) {
          // still scrolling
          ScrollingLines.Line[i].CurrentLeft = iCurrentLeft;
          updateDisplay                      = true;
          display->setColor(BLACK);
          display->fillRect(0, ScrollingLines.Line[i].ypos + 1, P36_MaxDisplayWidth,
                            ScrollingLines.Space + 1); // clearing window was too high
          display->setColor(WHITE);

          if (((ScrollingLines.Line[i].CurrentLeft - getDisplaySizeSettings(disp_resolution).PixLeft) +
               ScrollingLines.Line[i].Width) >= getDisplaySizeSettings(disp_resolution).Width) {
            display->setTextAlignment(TEXT_ALIGN_LEFT);
            display->drawString(ScrollingLines.Line[i].CurrentLeft,
                                ScrollingLines.Line[i].ypos,
                                ScrollingLines.Line[i].LineContent);
          } else {
            // line scrolling finished -> line is shown as aligned right
            display->setTextAlignment(TEXT_ALIGN_RIGHT);
            display->drawString(P36_MaxDisplayWidth - getDisplaySizeSettings(disp_resolution).PixLeft,
                                ScrollingPages.ypos[i],
                                ScrollingLines.Line[i].LineContent);
            ScrollingLines.Line[i].Width = 0; // Stop scrolling
          }
        }
      }
    }

    if (updateDisplay && (ScrollingPages.Scrolling == 0)) {
      update_display();
    }
  }
}

// Draw Signal Strength Bars, return true when there was an update.
bool P036_data_struct::display_wifibars() {
  if (!isInitialized()) {
    return false;
  }

  if (bHideHeader) { //  hide header
    return false;
  }

  const bool connected    = NetworkConnected();
  const int  nbars_filled = (WiFi.RSSI() + 100) / 12; // all bars filled if RSSI better than -46dB
  const int  newState     = connected ? nbars_filled : P36_WIFI_STATE_NOT_CONNECTED;

  if (newState == lastWiFiState) {
    return false; // nothing to do.
  }
  const int x         = getDisplaySizeSettings(disp_resolution).WiFiIndicatorLeft;
  const int y         = TopLineOffset;
  int size_x          = getDisplaySizeSettings(disp_resolution).WiFiIndicatorWidth;
  int size_y          = GetHeaderHeight() - 2;
  const int nbars     = 5;
  const int16_t width = (size_x / nbars);

  size_x = width * nbars - 1; // Correct for round errors.

  //  x,y are the x,y locations
  //  sizex,sizey are the sizes (should be a multiple of the number of bars)
  //  nbars is the number of bars and nbars_filled is the number of filled bars.

  //  We leave a 1 pixel gap between bars
  display->setColor(BLACK);
  display->fillRect(x, y, size_x, size_y);
  display->setColor(WHITE);

  if (NetworkConnected()) {
    for (uint8_t ibar = 0; ibar < nbars; ibar++) {
      int16_t height = size_y * (ibar + 1) / nbars;
      int16_t xpos   = x + ibar * width;
      int16_t ypos   = y + size_y - height;

      if (ibar <= nbars_filled) {
        // Fill complete bar
        display->fillRect(xpos, ypos, width - 1, height);
      } else {
        // Only draw top and bottom.
        display->fillRect(xpos, ypos,           width - 1, 1);
        display->fillRect(xpos, y + size_y - 1, width - 1, 1);
      }
    }
  } else {
    // Draw a not connected sign (empty rectangle)
    display->drawRect(x, y, size_x, size_y - 1);
  }
  return true;
}

void P036_data_struct::update_display()
{
  if (isInitialized()) {
    display->display();
  }
}

void P036_data_struct::P036_JumpToPage(struct EventStruct *event, uint8_t nextFrame)
{
  if (!isInitialized()) {
    return;
  }
  Scheduler.schedule_task_device_timer(event->TaskIndex,
                                       millis() + (Settings.TaskDeviceTimer[event->TaskIndex] * 1000)); // reschedule page change
  nextFrameToDisplay    = nextFrame;
  bPageScrollDisabled   = true;                                                                         //  show next page without scrolling
  disableFrameChangeCnt = 2;                                                                            //  disable next page change in
                                                                                                        // PLUGIN_READ if
  // PLUGIN_READ was already scheduled
  P036_DisplayPage(event);                                                                              //  Display the selected page,
                                                                                                        // function needs
                                                                                                        // 65ms!
  displayTimer = PCONFIG(4);                                                                            //  Restart timer
}

void P036_data_struct::P036_DisplayPage(struct EventStruct *event)
{
  # ifdef PLUGIN_036_DEBUG
  addLog(LOG_LEVEL_INFO, F("P036_DisplayPage"));
  # endif // PLUGIN_036_DEBUG

  if (!isInitialized()) {
    return;
  }

  uint8_t NFrames; // the number of frames
  uint8_t prevFramesToDisplay = MaxFramesToDisplay;

  if (essentiallyEqual(UserVar[event->BaseVarIndex], 1.0f)) {
    // Display is on.
    ScrollingPages.Scrolling = 1;                                                              // page scrolling running -> no
    // line scrolling allowed
    NFrames                  = P36_Nlines / ScrollingPages.linesPerFrame;
    HeaderContent            = static_cast<eHeaderContent>(get8BitFromUL(PCONFIG_LONG(0), 8)); // Bit15-8 HeaderContent
    HeaderContentAlternative = static_cast<eHeaderContent>(get8BitFromUL(PCONFIG_LONG(0), 0)); // Bit 7-0
    // HeaderContentAlternative

    //      Construct the outgoing string
    for (uint8_t i = 0; i < ScrollingPages.linesPerFrame; i++)
    {
      String tmpString(DisplayLinesV1[(ScrollingPages.linesPerFrame * frameCounter) + i].Content);
      ScrollingPages.LineOut[i] = P36_parseTemplate(tmpString, 20);
    }

    // now loop round looking for the next frame with some content
    //   skip this frame if all lines in frame are blank
    // - we exit the while loop if any line is not empty
    bool foundText = false;
    int  ntries    = 0;

    while (!foundText) {
      //        Stop after framecount loops if no data found
      ntries += 1;

      if (ntries > NFrames) { break; }

      if (nextFrameToDisplay == 0xff) {
        // Increment the frame counter
        frameCounter++;

        if (frameCounter > NFrames - 1) {
          frameCounter          = 0;
          currentFrameToDisplay = 0;
        }
      }
      else {
        // next frame because content changed in PLUGIN_WRITE
        frameCounter = nextFrameToDisplay;
      }

      //        Contruct incoming strings
      for (uint8_t i = 0; i < ScrollingPages.linesPerFrame; i++)
      {
        String tmpString(DisplayLinesV1[(ScrollingPages.linesPerFrame * frameCounter) + i].Content);
        ScrollingPages.LineIn[i] = P36_parseTemplate(tmpString, 20);

        if (ScrollingPages.LineIn[i].length() > 0) { foundText = true; }
      }

      if (foundText) {
        if (nextFrameToDisplay == 0xff) {
          if (frameCounter != 0) {
            ++currentFrameToDisplay;
          }
        } else {
          currentFrameToDisplay = nextFrameToDisplay;
        }
      }
    }
    nextFrameToDisplay = 0xFF;

    // Update max page count
    if (MaxFramesToDisplay == 0xFF) {
      // not updated yet
      for (uint8_t i = 0; i < NFrames; i++) {
        for (uint8_t k = 0; k < ScrollingPages.linesPerFrame; k++) {
          String tmpString(DisplayLinesV1[(ScrollingPages.linesPerFrame * i) + k].Content);
          tmpString = P36_parseTemplate(tmpString, 20);

          if (tmpString.length() > 0) {
            // page not empty
            if (MaxFramesToDisplay == 0xFF) {
              MaxFramesToDisplay = 0;
            } else {
              MaxFramesToDisplay++;
            }
            break;
          }
        }
      }

      if (MaxFramesToDisplay == 0xFF) {
        // nothing to display
        MaxFramesToDisplay = 0;
      }
    }

    // Turn on/off the Indicator if the number of frames changes
    if (MaxFramesToDisplay != prevFramesToDisplay) {
      bHideFooter = bitRead(P036_FLAGS_0, P036_FLAG_HIDE_FOOTER);
      CalculateFontSettings(0); // Re-calculate fontsize
    }

    //      Update display
    if (bDisplayingLogo) {
      bDisplayingLogo = false;
      display->clear();        // resets all pixels to black
    }

    bAlternativHeader = false; // start with first header content
    HeaderCount       = 0;     // reset header count
    display_header();

    display_indicator();

    update_display();

    bool bScrollWithoutWifi = bitRead(PCONFIG_LONG(0), 24);                            // Bit 24
    bool bScrollLines       = bitRead(PCONFIG_LONG(0), 17);                            // Bit 17
    bLineScrollEnabled = (bScrollLines && (NetworkConnected() || bScrollWithoutWifi)); // scroll lines only if WifiIsConnected,
    // otherwise too slow

    ePageScrollSpeed lscrollspeed = static_cast<ePageScrollSpeed>(PCONFIG(3));

    if (bPageScrollDisabled) { lscrollspeed = ePageScrollSpeed::ePSS_Instant; } // first page after INIT without scrolling

    int lTaskTimer = Settings.TaskDeviceTimer[event->TaskIndex];

    if (display_scroll(lscrollspeed, lTaskTimer)) {
      Scheduler.setPluginTaskTimer(P36_PageScrollTimer, event->TaskIndex, event->Par1); // calls next page scrollng tick
    }

    if (NetworkConnected() || bScrollWithoutWifi) {
      // scroll lines only if WifiIsConnected, otherwise too slow
      bPageScrollDisabled = false; // next PLUGIN_READ will do page scrolling
    }
  } else {
    # ifdef PLUGIN_036_DEBUG
    addLog(LOG_LEVEL_INFO, F("P036_DisplayPage Display off"));
    # endif // PLUGIN_036_DEBUG
  }
}

// Perform some specific changes for OLED display
String P036_data_struct::P36_parseTemplate(String& tmpString, uint8_t lineSize) {
  String result = parseTemplate_padded(tmpString, lineSize);

  // OLED lib uses this routine to convert UTF8 to extended ASCII
  // http://playground.arduino.cc/Main/Utf8ascii
  // Attempt to display euro sign (FIXME)

  /*
     const char euro[4] = {0xe2, 0x82, 0xac, 0}; // Unicode euro symbol
     const char euro_oled[3] = {0xc2, 0x80, 0}; // Euro symbol OLED display font
     result.replace(euro, euro_oled);
   */
  if (textAlignment == TEXT_ALIGN_LEFT) {
    // result.rtrim();
    for (int16_t l = result.length() - 1; l >= 0; l--) {
      if (result[l] != ' ') {
        break;
      }
      result.remove(l);
    }
  } else {
    result.trim();
  }
  return result;
}

# ifdef P036_ENABLE_LEFT_ALIGN
void P036_data_struct::setTextAlignment(OLEDDISPLAY_TEXT_ALIGNMENT _textAlignment) {
  textAlignment = _textAlignment;

  if (_textAlignment == TEXT_ALIGN_LEFT) {
    textLeftMargin = 0;
  } else {
    textLeftMargin = getDisplaySizeSettings(disp_resolution).Width / 2;
  }

  MaxFramesToDisplay = 0xFF; // Recalculate page indicator
  nextFrameToDisplay = 0;    // Reset to first page
}

# endif // ifdef P036_ENABLE_LEFT_ALIGN

void P036_data_struct::registerButtonState(uint8_t newButtonState, bool bPin3Invers) {
  if ((ButtonLastState == 0xFF) || (bPin3Invers != (!!newButtonState))) {
    ButtonLastState = newButtonState;
    DebounceCounter++;

    if (RepeatCounter > 0) {
      RepeatCounter--;      // decrease the repeat count
    }
  } else {
    ButtonLastState = 0xFF; // Reset
    DebounceCounter = 0;
    RepeatCounter   = 0;
    ButtonState     = false;
  }

  if ((ButtonLastState == newButtonState) &&
      (DebounceCounter >= P36_DebounceTreshold) &&
      (RepeatCounter == 0)) {
    ButtonState = true;
  }
}

void P036_data_struct::markButtonStateProcessed() {
  ButtonState     = false;
  DebounceCounter = 0;
  RepeatCounter   = P36_RepeatDelay; //  Wait a bit before repeating the button action
}

#endif // ifdef USES_P036

#include "../PluginStructs/P037_data_struct.h"

#ifdef USES_P037

# include "../Helpers/ESPEasy_Storage.h"
# include "../Helpers/Numerical.h"
# include "../Helpers/RulesMatcher.h"
# include "../WebServer/Markup_Forms.h"
# include "../WebServer/WebServer.h"
# include "../WebServer/Markup.h"
# include "../WebServer/HTML_wrappers.h"
# include "../ESPEasyCore/ESPEasyRules.h"


P037_data_struct::P037_data_struct(taskIndex_t taskIndex) : _taskIndex(taskIndex) {
  loadSettings();
}

P037_data_struct::~P037_data_struct() {}

/**
 * Load the settings from file
 */
bool P037_data_struct::loadSettings() {
  if (_taskIndex < TASKS_MAX) {
    size_t offset = 0;
    LoadCustomTaskSettings(_taskIndex, mqttTopics,
                           VARS_PER_TASK, 41, offset);
    offset += VARS_PER_TASK * 41;

    LoadCustomTaskSettings(_taskIndex, jsonAttributes,
                           VARS_PER_TASK, 21, offset);
    offset += VARS_PER_TASK * 21;

    {
      String tmp[1];

      LoadCustomTaskSettings(_taskIndex, tmp,
                             1, 41, offset);
      globalTopicPrefix = std::move(tmp[0]);
      offset           += 41;
    }


    LoadCustomTaskSettings(_taskIndex, valueArray,
                           P037_ARRAY_SIZE, 0, offset + 1);
    return true;
  }
  return false;
}

String P037_data_struct::saveSettings() {
  String res;

  if (_taskIndex < TASKS_MAX) {
    size_t offset = 0;
    res += SaveCustomTaskSettings(_taskIndex, mqttTopics,
                                  VARS_PER_TASK, 41, offset);
    offset += VARS_PER_TASK * 41;

    res += SaveCustomTaskSettings(_taskIndex, jsonAttributes,
                                  VARS_PER_TASK, 21, offset);
    offset += VARS_PER_TASK * 21;

    {
      String tmp[1];
      tmp[0] = globalTopicPrefix;
      res   += SaveCustomTaskSettings(_taskIndex, tmp,
                                      1, 41, offset);
      offset += 41;
    }


    res += SaveCustomTaskSettings(_taskIndex, valueArray,
                                  P037_ARRAY_SIZE, 0, offset + 1);
  }
  return res;
}

String P037_data_struct::getFullMQTTTopic(uint8_t taskValueIndex) const {
  String topic;

  if ((taskValueIndex < VARS_PER_TASK) && (mqttTopics[taskValueIndex].length() > 0)) {
    topic.reserve(globalTopicPrefix.length() + mqttTopics[taskValueIndex].length());
    topic = globalTopicPrefix;
    topic.trim();
    topic += mqttTopics[taskValueIndex];
    topic.trim();
  }
  return topic;
}

bool P037_data_struct::shouldSubscribeToMQTTtopic(const String& topic) const {
  if (topic.length() == 0) { return false; }

  for (uint8_t x = 0; x < VARS_PER_TASK; x++)
  {
    if (topic.equalsIgnoreCase(getFullMQTTTopic(x))) {
      return true;
    }
  }
  return false;
}

# if P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT

/**
 * Parse the mappings and filters from the settings-string into arrays
 */
void P037_data_struct::parseMappings() {
  if (
    #  if P037_MAPPING_SUPPORT
    _maxIdx == -1
    #  endif // if P037_MAPPING_SUPPORT
    #  if P037_MAPPING_SUPPORT && P037_FILTER_SUPPORT
    ||
    #  endif // if P037_MAPPING_SUPPORT && P037_FILTER_SUPPORT
    #  if P037_FILTER_SUPPORT
    _maxFilter == -1
    #  endif // if P037_FILTER_SUPPORT
    ) {
    #  if P037_MAPPING_SUPPORT
    _maxIdx = 0;    // Initialize to empty
    #  endif // if P037_MAPPING_SUPPORT
    #  if P037_FILTER_SUPPORT
    _maxFilter = 0; // Initialize to empty
    #  endif // if P037_FILTER_SUPPORT

    #  if P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT
    int8_t idx;
    #  endif // if P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT
    #  if P037_MAPPING_SUPPORT
    idx = P037_MAX_MAPPINGS;

    for (uint8_t mappingOffset = P037_END_MAPPINGS; mappingOffset >= P037_START_MAPPINGS && _maxIdx == 0; mappingOffset--) {
      if (!valueArray[mappingOffset].isEmpty()) {
        _maxIdx = idx;
      }
      idx--;
    }
    #  endif // if P037_MAPPING_SUPPORT

    #  if P037_FILTER_SUPPORT
    idx = P037_MAX_FILTERS;

    for (uint8_t filterOffset = P037_END_FILTERS; filterOffset >= P037_START_FILTERS && _maxFilter == 0; filterOffset--) {
      if (!valueArray[filterOffset].isEmpty()) {
        _maxFilter = idx;
      }
      idx--;
    }
    #   ifdef P037_FILTER_PER_TOPIC

    if (_maxFilter > 0) { // For Filter-per-topic: Only activate filtering if at least 1 filter is defined
      _maxFilter = VARS_PER_TASK;
    }
    #   endif // ifdef P037_FILTER_PER_TOPIC
    #  endif // if P037_FILTER_SUPPORT
  }
} // parseMappings

# endif // if P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT

bool P037_data_struct::webform_load(
  # if P037_MAPPING_SUPPORT
  bool mappingEnabled
  # endif // if P037_MAPPING_SUPPORT
  # if P037_MAPPING_SUPPORT&& P037_FILTER_SUPPORT
  ,
  # endif // if P037_MAPPING_SUPPORT && P037_FILTER_SUPPORT
  # if P037_FILTER_SUPPORT
  bool filterEnabled
  # endif // if P037_FILTER_SUPPORT
  # if (P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT)&& defined(P037_JSON_SUPPORT)
  ,
  # endif // if (P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT)&& defined(P037_JSON_SUPPORT)
  # ifdef P037_JSON_SUPPORT
  bool jsonEnabled
  # endif // ifdef P037_JSON_SUPPORT
  ) {
  bool success = false;

  addFormSubHeader(F("Topic subscriptions"));

  // Global topic prefix
  addFormTextBox(F("Prefix for all topics"), F("topicprefix"), globalTopicPrefix, 40);

  # ifdef P037_JSON_SUPPORT

  if (jsonEnabled) {
    addRowLabel(F("MQTT Topic"));
    html_table(EMPTY_STRING, false); // Sub-table
    html_table_header(F("&nbsp;#&nbsp;"));
    html_table_header(F("Topic"),          500);
    html_table_header(F("JSON Attribute"), 200);
  }
  # endif // ifdef P037_JSON_SUPPORT

  for (uint8_t varNr = 0; varNr < VARS_PER_TASK; varNr++)
  {
    String id;
    # ifdef P037_JSON_SUPPORT

    if (jsonEnabled) { // Add a column with the json attribute to use for value
      html_TR_TD();
      addHtml(F("&nbsp;"));
      addHtmlInt(varNr + 1);
      html_TD();
      id  = F("template");
      id += (varNr + 1);
      addTextBox(id,
                 mqttTopics[varNr],
                 40,
                 false, false, EMPTY_STRING, F("wide"));
      html_TD();
      id  = F("attribute");
      id += (varNr + 1);
      addTextBox(id,
                 jsonAttributes[varNr],
                 20,
                 false, false, EMPTY_STRING, EMPTY_STRING);
      html_TD();
    } else
    # endif // ifdef P037_JSON_SUPPORT
    {
      String label = F("MQTT Topic ");
      label += (varNr + 1);
      id     = F("template");
      id    += (varNr + 1);
      addFormTextBox(label, id, mqttTopics[varNr], 40);
    }
  }
  # ifdef P037_JSON_SUPPORT

  if (jsonEnabled) {
    html_end_table();
  }
  # endif // ifdef P037_JSON_SUPPORT

  # if P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT
  parseMappings();
  # endif // if P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT

  # if P037_FILTER_SUPPORT

  if (filterEnabled) {
    addFormSubHeader(F("Name - value filters"));

    #  ifdef P037_FILTER_PER_TOPIC
    addRowLabel(F("Filter for MQTT Topic"));
    #  else // ifdef P037_FILTER_PER_TOPIC
    addRowLabel(F("Filter"));
    #  endif // ifdef P037_FILTER_PER_TOPIC
    html_table(F(""), false); // Sub-table
    html_table_header(F("&nbsp;#&nbsp;"));
    html_table_header(F("Name[;Index]"));
    html_table_header(F("Operand"), 180);
    html_table_header(F("Value"));

    const __FlashStringHelper *filterOptions[] = {
      F("equals"), // map name to value
      F("range")   // between 2 values
      #  if P037_FILTER_COUNT >= 3
      , F("list")  // list of multiple values
      #  endif // if P037_FILTER_COUNT >= 3
    };
    int filterIndices[] = { 0, 1
      #  if P037_FILTER_COUNT >= 3
                            , 2
      #  endif // if P037_FILTER_COUNT >= 3
    };

    String filters = P037_FILTER_LIST; // Anticipate more filters
    int8_t filterIndex;

    int8_t idx      = 0;
    int8_t filterNr = 1;
    #  ifdef P037_FILTER_PER_TOPIC

    if (_maxFilter <= 0) { _maxFilter = (VARS_PER_TASK * 3); }
    #  endif // ifdef P037_FILTER_PER_TOPIC

    for (uint8_t filterOffset = P037_START_FILTERS; filterOffset <= P037_END_FILTERS && idx < (_maxFilter * 3); filterOffset++) {
      {
        html_TR_TD();
        addHtml(F("&nbsp;"));
        addHtmlInt(filterNr);
        html_TD();
        addTextBox(getPluginCustomArgName(idx + 100 + 0),
                   parseStringKeepCase(valueArray[filterOffset], 1, P037_VALUE_SEPARATOR),
                   32, false, false, EMPTY_STRING, EMPTY_STRING);
      }
      {
        html_TD();
        filterIndex = filters.indexOf(parseString(valueArray[filterOffset], 2, P037_VALUE_SEPARATOR));
        addSelector(getPluginCustomArgName(idx + 100 + 1), P037_FILTER_COUNT, filterOptions, filterIndices, NULL, filterIndex);
        html_TD();

        addTextBox(getPluginCustomArgName(idx + 100 + 2), parseStringKeepCase(valueArray[filterOffset], 3, P037_VALUE_SEPARATOR),
                   32, false, false, EMPTY_STRING, EMPTY_STRING);
        addUnit(F("Range/List: separate values with ; "));
        html_TD();
      }

      filterNr++;
      idx += 3;
    }
    #  ifdef PLUGIN_037_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String info;
      info.reserve(25);
      info += F("P037 maxFilter: ");
      info += _maxFilter;
      info += F(" idx: ");
      info += idx;
      addLogMove(LOG_LEVEL_INFO, info);
    }
    #  endif // ifdef PLUGIN_037_DEBUG
    #  ifndef P037_FILTER_PER_TOPIC
    filterIndex = 0;
    uint8_t extraFilters = 0;

    while (extraFilters < P037_EXTRA_VALUES && idx < P037_MAX_FILTERS * 3) {
      {
        html_TR_TD();
        addHtml(F("&nbsp;"));
        addHtmlInt(filterNr);
        html_TD();
        addTextBox(getPluginCustomArgName(idx + 100 + 0), EMPTY_STRING,
                   32, false, false, EMPTY_STRING, EMPTY_STRING);
      }
      {
        html_TD();
        addSelector(getPluginCustomArgName(idx + 100 + 1), P037_FILTER_COUNT, filterOptions, filterIndices, NULL, filterIndex);
        html_TD();
        addTextBox(getPluginCustomArgName(idx + 100 + 2), EMPTY_STRING,
                   32, false, false, EMPTY_STRING, EMPTY_STRING);
        addUnit(F("Range/List: separate values with ; "));
        html_TD();
      }
      idx += 3;
      extraFilters++;
      filterNr++;
    }
    #  endif // ifndef P037_FILTER_PER_TOPIC
    html_end_table();
    #  ifndef P037_FILTER_PER_TOPIC
    #   ifdef PLUGIN_037_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      info  = F("P037 extraFilters: ");
      info += extraFilters;
      info += F(" idx: ");
      info += idx;
      addLogMove(LOG_LEVEL_INFO, info);
    }
    #   endif // ifdef PLUGIN_037_DEBUG
    #  endif  // ifndef P037_FILTER_PER_TOPIC
    addFormNote(F("Both Name and Value must be filled for a valid filter. Filters are case-sensitive."));
    #  ifndef P037_FILTER_PER_TOPIC

    if (extraFilters == P037_EXTRA_VALUES) {
      String moreMessage = F("After filling all filters, submitting this page will make extra filters available (up to ");
      moreMessage += P037_MAX_FILTERS;
      moreMessage += F(").");
      addFormNote(moreMessage);
    }
    #  endif // ifndef P037_FILTER_PER_TOPIC
  }

  # endif    // if P037_FILTER_SUPPORT

  # if P037_MAPPING_SUPPORT

  if (mappingEnabled) {
    addFormSubHeader(F("Name - value mappings"));

    addRowLabel(F("Mapping"));
    html_table(F(""), false); // Sub-table
    html_table_header(F("&nbsp;#&nbsp;"));
    html_table_header(F("Name"));
    html_table_header(F("Operand"), 180);
    html_table_header(F("Value"));

    const __FlashStringHelper *operandOptions[] = {
      F("map"),                          // map name to int
      F("percentage") };                 // map attribute value to percentage of provided value
    int operandIndices[] = { 0, 1 };

    String operands = P037_OPERAND_LIST; // Anticipate more operations
    int8_t operandIndex;

    int8_t idx   = 0;
    int8_t mapNr = 1;

    for (uint8_t mappingOffset = P037_START_MAPPINGS; mappingOffset <= P037_END_MAPPINGS && idx < (_maxIdx * 3); mappingOffset++) {
      {
        html_TR_TD();
        addHtml(F("&nbsp;"));
        addHtmlInt(mapNr);
        html_TD();
        addTextBox(getPluginCustomArgName(idx + 0),
                   parseStringKeepCase(valueArray[mappingOffset], 1, P037_VALUE_SEPARATOR),
                   32, false, false, EMPTY_STRING, EMPTY_STRING);
      }
      {
        html_TD();
        operandIndex = operands.indexOf(parseString(valueArray[mappingOffset], 2, P037_VALUE_SEPARATOR));
        addSelector(getPluginCustomArgName(idx + 1), P037_OPERAND_COUNT, operandOptions, operandIndices, NULL, operandIndex);
        html_TD();
        addTextBox(getPluginCustomArgName(idx + 2),
                   parseStringKeepCase(valueArray[mappingOffset], 3, P037_VALUE_SEPARATOR),
                   32, false, false, EMPTY_STRING, EMPTY_STRING);
        html_TD();
      }
      mapNr++;
      idx += 3;
    }
    #  ifdef PLUGIN_037_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String info;
      info.reserve(25);
      info += F("P037 maxIdx: ");
      info += _maxIdx;
      info += F(" idx: ");
      info += idx;
      addLogMove(LOG_LEVEL_INFO, info);
    }
    #  endif // ifdef PLUGIN_037_DEBUG
    operandIndex = 0;
    uint8_t extraMappings = 0;

    while (extraMappings < P037_EXTRA_VALUES && idx < P037_MAX_MAPPINGS * 3) {
      {
        html_TR_TD();
        addHtml(F("&nbsp;"));
        addHtmlInt(mapNr);
        html_TD();
        addTextBox(getPluginCustomArgName(idx + 0), EMPTY_STRING,
                   32, false, false, EMPTY_STRING, EMPTY_STRING);
      }
      {
        html_TD();
        addSelector(getPluginCustomArgName(idx + 1), P037_OPERAND_COUNT, operandOptions, operandIndices, NULL, operandIndex);
        html_TD();
        addTextBox(getPluginCustomArgName(idx + 2), EMPTY_STRING,
                   32, false, false, EMPTY_STRING, EMPTY_STRING);
        html_TD();
      }
      idx += 3;
      extraMappings++;
      mapNr++;
    }
    html_end_table();
    #  ifdef PLUGIN_037_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String info;
      info += F("P037 extraMappings: ");
      info += extraMappings;
      info += F(" idx: ");
      info += idx;
      addLogMove(LOG_LEVEL_INFO, info);
    }
    #  endif // ifdef PLUGIN_037_DEBUG
    addFormNote(F("Both Name and Value must be filled for a valid mapping. Mappings are case-sensitive."));

    if (extraMappings == P037_EXTRA_VALUES) {
      String moreMessage = F("After filling all mappings, submitting this page will make extra mappings available (up to ");
      moreMessage += P037_MAX_MAPPINGS;
      moreMessage += F(").");
      addFormNote(moreMessage);
    }
  }
  # endif // if P037_MAPPING_SUPPORT

  success = true;
  return success;
} // webform_load

bool P037_data_struct::webform_save(
  # if P037_FILTER_SUPPORT
  bool filterEnabled
  # endif // if P037_FILTER_SUPPORT
  # if P037_FILTER_SUPPORT&& defined(P037_JSON_SUPPORT)
  ,
  # endif // if P037_FILTER_SUPPORT && defined(P037_JSON_SUPPORT)
  # ifdef P037_JSON_SUPPORT
  bool jsonEnabled
  # endif // ifdef P037_JSON_SUPPORT
  ) {
  bool success = false;

  String error;

  error.reserve(80); // Estimated

  for (uint8_t varNr = 0; varNr < VARS_PER_TASK; varNr++)
  {
    String argName = F("template");
    argName += (varNr + 1);

    mqttTopics[varNr] = web_server.arg(argName);

    # ifdef P037_JSON_SUPPORT

    if (jsonEnabled) {
      argName               = F("attribute");
      argName              += (varNr + 1);
      jsonAttributes[varNr] = web_server.arg(argName);
    }
    # endif // P037_JSON_SUPPORT
  }

  globalTopicPrefix = web_server.arg(F("topicprefix"));

  # if P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT
  String left, right;
  bool   firstError;
  int8_t idx = 0;
  # endif // if P037_MAPPING_SUPPORT || P037_FILTER_SUPPORT

  // Mappings are processed first
  # if P037_MAPPING_SUPPORT
  firstError = true;
  String  operands = P037_OPERAND_LIST;
  uint8_t mapNr    = 1;
  left.reserve(32);
  right.reserve(32);

  for (uint8_t mappingOffset = P037_START_MAPPINGS; mappingOffset <= P037_END_MAPPINGS; mappingOffset++) {
    left.clear();
    left +=  web_server.arg(getPluginCustomArgName(idx + 0));
    left.trim();
    right.clear();
    right += web_server.arg(getPluginCustomArgName(idx + 2));
    right.trim();

    if (!left.isEmpty() || !right.isEmpty()) {
      valueArray[mappingOffset]  = wrapWithQuotes(left);
      valueArray[mappingOffset] += P037_VALUE_SEPARATOR;
      uint8_t oper = getFormItemInt(getPluginCustomArgName(idx + 1));
      valueArray[mappingOffset] += operands.substring(oper, oper + 1);
      valueArray[mappingOffset] += P037_VALUE_SEPARATOR;
      valueArray[mappingOffset] += wrapWithQuotes(right);
    } else {
      valueArray[mappingOffset] = EMPTY_STRING;
    }

    if (left.isEmpty() != right.isEmpty()) {
      if (firstError) {
        error     += F("Name and value should both be filled for mapping ");
        firstError = false;
      } else {
        error += ',';
      }
      error += mapNr;
    }
    mapNr++;
    idx += 3;
    delay(0); // leave some yield
  }

  if (!firstError) {
    error += '\n';
  }
  # endif // if P037_MAPPING_SUPPORT

  # if P037_FILTER_SUPPORT
  String filters = P037_FILTER_LIST;
  firstError = true;
  uint8_t filterNr = 1;
  idx = 0;

  for (uint8_t filterOffset = P037_START_FILTERS; filterOffset <= P037_END_FILTERS; filterOffset++) {
    left =  web_server.arg(getPluginCustomArgName(idx + 100 + 0));
    left.trim();
    right = web_server.arg(getPluginCustomArgName(idx + 100 + 2));
    right.trim();

    if (!left.isEmpty() || !right.isEmpty()
        #  ifdef P037_FILTER_PER_TOPIC
        || true // Store all filters and in the same order, including empty filters
        #  endif // ifdef P037_FILTER_PER_TOPIC
        ) {
      valueArray[filterOffset]  = wrapWithQuotes(left);
      valueArray[filterOffset] += P037_VALUE_SEPARATOR;
      uint8_t oper = getFormItemInt(getPluginCustomArgName(idx + 100 + 1));
      valueArray[filterOffset] += filters.substring(oper, oper + 1);
      valueArray[filterOffset] += P037_VALUE_SEPARATOR;
      valueArray[filterOffset] += wrapWithQuotes(right);
    } else {
      valueArray[filterOffset] = EMPTY_STRING;
    }

    if (left.isEmpty() != right.isEmpty()) {
      if (firstError) {
        error     += F("Name and value should both be filled for filter ");
        firstError = false;
      } else {
        error += ',';
      }
      error += filterNr;
    }
    filterNr++;
    idx += 3;
    delay(0); // leave some yield
  }
  #  ifndef P037_FILTER_PER_TOPIC

  if (!firstError) {
    error += '\n';
  }
  #  endif // ifndef P037_FILTER_PER_TOPIC
  # endif  // if P037_FILTER_SUPPORT

  error += saveSettings();

  if (!error.isEmpty()) {
    addHtmlError(error);
  }
  # if P037_MAPPING_SUPPORT
  _maxIdx = -1; // Invalidate current mappings and filters
  # endif // if P037_MAPPING_SUPPORT
  # if P037_FILTER_SUPPORT
  _maxFilter = -1;
  # endif // if P037_FILTER_SUPPORT

  success = true;

  return success;
} // webform_save

# if P037_MAPPING_SUPPORT
#  ifdef PLUGIN_037_DEBUG
void P037_data_struct::logMapValue(const String& input, const String& result) {
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String info;
    info.reserve(45);
    info += F("IMPT : MQTT mapped value '");
    info += input;
    info += F("' to '");
    info += result;
    info += '\'';
    addLogMove(LOG_LEVEL_INFO, info);
  }
} // logMapValue

#  endif // ifdef PLUGIN_037_DEBUG

/**
 * Map a string to a (numeric) value, unchanged if no mapping found
 */
String P037_data_struct::mapValue(const String& input, const String& attribute) {
  String result = String(input); // clone

  if (!input.isEmpty()) {
    parseMappings();
    const String operands = P037_OPERAND_LIST;

    int8_t idx = 0;

    for (uint8_t mappingOffset = P037_START_MAPPINGS; mappingOffset <= P037_END_MAPPINGS && idx <= _maxIdx; mappingOffset++) {
      const String name = parseStringKeepCase(valueArray[mappingOffset], 1, P037_VALUE_SEPARATOR);
      const String oper = parseString(valueArray[mappingOffset], 2, P037_VALUE_SEPARATOR);
      const String valu = parseStringKeepCase(valueArray[mappingOffset], 3, P037_VALUE_SEPARATOR);

      if ((name == input) || ((!attribute.isEmpty()) && (name == attribute))) {
        int8_t operandIndex = operands.indexOf(oper);

        switch (operandIndex) {
          case 0: // = => 1:1 mapping
          {
            if (!valu.isEmpty()) {
              result = valu;
              #  ifdef PLUGIN_037_DEBUG
              logMapValue(input, result);
              #  endif // ifdef PLUGIN_037_DEBUG
            }
            break;
          }
          case 1: // % => percentage of mapping
          {
            double inputDouble;
            double mappingDouble;

            if (validDoubleFromString(input, inputDouble) &&
                validDoubleFromString(valu, mappingDouble)) {
              if (compareDoubleValues('>', mappingDouble, 0.0)) {
                double resultDouble = (100.0 / mappingDouble) * inputDouble; // Simple calculation to percentage
                int8_t decimals     = 0;
                int8_t dotPos       = input.indexOf('.');

                if (dotPos > -1) {
                  String decPart = input.substring(dotPos + 1);
                  decimals = decPart.length();             // Take the number of decimals to the output value
                }
                result = toString(resultDouble, decimals); // Percentage with same decimals as input
                #  ifdef PLUGIN_037_DEBUG
                logMapValue(input, result);
                #  endif // ifdef PLUGIN_037_DEBUG
              }
            }
            break;
          }
          default:
            break;
        }
      }
      idx++;
    }
  }

  return result;
} // mapValue

# endif // if P037_MAPPING_SUPPORT

# if P037_FILTER_SUPPORT

/**
 * do we have filter values?
 */
bool P037_data_struct::hasFilters() {
  parseMappings(); // When not parsed yet
  #  ifdef PLUGIN_037_DEBUG
  String log;
  log += F("p037 hasFilter: ");
  log += _maxFilter;
  addLogMove(LOG_LEVEL_INFO, log);
  #  endif // ifdef PLUGIN_037_DEBUG
  return _maxFilter > 0;
} // hasFilters

#  ifdef P037_FILTER_PER_TOPIC
String P037_data_struct::getFilterAsTopic(uint8_t topicId) {
  String result;

  if (hasFilters() &&
      (topicId > 0) &&
      (topicId <= VARS_PER_TASK)) {
    result.reserve(32);
    uint8_t fltBase = (topicId - 1) + P037_START_FILTERS;
    String  name    = parseStringKeepCase(valueArray[fltBase], 1, P037_VALUE_SEPARATOR);
    String  valu    = parseStringKeepCase(valueArray[fltBase], 3, P037_VALUE_SEPARATOR);

    if ((!name.isEmpty()) &&
        (!valu.isEmpty())) {
      result += '/';
      result += name;
      result += '/';

      if (!_filterListItem.isEmpty()) {
        result += _filterListItem;
      } else {
        result += valu;
      }
    }
  }
  return result;
}

#  endif // P037_FILTER_PER_TOPIC

#  ifdef PLUGIN_037_DEBUG
void P037_data_struct::logFilterValue(const String& text, const String& key, const String& value, const String& match) {
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log;
    log.reserve(text.length() + key.length() + value.length() + match.length() + 16);
    log += text;
    log += key;
    log += F(" value: ");
    log += value;
    log += F(" match: ");
    log += match;
    addLogMove(LOG_LEVEL_INFO, log);
  }
} // logFilterValue

#  endif // PLUGIN_037_DEBUG

/**
 * checkFilters
 * Algorithm: (all comparisons are case-sensitive)
 * - If key is not found in the list, return true
 * - If key is found and value matches, return true
 * - if key is found but value doesn't match, return false
 * key can be in the list multiple times
 */
bool P037_data_struct::checkFilters(const String& key, const String& value, int8_t topicId) {
  bool result = true;

  if ((!key.isEmpty()) &&
      (!value.isEmpty())) { // Ignore empty input(s)
    String  filters = P037_FILTER_LIST;
    String  valueData = value;
    String  fltKey, fltIndex, filterData, fltOper;
    double  from, to, doubleValue;
    int8_t  rangeSeparator;
    bool    accept       = true;
    bool    matchTopicId = true;
    uint8_t fltFrom      = P037_START_FILTERS;
    uint8_t fltMax       = P037_START_FILTERS + _maxFilter;
    #  ifdef P037_FILTER_PER_TOPIC

    if (topicId > 0) {
      fltFrom = (topicId - 1) + P037_START_FILTERS;
      fltMax  = topicId + P037_START_FILTERS;
    }
    #  endif // ifdef P037_FILTER_PER_TOPIC

    for (uint8_t flt = fltFrom; flt < fltMax; flt++) {
      fltOper        = parseStringKeepCase(valueArray[flt], 2, P037_VALUE_SEPARATOR);
      fltKey         = parseStringKeepCase(valueArray[flt], 1, P037_VALUE_SEPARATOR);
      rangeSeparator = parseString(fltKey, 2).toInt();

      if (rangeSeparator > 0) {
        valueData.replace(';', ',');
        valueData = parseString(valueData, rangeSeparator);
      }
      fltKey = parseString(fltKey, 1);
      fltKey.trim();

      if (fltKey == key) {
        result = false;                          // Matched key, so now we are looking for matching value
        int8_t filterIndex = filters.indexOf(fltOper);
        filterData = parseStringKeepCase(valueArray[flt], 3, P037_VALUE_SEPARATOR);
        parseSystemVariables(filterData, false); // Replace system variables

        switch (filterIndex) {
          case 0:                                // = => equals
          {
            _filterListItem = EMPTY_STRING;

            if (filterData == valueData) {
              #  ifdef PLUGIN_037_DEBUG
              String match;
              match.reserve(30);
              match += parseStringKeepCase(valueArray[flt], 3, P037_VALUE_SEPARATOR); // re-parse

              if (topicId > 0) {
                match += F(" topic match: ");
                match += matchTopicId ? F("yes") : F("no");
              }
              logFilterValue(F("P037 filter equals key: "), key, valueData, match);
              #  endif // ifdef PLUGIN_037_DEBUG
              return matchTopicId; // Match, don't look any further
            }
            break;
          }
          case 1:                                       // - => range x-y (inside) or y-x (outside)
          {
            _filterListItem = EMPTY_STRING;
            rangeSeparator  = filterData.indexOf(';');  // Semicolons

            if (rangeSeparator == -1) {
              rangeSeparator = filterData.indexOf('-'); // Fall-back test for dash
            }

            if (rangeSeparator > -1) {
              accept = false;

              if (validDoubleFromString(filterData.substring(0, rangeSeparator),  from) &&
                  validDoubleFromString(filterData.substring(rangeSeparator + 1), to) &&
                  validDoubleFromString(valueData,                                doubleValue)) {
                if (compareDoubleValues('>' + '=', to, from)) { // Normal low - high range: between low and high
                  if (compareDoubleValues('>' + '=', doubleValue, from) &&
                      compareDoubleValues('<' + '=', doubleValue, to)) {
                    accept = true;
                  }
                } else { // Alternative high - low range: outside low and high values
                  if (compareDoubleValues('>' + '=', doubleValue, from) ||
                      compareDoubleValues('<' + '=', doubleValue, to)) {
                    accept = true;
                  }
                }

                #  ifdef PLUGIN_037_DEBUG

                if (loglevelActiveFor(LOG_LEVEL_INFO)) {
                  String match;
                  match.reserve(30);
                  match += F("P037 filter ");
                  match += accept ? EMPTY_STRING : F("NOT ");
                  match += F("in range key: ");
                  logFilterValue(match, key, valueData, parseStringKeepCase(valueArray[flt], 3, P037_VALUE_SEPARATOR));
                }
                #  endif // ifdef PLUGIN_037_DEBUG

                if (accept) {
                  return matchTopicId; // bail out, we're done
                }
              }
            }
            break;
          }
          #  if P037_FILTER_COUNT >= 3
          case 2: // : => Match against a semicolon-separated list
          {
            _filterListItem = EMPTY_STRING;
            String item;
            rangeSeparator = filterData.indexOf(';');

            if ((rangeSeparator > -1) &&
                validDoubleFromString(valueData, doubleValue)) {
              accept = false;

              do {
                item = filterData.substring(0, rangeSeparator);
                item.trim();
                filterData = filterData.substring(rangeSeparator + 1);
                filterData.trim();
                rangeSeparator = filterData.indexOf(';');

                if (rangeSeparator == -1) {
                  rangeSeparator = filterData.length(); // Last value
                }

                if (validDoubleFromString(item, from) &&
                    compareDoubleValues('=', doubleValue, from)) {
                  accept          = true;
                  _filterListItem = item;
                }
              } while (!filterData.isEmpty() && !accept);

              #   ifdef PLUGIN_037_DEBUG

              if (loglevelActiveFor(LOG_LEVEL_INFO)) {
                String match;
                match.reserve(30);
                match += F("P037 filter ");
                match += accept ? EMPTY_STRING : F("NOT ");
                match += F("in list key: ");
                logFilterValue(match, key, valueData, parseStringKeepCase(valueArray[flt], 3, P037_VALUE_SEPARATOR));
              }
              #   endif // ifdef PLUGIN_037_DEBUG

              if (accept) {
                return matchTopicId; // bail out, we're done
              }
            }
            break;
          }
          #  endif // if P037_FILTER_COUNT >= 3
          default:
            break;
        }
      }
      delay(0); // Allow some yield
    }
  }
  return result;
}

# endif // if P037_FILTER_SUPPORT

# ifdef P037_JSON_SUPPORT

/**
 * Allocate a DynamicJsonDocument and parse the message.
 * Returns true if the operation succeeded, and doc and iter can be used, when n ot successful the state of those variables is undefined.
 */
bool P037_data_struct::parseJSONMessage(const String& message) {
  bool result = false;

  if ((nullptr != root) &&
      (message.length() * 1.5 > lastJsonMessageLength)) {
    cleanupJSON();
  }

  if (message.length() * 1.5 > lastJsonMessageLength) {
    lastJsonMessageLength = message.length() * 1.5;
    #  ifdef PLUGIN_037_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      log.reserve(35);
      log += F("IMPT : JSON buffer increased to ");
      log += lastJsonMessageLength;
      addLogMove(LOG_LEVEL_INFO, log);
    }
    #  endif // ifdef PLUGIN_037_DEBUG
  }

  if (nullptr == root) {
    root = new (std::nothrow) DynamicJsonDocument(lastJsonMessageLength); // Dynamic allocation
  }

  if (nullptr != root) {
    deserializeJson(*root, message.c_str());

    if (!root->isNull()) {
      result = true;
      doc    = root->as<JsonObject>();
      iter   = doc.begin();
    }
  }
  return result;
}

/**
 * Release the created DynamicJsonDocument (if it was allocated)
 */
void P037_data_struct::cleanupJSON() {
  if (nullptr != root) {
    root->clear();
    delete root;
    root = nullptr;
  }
}

# endif // P037_JSON_SUPPORT

#endif  // ifdef USES_P037

#include "../PluginStructs/P120_data_struct.h"

#if defined(USES_P120) || defined(USES_P125)

// **************************************************************************/
// Constructor I2C
// **************************************************************************/
P120_data_struct::P120_data_struct(
  uint8_t i2c_addr,
  uint8_t aSize)
  : _i2c_addr(i2c_addr), _aSize(aSize)
{
  i2c_mode = true;
  initialization();
}

// **************************************************************************/
// Constructor SPI
// **************************************************************************/
P120_data_struct::P120_data_struct(
  int     cs_pin,
  uint8_t aSize)
  : _cs_pin(cs_pin), _aSize(aSize)
{
  i2c_mode = false;
  initialization();
}

// **************************************************************************/
// Common initialization
// **************************************************************************/
void P120_data_struct::initialization() {
  if (_aSize == 0) { _aSize = 1; }
  _XA.resize(_aSize, 0);
  _YA.resize(_aSize, 0);
  _ZA.resize(_aSize, 0);
  _aUsed = 0;
  _aMax  = 0;
}

// **************************************************************************/
// Destructor
// **************************************************************************/
P120_data_struct::~P120_data_struct() {
  if (initialized()) {
    delete adxl345;
    adxl345 = nullptr;
  }
}

// **************************************************************************/
// Initialize sensor and read data from ADXL345
// **************************************************************************/
bool P120_data_struct::read_sensor(struct EventStruct *event) {
  # if PLUGIN_120_DEBUG
  String log;
  # endif // if PLUGIN_120_DEBUG

  if (!initialized()) {
    init_sensor(event);
    # if PLUGIN_120_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG) &&
        log.reserve(55)) {
      if (i2c_mode) {
        #  ifdef USES_P120
        log  = F("ADXL345: i2caddress: 0x");
        log += String(_i2cAddress, HEX);
        #  endif // ifdef USES_P120
      } else {
        #  ifdef USES_P125
        log  = F("ADXL345: CS-pin: ");
        log += _cs_pin;
        #  endif // ifdef USES_P125
      }
      log += F(", initialized: ");
      log += String(initialized() ? F("true") : F("false"));
      log += F(", ID=0x");
      log += String(adxl345->getDevID(), HEX);
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
    # endif // if PLUGIN_120_DEBUG
  }

  if (initialized()) {
    _x = 0; _y = 0; _z = 0;
    adxl345->readAccel(&_x, &_y, &_z);
    _XA[_aUsed] = _x;
    _YA[_aUsed] = _y;
    _ZA[_aUsed] = _z;

    _aUsed++;

    if ((_aMax < _aUsed) && (_aUsed < _aSize)) {
      _aMax = _aUsed;
    }

    if (_aUsed == _aSize) {
      _aUsed = 0;
    }

    # if PLUGIN_120_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG) &&
        log.reserve(40)) {
      log  = F("ADXL345: X: ");
      log += _x;
      log += F(", Y: ");
      log += _y;
      log += F(", Z: ");
      log += _z;
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
    # endif // if PLUGIN_120_DEBUG

    sensor_check_interrupt(event); // Process any interrupt

    return true;
  }
  return false;
}

// **************************************************************************/
// Average the measurements and return the results
// **************************************************************************/
bool P120_data_struct::read_data(struct EventStruct *event, int& X, int& Y, int& Z) {
  X = 0;
  Y = 0;
  Z = 0;

  if (initialized()) {
    for (uint8_t n = 0; n <= _aMax; n++) {
      X += _XA[n];
      Y += _YA[n];
      Z += _ZA[n];
    }

    X /= _aMax; // Average available measurements
    Y /= _aMax;
    Z /= _aMax;

    # if PLUGIN_120_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log;

      if (log.reserve(40)) {
        log  = F("ADXL345: averages, X: ");
        log += X;
        log += F(", Y: ");
        log += Y;
        log += F(", Z: ");
        log += Z;
        addLogMove(LOG_LEVEL_DEBUG, log);
      }
    }
    # endif // if PLUGIN_120_DEBUG
  }
  return initialized();
}

// **************************************************************************/
// Initialize ADXL345
// **************************************************************************/
bool P120_data_struct::init_sensor(struct EventStruct *event) {
  if (i2c_mode) {
    adxl345 = new (std::nothrow) ADXL345(_i2c_addr); // Init using I2C
  } else {
    adxl345 = new (std::nothrow) ADXL345(_cs_pin);   // Init using SPI
  }

  if (initialized()) {
    uint8_t act = 0, freeFall = 0, singleTap = 0, doubleTap = 0;
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String  log = F("ADXL345: Initializing sensor for ");

      if (i2c_mode) {
        log += F("I2C");
      } else {
        log += F("SPI");
      }
      log += F("...");
      addLogMove(LOG_LEVEL_INFO, log);
    }
    adxl345->powerOn();
    adxl345->setRangeSetting(2 ^ (get2BitFromUL(P120_CONFIG_FLAGS1, P120_FLAGS1_RANGE) + 1)); // Range is stored in 2 bits, only 4 possible
                                                                                              // options
    // Activity triggering
    // Inactivity triggering, same axis
    adxl345->setActivityXYZ(bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_X),
                            bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Y),
                            bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Z));
    adxl345->setInactivityXYZ(bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_X),
                              bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Y),
                              bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Z));

    if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_X) ||
                              bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Y) ||
                              bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Z)) {
      adxl345->setActivityThreshold(get8BitFromUL(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_TRESHOLD));
      adxl345->setInactivityThreshold(get8BitFromUL(P120_CONFIG_FLAGS1, P120_FLAGS1_INACTIVITY_TRESHOLD));
      act = 1;
    }

    // Axis Offsets
    adxl345->setAxisOffset(get8BitFromUL(P120_CONFIG_FLAGS4, P120_FLAGS4_OFFSET_X) - 0x80,
                           get8BitFromUL(P120_CONFIG_FLAGS4, P120_FLAGS4_OFFSET_Y) - 0x80,
                           get8BitFromUL(P120_CONFIG_FLAGS4, P120_FLAGS4_OFFSET_Z) - 0x80);

    // Tap triggering
    adxl345->setTapDetectionOnXYZ(bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_X),
                                  bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_Y),
                                  bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_Z));

    // Tap detection
    if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_X) ||
                                  bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_Y) ||
                                  bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_Z)) {
      adxl345->setTapThreshold(get8BitFromUL(P120_CONFIG_FLAGS2, P120_FLAGS2_TAP_TRESHOLD));
      adxl345->setTapDuration(get8BitFromUL(P120_CONFIG_FLAGS2, P120_FLAGS2_TAP_DURATION));
      singleTap = 1;

      // Double-tap detection
      if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_DBL_TAP)) {
        adxl345->setDoubleTapLatency(get8BitFromUL(P120_CONFIG_FLAGS2, P120_FLAGS2_DBL_TAP_LATENCY));
        adxl345->setDoubleTapWindow(get8BitFromUL(P120_CONFIG_FLAGS2, P120_FLAGS2_DBL_TAP_WINDOW));
        doubleTap = 1;
      } else {
        adxl345->setDoubleTapLatency(0); // Off
        adxl345->setDoubleTapWindow(0);
      }
    } else {
      adxl345->setTapThreshold(0); // Off
      adxl345->setTapDuration(0);
      adxl345->setDoubleTapLatency(0);
      adxl345->setDoubleTapWindow(0);
    }

    // Free-fall detection
    if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_FREE_FALL)) {
      adxl345->setFreeFallThreshold(get8BitFromUL(P120_CONFIG_FLAGS3, P120_FLAGS3_FREEFALL_TRESHOLD));
      adxl345->setFreeFallDuration(get8BitFromUL(P120_CONFIG_FLAGS3,  P120_FLAGS3_FREEFALL_DURATION));
      freeFall = 1;
    } else {
      adxl345->setFreeFallThreshold(0);
      adxl345->setFreeFallDuration(0);
    }

    // Enable interrupts
    adxl345->setImportantInterruptMapping(singleTap, doubleTap, freeFall, act, act);
    adxl345->ActivityINT(act);
    adxl345->InactivityINT(act);
    adxl345->singleTapINT(singleTap);
    adxl345->doubleTapINT(doubleTap);
    adxl345->FreeFallINT(freeFall);

    addLog(LOG_LEVEL_INFO, F("ADXL345: Initialization done."));
  } else {
    addLog(LOG_LEVEL_ERROR, F("ADXL345: Initialization of sensor failed."));
    return false;
  }

  #ifndef BUILD_NO_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log;

    if (log.reserve(25)) {
      if (i2c_mode) {
        # ifdef USES_P120
        log  = F("ADXL345: Address: 0x");
        log += String(_i2c_addr, HEX);
        # endif // ifdef USES_P120
      } else {
        # ifdef USES_P125
        log  = F("ADXL345: CS-pin: ");
        log += _cs_pin;
        # endif // ifdef USES_P125
      }
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
  }
  #endif

  return true;
}

/* Look for Interrupts and Triggered Action    */
void P120_data_struct::sensor_check_interrupt(struct EventStruct *event) {
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  uint8_t interrupts = adxl345->getInterruptSource();
  String  payload;

  payload.reserve(30);

  // Free Fall Detection
  if (adxl345->triggered(interrupts, ADXL345_FREE_FALL)) {
    if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_LOG_ACTIVITY)) {
      addLog(LOG_LEVEL_INFO, F("ADXL345: *** FREE FALL ***"));
    }
    payload = F("FreeFall=");
    appendPayloadXYZ(event, payload, 1u, 1u, 1u); // Use all values
    send_task_event(event, payload);
  }

  // Inactivity
  if (adxl345->triggered(interrupts, ADXL345_INACTIVITY) &&
      bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_SEND_ACTIVITY)) {
    if (!inactivityTriggered) {
      if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_LOG_ACTIVITY)) {
        addLog(LOG_LEVEL_INFO, F("ADXL345: *** INACTIVITY ***"));
      }
      payload = F("Inactivity=");
      appendPayloadXYZ(event, payload,
                       bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_X),
                       bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Y),
                       bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Z));
      send_task_event(event, payload);
    }
    inactivityTriggered = true;
    activityTriggered   = false;
  }

  // Activity
  if (adxl345->triggered(interrupts, ADXL345_ACTIVITY) &&
      bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_SEND_ACTIVITY)) {
    if (!activityTriggered) {
      if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_LOG_ACTIVITY)) {
        addLog(LOG_LEVEL_INFO, F("ADXL345: *** ACTIVITY ***"));
      }
      payload = F("Activity=");
      appendPayloadXYZ(event, payload,
                       bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_X),
                       bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Y),
                       bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Z));
      send_task_event(event, payload);
    }
    activityTriggered   = true;
    inactivityTriggered = false;
  }

  // Double Tap Detection
  // Tap Detection
  if (adxl345->triggered(interrupts, ADXL345_DOUBLE_TAP) ||
      (adxl345->triggered(interrupts, ADXL345_SINGLE_TAP))) {
    if (adxl345->triggered(interrupts, ADXL345_SINGLE_TAP)) {
      if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_LOG_ACTIVITY)) {
        addLog(LOG_LEVEL_INFO, F("ADXL345: *** TAP ***"));
      }
      payload = F("Tapped");
    }

    if (adxl345->triggered(interrupts, ADXL345_DOUBLE_TAP)) {      // tonhuisman: Double-tap overrides single-tap event
      if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_LOG_ACTIVITY)) { // This is on purpose and as intended!
        addLog(LOG_LEVEL_INFO, F("ADXL345: *** DOUBLE TAP ***"));
      }
      payload = F("DoubleTapped");
    }
    payload += '=';

    appendPayloadXYZ(event, payload,
                     bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_X),
                     bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_Y),
                     bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_Z));
    send_task_event(event, payload);
  }
}

// *******************************************************************
// Append X, Y and Z arguments where configured
// *******************************************************************
void P120_data_struct::appendPayloadXYZ(struct EventStruct *event, String& payload,
                                        uint8_t useX,
                                        uint8_t useY,
                                        uint8_t useZ) {
  if (useX == 1) {
    if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_EVENT_RAW_VALUES)) {
      payload += _x;
    } else {
      payload += formatUserVarNoCheck(event, 0);
    }
  } else {
    payload += 0;
  }
  payload += ',';

  if (useY == 1) {
    if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_EVENT_RAW_VALUES)) {
      payload += _y;
    } else {
      payload += formatUserVarNoCheck(event, 1);
    }
  } else {
    payload += 0;
  }
  payload += ',';

  if (useZ == 1) {
    if (bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_EVENT_RAW_VALUES)) {
      payload += _z;
    } else {
      payload += formatUserVarNoCheck(event, 2);
    }
  } else {
    payload += 0;
  }
}

// *******************************************************************
// Send out an event for the current event, aith provided payload
// *******************************************************************
void P120_data_struct::send_task_event(struct EventStruct *event,
                                       String            & eventPayload) {
  if (Settings.UseRules &&
      !eventPayload.isEmpty()) {
    String RuleEvent;
    RuleEvent += getTaskDeviceName(event->TaskIndex);
    RuleEvent += '#';
    RuleEvent += eventPayload;
    eventQueue.addMove(std::move(RuleEvent));
  }
}

// *******************************************************************
// Load the configuration interface
// *******************************************************************
bool P120_data_struct::plugin_webform_load(struct EventStruct *event) {
  if (!i2c_mode) {
    addFormSubHeader(F("Device Settings"));
  }

  // Range
  {
    const __FlashStringHelper *rangeOptions[] = {
      F("2g"),
      F("4g"),
      F("8g"),
      F("16g (default)") };
    int rangeValues[] = { P120_RANGE_2G, P120_RANGE_4G, P120_RANGE_8G, P120_RANGE_16G };
    addFormSelector(F("Range"), F("p120_range"), 4, rangeOptions, rangeValues,
                    get2BitFromUL(P120_CONFIG_FLAGS1, P120_FLAGS1_RANGE));
  }

  // Axis selection
  {
    addFormCheckBox(F("X-axis activity sensing"), F("p120_activity_x"), bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_X) == 1);
    addFormCheckBox(F("Y-axis activity sensing"), F("p120_activity_y"), bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Y) == 1);
    addFormCheckBox(F("Z-axis activity sensing"), F("p120_activity_z"), bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_Z) == 1);
    addFormNumericBox(F("Activity treshold"), F("p120_activity_treshold"),
                      get8BitFromUL(P120_CONFIG_FLAGS1, P120_FLAGS1_ACTIVITY_TRESHOLD), 1, 255);
    addUnit(F("1..255 * 62.5 mg"));
    addFormNumericBox(F("In-activity treshold"), F("p120_inactivity_treshold"),
                      get8BitFromUL(P120_CONFIG_FLAGS1, P120_FLAGS1_INACTIVITY_TRESHOLD), 1, 255);
    addUnit(F("1..255 * 62.5 mg"));
  }

  // Activity logging and send events for (in)activity
  {
    addFormCheckBox(F("Enable (in)activity events"), F("p120_send_activity"),
                    bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_SEND_ACTIVITY) == 1);
    addFormCheckBox(F("Log sensor activity (INFO)"), F("p120_log_activity"),
                    bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_LOG_ACTIVITY) == 1);
    addFormCheckBox(F("Events with raw measurements"), F("p120_raw_measurement"),
                    bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_EVENT_RAW_VALUES) == 1);
  }

  // Tap detection
  {
    addFormSubHeader(F("Tap detection"));

    addFormCheckBox(F("X-axis"), F("p120_tap_x"), bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_X) == 1);
    addFormCheckBox(F("Y-axis"), F("p120_tap_y"), bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_Y) == 1);
    addFormCheckBox(F("Z-axis"), F("p120_tap_z"), bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_TAP_Z) == 1);
    addFormNote(F("Also enables taskname#Tapped event."));
    addFormNumericBox(F("Tap treshold"), F("p120_tap_treshold"),
                      get8BitFromUL(P120_CONFIG_FLAGS2, P120_FLAGS2_TAP_TRESHOLD), 1, 255);
    addUnit(F("1..255 * 62.5 mg"));
    addFormNumericBox(F("Tap duration"), F("p120_tap_duration"),
                      get8BitFromUL(P120_CONFIG_FLAGS2, P120_FLAGS2_TAP_DURATION), 1, 255);
    addUnit(F("1..255 * 625 &micro;s"));
  }

  // Double-tap detection
  {
    addFormCheckBox(F("Enable double-tap detection"), F("p120_dbl_tap"), bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_DBL_TAP) == 1);
    addFormNote(F("Also enables taskname#DoubleTapped event."));
    addFormNumericBox(F("Double-tap latency"), F("p120_dbl_tap_latency"),
                      get8BitFromUL(P120_CONFIG_FLAGS2, P120_FLAGS2_DBL_TAP_LATENCY), 1, 255);
    addUnit(F("1..255 * 1.25 ms"));
    addFormNumericBox(F("Double-tap window"), F("p120_dbl_tap_window"),
                      get8BitFromUL(P120_CONFIG_FLAGS2, P120_FLAGS2_DBL_TAP_WINDOW), 1, 255);
    addUnit(F("1..255 * 1.25 ms"));
  }

  // Free-fall detection
  {
    addFormSubHeader(F("Free-fall detection"));

    addFormCheckBox(F("Enable free-fall detection"), F("p120_free_fall"), bitRead(P120_CONFIG_FLAGS1, P120_FLAGS1_FREE_FALL) == 1);
    addFormNote(F("Also enables taskname#FreeFall event."));
    addFormNumericBox(F("Free-fall treshold"), F("p120_free_fall_treshold"),
                      get8BitFromUL(P120_CONFIG_FLAGS3, P120_FLAGS3_FREEFALL_TRESHOLD), 1, 255);
    addUnit(F("1..255 * 62.5 mg"));
    addFormNumericBox(F("Free-fall duration"), F("p120_free_fall_duration"),
                      get8BitFromUL(P120_CONFIG_FLAGS3, P120_FLAGS3_FREEFALL_DURATION), 1, 255);
    addUnit(F("1..255 * 625 &micro;s"));
  }

  // Axis Offsets (calibration)
  {
    addFormSubHeader(F("Axis calibration"));
    addFormNumericBox(F("X-Axis offset"), F("p120_offset_x"),
                      get8BitFromUL(P120_CONFIG_FLAGS4, P120_FLAGS4_OFFSET_X) - 0x80, -127, 127);
    addUnit(F("-127..127 * 15.6 mg"));
    addFormNumericBox(F("Y-Axis offset"), F("p120_offset_y"),
                      get8BitFromUL(P120_CONFIG_FLAGS4, P120_FLAGS4_OFFSET_Y) - 0x80, -127, 127);
    addUnit(F("-127..127 * 15.6 mg"));
    addFormNumericBox(F("Z-Axis offset"), F("p120_offset_z"),
                      get8BitFromUL(P120_CONFIG_FLAGS4, P120_FLAGS4_OFFSET_Z) - 0x80, -127, 127);
    addUnit(F("-127..127 * 15.6 mg"));
  }

  // Data retrieval options
  {
    addFormSubHeader(F("Data retrieval"));

    addFormNumericBox(F("Averaging buffer size"), F("p120_average_buf"), P120_AVERAGE_BUFFER, 1, 100);
    addUnit(F("1..100"));

    const __FlashStringHelper *frequencyOptions[] = {
      F("10"),
      F("50") };
    int frequencyValues[] = { P120_FREQUENCY_10, P120_FREQUENCY_50 };
    addFormSelector(F("Measuring frequency"), F("p120_frequency"), 2, frequencyOptions, frequencyValues, P120_FREQUENCY);
    addUnit(F("Hz"));
    addFormNote(F("Values X/Y/Z are updated 1x per second, Controller updates &amp; Value-events are based on 'Interval' setting."));
  }

  return true;
}

// *******************************************************************
// Save the configuration interface
// *******************************************************************
bool P120_data_struct::plugin_webform_save(struct EventStruct *event) {
  P120_FREQUENCY = getFormItemInt(F("p120_frequency"));
  uint32_t flags = 0ul;

  set2BitToUL(flags, P120_FLAGS1_RANGE, getFormItemInt(F("p120_range")));
  bitWrite(flags, P120_FLAGS1_ACTIVITY_X,       isFormItemChecked(F("p120_activity_x")));
  bitWrite(flags, P120_FLAGS1_ACTIVITY_Y,       isFormItemChecked(F("p120_activity_y")));
  bitWrite(flags, P120_FLAGS1_ACTIVITY_Z,       isFormItemChecked(F("p120_activity_z")));
  bitWrite(flags, P120_FLAGS1_TAP_X,            isFormItemChecked(F("p120_tap_x")));
  bitWrite(flags, P120_FLAGS1_TAP_Y,            isFormItemChecked(F("p120_tap_y")));
  bitWrite(flags, P120_FLAGS1_TAP_Z,            isFormItemChecked(F("p120_tap_z")));
  bitWrite(flags, P120_FLAGS1_DBL_TAP,          isFormItemChecked(F("p120_dbl_tap")));
  bitWrite(flags, P120_FLAGS1_FREE_FALL,        isFormItemChecked(F("p120_free_fall")));
  bitWrite(flags, P120_FLAGS1_SEND_ACTIVITY,    isFormItemChecked(F("p120_send_activity")));
  bitWrite(flags, P120_FLAGS1_LOG_ACTIVITY,     isFormItemChecked(F("p120_log_activity")));
  bitWrite(flags, P120_FLAGS1_EVENT_RAW_VALUES, isFormItemChecked(F("p120_raw_measurement")));
  set8BitToUL(flags, P120_FLAGS1_ACTIVITY_TRESHOLD,   getFormItemInt(F("p120_activity_treshold")));
  set8BitToUL(flags, P120_FLAGS1_INACTIVITY_TRESHOLD, getFormItemInt(F("p120_inactivity_treshold")));
  P120_CONFIG_FLAGS1 = flags;

  flags = 0ul;
  set8BitToUL(flags, P120_FLAGS2_TAP_TRESHOLD,    getFormItemInt(F("p120_tap_treshold")));
  set8BitToUL(flags, P120_FLAGS2_TAP_DURATION,    getFormItemInt(F("p120_tap_duration")));
  set8BitToUL(flags, P120_FLAGS2_DBL_TAP_LATENCY, getFormItemInt(F("p120_dbl_tap_latency")));
  set8BitToUL(flags, P120_FLAGS2_DBL_TAP_WINDOW,  getFormItemInt(F("p120_dbl_tap_window")));
  P120_CONFIG_FLAGS2 = flags;

  flags = 0ul;
  set8BitToUL(flags, P120_FLAGS3_FREEFALL_TRESHOLD, getFormItemInt(F("p120_free_fall_treshold")));
  set8BitToUL(flags, P120_FLAGS3_FREEFALL_DURATION, getFormItemInt(F("p120_free_fall_duration")));
  P120_CONFIG_FLAGS3 = flags;

  flags = 0ul;
  set8BitToUL(flags, P120_FLAGS4_OFFSET_X, getFormItemInt(F("p120_offset_x")) + 0x80);
  set8BitToUL(flags, P120_FLAGS4_OFFSET_Y, getFormItemInt(F("p120_offset_y")) + 0x80);
  set8BitToUL(flags, P120_FLAGS4_OFFSET_Z, getFormItemInt(F("p120_offset_z")) + 0x80);
  P120_CONFIG_FLAGS4 = flags;

  return true;
}

// *******************************************************************
// Set defaultss for the configuration interface
// *******************************************************************
bool P120_data_struct::plugin_set_defaults(struct EventStruct *event) {
  bool success = false;

  uint32_t flags = 0ul;

  set2BitToUL(flags, P120_FLAGS1_RANGE, P120_RANGE_16G); // Default to 16g range for highest resolution
  bitSet(flags, P120_FLAGS1_ACTIVITY_X);                 // Detect activity on all axes
  bitSet(flags, P120_FLAGS1_ACTIVITY_Y);
  bitSet(flags, P120_FLAGS1_ACTIVITY_Z);
  set8BitToUL(flags, P120_FLAGS1_ACTIVITY_TRESHOLD,   P120_DEFAULT_ACTIVITY_TRESHOLD);
  set8BitToUL(flags, P120_FLAGS1_INACTIVITY_TRESHOLD, P120_DEFAULT_INACTIVITY_TRESHOLD);
  P120_CONFIG_FLAGS1 = flags;

  flags = 0ul;
  set8BitToUL(flags, P120_FLAGS2_TAP_TRESHOLD,    P120_DEFAULT_TAP_TRESHOLD);
  set8BitToUL(flags, P120_FLAGS2_TAP_DURATION,    P120_DEFAULT_TAP_DURATION);
  set8BitToUL(flags, P120_FLAGS2_DBL_TAP_LATENCY, P120_DEFAULT_DBL_TAP_LATENCY);
  set8BitToUL(flags, P120_FLAGS2_DBL_TAP_WINDOW,  P120_DEFAULT_DBL_TAP_WINDOW);
  P120_CONFIG_FLAGS2 = flags;

  flags = 0ul;
  set8BitToUL(flags, P120_FLAGS3_FREEFALL_TRESHOLD, P120_DEFAULT_FREEFALL_TRESHOLD);
  set8BitToUL(flags, P120_FLAGS3_FREEFALL_DURATION, P120_DEFAULT_FREEFALL_DURATION);
  P120_CONFIG_FLAGS3 = flags;

  flags = 0ul;
  set8BitToUL(flags, P120_FLAGS4_OFFSET_X, 0 + 0x80); // Offset 0 by default
  set8BitToUL(flags, P120_FLAGS4_OFFSET_Y, 0 + 0x80);
  set8BitToUL(flags, P120_FLAGS4_OFFSET_Z, 0 + 0x80);
  P120_CONFIG_FLAGS4 = flags;

  // No decimals plausible, as the outputs from the sensor are of type int
  for (uint8_t i = 0; i < VARS_PER_TASK; ++i) {
    ExtraTaskSettings.TaskDeviceValueDecimals[i] = 0;
  }

  success = true;
  return success;
}

#endif // if defined(USES_P120) || defined(USES_P125)


#include "../PluginStructs/P022_data_struct.h"

#ifdef USES_P022

bool P022_data_struct::p022_is_init(uint8_t address) {
  if ((address < PCA9685_ADDRESS) || (address > PCA9685_MAX_ADDRESS)) { return false; }
  uint32_t address_offset = address - PCA9685_ADDRESS;

  if (address_offset < 32) {
    return initializeState_lo & (1 << address_offset);
  } else {
    return initializeState_hi & (1 << (address_offset - 32));
  }
}

bool P022_data_struct::p022_set_init(uint8_t address) {
  if ((address < PCA9685_ADDRESS) || (address > PCA9685_MAX_ADDRESS)) { return false; }
  uint32_t address_offset = address - PCA9685_ADDRESS;

  if (address_offset < 32) {
    initializeState_lo |= (1 << address_offset);
  } else {
    initializeState_hi |= (1 << (address_offset - 32));
  }
  return true;
}

bool P022_data_struct::p022_clear_init(uint8_t address) {
  if ((address < PCA9685_ADDRESS) || (address > PCA9685_MAX_ADDRESS)) { return false; }
  uint32_t address_offset = address - PCA9685_ADDRESS;

  if (address_offset < 32) {
    initializeState_lo &= ~(1 << address_offset);
  } else {
    initializeState_hi &= ~(1 << (address_offset - 32));
  }
  return true;
}

// ********************************************************************************
// PCA9685 config
// ********************************************************************************
void P022_data_struct::Plugin_022_writeRegister(int i2cAddress, int regAddress, uint8_t data) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t P022_data_struct::Plugin_022_readRegister(int i2cAddress, int regAddress) {
  uint8_t res = 0;

  Wire.requestFrom(i2cAddress, 1, 1);

  while (Wire.available()) {
    res = Wire.read();
  }
  return res;
}

// ********************************************************************************
// PCA9685 write
// ********************************************************************************
void P022_data_struct::Plugin_022_Off(int address, int pin)
{
  Plugin_022_Write(address, pin, 0);
}

void P022_data_struct::Plugin_022_On(int address, int pin)
{
  Plugin_022_Write(address, pin, PCA9685_MAX_PWM);
}

void P022_data_struct::Plugin_022_Write(int address, int Par1, int Par2)
{
  int i2cAddress = address;

  // boolean success = false;
  int regAddress = Par1 == -1
                   ? PCA9685_ALLLED_REG
                   : PCA9685_LED0 + 4 * Par1;
  uint16_t LED_ON  = 0;
  uint16_t LED_OFF = Par2;

  Wire.beginTransmission(i2cAddress);
  Wire.write(regAddress);
  Wire.write(lowByte(LED_ON));
  Wire.write(highByte(LED_ON));
  Wire.write(lowByte(LED_OFF));
  Wire.write(highByte(LED_OFF));
  Wire.endTransmission();
}

void P022_data_struct::Plugin_022_Frequency(int address, uint16_t freq)
{
  int i2cAddress = address;

  Plugin_022_writeRegister(i2cAddress, PLUGIN_022_PCA9685_MODE1, (uint8_t)0x0);
  freq *= 0.9f;

  //  prescale = 25000000 / 4096;
  uint16_t prescale = 6103;

  prescale /=  freq;
  prescale -= 1;
  uint8_t oldmode = Plugin_022_readRegister(i2cAddress, 0);
  uint8_t newmode = (oldmode & 0x7f) | 0x10;

  Plugin_022_writeRegister(i2cAddress, PLUGIN_022_PCA9685_MODE1, (uint8_t)newmode);
  Plugin_022_writeRegister(i2cAddress, 0xfe,                     (uint8_t)prescale); // prescale register
  Plugin_022_writeRegister(i2cAddress, PLUGIN_022_PCA9685_MODE1, (uint8_t)oldmode);
  delayMicroseconds(5000);
  Plugin_022_writeRegister(i2cAddress, PLUGIN_022_PCA9685_MODE1, (uint8_t)oldmode | 0xa1);
}

void P022_data_struct::Plugin_022_initialize(int address)
{
  int i2cAddress = address;

  // default mode is open drain output, drive leds connected to VCC
  Plugin_022_writeRegister(i2cAddress, PLUGIN_022_PCA9685_MODE1, (uint8_t)0x01);      // reset the device
  delay(1);
  Plugin_022_writeRegister(i2cAddress, PLUGIN_022_PCA9685_MODE1, (uint8_t)B10100000); // set up for auto increment
  // Plugin_022_writeRegister(i2cAddress, PCA9685_MODE2, (uint8_t)0x10); // set to output
  p022_set_init(address);
}

#endif // ifdef USES_P022

#include "../PluginStructs/P112_data_struct.h"

#ifdef USES_P112

// #######################################################################################################
// #################### Plugin 112 I2C AS7265X Triad Spectroscopy Sensor and White, IR and UV LED ########
// #######################################################################################################
//
// Triad Spectroscopy Sensor and White, IR and UV LED
// like this one: https://www.sparkfun.com/products/15050
// based on this library: https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library
// this code is based on 29 Mar 2019-03-29 version of the above library
//
// 2021-03-29 heinemannj: Initial commit
//

// Needed also here for PlatformIO's library finder as the .h file
// is in a directory which is excluded in the src_filter
#include <SparkFun_AS7265X.h>

bool P112_data_struct::begin()
{
  if (!initialized) {
    initialized = sensor.begin();

    if (initialized) {
//      sensor.takeMeasurementsWithBulb();
    }
  }
  return initialized;
}

#endif // ifdef USES_P112

// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter

#include "../PluginStructs/P050_data_struct.h"


#ifdef USES_P050

# include "../Helpers/ESPEasy_Storage.h"

# include <Adafruit_TCS34725.h>

P050_data_struct::P050_data_struct(uint16_t integrationSetting, uint16_t gainSetting) {

  // Map integration time setting (uint16_t to enum)
  _integrationTime = static_cast<tcs34725IntegrationTime_t>(integrationSetting);

  // Map gain setting (uint16_t -> enum)
  _gain = static_cast<tcs34725Gain_t>(gainSetting);

  /* Initialise with specific int time and gain values */
  tcs = Adafruit_TCS34725(_integrationTime, _gain);

  resetTransformation();

  // String log = F("P050_data sizeof(TransformationSettings): ");
  // log += sizeof(TransformationSettings);
  // addLog(LOG_LEVEL_INFO, log);
}

/**
 * resetTransformation
 * Effectgively sets matrix[0][0], matrix[1][1] and matrix[2][2] to 1.0f, all other fields to 0.0f 
 */
void P050_data_struct::resetTransformation() {
  // Initialize Transformationn defaults
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      TransformationSettings.matrix[i][j] = i == j ? 1.0f : 0.0f;
    }
  }
}

/**
 * applyTransformation : calibrate r/g/b inputs (uint16_t) to rc/gc/bc outputs (float, by reference)
 */
void P050_data_struct::applyTransformation(uint16_t r, uint16_t g, uint16_t b, float *rc, float *gc, float *bc) {
  *rc = TransformationSettings.matrix[0][0] * static_cast<float>(r) + TransformationSettings.matrix[0][1] * static_cast<float>(g) + TransformationSettings.matrix[0][2] * static_cast<float>(b);
  *gc = TransformationSettings.matrix[1][0] * static_cast<float>(r) + TransformationSettings.matrix[1][1] * static_cast<float>(g) + TransformationSettings.matrix[1][2] * static_cast<float>(b);
  *bc = TransformationSettings.matrix[2][0] * static_cast<float>(r) + TransformationSettings.matrix[2][1] * static_cast<float>(g) + TransformationSettings.matrix[2][2] * static_cast<float>(b);
}

/**
 * applyTransformation : calibrate normalized r/g/b inputs (float) to rc/gc/bc outputs (float, by reference)
 */
void P050_data_struct::applyTransformation(float nr, float ng, float nb, float *rc, float *gc, float *bc) {
  *rc = TransformationSettings.matrix[0][0] * static_cast<float>(nr) + TransformationSettings.matrix[0][1] * static_cast<float>(ng) + TransformationSettings.matrix[0][2] * static_cast<float>(nb);
  *gc = TransformationSettings.matrix[1][0] * static_cast<float>(nr) + TransformationSettings.matrix[1][1] * static_cast<float>(ng) + TransformationSettings.matrix[1][2] * static_cast<float>(nb);
  *bc = TransformationSettings.matrix[2][0] * static_cast<float>(nr) + TransformationSettings.matrix[2][1] * static_cast<float>(ng) + TransformationSettings.matrix[2][2] * static_cast<float>(nb);
}

bool P050_data_struct::loadSettings(taskIndex_t taskIndex) {
  LoadCustomTaskSettings(taskIndex, reinterpret_cast<uint8_t *>(&TransformationSettings), sizeof(TransformationSettings));
  return  true;
}

bool P050_data_struct::saveSettings(taskIndex_t taskIndex) {
  SaveCustomTaskSettings(taskIndex, reinterpret_cast<const uint8_t *>(&TransformationSettings), sizeof(TransformationSettings));
  return true;
}

#endif // ifdef USES_P050

#include "../PluginStructs/P126_data_struct.h"

#ifdef USES_P126

// **************************************************************************/
// Constructor
// **************************************************************************/
P126_data_struct::P126_data_struct(int8_t  dataPin,
                                   int8_t  clockPin,
                                   int8_t  latchPin,
                                   uint8_t chipCount)
  : _dataPin(dataPin), _clockPin(clockPin), _latchPin(latchPin), _chipCount(chipCount) {
  shift = new (std::nothrow) ShiftRegister74HC595_NonTemplate(_chipCount, _dataPin, _clockPin, _latchPin);
}

// **************************************************************************/
// Destructor
// **************************************************************************/
P126_data_struct::~P126_data_struct() {
  if (nullptr != shift) {
    delete shift;
    shift = nullptr;
  }
}

bool P126_data_struct::plugin_init(struct EventStruct *event) {
  if (P126_CONFIG_FLAGS_GET_VALUES_RESTORE) { // Restore only when enabled
    uint8_t idx = P126_CONFIG_SHOW_OFFSET;
    std::vector<uint8_t> value;

    value.resize(_chipCount, 0);             // Initialize vector to 0's

    const uint8_t *pvalue = shift->getAll(); // Get current state

    for (uint8_t i = 0; i < _chipCount; i++) {
      value[i] = pvalue[i];
    }

    const uint16_t maxVar = min(static_cast<uint8_t>(VARS_PER_TASK),
                                static_cast<uint8_t>(ceil((P126_CONFIG_CHIP_COUNT - P126_CONFIG_SHOW_OFFSET) / 4.0)));
    uint32_t par;

    for (uint16_t varNr = 0; varNr < maxVar; varNr++) {
      par = UserVar.getUint32(event->TaskIndex, varNr);

      for (uint8_t n = 0; n < 4 && idx < _chipCount; n++, idx++) {
        value[idx] = ((par >> (n * 8)) & 0xff);

        # ifdef P126_DEBUG_LOG

        if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
          String log;
          log.reserve(64);
          log += F("SHIFTOUT: plugin_init: value[");
          log += idx;
          log += F("] : ");
          log += value[idx];
          log += F("/0x");
          log += String(value[idx], HEX);
          log += F(", n * 8: ");
          log += n;
          log += '/';
          log += n * 8;
          log += F(", varNr: ");
          log += varNr;
          addLogMove(LOG_LEVEL_DEBUG, log);
        }
        # endif // ifdef P126_DEBUG_LOG
      }
    }
    shift->setAll(&value[0], false); // DO NOT SEND OUTPUT TO REGISTERS
  }
  return true;
}

const uint32_t P126_data_struct::getChannelState(uint8_t offset, uint8_t size) const {
  uint32_t result       = 0u;
  const uint8_t *pvalue = shift->getAll();
  const uint8_t  last   = offset + size;

  if (nullptr != pvalue) {
    uint16_t sft = 0u;

    for (uint8_t ofs = offset; ofs < last; ofs++, sft++) {
      result += (pvalue[ofs] << (8 * sft));
    }

    # ifdef P126_DEBUG_LOG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log = F("SHIFTOUT: getChannelState offset: ");
      log += offset;
      log += F(", size: ");
      log += size;
      log += F(", result: ");
      log += result;
      log += F("/0x");
      log += String(result, HEX);
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
    # endif // ifdef P126_DEBUG_LOG
  }
  return result;
}

bool P126_data_struct::plugin_read(struct EventStruct *event) {
  const uint16_t last = P126_CONFIG_SHOW_OFFSET + (VARS_PER_TASK * 4);
  uint8_t varNr       = 0;

  for (uint16_t index = P126_CONFIG_SHOW_OFFSET; index < _chipCount && index < last && varNr < VARS_PER_TASK; index += 4, varNr++) {
    uint32_t result = getChannelState(index, min(VARS_PER_TASK, _chipCount - index));
    UserVar.setUint32(event->TaskIndex, varNr, result);
  }
  return true;
}

bool P126_data_struct::plugin_write(struct EventStruct *event,
                                    const String      & string) {
  bool   success = false;
  String command = parseString(string, 1);

  if (command.equals(F("shiftout"))) {
    const String subcommand = parseString(string,2);
    const bool hc_update = subcommand.indexOf(F("noupdate")) == -1;

    if (subcommand.equals(F("set")) || subcommand.equals(F("setnoupdate"))) {
      const uint8_t  pin   = event->Par2;
      const uint16_t value = event->Par3;

      if (validChannel(pin) && ((value == 0) || (value == 1))) {
        shift->set(pin - 1, value, hc_update);
        success = true;
        # ifdef P126_DEBUG_LOG

        if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
          String log = command;
          log += F(", pin: ");
          log += pin;
          log += F(", value: ");
          log += value;
          addLogMove(LOG_LEVEL_DEBUG, log);
        }
        # endif // ifdef P126_DEBUG_LOG
      }
    } else if (subcommand.equals(F("update"))) {
      shift->updateRegisters();
      success = true;
    } else if (subcommand.equals(F("setall")) || subcommand.equals(F("setallnoupdate"))) {
      success = true;
      std::vector<uint8_t> value;
      value.resize(_chipCount, 0);             // Initialize vector to 0's

      const uint8_t *pvalue = shift->getAll(); // Get current state

      for (uint8_t i = 0; i < _chipCount; i++) {
        value[i] = pvalue[i];
      }

      uint32_t par   = 0u;
      uint8_t  param = 3; // Start with an offset
      uint8_t  width = 4;
      uint8_t  idx   = 0;
      String   arg   = parseString(string, param);

      while (!arg.isEmpty() && idx < _chipCount && success) {
        int colon = arg.indexOf(':'); // First colon: Chip-index, range 1.._chipCount
        int itmp  = 0;

        if (colon != -1) {
          String cis = arg.substring(0, colon);
          arg = arg.substring(colon + 1);

          if (!cis.isEmpty() && validIntFromString(cis, itmp) && (itmp > 0) && (itmp <= _chipCount)) {
            idx = itmp - 1;       // Actual range is 0.._chipCount - 1
          } else {
            success = false;      // Cancel entire operation on error
          }
        }
        colon = arg.indexOf(':'); // Second colon: data width, range 1..4 bytes
        width = 4;                // Set default data width to 4 = 32 bits

        if (colon != -1) {
          String lis = arg.substring(0, colon);
          arg = arg.substring(colon + 1);

          if (!lis.isEmpty() && validIntFromString(lis, itmp) && (itmp > 0) && (itmp <= 4)) {
            width = itmp;
          } else {
            success = false; // Cancel entire operation on error
          }
        }
        int64_t tmp = 0;
        par = 0u; // reset

        if (validInt64FromString(arg, tmp)) {
          par = static_cast<uint32_t>(tmp);
        }

        # ifdef P126_DEBUG_LOG

        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log = command;
          log += F(": arg: ");
          log += arg;
          log += F(", tmp: ");
          log += ull2String(tmp);
          log += F("/0x");
          log += ull2String(tmp, HEX);
          log += F(", par: ");
          log += par;
          log += F("/0x");
          log += ull2String(par, HEX);
          log += F(", chip:");
          log += idx;
          log += F(", width:");
          log += width;
          addLogMove(LOG_LEVEL_INFO, log);
        }
        # endif // ifdef P126_DEBUG_LOG

        param++; // Process next argument
        arg = parseString(string, param);

        for (uint8_t n = 0; n < width && idx < _chipCount; n++, idx++) {
          value[idx] = ((par >> (n * 8)) & 0xff);

          # ifdef P126_DEBUG_LOG

          if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
            String log = command;
            log += F(": value[");
            log += idx;
            log += F("] : ");
            log += value[idx];
            log += F("/0x");
            log += String(value[idx], HEX);
            log += F(", n * 8: ");
            log += n;
            log += '/';
            log += n * 8;
            addLogMove(LOG_LEVEL_DEBUG, log);
          }
          # endif // ifdef P126_DEBUG_LOG
        }
      }

      if (success) {
        shift->setAll(&value[0], hc_update);
      }
    } else if (subcommand.equals(F("setalllow"))) {
      shift->setAllLow();
      success = true;
    } else if (subcommand.equals(F("setallhigh"))) {
      shift->setAllHigh();
      success = true;
    } else if (subcommand.equals(F("setoffset"))) {
      if ((event->Par2 >= 0) && (event->Par2 <= P126_MAX_SHOW_OFFSET)) {
        uint8_t previousOffset = P126_CONFIG_SHOW_OFFSET;
        P126_CONFIG_SHOW_OFFSET = event->Par2;

        if (P126_CONFIG_SHOW_OFFSET >= P126_CONFIG_CHIP_COUNT) {
          P126_CONFIG_SHOW_OFFSET = 0;
        }
        P126_CONFIG_SHOW_OFFSET -= (P126_CONFIG_SHOW_OFFSET % 4);

        if ((P126_CONFIG_CHIP_COUNT > 4) &&
            (P126_CONFIG_SHOW_OFFSET > P126_CONFIG_CHIP_COUNT - 4) &&
            (P126_CONFIG_CHIP_COUNT < P126_MAX_SHOW_OFFSET)) {
          P126_CONFIG_SHOW_OFFSET -= 4;
        }

        // Reset State_A..D values when changing the offset
        if ((previousOffset != P126_CONFIG_SHOW_OFFSET) && P126_CONFIG_FLAGS_GET_VALUES_RESTORE) {
          for (uint8_t varNr = 0; varNr < VARS_PER_TASK; varNr++) {
            UserVar.setUint32(event->TaskIndex, varNr, 0u);
          }
          # ifdef P126_DEBUG_LOG
          addLog(LOG_LEVEL_INFO, F("SHIFTOUT: 'Offset for display' changed: state values reset."));
          # endif // ifdef P126_DEBUG_LOG
        }
        success = true;
      }
    } else if (subcommand.equals(F("setchipcount"))) {
      if ((event->Par2 >= 1) && (event->Par2 <= P126_MAX_CHIP_COUNT)) {
        P126_CONFIG_CHIP_COUNT = event->Par2;
        _chipCount             = event->Par2;
        shift->setSize(P126_CONFIG_CHIP_COUNT);
        success = true;
      }
    # ifdef P126_SHOW_VALUES
    } else if (subcommand.equals(F("sethexbin"))) {
      if ((event->Par2 == 0) || (event->Par2 == 1)) {
        uint32_t lSettings = P126_CONFIG_FLAGS;
        bitWrite(lSettings, P126_FLAGS_VALUES_DISPLAY, event->Par2 == 1);
        P126_CONFIG_FLAGS = lSettings;
        success           = true;
      }
    # endif // ifdef P126_SHOW_VALUES
    }
    # ifdef P126_DEBUG_LOG

    if (success) {
      addLog(LOG_LEVEL_DEBUG, string);
    }
    # endif // ifdef P126_DEBUG_LOG
  }
  return success;
}

#endif // ifdef USES_P126

#include "../PluginStructs/P039_data_struct.h"

#ifdef USES_P039

P039_data_struct::P039_data_struct(
      uint16_t       l_conversionResult,
      uint8_t        l_devicefaults,
      unsigned long  l_timer,
      bool           l_sensorFault, 
      bool           l_convReady)
  :  conversionResult(l_conversionResult), deviceFaults(l_devicefaults), timer(l_timer), sensorFault(l_sensorFault), convReady(l_convReady) {}

bool P039_data_struct::begin()
{
  return false;
}

bool P039_data_struct::read()
{
 return false;
}

bool P039_data_struct::write()
{
 return false;
}

#endif // ifdef USES_P039
#include "../PluginStructs/P045_data_struct.h"

#ifdef USES_P045

# define MPU6050_RA_GYRO_CONFIG              0x1B
# define MPU6050_RA_ACCEL_CONFIG             0x1C
# define MPU6050_RA_ACCEL_XOUT_H             0x3B
# define MPU6050_RA_PWR_MGMT_1               0x6B
# define MPU6050_ACONFIG_AFS_SEL_BIT         4
# define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
# define MPU6050_GCONFIG_FS_SEL_BIT          4
# define MPU6050_GCONFIG_FS_SEL_LENGTH       2
# define MPU6050_CLOCK_PLL_XGYRO             0x01
# define MPU6050_GYRO_FS_250                 0x00
# define MPU6050_ACCEL_FS_2                  0x00
# define MPU6050_PWR1_SLEEP_BIT              6
# define MPU6050_PWR1_CLKSEL_BIT             2
# define MPU6050_PWR1_CLKSEL_LENGTH          3

P045_data_struct::P045_data_struct(uint8_t i2c_addr) : i2cAddress(i2c_addr)
{
  // Initialize the MPU6050, for details look at the MPU6050 library: MPU6050::Initialize
  writeBits(MPU6050_RA_PWR_MGMT_1,   MPU6050_PWR1_CLKSEL_BIT,     MPU6050_PWR1_CLKSEL_LENGTH,     MPU6050_CLOCK_PLL_XGYRO);
  writeBits(MPU6050_RA_GYRO_CONFIG,  MPU6050_GCONFIG_FS_SEL_BIT,  MPU6050_GCONFIG_FS_SEL_LENGTH,  MPU6050_GYRO_FS_250);
  writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
  writeBits(MPU6050_RA_PWR_MGMT_1,   MPU6050_PWR1_SLEEP_BIT,      1,                              0);

  // Read the MPU6050 once to clear out zeros (1st time reading MPU6050 returns all 0s)
  int16_t ax, ay, az, gx, gy, gz;

  getRaw6AxisMotion(&ax, &ay, &az, &gx, &gy, &gz);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 5; ++j) {
      _axis[i][j] = 0;
    }
  }
}

void P045_data_struct::loop()
{
  // Read the sensorvalues, we run this bit every 1/10th of a second
  getRaw6AxisMotion(&_axis[0][3],
                    &_axis[1][3],
                    &_axis[2][3],
                    &_axis[0][4],
                    &_axis[1][4],
                    &_axis[2][4]);

  // Set the minimum and maximum value for each axis a-value, overwrite previous values if smaller/larger
  trackMinMax(_axis[0][3], &_axis[0][0], &_axis[0][1]);
  trackMinMax(_axis[1][3], &_axis[1][0], &_axis[1][1]);
  trackMinMax(_axis[2][3], &_axis[2][0], &_axis[2][1]);

  //          ^ current value @ 3   ^ min val @ 0         ^ max val @ 1

  /*
     // Uncomment this block if you want to debug your MPU6050, but be prepared for a log overload
     String log = F("MPU6050 : axis values: ");

     log += _axis[0][3];
     log += F(", ");
     log += _axis[1][3];
     log += F(", ");
     log += _axis[2][3];
     log += F(", g values: ");
     log += _axis[0][4];
     log += F(", ");
     log += _axis[1][4];
     log += F(", ");
     log += _axis[2][4];
     addLog(LOG_LEVEL_INFO, log);
   */

  // Run this bit every 5 seconds per deviceaddress (not per instance)
  if (timeOutReached(_timer + 5000))
  {
    _timer = millis();

    // Determine the maximum measured range of each axis
    for (uint8_t i = 0; i < 3; i++) {
      _axis[i][2] = abs(_axis[i][1] - _axis[i][0]);
      _axis[i][0] = _axis[i][3];
      _axis[i][1] = _axis[i][3];
    }
  }
}

void P045_data_struct::trackMinMax(int16_t current, int16_t *min, int16_t *max)
{
  // From nodemcu-laundry.ino by Nolan Gilley
  if (current > *max)
  {
    *max = current;
  }
  else if (current < *min)
  {
    *min = current;
  }
}

void P045_data_struct::getRaw6AxisMotion(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
  // From I2Cdev::readBytes and MPU6050::getMotion6, both by Jeff Rowberg
  uint8_t buffer[14];
  uint8_t count = 0;

  I2C_write8(i2cAddress, MPU6050_RA_ACCEL_XOUT_H);
  Wire.requestFrom(i2cAddress, (uint8_t)14);

  for (; Wire.available(); count++) {
    buffer[count] = Wire.read();
  }
  *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
  *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
  *az = (((int16_t)buffer[4]) << 8) | buffer[5];
  *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
  *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
  *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

void P045_data_struct::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
  // From I2Cdev::writeBits by Jeff Rowberg
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value
  bool is_ok = true;
  uint8_t b  = I2C_read8_reg(i2cAddress, regAddr, &is_ok);

  if (is_ok) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data  &= mask;                    // zero all non-important bits in data
    b     &= ~(mask);                 // zero all important bits in existing byte
    b     |= data;                    // combine data with existing byte
    I2C_write8_reg(i2cAddress, regAddr, b);
  }
}

#endif // ifdef USES_P045

#include "../PluginStructs/P089_data_struct.h"

#if defined(USES_P089) && defined(ESP8266)


P089_data_struct::P089_data_struct() {
  destIPAddress.addr = 0;
  idseq              = 0;

  if (nullptr == P089_data) {
    P089_data = new (std::nothrow) P089_icmp_pcb();

    if (P089_data != nullptr) {
      P089_data->m_IcmpPCB = raw_new(IP_PROTO_ICMP);
      raw_recv(P089_data->m_IcmpPCB, PingReceiver, nullptr);
      raw_bind(P089_data->m_IcmpPCB, IP_ADDR_ANY);
    }
  } else {
    P089_data->instances++;
  }
}

P089_data_struct::~P089_data_struct() {
  if (P089_data != nullptr) {
    P089_data->instances--;

    if (P089_data->instances == 0) {
      raw_remove(P089_data->m_IcmpPCB);
      delete P089_data;
      P089_data = nullptr;
    }
  }
}

bool P089_data_struct::send_ping(struct EventStruct *event) {
  bool is_failure = false;
  IPAddress ip;

  // Do we have unanswered pings? If we are sending new one, this means old one is lost
  if (destIPAddress.addr != 0) {
    is_failure = true;
  }

  /* This ping lost for sure */
  if (!NetworkConnected()) {
    return true;
  }

  char hostname[PLUGIN_089_HOSTNAME_SIZE];

  LoadCustomTaskSettings(event->TaskIndex, (uint8_t *)&hostname, PLUGIN_089_HOSTNAME_SIZE);

  /* This one lost as well, DNS dead? */
  if (WiFi.hostByName(hostname, ip) == false) {
    return true;
  }
  destIPAddress.addr = ip;

  /* Generate random ID & seq */
  idseq = random(UINT32_MAX);
  u16_t ping_len            = ICMP_PAYLOAD_LEN + sizeof(struct icmp_echo_hdr);
  struct pbuf *packetBuffer = pbuf_alloc(PBUF_IP, ping_len, PBUF_RAM);

  /* Lost for sure, TODO: Might be good to log such failures, this means we are short on ram? */
  if (packetBuffer == nullptr) {
    return true;
  }

  struct icmp_echo_hdr *echoRequestHeader = (struct icmp_echo_hdr *)packetBuffer->payload;

  ICMPH_TYPE_SET(echoRequestHeader, ICMP_ECHO);
  ICMPH_CODE_SET(echoRequestHeader, 0);
  echoRequestHeader->chksum = 0;
  echoRequestHeader->id     = (uint16_t)((idseq & 0xffff0000) >> 16);
  echoRequestHeader->seqno  = (uint16_t)(idseq & 0xffff);
  size_t icmpHeaderLen = sizeof(struct icmp_echo_hdr);
  size_t icmpDataLen   = ping_len - icmpHeaderLen;
  char   dataByte      = 0x61;

  for (size_t i = 0; i < icmpDataLen; i++) {
    ((char *)echoRequestHeader)[icmpHeaderLen + i] = dataByte;
    ++dataByte;

    if (dataByte > 0x77) // 'w' character
    {
      dataByte = 0x61;
    }
  }
  echoRequestHeader->chksum = inet_chksum(echoRequestHeader, ping_len);
  ip_addr_t destIPAddress;

  destIPAddress.addr = ip;
  raw_sendto(P089_data->m_IcmpPCB, packetBuffer, &destIPAddress);

  pbuf_free(packetBuffer);

  return is_failure;
}

uint8_t PingReceiver(void *origin, struct raw_pcb *pcb, struct pbuf *packetBuffer, const ip_addr_t *addr)
{
  if ((packetBuffer == nullptr) || (addr == nullptr)) {
    return 0;
  }

  if (packetBuffer->len < sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr) + ICMP_PAYLOAD_LEN) {
    return 0;
  }

  // TODO: Check some ipv4 header values?
  // struct ip_hdr * ip = (struct ip_hdr *)packetBuffer->payload;

  if (pbuf_header(packetBuffer, -PBUF_IP_HLEN) != 0) {
    return 0;
  }

  // After the IPv4 header, we can access the icmp echo header
  struct icmp_echo_hdr *icmp_hdr = (struct icmp_echo_hdr *)packetBuffer->payload;

  // Is it echo reply?
  if (icmp_hdr->type != 0) {
    pbuf_header(packetBuffer, PBUF_IP_HLEN);
    return 0;
  }

  bool is_found = false;

  for (taskIndex_t index = 0; index < TASKS_MAX; index++) {
    deviceIndex_t deviceIndex = getDeviceIndex_from_TaskIndex(index);

    // Match all ping plugin instances and check them
    if (validDeviceIndex(deviceIndex) && (DeviceIndex_to_Plugin_id[deviceIndex] == PLUGIN_ID_089)) {
      P089_data_struct *P089_taskdata = static_cast<P089_data_struct *>(getPluginTaskData(index));

      if ((P089_taskdata != nullptr) && (icmp_hdr->id == (uint16_t)((P089_taskdata->idseq & 0xffff0000) >> 16)) &&
          (icmp_hdr->seqno == (uint16_t)(P089_taskdata->idseq & 0xffff))) {
        UserVar[index * VARS_PER_TASK]    = 0; // Reset "fails", we got reply
        P089_taskdata->idseq              = 0;
        P089_taskdata->destIPAddress.addr = 0;
        is_found                          = true;
      }
    }
  }

  if (!is_found) {
    pbuf_header(packetBuffer, PBUF_IP_HLEN);
    return 0;
  }

  // Everything fine, release the kraken, ehm, buffer
  pbuf_free(packetBuffer);
  return 1;
}

#endif // if defined(USES_P089) && defined(ESP8266)

#include "../PluginStructs/P003_data_struct.h"


#ifdef USES_P003
P003_data_struct::P003_data_struct(const Internal_GPIO_pulseHelper::pulseCounterConfig& config)
  :
  pulseHelper(config)
{}

P003_data_struct::~P003_data_struct()
{}
#endif
#include "../PluginStructs/P074_data_struct.h"

#ifdef USES_P074


P074_data_struct::P074_data_struct() {
  tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier
                                // (for your use later)
}

// Changing the integration time gives you a longer time over which to sense
// light
// longer timelines are slower, but are good in very low light situtations!
void P074_data_struct::setIntegrationTime(int time) {
  switch (time) {
    default:
    case 0: tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS); break;
    case 1: tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS); break;
    case 2: tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS); break;
    case 3: tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS); break;
    case 4: tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS); break;
    case 5: tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS); break;
  }
}

// You can change the gain on the fly, to adapt to brighter/dimmer light
// situations
void P074_data_struct::setGain(int gain) {
  switch (gain) {
    default:
    case 0: tsl.setGain(TSL2591_GAIN_LOW);  break; // 1x gain (bright light)
    case 1: tsl.setGain(TSL2591_GAIN_MED);  break; // 25x (Medium)
    case 2: tsl.setGain(TSL2591_GAIN_HIGH); break; // 428x (High)
    case 3: tsl.setGain(TSL2591_GAIN_MAX);  break; // 9876x (Max)
  }
}

// Return true when value is present.
bool P074_data_struct::getFullLuminosity(uint32_t& value) {
  value = 0;

  if (newValuePresent) {
    // don't try to read a new value until the last one was processed.
    return false;
  }

  if (!integrationActive) {
    if (startIntegrationNeeded) {
      // Fix to re-set the gain/timing before every read.
      // See https://github.com/letscontrolit/ESPEasy/issues/3347
      if (tsl.begin()) {
        tsl.enable();
        integrationStart       = millis();
        duration               = 0;
        integrationActive      = true;
        startIntegrationNeeded = false;
      }
    }
    return false; // Started integration, so no value possible yet.
  }
  bool finished = false;

  value    = tsl.getFullLuminosity(finished);
  duration = timePassedSince(integrationStart);

  if (finished) {
    integrationActive = false;
    integrationStart  = 0;
    newValuePresent   = true;
  } else {
    if (duration > 1000) {
      // Max integration time is 600 msec, so if we still have no value, reset the current state
      integrationStart       = 0;
      integrationActive      = false;
      newValuePresent        = false;
      startIntegrationNeeded = true; // Apparently a value was needed
    }
  }

  if (!integrationActive) {
    tsl.disable();
  }
  return finished;
}

#endif // ifdef USES_P074

#include "../PluginStructs/P012_data_struct.h"

// Needed also here for PlatformIO's library finder as the .h file
// is in a directory which is excluded in the src_filter
#include <LiquidCrystal_I2C.h>

#ifdef USES_P012

P012_data_struct::P012_data_struct(uint8_t addr,
                                   uint8_t lcd_size,
                                   uint8_t mode,
                                   uint8_t    timer) :
  lcd(addr, 20, 4),
  Plugin_012_mode(mode),
  displayTimer(timer) {
  switch (lcd_size)
  {
    case 1:
      Plugin_012_rows = 2;
      Plugin_012_cols = 16;
      break;
    case 2:
      Plugin_012_rows = 4;
      Plugin_012_cols = 20;
      break;

    default:
      Plugin_012_rows = 2;
      Plugin_012_cols = 16;
      break;
  }


  // Setup LCD display
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.print(F("ESP Easy"));
  createCustomChars();
}

void P012_data_struct::setBacklightTimer(uint8_t timer) {
  displayTimer = timer;
  lcd.backlight();
}

void P012_data_struct::checkTimer() {
  if (displayTimer > 0)
  {
    displayTimer--;

    if (displayTimer == 0) {
      lcd.noBacklight();
    }
  }
}

void P012_data_struct::lcdWrite(const String& text, uint8_t col, uint8_t row) {
  // clear line before writing new string
  if (Plugin_012_mode == 2) {
    lcd.setCursor(col, row);

    for (uint8_t i = col; i < Plugin_012_cols; i++) {
      lcd.print(' ');
    }
  }

  lcd.setCursor(col, row);

  if ((Plugin_012_mode == 1) || (Plugin_012_mode == 2)) {
    lcd.setCursor(col, row);

    for (uint8_t i = 0; i < Plugin_012_cols - col; i++) {
      if (text[i]) {
        lcd.print(text[i]);
      }
    }
  }

  // message exceeding cols will continue to next line
  else {
    // Fix Weird (native) lcd display behaviour that split long string into row 1,3,2,4, instead of 1,2,3,4
    bool stillProcessing = 1;
    uint8_t charCount       = 1;

    while (stillProcessing) {
      if (++col > Plugin_012_cols) { // have we printed 20 characters yet (+1 for the logic)
        row += 1;
        lcd.setCursor(0, row);       // move cursor down
        col = 1;
      }

      // dont print if "lower" than the lcd
      if (row < Plugin_012_rows) {
        lcd.print(text[charCount - 1]);
      }

      if (!text[charCount]) { // no more chars to process?
        stillProcessing = 0;
      }
      charCount += 1;
    }

    // lcd.print(text.c_str());
    // end fix
  }
}

// Perform some specific changes for LCD display
// https://www.letscontrolit.com/forum/viewtopic.php?t=2368
String P012_data_struct::P012_parseTemplate(String& tmpString, uint8_t lineSize) {
  String result            = parseTemplate_padded(tmpString, lineSize);
  const char degree[3]     = { 0xc2, 0xb0, 0 }; // Unicode degree symbol
  const char degree_lcd[2] = { 0xdf, 0 };       // P012_LCD degree symbol

  result.replace(degree, degree_lcd);

  char unicodePrefix = 0xc4;

# ifdef USES_P012_POLISH_CHARS

  if (result.indexOf(unicodePrefix) != -1) {
    const char znak_a_uni[3] = { 0xc4, 0x85, 0 }; // Unicode znak a
    const char znak_a_lcd[2] = { 0x05, 0 };       // P012_LCD znak a
    result.replace(znak_a_uni, znak_a_lcd);

    const char znak_A_uni[3] = { 0xc4, 0x84, 0 }; // Unicode znak A
    result.replace(znak_A_uni, znak_a_lcd);

    const char znak_c_uni[3] = { 0xc4, 0x87, 0 }; // Unicode znak c
    const char znak_c_lcd[2] = { 0x03, 0 };       // P012_LCD znak c
    result.replace(znak_c_uni, znak_c_lcd);

    const char znak_C_uni[3] = { 0xc4, 0x86, 0 }; // Unicode znak C
    result.replace(znak_C_uni, znak_c_lcd);

    const char znak_e_uni[3] = { 0xc4, 0x99, 0 }; // Unicode znak e
    const char znak_e_lcd[2] = { 0x02, 0 };       // P012_LCD znak e
    result.replace(znak_e_uni, znak_e_lcd);

    const char znak_E_uni[3] = { 0xc4, 0x98, 0 }; // Unicode znak E
    result.replace(znak_E_uni, znak_e_lcd);
  }

  unicodePrefix = 0xc5;

  if (result.indexOf(unicodePrefix) != -1) {
    const char znak_l_uni[3] = { 0xc5, 0x82, 0 };  // Unicode znak l
    const char znak_l_lcd[2] = { 0x01, 0 };        // P012_LCD znak l
    result.replace(znak_l_uni, znak_l_lcd);

    const char znak_L_uni[3] = { 0xc5, 0x81, 0 };  // Unicode znak L
    result.replace(znak_L_uni, znak_l_lcd);

    const char znak_n_uni[3] = { 0xc5, 0x84, 0 };  // Unicode znak n
    const char znak_n_lcd[2] = { 0x04, 0 };        // P012_LCD znak n
    result.replace(znak_n_uni, znak_n_lcd);

    const char znak_N_uni[3] = { 0xc5, 0x83, 0 };  // Unicode znak N
    result.replace(znak_N_uni, znak_n_lcd);

    const char znak_s_uni[3] = { 0xc5, 0x9b, 0 };  // Unicode znak s
    const char znak_s_lcd[2] = { 0x06, 0 };        // P012_LCD znak s
    result.replace(znak_s_uni, znak_s_lcd);

    const char znak_S_uni[3] = { 0xc5, 0x9a, 0 };  // Unicode znak S
    result.replace(znak_S_uni, znak_s_lcd);

    const char znak_z1_uni[3] = { 0xc5, 0xba, 0 }; // Unicode znak z z kreska
    const char znak_z1_lcd[2] = { 0x07, 0 };       // P012_LCD znak z z kropka
    result.replace(znak_z1_uni, znak_z1_lcd);

    const char znak_Z1_uni[3] = { 0xc5, 0xb9, 0 }; // Unicode znak Z z kreska
    result.replace(znak_Z1_uni, znak_z1_lcd);

    const char znak_z2_uni[3] = { 0xc5, 0xbc, 0 }; // Unicode znak z z kropka
    const char znak_z2_lcd[2] = { 0x07, 0 };       // P012_LCD znak z z kropka
    result.replace(znak_z2_uni, znak_z2_lcd);

    const char znak_Z2_uni[3] = { 0xc5, 0xbb, 0 }; // Unicode znak Z z kropka
    result.replace(znak_Z2_uni, znak_z2_lcd);
  }

  unicodePrefix = 0xc3;

  if (result.indexOf(unicodePrefix) != -1) {
    const char znak_o_uni[3] = { 0xc3, 0xB3, 0 }; // Unicode znak o
    const char znak_o_lcd[2] = { 0x08, 0 };       // P012_LCD znak o
    result.replace(znak_o_uni, znak_o_lcd);

    const char znak_O_uni[3] = { 0xc3, 0x93, 0 }; // Unicode znak O
    result.replace(znak_O_uni, znak_o_lcd);
  }
# endif // USES_P012_POLISH_CHARS

  unicodePrefix = 0xc3;

  if (result.indexOf(unicodePrefix) != -1) {
    // See: https://github.com/letscontrolit/ESPEasy/issues/2081

    const char umlautAE_uni[3] = { 0xc3, 0x84, 0 };  // Unicode Umlaute AE
    const char umlautAE_lcd[2] = { 0xe1, 0 };        // P012_LCD Umlaute
    result.replace(umlautAE_uni,  umlautAE_lcd);

    const char umlaut_ae_uni[3] = { 0xc3, 0xa4, 0 }; // Unicode Umlaute ae
    result.replace(umlaut_ae_uni, umlautAE_lcd);

    const char umlautOE_uni[3] = { 0xc3, 0x96, 0 };  // Unicode Umlaute OE
    const char umlautOE_lcd[2] = { 0xef, 0 };        // P012_LCD Umlaute
    result.replace(umlautOE_uni,  umlautOE_lcd);

    const char umlaut_oe_uni[3] = { 0xc3, 0xb6, 0 }; // Unicode Umlaute oe
    result.replace(umlaut_oe_uni, umlautOE_lcd);

    const char umlautUE_uni[3] = { 0xc3, 0x9c, 0 };  // Unicode Umlaute UE
    const char umlautUE_lcd[2] = { 0xf5, 0 };        // P012_LCD Umlaute
    result.replace(umlautUE_uni,  umlautUE_lcd);

    const char umlaut_ue_uni[3] = { 0xc3, 0xbc, 0 }; // Unicode Umlaute ue
    result.replace(umlaut_ue_uni, umlautUE_lcd);

    const char umlaut_sz_uni[3] = { 0xc3, 0x9f, 0 }; // Unicode Umlaute sz
    const char umlaut_sz_lcd[2] = { 0xe2, 0 };       // P012_LCD Umlaute
    result.replace(umlaut_sz_uni, umlaut_sz_lcd);
  }
  return result;
}

void P012_data_struct::createCustomChars() {
# ifdef USES_P012_POLISH_CHARS
/*
  static const char LETTER_null[8] PROGMEM = { // spacja
    0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000
  };
*/
  static const char LETTER_a[8] PROGMEM = {    // a
    0b00000, 0b00000, 0b01110, 0b00001, 0b01111, 0b10001, 0b01111, 0b00010
  };
  static const char LETTER_c[8] PROGMEM = {    // c
    0b00010, 0b00100, 0b01110, 0b10000, 0b10000, 0b10001, 0b01110, 0b00000
  };
  static const char LETTER_e[8] PROGMEM = {    // e
    0b00000, 0b00000, 0b01110, 0b10001, 0b11111, 0b10000, 0b01110, 0b00010
  };
  static const char LETTER_l[8] PROGMEM = {    // l
    0b01100, 0b00100, 0b00101, 0b00110, 0b01100, 0b00100, 0b01110, 0b00000
  };
  static const char LETTER_n[8] PROGMEM = {    // n
    0b00010, 0b00100, 0b10110, 0b11001, 0b10001, 0b10001, 0b10001, 0b00000
  };
  static const char LETTER_o[8] PROGMEM = {    // o
    0b00010, 0b00100, 0b01110, 0b10001, 0b10001, 0b10001, 0b01110, 0b00000
  };
  static const char LETTER_s[8] PROGMEM = {    // s
    0b00010, 0b00100, 0b01110, 0b10000, 0b01110, 0b00001, 0b11110, 0b00000
  };
  /*
  static const char LETTER_z1[8] PROGMEM = {   // z z kreska
    0b00010, 0b00100, 0b11111, 0b00010, 0b00100, 0b01000, 0b11111, 0b00000
  };
  */
  static const char LETTER_z2[8] PROGMEM = {   // z z kropka
    0b00100, 0b00000, 0b11111, 0b00010, 0b00100, 0b01000, 0b11111, 0b00000
  };
  lcd.createChar(0, LETTER_o);  // probably defected memory cell
  lcd.createChar(1, LETTER_l);
  lcd.createChar(2, LETTER_e);
  lcd.createChar(3, LETTER_c);
  lcd.createChar(4, LETTER_n);
  lcd.createChar(5, LETTER_a);
  lcd.createChar(6, LETTER_s);
  lcd.createChar(7, LETTER_z2);
  lcd.createChar(8, LETTER_o);
# endif // ifdef USES_P012_POLISH_CHARS
}

#endif  // ifdef USES_P012

#include "../PluginStructs/P060_data_struct.h"


#ifdef USES_P060

P060_data_struct::P060_data_struct(uint8_t i2c_addr) : i2cAddress(i2c_addr) {}

void P060_data_struct::overSampleRead()
{
  OversamplingValue += readMCP3221();
  OversamplingCount++;
}

float P060_data_struct::getValue()
{
  float value;

  if (OversamplingCount > 0)
  {
    value             = static_cast<float>(OversamplingValue) / OversamplingCount;
    OversamplingValue = 0;
    OversamplingCount = 0;
  } else {
    value = readMCP3221();
  }
  return value;
}

uint16_t P060_data_struct::readMCP3221()
{
  uint16_t value;

  Wire.requestFrom(i2cAddress, (uint8_t)2);

  if (Wire.available() == 2)
  {
    value = (Wire.read() << 8) | Wire.read();
  }
  else {
    value = 9999;
  }

  return value;
}

#endif // ifdef USES_P060

#include "../PluginStructs/P027_data_struct.h"

#ifdef USES_P027


# define INA219_READ                            (0x01)
# define INA219_REG_CONFIG                      (0x00)
# define INA219_CONFIG_RESET                    (0x8000) // Reset Bit

# define INA219_CONFIG_BVOLTAGERANGE_MASK       (0x2000) // Bus Voltage Range Mask
# define INA219_CONFIG_BVOLTAGERANGE_16V        (0x0000) // 0-16V Range
# define INA219_CONFIG_BVOLTAGERANGE_32V        (0x2000) // 0-32V Range

# define INA219_CONFIG_GAIN_MASK                (0x1800) // Gain Mask
# define INA219_CONFIG_GAIN_1_40MV              (0x0000) // Gain 1, 40mV Range
# define INA219_CONFIG_GAIN_2_80MV              (0x0800) // Gain 2, 80mV Range
# define INA219_CONFIG_GAIN_4_160MV             (0x1000) // Gain 4, 160mV Range
# define INA219_CONFIG_GAIN_8_320MV             (0x1800) // Gain 8, 320mV Range

# define INA219_CONFIG_BADCRES_MASK             (0x0780) // Bus ADC Resolution Mask
# define INA219_CONFIG_BADCRES_9BIT             (0x0080) // 9-bit bus res = 0..511
# define INA219_CONFIG_BADCRES_10BIT            (0x0100) // 10-bit bus res = 0..1023
# define INA219_CONFIG_BADCRES_11BIT            (0x0200) // 11-bit bus res = 0..2047
# define INA219_CONFIG_BADCRES_12BIT            (0x0400) // 12-bit bus res = 0..4097

# define INA219_CONFIG_SADCRES_MASK             (0x0078) // Shunt ADC Resolution and Averaging Mask
# define INA219_CONFIG_SADCRES_9BIT_1S_84US     (0x0000) // 1 x 9-bit shunt sample
# define INA219_CONFIG_SADCRES_10BIT_1S_148US   (0x0008) // 1 x 10-bit shunt sample
# define INA219_CONFIG_SADCRES_11BIT_1S_276US   (0x0010) // 1 x 11-bit shunt sample
# define INA219_CONFIG_SADCRES_12BIT_1S_532US   (0x0018) // 1 x 12-bit shunt sample
# define INA219_CONFIG_SADCRES_12BIT_2S_1060US  (0x0048) // 2 x 12-bit shunt samples averaged together
# define INA219_CONFIG_SADCRES_12BIT_4S_2130US  (0x0050) // 4 x 12-bit shunt samples averaged together
# define INA219_CONFIG_SADCRES_12BIT_8S_4260US  (0x0058) // 8 x 12-bit shunt samples averaged together
# define INA219_CONFIG_SADCRES_12BIT_16S_8510US (0x0060) // 16 x 12-bit shunt samples averaged together
# define INA219_CONFIG_SADCRES_12BIT_32S_17MS   (0x0068) // 32 x 12-bit shunt samples averaged together
# define INA219_CONFIG_SADCRES_12BIT_64S_34MS   (0x0070) // 64 x 12-bit shunt samples averaged together
# define INA219_CONFIG_SADCRES_12BIT_128S_69MS  (0x0078) // 128 x 12-bit shunt samples averaged together

# define INA219_CONFIG_MODE_MASK                (0x0007) // Operating Mode Mask
# define INA219_CONFIG_MODE_POWERDOWN           (0x0000)
# define INA219_CONFIG_MODE_SVOLT_TRIGGERED     (0x0001)
# define INA219_CONFIG_MODE_BVOLT_TRIGGERED     (0x0002)
# define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED (0x0003)
# define INA219_CONFIG_MODE_ADCOFF              (0x0004)
# define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    (0x0005)
# define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    (0x0006)
# define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)

# define INA219_REG_SHUNTVOLTAGE                (0x01)
# define INA219_REG_BUSVOLTAGE                  (0x02)
# define INA219_REG_POWER                       (0x03)
# define INA219_REG_CURRENT                     (0x04)
# define INA219_REG_CALIBRATION                 (0x05)


P027_data_struct::P027_data_struct(uint8_t i2c_addr) : i2caddr(i2c_addr) {}

void P027_data_struct::setCalibration_32V_2A() {
  calValue = 4027;

  // Set multipliers to convert raw current/power values
  currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)

  // Set Calibration register to 'Cal' calculated above
  wireWriteRegister(INA219_REG_CALIBRATION, calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  wireWriteRegister(INA219_REG_CONFIG, config);
}

void P027_data_struct::setCalibration_32V_1A() {
  calValue = 10240;

  // Set multipliers to convert raw current/power values
  currentDivider_mA = 25; // Current LSB = 40uA per bit (1000/40 = 25)

  // Set Calibration register to 'Cal' calculated above
  wireWriteRegister(INA219_REG_CALIBRATION, calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  wireWriteRegister(INA219_REG_CONFIG, config);
}

void P027_data_struct::setCalibration_16V_400mA() {
  calValue = 8192;

  // Set multipliers to convert raw current/power values
  currentDivider_mA = 20; // Current LSB = 50uA per bit (1000/50 = 20)

  // Set Calibration register to 'Cal' calculated above
  wireWriteRegister(INA219_REG_CALIBRATION, calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  wireWriteRegister(INA219_REG_CONFIG, config);
}

int16_t P027_data_struct::getBusVoltage_raw() {
  uint16_t value;

  wireReadRegister(INA219_REG_BUSVOLTAGE, &value);

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)((value >> 3) * 4);
}

int16_t P027_data_struct::getShuntVoltage_raw() {
  uint16_t value;

  wireReadRegister(INA219_REG_SHUNTVOLTAGE, &value);
  return (int16_t)value;
}

int16_t P027_data_struct::getCurrent_raw() {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  wireWriteRegister(INA219_REG_CALIBRATION, calValue);

  // Now we can safely read the CURRENT register!
  wireReadRegister(INA219_REG_CURRENT, &value);

  return (int16_t)value;
}

float P027_data_struct::getShuntVoltage_mV() {
  int16_t value;

  value = P027_data_struct::getShuntVoltage_raw();
  return value * 0.01f;
}

float P027_data_struct::getBusVoltage_V() {
  int16_t value = getBusVoltage_raw();

  return value * 0.001f;
}

float P027_data_struct::getCurrent_mA() {
  float valueDec = getCurrent_raw();

  valueDec /= currentDivider_mA;
  return valueDec;
}

void P027_data_struct::wireWriteRegister(uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(reg);                 // Register
  Wire.write((value >> 8) & 0xFF); // Upper 8-bits
  Wire.write(value & 0xFF);        // Lower 8-bits
  Wire.endTransmission();
}

void P027_data_struct::wireReadRegister(uint8_t reg, uint16_t *value)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(reg); // Register
  Wire.endTransmission();

  delay(1);        // Max 12-bit conversion time is 586us per sample

  Wire.requestFrom(i2caddr, (uint8_t)2);

  // Shift values to create properly formed integer
  *value = ((Wire.read() << 8) | Wire.read());
}

#endif // ifdef USES_P027

#include "../PluginStructs/P121_data_struct.h"

#ifdef USES_P121

// Needed also here for PlatformIO's library finder as the .h file
// is in a directory which is excluded in the src_filter
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

bool P121_data_struct::begin(int taskid)
{
  if (!initialized)
  {
    mag = Adafruit_HMC5883_Unified(taskid);
    initialized = mag.begin();

    if (initialized)
    {
      // Set up oversampling and filter initialization
      sensor_t sensor;
      mag.getSensor(&sensor);
      #ifndef BUILD_NO_DEBUG
      if (loglevelActiveFor(LOG_LEVEL_DEBUG))
      {
        String log = F("HMC5883_U: Sensor:       ");
        log += F(sensor.name);
        log += F(" Driver Ver:   "); 
        log += sensor.version;
        log += F(" Unique ID:    "); 
        log += sensor.sensor_id;
        log += F(" Max Value:    "); 
        log += toString(sensor.max_value);
        log += F(" Min Value:    ");
        log += toString(sensor.min_value);
        log += F(" Resolution:   "); 
        log += toString(sensor.resolution);
        addLogMove(LOG_LEVEL_DEBUG, log);
      }
      #endif
    }
  }

  return initialized;
}

#endif // ifdef USES_P121

#include "../PluginStructs/P070_data_struct.h"

#ifdef USES_P070


P070_data_struct::~P070_data_struct() {
  reset();
}

void P070_data_struct::reset() {
  if (Plugin_070_pixels != nullptr) {
    delete Plugin_070_pixels;
    Plugin_070_pixels = nullptr;
  }
}

void P070_data_struct::init(struct EventStruct *event) {
  if (!Plugin_070_pixels)
  {
    Plugin_070_pixels = new (std::nothrow) Adafruit_NeoPixel(NUMBER_LEDS, CONFIG_PIN1, NEO_GRB + NEO_KHZ800);

    if (Plugin_070_pixels == nullptr) {
      return;
    }
    Plugin_070_pixels->begin(); // This initializes the NeoPixel library.
  }
  set(event);
}

void P070_data_struct::set(struct EventStruct *event) {
  display_enabled       = PCONFIG(0);
  brightness            = PCONFIG(1);
  brightness_hour_marks = PCONFIG(2);
  offset_12h_mark       = PCONFIG(3);
  thick_12_mark         = PCONFIG(4);
}

void P070_data_struct::Clock_update()
{
  clearClock();              // turn off the LEDs

  if (display_enabled > 0) { // if the display is enabled, calculate the LEDs to turn on
    int Hours   = node_time.hour();
    int Minutes = node_time.minute();
    int Seconds = node_time.second();
    timeToStrip(Hours, Minutes, Seconds);
  }
  Plugin_070_pixels->show(); // This sends the updated pixel color to the hardware.
}

void P070_data_struct::calculateMarks()
{ // generate a list of the LEDs that have hour marks
  for (int i = 0; i < 12; i++) {
    marks[i] = 5 * i + (offset_12h_mark % 5);
  }

  if (thick_12_mark) {
    if (offset_12h_mark == 0) {
      marks[12] = 1;
      marks[13] = 59;
    }
    else if (offset_12h_mark == 59) {
      marks[12] = 0;
      marks[13] = 58;
    }
    else {
      marks[12] = offset_12h_mark + 1;
      marks[13] = offset_12h_mark - 1;
    }
  }
  else {
    marks[12] = 255;
    marks[13] = 255;
  }
}

void P070_data_struct::clearClock() {
  for (int i = 0; i < NUMBER_LEDS; i++) {
    Plugin_070_pixels->setPixelColor(i, Plugin_070_pixels->Color(0, 0, 0));
  }
}

void P070_data_struct::timeToStrip(int hours, int minutes, int seconds) {
  if (hours > 11) { hours = hours - 12; }
  hours = (hours * 5) + (minutes / 12) + offset_12h_mark; // make the hour hand move each 12 minutes and apply the offset

  if (hours > 59) { hours = hours - 60; }
  minutes = minutes + offset_12h_mark;                    // apply offset to minutes

  if (minutes > 59) { minutes = minutes - 60; }
  seconds = seconds + offset_12h_mark;                    // apply offset to seconds

  if (seconds > 59) { seconds = seconds - 60; }

  for (int i = 0; i < 14; i++) {                                                                      // set the hour marks as white;
    if ((marks[i] != hours) && (marks[i] != minutes) && (marks[i] != seconds) && (marks[i] != 255)) { // do not draw a mark there is a clock
                                                                                                      // hand in that position
      Plugin_070_pixels->setPixelColor(marks[i],
                                       Plugin_070_pixels->Color(brightness_hour_marks, brightness_hour_marks, brightness_hour_marks));
    }
  }
  uint32_t currentColor;
  uint8_t  r_val, g_val;                  // , b_val;

  for (int i = 0; i < NUMBER_LEDS; i++) { // draw the clock hands, adding the colors together
    if (i == hours) {                     // hours hand is RED
      Plugin_070_pixels->setPixelColor(i, Plugin_070_pixels->Color(brightness, 0, 0));
    }

    if (i == minutes) { // minutes hand is GREEN
      currentColor = Plugin_070_pixels->getPixelColor(i);
      r_val        = (uint8_t)(currentColor >> 16);
      Plugin_070_pixels->setPixelColor(i, Plugin_070_pixels->Color(r_val, brightness, 0));
    }

    if (i == seconds) { // seconds hand is BLUE
      currentColor = Plugin_070_pixels->getPixelColor(i);
      r_val        = (uint8_t)(currentColor >> 16);
      g_val        = (uint8_t)(currentColor >>  8);
      Plugin_070_pixels->setPixelColor(i, Plugin_070_pixels->Color(r_val, g_val, brightness));
    }
  }
}

#endif // ifdef USES_P070

#include "../PluginStructs/P044_data_struct.h"

#ifdef USES_P044

#include "../ESPEasyCore/Serial.h"
#include "../ESPEasyCore/ESPEasyNetwork.h"

#include "../Globals/EventQueue.h"

#include "../Helpers/ESPEasy_Storage.h"
#include "../Helpers/Misc.h"

#define P044_RX_WAIT              PCONFIG(0)


P044_Task::P044_Task() {
  clearBuffer();
}

P044_Task::~P044_Task() {
  stopServer();
  serialEnd();
}

bool P044_Task::serverActive(WiFiServer *server) {
#if defined(ESP8266)
  return nullptr != server && server->status() != CLOSED;
#elif defined(ESP32)
  return nullptr != server && *server;
#endif // if defined(ESP8266)
}

void P044_Task::startServer(uint16_t portnumber) {
  if ((gatewayPort == portnumber) && serverActive(P1GatewayServer)) {
    // server is already listening on this port
    return;
  }
  stopServer();
  gatewayPort     = portnumber;
  P1GatewayServer = new (std::nothrow) WiFiServer(portnumber);

  if ((nullptr != P1GatewayServer) && NetworkConnected()) {
    P1GatewayServer->begin();

    if (serverActive(P1GatewayServer)) {
      addLog(LOG_LEVEL_INFO, String(F("P1   : WiFi server started at port ")) + portnumber);
    } else {
      addLog(LOG_LEVEL_ERROR, String(F("P1   : WiFi server start failed at port ")) +
             portnumber + String(F(", retrying...")));
    }
  }
}

void P044_Task::checkServer() {
  if ((nullptr != P1GatewayServer) && !serverActive(P1GatewayServer) && NetworkConnected()) {
    P1GatewayServer->close();
    P1GatewayServer->begin();

    if (serverActive(P1GatewayServer)) {
      addLog(LOG_LEVEL_INFO, F("P1   : WiFi server started"));
    }
  }
}

void P044_Task::stopServer() {
  if (nullptr != P1GatewayServer) {
    if (P1GatewayClient) { P1GatewayClient.stop(); }
    clientConnected = false;
    P1GatewayServer->close();
    addLog(LOG_LEVEL_INFO, F("P1   : WiFi server closed"));
    delete P1GatewayServer;
    P1GatewayServer = nullptr;
  }
}

bool P044_Task::hasClientConnected() {
  if ((nullptr != P1GatewayServer) && P1GatewayServer->hasClient())
  {
    if (P1GatewayClient) { P1GatewayClient.stop(); }
    P1GatewayClient = P1GatewayServer->available();
    P1GatewayClient.setTimeout(CONTROLLER_CLIENTTIMEOUT_DFLT);
    addLog(LOG_LEVEL_INFO, F("P1   : Client connected!"));
  }

  if (P1GatewayClient.connected())
  {
    clientConnected = true;
  }
  else
  {
    if (clientConnected) // there was a client connected before...
    {
      clientConnected = false;
      addLog(LOG_LEVEL_INFO, F("P1   : Client disconnected!"));
    }
  }
  return clientConnected;
}

void P044_Task::discardClientIn() {
  // flush all data received from the WiFi gateway
  // as a P1 meter does not receive data
  while (P1GatewayClient.available()) {
    P1GatewayClient.read();
  }
}

void P044_Task::blinkLED() {
  blinkLEDStartTime = millis();
  digitalWrite(P044_STATUS_LED, 1);
}

void P044_Task::checkBlinkLED() {
  if ((blinkLEDStartTime > 0) && (timePassedSince(blinkLEDStartTime) >= 500)) {
    digitalWrite(P044_STATUS_LED, 0);
    blinkLEDStartTime = 0;
  }
}

void P044_Task::clearBuffer() {
  if (serial_buffer.length() > maxMessageSize) {
    maxMessageSize = _min(serial_buffer.length(), P044_DATAGRAM_MAX_SIZE);
  }

  serial_buffer = String();
  serial_buffer.reserve(maxMessageSize);
}

void P044_Task::addChar(char ch) {
  serial_buffer += ch;
}

/*  checkDatagram
    checks whether the P044_CHECKSUM of the data received from P1 matches the P044_CHECKSUM
    attached to the telegram
 */
bool P044_Task::checkDatagram() const {
  int endChar = serial_buffer.length() - 1;

  if (CRCcheck) {
    endChar -= P044_CHECKSUM_LENGTH;
  }

  if ((endChar < 0) || (serial_buffer[0] != P044_DATAGRAM_START_CHAR) ||
      (serial_buffer[endChar] != P044_DATAGRAM_END_CHAR)) { return false; }

  if (!CRCcheck) { return true; }

  const int checksumStartIndex = endChar + 1;

  #ifdef PLUGIN_044_DEBUG
    for (unsigned int cnt = 0; cnt < serial_buffer.length(); ++cnt) {
      serialPrint(serial_buffer.substring(cnt, 1));
    }
  #endif

  // calculate the CRC and check if it equals the hexadecimal one attached to the datagram
  unsigned int crc = CRC16(serial_buffer, checksumStartIndex);
  return strtoul(serial_buffer.substring(checksumStartIndex).c_str(), nullptr, 16) == crc;
}

/*
   CRC16
      based on code written by Jan ten Hove
     https://github.com/jantenhove/P1-Meter-ESP8266
 */
unsigned int P044_Task::CRC16(const String& buf, int len)
{
  unsigned int crc = 0;

  for (int pos = 0; pos < len; pos++)
  {
    crc ^= static_cast<const unsigned int>(buf[pos]); // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {                    // Loop over each bit
      if ((crc & 0x0001) != 0) {                      // If the LSB is set
        crc >>= 1;                                    // Shift right and XOR 0xA001
        crc  ^= 0xA001;
      }
      else {                                          // Else LSB is not set
        crc >>= 1;                                    // Just shift right
      }
    }
  }

  return crc;
}

/*
   validP1char
       Checks if the character is valid as part of the P1 datagram contents and/or checksum.
       Returns false on a datagram start ('/'), end ('!') or invalid character
 */
bool P044_Task::validP1char(char ch) {
  if (isAlphaNumeric(ch))
  {
    return true;
  }

  switch (ch) {
    case '.':
    case ' ':
    case '\\': // Single backslash, but escaped in C++
    case '\r':
    case '\n':
    case '(':
    case ')':
    case '-':
    case '*':
    case ':':
    case '_':
      return true;
  }
  return false;
}

void P044_Task::serialBegin(const ESPEasySerialPort port, int16_t rxPin, int16_t txPin,
                            unsigned long baud, uint8_t config) {
  serialEnd();

  if (rxPin >= 0) {
    P1EasySerial = new (std::nothrow) ESPeasySerial(port, rxPin, txPin);

    if (nullptr != P1EasySerial) {
#if defined(ESP8266)
      P1EasySerial->begin(baud, (SerialConfig)config);
#elif defined(ESP32)
      P1EasySerial->begin(baud, config);
#endif // if defined(ESP8266)
      addLog(LOG_LEVEL_DEBUG, F("P1   : Serial opened"));
    }
  }
  state = ParserState::WAITING;
}

void P044_Task::serialEnd() {
  if (nullptr != P1EasySerial) {
    delete P1EasySerial;
    P1EasySerial = nullptr;
    addLog(LOG_LEVEL_DEBUG, F("P1   : Serial closed"));
  }
}

void P044_Task::handleSerialIn(struct EventStruct *event) {
  if (nullptr == P1EasySerial) { return; }
  int  RXWait  = P044_RX_WAIT;
  bool done    = false;
  int  timeOut = RXWait;

  do {
    if (P1EasySerial->available()) {
      digitalWrite(P044_STATUS_LED, 1);
      done = handleChar(P1EasySerial->read());
      digitalWrite(P044_STATUS_LED, 0);

      if (done) { break; }
      timeOut = RXWait; // if serial received, reset timeout counter
    } else {
      if (timeOut <= 0) { break; }
      delay(1);
      --timeOut;
    }
  } while (true);

  if (done) {
    P1GatewayClient.print(serial_buffer);
    P1GatewayClient.flush();

    addLog(LOG_LEVEL_DEBUG, F("P1   : data send!"));
    blinkLED();

    if (Settings.UseRules)
    {
      LoadTaskSettings(event->TaskIndex);
      String eventString = getTaskDeviceName(event->TaskIndex);
      eventString += F("#Data");
      eventQueue.addMove(std::move(eventString));
    }
  } // done
}

bool P044_Task::handleChar(char ch) {
  if (serial_buffer.length() >= P044_DATAGRAM_MAX_SIZE - 2) { // room for cr/lf
    addLog(LOG_LEVEL_DEBUG, F("P1   : Error: Buffer overflow, discarded input."));
    state = ParserState::WAITING;                             // reset
  }

  bool done    = false;
  bool invalid = false;

  switch (state) {
    case ParserState::WAITING:

      if (ch == P044_DATAGRAM_START_CHAR)  {
        clearBuffer();
        addChar(ch);
        state = ParserState::READING;
      } // else ignore data
      break;
    case ParserState::READING:

      if (validP1char(ch)) {
        addChar(ch);
      } else if (ch == P044_DATAGRAM_END_CHAR) {
        addChar(ch);

        if (CRCcheck) {
          checkI = 0;
          state  = ParserState::CHECKSUM;
        } else {
          done = true;
        }
      } else if (ch == P044_DATAGRAM_START_CHAR) {
        addLog(LOG_LEVEL_DEBUG, F("P1   : Error: Start detected, discarded input."));
        state = ParserState::WAITING; // reset
        return handleChar(ch);
      } else {
        invalid = true;
      }
      break;
    case ParserState::CHECKSUM:

      if (validP1char(ch)) {
        addChar(ch);
        ++checkI;

        if (checkI == P044_CHECKSUM_LENGTH) {
          done = true;
        }
      } else {
        invalid = true;
      }
      break;
  } // switch

  if (invalid) {
    // input is not a datagram char
    addLog(LOG_LEVEL_DEBUG, F("P1   : Error: DATA corrupt, discarded input."));

    #ifdef PLUGIN_044_DEBUG
      serialPrint(F("faulty char>"));
      serialPrint(String(ch));
      serialPrintln("<");
    #endif
    state = ParserState::WAITING; // reset
  }

  if (done) {
    done = checkDatagram();

    if (done) {
      // add the cr/lf pair to the datagram ahead of reading both
      // from serial as the datagram has already been validated
      addChar('\r');
      addChar('\n');
    } else if (CRCcheck) {
      addLog(LOG_LEVEL_DEBUG, F("P1   : Error: Invalid CRC, dropped data"));
    } else {
      addLog(LOG_LEVEL_DEBUG, F("P1   : Error: Invalid datagram, dropped data"));
    }
    state = ParserState::WAITING; // prepare for next one
  }

  return done;
}

void P044_Task::discardSerialIn() {
  if (nullptr != P1EasySerial) {
    while (P1EasySerial->available()) {
      P1EasySerial->read();
    }
  }
  state = ParserState::WAITING;
}

bool P044_Task::isInit() const {
  return nullptr != P1GatewayServer && nullptr != P1EasySerial;
}

#endif

#include "../PluginStructs/P028_data_struct.h"

#ifdef USES_P028

# include "../Helpers/Convert.h"

// It takes at least 1.587 sec for valid measurements to complete.
// The datasheet names this the "T63" moment.
// 1 second = 63% of the time needed to perform a measurement.
# define P028_MEASUREMENT_TIMEOUT 1.587f

P028_data_struct::P028_data_struct(uint8_t addr, float tempOffset) :
  i2cAddress(addr), temp_offset(tempOffset) {}


uint8_t P028_data_struct::get_config_settings() const {
  return sensorID == Unknown_DEVICE ? 0u : 0x28; // Tstandby 62.5ms, filter 4, 3-wire SPI Disable
}

uint8_t P028_data_struct::get_control_settings() const {
  return sensorID == Unknown_DEVICE ? 0u : 0x93; // Oversampling: 8x P, 8x T, normal mode
}

const __FlashStringHelper * P028_data_struct::getDeviceName() const {
  switch (sensorID) {
    case BMP280_DEVICE_SAMPLE1:
    case BMP280_DEVICE_SAMPLE2: return F("BMP280 sample");
    case BMP280_DEVICE:         return F("BMP280");
    case BME280_DEVICE:         return F("BME280");
    default: return F("Unknown");
  }
}

bool P028_data_struct::hasHumidity() const {
  return sensorID == BME280_DEVICE;
}

bool P028_data_struct::initialized() const {
  return state != BMx_Uninitialized;
}

void P028_data_struct::setUninitialized() {
  state = BMx_Uninitialized;
}

bool P028_data_struct::measurementInProgress() const {
  if ((state != BMx_Wait_for_samples) || (last_measurement == 0)) { return false; }

  return !timeOutReached(last_measurement + P028_MEASUREMENT_TIMEOUT);
}

void P028_data_struct::startMeasurement() {
  if (measurementInProgress()) { return; }

  if (!initialized()) {
    if (begin()) {
      state            = BMx_Initialized;
      last_measurement = 0;
    }
  }
  check(); // Check id device is present

  if (state != BMx_Error) {
    // Set the Sensor in sleep to be make sure that the following configs will be stored
    I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONTROL, 0x00);

    if (hasHumidity()) {
      I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONTROLHUMID, BME280_CONTROL_SETTING_HUMIDITY);
    }
    I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONFIG,  get_config_settings());
    I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONTROL, get_control_settings());
    state            = BMx_Wait_for_samples;
    last_measurement = millis();
  } else {
    lastMeasurementError = true;
  }
}

// Only perform the measurements with big interval to prevent the sensor from warming up.
bool P028_data_struct::updateMeasurements(taskIndex_t task_index) {
  if ((state != BMx_Wait_for_samples) || measurementInProgress()) {
    // Nothing to do in processing the measurement
    return false;
  }

  if (!readUncompensatedData()) {
    return false;
  }

  // Set to sleep mode again to prevent the sensor from heating up.
  I2C_write8_reg(i2cAddress, BMx280_REGISTER_CONTROL, 0x00);

  lastMeasurementError = false;
  state                = BMx_New_values;
  last_temp_val        = readTemperature();
  last_press_val       = readPressure();
  last_hum_val         = readHumidity();


# ifndef LIMIT_BUILD_SIZE
  String log;

  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    log.reserve(120); // Prevent re-allocation
    log  = getDeviceName();
    log += ':';
  }
  bool logAdded = false;
# endif // ifndef LIMIT_BUILD_SIZE

  if (hasHumidity()) {
    // Apply half of the temp offset, to correct the dew point offset.
    // The sensor is warmer than the surrounding air, which has effect on the perceived humidity.
    last_dew_temp_val = compute_dew_point_temp(last_temp_val + (temp_offset / 2.0f), last_hum_val);
  } else {
    // No humidity measurement, thus set dew point equal to air temperature.
    last_dew_temp_val = last_temp_val;
  }

  if (!approximatelyEqual(temp_offset, 0.0f)) {
    # ifndef LIMIT_BUILD_SIZE

    // There is some offset to apply.
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log += F(" Apply temp offset ");
      log += temp_offset;
      log += 'C';
    }
    # endif // ifndef LIMIT_BUILD_SIZE

    if (hasHumidity()) {
      # ifndef LIMIT_BUILD_SIZE

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        log += F(" humidity ");
        log += last_hum_val;
      }
      # endif // ifndef LIMIT_BUILD_SIZE
      last_hum_val = compute_humidity_from_dewpoint(last_temp_val + temp_offset, last_dew_temp_val);

      # ifndef LIMIT_BUILD_SIZE

      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        log += F("% => ");
        log += last_hum_val;
        log += F("%");
      }
      # endif // ifndef LIMIT_BUILD_SIZE
    } else {
      last_hum_val = 0.0f;
    }

# ifndef LIMIT_BUILD_SIZE

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log += F(" temperature ");
      log += last_temp_val;
    }
# endif // ifndef LIMIT_BUILD_SIZE
    last_temp_val = last_temp_val + temp_offset;

# ifndef LIMIT_BUILD_SIZE

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log     += F("C => ");
      log     += last_temp_val;
      log     += 'C';
      logAdded = true;
    }
# endif // ifndef LIMIT_BUILD_SIZE
  }

# ifndef LIMIT_BUILD_SIZE

  if (hasHumidity()) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      log     += F(" dew point ");
      log     += last_dew_temp_val;
      log     += 'C';
      logAdded = true;
    }
  }

  if (logAdded && loglevelActiveFor(LOG_LEVEL_INFO)) {
    addLogMove(LOG_LEVEL_INFO, log);
  }
# endif // ifndef LIMIT_BUILD_SIZE
  return true;
}

// **************************************************************************/
// Check BME280 presence
// **************************************************************************/
bool P028_data_struct::check() {
  bool wire_status      = false;
  const uint8_t chip_id = I2C_read8_reg(i2cAddress, BMx280_REGISTER_CHIPID, &wire_status);

  if (!wire_status) { setUninitialized(); }

  switch (chip_id) {
    case BMP280_DEVICE_SAMPLE1:
    case BMP280_DEVICE_SAMPLE2:
    case BMP280_DEVICE:
    case BME280_DEVICE: {
      if (wire_status) {
        // Store detected chip ID when chip found.
        if (sensorID != chip_id) {
          sensorID = static_cast<BMx_ChipId>(chip_id);
          setUninitialized();

          if (loglevelActiveFor(LOG_LEVEL_INFO)) {
            String log = F("BMx280 : Detected ");
            log += getDeviceName();
            addLogMove(LOG_LEVEL_INFO, log);
          }
        }
      } else {
        sensorID = Unknown_DEVICE;
      }
      break;
    }
    default:
      sensorID = Unknown_DEVICE;
      break;
  }

  if (sensorID == Unknown_DEVICE) {
    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("BMx280 : Unable to detect chip ID (");
      log += chip_id;

      if (!wire_status) {
        log += F(", failed");
      }
      log += ')';
      addLogMove(LOG_LEVEL_INFO, log);
    }

    state = BMx_Error;

    return false;
  }
  return wire_status;
}

bool P028_data_struct::begin() {
  if (!check()) {
    return false;
  }

  // Perform soft reset
  I2C_write8_reg(i2cAddress, BMx280_REGISTER_SOFTRESET, 0xB6);
  delay(2); // Startup time is 2 ms (datasheet)
  readCoefficients();

  //  delay(65); //May be needed here as well to fix first wrong measurement?
  return true;
}

void P028_data_struct::readCoefficients()
{
  calib.dig_T1 = I2C_read16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_T1);
  calib.dig_T2 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_T2);
  calib.dig_T3 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_T3);

  calib.dig_P1 = I2C_read16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_P1);
  calib.dig_P2 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_P2);
  calib.dig_P3 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_P3);
  calib.dig_P4 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_P4);
  calib.dig_P5 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_P5);
  calib.dig_P6 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_P6);
  calib.dig_P7 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_P7);
  calib.dig_P8 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_P8);
  calib.dig_P9 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_P9);

  if (hasHumidity()) {
    calib.dig_H1 = I2C_read8_reg(i2cAddress, BMx280_REGISTER_DIG_H1);
    calib.dig_H2 = I2C_readS16_LE_reg(i2cAddress, BMx280_REGISTER_DIG_H2);
    calib.dig_H3 = I2C_read8_reg(i2cAddress, BMx280_REGISTER_DIG_H3);
    calib.dig_H4 = (I2C_read8_reg(i2cAddress, BMx280_REGISTER_DIG_H4) << 4) | (I2C_read8_reg(i2cAddress, BMx280_REGISTER_DIG_H4 + 1) & 0xF);
    calib.dig_H5 = (I2C_read8_reg(i2cAddress, BMx280_REGISTER_DIG_H5 + 1) << 4) | (I2C_read8_reg(i2cAddress, BMx280_REGISTER_DIG_H5) >> 4);
    calib.dig_H6 = (int8_t)I2C_read8_reg(i2cAddress, BMx280_REGISTER_DIG_H6);
  }
}

bool P028_data_struct::readUncompensatedData() {
  // wait until measurement has been completed, otherwise we would read
  // the values from the last measurement
  if (I2C_read8_reg(i2cAddress, BMx280_REGISTER_STATUS) & 0x08) {
    return false;
  }

  I2Cdata_bytes BME280_data(BME280_P_T_H_DATA_LEN, BME280_DATA_ADDR);
  const bool    allDataRead = I2C_read_bytes(i2cAddress, BME280_data);

  if (!allDataRead) {
    return false;
  }

  /* Variables to store the sensor data */
  uint32_t data_xlsb;
  uint32_t data_lsb;
  uint32_t data_msb;

  /* Store the parsed register values for pressure data */
  data_msb               = (uint32_t)BME280_data[BME280_DATA_ADDR + 0] << 12;
  data_lsb               = (uint32_t)BME280_data[BME280_DATA_ADDR + 1] << 4;
  data_xlsb              = (uint32_t)BME280_data[BME280_DATA_ADDR + 2] >> 4;
  uncompensated.pressure = data_msb | data_lsb | data_xlsb;

  /* Store the parsed register values for temperature data */
  data_msb                  = (uint32_t)BME280_data[BME280_DATA_ADDR + 3] << 12;
  data_lsb                  = (uint32_t)BME280_data[BME280_DATA_ADDR + 4] << 4;
  data_xlsb                 = (uint32_t)BME280_data[BME280_DATA_ADDR + 5] >> 4;
  uncompensated.temperature = data_msb | data_lsb | data_xlsb;

  /* Store the parsed register values for temperature data */
  data_lsb               = (uint32_t)BME280_data[BME280_DATA_ADDR + 6] << 8;
  data_msb               = (uint32_t)BME280_data[BME280_DATA_ADDR + 7];
  uncompensated.humidity = data_msb | data_lsb;
  return true;
}

float P028_data_struct::readTemperature()
{
  const int32_t adc_T = uncompensated.temperature;

  const int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) *
                        ((int32_t)calib.dig_T2)) >> 11;

  const int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
                          ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
                        ((int32_t)calib.dig_T3)) >> 14;

  calib.t_fine = var1 + var2;

  return static_cast<float>((calib.t_fine * 5 + 128) >> 8) / 100.0f;
}

float P028_data_struct::readPressure() const
{
  int64_t var1, var2, p;
  int32_t adc_P = uncompensated.pressure;

  var1 = ((int64_t)calib.t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) +
         ((var1 * (int64_t)calib.dig_P2) << 12);
  var1 = ((((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1)) >> 33;

  if (var1 == 0) {
    return 0.0f; // avoid exception caused by division by zero
  }
  p    = 1048576 - adc_P;
  p    = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
  return static_cast<float>(p) / 25600.0f;
}

float P028_data_struct::readHumidity() const
{
  if (!hasHumidity()) {
    // No support for humidity
    return 0.0f;
  }
  const int32_t adc_H = uncompensated.humidity;

  int32_t v_x1_u32r;

  v_x1_u32r = (calib.t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) -
                  (((int32_t)calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
               (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                  ((int32_t)2097152)) * ((int32_t)calib.dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)calib.dig_H1)) >> 4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  return static_cast<float>(v_x1_u32r >> 12) / 1024.0f;
}

#endif // ifdef USES_P028

#include "../PluginStructs/P005_data_struct.h"

#ifdef USES_P005


// DEBUG code using logic analyzer for timings
// #define DEBUG_LOGIC_ANALYZER_PIN  27 


// Macros to perform direct access on GPIOs
// Macros written by Paul Stoffregen
// See: https://github.com/PaulStoffregen/OneWire/blob/master/util/
# include <GPIO_Direct_Access.h>

enum struct P005_logNr {
  P005_error_no_reading,
  P005_error_protocol_timeout,
  P005_error_checksum_error,
  P005_error_invalid_NAN_reading,
  P005_info_temperature,
  P005_info_humidity
};

const __FlashStringHelper* P005_logString(P005_logNr logNr) {
  switch (logNr) {
    case P005_logNr::P005_error_no_reading:          return F("No Reading");
    case P005_logNr::P005_error_protocol_timeout:    return F("Protocol Timeout");
    case P005_logNr::P005_error_checksum_error:      return F("Checksum Error");
    case P005_logNr::P005_error_invalid_NAN_reading: return F("Invalid NAN reading");
    case P005_logNr::P005_info_temperature:          return F("Temperature: ");
    case P005_logNr::P005_info_humidity:             return F("Humidity: ");
  }
  return F("");
}

/*********************************************************************************************\
* DHT sub to log an error
\*********************************************************************************************/
void P005_log(struct EventStruct *event, P005_logNr logNr)
{
  bool isError = true;

  switch (logNr) {
    case P005_logNr::P005_info_temperature:
    case P005_logNr::P005_info_humidity:
      isError = false;
      break;

    default:
      UserVar[event->BaseVarIndex]     = NAN;
      UserVar[event->BaseVarIndex + 1] = NAN;
      break;
  }

  if (loglevelActiveFor(isError ? LOG_LEVEL_ERROR : LOG_LEVEL_INFO)) {
    String text;
    text  = F("DHT  : ");
    text += P005_logString(logNr);

    if (logNr == P005_logNr::P005_info_temperature) {
      text += formatUserVarNoCheck(event->TaskIndex, 0);
    }
    else if (logNr == P005_logNr::P005_info_humidity) {
      text += formatUserVarNoCheck(event->TaskIndex, 1);
    }
    addLogMove(isError ? LOG_LEVEL_ERROR : LOG_LEVEL_INFO, text);
  }
}

P005_data_struct::P005_data_struct(struct EventStruct *event) {
  SensorModel = PCONFIG(0);
  DHT_pin = CONFIG_PIN1;
}

/*********************************************************************************************\
* DHT sub to wait until a pin is in a certain state
\*********************************************************************************************/
bool P005_data_struct::waitState(uint32_t state)
{
  const uint64_t   timeout          = getMicros64() + 100;

#ifdef DEBUG_LOGIC_ANALYZER_PIN
  // DEBUG code using logic analyzer for timings
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 0);
#endif

  while (DIRECT_pinRead(DHT_pin) != state)
  {
    if (usecTimeOutReached(timeout)) { return false; }
  }

#ifdef DEBUG_LOGIC_ANALYZER_PIN
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 1);
#endif
  return true;
}

/*********************************************************************************************\
* Perform the actual reading + interpreting of data.
\*********************************************************************************************/
bool P005_data_struct::readDHT(struct EventStruct *event) {
  // Call the "slow" function to make sure the pin is in a defined state.
  // Apparently the pull-up state may not always be in a well known state
  // With the direct pinmode calls we don't set the pull-up or -down resistors.
  pinMode(DHT_pin, INPUT_PULLUP);

#ifdef DEBUG_LOGIC_ANALYZER_PIN
  // DEBUG code using logic analyzer for timings
  DIRECT_PINMODE_OUTPUT(DEBUG_LOGIC_ANALYZER_PIN);
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 0);
#endif

  // To begin asking the DHT22 for humidity and temperature data,
  // Start sequence to get data from a DHTxx sensor:
  // Pin must be a logic 0 (low) for at least 500 microseconds (DHT22, others may need different timing)
  // followed by a logic 1 (high).
  DIRECT_PINMODE_OUTPUT(DHT_pin);
  DIRECT_pinWrite(DHT_pin, 0);           // Pull low

  switch (SensorModel) {
    case P005_DHT11:  delay(19); break;  // minimum 18ms
    case P005_DHT22:  delay(2);  break;  // minimum 1ms
    case P005_DHT12:  delay(200); break; // minimum 200ms
    case P005_AM2301: delayMicroseconds(900); break;
    case P005_SI7021: delayMicroseconds(500); break;
  }

  {
#ifdef DEBUG_LOGIC_ANALYZER_PIN
    DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 1);
#endif

    DIRECT_PINMODE_INPUT(DHT_pin);
    // pinMode(DHT_pin, INPUT_PULLUP);  // Way too slow, takes upto 227 usec

#ifdef DEBUG_LOGIC_ANALYZER_PIN
    DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 0);
#endif
  }

  bool readingAborted = false;
  uint8_t dht_dat[5]  = { 0 };

  uint8_t  dht_byte      = 0;
  uint32_t avg_low_total = 0;


  // Response from DHTxx: (N = 80 usec for DHT22)
  // Low for N usec
  // Hight for N usec
  // Low for 50 usec
  bool receive_start;

  uint8_t timings[16] = { 0 };


  noInterrupts();
  receive_start = waitState(0) && waitState(1) && waitState(0);

#ifdef DEBUG_LOGIC_ANALYZER_PIN
  DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 0);
#endif

  if (receive_start) {
    // We know we're now at a "low" state.
    uint64_t last_micros = getMicros64();
    uint64_t prev_edge   = last_micros;

    for (dht_byte = 0; dht_byte < 5 && !readingAborted; ++dht_byte)
    {
      // Start reading next byte
#ifdef DEBUG_LOGIC_ANALYZER_PIN
      DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 1);
#endif
      for (uint8_t t = 0; t < 16 && !readingAborted; ++t) {
        // "even" index = "low" duration
        // "odd"  index = "high" duration
        const uint32_t current_state = (t & 1);

        // Wait till pin state has changed, or timeout.
        while (DIRECT_pinRead(DHT_pin) == current_state && !readingAborted)
        {
          // Keep track of last microsecond the state had not yet changed.
          // This way we are less dependent on any jitter caused by
          // the delay call or rise times of the voltage on the pin.
          last_micros = getMicros64();

          if (timeDiff64(prev_edge, last_micros) > 100) {
            readingAborted = true;
          }
        }

        if (!readingAborted) {
          // We know it is less than 100 usec, so it does fit in the uint8_t timings array.
          timings[t] = usecPassedSince(prev_edge);
          prev_edge  = last_micros;
        } else {
          timings[t] = 255;
        }
      }
#ifdef DEBUG_LOGIC_ANALYZER_PIN
      DIRECT_pinWrite(DEBUG_LOGIC_ANALYZER_PIN, 0);
#endif

      if (!readingAborted) {
        // Evaluate the timings
        // timings on even indices represent "duration low"
        // timings on odd  indices represent "duration high"
        //
        // Timing for a single bit:
        // Logic "1":  50 usec low, 70 usec high
        // Logic "0":  50 usec low, 26 usec high
        // There is a significant difference between the "high" state durations
        // Thus "high duration" > "avg_low duration" means it is an "1".
        //
        // By taking the average low duration, we get rid of
        // critical timing differences among modules and
        // environmental effects which may change these timings.
        // It is all about the relative timings.
        uint32_t avg_low = 0;

        // Don't take the 1st "low" period into account for computing avg_low
        // as there might be an extra wait between bytes.
        // Just to be sure as it is not clear from the documentation if all models act the same.
        for (uint8_t t = 2; t < 16; t += 2) {
          avg_low += timings[t];
        }
        avg_low       /= 7;
        avg_low_total += avg_low;

        dht_dat[dht_byte] = 0;

        for (uint8_t bit = 0; bit < 8; ++bit) {
          if (timings[2 * bit + 1] > avg_low) {
            dht_dat[dht_byte] |= (1 << (7 - bit));
          }
        }
      }
    }
  }
  interrupts();


  if (!receive_start) {
    P005_log(event, P005_logNr::P005_error_no_reading);
    return false;
  }

  # ifndef BUILD_NO_DEBUG

  if (dht_byte != 0) {
    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log = F("DHT  : ");
      log += F("Avg Low: ");
      log += static_cast<float>(avg_low_total) / dht_byte;
      log += F(" usec ");
      log += dht_byte;
      log += F(" bytes:");
      for (int i = 0; i < dht_byte; ++i) {
        log += ' ';
        log += formatToHex_no_prefix(dht_dat[i], 2);
      }
      log += F(" timings:");
      for (int i = 0; i < 16; ++i) {
        log += ' ';
        log += timings[i];
      }
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
  }
  # endif // ifndef BUILD_NO_DEBUG


  if (readingAborted) {
    P005_log(event, P005_logNr::P005_error_protocol_timeout);
    return false;
  }

  // Checksum calculation is a Rollover Checksum by design!
  uint8_t dht_check_sum = (dht_dat[0] + dht_dat[1] + dht_dat[2] + dht_dat[3]) & 0xFF; // check check_sum

  if (dht_dat[4] != dht_check_sum)
  {
    P005_log(event, P005_logNr::P005_error_checksum_error);
    return false;
  }

  float temperature = NAN;
  float humidity    = NAN;

  switch (SensorModel) {
    case P005_DHT11:
    case P005_DHT12:
      temperature = float(dht_dat[2] * 10 + (dht_dat[3] & 0x7f)) / 10.0f; // Temperature

      if (dht_dat[3] & 0x80) { temperature = -temperature; } // Negative temperature
      humidity = float(dht_dat[0] * 10 + dht_dat[1]) / 10.0f;             // Humidity
      break;
    case P005_DHT22:
    case P005_AM2301:
    case P005_SI7021:

      if (dht_dat[2] & 0x80) { // negative temperature
        temperature = -0.1f * word(dht_dat[2] & 0x7F, dht_dat[3]);
      }
      else {
        temperature = 0.1f * word(dht_dat[2], dht_dat[3]);
      }
      humidity = 0.1f * word(dht_dat[0], dht_dat[1]); // Humidity
      break;
  }

  if (isnan(temperature) || isnan(humidity))
  {     P005_log(event, P005_logNr::P005_error_invalid_NAN_reading);
        return false; }

  UserVar[event->BaseVarIndex]     = temperature;
  UserVar[event->BaseVarIndex + 1] = humidity;
  P005_log(event, P005_logNr::P005_info_temperature);
  P005_log(event, P005_logNr::P005_info_humidity);
  return true;
}

#endif // ifdef USES_P005

#include "../PluginStructs/P124_data_struct.h"

#ifdef USES_P124

// **************************************************************************/
// Constructor
// **************************************************************************/
P124_data_struct::P124_data_struct(int8_t  i2c_address,
                                   uint8_t relayCount,
                                   bool    changeAddress)
  : _i2c_address(i2c_address), _relayCount(relayCount) {
  relay = new (std::nothrow) Multi_Channel_Relay();

  if (isInitialized()) {
    relay->begin(_i2c_address);

    if (changeAddress) {
      // This increment shpould match with the range of addresses in _P124_MultiRelay.ino PLUGIN_I2C_HAS_ADDRESS
      uint8_t _new_address = _i2c_address == 0x18 ? 0x11 : _i2c_address + 1; // Set to next address
      relay->changeI2CAddress(_new_address, _i2c_address);
      # ifndef BUILD_NO_DEBUG
      if (loglevelActiveFor(LOG_LEVEL_INFO)) {
        String log = F("MultiRelay: Change I2C address 0x");
        log += String(_i2c_address, HEX);
        log += F(" to 0x");
        log += String(_new_address, HEX);
        addLogMove(LOG_LEVEL_INFO, log);
      }
      # endif // ifndef BUILD_NO_DEBUG
    }
  }
}

// **************************************************************************/
// Destructor
// **************************************************************************/
P124_data_struct::~P124_data_struct() {
  if (isInitialized()) {
    delete relay;
    relay = nullptr;
  }
}

uint8_t P124_data_struct::getChannelState() {
  if (isInitialized()) {
    return relay->getChannelState();
  }
  return 0u;
}

uint8_t P124_data_struct::getFirmwareVersion() {
  if (isInitialized()) {
    return relay->getFirmwareVersion();
  }
  return 0u;
}

bool P124_data_struct::channelCtrl(uint8_t state) {
  if (isInitialized()) {
    relay->channelCtrl(state);
    return true;
  }
  return false;
}

bool P124_data_struct::turn_on_channel(uint8_t channel) {
  if (isInitialized() && (validChannel(channel))) {
    relay->turn_on_channel(channel);
    return true;
  }
  return false;
}

bool P124_data_struct::turn_off_channel(uint8_t channel) {
  if (isInitialized() && (validChannel(channel))) {
    relay->turn_off_channel(channel);
    return true;
  }
  return false;
}

uint8_t P124_data_struct::getNextLoop() {
  if (_loopEnabled) {
    _getLoop++;

    if (_getLoop > _relayCount) {
      _getLoop = 1;
    }
    return _getLoop;
  }
  return 0;
}

#endif // ifdef USES_P124

#include "../PluginStructs/P108_data_struct.h"

#ifdef USES_P108


P108_data_struct::~P108_data_struct() {
  reset();
}

void P108_data_struct::reset() {
  modbus.reset();
}

bool P108_data_struct::init(ESPEasySerialPort port, const int16_t serial_rx, const int16_t serial_tx, int8_t dere_pin,
                            unsigned int baudrate, uint8_t modbusAddress) {
  return modbus.init(port, serial_rx, serial_tx, baudrate, modbusAddress, dere_pin);
}

bool P108_data_struct::isInitialized() const {
  return modbus.isInitialized();
}

const __FlashStringHelper* Plugin_108_valuename(uint8_t value_nr, bool displayString) {
  switch (value_nr) {
    case P108_QUERY_V: return displayString ? F("Voltage (V)") : F("V");
    case P108_QUERY_A: return displayString ? F("Current (A)") : F("A");
    case P108_QUERY_W: return displayString ? F("Active Power (W)") : F("W");
    case P108_QUERY_VA: return displayString ? F("Reactive Power (VA)") : F("VA");
    case P108_QUERY_PF: return displayString ? F("Power Factor (Pf)") : F("Pf");
    case P108_QUERY_F: return displayString ? F("Frequency (Hz)") : F("Hz");
    case P108_QUERY_Wh_imp: return displayString ? F("Import Energy (Wh)") : F("Wh_imp");
    case P108_QUERY_Wh_exp: return displayString ? F("Export Energy (Wh)") : F("Wh_exp");
    case P108_QUERY_Wh_tot: return displayString ? F("Total Energy (Wh)") : F("Wh_tot");
  }
  return F("");
}

int p108_storageValueToBaudrate(uint8_t baudrate_setting) {
  switch (baudrate_setting) {
    case 0:
      return 1200;
    case 1:
      return 2400;
    case 2:
      return 4800;
    case 3:
      return 9600;
  }
  return 9600;
}

float p108_readValue(uint8_t query, struct EventStruct *event) {
  uint8_t errorcode           = -1;   // DF - not present in P085
  float   value               = 0.0f; // DF - not present in P085
  P108_data_struct *P108_data =
    static_cast<P108_data_struct *>(getPluginTaskData(event->TaskIndex));

  if ((nullptr != P108_data) && P108_data->isInitialized()) {
    switch (query) {
      case P108_QUERY_V:
        value = P108_data->modbus.readHoldingRegister(0x0C, errorcode) / 10.0f;  // 0.1 V => V
        break;
      case P108_QUERY_A:
        value = P108_data->modbus.readHoldingRegister(0x0D, errorcode) / 100.0f; // 0.01 A => A
        break;
      case P108_QUERY_W:
        value =  P108_data->modbus.readHoldingRegister(0x0E, errorcode);

        if (value > 32767) { value -= 65535; }
        break;
      case P108_QUERY_VA:
        value = P108_data->modbus.readHoldingRegister(0x0F, errorcode);

        if (value > 32767) { value -= 65535; }
        break;
      case P108_QUERY_PF:
        value = P108_data->modbus.readHoldingRegister(0x10, errorcode) / 1000.0f; // 0.001 Pf => Pf
        break;
      case P108_QUERY_F:
        value = P108_data->modbus.readHoldingRegister(0x11, errorcode) / 100.0f;  // 0.01 Hz => Hz
        break;
      case P108_QUERY_Wh_imp:
        return P108_data->modbus.read_32b_HoldingRegister(0x0A) * 10.0f;          // 0.01 kWh => Wh
        break;
      case P108_QUERY_Wh_exp:
        return P108_data->modbus.read_32b_HoldingRegister(0x08) * 10.0f;          // 0.01 kWh => Wh
        break;
      case P108_QUERY_Wh_tot:
        return P108_data->modbus.read_32b_HoldingRegister(0x00) * 10.0f;          // 0.01 kWh => Wh
        break;
    }
  }

  if (errorcode == 0) { return value; // DF - not present in P085
  }
  return 0.0f;
}

void p108_showValueLoadPage(uint8_t query, struct EventStruct *event) {
  addRowLabel(Plugin_108_valuename(query, true));
  addHtml(String(p108_readValue(query, event)));
}

#endif // ifdef USES_P108

#include "../PluginStructs/P079_data_struct.h"

#ifdef USES_P079



WemosMotor::WemosMotor(uint8_t address, uint8_t motor, uint32_t freq)
  : _address(address)
{
  _use_STBY_IO = false;

  if (motor == P079_MOTOR_A) {
    _motor = P079_MOTOR_A;
  }
  else {
    _motor = P079_MOTOR_B;
  }

  setfreq(freq);
}

WemosMotor::WemosMotor(uint8_t address, uint8_t motor, uint32_t freq, uint8_t STBY_IO)
  : _address(address)
{
  _use_STBY_IO = true;
  _STBY_IO     = STBY_IO;

  if (motor == P079_MOTOR_A) {
    _motor = P079_MOTOR_A;
  }
  else {
    _motor = P079_MOTOR_B;
  }

  setfreq(freq);

  pinMode(_STBY_IO, OUTPUT);
  digitalWrite(_STBY_IO, LOW);
}

/* setfreq() -- set PWM's frequency
   freq: PWM's frequency

   total 4bytes
 |0.5byte CMD     | 3.5byte Parm|
 |CMD             | parm        |
 |0x0X  set freq  | uint32  freq|
 */
void WemosMotor::setfreq(uint32_t freq)
{
  Wire.beginTransmission(_address);
  Wire.write(((uint8_t)(freq >> 24)) & (uint8_t)0x0f);
  Wire.write((uint8_t)(freq >> 16));
  Wire.write((uint8_t)(freq >> 8));
  Wire.write((uint8_t)freq);
  Wire.endTransmission(); // stop transmitting
  delay(0);
}

/* setmotor() -- set motor
   motor:
        P079_MOTOR_A    0   Motor A
        P079_MOTOR_B    1   Motor B

   dir:
        P079_SHORT_BRAKE  0
        P079_CCW          1
        P079_CW               2
        P079_STOP         3
        P079_STANDBY      4

   pwm_val:
        0.00 - 100.00  (%)

   total 4bytes
 |0.5byte CMD      | 3.5byte Parm         |
 |CMD              | parm                 |
 |0x10  set motorA | uint8 dir  uint16 pwm|
 |0x11  set motorB | uint8 dir  uint16 pwm|
 */
void WemosMotor::setmotor(uint8_t dir, float pwm_val)
{
  uint16_t _pwm_val;

  if (_use_STBY_IO == true) {
    if (dir == P079_STANDBY) {
      digitalWrite(_STBY_IO, LOW);
      return;
    } else {
      digitalWrite(_STBY_IO, HIGH);
    }
  }

  Wire.beginTransmission(_address);
  Wire.write(_motor | (uint8_t)0x10); // CMD either 0x10 or 0x11
  Wire.write(dir);

  // PWM in %
  _pwm_val = uint16_t(pwm_val * 100);

  if (_pwm_val > 10000) { // _pwm_val > 100.00
    _pwm_val = 10000;
  }

  Wire.write((uint8_t)(_pwm_val >> 8));
  Wire.write((uint8_t)_pwm_val);
  Wire.endTransmission(); // stop transmitting

  delay(0);
}

void WemosMotor::setmotor(uint8_t dir)
{
  setmotor(dir, 100);
}

LOLIN_I2C_MOTOR::LOLIN_I2C_MOTOR(uint8_t address) : _address(address) {}

/*
    Change Motor Status.
    ch: Motor Channel
        MOTOR_CH_A
        MOTOR_CH_B
        MOTOR_CH_BOTH

    sta: Motor Status
        MOTOR_STATUS_STOP
        MOTOR_STATUS_CCW
        MOTOR_STATUS_CW
        MOTOR_STATUS_SHORT_BRAKE
        MOTOR_STATUS_STANDBY
 */
unsigned char LOLIN_I2C_MOTOR::changeStatus(unsigned char ch, unsigned char sta)
{
  send_data[0] = CHANGE_STATUS;
  send_data[1] = ch;
  send_data[2] = sta;
  unsigned char result = sendData(send_data, 3);

  return result;
}

/*
    Change Motor Frequency
        ch: Motor Channel
            MOTOR_CH_A
            MOTOR_CH_B
            MOTOR_CH_BOTH

        freq: PWM frequency (Hz)
            1 - 80KHz, typically 1000Hz
 */
unsigned char LOLIN_I2C_MOTOR::changeFreq(unsigned char ch, uint32_t freq)
{
  send_data[0] = CHANGE_FREQ;
  send_data[1] = ch;

  send_data[2] = (uint8_t)(freq & 0xff);
  send_data[3] = (uint8_t)((freq >> 8) & 0xff);
  send_data[4] = (uint8_t)((freq >> 16) & 0xff);
  unsigned char result = sendData(send_data, 5);

  return result;
}

/*
    Change Motor Duty.
    ch: Motor Channel
        MOTOR_CH_A
        MOTOR_CH_B
        MOTOR_CH_BOTH

    duty: PWM Duty (%)
        0.01 - 100.00 (%)
 */
unsigned char LOLIN_I2C_MOTOR::changeDuty(unsigned char ch, float duty)
{
  uint16_t _duty;

  _duty = (uint16_t)(duty * 100);

  send_data[0] = CHANGE_DUTY;
  send_data[1] = ch;

  send_data[2] = (uint8_t)(_duty & 0xff);
  send_data[3] = (uint8_t)((_duty >> 8) & 0xff);
  unsigned char result = sendData(send_data, 4);

  return result;
}

/*
    Reset Device.
 */
unsigned char LOLIN_I2C_MOTOR::reset()
{
  send_data[0] = RESET_SLAVE;
  unsigned char result = sendData(send_data, 1);

  return result;
}

/*
    Change Device I2C address
    address: when address=0, address>=127, will change address to default I2C address 0x31
 */
unsigned char LOLIN_I2C_MOTOR::changeAddress(unsigned char address)
{
  send_data[0] = CHANGE_I2C_ADDRESS;
  send_data[1] = address;
  unsigned char result = sendData(send_data, 2);

  return result;
}

/*
    Get PRODUCT_ID and Firmwave VERSION
 */
unsigned char LOLIN_I2C_MOTOR::getInfo(void)
{
  send_data[0] = GET_SLAVE_STATUS;
  unsigned char result = sendData(send_data, 1);

  if (result == 0)
  {
    PRODUCT_ID = get_data[0];
    VERSION_ID = get_data[1];
  }
  else
  {
    PRODUCT_ID = 0;
    VERSION_ID = 0;
  }

  return result;
}

/*
    Send and Get I2C Data
 */
unsigned char LOLIN_I2C_MOTOR::sendData(unsigned char *data, unsigned char len)
{
  unsigned char i;

  if ((_address == 0) || (_address >= 127))
  {
    return 1;
  }
  else
  {
    Wire.beginTransmission(_address);

    for (i = 0; i < len; i++) {
      Wire.write(data[i]);
    }
    Wire.endTransmission();
    delay(50);

    if (data[0] == GET_SLAVE_STATUS) {
      Wire.requestFrom(static_cast<int>(_address), 2);
    }
    else {
      Wire.requestFrom(static_cast<int>(_address), 1);
    }

    i = 0;

    while (Wire.available())
    {
      get_data[i] = Wire.read();
      i++;
    }

    return 0;
  }
}

#endif // ifdef USES_P079

#include "../PluginStructs/P057_data_struct.h"


// Needed also here for PlatformIO's library finder as the .h file 
// is in a directory which is excluded in the src_filter
# include <HT16K33.h>

#ifdef USES_P057

P057_data_struct::P057_data_struct(uint8_t i2c_addr) : i2cAddress(i2c_addr) {
  ledMatrix.Init(i2cAddress);
}

#endif // ifdef USES_P057

#include "../PluginStructs/P032_data_struct.h"

#ifdef USES_P032

#include "../Globals/I2Cdev.h"


enum
{
  MS5xxx_CMD_RESET    = 0x1E, // perform reset
  MS5xxx_CMD_ADC_READ = 0x00, // initiate read sequence
  MS5xxx_CMD_ADC_CONV = 0x40, // start conversion
  MS5xxx_CMD_ADC_D1   = 0x00, // read ADC 1
  MS5xxx_CMD_ADC_D2   = 0x10, // read ADC 2
  MS5xxx_CMD_ADC_256  = 0x00, // set ADC oversampling ratio to 256
  MS5xxx_CMD_ADC_512  = 0x02, // set ADC oversampling ratio to 512
  MS5xxx_CMD_ADC_1024 = 0x04, // set ADC oversampling ratio to 1024
  MS5xxx_CMD_ADC_2048 = 0x06, // set ADC oversampling ratio to 2048
  MS5xxx_CMD_ADC_4096 = 0x08, // set ADC oversampling ratio to 4096
  MS5xxx_CMD_PROM_RD  = 0xA0  // initiate readout of PROM registers
};


P032_data_struct::P032_data_struct(uint8_t i2c_addr) : i2cAddress(i2c_addr) {}


// **************************************************************************/
// Initialize MS5611
// **************************************************************************/
bool P032_data_struct::begin() {
  Wire.beginTransmission(i2cAddress);
  uint8_t ret = Wire.endTransmission(true);

  return ret == 0;
}

// **************************************************************************/
// Reads the PROM of MS5611
// There are in total 8 addresses resulting in a total memory of 128 bit.
// Address 0 contains factory data and the setup, addresses 1-6 calibration
// coefficients and address 7 contains the serial code and CRC.
// The command sequence is 8 bits long with a 16 bit result which is
// clocked with the MSB first.
// **************************************************************************/
void P032_data_struct::read_prom() {
  I2C_write8(i2cAddress, MS5xxx_CMD_RESET);
  delay(3);

  for (uint8_t i = 0; i < 8; i++)
  {
    ms5611_prom[i] = I2C_read16_reg(i2cAddress, MS5xxx_CMD_PROM_RD + 2 * i);
  }
}

// **************************************************************************/
// Read analog/digital converter
// **************************************************************************/
unsigned long P032_data_struct::read_adc(unsigned char aCMD)
{
  I2C_write8(i2cAddress, MS5xxx_CMD_ADC_CONV + aCMD); // start DAQ and conversion of ADC data

  switch (aCMD & 0x0f)
  {
    case MS5xxx_CMD_ADC_256: delayMicroseconds(900);
      break;
    case MS5xxx_CMD_ADC_512: delay(3);
      break;
    case MS5xxx_CMD_ADC_1024: delay(4);
      break;
    case MS5xxx_CMD_ADC_2048: delay(6);
      break;
    case MS5xxx_CMD_ADC_4096: delay(10);
      break;
  }

  // read out values
  return I2C_read24_reg(i2cAddress, MS5xxx_CMD_ADC_READ);
}

// **************************************************************************/
// Readout
// **************************************************************************/
void P032_data_struct::readout() {
  unsigned long D1 = 0, D2 = 0;

  double dT;
  double Offset;
  double SENS;

  D2 = read_adc(MS5xxx_CMD_ADC_D2 + MS5xxx_CMD_ADC_4096);
  D1 = read_adc(MS5xxx_CMD_ADC_D1 + MS5xxx_CMD_ADC_4096);

  // calculate 1st order pressure and temperature (MS5611 1st order algorithm)
  dT                 = D2 - ms5611_prom[5] * static_cast<double>(1 << 8);
  Offset             = ms5611_prom[2] * static_cast<double>(1 << 16) + dT * ms5611_prom[4] / static_cast<double>(1 << 7);
  SENS               = ms5611_prom[1] * static_cast<double>(1 << 15) + dT * ms5611_prom[3] / static_cast<double>(1 << 8);
  ms5611_temperature = (2000 + (dT * ms5611_prom[6]) / static_cast<double>(1 << 23));

  // perform higher order corrections
  double T2 = 0., OFF2 = 0., SENS2 = 0.;

  if (ms5611_temperature < 2000) {
    T2    = dT * dT / static_cast<double>(1 << 31);
    const double temp_20deg = ms5611_temperature - 2000;
    OFF2  = 5.0 * temp_20deg * temp_20deg / static_cast<double>(1 << 1);
    SENS2 = 5.0 * temp_20deg * temp_20deg / static_cast<double>(1 << 2);

    if (ms5611_temperature < -1500) {
      const double temp_min15deg = ms5611_temperature + 1500;
      OFF2  += 7.0 * temp_min15deg * temp_min15deg;
      SENS2 += 11.0 * temp_min15deg * temp_min15deg / static_cast<double>(1 << 1);
    }
  }

  ms5611_temperature -= T2;
  Offset             -= OFF2;
  SENS               -= SENS2;
  ms5611_pressure     = (((D1 * SENS) / static_cast<double>(1 << 21) - Offset) / static_cast<double>(1 << 15));
}


#endif // ifdef USES_P032

#include "../PluginStructs/P115_data_struct.h"

#ifdef USES_P115

P115_data_struct::P115_data_struct(
  uint8_t                i2c_addr,
  sfe_max1704x_devices_e device,
  int                    threshold)
  : _device(device), lipo(device), _threshold(threshold), initialized(false)
{
  // Call begin() immediately, or else _i2cPort in lipo object is still a nullptr
  lipo.begin();
  _threshold = constrain(_threshold, 1, 32);
}

bool P115_data_struct::begin()
{
  // quickStart() - Restarts the MAX1704x to allow it to re-"guess" the
  // parameters that go into its SoC algorithms. Calling this in your setup()
  // usually results in more accurate SoC readings.
  // Output: 0 on success, positive integer on fail.

  // FIXME TD-er: Looks like begin() and/or quickStart() may not return expected values.
  // const bool success = lipo.begin() && (lipo.quickStart() == 0);
  lipo.begin();
  lipo.quickStart();
  const bool success = true;

  if (success) {
    switch (_device) {
      case MAX1704X_MAX17043:
      case MAX1704X_MAX17044:
        break;
      case MAX1704X_MAX17048:
      case MAX1704X_MAX17049:
        lipo.enableSOCAlert();
        break;
    }
    lipo.setThreshold(_threshold);
    initialized = true;
    return true;
  }
  initialized = false;
  return false;
}

bool P115_data_struct::read(bool clearAlert)
{
  voltage    = lipo.getVoltage();
  soc        = lipo.getSOC();
  changeRate = lipo.getChangeRate();
  return lipo.getAlert(clearAlert);
}

void P115_data_struct::clearAlert() {
  alert = false;
  lipo.clearAlert();
}

#endif // ifdef USES_P115

#include "../PluginStructs/P068_data_struct.h"

#ifdef USES_P068

// ==============================================
// P068_SHT3X LIBRARY - SHT3X.cpp
// =============================================
P068_SHT3X::P068_SHT3X(uint8_t addr) : _i2c_device_address(addr)
{
  // Set to periodic mode
  I2C_write8_reg(
    _i2c_device_address,
    0x20, // periodic 0.5mps
    0x32  // repeatability high
    );
}

void P068_SHT3X::readFromSensor()
{
  I2C_write8_reg(
    _i2c_device_address,
    0xE0, // fetch data command
    0x00
    );

  // FIXME TD-er: Currently the I2Cdev::readBytes does not support writing 2 bytes before reading.
  Wire.requestFrom(_i2c_device_address, (uint8_t)6);

  if (Wire.available() == 6)
  {
    uint16_t data[6];

    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();

    // TODO: check CRC (data[2] and data[5])
    if (CRC8(data[0], data[1], data[2]) &&
        CRC8(data[3], data[4], data[5]))
    {
      tmp = ((((data[0] << 8) | data[1]) * 175.0f) / 65535.0f) - 45.0f;
      hum = ((((data[3] << 8) | data[4]) * 100.0f) / 65535.0f);

      // Humidity temperature compensation borrowed from P028 BME280
      if (!essentiallyEqual(tmpOff, 0.0f)) {
        float last_dew_temp_val = compute_dew_point_temp(tmp + (tmpOff / 2.0f), hum);
        hum = compute_humidity_from_dewpoint(tmp + tmpOff, last_dew_temp_val);
        tmp = tmp + tmpOff;
      }
    }
  }
  else
  {
    tmp = NAN;
    hum = NAN;

    // Set to periodic mode
    Wire.beginTransmission(_i2c_device_address);
    Wire.write(0x20); // periodic 0.5mps
    Wire.write(0x32); // repeatability high
    Wire.endTransmission();
  }
}

// FIXME TD-er: Try to make some collection of used CRC algorithms
// See http://reveng.sourceforge.net/crc-catalogue/1-15.htm#crc.cat.crc-8-dvb-s2
bool P068_SHT3X::CRC8(uint8_t MSB, uint8_t LSB, uint8_t CRC)
{
  /*
   *	Name           : CRC-8
   * Polynomial     : 0x31 (x8 + x5 + x4 + 1)
   * Initialization : 0xFF
   * Reflect input  : False
   * Reflect output : False
   * Final          : XOR 0x00
   *	Example        : CRC8( 0xBE, 0xEF, 0x92) should be true
   */
  uint8_t crc = 0xFF;

  for (uint8_t bytenr = 0; bytenr < 2; ++bytenr) {
    crc ^= (bytenr == 0) ? MSB : LSB;

    for (uint8_t i = 0; i < 8; ++i) {
      crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }
  }
  return crc == CRC;
}

#endif // ifdef USES_P068

#include "../PluginStructs/P119_data_struct.h"

#ifdef USES_P119

// **************************************************************************/
// Constructor
// **************************************************************************/
P119_data_struct::P119_data_struct(uint8_t i2c_addr, bool rawData, uint8_t aSize)
  : _i2cAddress(i2c_addr), _rawData(rawData), _aSize(aSize) {
  if (_aSize == 0) { _aSize = 1; }
  _XA.resize(_aSize, 0);
  _YA.resize(_aSize, 0);
  _ZA.resize(_aSize, 0);
  _aUsed = 0;
  _aMax  = 0;
}

// **************************************************************************/
// Destructor
// **************************************************************************/
P119_data_struct::~P119_data_struct() {
  if (initialized()) {
    delete itg3205;
    itg3205 = nullptr;
  }
}

// **************************************************************************/
// Initialize sensor and read data from ITG3205
// **************************************************************************/
bool P119_data_struct::read_sensor() {
  #ifdef PLUGIN_119_DEBUG
  String log;
  # endif // if PLUGIN_119_DEBUG

  if (!initialized()) {
    init_sensor();

    #ifdef PLUGIN_119_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG) &&
        log.reserve(55)) {
      log  = F("ITG3205: i2caddress: 0x");
      log += String(_i2cAddress, HEX);
      log += F(", initialized: ");
      log += String(initialized() ? F("true") : F("false"));
      log += F(", ID=0x");
      log += String(itg3205->readWhoAmI(), HEX);
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
    # endif // if PLUGIN_119_DEBUG
  }

  if (initialized()) {
    if (_rawData) {
      itg3205->readGyroRaw();
    } else {
      itg3205->readGyro();
    }
    _XA[_aUsed] = itg3205->g.x;
    _YA[_aUsed] = itg3205->g.y;
    _ZA[_aUsed] = itg3205->g.z;

    _aUsed++;

    if ((_aMax < _aUsed) && (_aUsed < _aSize)) {
      _aMax = _aUsed;
    }

    if (_aUsed == _aSize) {
      _aUsed = 0;
    }

    #ifdef PLUGIN_119_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG) &&
        log.reserve(40)) {
      log  = F("ITG3205: ");
      log += (_rawData ? F("raw ") : F(""));
      log += F(", X: ");
      log += itg3205->g.x;
      log += F(", Y: ");
      log += itg3205->g.y;
      log += F(", Z: ");
      log += itg3205->g.z;
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
    # endif // if PLUGIN_119_DEBUG
    return true;
  }
  return false;
}

// **************************************************************************/
// Average the measurements and return the results
// **************************************************************************/
bool P119_data_struct::read_data(int& X, int& Y, int& Z) {
  X = 0;
  Y = 0;
  Z = 0;

  if (initialized()) {
    for (uint8_t n = 0; n <= _aMax; n++) {
      X += _XA[n];
      Y += _YA[n];
      Z += _ZA[n];
    }

    X /= _aMax; // Average available measurements
    Y /= _aMax;
    Z /= _aMax;

    #ifdef PLUGIN_119_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
      String log;

      if (log.reserve(40)) {
        log  = F("ITG3205: averages, X: ");
        log += X;
        log += F(", Y: ");
        log += Y;
        log += F(", Z: ");
        log += Z;
        addLogMove(LOG_LEVEL_DEBUG, log);
      }
    }
    # endif // if PLUGIN_119_DEBUG
  }
  return initialized();
}

// **************************************************************************/
// Initialize ITG3205
// **************************************************************************/
bool P119_data_struct::init_sensor() {
  itg3205 = new (std::nothrow) ITG3205(_i2cAddress);

  if (initialized()) {
    addLog(LOG_LEVEL_INFO, F("ITG3205: Initializing Gyro..."));
    itg3205->initGyro();
    addLog(LOG_LEVEL_INFO, F("ITG3205: Calibrating Gyro..."));
    itg3205->calibrate();
    addLog(LOG_LEVEL_INFO, F("ITG3205: Calibration done."));
  } else {
    addLog(LOG_LEVEL_ERROR, F("ITG3205: Initialization of Gyro failed."));
    return false;
  }

  #ifdef PLUGIN_119_DEBUG
  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log;

    if (log.reserve(25)) {
      log  = F("ITG3205: Address: 0x");
      log += String(_i2cAddress, HEX);
      addLogMove(LOG_LEVEL_DEBUG, log);
    }
  }
  #endif

  return true;
}

#endif // ifdef USES_P119

#include "../PluginStructs/P004_data_struct.h"

#ifdef USES_P004


P004_data_struct::P004_data_struct(int8_t pin_rx, int8_t pin_tx, const uint8_t addr[], uint8_t res) : _gpio_rx(pin_rx), _gpio_tx(pin_tx), _res(res)
{
  // Explicitly set the pinMode using the "slow" pinMode function
  // This way we know for sure the state of any pull-up or -down resistor is known.
  pinMode(_gpio_rx, INPUT);

  // The Shelly 1 temp. addon uses separate
  // input and output pins, and therefore
  // doesn't switch between input and output
  // when running.
  if (_gpio_rx != _gpio_tx) {
    pinMode(_gpio_tx, OUTPUT);
  }

  if ((_res < 9) || (_res > 12)) { _res = 12; }

  add_addr(addr, 0);
  set_measurement_inactive();
}

void P004_data_struct::add_addr(const uint8_t addr[], uint8_t index) {
  if (index < 4) {
    _sensors[index].addr = Dallas_addr_to_uint64(addr);

    // If the address already exists, set it to 0 to avoid duplicates
    for (uint8_t i = 0; i < 4; ++i) {
      if (index != i) {
        if (_sensors[index].addr == _sensors[i].addr) {
          _sensors[index].addr = 0;
        }
      }
    }
    _sensors[index].check_sensor(_gpio_rx, _gpio_tx, _res);
  }
}

bool P004_data_struct::initiate_read() {
  _measurementStart = millis();

  for (uint8_t i = 0; i < 4; ++i) {
    if (_sensors[i].initiate_read(_gpio_rx, _gpio_tx, _res)) {
      if (!measurement_active()) {
        // Set the timer right after initiating the first sensor


        /*********************************************************************************************\
        *  Dallas Start Temperature Conversion, expected max duration:
        *    9 bits resolution ->  93.75 ms
        *   10 bits resolution -> 187.5 ms
        *   11 bits resolution -> 375 ms
        *   12 bits resolution -> 750 ms
        \*********************************************************************************************/
        _timer = millis() + (800 / (1 << (12 - _res)));
      }
      _sensors[i].measurementActive = true;
    }
  }

  return measurement_active();
}

bool P004_data_struct::collect_values() {
  bool success = false;

  for (uint8_t i = 0; i < 4; ++i) {
    if (_sensors[i].collect_value(_gpio_rx, _gpio_tx)) {
      success = true;
    }
  }
  return success;
}

bool P004_data_struct::read_temp(float& value, uint8_t index) const {
    if (index >= 4) return false;
  if ((_sensors[index].addr == 0) || !_sensors[index].valueRead) { return false; }

  value = _sensors[index].value;
  return true;
}

String P004_data_struct::get_formatted_address(uint8_t index) const {
    if (index < 4) return _sensors[index].get_formatted_address();
    return "";
}

bool P004_data_struct::measurement_active() const {
  for (uint8_t i = 0; i < 4; ++i) {
    if (_sensors[i].measurementActive) { return true; }
  }

  return false;
}

bool P004_data_struct::measurement_active(uint8_t index) const {
  if (index < 4) {
    return _sensors[index].measurementActive;
  }
  return false;
}

void P004_data_struct::set_measurement_inactive() {
  for (uint8_t i = 0; i < 4; ++i) {
    _sensors[i].set_measurement_inactive();
  }
}

Dallas_SensorData P004_data_struct::get_sensor_data(uint8_t index) const {
    if (index < 4) return _sensors[index];
    return Dallas_SensorData();
}


#endif // ifdef USES_P004

#include "../PluginStructs/P023_data_struct.h"
#ifdef USES_P023

#include "../Helpers/Misc.h"
#include "../Helpers/StringParser.h"


const char Plugin_023_myFont_Size[] PROGMEM = {
  0x05, // SPACE
  0x05, // !
  0x07, // "
  0x08, // #
  0x08, // $
  0x08, // %
  0x08, // &
  0x06, // '
  0x06, // (
  0x06, // )
  0x08, // *
  0x08, // +
  0x05, // ,
  0x08, // -
  0x05, // .
  0x08, // /
  0x08, // 0
  0x07, // 1
  0x08, // 2
  0x08, // 3
  0x08, // 4
  0x08, // 5
  0x08, // 6
  0x08, // 7
  0x08, // 8
  0x08, // 9
  0x06, // :
  0x06, // ;
  0x07, // <
  0x08, // =
  0x07, // >
  0x08, // ?
  0x08, // @
  0x08, // A
  0x08, // B
  0x08, // C
  0x08, // D
  0x08, // E
  0x08, // F
  0x08, // G
  0x08, // H
  0x06, // I
  0x08, // J
  0x08, // K
  0x08, // L
  0x08, // M
  0x08, // N
  0x08, // O
  0x08, // P
  0x08, // Q
  0x08, // R
  0x08, // S
  0x08, // T
  0x08, // U
  0x08, // V
  0x08, // W
  0x08, // X
  0x08, // Y
  0x08, // Z
  0x06, // [
  0x08, // BACKSLASH
  0x06, // ]
  0x08, // ^
  0x08, // _
  0x06, // `
  0x08, // a
  0x08, // b
  0x07, // c
  0x08, // d
  0x08, // e
  0x07, // f
  0x08, // g
  0x08, // h
  0x05, // i
  0x06, // j
  0x07, // k
  0x06, // l
  0x08, // m
  0x07, // n
  0x07, // o
  0x07, // p
  0x07, // q
  0x07, // r
  0x07, // s
  0x06, // t
  0x07, // u
  0x08, // v
  0x08, // w
  0x08, // x
  0x07, // y
  0x08, // z
  0x06, // {
  0x05, // |
  0x06, // }
  0x08, // ~
  0x08  // DEL
};


const char Plugin_023_myFont[][8] PROGMEM = {
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // SPACE
  { 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00, 0x00 }, // !
  { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00 }, // "
  { 0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, 0x00 }, // #
  { 0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, 0x00 }, // $
  { 0x00, 0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 0x00 }, // %
  { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x00 }, // &
  { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00 }, // '
  { 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00 }, // (
  { 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00 }, // )
  { 0x00, 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00, 0x00 }, // *
  { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00 }, // +
  { 0x00, 0xA0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 }, // ,
  { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00 }, // -
  { 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 }, // .
  { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00 }, // /
  { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00 }, // 0
  { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00 }, // 1
  { 0x00, 0x62, 0x51, 0x49, 0x49, 0x46, 0x00, 0x00 }, // 2
  { 0x00, 0x22, 0x41, 0x49, 0x49, 0x36, 0x00, 0x00 }, // 3
  { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00 }, // 4
  { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00 }, // 5
  { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00 }, // 6
  { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00 }, // 7
  { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00 }, // 8
  { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00 }, // 9
  { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00 }, // :
  { 0x00, 0x00, 0xAC, 0x6C, 0x00, 0x00, 0x00, 0x00 }, // ;
  { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00 }, // <
  { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00 }, // =
  { 0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00, 0x00 }, // >
  { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06, 0x00, 0x00 }, // ?
  { 0x00, 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00, 0x00 }, // @
  { 0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E, 0x00, 0x00 }, // A
  { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00 }, // B
  { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00 }, // C
  { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00 }, // D
  { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00 }, // E
  { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00 }, // F
  { 0x00, 0x3E, 0x41, 0x41, 0x51, 0x72, 0x00, 0x00 }, // G
  { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00 }, // H
  { 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00, 0x00 }, // I
  { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 0x00 }, // J
  { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00 }, // K
  { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00 }, // L
  { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00, 0x00 }, // M
  { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00 }, // N
  { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00 }, // O
  { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00 }, // P
  { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 0x00 }, // Q
  { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00 }, // R
  { 0x00, 0x26, 0x49, 0x49, 0x49, 0x32, 0x00, 0x00 }, // S
  { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, 0x00 }, // T
  { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 0x00 }, // U
  { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 0x00 }, // V
  { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, 0x00 }, // W
  { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 0x00 }, // X
  { 0x00, 0x03, 0x04, 0x78, 0x04, 0x03, 0x00, 0x00 }, // Y
  { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x00 }, // Z
  { 0x00, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00, 0x00 }, // [
  { 0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00 }, // BACKSLASH
  { 0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00, 0x00 }, // ]
  { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00, 0x00 }, // ^
  { 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 }, // _
  { 0x00, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00 }, // `
  { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78, 0x00, 0x00 }, // a
  { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, 0x00 }, // b
  { 0x00, 0x38, 0x44, 0x44, 0x28, 0x00, 0x00, 0x00 }, // c
  { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00, 0x00 }, // d
  { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 0x00 }, // e
  { 0x00, 0x08, 0x7E, 0x09, 0x02, 0x00, 0x00, 0x00 }, // f
  { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C, 0x00, 0x00 }, // g
  { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00 }, // h
  { 0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00 }, // i
  { 0x00, 0x80, 0x84, 0x7D, 0x00, 0x00, 0x00, 0x00 }, // j
  { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00 }, // k
  { 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00 }, // l
  { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, 0x00 }, // m
  { 0x00, 0x7C, 0x08, 0x04, 0x7C, 0x00, 0x00, 0x00 }, // n
  { 0x00, 0x38, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00 }, // o
  { 0x00, 0xFC, 0x24, 0x24, 0x18, 0x00, 0x00, 0x00 }, // p
  { 0x00, 0x18, 0x24, 0x24, 0xFC, 0x00, 0x00, 0x00 }, // q
  { 0x00, 0x00, 0x7C, 0x08, 0x04, 0x00, 0x00, 0x00 }, // r
  { 0x00, 0x48, 0x54, 0x54, 0x24, 0x00, 0x00, 0x00 }, // s
  { 0x00, 0x04, 0x7F, 0x44, 0x00, 0x00, 0x00, 0x00 }, // t
  { 0x00, 0x3C, 0x40, 0x40, 0x7C, 0x00, 0x00, 0x00 }, // u
  { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, 0x00 }, // v
  { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, 0x00 }, // w
  { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 0x00 }, // x
  { 0x00, 0x1C, 0xA0, 0xA0, 0x7C, 0x00, 0x00, 0x00 }, // y
  { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x00 }, // z
  { 0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00, 0x00 }, // {
  { 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00 }, // |
  { 0x00, 0x41, 0x36, 0x08, 0x00, 0x00, 0x00, 0x00 }, // }
  { 0x00, 0x02, 0x01, 0x01, 0x02, 0x01, 0x00, 0x00 }, // ~
  { 0x00, 0x02, 0x05, 0x05, 0x02, 0x00, 0x00, 0x00 }  // DEL
};


P023_data_struct::P023_data_struct(uint8_t _address,   uint8_t _type, P023_data_struct::Spacing _font_spacing, uint8_t _displayTimer,uint8_t _use_sh1106)
  :  address(_address), type(_type),  font_spacing(_font_spacing),  displayTimer(_displayTimer), use_sh1106(_use_sh1106)
{}

void P023_data_struct::setDisplayTimer(uint8_t _displayTimer) {
  displayOn();
  displayTimer = _displayTimer;
}

void P023_data_struct::checkDisplayTimer() {
  if (displayTimer > 0)
  {
    displayTimer--;

    if (displayTimer == 0) {
      displayOff();
    }
  }
}

// Perform some specific changes for OLED display
String P023_data_struct::parseTemplate(String& tmpString, uint8_t lineSize) {
  String result             = parseTemplate_padded(tmpString, lineSize);
  const char degree[3]      = { 0xc2, 0xb0, 0 }; // Unicode degree symbol
  const char degree_oled[2] = { 0x7F, 0 };       // P023_OLED degree symbol

  result.replace(degree, degree_oled);
  return result;
}

void P023_data_struct::resetDisplay()
{
  displayOff();
  clearDisplay();
  displayOn();
}

void P023_data_struct::StartUp_OLED()
{
  init_OLED();
  resetDisplay();
  displayOff();
  setXY(0, 0);
  clearDisplay();
  displayOn();
}

void P023_data_struct::displayOn()
{
  sendCommand(0xaf); // display on
}

void P023_data_struct::displayOff()
{
  sendCommand(0xae); // display off
}

void P023_data_struct::clearDisplay()
{
  unsigned char i, k;

  for (k = 0; k < 8; k++)
  {
    setXY(k, 0);
    {
      for (i = 0; i < 128; i++) // clear all COL
      {
        sendChar(0);            // clear all COL
      }
    }
  }
}

// Actually this sends a byte, not a char to draw in the display.
void P023_data_struct::sendChar(unsigned char data)
{
  Wire.beginTransmission(address); // begin transmitting
  Wire.write(0x40);                // data mode
  Wire.write(data);
  Wire.endTransmission();          // stop transmitting
}

// Prints a display char (not just a byte) in coordinates X Y,
// currently unused:
// void P023_data_struct::Plugin_023_sendCharXY(unsigned char data, int X, int Y)
// {
//   //if (interrupt && !doing_menu) return; // Stop printing only if interrupt is call but not in button functions
//   setXY(X, Y);
//   Wire.beginTransmission(Plugin_023_OLED_address); // begin transmitting
//   Wire.write(0x40);//data mode
//
//   for (int i = 0; i < 8; i++)
//     Wire.write(pgm_read_byte(Plugin_023_myFont[data - 0x20] + i));
//
//   Wire.endTransmission();    // stop transmitting
// }


void P023_data_struct::sendCommand(unsigned char com)
{
  Wire.beginTransmission(address); // begin transmitting
  Wire.write(0x80);                // command mode
  Wire.write(com);
  Wire.endTransmission();          // stop transmitting
}

// Set the cursor position in a 16 COL * 8 ROW map (128x64 pixels)
// or 8 COL * 5 ROW map (64x48 pixels)
void P023_data_struct::setXY(unsigned char row, unsigned char col)
{
  unsigned char col_offset = 0;

  if (use_sh1106) {
      col_offset = 0x02;    // offset of 2 when using SSH1106 controller
  }

  switch (type)
  {
    case OLED_64x48:
      col += 4;
      break;
    case OLED_64x48 | OLED_rotated:
      col += 4;
      row += 2;
  }

  sendCommand(0xb0 + row);                              // set page address
  sendCommand(0x00 + ((8 * col + col_offset) & 0x0f));  // set low col address
  sendCommand(0x10 + (((8 * col) >> 4) & 0x0f));          // set high col address
}

// Prints a string regardless the cursor position.
// unused:
// void P023_data_struct::Plugin_023_sendStr(unsigned char *string)
// {
//   unsigned char i = 0;
//   while (*string)
//   {
//     for (i = 0; i < 8; i++)
//     {
//       sendChar(pgm_read_byte(Plugin_023_myFont[*string - 0x20] + i));
//     }
//     string++;
//   }
// }


// Prints a string in coordinates X Y, being multiples of 8.
// This means we have 16 COLS (0-15) and 8 ROWS (0-7).
void P023_data_struct::sendStrXY(const char *string, int X, int Y)
{
  setXY(X, Y);
  unsigned char i             = 0;
  unsigned char char_width    = 0;
  unsigned char currentPixels = Y * 8; // setXY always uses char_width = 8, Y = 0-based
  unsigned char maxPixels     = 128;   // Assumed default display width

  switch (type) {                      // Cater for that 1 smaller size display
    case OLED_64x48:
    case OLED_64x48 | OLED_rotated:
      maxPixels = 64;
      break;
  }

  while (*string && currentPixels < maxPixels) // Prevent display overflow on the character level
  {
    switch (font_spacing)
    {
      case Spacing::optimized:
        char_width = pgm_read_byte(&(Plugin_023_myFont_Size[*string - 0x20]));
        break;
      default:
        char_width = 8;
    }

    for (i = 0; i < char_width && currentPixels + i < maxPixels; i++) // Prevent display overflow on the pixel-level
    {
      sendChar(pgm_read_byte(Plugin_023_myFont[*string - 0x20] + i));
    }
    currentPixels += char_width;
    string++;
  }
}

void P023_data_struct::init_OLED()
{
  unsigned char multiplex;
  unsigned char compins;

  switch (type)
  {
    case OLED_128x32:
      multiplex = 0x1F;
      compins   = 0x02;
      break;
    default:
      multiplex = 0x3F;
      compins   = 0x12;
  }

  sendCommand(0xAE);       // display off
  sendCommand(0xD5);       // SETDISPLAYCLOCKDIV
  sendCommand(0x80);       // the suggested ratio 0x80
  sendCommand(0xA8);       // SSD1306_SETMULTIPLEX
  sendCommand(multiplex);  // 0x1F if 128x32, 0x3F if others (e.g. 128x64)
  sendCommand(0xD3);       // SETDISPLAYOFFSET
  sendCommand(0x00);       // no offset
  sendCommand(0x40 | 0x0); // SETSTARTLINE
  if (use_sh1106) {
    sendCommand(0xAD);       // CHARGEPUMP mode SH1106
    sendCommand(0x8B);       // CHARGEPUMP On SH1106
    sendCommand(0x32);       // CHARGEPUMP voltage 8V SH1106
    sendCommand(0x81);       // SETCONTRAS
    sendCommand(0x80);       // SH1106
  } else {
    sendCommand(0x8D);       // CHARGEPUMP
    sendCommand(0x14);
    sendCommand(0x81);       // SETCONTRAS
    sendCommand(0xCF);
  }
  sendCommand(0x20);       // MEMORYMODE
  sendCommand(0x00);       // 0x0 act like ks0108
  sendCommand(0xA0);       // 128x32 ???
  sendCommand(0xC0);       // 128x32 ???
  sendCommand(0xDA);       // COMPINS
  sendCommand(compins);    // 0x02 if 128x32, 0x12 if others (e.g. 128x64)
  sendCommand(0xD9);       // SETPRECHARGE
  sendCommand(0xF1);
  sendCommand(0xDB);       // SETVCOMDETECT
  sendCommand(0x40);
  sendCommand(0xA4);       // DISPLAYALLON_RESUME
  sendCommand(0xA6);       // NORMALDISPLAY

  clearDisplay();
  sendCommand(0x2E);       // stop scroll
  sendCommand(0x20);       // Set Memory Addressing Mode
  sendCommand(0x00);       // Set Memory Addressing Mode ab Horizontal addressing mode
}

#endif // ifdef USES_P023

#include "../PluginStructs/P116_data_struct.h"

#ifdef USES_P116

# include "../Helpers/Hardware.h"

/****************************************************************************
 * ST77xx_type_toString: Display-value for the device selected
 ***************************************************************************/
const __FlashStringHelper* ST77xx_type_toString(ST77xx_type_e device) {
  switch (device) {
    case ST77xx_type_e::ST7735s_128x128: return F("ST7735 128 x 128px");
    case ST77xx_type_e::ST7735s_128x160: return F("ST7735 128 x 160px");
    case ST77xx_type_e::ST7735s_80x160: return F("ST7735 80 x 160px");
    case ST77xx_type_e::ST7735s_80x160_M5: return F("ST7735 80 x 160px (Color inverted)");
    case ST77xx_type_e::ST7789vw_240x320: return F("ST7789 240 x 320px");
    case ST77xx_type_e::ST7789vw_240x240: return F("ST7789 240 x 240px");
    case ST77xx_type_e::ST7789vw_240x280: return F("ST7789 240 x 280px");
    case ST77xx_type_e::ST7789vw_135x240: return F("ST7789 135 x 240px");
    case ST77xx_type_e::ST7796s_320x480: return F("ST7796 320 x 480px");
    case ST77xx_type_e::ST77xx_MAX: break;
  }
  return F("Unsupported type!");
}

/****************************************************************************
 * ST77xx_type_toResolution: X and Y resolution for the selected type
 ***************************************************************************/
void ST77xx_type_toResolution(ST77xx_type_e device, uint16_t& x, uint16_t& y) {
  switch (device) {
    case ST77xx_type_e::ST7735s_128x128:
      x = 128;
      y = 128;
      break;
    case ST77xx_type_e::ST7735s_128x160:
      x = 128;
      y = 160;
      break;
    case ST77xx_type_e::ST7735s_80x160_M5:
    case ST77xx_type_e::ST7735s_80x160:
      x = 80;
      y = 160;
      break;
    case ST77xx_type_e::ST7789vw_240x320:
      x = 240;
      y = 320;
      break;
    case ST77xx_type_e::ST7789vw_240x240:
      x = 240;
      y = 240;
      break;
    case ST77xx_type_e::ST7789vw_240x280:
      x = 240;
      y = 280;
      break;
    case ST77xx_type_e::ST7789vw_135x240:
      x = 135;
      y = 240;
      break;
    case ST77xx_type_e::ST7796s_320x480:
      x = 320;
      y = 480;
      break;
    case ST77xx_type_e::ST77xx_MAX:
      break;
  }
}

/****************************************************************************
 * P116_CommandTrigger_toString: return the command string selected
 ***************************************************************************/
const __FlashStringHelper* P116_CommandTrigger_toString(P116_CommandTrigger cmd) {
  switch (cmd) {
    case P116_CommandTrigger::tft: return F("tft");
    case P116_CommandTrigger::st7735: return F("st7735");
    case P116_CommandTrigger::st7789: return F("st7789");
    case P116_CommandTrigger::st7796: return F("st7796");
    case P116_CommandTrigger::MAX: return F("None");
    case P116_CommandTrigger::st77xx: break;
  }
  return F("st77xx"); // Default command trigger
}

/****************************************************************************
 * Constructor
 ***************************************************************************/
P116_data_struct::P116_data_struct(ST77xx_type_e       device,
                                   uint8_t             rotation,
                                   uint8_t             fontscaling,
                                   AdaGFXTextPrintMode textmode,
                                   int8_t              backlightPin,
                                   uint8_t             backlightPercentage,
                                   uint32_t            displayTimer,
                                   String              commandTrigger,
                                   uint16_t            fgcolor,
                                   uint16_t            bgcolor,
                                   bool                textBackFill)
  : _device(device), _rotation(rotation), _fontscaling(fontscaling), _textmode(textmode), _backlightPin(backlightPin),
  _backlightPercentage(backlightPercentage), _displayTimer(displayTimer), _displayTimeout(displayTimer),
  _commandTrigger(commandTrigger), _fgcolor(fgcolor), _bgcolor(bgcolor), _textBackFill(textBackFill)
{
  ST77xx_type_toResolution(_device, _xpix, _ypix);

  updateFontMetrics();
  _commandTrigger.toLowerCase();
  _commandTriggerCmd  = _commandTrigger;
  _commandTriggerCmd += F("cmd");
}

/****************************************************************************
 * Destructor
 ***************************************************************************/
P116_data_struct::~P116_data_struct() {
  cleanup();
}

/****************************************************************************
 * plugin_init: Initialize display
 ***************************************************************************/
bool P116_data_struct::plugin_init(struct EventStruct *event) {
  bool success = false;

  ButtonState     = false; // button not touched
  ButtonLastState = 0xFF;  // Last state checked (debouncing in progress)
  DebounceCounter = 0;     // debounce counter

  if (nullptr == st77xx) {
    addLog(LOG_LEVEL_INFO, F("ST77xx: Init start."));
    uint8_t initRoptions = 0xFF;

    switch (_device) {
      case ST77xx_type_e::ST7735s_128x128:

        if (initRoptions == 0xFF) {
          initRoptions = INITR_144GREENTAB; // 128x128px
        }

      // fall through
      case ST77xx_type_e::ST7735s_128x160:

        if (initRoptions == 0xFF) {
          initRoptions = INITR_BLACKTAB; // 128x160px
        }

      // fall through
      case ST77xx_type_e::ST7735s_80x160_M5:

        if (initRoptions == 0xFF) {
          initRoptions = INITR_GREENTAB160x80; // 80x160px ST7735sv, inverted (M5Stack StickC)
        }

      // fall through
      case ST77xx_type_e::ST7735s_80x160:
      {
        if (initRoptions == 0xFF) {
          initRoptions = INITR_MINI160x80; // 80x160px
        }

        st7735 = new (std::nothrow) Adafruit_ST7735(PIN(0), PIN(1), PIN(2));

        if (nullptr != st7735) {
          st7735->initR(initRoptions); // initialize a ST7735s chip
          st77xx = st7735;             // pass pointer after initialization
        }
        break;
      }
      case ST77xx_type_e::ST7789vw_240x320: // Fall through
      case ST77xx_type_e::ST7789vw_240x240:
      case ST77xx_type_e::ST7789vw_240x280:
      case ST77xx_type_e::ST7789vw_135x240:
      {
        st7789 = new (std::nothrow) Adafruit_ST7789(PIN(0), PIN(1), PIN(2));

        if (nullptr != st7789) {
          st7789->init(_xpix, _ypix, SPI_MODE2);
          st77xx = st7789;
        }
        break;
      }
      case ST77xx_type_e::ST7796s_320x480:
      {
        st7796 = new (std::nothrow) Adafruit_ST7796S_kbv(PIN(0), PIN(1), PIN(2));

        if (nullptr != st7796) {
          st7796->begin();
          st77xx = st7796;
        }
        break;
      }
      case ST77xx_type_e::ST77xx_MAX:
        break;
    }

    # ifndef BUILD_NO_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log;
      log.reserve(90);
      log += F("ST77xx: Init done, address: 0x");
      log += String(reinterpret_cast<ulong>(st77xx), HEX);
      log += ' ';

      if (nullptr == st77xx) {
        log += F("in");
      }
      log += F("valid, commands: ");
      log += _commandTrigger;
      log += F(", display: ");
      log += ST77xx_type_toString(_device);
      addLogMove(LOG_LEVEL_INFO, log);
    }
    # endif // ifndef BUILD_NO_DEBUG
  } else {
    addLog(LOG_LEVEL_INFO, F("ST77xx: No init?"));
  }

  if (nullptr != st77xx) {
    gfxHelper = new (std::nothrow) AdafruitGFX_helper(st77xx,
                                                      _commandTrigger,
                                                      _xpix,
                                                      _ypix,
                                                      AdaGFXColorDepth::FullColor,
                                                      _textmode,
                                                      _fontscaling,
                                                      _fgcolor,
                                                      _bgcolor,
                                                      true,
                                                      _textBackFill);

    displayOnOff(true);

    gfxHelper->setRotation(_rotation);
    st77xx->fillScreen(_bgcolor);             // fill screen with black color
    st77xx->setTextColor(_fgcolor, _bgcolor); // set text color to white and black background

    # ifdef P116_SHOW_SPLASH
    uint16_t yPos = 0;
    gfxHelper->printText(String(F("ESPEasy")).c_str(), 0, yPos, 3, ST77XX_WHITE, ST77XX_BLUE);
    yPos += (3 * _fontheight);
    gfxHelper->printText(String(F("ST77xx")).c_str(),  0, yPos, 2, ST77XX_BLUE,  ST77XX_WHITE);
    delay(100); // Splash
    # endif // ifdef P116_SHOW_SPLASH

    gfxHelper->setColumnRowMode(bitRead(P116_CONFIG_FLAGS, P116_CONFIG_FLAG_USE_COL_ROW));
    st77xx->setTextSize(_fontscaling); // Handles 0 properly, text size, default 1 = very small
    st77xx->setCursor(0, 0);           // move cursor to position (0, 0) pixel
    updateFontMetrics();


    if (P116_CONFIG_BUTTON_PIN != -1) {
      pinMode(P116_CONFIG_BUTTON_PIN, INPUT_PULLUP);
    }
    success = true;
  }
  return success;
}

/****************************************************************************
 * updateFontMetrics: recalculate x and y columns, based on font size and font scale
 ***************************************************************************/
void P116_data_struct::updateFontMetrics() {
  if (nullptr != gfxHelper) {
    gfxHelper->getTextMetrics(_textcols, _textrows, _fontwidth, _fontheight, _fontscaling, _heightOffset, _xpix, _ypix);
    gfxHelper->getColors(_fgcolor, _bgcolor);
  } else {
    _textcols = _xpix / (_fontwidth * _fontscaling);
    _textrows = _ypix / (_fontheight * _fontscaling);
  }
}

/****************************************************************************
 * plugin_exit: De-initialize before destruction
 ***************************************************************************/
bool P116_data_struct::plugin_exit(struct EventStruct *event) {
  addLog(LOG_LEVEL_INFO, F("ST77xx: Exit."));

  if ((nullptr != st77xx) && bitRead(P116_CONFIG_FLAGS, P116_CONFIG_FLAG_CLEAR_ON_EXIT)) {
    st77xx->fillScreen(ADAGFX_BLACK); // fill screen with black color
    displayOnOff(false);
  }
  cleanup();
  return true;
}

/****************************************************************************
 * cleanup: De-initialize pointers
 ***************************************************************************/
void P116_data_struct::cleanup() {
  if (nullptr != gfxHelper) { delete gfxHelper; }
  gfxHelper = nullptr;

  if (nullptr != st7735) { delete st7735; }
  st7735 = nullptr;

  if (nullptr != st7789) { delete st7789; }
  st7789 = nullptr;
  st77xx = nullptr; // Only used as a proxy
}

/****************************************************************************
 * plugin_read: Re-draw the default content
 ***************************************************************************/
bool P116_data_struct::plugin_read(struct EventStruct *event) {
  if (nullptr != st77xx) {
    String strings[P116_Nlines];
    LoadCustomTaskSettings(event->TaskIndex, strings, P116_Nlines, 0);

    bool hasContent = false;

    for (uint8_t x = 0; x < P116_Nlines && !hasContent; x++) {
      hasContent = !strings[x].isEmpty();
    }

    if (hasContent) {
      gfxHelper->setColumnRowMode(false); // Turn off column mode

      int yPos = 0;

      for (uint8_t x = 0; x < P116_Nlines; x++) {
        String newString = AdaGFXparseTemplate(strings[x], _textcols, gfxHelper);

        # if ADAGFX_PARSE_SUBCOMMAND
        updateFontMetrics();
        # endif // if ADAGFX_PARSE_SUBCOMMAND

        if (yPos < _ypix) {
          gfxHelper->printText(newString.c_str(), 0, yPos, _fontscaling, _fgcolor, _bgcolor);
        }
        delay(0);
        yPos += (_fontheight * _fontscaling);
      }
      gfxHelper->setColumnRowMode(bitRead(P116_CONFIG_FLAGS, P116_CONFIG_FLAG_USE_COL_ROW)); // Restore column mode
      int16_t curX, curY;
      gfxHelper->getCursorXY(curX, curY);                                                    // Get current X and Y coordinates,
      UserVar[event->BaseVarIndex]     = curX;                                               // and put into Values
      UserVar[event->BaseVarIndex + 1] = curY;
    }
  }
  return false; // Always return false, so no attempt to send to
                // Controllers or generate events is started
}

/****************************************************************************
 * plugin_ten_per_second: check button, if any, that wakes up the display
 ***************************************************************************/
bool P116_data_struct::plugin_ten_per_second(struct EventStruct *event) {
  if ((P116_CONFIG_BUTTON_PIN != -1) && (getButtonState()) && (nullptr != st77xx)) {
    displayOnOff(true);
    markButtonStateProcessed();
  }
  return true;
}

/****************************************************************************
 * plugin_once_a_second: Count down display timer, if any, and turn display off if countdown reached
 ***************************************************************************/
bool P116_data_struct::plugin_once_a_second(struct EventStruct *event) {
  if (_displayTimer > 0) {
    _displayTimer--;

    if ((nullptr != st77xx) && (_displayTimer == 0)) {
      displayOnOff(false);
    }
  }
  return true;
}

/****************************************************************************
 * plugin_write: Handle commands
 ***************************************************************************/
bool P116_data_struct::plugin_write(struct EventStruct *event, const String& string) {
  bool   success = false;
  String cmd     = parseString(string, 1);

  if ((nullptr != st77xx) && cmd.equals(_commandTriggerCmd)) {
    String arg1 = parseString(string, 2);
    success = true;

    if (arg1.equals(F("off"))) {
      displayOnOff(false);
    }
    else if (arg1.equals(F("on"))) {
      displayOnOff(true);
    }
    else if (arg1.equals(F("clear"))) {
      st77xx->fillScreen(_bgcolor);
    }
    else if (arg1.equals(F("backlight"))) {
      String arg2 = parseString(string, 3);
      int    nArg2;

      if ((P116_CONFIG_BACKLIGHT_PIN != -1) && // All is valid?
          validIntFromString(arg2, nArg2) &&
          (nArg2 > 0) &&
          (nArg2 <= 100)) {
        P116_CONFIG_BACKLIGHT_PERCENT = nArg2; // Set but don't store
        displayOnOff(true);
      } else {
        success = false;
      }
    } else {
      success = false;
    }
  }
  else if (st77xx && (cmd.equals(_commandTrigger) ||
                      (gfxHelper && gfxHelper->isAdaGFXTrigger(cmd)))) {
    success = true;

    if (!bitRead(P116_CONFIG_FLAGS, P116_CONFIG_FLAG_NO_WAKE)) { // Wake display?
      displayOnOff(true);
    }

    if (nullptr != gfxHelper) {
      String tmp = string;

      // Hand it over after replacing variables
      success = gfxHelper->processCommand(AdaGFXparseTemplate(tmp, _textcols, gfxHelper));

      updateFontMetrics(); // Font or color may have changed

      if (success) {
        int16_t curX, curY;
        gfxHelper->getCursorXY(curX, curY); // Get current X and Y coordinates, and put into Values
        UserVar[event->BaseVarIndex]     = curX;
        UserVar[event->BaseVarIndex + 1] = curY;
      }
    }
  }
  return success;
}

/****************************************************************************
 * displayOnOff: Turn display on or off
 ***************************************************************************/
void P116_data_struct::displayOnOff(bool state) {
  if (_backlightPin != -1) {
    # if defined(ESP8266)
    analogWrite(_backlightPin, state ? ((1024 / 100) * _backlightPercentage) : 0);
    # endif // if defined(ESP8266)
    # if defined(ESP32)
    analogWriteESP32(_backlightPin, state ? ((1024 / 100) * _backlightPercentage) : 0, 0);
    # endif // if defined(ESP32)
  }
  st77xx->enableDisplay(state); // Display on
  _displayTimer = (state ? _displayTimeout : 0);
}

/****************************************************************************
 * registerButtonState: the button has been pressed, apply some debouncing
 ***************************************************************************/
void P116_data_struct::registerButtonState(uint8_t newButtonState, bool bPin3Invers) {
  if ((ButtonLastState == 0xFF) || (bPin3Invers != (!!newButtonState))) {
    ButtonLastState = newButtonState;
    DebounceCounter++;
  } else {
    ButtonLastState = 0xFF; // Reset
    DebounceCounter = 0;
    ButtonState     = false;
  }

  if ((ButtonLastState == newButtonState) &&
      (DebounceCounter >= P116_DebounceTreshold)) {
    ButtonState = true;
  }
}

/****************************************************************************
 * markButtonStateProcessed: reset the button state
 ***************************************************************************/
void P116_data_struct::markButtonStateProcessed() {
  ButtonState     = false;
  DebounceCounter = 0;
}

#endif // ifdef USES_P116

#include "../PluginStructs/P133_data_struct.h"

#ifdef USES_P133

/**************************************************************************
* Constructor
**************************************************************************/
P133_data_struct::P133_data_struct(P133_selectMode_e   selectMode,
                                   ltr390_gain_t       uvGain,
                                   ltr390_resolution_t uvResolution,
                                   ltr390_gain_t       alsGain,
                                   ltr390_resolution_t alsResolution,
                                   bool                initReset)
  : _selectMode(selectMode), _uvGain(uvGain), _uvResolution(uvResolution),
  _alsGain(alsGain), _alsResolution(alsResolution), _initReset(initReset)
{}

/*****************************************************
* Destructor
*****************************************************/
P133_data_struct::~P133_data_struct() {
  if (nullptr != ltr390) { delete ltr390; }
  ltr390 = nullptr;
}

/*****************************************************
* plugin_read
*****************************************************/
bool P133_data_struct::plugin_read(struct EventStruct *event)           {
  if (initialised) {
    // Last obtained values
    UserVar[event->BaseVarIndex]     = uvValue;
    UserVar[event->BaseVarIndex + 1] = uviValue;
    UserVar[event->BaseVarIndex + 2] = alsValue;
    UserVar[event->BaseVarIndex + 3] = luxValue;
    sensorRead                       = false;
    return true;
  }
  return false;
}

/*****************************************************
* plugin_ten_per_second
*****************************************************/
bool P133_data_struct::plugin_ten_per_second(struct EventStruct *event) {
  if (initialised && ltr390->newDataAvailable()) {
    if (mode == LTR390_MODE_UVS) {
      uvValue  = ltr390->readUVS();
      uviValue = ltr390->getUVI();
    } else {
      alsValue = ltr390->readALS();
      luxValue = ltr390->getLUX();
    }

    // Dual Mode: Switch mode after measuring a value for a few loop cycles, so the data is stable
    if (_selectMode == P133_selectMode_e::DualMode) {
      if ((loopCounter == 0) || (loopCounter > P133_LOOP_INTERVAL)) {
        if (mode == LTR390_MODE_UVS) {
          mode = LTR390_MODE_ALS;
          ltr390->setGain(_alsGain);
          ltr390->setResolution(_alsResolution);
        } else {
          mode = LTR390_MODE_UVS;
          ltr390->setGain(_uvGain);
          ltr390->setResolution(_uvResolution);
        }
        ltr390->setMode(mode);

        loopCounter = P133_LOOP_INTERVAL;
      } else {
        loopCounter--;
      }
    }

    # if PLUGIN_133_DEBUG

    if (!sensorRead && loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("LTR390: data read. Mode: ");
      log += mode;
      log += F(", UV: ");
      log += uvValue;
      log += F(", UVindex: ");
      log += uviValue;
      log += F(", ALS: ");
      log += alsValue;
      log += F(", Lux: ");
      log += luxValue;
      addLogMove(LOG_LEVEL_INFO, log);
    }
    # endif // if PLUGIN_133_DEBUG
    sensorRead = true;
    return true;
  }
  return false;
}

/**************************************************************************
* plugin_init Initialize sensor and prepare for reading
**************************************************************************/
bool P133_data_struct::plugin_init(struct EventStruct *event) {
  if (!initialised) {
    initialised = init_sensor(); // Check if device is present
  }

  if (initialised) {
    switch (_selectMode) {
      case P133_selectMode_e::DualMode:
      case P133_selectMode_e::UVMode:
        mode = LTR390_MODE_UVS;
        ltr390->setGain(_uvGain);
        ltr390->setResolution(_uvResolution);
        break;
      case P133_selectMode_e::ALSMode:
        mode = LTR390_MODE_ALS;
        ltr390->setGain(_alsGain);
        ltr390->setResolution(_alsResolution);
        break;
    }
    ltr390->setMode(mode);

    # if PLUGIN_133_DEBUG

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("LTR390: Configured, mode: ");
      log += mode;
      log += F(", UV gain: ");
      log += _uvGain;
      log += F(", UV resolution: ");
      log += _uvResolution;
      log += F(", ALS gain: ");
      log += _alsGain;
      log += F(", ALS resolution: ");
      log += _alsResolution;
      addLogMove(LOG_LEVEL_INFO, log);
    }
    # endif // if PLUGIN_133_DEBUG
  }

  return initialised;
}

/**************************************************************************
* Check LTR390 presence and initialize
**************************************************************************/
bool P133_data_struct::init_sensor() {
  if (!initialised) {
    ltr390 = new (std::nothrow) UVlight_LTR390();

    if (nullptr != ltr390) {
      initialised = ltr390->init(_initReset);
    }

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("LTR390: Initialized: ");
      log += initialised ? F("OK") : F("ERROR");
      log += F(", chip ID: 0x");
      log += String(ltr390->getChipID(), HEX);
      addLogMove(LOG_LEVEL_INFO, log);
    }
  }

  return initialised;
}

#endif // ifdef USES_P133

#include "../PluginStructs/P081_data_struct.h"

#ifdef USES_P081

P081_data_struct::P081_data_struct(const String& expression)
{
  const char *error;

  memset(&_expr, 0, sizeof(_expr));
  cron_parse_expr(expression.c_str(), &_expr, &error);

  if (!error) {
    _initialized = true;
  } else {
    _error = String(error);
  }
}

bool P081_data_struct::hasError(String& error) const {
  if (_initialized) { return false; }
  error = _error;
  return true;
}

time_t P081_data_struct::get_cron_next(time_t date) const {
  if (!_initialized) { return CRON_INVALID_INSTANT; }
  return cron_next((cron_expr *)&_expr, date);
}

time_t P081_data_struct::get_cron_prev(time_t date) const {
  if (!_initialized) { return CRON_INVALID_INSTANT; }
  return cron_prev((cron_expr *)&_expr, date);
}

String P081_getCronExpr(taskIndex_t taskIndex)
{
  char expression[PLUGIN_081_EXPRESSION_SIZE + 1];

  ZERO_FILL(expression);
  LoadCustomTaskSettings(taskIndex, reinterpret_cast<uint8_t *>(&expression), PLUGIN_081_EXPRESSION_SIZE);
  String res(expression);

  res.trim();
  return res;
}

time_t P081_computeNextCronTime(taskIndex_t taskIndex, time_t last)
{
  P081_data_struct *P081_data =
    static_cast<P081_data_struct *>(getPluginTaskData(taskIndex));

  if ((nullptr != P081_data) && P081_data->isInitialized()) {
    //    int32_t freeHeapStart = ESP.getFreeHeap();

    time_t res = P081_data->get_cron_next(last);

    /*
        int32_t freeHeapEnd = ESP.getFreeHeap();

        if (freeHeapEnd < freeHeapStart) {
          String log = F("Cron: Free Heap Decreased: ");
          log += String(freeHeapStart - freeHeapEnd);
          log += F(" (");
          log += freeHeapStart;
          log += F(" -> ");
          log += freeHeapEnd;
          addLog(LOG_LEVEL_INFO, log);
        }
     */
    return res;
  }
  return CRON_INVALID_INSTANT;
}

time_t P081_getCronExecTime(taskIndex_t taskIndex, uint8_t varNr)
{
  return static_cast<time_t>(UserVar.getUint32(taskIndex, varNr));
}

void P081_setCronExecTimes(struct EventStruct *event, time_t lastExecTime, time_t nextExecTime) {
  UserVar.setUint32(event->TaskIndex, LASTEXECUTION, static_cast<uint32_t>(lastExecTime));
  UserVar.setUint32(event->TaskIndex, NEXTEXECUTION, static_cast<uint32_t>(nextExecTime));
}

time_t P081_getCurrentTime()
{
  node_time.now();

  // FIXME TD-er: Why work on a deepcopy of tm?
  struct tm current = node_time.tm;

  return mktime((struct tm *)&current);
}

void P081_check_or_init(struct EventStruct *event)
{
  if (node_time.systemTimePresent()) {
    const time_t current_time = P081_getCurrentTime();
    time_t last_exec_time     = P081_getCronExecTime(event->TaskIndex, LASTEXECUTION);
    time_t next_exec_time     = P081_getCronExecTime(event->TaskIndex, NEXTEXECUTION);

    // Must check if the values of LASTEXECUTION and NEXTEXECUTION make sense.
    // These can be invalid values from a reboot, or simply contain uninitialized values.
    if ((last_exec_time > current_time) || (last_exec_time == CRON_INVALID_INSTANT) || (next_exec_time == CRON_INVALID_INSTANT)) {
      // Last execution time cannot be correct.
      last_exec_time = CRON_INVALID_INSTANT;
      const time_t tmp_next = P081_computeNextCronTime(event->TaskIndex, current_time);

      if ((tmp_next < next_exec_time) || (next_exec_time == CRON_INVALID_INSTANT)) {
        next_exec_time = tmp_next;
      }
      P081_setCronExecTimes(event, CRON_INVALID_INSTANT, next_exec_time);
    }
  }
}

# if PLUGIN_081_DEBUG
void PrintCronExp(struct cron_expr_t e) {
  serialPrintln(F("===DUMP Cron Expression==="));
  serialPrint(F("Seconds:"));

  for (int i = 0; i < 8; i++)
  {
    serialPrint(e.seconds[i]);
    serialPrint(",");
  }
  serialPrintln();
  serialPrint(F("Minutes:"));

  for (int i = 0; i < 8; i++)
  {
    serialPrint(e.minutes[i]);
    serialPrint(",");
  }
  serialPrintln();
  serialPrint(F("hours:"));

  for (int i = 0; i < 3; i++)
  {
    serialPrint(e.hours[i]);
    serialPrint(",");
  }
  serialPrintln();
  serialPrint(F("months:"));

  for (int i = 0; i < 2; i++)
  {
    serialPrint(e.months[i]);
    serialPrint(",");
  }
  serialPrintln();
  serialPrint(F("days_of_week:"));

  for (int i = 0; i < 1; i++)
  {
    serialPrint(e.days_of_week[i]);
    serialPrint(",");
  }
  serialPrintln();
  serialPrint(F("days_of_month:"));

  for (int i = 0; i < 4; i++)
  {
    serialPrint(e.days_of_month[i]);
    serialPrint(",");
  }
  serialPrintln();
  serialPrintln(F("END=DUMP Cron Expression==="));
}

# endif // if PLUGIN_081_DEBUG


String P081_formatExecTime(taskIndex_t taskIndex, uint8_t varNr) {
  time_t exec_time = P081_getCronExecTime(taskIndex, varNr);

  if (exec_time != CRON_INVALID_INSTANT) {
    return ESPEasy_time::getDateTimeString(*gmtime(&exec_time));
  }
  return F("-");
}

void P081_html_show_cron_expr(struct EventStruct *event) {
  P081_data_struct *P081_data =
    static_cast<P081_data_struct *>(getPluginTaskData(event->TaskIndex));

  if ((nullptr != P081_data) && P081_data->isInitialized()) {
    String error;

    if (P081_data->hasError(error)) {
      addRowLabel(F("Error"));
      addHtml(error);
    } else {
      addRowLabel(F("Last Exec Time"));
      addHtml(P081_formatExecTime(event->TaskIndex, LASTEXECUTION));
      addRowLabel(F("Next Exec Time"));
      addHtml(P081_formatExecTime(event->TaskIndex, NEXTEXECUTION));
    }
  }
}

#endif // ifdef USES_P081

#include "../PluginStructs/P002_data_struct.h"

#ifdef USES_P002

# include "../Helpers/Rules_calculate.h"


# ifndef DEFAULT_VREF
#  define DEFAULT_VREF 1100
# endif // ifndef DEFAULT_VREF


P002_data_struct::P002_data_struct(struct EventStruct *event)
{
  _sampleMode = P002_OVERSAMPLING;

  # ifdef ESP8266
  _pin_analogRead = A0;
  # endif // ifdef ESP8266
  # ifdef ESP32
  _pin_analogRead        = CONFIG_PIN1;
  _useFactoryCalibration = useFactoryCalibration(event);
  _attenuation           = getAttenuation(event);
  const int adc = getADC_num_for_gpio(_pin_analogRead);

  if ((adc == 1) || (adc == 2)) {
    analogSetPinAttenuation(_pin_analogRead, static_cast<adc_attenuation_t>(_attenuation));
  }

  # endif // ifdef ESP32

  if (P002_CALIBRATION_ENABLED) {
    _use2pointCalibration = true;
    _calib_adc1           = P002_CALIBRATION_POINT1;
    _calib_adc2           = P002_CALIBRATION_POINT2;
    _calib_out1           = P002_CALIBRATION_VALUE1;
    _calib_out2           = P002_CALIBRATION_VALUE2;
  }
# ifndef LIMIT_BUILD_SIZE
  LoadTaskSettings(event->TaskIndex);
  _nrDecimals        = ExtraTaskSettings.TaskDeviceValueDecimals[0];
  _nrMultiPointItems = P002_NR_MULTIPOINT_ITEMS;
  _useMultipoint     = P002_MULTIPOINT_ENABLED;

  load(event);
# endif // ifndef LIMIT_BUILD_SIZE
}

# ifndef LIMIT_BUILD_SIZE
void P002_data_struct::load(struct EventStruct *event)
{
  const size_t nr_lines = P002_Nlines;

  {
    String lines[nr_lines];
    LoadCustomTaskSettings(event->TaskIndex, lines, nr_lines, 0);
    const int stored_nr_lines = lines[P002_SAVED_NR_LINES].toInt();
    _formula              = lines[P002_LINE_INDEX_FORMULA];
    _formula_preprocessed = RulesCalculate_t::preProces(_formula);

    for (size_t i = P002_LINE_IDX_FIRST_MP; i < nr_lines && static_cast<int>(i) < stored_nr_lines; i += P002_STRINGS_PER_MP) {
      float adc, value = 0.0f;

      if (validFloatFromString(lines[i], adc) && validFloatFromString(lines[i + 1], value)) {
        _multipoint.emplace_back(adc, value);
      }
    }
  }
  std::sort(_multipoint.begin(), _multipoint.end());

  _binning.resize(_multipoint.size(), 0);
  _binningRange.resize(_multipoint.size());
}

# endif // ifndef LIMIT_BUILD_SIZE

void P002_data_struct::webformLoad_2p_calibPoint(
  const __FlashStringHelper *label,
  const __FlashStringHelper *id_point,
  const __FlashStringHelper *id_value,
  int                        point,
  float                      value) const
{
  addRowLabel_tr_id(label, id_point);
  addTextBox(id_point, String(point), 10, false, false, EMPTY_STRING, F("number"));

#ifdef ESP32
  if (_useFactoryCalibration) {
    addUnit(F("mV"));
  }
#endif

  html_add_estimate_symbol();
  const unsigned int display_nrDecimals = _nrDecimals > 3 ? _nrDecimals : 3;

  addTextBox(id_value, toString(value, display_nrDecimals), 10, false, false, EMPTY_STRING, F("number"));
}

void P002_data_struct::webformLoad(struct EventStruct *event)
{
  // Output the statistics for the current settings.
  int raw_value            = 0;
  const float currentValue = P002_data_struct::getCurrentValue(event, raw_value);

# if FEATURE_PLUGIN_STATS

  if (getPluginStats(0) != nullptr) {
    getPluginStats(0)->trackPeak(raw_value);
  }
# endif // if FEATURE_PLUGIN_STATS

# ifdef ESP32
  addRowLabel(F("Analog Pin"));
  addADC_PinSelect(AdcPinSelectPurpose::ADC_Touch_HallEffect, F("taskdevicepin1"), CONFIG_PIN1);

  {
    const __FlashStringHelper *outputOptions[] = {
      F("11 dB"),
      F("6 dB"),
      F("2.5 dB"),
      F("0 dB")
    };
    const int outputOptionValues[] = {
      P002_ADC_11db,
      P002_ADC_6db,
      P002_ADC_2_5db,
      P002_ADC_0db
    };
    addFormSelector(F("Attenuation"), F("p002_attn"), 4, outputOptions, outputOptionValues, P002_ATTENUATION);
  }

# endif // ifdef ESP32

  {
    const __FlashStringHelper *outputOptions[] = {
      F("Use Current Sample"),
      F("Oversampling")
# ifndef LIMIT_BUILD_SIZE
      , F("Binning")
# endif // ifndef LIMIT_BUILD_SIZE
    };
    const int outputOptionValues[] = {
      P002_USE_CURENT_SAMPLE,
      P002_USE_OVERSAMPLING
# ifndef LIMIT_BUILD_SIZE
      , P002_USE_BINNING
# endif // ifndef LIMIT_BUILD_SIZE
    };
# ifndef LIMIT_BUILD_SIZE
    const int nrOptions = 3;
# else // ifndef LIMIT_BUILD_SIZE
    const int nrOptions = 2;
# endif // ifndef LIMIT_BUILD_SIZE
    addFormSelector(F("Oversampling"), F("p002_oversampling"), nrOptions, outputOptions, outputOptionValues, P002_OVERSAMPLING);
  }

# ifdef ESP32
  addFormSubHeader(F("Factory Calibration"));
  addFormCheckBox(F("Apply Factory Calibration"), F("p002_fac_cal"), P002_APPLY_FACTORY_CALIB, !hasADC_factory_calibration());
  addFormNote(F("When checked, reading is in mV"));

  if (hasADC_factory_calibration()) {
    addRowLabel(F("Factory Calibration Type"));
    addHtml(getADC_factory_calibration_type());
    #  if FEATURE_CHART_JS
    webformLoad_calibrationCurve(event);
    #  endif // if FEATURE_CHART_JS
    formatADC_statistics(F("Current ADC to mV"), raw_value);

    for (size_t att = 0; att < ADC_ATTEN_MAX; ++att) {
      const int   low  = esp_adc_cal_raw_to_voltage(0, &adc_chars[att]);
      const int   high = esp_adc_cal_raw_to_voltage(MAX_ADC_VALUE, &adc_chars[att]);
      const float step = static_cast<float>(high - low) / MAX_ADC_VALUE;

      String rowlabel = F("Attenuation @");
      rowlabel += AttenuationToString(static_cast<adc_atten_t>(att));
      addRowLabel(rowlabel);
      addHtml(F("Range / Step: "));
      addHtmlInt(low);
      addHtml(F(" ... "));
      addHtmlInt(high);
      addUnit(F("mV"));
      addHtml(F(" / "));
      addHtmlFloat(step, 3); // calibration output is int value in mV, so doesn't really matter how many decimals
      addUnit(F("mV"));
    }
  }
# endif // ifdef ESP32

  addFormSubHeader(F("Two Point Calibration"));

  addFormCheckBox(F("Calibration Enabled"), F("p002_cal"), P002_CALIBRATION_ENABLED);

  webformLoad_2p_calibPoint(
    F("Point 1"),
    F("p002_adc1"),
    F("p002_out1"),
    P002_CALIBRATION_POINT1,
    P002_CALIBRATION_VALUE1);
  webformLoad_2p_calibPoint(
    F("Point 2"),
    F("p002_adc2"),
    F("p002_out2"),
    P002_CALIBRATION_POINT2,
    P002_CALIBRATION_VALUE2);

  addFormNote(F("Input float values will be stored as int, calibration values will be adjusted accordingly"));

  {
    // Output the statistics for the current settings.
    if (P002_CALIBRATION_ENABLED) {
      # if FEATURE_CHART_JS
      webformLoad_2pt_calibrationCurve(event);
      # endif // if FEATURE_CHART_JS

      int minInputValue, maxInputValue;
      getInputRange(event, minInputValue, maxInputValue);

      const float minY_value         = P002_data_struct::applyCalibration(event, minInputValue);
      const float maxY_value         = P002_data_struct::applyCalibration(event, maxInputValue);
      const float current_calibrated = P002_data_struct::applyCalibration(event, currentValue);

      format_2point_calib_statistics(F("Current"), currentValue,  current_calibrated);
      format_2point_calib_statistics(F("Minimum"), minInputValue, minY_value);
      format_2point_calib_statistics(F("Maximum"), maxInputValue, maxY_value);

      const float stepsize = (maxY_value - minY_value) / (MAX_ADC_VALUE + 1);
      addRowLabel(F("Step Size"));
      addHtmlFloat(stepsize, _nrDecimals);
    } else {
      addRowLabel(F("Current"));
      addHtmlFloat(currentValue, _nrDecimals);
    }
  }
# ifndef LIMIT_BUILD_SIZE
  const bool useBinning = P002_OVERSAMPLING == P002_USE_BINNING;
  addFormSubHeader(useBinning ? F("Binning Processing") : F("Multipoint Processing"));
  addFormCheckBox(useBinning ? F("Binning Processing Enabled") : F("Multipoint Processing Enabled"),
                  F("p002_multi_en"),
                  P002_MULTIPOINT_ENABLED);

  if (useBinning) {
    addFormTextBox(F("Binning Formula"), getPluginCustomArgName(P002_LINE_INDEX_FORMULA), _formula, P002_MAX_FORMULA_LENGTH);
  }

  addFormNumericBox(useBinning ? F("Nr of Bins") : F("Nr Multipoint Fields"),
                    F("p002_nr_mp"),
                    P002_NR_MULTIPOINT_ITEMS,
                    0,
                    P002_MAX_NR_MP_ITEMS);

  // Checkbox needed to explicitly allow to split-paste over each field
  addFormCheckBox(useBinning ? F("Split-Paste Binning Fields") : F("Split-Paste Multipoint Fields"), F("splitpaste"), false);
  addFormNote(F("When checked, a set of tab, space or newline separated values can be pasted at once."));

  size_t line_nr = 0;

  for (int varNr = P002_LINE_IDX_FIRST_MP; varNr < P002_Nlines; varNr += P002_STRINGS_PER_MP)
  {
    const String label = String(useBinning ? F("Bin ") : F("Point ")) + String(line_nr + 1);
    addFormTextBox(F("query-input widenumber"),
                   label,
                   getPluginCustomArgName(varNr),
                   _multipoint.size() > line_nr ? doubleToString(static_cast<double>(_multipoint[line_nr]._adc), _nrDecimals,
                                                                 true) : EMPTY_STRING,
                   0);
    html_add_estimate_symbol();
    addTextBox(getPluginCustomArgName(varNr + 1),
               _multipoint.size() > line_nr ?  doubleToString(static_cast<double>(_multipoint[line_nr]._value), _nrDecimals,
                                                              true) : EMPTY_STRING,
               0,
               false,
               false,
               EMPTY_STRING,
               F("query-input widenumber"));

    ++line_nr;
  }
  #  if FEATURE_CHART_JS
  webformLoad_multipointCurve(event);
  #  endif // if FEATURE_CHART_JS
# endif    // ifndef LIMIT_BUILD_SIZE
}

# if FEATURE_PLUGIN_STATS
bool P002_data_struct::webformLoad_show_stats(struct EventStruct *event)
{
  bool somethingAdded = false;

  if (getPluginStats(0) != nullptr) {
    if (getPluginStats(0)->webformLoad_show_avg(event)) { somethingAdded = true; }

    if (getPluginStats(0)->hasPeaks()) {
      formatADC_statistics(F("ADC Peak Low"),  getPluginStats(0)->getPeakLow(),  true);
      formatADC_statistics(F("ADC Peak High"), getPluginStats(0)->getPeakHigh(), true);
      somethingAdded = true;
    }
  }
  return somethingAdded;
}

# endif // if FEATURE_PLUGIN_STATS


# ifdef ESP32
#  if FEATURE_CHART_JS
void P002_data_struct::webformLoad_calibrationCurve(struct EventStruct *event)
{
  if (!hasADC_factory_calibration()) { return; }

  addRowLabel(F("Calibration Curve"));

  const int valueCount = 33;
  int xAxisValues[valueCount];

  getChartRange(event, xAxisValues, valueCount, true);

  String axisOptions;

  {
    const ChartJS_title xAxisTitle(F("ADC Value"));
    const ChartJS_title yAxisTitle(F("Input Voltage (mV)"));
    axisOptions = make_ChartJS_scale_options(xAxisTitle, yAxisTitle);
  }
  add_ChartJS_chart_header(
    F("line"),
    F("fact_cal"),
    F("Factory Calibration per Attenuation"),
    500,
    500,
    axisOptions);

  add_ChartJS_chart_labels(
    valueCount,
    xAxisValues);

  const __FlashStringHelper *colors[] = { F("#A52422"), F("#BEA57D"), F("#EFF2C0"), F("#A4BAB7") };

  size_t current_attenuation = getAttenuation(event);

  if (current_attenuation >= ADC_ATTEN_MAX) { current_attenuation = ADC_ATTEN_DB_11; }

  for (size_t att = 0; att < ADC_ATTEN_MAX; ++att)
  {
    float values[valueCount];

    for (int i = 0; i < valueCount; ++i) {
      values[i] = applyFactoryCalibration(xAxisValues[i], static_cast<adc_atten_t>(att));
    }

    add_ChartJS_dataset(
      AttenuationToString(static_cast<adc_atten_t>(att)),
      colors[att],
      values,
      valueCount,
      att != current_attenuation);
  }
  add_ChartJS_chart_footer();
}

#  endif // if FEATURE_CHART_JS
# endif  // ifdef ESP32

# if FEATURE_CHART_JS
const __FlashStringHelper * P002_data_struct::getChartXaxisLabel(struct EventStruct *event)
{
  #  ifdef ESP32

  if (useFactoryCalibration(event)) {
    // reading in mVolt, not ADC
    return F("Input Voltage (mV)");
  }
  #  endif // ifdef ESP32
  return F("ADC Value");
}

# endif // if FEATURE_CHART_JS

void P002_data_struct::getInputRange(struct EventStruct *event, int& minInputValue, int& maxInputValue, bool ignoreCalibration)
{
  minInputValue = 0;
  maxInputValue = MAX_ADC_VALUE;
  # ifdef ESP32

  if (useFactoryCalibration(event) && !ignoreCalibration) {
    // reading in mVolt, not ADC
    const size_t attenuation = getAttenuation(event);
    minInputValue = esp_adc_cal_raw_to_voltage(0, &adc_chars[attenuation]);
    maxInputValue = esp_adc_cal_raw_to_voltage(MAX_ADC_VALUE, &adc_chars[attenuation]);
  }
  # endif // ifdef ESP32
}

# if FEATURE_CHART_JS

void P002_data_struct::getChartRange(struct EventStruct *event, int values[], int count, bool ignoreCalibration)
{
  int minInputValue, maxInputValue;

  getInputRange(event, minInputValue, maxInputValue, ignoreCalibration);

  const float stepSize = static_cast<float>(maxInputValue + 1 - minInputValue) / (count - 1);

  for (int i = 0; i < count; ++i) {
    values[i] = minInputValue + i * stepSize;
  }
}

void P002_data_struct::webformLoad_2pt_calibrationCurve(struct EventStruct *event)
{
  addRowLabel(F("Two Point Calibration"));

  const int valueCount = 33;
  int xAxisValues[valueCount];

  getChartRange(event, xAxisValues, valueCount);

  String axisOptions;

  {
    const ChartJS_title xAxisTitle(getChartXaxisLabel(event));
    const ChartJS_title yAxisTitle(F("Calibrated Output"));
    axisOptions = make_ChartJS_scale_options(xAxisTitle, yAxisTitle);
  }


  add_ChartJS_chart_header(
    F("line"),
    F("twoPointCurve"),
    F("Two Point Calibration Curve"),
    500,
    500,
    axisOptions);

  add_ChartJS_chart_labels(
    valueCount,
    xAxisValues);

  {
    float values[valueCount];

    for (int i = 0; i < valueCount; ++i) {
      values[i] = P002_data_struct::applyCalibration(event, xAxisValues[i]);
    }

    add_ChartJS_dataset(
      F("2 Point Calibration"),
      F("rgb(255, 99, 132)"),
      values,
      valueCount);
  }
  add_ChartJS_chart_footer();
}

# endif // if FEATURE_CHART_JS

void P002_data_struct::formatADC_statistics(const __FlashStringHelper *label, int raw, bool includeOutputValue) const
{
  addRowLabel(label);
  addHtmlInt(raw);

  float float_value = raw;

# ifdef ESP32

  if (_useFactoryCalibration) {
    float_value = applyFactoryCalibration(raw, _attenuation);

    html_add_estimate_symbol();
    addHtmlFloat(float_value, _nrDecimals);
    addUnit(F("mV"));
  }
# endif // ifdef ESP32

  if (includeOutputValue) {
    addHtml(' ');
    addHtml(F("&rarr; "));
    float_value =  applyCalibration(float_value);

# ifndef LIMIT_BUILD_SIZE

    switch (_sampleMode) {
      case P002_USE_OVERSAMPLING:
        float_value = applyMultiPointInterpolation(float_value);
        break;
      case P002_USE_BINNING:
      {
        const int index = computeADC_to_bin(raw);

        if ((index >= 0) && (static_cast<int>(_binning.size()) > index)) {
          float_value = _multipoint[index]._value;
        }

        break;
      }
    }
# endif // ifndef LIMIT_BUILD_SIZE
    addHtmlFloat(float_value, _nrDecimals);
  }
}

void P002_data_struct::format_2point_calib_statistics(const __FlashStringHelper *label, int raw, float float_value) const
{
  addRowLabel(label);
  addHtmlInt(raw);
  # ifdef ESP32
  addUnit(_useFactoryCalibration ? F("mV") : F("raw"));
  # else // ifdef ESP32
  addUnit(F("raw"));
  # endif // ifdef ESP32
  html_add_estimate_symbol();
  addHtmlFloat(float_value, _nrDecimals);
}

# ifdef ESP32
const __FlashStringHelper * P002_data_struct::AttenuationToString(adc_atten_t attenuation) {
  const __FlashStringHelper *datalabels[] = { F("0 dB"), F("2.5 dB"), F("6 dB"), F("11 dB") };

  if (attenuation < 4) { return datalabels[attenuation]; }
  return F("Unknown");
}

adc_atten_t P002_data_struct::getAttenuation(struct EventStruct *event) {
  if ((P002_ATTENUATION >= P002_ADC_0db) && (P002_ATTENUATION <= P002_ADC_11db)) {
    // Make sure the attenuation is only set to correct values or else it may damage the board
    return static_cast<adc_atten_t>(P002_ATTENUATION - 10);
  }
  P002_ATTENUATION = P002_ADC_11db;
  return ADC_ATTEN_DB_11;
}

# endif // ifdef ESP32

# if FEATURE_CHART_JS
void P002_data_struct::webformLoad_multipointCurve(struct EventStruct *event) const
{
  if (P002_MULTIPOINT_ENABLED)
  {
    const bool useBinning = P002_OVERSAMPLING == P002_USE_BINNING;
    addRowLabel(useBinning ? F("Binning Curve") : F("Multipoint Curve"));

    String axisOptions;

    {
      const ChartJS_title xAxisTitle(useBinning ? F("Bin Center Value") : F("Input"));
      const ChartJS_title yAxisTitle(useBinning ? F("Bin Output Value") : F("Output"));
      axisOptions = make_ChartJS_scale_options(xAxisTitle, yAxisTitle);
    }

    add_ChartJS_chart_header(
      useBinning ? F("bar") : F("line"),
      F("mpcurve"),
      useBinning ? F("Bin Values") : F("Multipoint Curve"),
      500,
      500,
      axisOptions);

    // Add labels
    for (size_t i = 0; i < _multipoint.size(); ++i) {
      if (i != 0) {
        addHtml(',');
      }
      addHtmlFloat(_multipoint[i]._adc, _nrDecimals);
    }
    addHtml(F("],datasets: ["));

    add_ChartJS_dataset_header(
      useBinning ? F("Bins") : F("Multipoint Values"),
      F("rgb(255, 99, 132)"));

    for (size_t i = 0; i < _multipoint.size(); ++i) {
      if (i != 0) {
        addHtml(',');
      }
      addHtmlFloat(_multipoint[i]._value, _nrDecimals);
    }
    add_ChartJS_dataset_footer();
    add_ChartJS_chart_footer();

    if (!useBinning) {
      // Try to compute the expected mapping from ADC to multipoint values
      addRowLabel(F("Input to Output Curve"));
      const int valueCount = 33;
      int xAxisValues[valueCount];
      getChartRange(event, xAxisValues, valueCount);

      String axisOptions;

      {
        const ChartJS_title xAxisTitle(getChartXaxisLabel(event));
        const ChartJS_title yAxisTitle(F("Output"));
        axisOptions = make_ChartJS_scale_options(xAxisTitle, yAxisTitle);
      }
      add_ChartJS_chart_header(
        F("line"),
        F("mpCurveSimulated"),
        F("Simulated Input to Output Curve"),
        500,
        500,
        axisOptions);

      add_ChartJS_chart_labels(
        valueCount,
        xAxisValues);

      const __FlashStringHelper *label = F("Multipoint");
      const __FlashStringHelper *color = F("rgb(255, 99, 132)");

      for (int step = 0; step < 3; ++step)
      {
        float values[valueCount];
        bool  use2PointCalib = false;
        bool  useMultiPoint  = false;

        switch (step) {
          case 0:
            useMultiPoint = true;
            break;
          case 1:
            label          = F("2 Point Calibration & Multipoint");
            color          = F("rgb(54, 162, 235)");
            use2PointCalib = true;
            useMultiPoint  = true;
            break;
          case 2:
            label          = F("2 Point Calibration");
            color          = F("rgb(153, 102, 255)");
            use2PointCalib = true;
            break;
        }

        bool hidden = !((use2PointCalib == _use2pointCalibration) &&
                        useMultiPoint);

        for (int i = 0; i < valueCount; ++i) {
          values[i] = xAxisValues[i];

          if (use2PointCalib) {
            values[i] = P002_data_struct::applyCalibration(event, values[i], true);
          }

          if (useMultiPoint) {
            values[i] = applyMultiPointInterpolation(values[i], true);
          }
        }

        add_ChartJS_dataset(
          label,
          color,
          values,
          valueCount,
          hidden);
      }
      add_ChartJS_chart_footer();
    }
  }
}

# endif // if FEATURE_CHART_JS

String P002_data_struct::webformSave(struct EventStruct *event)
{
  P002_OVERSAMPLING = getFormItemInt(F("p002_oversampling"), 0); // Set a default for LIMIT_BUILD_SIZE

  P002_CALIBRATION_ENABLED = isFormItemChecked(F("p002_cal"));
  # ifdef ESP32
  P002_APPLY_FACTORY_CALIB = isFormItemChecked(F("p002_fac_cal"));
  P002_ATTENUATION         = getFormItemInt(F("p002_attn"));
  # endif // ifdef ESP32

  {
    // Map the input "point" values to the nearest int.
    const float adc1 = getFormItemFloat(F("p002_adc1"));
    const float adc2 = getFormItemFloat(F("p002_adc2"));

    const float out1 = getFormItemFloat(F("p002_out1"));
    const float out2 = getFormItemFloat(F("p002_out2"));


    P002_CALIBRATION_POINT1 = roundf(adc1);
    P002_CALIBRATION_POINT2 = roundf(adc2);
    P002_CALIBRATION_VALUE1 = mapADCtoFloat(
      P002_CALIBRATION_POINT1,
      adc1, adc2,
      out1, out2);
    P002_CALIBRATION_VALUE2 = mapADCtoFloat(
      P002_CALIBRATION_POINT2,
      adc1, adc2,
      out1, out2);
  }

# ifndef LIMIT_BUILD_SIZE
  P002_MULTIPOINT_ENABLED = isFormItemChecked(F("p002_multi_en"));

  P002_NR_MULTIPOINT_ITEMS = getFormItemInt(F("p002_nr_mp"));

  const size_t nr_lines = P002_Nlines;
  String lines[nr_lines];

  // Store nr of lines that were saved, so no 'old' data will be read when nr of multi-point items has changed.
  lines[P002_SAVED_NR_LINES] = String(nr_lines);

  if (web_server.hasArg(getPluginCustomArgName(P002_LINE_INDEX_FORMULA))) {
    lines[P002_LINE_INDEX_FORMULA] = webArg(getPluginCustomArgName(P002_LINE_INDEX_FORMULA));
  }

  // const int nrDecimals = webArg(F("TDVD1")).toInt();

  for (size_t varNr = P002_LINE_IDX_FIRST_MP; varNr < nr_lines; varNr += P002_STRINGS_PER_MP)
  {
    float adc, value = 0.0f;
    const String adc_str = webArg(getPluginCustomArgName(varNr));
    const String val_str = webArg(getPluginCustomArgName(varNr + 1));

    if (validFloatFromString(adc_str, adc) && validFloatFromString(val_str, value)) {
      // Only store valid floats
      lines[varNr]     = adc_str;
      lines[varNr + 1] = val_str;
    }
  }

  return SaveCustomTaskSettings(event->TaskIndex, lines, nr_lines, 0);
# else // ifndef LIMIT_BUILD_SIZE
  return EMPTY_STRING;
# endif // ifndef LIMIT_BUILD_SIZE
}

void P002_data_struct::takeSample()
{
  if (_sampleMode == P002_USE_CURENT_SAMPLE) { return; }
  int raw = espeasy_analogRead(_pin_analogRead);

# if FEATURE_PLUGIN_STATS

  if (getPluginStats(0) != nullptr) {
    getPluginStats(0)->trackPeak(raw);
  }
# endif // if FEATURE_PLUGIN_STATS

  switch (_sampleMode) {
    case P002_USE_OVERSAMPLING:
      addOversamplingValue(raw);
      break;
# ifndef LIMIT_BUILD_SIZE
    case P002_USE_BINNING:
      addBinningValue(raw);
      break;
# endif // ifndef LIMIT_BUILD_SIZE
  }
}

bool P002_data_struct::getValue(float& float_value,
                                int  & raw_value) const
{
  bool mustTakeSample = false;

  switch (_sampleMode) {
    case P002_USE_OVERSAMPLING:

      if (getOversamplingValue(float_value, raw_value)) {
        return true;
      }
      mustTakeSample = true;
      break;
# ifndef LIMIT_BUILD_SIZE
    case P002_USE_BINNING:

      if (getBinnedValue(float_value, raw_value)) {
        return true;
      }
      mustTakeSample = true;
      break;
# endif // ifndef LIMIT_BUILD_SIZE
    case P002_USE_CURENT_SAMPLE:
      mustTakeSample = true;
      break;
  }

  if (!mustTakeSample) {
    return false;
  }

  raw_value = espeasy_analogRead(_pin_analogRead);
# if FEATURE_PLUGIN_STATS

  if (getPluginStats(0) != nullptr) {
    getPluginStats(0)->trackPeak(raw_value);
  }
# endif // if FEATURE_PLUGIN_STATS
  float_value = raw_value;
  # ifdef ESP32

  if (_useFactoryCalibration) {
    float_value = applyFactoryCalibration(raw_value, _attenuation);
  }
  # endif // ifdef ESP32

  float_value = applyCalibration(float_value);

# ifndef LIMIT_BUILD_SIZE

  switch (_sampleMode) {
    case P002_USE_OVERSAMPLING:
      float_value = applyMultiPointInterpolation(float_value);
      break;
    case P002_USE_BINNING:
    {
      const int index = computeADC_to_bin(raw_value);

      if ((index >= 0) && (static_cast<int>(_binning.size()) > index)) {
        float_value = _multipoint[index]._value;
      }

      break;
    }
  }
# endif // ifndef LIMIT_BUILD_SIZE

  return true;
}

void P002_data_struct::reset()
{
# ifndef LIMIT_BUILD_SIZE

  switch (_sampleMode) {
    case P002_USE_OVERSAMPLING:
      resetOversampling();
      break;
    case P002_USE_BINNING:
    {
      for (auto it = _binning.begin(); it != _binning.end(); ++it) {
        *it = 0;
      }

      break;
    }
  }
# else // ifndef LIMIT_BUILD_SIZE
  resetOversampling();
# endif // ifndef LIMIT_BUILD_SIZE
}

void P002_data_struct::resetOversampling() {
  OversamplingValue  = 0;
  OversamplingCount  = 0;
  OversamplingMinVal = MAX_ADC_VALUE;
  OversamplingMaxVal = -MAX_ADC_VALUE;
}

void P002_data_struct::addOversamplingValue(int currentValue) {
  // Extra check to only add min or max readings once.
  // They will be taken out of the averaging only one time.
  if ((currentValue == 0) && (currentValue == OversamplingMinVal)) {
    return;
  }

  if ((currentValue == MAX_ADC_VALUE) && (currentValue == OversamplingMaxVal)) {
    return;
  }

  OversamplingValue += currentValue;
  ++OversamplingCount;

  if (currentValue > OversamplingMaxVal) {
    OversamplingMaxVal = currentValue;
  }

  if (currentValue < OversamplingMinVal) {
    OversamplingMinVal = currentValue;
  }
}

bool P002_data_struct::getOversamplingValue(float& float_value, int& raw_value) const {
  if (OversamplingCount > 0) {
    float sum   = static_cast<float>(OversamplingValue);
    float count = static_cast<float>(OversamplingCount);

    if (OversamplingCount >= 3) {
      sum   -= OversamplingMaxVal;
      sum   -= OversamplingMinVal;
      count -= 2;
    }
    float_value = sum / count;
    raw_value   = static_cast<int>(float_value);

# ifdef ESP32

    if (_useFactoryCalibration) {
      float_value = applyFactoryCalibration(float_value, _attenuation);
    }
# endif // ifdef ESP32

    // We counted the raw oversampling values, so now we need to apply the calibration and multi-point processing
    float_value = applyCalibration(float_value);
# ifndef LIMIT_BUILD_SIZE
    float_value = applyMultiPointInterpolation(float_value);
# endif // ifndef LIMIT_BUILD_SIZE

    return true;
  }
  return false;
}

# ifndef LIMIT_BUILD_SIZE
int P002_data_struct::getBinIndex(float currentValue) const
{
  const size_t mp_size = _multipoint.size();

  if (mp_size == 0) { return -1; }

  if (mp_size == 1) { return 0; }

  if (currentValue <= _multipoint[0]._adc) { return 0; }

  const size_t last_mp_index = mp_size - 1;

  if (currentValue >= _multipoint[last_mp_index]._adc) { return last_mp_index; }

  for (unsigned int i = 0; i < last_mp_index; ++i) {
    const float dist_left  = currentValue - _multipoint[i]._adc;
    const float dist_right = _multipoint[i + 1]._adc - currentValue;

    if ((dist_left >= 0) && (dist_right >= 0)) {
      // Inbetween 2 points of the multipoint array
      return (dist_left < dist_right) ? i : i + 1;
    }
  }

  return -1;
}

int P002_data_struct::computeADC_to_bin(const int& currentValue) const
{
  // First apply calibration, then find the bin index
  float calibrated_value = static_cast<float>(currentValue);

#  ifdef ESP32

  if (_useFactoryCalibration) {
    calibrated_value = applyFactoryCalibration(calibrated_value, _attenuation);
  }
#  endif // ifdef ESP32


  calibrated_value = applyCalibration(calibrated_value);

  if (!_formula_preprocessed.isEmpty()) {
    // Formula, must be applied before binning
    String formula = _formula_preprocessed;

    formula.replace(F("%value%"), toString(calibrated_value, _nrDecimals));

    double result = 0;

    if (!isError(RulesCalculate.doCalculate(parseTemplate(formula).c_str(), &result))) {
      calibrated_value = result;
    }
  }

  return getBinIndex(calibrated_value);
}

void P002_data_struct::addBinningValue(int currentValue)
{
  for (size_t index = 0; index < _binningRange.size(); ++index) {
    if (_binningRange[index].inRange(currentValue)) {
      ++_binning[index];
      return;
    }
  }

  const int index = computeADC_to_bin(currentValue);

  if ((index >= 0) && (static_cast<int>(_binning.size()) > index)) {
    _binningRange[index].set(currentValue);
    ++_binning[index];
  }
}

bool P002_data_struct::getBinnedValue(float& float_value, int& raw_value) const
{
  unsigned int highest_bin_count = 0;

  const size_t nr_bin_elements = std::min(_binning.size(), _multipoint.size());

  for (size_t i = 0; i < nr_bin_elements; ++i) {
    if (_binning[i] > highest_bin_count) {
      highest_bin_count = _binning[i];
      float_value       = _multipoint[i]._value;
      raw_value         = _multipoint[i]._adc;
    }
  }
  #  ifndef BUILD_NO_DEBUG

  if (loglevelActiveFor(LOG_LEVEL_DEBUG)) {
    String log = F("ADC getBinnedValue: bin cnt: ");

    log += highest_bin_count;
    log += F(" Value: ");
    log += float_value;
    log += F(" RAW: ");
    log += raw_value;
    addLog(LOG_LEVEL_DEBUG, log);
  }
  #  endif // ifndef BUILD_NO_DEBUG

  return highest_bin_count != 0;
}

# endif // ifndef LIMIT_BUILD_SIZE

float P002_data_struct::applyCalibration(struct EventStruct *event, float float_value, bool force) {
  if (force || P002_CALIBRATION_ENABLED)
  {
    float_value = mapADCtoFloat(float_value,
                                P002_CALIBRATION_POINT1,
                                P002_CALIBRATION_POINT2,
                                P002_CALIBRATION_VALUE1,
                                P002_CALIBRATION_VALUE2);
  }
  return float_value;
}

float P002_data_struct::getCurrentValue(struct EventStruct *event, int& raw_value)
{
  # ifdef ESP8266
  const int pin = A0;
  # endif // ifdef ESP8266
  # ifdef ESP32
  const int pin = CONFIG_PIN1;
  # endif // ifdef ESP32

  raw_value = espeasy_analogRead(pin);

  # ifdef ESP32

  if (useFactoryCalibration(event)) {
    return applyFactoryCalibration(raw_value, getAttenuation(event));
  }
  # endif // ifdef ESP32

  return raw_value;
}

float P002_data_struct::applyCalibration(float float_value) const
{
  if (!_use2pointCalibration) { return float_value; }
  return mapADCtoFloat(
    float_value,
    _calib_adc1,
    _calib_adc2,
    _calib_out1,
    _calib_out2);
}

# ifdef ESP32
bool P002_data_struct::useFactoryCalibration(struct EventStruct *event) {
  if (P002_APPLY_FACTORY_CALIB) {
    const int adc_num = getADC_num_for_gpio(CONFIG_PIN1);

    if ((adc_num == 1) || (adc_num == 2)) {
      return true;
    }
  }
  return false;
}

float P002_data_struct::applyFactoryCalibration(float raw_value, adc_atten_t attenuation)
{
  if (attenuation == adc_atten_t::ADC_ATTEN_DB_11) {
    return esp_adc_cal_raw_to_voltage(raw_value, &adc_chars[attenuation]);
  }

  // All other attenuations do appear to have a straight calibration curve.
  // But applying the factory calibration then reduces resolution.
  // So we interpolate using the calibrated extremes

  // Cache the computing of the values.
  static adc_atten_t last_Attn = ADC_ATTEN_MAX;
  static float last_out1       = 0.0;
  static float last_out2       = MAX_ADC_VALUE;

  if (last_Attn != attenuation) {
    last_Attn = attenuation;
    last_out1 = esp_adc_cal_raw_to_voltage(0, &adc_chars[attenuation]);
    last_out2 = esp_adc_cal_raw_to_voltage(MAX_ADC_VALUE, &adc_chars[attenuation]);
  }

  return mapADCtoFloat(
    raw_value,
    0,
    MAX_ADC_VALUE,
    last_out1,
    last_out2);
}

# endif // ifdef ESP32

# ifndef LIMIT_BUILD_SIZE
float P002_data_struct::applyMultiPointInterpolation(float float_value, bool force) const
{
  if (!_useMultipoint && !force) { return float_value; }

  // First find the surrounding bins
  const size_t mp_size = _multipoint.size();

  if (mp_size == 0) { return float_value; }

  if (float_value <= _multipoint[0]._adc) {
    if (mp_size > 1) {
      // Just extrapolate the first multipoint line segment.
      return mapADCtoFloat(
        float_value,
        _multipoint[0]._adc,
        _multipoint[1]._adc,
        _multipoint[0]._value,
        _multipoint[1]._value);
    }

    // just one point, so all we can do is consider it to be a slight deviation of the calibration.
    return mapADCtoFloat(
      float_value,
      0,
      _multipoint[0]._adc,
      applyCalibration(0),
      _multipoint[0]._value);
  }

  const size_t last_mp_index = mp_size - 1;

  if (float_value >= _multipoint[last_mp_index]._adc)
  {
    if (mp_size > 1) {
      // Just extrapolate the last multipoint line segment.
      return mapADCtoFloat(
        float_value,
        _multipoint[last_mp_index - 1]._adc,
        _multipoint[last_mp_index]._adc,
        _multipoint[last_mp_index - 1]._value,
        _multipoint[last_mp_index]._value);
    }

    // just one point, so all we can do is consider it to be a slight deviation of the calibration.
    return mapADCtoFloat(
      float_value,
      _multipoint[last_mp_index]._adc,
      MAX_ADC_VALUE,
      _multipoint[last_mp_index]._value,
      applyCalibration(MAX_ADC_VALUE));
  }

  for (unsigned int i = 0; i < last_mp_index; ++i) {
    const float dist_left  = float_value - _multipoint[i]._adc;
    const float dist_right = _multipoint[i + 1]._adc - float_value;

    if ((dist_left >= 0) && (dist_right >= 0) &&
        (_multipoint[i]._adc != _multipoint[i + 1]._adc)) {
      // Inbetween 2 points of the multipoint array
      return mapADCtoFloat(
        float_value,
        _multipoint[i]._adc,
        _multipoint[i + 1]._adc,
        _multipoint[i]._value,
        _multipoint[i + 1]._value);
    }
  }

  return float_value;
}

# endif // ifndef LIMIT_BUILD_SIZE

float P002_data_struct::mapADCtoFloat(float float_value,
                                      float adc1,
                                      float adc2,
                                      float out1,
                                      float out2)
{
  if (!approximatelyEqual(adc1, adc2))
  {
    const float normalized = static_cast<float>(float_value - adc1) / static_cast<float>(adc2 - adc1);
    float_value = normalized * (out2 - out1) + out1;
  }
  return float_value;
}


#endif // ifdef USES_P002

