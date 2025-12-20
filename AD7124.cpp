#include "AD7124.h"

// Out-of-class defs for static constexpr class-type members (needed pre-C++17)
constexpr AD7124::Register AD7124::Status;
constexpr AD7124::Register AD7124::Control;
constexpr AD7124::Register AD7124::Data;
constexpr AD7124::Register AD7124::DataPlusStatus;
constexpr AD7124::Register AD7124::ID;
constexpr AD7124::Register AD7124::Error;
constexpr AD7124::Register AD7124::ErrorEn;
constexpr AD7124::Register AD7124::Channel0;
constexpr AD7124::Register AD7124::Config0;
constexpr AD7124::Register AD7124::Filter0;
constexpr AD7124::Register AD7124::Offset0;
constexpr AD7124::Register AD7124::Gain0;

// ───────────────────────────────────────────────────────────────
// Constructor
// ───────────────────────────────────────────────────────────────
AD7124::AD7124(uint8_t csPin, uint8_t rdyPin)
    : _cs(csPin), _rdy(rdyPin), _spiSettings(1000000, MSBFIRST, SPI_MODE3) {}

// ───────────────────────────────────────────────────────────────
// Low‑level helpers  (CS framing inside each call)
// ───────────────────────────────────────────────────────────────
AD7124::ErrorCode AD7124::write(Register reg, uint32_t value, bool verification, uint32_t pause)
{
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);
    delayMicroseconds(1);

    SPI.transfer(0x00 | (reg.address & 0x3F));      // comm‑byte (write)
    for (uint8_t i = 0; i < reg.size; ++i)           // MSB first
        SPI.transfer((value >> ((reg.size - 1 - i) * 8)) & 0xFF);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();

    delayMicroseconds(pause);
    if (verification) {
        uint32_t readback = read(reg);
        if (readback != value) {
            Serial.print(F("AD7124 write failed for register 0x"));
            Serial.print(reg.address, HEX);
            Serial.print(F(", expected 0x"));
            Serial.print(value, HEX);
            Serial.print(F(", got 0x"));
            Serial.println(readback, HEX);
            AD7124::ErrorCode err = errorCheck();
            if (err == OK) {
                return Write_Failure;
            }
            else {
                return err; // return error from errorCheck
            }
        }
    }
    return OK;
}

uint32_t AD7124::read(Register reg)
{
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);
    delayMicroseconds(1);

    SPI.transfer(0x40 | (reg.address & 0x3F));      // comm‑byte (read)
    uint32_t val = 0;
    for (uint8_t i = 0; i < reg.size; ++i)
        val = (val << 8) | SPI.transfer(0x00);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    return val;
}


AD7124::ErrorCode AD7124::reset()
{
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);
    delayMicroseconds(1);

    for (int i = 0; i < 8; ++i) SPI.transfer(0xFF); // 64 clocks
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();

    delayMicroseconds(RESET_TIME); // wait for reset to complete
    return errorCheck(); // check for errors after reset
}

AD7124::ErrorCode AD7124::declareSetup()
{
    ErrorCode e;
    e = write(Control, Normal_Operation, true, SETTLING_TIME);
    if (e != OK) return e;
    Serial.println(F("AD7124 Control setup complete."));
    e = write(Channel0, Channel_0_Setup, true, SETTLING_TIME);
    if (e != OK) return e;
    Serial.println(F("AD7124 Channel_0 setup complete."));
    e = write(Config0,  Config_0_Setup, true, SETTLING_TIME);
    if (e != OK) return e;
    Serial.println(F("AD7124 Config_0 setup complete."));
    e = write(Filter0,  Filter_0_Setup, true, SETTLING_TIME);
    if (e != OK) return e;
    Serial.println(F("AD7124 Filter_0 setup complete."));
    e = write(ErrorEn,  Error_En_Setup, true, SETTLING_TIME);
    if (e != OK) return e;
    Serial.println(F("AD7124 Error_EN setup complete."));

    return OK;
}

AD7124::ErrorCode AD7124::calibrate() 
{
    ErrorCode e;
    e = write(Offset0,  Default_Offset, true, SETTLING_TIME);
    if (e != OK) return e;
    Serial.println(F("AD7124 Offset_0 setup complete."));
    e = write(Control, Full_Scale_Cal, false, CALIBRATION_TIME);
    if (e != OK) return e;
    Serial.println(F("AD7124 Full Scale Calibration complete."));
    e = write(Control, Zero_Scale_Cal, false, CALIBRATION_TIME);
    if (e != OK) return e;
    Serial.println(F("AD7124 Zero Scale Calibration complete."));
    e = write(Control, Normal_Operation, true, SETTLING_TIME);
    if (e != OK) return e;
    Serial.println(F("AD7124 Control reset to Normal Operation."));

    return OK;
}


// ───────────────────────────────────────────────────────────────
// Public API
// ───────────────────────────────────────────────────────────────
AD7124::ErrorCode AD7124::begin()
{
    pinMode(_cs, OUTPUT);
    pinMode(_rdy, INPUT);
    digitalWrite(_cs, HIGH);
    SPI.begin();
    delay(100);

    ErrorCode e;

    e = reset();
    delay(1000);             // extra delay for good measure
    if (e != OK) {
        e = reset();
        if (e != OK) return e; // reset failed twice
    }
    Serial.println(F("AD7124 reset complete."));

    // -------- verify ID ----------
    uint32_t id = read(ID);
    if (id != Healthy_ID) {
        delay(1000);
        id = read(ID); // try reading ID again
        if (id != Healthy_ID) {
            return ID_Mismatch;
        }
    }
    Serial.print(F("AD7124 ID: 0x"));
    Serial.println(id, HEX);

    // -------- configure chip -----
    e = declareSetup();
    if (e != OK) return e;
    Serial.println(F("AD7124 setup declared."));

    e = calibrate();
    if (e != OK) return e;
    Serial.println(F("AD7124 calibration complete."));

    return errorCheck(); // final check for errors
}

AD7124::Result AD7124::readData(uint32_t timeoutMicros)
{
    unsigned long start = micros();
    bool second_try = false;

    while (micros() - start < timeoutMicros) {
        uint8_t status = read(Status) & 0xFF;
        if ((status & STATUS_RDY) == 0) {
            uint32_t bytes = read(DataPlusStatus);
            uint32_t data = (bytes >> 8) & 0xFFFFFF;
            AD7124::ErrorCode err = errorCheck(status);
            
            if (err != OK) {
                if (second_try) return {0, err};               // return error after retry
                second_try = true;                             // try once more
                continue;
            } else {
                return {data, OK};                        // return data if no error
            }
        }
    }

    return {0, Timeout};
}

AD7124::WeightResult AD7124::getWeight(uint32_t timeoutMicros)
{
    AD7124::Result result = readData(timeoutMicros);
    if (result.status_code == AD7124::OK) {
        // Proper 24-bit sign extension
        int32_t signed24 = (int32_t)(result.data & 0x00FFFFFF) - 0x800000;

        float lbs = ( signed24 - MEASURED_OFFSET ) / MEASURED_SCALE_FACTOR_LBS;
        lbs -= TARE_OFFSET;  // see next tweak
        return {lbs, AD7124::OK};
    } else {
        return {0, result.status_code};
    }
}


void AD7124::tare()
{
    AD7124::WeightResult wR = getWeight();
    TARE_OFFSET = TARE_OFFSET + wR.weight;
}

// ───────────────────────────────────────────────────────────────
// Error parser (unchanged logic)
// ───────────────────────────────────────────────────────────────
AD7124::ErrorCode AD7124::parseError(uint32_t e)
{
    if (e == 0)                         return OK;
    if (e & SPI_ERRORS)                 return SPI_Error;
    if (e & CHIP_FAULTS)                return Chip_Fault;
    if (e & CALIBRATION_ERRORS)         return Calibration_Error;
    if (e & CONVERSION_ERRORS)          return Conversion_Error;
    if (e & HARDWARE_FAULTS)            return Hardware_Fault;
    return Unknown;
}

AD7124::ErrorCode AD7124::errorCheck(uint32_t status_byte) {
    if (status_byte == 0) {
        status_byte = read(Status); // Read current status if not provided
    }
    
    if (!(status_byte & STATUS_ERR)) return OK; // No error
    
    uint32_t errorReg = read(Error);
    return parseError(errorReg);
}

uint32_t AD7124::readErrorRegister() {
    return read(Error); 
}

void AD7124::dumpAllRegisters(Stream &out) const
{
    struct RegInfo { Register reg; const char *name; };
    constexpr RegInfo table[] = {
        { Status,           "Status       " },
        { Control,          "Control      " },
        { Data,             "Data         " },      // will show last conversion
        { ID,               "ID           " },
        { Error,            "Error        " },
        { ErrorEn,          "Error_En     " },
        { Channel0,         "Channel_0    " },
        { Config0,          "Config_0     " },
        { Filter0,          "Filter_0     " },
        { Offset0,          "Offset_0     " },
        { Gain0,            "Gain_0       " },
    };

    out.println(F("\n---- AD7124 register dump ----"));
    for (auto &r : table)
    {
        uint32_t val = const_cast<AD7124*>(this)->read(r.reg);   // reuse existing read()
        out.print("0x");                 // address
        out.print(r.reg.address, HEX);
        out.print(" (");
        out.print(r.name);
        out.print(") = 0x");

        uint8_t n = r.reg.size;
        for (int8_t i = (n - 1) * 8; i >= 0; i -= 8) {           // MSB first
            uint8_t b = (val >> i) & 0xFF;
            if (b < 0x10) out.print('0');
            out.print(b, HEX);
        }
        out.println();
    }
    out.println(F("------------------------------"));
}
