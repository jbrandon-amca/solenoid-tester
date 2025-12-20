#ifndef AD7124_H
#define AD7124_H

#include <Arduino.h>
#include <SPI.h>

class AD7124 {
public:
    enum ErrorCode : uint8_t {
        OK = 0,
        ID_Mismatch = 1,
        SPI_Error = 2,
        Chip_Fault = 3,
        Calibration_Error = 4,
        Conversion_Error = 5,
        Hardware_Fault = 6,
        Timeout = 7,
        Write_Failure = 8,
        Unknown= 255,
    };

    struct Result {
        uint32_t  data{0};
        ErrorCode status_code{OK};

        constexpr Result() = default;
        constexpr Result(uint32_t d, ErrorCode s) : data(d), status_code(s) {}
    };

    struct WeightResult {
        float     weight{0.0f};
        ErrorCode status_code{OK};

        constexpr WeightResult() = default;
        constexpr WeightResult(float w, ErrorCode s) : weight(w), status_code(s) {}
    };


    AD7124(uint8_t csPin, uint8_t rdyPin);
    ErrorCode begin();                         
    Result readData(uint32_t timeoutMicros = 2000);
    uint32_t readErrorRegister();  
    void dumpAllRegisters(Stream &out = Serial) const;
    WeightResult getWeight(uint32_t timeoutMicros = 2000);
    void tare();

    static constexpr int32_t MEASURED_OFFSET = (int32_t)8344236 - 0x800000;;   // Calculated via calibration
    static constexpr float MEASURED_SCALE_FACTOR_LBS = 25199.27241;  // Calculated via calibration
    float TARE_OFFSET = 0.0f;

private:
    static constexpr uint32_t CLOCK = 614400;
    static constexpr uint32_t SPS = 1000;
    static constexpr uint32_t FS = CLOCK / (32 * SPS);
    static constexpr uint32_t SAFETY_FACTOR = 10;   // added margin for safety
    static constexpr uint32_t SETTLING_TIME = SAFETY_FACTOR * 1e6 * (4 * 32 * FS + 95) / CLOCK;   // in microseconds
    static constexpr uint32_t RESET_TIME = SAFETY_FACTOR * 1e6 * 90 / CLOCK;  // in microseconds, based on datasheet
    static constexpr uint32_t CALIBRATION_TIME = 4 * SETTLING_TIME;
    static constexpr uint32_t TIMEOUT = 1e6 * 3 / SPS;  // Exit after trying for 3 

    struct Register {
        uint8_t address;
        uint8_t size;
    };

    uint8_t _cs, _rdy;
    SPISettings _spiSettings;

    ErrorCode reset();
    ErrorCode calibrate();
    ErrorCode declareSetup();
    ErrorCode write(Register reg, uint32_t value, bool verification = true, uint32_t pause = 10);
    uint32_t read(Register reg);
    ErrorCode errorCheck(uint32_t status_byte = 0);
    ErrorCode parseError(uint32_t error);

    static constexpr Register Status           = { 0x00, 1 };
    static constexpr Register Control          = { 0x01, 2 };
    static constexpr Register Data             = { 0x02, 3 };
    static constexpr Register DataPlusStatus   = { 0x02, 4 };
    static constexpr Register ID               = { 0x05, 1 };
    static constexpr Register Error            = { 0x06, 3 };
    static constexpr Register ErrorEn          = { 0x07, 3 };
    static constexpr Register Channel0         = { 0x09, 2 };
    static constexpr Register Config0          = { 0x19, 2 };
    static constexpr Register Filter0          = { 0x21, 3 };
    static constexpr Register Offset0          = { 0x29, 3 };
    static constexpr Register Gain0            = { 0x31, 3 };

// ───────────────────────────────────────────────────────────────
// Bit masks
// ───────────────────────────────────────────────────────────────
   
    // Status Register

    static constexpr uint32_t STATUS_RDY = (uint32_t)1 << 7; 
    static constexpr uint32_t STATUS_ERR = (uint32_t)1 << 6;
    static constexpr uint32_t STATUS_POR = (uint32_t)1 << 4;  // Power on or reset

    // ADC_Control Register

    static constexpr uint32_t DOUT_RDY_DELAY = (uint32_t)1 << 12;
    static constexpr uint32_t CONTINUOUS_READ = (uint32_t)1 << 11;
    static constexpr uint32_t TRANSMIT_STATUS_WITH_DATA = (uint32_t)1 << 10;
    static constexpr uint32_t CS_EN = (uint32_t)1 << 9;
    static constexpr uint32_t INTERNAL_REFERENCE = (uint32_t)1 << 8;
    static constexpr uint32_t FULL_POWER = (uint32_t)0b11 << 6;
    static constexpr uint32_t MID_POWER = (uint32_t)0b01 << 6;
    static constexpr uint32_t LOW_POWER = (uint32_t)0b00 << 6;
    static constexpr uint32_t CONTINUOUS_CONVERSION = (uint32_t)0b0000 << 2;
    static constexpr uint32_t FULL_SCALE_CALIBRATION = (uint32_t)0b0110 << 2;
    static constexpr uint32_t ZERO_SCALE_CALIBRATION = (uint32_t)0b0101 << 2;
    static constexpr uint32_t INTERNAL_CLOCK = (uint32_t)0b00;

    static constexpr uint32_t Normal_Operation = 
        TRANSMIT_STATUS_WITH_DATA | 
        CS_EN | 
        FULL_POWER | 
        CONTINUOUS_CONVERSION | 
        INTERNAL_CLOCK;

    static constexpr uint32_t Full_Scale_Cal = 
        TRANSMIT_STATUS_WITH_DATA |
        CS_EN |
        MID_POWER |
        FULL_SCALE_CALIBRATION |
        INTERNAL_CLOCK;

    static constexpr uint32_t Zero_Scale_Cal = 
        TRANSMIT_STATUS_WITH_DATA |
        CS_EN |
        MID_POWER |
        ZERO_SCALE_CALIBRATION |
        INTERNAL_CLOCK;

    // Offset Register

    static constexpr uint32_t Default_Offset = 0x800000; // Default offset value
    
    // ID Register

    static constexpr uint32_t Healthy_ID = 0x07; // Expected ID for AD7124

    // Error Register

    static constexpr uint32_t CALIBRATION_ERRORS = (uint32_t)1 << 18;
    static constexpr uint32_t CONVERSION_ERRORS = (uint32_t)0b11 << 16;
    static constexpr uint32_t HARDWARE_FAULTS = (uint32_t)0b11111 << 11;
    static constexpr uint32_t SPI_ERRORS = (uint32_t)0b11111 << 2;
    static constexpr uint32_t CHIP_FAULTS = (uint32_t)0b11;

    // Error_EN Register

    static constexpr uint32_t Error_En_Setup = 
        CALIBRATION_ERRORS | 
        CONVERSION_ERRORS | 
        HARDWARE_FAULTS | 
        SPI_ERRORS | 
        CHIP_FAULTS;

    // Channel 0 Register

    static constexpr uint32_t CHANNEL_ENABLE = (uint32_t)1 << 15;
    static constexpr uint32_t SETUP_0 = (uint32_t)0b000 << 12;
    static constexpr uint32_t AINP_AIN7 = (uint32_t)0b00111 << 5;
    static constexpr uint32_t AINM_AIN6 = (uint32_t)0b00110;

    static constexpr uint32_t Channel_0_Setup = 
        CHANNEL_ENABLE | 
        SETUP_0 | 
        AINP_AIN7 | 
        AINM_AIN6;

    // Config 0 Register

    static constexpr uint32_t BIPOLAR = (uint32_t)1 << 11;
    static constexpr uint32_t REF_BUFFER = (uint32_t)0b11 << 7; // Reference buffer enable
    static constexpr uint32_t AIN_BUFFER = (uint32_t)0b11 << 5; // Reference voltage selection
    static constexpr uint32_t REF_SEL = (uint32_t)0b00 << 3; // Internal reference
    static constexpr uint32_t PGA_64  = 0b110;
    static constexpr uint32_t PGA_128 = 0b111;

    static constexpr uint32_t Config_0_Setup = 
        BIPOLAR | 
        REF_BUFFER | 
        AIN_BUFFER | 
        REF_SEL | 
        // PGA_64;
        PGA_128;

    // Filter 0 Register

    static constexpr uint32_t SINC_4 = (uint32_t)0b000 << 21;
    static constexpr uint32_t SINC_3 = (uint32_t)0b010 << 21;
    static constexpr uint32_t FS_10 = FS & (uint32_t)0x3FF;

    static constexpr uint32_t Filter_0_Setup = 
        SINC_4 | 
        FS_10;
    
};

#endif
