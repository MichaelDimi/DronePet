#include "GyroISM330DHCX.hpp"

#include "Hal/Time.hpp"
#include "Utils/MemoryHelper.h"

namespace {

// I2C addresses, used only if this driver is ever used over I2C.
constexpr uint8_t ADDRESS_LOW  = 0x6A;
constexpr uint8_t ADDRESS_HIGH = 0x6B;


// --------------------------------------------------
// ISM330DHCX register map
// --------------------------------------------------

constexpr uint8_t REG_WHO_AM_I = 0x0F;

constexpr uint8_t REG_CTRL1_XL = 0x10;
constexpr uint8_t REG_CTRL2_G  = 0x11;
constexpr uint8_t REG_CTRL3_C  = 0x12;
constexpr uint8_t REG_CTRL4_C  = 0x13;
constexpr uint8_t REG_CTRL6_C  = 0x15;
constexpr uint8_t REG_CTRL7_G  = 0x16;
constexpr uint8_t REG_CTRL8_XL = 0x17;
constexpr uint8_t REG_CTRL9_XL = 0x18;

constexpr uint8_t REG_OUTX_L_G  = 0x22;
constexpr uint8_t REG_OUTX_L_XL = 0x28;

// Exact ISM330DHCX ID.
constexpr uint8_t WHO_AM_I_VALUE = 0x6B;

// --------------------------------------------------
// CTRL1_XL and CTRL8_XL
// --------------------------------------------------

constexpr uint8_t CTRL1_LPF2_XL_EN = 0x02;

constexpr uint8_t CTRL8_HPCF_XL_MASK = 0xE0;
constexpr uint8_t CTRL8_HPCF_XL_ODR_10 = 0x20;
constexpr uint8_t CTRL8_HP_REF_MODE_XL = 0x10;
constexpr uint8_t CTRL8_FASTSETTL_MODE_XL = 0x08;
constexpr uint8_t CTRL8_HP_SLOPE_XL_EN = 0x04;

// --------------------------------------------------
// CTRL3_C
// --------------------------------------------------

constexpr uint8_t CTRL3_SW_RESET = 0x01;
constexpr uint8_t CTRL3_IF_INC   = 0x04;
constexpr uint8_t CTRL3_SIM      = 0x08;
constexpr uint8_t CTRL3_BDU      = 0x40;


// --------------------------------------------------
// CTRL4_C
// --------------------------------------------------

constexpr uint8_t CTRL4_LPF1_SEL_G = 0x02;
constexpr uint8_t CTRL4_I2C_DISABLE = 0x04;


// --------------------------------------------------
// CTRL6_C
// --------------------------------------------------

constexpr uint8_t CTRL6_FTYPE_MASK  = 0x07;
constexpr uint8_t CTRL6_XL_HM_MODE  = 0x10;

// FTYPE = 010.
// At 3332/3333 Hz ODR this is about a 153 Hz LPF1 bandwidth.
constexpr uint8_t CTRL6_FTYPE_153HZ = 0x02;


// --------------------------------------------------
// CTRL7_G
// --------------------------------------------------

constexpr uint8_t CTRL7_HP_EN_G    = 0x40;
constexpr uint8_t CTRL7_G_HM_MODE  = 0x80;


// --------------------------------------------------
// CTRL9_XL
// --------------------------------------------------

constexpr uint8_t CTRL9_DEVICE_CONF = 0x02;

// --------------------------------------------------
// Accelerometer
// --------------------------------------------------

// ODR_XL = 0111 -> 833 Hz
constexpr uint8_t ACCEL_ODR_833HZ = 0x07;

// FS_XL = 01 -> +/-16 g
constexpr uint8_t ACCEL_FS_16G = 0x01;


// --------------------------------------------------
// Gyroscope
// --------------------------------------------------

// ODR_G = 1001 -> 3332 Hz
constexpr uint8_t GYRO_ODR_3332HZ = 0x09;

// FS_G nibble = 1100 -> +/-2000 dps
constexpr uint8_t GYRO_FS_2000DPS = 0x0C;

constexpr int GYRO_RATE_HZ = 3332;

} // namespace


namespace Espfc::Device::Gyro {

int GyroISM330DHCX::begin(BusDevice* bus)
{
    return begin(bus, ADDRESS_LOW)
        ? 1
        : begin(bus, ADDRESS_HIGH)
            ? 1
            : 0;
}


int GyroISM330DHCX::begin(
    BusDevice* bus,
    uint8_t addr
)
{
    setBus(bus, addr);

    if (!testConnection()) {
        return 0;
    }

    if (!reset()) {
        return 0;
    }

    if (!configure()) {
        return 0;
    }

    if (!verifyConfiguration()) {
        return 0;
    }

    return 1;
}


GyroDeviceType GyroISM330DHCX::getType() const
{
    return GYRO_ISM330DHCX;
}


int FAST_CODE_ATTR GyroISM330DHCX::readGyro(
    VectorInt16& v
)
{
    int16_t buffer[3] = {};

    if (
        _bus->readFast(
            _addr,
            REG_OUTX_L_G,
            6,
            reinterpret_cast<uint8_t*>(buffer)
        ) != 6
    ) {
        return 0;
    }

    v.x = buffer[0];
    v.y = buffer[1];
    v.z = buffer[2];

    return 1;
}


int GyroISM330DHCX::readAccel(
    VectorInt16& v
)
{
    int16_t buffer[3] = {};

    if (
        _bus->readFast(
            _addr,
            REG_OUTX_L_XL,
            6,
            reinterpret_cast<uint8_t*>(buffer)
        ) != 6
    ) {
        return 0;
    }

    v.x = buffer[0];
    v.y = buffer[1];
    v.z = buffer[2];

    return 1;
}


void GyroISM330DHCX::setDLPFMode(
    uint8_t mode
)
{
    // ESP-FC's gyro_dlpf setting is based on other IMUs and
    // does not map directly to the ISM330DHCX FTYPE values.
    //
    // Keep our explicit datasheet-derived LPF1 configuration.
    (void)mode;

    _bus->writeMask(
        _addr,
        REG_CTRL6_C,
        CTRL6_FTYPE_MASK,
        CTRL6_FTYPE_153HZ
    );
}


int GyroISM330DHCX::getRate() const
{
    return GYRO_RATE_HZ;
}


void GyroISM330DHCX::setRate(
    int rate
)
{
    // DronePet currently uses a fixed 3332 Hz sensor rate.
    //
    // ESP-FC calls setRate(getRate()) during startup, so
    // there is nothing further to change here.
    (void)rate;
}


bool GyroISM330DHCX::testConnection()
{
    uint8_t whoami = 0;

    if (
        _bus->readByte(
            _addr,
            REG_WHO_AM_I,
            &whoami
        ) != 1
    ) {
        return false;
    }

    setChipId(whoami);

    return whoami == WHO_AM_I_VALUE;
}


bool GyroISM330DHCX::reset()
{
    if (
        !_bus->writeMask(
            _addr,
            REG_CTRL3_C,
            CTRL3_SW_RESET,
            CTRL3_SW_RESET
        )
    ) {
        return false;
    }

    // The ST initialization sequence waits until
    // SW_RESET clears before configuring the device.
    for (int i = 0; i < 100; ++i) {

        uint8_t value = 0;

        if (
            _bus->readByte(
                _addr,
                REG_CTRL3_C,
                &value
            ) != 1
        ) {
            return false;
        }

        if ((value & CTRL3_SW_RESET) == 0) {
            return true;
        }

        delay(1);
    }

    return false;
}


bool GyroISM330DHCX::configure()
{
    // DEVICE_CONF = 1.
    //
    // This is an ISM330DHCX-specific bit.
    // The old LSM6DSO driver incorrectly called this
    // an "I3C disable" bit.
    if (
        !_bus->writeMask(
            _addr,
            REG_CTRL9_XL,
            CTRL9_DEVICE_CONF,
            CTRL9_DEVICE_CONF
        )
    ) {
        return false;
    }


    // Block Data Update:
    // do not update the high/low bytes in the middle
    // of one sample read.
    //
    // IF_INC:
    // automatically move through X/Y/Z registers during
    // our six-byte burst read.
    //
    // SIM = 0:
    // four-wire SPI.
    const uint8_t ctrl3Mask =
        CTRL3_BDU |
        CTRL3_SIM |
        CTRL3_IF_INC;

    const uint8_t ctrl3Value =
        CTRL3_BDU |
        CTRL3_IF_INC;

    if (
        !_bus->writeMask(
            _addr,
            REG_CTRL3_C,
            ctrl3Mask,
            ctrl3Value
        )
    ) {
        return false;
    }


    // Enable the UI gyro LPF1.
    //
    // Only disable I2C when we are actually using SPI.
    const uint8_t ctrl4Value =
        CTRL4_LPF1_SEL_G |
        (_bus->isSPI()
            ? CTRL4_I2C_DISABLE
            : 0);

    if (
        !_bus->writeMask(
            _addr,
            REG_CTRL4_C,
            CTRL4_LPF1_SEL_G |
                CTRL4_I2C_DISABLE,
            ctrl4Value
        )
    ) {
        return false;
    }


    // Accelerometer high-performance mode:
    // XL_HM_MODE = 0.
    //
    // Gyro LPF1:
    // FTYPE = 010 -> ~153 Hz at 3332 Hz ODR.
    if (
        !_bus->writeMask(
            _addr,
            REG_CTRL6_C,
            CTRL6_XL_HM_MODE |
                CTRL6_FTYPE_MASK,
            CTRL6_FTYPE_153HZ
        )
    ) {
        return false;
    }


    // Gyroscope high-performance mode:
    // G_HM_MODE = 0.
    //
    // Do not enable the gyro high-pass filter.
    if (
        !_bus->writeMask(
            _addr,
            REG_CTRL7_G,
            CTRL7_G_HM_MODE |
                CTRL7_HP_EN_G,
            0
        )
    ) {
        return false;
    }


    // Accelerometer LPF2:
    // low-pass path, cutoff = ODR / 10.
    // At 833 Hz this is approximately 83 Hz.
    // Keep HP reference, fast-settle, and high-pass/slope modes disabled.
    if (
        !_bus->writeMask(
            _addr,
            REG_CTRL8_XL,
            CTRL8_HPCF_XL_MASK |
                CTRL8_HP_REF_MODE_XL |
                CTRL8_FASTSETTL_MODE_XL |
                CTRL8_HP_SLOPE_XL_EN,
            CTRL8_HPCF_XL_ODR_10
        )
    ) {
        return false;
    }


    // Accelerometer:
    // 833 Hz
    // +/-16 g
    // LPF2 enabled
    const uint8_t ctrl1 =
        static_cast<uint8_t>(
            (ACCEL_ODR_833HZ << 4) |
            (ACCEL_FS_16G << 2) |
            CTRL1_LPF2_XL_EN
        );

    if (
        !_bus->writeByte(
            _addr,
            REG_CTRL1_XL,
            ctrl1
        )
    ) {
        return false;
    }


    // Gyroscope:
    // 3332 Hz
    // +/-2000 dps
    //
    // Configure it last so the gyro begins producing
    // samples only after the rest of the signal chain
    // has been configured.
    const uint8_t ctrl2 =
        static_cast<uint8_t>(
            (GYRO_ODR_3332HZ << 4) |
            GYRO_FS_2000DPS
        );

    if (
        !_bus->writeByte(
            _addr,
            REG_CTRL2_G,
            ctrl2
        )
    ) {
        return false;
    }

    // LPF2 at ODR/10 has a 10-sample settling time.
    // At 833 Hz that is about 12 ms, so allow 20 ms here.
    delay(20);

    return true;
}


bool GyroISM330DHCX::verifyConfiguration()
{
    uint8_t value = 0;


    // CTRL1_XL should be:
    //
    // ODR  = 833 Hz
    // FS   = +/-16 g
    // LPF2 = enabled
    const uint8_t expectedCtrl1 =
        static_cast<uint8_t>(
            (ACCEL_ODR_833HZ << 4) |
            (ACCEL_FS_16G << 2) |
            CTRL1_LPF2_XL_EN
        );

    if (
        _bus->readByte(
            _addr,
            REG_CTRL1_XL,
            &value
        ) != 1 ||
        value != expectedCtrl1
    ) {
        return false;
    }


    // CTRL8_XL should be:
    //
    // HPCF_XL = 001 -> LPF2 cutoff = ODR / 10 (~83 Hz)
    // HP_REF_MODE_XL = 0
    // FASTSETTL_MODE_XL = 0
    // HP_SLOPE_XL_EN = 0 -> low-pass path
    if (
        _bus->readByte(
            _addr,
            REG_CTRL8_XL,
            &value
        ) != 1
    ) {
        return false;
    }

    if ((value & CTRL8_HPCF_XL_MASK) != CTRL8_HPCF_XL_ODR_10) {
        return false;
    }

    if ((value & CTRL8_HP_REF_MODE_XL) != 0) {
        return false;
    }

    if ((value & CTRL8_FASTSETTL_MODE_XL) != 0) {
        return false;
    }

    if ((value & CTRL8_HP_SLOPE_XL_EN) != 0) {
        return false;
    }


    // CTRL2_G should be:
    //
    // ODR = 3332 Hz
    // FS  = +/-2000 dps
    const uint8_t expectedCtrl2 =
        static_cast<uint8_t>(
            (GYRO_ODR_3332HZ << 4) |
            GYRO_FS_2000DPS
        );

    if (
        _bus->readByte(
            _addr,
            REG_CTRL2_G,
            &value
        ) != 1 ||
        value != expectedCtrl2
    ) {
        return false;
    }


    if (
        _bus->readByte(
            _addr,
            REG_CTRL3_C,
            &value
        ) != 1
    ) {
        return false;
    }

    if ((value & CTRL3_BDU) == 0) {
        return false;
    }

    if ((value & CTRL3_IF_INC) == 0) {
        return false;
    }

    if ((value & CTRL3_SIM) != 0) {
        return false;
    }


    if (
        _bus->readByte(
            _addr,
            REG_CTRL9_XL,
            &value
        ) != 1
    ) {
        return false;
    }

    if ((value & CTRL9_DEVICE_CONF) == 0) {
        return false;
    }


    return true;
}

} // namespace Espfc::Device::Gyro