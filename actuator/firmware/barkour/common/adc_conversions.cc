// Copyright 2024 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ==============================================================================

#include "actuator/firmware/barkour/common/adc_conversions.h"

#include <cmath>
#include <cstdint>

#include "actuator/firmware/barkour/common/interfaces/adc_interface.h"
#include "pw_result/result.h"
#include "pw_status/status.h"

namespace barkour {

namespace {

constexpr uint32_t kAdcMaxCount = 65536;
constexpr float kAdcReferenceVoltage = 3.3f;
constexpr float kAdcPwrInGain = 20.0f;
constexpr float kAdcVoltsPerCount =
    kAdcReferenceVoltage / static_cast<float>(UINT16_MAX);

constexpr float kShuntResistance = 0.0015f;

// Lookup table for the thermistor temp sensing transfer function.
//
// In order to obtain a temperature in Celsius, divide the ADC reading by the
// size of the lookup table.
//
// Note: performing the division in floating point, and rounding to the nearest
// value will result in improved accuracy.
//
// Further Details:
//   Thermistor: NCP15XH103F03RC
//
//   On Cortes, the thermistor is in a voltage divider configuration, with
//   the thermistor on the bottom of the divider. Because of this, the output
//   voltage of the sensing circuit decreases as the temperature sensed
//   increases. This is the reason why the temperatures in the table below
//   decrease as the ADC count increases. Because the sensing circuit is in a
//   voltage divider configuration, indices at both the low and high extremes of
//   the table will not be accessed.
constexpr uint16_t kPhaseThermistorLookupTableLength = 256;
constexpr float
    kCortesV5PhaseThermistorLookup[kPhaseThermistorLookupTableLength] = {
        -273.150, 297.851, 238.863, 209.574,  190.674, 176.949, 166.282,
        157.616,  150.352, 144.120, 138.677,  133.855, 129.533, 125.621,
        122.052,  118.772, 115.741, 112.924,  110.295, 107.830, 105.511,
        103.322,  101.249, 99.282,  97.409,   95.623,  93.916,  92.281,
        90.712,   89.204,  87.753,  86.354,   85.005,  83.700,  82.438,
        81.215,   80.030,  78.879,  77.761,   76.674,  75.616,  74.586,
        73.581,   72.602,  71.645,  70.711,   69.798,  68.905,  68.031,
        67.175,   66.337,  65.515,  64.709,   63.919,  63.143,  62.381,
        61.632,   60.896,  60.172,  59.460,   58.760,  58.070,  57.391,
        56.722,   56.063,  55.414,  54.773,   54.141,  53.517,  52.902,
        52.295,   51.695,  51.102,  50.517,   49.938,  49.366,  48.801,
        48.242,   47.688,  47.141,  46.599,   46.063,  45.532,  45.006,
        44.485,   43.969,  43.458,  42.951,   42.449,  41.951,  41.457,
        40.967,   40.481,  39.998,  39.520,   39.045,  38.573,  38.105,
        37.640,   37.178,  36.720,  36.264,   35.811,  35.361,  34.914,
        34.469,   34.027,  33.587,  33.150,   32.715,  32.282,  31.851,
        31.423,   30.996,  30.572,  30.149,   29.728,  29.309,  28.892,
        28.476,   28.062,  27.650,  27.239,   26.829,  26.420,  26.013,
        25.607,   25.203,  24.799,  24.397,   23.995,  23.595,  23.195,
        22.796,   22.398,  22.001,  21.604,   21.208,  20.813,  20.418,
        20.024,   19.630,  19.237,  18.844,   18.451,  18.059,  17.666,
        17.274,   16.882,  16.490,  16.098,   15.706,  15.314,  14.921,
        14.529,   14.136,  13.743,  13.349,   12.955,  12.561,  12.166,
        11.770,   11.374,  10.977,  10.580,   10.181,  9.782,   9.381,
        8.980,    8.577,   8.174,   7.769,    7.363,   6.955,   6.546,
        6.136,    5.724,   5.310,   4.895,    4.478,   4.058,   3.637,
        3.214,    2.789,   2.361,   1.931,    1.499,   1.064,   0.626,
        0.185,    -0.258,  -0.705,  -1.155,   -1.608,  -2.064,  -2.524,
        -2.988,   -3.456,  -3.928,  -4.404,   -4.885,  -5.370,  -5.860,
        -6.355,   -6.855,  -7.361,  -7.873,   -8.390,  -8.914,  -9.444,
        -9.982,   -10.526, -11.078, -11.639,  -12.207, -12.784, -13.371,
        -13.967,  -14.573, -15.190, -15.819,  -16.460, -17.114, -17.781,
        -18.463,  -19.160, -19.874, -20.605,  -21.355, -22.126, -22.918,
        -23.733,  -24.575, -25.443, -26.342,  -27.274, -28.242, -29.249,
        -30.301,  -31.402, -32.559, -33.778,  -35.068, -36.441, -37.909,
        -39.490,  -41.205, -43.086, -45.174,  -47.527, -50.238, -53.455,
        -57.450,  -62.818, -71.358, -273.150,
};

float CortesTherm123AdcReadingToTemp(int32_t reading) {
  float table_position =
      static_cast<float>(reading) / kPhaseThermistorLookupTableLength;
  float whole;
  float fractional;
  float result;
  fractional = std::modf(table_position, &whole);

  if (whole < (kPhaseThermistorLookupTableLength - 1)) {
    float v1 = kCortesV5PhaseThermistorLookup[static_cast<int>(whole)];
    float v2 = kCortesV5PhaseThermistorLookup[static_cast<int>(whole) + 1];
    result = v1 + fractional * (v2 - v1);
  } else {
    result = kCortesV5PhaseThermistorLookup[static_cast<int>(whole)];
  }
  return result;
}

float GebruPhaseShuntAdcReadingToAmps(int32_t reading) {
  constexpr float kShuntCsaGain = 20.0f;
  constexpr float divby = kShuntCsaGain * kShuntResistance;

  constexpr float toAmps = kAdcVoltsPerCount / divby;

  return (static_cast<float>(reading)) * toAmps;
}

float Adc24VSafeReadingToVolts(int32_t reading) {
  constexpr float kDividerHighResistance = 56000.0f;
  constexpr float kDividerLowResistance = 6190.0f;
  constexpr float kDividerGain =
      kDividerLowResistance / (kDividerLowResistance + kDividerHighResistance);
  constexpr float kConversionFactor =
      kAdcReferenceVoltage / (kDividerGain * kAdcMaxCount);
  return (kConversionFactor * reading);
}

float Therm123AdcReadingToTemp(int32_t reading) {
  return CortesTherm123AdcReadingToTemp(reading);
}

float PhaseShuntAdcReadingToAmps(int32_t reading) {
  return GebruPhaseShuntAdcReadingToAmps(reading);
}

float PwrInAdcReadingToVolts(int32_t reading) {
  return kAdcPwrInGain * kAdcReferenceVoltage * reading / kAdcMaxCount;
}

}  // namespace

pw::Result<float> AdcReadingToFloat(AdcAnalogSignal signal, int32_t reading) {
  float result;
  switch (signal) {
    // Phase resistances.
    case AdcAnalogSignal::kPhaseACurrent:
    case AdcAnalogSignal::kPhaseBCurrent:
    case AdcAnalogSignal::kPhaseCCurrent: {
      result = PhaseShuntAdcReadingToAmps(reading);
      break;
    }
    case AdcAnalogSignal::k24VSafeVoltage: {
      result = Adc24VSafeReadingToVolts(reading);
      break;
    }
    case AdcAnalogSignal::kBusVoltage: {
      result = PwrInAdcReadingToVolts(reading);
      break;
    }
    case AdcAnalogSignal::kMotorTherm1:
    case AdcAnalogSignal::kMotorTherm2:
    case AdcAnalogSignal::kMotorTherm3:
    case AdcAnalogSignal::kEncoderTherm: {
      result = Therm123AdcReadingToTemp(reading);
      break;
    }
    // Conversions for the following readings have not yet been implemented.
    case AdcAnalogSignal::kDriveTherm:
    case AdcAnalogSignal::kH7TempSensor:
    case AdcAnalogSignal::kDrainSourceVoltageA:
    case AdcAnalogSignal::kDrainSourceVoltageB:
    case AdcAnalogSignal::kDrainSourceVoltageC:
    case AdcAnalogSignal::kPhaseAVoltage:
    case AdcAnalogSignal::kPhaseBVoltage:
    case AdcAnalogSignal::kPhaseCVoltage:
    case AdcAnalogSignal::kMotorCurrent:
    default: {
      return pw::Status::Unimplemented();
    }
  }

  return result;
}

}  // namespace barkour
