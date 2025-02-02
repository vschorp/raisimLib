/**
 *
 * "A Collection of Useful C++ Classes for Digital Signal Processing"
 * By Vinnie Falco and Bernd Porr
 *
 * Official project location:
 * https://github.com/berndporr/iir1
 *
 * See Documentation.txt for contact information, notes, and bibliography.
 *
 * -----------------------------------------------------------------
 *
 * License: MIT License (http://www.opensource.org/licenses/mit-license.php)
 * Copyright (c) 2009 by Vinnie Falco
 * Copyright (c) 2011-2019 by Bernd Porr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 **/

#ifndef IIR1_CHEBYSHEVI_H
#define IIR1_CHEBYSHEVI_H

#include "Cascade.h"
#include "Common.h"
#include "PoleFilter.h"
#include "State.h"

namespace Iir {

/**
 * Filters with Chebyshev response characteristics. The last parameter
 * defines the passband ripple in decibel.
 **/
namespace ChebyshevI {

// Half-band analog prototypes (s-plane)

class DllExport AnalogLowPass : public LayoutBase {
public:
  AnalogLowPass();

  void design(const int numPoles, double rippleDb);

private:
  int m_numPoles;
  double m_rippleDb;
};

//------------------------------------------------------------------------------

class DllExport AnalogLowShelf : public LayoutBase {
public:
  AnalogLowShelf();

  void design(int numPoles, double gainDb, double rippleDb);

private:
  int m_numPoles;
  double m_rippleDb;
  double m_gainDb;
};

//------------------------------------------------------------------------------

// Factored implementations to reduce template instantiations

struct DllExport LowPassBase : PoleFilterBase<AnalogLowPass> {
  void setup(int order, double sampleRate, double cutoffFrequency,
             double rippleDb);
};

struct DllExport HighPassBase : PoleFilterBase<AnalogLowPass> {
  void setup(int order, double sampleRate, double cutoffFrequency,
             double rippleDb);
};

struct DllExport BandPassBase : PoleFilterBase<AnalogLowPass> {
  void setup(int order, double sampleRate, double centerFrequency,
             double widthFrequency, double rippleDb);
};

struct DllExport BandStopBase : PoleFilterBase<AnalogLowPass> {
  void setup(int order, double sampleRate, double centerFrequency,
             double widthFrequency, double rippleDb);
};

struct DllExport LowShelfBase : PoleFilterBase<AnalogLowShelf> {
  void setup(int order, double sampleRate, double cutoffFrequency,
             double gainDb, double rippleDb);
};

struct DllExport HighShelfBase : PoleFilterBase<AnalogLowShelf> {
  void setup(int order, double sampleRate, double cutoffFrequency,
             double gainDb, double rippleDb);
};

struct DllExport BandShelfBase : PoleFilterBase<AnalogLowShelf> {
  void setup(int order, double sampleRate, double centerFrequency,
             double widthFrequency, double gainDb, double rippleDb);
};

//------------------------------------------------------------------------------

//
// Userland filters
//

/**
 * ChebyshevI lowpass filter
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 */
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport LowPass : PoleFilter<LowPassBase, StateType, FilterOrder> {
  /**
   * Calculates the coefficients of the filter at the order FilterOrder
   * \param sampleRate Sampling rate
   * \param cutoffFrequency Cutoff frequency.
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(double sampleRate, double cutoffFrequency, double rippleDb) {
    LowPassBase::setup(FilterOrder, sampleRate, cutoffFrequency, rippleDb);
  }

  /**
   * Calculates the coefficients of the filter at specified order
   * \param reqOrder Actual order for the filter calculations
   * \param sampleRate Sampling rate
   * \param cutoffFrequency Cutoff frequency.
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(int reqOrder, double sampleRate, double cutoffFrequency,
             double rippleDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    LowPassBase::setup(reqOrder, sampleRate, cutoffFrequency, rippleDb);
  }
};

/**
 * ChebyshevI highpass filter
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 */
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport HighPass : PoleFilter<HighPassBase, StateType, FilterOrder> {
  /**
   * Calculates the coefficients of the filter at the order FilterOrder
   * \param sampleRate Sampling rate
   * \param cutoffFrequency Cutoff frequency.
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(double sampleRate, double cutoffFrequency, double rippleDb) {
    HighPassBase::setup(FilterOrder, sampleRate, cutoffFrequency, rippleDb);
  }

  /**
   * Calculates the coefficients of the filter at specified order
   * \param reqOrder Actual order for the filter calculations
   * \param sampleRate Sampling rate
   * \param cutoffFrequency Cutoff frequency.
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(int reqOrder, double sampleRate, double cutoffFrequency,
             double rippleDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    HighPassBase::setup(reqOrder, sampleRate, cutoffFrequency, rippleDb);
  }
};

/**
 * ChebyshevI bandpass filter
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 */
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport BandPass
    : PoleFilter<BandPassBase, StateType, FilterOrder, FilterOrder * 2> {
  /**
   * Calculates the coefficients of the filter at the order FilterOrder
   * \param sampleRate Sampling rate
   * \param centerFrequency Center frequency of the bandpass
   * \param widthFrequency Frequency with of the passband
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(double sampleRate, double centerFrequency, double widthFrequency,
             double rippleDb) {
    BandPassBase::setup(FilterOrder, sampleRate, centerFrequency,
                        widthFrequency, rippleDb);
  }

  /**
   * Calculates the coefficients of the filter at specified order
   * \param reqOrder Actual order for the filter calculations
   * \param sampleRate Sampling rate
   * \param centerFrequency Center frequency of the bandpass
   * \param widthFrequency Frequency with of the passband
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(int reqOrder, double sampleRate, double centerFrequency,
             double widthFrequency, double rippleDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    BandPassBase::setup(reqOrder, sampleRate, centerFrequency, widthFrequency,
                        rippleDb);
  }
};

/**
 * ChebyshevI bandstop filter
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 */
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport BandStop
    : PoleFilter<BandStopBase, StateType, FilterOrder, FilterOrder * 2> {
  /**
   * Calculates the coefficients of the filter at the order FilterOrder
   * \param sampleRate Sampling rate
   * \param centerFrequency Center frequency of the notch
   * \param widthFrequency Frequency with of the notch
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(double sampleRate, double centerFrequency, double widthFrequency,
             double rippleDb) {
    BandStopBase::setup(FilterOrder, sampleRate, centerFrequency,
                        widthFrequency, rippleDb);
  }

  /**
   * Calculates the coefficients of the filter at specified order
   * \param reqOrder Actual order for the filter calculations
   * \param sampleRate Sampling rate
   * \param centerFrequency Center frequency of the notch
   * \param widthFrequency Frequency with of the notch
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(int reqOrder, double sampleRate, double centerFrequency,
             double widthFrequency, double rippleDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    BandStopBase::setup(reqOrder, sampleRate, centerFrequency, widthFrequency,
                        rippleDb);
  }
};

/**
 * ChebyshevI low shelf filter. Specified gain in the passband. Otherwise 0 dB.
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 **/
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport LowShelf : PoleFilter<LowShelfBase, StateType, FilterOrder> {
  /**
   * Calculates the coefficients of the filter at the order FilterOrder
   * \param sampleRate Sampling rate
   * \param cutoffFrequency Cutoff frequency.
   * \param gainDb Gain in the passband
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(double sampleRate, double cutoffFrequency, double gainDb,
             double rippleDb) {
    LowShelfBase::setup(FilterOrder, sampleRate, cutoffFrequency, gainDb,
                        rippleDb);
  }

  /**
   * Calculates the coefficients of the filter at specified order
   * \param reqOrder Actual order for the filter calculations
   * \param sampleRate Sampling rate
   * \param cutoffFrequency Cutoff frequency.
   * \param gainDb Gain in the passband
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(int reqOrder, double sampleRate, double cutoffFrequency,
             double gainDb, double rippleDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    LowShelfBase::setup(reqOrder, sampleRate, cutoffFrequency, gainDb,
                        rippleDb);
  }
};

/**
 * ChebyshevI high shelf filter. Specified gain in the passband. Otherwise 0 dB.
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 **/
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport HighShelf : PoleFilter<HighShelfBase, StateType, FilterOrder> {
  /**
   * Calculates the coefficients of the filter at the order FilterOrder
   * \param sampleRate Sampling rate
   * \param cutoffFrequency Cutoff frequency.
   * \param gainDb Gain in the passband
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(double sampleRate, double cutoffFrequency, double gainDb,
             double rippleDb) {
    HighShelfBase::setup(FilterOrder, sampleRate, cutoffFrequency, gainDb,
                         rippleDb);
  }

  /**
   * Calculates the coefficients of the filter at specified order
   * \param reqOrder Actual order for the filter calculations
   * \param sampleRate Sampling rate
   * \param cutoffFrequency Cutoff frequency.
   * \param gainDb Gain in the passband
   * \param rippleDb Permitted ripples in dB in the passband
   **/
  void setup(int reqOrder, double sampleRate, double cutoffFrequency,
             double gainDb, double rippleDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    HighShelfBase::setup(reqOrder, sampleRate, cutoffFrequency, gainDb,
                         rippleDb);
  }
};

/**
 * ChebyshevI bandshelf filter. Specified gain in the passband. Otherwise 0 dB.
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 **/
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport BandShelf
    : PoleFilter<BandShelfBase, StateType, FilterOrder, FilterOrder * 2> {
  /**
   * Calculates the coefficients of the filter at the order FilterOrder
   * \param sampleRate Sampling rate
   * \param centerFrequency Center frequency of the passband
   * \param widthFrequency Width of the passband.
   * \param gainDb Gain in the passband. The stopband has 0 dB.
   * \param rippleDb Permitted ripples in dB in the passband.
   **/
  void setup(double sampleRate, double centerFrequency, double widthFrequency,
             double gainDb, double rippleDb) {
    BandShelfBase::setup(FilterOrder, sampleRate, centerFrequency,
                         widthFrequency, gainDb, rippleDb);
  }

  /**
   * Calculates the coefficients of the filter at specified order
   * \param reqOrder Actual order for the filter calculations
   * \param sampleRate Sampling rate
   * \param centerFrequency Center frequency of the passband
   * \param widthFrequency Width of the passband.
   * \param gainDb Gain in the passband. The stopband has 0 dB.
   * \param rippleDb Permitted ripples in dB in the passband.
   **/
  void setup(int reqOrder, double sampleRate, double centerFrequency,
             double widthFrequency, double gainDb, double rippleDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    BandShelfBase::setup(reqOrder, sampleRate, centerFrequency, widthFrequency,
                         gainDb, rippleDb);
  }
};

} // namespace ChebyshevI

} // namespace Iir

#endif
