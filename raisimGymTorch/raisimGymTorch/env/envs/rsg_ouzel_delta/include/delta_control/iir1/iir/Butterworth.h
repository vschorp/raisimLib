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

#ifndef IIR1_BUTTERWORTH_H
#define IIR1_BUTTERWORTH_H

#include "Cascade.h"
#include "Common.h"
#include "PoleFilter.h"
#include "State.h"

namespace Iir {

/**
 * Filters with Butterworth response characteristics. The filter order is
 *usually set via the template parameter which reserves the correct space and is
 *then automatically passed to the setup function. Optionally one can also
 *provde the filter order at setup time to force a lower order than the default
 *one.
 **/
namespace Butterworth {

/**
 * Half-band analog prototypes (s-plane)
 **/
class DllExport AnalogLowPass : public LayoutBase {
public:
  AnalogLowPass();

  void design(const int numPoles);

private:
  int m_numPoles;
};

//------------------------------------------------------------------------------

class DllExport AnalogLowShelf : public LayoutBase {
public:
  AnalogLowShelf();

  void design(int numPoles, double gainDb);

private:
  int m_numPoles;
  double m_gainDb;
};

//------------------------------------------------------------------------------

// Factored implementations to reduce template instantiations

struct DllExport LowPassBase : PoleFilterBase<AnalogLowPass> {
  void setup(int order, double sampleRate, double cutoffFrequency);
};

struct DllExport HighPassBase : PoleFilterBase<AnalogLowPass> {
  void setup(int order, double sampleRate, double cutoffFrequency);
};

struct DllExport BandPassBase : PoleFilterBase<AnalogLowPass> {
  void setup(int order, double sampleRate, double centerFrequency,
             double widthFrequency);
};

struct DllExport BandStopBase : PoleFilterBase<AnalogLowPass> {
  void setup(int order, double sampleRate, double centerFrequency,
             double widthFrequency);
};

struct DllExport LowShelfBase : PoleFilterBase<AnalogLowShelf> {
  void setup(int order, double sampleRate, double cutoffFrequency,
             double gainDb);
};

struct DllExport HighShelfBase : PoleFilterBase<AnalogLowShelf> {
  void setup(int order, double sampleRate, double cutoffFrequency,
             double gainDb);
};

struct DllExport BandShelfBase : PoleFilterBase<AnalogLowShelf> {
  void setup(int order, double sampleRate, double centerFrequency,
             double widthFrequency, double gainDb);
};

//------------------------------------------------------------------------------

//
// Filters for the user
//

/**
 * Butterworth Lowpass filter.
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 */
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport LowPass : PoleFilter<LowPassBase, StateType, FilterOrder> {
  /**
   * Calculates the coefficients
   * \param sampleRate Sampling rate
   * \param cutoffFrequency Cutoff
   **/
  void setup(double sampleRate, double cutoffFrequency) {
    LowPassBase::setup(FilterOrder, sampleRate, cutoffFrequency);
  }
  /**
   * Calculates the coefficients
   * \param reqOrder The actual order which can be less than the instantiated
   *one \param sampleRate Sampling rate \param cutoffFrequency Cutoff
   **/
  void setup(int reqOrder, double sampleRate, double cutoffFrequency) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    LowPassBase::setup(reqOrder, sampleRate, cutoffFrequency);
  }
};

/**
 * Butterworth Highpass filter.
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 */
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport HighPass : PoleFilter<HighPassBase, StateType, FilterOrder> {
  /**
   * Calculates the coefficients with the filter order provided by the
   *instantiation \param sampleRate Sampling rate \param cutoffFrequency Cutoff
   **/
  void setup(double sampleRate, double cutoffFrequency) {
    HighPassBase::setup(FilterOrder, sampleRate, cutoffFrequency);
  }
  /**
   * Calculates the coefficients
   * \param reqOrder The actual order which can be less than the instantiated
   *one \param sampleRate Sampling rate \param cutoffFrequency Cutoff
   **/
  void setup(int reqOrder, double sampleRate, double cutoffFrequency) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    HighPassBase::setup(reqOrder, sampleRate, cutoffFrequency);
  }
};

/**
 * Butterworth  Bandpass filter.
 * \param FilterOrder  Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 */
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport BandPass
    : PoleFilter<BandPassBase, StateType, FilterOrder, FilterOrder * 2> {
  /**
   * Calculates the coefficients with the filter order provided by the
   *instantiation \param sampleRate Sampling rate \param centerFrequency Centre
   *frequency of the bandpass \param widthFrequency Width of the bandpass
   **/
  void setup(double sampleRate, double centerFrequency, double widthFrequency) {
    BandPassBase::setup(FilterOrder, sampleRate, centerFrequency,
                        widthFrequency);
  }

  /**
   * Calculates the coefficients
   * \param reqOrder The actual order which can be less than the instantiated
   *one \param sampleRate Sampling rate \param centerFrequency Centre frequency
   *of the bandpass \param widthFrequency Width of the bandpass
   **/
  void setup(int reqOrder, double sampleRate, double centerFrequency,
             double widthFrequency) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    BandPassBase::setup(reqOrder, sampleRate, centerFrequency, widthFrequency);
  }
};

/**
 * Butterworth Bandstop filter.
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 */
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport BandStop
    : PoleFilter<BandStopBase, StateType, FilterOrder, FilterOrder * 2> {
  /**
   * Calculates the coefficients with the filter order provided by the
   *instantiation \param sampleRate Sampling rate \param centerFrequency Centre
   *frequency of the bandstop \param widthFrequency Width of the bandstop
   **/
  void setup(double sampleRate, double centerFrequency, double widthFrequency) {
    BandStopBase::setup(FilterOrder, sampleRate, centerFrequency,
                        widthFrequency);
  }

  /**
   * Calculates the coefficients
   * \param reqOrder The actual order which can be less than the instantiated
   *one \param sampleRate Sampling rate \param centerFrequency Centre frequency
   *of the bandstop \param widthFrequency Width of the bandstop
   **/
  void setup(int reqOrder, double sampleRate, double centerFrequency,
             double widthFrequency) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    BandStopBase::setup(reqOrder, sampleRate, centerFrequency, widthFrequency);
  }
};

/**
 * Butterworth low shelf filter: below the cutoff it has
 * a specified gain and above the cutoff the gain is 0 dB.
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 **/
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport LowShelf : PoleFilter<LowShelfBase, StateType, FilterOrder> {
  /**
   * Calculates the coefficients with the filter order provided by the
   *instantiation \param sampleRate Sampling rate \param cutoffFrequency Cutoff
   * \param gainDb Gain in dB of the filter in the passband
   **/
  void setup(double sampleRate, double cutoffFrequency, double gainDb) {
    LowShelfBase::setup(FilterOrder, sampleRate, cutoffFrequency, gainDb);
  }

  /**
   * Calculates the coefficients
   * \param reqOrder The actual order which can be less than the instantiated
   *one \param sampleRate Sampling rate \param cutoffFrequency Cutoff \param
   *gainDb Gain in dB of the filter in the passband
   **/
  void setup(int reqOrder, double sampleRate, double cutoffFrequency,
             double gainDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    LowShelfBase::setup(reqOrder, sampleRate, cutoffFrequency, gainDb);
  }
};

/**
 * Butterworth high shelf filter. Above the cutoff the filter has
 * a specified gain and below it has 0 dB.
 * \param FilterOrder Reserves memory for a filter of the order FilterOrder
 * \param StateType The filter topology: DirectFormI, DirectFormII, ...
 **/
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport HighShelf : PoleFilter<HighShelfBase, StateType, FilterOrder> {
  /**
   * Calculates the coefficients with the filter order provided by the
   *instantiation \param sampleRate Sampling rate \param cutoffFrequency Cutoff
   * \param gainDb Gain in dB of the filter in the passband
   **/
  void setup(double sampleRate, double cutoffFrequency, double gainDb) {
    HighShelfBase::setup(FilterOrder, sampleRate, cutoffFrequency, gainDb);
  }

  /**
   * Calculates the coefficients
   * \param reqOrder The actual order which can be less than the instantiated
   *one \param sampleRate Sampling rate \param cutoffFrequency Cutoff \param
   *gainDb Gain in dB of the filter in the passband
   **/
  void setup(int reqOrder, double sampleRate, double cutoffFrequency,
             double gainDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    HighShelfBase::setup(reqOrder, sampleRate, cutoffFrequency, gainDb);
  }
};

/**
 * Butterworth Bandshelf filter: it is a bandpass filter which amplifies at a
 * specified gain in dB the frequencies in the passband. \param FilterOrder
 * Reserves memory for a filter of the order FilterOrder \param StateType The
 * filter topology: DirectFormI, DirectFormII, ...
 */
template <int FilterOrder, class StateType = DEFAULT_STATE>
struct DllExport BandShelf
    : PoleFilter<BandShelfBase, StateType, FilterOrder, FilterOrder * 2> {
  /**
   * Calculates the coefficients with the filter order provided by the
   *instantiation \param sampleRate Sampling rate \param centerFrequency Centre
   *frequency of the passband \param widthFrequency Width of the passband \param
   *gainDb The gain in the passband
   **/
  void setup(double sampleRate, double centerFrequency, double widthFrequency,
             double gainDb) {
    BandShelfBase::setup(FilterOrder, sampleRate, centerFrequency,
                         widthFrequency, gainDb);
  }

  /**
   * Calculates the coefficients
   * \param reqOrder The actual order which can be less than the instantiated
   *one \param sampleRate Sampling rate \param centerFrequency Centre frequency
   *of the passband \param widthFrequency Width of the passband \param gainDb
   *The gain in the passband
   **/
  void setup(int reqOrder, double sampleRate, double centerFrequency,
             double widthFrequency, double gainDb) {
    if (reqOrder > FilterOrder)
      throw std::invalid_argument(orderTooHigh);
    BandShelfBase::setup(reqOrder, sampleRate, centerFrequency, widthFrequency,
                         gainDb);
  }
};

} // namespace Butterworth

} // namespace Iir

#endif
