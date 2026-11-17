/* Copyright (C) 2003, 2004, 2005, 2006, 2008, 2009 Dean Beeler, Jerome Fisher
 * Copyright (C) 2011-2026 Dean Beeler, Jerome Fisher, Sergey V. Mikayev
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MT32EMU_LA32_WAVE_GENERATOR_H
#define MT32EMU_LA32_WAVE_GENERATOR_H

#include <cstddef>

#include "globals.h"
#include "internals.h"
#include "Types.h"

namespace MT32Emu {

struct Tables;

/**
 * LA32 performs wave generation in the log-space that allows replacing multiplications by cheap additions
 * It's assumed that only low-bit multiplications occur in a few places which are unavoidable like these:
 * - interpolation of exponent table (obvious, a delta value has 4 bits)
 * - computation of resonance amp decay envelope (the table contains values with 1-2 "1" bits except the very first value 31 but this case can be found using inversion)
 * - interpolation of PCM samples (obvious, the wave position counter is in the linear space, there is no log() table in the chip)
 * and it seems to be implemented in the same way as in the Boss chip, i.e. right shifted additions which involved noticeable precision loss
 * Subtraction is supposed to be replaced by simple inversion
 * As the logarithmic sine is always negative, all the logarithmic values are treated as decrements
 */
struct LogSample {
	// 16-bit fixed point value, includes 12-bit fractional part
	// 4-bit integer part allows to present any 16-bit sample in the log-space
	// Obviously, the log value doesn't contain the sign of the resulting sample
	Bit16u logValue;
	enum {
		POSITIVE,
		NEGATIVE
	} sign;
};

// These tables are accessed extremely often. Keeping the direct pointers here significantly improves performance in most cases.
extern const Bit16u *exp9;
extern const Bit16u *logsin9;
// This is accessed less often, but it's still on the critical path.
extern const Bit8u *intResAmpDecayFactors;
extern Bit16u expInterpolationTable[4096];

static const LogSample SILENCE = {65535, LogSample::POSITIVE};

namespace LA32Utilites {

static inline Bit16u interpolateExp(const Bit16u fract) {
	return expInterpolationTable[fract];
}

static inline Bit16s unlog(const LogSample &logSample) {
	//Bit16s sample = (Bit16s)EXP2F(13.0f - logSample.logValue / 4096.0f);
	Bit32u intLogValue = logSample.logValue >> 12;
	Bit16u fracLogValue = logSample.logValue & 4095;
	Bit16s sample = interpolateExp(fracLogValue) >> intLogValue;
	return logSample.sign == LogSample::POSITIVE ? sample : -sample;
}

static inline void addLogSamples(LogSample &logSample1, const LogSample &logSample2) {
	Bit32u logSampleValue = logSample1.logValue + logSample2.logValue;
	logSample1.logValue = logSampleValue < 65536 ? Bit16u(logSampleValue) : 65535;
	logSample1.sign = logSample1.sign == logSample2.sign ? LogSample::POSITIVE : LogSample::NEGATIVE;
}

} // namespace LA32Utilites

/**
 * LA32WaveGenerator is aimed to represent the exact model of LA32 wave generator.
 * The output square wave is created by adding high / low linear segments in-between
 * the rising and falling cosine segments. Basically, it's very similar to the phase distortion synthesis.
 * Behaviour of a true resonance filter is emulated by adding decaying sine wave.
 * The beginning and the ending of the resonant sine is multiplied by a cosine window.
 * To synthesise sawtooth waves, the resulting square wave is multiplied by synchronous cosine wave.
 */
class LA32WaveGenerator {
	//***************************************************************************
	//  The local copy of partial parameters below
	//***************************************************************************

	bool active;

	// True means the resulting square wave is to be multiplied by the synchronous cosine
	bool sawtoothWaveform;

	// Values in range [1..31]
	// Value 1 corresponds to the minimum resonance
	Bit8u resonance;

	// Processed value in range [0..255]
	// Values in range [0..128] have no effect and the resulting wave remains symmetrical
	// Value 255 corresponds to the maximum possible asymmetric of the resulting wave
	Bit8u pulseWidth;

	// Logarithmic PCM sample start address
	const Bit16s *pcmWaveAddress;

	// Logarithmic PCM sample length
	Bit32u pcmWaveLength;

	// true for looped logarithmic PCM samples
	bool pcmWaveLooped;

	// false for slave PCM partials in the structures with the ring modulation
	bool pcmWaveInterpolated;

	//***************************************************************************
	// Internal variables below
	//***************************************************************************

	// Logarithmic amp of the wave generator, scaled down by factor 2^-10
	Bit32u amp;

	// The increments applied to wavePosition per sample, depend on pitch
	Bit32u sampleStep;
	Bit32u pcmSampleStep;

	// Relative position within either the synth wave or the PCM sampled wave
	// 0 - start of the positive rising sine segment of the square wave or start of the PCM sample
	// 1048576 (2^20) - end of the negative rising sine segment of the square wave
	// For PCM waves, the address of the currently playing sample equals (wavePosition / 256)
	Bit32u wavePosition;

	// Relative position within a square wave phase:
	// 0             - start of the phase
	// 262144 (2^18) - end of a sine phase in the square wave
	Bit32u squareWavePosition;

	// Relative position within the positive or negative wave segment:
	// 0 - start of the corresponding positive or negative segment of the square wave
	// 262144 (2^18) - corresponds to end of the first sine phase in the square wave
	// The same increment sampleStep is used to indicate the current position
	// since the length of the resonance wave is always equal to four square wave sine segments.
	Bit32u resonanceSinePosition;

	// The amp of the resonance sine wave grows with the resonance value
	// As the resonance value cannot change while the partial is active, it is initialised once
	Bit32u resonanceAmpSubtraction;

	// The decay speed of resonance sine wave, depends on the resonance value
	Bit32u resAmpDecayFactor;

	// The relative lengths of the sine, high and low linear segments in terms of SINE_SEGMENT_RELATIVE_LENGTH, depend on cutoff
	Bit32u highLinearLength;
	Bit32u lowLinearLength;
	Bit32u resonanceWaveLengthFactor;

	// The square wave decay level, applied when cutoff is below MIDDLE_CUTOFF_VALUE, depends on cutoff
	Bit32u squareWaveAmpSubtraction;
	// The resonance wave decay level, applied when cutoff is below RESONANCE_DECAY_THRESHOLD_CUTOFF_VALUE, depends on cutoff
	Bit32u resonanceWaveAmpSubtraction;

	// Fractional part of the pcmPosition
	Bit32u pcmInterpolationFactor;

	// Current phase of the square wave
	enum {
		POSITIVE_RISING_SINE_SEGMENT,
		POSITIVE_LINEAR_SEGMENT,
		POSITIVE_FALLING_SINE_SEGMENT,
		NEGATIVE_FALLING_SINE_SEGMENT,
		NEGATIVE_LINEAR_SEGMENT,
		NEGATIVE_RISING_SINE_SEGMENT
	} phase;

	// Current phase of the resonance wave
	enum ResonancePhase {
		POSITIVE_RISING_RESONANCE_SINE_SEGMENT,
		POSITIVE_FALLING_RESONANCE_SINE_SEGMENT,
		NEGATIVE_FALLING_RESONANCE_SINE_SEGMENT,
		NEGATIVE_RISING_RESONANCE_SINE_SEGMENT
	} resonancePhase;

	// Resulting log-space samples of the square and resonance waves
	LogSample squareLogSample;
	LogSample resonanceLogSample;

	// Processed neighbour log-space samples of the PCM wave
	LogSample firstPCMLogSample;
	LogSample secondPCMLogSample;

	//***************************************************************************
	// Internal methods below
	//***************************************************************************

	static Bit32u getResonanceWaveLengthFactor(Bit32u effectiveCutoffValue);
	Bit32u getHighLinearLength(Bit32u effectiveCutoffValue);

	void computePositions();
	void advancePosition();

	void generateNextSquareWaveLogSample();
	void generateNextResonanceWaveLogSample();
	void generateNextSawtoothCosineLogSample(LogSample &logSample) const;

	void pcmSampleToLogSample(LogSample &logSample, const Bit16s pcmSample) const;
	void generateNextPCMWaveLogSamples();

public:
	// Initialise the WG engine for generation of synth partial samples and set up the invariant parameters
	void initSynth(const bool sawtoothWaveform, const Bit8u pulseWidth, const Bit8u resonance);

	// Initialise the WG engine for generation of PCM partial samples and set up the invariant parameters
	void initPCM(const Bit16s * const pcmWaveAddress, const Bit32u pcmWaveLength, const bool pcmWaveLooped, const bool pcmWaveInterpolated);

	// Setters below update WG parameters with respect to TVP, TVA and TVF; should only be called when the values change
	// Logarithmic amp of the wave generator
	void setAmp(const Bit32u amp);
	// Logarithmic frequency of the resulting wave
	void setPitch(const Bit16u pitch);
	// Composed of the base cutoff in range [78..178] left-shifted by 18 bits and the TVF modifier
	void setCutoff(const Bit32u cutoff);

	// Generate the next sample using the current WG parameters
	void generateNextSample();

	// WG output in the log-space consists of two components which are to be added (or ring modulated) in the linear-space afterwards
	LogSample getOutputLogSample(const bool first) const;

	// Deactivate the WG engine
	void deactivate();

	// Return active state of the WG engine
	bool isActive() const;

	// Return true if the WG engine generates PCM wave samples
	bool isPCMWave() const;

	// Return current PCM interpolation factor
	Bit32u getPCMInterpolationFactor() const;
}; // class LA32WaveGenerator

inline LogSample LA32WaveGenerator::getOutputLogSample(const bool first) const {
	if (!isActive()) {
		return SILENCE;
	}
	if (isPCMWave()) {
		return first ? firstPCMLogSample : secondPCMLogSample;
	}
	return first ? squareLogSample : resonanceLogSample;
}

inline void LA32WaveGenerator::deactivate() {
	active = false;
}

inline bool LA32WaveGenerator::isActive() const {
	return active;
}

inline bool LA32WaveGenerator::isPCMWave() const {
	return pcmWaveAddress != NULL;
}

inline Bit32u LA32WaveGenerator::getPCMInterpolationFactor() const {
	return pcmInterpolationFactor;
}

// LA32PartialPair contains a structure of two partials being mixed / ring modulated
class LA32PartialPair {
public:
	enum PairType {
		MASTER,
		SLAVE
	};

	virtual ~LA32PartialPair() {}

	// ringModulated should be set to false for the structures with mixing or stereo output
	// ringModulated should be set to true for the structures with ring modulation
	// mixed is used for the structures with ring modulation and indicates whether the master partial output is mixed to the ring modulator output
	virtual void init(const bool ringModulated, const bool mixed) = 0;

	// Initialise the WG engine for generation of synth partial samples and set up the invariant parameters
	virtual void initSynth(const PairType master, const bool sawtoothWaveform, const Bit8u pulseWidth, const Bit8u resonance) = 0;

	// Initialise the WG engine for generation of PCM partial samples and set up the invariant parameters
	virtual void initPCM(const PairType master, const Bit16s * const pcmWaveAddress, const Bit32u pcmWaveLength, const bool pcmWaveLooped) = 0;

	// Deactivate the WG engine
	virtual void deactivate(const PairType master) = 0;
}; // class LA32PartialPair

class LA32IntPartialPair : public LA32PartialPair {
	LA32WaveGenerator master;
	LA32WaveGenerator slave;
	bool ringModulated;
	bool mixed;

	static Bit16s unlogAndMixWGOutput(const LA32WaveGenerator &wg);

public:
	static void initTables(const Tables &tables);

	// ringModulated should be set to false for the structures with mixing or stereo output
	// ringModulated should be set to true for the structures with ring modulation
	// mixed is used for the structures with ring modulation and indicates whether the master partial output is mixed to the ring modulator output
	void init(const bool ringModulated, const bool mixed);

	// Initialise the WG engine for generation of synth partial samples and set up the invariant parameters
	void initSynth(const PairType master, const bool sawtoothWaveform, const Bit8u pulseWidth, const Bit8u resonance);

	// Initialise the WG engine for generation of PCM partial samples and set up the invariant parameters
	void initPCM(const PairType master, const Bit16s * const pcmWaveAddress, const Bit32u pcmWaveLength, const bool pcmWaveLooped);

	// Update parameters with respect to TVP, TVA and TVF; should only be called when the values change
	void setAmp(const PairType master, const Bit32u amp);
	void setPitch(const PairType master, const Bit16u pitch);
	void setCutoff(const PairType master, const Bit32u cutoff);

	// Generate the next sample using the current WG parameters
	void generateNextSample(const PairType master);
	// Generate the next sample when the caller has already verified that the master is active
	void generateNextActiveSample(const PairType master);

	// Perform mixing / ring modulation of WG output and return the result
	// Although, LA32 applies panning itself, we assume it is applied in the mixer, not within a pair
	Bit16s nextOutSample();

	// Deactivate the WG engine
	void deactivate(const PairType master);

	// Return active state of the WG engine
	bool isActive(const PairType master) const;
}; // class LA32IntPartialPair

inline void LA32IntPartialPair::setAmp(const PairType useMaster, const Bit32u amp) {
	if (useMaster == MASTER) {
		master.setAmp(amp);
	} else {
		slave.setAmp(amp);
	}
}

inline void LA32IntPartialPair::setPitch(const PairType useMaster, const Bit16u pitch) {
	if (useMaster == MASTER) {
		master.setPitch(pitch);
	} else {
		slave.setPitch(pitch);
	}
}

inline void LA32IntPartialPair::setCutoff(const PairType useMaster, const Bit32u cutoff) {
	if (useMaster == MASTER) {
		master.setCutoff(cutoff);
	} else {
		slave.setCutoff(cutoff);
	}
}

inline void LA32IntPartialPair::generateNextSample(const PairType useMaster) {
	if (useMaster == MASTER) {
		master.generateNextSample();
	} else {
		slave.generateNextSample();
	}
}

inline void LA32IntPartialPair::generateNextActiveSample(const PairType useMaster) {
	generateNextSample(useMaster);
}

inline Bit16s LA32IntPartialPair::unlogAndMixWGOutput(const LA32WaveGenerator &wg) {
	if (!wg.isActive()) {
		return 0;
	}
	Bit16s firstSample = LA32Utilites::unlog(wg.getOutputLogSample(true));
	Bit16s secondSample = LA32Utilites::unlog(wg.getOutputLogSample(false));
	if (wg.isPCMWave()) {
		return Bit16s(firstSample + (((Bit32s(secondSample) - Bit32s(firstSample)) * wg.getPCMInterpolationFactor()) >> 7));
	}
	return firstSample + secondSample;
}

static inline Bit16s produceDistortedIntSample(Bit16s sample) {
	return ((sample & 0x2000) == 0) ? Bit16s(sample & 0x1fff) : Bit16s(sample | ~0x1fff);
}

inline Bit16s LA32IntPartialPair::nextOutSample() {
	if (!ringModulated) {
		return unlogAndMixWGOutput(master) + unlogAndMixWGOutput(slave);
	}

	Bit16s masterSample = unlogAndMixWGOutput(master); // Store master partial sample for further mixing

	/* SEMI-CONFIRMED from sample analysis:
	 * We observe that for partial structures with ring modulation the interpolation is not applied to the slave PCM partial.
	 * It's assumed that the multiplication circuitry intended to perform the interpolation on the slave PCM partial
	 * is borrowed by the ring modulation circuit (or the LA32 chip has a similar lack of resources assigned to each partial pair).
	 */
	Bit16s slaveSample = slave.isPCMWave() ? LA32Utilites::unlog(slave.getOutputLogSample(true)) : unlogAndMixWGOutput(slave);

	/* SEMI-CONFIRMED: Ring modulation model derived from sample analysis of specially constructed patches which exploit distortion.
	 * LA32 ring modulator found to produce distorted output in case if the absolute value of maximal amplitude of one of the input partials exceeds 8191.
	 * This is easy to reproduce using synth partials with resonance values close to the maximum. It looks like an integer overflow happens in this case.
	 * As the distortion is strictly bound to the amplitude of the complete mixed square + resonance wave in the linear space,
	 * it is reasonable to assume the ring modulation is performed also in the linear space by sample multiplication.
	 * Most probably the overflow is caused by limited precision of the multiplication circuit as the very similar distortion occurs with panning.
	 */
	Bit16s ringModulatedSample = Bit16s((Bit32s(produceDistortedIntSample(masterSample)) * Bit32s(produceDistortedIntSample(slaveSample))) >> 13);

	return mixed ? masterSample + ringModulatedSample : ringModulatedSample;
}

inline void LA32IntPartialPair::deactivate(const PairType useMaster) {
	if (useMaster == MASTER) {
		master.deactivate();
	} else {
		slave.deactivate();
	}
}

inline bool LA32IntPartialPair::isActive(const PairType useMaster) const {
	return useMaster == MASTER ? master.isActive() : slave.isActive();
}

} // namespace MT32Emu

#endif // #ifndef MT32EMU_LA32_WAVE_GENERATOR_H
