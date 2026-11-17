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

#ifndef MT32EMU_LA32_FLOAT_WAVE_GENERATOR_H
#define MT32EMU_LA32_FLOAT_WAVE_GENERATOR_H

#include "globals.h"
#include "internals.h"
#include "Types.h"
#include "LA32WaveGenerator.h"
#include "LA32FloatMath.h"

namespace MT32Emu {

#ifndef MT32EMU_FASTER_MATH
#define MT32EMU_FASTER_MATH 1
#endif

#if MT32EMU_FASTER_MATH == 0

typedef LA32FloatMath::VanillaMath LA32FloatMathImpl;

#elif MT32EMU_FASTER_MATH == 1

// In this mode we strive to minimise the number of operations on the critical path at the expense of increased memory usage,
// hence it may well not be the fastest option under memory pressure. The LUT sizes are picked so to achieve a good accuracy with
// the SNR about 126dB. Albeit, we don't need the exponent table that big with the linear interpolation: 10 bits would be enough.
// We pick the table size this big to simplify fexp2i a lot, leading to a considerable speedup.
typedef LA32FloatMath::FasterMath<LA32FloatMath::LinearInterpolator, 12, 12> LA32FloatMathImpl;

#elif MT32EMU_FASTER_MATH == 2

// The quadratic interpolation allows to reduce the memory needed for the LUTs significantly, but also slightly increases the code
// complexity. The same level of accuracy is expected as with the option above.
typedef LA32FloatMath::FasterMath<LA32FloatMath::QuadraticInterpolator, 6, 9> LA32FloatMathImpl;

#else

#error "Wrong MT32EMU_FASTER_MATH"

#endif

extern const Bit8u *resAmpDecayFactors;
extern float pcmSampleTable[65536];

/**
 * LA32WaveGenerator is aimed to represent the exact model of LA32 wave generator.
 * The output square wave is created by adding high / low linear segments in-between
 * the rising and falling cosine segments. Basically, it's very similar to the phase distortion synthesis.
 * Behaviour of a true resonance filter is emulated by adding decaying sine wave.
 * The beginning and the ending of the resonant sine is multiplied by a cosine window.
 * To synthesise sawtooth waves, the resulting square wave is multiplied by synchronous cosine wave.
 */
class LA32FloatWaveGenerator {
	static const float RESONANCE_DECAY_THRESHOLD_CUTOFF_VALUE;

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

	float wavePos;
	float pcmPosition;
	float pcmPositionDelta;

	float resAmpBase;
	float resAmp;

	float ampFactor;
	float freq;

	// Scaled cutoff value to simplify calculations
	float cutoff;

	// Wave length in samples
	float waveLen;
	// Cosine segment length in samples
	float cosineLen;
	// Ratio of positive segment to wave length
	float pulseLenFactor;

	// Cached segment boundaries, recomputed only when waveform parameters change
	float halfCosineLen;
	float hLen;
	float highSegmentEnd;
	float secondCosineEnd;
	float resNegativeSegmentStart;
	float resPositiveSegmentStart;

	// For cutoff below 50, samples get attenuated exponentially, found by sample analysis
	float lowCutoffAttenuationFactor;

	void updateWaveFormParameters();
	float getPCMSample(unsigned int position);

public:
	// Initialise the WG engine for generation of synth partial samples and set up the invariant parameters
	void initSynth(const bool sawtoothWaveform, const Bit8u pulseWidth, const Bit8u resonance);

	// Initialise the WG engine for generation of PCM partial samples and set up the invariant parameters
	void initPCM(const Bit16s * const pcmWaveAddress, const Bit32u pcmWaveLength, const bool pcmWaveLooped, const bool pcmWaveInterpolated);

	// Update parameters with respect to TVP, TVA and TVF; should only be called when the values change
	// Logarithmic amp of the wave generator
	void setAmp(const Bit32u amp);
	// Logarithmic frequency of the resulting wave
	void setPitch(const Bit16u pitch);
	// Composed of the base cutoff in range [78..178] left-shifted by 18 bits and the TVF modifier
	void setCutoff(const Bit32u cutoff);

	// Generate the next sample using the current WG parameters
	float generateNextSample();
	float generateNextSampleUnchecked();

	// Deactivate the WG engine
	void deactivate();

	// Return active state of the WG engine
	bool isActive() const;

	// Return true if the WG engine generates PCM wave samples
	bool isPCMWave() const;
}; // class LA32FloatWaveGenerator

inline void LA32FloatWaveGenerator::updateWaveFormParameters() {
	cosineLen = 0.5f * waveLen;
	if (cutoff > MIDDLE_CUTOFF_VALUE) {
		cosineLen *= LA32FloatMathImpl::cosineLengthFactor(cutoff);
	}
	halfCosineLen = 0.5f * cosineLen;
	hLen = pulseLenFactor * waveLen - cosineLen;
	if (hLen < 0.0f) {
		hLen = 0.0f;
	}
	highSegmentEnd = cosineLen + hLen;
	secondCosineEnd = 2 * cosineLen + hLen;
	resNegativeSegmentStart = waveLen - halfCosineLen;
	resPositiveSegmentStart = hLen + halfCosineLen;
}

inline float LA32FloatWaveGenerator::getPCMSample(unsigned int position) {
	if (position >= pcmWaveLength) {
		if (!pcmWaveLooped) {
			return 0;
		}
		position = position % pcmWaveLength;
	}
	return pcmSampleTable[Bit16u(pcmWaveAddress[position])];
}

inline void LA32FloatWaveGenerator::setAmp(const Bit32u amp) {
	ampFactor = LA32FloatMathImpl::linearAmpFactor(amp);
}

inline void LA32FloatWaveGenerator::setPitch(const Bit16u pitch) {
	float lastFreq = freq;
	freq = LA32FloatMathImpl::frequency(pitch);

	if (isPCMWave()) {
		pcmPositionDelta = freq * 2048.0f / SAMPLE_RATE;
	} else {
		waveLen = SAMPLE_RATE / freq;
		wavePos *= lastFreq / freq;
		updateWaveFormParameters();
	}
}

inline void LA32FloatWaveGenerator::setCutoff(const Bit32u cutoffVal) {
	cutoff = cutoffVal / 262144.0f;

	// Correct resAmp for cutoff in range 50..66
	if ((cutoff >= MIDDLE_CUTOFF_VALUE) && (cutoff < RESONANCE_DECAY_THRESHOLD_CUTOFF_VALUE)) {
		resAmp = resAmpBase * LA32FloatMathImpl::fsin((cutoff - MIDDLE_CUTOFF_VALUE) / 32.0f);
	} else {
		resAmp = resAmpBase;
	}
	lowCutoffAttenuationFactor = cutoff < MIDDLE_CUTOFF_VALUE ? LA32FloatMathImpl::attenuationFactorBelowMiddleCutoff(cutoff) : 1;
	updateWaveFormParameters();
}

inline float LA32FloatWaveGenerator::generateNextSample() {
	if (!active) {
		return 0.0f;
	}
	return generateNextSampleUnchecked();
}

inline float LA32FloatWaveGenerator::generateNextSampleUnchecked() {
	float sample = 0.0f;

	if (isPCMWave()) {
		// Render PCM waveform
		int len = pcmWaveLength;
		int intPCMPosition = int(pcmPosition);
		if (intPCMPosition >= len && !pcmWaveLooped) {
			// We're now past the end of a non-looping PCM waveform so it's time to die.
			deactivate();
			return 0.0f;
		}

		// Linear interpolation
		float firstSample = getPCMSample(intPCMPosition);
		// We observe that for partial structures with ring modulation the interpolation is not applied to the slave PCM partial.
		// It's assumed that the multiplication circuitry intended to perform the interpolation on the slave PCM partial
		// is borrowed by the ring modulation circuit (or the LA32 chip has a similar lack of resources assigned to each partial pair).
		if (pcmWaveInterpolated) {
			sample = firstSample + (getPCMSample(intPCMPosition + 1) - firstSample) * (pcmPosition - intPCMPosition);
		} else {
			sample = firstSample;
		}

		float newPCMPosition = pcmPosition + pcmPositionDelta;
		if (pcmWaveLooped) {
			if (newPCMPosition >= float(pcmWaveLength)) {
				newPCMPosition = fmod(newPCMPosition, float(pcmWaveLength));
			}
		}
		pcmPosition = newPCMPosition;
	} else {
		// Render synthesised waveform

		// Start playing in center of first cosine segment
		// relWavePos is shifted by a half of cosineLen
		float relWavePos = wavePos + halfCosineLen;
		if (relWavePos > waveLen) {
			relWavePos -= waveLen;
		}

		// Produce filtered square wave with 2 cosine waves on slopes
		float squareWaveSample;

		// 1st cosine segment
		if (relWavePos < cosineLen) {
			squareWaveSample = -LA32FloatMathImpl::fcos(relWavePos / cosineLen);
		} else

		// high linear segment
		if (relWavePos < highSegmentEnd) {
			squareWaveSample = 1.f;
		} else

		// 2nd cosine segment
		if (relWavePos < secondCosineEnd) {
			squareWaveSample = LA32FloatMathImpl::fcos((relWavePos - highSegmentEnd) / cosineLen);
		} else {

		// low linear segment
			squareWaveSample = -1.f;
		}

		sample = squareWaveSample;

		if (cutoff < MIDDLE_CUTOFF_VALUE) {
			sample *= lowCutoffAttenuationFactor;
		} else {

			// Add resonance sine. Effective for cutoff > 50 only
			float resSample = 1.0f;

			// Resonance decay speed factor
			float resAmpDecayFactor = resAmpDecayFactors[resonance >> 2];

			// Now relWavePos counts from the middle of first cosine
			relWavePos = wavePos;

			// negative segments
			if (!(relWavePos < highSegmentEnd)) {
				resSample = -resSample;
				relWavePos -= highSegmentEnd;

				// From the digital captures, the decaying speed of the resonance sine is found a bit different for the positive and the negative segments
				resAmpDecayFactor += 0.25f;
			}

			// Resonance sine WG
			resSample *= LA32FloatMathImpl::fsin(relWavePos / cosineLen);

			// Resonance sine amp
			float resAmpFadeLog2 = -0.125f * resAmpDecayFactor * (relWavePos / cosineLen); // seems to be exact
			float resAmpFade = LA32FloatMathImpl::unlogResonanceAmpFade(resAmpFadeLog2);

			// Now relWavePos set negative to the left from center of any cosine
			relWavePos = wavePos;

			// negative segment
			if (!(wavePos < resNegativeSegmentStart)) {
				relWavePos -= waveLen;
			} else

			// positive segment
			if (!(wavePos < resPositiveSegmentStart)) {
				relWavePos -= highSegmentEnd;
			}

			// To ensure the output wave has no breaks, two different windows are applied to the beginning and the ending of the resonance sine segment
			if (relWavePos < halfCosineLen) {
				if (relWavePos < 0.0f) {
					// The window is synchronous square sine here
					resAmpFade *= squareWaveSample * squareWaveSample;
				} else {
					// The window is synchronous sine here
					resAmpFade *= squareWaveSample < 0 ? -squareWaveSample : squareWaveSample;
				}
			}

			sample += resSample * resAmp * resAmpFade;
		}

		// sawtooth waves
		if (sawtoothWaveform) {
			sample *= LA32FloatMathImpl::fcos(2 * wavePos / waveLen);
		}

		wavePos++;

		// wavePos isn't supposed to be > waveLen
		if (wavePos > waveLen) {
			wavePos -= waveLen;
		}
	}

	// Multiply sample with current TVA value
	sample *= ampFactor;
	return sample;
}

inline void LA32FloatWaveGenerator::deactivate() {
	active = false;
}

inline bool LA32FloatWaveGenerator::isActive() const {
	return active;
}

inline bool LA32FloatWaveGenerator::isPCMWave() const {
	return pcmWaveAddress != NULL;
}

class LA32FloatPartialPair : public LA32PartialPair {
	LA32FloatWaveGenerator master;
	LA32FloatWaveGenerator slave;
	bool ringModulated;
	bool mixed;
	float masterOutputSample;
	float slaveOutputSample;

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

	// Perform mixing / ring modulation and return the result
	float nextOutSample();

	// Deactivate the WG engine
	void deactivate(const PairType master);

	// Return active state of the WG engine
	bool isActive(const PairType master) const;
}; // class LA32FloatPartialPair

inline void LA32FloatPartialPair::setAmp(const PairType useMaster, const Bit32u amp) {
	if (useMaster == MASTER) {
		master.setAmp(amp);
	} else {
		slave.setAmp(amp);
	}
}

inline void LA32FloatPartialPair::setPitch(const PairType useMaster, const Bit16u pitch) {
	if (useMaster == MASTER) {
		master.setPitch(pitch);
	} else {
		slave.setPitch(pitch);
	}
}

inline void LA32FloatPartialPair::setCutoff(const PairType useMaster, const Bit32u cutoff) {
	if (useMaster == MASTER) {
		master.setCutoff(cutoff);
	} else {
		slave.setCutoff(cutoff);
	}
}

inline void LA32FloatPartialPair::generateNextSample(const PairType useMaster) {
	if (useMaster == MASTER) {
		masterOutputSample = master.generateNextSample();
	} else {
		slaveOutputSample = slave.generateNextSample();
	}
}

inline void LA32FloatPartialPair::generateNextActiveSample(const PairType useMaster) {
	if (useMaster == MASTER) {
		masterOutputSample = master.generateNextSampleUnchecked();
	} else {
		slaveOutputSample = slave.generateNextSample();
	}
}

inline float produceDistortedLA32Sample(float sample) {
	if (sample < -1.0f) {
		return sample + 2.0f;
	} else if (1.0f < sample) {
		return sample - 2.0f;
	}
	return sample;
}

inline float LA32FloatPartialPair::nextOutSample() {
	// Note, LA32FloatWaveGenerator produces each sample normalised in terms of a single playing partial,
	// so the unity sample corresponds to the internal LA32 logarithmic fixed-point unity sample.
	// However, each logarithmic sample is then unlogged to a 14-bit signed integer value, i.e. the max absolute value is 8192.
	// Thus, considering that samples are further mapped to a 16-bit signed integer,
	// we apply a conversion factor 0.25 to produce properly normalised float samples.
	if (!ringModulated) {
		return 0.25f * (masterOutputSample + slaveOutputSample);
	}
	/*
	 * SEMI-CONFIRMED: Ring modulation model derived from sample analysis of specially constructed patches which exploit distortion.
	 * LA32 ring modulator found to produce distorted output in case if the absolute value of maximal amplitude of one of the input partials exceeds 8191.
	 * This is easy to reproduce using synth partials with resonance values close to the maximum. It looks like an integer overflow happens in this case.
	 * As the distortion is strictly bound to the amplitude of the complete mixed square + resonance wave in the linear space,
	 * it is reasonable to assume the ring modulation is performed also in the linear space by sample multiplication.
	 * Most probably the overflow is caused by limited precision of the multiplication circuit as the very similar distortion occurs with panning.
	 */
	float ringModulatedSample = produceDistortedLA32Sample(masterOutputSample) * produceDistortedLA32Sample(slaveOutputSample);
	return 0.25f * (mixed ? masterOutputSample + ringModulatedSample : ringModulatedSample);
}

inline void LA32FloatPartialPair::deactivate(const PairType useMaster) {
	if (useMaster == MASTER) {
		master.deactivate();
		masterOutputSample = 0.0f;
	} else {
		slave.deactivate();
		slaveOutputSample = 0.0f;
	}
}

inline bool LA32FloatPartialPair::isActive(const PairType useMaster) const {
	return useMaster == MASTER ? master.isActive() : slave.isActive();
}

} // namespace MT32Emu

#endif // #ifndef MT32EMU_LA32_FLOAT_WAVE_GENERATOR_H
