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

#include <cstddef>

#include "internals.h"

#include "LA32WaveGenerator.h"
#include "Tables.h"

namespace MT32Emu {

static const Bit32u SINE_SEGMENT_RELATIVE_LENGTH = 1 << 18;
static const Bit32u MIDDLE_CUTOFF_VALUE = 128 << 18;
static const Bit32u RESONANCE_DECAY_THRESHOLD_CUTOFF_VALUE = 144 << 18;

const Bit16u *exp9;
const Bit16u *logsin9;
const Bit8u *intResAmpDecayFactors;
Bit16u expInterpolationTable[4096];

Bit32u LA32WaveGenerator::getResonanceWaveLengthFactor(Bit32u effectiveCutoffValue) {
	// resonanceWaveLengthFactor = (Bit32u)EXP2F(12.0f + effectiveCutoffValue / 4096.0f);
	Bit32u factor = LA32Utilites::interpolateExp(~effectiveCutoffValue & 4095);
	factor <<= effectiveCutoffValue >> 12;
	return factor;
}

Bit32u LA32WaveGenerator::getHighLinearLength(Bit32u effectiveCutoffValue) {
	// Ratio of positive segment to wave length
	Bit32u effectivePulseWidthValue = 0;
	if (pulseWidth > 128) {
		effectivePulseWidthValue = (pulseWidth - 128) << 6;
	}

	// highLinearLength = EXP2F(19.0f - effectivePulseWidthValue / 4096.0f + effectiveCutoffValue / 4096.0f) - 2 * SINE_SEGMENT_RELATIVE_LENGTH;
	if (effectivePulseWidthValue < effectiveCutoffValue) {
		Bit32u expArg = effectiveCutoffValue - effectivePulseWidthValue;
		Bit32u length = LA32Utilites::interpolateExp(~expArg & 4095);
		length <<= 7 + (expArg >> 12);
		length -= 2 * SINE_SEGMENT_RELATIVE_LENGTH;
		return length;
	}
	return 0;
}

void LA32WaveGenerator::computePositions() {
	// Assuming 12-bit multiplication used here
	squareWavePosition = resonanceSinePosition = (wavePosition >> 8) * (resonanceWaveLengthFactor >> 4);
	if (squareWavePosition < SINE_SEGMENT_RELATIVE_LENGTH) {
		phase = POSITIVE_RISING_SINE_SEGMENT;
		return;
	}
	squareWavePosition -= SINE_SEGMENT_RELATIVE_LENGTH;
	if (squareWavePosition < highLinearLength) {
		phase = POSITIVE_LINEAR_SEGMENT;
		return;
	}
	squareWavePosition -= highLinearLength;
	if (squareWavePosition < SINE_SEGMENT_RELATIVE_LENGTH) {
		phase = POSITIVE_FALLING_SINE_SEGMENT;
		return;
	}
	squareWavePosition -= SINE_SEGMENT_RELATIVE_LENGTH;
	resonanceSinePosition = squareWavePosition;
	if (squareWavePosition < SINE_SEGMENT_RELATIVE_LENGTH) {
		phase = NEGATIVE_FALLING_SINE_SEGMENT;
		return;
	}
	squareWavePosition -= SINE_SEGMENT_RELATIVE_LENGTH;
	if (squareWavePosition < lowLinearLength) {
		phase = NEGATIVE_LINEAR_SEGMENT;
		return;
	}
	squareWavePosition -= lowLinearLength;
	phase = NEGATIVE_RISING_SINE_SEGMENT;
}

void LA32WaveGenerator::advancePosition() {
	wavePosition += sampleStep;
	wavePosition %= 4 * SINE_SEGMENT_RELATIVE_LENGTH;
	computePositions();
	resonancePhase = ResonancePhase(((resonanceSinePosition >> 18) + (phase > POSITIVE_FALLING_SINE_SEGMENT ? 2 : 0)) & 3);
}

void LA32WaveGenerator::generateNextSquareWaveLogSample() {
	Bit32u logSampleValue;
	switch (phase) {
		case POSITIVE_RISING_SINE_SEGMENT:
		case NEGATIVE_FALLING_SINE_SEGMENT:
			logSampleValue = logsin9[(squareWavePosition >> 9) & 511];
			break;
		case POSITIVE_FALLING_SINE_SEGMENT:
		case NEGATIVE_RISING_SINE_SEGMENT:
			logSampleValue = logsin9[~(squareWavePosition >> 9) & 511];
			break;
		case POSITIVE_LINEAR_SEGMENT:
		case NEGATIVE_LINEAR_SEGMENT:
		default:
			logSampleValue = 0;
			break;
	}
	logSampleValue <<= 2;
	logSampleValue += amp + squareWaveAmpSubtraction;

	squareLogSample.logValue = logSampleValue < 65536 ? Bit16u(logSampleValue) : 65535;
	squareLogSample.sign = phase < NEGATIVE_FALLING_SINE_SEGMENT ? LogSample::POSITIVE : LogSample::NEGATIVE;
}

void LA32WaveGenerator::generateNextResonanceWaveLogSample() {
	Bit32u logSampleValue;
	if (resonancePhase == POSITIVE_FALLING_RESONANCE_SINE_SEGMENT || resonancePhase == NEGATIVE_RISING_RESONANCE_SINE_SEGMENT) {
		logSampleValue = logsin9[~(resonanceSinePosition >> 9) & 511];
	} else {
		logSampleValue = logsin9[(resonanceSinePosition >> 9) & 511];
	}
	logSampleValue <<= 2;
	logSampleValue += amp;

	// From the digital captures, the decaying speed of the resonance sine is found a bit different for the positive and the negative segments
	Bit32u decayFactor = phase < NEGATIVE_FALLING_SINE_SEGMENT ? resAmpDecayFactor : resAmpDecayFactor + 1;
	// Unsure about resonanceSinePosition here. It's possible that dedicated counter & decrement are used. Although, cutoff is finely ramped, so maybe not.
	logSampleValue += resonanceAmpSubtraction + (((resonanceSinePosition >> 4) * decayFactor) >> 8);

	// To ensure the output wave has no breaks, two different windows are applied to the beginning and the ending of the resonance sine segment
	if (phase == POSITIVE_RISING_SINE_SEGMENT || phase == NEGATIVE_FALLING_SINE_SEGMENT) {
		// The window is synchronous sine here
		logSampleValue += logsin9[(squareWavePosition >> 9) & 511] << 2;
	} else if (phase == POSITIVE_FALLING_SINE_SEGMENT || phase == NEGATIVE_RISING_SINE_SEGMENT) {
		// The window is synchronous square sine here
		logSampleValue += logsin9[~(squareWavePosition >> 9) & 511] << 3;
	}

	logSampleValue += resonanceWaveAmpSubtraction;

	// After all the amp decrements are added, it should be safe now to adjust the amp of the resonance wave to what we see on captures
	logSampleValue -= 1 << 12;

	resonanceLogSample.logValue = logSampleValue < 65536 ? Bit16u(logSampleValue) : 65535;
	resonanceLogSample.sign = resonancePhase < NEGATIVE_FALLING_RESONANCE_SINE_SEGMENT ? LogSample::POSITIVE : LogSample::NEGATIVE;
}

void LA32WaveGenerator::generateNextSawtoothCosineLogSample(LogSample &logSample) const {
	Bit32u sawtoothCosinePosition = wavePosition + (1 << 18);
	if ((sawtoothCosinePosition & (1 << 18)) > 0) {
		logSample.logValue = logsin9[~(sawtoothCosinePosition >> 9) & 511];
	} else {
		logSample.logValue = logsin9[(sawtoothCosinePosition >> 9) & 511];
	}
	logSample.logValue <<= 2;
	logSample.sign = ((sawtoothCosinePosition & (1 << 19)) == 0) ? LogSample::POSITIVE : LogSample::NEGATIVE;
}

void LA32WaveGenerator::pcmSampleToLogSample(LogSample &logSample, const Bit16s pcmSample) const {
	Bit32u logSampleValue = (32787 - (pcmSample & 32767)) << 1;
	logSampleValue += amp;
	logSample.logValue = logSampleValue < 65536 ? Bit16u(logSampleValue) : 65535;
	logSample.sign = pcmSample < 0 ? LogSample::NEGATIVE : LogSample::POSITIVE;
}

void LA32WaveGenerator::generateNextPCMWaveLogSamples() {
	// This should emulate the ladder we see in the PCM captures for pitches 01, 02, 07, etc.
	// The most probable cause is the factor in the interpolation formula is one bit less
	// accurate than the sample position counter
	pcmInterpolationFactor = (wavePosition & 255) >> 1;
	Bit32u pcmWaveTableIx = wavePosition >> 8;
	pcmSampleToLogSample(firstPCMLogSample, pcmWaveAddress[pcmWaveTableIx]);
	if (pcmWaveInterpolated) {
		pcmWaveTableIx++;
		if (pcmWaveTableIx < pcmWaveLength) {
			pcmSampleToLogSample(secondPCMLogSample, pcmWaveAddress[pcmWaveTableIx]);
		} else {
			if (pcmWaveLooped) {
				pcmWaveTableIx -= pcmWaveLength;
				pcmSampleToLogSample(secondPCMLogSample, pcmWaveAddress[pcmWaveTableIx]);
			} else {
				secondPCMLogSample = SILENCE;
			}
		}
	} else {
		secondPCMLogSample = SILENCE;
	}
	wavePosition += pcmSampleStep;
	if (wavePosition >= (pcmWaveLength << 8)) {
		if (pcmWaveLooped) {
			wavePosition -= pcmWaveLength << 8;
		} else {
			deactivate();
		}
	}
}

void LA32WaveGenerator::initSynth(const bool useSawtoothWaveform, const Bit8u usePulseWidth, const Bit8u useResonance) {
	sawtoothWaveform = useSawtoothWaveform;
	pulseWidth = usePulseWidth;
	resonance = useResonance;

	wavePosition = 0;

	squareWavePosition = 0;
	phase = POSITIVE_RISING_SINE_SEGMENT;

	resonanceSinePosition = 0;
	resonancePhase = POSITIVE_RISING_RESONANCE_SINE_SEGMENT;
	resonanceAmpSubtraction = (32 - resonance) << 10;
	resAmpDecayFactor = intResAmpDecayFactors[resonance >> 2] << 2;

	setAmp(0);
	setPitch(0);
	setCutoff(0);

	pcmWaveAddress = NULL;
	active = true;
}

void LA32WaveGenerator::initPCM(const Bit16s * const usePCMWaveAddress, const Bit32u usePCMWaveLength, const bool usePCMWaveLooped, const bool usePCMWaveInterpolated) {
	pcmWaveAddress = usePCMWaveAddress;
	pcmWaveLength = usePCMWaveLength;
	pcmWaveLooped = usePCMWaveLooped;
	pcmWaveInterpolated = usePCMWaveInterpolated;

	setAmp(0);
	setPitch(0);

	wavePosition = 0;
	active = true;
}

void LA32WaveGenerator::setAmp(const Bit32u useAmp) {
	amp = useAmp >> 10;
}

void LA32WaveGenerator::setPitch(const Bit16u pitch) {
	// sampleStep = EXP2F(pitch / 4096.0f + 4.0f)
	// pcmSampleStep = EXP2F(pitch / 4096.0f + 3.0f);
	Bit32u newStep = LA32Utilites::interpolateExp(~pitch & 4095);
	newStep <<= pitch >> 12;
	if (isPCMWave()) {
		// Seeing the actual lengths of the PCM wave for pitches 00..12,
		// the wavePosition counter can be assumed to have 8-bit fractions
		pcmSampleStep = newStep >> 9;
	} else {
		sampleStep = (newStep >> 8) & ~1;
	}
}

void LA32WaveGenerator::setCutoff(const Bit32u cutoffVal) {
	Bit32u effectiveCutoffValue = (cutoffVal > MIDDLE_CUTOFF_VALUE) ? (cutoffVal - MIDDLE_CUTOFF_VALUE) >> 10 : 0;
	resonanceWaveLengthFactor = getResonanceWaveLengthFactor(effectiveCutoffValue);
	highLinearLength = getHighLinearLength(effectiveCutoffValue);
	lowLinearLength = (resonanceWaveLengthFactor << 8) - 4 * SINE_SEGMENT_RELATIVE_LENGTH - highLinearLength;

	if (cutoffVal < MIDDLE_CUTOFF_VALUE) {
		squareWaveAmpSubtraction = (MIDDLE_CUTOFF_VALUE - cutoffVal) >> 9;
		// For the cutoff values below the cutoff middle point, it seems the amp of the resonance wave is exponentially decayed
		resonanceWaveAmpSubtraction = 31743 + ((MIDDLE_CUTOFF_VALUE - cutoffVal) >> 9);
	} else if (cutoffVal < RESONANCE_DECAY_THRESHOLD_CUTOFF_VALUE) {
		squareWaveAmpSubtraction = 0;
		// For the cutoff values below this point, the amp of the resonance wave is sinusoidally decayed
		Bit32u sineIx = (cutoffVal - MIDDLE_CUTOFF_VALUE) >> 13;
		resonanceWaveAmpSubtraction = logsin9[sineIx] << 2;
	} else {
		squareWaveAmpSubtraction = 0;
		resonanceWaveAmpSubtraction = 0;
	}
}

void LA32WaveGenerator::generateNextSample() {
	if (!active) {
		return;
	}

	if (isPCMWave()) {
		generateNextPCMWaveLogSamples();
		return;
	}

	generateNextSquareWaveLogSample();
	generateNextResonanceWaveLogSample();
	if (sawtoothWaveform) {
		LogSample cosineLogSample;
		generateNextSawtoothCosineLogSample(cosineLogSample);
		LA32Utilites::addLogSamples(squareLogSample, cosineLogSample);
		LA32Utilites::addLogSamples(resonanceLogSample, cosineLogSample);
	}
	advancePosition();
}

void LA32IntPartialPair::initTables(const Tables &tables) {
	exp9 = tables.exp9;
	logsin9 = tables.logsin9;
	intResAmpDecayFactors = tables.resAmpDecayFactors;
	for (Bit16u i = 0; i < 4096; i++) {
		Bit16u expTabIndex = i >> 3;
		Bit16u extraBits = ~i & 7;
		Bit16u expTabEntry2 = 8191 - exp9[expTabIndex];
		Bit16u expTabEntry1 = expTabIndex == 0 ? 8191 : (8191 - exp9[expTabIndex - 1]);
		expInterpolationTable[i] = expTabEntry2 + (((expTabEntry1 - expTabEntry2) * extraBits) >> 3);
	}
}

void LA32IntPartialPair::init(const bool useRingModulated, const bool useMixed) {
	ringModulated = useRingModulated;
	mixed = useMixed;
}

void LA32IntPartialPair::initSynth(const PairType useMaster, const bool sawtoothWaveform, const Bit8u pulseWidth, const Bit8u resonance) {
	if (useMaster == MASTER) {
		master.initSynth(sawtoothWaveform, pulseWidth, resonance);
	} else {
		slave.initSynth(sawtoothWaveform, pulseWidth, resonance);
	}
}

void LA32IntPartialPair::initPCM(const PairType useMaster, const Bit16s *pcmWaveAddress, const Bit32u pcmWaveLength, const bool pcmWaveLooped) {
	if (useMaster == MASTER) {
		master.initPCM(pcmWaveAddress, pcmWaveLength, pcmWaveLooped, true);
	} else {
		slave.initPCM(pcmWaveAddress, pcmWaveLength, pcmWaveLooped, !ringModulated);
	}
}

} // namespace MT32Emu
