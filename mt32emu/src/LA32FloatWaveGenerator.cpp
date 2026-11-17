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

#include "LA32FloatWaveGenerator.h"

#include "LA32FloatMath.h"
#include "mmath.h"
#include "Tables.h"

namespace MT32Emu {

const float LA32FloatWaveGenerator::RESONANCE_DECAY_THRESHOLD_CUTOFF_VALUE = 144.0f;
const Bit8u *resAmpDecayFactors;
float pcmSampleTable[65536];

void LA32FloatWaveGenerator::initSynth(const bool useSawtoothWaveform, const Bit8u usePulseWidth, const Bit8u useResonance) {
	sawtoothWaveform = useSawtoothWaveform;
	pulseWidth = usePulseWidth;
	resonance = useResonance;

	resAmpBase = LA32FloatMathImpl::resonanceAmpBase(resonance);
	resAmp = resAmpBase;

	pulseLenFactor = (pulseWidth > 128) ? LA32FloatMathImpl::pulseLengthFactor(pulseWidth) : 0.5f;

	cutoff = 0.0f;
	freq = 0.0f;
	waveLen = 0.0f;
	wavePos = 0.0f;
	updateWaveFormParameters();

	pcmWaveAddress = NULL;
	active = true;
}

void LA32FloatWaveGenerator::initPCM(const Bit16s * const usePCMWaveAddress, const Bit32u usePCMWaveLength, const bool usePCMWaveLooped, const bool usePCMWaveInterpolated) {
	pcmWaveAddress = usePCMWaveAddress;
	pcmWaveLength = usePCMWaveLength;
	pcmWaveLooped = usePCMWaveLooped;
	pcmWaveInterpolated = usePCMWaveInterpolated;

	pcmPosition = 0.0f;
	active = true;
}

void LA32FloatPartialPair::initTables(const Tables &tables) {
	resAmpDecayFactors = tables.resAmpDecayFactors;
	LA32FloatMathImpl::init();
	for (Bit32u i = 0; i < 65536; i++) {
		float sampleValue = LA32FloatMathImpl::unlogPCMSample(i & 32767);
		pcmSampleTable[i] = (i & 32768) == 0 ? sampleValue : -sampleValue;
	}
}

void LA32FloatPartialPair::init(const bool useRingModulated, const bool useMixed) {
	ringModulated = useRingModulated;
	mixed = useMixed;
	masterOutputSample = 0.0f;
	slaveOutputSample = 0.0f;
}

void LA32FloatPartialPair::initSynth(const PairType useMaster, const bool sawtoothWaveform, const Bit8u pulseWidth, const Bit8u resonance) {
	if (useMaster == MASTER) {
		master.initSynth(sawtoothWaveform, pulseWidth, resonance);
	} else {
		slave.initSynth(sawtoothWaveform, pulseWidth, resonance);
	}
}

void LA32FloatPartialPair::initPCM(const PairType useMaster, const Bit16s *pcmWaveAddress, const Bit32u pcmWaveLength, const bool pcmWaveLooped) {
	if (useMaster == MASTER) {
		master.initPCM(pcmWaveAddress, pcmWaveLength, pcmWaveLooped, true);
	} else {
		slave.initPCM(pcmWaveAddress, pcmWaveLength, pcmWaveLooped, !ringModulated);
	}
}

} // namespace MT32Emu
