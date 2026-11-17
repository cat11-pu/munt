/* Copyright (C) 2026 Dean Beeler, Jerome Fisher, Sergey V. Mikayev
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

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../File.h"
#include "../ROMInfo.h"
#include "../Synth.h"
#include "../Structures.h"

#include "FakeROMs.h"
#include "TestAccessors.h"

namespace {

static const MT32Emu::Bit8u PATCH_SYSEX[] = {
	0x00, 0x00, 0x00,
	0x00, 0x00, 0x18, 0x19, 0x0c, 0x00, 0x00, 0x00,
	0x64, 0x00
};

static const MT32Emu::Bit8u TIMBRE_SYSEX[] = {
	0x02, 0x00, 0x00,
	'T', 'e', 's', 't', '-', 's', 'i', 'n', 'e', '.', 0x00, 0x00, 0x01, 0x00,
	0x24, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32,
	0x00, 0x0b, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x64, 0x32, 0x00, 0x0c, 0x00, 0x0c, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x64,
	0x64, 0x64
};

static void sendPatch(MT32Emu::Synth &synth, MT32Emu::Bit8u channel, bool usePCM) {
	synth.writeSysex(channel, PATCH_SYSEX, sizeof PATCH_SYSEX);
	MT32Emu::Bit8u timbre[sizeof TIMBRE_SYSEX];
	std::memcpy(timbre, TIMBRE_SYSEX, sizeof TIMBRE_SYSEX);
	timbre[13] = usePCM ? 2 : 0;
	synth.writeSysex(channel, timbre, sizeof timbre);
}

static void printUsage(const char *program) {
	std::fprintf(stderr,
		"Usage: %s <float|int> <synth|pcm> <polyphony> <render-seconds> [warmup-seconds] [raw-output]\n",
		program);
}

static void configurePCM(MT32Emu::Test::ROMSet &romSet, bool usePCM) {
	if (!usePCM) return;

	const MT32Emu::ControlROMMap *controlROMMap = MT32Emu::Test::getControlROMMap("ctrl_mt32_2_07");
	if (controlROMMap == NULL) {
		std::fprintf(stderr, "Failed to locate the control ROM map\n");
		std::exit(1);
	}

	MT32Emu::Bit8u *controlROMData = const_cast<MT32Emu::Bit8u *>(romSet.getControlROMImage()->getFile()->getData());
	MT32Emu::ControlROMPCMStruct *pcmTable = reinterpret_cast<MT32Emu::ControlROMPCMStruct *>(controlROMData + controlROMMap->pcmTable);
	pcmTable[0].pos = 0;
	pcmTable[0].len = 0x80 | 0x30;
	pcmTable[0].pitchLSB = 0x40;
	pcmTable[0].pitchMSB = 0x9C;

	MT32Emu::Bit8u *pcmROMData = const_cast<MT32Emu::Bit8u *>(romSet.getPCMROMImage()->getFile()->getData());
	const size_t pcmROMSize = romSet.getPCMROMImage()->getFile()->getSize();
	MT32Emu::Bit32u state = 0x9E3779B9;
	for (size_t i = 0; i < pcmROMSize; i++) {
		state ^= state << 13;
		state ^= state >> 17;
		state ^= state << 5;
		pcmROMData[i] = MT32Emu::Bit8u(state);
	}
}

} // namespace

int main(int argc, char *argv[]) {
	if (argc < 5) {
		printUsage(argv[0]);
		return 2;
	}

	const bool useFloat = std::strcmp(argv[1], "float") == 0;
	if (!useFloat && std::strcmp(argv[1], "int") != 0) {
		printUsage(argv[0]);
		return 2;
	}

	const bool usePCM = std::strcmp(argv[2], "pcm") == 0;
	if (!usePCM && std::strcmp(argv[2], "synth") != 0) {
		printUsage(argv[0]);
		return 2;
	}

	const unsigned int polyphony = unsigned(std::strtoul(argv[3], NULL, 10));
	const double renderSeconds = std::strtod(argv[4], NULL);
	const double warmupSeconds = argc > 5 ? std::strtod(argv[5], NULL) : 1.0;
	const char *rawOutput = argc > 6 ? argv[6] : NULL;

	if (polyphony == 0 || polyphony > 32 || renderSeconds <= 0.0 || warmupSeconds < 0.0) {
		printUsage(argv[0]);
		return 2;
	}

	MT32Emu::Test::ROMSet romSet;
	romSet.initMT32New();
	configurePCM(romSet, usePCM);

	MT32Emu::Synth synth;
	if (useFloat) {
		synth.selectRendererType(MT32Emu::RendererType_FLOAT);
	} else {
		synth.selectRendererType(MT32Emu::RendererType_BIT16S);
	}
	if (!synth.open(*romSet.getControlROMImage(), *romSet.getPCMROMImage(), 32, MT32Emu::AnalogOutputMode_DIGITAL_ONLY)) {
		std::fprintf(stderr, "Failed to open the synth\n");
		return 1;
	}
	sendPatch(synth, 1, usePCM);
	for (unsigned int i = 0; i < polyphony; i++) {
		const unsigned int note = 48 + i;
		synth.playMsgNow(0x91 | (note << 8) | (127 << 16));
	}
	if (std::getenv("MT32EMU_BENCH_DEBUG") != NULL) {
		std::vector<MT32Emu::PartialState> states(synth.getPartialCount());
		synth.getPartialStates(states.data());
		unsigned int active = 0;
		for (size_t i = 0; i < states.size(); i++) {
			if (states[i] != MT32Emu::PartialState_INACTIVE) active++;
		}
		std::fprintf(stderr, "debug: active partials before warmup=%u/%u\n", active, synth.getPartialCount());
	}

	const MT32Emu::Bit32u frameCount = MT32Emu::Bit32u(renderSeconds * MT32Emu::SAMPLE_RATE + 0.5);
	const MT32Emu::Bit32u warmupFrameCount = MT32Emu::Bit32u(warmupSeconds * MT32Emu::SAMPLE_RATE + 0.5);
	std::vector<float> floatBuffer(frameCount << 1);
	std::vector<MT32Emu::Bit16s> intBuffer(frameCount << 1);

	auto renderFrames = [&](MT32Emu::Bit32u frames, float *floatOut, MT32Emu::Bit16s *intOut) {
		MT32Emu::Bit32u remaining = frames;
		while (remaining > 0) {
			const MT32Emu::Bit32u passLen = remaining > MT32Emu::MAX_SAMPLES_PER_RUN ? MT32Emu::MAX_SAMPLES_PER_RUN : remaining;
			if (useFloat) {
				synth.render(floatOut, passLen);
				if (floatOut != NULL) floatOut += passLen << 1;
			} else {
				synth.render(intOut, passLen);
				if (intOut != NULL) intOut += passLen << 1;
			}
			remaining -= passLen;
		}
	};

	if (warmupFrameCount > 0) {
		if (useFloat) {
			renderFrames(warmupFrameCount, floatBuffer.data(), NULL);
		} else {
			renderFrames(warmupFrameCount, NULL, intBuffer.data());
		}
	}

	const auto start = std::chrono::steady_clock::now();
	if (useFloat) {
		renderFrames(frameCount, floatBuffer.data(), NULL);
	} else {
		renderFrames(frameCount, NULL, intBuffer.data());
	}
	const auto end = std::chrono::steady_clock::now();

	const double elapsed = std::chrono::duration<double>(end - start).count();
	const double audioSeconds = double(frameCount) / MT32Emu::SAMPLE_RATE;
	std::printf("renderer=%s source=%s polyphony=%u frames=%u seconds=%.6f audio_seconds=%.6f realtime_ratio=%.3f\n",
		useFloat ? "float" : "int", usePCM ? "pcm" : "synth", polyphony, frameCount, elapsed, audioSeconds, audioSeconds / elapsed);

	if (rawOutput != NULL) {
		std::FILE *file = std::fopen(rawOutput, "wb");
		if (file == NULL) {
			std::perror(rawOutput);
			return 1;
		}
		if (useFloat) {
			std::fwrite(floatBuffer.data(), sizeof(float), floatBuffer.size(), file);
		} else {
			std::fwrite(intBuffer.data(), sizeof(MT32Emu::Bit16s), intBuffer.size(), file);
		}
		std::fclose(file);
	}

	unsigned long long hash = 1469598103934665603ULL;
	const unsigned char *bytes;
	size_t byteCount;
	if (useFloat) {
		bytes = reinterpret_cast<const unsigned char *>(floatBuffer.data());
		byteCount = floatBuffer.size() * sizeof(float);
	} else {
		bytes = reinterpret_cast<const unsigned char *>(intBuffer.data());
		byteCount = intBuffer.size() * sizeof(MT32Emu::Bit16s);
	}
	for (size_t i = 0; i < byteCount; i++) {
		hash ^= bytes[i];
		hash *= 1099511628211ULL;
	}
	std::printf("fnv1a64=%016llx\n", hash);

	synth.close();
	return 0;
}
