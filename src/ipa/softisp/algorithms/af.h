/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Contrast detection autofocus for the software ISP
 */

#pragma once

#include <map>
#include <optional>
#include <stdint.h>
#include <vector>

#include <libcamera/controls.h>

#include "libipa/pwl.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

class Af : public Algorithm
{
public:
	Af();
	~Af() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	enum class Phase {
		Idle,
		Probe,
		Coarse,
		Fine,
		Focused,
	};

	void startScan();
	void startProbe();
	void probeStep(IPAContext &context, double contrast);
	void finishScan();
	void moveTo(IPAContext &context, int32_t position);
	void monitor(double contrast, double luma);

	/* Tuning */
	Pwl dioptreToPosition_;
	Pwl positionToDioptre_;
	int32_t scanMin_;
	int32_t scanMax_;
	int32_t coarseStep_;
	int32_t fineStep_;
	double dropRatio_;
	unsigned int dropFrames_;

	/* Lens */
	bool hasLens_;
	int32_t lensMin_;
	int32_t lensMax_;

	/* State */
	int32_t mode_;
	Phase phase_;
	int32_t afState_;
	int32_t position_;
	int32_t scanPos_;
	int32_t scanEnd_;
	int32_t scanStep_;
	std::map<int32_t, double> scores_;
	std::vector<int32_t> probePositions_;
	unsigned int probeIndex_;
	int32_t bestPos_;
	double bestScore_;
	double focusedScore_;
	double focusedLuma_;
	unsigned int dropCount_;
	unsigned int stableCount_;
	bool triggered_;
};

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
