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
		Search,
		Focused,
		Wait,
	};

	void startSearch();
	void searchStep(IPAContext &context, double contrast);
	void refineStep(IPAContext &context, double contrast);
	void record(double contrast);
	int32_t interpolatePeak() const;
	void finishSearch(IPAContext &context);
	void moveTo(IPAContext &context, int32_t position);
	void monitor(double contrast, double luma);
	void wait(double contrast);
	void waitStable(IPAContext &context, double contrast);

	/* Tuning */
	Pwl dioptreToPosition_;
	Pwl positionToDioptre_;
	int32_t scanMin_;
	int32_t scanMax_;
	int32_t defaultPos_;
	int32_t step_;
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
	int32_t startPos_;
	int32_t dir_;
	bool reversed_;
	std::map<int32_t, double> scores_;
	std::vector<int32_t> refine_;
	bool refining_;
	int32_t bestPos_;
	double bestScore_;
	double lowestScore_;
	unsigned int worseCount_;
	double focusedScore_;
	double focusedLuma_;
	double lastScore_;
	double exposure_;
	double searchExposure_;
	double lastExposure_;
	unsigned int abortsLeft_;
	unsigned int dropCount_;
	unsigned int waitCount_;
	bool locked_;
	bool skip_;
};

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
