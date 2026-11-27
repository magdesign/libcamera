/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Contrast detection autofocus for the software ISP
 */

#include "af.h"

#include <algorithm>
#include <cmath>

#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

namespace libcamera {

namespace ipa::softisp::algorithms {

/**
 * \class Af
 * \brief Contrast detection autofocus
 *
 * The algorithm moves the lens through its range and measures the contrast
 * of the centre of the image at every position, using the sharpness
 * statistic of the software ISP normalised by the luminance. A coarse scan
 * over the configured range is followed by a fine scan around the best
 * coarse position, after which the lens is moved to the sharpest position.
 *
 * Each lens position is evaluated on the next statistics frame, which the
 * software ISP produces every kStatPerNumFrames frames. This leaves the lens
 * time to settle.
 *
 * In continuous mode the contrast at the focused position is monitored, and
 * a new scan starts when it drops for a few statistics frames in a row (the
 * subject or the scene has changed).
 *
 * Tuning parameters (all optional):
 * - map: piecewise linear function from dioptres to lens driver positions,
 *   used for the LensPosition control and metadata
 * - scanRange: [min, max] lens positions to scan, defaults to the positions
 *   for 0 and 10 dioptres
 * - coarseStep, fineStep: scan steps in lens positions
 * - dropRatio, dropFrames: in continuous mode, rescan when the contrast falls
 *   below dropRatio times the focused contrast for dropFrames statistics
 */

LOG_DEFINE_CATEGORY(IPASoftIspAf)

namespace {

constexpr double kMaxDioptre = 10.0;

/* A scan whose best contrast is not this much above its worst has failed */
constexpr double kMinContrastRange = 1.05;

} /* namespace */

Af::Af()
	: scanMin_(0), scanMax_(0), coarseStep_(0), fineStep_(0),
	  dropRatio_(0.75), dropFrames_(3), hasLens_(false), lensMin_(0),
	  lensMax_(0), mode_(controls::AfModeContinuous), phase_(Phase::Idle),
	  afState_(controls::AfStateIdle), position_(0), scanPos_(0),
	  scanEnd_(0), scanStep_(0), bestPos_(0), bestScore_(0.0),
	  focusedScore_(0.0), focusedLuma_(0.0), dropCount_(0), stableCount_(0),
	  triggered_(false)
{
}

int Af::init([[maybe_unused]] IPAContext &context, const ValueNode &tuningData)
{
	if (tuningData.contains("map")) {
		auto map = tuningData["map"].get<Pwl>();
		if (!map) {
			LOG(IPASoftIspAf, Error) << "Invalid 'map'";
			return -EINVAL;
		}
		dioptreToPosition_ = std::move(*map);
	}

	auto range = tuningData["scanRange"].get<std::vector<int32_t>>();
	if (range && range->size() == 2) {
		scanMin_ = (*range)[0];
		scanMax_ = (*range)[1];
	}

	coarseStep_ = tuningData["coarseStep"].get<int32_t>(0);
	fineStep_ = tuningData["fineStep"].get<int32_t>(0);
	dropRatio_ = tuningData["dropRatio"].get<double>(0.75);
	dropFrames_ = tuningData["dropFrames"].get<uint32_t>(3);

	return 0;
}

int Af::configure(IPAContext &context,
		  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	const auto &lensControls = context.lensControls;
	auto it = lensControls.find(V4L2_CID_FOCUS_ABSOLUTE);
	hasLens_ = it != lensControls.end();

	if (!hasLens_) {
		context.ctrlMap.erase(&controls::AfMode);
		context.ctrlMap.erase(&controls::AfTrigger);
		context.ctrlMap.erase(&controls::LensPosition);
		return 0;
	}

	lensMin_ = it->second.min().get<int32_t>();
	lensMax_ = it->second.max().get<int32_t>();

	/* Without a calibrated map, spread 0 to kMaxDioptre over the range. */
	if (dioptreToPosition_.empty())
		dioptreToPosition_ = Pwl({ Pwl::Point({ 0.0, static_cast<double>(lensMin_) }),
					   Pwl::Point({ kMaxDioptre, static_cast<double>(lensMax_) }) });

	auto [inverse, ok] = dioptreToPosition_.inverse();
	if (!ok) {
		LOG(IPASoftIspAf, Error) << "Lens map is not invertible";
		return -EINVAL;
	}
	positionToDioptre_ = std::move(inverse);

	if (scanMin_ >= scanMax_) {
		scanMin_ = std::lround(dioptreToPosition_.eval(0.0));
		scanMax_ = std::lround(dioptreToPosition_.eval(kMaxDioptre));
	}
	scanMin_ = std::clamp(scanMin_, lensMin_, lensMax_);
	scanMax_ = std::clamp(scanMax_, lensMin_, lensMax_);
	if (coarseStep_ <= 0)
		coarseStep_ = std::max((scanMax_ - scanMin_) / 12, 1);
	if (fineStep_ <= 0)
		fineStep_ = std::max(coarseStep_ / 4, 1);

	Pwl::Interval dioptres = dioptreToPosition_.domain();
	context.ctrlMap[&controls::AfMode] =
		ControlInfo(controls::AfModeValues, ControlValue(controls::AfModeContinuous));
	context.ctrlMap[&controls::AfTrigger] =
		ControlInfo(controls::AfTriggerValues);
	context.ctrlMap[&controls::LensPosition] =
		ControlInfo(static_cast<float>(dioptres.start),
			    static_cast<float>(dioptres.end),
			    static_cast<float>(positionToDioptre_.eval(scanMin_)));

	mode_ = controls::AfModeContinuous;
	position_ = scanMin_;
	context.activeState.lens.position = position_;
	startScan();

	LOG(IPASoftIspAf, Debug)
		<< "Lens " << lensMin_ << "-" << lensMax_ << ", scan "
		<< scanMin_ << "-" << scanMax_ << " steps " << coarseStep_
		<< "/" << fineStep_;

	return 0;
}

void Af::startScan()
{
	phase_ = Phase::Coarse;
	afState_ = controls::AfStateScanning;
	scanPos_ = scanMin_;
	scanEnd_ = scanMax_;
	scanStep_ = coarseStep_;
	scores_.clear();
	bestScore_ = 0.0;
	bestPos_ = scanMin_;
	dropCount_ = 0;
	stableCount_ = 0;
	/* The first statistics after a scan starts predate the lens move. */
	triggered_ = true;
}

void Af::moveTo(IPAContext &context, int32_t position)
{
	position_ = std::clamp(position, lensMin_, lensMax_);
	context.activeState.lens.position = position_;
}

void Af::queueRequest(IPAContext &context,
		      [[maybe_unused]] const uint32_t frame,
		      [[maybe_unused]] IPAFrameContext &frameContext,
		      const ControlList &controls)
{
	if (!hasLens_)
		return;

	const auto &mode = controls.get(controls::AfMode);
	if (mode && *mode != mode_) {
		mode_ = *mode;
		if (mode_ == controls::AfModeContinuous) {
			startScan();
			moveTo(context, scanPos_);
		} else {
			phase_ = Phase::Idle;
			afState_ = controls::AfStateIdle;
		}
	}

	const auto &trigger = controls.get(controls::AfTrigger);
	if (trigger && mode_ != controls::AfModeManual) {
		if (*trigger == controls::AfTriggerStart) {
			startScan();
			moveTo(context, scanPos_);
		} else if (*trigger == controls::AfTriggerCancel) {
			phase_ = Phase::Idle;
			afState_ = controls::AfStateIdle;
		}
	}

	const auto &lensPosition = controls.get(controls::LensPosition);
	if (lensPosition && mode_ == controls::AfModeManual)
		moveTo(context, std::lround(dioptreToPosition_.eval(*lensPosition)));
}

void Af::finishScan()
{
	double worst = bestScore_;
	for (const auto &[pos, score] : scores_)
		worst = std::min(worst, score);

	bool ok = worst > 0.0 && bestScore_ / worst >= kMinContrastRange;
	afState_ = ok ? controls::AfStateFocused : controls::AfStateFailed;
	phase_ = Phase::Focused;
	focusedScore_ = 0.0;
	dropCount_ = 0;

	LOG(IPASoftIspAf, Debug)
		<< "Scan done: best position " << bestPos_ << " contrast "
		<< bestScore_ << " (worst " << worst << ")"
		<< (ok ? "" : ", no clear peak");
}

void Af::monitor(double contrast, double luma)
{
	/* The first statistics at the focused position set the reference. */
	if (focusedScore_ == 0.0) {
		focusedScore_ = contrast;
		focusedLuma_ = luma;
		return;
	}

	double lumaChange = std::abs(luma / focusedLuma_ - 1.0);
	if (contrast < dropRatio_ * focusedScore_ || lumaChange > 0.5)
		dropCount_++;
	else
		dropCount_ = 0;

	if (dropCount_ >= dropFrames_) {
		LOG(IPASoftIspAf, Debug)
			<< "Contrast dropped from " << focusedScore_ << " to "
			<< contrast << ", rescanning";
		startScan();
	}
}

void Af::process(IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const SwIspStats *stats,
		 ControlList &metadata)
{
	if (!hasLens_)
		return;

	metadata.set(controls::AfState, afState_);
	metadata.set(controls::LensPosition,
		     static_cast<float>(positionToDioptre_.eval(position_)));

	if (!stats->valid || !stats->sharpnessLuma)
		return;

	double contrast = static_cast<double>(stats->sharpness) / stats->sharpnessLuma;
	double luma = static_cast<double>(stats->sharpnessLuma);

	switch (phase_) {
	case Phase::Idle:
		return;

	case Phase::Focused:
		if (mode_ == controls::AfModeContinuous) {
			monitor(contrast, luma);
			if (phase_ == Phase::Coarse)
				moveTo(context, scanPos_);
		}
		return;

	case Phase::Coarse:
	case Phase::Fine:
		break;
	}

	if (triggered_) {
		triggered_ = false;
		moveTo(context, scanPos_);
		return;
	}

	scores_[position_] = contrast;
	if (contrast > bestScore_) {
		bestScore_ = contrast;
		bestPos_ = position_;
	}

	/* Stop the coarse scan early once well past a clear peak. */
	bool pastPeak = phase_ == Phase::Coarse &&
			position_ >= bestPos_ + 2 * scanStep_ &&
			contrast < 0.6 * bestScore_;

	scanPos_ += scanStep_;
	if (scanPos_ <= scanEnd_ && !pastPeak) {
		moveTo(context, scanPos_);
		return;
	}

	if (phase_ == Phase::Coarse) {
		phase_ = Phase::Fine;
		scanPos_ = std::max(bestPos_ - coarseStep_ + fineStep_, scanMin_);
		scanEnd_ = std::min(bestPos_ + coarseStep_ - fineStep_, scanMax_);
		scanStep_ = fineStep_;
		moveTo(context, scanPos_);
		return;
	}

	finishScan();
	moveTo(context, bestPos_);
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
