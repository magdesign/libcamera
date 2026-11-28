/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Contrast detection autofocus for the software ISP
 */

#include "af.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

namespace libcamera {

namespace ipa::softisp::algorithms {

/**
 * \class Af
 * \brief Contrast detection autofocus
 *
 * The algorithm measures the contrast of the centre of the image, using the
 * sharpness statistic of the software ISP normalised by the luminance, and
 * searches for the lens position that maximises it.
 *
 * A search climbs the contrast curve from the current lens position: it
 * steps the lens in one direction while the contrast rises, turns around if
 * the first step makes it worse, and stops two steps past the peak. A
 * parabola through the logarithms of the contrast at the best position and
 * its two neighbours (a Gaussian through the three values) then places the
 * lens between the steps. Starting where the lens already is keeps the
 * search short and avoids sweeping the whole range, which is visible as the
 * preview going out of focus. When the contrast does not change noticeably
 * the search keeps walking to the end of the range, and then explores the
 * other side of the starting position.
 *
 * Each lens position is evaluated on the next statistics frame, which the
 * software ISP produces every kStatPerNumFrames frames. This leaves the lens
 * time to settle.
 *
 * In continuous mode the contrast at the focused position is monitored. When
 * it changes by more than the drop ratio for a few statistics frames in a
 * row, the subject or the scene has changed. The algorithm then waits for
 * the contrast and luminance to be stable again before it searches, so that
 * moving the camera does not make the lens hunt.
 *
 * An AfTrigger start searches from the current position. When focus is
 * already good, this takes three statistics frames.
 *
 * Tuning parameters (all optional):
 * - map: piecewise linear function from dioptres to lens driver positions,
 *   used for the LensPosition control and metadata
 * - scanRange: [min, max] lens positions to search, defaults to the
 *   positions for 0 and 10 dioptres
 * - step: search step in lens positions
 * - dropRatio, dropFrames: in continuous mode, refocus when the contrast
 *   falls below dropRatio times the focused contrast (or rises above it
 *   divided by dropRatio) for dropFrames statistics
 */

LOG_DEFINE_CATEGORY(IPASoftIspAf)

namespace {

constexpr double kMaxDioptre = 10.0;

/* A search whose best contrast is not this much above its worst has failed */
constexpr double kMinContrastRange = 1.05;

/*
 * A peak counts once the best contrast is this much above the lowest one
 * seen. Lower values let measurement noise in the flat, blurred parts of the
 * contrast curve pass for a peak.
 */
constexpr double kPeakRatio = 1.3;

/* Contrast below this fraction of the best counts as past the peak */
constexpr double kWorseRatio = 0.92;

/* The first step turns around when it lowers the contrast below this */
constexpr double kFirstStepRatio = 0.9;

/* Relative change of contrast and luminance between stable statistics */
constexpr double kStableChange = 0.08;

/* Stable statistics in a row needed before refocusing */
constexpr unsigned int kStableFrames = 2;

} /* namespace */

Af::Af()
	: scanMin_(0), scanMax_(0), step_(0), dropRatio_(0.75), dropFrames_(3),
	  hasLens_(false), lensMin_(0), lensMax_(0),
	  mode_(controls::AfModeContinuous), phase_(Phase::Idle),
	  afState_(controls::AfStateIdle), position_(0), startPos_(0), dir_(1),
	  reversed_(false), bestPos_(0), bestScore_(0.0), lowestScore_(0.0),
	  worseCount_(0),
	  focusedScore_(0.0), focusedLuma_(0.0), lastScore_(0.0), lastLuma_(0.0),
	  dropCount_(0), stableCount_(0), skip_(false)
{
}

int Af::init(IPAContext &context, const ValueNode &tuningData)
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

	step_ = tuningData["step"].get<int32_t>(0);
	dropRatio_ = tuningData["dropRatio"].get<double>(0.75);
	dropFrames_ = tuningData["dropFrames"].get<uint32_t>(3);

	/*
	 * Advertise the controls already, applications check them before the
	 * camera is configured. configure() refines LensPosition from the lens,
	 * or removes the controls if the camera turns out to have no lens.
	 */
	Pwl::Interval dioptres = dioptreToPosition_.empty()
					 ? Pwl::Interval(0.0, kMaxDioptre)
					 : dioptreToPosition_.domain();
	context.ctrlMap[&controls::AfMode] =
		ControlInfo(controls::AfModeValues, ControlValue(controls::AfModeContinuous));
	context.ctrlMap[&controls::AfTrigger] = ControlInfo(controls::AfTriggerValues);
	context.ctrlMap[&controls::LensPosition] =
		ControlInfo(static_cast<float>(dioptres.start),
			    static_cast<float>(dioptres.end),
			    static_cast<float>(dioptres.start));

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
	if (step_ <= 0)
		step_ = std::max((scanMax_ - scanMin_) / 12, 1);

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
	moveTo(context, scanMin_);
	startSearch();
	/* The lens is only moved once streaming starts. */
	skip_ = true;

	LOG(IPASoftIspAf, Debug)
		<< "Lens " << lensMin_ << "-" << lensMax_ << ", scan "
		<< scanMin_ << "-" << scanMax_ << " step " << step_;

	return 0;
}

void Af::startSearch()
{
	phase_ = Phase::Search;
	afState_ = controls::AfStateScanning;
	startPos_ = position_;
	dir_ = position_ + step_ <= scanMax_ ? 1 : -1;
	reversed_ = false;
	scores_.clear();
	bestScore_ = 0.0;
	lowestScore_ = 0.0;
	bestPos_ = position_;
	worseCount_ = 0;
	dropCount_ = 0;
	stableCount_ = 0;
	skip_ = false;
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
			startSearch();
		} else {
			phase_ = Phase::Idle;
			afState_ = controls::AfStateIdle;
		}
	}

	const auto &trigger = controls.get(controls::AfTrigger);
	if (trigger && mode_ != controls::AfModeManual) {
		/* A search already in progress completes the trigger. */
		if (*trigger == controls::AfTriggerStart && phase_ != Phase::Search) {
			startSearch();
		} else if (*trigger == controls::AfTriggerCancel) {
			phase_ = Phase::Idle;
			afState_ = controls::AfStateIdle;
		}
	}

	const auto &lensPosition = controls.get(controls::LensPosition);
	if (lensPosition && mode_ == controls::AfModeManual)
		moveTo(context, std::lround(dioptreToPosition_.eval(*lensPosition)));
}

/*
 * Place the lens on the peak of a Gaussian through the best position and its
 * two neighbours, the classic three point peak interpolation of contrast
 * autofocus. Without both neighbours, the best position is used as is.
 */
int32_t Af::interpolatePeak() const
{
	auto lower = scores_.find(bestPos_ - step_);
	auto upper = scores_.find(bestPos_ + step_);
	if (lower == scores_.end() || upper == scores_.end() ||
	    lower->second <= 0.0 || upper->second <= 0.0)
		return bestPos_;

	double lm = std::log(lower->second);
	double lb = std::log(bestScore_);
	double lp = std::log(upper->second);
	double denom = lm - 2.0 * lb + lp;
	if (denom >= 0.0)
		return bestPos_;

	double offset = 0.5 * (lm - lp) / denom;
	offset = std::clamp(offset, -0.5, 0.5);

	return bestPos_ + std::lround(offset * step_);
}

void Af::finishSearch(IPAContext &context)
{
	double worst = lowestScore_;
	bool ok = worst > 0.0 && bestScore_ / worst >= kMinContrastRange;
	int32_t target = ok ? interpolatePeak() : bestPos_;

	afState_ = ok ? controls::AfStateFocused : controls::AfStateFailed;
	phase_ = Phase::Focused;
	focusedScore_ = 0.0;
	dropCount_ = 0;

	LOG(IPASoftIspAf, Debug)
		<< "Search done after " << scores_.size() << " positions: best "
		<< bestPos_ << " contrast " << bestScore_ << " (worst " << worst
		<< "), lens to " << target << (ok ? "" : ", no clear peak");

	moveTo(context, target);
}

void Af::searchStep(IPAContext &context, double contrast)
{
	scores_[position_] = contrast;
	lowestScore_ = scores_.size() == 1 ? contrast
					   : std::min(lowestScore_, contrast);

	if (contrast > bestScore_) {
		bestScore_ = contrast;
		bestPos_ = position_;
		worseCount_ = 0;
	}

	bool peak = bestScore_ >= kPeakRatio * lowestScore_;
	if (peak && contrast < kWorseRatio * bestScore_)
		worseCount_++;

	/* Two positions past a clear peak: done. */
	if (worseCount_ >= 2) {
		finishSearch(context);
		return;
	}

	int32_t next = position_ + dir_ * step_;
	bool atEnd = next < scanMin_ || next > scanMax_;

	/*
	 * Turn around when the first step away from the start clearly lowers
	 * the contrast, or when the end of the range is reached without having
	 * climbed to a peak.
	 */
	bool firstStepWorse = scores_.size() == 2 && bestPos_ == startPos_ &&
			      contrast < kFirstStepRatio * bestScore_;
	bool climbed = peak && bestPos_ != startPos_;

	if (!reversed_ && (firstStepWorse || (atEnd && !climbed))) {
		dir_ = -dir_;
		reversed_ = true;
		if (!firstStepWorse)
			worseCount_ = 0;
		next = startPos_ + dir_ * step_;
		atEnd = next < scanMin_ || next > scanMax_;
	}

	if (atEnd) {
		finishSearch(context);
		return;
	}

	moveTo(context, next);
}

void Af::monitor(double contrast, double luma)
{
	/* The first statistics at the focused position set the reference. */
	if (focusedScore_ == 0.0) {
		focusedScore_ = contrast;
		focusedLuma_ = luma;
		return;
	}

	double ratio = contrast / focusedScore_;
	double lumaChange = std::abs(luma / focusedLuma_ - 1.0);
	if (ratio < dropRatio_ || ratio > 1.0 / dropRatio_ || lumaChange > 0.5)
		dropCount_++;
	else
		dropCount_ = 0;

	if (dropCount_ < dropFrames_)
		return;

	LOG(IPASoftIspAf, Debug)
		<< "Contrast changed from " << focusedScore_ << " to "
		<< contrast << ", waiting for a stable scene";

	phase_ = Phase::Wait;
	afState_ = controls::AfStateScanning;
	stableCount_ = 0;
	lastScore_ = contrast;
	lastLuma_ = luma;
}

void Af::waitStable(IPAContext &context, double contrast, double luma)
{
	bool stable = std::abs(contrast / lastScore_ - 1.0) < kStableChange &&
		      std::abs(luma / lastLuma_ - 1.0) < kStableChange;
	stableCount_ = stable ? stableCount_ + 1 : 0;
	lastScore_ = contrast;
	lastLuma_ = luma;

	if (stableCount_ < kStableFrames)
		return;

	/* These statistics are the first sample of the search. */
	startSearch();
	searchStep(context, contrast);
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
		if (mode_ == controls::AfModeContinuous)
			monitor(contrast, luma);
		return;

	case Phase::Wait:
		waitStable(context, contrast, luma);
		return;

	case Phase::Search:
		/* The first statistics after a move outside process() predate it. */
		if (skip_) {
			skip_ = false;
			return;
		}
		searchStep(context, contrast);
		return;
	}
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
