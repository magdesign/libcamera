/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#include "agc.h"

#include <algorithm>
#include <cmath>
#include <stdint.h>

#include <libcamera/base/log.h>

#include "control_ids.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftExposure)

namespace ipa::soft::algorithms {

/*
 * The number of bins to use for the optimal exposure calculations.
 */
static constexpr unsigned int kExposureBinsCount = 5;

/*
 * The exposure is optimal when the mean sample value of the histogram is
 * in the middle of the range.
 */
static constexpr float kExposureOptimal = kExposureBinsCount / 2.0;

/*
 * This implements the hysteresis for the exposure adjustment.
 * It is small enough to have the exposure close to the optimal, and is big
 * enough to prevent the exposure from wobbling around the optimal value.
 */
static constexpr float kExposureSatisfactory = 0.2;

/*
 * Proportional gain for exposure/gain adjustment. Maps the MSV error to a
 * multiplicative correction factor:
 *
 *   factor = 1.0 + kExpProportionalGain * error
 *
 * With kExpProportionalGain = 0.04:
 *   - max error ~2.5 -> factor 1.10 (~10% step, same as before)
 *   - error 1.0      -> factor 1.04 (~4% step)
 *   - error 0.3      -> factor 1.012 (~1.2% step)
 *
 * This replaces the fixed 10% bang-bang step with a proportional correction
 * that converges smoothly and avoids overshooting near the target.
 */
static constexpr float kExpProportionalGain = 0.04;

/*
 * Maximum multiplicative step per frame, to bound the correction when the
 * scene changes dramatically.
 */
static constexpr float kExpMaxStep = 0.15;

Agc::Agc()
{
}

int Agc::init(IPAContext &context,
	      [[maybe_unused]] const ValueNode &tuningData)
{
	context.ctrlMap[&controls::AeEnable] = ControlInfo(false, true, true);
	context.ctrlMap[&controls::Brightness] = ControlInfo(0.0f, 2.0f, 1.0f);
	context.ctrlMap[&controls::ExposureValue] = ControlInfo(0.0f, 0.5f, 1.0f);

	return 0;
}

int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	context.activeState.knobs.brightness = std::optional<double>();
	context.activeState.knobs.ae_enabled = std::optional<bool>();

	return 0;
}

void Agc::queueRequest(typename Module::Context &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] typename Module::FrameContext &frameContext,
		       const ControlList &controls)
{
	const auto &brightness = controls.get(controls::Brightness);
	const auto &ae_enabled = controls.get(controls::AeEnable);
	const auto &exposure_value = controls.get(controls::ExposureValue);
	if (brightness.has_value()) {
		context.activeState.knobs.brightness = brightness;
		LOG(IPASoftExposure, Debug) << "Setting brightness to " << brightness.value();
	}
	if (ae_enabled.has_value()) {
		context.activeState.knobs.ae_enabled = ae_enabled;
		LOG(IPASoftExposure, Debug) << "Setting ae_enable to " << ae_enabled.value();
	}
	if (exposure_value.has_value()) {
		context.activeState.knobs.exposure_value = exposure_value.value();
		LOG(IPASoftExposure, Debug) << "Setting exposure value to " << exposure_value.value();
	}
}

void Agc::updateExposure(IPAContext &context, IPAFrameContext &frameContext, double exposureMSV)
{
	int32_t &exposure = frameContext.sensor.exposure;
	double &again = frameContext.sensor.gain;

	const auto brightness = context.activeState.knobs.brightness.value_or(1.0);
	exposureMSV /= brightness;

	double error = kExposureOptimal - exposureMSV;

	if (std::abs(error) <= kExposureSatisfactory)
		return;

	/*
	 * Compute a proportional correction factor. The sign of the error
	 * determines the direction: positive error means too dark (increase),
	 * negative means too bright (decrease).
	 */
	float step = std::clamp(static_cast<float>(error) * kExpProportionalGain,
				-kExpMaxStep, kExpMaxStep);
	float factor = 1.0f + step;

	if (factor > 1.0f) {
		/* Scene too dark: increase exposure first, then gain. */
		if (exposure < context.configuration.agc.exposureMax) {
			int32_t next = static_cast<int32_t>(exposure * factor);
			exposure = std::max(next, exposure + 1);
		} else {
			double next = again * factor;
			if (next - again < context.configuration.agc.againMinStep)
				again += context.configuration.agc.againMinStep;
			else
				again = next;
		}
	} else {
		/* Scene too bright: decrease gain first, then exposure. */
		if (again > context.configuration.agc.again10) {
			double next = again * factor;
			if (again - next < context.configuration.agc.againMinStep)
				again -= context.configuration.agc.againMinStep;
			else
				again = next;
		} else {
			int32_t next = static_cast<int32_t>(exposure * factor);
			exposure = std::min(next, exposure - 1);
		}
	}

	exposure = std::clamp(exposure, context.configuration.agc.exposureMin,
			      context.configuration.agc.exposureMax);
	again = std::clamp(again, context.configuration.agc.againMin,
			   context.configuration.agc.againMax);

	context.activeState.agc.exposure = exposure;
	context.activeState.agc.again = again;

	LOG(IPASoftExposure, Debug)
		<< "exposureMSV " << exposureMSV
		<< " error " << error << " factor " << factor
		<< " exp " << exposure << " again " << again;
}

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	const auto ae_enable = context.activeState.knobs.ae_enabled.value_or(true);

	if (!ae_enable)
		frameContext.sensor.exposure = (int32_t)( context.activeState.knobs.exposure_value.value_or(0.5) * (context.configuration.agc.exposureMax - context.configuration.agc.exposureMin));
	utils::Duration exposureTime =
		context.configuration.agc.lineDuration * frameContext.sensor.exposure;
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	LOG(IPASoftExposure, Debug) << "Setting exposure value to " << frameContext.sensor.exposure;
	if (!ae_enable)
		return;

	if (!context.activeState.agc.valid) {
		/*
		 * Init active-state from sensor values in case updateExposure()
		 * does not run for the first frame.
		 */
		context.activeState.agc.exposure = frameContext.sensor.exposure;
		context.activeState.agc.again = frameContext.sensor.gain;
		context.activeState.agc.valid = true;
	}

	if (!stats->valid) {
		/*
		 * Use the new exposure and gain values calculated the last time
		 * there were valid stats.
		 */
		frameContext.sensor.exposure = context.activeState.agc.exposure;
		frameContext.sensor.gain = context.activeState.agc.again;
		return;
	}

	/*
	 * Calculate Mean Sample Value (MSV) according to formula from:
	 * https://www.araa.asn.au/acra/acra2007/papers/paper84final.pdf
	 */
	const auto &histogram = stats->yHistogram;
	const unsigned int blackLevelHistIdx =
		context.activeState.blc.level / (256 / SwIspStats::kYHistogramSize);
	const unsigned int histogramSize =
		SwIspStats::kYHistogramSize - blackLevelHistIdx;
	const unsigned int yHistValsPerBin = histogramSize / kExposureBinsCount;
	const unsigned int yHistValsPerBinMod =
		histogramSize / (histogramSize % kExposureBinsCount + 1);
	int exposureBins[kExposureBinsCount] = {};
	unsigned int denom = 0;
	unsigned int num = 0;

	if (yHistValsPerBin == 0) {
		LOG(IPASoftExposure, Debug)
			<< "Not adjusting exposure due to insufficient histogram data";
		return;
	}

	for (unsigned int i = 0; i < histogramSize; i++) {
		unsigned int idx = (i - (i / yHistValsPerBinMod)) / yHistValsPerBin;
		exposureBins[idx] += histogram[blackLevelHistIdx + i];
	}

	for (unsigned int i = 0; i < kExposureBinsCount; i++) {
		LOG(IPASoftExposure, Debug) << i << ": " << exposureBins[i];
		denom += exposureBins[i];
		num += exposureBins[i] * (i + 1);
	}

	float exposureMSV = (denom == 0 ? 0 : static_cast<float>(num) / denom);
	updateExposure(context, frameContext, exposureMSV);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
