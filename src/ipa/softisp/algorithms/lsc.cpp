/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Lens shading correction
 */

#include "lsc.h"

#include <algorithm>
#include <array>

#include <libcamera/base/log.h>

namespace libcamera {

namespace ipa::softisp::algorithms {

LOG_DEFINE_CATEGORY(IPASoftIspLsc)

int Lsc::init(IPAContext &context, const ValueNode &tuningData)
{
	static constexpr unsigned int kGridSize = DebayerParams::kLscGridSize;

	for (unsigned int i = 0; i < kGridSize; i++)
		gridPos_.push_back(static_cast<double>(i) / (kGridSize - 1));

	int ret = lscAlgo_.init(tuningData, context.ctrlMap,
			     { .keys = { "r", "g", "b" },
			       .numHSamples = kGridSize,
			       .numVSamples = kGridSize,
			       .sensorSize = context.sensorInfo.activeArea.size() });
	if (ret)
		return ret;

	context.lscEnabled = true;

	return 0;
}

int Lsc::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	return lscAlgo_.configure(context.activeState.lsc,
				  context.sensorInfo.analogCrop,
				  gridPos_, gridPos_);
}

void Lsc::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	unsigned int ct = frameContext.awb.colourTemperature;
	constexpr unsigned int minTemperatureChange = 100;

	if (!frameContext.lsc.enabled) {
		context.activeState.lscGains = RGB<double>{ { 1.0, 1.0, 1.0 } };
		if (lastAppliedCt_ != 0) {
			params->lscLut = DebayerParams::identityLscLut;
			params->lscLutVersion++;
			lastAppliedCt_ = 0;

			LOG(IPASoftIspLsc, Debug)
				<< "LSC disabled (version " << params->lscLutVersion << ")";
		}
		return;
	}

	context.activeState.lscGains = meanGains_;

	if (utils::abs_diff(ct, lastAppliedCt_) < minTemperatureChange)
		return;

	const auto &set = lscAlgo_.interpolateComponents(ct);

	const auto &red = set.at("r");
	const auto &green = set.at("g");
	const auto &blue = set.at("b");

	DebayerParams::LscLookupTable &lut = params->lscLut;
	constexpr unsigned int gridSize = DebayerParams::kLscGridSize;
	for (unsigned int i = 0, j = 0; i < gridSize * gridSize; i++) {
		lut[j++] = red[i];
		lut[j++] = green[i];
		lut[j++] = blue[i];
		lut[j++] = 0; /* padding */
	}
	params->lscLutVersion++;

	meanGains_ = meanGains(red, green, blue);
	context.activeState.lscGains = meanGains_;

	LOG(IPASoftIspLsc, Debug)
		<< "LSC updated to version " << params->lscLutVersion
		<< " and temperature " << ct << " (was " << lastAppliedCt_
		<< "), mean gains " << meanGains_;

	lastAppliedCt_ = ct;
}

/*
 * The mean gain the tables apply to each colour, relative to green. A uniform
 * grey surface reaches the sensor as 1 / gain of the image centre at each
 * position, so the statistics, which are gathered before the correction, see
 * the mean of 1 / gain: the harmonic mean of the gains, with the grid points
 * weighted by the image area they cover, converts them to corrected values.
 */
template<typename T>
RGB<double> Lsc::meanGains(const T &red, const T &green, const T &blue)
{
	constexpr unsigned int gridSize = DebayerParams::kLscGridSize;
	const std::array<const T *, 3> tables = { &red, &green, &blue };
	RGB<double> gains;

	for (unsigned int c = 0; c < 3; c++) {
		double area = 0.0;
		double inverse = 0.0;

		for (unsigned int y = 0; y < gridSize; y++) {
			for (unsigned int x = 0; x < gridSize; x++) {
				double weight = (x == 0 || x == gridSize - 1 ? 0.5 : 1.0) *
						(y == 0 || y == gridSize - 1 ? 0.5 : 1.0);
				double value = static_cast<double>((*tables[c])[y * gridSize + x]);

				area += weight;
				inverse += weight / std::max(value, 1.0);
			}
		}

		gains[c] = area / inverse;
	}

	return gains / gains.g();
}

void Lsc::queueRequest(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext, const ControlList &controls)
{
	lscAlgo_.queueRequest(context.activeState.lsc, frameContext.lsc,
			      controls);
}

void Lsc::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const SwIspStats *stats,
		  ControlList &metadata)
{
	lscAlgo_.process(frameContext.lsc, metadata);
}

REGISTER_IPA_ALGORITHM(Lsc, "Lsc")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
