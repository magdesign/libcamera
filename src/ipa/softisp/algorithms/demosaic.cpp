/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Gain dependent demosaicing strength for the software ISP
 */

#include "demosaic.h"

#include <algorithm>

#include <libcamera/base/log.h>

namespace libcamera {

namespace ipa::softisp::algorithms {

/**
 * \class Demosaic
 * \brief Select the demosaicing strength from the sensor gain
 *
 * The GPU debayering of regular Bayer patterns uses the Malvar-He-Cutler
 * filters, which add the Laplacian of the sampled colour to a bilinear
 * interpolation. That recovers fine detail at low gain, but it also
 * amplifies the sensor noise, which dominates at high gain. The strength
 * blends between bilinear (0.0) and full Malvar-He-Cutler (1.0)
 * interpolation.
 *
 * The tuning data holds the strength as a piecewise linear function of the
 * analogue gain, as a flat list of gain and strength pairs:
 *
 * \code{.yaml}
 * - Demosaic:
 *     strength: [ 1.0, 1.0, 2.0, 1.0, 8.0, 0.3 ]
 * \endcode
 *
 * Without the strength key, full Malvar-He-Cutler interpolation is used at
 * all gains.
 */

LOG_DEFINE_CATEGORY(IPASoftIspDemosaic)

int Demosaic::init([[maybe_unused]] IPAContext &context,
		   const ValueNode &tuningData)
{
	if (!tuningData.contains("strength"))
		return 0;

	auto strength = tuningData["strength"].get<Pwl>();
	if (!strength) {
		LOG(IPASoftIspDemosaic, Error) << "Invalid strength";
		return -EINVAL;
	}

	strength_ = std::move(*strength);
	return 0;
}

int Demosaic::configure([[maybe_unused]] IPAContext &context,
			[[maybe_unused]] const IPAConfigInfo &configInfo)
{
	gain_ = 1.0;
	return 0;
}

void Demosaic::prepare([[maybe_unused]] IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       DebayerParams *params)
{
	/*
	 * The sensor gain of the frame being prepared is only known once its
	 * statistics have been processed, so use the most recent one. The
	 * gain changes slowly enough for that to make no visible difference.
	 */
	double strength = strength_.empty() ? 1.0 : strength_.eval(gain_);

	params->demosaicStrength = std::clamp(strength, 0.0, 1.0);
}

void Demosaic::process([[maybe_unused]] IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       [[maybe_unused]] const SwIspStats *stats,
		       [[maybe_unused]] ControlList &metadata)
{
	if (frameContext.sensor.gain > 0.0)
		gain_ = frameContext.sensor.gain;
}

REGISTER_IPA_ALGORITHM(Demosaic, "Demosaic")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
