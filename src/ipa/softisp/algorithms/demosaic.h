/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Gain dependent demosaicing strength for the software ISP
 */

#pragma once

#include <stdint.h>

#include "libipa/pwl.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::softisp::algorithms {

class Demosaic : public Algorithm
{
public:
	Demosaic() = default;
	~Demosaic() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void prepare(IPAContext &context,
		     const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	Pwl strength_;
	Pwl chromaDenoise_;
	double gain_ = 1.0;
};

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
