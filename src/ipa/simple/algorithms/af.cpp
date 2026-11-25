/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Vasiliy Doylov <nekodevelopper@gmail.com>
 *
 * Auto focus
 */

#include "af.h"

#include <stdint.h>

#include <libcamera/base/log.h>

#include "control_ids.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAutoFocus)

namespace ipa::soft::algorithms {

Af::Af()
{
}

int Af::init(IPAContext &context,
	     [[maybe_unused]] const ValueNode &tuningData)
{
	context.ctrlMap[&controls::LensPosition] = ControlInfo(0.0f, 100.0f, 50.0f);
	context.ctrlMap[&controls::AfTrigger] = ControlInfo(0, 1, 0);
	return 0;
}

int Af::configure(IPAContext &context,
		  [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	context.activeState.knobs.focus_sweep = std::optional<bool>();
	context.activeState.knobs.focus_pos = std::optional<double>();
	context.activeState.knobs.focus_sweep = false;
	context.activeState.knobs.focus_pos = 0;
	return 0;
}

void Af::queueRequest([[maybe_unused]] typename Module::Context &context,
		      [[maybe_unused]] const uint32_t frame,
		      [[maybe_unused]] typename Module::FrameContext &frameContext,
		      const ControlList &controls)
{
	const auto &focus_pos = controls.get(controls::LensPosition);
	const auto &af_trigger = controls.get(controls::AfTrigger);
	if (focus_pos.has_value()) {
		context.activeState.knobs.focus_pos = focus_pos;
		LOG(IPASoftAutoFocus, Debug) << "Setting focus position to " << focus_pos.value();
	}
	if (af_trigger.has_value()) {
		context.activeState.knobs.focus_sweep = af_trigger.value() == 1;
		if(context.activeState.knobs.focus_sweep){
			context.activeState.knobs.focus_pos = 0;
			context.configuration.focus.focus_max_pos = 0;
			context.configuration.focus.sharpness_max = 0;
			context.configuration.focus.start = 0;
			context.configuration.focus.stop = 100;
			context.configuration.focus.step = 25;
			LOG(IPASoftAutoFocus, Info) << "Starting focus sweep";
		}
	}

}

void Af::updateFocus([[maybe_unused]] IPAContext &context, [[maybe_unused]] IPAFrameContext &frameContext, [[maybe_unused]] double exposureMSV)
{
	frameContext.lens.focus_pos = context.activeState.knobs.focus_pos.value_or(50.0) / 100.0 * (context.configuration.focus.focus_max - context.configuration.focus.focus_min);
}

void Af::step(double& start, double& stop, double& step, double& focus_pos, double& max_pos, uint64_t& max_sharp, uint64_t sharp, bool& sweep){
	if(!sweep)
		return;
	if(focus_pos < start) {
		focus_pos = start;
		return;
	}
	if(sharp > max_sharp) {
		max_sharp = sharp;
		max_pos = focus_pos;
	}
	if(focus_pos >= stop) {
		LOG(IPASoftAutoFocus, Info) << "Best focus on step " <<step << ": " << focus_pos;
		start = std::clamp(max_pos - step, 0.0, 100.0);
		stop = std::clamp(max_pos +  step, 0.0, 100.0);
		focus_pos = start;
		max_sharp = 0;
		step /= 2;
		if(step <= 0.2){
			sweep = false;
			LOG(IPASoftAutoFocus, Info) << "Sweep end. Best focus: " << max_pos;
			focus_pos = max_pos;
		}
		return;
	}

	focus_pos += step;
}

void Af::process([[maybe_unused]] IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 [[maybe_unused]] const SwIspStats *stats,
		 [[maybe_unused]] ControlList &metadata)
{
	if (stats->valid) {
		step(context.configuration.focus.start,
			context.configuration.focus.stop,
			context.configuration.focus.step,
			context.activeState.knobs.focus_pos.value(),
			context.configuration.focus.focus_max_pos,
			context.configuration.focus.sharpness_max,
			stats->sharpness,
			context.activeState.knobs.focus_sweep.value());
	}

	updateFocus(context, frameContext, 0);
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
