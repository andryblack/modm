/*
 * Copyright (c) 2022, Andrey Kunitsyn
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "pio_asm.hpp"

namespace modm::platform
{
	
	struct PIOProgram {
		const pio::Instruction* instructions;
		const uint8_t length;
		int8_t origin;
		constexpr PIOProgram(const pio::Instruction* code,uint8_t length,int8_t origin) : instructions(code),length(length),origin(origin) {}
		static constexpr pio::EmptyProgram begin() {
			return pio::ProgramBegin();
		}
		template <typename Program>
		static constexpr PIOProgram get(const Program& data) {
			return PIOProgram(data.code.data(),data.code.size(),-1);
		}
	};

}