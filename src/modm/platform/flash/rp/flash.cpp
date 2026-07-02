/*
 * Copyright (c) 2026, Andrey Kunitsyn
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "flash.hpp"
#include <modm/platform/core/rom.hpp>


namespace modm::platform
{

static constexpr uint8_t FLASH_BLOCK_ERASE_CMD = 0xd8;


void Flash::eraseSectors(size_t startSertor, size_t count) {
	// rom_connect_internal_flash_fn connect_internal_flash_func = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    // rom_flash_exit_xip_fn flash_exit_xip_func = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    // rom_flash_range_erase_fn flash_range_erase_func = (rom_flash_range_erase_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_ERASE);
    // rom_flash_flush_cache_fn flash_flush_cache_func = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    
    // flash_init_boot2_copyout();
    // flash_hardware_save_state_t state;
    // flash_save_hardware_state(&state);

    // // No flash accesses after this point
    // __compiler_memory_barrier();

    // connect_internal_flash_func();
    // flash_exit_xip_func();
    // flash_range_erase_func(startSertor*SectorSize, count*SectorSize, BlockSize, FLASH_BLOCK_ERASE_CMD);
    // flash_flush_cache_func(); // Note this is needed to remove CSn IO force as well as cache flushing
    // flash_enable_xip_via_boot2();
    // flash_restore_hardware_state(&state);
}

void Flash::programPages(size_t startPage, const void* data, size_t count) {

}

uint64_t Flash::getUniqueId() {
	return 0;
}

void Flash::doCmd(const void *txbuf, void *rxbuf, size_t count) {

}

void Flash::flush() {
	auto func = ROM::flash_flush_cache::get();
	func();
}

} // namespace modm::platform
