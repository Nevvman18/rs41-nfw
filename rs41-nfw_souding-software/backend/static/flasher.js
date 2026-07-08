/* flasher.js - RS41-NFW browser flashing over WebUSB.
 *
 * Drives an ST-Link (WebUSB) to write the compiled firmware to the sonde, using the
 * vendored webstlink library (Devan Lai, MPL-2.0) under vendor/webstlink/. This module
 * only wires that library to the Firmware Builder UI; it does not reimplement the
 * ST-Link protocol.
 *
 * Board support: RSM4x2 / RSM4x1 (STM32F100) via webstlink's STM32FP driver, and
 * RSM4x4 / RSM4x5 (STM32L412) via the STM32L4 driver added for RS41-NFW under
 * vendor/webstlink/lib/stm32l4.js (upstream webstlink has none). See fw/FLASHING.md
 * (Method D).
 *
 * Factory-locked sondes: if read-out or write protection is set, this clears it first
 * (unlock_option_bytes on the driver), which mass-erases and resets the MCU; it then
 * re-attaches and flashes the now-open chip - so a brand-new sonde no longer needs a
 * desktop tool. The STM32L4 unlock only ever writes RDP level 0 and hard-refuses level 2
 * (the permanent lock). A failed SWD flash is recoverable: SWD stays accessible, so any
 * method can re-flash.
 *
 * EXPERIMENTAL: this option-byte / L4 code has been written against the reference manuals
 * but validate it on a spare sonde before relying on it.
 */

import WebStlink from './vendor/webstlink/webstlink.js';
import * as libstlink from './vendor/webstlink/lib/package.js';

const FLASH_BASE = 0x08000000;

// MCU types we accept per board (matched against webstlink's detected type string).
const BOARD_MCU = {
  RSM4x2: ['STM32F100'],            // old F100 boards (STM32FP driver)
  RSM4x1: ['STM32F100'],
  RSM4x4: ['STM32L412', 'STM32L4'], // new L412 boards (STM32L4 driver added for RS41-NFW)
  RSM4x5: ['STM32L412', 'STM32L4'],
};

function boardSupportedInBrowser(platform) {
  return platform in BOARD_MCU;
}

const sleep = ms => new Promise(r => setTimeout(r, ms));

// Flash the given board's just-built firmware. opts:
//   platform : 'RSM4x2' etc.
//   binUrl   : URL to fetch the compiled .bin from (same-origin)
//   logEl    : DOM element webstlink appends its log/progress to
//   onState  : (text, kind) => void   kind = 'info' | 'ok' | 'err'
async function flashFirmware({ platform, binUrl, logEl, onState }) {
  const state = (t, k) => { if (onState) onState(t, k || 'info'); };

  if (!('usb' in navigator)) {
    throw new Error('WebUSB is not available in this browser. Use desktop Chrome, Edge or Opera.');
  }
  if (!boardSupportedInBrowser(platform)) {
    throw new Error('Browser flashing is not available for board "' + platform + '". Use OpenOCD or '
      + 'STM32CubeProgrammer - see the flashing guide.');
  }

  state('Downloading the compiled firmware...');
  const resp = await fetch(binUrl);
  if (!resp.ok) throw new Error('Could not fetch the compiled firmware (' + resp.status + ').');
  const data = new Uint8Array(await resp.arrayBuffer());
  if (!data.length) throw new Error('The compiled firmware is empty.');

  if (logEl) logEl.replaceChildren();
  const logger = new libstlink.Logger(1, logEl || null);

  state('Select your ST-Link in the browser prompt...');
  let device;
  try {
    device = await navigator.usb.requestDevice({ filters: libstlink.usb.filters });
  } catch (e) {
    if (e && e.name === 'NotFoundError') {
      const err = new Error('No ST-Link selected. If it was already there, try again and pick it. '
        + 'If the picker was empty, your OS is not giving the browser access yet: open '
        + '"First-time setup" below (Zadig/WinUSB on Windows, a udev rule on Linux), then reconnect the '
        + 'ST-Link. As an alternative you can flash with a desktop tool (see the flashing guide, Method D).');
      err.openSetup = true;   // hint to the UI to expand the first-time-setup section
      throw err;
    }
    throw e;
  }

  const wanted = BOARD_MCU[platform];

  // Up to two passes: if the sonde is factory-locked we clear read-out + write protection
  // (which mass-erases and resets it) on the first pass, then re-attach and flash the open
  // MCU on the second. An unlocked sonde flashes in one pass.
  for (let pass = 1; pass <= 2; pass++) {
    const stlink = new WebStlink(logger);
    state(pass === 1 ? 'Connecting to the ST-Link...' : 'Re-connecting after unlock...');
    await stlink.attach(device, logger);
    let didUnlock = false;
    try {
      state('Detecting the sonde MCU...');
      try {
        await stlink.detect_cpu([], null);
      } catch (e) {
        throw new Error('Could not identify the MCU over SWD. Check the ST-Link wiring and that the sonde '
          + 'is powered. (' + (e && e.message ? e.message : e) + ')');
      }
      const types = (stlink._mcus || []).map(m => m.type).join('/');
      if (!wanted.some(w => types.indexOf(w) !== -1)) {
        throw new Error('The connected MCU (' + (types || 'unknown') + ') does not match the selected board '
          + platform + '. Pick the board that matches your sonde.');
      }

      // Clear protection if present, so we always program a fully open MCU.
      const drv = stlink._driver;
      if (pass === 1 && drv && typeof drv.read_protection === 'function') {
        const prot = await drv.read_protection();
        if (prot.level2) {
          throw new Error('This MCU is at RDP level 2, which is permanent. It cannot be unlocked by any tool.');
        }
        if (prot.rdp_locked || prot.wrp_active) {
          state('Factory-locked sonde detected. Clearing read-out and write protection - this mass-erases the chip...');
          await drv.unlock_option_bytes();
          logger.info('Sonde unlocked and erased successfully (read-out + write protection cleared).');
          didUnlock = true;
        }
      }

      if (!didUnlock) {
        state('Found ' + types + '. Flashing ' + data.length + ' bytes (erase, program, verify)...');
        await stlink.halt();
        await stlink.flash(FLASH_BASE, data);
        state('Resetting the sonde to start the new firmware...');
        // Issue an MCU system reset over SWD so the sonde restarts into the new firmware
        // on its own (works on both the F100 Cortex-M3 and the L412 Cortex-M4).
        try { await stlink.reset(false); } catch (e) { /* falls back to a manual power cycle */ }
        logger.info('Flash complete and verified. The sonde has been reset and should be running the new firmware.');
        state('Done. ' + data.length + ' bytes flashed and verified. The sonde was reset automatically - '
          + 'if it does not start, power-cycle it (remove and reinsert the batteries). Then disconnect the ST-Link.', 'ok');
        return;
      }
    } finally {
      try { await stlink.detach(); } catch (e) { /* ignore */ }
    }
    // reached only right after an unlock: let the target settle, then re-attach and flash
    state('Sonde unlocked and erased successfully. Re-attaching to write the firmware...');
    await sleep(1500);
  }
  throw new Error('The sonde was unlocked and erased, but could not be re-attached automatically to flash it. '
    + 'Power-cycle the sonde (and reconnect the ST-Link if needed), then click Flash again - it is now an open MCU.');
}

// Expose to the (classic-script) Firmware Builder UI.
window.nfwFlash = flashFirmware;
window.nfwFlashBoardSupported = boardSupportedInBrowser;
window.dispatchEvent(new Event('nfw-flasher-ready'));
