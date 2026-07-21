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
 * Detecting a locked sonde is itself awkward, and getting it wrong is what used to make a
 * never-unlocked RSM4x2 report "no sonde detected":
 *   - webstlink identifies the part partly from the flash size register, which lives in the
 *     flash information block. An RDP-locked STM32F100 will not show that block to the
 *     debugger, so the read returns 0x0000/0xFFFF and identification fails even though the
 *     MCU is answering perfectly well. webstlink.js now falls back to the smallest variant of
 *     the detected dev_id and flags flash_size_unknown, so we can still reach the unlock. The
 *     L412 on an RSM4x4 keeps that register readable under RDP1, which is why only the older
 *     boards ever hit this.
 *   - a factory sonde boots into the Vaisala firmware, which can drop into a low-power mode
 *     where SWD stops answering. If the first detect fails we retry with the sonde held in
 *     reset (connect_under_reset) so that firmware never gets to run.
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

const errText = e => ((e && e.message) ? e.message : String(e));

// Identify the MCU, retrying with the sonde held in reset if the straight attempt fails.
// A factory sonde is running the Vaisala firmware, which can drop into a low-power mode where
// SWD stops answering; holding NRST low while we attach wins that race. Returns the info object
// from detect_cpu.
async function detectSonde(stlink, logger) {
  try {
    return await stlink.detect_cpu([], null);
  } catch (first) {
    logger.warning('Could not identify the sonde on the first try (' + errText(first)
      + '). Retrying with the sonde held in reset...');
    try {
      await stlink.connect_under_reset();
      return await stlink.detect_cpu([], null);
    } catch (second) {
      // Report whichever failure is more informative. If the reset path failed for its own
      // reason (an ST-Link too old for the NRST command, say), the first error is the real one.
      const err = new Error('Could not identify the MCU over SWD, with the sonde running and with it '
        + 'held in reset. Check the ST-Link wiring (SWDIO, SWCLK, GND, and NRST if your adapter has '
        + 'it) and that the sonde is powered. (' + errText(first) + ' / then ' + errText(second) + ')');
      err.cause = first;
      throw err;
    }
  }
}

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
  // Verbosity 2 so the log carries CPUID / IDCODE / flash size. On a locked sonde those lines
  // are the difference between "it is protected" and "the wiring is wrong", and the user is
  // the only one who can see them.
  const logger = new libstlink.Logger(2, logEl || null);

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
  let unlockAttempted = false;
  for (let pass = 1; pass <= 2; pass++) {
    const stlink = new WebStlink(logger);
    state(pass === 1 ? 'Connecting to the ST-Link...' : 'Re-connecting after unlock...');
    await stlink.attach(device, logger);
    let didUnlock = false;
    try {
      state('Detecting the sonde MCU...');
      const info = await detectSonde(stlink, logger);

      const types = (stlink._mcus || []).map(m => m.type).join('/');
      if (!wanted.some(w => types.indexOf(w) !== -1)) {
        throw new Error('The connected MCU (' + (types || 'unknown') + ') does not match the selected board '
          + platform + '. Pick the board that matches your sonde.');
      }

      // Clear protection if present, so we always program a fully open MCU. This runs on
      // either pass: on a locked sonde the flash size register is unreadable, so identification
      // above is only provisional and the protection registers are the reliable signal.
      const drv = stlink._driver;
      const prot = (drv && typeof drv.read_protection === 'function')
        ? await drv.read_protection() : null;

      if (prot && prot.level2) {
        throw new Error('This MCU is at RDP level 2, which is permanent. It cannot be unlocked by any tool.');
      }
      if (prot && (prot.rdp_locked || prot.wrp_active) && !unlockAttempted) {
        state('Factory-locked sonde detected. Clearing read-out and write protection - this mass-erases the chip...');
        unlockAttempted = true;
        await drv.unlock_option_bytes();
        // Make the new option bytes actually take effect. The L4 path ends with OBL_LAUNCH,
        // which reloads them itself, but the F1 has no OBL_LAUNCH: its option byte loader runs
        // at reset, and the core-level SYSRESETREQ that unlock_option_bytes ends with does not
        // reliably re-run it. Pulsing NRST does.
        try {
          await stlink._stlink.drive_nrst('pulse');
        } catch (e) {
          logger.warning('Could not pulse NRST after unlocking (' + errText(e) + '). If the sonde '
            + 'still reports as protected, power-cycle it and flash again.');
        }
        logger.info('Sonde unlocked and erased successfully (read-out + write protection cleared).');
        didUnlock = true;
      } else if (prot && (prot.rdp_locked || prot.wrp_active)) {
        // Second pass and still protected: the option bytes did not take. Flashing now would
        // silently fail to write the protected sectors, so stop instead of claiming success.
        throw new Error('The sonde is still protected after the unlock attempt (read-out protection '
          + (prot.rdp_locked ? 'ON' : 'off') + ', write protection ' + (prot.wrp_active ? 'ON' : 'off')
          + '). Power-cycle the sonde and try again, or clear it with STM32CubeProgrammer - see the '
          + 'flashing guide.');
      }

      if (!didUnlock) {
        // Identification is only provisional while the flash information block is unreadable.
        // That is expected on a locked chip and resolves itself after the unlock, but if we get
        // here with it unresolved we do not know the real flash size, so we must not program.
        if (info && info.flash_size_unknown) {
          throw new Error('The sonde reports no readable flash size, but its protection registers read '
            + 'as unlocked. Refusing to flash a part of unknown size. Power-cycle the sonde and try '
            + 'again, or use STM32CubeProgrammer - see the flashing guide.');
        }
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
