/* stm32l4.js - STM32L4 (L41x/L42x) flash + option-byte driver for webstlink.
 *
 * NOT part of upstream webstlink (its L4 groups ship with flash_driver:null). Added for
 * RS41-NFW to flash and unlock the RSM4x4 / RSM4x5 (STM32L412RBT6) boards from the browser.
 * Written against RM0394 (STM32L41x/L42x/L43x/... reference manual).
 *
 * Registers (FLASH base 0x40022000): KEYR 0x08, OPTKEYR 0x0C, SR 0x10, CR 0x14,
 * OPTR 0x20, WRP1AR 0x2C, WRP1BR 0x30. Flash is programmed 64 bits at a time; pages
 * are 2 KB; L41x/L42x are single bank.
 *
 * SAFETY: option-byte code here only ever sets RDP = 0xAA (level 0). It refuses to write
 * 0xCC (level 2), which is PERMANENT and would brick the MCU. See unlock_option_bytes().
 */

import { Exception } from './stlinkex.js';
import { Stm32 } from './stm32.js';
import { hex_word as H32, async_sleep } from './util.js';

const FLASH_BASE   = 0x40022000;
const FLASH_KEYR   = FLASH_BASE + 0x08;
const FLASH_OPTKEYR= FLASH_BASE + 0x0c;
const FLASH_SR     = FLASH_BASE + 0x10;
const FLASH_CR     = FLASH_BASE + 0x14;
const FLASH_OPTR   = FLASH_BASE + 0x20;
const FLASH_WRP1AR = FLASH_BASE + 0x2c;
const FLASH_WRP1BR = FLASH_BASE + 0x30;

const KEY1 = 0x45670123, KEY2 = 0xcdef89ab;       // FLASH unlock
const OPTKEY1 = 0x08192a3b, OPTKEY2 = 0x4c5d6e7f; // option-byte unlock

const CR_PG        = (1 << 0);
const CR_PER       = (1 << 1);
const CR_MER1      = (1 << 2);
const CR_PNB_SHIFT = 3;
const CR_PNB_MASK  = (0x3f << 3);
const CR_STRT      = (1 << 16);
const CR_OPTSTRT   = (1 << 17);
const CR_OBL_LAUNCH= (1 << 27);
const CR_OPTLOCK   = (1 << 30);
const CR_LOCK      = (1 << 31);

const SR_BSY   = (1 << 16);
const SR_EOP   = (1 << 0);
const SR_ERR_MASK = (1<<1)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<9)|(1<<14)|(1<<15);

const PAGE_SIZE = 2048;

const RDP_LEVEL0 = 0xaa;
const RDP_LEVEL2 = 0xcc;   // PERMANENT - never write this

async function wait_busy(stlink, timeout_s = 2.0) {
    const end = Date.now() + timeout_s * 1000;
    for (;;) {
        const sr = await stlink.get_debugreg32(FLASH_SR);
        if (!(sr & SR_BSY)) {
            if (sr & SR_ERR_MASK) {
                await stlink.set_debugreg32(FLASH_SR, sr & (SR_ERR_MASK | SR_EOP)); // clear flags
                throw new Exception('FLASH error, SR=0x' + H32(sr));
            }
            if (sr & SR_EOP) await stlink.set_debugreg32(FLASH_SR, SR_EOP); // clear EOP
            return sr;
        }
        if (Date.now() > end) throw new Exception('FLASH operation timeout (SR=0x' + H32(sr) + ')');
        await async_sleep(0.002);
    }
}

class Stm32L4 extends Stm32 {

    async _unlock_flash() {
        await this.core_reset_halt();
        let cr = await this._stlink.get_debugreg32(FLASH_CR);
        if (cr & CR_LOCK) {
            await this._stlink.set_debugreg32(FLASH_KEYR, KEY1);
            await this._stlink.set_debugreg32(FLASH_KEYR, KEY2);
            cr = await this._stlink.get_debugreg32(FLASH_CR);
            if (cr & CR_LOCK) throw new Exception('Error unlocking FLASH (still locked)');
        }
        // clear any stale status flags
        await this._stlink.set_debugreg32(FLASH_SR, SR_ERR_MASK | SR_EOP);
    }

    async _lock_flash() {
        await this._stlink.set_debugreg32(FLASH_CR, CR_LOCK);
    }

    async _erase_page(page) {
        let cr = CR_PER | ((page << CR_PNB_SHIFT) & CR_PNB_MASK);
        await this._stlink.set_debugreg32(FLASH_CR, cr);
        await this._stlink.set_debugreg32(FLASH_CR, cr | CR_STRT);
        await wait_busy(this._stlink, 2.0);
        await this._stlink.set_debugreg32(FLASH_CR, 0);   // clear PER
    }

    async flash_erase_all() {
        this._dbg.debug('Stm32L4.flash_erase_all()');
        await this._unlock_flash();
        await this._stlink.set_debugreg32(FLASH_CR, CR_MER1);
        await this._stlink.set_debugreg32(FLASH_CR, CR_MER1 | CR_STRT);
        await wait_busy(this._stlink, 30.0);
        await this._stlink.set_debugreg32(FLASH_CR, 0);
        await this._lock_flash();
    }

    async flash_write(addr, data, { erase = false, verify = false } = {}) {
        this._dbg.debug(`Stm32L4.flash_write(0x${H32(addr)}, ${data.length}B, erase=${erase}, verify=${verify})`);
        if (addr === null || addr === undefined) addr = this.FLASH_START;
        if (addr % 8) throw new Exception('L4 flash address must be 8-byte aligned');

        // pad to a whole number of 64-bit double words
        if (data.length % 8) {
            const padded = new Uint8Array(data.length + (8 - (data.length % 8)));
            padded.set(data); padded.fill(0xff, data.length);
            data = padded;
        }

        await this._unlock_flash();

        if (erase) {
            const first = Math.floor((addr - this.FLASH_START) / PAGE_SIZE);
            const last  = Math.floor((addr - this.FLASH_START + data.length - 1) / PAGE_SIZE);
            this._dbg.bargraph_start('Erasing FLASH', { value_min: first, value_max: last });
            for (let p = first; p <= last; p++) {
                this._dbg.bargraph_update({ value: p });
                await this._erase_page(p);
            }
            this._dbg.bargraph_done();
        }

        // program 64 bits at a time
        this._dbg.bargraph_start('Writing FLASH', { value_min: addr, value_max: addr + data.length });
        await this._stlink.set_debugreg32(FLASH_CR, CR_PG);
        for (let off = 0; off < data.length; off += 8) {
            const dw = data.slice(off, off + 8);
            if (!dw.every(b => b === 0xff)) {          // skip already-erased double words
                await this._stlink.set_mem32(addr + off, dw);
                await wait_busy(this._stlink, 1.0);
            }
            if ((off & 0x3ff) === 0) this._dbg.bargraph_update({ value: addr + off });
        }
        await this._stlink.set_debugreg32(FLASH_CR, 0);   // clear PG
        this._dbg.bargraph_done();

        if (verify) {
            this._dbg.bargraph_start('Verifying FLASH', { value_min: addr, value_max: addr + data.length });
            for (let off = 0; off < data.length; off += this._stlink.maximum_transfer_size) {
                const n = Math.min(this._stlink.maximum_transfer_size, data.length - off);
                const got = new Uint8Array((await this._stlink.get_mem32(addr + off, (n + 3) & ~3)).buffer);
                for (let i = 0; i < n; i++) {
                    if (got[i] !== data[off + i]) {
                        throw new Exception('Verify error at 0x' + H32(addr + off + i));
                    }
                }
                this._dbg.bargraph_update({ value: addr + off });
            }
            this._dbg.bargraph_done();
        }

        await this._lock_flash();
    }

    // ---- Option bytes: read-out protection (RDP) and write protection (WRP) ----

    async read_protection() {
        const optr = await this._stlink.get_debugreg32(FLASH_OPTR);
        const rdp = optr & 0xff;
        const wrpa = await this._stlink.get_debugreg32(FLASH_WRP1AR);
        const wrpb = await this._stlink.get_debugreg32(FLASH_WRP1BR);
        // A WRP zone protects real flash only when start <= end AND the start page exists
        // (L412RB has 64 pages). This avoids reading the "disabled" default (start page far
        // past the flash, so start > end) as active and triggering a needless mass erase.
        const MAX_PAGE = 63;
        const zone_active = (w) => {
            const strt = w & 0x7f, end = (w >> 16) & 0x7f;
            return (strt <= end) && (strt <= MAX_PAGE);
        };
        const wrp_active = zone_active(wrpa) || zone_active(wrpb);
        return { rdp, rdp_locked: (rdp !== RDP_LEVEL0), wrp_active, level2: (rdp === RDP_LEVEL2) };
    }

    async _unlock_options() {
        let cr = await this._stlink.get_debugreg32(FLASH_CR);
        if (cr & CR_OPTLOCK) {
            await this._stlink.set_debugreg32(FLASH_OPTKEYR, OPTKEY1);
            await this._stlink.set_debugreg32(FLASH_OPTKEYR, OPTKEY2);
            cr = await this._stlink.get_debugreg32(FLASH_CR);
            if (cr & CR_OPTLOCK) throw new Exception('Error unlocking option bytes');
        }
    }

    // Clear RDP (to level 0) and all write protection, then reload the option bytes.
    // Reloading resets the MCU (and mass-erases when RDP drops from level 1), so after this
    // the debug link is gone: the caller must re-attach. Returns when the request is issued.
    async unlock_option_bytes() {
        const before = await this.read_protection();
        this._dbg.info(`L4 option bytes: RDP=0x${before.rdp.toString(16)} (level ${before.rdp === RDP_LEVEL0 ? 0 : (before.level2 ? 2 : 1)}), write-protection ${before.wrp_active ? 'ON' : 'off'}`);
        if (before.level2) {
            throw new Exception('This MCU is at RDP level 2 (permanent). It cannot be unlocked by any tool.');
        }

        this._dbg.info('Unlocking FLASH and option bytes...');
        await this._unlock_flash();
        await this._unlock_options();

        // read-modify-write OPTR: set only the RDP byte to level 0, keep everything else.
        let optr = await this._stlink.get_debugreg32(FLASH_OPTR);
        const new_optr = (optr & 0xffffff00) | RDP_LEVEL0;
        if ((new_optr & 0xff) === RDP_LEVEL2) {           // belt-and-braces guard
            throw new Exception('Refusing to write RDP level 2');
        }
        this._dbg.info('Setting read-out protection to level 0 (0xAA)...');
        await this._stlink.set_debugreg32(FLASH_OPTR, new_optr);

        // disable both write-protection zones (empty range: start > end)
        this._dbg.info('Clearing write-protection zones WRP1A / WRP1B...');
        await this._stlink.set_debugreg32(FLASH_WRP1AR, 0x000000ff);
        await this._stlink.set_debugreg32(FLASH_WRP1BR, 0x000000ff);

        // commit the option bytes
        await wait_busy(this._stlink, 2.0);
        let cr = await this._stlink.get_debugreg32(FLASH_CR);
        await this._stlink.set_debugreg32(FLASH_CR, cr | CR_OPTSTRT);
        await wait_busy(this._stlink, 2.0);

        // reload option bytes -> system reset + mass erase (link drops here)
        this._dbg.info('Reloading option bytes (OBL_LAUNCH): the chip resets and mass-erases now.');
        cr = await this._stlink.get_debugreg32(FLASH_CR);
        try {
            await this._stlink.set_debugreg32(FLASH_CR, cr | CR_OBL_LAUNCH);
        } catch (e) {
            // OBL_LAUNCH resets the target mid-transaction; a USB error here is expected.
        }
        return { erased: before.rdp_locked };
    }
}

export { Stm32L4 };
