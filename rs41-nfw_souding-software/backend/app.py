import os, re, json, uuid, shutil, threading, queue, subprocess, zipfile, base64, secrets, time, random, sys, logging
from datetime import datetime
from pathlib import Path
from io import BytesIO
from flask import Flask, request, jsonify, send_file, send_from_directory

logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
    level=logging.INFO,
    stream=sys.stdout,
    force=True,
)

app = Flask(__name__, static_folder='static', static_url_path='')

# Cap upload size so nobody can fill the disk with a giant POST (firmware zips are small).
app.config['MAX_CONTENT_LENGTH'] = 50 * 1024 * 1024   # 50 MB


@app.errorhandler(413)
def _too_large(e):
    # Flask aborts the request before our route runs, so without this the client
    # gets an HTML page and the upload looks like it silently did nothing.
    return jsonify(ok=False, error='Zip is larger than the 50 MB upload limit. '
                                   'Zip only the firmware folder, without .git or build output.'), 413

MAX_QUEUE = 20   # refuse new builds past this, so the queue can't be flooded

# Set ALLOW_ZIP_UPLOAD=0 in docker-compose before publishing to leave only GitHub fetch.
ALLOW_ZIP_UPLOAD = os.environ.get('ALLOW_ZIP_UPLOAD', '1').strip().lower() in ('1', 'true', 'yes', 'on')

# Optional analytics: set ANALYTICS_HEAD to a raw HTML snippet (e.g. an Umami
# <script> tag) and it is injected into the page <head>. Empty = nothing injected.
ANALYTICS_HEAD = os.environ.get('ANALYTICS_HEAD', '').strip()

# ─── lightweight anti-bot gate ───────────────────────────────────────────────────
# A retype-the-code challenge. The answer never leaves the server, only the image
# does. Solving it issues a short-lived token that the expensive endpoints require.
# This is a basic layer on top of the Cloudflare protection in front of the site.
CAPTCHA_CHARS = 'ABCDEFGHJKLMNPQRSTUVWXYZ23456789'   # no ambiguous 0/O/1/I/L
CAPTCHA_TTL   = 300            # seconds a challenge stays solvable
AUTH_TTL      = 3 * 3600       # seconds an unlocked session stays valid
_captchas     = {}            # token -> {'a': answer, 'e': expiry}
_auth_tokens  = {}            # token -> expiry
_captcha_lock = threading.Lock()


def _purge_tokens():
    now = time.time()
    with _captcha_lock:
        for k in [k for k, v in _captchas.items() if v['e'] < now]:
            _captchas.pop(k, None)
        for k in [k for k, e in _auth_tokens.items() if e < now]:
            _auth_tokens.pop(k, None)


def _render_captcha(text):
    """Return a data: URL for the challenge image. A real raster PNG via Pillow when
    available (genuinely non-copyable, no text in the DOM); an SVG fallback otherwise."""
    try:
        from PIL import Image, ImageDraw, ImageFont, ImageFilter
        W, H = 210, 70
        img = Image.new('RGB', (W, H), (15, 23, 42))
        d = ImageDraw.Draw(img)
        for _ in range(6):
            d.line([(random.randint(0, W), random.randint(0, H)),
                    (random.randint(0, W), random.randint(0, H))],
                   fill=(random.randint(40, 90),) * 3, width=1)
        try:
            font = ImageFont.load_default(size=40)
        except TypeError:
            font = ImageFont.load_default()
        x = 16
        for ch in text:
            g = Image.new('RGBA', (42, 58), (0, 0, 0, 0))
            ImageDraw.Draw(g).text(
                (6, 4), ch, font=font,
                fill=(random.randint(120, 200), random.randint(160, 220), random.randint(200, 255)))
            g = g.rotate(random.uniform(-28, 28), expand=1, resample=Image.BICUBIC)
            img.paste(g, (x, random.randint(0, 12)), g)
            x += 31
        for _ in range(200):
            img.putpixel((random.randint(0, W - 1), random.randint(0, H - 1)),
                         (random.randint(60, 120),) * 3)
        img = img.filter(ImageFilter.SMOOTH)
        buf = BytesIO(); img.save(buf, 'PNG')
        return 'data:image/png;base64,' + base64.b64encode(buf.getvalue()).decode()
    except Exception:
        parts = []
        for i, ch in enumerate(text):
            cx = 20 + i * 30
            parts.append(f'<text x="{cx}" y="48" fill="#bcd6f5" font-size="40" '
                         f'font-family="monospace" transform="rotate({random.uniform(-22,22):.0f} {cx} 40)">{ch}</text>')
        svg = ('<svg xmlns="http://www.w3.org/2000/svg" width="210" height="70">'
               '<rect width="210" height="70" fill="#0f172a"/>' + ''.join(parts) + '</svg>')
        return 'data:image/svg+xml;base64,' + base64.b64encode(svg.encode()).decode()


def _is_verified():
    tok = request.headers.get('X-NFW-Auth', '')
    with _captcha_lock:
        exp = _auth_tokens.get(tok)
    return bool(exp and exp >= time.time())


@app.after_request
def _security_headers(resp):
    resp.headers['X-Content-Type-Options'] = 'nosniff'
    resp.headers['X-Frame-Options'] = 'DENY'
    resp.headers['Referrer-Policy'] = 'no-referrer'
    return resp


# Mounted volumes
WORKSPACE = Path(os.environ.get('WORKSPACE_DIR', '/workspace'))
LOGS_DIR  = Path(os.environ.get('LOGS_DIR', '/logs'))

REPOS_DIR  = WORKSPACE / 'repos'    # git clones and uploads
SKETCH_DIR = WORKSPACE / 'sketch'   # one sketch dir per board, kept around so builds are incremental
BUILD_DIR  = WORKSPACE / 'build'    # arduino-cli build path, this is where the compile cache lives
TMP_DIR    = WORKSPACE / 'tmp'      # scratch output, wiped after each build
SONDES_DIR = Path(os.environ.get('SONDES_DIR', str(WORKSPACE / 'sondes')))  # per-serial factory cal library


GITHUB_REPO = 'https://github.com/Nevvman18/rs41-nfw'

# Optimization (STM32 core "opt" menu):
#   RSM4x2 (F100, 64 KB flash) -> oslto = "Smallest (-Os) with LTO". The plain -Os
#     build overflows by ~4.4 KB and ozone is only ~2-3 KB, so LTO is what actually
#     makes it fit (it gives ~8 KB headroom). Safe here: the one LTO-surfaced bug
#     (a 1-byte overflow in the Horus V3 encoder) is fixed.
#   RSM4x4 (L412, 128 KB flash) -> osstd = plain "Smallest (-Os)", no LTO needed.
FQBN = {
    'RSM4x2': 'STMicroelectronics:stm32:GenF1:pnum=GENERIC_F100C8TX,opt=oslto',
    'RSM4x4': 'STMicroelectronics:stm32:GenL4:pnum=GENERIC_L412RBTXP,opt=osstd',
}

jobs = {}
firmware_state = {'source': None, 'path': None, 'l412_installed': False}

# Persist which firmware is loaded (github vs uploaded zip) across server/container
# restarts. The fetched repo / extracted zip live on the workspace volume, so after
# a restart the files are still there - only this in-memory marker was being lost,
# which made the UI show "No firmware loaded" until you re-fetched. Keep it on disk.
STATE_FILE = WORKSPACE / 'firmware_state.json'

def _save_state():
    try:
        STATE_FILE.write_text(json.dumps({
            'source':         firmware_state.get('source'),
            'path':           firmware_state.get('path'),
            'l412_installed': firmware_state.get('l412_installed', False),
        }))
    except OSError:
        pass   # best-effort, never break a request over this

def _load_state():
    try:
        d = json.loads(STATE_FILE.read_text())
    except (OSError, ValueError):
        return
    # Only trust the marker if the firmware it points at is still on disk.
    p = d.get('path')
    if p and Path(p).exists():
        firmware_state.update(
            source=d.get('source'),
            path=p,
            l412_installed=bool(d.get('l412_installed', False)),
        )

# One worker pulls from this queue, so builds run one at a time. That keeps the
# CPU sane and lets us safely reuse a single build cache per board.
compile_queue = queue.Queue()
queue_lock    = threading.Lock()
queue_order   = []   # job ids still queued or running, oldest first


# ─── helpers ───────────────────────────────────────────────────────────────────

def get_stm32_core_path():
    for base in [Path.home() / '.arduino15', Path('/root/.arduino15')]:
        try:
            hw = base / 'packages' / 'STMicroelectronics' / 'hardware' / 'stm32'
            if hw.exists():
                versions = sorted(hw.iterdir())
                if versions:
                    return versions[-1]
        except (PermissionError, OSError):
            continue
    # Fall back to asking arduino-cli (handles different config formats)
    try:
        r = subprocess.run(['arduino-cli', 'config', 'dump', '--format', 'json'],
                           capture_output=True, text=True, timeout=10)
        cfg = json.loads(r.stdout)
        # arduino-cli 0.x: cfg["data_dir"], 1.x: cfg["config"]["directories"]["data"]
        data_dir = (
            cfg.get('config', {}).get('directories', {}).get('data') or
            cfg.get('directories', {}).get('data') or
            cfg.get('data_dir') or
            '/root/.arduino15'
        )
        hw = Path(data_dir) / 'packages' / 'STMicroelectronics' / 'hardware' / 'stm32'
        if hw.exists():
            versions = sorted(hw.iterdir())
            if versions:
                return versions[-1]
    except Exception:
        pass
    return None


def install_l412_variant(repo_path: Path):
    core = get_stm32_core_path()
    if not core:
        return False, 'STM32 core not found'

    src = repo_path / 'fw' / 'arduino-ide_variant-files' / 'variants' / 'STM32L4xx' / 'L412RB(I-T)xP'
    dst = core / 'variants' / 'STM32L4xx' / 'L412RB(I-T)xP'
    if not src.exists():
        return False, f'Variant source not found: {src}'
    shutil.copytree(src, dst, dirs_exist_ok=True)

    boards_src = repo_path / 'fw' / 'arduino-ide_variant-files' / 'boards.txt'
    boards_dst = core / 'boards.txt'
    if boards_src.exists() and boards_dst.exists():
        entry = boards_src.read_text()
        existing = boards_dst.read_text()
        if 'GENERIC_L412RBTXP' not in existing:
            with open(boards_dst, 'a') as f:
                f.write('\n' + entry)

    firmware_state['l412_installed'] = True
    _save_state()
    return True, 'L412 variant installed'


def read_ino(fw_path: Path):
    p = fw_path / 'rs41-nfw_sonde-firmware.ino'
    return p.read_text(encoding='utf-8') if p.exists() else None


def read_config_source(fw_path: Path):
    """Which file holds the live settings depends on the firmware version.
    v66 and later keep them in CONFIG.h (the .ino copy is fenced off in #if 0),
    while v65 and older keep everything inline in the .ino. Prefer CONFIG.h when
    it exists so we always read the values that actually get compiled."""
    cfg_h = fw_path / 'CONFIG.h'
    if cfg_h.exists():
        return cfg_h.read_text(encoding='utf-8')
    return read_ino(fw_path)


def parse_config(ino: str) -> dict:
    cfg = {}

    # platform
    if re.search(r'^#define RSM4x4\b', ino, re.MULTILINE):
        cfg['_platform'] = 'RSM4x4'
    elif re.search(r'^#define RSM4x2\b', ino, re.MULTILINE):
        cfg['_platform'] = 'RSM4x2'
    else:
        cfg['_platform'] = 'none'

    # NFW version
    m = re.search(r'#define NFW_VERSION\s+"([^"]*)"', ino)
    if m:
        cfg['_nfw_version'] = m.group(1)

    # #define string macros
    for m in re.finditer(r'^#define\s+(CALLSIGN|HORUS_V3_CALLSIGN)\s+"([^"]*)"', ino, re.MULTILINE):
        cfg[m.group(1)] = m.group(2)

    # bool
    for m in re.finditer(r'^(?:constexpr\s+)?bool\s+(\w+)\s*=\s*(true|false)\s*;', ino, re.MULTILINE):
        cfg[m.group(1)] = (m.group(2) == 'true')

    # typed numeric (constexpr or plain)
    for m in re.finditer(
        r'^(?:constexpr\s+)?(?:uint8_t|uint16_t|uint32_t|int8_t|int16_t|float|double)\s+(\w+)\s*=\s*(-?[\d.]+(?:e[+-]?\d+)?)\s*;',
        ino, re.MULTILINE
    ):
        val = m.group(2)
        try:
            cfg[m.group(1)] = float(val) if ('.' in val or 'e' in val.lower()) else int(val)
        except Exception:
            pass

    # unsigned long
    for m in re.finditer(r'^unsigned long\s+(\w+)\s*=\s*(\d+)\s*;', ino, re.MULTILINE):
        try:
            cfg[m.group(1)] = int(m.group(2))
        except Exception:
            pass

    # unsigned int (e.g. dataRecorderInterval)
    for m in re.finditer(r'^(?:constexpr\s+)?unsigned int\s+(\w+)\s*=\s*(\d+)\s*;', ino, re.MULTILINE):
        try:
            cfg[m.group(1)] = int(m.group(2))
        except Exception:
            pass

    # char scalar holding a number (e.g. aprsSsid = 11); ignores char-literal 'O'
    for m in re.finditer(r'^(?:constexpr\s+)?char\s+(\w+)\s*=\s*(-?\d+)\s*;', ino, re.MULTILINE):
        try:
            cfg[m.group(1)] = int(m.group(2))
        except Exception:
            pass

    # plain float
    for m in re.finditer(r'^float\s+(\w+)\s*=\s*(-?[\d.]+)\s*;', ino, re.MULTILINE):
        try:
            cfg[m.group(1)] = float(m.group(2))
        except Exception:
            pass

    # plain uint8_t / int without constexpr
    for m in re.finditer(r'^uint8_t\s+(\w+)\s*=\s*(\d+)\s*;', ino, re.MULTILINE):
        try:
            cfg[m.group(1)] = int(m.group(2))
        except Exception:
            pass

    # char scalar holding a character literal (e.g. aprsSymbolOverlay = 'O')
    for m in re.finditer(r"^(?:constexpr\s+)?char\s+(\w+)\s*=\s*'(.)'\s*;", ino, re.MULTILINE):
        cfg[m.group(1)] = m.group(2)

    # char[] strings
    for m in re.finditer(r'^(?:constexpr\s+)?char\s+(\w+)\[\]\s*=\s*"([^"]*)"', ino, re.MULTILINE):
        cfg[m.group(1)] = m.group(2)

    # String variables
    for m in re.finditer(r'^String\s+(\w+)\s*=\s*"([^"]*)"', ino, re.MULTILINE):
        cfg[m.group(1)] = m.group(2)

    # frequency tables: read the full list so multi-frequency cycling is editable
    for m in re.finditer(
        r'^(?:constexpr\s+)?float\s+(\w+FreqTable)\[\]\s*=\s*\{([^}]+)\}', ino, re.MULTILINE
    ):
        vals = []
        for v in m.group(2).split(','):
            v = v.strip()
            if not v:
                continue
            try:
                vals.append(float(v))
            except Exception:
                pass
        cfg[m.group(1)] = vals if vals else [437.6]

    return cfg


def _fmt_num(v):
    if isinstance(v, float):
        # Whole-number floats print as plain integers (750.0 -> "750"), matching
        # the old behaviour. Everything else uses repr() so it round-trips at full
        # precision - factory calibration coefficients carry tiny terms like
        # 8.2e-06 that a fixed 6-decimal format would silently truncate.
        if v.is_integer() and abs(v) < 1e16:
            return str(int(v))
        return repr(v)
    return str(int(v))


def apply_config(ino: str, user_cfg: dict, platform: str) -> str:
    c = ino

    # platform defines
    c = re.sub(r'^(// )?#define RSM4x4\b[^\n]*', '// #define RSM4x4 // New PCB versions (STM32L412RBT6)', c, flags=re.MULTILINE)
    c = re.sub(r'^(// )?#define RSM4x2\b[^\n]*', '// #define RSM4x2  // Old PCB versions (STM32F100C8T6B)', c, flags=re.MULTILINE)
    if platform == 'RSM4x4':
        c = re.sub(r'^// #define RSM4x4\b', '#define RSM4x4', c, flags=re.MULTILINE, count=1)
    elif platform == 'RSM4x2':
        c = re.sub(r'^// #define RSM4x2\b', '#define RSM4x2', c, flags=re.MULTILINE, count=1)

    for key, value in user_cfg.items():
        if key.startswith('_'):
            continue

        if key in ('CALLSIGN', 'HORUS_V3_CALLSIGN'):
            c = re.sub(
                rf'^(#define\s+{re.escape(key)}\s+)"[^"]*"',
                rf'\g<1>"{value}"', c, flags=re.MULTILINE, count=1)
            continue

        if isinstance(value, bool):
            vs = 'true' if value else 'false'
            c = re.sub(
                rf'^((?:constexpr\s+)?bool\s+{re.escape(key)}\s*=\s*)(true|false)(\s*;)',
                rf'\g<1>{vs}\3', c, flags=re.MULTILINE, count=1)

        elif isinstance(value, (int, float)):
            vs = _fmt_num(value)
            # constexpr typed / plain typed
            c = re.sub(
                rf'^((?:constexpr\s+)?(?:uint8_t|uint16_t|uint32_t|int8_t|int16_t|float|double)\s+{re.escape(key)}\s*=\s*)-?[\d.]+(?:e[+-]?\d+)?(\s*;)',
                rf'\g<1>{vs}\2', c, flags=re.MULTILINE, count=1)
            # unsigned long
            c = re.sub(
                rf'^(unsigned long\s+{re.escape(key)}\s*=\s*)\d+(\s*;)',
                rf'\g<1>{vs}\2', c, flags=re.MULTILINE, count=1)
            # unsigned int
            c = re.sub(
                rf'^((?:constexpr\s+)?unsigned int\s+{re.escape(key)}\s*=\s*)\d+(\s*;)',
                rf'\g<1>{vs}\2', c, flags=re.MULTILINE, count=1)
            # char scalar holding a number (e.g. aprsSsid)
            c = re.sub(
                rf'^((?:constexpr\s+)?char\s+{re.escape(key)}\s*=\s*)-?\d+(\s*;)',
                rf'\g<1>{vs}\2', c, flags=re.MULTILINE, count=1)
            # plain float
            c = re.sub(
                rf'^(float\s+{re.escape(key)}\s*=\s*)-?[\d.]+(\s*;)',
                rf'\g<1>{vs}\2', c, flags=re.MULTILINE, count=1)
            # plain uint8_t
            c = re.sub(
                rf'^(uint8_t\s+{re.escape(key)}\s*=\s*)\d+(\s*;)',
                rf'\g<1>{vs}\2', c, flags=re.MULTILINE, count=1)
            # frequency table written as a single value, e.g. {437.6}
            if 'FreqTable' in key:
                c = re.sub(
                    rf'^((?:constexpr\s+)?float\s+{re.escape(key)}\[\]\s*=\s*\{{)[^}}]+(\}})',
                    rf'\g<1>{vs}\2', c, flags=re.MULTILINE, count=1)

        elif isinstance(value, list):
            # factory humidity matrix: float factoryMatrixU[42] = { ... };
            if key == 'factoryMatrixU' and value:
                vs = ', '.join(_fmt_num(x) for x in value)
                c = re.sub(
                    rf'^((?:const\s+)?float\s+{re.escape(key)}\[\d*\]\s*=\s*\{{)[^}}]*(\}})',
                    rf'\g<1>{vs}\2', c, flags=re.MULTILINE, count=1)
                continue
            # frequency table with one or more entries, e.g. {437.6, 434.714, 433.8}
            if 'FreqTable' in key and value:
                vs = ', '.join(_fmt_num(x) for x in value)
                c = re.sub(
                    rf'^((?:constexpr\s+)?float\s+{re.escape(key)}\[\]\s*=\s*\{{)[^}}]*(\}})',
                    rf'\g<1>{vs}\2', c, flags=re.MULTILINE, count=1)

        elif isinstance(value, str):
            # char scalar literal (e.g. aprsSymbolOverlay = 'O'); one safe char only
            if len(value) == 1 and value not in ("'", '\\'):
                c = re.sub(
                    rf"^((?:constexpr\s+)?char\s+{re.escape(key)}\s*=\s*)'[^']'(\s*;)",
                    rf"\g<1>'{value}'\2", c, flags=re.MULTILINE, count=1)
            # char[]
            c = re.sub(
                rf'^((?:constexpr\s+)?char\s+{re.escape(key)}\[\]\s*=\s*)"[^"]*"',
                rf'\g<1>"{value}"', c, flags=re.MULTILINE, count=1)
            # String
            c = re.sub(
                rf'^(String\s+{re.escape(key)}\s*=\s*)"[^"]*"',
                rf'\g<1>"{value}"', c, flags=re.MULTILINE, count=1)

    return c


_KEY_RE = re.compile(r'^[A-Za-z_]\w{0,63}$')

def sanitize_config(user_cfg) -> dict:
    """Clean everything coming from the browser before it ever touches the firmware
    source. Keys must look like C identifiers. Strings cannot contain quotes,
    backslashes, newlines or other control characters: those would let someone close
    the string literal early and inject arbitrary code into the build. Numbers must
    be real numbers, and frequency lists must be plausible MHz values. Anything that
    does not fit is simply dropped rather than trusted."""
    clean = {}
    if not isinstance(user_cfg, dict):
        return clean
    for k, v in user_cfg.items():
        if not isinstance(k, str) or not _KEY_RE.match(k):
            continue
        if isinstance(v, bool):
            clean[k] = v
        elif isinstance(v, int):
            clean[k] = v
        elif isinstance(v, float):
            if v != v or v in (float('inf'), float('-inf')):
                continue
            clean[k] = v
        elif isinstance(v, str):
            if len(v) > 200 or '"' in v or '\\' in v or any(ord(ch) < 32 for ch in v):
                continue
            clean[k] = v
        elif isinstance(v, list):
            # Factory humidity matrix: exactly 42 finite floats, any magnitude.
            if k == 'factoryMatrixU':
                if len(v) == 42 and all(
                        isinstance(x, (int, float)) and not isinstance(x, bool)
                        and float(x) == float(x) and float(x) not in (float('inf'), float('-inf'))
                        for x in v):
                    clean[k] = [float(x) for x in v]
                continue
            nums = []
            for x in v[:16]:
                if isinstance(x, bool):
                    continue
                if isinstance(x, (int, float)) and 24.0 <= float(x) <= 1100.0:
                    nums.append(float(x))
            if nums:
                clean[k] = nums
    return clean


def extract_version(ino: str) -> str:
    m = re.search(r'NFW_VERSION.*?v(\d+)', ino) or re.search(r'Version\s+(\d+)', ino or '')
    return f'v{m.group(1)}' if m else 'v0'


# ─── compile worker ─────────────────────────────────────────────────────────────

def _stage_sketch(fw_path: Path, platform: str, patched_ino: str, user_cfg: dict):
    """Put the sketch into a fixed folder per board and only rewrite the two files
    that carry config (the .ino and CONFIG.h). We copy the whole firmware tree only
    when the source actually changes, so arduino-cli can reuse its cache and skip
    rebuilding the STM32 core and the other untouched sources every time.

    Returns (sketch_root, fresh) where fresh is True when we just re-copied."""
    plat_dir    = SKETCH_DIR / platform
    sketch_root = plat_dir / 'rs41-nfw_sonde-firmware'
    marker      = plat_dir / '.staged_from'

    # fingerprint the source by path + the .ino mtime so we notice a new fetch/upload
    ino_file = fw_path / 'rs41-nfw_sonde-firmware.ino'
    sig = f'{fw_path}|{ino_file.stat().st_mtime_ns}'
    fresh = (not marker.exists()) or marker.read_text() != sig or not sketch_root.exists()

    if fresh:
        if sketch_root.exists():
            shutil.rmtree(sketch_root)
        shutil.copytree(fw_path, sketch_root)
        plat_dir.mkdir(parents=True, exist_ok=True)
        marker.write_text(sig)
        # new source means the old cache is stale, throw it away
        bp = BUILD_DIR / platform
        if bp.exists():
            shutil.rmtree(bp)

    # On v66+ the live config sits in CONFIG.h. We patch the .ino too because older
    # firmware kept everything inline there.
    (sketch_root / 'rs41-nfw_sonde-firmware.ino').write_text(patched_ino, encoding='utf-8')
    config_h = sketch_root / 'CONFIG.h'
    if config_h.exists():
        patched_cfg = apply_config(config_h.read_text(encoding='utf-8'), user_cfg, platform)
        config_h.write_text(patched_cfg, encoding='utf-8')

    return sketch_root, fresh


def do_compile(job):
    job_id   = job['id']
    platform = job['platform']
    fw_path  = Path(job['fw_path'])
    user_cfg = job['config']

    def log(msg):
        job['logs'].append(msg)
        print(msg, flush=True)

    ts = lambda: datetime.now().strftime('%H:%M:%S')
    out_tmp = TMP_DIR / job_id

    try:
        job['status'] = 'running'
        log(f'[{ts()}] Compiling for {platform}...')

        ino = read_ino(fw_path)
        if not ino:
            raise RuntimeError('Cannot read .ino file')
        patched = apply_config(ino, user_cfg, platform)

        sketch_root, fresh = _stage_sketch(fw_path, platform, patched, user_cfg)
        if fresh:
            log(f'[{ts()}] Fresh source, this first build has to rebuild the core (slower).')
        else:
            log(f'[{ts()}] Reusing the build cache, only your config gets recompiled.')

        build_path = BUILD_DIR / platform
        build_path.mkdir(parents=True, exist_ok=True)
        out_tmp.mkdir(parents=True, exist_ok=True)

        cmd = [
            'arduino-cli', 'compile',
            '--fqbn', FQBN[platform],
            '--build-path', str(build_path),   # reused between builds, this is the cache
            '--output-dir', str(out_tmp),
            str(sketch_root),
        ]
        log(f'[{ts()}] {" ".join(cmd)}')

        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        for line in (proc.stdout + proc.stderr).splitlines():
            log(line)
        if proc.returncode != 0:
            raise RuntimeError(f'arduino-cli exited {proc.returncode}')

        bins = list(out_tmp.glob('*.bin'))
        if not bins:
            raise RuntimeError('No .bin in output directory')

        version = extract_version(ino)
        now = datetime.now()
        stamp_file = now.strftime('%y-%m-%d_%H-%M-%S')   # goes in front of the .bin name
        stamp_dir  = now.strftime('%Y-%m-%d_%H-%M-%S')   # name of this build's log folder
        out_name   = f'{stamp_file}_rs41-nfw_{version}_{platform}.bin'

        # keep a copy of this build: the binary, the config it was built with, and the log
        log_dir = LOGS_DIR / stamp_dir
        log_dir.mkdir(parents=True, exist_ok=True)
        final_bin = log_dir / out_name
        shutil.copyfile(bins[0], final_bin)

        cfg_src = sketch_root / 'CONFIG.h'
        if cfg_src.exists():
            shutil.copyfile(cfg_src, log_dir / 'CONFIG.h')
        else:
            shutil.copyfile(sketch_root / 'rs41-nfw_sonde-firmware.ino', log_dir / 'config.ino')

        job.update(status='done', bin_path=str(final_bin), bin_name=out_name,
                   log_dir=str(log_dir))
        log(f'[{ts()}] Done, built {out_name}')
        (log_dir / 'compile.log').write_text('\n'.join(job['logs']), encoding='utf-8')

    except Exception as e:
        job.update(status='error', error=str(e))
        log(f'[{ts()}] ERROR: {e}')
        try:
            fail_dir = LOGS_DIR / (datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '_FAILED')
            fail_dir.mkdir(parents=True, exist_ok=True)
            (fail_dir / 'compile.log').write_text('\n'.join(job['logs']), encoding='utf-8')
        except Exception:
            pass

    finally:
        # only the scratch output goes away, the cache/sketch/logs stay put
        shutil.rmtree(out_tmp, ignore_errors=True)
        with queue_lock:
            if job_id in queue_order:
                queue_order.remove(job_id)


def compile_worker():
    while True:
        job = compile_queue.get()
        try:
            do_compile(job)
        except Exception as e:
            print('Worker error:', e, flush=True)
        finally:
            compile_queue.task_done()


# ─── routes ────────────────────────────────────────────────────────────────────

@app.route('/')
def serve_index():
    # Inject the optional analytics snippet into <head> when configured; otherwise
    # serve the static file untouched.
    if ANALYTICS_HEAD:
        try:
            html = (Path(app.static_folder) / 'index.html').read_text(encoding='utf-8')
            html = html.replace('</head>', ANALYTICS_HEAD + '\n</head>', 1)
            return app.response_class(html, mimetype='text/html')
        except Exception:
            pass
    return send_from_directory('static', 'index.html')


@app.route('/api/captcha')
def api_captcha():
    _purge_tokens()
    text  = ''.join(secrets.choice(CAPTCHA_CHARS) for _ in range(6))
    token = secrets.token_hex(16)
    with _captcha_lock:
        _captchas[token] = {'a': text, 'e': time.time() + CAPTCHA_TTL}
    return jsonify(ok=True, token=token, image=_render_captcha(text))


@app.route('/api/verify', methods=['POST'])
def api_verify():
    body   = request.get_json(silent=True) or {}
    token  = body.get('token', '')
    answer = (body.get('answer', '') or '').strip().upper()
    with _captcha_lock:
        rec = _captchas.pop(token, None)
    if not rec or rec['e'] < time.time() or rec['a'] != answer:
        return jsonify(ok=False, error='Incorrect code, please try again'), 400
    auth = secrets.token_hex(16)
    with _captcha_lock:
        _auth_tokens[auth] = time.time() + AUTH_TTL
    return jsonify(ok=True, auth=auth, expires_in=AUTH_TTL)


@app.route('/api/status')
def api_status():
    fw_path = firmware_state.get('path')
    version = None
    if fw_path:
        ino = read_ino(Path(fw_path))
        if ino:
            version = extract_version(ino)
    return jsonify(
        source=firmware_state.get('source'),
        version=version,
        l412_installed=firmware_state.get('l412_installed', False),
        ready=bool(fw_path and Path(fw_path).exists()),
        upload_enabled=ALLOW_ZIP_UPLOAD,
    )


@app.route('/api/firmware/fetch', methods=['POST'])
def api_fetch():
    if not _is_verified():
        return jsonify(ok=False, error='Human verification required', need_captcha=True), 401
    repo_dir = REPOS_DIR / 'rs41-nfw'
    try:
        if repo_dir.exists():
            r = subprocess.run(['git', 'pull'], cwd=repo_dir,
                               capture_output=True, text=True, timeout=120)
        else:
            r = subprocess.run(['git', 'clone', '--depth=1', GITHUB_REPO, str(repo_dir)],
                               capture_output=True, text=True, timeout=120)
        if r.returncode != 0:
            return jsonify(ok=False, error=(r.stderr or r.stdout).strip()), 500

        fw_path = repo_dir / 'rs41-nfw_sonde-firmware'
        if not fw_path.exists():
            return jsonify(ok=False, error='sonde-firmware folder not found'), 500

        firmware_state.update(source='github', path=str(fw_path))
        _save_state()

        ok, vmsg = install_l412_variant(repo_dir)
        if not ok:
            return jsonify(ok=False, error=f'Repo fetched but variant install failed: {vmsg}'), 500

        ino = read_ino(fw_path)
        version = extract_version(ino) if ino else 'unknown'
        return jsonify(ok=True, version=version, variant_msg=vmsg)

    except subprocess.TimeoutExpired:
        return jsonify(ok=False, error='Git timed out'), 500
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500


@app.route('/api/firmware/upload', methods=['POST'])
def api_upload():
    if not _is_verified():
        return jsonify(ok=False, error='Human verification required', need_captcha=True), 401
    if not ALLOW_ZIP_UPLOAD:
        return jsonify(ok=False, error='ZIP upload is disabled on this server'), 403
    if 'file' not in request.files:
        return jsonify(ok=False, error='No file'), 400
    f = request.files['file']
    if not f.filename.endswith('.zip'):
        return jsonify(ok=False, error='Must be a .zip'), 400

    upload_dir = REPOS_DIR / 'upload'
    if upload_dir.exists():
        shutil.rmtree(upload_dir)
    upload_dir.mkdir(parents=True)

    zip_path = upload_dir / 'fw.zip'
    f.save(str(zip_path))
    try:
        with zipfile.ZipFile(zip_path) as zf:
            dest = upload_dir.resolve()
            total = 0
            for info in zf.infolist():
                # Zip-slip guard: every entry must resolve to inside upload_dir.
                target = (upload_dir / info.filename).resolve()
                if target != dest and dest not in target.parents:
                    return jsonify(ok=False, error='Unsafe path in zip'), 400
                total += info.file_size
                if total > 200 * 1024 * 1024:   # zip-bomb guard on uncompressed size
                    return jsonify(ok=False, error='Zip contents too large'), 400
            zf.extractall(upload_dir)
    except zipfile.BadZipFile:
        return jsonify(ok=False, error='Not a valid .zip file (it may be corrupted or incomplete)'), 400
    except Exception as e:
        return jsonify(ok=False, error='Could not extract the zip: ' + str(e)), 400

    inos = list(upload_dir.rglob('rs41-nfw_sonde-firmware.ino'))
    if not inos:
        return jsonify(ok=False, error='rs41-nfw_sonde-firmware.ino was not found inside the zip. '
                                       'Zip the rs41-nfw_sonde-firmware folder (or the whole repo), not just CONFIG.h.'), 400

    fw_path = inos[0].parent
    firmware_state.update(source='upload', path=str(fw_path))
    _save_state()
    ino = read_ino(fw_path)
    version = extract_version(ino) if ino else 'unknown'
    return jsonify(ok=True, version=version)


@app.route('/api/config')
def api_config():
    fw_path = firmware_state.get('path')
    if not fw_path:
        return jsonify(ok=False, error='No firmware loaded'), 400
    src = read_config_source(Path(fw_path))
    if not src:
        return jsonify(ok=False, error='Cannot read firmware config'), 500
    return jsonify(ok=True, config=parse_config(src))


_SERIAL_RE = re.compile(r'^[A-Za-z][A-Za-z0-9]{3,11}$')

@app.route('/api/sonde_cal/<serial>')
def api_sonde_cal(serial):
    """Fetch a sonde's factory PTU calibration from SondeHub by serial number,
    map it to the firmware's factory* coefficients, cache it in the per-serial
    library, and return it for the Firmware Builder (sensor calibration mode 2)."""
    if not _is_verified():
        return jsonify(ok=False, error='Human verification required', need_captcha=True), 401

    serial = (serial or '').strip().upper()
    if not _SERIAL_RE.match(serial):
        return jsonify(ok=False, error='Invalid serial number format.'), 400

    # Import lazily so a missing optional dep never breaks the rest of the app.
    try:
        from rs41_subframe import (download_subframe_data, RS41Subframe,
                                    extract_firmware_cal, validate_cal)
    except Exception as e:
        logging.exception('rs41_subframe import failed')
        return jsonify(ok=False, error='Calibration module unavailable on the server.'), 500

    cache = SONDES_DIR / serial / 'cal.json'

    try:
        sub_bytes, telem = download_subframe_data(serial)
    except requests.exceptions.RequestException as e:
        # Network/SondeHub problem - fall back to a cached copy if we have one.
        if cache.exists():
            cached = json.loads(cache.read_text(encoding='utf-8'))
            cached['cached'] = True
            return jsonify(ok=True, **cached)
        return jsonify(ok=False, error=f'Could not reach SondeHub: {e}'), 502
    except Exception:
        logging.exception('subframe download failed')
        return jsonify(ok=False, error='Unexpected error contacting SondeHub.'), 502

    if not sub_bytes:
        if cache.exists():
            cached = json.loads(cache.read_text(encoding='utf-8'))
            cached['cached'] = True
            return jsonify(ok=True, **cached)
        return jsonify(ok=False, error='No calibration subframe found on SondeHub for '
                                       f'serial {serial}. The sonde may never have been '
                                       'received, or it is not an RS41.'), 404

    try:
        sub = RS41Subframe(raw_bytes=sub_bytes)
    except Exception as e:
        return jsonify(ok=False, error=f'Subframe could not be parsed: {e}'), 422

    ok, msg = validate_cal(sub.data)
    if not ok:
        return jsonify(ok=False, error=f'Calibration looks invalid: {msg}'), 422

    fw_cal = extract_firmware_cal(sub.data)

    # A small sanity snapshot from the telemetry frame the subframe rode on.
    telem = telem or {}
    snapshot = {k: telem.get(k) for k in
                ('temp', 'humidity', 'datetime', 'subtype', 'rs41_mainboard') if telem.get(k) is not None}

    result = {
        'serial': sub.data.get('serial') or serial,
        'cal': fw_cal,
        'mainboard': sub.data.get('mainboard_version'),
        'variant': sub.data.get('variant'),
        'telemetry': snapshot,
        'fetched': datetime.now().isoformat(timespec='seconds'),
    }

    # Persist into the per-serial library (best effort - never fail the request on IO).
    try:
        (SONDES_DIR / serial).mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps(result, indent=2), encoding='utf-8')
    except Exception:
        logging.exception('could not write sonde cal cache')

    return jsonify(ok=True, cached=False, **result)


@app.route('/api/compile', methods=['POST'])
def api_compile():
    if not _is_verified():
        return jsonify(ok=False, error='Human verification required', need_captcha=True), 401
    fw_path = firmware_state.get('path')
    if not fw_path or not Path(fw_path).exists():
        return jsonify(ok=False, error='No firmware loaded'), 400

    body = request.get_json() or {}
    platform = body.get('platform', 'RSM4x4')
    if platform not in FQBN:
        return jsonify(ok=False, error=f'Unknown platform: {platform}'), 400

    job_id = str(uuid.uuid4())[:8]
    job = {
        'id': job_id, 'status': 'queued', 'platform': platform,
        'fw_path': fw_path, 'config': sanitize_config(body.get('config', {})),
        'logs': [], 'bin_path': None, 'bin_name': None, 'error': None,
        'log_dir': None, 'created': datetime.now().isoformat(),
    }

    with queue_lock:
        if len(queue_order) >= MAX_QUEUE:
            return jsonify(ok=False, error='Build queue is full, please try again shortly'), 429
        jobs[job_id] = job
        queue_order.append(job_id)
        position = len(queue_order) - 1   # 0 = will run next / now
    compile_queue.put(job)

    # Revoke the auth token immediately so every compile needs a fresh captcha solve.
    tok = request.headers.get('X-NFW-Auth', '')
    with _captcha_lock:
        _auth_tokens.pop(tok, None)

    return jsonify(ok=True, job_id=job_id, queue_position=position)


def _queue_counts():
    with queue_lock:
        order = list(queue_order)
    running = sum(1 for j in order if jobs.get(j, {}).get('status') == 'running')
    waiting = sum(1 for j in order if jobs.get(j, {}).get('status') == 'queued')
    return order, running, waiting


@app.route('/api/queue')
def api_queue():
    order, running, waiting = _queue_counts()
    return jsonify(ok=True, running=running, waiting=waiting, total=len(order))


@app.route('/api/job/<job_id>')
def api_job(job_id):
    job = jobs.get(job_id)
    if not job:
        return jsonify(ok=False, error='Not found'), 404
    order, _, waiting = _queue_counts()
    out = {k: v for k, v in job.items() if k not in ('config', 'fw_path')}
    out['queue_position'] = order.index(job_id) if job_id in order else -1
    out['queue_waiting']  = waiting
    return jsonify(ok=True, **out)


@app.route('/api/job/<job_id>/download')
def api_download(job_id):
    job = jobs.get(job_id)
    if not job or job['status'] != 'done':
        return jsonify(ok=False, error='Not ready'), 400
    p = Path(job['bin_path'])
    if not p.exists():
        return jsonify(ok=False, error='File missing'), 500
    return send_file(str(p), as_attachment=True, download_name=job['bin_name'],
                     mimetype='application/octet-stream')


@app.route('/api/job/<job_id>/config')
def api_job_config(job_id):
    job = jobs.get(job_id)
    if not job or not job.get('log_dir'):
        return jsonify(ok=False, error='Config not available'), 404
    log_dir = Path(job['log_dir'])
    src = log_dir / 'CONFIG.h'
    if not src.exists():
        src = log_dir / 'config.ino'
    if not src.exists():
        return jsonify(ok=False, error='Config not available'), 404
    cfg = parse_config(src.read_text(encoding='utf-8'))
    lines = [f'# RS41-NFW Config Export: {datetime.now().isoformat()}', '']
    for k, v in sorted(cfg.items()):
        if not k.startswith('_'):
            lines.append(f'{k} = {v}')
    buf = BytesIO('\n'.join(lines).encode())
    buf.seek(0)
    return send_file(buf, as_attachment=True, download_name='rs41-nfw-config.txt',
                     mimetype='text/plain')


def _bootstrap():
    for d in (REPOS_DIR, SKETCH_DIR, BUILD_DIR, TMP_DIR, LOGS_DIR):
        d.mkdir(parents=True, exist_ok=True)
    _load_state()   # restore "which firmware is loaded" marker after a restart
    # Clear transient leftovers from a previous run (keep build cache, sketch, logs).
    if TMP_DIR.exists():
        for child in TMP_DIR.iterdir():
            if child.is_dir():
                shutil.rmtree(child, ignore_errors=True)
            else:
                try: child.unlink()
                except OSError: pass
    threading.Thread(target=compile_worker, daemon=True).start()


_bootstrap()

if __name__ == '__main__':
    # Serve with waitress (a real WSGI server) when available, so the app is safe to
    # put behind a reverse proxy and exposed to the internet. It stays single-process
    # and multi-threaded, which is exactly what the in-memory build queue needs (one
    # shared queue and one worker thread). Fall back to the dev server if waitress is
    # missing, e.g. when running locally without installing it.
    try:
        from waitress import serve
        serve(app, host='0.0.0.0', port=5000, threads=8)
    except ImportError:
        app.run(host='0.0.0.0', port=5000, threaded=True)
