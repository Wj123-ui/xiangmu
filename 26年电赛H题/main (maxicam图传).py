"""MaixVision paste-and-run wireless ball-trough video transmitter.

By default MaixCAM creates its own Wi-Fi hotspot. The receiving device joins
that hotspot and opens the URL printed in the MaixVision terminal.
The page shows the live MJPEG stream and records it in the browser.

Tested against the current MaixPy v4 API documented by Sipeed.
"""

from maix import app, camera, err, gpio, http, key, network, pinmap, time


# -------------------------- User configuration ---------------------------

# True: MaixCAM creates its own hotspot (recommended for the competition).
# False: use the Wi-Fi already configured in MaixCAM Settings.
USE_AP_MODE = True
AP_SSID = "MaixCAM-Ball"
AP_PASSWORD = "12345678"
AP_IP = "192.168.66.1"
AP_CHANNEL = 6
# Keep False during normal runs. Set True only once if you need to change the
# SSID/password/channel; restarting AP will temporarily disconnect clients.
FORCE_RESTART_AP = False

WIDTH = 720
HEIGHT = 540
TARGET_FPS = 20
JPEG_QUALITY = 75
HTTP_PORT = 8000
# Give launcher, Wi-Fi and camera services time to settle during boot.
STARTUP_DELAY_MS = 5000

# Built-in white status LED on a standard MaixCAM (active high).
# If your board is MaixCAM-Pro and you want its illumination LED beside the
# camera, change these two values to "B3" and "GPIOB3" respectively.
LIGHT_PIN = "B3"
LIGHT_GPIO = "GPIOB3"


WEB_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <title>MaixCAM 钢球无线图传</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; background: #0b1020; color: #edf2ff; }
    main { width: min(100%, 980px); margin: auto; padding: 18px; }
    h1 { margin: 0 0 6px; font-size: clamp(20px, 4vw, 30px); }
    .hint { margin: 0 0 14px; color: #aebbd5; }
    .viewer { position: relative; overflow: hidden; border: 1px solid #32415f; border-radius: 14px; background: #000; }
    #live { display: block; width: 100%; height: auto; aspect-ratio: 4/3; object-fit: contain; }
    .badge { position: absolute; top: 10px; left: 10px; padding: 6px 9px; border-radius: 999px; background: rgba(0,0,0,.68); font-size: 13px; }
    .dot { display: inline-block; width: 9px; height: 9px; margin-right: 6px; border-radius: 50%; background: #ffb020; }
    .online .dot { background: #2bd576; }
    .recording .dot { background: #ff3b4f; animation: blink 1s infinite; }
    @keyframes blink { 50% { opacity: .25; } }
    .controls { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 14px; }
    button { border: 0; border-radius: 10px; padding: 11px 16px; color: white; background: #2b65d9; font-size: 15px; cursor: pointer; }
    button.secondary { background: #344158; }
    button.danger { background: #bd2f42; }
    button:disabled { opacity: .45; cursor: not-allowed; }
    .note { margin-top: 12px; color: #aebbd5; font-size: 13px; line-height: 1.6; }
    #recordCanvas { position: fixed; left: -10000px; top: 0; width: 640px; height: 480px; }
  </style>
</head>
<body>
  <main>
    <h1>MaixCAM 钢球无线图传</h1>
    <p class="hint">实时画面 · 接收端录像 · 本地回放</p>
    <section class="viewer" id="viewer">
      <img id="live" src="/stream" alt="MaixCAM 实时画面">
      <div class="badge" id="badge"><span class="dot"></span><span id="state">正在连接</span></div>
    </section>
    <div class="controls">
      <button id="startBtn">开始录像</button>
      <button id="stopBtn" class="danger" disabled>停止并保存</button>
      <button id="shotBtn" class="secondary">保存截图</button>
      <button id="fullBtn" class="secondary">全屏</button>
    </div>
    <div class="note">录像文件保存在电脑/PAD 的下载目录。测试开始前点击“开始录像”，测试结束后点击“停止并保存”。推荐使用最新版 Chrome 或 Edge。</div>
    <canvas id="recordCanvas" width="640" height="480"></canvas>
  </main>
  <script>
    const live = document.getElementById('live');
    const canvas = document.getElementById('recordCanvas');
    const ctx = canvas.getContext('2d', { alpha: false });
    const badge = document.getElementById('badge');
    const state = document.getElementById('state');
    const startBtn = document.getElementById('startBtn');
    const stopBtn = document.getElementById('stopBtn');
    let recorder = null;
    let chunks = [];
    let recordStartedAt = 0;
    let clockTimer = null;

    function stamp() {
      const d = new Date();
      const p = n => String(n).padStart(2, '0');
      return `${d.getFullYear()}${p(d.getMonth()+1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
    }

    function download(blob, filename) {
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filename;
      document.body.appendChild(a);
      a.click();
      a.remove();
      setTimeout(() => URL.revokeObjectURL(url), 1500);
    }

    function drawFrame() {
      if (live.naturalWidth > 0) {
        ctx.drawImage(live, 0, 0, canvas.width, canvas.height);
      }
    }

    // Keep a same-origin canvas synchronized with the MJPEG image. The canvas
    // is what MediaRecorder records on the receiving computer.
    setInterval(drawFrame, 50);

    live.onload = () => {
      badge.className = 'badge online';
      state.textContent = '画面已连接';
    };
    live.onerror = () => {
      badge.className = 'badge';
      state.textContent = '连接中断，正在重试';
      setTimeout(() => { live.src = '/stream?reconnect=' + Date.now(); }, 1000);
    };

    startBtn.onclick = () => {
      if (!window.MediaRecorder || !canvas.captureStream) {
        alert('当前浏览器不支持网页录像，请改用最新版 Chrome 或 Edge。');
        return;
      }
      drawFrame();
      chunks = [];
      const mediaStream = canvas.captureStream(20);
      const choices = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm'];
      const mimeType = choices.find(t => MediaRecorder.isTypeSupported(t));
      recorder = mimeType ? new MediaRecorder(mediaStream, { mimeType }) : new MediaRecorder(mediaStream);
      recorder.ondataavailable = e => { if (e.data && e.data.size) chunks.push(e.data); };
      recorder.onstop = () => {
        const type = recorder.mimeType || 'video/webm';
        download(new Blob(chunks, { type }), 'maixcam_' + stamp() + '.webm');
      };
      recorder.start(1000);
      recordStartedAt = Date.now();
      startBtn.disabled = true;
      stopBtn.disabled = false;
      badge.className = 'badge recording';
      clockTimer = setInterval(() => {
        const seconds = Math.floor((Date.now() - recordStartedAt) / 1000);
        state.textContent = '录像中 ' + Math.floor(seconds / 60) + ':' + String(seconds % 60).padStart(2, '0');
      }, 250);
    };

    stopBtn.onclick = () => {
      if (recorder && recorder.state !== 'inactive') recorder.stop();
      clearInterval(clockTimer);
      startBtn.disabled = false;
      stopBtn.disabled = true;
      badge.className = 'badge online';
      state.textContent = '录像已保存';
    };

    document.getElementById('shotBtn').onclick = () => {
      drawFrame();
      canvas.toBlob(blob => download(blob, 'maixcam_' + stamp() + '.png'), 'image/png');
    };

    document.getElementById('fullBtn').onclick = () => {
      const viewer = document.getElementById('viewer');
      if (viewer.requestFullscreen) viewer.requestFullscreen();
    };
  </script>
</body>
</html>
"""


def setup_wifi():
    wifi = network.wifi.Wifi()

    if USE_AP_MODE:
        # Do not restart an already-running AP. Restarting it is what causes
        # MaixVision and the browser to be disconnected.
        if wifi.is_ap_mode() and not FORCE_RESTART_AP:
            # Some firmware versions return the device hostname here rather
            # than an address (for example "hezhihao"). Use the AP address
            # configured above so the browser always gets a usable URL.
            print("Wi-Fi AP already active:", AP_IP)
            return wifi, AP_IP

        # Use USB MaixVision if AP must be restarted, because changing from
        # STA to AP (or restarting AP) temporarily disconnects Wi-Fi clients.
        if wifi.is_ap_mode():
            wifi.stop_ap()
            time.sleep_ms(300)
        elif wifi.is_connected():
            wifi.disconnect()
            time.sleep_ms(300)

        result = wifi.start_ap(
            AP_SSID,
            AP_PASSWORD,
            mode="g",
            channel=AP_CHANNEL,
            ip=AP_IP,
            netmask="255.255.255.0",
            hidden=False,
        )
        err.check_raise(result, "start MaixCAM hotspot failed")
        time.sleep_ms(800)
        return wifi, AP_IP

    ip = wifi.get_ip()
    if not ip:
        raise RuntimeError(
            "MaixCAM has no Wi-Fi IP. Open Settings -> WiFi, connect it to "
            "the same Wi-Fi as the receiving computer, then run again."
        )
    return wifi, ip


light_on = False
light = None
long_pressed = False


def on_key(key_id, state):
    # Creating key.Key disables the normal USER/OK exit behavior.
    # Short press: toggle on release. Long press: exit and do not toggle.
    global light_on, long_pressed
    if key_id != key.Keys.KEY_OK:
        return
    if state == key.State.KEY_PRESSED:
        long_pressed = False
    elif state == key.State.KEY_LONG_PRESSED:
        long_pressed = True
        print("USER long press: exiting application")
        app.set_exit_flag(True)
    elif state == key.State.KEY_RELEASED and not long_pressed:
        light_on = not light_on
        light.value(1 if light_on else 0)
        print("White light:", "ON" if light_on else "OFF")


print("Waiting for system services before starting video...")
time.sleep_ms(STARTUP_DELAY_MS)

wifi, device_ip = setup_wifi()
err.check_raise(pinmap.set_pin_function(LIGHT_PIN, LIGHT_GPIO), "set white light pin failed")
light = gpio.GPIO(LIGHT_GPIO, gpio.Mode.OUT)
light.value(0)  # Always start with the light off.
user_key = key.Key(on_key)
user_key.long_press_time(2000)  # hold USER for 2 seconds to exit
cam = camera.Camera(WIDTH, HEIGHT)
streamer = http.JpegStreamer(host="", port=HTTP_PORT, client_number=2)
streamer.set_html(WEB_PAGE)
streamer.start()

print("Wi-Fi SSID:", AP_SSID if USE_AP_MODE else wifi.get_ssid())
print("Open in Chrome/Edge: http://{}:{}".format(AP_IP if USE_AP_MODE else device_ip, HTTP_PORT))

frame_period_ms = max(1, 1000 // TARGET_FPS)

while not app.need_exit():
    frame_started = time.ticks_ms()
    frame = cam.read()

    # Add steel-ball position detection/annotation here if needed. Keep the
    # processing before to_jpeg(), so the receiver sees the annotated result.

    jpeg = frame.to_jpeg(JPEG_QUALITY)
    streamer.write(jpeg)

    elapsed_ms = time.ticks_ms() - frame_started
    if elapsed_ms < frame_period_ms:
        time.sleep_ms(frame_period_ms - elapsed_ms)
