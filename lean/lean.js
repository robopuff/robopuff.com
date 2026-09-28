(function () {
  'use strict';

  /* ---------- Configuration ---------- */
  var ALPHA = 0.985;          // complementary filter weight for gyroscope
  var DISPLAY_TAU = 0.06;     // display smoothing time constant (seconds)
  var GAUGE_RANGE = 50;       // degrees shown on each side of the gauge
  var GYRO_CLAMP = 500;       // deg/s sanity clamp
  var GRAVITY = 9.80665;      // m/s^2
  var DEADBAND = 0.35;        // deg, kills stationary jitter
  var SIGN_THRESHOLD = 3;     // deg^2, gyro/accel sign agreement
  var ACC_GATE = 1.2;         // m/s^2; |a|-g beyond this is not a steady reference
  var TURN_RATE_MIN = 1.5;    // deg/s heading rate that counts as a turn
  var GPS_LEAN_TAU = 2.5;     // s; GPS lean correction time constant
  var GPS_V_MIN = 3.5;        // m/s (~12.6 km/h); min speed for heading-based lean
  var LEAN_MAX = 60;          // deg; clamp for the GPS lean estimate

  /* ---------- State ---------- */
  var state = {
    angle: 0,        // complementary-filtered lean angle (deg), + = right
    display: 0,      // smoothed value driving the UI
    zero: 0,         // calibration offset
    maxLeft: 0,      // most negative lean recorded
    maxRight: 0,     // most positive lean recorded
    lastT: 0,
    startT: 0,
    init: false,
    gyroSign: 1,
    signCorr: 0,
    signLocked: false,
    prevAccel: null,
    running: false,
    zeroed: false,
    magLp: 0
  };

  // GPS-derived speed and heading. coords.speed is preferred; distance/time is
  // the fallback. Heading rate gives a physically exact lean via atan(v*w/g).
  var geo = {
    watchId: null,
    lastPos: null,
    rawMs: 0,        // latest speed in m/s
    displayKmh: 0,   // smoothed speed in km/h
    hasFix: false,
    denied: false,
    units: 'kmh',    // 'kmh' or 'mph'
    lastFrame: 0,
    prevHeading: null,
    prevHeadingT: 0,
    omega: 0,        // smoothed heading rate, deg/s
    lean: 0,         // GPS/physics lean estimate, deg
    leanValid: false,
    distance: 0      // cumulative travelled distance, metres
  };

  /* ---------- DOM ---------- */
  function $(id) { return document.getElementById(id); }

  var elAngle = $('angle');
  var elIndicator = $('gauge-indicator');
  var elLeft = $('stat-left').getElementsByClassName('value')[0];
  var elRight = $('stat-right').getElementsByClassName('value')[0];
  var elStart = $('start-screen');
  var elNote = $('start-note');
  var elSpeed = $('speed-value');
  var elSpeedUnit = $('speed-unit');
  var elDuration = $('duration-value');
  var elDistance = $('distance-value');
  var elDistanceUnit = $('distance-unit');

  /* ---------- Helpers ---------- */
  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

  function wrap180(a) {
    a = a % 360;
    if (a > 180) a -= 360;
    if (a < -180) a += 360;
    return a;
  }

  /* ---------- Sensor handling ---------- */
  function handleMotion(e) {
    var now = performance.now();
    if (!state.lastT) {
      state.lastT = now;
      if (!state.startT) state.startT = now;
      return;
    }

    var dt = (now - state.lastT) / 1000;
    state.lastT = now;
    if (dt <= 0) return;
    if (dt > 0.5) dt = 0.5;

    var acc = e.accelerationIncludingGravity;
    var rr = e.rotationRate;

    // Accelerometer roll angle: + = lean right. Ratio is unaffected by pitch,
    // so the phone may be mounted at any fore/aft angle.
    var accelAngle = null;
    var accMag = 0;
    if (acc && acc.x != null && acc.y != null) {
      accelAngle = Math.atan2(-acc.x, acc.y) * 180 / Math.PI;
      var z = (acc.z == null ? 0 : acc.z);
      accMag = Math.sqrt(acc.x * acc.x + acc.y * acc.y + z * z);
    }

    // Low-passed specific-force magnitude: road/engine vibration makes the
    // instantaneous value noisy, which would otherwise flicker the turn gate.
    if (state.magLp === 0) state.magLp = accMag;
    state.magLp += (accMag - state.magLp) * (1 - Math.exp(-dt / 0.25));
    var m = state.magLp;

    // Gyroscope roll rate about the screen-normal axis (deg/s).
    var gyroRate = 0;
    if (rr && rr.alpha != null) {
      gyroRate = clamp(-rr.alpha, -GYRO_CLAMP, GYRO_CLAMP);
    }

    // Seed the filter from the accelerometer on the first usable sample.
    if (!state.init) {
      if (accelAngle != null) {
        state.angle = accelAngle;
        state.init = true;
        state.prevAccel = accelAngle;
      }
      return;
    }

    // Auto-detect gyro sign by correlating with accelerometer change, so the
    // filter works regardless of mount orientation / sensor sign convention.
    if (accelAngle != null && !state.signLocked && state.prevAccel != null) {
      var dAcc = wrap180(accelAngle - state.prevAccel);
      var gyroDelta = gyroRate * dt;
      if (Math.abs(gyroDelta) > 0.1 && Math.abs(dAcc) < 8) {
        state.signCorr += gyroDelta * dAcc;
        if (state.signCorr > SIGN_THRESHOLD) { state.gyroSign = 1; state.signLocked = true; }
        else if (state.signCorr < -SIGN_THRESHOLD) { state.gyroSign = -1; state.signLocked = true; }
      }
      state.prevAccel = accelAngle;
    }

    // Gyroscope integration (fast response).
    state.angle = wrap180(state.angle + state.gyroSign * gyroRate * dt);

    // Reference-angle correction (long-term drift reduction).
    //
    // The accelerometer is only a valid roll reference when the specific force
    // is ~1 g (straight and steady). In a coordinated turn it points along the
    // bike's own vertical and reads ~0 lean, so it must be gated off. Instead:
    //   - turning      -> use the exact relation atan(v*w/g) from GPS speed and
    //                     heading rate, or the magnitude relation acos(g/|a|)
    //   - straight/steady -> use the accelerometer roll direction
    //   - otherwise    -> trust the gyro alone
    var turning = geo.hasFix && geo.leanValid && Math.abs(geo.omega) > TURN_RATE_MIN;
    var steady = Math.abs(m - GRAVITY) < ACC_GATE;

    if (turning) {
      // 1) Fast term: accelerometer magnitude, |a| = g / cos(lean).
      if (m > GRAVITY && accelAngle != null) {
        var magLean = Math.acos(clamp(GRAVITY / m, -1, 1)) * 180 / Math.PI;
        var sgn = geo.lean >= 0 ? 1 : -1;
        var magDiff = wrap180(sgn * magLean - state.angle);
        state.angle = wrap180(state.angle + (1 - ALPHA) * 0.5 * magDiff);
      }
      // 2) Slow term: exact physics atan(v*w/g) corrects gyro bias/scale.
      var geoDiff = wrap180(geo.lean - state.angle);
      state.angle = wrap180(state.angle + (1 - Math.exp(-dt / GPS_LEAN_TAU)) * geoDiff);
    } else if (steady && accelAngle != null) {
      var diff = wrap180(accelAngle - state.angle);
      state.angle = wrap180(state.angle + (1 - ALPHA) * diff);
    }

    // Smoothing + maxima live in the sensor handler (not the render loop) so
    // tracking keeps working even if requestAnimationFrame is paused while the
    // page is in the background.
    var lean = wrap180(state.angle - state.zero);
    var factor = 1 - Math.exp(-dt / DISPLAY_TAU);
    state.display += (lean - state.display) * factor;

    var shown = state.display;
    if (Math.abs(shown) < DEADBAND) shown = 0;

    if (shown > state.maxRight) state.maxRight = shown;
    if (shown < state.maxLeft) state.maxLeft = shown;
  }

  /* ---------- GPS speed and heading ---------- */
  function haversine(lat1, lon1, lat2, lon2) {
    var R = 6371000; // metres
    var toRad = Math.PI / 180;
    var dLat = (lat2 - lat1) * toRad;
    var dLon = (lon2 - lon1) * toRad;
    var a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
            Math.cos(lat1 * toRad) * Math.cos(lat2 * toRad) *
            Math.sin(dLon / 2) * Math.sin(dLon / 2);
    return 2 * R * Math.asin(Math.min(1, Math.sqrt(a)));
  }

  function bearing(lat1, lon1, lat2, lon2) {
    var toRad = Math.PI / 180;
    var toDeg = 180 / Math.PI;
    var y = Math.sin((lon2 - lon1) * toRad) * Math.cos(lat2 * toRad);
    var x = Math.cos(lat1 * toRad) * Math.sin(lat2 * toRad) -
            Math.sin(lat1 * toRad) * Math.cos(lat2 * toRad) *
            Math.cos((lon2 - lon1) * toRad);
    return (Math.atan2(y, x) * toDeg + 360) % 360;
  }

  function onPosition(pos) {
    var c = pos.coords;
    var now = pos.timestamp || Date.now();
    var speedMs = null;
    var d = 0;
    var dt = 0;

    if (geo.lastPos) {
      d = haversine(geo.lastPos.lat, geo.lastPos.lon, c.latitude, c.longitude);
      dt = (now - geo.lastPos.t) / 1000;
    }

    if (typeof c.speed === 'number' && isFinite(c.speed) && c.speed >= 0) {
      speedMs = c.speed;
    } else if (geo.lastPos) {
      // Ignore sub-accuracy jitter so a stationary bike reads 0.
      if (dt > 0.2 && d > (c.accuracy || 0) * 0.5) speedMs = d / dt;
      else speedMs = 0;
    } else {
      speedMs = 0;
    }

    // Accumulate travelled distance, dropping jitter below the fix accuracy.
    if (geo.lastPos && dt > 0.2 && dt < 30 && d > (c.accuracy || 0) * 0.5) {
      geo.distance += d;
    }

    // Heading for turn-rate lean. Prefer GPS course; otherwise derive a bearing
    // from the previous fix.
    var heading = null;
    if (typeof c.heading === 'number' && isFinite(c.heading)) {
      heading = c.heading;
    } else if (geo.lastPos) {
      heading = bearing(geo.lastPos.lat, geo.lastPos.lon, c.latitude, c.longitude);
    }

    if (heading != null && geo.prevHeading != null && speedMs >= GPS_V_MIN) {
      var hdt = (now - geo.prevHeadingT) / 1000;
      if (hdt > 0.3 && hdt < 5) {
        var dh = wrap180(heading - geo.prevHeading);
        var rawOmega = dh / hdt; // deg/s
        var of = 1 - Math.exp(-hdt / 0.8); // smooth the ~1 Hz GPS heading
        geo.omega += (rawOmega - geo.omega) * of;
        var omegaRad = geo.omega * Math.PI / 180;
        geo.lean = clamp(
          Math.atan(speedMs * omegaRad / GRAVITY) * 180 / Math.PI,
          -LEAN_MAX, LEAN_MAX
        );
        geo.leanValid = true;
      }
    } else if (speedMs < GPS_V_MIN) {
      geo.leanValid = false;
      geo.omega = 0;
    }

    if (heading != null) {
      geo.prevHeading = heading;
      geo.prevHeadingT = now;
    }

    geo.lastPos = { lat: c.latitude, lon: c.longitude, t: now };
    geo.rawMs = speedMs;
    geo.hasFix = true;
  }

  function onGeoError(err) {
    // Keep the last good reading on transient errors; only mark unavailable
    // when we have never had a fix (e.g. permission denied).
    if (!geo.hasFix) {
      geo.denied = true;
    }
  }

  function startGeolocation() {
    if (!navigator.geolocation) { geo.denied = true; return; }
    try {
      geo.watchId = navigator.geolocation.watchPosition(onPosition, onGeoError, {
        enableHighAccuracy: true,
        maximumAge: 1000,
        timeout: 20000
      });
    } catch (err) {
      geo.denied = true;
    }
  }

  function formatSpeed() {
    if (!geo.hasFix) return '--';
    var v = geo.units === 'mph' ? geo.displayKmh / 1.609344 : geo.displayKmh;
    return String(Math.round(v));
  }

  function formatDistance() {
    if (!geo.hasFix) return '--';
    var v = geo.units === 'mph' ? geo.distance / 1609.344 : geo.distance / 1000;
    return v.toFixed(v < 10 ? 2 : 1);
  }

  function formatDuration(ms) {
    if (!state.startT) return '--';
    var total = Math.floor(ms / 1000);
    if (total < 0) total = 0;
    var h = Math.floor(total / 3600);
    var m = Math.floor((total % 3600) / 60);
    var s = total % 60;
    function pad(n) { return (n < 10 ? '0' : '') + n; }
    return h > 0 ? (h + ':' + pad(m) + ':' + pad(s)) : (pad(m) + ':' + pad(s));
  }

  function toggleUnits() {
    geo.units = geo.units === 'kmh' ? 'mph' : 'kmh';
    elSpeedUnit.textContent = geo.units === 'mph' ? 'mph' : 'km/h';
    elDistanceUnit.textContent = geo.units === 'mph' ? 'mi' : 'km';
  }

  /* ---------- Rendering ---------- */
  function setLeanColor(el, deg) {
    el.classList.remove('amber', 'red');
    if (deg >= 30) el.classList.add('red');
    else if (deg >= 15) el.classList.add('amber');
  }

  function render() {
    if (state.running) {
      // Smooth GPS speed toward the latest reading.
      var frameNow = performance.now();
      var fdt = geo.lastFrame ? (frameNow - geo.lastFrame) / 1000 : 0.016;
      geo.lastFrame = frameNow;
      if (geo.hasFix) {
        var targetKmh = geo.rawMs * 3.6;
        var sf = 1 - Math.exp(-fdt / 0.3);
        geo.displayKmh += (targetKmh - geo.displayKmh) * sf;
        if (geo.displayKmh < 1.5) geo.displayKmh = 0;
      }
      elSpeed.textContent = formatSpeed();
      elDuration.textContent = formatDuration(frameNow - state.startT);
      elDistance.textContent = formatDistance();

      // Auto-zero the lean reference on the first move: if the user has not
      // calibrated and speed passes 10 km/h, treat the current attitude as
      // upright. This only happens once.
      if (!state.zeroed && geo.hasFix && geo.displayKmh > 10) {
        zeroCalibration();
      }

      var shown = state.display;
      if (Math.abs(shown) < DEADBAND) shown = 0;

      var absShown = Math.abs(shown);

      elAngle.textContent = Math.round(shown);
      setLeanColor(elAngle, absShown);

      elLeft.textContent = Math.round(Math.abs(state.maxLeft));
      elRight.textContent = Math.round(state.maxRight);

      var pos = clamp((shown + GAUGE_RANGE) / (2 * GAUGE_RANGE) * 100, 0, 100);
      elIndicator.style.left = pos + '%';
    }
    requestAnimationFrame(render);
  }

  /* ---------- Fullscreen / landscape / wake lock ---------- */
  function goFullscreen() {
    var el = document.documentElement;
    var fn = el.requestFullscreen || el.webkitRequestFullscreen ||
             el.mozRequestFullScreen || el.msRequestFullscreen;
    if (fn) {
      try {
        var p = fn.call(el);
        if (p && p['catch']) p['catch'](function () {});
      } catch (err) {}
    }
  }

  function lockLandscape() {
    try {
      if (screen.orientation && screen.orientation.lock) {
        var p = screen.orientation.lock('landscape');
        if (p && p['catch']) p['catch'](function () {});
      }
    } catch (err) {}
  }

  /* ---------- Keep awake / background execution ---------- */
  // Two independent mechanisms:
  //   1. Screen Wake Lock API  -> stops the display from sleeping.
  //   2. A silent, looping audio stream -> keeps the tab from being frozen in
  //      the background so timers/sensors keep flowing on Android Chrome.
  var keepAwake = {
    enabled: true,
    wakeLock: null,
    audio: null,
    audioUrl: null
  };

  // Build a short silent WAV in memory (no external files / no libraries).
  function makeSilentWavUrl() {
    try {
      var sampleRate = 8000;
      var dataSize = sampleRate; // 1 second, 8-bit mono
      var buffer = new ArrayBuffer(44 + dataSize);
      var view = new DataView(buffer);
      function str(offset, s) {
        for (var i = 0; i < s.length; i++) view.setUint8(offset + i, s.charCodeAt(i));
      }
      str(0, 'RIFF'); view.setUint32(4, 36 + dataSize, true); str(8, 'WAVE');
      str(12, 'fmt '); view.setUint32(16, 16, true);
      view.setUint16(20, 1, true); view.setUint16(22, 1, true);
      view.setUint32(24, sampleRate, true); view.setUint32(28, sampleRate, true);
      view.setUint16(32, 1, true); view.setUint16(34, 8, true);
      str(36, 'data'); view.setUint32(40, dataSize, true);
      for (var j = 0; j < dataSize; j++) view.setUint8(44 + j, 128); // silence
      return URL.createObjectURL(new Blob([view], { type: 'audio/wav' }));
    } catch (err) {
      return null;
    }
  }

  function initSilentAudio() {
    if (keepAwake.audio) return;
    try {
      keepAwake.audioUrl = makeSilentWavUrl();
      if (!keepAwake.audioUrl) return;
      var audio = document.createElement('audio');
      audio.setAttribute('playsinline', '');
      audio.setAttribute('webkit-playsinline', '');
      audio.loop = true;
      audio.preload = 'auto';
      audio.volume = 1;
      audio.style.display = 'none';
      audio.src = keepAwake.audioUrl;
      document.body.appendChild(audio);
      keepAwake.audio = audio;
    } catch (err) {}
  }

  function playSilentAudio() {
    initSilentAudio();
    if (!keepAwake.audio) return;
    try {
      var p = keepAwake.audio.play();
      if (p && p['catch']) p['catch'](function () {});
    } catch (err) {}
  }

  function pauseSilentAudio() {
    if (!keepAwake.audio) return;
    try { keepAwake.audio.pause(); } catch (err) {}
  }

  function acquireWakeLock() {
    try {
      if (navigator.wakeLock && navigator.wakeLock.request) {
        navigator.wakeLock.request('screen').then(function (wl) {
          keepAwake.wakeLock = wl;
          if (wl && wl.addEventListener) {
            wl.addEventListener('release', function () { keepAwake.wakeLock = null; });
          }
        })['catch'](function () {});
      }
    } catch (err) {}
  }

  function releaseWakeLock() {
    if (keepAwake.wakeLock) {
      try { keepAwake.wakeLock.release(); } catch (err) {}
      keepAwake.wakeLock = null;
    }
  }

  function updateAwakeButton() {
    var b = $('btn-awake');
    if (!b) return;
    if (keepAwake.enabled) {
      b.textContent = 'Stay Awake: On';
      b.classList.add('on');
      b.classList.remove('off');
    } else {
      b.textContent = 'Stay Awake: Off';
      b.classList.remove('on');
      b.classList.add('off');
    }
  }

  function applyKeepAwake() {
    if (keepAwake.enabled) {
      acquireWakeLock();
      playSilentAudio();
    } else {
      releaseWakeLock();
      pauseSilentAudio();
    }
    updateAwakeButton();
  }

  function toggleKeepAwake() {
    keepAwake.enabled = !keepAwake.enabled;
    applyKeepAwake();
  }

  /* ---------- Startup ---------- */
  function start() {
    elNote.textContent = '';

    if (typeof DeviceMotionEvent === 'undefined') {
      elNote.textContent = 'Motion sensors are not supported on this device.';
      return;
    }

    function launch() {
      window.addEventListener('devicemotion', handleMotion, { passive: true });
      state.running = true;
      state.lastT = 0;
      state.startT = performance.now();
      document.body.classList.add('started');
      goFullscreen();
      lockLandscape();
      applyKeepAwake();
      startGeolocation();
    }

    if (typeof DeviceMotionEvent.requestPermission === 'function') {
      DeviceMotionEvent.requestPermission().then(function (res) {
        if (res === 'granted') {
          launch();
        } else {
          elNote.textContent = 'Motion access denied. Allow motion access and reload.';
        }
      })['catch'](function () {
        elNote.textContent = 'Unable to request motion access.';
      });
    } else {
      launch();
    }
  }

  /* ---------- Controls ---------- */
  function zeroCalibration() {
    state.zero = state.angle;
    state.display = 0;
    state.zeroed = true;
  }

  function resetMaxima() {
    state.maxLeft = 0;
    state.maxRight = 0;
    state.startT = performance.now();
  }

  /* ---------- Orientation handling ---------- */
  function updateOrientation() {
    var portrait = window.matchMedia
      ? window.matchMedia('(orientation: portrait)').matches
      : false;
    // Fallback for browsers with unreliable orientation media queries.
    if (!portrait) portrait = window.innerHeight > window.innerWidth;
    document.body.classList.toggle('portrait', portrait);
  }

  /* ---------- Wiring ---------- */
  elStart.addEventListener('click', start);
  $('btn-zero').addEventListener('click', zeroCalibration);
  $('btn-reset').addEventListener('click', resetMaxima);
  $('btn-awake').addEventListener('click', toggleKeepAwake);
  $('stat-speed').addEventListener('click', toggleUnits);

  window.addEventListener('resize', updateOrientation);
  window.addEventListener('orientationchange', updateOrientation);

  if (window.matchMedia) {
    var mq = window.matchMedia('(orientation: portrait)');
    if (mq.addEventListener) mq.addEventListener('change', updateOrientation);
    else if (mq.addListener) mq.addListener(updateOrientation);
  }

  document.addEventListener('visibilitychange', function () {
    if (document.visibilityState === 'visible') {
      // Returning to the foreground: reset the timer so the filter does not
      // take a huge dt jump, and re-arm the wake lock / audio keep-alive.
      state.lastT = 0;
      if (state.running) applyKeepAwake();
    }
  });

  // Some browsers only allow the silent audio to start after a user gesture.
  document.addEventListener('touchstart', function once() {
    if (keepAwake.enabled) playSilentAudio();
    document.removeEventListener('touchstart', once);
  }, { passive: true });

  updateOrientation();
  updateAwakeButton();
  requestAnimationFrame(render);

  // PWA: register the service worker for offline use (needs https/localhost).
  if ('serviceWorker' in navigator) {
    window.addEventListener('load', function () {
      navigator.serviceWorker.register('./sw.js')['catch'](function () {});
    });
  }
})();
