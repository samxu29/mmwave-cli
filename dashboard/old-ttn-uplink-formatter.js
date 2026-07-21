function decodeUplink(input) {
  var bytes = input.bytes;
  var data = {};

  // ── Byte 0-3 : Unix Timestamp (uint32, big-endian) ──────────────
  var ts = ((bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3]) >>> 0;
  data.timestamp_unix = ts;
  data.timestamp_iso  = new Date(ts * 1000).toISOString();

  // ── Byte 4-5 : Natural Frequency (uint16 × 0.01 Hz) ─────────────
  var freq_raw = (bytes[4] << 8) | bytes[5];
  data.natural_frequency_hz = freq_raw / 100.0;

  // ── Byte 6-7 : Displacement RMS (uint16 × 0.001 mm) ─────────────
  var rms_raw = (bytes[6] << 8) | bytes[7];
  data.displacement_rms_mm = rms_raw / 1000.0;
  data.displacement_rms_um = rms_raw / 1.0;   // same value in μm (more intuitive on the dashboard)

  // ── Byte 8-9 : Max Deflection (uint16 × 0.001 mm) ───────────────
  var max_raw = (bytes[8] << 8) | bytes[9];
  data.max_deflection_mm = max_raw / 1000.0;
  data.max_deflection_um = max_raw / 1.0;      // same value in μm

  // ── Static coordinates (hardcoded to save payload bytes) ───────────
  data.latitude  = 43.8156;   // Muroran IT
  data.longitude = 140.9723;

  return { data: data };
}

function decodeUplinkOld(input) {
  var bytes = input.bytes;
  var data = {};

  // 1. Natural Frequency (e.g. first 2 bytes)
  data.natural_frequency = (bytes[0] << 8) | bytes[1];

  // 2. Displacement PS Point 1 - 8 (2 bytes each)
  // Index runs from byte 2 through byte 17
  for (var i = 0; i < 8; i++) {
    var startByte = 2 + (i * 2);
    data['displacement_ps' + (i + 1)] = (bytes[startByte] << 8) | bytes[startByte + 1];
  }

  // 3. Temperature (byte 18)
  var temp = bytes[18];
  data.temperature = temp > 127 ? temp - 256 : temp;

  // 4. Static coordinates (hardcoded to save payload bytes)
  data.latitude = 43.8156;  // Example coordinates for Muroran IT
  data.longitude = 140.9723;

  return { data: data };
}