/*
  Example: CodeCell Wi-Fi 3D Rotation Visualizer
  Boards: CodeCell C3 / CodeCell C6 / CodeCell C6 Drive

  The CodeCell creates its own Wi-Fi network and hosts a webpage.
  The webpage displays a 3D board that follows the IMU orientation.

  A WebSocket sends the angles continuously at about 30 updates per
  second. Quaternion smoothing prevents jumps around angle boundaries.

  Before compiling, install "WebSockets" by Markus Sattler from:
  Arduino IDE > Tools > Manage Libraries

  Instructions:
  1. Upload this sketch.
  2. Connect to Wi-Fi: CodeCell-IMU
  3. Password: codecell
  4. Open url
*/

#include <CodeCell.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

CodeCell myCodeCell;

// Port 80 serves the webpage. Port 81 carries the live IMU data.
WebServer webServer(80);
WebSocketsServer webSocket(81);

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

// Change these if you want the CodeCell to use a different network name.
// Wi-Fi passwords must contain at least eight characters.
const char* wifiName = "CodeCell-IMU";
const char* wifiPassword = "codecell";

// PROGMEM keeps the webpage in flash instead of using normal RAM.
const char webpage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">

<head>
  <meta charset="UTF-8">

  <meta
    name="viewport"
    content="width=device-width, initial-scale=1.0"
  >

  <title>CodeCell IMU Visualizer</title>

  <style>
    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      overflow: hidden;
      color: white;
      background:
        radial-gradient(circle at center, #303030 0%, #101010 72%);
      font-family: Arial, Helvetica, sans-serif;
    }

    .header {
      position: absolute;
      top: 25px;
      left: 30px;
      z-index: 10;
    }

    h1 {
      margin: 0 0 8px;
      font-size: 28px;
    }

    #status {
      color: #ffb380;
      font-size: 15px;
    }

    .scene {
      position: absolute;
      top: 47%;
      left: 50%;
      width: 240px;
      height: 240px;
      perspective: 850px;
      transform: translate(-50%, -50%);
    }

    /* Fixed viewing angle so the top remains visible when the board is flat. */
    .camera {
      position: relative;
      width: 240px;
      height: 240px;
      transform-style: preserve-3d;
      transform: rotateX(65deg);
    }

    .board {
      position: relative;
      width: 240px;
      height: 240px;
      transform-style: preserve-3d;

      /* JavaScript handles smoothing, so no CSS transition is needed. */
      transition: none;
      will-change: transform;
    }

    .face {
      position: absolute;
      border: 2px solid rgba(255, 255, 255, 0.55);
      backface-visibility: visible;
    }

    .front,
    .back {
      width: 240px;
      height: 240px;
      background: linear-gradient(135deg, #ff984f, #ff6600);
    }

    .front {
      transform: translateZ(15px);
    }

    .back {
      transform: rotateY(180deg) translateZ(15px);
    }

    .left,
    .right {
      left: 105px;
      width: 30px;
      height: 240px;
      background: #a84000;
    }

    .left {
      transform: rotateY(-90deg) translateZ(120px);
    }

    .right {
      transform: rotateY(90deg) translateZ(120px);
    }

    .top,
    .bottom {
      top: 105px;
      width: 240px;
      height: 30px;
      background: #d65300;
    }

    .top {
      transform: rotateX(90deg) translateZ(120px);
    }

    .bottom {
      transform: rotateX(-90deg) translateZ(120px);
    }

    .logo {
      position: absolute;
      top: 50%;
      left: 50%;
      color: white;
      font-size: 23px;
      font-weight: bold;
      letter-spacing: 1px;
      transform: translate(-50%, -50%) translateZ(17px);
    }

    .direction {
      position: absolute;
      top: 15px;
      left: 50%;
      width: 0;
      height: 0;
      border-right: 14px solid transparent;
      border-bottom: 28px solid white;
      border-left: 14px solid transparent;
      transform: translateX(-50%) translateZ(18px);
    }

    .readings {
      position: absolute;
      bottom: 25px;
      left: 50%;
      display: flex;
      gap: 12px;
      transform: translateX(-50%);
      z-index: 10;
    }

    .reading {
      width: 115px;
      padding: 12px;
      text-align: center;
      background: rgba(255, 102, 0, 0.12);
      border: 1px solid rgba(255, 102, 0, 0.5);
      border-radius: 12px;
      backdrop-filter: blur(8px);
    }

    .label {
      margin-bottom: 5px;
      color: #ffb380;
      font-size: 12px;
      text-transform: uppercase;
    }

    .value {
      font-size: 20px;
      font-weight: bold;
    }

    @media (max-width: 500px) {
      .scene {
        transform: translate(-50%, -50%) scale(0.72);
      }

      .readings {
        gap: 5px;
      }

      .reading {
        width: 100px;
      }
    }
  </style>
</head>

<body>
  <div class="header">
    <h1>CodeCell IMU</h1>
    <div id="status">Connecting...</div>
  </div>

  <div class="scene">
    <div class="camera">
      <div id="board" class="board">
        <div class="face front"></div>
        <div class="face back"></div>
        <div class="face left"></div>
        <div class="face right"></div>
        <div class="face top"></div>
        <div class="face bottom"></div>

        <div class="logo">CodeCell</div>
        <div class="direction"></div>
      </div>
    </div>
  </div>

  <div class="readings">
    <div class="reading">
      <div class="label">Roll</div>
      <div id="roll" class="value">0.0°</div>
    </div>

    <div class="reading">
      <div class="label">Pitch</div>
      <div id="pitch" class="value">0.0°</div>
    </div>

    <div class="reading">
      <div class="label">Yaw</div>
      <div id="yaw" class="value">0.0°</div>
    </div>
  </div>

  <script>
    const board = document.getElementById("board");
    const statusText = document.getElementById("status");

    const rollText = document.getElementById("roll");
    const pitchText = document.getElementById("pitch");
    const yawText = document.getElementById("yaw");

    let socket;
    let reconnectTimer;
    let orientationReceived = false;

    // A quaternion stores a 3D orientation without an angle-wrap jump.
    // The four values are arranged as [x, y, z, w].
    let currentQuaternion = [0, 0, 0, 1];
    let targetQuaternion  = [0, 0, 0, 1];

    function degreesToRadians(degrees) {
      return degrees * Math.PI / 180;
    }

    // Convert the IMU angles into one 3D orientation.
    // Rotation order: Yaw (Z), Pitch (Y), then Roll (X).
    function eulerToQuaternion(roll, pitch, yaw) {
      const halfRoll  = degreesToRadians(roll) * 0.5;
      const halfPitch = degreesToRadians(pitch) * 0.5;
      const halfYaw   = degreesToRadians(yaw) * 0.5;

      const cr = Math.cos(halfRoll);
      const sr = Math.sin(halfRoll);
      const cp = Math.cos(halfPitch);
      const sp = Math.sin(halfPitch);
      const cy = Math.cos(halfYaw);
      const sy = Math.sin(halfYaw);

      return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
      ];
    }

    function quaternionDot(a, b) {
      return (
        a[0] * b[0] +
        a[1] * b[1] +
        a[2] * b[2] +
        a[3] * b[3]
      );
    }

    function normalizeQuaternion(q) {
      const length = Math.sqrt(quaternionDot(q, q));

      if (length < 0.000001) {
        return [0, 0, 0, 1];
      }

      return [
        q[0] / length,
        q[1] / length,
        q[2] / length,
        q[3] / length
      ];
    }

    // Smoothly move between orientations using the shortest rotation.
    function slerpQuaternion(start, end, amount) {
      let target = [...end];
      let dot = quaternionDot(start, target);

      if (dot < 0) {
        target = target.map(value => -value);
        dot = -dot;
      }

      dot = Math.min(1, Math.max(-1, dot));

      // Nearby orientations can use a simpler interpolation safely.
      if (dot > 0.9995) {
        return normalizeQuaternion([
          start[0] + amount * (target[0] - start[0]),
          start[1] + amount * (target[1] - start[1]),
          start[2] + amount * (target[2] - start[2]),
          start[3] + amount * (target[3] - start[3])
        ]);
      }

      const angle = Math.acos(dot);
      const sinAngle = Math.sin(angle);

      const startWeight =
        Math.sin((1 - amount) * angle) / sinAngle;

      const targetWeight =
        Math.sin(amount * angle) / sinAngle;

      return [
        start[0] * startWeight + target[0] * targetWeight,
        start[1] * startWeight + target[1] * targetWeight,
        start[2] * startWeight + target[2] * targetWeight,
        start[3] * startWeight + target[3] * targetWeight
      ];
    }

    // Convert the orientation into a transform understood by the browser.
    function quaternionToMatrix3d(q) {
      const x = q[0];
      const y = q[1];
      const z = q[2];
      const w = q[3];

      const xx = x * x;
      const yy = y * y;
      const zz = z * z;

      const xy = x * y;
      const xz = x * z;
      const yz = y * z;

      const wx = w * x;
      const wy = w * y;
      const wz = w * z;

      const m00 = 1 - 2 * (yy + zz);
      const m01 = 2 * (xy - wz);
      const m02 = 2 * (xz + wy);

      const m10 = 2 * (xy + wz);
      const m11 = 1 - 2 * (xx + zz);
      const m12 = 2 * (yz - wx);

      const m20 = 2 * (xz - wy);
      const m21 = 2 * (yz + wx);
      const m22 = 1 - 2 * (xx + yy);

      // CSS matrix3d expects the values in column-major order.
      return `matrix3d(
        ${m00}, ${m10}, ${m20}, 0,
        ${m01}, ${m11}, ${m21}, 0,
        ${m02}, ${m12}, ${m22}, 0,
        0, 0, 0, 1
      )`;
    }

    // The browser draws at about 60 Hz and smooths between 30 Hz readings.
    function animate() {
      if (orientationReceived) {
        currentQuaternion = slerpQuaternion(
          currentQuaternion,
          targetQuaternion,
          0.35
        );

        board.style.transform =
          quaternionToMatrix3d(currentQuaternion);
      }

      requestAnimationFrame(animate);
    }

    function connectWebSocket() {
      clearTimeout(reconnectTimer);

      statusText.textContent = "Connecting...";
      statusText.style.color = "#ffb380";

      // Connect back to this CodeCell on the WebSocket port.
      socket = new WebSocket(
        "ws://" + window.location.hostname + ":81/"
      );

      socket.onopen = function() {
        statusText.textContent = "Live · 30 Hz";
        statusText.style.color = "#ff6600";
      };

      socket.onmessage = function(event) {
        const values = event.data.split(",");

        if (values.length !== 3) {
          return;
        }

        const roll = Number(values[0]);
        const pitch = Number(values[1]);
        const yaw = Number(values[2]);

        if (
          !Number.isFinite(roll) ||
          !Number.isFinite(pitch) ||
          !Number.isFinite(yaw)
        ) {
          return;
        }

        rollText.textContent = roll.toFixed(1) + "°";
        pitchText.textContent = pitch.toFixed(1) + "°";
        yawText.textContent = yaw.toFixed(1) + "°";

        const newQuaternion =
          eulerToQuaternion(roll, pitch, yaw);

        // q and -q mean the same orientation; choose the nearest one.
        if (
          quaternionDot(currentQuaternion, newQuaternion) < 0
        ) {
          targetQuaternion = newQuaternion.map(
            value => -value
          );
        }
        else {
          targetQuaternion = newQuaternion;
        }

        // Start directly at the first reading instead of animating to it.
        if (!orientationReceived) {
          currentQuaternion = [...targetQuaternion];
          orientationReceived = true;
        }
      };

      socket.onerror = function() {
        socket.close();
      };

      socket.onclose = function() {
        statusText.textContent =
          "Disconnected · reconnecting...";

        statusText.style.color = "#ff8a80";

        reconnectTimer =
          setTimeout(connectWebSocket, 1000);
      };
    }

    animate();
    connectWebSocket();
  </script>
</body>
</html>
)HTML";

void handleWebpage() {
  // Send the webpage stored above whenever the browser opens the address.
  webServer.send_P(
    200,
    "text/html",
    webpage);
}

void webSocketEvent(
  uint8_t clientNumber,
  WStype_t eventType,
  uint8_t* payload,
  size_t payloadLength) {
  // These messages are useful when checking connections in Serial Monitor.
  switch (eventType) {
    case WStype_CONNECTED:
      Serial.printf(
        "Browser %u connected\n",
        clientNumber);
      break;

    case WStype_DISCONNECTED:
      Serial.printf(
        "Browser %u disconnected\n",
        clientNumber);
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Start the CodeCell IMU in Roll, Pitch and Yaw mode.
  myCodeCell.Init(MOTION_ROTATION);

  // Access-point mode lets a phone or computer connect without a router.
  WiFi.mode(WIFI_AP);

  // Wi-Fi sleep saves power but can make live movement appear delayed.
  WiFi.setSleep(false);

  WiFi.softAP(
    wifiName,
    wifiPassword);

  Serial.println();
  Serial.println("CodeCell IMU visualizer ready");

  Serial.print("Wi-Fi: ");
  Serial.println(wifiName);

  Serial.print("Password: ");
  Serial.println(wifiPassword);

  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());

  // Start the webpage and live-data servers.
  webServer.on("/", handleWebpage);
  webServer.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  // These must run frequently to keep browser connections responsive.
  webServer.handleClient();
  webSocket.loop();

  // Read the IMU and send one CSV message about 30 times per second.
  // Example message: 12.34,-5.67,98.10
  if (myCodeCell.Run(30)) {
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);

    char rotationData[64];

    snprintf(
      rotationData,
      sizeof(rotationData),
      "%.2f,%.2f,%.2f",
      Roll,
      Pitch,
      Yaw);

    webSocket.broadcastTXT(rotationData);
  }
}
