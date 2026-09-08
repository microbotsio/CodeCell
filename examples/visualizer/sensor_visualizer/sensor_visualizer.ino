/*
  CodeCell Wi-Fi Sensor Visualizer

  The CodeCell creates its own Wi-Fi network and hosts a webpage.
  The webpage displays a 3D board plus live readings for proximity,
  ambient light, acceleration, gyroscope, magnetometer and steps.

  A WebSocket sends the angles continuously at about 30 updates per
  second. Roll, Pitch and Yaw directly control the 3D board.

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

float AccelX = 0.0;
float AccelY = 0.0;
float AccelZ = 0.0;

float GyroX = 0.0;
float GyroY = 0.0;
float GyroZ = 0.0;

float MagX = 0.0;
float MagY = 0.0;
float MagZ = 0.0;

uint16_t Proximity = 0;
uint16_t AmbientLight = 0;
uint16_t StepCount = 0;

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

  <title>CodeCell Sensor Visualizer</title>

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
      transition: color 120ms linear;
    }

    body::before {
      content: "";
      position: fixed;
      inset: 0;
      pointer-events: none;
      background-image:
        linear-gradient(rgba(255, 102, 0, 0.045) 1px, transparent 1px),
        linear-gradient(90deg, rgba(255, 102, 0, 0.045) 1px, transparent 1px);
      background-size: 42px 42px;
      mask-image: radial-gradient(circle at 43% 46%, black, transparent 72%);
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

    .toolbar {
      display: flex;
      gap: 8px;
      margin-top: 13px;
    }

    .toolbar button {
      padding: 7px 12px;
      color: inherit;
      background: rgba(255, 102, 0, 0.13);
      border: 1px solid rgba(255, 140, 70, 0.55);
      border-radius: 999px;
      cursor: pointer;
      font-weight: bold;
    }

    .toolbar button:hover { background: rgba(255, 102, 0, 0.28); }

    .scene {
      position: absolute;
      top: 47%;
      left: 43%;
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

      /* Avoid CSS taking the long route when an angle wraps at ±180°. */
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
      left: 43%;
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

    .sensor-panel {
      position: absolute;
      top: 95px;
      right: 30px;
      display: grid;
      grid-template-columns: repeat(2, minmax(220px, 1fr));
      width: min(510px, 48vw);
      gap: 10px;
    }

    .sensor-card {
      padding: 12px 14px;
      background: rgba(255, 102, 0, 0.09);
      border: 1px solid rgba(255, 102, 0, 0.35);
      border-radius: 12px;
      min-height: 128px;
    }

    .mini-chart {
      display: block;
      width: 100%;
      height: 58px;
      margin-top: 7px;
    }

    .sensor-row { display: flex; align-items: stretch; gap: 10px; }
    .sensor-row .axis-values {
      flex: 0 0 66px;
      grid-template-columns: 1fr;
      align-content: center;
    }
    .sensor-row .mini-chart { flex: 1; min-width: 0; }

    .reading .mini-chart { height: 34px; margin-top: 5px; }

    .sensor-title {
      margin-bottom: 6px;
      color: #ffb380;
      font-size: 12px;
      text-transform: uppercase;
    }

    .sensor-value {
      font-size: 20px;
      font-weight: bold;
    }

    .sensor-note {
      margin-top: 4px;
      color: #999;
      font-size: 11px;
    }

    .axis-values {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 5px;
      font-size: 14px;
    }

    @media (max-width: 1100px) and (min-width: 701px) {
      .scene, .readings { left: 27%; }
      .scene { transform: translate(-50%, -50%) scale(0.82); }
      .sensor-panel { right: 15px; width: 50vw; }
      .sensor-card { padding: 9px 11px; }
    }

    @media (max-width: 700px) {
      body {
        overflow-y: auto;
      }

      .scene {
        top: 285px;
        left: 50%;
        transform: translate(-50%, -50%) scale(0.68);
      }

      .readings {
        position: absolute;
        top: 445px;
        bottom: auto;
        left: 50%;
        gap: 5px;
      }

      .reading {
        width: 100px;
      }

      .sensor-panel {
        position: absolute;
        top: 525px;
        right: auto;
        left: 50%;
        width: calc(100% - 30px);
        grid-template-columns: 1fr;
        padding-bottom: 25px;
        transform: translateX(-50%);
      }

      .sensor-card { min-height: 120px; }
    }
  </style>
</head>

<body>
  <div class="header">
    <h1>CodeCell Sensors</h1>
    <div id="status">Connecting...</div>
    <div class="toolbar">
      <button id="freezeButton" type="button">Freeze</button>
      <button id="fullscreenButton" type="button">Full screen</button>
    </div>
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
      <canvas id="rollChart" class="mini-chart"></canvas>
    </div>

    <div class="reading">
      <div class="label">Pitch</div>
      <div id="pitch" class="value">0.0°</div>
      <canvas id="pitchChart" class="mini-chart"></canvas>
    </div>

    <div class="reading">
      <div class="label">Yaw</div>
      <div id="yaw" class="value">0.0°</div>
      <canvas id="yawChart" class="mini-chart"></canvas>
    </div>
  </div>

  <div class="sensor-panel">
    <div class="sensor-card">
      <div class="sensor-title">Proximity</div>
      <div id="proximity" class="sensor-value">0</div>
      <div class="sensor-note">Raw value · higher means closer</div>
      <canvas id="proximityChart" class="mini-chart"></canvas>
    </div>

    <div class="sensor-card">
      <div class="sensor-title">Ambient light</div>
      <div id="ambient" class="sensor-value">0</div>
      <div class="sensor-note">Raw sensor value</div>
      <canvas id="ambientChart" class="mini-chart"></canvas>
    </div>

    <div class="sensor-card">
      <div class="sensor-title">Acceleration</div>
      <div class="sensor-row">
        <div class="axis-values">
          <span>X: <b id="accelX">0.00</b></span>
          <span>Y: <b id="accelY">0.00</b></span>
          <span>Z: <b id="accelZ">0.00</b></span>
        </div>
        <canvas id="accelChart" class="mini-chart"></canvas>
      </div>
      <div class="sensor-note">Measured in g</div>
    </div>

    <div class="sensor-card">
      <div class="sensor-title">Raw gyroscope</div>
      <div class="sensor-row">
        <div class="axis-values">
          <span>X: <b id="gyroX">0.00</b></span>
          <span>Y: <b id="gyroY">0.00</b></span>
          <span>Z: <b id="gyroZ">0.00</b></span>
        </div>
        <canvas id="gyroChart" class="mini-chart"></canvas>
      </div>
      <div class="sensor-note">Measured in rad/s</div>
    </div>

    <div class="sensor-card">
      <div class="sensor-title">Raw magnetometer</div>
      <div class="sensor-row">
        <div class="axis-values">
          <span>X: <b id="magX">0.00</b></span>
          <span>Y: <b id="magY">0.00</b></span>
          <span>Z: <b id="magZ">0.00</b></span>
        </div>
        <canvas id="magChart" class="mini-chart"></canvas>
      </div>
      <div class="sensor-note">Measured in µT</div>
    </div>

    <div class="sensor-card">
      <div class="sensor-title">Step counter</div>
      <div id="steps" class="sensor-value">0</div>
      <canvas id="stepsChart" class="mini-chart"></canvas>
    </div>

  </div>

  <script>
    const board = document.getElementById("board");
    const statusText = document.getElementById("status");
    const freezeButton = document.getElementById("freezeButton");
    const fullscreenButton = document.getElementById("fullscreenButton");

    const rollText = document.getElementById("roll");
    const pitchText = document.getElementById("pitch");
    const yawText = document.getElementById("yaw");

    const proximityText = document.getElementById("proximity");
    const ambientText = document.getElementById("ambient");
    const accelXText = document.getElementById("accelX");
    const accelYText = document.getElementById("accelY");
    const accelZText = document.getElementById("accelZ");
    const gyroXText = document.getElementById("gyroX");
    const gyroYText = document.getElementById("gyroY");
    const gyroZText = document.getElementById("gyroZ");
    const magXText = document.getElementById("magX");
    const magYText = document.getElementById("magY");
    const magZText = document.getElementById("magZ");
    const stepsText = document.getElementById("steps");

    const mainFaces = document.querySelectorAll(".front, .back");
    const sideFaces = document.querySelectorAll(".left, .right, .top, .bottom");

    let socket;
    let reconnectTimer;
    let displayRoll = null;
    let displayPitch = null;
    let displayYaw = null;
    let displayedAmbient = 0;
    let displayedProximity = 0;
    let frozen = false;

    const histories = {
      roll: [], pitch: [], yaw: [], proximity: [], ambient: [],
      accelX: [], accelY: [], accelZ: [],
      gyroX: [], gyroY: [], gyroZ: [],
      magX: [], magY: [], magZ: [], steps: []
    };

    const AMBIENT_DARK_LEVEL = 100;
    const AMBIENT_WHITE_LEVEL = 65000;

    // Adjust this if your normal close-range proximity values are different.
    const PROXIMITY_COLOR_MAX = 1000;

    function clamp(value, minimum, maximum) {
      return Math.min(maximum, Math.max(minimum, value));
    }

    function pushHistory(name, value) {
      histories[name].push(value);
      if (histories[name].length > 90) histories[name].shift();
    }

    function drawChart(canvasId, series, colours) {
      const canvas = document.getElementById(canvasId);
      const width = canvas.clientWidth;
      const height = canvas.clientHeight;
      const allValues = series.flat();
      if (!width || !height || allValues.length < 2) return;

      const scale = window.devicePixelRatio || 1;
      canvas.width = width * scale;
      canvas.height = height * scale;
      const context = canvas.getContext("2d");
      context.scale(scale, scale);
      context.clearRect(0, 0, width, height);

      let minimum = Math.min(...allValues);
      let maximum = Math.max(...allValues);
      if (maximum === minimum) maximum = minimum + 1;

      series.forEach((values, seriesIndex) => {
        if (values.length < 2) return;
        const points = values.map((value, index) => ({
          x: index * width / (values.length - 1),
          y: height - 5 - (value - minimum) / (maximum - minimum) * (height - 10)
        }));

        if (series.length === 1) {
          const fill = context.createLinearGradient(0, 0, 0, height);
          fill.addColorStop(0, colours[0] + "55");
          fill.addColorStop(1, colours[0] + "00");
          context.beginPath();
          context.moveTo(points[0].x, height);
          points.forEach(point => context.lineTo(point.x, point.y));
          context.lineTo(points[points.length - 1].x, height);
          context.closePath();
          context.fillStyle = fill;
          context.fill();
        }

        context.beginPath();
        points.forEach((point, index) =>
          index ? context.lineTo(point.x, point.y) : context.moveTo(point.x, point.y)
        );
        context.strokeStyle = colours[seriesIndex];
        context.lineWidth = 2;
        context.shadowColor = colours[seriesIndex];
        context.shadowBlur = 5;
        context.stroke();
      });
    }

    function updateSensorColours(ambient, proximity) {
      // Gentle filtering prevents small sensor changes from flickering onscreen.
      displayedAmbient += (ambient - displayedAmbient) * 0.12;
      displayedProximity += (proximity - displayedProximity) * 0.2;

      const lightLevel = clamp(
        (displayedAmbient - AMBIENT_DARK_LEVEL) /
        (AMBIENT_WHITE_LEVEL - AMBIENT_DARK_LEVEL),
        0,
        1
      );

      const edgeShade = Math.round(16 + lightLevel * 239);
      const centreShade = Math.min(255, edgeShade + 32);

      document.body.style.background =
        `radial-gradient(circle at center,
          rgb(${centreShade}, ${centreShade}, ${centreShade}) 0%,
          rgb(${edgeShade}, ${edgeShade}, ${edgeShade}) 72%)`;

      // Switch to dark text when the background becomes bright.
      document.body.style.color = lightLevel > 0.55 ? "#181818" : "#ffffff";

      const proximityLevel = clamp(
        displayedProximity / PROXIMITY_COLOR_MAX,
        0,
        1
      );

      // The board changes from CodeCell orange toward yellow as an object nears.
      const hue = 24 + proximityLevel * 34;
      const faceLightness = 50 + proximityLevel * 18;
      const sideLightness = 32 + proximityLevel * 15;

      mainFaces.forEach(face => {
        face.style.background =
          `hsl(${hue}, 100%, ${faceLightness}%)`;
      });

      sideFaces.forEach(face => {
        face.style.background =
          `hsl(${hue}, 100%, ${sideLightness}%)`;
      });
    }

    // Keep each angle continuous when the IMU crosses -180° or +180°.
    function unwrapAngle(newAngle, previousAngle) {
      if (previousAngle === null) {
        return newAngle;
      }

      const previousWrapped =
        ((previousAngle + 180) % 360 + 360) % 360 - 180;

      let difference = newAngle - previousWrapped;

      if (difference > 180) difference -= 360;
      if (difference < -180) difference += 360;

      return previousAngle + difference;
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
        // Start fresh after reconnecting to the CodeCell.
        displayRoll = null;
        displayPitch = null;
        displayYaw = null;

        statusText.textContent = "Live · 30 Hz";
        statusText.style.color = "#ff6600";
      };

      socket.onmessage = function(event) {
        if (frozen) return;
        const values = event.data.split(",");

        // roll,pitch,yaw,proximity,light,accel xyz,gyro xyz,mag xyz,steps
        if (values.length !== 15) {
          return;
        }

        const data = values.map(Number);

        if (!data.every(Number.isFinite)) {
          return;
        }

        const [
          roll, pitch, yaw,
          proximity, ambient,
          accelX, accelY, accelZ,
          gyroX, gyroY, gyroZ,
          magX, magY, magZ,
          steps
        ] = data;

        rollText.textContent = roll.toFixed(1) + "°";
        pitchText.textContent = pitch.toFixed(1) + "°";
        yawText.textContent = yaw.toFixed(1) + "°";

        displayRoll = unwrapAngle(roll, displayRoll);
        displayPitch = unwrapAngle(pitch, displayPitch);
        displayYaw = unwrapAngle(yaw, displayYaw);

        // Apply the readable Roll, Pitch and Yaw values directly.
        board.style.transform =
          `rotateZ(${displayYaw}deg)
           rotateY(${displayPitch}deg)
           rotateX(${displayRoll}deg)`;

        proximityText.textContent = Math.round(proximity);
        ambientText.textContent = Math.round(ambient);
        accelXText.textContent = accelX.toFixed(2);
        accelYText.textContent = accelY.toFixed(2);
        accelZText.textContent = accelZ.toFixed(2);
        gyroXText.textContent = gyroX.toFixed(2);
        gyroYText.textContent = gyroY.toFixed(2);
        gyroZText.textContent = gyroZ.toFixed(2);
        magXText.textContent = magX.toFixed(2);
        magYText.textContent = magY.toFixed(2);
        magZText.textContent = magZ.toFixed(2);
        stepsText.textContent = Math.round(steps);
        pushHistory("roll", roll);
        pushHistory("pitch", pitch);
        pushHistory("yaw", yaw);
        pushHistory("proximity", proximity);
        pushHistory("ambient", ambient);
        pushHistory("accelX", accelX); pushHistory("accelY", accelY); pushHistory("accelZ", accelZ);
        pushHistory("gyroX", gyroX); pushHistory("gyroY", gyroY); pushHistory("gyroZ", gyroZ);
        pushHistory("magX", magX); pushHistory("magY", magY); pushHistory("magZ", magZ);
        pushHistory("steps", steps);

        const xyzColours = ["#ff6b5f", "#55e6a5", "#65b7ff"];
        drawChart("rollChart", [histories.roll], ["#ff8a3d"]);
        drawChart("pitchChart", [histories.pitch], ["#ffd45c"]);
        drawChart("yawChart", [histories.yaw], ["#65b7ff"]);
        drawChart("proximityChart", [histories.proximity], ["#ff8a3d"]);
        drawChart("ambientChart", [histories.ambient], ["#ffd45c"]);
        drawChart("accelChart", [histories.accelX, histories.accelY, histories.accelZ], xyzColours);
        drawChart("gyroChart", [histories.gyroX, histories.gyroY, histories.gyroZ], xyzColours);
        drawChart("magChart", [histories.magX, histories.magY, histories.magZ], xyzColours);
        drawChart("stepsChart", [histories.steps], ["#55e6a5"]);

        updateSensorColours(ambient, proximity);

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

    freezeButton.addEventListener("click", function() {
      frozen = !frozen;
      freezeButton.textContent = frozen ? "Resume" : "Freeze";
      statusText.textContent = frozen ? "Frozen · inspect readings" : "Live · 30 Hz";
    });

    fullscreenButton.addEventListener("click", function() {
      if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen();
      } else {
        document.exitFullscreen();
      }
    });

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

  // The | symbol enables several CodeCell sensors at the same time.
  myCodeCell.Init(LIGHT + MOTION_ROTATION + MOTION_ACCELEROMETER + MOTION_GYRO + MOTION_MAGNETOMETER + MOTION_STEP_COUNTER);

  // Access-point mode lets a phone or computer connect without a router.
  WiFi.mode(WIFI_AP);

  // Wi-Fi sleep saves power but can make live movement appear delayed.
  WiFi.setSleep(false);

  WiFi.softAP(wifiName,wifiPassword);

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

  // Read the sensors and send one CSV message about 30 times per second.
  if (myCodeCell.Run(30)) {
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);

    myCodeCell.Motion_AccelerometerRead(AccelX, AccelY, AccelZ);

    myCodeCell.Motion_GyroRead(GyroX, GyroY, GyroZ);

    myCodeCell.Motion_MagnetometerRead(MagX, MagY, MagZ);

    Proximity = myCodeCell.Light_ProximityRead();
    AmbientLight = myCodeCell.Light_AmbientRead();
    StepCount = myCodeCell.Motion_StepCounterRead();

    char sensorData[220];

    snprintf(
      sensorData,
      sizeof(sensorData),
      "%.2f,%.2f,%.2f,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u",
      Roll,
      Pitch,
      Yaw,
      Proximity,
      AmbientLight,
      AccelX,
      AccelY,
      AccelZ,
      GyroX,
      GyroY,
      GyroZ,
      MagX,
      MagY,
      MagZ,
      StepCount
    );

    webSocket.broadcastTXT(sensorData);
  }
}
