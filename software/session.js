// -----------------------------------------------------------------------------
// VIDEO SETUP
// Put your four videos in a folder named "videos" beside index.html, or replace
// these paths with direct MP4 URLs. MP4/H.264 is the safest browser format.
// Keep the videos muted: mobile browsers commonly block autoplay with sound.
// -----------------------------------------------------------------------------


const $ = (id) => document.getElementById(id);
const video = $("backgroundVideo");

let demoRunning = true;
let demoTimer = null;
let motionFrame = null;
let rpm = 78, speed = 24.5, torque = 112;
let visual = { rpm: 78, speed: 24.5, torque: 112 };
let animationFrom = { ...visual };
let animationTo = { ...visual };
let animationStarted = 0;
const animationDuration = 360;

const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const easeOutCubic = (t) => 1 - Math.pow(1 - t, 3);

// Draw fine, evenly spaced dial marks along the same 270-degree open arc.
function buildTicks(groupId, cx, cy, radius, count, innerLength, outerLength) {
  const group = $(groupId);
  const ns = "http://www.w3.org/2000/svg";
  for (let index = 0; index <= count; index += 1) {
    const angle = (135 + (270 * index / count)) * Math.PI / 180;
    const x1 = cx + Math.cos(angle) * (radius - innerLength);
    const y1 = cy + Math.sin(angle) * (radius - innerLength);
    const x2 = cx + Math.cos(angle) * (radius + outerLength);
    const y2 = cy + Math.sin(angle) * (radius + outerLength);
    const line = document.createElementNS(ns, "line");
    line.setAttribute("x1", x1.toFixed(2));
    line.setAttribute("y1", y1.toFixed(2));
    line.setAttribute("x2", x2.toFixed(2));
    line.setAttribute("y2", y2.toFixed(2));
    group.appendChild(line);
  }
}

buildTicks("rpmTicks", 60, 50, 44, 34, 2.5, 1.2);
buildTicks("torqueTicks", 60, 50, 44, 34, 2.5, 1.2);
buildTicks("speedTicks", 110, 96, 86, 42, 3.2, 1.3);

function paintMetrics(values) {
  $("rpmValue").textContent = Math.round(values.rpm);
  $("speedValue").textContent = values.speed.toFixed(1);
  $("torqueValue").textContent = Math.round(values.torque);

  // SVG pathLength is normalized to 1000, so progress is accurate and smooth.
  $("rpmProgress").style.strokeDashoffset = 1000 * (1 - clamp(values.rpm / 140, 0, 1));
  $("speedProgress").style.strokeDashoffset = 1000 * (1 - clamp(values.speed / 40, 0, 1));
  $("torqueProgress").style.strokeDashoffset = 1000 * (1 - clamp(values.torque / 180, 0, 1));
}

// Interpolates both the readouts and the SVG arcs instead of jumping between values.
function animateMetrics(now) {
  const progress = clamp((now - animationStarted) / animationDuration, 0, 1);
  const eased = easeOutCubic(progress);
  visual = {
    rpm: animationFrom.rpm + (animationTo.rpm - animationFrom.rpm) * eased,
    speed: animationFrom.speed + (animationTo.speed - animationFrom.speed) * eased,
    torque: animationFrom.torque + (animationTo.torque - animationFrom.torque) * eased
  };
  paintMetrics(visual);

  if (progress < 1) {
    motionFrame = requestAnimationFrame(animateMetrics);
  } else {
    visual = { ...animationTo };
    paintMetrics(visual);
    motionFrame = null;
  }
}

function setMetricTargets(next) {
  animationFrom = { ...visual };
  animationTo = {
    rpm: clamp(Number(next.rpm), 0, 140),
    speed: clamp(Number(next.speed), 0, 40),
    torque: clamp(Number(next.torque), 0, 180)
  };
  animationStarted = performance.now();
  if (motionFrame === null) motionFrame = requestAnimationFrame(animateMetrics);
}

function updateFromESP(data) {
  // Expected JSON: {"rpm":78,"speed":24.5,"torque":112}
  if (!data || typeof data !== "object") return;
  const next = {
    rpm: Number.isFinite(Number(data.rpm)) ? Number(data.rpm) : animationTo.rpm,
    speed: Number.isFinite(Number(data.speed)) ? Number(data.speed) : animationTo.speed,
    torque: Number.isFinite(Number(data.torque)) ? Number(data.torque) : animationTo.torque
  };
  setMetricTargets(next);
}

// Demo values let you inspect the UI before the ESP32 is connected.
function startDemo() {
  stopDemo();
  demoRunning = true;
  demoTimer = setInterval(() => {
    rpm = clamp(rpm + (Math.random() - 0.48) * 5, 45, 125);
    speed = clamp(speed + (Math.random() - 0.5) * 1.1, 5, 40);
    torque = clamp(torque + (Math.random() - 0.5) * 7, 35, 175);
    setMetricTargets({ rpm, speed, torque });
  }, 250);
}
function stopDemo() {
  if (demoTimer !== null) clearInterval(demoTimer);
  demoTimer = null;
  demoRunning = false;
}

// -----------------------------------------------------------------------------
// VIDEO HANDLING
// A selected scenery button swaps the video source, reloads it and attempts to
// play it. A tap-to-play control appears if the browser blocks autoplay.
// -----------------------------------------------------------------------------
const SCENE_VIDEOS = {
    forest: "https://www.youtube.com/watch?v=MWyBgudQqsI&pp=ygUYZm9yZXN0IGN5Y2xpbmcgcG92IHZpZGVv",
    beach: "https://www.youtube.com/watch?v=YOUR_BEACH_ID",
    nightCity: "https://www.youtube.com/watch?v=XX8km2vJbkQ&pp=ygUTdG9reW8gbmlnaHQgY3ljbGluZw%3D%3D",
    mountains: "https://www.youtube.com/shorts/1euzUN4eSkE"
};

const youtubeFrame = document.getElementById("youtubeBackground");

function getYouTubeID(url) {
    try {
        const parsed = new URL(url);

        if (parsed.hostname.includes("youtu.be")) {
            return parsed.pathname.substring(1);
        }

        if (parsed.hostname.includes("youtube.com")) {
            if (parsed.pathname === "/watch") {
                return parsed.searchParams.get("v");
            }

            if (parsed.pathname.startsWith("/embed/") || parsed.pathname.startsWith("/shorts/")) {
                return parsed.pathname.split("/")[2];
            }
        }
    } catch (error) {
        console.error("Invalid YouTube URL", error);
    }

    return null;
}

function changeScene(scene) {
    const url = SCENE_VIDEOS[scene];
    const videoID = getYouTubeID(url);

    if (!videoID) {
        console.error("Invalid YouTube video URL");
        return;
    }

    // Stop the previous MP4 video, if any.
    video.pause();
    video.removeAttribute("src");
    video.load();

    // Load the selected YouTube video.
    youtubeFrame.src =
        `https://www.youtube.com/embed/${videoID}` +
        `?autoplay=1&mute=1&controls=0&loop=1` +
        `&playlist=${videoID}&playsinline=1&rel=0`;
}

// ESP32 WebSocket integration — uncomment when the firmware endpoint is ready.
// const socket = new WebSocket("ws://192.168.4.1/ws");
// socket.addEventListener("message", (event) => {
//   try { updateFromESP(JSON.parse(event.data)); }
//   catch (error) { console.warn("Invalid ESP32 data packet", error); }
// });
// socket.addEventListener("close", () => console.log("ESP32 disconnected"));

function updateClock() {
  $("clock").textContent = new Intl.DateTimeFormat([], {
    hour: "2-digit", minute: "2-digit"
  }).format(new Date());
}
updateClock();
setInterval(updateClock, 15000);
paintMetrics(visual);
startDemo();
changeScene("forest");

$("menuButton").addEventListener("click", () => $("menuPanel").classList.toggle("open"));
$("demoToggle").addEventListener("click", (event) => {
  if (demoRunning) {
    stopDemo();
    event.currentTarget.textContent = "Resume demo data";
  } else {
    startDemo();
    event.currentTarget.textContent = "Pause demo data";
  }
});
$("fullscreenButton").addEventListener("click", async () => {
  try {
    if (!document.fullscreenElement) await $("dashboard").requestFullscreen();
    else await document.exitFullscreen();
  } catch (error) {
    console.warn("Fullscreen unavailable", error);
  }
});

const switchSessionButton = $("switchSessionButton");
const backDashboardButton = $("backDashboardButton");
if (window.location.pathname.startsWith("/session")) {
  if (switchSessionButton) {
    switchSessionButton.hidden = false;
    switchSessionButton.addEventListener("click", () => {
      window.location.assign("/session/");
    });
  }
  if (backDashboardButton) {
    backDashboardButton.hidden = false;
    backDashboardButton.addEventListener("click", () => {
      window.location.assign("/dashboard");
    });
  }
}

document.querySelectorAll(".scene-option").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll(".scene-option").forEach((item) => {
      item.classList.remove("active");
      item.setAttribute("aria-pressed", "false");
    });
    button.classList.add("active");
    button.setAttribute("aria-pressed", "true");
    $("sceneName").textContent = button.dataset.label;
    changeScene(button.dataset.scene);
  });
});
