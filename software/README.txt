CYCLING DASHBOARD - UPDATED UI

FILES
- sessiom.html : dashboard structure
- session.css  : layout, styling and SVG gauge appearance
- session.js    : smooth metric animation, demo data, scene/video switching

ADD YOUR FOUR SCENERY VIDEOS
Create a folder named "videos" beside index.html and put these files in it:
  videos/forest.mp4
  videos/beach.mp4
  videos/night-city.mp4
  videos/mountains.mp4

Alternatively, open app.js and replace the four values in SCENE_VIDEOS with direct video URLs.
Use MP4 encoded with H.264 video for best mobile browser compatibility. Videos are intentionally muted,
looped and inline so mobile browsers have the best chance of allowing autoplay.
Some mobile browsers still require the user to tap the on-screen play button once.

RUN
Serve the folder over HTTP (for example, VS Code Live Server). Avoid opening index.html directly
with file:// because browsers may restrict local media loading.

ESP32
The WebSocket example is near the bottom of app.js. Uncomment it when the ESP32 endpoint is ready.
The expected JSON packet is: {"rpm":78,"speed":24.5,"torque":112}
