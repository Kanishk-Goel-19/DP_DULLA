# DOULA — Knee Rehabilitation Website Prototype (Exercise Tracker)

A complete, dependency-free Node.js prototype with login, Gmail OTP, dashboard, and the Exercise Mode Tracker interface adapted from Priyanshi's Figma export. The exercise interface runs inside the existing site; there is no separate React/Vite server to launch.

## 1. Run the website

Install **Node.js 22 or later**. From the `DOULA_Website` folder, run:

```bash
npm start
```

Open **http://localhost:3000** in a browser. No `npm install` is required: this version uses only built-in Node modules and normal HTML, CSS, and JavaScript.

## 2. Create a new account

1. Click **Make a new account**.
2. Enter a name and a Gmail address.
3. Enter the **4-digit OTP**.
4. Set a password (one field, at least 6 characters).
5. The dashboard opens automatically. Next time, use Gmail + password to log in.

### Where does the OTP go?

**Local demo (works immediately):** If you have not configured email, look in the same Terminal window where you ran `npm start`. It prints the 4-digit code for that email. This is an intentional local-only fallback, **not** a delivered email.

**Real Gmail delivery:**

1. Copy `.env.example` to `.env`.
2. Set `SMTP_USER` to a Gmail address that will send the OTP.
3. Turn on Google 2-Step Verification on that sending account and create a **Gmail App Password**.
4. Put the app password in `SMTP_APP_PASSWORD`. Do **not** use your normal Gmail password.
5. Restart `npm start`.

The backend then emails the OTP using Gmail SMTP. The receiver can use their own Gmail account. Credentials stay on the backend; they are never placed in browser code. Email delivery requires a working internet connection and valid Google app-password eligibility.

There is no Confirm Password, Forgot Password, or other account-management UI.

## 3. Dashboard, exercise modes and logout

- The dashboard matches the white/olive desktop and mobile images in `design-reference/`.
- **Log Out** ends the current session and returns to the login screen.
- **Connect Device** checks the configured ESP32 WebSocket endpoint. On success, it opens the Exercise Mode Tracker.
- **Try without ESP32** always opens the same interface in clearly marked demo mode, even when no hardware is powered on.
- Select **Flexion–Extension** and then **Support** or **Resistance**, or select **Cycling**.
- Enter **1–99 repetitions**. Start, pause/resume, and finish the timer-based trial. The cycling mode also embeds the video from the Figma export when internet access allows.
- When the timed countdown finishes, the backend automatically saves the exercise type, number of repetitions, date/time and trial mode to `backend/data/progress.json`.
- **Track Progress** displays the saved history for the currently logged-in account, plus a session count and total repetitions. A user only sees their own records.
- **All counts are simulated** in this prototype. Even after an ESP32 WebSocket responds, the rep countdown is **not** ESP32 telemetry. Saved records are labeled **Demo** or **Connected preview**; neither represents validated physical exercise.
- **How-to** and **About Us** are simple placeholders awaiting your documents.

## 4. ESP32 — Connect Device button

When U1/ESP_MASTER's WebSocket URL is known:

1. Edit `.env` and set `ESP32_WS_URL`, such as `ws://192.168.4.1:81/` **only if that matches your actual firmware**.
2. The computer running the backend must be able to reach the ESP32 on the same network.
3. Restart the server.
4. Click **Connect Device**. The backend attempts a WebSocket handshake with U1.
5. If the handshake succeeds, the browser opens the Exercise Mode Tracker. If it fails, the dashboard stays open; you can still click **Try without ESP32**.

A successful handshake only means the ESP32 WebSocket endpoint responded; it does not prove that every sensor or mechanical subsystem is healthy. A proper heartbeat/health response can be added once firmware protocol is shared.

The exercise interface currently contains **no commands that move motors** or alter ESP32 control loops. DOULA's emergency stop and all mechanical motion limits must remain enforced by hardware/firmware, independent of the browser. Real device-based repetition counts and actuation require a separately specified, tested ESP32 protocol.

## 5. Folder guide

- `public/index.html`: HTML entry point
- `public/styles.css`: responsive DOULA styles
- `public/app.js`: frontend logic, navigation, and form interactions
- `public/exercise.js`: integrated Figma exercise-mode UI, timer simulation, and session saving
- `public/exercise.css`: visual styles for the exercise-mode UI, adapted to the dashboard
- `public/assets/rehabilitation.jpg`: rehabilitation image cropped from the supplied Figma screenshot
- `server.mjs`: web server and API routes
- `backend/core.mjs`: users, four-digit OTP validation, password hashing, login sessions, and saved exercise records
- `backend/email.mjs`: Gmail SMTP sender
- `backend/data/`: stores users and exercise history locally (created when needed; never share users.json or progress.json)
- `design-reference/`: the user-provided dashboard screenshots and assets
- `tests/`: basic backend tests

## Notes

This is for a **local prototype**, not an internet-exposed clinical service. User and exercise records are stored locally as JSON. OTP requests and login sessions are held in memory, so restarting the server during signup resets the in-progress OTP; registered accounts and saved session history remain available. Passwords are hashed with `scrypt` and cookie sessions are HTTP-only. A public deployment would need additional protections, appropriate safeguards for rehabilitation records, and secure HTTPS configuration.

## 6. Updating the team GitHub repository

In your cloned `DP_DULLA` repository, replace **the contents of `software_new/`** with the contents of this `DOULA_Website/` folder (do not delete the unrelated sibling `software/` folder). Use GitHub Desktop to review the files, commit your changes, push the branch, and create a Pull Request. Do not commit `.env`, `backend/data/users.json`, or `backend/data/progress.json`.
