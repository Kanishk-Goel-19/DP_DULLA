# DOULA — Knee Rehabilitation Website Prototype

A complete, small, dependency-free Node.js website with the DOULA login and responsive dashboard based on Priyanshi's uploaded Figma screenshots and login-page ZIP.

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

## 3. Dashboard and logout

- The dashboard matches the white/olive desktop and mobile images in `design-reference/`.
- **Log Out** ends the current session and returns to the login screen.
- **Track Progress** shows saved sessions (initially empty, because device telemetry has not been integrated).
- **How-to** and **About Us** are simple placeholders awaiting your documents.

## 4. ESP32 — Connect Device button

When U1/ESP_MASTER's WebSocket URL is known:

1. Edit `.env` and set `ESP32_WS_URL`, such as `ws://192.168.4.1:81/` **only if that matches your actual firmware**.
2. The computer running the backend must be able to reach the ESP32 on the same network.
3. Restart the server.
4. Click **Connect Device**. The backend attempts a WebSocket handshake with U1.
5. If the handshake succeeds, the browser goes to the new device page. If it fails, a message is shown and the dashboard stays open.

A successful handshake only means the ESP32 WebSocket endpoint responded; it does not prove that every sensor or mechanical subsystem is healthy. A proper heartbeat/health response can be added once firmware protocol is shared.

The device page currently contains **no commands that move motors**. DOULA's emergency stop and all mechanical motion limits must remain enforced by the hardware/firmware, independent of the browser.

## 5. Folder guide

- `public/index.html`: HTML entry point
- `public/styles.css`: responsive DOULA styles
- `public/app.js`: frontend logic, navigation, and form interactions
- `public/assets/rehabilitation.jpg`: rehabilitation image cropped from the supplied Figma screenshot
- `server.mjs`: web server and API routes
- `backend/core.mjs`: users, four-digit OTP validation, password hashing, sessions
- `backend/email.mjs`: Gmail SMTP sender
- `backend/data/`: stores users locally (created on first signup; never share users.json)
- `design-reference/`: the user-provided dashboard screenshots and assets
- `tests/`: basic backend tests

## Notes

This is for a **local prototype**, not an internet-exposed clinical service. User records are stored locally as JSON. OTP requests and active sessions are held in memory, so restarting the server during a signup resets the in-progress OTP; registered users remain saved. Passwords are hashed with `scrypt` and cookie sessions are HTTP-only. A public deployment would need additional protections and secure HTTPS configuration.
