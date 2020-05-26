#!/bin/bash
sleep 3
chromium-browser http://localhost:8080/ --headless --disable-gpu --no-sandbox --remote-debugging-port=9222
