#!/bin/bash
sleep 1
chromium http://localhost:8080/ --headless --disable-gpu --remote-debugging-port=9222
