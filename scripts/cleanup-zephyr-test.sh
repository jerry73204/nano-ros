#!/bin/bash
# Cleanup script for Zephyr E2E tests
# Kills any lingering zenohd and Zephyr processes

# Kill Zephyr processes
pkill -9 -f 'zephyr\.exe' 2>/dev/null || true

# Kill zenohd
pkill -9 zenohd 2>/dev/null || true

# Wait for processes to terminate
sleep 1

# Verify cleanup
remaining=$(pgrep -f 'zephyr\.exe|zenohd' 2>/dev/null || true)
if [ -n "$remaining" ]; then
    echo "Warning: Some processes may still be running:"
    ps aux | grep -E 'zephyr\.exe|zenohd' | grep -v grep
else
    echo "Cleanup complete"
fi
