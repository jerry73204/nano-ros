#!/usr/bin/env python3
"""Debug script to see what Zenoh keys are being used."""

import time
import sys

try:
    import zenoh
except ImportError:
    print("zenoh-python not installed. Install with: pip install eclipse-zenoh")
    sys.exit(1)

def main():
    print("Opening Zenoh session in client mode...")
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')

    session = zenoh.open(config)
    print(f"Session opened, ZID: {session.zid()}")

    # Subscribe to ALL keys to see what's being published
    print("\nSubscribing to ** (all keys)...")
    print("Will show any keys that receive data:\n")

    def callback(sample):
        print(f"KEY: {sample.key_expr}")
        print(f"  payload: {len(sample.payload)} bytes: {bytes(sample.payload)[:40]}...")
        print()

    subscriber = session.declare_subscriber("**", callback)

    # Wait for messages
    start = time.time()
    while time.time() - start < 10:
        time.sleep(0.1)

    subscriber.undeclare()
    session.close()
    print("Done.")

if __name__ == "__main__":
    main()
