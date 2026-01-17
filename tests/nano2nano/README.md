# nano-ros to nano-ros Tests

Tests for communication between two nano-ros nodes.

## Tests

### Basic Pub/Sub
Tests that a nano-ros talker can publish messages that a nano-ros listener receives.

### Multiple Messages
Tests that multiple messages are received correctly over time.

## Usage

```bash
# Run all tests (uses zenohd router)
./tests/nano2nano/run.sh

# Run in peer mode (no router required)
./tests/nano2nano/run.sh --peer

# Verbose output
./tests/nano2nano/run.sh --verbose
```

## Requirements

- zenohd (unless using `--peer` mode)
- nano-ros built with zenoh feature

## Architecture

```
┌─────────────┐     ┌─────────┐     ┌──────────────┐
│ nano-ros    │────▶│ zenohd  │────▶│ nano-ros     │
│ talker      │     │ router  │     │ listener     │
└─────────────┘     └─────────┘     └──────────────┘
```

In peer mode:
```
┌─────────────┐                     ┌──────────────┐
│ nano-ros    │────────────────────▶│ nano-ros     │
│ talker      │   (peer discovery)  │ listener     │
└─────────────┘                     └──────────────┘
```
