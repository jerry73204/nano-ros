# nros-platform-esp-idf

Native C implementation of the nano-ros canonical platform ABI (`<nros/platform.h>`) for [Espressif ESP-IDF](https://docs.espressif.com/projects/esp-idf/).

Sibling to `nros-platform-freertos-c` — ESP-IDF ships its own FreeRTOS fork (with SMP support on ESP32 / ESP32-S3), so the threading layer reuses the same `xTaskCreate` + `xSemaphore*` shape. The differences from vanilla FreeRTOS:

| Capability | ESP-IDF override |
|---|---|
| Clock      | `esp_timer_get_time()` — microsecond resolution, monotonic since boot. (FreeRTOS `xTaskGetTickCount` is tick-granular only.) |
| Allocation | `malloc` / `realloc` / `free`. ESP-IDF redirects these to `heap_caps_malloc(MALLOC_CAP_DEFAULT)`. |
| Sleep      | `vTaskDelay` for ≥ one tick; `esp_rom_delay_us` for sub-tick busy-waits. |
| Random     | `esp_random()` (uses the hardware RNG when WiFi / BT are active); `esp_fill_random` for byte fills. |
| Time       | `time(NULL)` — reads the system clock; SNTP / RTC sync drives the value. Returns 0 when no time source is configured. |

Storage layouts (`ZTask`, `ZMutex`, `ZCondvar`) match the Rust `nros-platform-freertos::types` byte-for-byte so this port is wire-compatible with zenoh-pico's FreeRTOS expectations.

## Build as an ESP-IDF component

Register the directory in your IDF project's top-level `CMakeLists.txt`:

```cmake
set(EXTRA_COMPONENT_DIRS path/to/nano-ros/packages/core)
```

Then add `nros-platform-esp-idf` to your project's `REQUIRES` list.

## License

Apache-2.0 or MIT at your option.
