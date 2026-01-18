// Quick standalone subscriber test for Zephyr E2E testing
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

fn main() {
    let config = zenoh_pico::Config::client("tcp/127.0.0.1:7447").expect("config");
    let session = zenoh_pico::Session::open(config).expect("session");
    eprintln!("[sub] Connected to zenoh router");

    let keyexpr = zenoh_pico::KeyExpr::new("demo/chatter").expect("keyexpr");
    let count = Arc::new(AtomicUsize::new(0));
    let count_clone = count.clone();

    let _sub = session
        .declare_subscriber(&keyexpr, move |sample| {
            let n = count_clone.fetch_add(1, Ordering::SeqCst);
            let payload = &sample.payload;

            // Try to decode as Int32 (raw little-endian, no CDR header)
            if payload.len() >= 4 {
                let v = i32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                eprintln!(
                    "[sub] #{} Received Int32: {} ({} bytes)",
                    n,
                    v,
                    payload.len()
                );
            } else {
                eprintln!(
                    "[sub] #{} Received {} bytes: {:02x?}",
                    n,
                    payload.len(),
                    payload
                );
            }
        })
        .expect("subscriber");

    eprintln!("[sub] Waiting for messages on demo/chatter...");

    for _ in 0..100 {
        std::thread::sleep(std::time::Duration::from_millis(100));
        if count.load(Ordering::SeqCst) >= 3 {
            eprintln!("[sub] SUCCESS: received messages from Zephyr!");
            return;
        }
    }
    eprintln!("[sub] TIMEOUT: no messages received");
}
