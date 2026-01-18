use zenoh_pico::{Config, KeyExpr, Session};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Duration;

fn main() {
    println!("Simple zenoh subscriber for demo/chatter");
    
    let config = Config::client("tcp/127.0.0.1:7447").unwrap();
    let session = Session::open(config).unwrap();
    println!("Connected to zenoh router");
    
    let keyexpr = KeyExpr::new("demo/chatter").unwrap();
    let count = Arc::new(AtomicUsize::new(0));
    let count_clone = count.clone();
    
    let _subscriber = session.declare_subscriber(&keyexpr, move |sample| {
        let c = count_clone.fetch_add(1, Ordering::SeqCst);
        print!("Received {} bytes: ", sample.payload.len());
        for b in sample.payload.iter().take(16) {
            print!("{:02x} ", b);
        }
        // Try to decode as CDR Int32
        if sample.payload.len() >= 8 {
            let value = i32::from_le_bytes([
                sample.payload[4], sample.payload[5], 
                sample.payload[6], sample.payload[7]
            ]);
            println!(" -> Int32 value: {}", value);
        } else {
            println!();
        }
    }).unwrap();
    println!("Subscribed to demo/chatter, waiting for messages...");
    
    for _ in 0..100 {
        std::thread::sleep(Duration::from_millis(100));
        if count.load(Ordering::SeqCst) >= 3 {
            println!("Received enough messages, test passed!");
            return;
        }
    }
    if count.load(Ordering::SeqCst) == 0 {
        println!("No messages received!");
    }
}
