//! Session configuration

use crate::{Error, Result};
use zenoh_pico_sys::*;

/// Session mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Client mode - connects to a router
    Client,
    /// Peer mode - connects directly to other peers
    Peer,
}

/// Configuration for a zenoh session
pub struct Config {
    inner: z_owned_config_t,
}

impl Config {
    /// Create a new configuration with default settings
    pub fn new() -> Result<Self> {
        unsafe {
            let mut config = core::mem::MaybeUninit::<z_owned_config_t>::uninit();
            let result = z_config_default(config.as_mut_ptr());
            if result < 0 {
                return Err(Error::InvalidConfig);
            }
            Ok(Self {
                inner: config.assume_init(),
            })
        }
    }

    /// Create a client configuration that connects to a router
    pub fn client(locator: &str) -> Result<Self> {
        let mut config = Self::new()?;
        config.set_mode(Mode::Client)?;
        config.set_connect(locator)?;
        Ok(config)
    }

    /// Create a peer configuration
    pub fn peer() -> Result<Self> {
        let mut config = Self::new()?;
        config.set_mode(Mode::Peer)?;
        Ok(config)
    }

    /// Set the session mode
    pub fn set_mode(&mut self, mode: Mode) -> Result<()> {
        unsafe {
            let mode_str = match mode {
                Mode::Client => Z_CONFIG_MODE_CLIENT.as_ptr() as *const i8,
                Mode::Peer => Z_CONFIG_MODE_PEER.as_ptr() as *const i8,
            };
            let loaned = z_config_loan_mut(&mut self.inner);
            zp_config_insert(loaned, Z_CONFIG_MODE_KEY as u8, mode_str);
            Ok(())
        }
    }

    /// Set the connection endpoint
    pub fn set_connect(&mut self, locator: &str) -> Result<()> {
        unsafe {
            // Create null-terminated buffer
            let mut buf = [0u8; 256];
            let bytes = locator.as_bytes();
            let len = bytes.len().min(buf.len() - 1);
            buf[..len].copy_from_slice(&bytes[..len]);

            let loaned = z_config_loan_mut(&mut self.inner);
            zp_config_insert(
                loaned,
                Z_CONFIG_CONNECT_KEY as u8,
                buf.as_ptr() as *const i8,
            );
            Ok(())
        }
    }

    /// Consume the config and return the inner owned type
    pub(crate) fn into_inner(self) -> z_owned_config_t {
        let inner = self.inner;
        core::mem::forget(self);
        inner
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new().expect("Failed to create default config")
    }
}

impl Drop for Config {
    fn drop(&mut self) {
        unsafe {
            z_config_drop(z_config_move(&mut self.inner));
        }
    }
}
