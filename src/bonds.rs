//! Persists BLE bonds in the `nvs` flash partition so paired phones are
//! remembered across reboots.
//!
//! Each bond is a sequential-storage map item keyed by the peer's identity
//! address. Re-bonding the same peer appends a newer copy; on load the last copy
//! wins because `Stack::add_bond_information` replaces matching identities.

use core::ops::Range;

use embassy_embedded_hal::adapter::BlockingAsync;
use esp_bootloader_esp_idf::partitions::{self, FlashRegion};
use esp_println::println;
use esp_storage::FlashStorage;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{self, Key, SerializationError, Value};
use trouble_host::prelude::*;

/// trouble-host 0.6 holds at most 10 bonds in RAM (its private `BI_COUNT`).
pub const MAX_BONDS: usize = 10;

/// Fits one serialized item: 6-byte key + 34-byte value + item header.
const BUFFER_LEN: usize = 64;

type Flash<'a> = BlockingAsync<FlashRegion<'a, FlashStorage<'static>>>;
type StorageError = sequential_storage::Error<partitions::Error>;

pub struct BondStore<'a> {
    flash: Flash<'a>,
    range: Range<u32>,
}

impl<'a> BondStore<'a> {
    pub fn new(partition: FlashRegion<'a, FlashStorage<'static>>) -> Self {
        let range = 0..partition.partition_size() as u32;
        Self {
            flash: BlockingAsync::new(partition),
            range,
        }
    }

    /// Adds every stored bond to the stack. If the partition holds anything we
    /// can't parse (e.g. leftovers from ESP-IDF's own NVS), it is wiped.
    pub async fn load_into<C: Controller, P: PacketPool>(&mut self, stack: &Stack<'_, C, P>) {
        if let Err(e) = self.try_load_into(stack).await {
            println!("[bond] storage unreadable ({:?}), erasing it", e);
            if let Err(e) = sequential_storage::erase_all(&mut self.flash, self.range.clone()).await {
                println!("[bond] erase failed: {:?}", e);
            }
        }
    }

    async fn try_load_into<C: Controller, P: PacketPool>(
        &mut self,
        stack: &Stack<'_, C, P>,
    ) -> Result<(), StorageError> {
        let mut buffer = [0u8; BUFFER_LEN];
        let mut cache = NoCache::new();
        let mut items = map::fetch_all_items::<StoredAddr, _, _>(
            &mut self.flash,
            self.range.clone(),
            &mut cache,
            &mut buffer,
        )
        .await?;
        while let Some((addr, bond)) = items.next::<StoredBond>(&mut buffer).await? {
            if let Err(e) = stack.add_bond_information(bond.into_info(addr.0)) {
                println!("[bond] could not restore bond for {:?}: {:?}", addr.0, e);
            }
        }
        Ok(())
    }

    pub async fn save(&mut self, bond: &BondInformation) -> Result<(), StorageError> {
        let mut buffer = [0u8; BUFFER_LEN];
        map::store_item(
            &mut self.flash,
            self.range.clone(),
            &mut NoCache::new(),
            &mut buffer,
            &StoredAddr(bond.identity.bd_addr),
            &StoredBond::from(bond),
        )
        .await
    }
}

#[derive(Clone, PartialEq, Eq)]
struct StoredAddr(BdAddr);

impl Key for StoredAddr {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let buffer = buffer.get_mut(..6).ok_or(SerializationError::BufferTooSmall)?;
        buffer.copy_from_slice(self.0.raw());
        Ok(6)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        let raw = buffer.get(..6).ok_or(SerializationError::BufferTooSmall)?;
        Ok((StoredAddr(BdAddr::new(raw.try_into().unwrap())), 6))
    }
}

/// Layout: LTK (16) | security level (1) | IRK present (1) | IRK (16).
///
/// The IRK matters for phones: they connect from rotating private addresses,
/// and the IRK is how the lamp recognizes them again after a reboot.
struct StoredBond {
    ltk: LongTermKey,
    security_level: SecurityLevel,
    irk: Option<IdentityResolvingKey>,
}

const STORED_BOND_LEN: usize = 34;

impl StoredBond {
    fn into_info(self, bd_addr: BdAddr) -> BondInformation {
        let identity = Identity {
            bd_addr,
            irk: self.irk,
        };
        BondInformation::new(identity, self.ltk, self.security_level, true)
    }
}

impl From<&BondInformation> for StoredBond {
    fn from(bond: &BondInformation) -> Self {
        Self {
            ltk: bond.ltk,
            security_level: bond.security_level,
            irk: bond.identity.irk,
        }
    }
}

impl<'a> Value<'a> for StoredBond {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let buffer = buffer
            .get_mut(..STORED_BOND_LEN)
            .ok_or(SerializationError::BufferTooSmall)?;
        buffer[..16].copy_from_slice(&self.ltk.to_le_bytes());
        buffer[16] = match self.security_level {
            SecurityLevel::NoEncryption => 0,
            SecurityLevel::Encrypted => 1,
            SecurityLevel::EncryptedAuthenticated => 2,
        };
        buffer[17] = self.irk.is_some() as u8;
        buffer[18..].copy_from_slice(&self.irk.map_or([0; 16], |irk| irk.to_le_bytes()));
        Ok(STORED_BOND_LEN)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError> {
        let buffer = buffer
            .get(..STORED_BOND_LEN)
            .ok_or(SerializationError::BufferTooSmall)?;
        let security_level = match buffer[16] {
            0 => SecurityLevel::NoEncryption,
            1 => SecurityLevel::Encrypted,
            2 => SecurityLevel::EncryptedAuthenticated,
            _ => return Err(SerializationError::InvalidData),
        };
        let irk = match buffer[17] {
            0 => None,
            1 => Some(IdentityResolvingKey::from_le_bytes(buffer[18..].try_into().unwrap())),
            _ => return Err(SerializationError::InvalidData),
        };
        Ok(Self {
            ltk: LongTermKey::from_le_bytes(buffer[..16].try_into().unwrap()),
            security_level,
            irk,
        })
    }
}
