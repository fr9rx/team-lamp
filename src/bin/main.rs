#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

//! Team Lamp: a BLE-controlled status light.
//!
//! Exposes one GATT characteristic holding `[mode, r, g, b]`. Writing it changes
//! the onboard WS2812 LED; reading it returns the current state. The characteristic
//! requires an encrypted link, so a phone has to pair (Just Works) before using it.

use core::cell::Cell;

use ble::bonds::{BondStore, MAX_BONDS};
use bt_hci::controller::ExternalController;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_time::{Duration, Ticker, Timer};
use esp_bootloader_esp_idf::partitions::{self, DataPartitionSubType, PartitionType};
use esp_hal::Blocking;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Level;
use esp_hal::rmt::{Channel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator};
use esp_hal::rng::{Trng, TrngSource};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use trouble_host::prelude::*;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("PANIC: {}", info);
    // A lamp that reboots is better than one that silently freezes.
    esp_hal::system::software_reset()
}

extern crate alloc;

const DEVICE_NAME: &str = "TeamLamp";

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

const LAMP_SERVICE_UUID: u128 = 0xb47c507f_f216_421c_a88b_839a532d72aa;
const LIGHT_CHAR_UUID: u128 = 0xb47c5080_f216_421c_a88b_839a532d72aa;

const MODE_BREATHE: u8 = 1;
const MODE_RAINBOW: u8 = 2;
const MODE_COUNT: u8 = 3;

/// Soft blue breathing on boot, so you can see it's alive and advertising.
const DEFAULT_LIGHT: [u8; 4] = [MODE_BREATHE, 0, 90, 255];

/// Number of WS2812 LEDs on the data line (1 = the DevKit's onboard LED).
const NUM_LEDS: usize = 1;
/// Caps overall brightness; the onboard LED at full power is blinding.
const MAX_BRIGHTNESS: u16 = 96;
const FRAME_MS: u64 = 20;

// WS2812 bit timings at 32 MHz RMT clock (31.25 ns per tick).
// 0-bit: 406 ns high, 844 ns low. 1-bit: 812 ns high, 437 ns low.
const BIT_0: PulseCode = PulseCode::new(Level::High, 13, Level::Low, 27);
const BIT_1: PulseCode = PulseCode::new(Level::High, 26, Level::Low, 14);

type BleController = ExternalController<BleConnector<'static>, 1>;

/// Light state shared between the BLE handler and the LED task: `[mode, r, g, b]`.
static LIGHT: Mutex<Cell<[u8; 4]>> = Mutex::new(Cell::new(DEFAULT_LIGHT));

#[gatt_server(connections_max = CONNECTIONS_MAX)]
struct Server {
    lamp: LampService,
}

#[gatt_service(uuid = LAMP_SERVICE_UUID)]
struct LampService {
    /// `[mode, r, g, b]`, mode: 0 = solid, 1 = breathe, 2 = rainbow.
    #[characteristic(uuid = LIGHT_CHAR_UUID, read, write, value = DEFAULT_LIGHT, permissions(encrypted))]
    light: [u8; 4],
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32h2 -o esp32h2-mini-1 -o unstable-hal -o alloc -o embassy -o ble-trouble -o zed

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The following pins are used to bootstrap the chip. They are available
    // for use, but check the datasheet of the module for more information on them.
    // - GPIO8 (drives the onboard WS2812 on the ESP32-H2-DevKitM-1)
    // - GPIO9
    // - GPIO25
    // These GPIO pins are in use by some feature of the module and should not be used.
    let _ = peripherals.GPIO6;
    let _ = peripherals.GPIO7;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 69392);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(32)).unwrap();
    let led = rmt
        .channel0
        .configure_tx(
            &TxChannelConfig::default()
                .with_clk_divider(1)
                .with_idle_output_level(Level::Low)
                .with_idle_output(true)
                .with_carrier_modulation(false),
        )
        .unwrap()
        .with_pin(peripherals.GPIO8);
    spawner.spawn(led_task(led).unwrap());

    // Seeds the security manager's key generation. The source must stay alive while
    // the TRNG is in use; main never returns, so it does.
    let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1);
    let mut trng = Trng::try_new().unwrap();

    let address = random_static_address();
    let transport = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    enable_modem_security_clocks();
    let ble_controller: BleController = ExternalController::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(ble_controller, &mut resources)
        .set_random_address(address)
        .set_random_generator_seed(&mut trng);

    // Bonds live in the `nvs` partition, which nothing else uses in this firmware.
    let mut flash = FlashStorage::new(peripherals.FLASH);
    let mut table_buffer = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let nvs = partitions::read_partition_table(&mut flash, &mut table_buffer)
        .unwrap()
        .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
        .unwrap()
        .expect("partition table has no nvs partition");
    let mut bonds = BondStore::new(nvs.as_embedded_storage(&mut flash));
    bonds.load_into(&stack).await;
    println!("[bond] {} bonded device(s) loaded", stack.get_bond_information().len());

    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: DEVICE_NAME,
        appearance: &appearance::light_source::GENERIC_LIGHT_SOURCE,
    }))
    .unwrap();

    let a = address.addr.raw();
    println!(
        "[lamp] {} up, address {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        DEVICE_NAME, a[5], a[4], a[3], a[2], a[1], a[0]
    );

    select(
        run_host(&mut runner),
        serve(&mut peripheral, &stack, &server, &mut bonds),
    )
    .await;
    panic!("BLE stack stopped");
}

/// Workaround for esp-radio 0.18.0 on ESP32-H2: it never enables the modem security
/// engine clocks, so the BLE controller panics (`ASSERT r_ble_hw_encrypt_block:443`)
/// as soon as pairing encrypts the link. Same bits as the upstream fix, esp-hal#5852
/// (shipped in esp-radio 1.0.0-beta.1); delete this after upgrading.
/// Must run after `BleConnector::new`, which turns on the rest of the modem clocks.
fn enable_modem_security_clocks() {
    esp_hal::peripherals::MODEM_SYSCON::regs()
        .clk_conf()
        .modify(|_, w| {
            w.clk_etm_en().set_bit();
            w.clk_modem_sec_en().set_bit();
            w.clk_modem_sec_ecb_en().set_bit();
            w.clk_modem_sec_ccm_en().set_bit();
            w.clk_modem_sec_bah_en().set_bit();
            w.clk_modem_sec_apb_en().set_bit();
            w.clk_ble_timer_en().set_bit()
        });
}

/// Random static address derived from the chip's factory MAC, so it is stable
/// across reboots and unique per board.
fn random_static_address() -> Address {
    let mut addr = [0u8; 6];
    addr.copy_from_slice(esp_hal::efuse::base_mac_address().as_bytes());
    // The MAC is big-endian; BLE addresses go over the air little-endian.
    addr.reverse();
    // Random static addresses must have the two most significant bits set.
    addr[5] |= 0xC0;
    Address::random(addr)
}

async fn run_host(runner: &mut Runner<'_, BleController, DefaultPacketPool>) {
    if let Err(e) = runner.run().await {
        panic!("[ble] host stopped: {:?}", e);
    }
}

/// Advertise, serve one connection until it drops, repeat.
async fn serve(
    peripheral: &mut Peripheral<'_, BleController, DefaultPacketPool>,
    stack: &Stack<'_, BleController, DefaultPacketPool>,
    server: &Server<'_>,
    bonds: &mut BondStore<'_>,
) {
    let mut adv_data = [0; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids128(&[LAMP_SERVICE_UUID.to_le_bytes()]),
        ],
        &mut adv_data[..],
    )
    .unwrap();
    let mut scan_data = [0; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(DEVICE_NAME.as_bytes())],
        &mut scan_data[..],
    )
    .unwrap();

    loop {
        let advertiser = match peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_data[..adv_len],
                    scan_data: &scan_data[..scan_len],
                },
            )
            .await
        {
            Ok(advertiser) => advertiser,
            Err(e) => {
                println!("[ble] advertise failed: {:?}", e);
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };
        println!("[ble] advertising");

        let conn = match advertiser.accept().await {
            Ok(conn) => conn,
            Err(e) => {
                println!("[ble] accept failed: {:?}", e);
                continue;
            }
        };
        match conn.with_attribute_server(server) {
            Ok(conn) => {
                println!("[ble] connected");
                // Remember new phones while there's room; once full they can still pair,
                // they just have to pair again on every connection.
                let bondable = stack.get_bond_information().len() < MAX_BONDS;
                if let Err(e) = conn.raw().set_bondable(bondable) {
                    println!("[ble] set_bondable failed: {:?}", e);
                }
                handle_connection(&conn, stack, server, bonds).await;
            }
            Err(e) => println!("[ble] gatt setup failed: {:?}", e),
        }
    }
}

async fn handle_connection(
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    stack: &Stack<'_, BleController, DefaultPacketPool>,
    server: &Server<'_>,
    bonds: &mut BondStore<'_>,
) {
    let light = &server.lamp.light;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                println!("[ble] disconnected: {:?}", reason);
                return;
            }
            GattConnectionEvent::PairingComplete {
                security_level,
                bond,
            } => {
                println!(
                    "[ble] paired: {:?}, bonded: {}",
                    security_level,
                    bond.is_some()
                );
                if let Some(bond) = bond {
                    match bonds.save(&bond).await {
                        Ok(()) => println!("[bond] saved {:?}", bond.identity.bd_addr),
                        Err(e) => println!("[bond] save failed: {:?}", e),
                    }
                }
            }
            GattConnectionEvent::PairingFailed(e) => println!("[ble] pairing failed: {:?}", e),
            GattConnectionEvent::RequestConnectionParams(request) => {
                if let Err(e) = request.accept(None, stack).await {
                    println!("[ble] conn params update failed: {:?}", e);
                }
            }
            GattConnectionEvent::Gatt {
                event: GattEvent::Write(write),
            } if write.handle() == light.handle => {
                // `[u8; N]` parsing zero-pads short writes, so check the length ourselves.
                let reply = match *write.data() {
                    [mode, r, g, b] if mode < MODE_COUNT => {
                        println!("[lamp] set mode={} rgb=({}, {}, {})", mode, r, g, b);
                        critical_section::with(|cs| LIGHT.borrow(cs).set([mode, r, g, b]));
                        write.accept()
                    }
                    [_, _, _, _] => write.reject(AttErrorCode::VALUE_NOT_ALLOWED),
                    _ => write.reject(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH),
                };
                send_reply(reply).await;
            }
            GattConnectionEvent::Gatt { event } => {
                if let GattEvent::NotAllowed(e) = &event {
                    println!("[gatt] rejected access to handle {} (link not encrypted)", e.handle());
                }
                send_reply(event.accept()).await;
            }
            _ => {}
        }
    }
}

async fn send_reply(reply: Result<Reply<'_, DefaultPacketPool>, Error>) {
    match reply {
        Ok(reply) => reply.send().await,
        Err(e) => println!("[gatt] reply failed: {:?}", e),
    }
}

#[embassy_executor::task]
async fn led_task(mut channel: Channel<'static, Blocking, Tx>) {
    let mut ticker = Ticker::every(Duration::from_millis(FRAME_MS));
    let mut frame: u32 = 0;
    loop {
        let state = critical_section::with(|cs| LIGHT.borrow(cs).get());
        let mut pulses = [PulseCode::end_marker(); NUM_LEDS * 24 + 1];
        for led in 0..NUM_LEDS {
            let (r, g, b) = render(state, frame, led);
            encode_grb(&mut pulses[led * 24..][..24], r, g, b);
        }
        channel = match channel.transmit(&pulses) {
            Ok(tx) => match tx.wait() {
                Ok(channel) | Err((_, channel)) => channel,
            },
            Err((_, channel)) => channel,
        };
        frame = frame.wrapping_add(1);
        ticker.next().await;
    }
}

fn render([mode, r, g, b]: [u8; 4], frame: u32, led: usize) -> (u8, u8, u8) {
    let (r, g, b) = match mode {
        MODE_BREATHE => {
            // Triangle wave over 128 frames (~2.5 s), never fully dark.
            let phase = (frame % 128) as u16;
            let tri = if phase < 64 { phase * 4 } else { (127 - phase) * 4 };
            let level = 24 + tri * 231 / 252;
            (scale(r, level), scale(g, level), scale(b, level))
        }
        MODE_RAINBOW => {
            let offset = (led * 256 / NUM_LEDS) as u8;
            wheel((frame as u8).wrapping_mul(2).wrapping_add(offset))
        }
        _ => (r, g, b), // solid
    };
    (
        scale(r, MAX_BRIGHTNESS),
        scale(g, MAX_BRIGHTNESS),
        scale(b, MAX_BRIGHTNESS),
    )
}

fn scale(c: u8, level: u16) -> u8 {
    (u16::from(c) * level / 255) as u8
}

/// Maps 0..=255 around the color wheel: red -> green -> blue -> red.
fn wheel(pos: u8) -> (u8, u8, u8) {
    match pos {
        0..=84 => (255 - pos * 3, pos * 3, 0),
        85..=169 => {
            let p = pos - 85;
            (0, 255 - p * 3, p * 3)
        }
        _ => {
            let p = pos - 170;
            (p * 3, 0, 255 - p * 3)
        }
    }
}

/// WS2812 expects 24 bits per LED in GRB order, most significant bit first.
fn encode_grb(out: &mut [PulseCode], r: u8, g: u8, b: u8) {
    let bits = u32::from(g) << 16 | u32::from(r) << 8 | u32::from(b);
    for (i, code) in out.iter_mut().enumerate() {
        *code = if bits & (1 << (23 - i)) != 0 { BIT_1 } else { BIT_0 };
    }
}
