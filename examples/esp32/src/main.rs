//! This shows how to use cli via USB Serial/JTAG.
//! You need to connect via the Serial/JTAG interface to interact cli.
//!
//! //! The following wiring is assumed:
//! - LED => GPIO21

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s3

#![warn(rust_2018_idioms)]
#![no_std]
#![no_main]

use core::convert::Infallible;

use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    usb_serial_jtag::UsbSerialJtag, usb_serial_jtag::UsbSerialJtagTx,
    Blocking,
};
use embedded_cli::{
    cli::CliBuilder,
    cli::CliHandle,
    Command,
};
use embedded_hal_nb::serial::Read;
use panic_halt as _;
use ufmt::uwrite;
use ufmt::uwriteln;

#[derive(Debug, Command)]
enum BaseCommand<'a> {
    /// Control LEDs
    Led {
        /// LED id
        #[arg(long)]
        id: u8,

        #[command(subcommand)]
        command: LedCommand,
    },

    /// Control ADC
    Adc {
        /// ADC id
        #[arg(long)]
        id: u8,

        #[command(subcommand)]
        command: AdcCommand<'a>,
    },

    /// Show some status
    Status,
}

#[derive(Debug, Command)]
enum LedCommand {
    /// Get current LED value
    Get,

    /// Set LED value
    Set {
        /// LED brightness
        value: u8,
    },
}

#[derive(Debug, Command)]
enum AdcCommand<'a> {
    /// Read ADC value
    Read {
        /// Print extra info
        #[arg(short = 'V', long)]
        verbose: bool,

        /// Sample count (16 by default)
        #[arg(long)]
        samples: Option<u8>,

        #[arg(long)]
        sampler: &'a str,
    },
}

struct AppState {
    led_brightness: [u8; 4],
    num_commands: usize,
}

type Writer = UsbSerialJtagTx<'static, Blocking>;

fn on_led(
    cli: &mut CliHandle<'_, Writer, Infallible>,
    state: &mut AppState,
    id: u8,
    command: LedCommand,
) -> Result<(), Infallible> {
    state.num_commands += 1;

    if id as usize > state.led_brightness.len() {
        uwrite!(cli.writer(), "{}{}{}", "LED", id, " not found")?;
    } else {
        match command {
            LedCommand::Get => {
                uwrite!(
                    cli.writer(),
                    "{}{}{}{}",
                    "Current LED",
                    id,
                    " brightness: ",
                    state.led_brightness[id as usize]
                )?;
            }
            LedCommand::Set { value } => {
                state.led_brightness[id as usize] = value;
                uwrite!(
                    cli.writer(),
                    "{}{}{}{}",
                    "Setting LED",
                    id,
                    " brightness to ",
                    state.led_brightness[id as usize]
                )?;
            }
        }
    }

    Ok(())
}

fn on_adc(
    cli: &mut CliHandle<'_, Writer, Infallible>,
    state: &mut AppState,
    id: u8,
    command: AdcCommand<'_>,
) -> Result<(), Infallible> {
    state.num_commands += 1;

    match command {
        AdcCommand::Read {
            verbose,
            samples,
            sampler,
        } => {
            let samples = samples.unwrap_or(16);
            if verbose {
                cli.writer().write_str("Performing sampling with ")?;
                cli.writer().write_str(sampler)?;
                uwriteln!(
                    cli.writer(),
                    "{}{}{}",
                    "\nUsing ",
                    samples,
                    " samples"
                )?;
            }
            uwrite!(
                cli.writer(),
                "{}{}{}{}",
                "Current ADC",
                id,
                " readings: ",
                43
            )?;
        }
    }
    Ok(())
}

fn on_status(
    cli: &mut CliHandle<'_, Writer, Infallible>,
    state: &mut AppState,
) -> Result<(), Infallible> {
    state.num_commands += 1;
    uwriteln!(cli.writer(), "{}{}", "Received: ", state.num_commands)?;
    Ok(())
}

#[entry]
fn main() -> ! {
    try_run();

    // if run failed, stop execution
    panic!()
}

fn try_run() -> Option<()> {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let delay = Delay::new(&clocks);

    // Set GPIO0 as an output, and set its state high initially.
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = Output::new(io.pins.gpio21, Level::High);
    
    //let usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE, None);
    let usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
    let (writer, mut rx) = usb_serial.split();

    led.set_low();

    // create static buffers for use in cli (so we're not using stack memory)
    // History buffer is 1 byte longer so max command fits in it (it requires extra byte at end)
    // SAFETY: buffers are passed to cli and are used by cli only
    let (command_buffer, history_buffer) = unsafe {
        static mut COMMAND_BUFFER: [u8; 40] = [0; 40];
        static mut HISTORY_BUFFER: [u8; 41] = [0; 41];
        (COMMAND_BUFFER.as_mut(), HISTORY_BUFFER.as_mut())
    };
    let mut cli = CliBuilder::default()
        .writer(writer)
        .command_buffer(command_buffer)
        .history_buffer(history_buffer)
        .build()
        .ok()?;

    // Create global state, that will be used for entire application
    let mut state = AppState {
        led_brightness: [0; 4],
        num_commands: 0,
    };

    let _ = cli.write(|writer| {
        // storing big text in progmem
        // for small text it's usually better to use normal &str literals
        uwrite!(
            writer,
            "{}",
            "Cli is running.
Type \"help\" for a list of commands.
Use backspace and tab to remove chars and autocomplete.
Use up and down for history navigation.
Use left and right to move inside input."
        )?;
        Ok(())
    });

    loop {
        delay.delay_millis(10);
        led.toggle();

        let byte = nb::block!(rx.read()).unwrap();        
        // Process incoming byte
        // Command type is specified for autocompletion and help
        // Processor accepts closure where we can process parsed command
        // we can use different command and processor with each call
        let _ = cli.process_byte::<BaseCommand<'_>, _>(
            byte,
            &mut BaseCommand::processor(|cli, command| match command {
                BaseCommand::Led { id, command } => on_led(cli, &mut state, id, command),
                BaseCommand::Adc { id, command } => on_adc(cli, &mut state, id, command),
                BaseCommand::Status => on_status(cli, &mut state),
            }),
        );
    }
}
