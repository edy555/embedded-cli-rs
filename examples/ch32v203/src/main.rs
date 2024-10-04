//! # Pico USB Serial Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the main thread.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use core::convert::Infallible;

use ch32_hal::usart::{self, Instance};
use embassy_executor::Spawner;
//use embassy_time::{Duration, Timer};
//use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::usart::{Uart, UartTx};
use hal::mode::Blocking;
use {ch32_hal as hal, panic_halt as _};

use panic_halt as _;
//use core::fmt::Write;

use embedded_cli::{
    cli::CliBuilder,
    cli::CliHandle,
    Command,
};
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

struct Writer<'a, T: Instance> {
    writer: UartTx<'a, T, Blocking>,
}

impl<T: Instance> embedded_io::ErrorType for Writer<'_, T> {
    type Error = Infallible;
}

impl<T: Instance> embedded_io::Write for Writer<'_, T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.writer.blocking_write(buf).unwrap();
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.writer.blocking_flush();
        Ok(())
    }
}

fn on_led<T: Instance>(
    cli: &mut CliHandle<'_, Writer<T>, Infallible>,
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

fn on_adc<T: Instance>(
    cli: &mut CliHandle<'_, Writer<T>, Infallible>,
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

fn on_status<T: Instance>(
    cli: &mut CliHandle<'_, Writer<T>, Infallible>,
    state: &mut AppState,
) -> Result<(), Infallible> {
    state.num_commands += 1;
    uwriteln!(cli.writer(), "{}{}", "Received: ", state.num_commands)?;
    Ok(())
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    try_run();

    // if run failed, stop execution
    panic!()
}

fn try_run() -> Option<()> {
    let p = hal::init(Default::default());
    hal::embassy::init();

    let cfg = usart::Config::default();
    let uart = Uart::new_blocking(p.USART1, p.PB7, p.PB6, cfg).unwrap();
    let (tx, mut rx) = uart.split();

    // create static buffers for use in cli (so we're not using stack memory)
    // History buffer is 1 byte longer so max command fits in it (it requires extra byte at end)
    // SAFETY: buffers are passed to cli and are used by cli only
    let (command_buffer, history_buffer) = unsafe {
        static mut COMMAND_BUFFER: [u8; 40] = [0; 40];
        static mut HISTORY_BUFFER: [u8; 41] = [0; 41];
        (COMMAND_BUFFER.as_mut(), HISTORY_BUFFER.as_mut())
    };
    let w = Writer { writer: tx };
    let mut cli = CliBuilder::default()
        .writer(w)
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
        let mut buf = [0u8; 1];
        match rx.blocking_read(&mut buf) {
            Err(_e) => {
                // Do nothing
            }
            Ok(()) => {
                // Convert to upper case
                buf.iter().for_each(|b| {
                    let byte = *b;
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
                });
            }
        }
    }
}

// End of file
