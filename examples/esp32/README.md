# ESP32 example

This example shows how to build cli with ESP series and its usb serial channel.
Tested on ESP32C6(RISC/V).

# Preparation

```shell
rustup target add riscv32imac-unknown-none-elf
cargo install espflash
```

```shell
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

# Running

Run with:
```shell
cargo run --features=esp32c6 --release --target=riscv32imac-unknown-none-elf 
```

Or run with alias defined in [config.toml](.cargo/config.toml).

```shell
cargo esp32c6
```

# Memory usage

```shell
cargo  size --features=esp32c6 --release --target=riscv32imac-unknown-none-elf 
```
