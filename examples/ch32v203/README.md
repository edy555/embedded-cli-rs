# CH32 example

This example shows how to build cli with ch32 series and its uart channel.
Tested on CH32V203.

# Preparation

```shell
rustup target add riscv32imac-unknown-none-elf
cargo install wlink
```

# Running

Run with:
```shell
cargo +nightly run --release 
```
