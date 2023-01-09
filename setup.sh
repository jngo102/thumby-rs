#!/bin/bash

rustup default stable
rustup update
cargo update
cargo clean
rustup target add thumbv6-none-eabi
cargo install elf2uf2-rs --force
cargo install flip-link --force
