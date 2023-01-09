# thumby-rs
This is a Rust crate that may be used to interface with the [TinyCircuits Thumby](https://thumby.us/) device.

## Downloading an installing the binary
1.  Navigate to the [releases page](https://github.com/jngo102/thumby-rs/releases).
2.  Download the `.uf2` file from the `Assets` dropdown.
3.  Make sure your Thumby is in BOOTSEL mode
    - Power on your Thumby by toggling the switch on top to the right.
    - While holding the `down` button on the D-pad, switch your Thumby off by toggling the switch to the left.
    - While continuing to hold the `down` button, power on your Thumby again.
    - Your Thumby should now be mounted on your system.
4.  Move the downloaded `.uf2` file to your mounted Thumby. It should automatically unmount itself.

## Instructions for downloading and running examples
1.  Download [Rust](https://www.rust-lang.org/tools/install).
2.  Run the `setup.sh` script. This requires Bash CLI, which on Windows may be obtained through [Git Bash](https://git-scm.com/downloads).
3.  Make sure your Thumby is in BOOTSEL mode
    - Power on your Thumby by toggling the switch on top to the right.
    - While holding the `down` button on the D-pad, switch your Thumby off by toggling the switch to the left.
    - While continuing to hold the `down` button, power on your Thumby again.
    - Your Thumby should now be mounted on your system.
4.  *Optional:* Install [Visual Studio Code](https://code.visualstudio.com/).
5.  Clone this repository:  
    <clipboard-copy class="btn btn-sm BtnGroup-item">
      `git clone https://github.com/jngo102/thumby-rs`
    </clipboard-copy>
6.  If you installed Visual Studio Code, you can quickly deploy to your mounted Thumby by opening the command palette `(Ctrl/Cmd+Shift+P)`, executing the `Deploy to Thumby` task, then answering the prompts.
7.  If you did *not* install Visual Studio Code, you may open a terminal, navigate to the path of the cloned repository: `cd /path/to/thumby-rs`, and run the command `cargo build --example {exampleName}`, where `{exampleName}` should be replaced with the name of an example that may be found in [the examples folder](https://github.com/jngo102/thumby-rs/blob/bw/examples).
8.  After the build is complete, navigate to the produced binary: `cd ./target/thumbv6m-none-eabi/debug/examples/{exampleName}`
9.  Run the command `elf2uf2-rs {exampleName}` to turn the ELF binary into a UF2 binary that may be used by the Thumby.
10. Copy the resulting UF2 file into the location of your mounted Thumby. It should automatically unmount itself.

## Using this crate in a project
1.  Clone the [rp2040-hal template project](https://github.com/rp-rs/rp2040-project-template) to your computer.
2.  Open the cloned folder in an IDE.
3.  In the `Cargo.toml` file, under `[dependencies]`, add `thumby = "^0"`.
4.  You may now start importing thumby-rs modules to use in your game.
5.  If you have Visual Studio Code, it is recommended to have the [rust-analyzer extension](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer) installed.