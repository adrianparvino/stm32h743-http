with import <nixpkgs> {};
pkgs.mkShell {
  buildInputs = [gcc rustup dfu-util gcc-arm-embedded gdb openocd];
}
