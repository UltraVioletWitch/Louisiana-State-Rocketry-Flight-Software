{ pkgs ? import <nixpkgs> {} }:
let
in
  pkgs.mkShell {
    buildInputs = with pkgs; [
      platformio
      # optional: needed as a programmer i.e. for esp32
      avrdude
      teensy-loader-cli
    ];
}
