let
  pkgs = import <nixpkgs> { };
in

pkgs.mkShell {
  nativeBuildInputs = [
    pkgs.gcc15
    pkgs.gcc-arm-embedded
  ];
}