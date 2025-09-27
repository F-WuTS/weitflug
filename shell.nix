let
  pkgs = import <nixpkgs> { };
in

pkgs.mkShell {
  nativeBuildInputs = [
    pkgs.gcc-arm-embedded
  ];
}
