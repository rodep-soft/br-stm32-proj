{
  description = "STM32F767ZI Zenoh-Pico & Micro-CDR development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      supportedSystems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forEachSupportedSystem = f: nixpkgs.lib.genAttrs supportedSystems (system: f {
        pkgs = import nixpkgs { inherit system; };
      });
    in
    {
      devShells = forEachSupportedSystem ({ pkgs }: {
        default = pkgs.mkShell {
          packages = with pkgs; [
            gcc-arm-embedded
            cmake
            ninja
            gnumake
            python3
            python3Packages.pip
            stlink
            openocd
            zenoh
            git
          ];

          shellHook = ''
            echo "=========================================================="
            echo "  STM32F767ZI Zenoh-Pico Dev Shell (Nix Flake)"
            echo "=========================================================="
            echo "Compiler: $(arm-none-eabi-gcc --version | head -n 1)"
            echo "CMake:    $(cmake --version | head -n 1)"
            echo "Commands: make build, make flash, make test, make help"
            echo "=========================================================="
          '';
        };
      });
    };
}
