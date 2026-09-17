{
  description = "STM32F767ZI Zenoh-Pico & Micro-CDR development environment";

  nixConfig = {
    extra-substituters = [
      "https://cache.nixos.org"
      "https://nix-community.cachix.org"
      "https://roborescue-nix.cachix.org"
      "https://ros.cachix.org"
    ];
    extra-trusted-public-keys = [
      "cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
      "roborescue-nix.cachix.org-1:qy3rP4VwHob/xePMW77gUxZVvPMz8izs86rIdruro0U="
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };

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
