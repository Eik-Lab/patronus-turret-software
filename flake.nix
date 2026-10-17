{
  description = "Patronus turret software — dev shell";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";

    flake-parts.url = "github:hercules-ci/flake-parts";
    flake-parts.inputs.nixpkgs-lib.follows = "nixpkgs";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];

      perSystem =
        {
          config,
          pkgs,
          ...
        }:
        let
          buildInputs = [ ];

          # Development-only tools that do not take part in the build.
          devInputs =
            (with pkgs; [
              cmake-language-server
              bear
              clang-tools
              deadnix
              pre-commit
              typos
            ])
            ++ pkgs.lib.optionals (!pkgs.stdenv.hostPlatform.isDarwin) [
              pkgs.gdb
              pkgs.valgrind
            ];
        in
        {
          formatter = pkgs.treefmt.withConfig {
            settings = {
              tree-root-file = "flake.nix";
              excludes = [ "deps/**" ];
              formatter = {
                nixfmt = {
                  command = "nixfmt";
                  includes = [ "*.nix" ];
                };
                clang-format = {
                  command = "clang-format";
                  options = [ "-i" ];
                  includes = [
                    "*.c"
                    "*.cc"
                    "*.cpp"
                    "*.h"
                    "*.hh"
                    "*.hpp"
                    "*.cu"
                    "*.cuh"
                  ];
                };
                cmake-format = {
                  command = "cmake-format";
                  options = [ "-i" ];
                  includes = [
                    "*.cmake"
                    "CMakeLists.txt"
                  ];
                };
              };
            };
            runtimeInputs = [
              pkgs.nixfmt
              pkgs.clang-tools
              pkgs.cmake-format
            ];
          };

          devShells.default = pkgs.mkShellNoCC {
            packages = buildInputs ++ devInputs ++ [ config.formatter ];

            shellHook = ''
              # Pre-commit rejects even an explicit default hook path; preserve custom paths.
              if [ "$(git config --local --get core.hooksPath)" = ".git/hooks" ]; then
                git config --local --unset core.hooksPath
              fi
              pre-commit install
              export CMAKE_EXPORT_COMPILE_COMMANDS=ON
              # Let the host cmake resolve find_package(Eigen3) to the pinned eigen.
            '';
          };
        };
    };
}
