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
          # Only what the turret build cannot get from the host.
          #
          # DeepStream, CUDA, GStreamer, glib and libusb exist only as JetPack /
          # Ubuntu packages under /usr and /opt/nvidia, and they are built against
          # the host glibc and libstdc++. So the turret has to be compiled and
          # linked by the *host* cmake and gcc. The nixpkgs cmake deliberately does
          # not search /usr (CANdle-SDK then fails on "libusb library not found"),
          # and a nixpkgs gcc links against its own glibc, which does not mix with
          # the NVIDIA libraries.
          #
          # This shell therefore brings no cmake, compiler or pkg-config,
          # and never sets CXXFLAGS. It supplies Eigen, which the host
          # may lack and which is header-only, so there is no ABI to mismatch.
          buildInputs = [ pkgs.eigen ];

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
              export CMAKE_PREFIX_PATH="${pkgs.eigen}''${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
            '';
          };
        };
    };
}
