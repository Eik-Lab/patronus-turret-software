{
  description = "taken from: Kalman filter C++ ioe";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";

    flake-parts.url = "github:hercules-ci/flake-parts";
    flake-parts.inputs.nixpkgs-lib.follows = "nixpkgs";

    treefmt-nix.url = "github:numtide/treefmt-nix";
    treefmt-nix.inputs.nixpkgs.follows = "nixpkgs";

    pre-commit-hooks-nix.url = "github:cachix/pre-commit-hooks.nix";
    pre-commit-hooks-nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs =
    { self, ... }@inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];

      imports = [
        inputs."treefmt-nix".flakeModule
      ];

      perSystem =
        {
          config,
          pkgs,
          system,
          ...
        }:
        let
          # Build tools common to all phases
          nativeBuildInputs = with pkgs; [
            cmake
            ninja
            pkg-config
          ];

          # Runtime/library dependencies
          buildInputs = with pkgs; [
            eigen
            gtest
            gbenchmark
            opencv
          ];

          # Development-only tools
          devInputs =
            (with pkgs; [
              clang-tools
              cmake-language-server
              lldb
              bear
              tracy
            ])
            ++ pkgs.lib.optionals (!pkgs.stdenv.isDarwin) [
              pkgs.gdb
              pkgs.valgrind
              pkgs.linuxPackages.perf
              pkgs.samply
            ];

          kalman-cpp = pkgs.stdenv.mkDerivation {
            pname = "kalman-cpp";
            version = "0.1.0";

            src = self;

            inherit nativeBuildInputs;
            inherit buildInputs;

            cmakeFlags = [
              "-DCMAKE_BUILD_TYPE=Release"
              "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
            ];

            CXXFLAGS = [ "-std=c++17" ];
          };
        in
        {
          packages.default = kalman-cpp;

          checks = {
            pre-commit = inputs."pre-commit-hooks-nix".lib.${system}.run {
              src = self;
              hooks = {
                clang-format.enable = true;
                clang-tidy.enable = true;
                cmake-format.enable = true;
                nixfmt.enable = true;
                deadnix.enable = true;
                typos.enable = true;
              };
            };
          };

          treefmt = {
            projectRootFile = "flake.nix";
            programs = {
              nixfmt.enable = true;
              clang-format.enable = true;
              cmake-format.enable = true;
            };
          };

          devShells.default = pkgs.mkShell {
            inherit nativeBuildInputs;
            buildInputs = buildInputs ++ devInputs;

            shellHook = ''
              ${config.checks.pre-commit.shellHook}
              export CMAKE_EXPORT_COMPILE_COMMANDS=ON
              # Resolve find_package(Tracy) to the pinned nixpkgs tracy.
              export CMAKE_PREFIX_PATH="${pkgs.tracy}:''${CMAKE_PREFIX_PATH:-}"
            '';

            CXXFLAGS = "-std=c++17 -Wall -Wextra -Wpedantic -Werror";

            hardeningDisable = [ "fortify" ];
          };
        };
    };
}
