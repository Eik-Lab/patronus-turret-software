{
  description = "Patronus turret software — dev shell";

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
          # This shell therefore brings no cmake, compiler, pkg-config or
          # clang-tidy, and never sets CXXFLAGS. It supplies Eigen, which the host
          # may lack and which is header-only, so there is no ABI to mismatch.
          buildInputs = [ pkgs.eigen ];

          # Development-only tools that do not take part in the build.
          devInputs =
            (with pkgs; [
              cmake-language-server
              bear
            ])
            ++ pkgs.lib.optionals (!pkgs.stdenv.hostPlatform.isDarwin) [
              pkgs.gdb
              pkgs.valgrind
            ];
        in
        {
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

          devShells.default = pkgs.mkShellNoCC {
            packages = buildInputs ++ devInputs;

            shellHook = ''
              ${config.checks.pre-commit.shellHook}
              export CMAKE_EXPORT_COMPILE_COMMANDS=ON
              # Let the host cmake resolve find_package(Eigen3) to the pinned eigen.
              export CMAKE_PREFIX_PATH="${pkgs.eigen}''${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
            '';
          };
        };
    };
}
