{ pkgs, formatter }:

# CMake owns the application build and dependencies; use the host compiler and NVIDIA SDKs.
pkgs.mkShellNoCC {
  __noChroot = true;

  buildInputs = [ pkgs.libusb1 ];

  packages =
    (with pkgs; [
      cmake
      gnumake
      pkg-config
      git
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
    ]
    ++ [ formatter ];
}
