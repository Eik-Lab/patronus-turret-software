{
  description = "Patronus turret software — dev shell";

  nixConfig.sandbox = "relaxed";

  inputs.nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";

  outputs =
    { nixpkgs, ... }:
    let
      forAllSystems = nixpkgs.lib.genAttrs [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
      ];
      toolsFor =
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          formatter = pkgs.treefmt.withConfig {
            settings = builtins.fromTOML (builtins.readFile ./treefmt.toml);
            runtimeInputs = [
              pkgs.nixfmt
              pkgs.clang-tools
              pkgs.cmake-format
            ];
          };
        in
        {
          inherit formatter;
          devShell = pkgs.callPackage ./nix/package.nix { inherit formatter; };
        };
    in
    {
      formatter = forAllSystems (system: (toolsFor system).formatter);
      devShells = forAllSystems (system: {
        default = (toolsFor system).devShell;
      });
    };
}
