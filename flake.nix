{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "COMP4034-ROS-Noetic";
          shellHook = ''
            export TURTLEBOT3_MODEL=waffle
            export QT_PLUGIN_PATH=""
            export QT_QPA_PLATFORM=xcb
            if [ -f ./catkin_ws/devel/setup.bash ]; then
                source ./catkin_ws/devel/setup.bash
            fi
            exec zsh
          '';
          packages = [
            pkgs.glibcLocales
            pkgs.python311Packages.numpy # Add numpy
            pkgs.opencv # Add OpenCV
            # ... other non-ROS packages
            (with pkgs.rosPackages.noetic; buildEnv {
              paths = [
                rosbash
                turtlebot3-description
                turtlebot3-teleop
                turtlebot3-gazebo
                turtlebot3-bringup
                gazebo-plugins
                xacro
                catkin
                rviz
                vision-opencv
                image-proc
                # ... other ROS packages
              ];
            })
          ];
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
