# Set-up build environment on Linux

Linux is one of the easiest os'es to set-up as most packages and libraries can be found in the package repositories.

## Ubuntu and Debian

```bash
sudo apt-get update && sudo apt-get install build-essential cmake ninja-build libopencv-dev
```

If you have not installed VSCode yet, do not install using APT in ubuntu as it will install the sandboxed snap version.

**Which has many stupid issues do to the sandbox environment**

Use [this guide](https://code.visualstudio.com/docs/setup/linux) instead, which installs it using the APT repository from Microsoft themselves.


## Arch

```bash
sudo pacman -S opencv cmake gcc ninja
```

If you have not installed VSCode yet,

Install the `visual-studio-code-bin` package from AUR.


## Fedora

```bash
sudo dnf install opencv-devel gcc cmake ninja
```

If you have not installed VSCode yet, use [this guide](https://code.visualstudio.com/docs/setup/linux).