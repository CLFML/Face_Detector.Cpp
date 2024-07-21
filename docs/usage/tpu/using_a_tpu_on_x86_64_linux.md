# TPU usage on Linux (x86_64)

Getting a Coral TPU might be a bit challenging to setup. The challenge is mainly in getting the right version of the libedgetpu library to work with our TensorFlow version. The rest is a lot easier as it is just one argument to compile with edgetpu support enabled.

## Getting libedgetpu to work!

Libedgetpu is version tied to the Tensorflow lite version which we are using! Which is TensorFlow Lite 2.16.1; Which is always a bit outdated as there are already way newer versions of Tensorflow around. Another annoyance is that the packages from the APT, RPM and AUR are either out-of-date or pulled from the package repository's. Meaning that to install the library we cannot make use of precompiled libraries and have to compile it ourselfs.

### Compiling libedgetpu

Compiling libedgetpu is annoying, mainly do to the bazel buildsystem (which is Google's own build-system). Their buildscripts require a quite recent version of bazel, which can't be found in your APT and RPM repository's (in rolling distro's such as ARCH this is not a problem!). So to install these packages you run:

**On Debian & Ubuntu***;

```bash
sudo apt-get remove bazel-bootstrap -y &&
sudo apt install apt-transport-https curl gnupg -y
curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg
sudo mv bazel-archive-keyring.gpg /usr/share/keyrings
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list && sudo apt update && sudo apt -y install bazel make libusb-1.0-0.dev devscripts
```
On ubuntu install this package too, as it is not included by default and is needed to package up the .deb file:

```bash
sudo apt-get install dh-make
```

**On Fedora**;
```bash
dnf copr enable vbatts/bazel
dnf install bazel make
```

**On Suse**;
```bash
zypper install bazel make
```

**Then clone the [libedgetpu repository](https://github.com/google-coral/libedgetpu)**

```bash
git clone https://github.com/google-coral/libedgetpu.git
```

**And compile**:

```bash
cd libedgetpu && make
```

This should take around 5-10 minutes on a moderately fast computer (i5 10th gen). 

Now after compilation you should add these libraries and include files to your linux filesystem.

---
**On Debian & Ubuntu based distro's**:

```bash
debuild -us -uc -tc -b -d && cd ..
```
After which you get two deb files; libedgetpu-std-... and libedgetpu-max-...

The std underclocks the edgetpu so that it does not get hot. The max runs the edgetpu at maximum speed.

Install the std version:
```bash
sudo dpkg -i libedgetpu1-std*.deb libedgetpu-dev*.deb
```

---

**For Fedora and other RPM-based distro's follow [this guide](https://www.clarenceho.net/2023/05/using-google-coral-tpu-on-opensuse.html)**