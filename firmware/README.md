# Linetime Firmware

## Setting Up ChibiOS
```bash
$ git submodule update --init
$ cd ChibiOS/ext
$ 7za x fatfs-0.10b-patched.7z
$ 7za x lwip-1.4.1_patched.7z
$ cd ..
$ cp ../chibios_lwip_bindings.patch .
$ git apply chibios_lwip_bindings.patch
$ cp ../lwip_ip_addr.patch .
$ patch -p0 < lwip_ip_addr.patch
$ cp ../chibios_mac_dsb.patch .
$ git apply chibios_mac_dsb.patch
```
