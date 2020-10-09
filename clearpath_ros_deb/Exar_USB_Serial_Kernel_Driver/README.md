# Serial Driver Setup

`$ make`
`$ sudo cp xr\_usb\_serial\_common.ko /lib/modules/$(uname -r)/kernel/usb/serial`

add `xr_usb_serial_common` to `/etc/modules`

add `blacklist cdc-acm` to `/etc/modprobe.d/blacklist.conf`

`$ sudo update-initramfs -u`

restart
