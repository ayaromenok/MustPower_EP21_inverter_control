# MustPower_EP20_inverter_control

Must Power EP 20 series control utility. It may also work with newest EP 21 series.

### Other names

 - Official name looks like `MUST EP20 - 1012 PRO` (for 1000W version. 600W looks like `MUST EP20 - 0612 PRO` ).
 - Other name can be `MUST EP20 - 1000 PRO` (for 1000W version. 600W looks like `MUST EP20 - 600 PRO` ).
 - Windows Application `Solar/Power Monitor` can display different names like `Ep 2000 Pro`, `PV 2000 PK` depenfing on the version off application. 
 - It also can apper under other brand names, like `SYNAPSE` in South Africa.

### Idntification

Inverter use serial converter CH340, so appear in `lsusb` output as follow:

```Bus 001 Device 006: ID 1a86:7523 QinHeng Electronics CH340 serial converter```

### RaspberryPi issues

- Error: device reports readiness to read but returned no data (device disconnected or multiple access on port?)
 
`sudo systemctl mask serial-getty@ttyUSB0.service` and/or `sudo nano /boot/firmware/cmdline.txt`. remove `console=serial0,115200`