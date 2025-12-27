# Reverse engineering scripts

Scripts used to aid in reverse engineering binaries produced for Gaomon and Huion tablets

## load_gd32_structures.py

``This script was made for IDA Pro 9.2 and support for other versions is not guaranteed``

This script will create all sections that exist within the GD32F350 family of MCUs and populate their most important structures for reverse engineering purposes

### Usage

Just run the script in IDA Pro after the binary file has finished processing

## decrypt_fw_bin.py

This script can encrypt and decrypt firmwares that come from the Gaomon or Huion firmware CDNs

To get firmware files, download directly from the Gaomon or Huion CDNs at:

http://zyz.huion.cn/api/search.php

https://firmware.gaomon.cn/api/search.php

### Usage

To decrypt a firmware binary downloaded from the CDNs, run the script as follows ``python decrypt_fw_bin.py firmware_file``

To encrypt back a firmware binary (for whatever reason), run the script as follows ``python decrypt_fw_bin.py firmware_file -e``

The output will be the passed firmware name with a ``dec_`` or ``enc_`` prefix depending on the action done

### Notes

Each field in the CDN will look as follows:
```json
{
    "name":"H420X_HUION_T210_210611.bin",
    "url":"~SNIP~/f7a910019c49b0d9f9201cad7e0308e7.bin",
    "md5":"0c9c9e9a0bb5ab4b7c443ab6682174",
    "strJson":"{'FirmwareVersion':'210611','class':'HUION_T210','version':'1.1'}",
    "desc":"H420X"
}
```

The downloaded file will have a hash as a name (in the case above, ``f7a910019c49b0d9f9201cad7e0308e7.bin``), the actual file name is in the ``name`` field, just rename the downloaded file with that