This sample program includes three images.
1. [fmc_ld_boot] - The image runs on LDROM offset 0 (0x100000).
2. [fmc_isp]     - The image run on APROM offset 96K.
3. [fmc_ap_main] - The loader image which has "fmc_ld_boot.bin" and "fmc_isp.bin" inside. 

Please load the fmc_ap_main.bin to APROM. It will automatically load "fmc_ld_boot.bin" 
and "fmc_isp.bin" to their executing location.

To modify "fmc_ld_boot.bin" and "fmc_isp.bin", you have to build images with the corresponding
project. And then rebuild [fmc_ap_main] project to include new images to "fmc_ap_main.bin".


