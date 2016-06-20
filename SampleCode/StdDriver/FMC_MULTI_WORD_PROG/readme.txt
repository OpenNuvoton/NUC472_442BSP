This sample program includes two images.
1. [fmc_multi_word_prog]
   The image runs on SRAM address 0x4000. It executes multi-word-program demo on
   the whole APROM and LDROM.
2. [loader]      
   The loader image which has "fmc_multi_word_prog.bin" inside. The only purpose of this
   image is to load "fmc_multi_word_prog.bin" to SRAM and execute it. 


