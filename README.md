# RF driver for the NXP FRDM KW41Z SoC

This RF driver uses the existing Rf drivers provided by NXP, along with a wrapper layer required by the ARM mbed Nanostack.
Unnecessary code has been left out, to keep it as small as possible.

## Status

Sending and receiving messages seems to work OK. Still some functionality that has not been tested.
A problem that seems to originate in the mbed nanostack makes it difficult to do more testing.