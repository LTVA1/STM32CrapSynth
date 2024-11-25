# STM32CrapSynth

A software/hardware synth based on STM32F303RE, 4xAD9833 wave generators, one MM5437 noise generator and LTC1063 lowpass filter. Currently I seem to have burned the filter so instead there's 3rd PCM channel using the timer that was planned for filter clocking.

# Folders

`board` - KiCAD project of the board that mounts on top of custom teacher's board like Arduino shield.
`pc_program` - small CLI program using serial library to send data to the MCU.
`STM32CrapSynthFirmware` - MCU firmware written only with CMSIS register defs and simple functions. Zero HAL, LL, SPL libraries usage.

# Other programs

[STM32CrapSynth in Furnace](https://github.com/LTVA1/furnace/tree/stm32crapsynth) - A fork of Furnace chiptune tracker with added emulation of STM32CrapSynth. Also has an ability to export data (samples and stream of commands/wavetable data) into the file which is then used by `pc_program`.
