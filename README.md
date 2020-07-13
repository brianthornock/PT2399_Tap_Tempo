# PT2300_Tap_Tempo

This project uses an ATTiny85 microcontroller, an encoder, an opamp, transistor, switch and some passives to create full-featured PT2399 tap tempo solution. Features include:

 - Relative time adjustments: adjusting the delay time with the encoder doesn't cause delay time to skip around dramatically like some other solutions
 - No need for a digiPOT! No need to worry about the tolerances or serial communications.
 - Programmable division switch: Press the encoder switch and you can have it do any division you want, just change the code. Default is a 2x multiplier.
 - Blinking LED indicator: shows you the time you tapped in. If using divisions, it shows the actual delay time (delayTime/multiplier)
 
 For more detail on the project, see the webpage at: 
