The Segmented LCD driver and examples are tested with 16 segment VIM-878 display
available with CYTVII-C-E-4M-176-CPU board.

BasicSegment Example:
1. The example will show how to set, read or clear any pixel on the segment display.
2. It uses api's to set char, integer or string on the display.
3. The example shows and set custom pixel mapping.

MovingSegment:
1. The example enables all the segments one by one in contineous loop.

Reference:
Do not use.

Points to remember while testing Segmented LCD:
1. LCD viewing angle is vertical, on other angles disable pixels may visible.
2. For better performance use CM4 core for testing and remove R1002 to R1008 resistors.
3. Backlight is not available so contrast ratio can be increased for better visibility.
4. The compatible examples are "BasicSegment" and "MovingSegment", do not use example from "Reference" folder.
