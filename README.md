This project uses:

  -Raspberry Pi Pico
  
  -MPU6050 IMU (Inertial Measurement Unit)
  
  -128x32px OLED display connected via I2C

Everything is just ready to be flashed to the Pi Pico by just uploading everything to the microcontroller's root directory.
All the external libraryes are included here.

For the custom fonts I used https://github.com/peterhinch/micropython-font-to-py as should you in case you want to change the looks of it.

Display stuff is done with the help of https://github.com/FelixSchladt/Micropython-SSD1306-GFX to which I added some methods to enable the display of some nice intro animation.
