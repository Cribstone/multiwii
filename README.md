multiwii
========

Various distributions of multiwii that have come with various boards.

The board manufacturers are not particularly careful about revision
control, so I'm collecting them here as a public service.

Share and Enjoy!

MultiWii_Flip_NORMALQUADX
-------------------------

- Flip 1.5, standard distribution
- I think it's MW 2.3.  Uses MultiWii Config 2.3
- Tools/Board: Arduino Pro or Pro Mini (5V, 16 MHz) w/ ATmega328

MultiWii_Flip_PPMSUM
--------------------

- Flip 1.5, with PPMSUM
- Only diff is

::

    ./config.h
    333c333
    < //#define SERIAL_SUM_PPM PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
    >   #define SERIAL_SUM_PPM PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum

MultiWii_pocket_V4
------------------

- Now discontinued, HobbyKing PUMQ.  People were fiddling with this
  on rcgroups for quite a while.  Is there a later version of
  this anywhere on the Internet?
- Tools/Board: Arduino Leonardo
- discussion at: http://www.rcgroups.com/forums/showthread.php?t=1897199

BradWii
-------

- Thankfully, someone who tracks their code!  It's here:

  https://github.com/bradquick/bradwii
