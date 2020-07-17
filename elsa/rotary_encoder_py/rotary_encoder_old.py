#!/usr/bin/env python

import pigpio

class decoder:

   """Class to decode mechanical rotary encoder pulses."""

   def __init__(self, gpioA, gpioB, callback):

      """
      Instantiate the class with the gpios connected to
      rotary encoder contacts A and B.  The common contact
      should be connected to ground.  The callback is
      called when the rotary encoder is turned.  It takes
      one parameter which is +1 for clockwise and -1 for
      counterclockwise.

      EXAMPLE

      import time
      import pigpio

      import rotary_encoder

      pos = 0

      def callback(way):

         global pos

         pos += way

         print("pos={}".format(pos))

      pigpio.start()

      decoder = rotary_encoder.decoder(7, 8, callback)

      time.sleep(300)

      decoder.cancel()

      pigpio.stop()

      """

      self.gpioA = gpioA
      self.gpioB = gpioB
      self.callback = callback

      self.levA = 0
      self.levB = 0

      self.lastGpio = None

      pigpio.set_mode(gpioA, pigpio.INPUT)
      pigpio.set_mode(gpioB, pigpio.INPUT)

      pigpio.set_pull_up_down(gpioA, pigpio.PUD_UP)
      pigpio.set_pull_up_down(gpioB, pigpio.PUD_UP)

      self.cbA = pigpio.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
      self.cbB = pigpio.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

   def _pulse(self, gpio, level, tick):

      """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
      """

      if gpio == self.gpioA:
         self.levA = level
      else:
         self.levB = level;

      if gpio != self.lastGpio: # debounce
         self.lastGpio = gpio

         if   gpio == self.gpioA and level == 1:
            if self.levB == 1:
               self.callback(1)
         elif gpio == self.gpioB and level == 1:
            if self.levA == 1:
               self.callback(-1)

   def cancel(self):

      """
      Cancel the rotary encoder decoder.
      """

      self.cbA.cancel()
      self.cbB.cancel()

if __name__ == "__main__":

   import time
   import pigpio

   import rotary_encoder

   pos = 0

   def callback(way):

      global pos

      pos += way

      print("pos={}".format(pos))

   pigpio.start()

   decoder = rotary_encoder.decoder(7, 8, callback)

   time.sleep(300)

   decoder.cancel()

   pigpio.stop()

