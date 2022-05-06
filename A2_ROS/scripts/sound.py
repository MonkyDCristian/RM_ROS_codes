#! /usr/bin/env python3

import rospy
from sound_play.libsoundplay import SoundClient

class Sound(object):

  def __init__( self ):
    rospy.init_node('sound')
    self.sound_handler = SoundClient( blocking = True )
    self.running = True
  
  def run( self ):
    
    while self.running:
      text = input("What do you want to say?: ")
      
      if text != "out":
        self.sound_handler.say(text)
      
      else:
        self.running = False


if __name__ == '__main__':
  
  sound = Sound()
  sound.run()