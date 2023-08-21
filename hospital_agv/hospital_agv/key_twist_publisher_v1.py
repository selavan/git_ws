from email.policy import default
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from pynput.keyboard import Key, Listener

arg_msg = """
enter velocity args in format <linear vel> <angular vel>
        """

def process_args_vel():
    speed = 0.35
    turn = 0.7
    try:
        if len(sys.argv) == 1:
            print(arg_msg)
            print("using default values")
            return speed, turn
        else:
            print("using entered velocity values")
            speed = float(sys.argv[1])
            turn = float(sys.argv[2])
            return speed, turn
    except Exception as e:
        print(e)
        print(arg_msg)
        print("using default values")
        return speed, turn

msg = """
This node takes keypresses from the keyboard 
and publishes them as Twist messages.

------------------------------------------------
NONE CONTINOUS MODE (default mode)
drive around with arrow keys:

 [up/left]     [up]     [up/right]
                |
  [left] ---------------- [right]
                |
[down/left]   [down]   [down/right]

stops when no arrow key is pressed

For Holonomic mode (strafing), PRESS ALT key
(press again to return to non holonomic mode).

-------------------------------------------------
CONTINOUS DRIVE MODE (press CAPLOCK to activate
or deactivate back to non-continous mode)
drive around with arrow keys only:

              [up]  
                |
  [left] ---------------- [right]
                |
              [down]  

press s to stop robot

For Holonomic mode (strafing), PRESS ALT key
(press again to return to non holonomic mode).

-------------------------------------------------
q/z : increase/decrease only linear speed by 10%
w/x : increase/decrease only angular speed by 10%

press r to reset speed to the default or input speeds

CTRL-C to quit
"""

class Teleop(Node):
    def __init__(self):
        super().__init__(node_name="pynput_teleop_twist_keyboard_node") # initialize the node name


        self.speedBindings = {
            'q': (1.1, 1),
            'Q': (1.1, 1),
            'z': (.9, 1),
            'Z': (.9, 1),
            'w': (1, 1.1),
            'W': (1, 1.1),
            'x': (1, .9),
            'X': (1, .9),
        }
        self.speed_ctrl_keys = ['q', 'z', 'w', 'x', 'Q', 'Z', 'W', 'X', 'r', 'R', 's', 'S']

        self.default_speed, self.default_turn = process_args_vel()

        self.speed = self.default_speed
        self.turn = self.default_turn
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0

        self.holonomic_mode = False
        self.continuos_mode = False
        self.can_print = True

        self.status = 0

        self.Mode = ["non_holonomic","non_continous"]


        self.send_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        # # the timer will automatically run the timer callback function
        # # for every timer_period
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.publish_twist)

        # ...or, in a non-blocking fashion:
        listener = Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        # listener.join()

        print(msg)
        print("mode: ", self.Mode)
            


    def publish_twist(self):
        self.twist = Twist()
        
        self.twist.linear.x = self.x * self.speed
        self.twist.linear.y = self.y * self.speed
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = self.th * self.turn
        self.send_cmd.publish(self.twist)

        self.print_speed()

    def print_speed(self):
        if self.can_print:
            if (self.status == 10):
                print(msg)
                print("mode: ", self.Mode)
            self.status = (self.status + 1) % 11

            print('currently:\tspeed=%s\tturn=%s' % (self.speed, self.turn))
            self.can_print=False


    def reset_speed(self):
        self.speed = self.default_speed
        self.turn = self.default_turn
        self.can_print=True

    def on_press(self, key):       
        if key == Key.up:
            if self.continuos_mode:
                self.x = 1
                self.th = 0
                self.y = 0
            else:
                if self.x == 0:
                    self.x = 1
                
        elif key == Key.down:
            if self.continuos_mode:
                self.x = -1
                self.th = 0
                self.y = 0
            else:
                if self.x == 0:
                    self.x = -1



        if key == Key.left:
            if self.continuos_mode:
                if not self.holonomic_mode:
                    self.th = 1
                else:
                    self.y = 1
                self.x = 0
            else:
                if not self.holonomic_mode:
                    if self.th == 0:
                        self.th = 1
                else:
                    if self.y == 0:
                        self.y = 1
                
        elif key == Key.right:
            if self.continuos_mode:
                if not self.holonomic_mode:
                    self.th = -1
                else:
                    self.y = -1
                self.x = 0
            else:
                if not self.holonomic_mode:
                    if self.th == 0:
                        self.th = -1
                else:
                    if self.y == 0:
                        self.y = -1



        if key == Key.alt:
            if self.holonomic_mode==True:
                self.holonomic_mode=False
                self.Mode[0] = "non_holonomic"
            else:
                self.holonomic_mode=True
                self.Mode[0] = "holonomic"
                self.th = 0
            
            print("mode: ", self.Mode)
            
        
        if key == Key.caps_lock:
            if self.continuos_mode==True:
                self.continuos_mode = False
                self.Mode[1] = "non_continuous"
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.th = 0.0
            else:
                self.continuos_mode = True
                self.Mode[1] = "continuous"
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.th = 0.0
            
            print("mode: ", self.Mode)


        
        if hasattr(key, 'char'):
            if key.char in self.speed_ctrl_keys:
                if key.char == 'R' or key.char == 'r':
                    self.reset_speed()
                    self.can_print=True
                elif key.char == 'S' or key.char == 's':
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                else:
                    self.speed = self.speed * self.speedBindings[key.char][0]
                    self.turn = self.turn * self.speedBindings[key.char][1]
                    self.can_print=True

            


                    
    def on_release(self, key):

        if key == Key.up:
            if self.continuos_mode:
                pass
            else:
                if self.x == 1:
                    self.x = 0
                
        elif key == Key.down:
            if self.continuos_mode:
                pass
            else:
                if self.x == -1:
                    self.x = 0



        if key == Key.left:
            if self.continuos_mode:
                pass
            else:
                if not self.holonomic_mode:
                    if self.th == 1:
                        self.th = 0
                else:
                    if self.y == 1:
                        self.y = 0
     
        elif key == Key.right:
            if self.continuos_mode:
                pass
            else:
                if not self.holonomic_mode:
                    if self.th == -1:
                        self.th = 0
                else:
                    if self.y == -1:
                        self.y = 0

        if key == Key.esc:
            # Stop listener
            return False
        






def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the publisher node
    teleop = Teleop()

    # spin the node so the call back function is called
    rclpy.spin(teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown() 



if __name__=='__main__':
    main()

'''
#!/usr/bin/env python
import rclpy
from rclpy.qos import qos_profile_default

from geometry_msgs.msg import Twist

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main(args=None):	
	if args is None:
		args = sys.argv

	rclpy.init(args)
	node = rclpy.create_node('teleop_twist_keyboard')
		
	pub = node.create_publisher(Twist, 'cmd_vel', 	qos_profile_default)

	speed = 0.5
	turn = 1.0
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
			pub.publish(twist)

	except:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
'''

