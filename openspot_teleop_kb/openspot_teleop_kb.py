import sys

import argparse
import geometry_msgs.msg
import rcl_interfaces.srv
import rclpy
from transforms3d.euler import euler2quat

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
This node takes keypresses from the keyboard and publishes them
as openspot cmd messages. It works best with a US keyboard layout.
---------------------------
State:
    0 - 3

Gait:
    hold down the shift key: 0 - 4

(Locomotion):
    Moving:
        w
     a     d
        s

    Turn:
        j/l: turn left/right

    Anything else : stop

    Velocity:
        =/- : increase/decrease only linear speed by 10%
        ./, : increase/decrease only angular speed by 10%

(Balance Stand)
    Pose:
        U/O: roll left up/down
        I/K: pitch head down/up
        J/L: yaw turn left/right
    Height:
        T/Y: height up/down



CTRL-C to quit
"""

state_bindings = {
    '0': rclpy.Parameter('state', rclpy.Parameter.Type.INTEGER, 0).to_parameter_msg(),
    '1': rclpy.Parameter('state', rclpy.Parameter.Type.INTEGER, 1).to_parameter_msg(),
    '2': rclpy.Parameter('state', rclpy.Parameter.Type.INTEGER, 2).to_parameter_msg(),
    '3': rclpy.Parameter('state', rclpy.Parameter.Type.INTEGER, 3).to_parameter_msg()
}

gait_bindings = {
    ')': rclpy.Parameter('gait', rclpy.Parameter.Type.INTEGER, 0).to_parameter_msg(),
    '!': rclpy.Parameter('gait', rclpy.Parameter.Type.INTEGER, 1).to_parameter_msg(),
    '@': rclpy.Parameter('gait', rclpy.Parameter.Type.INTEGER, 2).to_parameter_msg(),
    '#': rclpy.Parameter('gait', rclpy.Parameter.Type.INTEGER, 3).to_parameter_msg()
}

move_bindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1)
}

speed_bindings = {
    '=': (1.1, 1),
    '-': (.9, 1),
    '.': (1, 1.1),
    ',': (1, .9),
}

pose_bindings = {
    'U': (1, 0, 0, 0),
    'O': (-1, 0, 0, 0),
    'I': (0, 1, 0, 0),
    'K': (0, -1, 0, 0),
    'J': (0, 0, 1, 0),
    'L': (0, 0, -1, 0),
    'T': (0, 0, 0, 1),
    'Y': (0, 0, 0, -1),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

def main():
    parser = argparse.ArgumentParser(description='openspot teleoperation by keyboard')
    parser.add_argument('--ns', help='namespace')
    parser.add_argument('--drive', default='/openspot_drive', help='drive node name')
    args, _ = parser.parse_known_args()

    settings = saveTerminalSettings()

    rclpy.init(args=sys.argv)

    node = rclpy.create_node('openspot_teleop_kb', namespace=args.ns)
    vel_pub = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
    pose_pub = node.create_publisher(geometry_msgs.msg.Pose, 'cmd_pose', 10)
    paramcli = node.create_client(rcl_interfaces.srv.SetParameters, '{}/set_parameters'.format(args.drive))
    while not paramcli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    paramreq = rcl_interfaces.srv.SetParameters.Request()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    angle = 0.087
    quat = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z
    h = 0.0


    try:
        print(msg)
        print(vels(speed, turn))
        while rclpy.ok():
            key = getKey(settings)
            if key in state_bindings:
                paramreq.parameters = [state_bindings[key]]
                paramcli.call_async(paramreq)
            elif key in gait_bindings:
                paramreq.parameters = [gait_bindings[key]]
                paramcli.call_async(paramreq)
            elif key in move_bindings:
                x = move_bindings[key][0]
                y = move_bindings[key][1]
                z = move_bindings[key][2]
                th = move_bindings[key][3]
            elif key in speed_bindings:
                speed = speed * speed_bindings[key][0]
                turn = turn * speed_bindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key in pose_bindings:
                quat = euler2quat(
                    pose_bindings[key][0] * angle,
                    pose_bindings[key][1] * angle,
                    pose_bindings[key][2] * angle
                    )
                h = pose_bindings[key][3] * 0.05
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                quat = (1.0, 0.0, 0.0, 0.0)
                h = 0.0
                if (key == '\x03'):
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            vel_pub.publish(twist)

            pose = geometry_msgs.msg.Pose()
            pose.orientation.w = quat[0]
            pose.orientation.x = quat[1]
            pose.orientation.y = quat[2]
            pose.orientation.z = quat[3]
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = h
            pose_pub.publish(pose)

    except Exception as e:
        node.get_logger().warn(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        vel_pub.publish(twist)

        pose = geometry_msgs.msg.Pose()
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose_pub.publish(pose)

        node.destroy_node()
        rclpy.shutdown()

        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
