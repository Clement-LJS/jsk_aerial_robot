#!/usr/bin/env python3
"""Explicit QP commands with current UInt64 session and acknowledgement."""
import argparse
import io
import threading
import time

import rospy
from std_msgs.msg import String, UInt64


def session_message(value):
    # Keep all 64 bits: nanosecond lock IDs must never pass through a float.
    if isinstance(value, bool) or not isinstance(value, int) or not 0 < value < 2**64:
        raise ValueError('Expected a nonzero uint64 lock session')
    message = UInt64(data=value)
    message.serialize(io.BytesIO())  # Fail locally before attempting publication.
    return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'align', 'start', 'pause', 'resume', 'cancel'])
    parser.add_argument('--namespace', default='/gimbalrotor/perching/roll_yaw')
    parser.add_argument('--timeout', type=float, default=3.0)
    args = parser.parse_args(rospy.myargv()[1:])
    if not 0 < args.timeout <= 30:
        parser.error('--timeout must be in (0, 30] seconds')
    rospy.init_node('roll_yaw_command', anonymous=True)
    prefix = args.namespace.rstrip('/')
    current = rospy.wait_for_message(prefix + '/session', UInt64, timeout=args.timeout)
    message = session_message(current.data)
    acknowledgement = threading.Event()
    reply = []
    sent = [False]
    expected = 'action={} session={} '.format(args.action, message.data)

    def receive(msg):
        if sent[0] and msg.data.startswith(expected):
            reply.append(msg.data)
            acknowledgement.set()

    subscriber = rospy.Subscriber(prefix + '/command_ack', String, receive, queue_size=1)
    publisher = rospy.Publisher(prefix + '/' + args.action, UInt64, queue_size=1)
    deadline = time.monotonic() + args.timeout
    while publisher.get_num_connections() == 0 and not rospy.is_shutdown():
        if time.monotonic() >= deadline:
            raise RuntimeError('No navigator subscriber; command was not sent')
        time.sleep(.01)
    sent[0] = True
    publisher.publish(message)
    if not acknowledgement.wait(args.timeout):
        raise RuntimeError('No matching acknowledgement; do not assume execution succeeded')
    print(reply[-1])
    subscriber.unregister()
    if ' accepted=true ' not in reply[-1]:
        raise RuntimeError('Navigator rejected the command')


if __name__ == '__main__':
    main()
