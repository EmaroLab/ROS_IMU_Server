#!/usr/bin/env python

import sys
import rospy
import signal
import socket
import select
import Queue
from sensor_msgs.msg import Imu
from imu_sensor import*



def signal_handler(signal, frame):
    """Signal handler of the data"""
    print('Signal Handler, you pressed Ctrl+C!')
    print('Server will be closed')
    sys.exit(0)


def main():
    imu_publisher = rospy.Publisher('imu_data', Imu, queue_size=10)
    rospy.init_node('wearami_socket', anonymous=True)
    r = rospy.Rate(20)  # period
    signal.signal(signal.SIGINT, signal_handler)

    print('Press Ctrl+C to exit of this server')
    # A server must perform the sequence socket(), bind(), listen(), accept()
    HOST = socket.gethostname()   # Get local machine name
    PORT = 8080                # Reserve a port for your service.

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setblocking(0)

    # Running an example several times with too small delay between executions, could lead to this error:
    # socket.error: [Errno 98] Address already in use
    # There is a socket flag to set, in order to prevent this, socket.SO_REUSEADDR:
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('', PORT))

    # Listen for incoming connections
    server.listen(5)

    # Sockets from which we expect to read
    inputs = [server]

    # Sockets to which we expect to write
    outputs = []

    # Outgoing message queues (socket:Queue)
    message_queues = {}
    imu_message = Imu()
    imu_sens = imu_sensor()
    while not rospy.is_shutdown():
        # Wait for at least one of the sockets to be ready for processing
        readable, writable, exceptional = select.select(inputs, outputs, inputs)
        # Handle inputs
        for s in readable:
            if s is server:
                # A "readable" server socket is ready to accept a connection
                connection, client_address = s.accept()
                print >>sys.stderr, 'new connection from', client_address
                connection.setblocking(0)
                inputs.append(connection)
                # Give the connection a queue for data we want to send
                message_queues[connection] = Queue.Queue()
            else:
                data = s.recv(1024)
                if data:
                    # A readable client socket has data
                    list_data = data.split("\n")
                    print list_data
                    imu_sens.receiver(data)
                    message_queues[s].put(data)
                    # Add output channel for response
                    if s not in outputs:
                        outputs.append(s)

        if imu_sens.new_flag:
            sys.stdout.write("\rSending ...")
            sys.stdout.flush()
            imu_message.header.stamp = imu_sens.time
            imu_message.linear_acceleration = imu_sens.acceleration
            imu_message.angular_velocity = imu_sens.velocity
            imu_publisher.publish(imu_message)
            imu_sens.new_flag = False
        else:
            sys.stdout.write("\rWaiting ...")
            sys.stdout.flush()
        #r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
