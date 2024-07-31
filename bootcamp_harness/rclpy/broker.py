import zmq

# https://gist.github.com/kianby/e1d455e5fb2a14f8dee3c02c337527f5

SUB_SOCKET_URL = 'tcp://localhost:5555'
PUB_SOCKET_URL = 'tcp://localhost:5556'

if __name__ == "__main__":
    context = zmq.Context()

    # Subscribers >--[5555]-- broker --[5556]--< Publishers

    sub_socket = context.socket(zmq.XPUB)
    sub_socket.bind(SUB_SOCKET_URL)

    pub_socket = context.socket(zmq.XSUB)
    pub_socket.bind(PUB_SOCKET_URL)

    zmq.proxy(sub_socket, pub_socket)

    # We never get here
    sub_socket.close()
    pub_socket.close()
    context.term()
