import collections

class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.written = []
        self.is_open = True
        self.timeout = kwargs.get('timeout', 1000)
        self.port = kwargs.get('port', 'XXX')
        self.read_queue = collections.deque()
    def write(self, data: bytes):
        self.written.append(data)


    def read(self, size: int = 1) -> bytes:
        # Return queued data one byte at a time (like a serial port)
        if not self.read_queue:
            return b">"
        chunk = self.read_queue[0][:size]
        self.read_queue[0] = self.read_queue[0][size:]
        if len(self.read_queue[0]) == 0:
            self.read_queue.popleft()
        return chunk

    def queue_response(self, data: bytes):
        # Add a full response to the queue
        self.read_queue.append(bytearray(data))


    def close(self):
        self.is_open = False

    def flushInput(self):
        return
    
    def flushOutput(self):
        return