import queue


class MovingAverage:
    def __init__(self, length):
        self.length = length
        self.reset()

    def reset(self):
        self.queue = queue.Queue(maxsize=self.length)
        self.sum = 0

    def add(self, sample):
        if self.queue.full():
            self.sum -= self.queue.get_nowait()
        self.queue.put_nowait(sample)
        self.sum += sample
        return self.sum / self.queue.qsize()

    def full(self):
        return self.queue.full()