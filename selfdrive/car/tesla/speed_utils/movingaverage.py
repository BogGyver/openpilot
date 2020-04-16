class MovingAverage():
  def __init__(self, length):
    self.position = 0
    self.length = length
    self.sum = 0.
    self.no_items = 0
    self.values = [0.] * length

  def reset(self):
    self.position = 0
    self.sum = 0.
    self.no_items = 0
    self.values = [0.] * self.length 

  def add(self,element):
    if self.no_items == self.length:
      self.no_items -= 1
      self.sum -= self.values[self.position]
    self.values[self.position] = element
    self.sum += self.values[self.position]
    self.no_items += 1
    self.position += 1
    if self.sum == 0.:
      #all empty so initialize
      self.position = 0
      self.sum = 0.
      self.no_items = 0
      return 0.
    self.position = self.position % self.length
    return self.sum/self.no_items

  def dele(self):
    if self.no_items == 0:
      return 0.
    if self.no_items > 0:
      self.no_items -= 1
      self.sum -= self.values[self.position]
      self.values[self.position] = 0.
      self.position -= 1
      if self.position < 0:
        self.position = self.length-1
    if self.sum == 0. or self.no_items == 0.:
      #all empty so initialize
      self.position = 0
      self.sum = 0.
      self.no_items = 0
      return 0.
    self.position = self.position % self.length
    return self.sum/self.no_items
