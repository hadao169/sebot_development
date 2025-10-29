import math

class Encoder():
  encoder_min =  -32768 #16-bit signed integer
  encoder_max =  32768
  encoder_range = encoder_max - encoder_min

	# Define wrap-around thresholds at 30% and 70% of the encoder range
  encoder_low_wrap = (encoder_range * 0.3) + encoder_min
  encoder_high_wrap = (encoder_range * 0.7) + encoder_min


  def __init__(self, wheel_radius, ticks_per_revolution):

    self.wheel_radius = wheel_radius
    
		#number of ticks per revolution
    self.ticks_per_revolution = ticks_per_revolution 

    # wheel radius R = 0.1m
    # encoder resolution E = 1000 ticks / revolution

    # wheel circumference: 2 * pi * R
    # revolutions per meter = 1m / wheel circumference
    # ticks per meter = E * (1m / (2 * pi * R)) = E / (2 * pi * R)

		#number of ticks per meter
    self.ticks_per_meter = self.ticks_per_revolution / (2 * math.pi * self.wheel_radius)

		#initial values
    self.offset = None #initialize the starting position being 0 ticks
    self.encoder = None
    self.prev_encoder = None

    self.position = 0
    self.prev_position = None

    self.multiplier = 0

	#called whenever a new encoder value is read - deals with wrap-around
  def update(self, encoder):
    if self.encoder == None:
      self.offset = encoder
      self.prev_encoder = encoder

    self.encoder = encoder

    # In encoders that measure rotation, there is often a limited range of values (e.g., 0–4095 for a 12-bit encoder).
    # When the encoder value "wraps around" (e.g., 4095 -> 0 or vice versa), this logic ensures that the correct position is maintained by tracking full revolutions.
    #
    # self.encoder: Current encoder reading.
    # self.prev_encoder: Previous encoder reading.
    # self.encoder_low_wrap: Threshold value near the lower limit (0 + margin)
    # self.encoder_high_wrap: Threshold value near the upper limit (4095 − margin)
    # self.multiplier: Counter that tracks full revolutions
    #
    # i.e., when the previous value is in the upper range (e.g., 4090-4095) and the current value is in the lower range
    # (e.g., 0-5), then the absolute position increases, and in the other direction, the absolute position decreases.
    # In all other cases, we are on the same revolution.

    if (self.encoder < self.encoder_low_wrap) and (self.prev_encoder > self.encoder_high_wrap):
      self.multiplier += 1
    if (self.encoder > self.encoder_high_wrap) and (self.prev_encoder < self.encoder_low_wrap):
      self.multiplier -= 1

		# Calculate the absolute position
    self.position = self.encoder + self.multiplier * self.encoder_range - self.offset

    self.prev_encoder = self.encoder # update previous encoder value


# Returns distance traveled by 1 wheel since last call in meters
  def deltam(self):
    if self.prev_position == None:
      self.prev_position = self.position
      return 0
    else:
      d = (self.prev_position - self.position) / self.ticks_per_meter
      self.prev_position = self.position # update previous position
      return d