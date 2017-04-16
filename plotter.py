#!/usr/bin/python
import argparse
import math
# import msgpackrpc
import socket
import time

# 762 508 508 0 10
INCREASE = 1
IDLE = 0
DECREASE = -1

# DISTANCE_PER_TICK = 1
DISTANCE_PER_TICK = 0.59817 # mm
SLEEP_TIME = .005

client = None
right_distance = 0
left_distance = 0
right_x = 0
right_y = 0
left_x = 0
left_y = 0
curr_x = 0
curr_y = 0

def parse_args():
  parser = argparse.ArgumentParser(description='Process some integers.')
  parser.add_argument('distance', metavar='D', type=float, help='Distance between motors')
  parser.add_argument('left', metavar='L', type=float, help='Length of left string')
  parser.add_argument('right', metavar='R', type=float, help='Length of right string')
  parser.add_argument('x', metavar='x', type=float, help='Length of right string')
  parser.add_argument('y', metavar='y', type=float, help='Length of right string')
  parser.add_argument('-H', '--host', default='localhost', help="Host")
  return parser.parse_args()

# calulates angle opposite of side c in radians
def law_of_cos(a, b, c):
  return math.acos((a**2 + b**2 - c**2)/(2* a * b))

def calculate_motor_pts(d, l, r):
  right_angle = law_of_cos(d, r, l)
  left_angle = law_of_cos(d, l, r)
  print math.degrees(right_angle)
  print math.degrees(left_angle)
  r_y = math.sin(right_angle) * r
  r_x = math.cos(right_angle) * r
  l_y = math.sin(left_angle) * l
  l_x = -math.cos(left_angle) * l
  return [(r_x, r_y), (l_x, l_y)]

def calc_distance(x1, y1, x2, y2):
  print math.sqrt(((x2-x1)**2) + ((y2-y1)**2))
  return math.sqrt(((x2-x1)**2) + ((y2-y1)**2))

def calc_left_distance(x,y):
  global right_x, left_x, right_y, left_y
  # if (x > right_x or x < left_x or y > right_y):
  #   return -1
  # else:
  return calc_distance(x, y, left_x, left_y)

def calc_right_distance(x,y):
  global right_x, left_x,right_y
  # if (x > right_x or x < left_x or y > right_y):
  #   return -1
  # else:
  return calc_distance(x, y, right_x, right_y)

# given current and desired distance returns number of ticks
def distance_to_ticks(curr, desired):
  print "curr: {}, desired {}".format(curr, desired)
  print ((desired - curr) * (1/DISTANCE_PER_TICK))
  return round((desired - curr) * (1/DISTANCE_PER_TICK))


def tick(left, right):
  global right_distance, left_distance, client
  left = IDLE
  right = IDLE

  if left != 0:
    if (left_distance < left):
      left = INCREASE
    else:
      left = DECREASE

  if right != 0:
    if (right_distance < right):
      right = INCREASE
    else:
      right = DECREASE
  client.call('tick', left, right)



def tick_right(desired):
  global right_distance, client
  if (right_distance < desired):
    # print "o"
    # sock.sendto("o", (UDP_IP, UDP_PORT))
    client.call('tick_right', INCREASE)
  else:
    # print "l"
    # sock.sendto("l", (UDP_IP, UDP_PORT))
    client.call('tick_right', DECREASE)

def tick_left(desired):
  global left_distance, sock
  if (left_distance < desired):
    # print "i"
    client.call('tick_left', INCREASE)
  else:
    # print "k"
    client.call('tick_left', DECREASE)


def move_to_pt(x, y):
  global right_distance, left_distance, sock, curr_x, curr_y

  # Segment line into multiple motions to get smoother lines
  num_segments = int(calc_distance(curr_x, curr_y, x, y)/10)

  # print int(calc_distance(curr_x, curr_y, x, y))

  dx = (x-curr_x)/num_segments
  dy = (y-curr_y)/num_segments

  # print dx
  # print dy

  for i in xrange(1, num_segments+1):
    print "At {},{} segments.".format(i, num_segments)
    # Calculate end of current line segment
    target_x = curr_x + dx
    target_y = curr_y + dy

    print "X: %s ->, %s, Y: %s -> %s"  % (curr_x, target_x, curr_y, target_y)

    curr_x = target_x
    curr_y = target_y;

    # Calculate required lengths for end of current line segment
    new_right_dist = calc_right_distance(target_x, target_y)
    new_left_dist = calc_left_distance(target_x, target_y)

    # print "right distance is: {}, used to be {}".format(new_right_dist, right_distance)
    # print "left distance is: {}, used to be {}".format(new_left_dist, left_distance)

    ticks_right = abs(distance_to_ticks(right_distance, new_right_dist))
    ticks_left = abs(distance_to_ticks(left_distance, new_left_dist))
    total_ticks = max(ticks_left, ticks_right)
    print "Right ticks: {}, left ticks {}". format(ticks_right, ticks_left)
    tick_ratio = max(ticks_right, ticks_left)/min(ticks_right, ticks_left)

    left_accum = 0
    right_accum = 0
    for _ in xrange(int(total_ticks)):

      left_steps = 0
      right_steps = 0

      left_accum += ticks_left
      if (left_accum > total_ticks):
        left_accum -= total_ticks
        left_steps = new_left_dist

      right_accum += ticks_right
      if (right_accum > total_ticks):
        right_accum -= total_ticks
        right_steps = new_right_dist

      # print "%s, %s, %s" % (left_accum, right_accum, total_ticks)
      tick(left_steps, right_steps)
      time.sleep(SLEEP_TIME)
    right_distance = new_right_dist
    left_distance = new_left_dist


def main():
  args = parse_args()
  dist = args.distance
  global left_distance, right_distance, client
  left_distance = args.left
  right_distance = args.right
  motor_pts = calculate_motor_pts(dist, left_distance, right_distance)
  print motor_pts

  global right_x,right_y, left_x, left_y
  right_x = motor_pts[0][0]
  right_y = motor_pts[0][1]
  left_x = motor_pts[1][0]
  left_y = motor_pts[1][1]

  # client = msgpackrpc.Client(msgpackrpc.Address(args.host, 18800))

  # client.call('pen_up')
  # client.call('reset')
  # client.call('pen_down')
  # move_to_pt(args.x,args.y)
  # move_to_pt(50, 0)
  # move_to_pt(0, -50)
  # move_to_pt(-50, 0)
  # move_to_pt(0, 50)
  # move_to_pt(50, 0)
  # move_to_pt(50, -50)
  # move_to_pt(0, -50)
  # move_to_pt(0, 0)
  move_to_pt(65, -130*.866)
  move_to_pt(-65, -130*.866)
  move_to_pt(0,0)
  print dist
  print left_distance
  print right_distance
  print calculate_motor_pts(dist, left_distance, right_distance)



if __name__ == '__main__':
  main()