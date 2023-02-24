#!/usr/bin/python3

import json
import argparse
import copy
import math

# "standard" field lengths, in *meters*
SHED_FIELD_LENGTH = 8.780
STD_FIELD_LENGTH = (54*12.0 + 3.25) * 0.0254

class TrajectoryFlipper:
    def __init__(self, length):
        self.field_length = length
        return

    def try_flip_coordinates(self, entry):
        # print('testing', entry)
        if type(entry) is dict and 'x' in entry and 'y' in entry:
            entry['x'] = self.field_length - entry['x']
            return True
        return False

    def try_flip_holonomic(self, entry):
        # print('testing', entry)
        if type(entry) is dict and 'holonomicAngle' in entry:
            # PathPlanner write out in degrees
            a = math.radians(entry['holonomicAngle'])

            # we need to *reflect* the angle, which is not a simple rotation
            # easiest is to use sin() and cos().
            # The x value (cos) goees negative, the y stays the same
            new_a = math.atan2(math.sin(a), -math.cos(a))

            entry['holonomicAngle'] = math.degrees(new_a)
        return

    def traverse_structure(self, curr_struct):
        t = type(curr_struct) 
        if t is dict:
            entries = curr_struct.items()
        elif t is list or t is tuple:
            entries = curr_struct
        else:
            return

        for entry in entries:
            # if this is a coorinate pair, it is flipped, and then we dont need to walk into it
            if self.try_flip_coordinates(entry):
                continue

            # flip any holonomicAngle
            self.try_flip_holonomic(entry)

            # check any substructure
            self.traverse_structure(entry)
        return

    def flip_trajectory(self, pp_traj):
        # Make a full copy of the data structure.
        # This might not be needed, but is a better practice, in most cases.
        new_traj = copy.deepcopy(pp_traj)

        # Walk the structure recursively
        self.traverse_structure(new_traj)
        return new_traj


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Remove columns from a CSV file')
    parser.add_argument('--field', help='Named field type: shed or (competition or standard)')
    parser.add_argument('--length', '-l', default=STD_FIELD_LENGTH, help='Field length (meters)')
    parser.add_argument('--output', '-o', required=True, help='Output file name')
    parser.add_argument('inputfile', nargs=1, help='Input file')

    args = parser.parse_args()

    # pick up the field length
    field_length = float(args.length)
    if args.field:
        if args.field.tolower == 'shed':
            field_length = SHED_FIELD_LENGTH
        else:
            field_length = STD_FIELD_LENGTH

    flipper = TrajectoryFlipper(float(args.length))

    with open(args.inputfile[0]) as instrm:
        pp_traj = json.load(instrm)

    new_traj = flipper.flip_trajectory(pp_traj)

    with open(args.output, 'w') as outstrm:
        json.dump(new_traj, outstrm)
