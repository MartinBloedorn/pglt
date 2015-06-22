#!/usr/local/bin/python

import argparse
import sys

from pglt_planner import Planner

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='PCB GCode Leveling Tool. '
                                                 'Probe or load a heightmap of the PCB you wish to mill. '
                                                 'Then load the GCode that will be fitted to the surface\'s '
                                                 'irregularities')
    parser.add_argument('-p', '--probe',
                        help='Use PGLT to probe the PCB surface, generate a heigthmap and save it to a file',
                        action='store_true')
    parser.add_argument('-c', '--conf',
                        help='PGLT configuration file. Command line options override the file\'s')
    parser.add_argument('-hm', '--heightmap', help='Location of the heightmap (save/load)')
    parser.add_argument('-gc', '--gcode',
                        help='Location of the GCode to load.')
    parser.add_argument('-o', '--outgcode',
                        help='Modified gcode output.')
    parser.add_argument('-ex', '--excellon',
                        help='Location of the excellon drill file (to avoid holes while probing - default)')
    parser.add_argument('-id', '--ignoredrills', action='store_true',
                        help='Ignore excellon file\'s drills while probing.')
    mirror_group = parser.add_mutually_exclusive_group()
    mirror_group.add_argument('-mx', '--mirrorx', type=float,
                        help='Mirror the drill file along an axis parallel to the X axis, at Y = MIRRORX')
    mirror_group.add_argument('-my', '--mirrory', type=float,
                        help='Mirror the drill file along an axis parallel to the Y axis, at X = MIRRORY')
    parser.add_argument('-plt', '--plot',
                        help='Enable plots. Displays drills and probe points in probe mode, '
                             'and surface heightmaps used for GCode levelig.',
                        action='store_true')
    parser.add_argument('-v', '--verbose',
                        help='Runs with verbose output.',
                        action='store_true')
    args = parser.parse_args()

    p = Planner()

    # Process arguments
    if not args.conf and not args.gcode and not args.probe:
        print 'Wrong usage! Use -h to get some help.'
        sys.exit(0)

    if args.plot:
        p.en_plot = True
    if args.verbose:
        p.en_verbose = True
    if args.conf:
        print 'Reading configuration file ({})...'.format(args.conf)
        p.read_conf(args.conf)
    if args.heightmap:
        p.hmap_path = args.heightmap
    if args.gcode:
        p.gcode_path = args.gcode
    if args.outgcode:
        p.gcode_out_path = args.outgcode
    if args.excellon:
        p.excellon_path = args.excellon
    if args.mirrorx:
        p.probe_drill_mirror_axis = 'x'
        p.probe_drill_mirror_location = args.mirrorx
    if args.mirrory:
        p.probe_drill_mirror_axis = 'y'
        p.probe_drill_mirror_location = args.mirrory
    if args.ignoredrills:
        print 'ignoring drills!'
        p.probe_drill_ignore = True

    # Main behaviour block
    if args.probe:
        #p.plan_hmap_probing(not args.ignoredrills)
        p.probe_hmap()

    elif p.gcode_path != '' and p.hmap_path != '':
        p.read_hmap()
        p.process_gcode()




