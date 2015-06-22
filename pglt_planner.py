import scipy as sp
from scipy import interpolate
from scipy.spatial.distance import pdist

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt

import serial
import time
import sys

import csv
import re

class Planner:
    """ Class for motion planning and file parsing.
    """

    def __init__(self):
        self.excellon_path = ''
        self.gcode_path = ''
        self.gcode_out_path = ''
        self.hmap_path = ''

        self.surf_f = []

        self.hmap_x = []
        self.hmap_y = []
        self.hmap_z = []

        self.en_verbose = False # enable verbosity
        self.en_plot = False # enable plots

        self.plan_method = 'linear'

        self.probe_drill_mirror_axis = 'none'
        self.probe_drill_mirror_location = 0.0
        self.probe_drill_tolerance = 1.0         # minimal distance of probing point to drill
        self.probe_drill_ignore = False          # ignore the drills when probing

        #params in conf file
        self.probe_xmin, self.probe_ymin = 0.0, 0.0     # probe origin (x,y)
        self.probe_xmax, self.probe_ymax = 10.0, 10.0   # probe end (x,y)
        self.probe_xpts, self.probe_ypts = 5, 4         # num of points along x and y. Total = xpts*ypts

        self.comm_port = '/dev/ttyUSB0'
        self.comm_baud = 115200
        self.comm_firmw = 'marlin'

    def read_conf(self, conf_path = './pglt.conf'):
        """ Reads a file with each line formatted as
            argument : value
            and sets the class accordingly
        """
        conf_file = open(conf_path, 'r')
        #print 'Reading configuration file {}'.format(conf_path)

        nline = 0
        for line in conf_file:
            nline += 1

            m = re.match('\s*(?P<cmd>\w+)\s*=\s*(?P<arg>\S*)', line)
            if m is not None and len(m.groups()) == 2:
                cmd = m.group('cmd')
                arg = m.group('arg').strip()

                if cmd == 'comm_port':
                    self.comm_port = arg
                    #print 'Setting comm port to {}'.format(arg)
                elif cmd == 'comm_baud':
                    self.comm_baud = int(arg)
                elif cmd == 'comm_firmw':
                    self.comm_firmw = arg
                elif cmd == 'probe_xmin':
                    self.probe_xmin = float(arg)
                elif cmd == 'probe_ymin':
                    self.probe_ymin = float(arg)
                elif cmd == 'probe_xmax':
                    self.probe_xmax = float(arg)
                elif cmd == 'probe_ymax':
                    self.probe_ymax = float(arg)
                elif cmd == 'probe_xpts':
                    self.probe_xpts = int(arg)
                elif cmd == 'probe_ypts':
                    self.probe_ypts = int(arg)
                #Files
                elif cmd == 'hmap_path':
                    #print 'Setting hmap_path to {}'.format(arg)
                    self.hmap_path = arg
                elif cmd == 'gcode_path':
                    self.gcode_path = arg
                elif cmd == 'gcode_out':
                    self.gcode_out_path = arg
                elif cmd == 'excellon_path':
                    self.excellon_path = arg

            elif re.match('\s*#.*', line) is not None and self.en_verbose:
                print 'Igonring comment in line {} in configuration file {}'.format(nline, conf_path)
            elif self.en_verbose:
                print 'Igonring line {} in {}'.format(nline, conf_path)

    def read_hmap(self):
        """ Reads heightmap from CSV file.
        """
        self.hmap_x, self.hmap_y, self.hmap_z = [], [], []

        if self.hmap_path != '':
            with open(self.hmap_path, 'rb') as csvf:
                line = csv.reader(csvf, delimiter=',')
                for row in line:
                    self.hmap_x.append(float(row[0]))
                    self.hmap_y.append(float(row[1]))
                    self.hmap_z.append(float(row[2]))

    def write_hmap(self, hmaplist):
        if self.hmap_path != '':
            hmap_out = open(self.hmap_path, 'w')
            hmap_out.flush()
            for p in hmaplist:
                hmap_out.writelines('{},{},{}\n'.format(p[0], p[1], p[2]))
            print 'Wrote heightmap to {}'.format(self.hmap_path)


    def plan_hmap_probing(self, avoid_drills=False):

        dlist = [] # drill points list
        plist = [] # probe points list
        avoid_drills = avoid_drills and self.excellon_path != ''

        if avoid_drills:
            drill_file = open(self.excellon_path, 'r')
            print 'Reading drills in {}'.format(self.excellon_path)

            for line in drill_file:
                m = re.match('\s*(?!;)\s*(X(?P<X>\s*[-0-9\.]*))?\s*(Y(?P<Y>\s*[-0-9\.]*))', line)
                if m is not None and m.group('X') and m.group('Y'):
                    d = sp.array([float(m.group('X')), float(m.group('Y'))])
                    dlist.append(d)

            #if self.en_verbose:
            #    for d in dlist:
            #        print 'Hole in X {} and Y {}'.format(d[0], d[1])

            # mirror drills if needed
            if self.probe_drill_mirror_axis == 'x':
                for d in dlist:
                    d[1] += 2*(self.probe_drill_mirror_location - d[1])
            elif self.probe_drill_mirror_axis == 'y':
                for d in dlist:
                    d[0] += 2*(self.probe_drill_mirror_location - d[0])

        # Probe points' increments in the X and Y direction
        x_inc = (self.probe_xmax - self.probe_xmin)/(self.probe_xpts - 1)
        y_inc = (self.probe_ymax - self.probe_ymin)/(self.probe_ypts - 1)

        for i in range(0,self.probe_xpts):
            for j in range(0,self.probe_ypts):
                p = sp.array([x_inc*i + self.probe_xmin, y_inc*j + self.probe_ymin])
                cdrill = [] #colliding drill

                if avoid_drills:
                    is_clear = True # set if probe is colliding with a drill
                    clist = [] # list of drills that are close

                    for d in dlist: # Iterate over drills
                        dist = pdist([p,d])[0]
                        if dist < self.probe_drill_tolerance:
                            clist.append(d)
                            cdrill = d
                            is_clear = False
                        elif dist < self.probe_drill_tolerance*5.0:
                            clist.append(d)

                    iter = 0
                    while not is_clear: # some collison is happening
                        # get all vectors from the drills in clist to p
                        d2p_list = []
                        for d in clist:
                            v = sp.subtract(p, d)
                            v = v/(sp.linalg.norm(v)**2)
                            d2p_list.append(v)
                        res_vec = sp.sum(d2p_list, axis=0)
                        res_vec = (self.probe_drill_tolerance/5.0)*(res_vec/sp.linalg.norm(res_vec)) # fixed step
                        p = sp.sum([p, res_vec], axis=0)

                        # verify
                        for d in clist:
                            is_clear = pdist([p, d])[0] > self.probe_drill_tolerance
                            if not is_clear:
                                break

                        if is_clear and self.en_verbose:
                            print 'Avoided drill X{} Y{} by moving probe to {}'.format(cdrill[0], cdrill[1], p.round(4))
                        iter += 1
                        if iter > 20 and not is_clear:
                            print 'Unable to avoid drill at X{} Y{}, exiting'.format(cdrill[0], cdrill[1])
                            sys.exit(0)
                        elif iter > 5 and not is_clear: # adding some spice if shit does not converge
                            rand_vec = (self.probe_drill_tolerance/5.0)*sp.array([sp.rand()-0.5, sp.rand()-0.5])
                            p = sp.sum([p, rand_vec], axis=0)

                plist.append(p.round(4))

        if self.en_plot:
            xp, yp = [], []
            xd, yd = [], []

            fig = plt.gcf()
            ax = plt.gca()

            for p in plist:
                xp.append(p[0])
                yp.append(p[1])
            for d in dlist:
                fig.gca().add_artist(plt.Circle((d[0], d[1]), self.probe_drill_tolerance, color='r', fill=False))
                xd.append(d[0])
                yd.append(d[1])

            ax.plot(xp, yp, linestyle='', marker='x')
            ax.plot(xd, yd, linestyle='', marker='o', color='r')
            ax.set_xlim(1.1*self.probe_xmin, 1.1*self.probe_xmax)
            ax.set_ylim(1.1*self.probe_ymin, 1.1*self.probe_ymax)
            plt.show()

        return plist


    def probe_hmap(self):
        """ Connects to mill, probes bed and writes it to an CSV file.
        """
        ser = []
        ans = ''
        hmap = [] # stores end hmap

        #generate the drills
        print 'Generating the probe points...'
        plist = self.plan_hmap_probing(not self.probe_drill_ignore)
        if self.en_plot:
            ans = raw_input('Probe points generated. Wish to begin probing? [y/N]')
        else:
            ans = raw_input('Probe points generated. Enable --plot to see them. Wish to begin probing? [y/N]:   ')

        if ans.strip() != 'y' and ans.strip() != 'Y':
            print 'Aborting...'
            return

        if self.hmap_path == '':
            print 'No path supplied to save heightmap, aborting.'
            return

        try:
            ser = serial.Serial(self.comm_port, self.comm_baud)
        except OSError as e:
            print "Trouble connecting to {}, got error: {}".format(self.comm_port, e.strerror)
        except:
            print "Unexpected error:", sys.exc_info()[0]
            raise

        # Connected succesfully
        if ser:
            print 'Connected succesfully to {}'.format(self.comm_port)
            time.sleep(1) #some time for marlin to wake up
            while ser.inWaiting() < 900:
                pass
            while ser.inWaiting():
                print ser.readline(),

            ser.flushInput()
            # begin probing, not caring for acks =(
            ser.write('G28 Z0\nG92 X{} Y{} Z0\n'.format(self.probe_xmin, self.probe_ymin))
            time.sleep(0.5)

            for p in plist:
                probedp = False
                print 'Sending cmds for point {} {}'.format(p[0], p[1])
                ser.write('G01 F1000 Z5\n')
                ser.write('G01 F10000 X{} Y{}\nG30\n'.format(p[0], p[1]))
                time.sleep(0.5)

                while not probedp:
                    while ser.inWaiting() == 0:
                        time.sleep(0.1)
                    while ser.inWaiting() > 0:
                        line = ser.readline()
                        m = re.match('\s*(Bed)\s*(X:(?P<X>\s*[-0-9\.]*))?\s*(Y:(?P<Y>\s*[-0-9\.]*))\s*(Z:(?P<Z>\s*[-0-9\.]*))', line)
                        if m is not None and m.group('Z') is not None:
                            probedp = True
                            z = float(m.group('Z'))
                            print 'Probed point X{} Y{} at Z{}'.format(p[0], p[1], z)
                            hmap.append([p[0], p[1], z])

            ser.write('G01 F1000 Z5\n')
            print 'Finished probing heightmap!'

            self.write_hmap(hmap)
            ser.close()


    def plan_to(self, tpoint):
        """ Generates intermediary points for gcode trajectory.

        Takes a target point, a surface funciont ( z = f(x,y) ) and a threshold.
        Generates intermediary points so that the z offset is < threshold.
        """

        # Re-leveling at target point
        tZ = self.surf_f(tpoint[0],tpoint[1])[0] + tpoint[2]

        return [tpoint[0], tpoint[1], tZ]

    def process_gcode(self):
        if len(self.hmap_z) >= 16:
            self.surf_f = interpolate.interp2d(self.hmap_x, self.hmap_y, self.hmap_z, kind='cubic')
        elif len(self.hmap_z) >= 4:
            self.surf_f = interpolate.interp2d(self.hmap_x, self.hmap_y, self.hmap_z, kind='linear')
        else:
            print 'Heightmap too small or not loaded. Aborting GCode processing.'
            return

        if self.en_plot:
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            ax.plot_trisurf(self.hmap_x, self.hmap_y, self.hmap_z, cmap=cm.jet, linewidth=0.2)
            plt.show()


        #Open g-code and iterate over lines
        gfile = open(self.gcode_path, 'r')
        outfile = open(self.gcode_out_path, 'w')

        print 'Processing GCode {} into {}'.format(self.gcode_path, self.gcode_out_path)

        # Incoming points
        gX, gY, gZ = 0.0, 0.0, 0.0
        lZ = 0.0
        nline = 0

        for line in gfile:
            nline += 1

            m = re.match('\s*(?P<G>G..)\s*(?P<F>F\s*[-0-9\.]*)?\s*(X(?P<X>\s*[-0-9\.]*))?\s*(Y(?P<Y>\s*[-0-9\.]*))?\s*(Z(?P<Z>\s*[-0-9\.]*))?', line)

            #got something
            if m is not None:

                #G00 and G01
                if m.group('G') == 'G00' or m.group('G') == 'G01':
                    gX = float(m.group('X')) if m.group('X') else gX
                    gY = float(m.group('Y')) if m.group('Y') else gY
                    gZ = float(m.group('Z')) if m.group('Z') else gZ

                    t = self.plan_to((gX, gY, gZ))

                    newline = m.group('G')
                    if m.group('F'):
                        newline += m.group('F')
                    if m.group('X'):
                        newline += 'X{}'.format(round(t[0], 4))
                    if m.group('Y'):
                        newline += 'Y{}'.format(round(t[1], 4))

                    if t[2] != lZ:
                        newline += 'Z{}\n'.format(round(t[2], 4))
                    else:
                        newline += '\n'
                    lZ = t[2]

                    outfile.writelines(newline)


                #Throughput everything else
                else:
                    outfile.writelines(line)
            else:
                outfile.writelines(line)

        outfile.close()
        gfile.close()
