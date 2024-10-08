import numpy as np
import math
import svgpathtools as svg

class Path:
    def __init__(self,config:dict):
        self.track_id = config['file']
        self.steps = config['steps']
        self.track_length = config['length']
        unique_radii = config['unique_radii']
        self.radii = []
        self.curvature = []
        self.xpos = []
        self.ypos = []
        self.dL = []
        self.dL_sum = []
        self.r_set = []
        self.norm = []
        self.r_min = 10
        # self.parse(self.track_id,self.track_length)
        # self.calc(steps=self.steps,unique_radii=unique_radii)
        
    # Read the SVG File
    def parse(self,track_id,track_length):
        svg_read = svg.svg2paths(track_id)
        svg_path = svg_read[0]
        self.track = svg.Path(svg_path[0])
        if self.track.iscontinuous()==False:
            print('Discontinuous Track SVG')
        if self.track.isclosed()==False:
            print('Open Tack SVG')
        if len(svg.kinks(self.track))>1:
            print(svg.kinks(self.track))
            self.track=svg.smoothed_path(self.track)
            print('Warning: Non-differentiable Track SVG, Smoothing')
            # Untested
        # Extract single path from path group
        while len(self.track)==1:
            self.track=self.track[0]
        # Scale track s.t. length = track length (eg. 2 kilometers for endurance)
        scalar=(track_length/self.track.length(0,1))
        self.track=self.track.scaled(scalar)

    # Calculate all track parameters
    def calc(self, steps, unique_radii):
        for i in range(steps):
            t=i/(steps)
            # Find path segment index and sub-position
            seg_index, sub_t = self.track.T2t(t)[0],self.track.T2t(t)[1],
            # Find curvature
            kappa = svg.path.segment_curvature(self.track[seg_index],sub_t)
            kappa_signed = kappa*self.turn_dir(t)
            self.curvature.append(kappa_signed)

            # Calculate radius
            if kappa==0:
                self.radii.append(0)
            else:
                r=(1/kappa)
                if r <= self.r_min:
                    self.radii.append(self.r_min)
                else:
                    self.radii.append(1/kappa)
            self.xpos.append(svg.real(self.track.point(t)))
            self.ypos.append(svg.imag(self.track.point(t)))
            self.dL.append(self.track.length(t,t+(1/(steps))))
            self.dL_sum.append(self.track.length(0,t+(1/(steps))))
        
        # Set of Radii for GGV  
        radii_nonzero = [i for i in self.radii if i != 0]
        radii_unique = list(set(radii_nonzero))
        radii_sorted = np.sort(radii_unique)
        if unique_radii > len(radii_unique):
            print('unique_radii exceeds the number of unique radii:', unique_radii,'>',len(radii_unique))
        elif unique_radii <= 0:
            print('unique_radii must be greater than or equal to 1')
        else:
            for i in range(unique_radii):
                n = (int)(i*(len(radii_sorted)/(unique_radii)))
                self.r_set.append(radii_sorted[n])

    def turn_dir(self,t): # This library doesn't support signed curvature :(
        dz = self.track.derivative(t)
        ddz = self.track.derivative(t,2)
        dx, dy = dz.real, dz.imag
        ddx, ddy = ddz.real, ddz.imag
        return np.sign(dx*ddy-dy*ddx)