import numpy as np
import yaml


class ColorScale():
    def __init__(self, args=None):
        self.bypass = not args['type'].lower() in ("linear_ends", "custom")
        self.colorscale = {0: [0,0,0], 1: [255,255,255]}
        self.unknown_color = [120, 120, 120]
        self.bounds = (0, 1)

        # Parse the colorscale
        if not self.bypass:
            self.unknown_color = np.fromstring(args['unknown'], dtype=np.uint8, sep=',')
            colorlist = [np.fromstring(values, dtype=np.uint8, sep=',') for values in args['values']]
            
            if args['type'].lower() == "linear_ends":
                indices = np.linspace(args['stops'][0], args['stops'][1], num=len(colorlist), endpoint=True)
                self.colorscale = dict(zip(indices, colorlist))
                self.bounds = (np.min(indices), np.max(indices))
            
            elif args['type'].lower() == "custom":
                indices = np.array(args['stops'])
                # There need to be enough indices for the colors provided!
                assert(len(indices) == len(colorlist))
                self.colorscale = dict(zip(indices, colorlist))
                self.bounds = (np.min(indices), np.max(indices))
            
            # check for absolute ends (optional)
            try:
                ends = args['ends']
                ends = {0: np.fromstring(ends['zero'], dtype=np.uint8, sep=','),
                        1: np.fromstring(ends['one'],  dtype=np.uint8, sep=','),}
                self.colorscale.update(ends)
                self.bounds = (0, 1)
            except:
                pass

    def mapToColor(self, property):
        # return unknown friction if we don't know it
        if property is None or property < self.bounds[0] or property > self.bounds[1]:
            return self.unknown_color[0], self.unknown_color[1], self.unknown_color[2]
        # set arbitrary which will change.
        lower_color = (-1, np.array([0,0,0]))
        upper_color = (1, np.array([255,255,255]))
        # get the stops
        for stop, color in self.colorscale.items():
            if stop >= lower_color[0] and stop < property:
                lower_color = (stop, np.array(color))
            elif stop <= upper_color[0] and stop >= property:
                upper_color = (stop, np.array(color))
        # compute the linear interpolation
        xp = np.array([lower_color[0], upper_color[0]])
        fp = np.column_stack((lower_color[1], upper_color[1])).astype(float)/255.0
        return np.interp(property, xp, fp[0,:]), np.interp(property, xp, fp[1,:]), np.interp(property, xp, fp[2,:])
    

