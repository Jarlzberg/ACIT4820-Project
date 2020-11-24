#! /usr/bin/env python
import numpy as np
import array

def save_map(map_array):
        """ Saving map-array as map.pgm, map.yaml and map.npy
            This will be used by the map_server, which is necessary for amcl
        """
        # get path to /maps directory
        path = __file__
        pkg_path = path.split("/src/map_saver.py")[0]
        map_path = pkg_path+"/maps"

        save_pgm(map_array, map_path)
        save_yaml(map_array, map_path)
        save_npy(map_array, map_path)

def save_pgm(map_arr, map_path):
    """ Create map.pgm-file from numpy-array"""
    image_array = (1 - map_arr) * 255

    image_data = array.array('B')
    for i in range(len(image_array)):
        for j in range(len(image_array[i])):
            image_data.append(int(image_array[i][j]))
    
    filename = map_path + "/map.pgm"

    with open(filename,"wb") as f:
        # Standard pgm-header
        size = len(map_arr)

        f.write('P5\n')
        f.write('{} {}\n'.format(str(size), str(size)))
        f.write('{}\n'.format(str(255)))
        
        # writing image data
        image_data.tofile(f)    
        f.close()

def save_yaml(map_arr, map_path):
    """ Create map.yaml-file """
    filename = map_path + "/map.yaml"
    size = len(map_arr)

    with open(filename,"wb") as f:
        f.write('image: ./map.pgm\n')
        f.write('resolution: 1.0\n')
        f.write('origin: [-{}, -{}, 0.000000]\n'.format(str(round(size/2,6)),str(round(size/2,6))))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.2\n')
        f.close()

def save_npy(map_arr, map_path):
    filename = map_path + "/map.npy"
    np.save(filename, map_arr)



### --- map.yaml-format --- ###

# image: ./map.pgm
# resolution: 0.050000
# origin: [-10.000000, -10.000000, 0.000000]
# negate: 0
# occupied_thresh: 0.65
# free_thresh: 0.196



### ---- map.pgm-format --- ###

# P5
# xsize ysize
# maxvalue
# imagedata