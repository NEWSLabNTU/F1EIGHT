#!/usr/bin/env python3
"""Export an OpenCV ArUco dictionary in the aruco library's text format.

    export_opencv_dictionary.py DICT_5X5_1000 > DICT_5X5_1000.dict

Autoware's ar_tag_based_localizer decodes with the aruco library (the one
aruco_ros vendors), not OpenCV's aruco module, and that library ships none of
OpenCV's DICT_* families. Its `dictionary` parameter also takes a file path,
in this format:

    name DICT_5X5_1000
    nbits 25
    <one row per marker id, in id order: the n x n code bits, row-major, 1 = white>

The bits are read off OpenCV's own rendering of each marker, so the file
matches a printed sheet by construction rather than by a bit-packing
convention. Checked against libaruco: ids decode, and the corner order is
OpenCV's, so the marker frame (and every pose) agrees with OpenCV's.
"""
import sys

import cv2


def main():
    name = sys.argv[1]
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    n = d.markerSize
    draw = getattr(cv2.aruco, 'drawMarker', None) or cv2.aruco.generateImageMarker
    print(f'name {name}')
    print(f'nbits {n * n}')
    for i in range(d.bytesList.shape[0]):
        img = draw(d, i, n + 2)   # one pixel per cell, one-cell black border
        print(''.join('1' if v > 127 else '0' for v in img[1:-1, 1:-1].flatten()))


if __name__ == '__main__':
    main()
