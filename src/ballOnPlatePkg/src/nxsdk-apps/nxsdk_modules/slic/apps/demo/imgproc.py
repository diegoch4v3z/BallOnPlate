"""
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2017-2021 Intel Corporation.

This software and the related documents are Intel copyrighted
materials, and your use of them is governed by the express 
license under which they were provided to you (License). Unless
the License provides otherwise, you may not use, modify, copy, 
publish, distribute, disclose or transmit  this software or the
related documents without Intel's prior written permission.

This software and the related documents are provided as is, with
no express or implied warranties, other than those that are 
expressly stated in the License.
"""

from binascii import unhexlify
from PIL import Image, ImageEnhance
import numpy as np
import math


def get_CoM(image):
    immat = image.load()
    (X, Y) = image.size
    m = np.zeros((X, Y))
    for x in range(X):
        for y in range(Y):
            m[x, y] = immat[(x, y)] != 0
    m = m / np.sum(np.sum(m))
    dx, dy = np.sum(m, 1), np.sum(m, 0)
    cx, cy = np.sum(dx*np.arange(X)), np.sum(dy*np.arange(Y))
    if math.isnan(cx) or math.isnan(cy):
        return X, Y
    return int(cx), int(cy)


def get_max_aperture_bounding_box(image, margin=30):
    contrast = ImageEnhance.Contrast(image)
    contrast = contrast.enhance(20.0)
    # contrast.save("image_contrast.jpg")
    th = 150
    # mask.save("image_mask.jpg")
    mask = contrast.point(lambda i: i < th and 255)
    immat = mask.load()
    (X, Y) = mask.size
    objsx = {}
    objsy = {}
    y_start = 0
    y_prev = ""
    y_dim = 0
    gap = 2
    for y in range(Y):
        for x in range(X):
            if immat[x, y] != 255:
                if y_prev == "":
                    y_start = y
                elif y > (y_prev + gap):
                    objsy[y_start] = y_dim
                    y_start = y
                    y_dim = 0
                y_prev = y
                y_dim += 1
                objsy[y_start] = y_dim
                break

    x_start = 0
    x_prev = ""
    x_dim = 0
    for x in range(X):
        for y in range(Y):
            if immat[x, y] != 255:
                if x_prev == "":
                    x_start = x
                elif x > (x_prev + gap):
                    objsx[x_start] = x_dim
                    x_start = x
                    x_dim = 0
                x_prev = x
                x_dim += 1
                objsx[x_start] = x_dim
                break
    (dimx, left) = sorted([(dim, x)
                           for (x, dim) in objsx.items()], reverse=True)[0]
    (dimy, up) = sorted([(dim, y)
                         for (y, dim) in objsy.items()], reverse=True)[0]
    if dimx > dimy:
        diff = dimx - dimy
        up = up-int(diff/2)
        dimy = dimx
    elif dimy > dimx:
        diff = dimy - dimx
        left = left-int(diff/2)
        dimx = dimy

    return (left-margin, up-margin, left+dimx+margin, up+dimy+margin)


def crop_to_box(image, box):
    return image.crop(box)


def crop_around_point(image, point, width, height):
    x, y = point
    ix, iy = image.size
    left = max(0, x-width/2)
    top = max(0, y-height/2)
    right = min(ix, x+width/2)
    bottom = min(iy, y+height/2)
    img = image.crop((left, top, right, bottom))
    return img


def comparator_about_mean(image, ceil=1, floor=0, offset=0):
    mean = np.mean(image) + offset
    return np.where(image > mean, ceil, floor)


def reshape(arr, width):
    padding = (width-len(arr) % width) % width
    for i in range(padding):
        arr = np.append(arr, 0)
    mat = np.reshape(arr, (int(len(arr)/width), width))
    return mat


def uint8_to_hex(arr):
    return ''.join(["{0:02x}".format(a) for a in arr])


def uhex_to_uint8(uhex, count=50):
    arr = np.empty((0))
    for h in uhex:
        decoded = np.fromstring(unhexlify(h), dtype='uint8', count=count)
        #decoded = np.fromstring(h.decode('hex'),dtype='uint8',count=count)
        arr = np.append(arr, decoded)
    return arr


def webcam_crop(image, orig_size, crop_size):
    x, y = orig_size
    cx, cy = crop_size
    dx, dy = x-cx, y-cy
    return image.crop((dx/2, dy/2, x-dx/2, y-dy/2))


def webcam_downsample(image, com_window, aperture, margin, offset):
    im = Image.fromarray(comparator_about_mean(
        image, ceil=255, floor=0, offset=offset).astype('uint8'))
    im = im.convert('L')
    if aperture == 1:
        box = get_max_aperture_bounding_box(im, margin=margin)
        im = crop_to_box(im, box)
    elif aperture == 2:
        im = crop_around_point(im, get_CoM(im), com_window, com_window)
    elif aperture == 3:
        box = get_max_aperture_bounding_box(im, margin=margin)
        x = (box[2]-box[0])/2+box[0]
        y = (box[3]-box[1])/2+box[1]
        im = crop_around_point(im, (x, y), com_window, com_window)
    elif aperture == 4:
        im = crop_around_point(im, get_CoM(im), com_window, com_window)
        box = get_max_aperture_bounding_box(im, margin=margin)
        im = crop_to_box(im, box)
    elif aperture == 5:
        box = get_max_aperture_bounding_box(im, margin=margin)
        x = (box[2]-box[0])/2+box[0]
        y = (box[3]-box[1])/2+box[1]
        im = crop_around_point(im, (x, y), com_window, com_window)
        box = get_max_aperture_bounding_box(im, margin=margin)
        im = crop_to_box(im, box)
    return im
