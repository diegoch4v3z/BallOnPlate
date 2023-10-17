###############################################################
# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
# Copyright © 2018-2021 Intel Corporation.
#
# This software and the related documents are Intel copyrighted
# materials, and your use of them is governed by the express
# license under which they were provided to you (License). Unless
# the License provides otherwise, you may not use, modify, copy,
# publish, distribute, disclose or transmit  this software or the
# related documents without Intel's prior written permission.
#
# This software and the related documents are provided as is, with
# no express or implied warranties, other than those that are
# expressly stated in the License.
###############################################################

import numpy as np


class Patch(object):
    """
    A Patch represents a location within an image to which a set of
    convolutional kernels is applied.
    It stores spatial properties of a patch and provides methods to compute
    the overlap between two patches or the location-adjusted dot product
    between dictionary elements.
    """

    def __init__(self, patchId, x0, y0, dx, dy):
        """Initializes patch.

        :param patchId: (int) Unique id of patch.
        :param x0: (int) x location of patch.
        :param y0: (int) y location of patch.
        :param dx: (int) Size of patch in x dimension.
        :param dy: (int) Size of patch in y dimension.
        """
        self.patchId = patchId
        self.x0 = x0
        self.y0 = y0
        self.dx = dx
        self.dy = dy

    def getMask(self, imgSizeX, imgSizeY):
        """Returns a binary mask with True for all pixels that this patch is
        associated with in image and False for all others.

        :param int imgSizeX: Size of image in x dimension
        :param int imgSizeY: Size of image in y dimension
        :return: numpy.array<bool>
        """
        mask = np.zeros((imgSizeY, imgSizeX), dtype=bool)
        mask[self.y0:self.y0+self.dy, self.x0:self.x0+self.dx] = True
        return mask

    def computeOverlapSignature(self, otherPatch, imgSizeX, imgSizeY):
        """Computes binary mask that represents the overlap between this and
        another patch.

        :param Patch otherPatch: Another patch object for which to
        compute overlap with.
        :param int imgSizeX: Size of image in x dimension.
        :param int imgSizeY: Size of image in y dimension.
        :returns: None if there is no overlap. Otherwise returns an OR of
        this and the other patch's masks, cropped to the bounding of the OR
        of the two masks.
        """
        mask1 = self.getMask(imgSizeX, imgSizeY)
        mask2 = otherPatch.getMask(imgSizeX, imgSizeY)
        if (mask1 & mask2).sum() > 0:
            # Only if there's overlap, compute overlapMask
            orMask = mask1 | (mask2 * 2)
            xTrueId = orMask.any(0)
            yTrueId = orMask.any(1)
            return orMask[np.ix_(yTrueId, xTrueId)]
        else:
            return None

    def getLinearOverlapIndices(self, otherPatch):
        x0 = min(self.x0, otherPatch.x0)
        x1 = max(self.x0+self.dx, otherPatch.x0+otherPatch.dx)
        y0 = min(self.y0, otherPatch.y0)
        y1 = max(self.y0 + self.dy, otherPatch.y0 + otherPatch.dy)
        dx = x1-x0
        dy = y1-y0

        frame = np.zeros((dy, dx), dtype=bool)
        frame[self.y0:self.y0+self.dy, self.x0:self.x0+self.dx] = True
        thisIdx = np.reshape(frame, (dx*dy), order='F')

        frame = np.zeros((dy, dx), dtype=bool)
        frame[otherPatch.y0:otherPatch.y0 + otherPatch.dy,
              otherPatch.x0:otherPatch.x0 + otherPatch.dx] = True
        otherIdx = np.reshape(frame, (dx*dy), order='F')

        return thisIdx, otherIdx, dx*dy

    def computeRelativeOverlapExtent(self, otherPatch, imgSizeX, imgSizeY):
        """Computes the extent of current overlap relative to full overlap
        between this and another patch.

        :param Patch otherPatch: Another patch object for which to
        compute overlap with.
        :param int imgSizeX: Size of image in x dimension
        :param int imgSizeY: Size of image in y dimension
        :returns: fraction (float) Sum(Current Overlap)/Sum(Full Overlap)
        """
        mask1 = self.getMask(imgSizeX, imgSizeY)
        mask2 = otherPatch.getMask(imgSizeX, imgSizeY)

        currentOverlap = (mask1 & mask2).sum()
        fullOverlap = self.dy * self.dx

        fraction = currentOverlap / fullOverlap

        return fraction

    def maskToHash(self, mask):
        """Converts binary mask into hash code that uniquely identifies the
        type of overlap.
        :param mask: (numpy.array) Binary overlap mask between two patches.
        :returns (int) hashCode or None if there's no overlap.
        """
        return tuple(mask.reshape(1, mask.size)[0])

    def getOverlapHash(self, otherPatch, imgSizeX, imgSizeY):
        """Computes overlap between this and otherPatch and returns unique
        hashCode identifier for the type of overlap. Returns None if there's
        no overlap."""
        overlapMask = self.computeOverlapSignature(
            otherPatch, imgSizeX, imgSizeY)
        hashCode = None
        if overlapMask is not None:
            frameMask = np.zeros((imgSizeY, imgSizeX), int)
            frameMask[0:overlapMask.shape[0],
                      0:overlapMask.shape[1]] = overlapMask
            hashCode = self.maskToHash(frameMask)
        return overlapMask, hashCode

    def isOverlap(self, otherPatch):
        """Checks if this patch overlaps with the other

        :param Patch otherPatch: The other patch, with which overlap is being
        checked.
        :returns (bool) isOverlap, True if there is overlap
        """
        isoverlap = False
        if (np.abs(self.x0 - otherPatch.x0) < self.dx) or (np.abs(self.y0 - otherPatch.y0) < self.dy):
            isoverlap = True
        return isoverlap
