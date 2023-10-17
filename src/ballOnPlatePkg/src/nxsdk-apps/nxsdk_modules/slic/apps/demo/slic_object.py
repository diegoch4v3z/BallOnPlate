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

import random
from nxsdk_modules.slic.apps.demo.imgproc import *


class Object:
    """
    Collects images and metadata for a single class of object
    Stores the images as a hex string and the name of the object
    as a string
    """

    def __init__(self, name, flags=None):
        self.name = name
        self.images = []
        if flags is None:
            self.flags = {'transpose': True,
                          'random': True}
        else:
            self.flags = flags

    def add_image(self, img):
        """
        Downsample an image and add it to list of images
        :param img: Image to be added
        """
        self.images.append(self.image_downsample(img))

    def add_images(self, imgs):
        """
        Add a list of images to ImageClassObject class
        :param imgs: List of images
        """
        for img in imgs:
            self.add_image(img)

    def add_image_file(self, img_file):
        """
        Reads an  image from the file, converts it into
        greyscale image and then add it to list of images.
        :param img_file: File where image is located
        """
        # Converts image into greyscale image
        img = Image.open(img_file).convert('L')
        self.add_image(img)

    def add_image_files(self, img_files):
        """
        Add a list of images from given list of files
        :param img_files: List of image files
        """
        for img_file in img_files:
            self.add_image_file(img_file)

    def add_from_db(self, db):
        """
        Adds all the images from the database belonging
        to class with name of the Object.
        :param db: db is a sqlite3 database containing classname -> images info
        """
        cur = db.cursor()
        qry = "SELECT data FROM images WHERE name=?"
        imgs = [x[0] for x in cur.execute(qry, (self.name,))]
        for img in imgs:
            self.images.append(img)

    def add_to_db(self, db):
        """
        Adds the list of images belonging to the
        class to the db.
        :param db: db is a sqlite3 database containing classname -> images info
        """
        cur = db.cursor()
        qry = "INSERT INTO images(name,data) VALUES(?,?)"
        for img in self.images:
            cur.execute(qry, (self.name, img))

    def as_label_image_tuple(self, label):
        """
        Returns list of label, image tuple
        """
        return [(label, img) for img in self.images]

    def as_train_infer_tuples(self, label, ntrain=36, ninfer=36):
        """
        Returns tuple of train and infer images
        :param label: Label of the object
        :param ntrain: Number of images requested for training
        :param ninfer: Number of images requested for inference
        :return:
        """
        shuffledImages = self.images
        if self.flags['random']:
            random.shuffle(shuffledImages)
        train = []
        infer = []
        for img in shuffledImages[0:ntrain]:
            train.append((label, [int(img[i:i+2], 16)
                                  for i in range(0, len(img), 2)]))
        if ninfer > 0:
            for img in shuffledImages[-ninfer:]:
                infer.append((label, [int(img[i:i+2], 16)
                                      for i in range(0, len(img), 2)]))
        else:
            infer = []
        return train, infer

    def image_downsample(self, img, mode='CoM', intermediate_size=128, final_size=20):
        """
        Downsample to intermediate_size for reduce complexity
        of downstream transforms
        :param img: Image to be downsampled
        :param mode: Mode used for downsampling ("LABB" or "CoM")
        :param intermediate_size: Intermediate size of the image
        :param final_size: Final Size of the image
        """
        X, Y = img.size
        scale = int(min(X/float(intermediate_size),
                        Y/float(intermediate_size)))
        scale = 1 if scale is 0 else scale
        img = img.resize((int(X/scale), int(Y/scale)), Image.ANTIALIAS)

        # Crop around Center of Mass
        # Crop Around Largest Aperture Bounding Box
        if mode == 'LABB':
            img = crop_to_box(img, get_max_aperture_bounding_box(img))
        else:
            img = crop_around_point(img, get_CoM(
                img), intermediate_size, intermediate_size)

        # Drop to final size
        img = img.resize((final_size, final_size), Image.ANTIALIAS)

        # Convert to black and white
        img = comparator_about_mean(img)

        # Apply Flags
        if self.flags['transpose']:
            img = np.transpose(img)

        return uint8_to_hex(np.packbits(img.flatten()))

class ObjectSet:
    """
    Set of SLIC Objects
    """
    def __init__(self, flags=None, size=20):
        self.objects = []
        self.size = size
        if flags is None:
            self.flags = {'transpose': True,
                          'random': True}
        else:
            self.flags = flags

    def get_name(self, idx):
        """
        Returns the name of the object
        :param idx: Index in the object list
        :return:
        """
        if idx < len(self.objects):
            return self.objects[idx].name
        else:
            return "Unknown Object"

    def add_object(self, obj):
        """
        Add object to the objectset
        :param obj: Object to be added
        """
        obj.flags = self.flags
        self.objects.append(obj)
        # If the size of the list is max, remove the first object
        if len(self.objects) > self.size:
            self.objects.pop(0)

    def as_train_infer_tuples(self, ntrain=36, ninfer=0):
        """
        Returns set of train and infer list
        :param ntrain:
        :param ninfer:
        :return:
        """
        train, infer = [], []
        for label, obj in enumerate(self.objects):
            t, i = obj.as_train_infer_tuples(label, ntrain=ntrain, ninfer=ninfer)
            train = train + t
            infer = infer + i
        if self.flags['random']:
            random.shuffle(train)
            random.shuffle(infer)
        return train, infer