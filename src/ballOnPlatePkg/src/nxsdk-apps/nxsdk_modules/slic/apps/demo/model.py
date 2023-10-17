import os
import sqlite3

from nxsdk_modules.slic.apps.demo import slic_object
from nxsdk_modules.slic.apps.demo.slic_classifier import SlicClassifier

class SlicModel:
    """
    SlicModel takes in slic classifier and clasifies
    given object.
    """
    def __init__(self):
        # Initializing the Slic Model
        self.classifier = SlicClassifier()
        # Coil object set
        self.obj_set = slic_object.ObjectSet()
        # Variable to keep track of trained objects
        self.trained = 0
        # Do the first round of training, so that demo can start with inference
        self.initialTraining()

    def initialTraining(self):
        """
        Does the intial training on existing object set
        """
        dbFile = (os.path.dirname(os.path.realpath(__file__))+ '/example.db')
        db = self.init_db(dbFile)
        for i in range(20):
            name = 'coil{0}'.format(i + 1)
            obj = slic_object.Object(name)
            obj.add_from_db(db)
            self.obj_set.add_object(obj)
        # Initialize the model
        self.loihi_train(ntrain=36, ninfer=0)
        self.trained = 0

    def addObject(self, object_label, object_images):
        """
        Add a new object to the object set
        :param object_label: Label of the object
        :param object_images: Images associated with the object
        """
        obj = slic_object.Object(object_label)
        obj.add_images(object_images)
        self.obj_set.add_object(obj)

    def get_object_name(self, objIdx):
        """
        Returns the name of the object, given its idx
        """
        return self.obj_set.get_name(objIdx)

    def loihi_train(self, ntrain=36, ninfer=0):
        """
        Run the training on loihi
        :param ntrain: Number of images per object class to be used for training
        :param ninfer: Number of images per object class to be used for inference
        """
        self.trained += 1
        print("Training Loihi .....")
        train, infer = self.obj_set.as_train_infer_tuples(ntrain, ninfer)
        self.classifier.train_on_dataset(train)

    def loihi_infer(self, image):
        """
        Runs inference on loihi
        :param image: Image to be classified
        :return: inference
        """
        print("Sending inference image to Loihi: %s\nWait Loihi's feedback....." % image)
        obj = slic_object.Object('infer obj')
        obj.add_image(image)
        train, infer = obj.as_train_infer_tuples(0, ntrain=0, ninfer=1)
        inference = self.classifier.infer_on_dataset(infer)
        expect, actual, counters = inference[0][0], inference[0][1][0], inference[0][1][1]
        return self.get_guess(counters)

    def sort_counters(self, counters):
        """
        Sort the counters value
        """
        tuples = [(counters[i], i) for i in range(len(counters))]
        return sorted(tuples, key=lambda x: x[0], reverse=True)

    def num_nonzero(self, scnt):
        """
        Returns number of object class having non-zero spike count
        """
        nonzero = 0
        for count, idx in scnt:
            if count > 0:
                nonzero += 1
        return nonzero

    def sum_counters(self, scnt):
        """
        Returns Sum of spikes for all the object class
        """
        total = 0
        for count, idx in scnt:
            total += count
        return total

    def is_unknown(self, scnt, threshold=5):
        """
        Returns if the object is unkown
        :param scnt: sorted value of counters obtained from slicnet
        :param threshold: threshold to make judgemnent
        """
        nonzero = self.num_nonzero(scnt)
        total = self.sum_counters(scnt)
        return (nonzero == 0 or  # No neurons spiking
                scnt[0][0] == scnt[1][0] or  # Neurons are tied.
                total < threshold)  # Not a lot of spikes

    def is_unsure(self, scnt, factor=2.1):
        """
        Returns whether any guess couldn't be made with certainity
        :param scnt: sorted value of counters obtained from slicnet
        :param factor: multiplier
        """
        return scnt[0][0] < scnt[1][0] * factor

    def get_guess(self, counters):
        """
        Returns the guess made by Loihi
        :return: tuple of (is_unsure, is_unkown, guessed_idx, actual_idx)
        """
        scnt = self.sort_counters(counters)
        return self.is_unsure(scnt), self.is_unknown(scnt), scnt[0][1], scnt[1][1]

    def init_db(self, db):
        """
        Initializes the database of images
        """
        db = sqlite3.connect(db)
        cur = db.cursor()
        qry = "CREATE TABLE IF NOT EXISTS images (name TEXT,data TEXT)"
        cur.execute(qry)
        return db
