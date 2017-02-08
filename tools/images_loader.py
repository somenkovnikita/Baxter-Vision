from tools import ImagesFinder
from tools import utils

finder = ImagesFinder(utils.NEURAL_NET_DIR)
print "Search for training set: "
for images in finder.images_filename:
    print images
finder.save(utils.NEURAL_SET_FILE)