from tools import ImagesFinder

# FIXME: hardcoded paths?
finder = ImagesFinder("../datasets/neural/training_set")
print "Found training set: "
for images in finder.images_filename:
    print images
finder.save("../config/training_set.txt")