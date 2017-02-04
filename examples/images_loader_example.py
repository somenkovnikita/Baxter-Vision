from tools import ImagesFinder

finder = ImagesFinder('../datasets/neural/training_set')
print 'Found training_set: '
for images in finder.images_filename:
    print images
finder.save('../config/training_set.txt')