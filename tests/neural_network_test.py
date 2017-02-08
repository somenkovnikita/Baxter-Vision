import numpy, sys
from tools import utils
from neural import NeuralNetwork


if __name__ == "__main__":
    if sys.argv[1] is None:
        print "Required arg: base dir"
        exit(-1)
    base_dir = sys.argv[1]

    image_size = (200, 200)
    in_layer_size = image_size[0] * image_size[1]
    hidden_layer_size = 2500
    out_layer_size = 30

    images = utils.read_images(base_dir)
    images = utils.setup_images(images, image_size)

    uniq_classes = set([cl for _, cl in images])
    classes = {cl: i for i, cl in enumerate(uniq_classes)}

    im_size = len(images)

    input_array = numpy.zeros((im_size, in_layer_size), "float")
    output_array = numpy.zeros((im_size, out_layer_size), "float") - 1.0

    print "Setup network arrays..."
    for i in range(im_size):
        (image, fl) = images[i]
        a = numpy.array(list(image), "float")
        print "#%d image from dir(class): " % i + fl
        input_array[i, :] = a.flatten()
        output_array[i, classes[fl]] = 1.0
        print input_array[i, :]
        print output_array[i, :]

    layers = (in_layer_size, hidden_layer_size, out_layer_size)
    network = NeuralNetwork(layers)
    network.train(input_array, output_array, True)
    print images[0][0]
    a = numpy.array(list(images[0][0]), "float")
    cc = numpy.zeros((im_size, in_layer_size), "float")
    cc[0, :] = a.flatten()
    res = network.classify(cc)
    print res[0].shape
