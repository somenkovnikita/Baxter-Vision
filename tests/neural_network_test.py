# coding=utf-8

import numpy, cv2, sys, os
from neural import NeuralNetwork


# Чтение изображений из папки
def read_images(folder):
    # type: (str) -> list(tuple)
    ext = {".jpg", ".jpeg", ".png", ".bmp"}
    files = []
    # Загрузка файлов из папки
    dirs = os.listdir(folder)

    # Убираем лишние файлы
    for d in dirs:
        # Изображения разбиты по классам
        # Классы это папки
        path = os.path.join(folder, d)
        if os.path.isdir(path):
            files_list = os.listdir(path)
            for f in files_list:
                for xt in ext:
                    # Проверяем формат файла
                    full_path = os.path.join(path, f)
                    if full_path.endswith(xt):
                        # Записываем путь до файла + класс(класс==имя папки)
                        files.append((full_path, d))
    # Чтение изображений
    im_data = []
    for (fl, cls) in files:
        image = cv2.imread(fl, cv2.IMREAD_GRAYSCALE)
        im_data.append((image, cls))
    return im_data


def setup_images(im_data, (width, height)):
    result = []

    for im in im_data:
        (image, fl) = im
        # Изображения для нейронки должны быть одного размера
        rs_image = cv2.resize(image, (width, height))
        # Нормализуем
        norm_image = cv2.normalize(rs_image, alpha=-1, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        result.append((norm_image, fl))
    return result


if __name__ == "__main__":
    if sys.argv[1] is None:
        print "Required arg: base dir"
        exit(-1)
    base_dir = sys.argv[1]

    image_size = (200, 200)
    in_layer_size = image_size[0] * image_size[1]
    hidden_layer_size = 2500
    out_layer_size = 30

    images = read_images(base_dir)
    images = setup_images(images, image_size)

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
