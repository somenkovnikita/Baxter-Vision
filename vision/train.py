import argparse
from time import strftime, gmtime

"""Train multilayer perceptron network for recognition of letters"""
"""
Format line in file with marked images:
<path-to-image> <class>
...
Ex:

cube.png    1
cube2.png   2
"""


def get_default_config_filename():
    hidden = '-'.join(str(h) for h in args.hidden_layers)
    time = strftime('%H:%M', gmtime())
    config_filename = 'net_{0}_{1}_{2}_{3}.wgh'.format(
        args.input_layer, args.output_layer, hidden, time)
    return config_filename


if __name__ == '__main__' :
    LETTER_COUNT = 30

    parser = argparse.ArgumentParser(
        description='Train multilayer perceptron network for recognition of letters')

    # required
    parser.add_argument('-i', '--input_layer', type=int, required=True, help='Size of input layer')
    parser.add_argument('-l', '--hidden_layers', metavar='L', type=int, nargs='+', required=True,
                        help='Size(s) of hidden layer')
    parser.add_argument('-s', '--train_set', required=True, help='File with path and marked images')

    # options
    parser.add_argument('-c', '--output_config', help='Specify own filename output weight file')
    parser.add_argument('-o', '--output_layer', type=int, help='Size of output layer', default=LETTER_COUNT)

    args = parser.parse_args()
    if args.output_config is None:
        args.output_config = get_default_config_filename()

    print args


