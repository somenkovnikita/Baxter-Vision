import argparse
from imghdr import what as isimage
from os import listdir
from os.path import isfile, join, abspath


def collect_images(basedir):
    full_path = abspath(basedir)
    all_files = (join(full_path, fn) for fn in listdir(basedir))
    return [fn for fn in all_files if isfile(fn) and isimage(fn)]


def save_images_list(filename, images_list):
    with open(filename, 'w') as out:
        out_str = '\n'.join(images_list)
        out.write(out_str)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Collect images in dir')
    parser.add_argument('-i', '--input_dir', help='Input dir with images', required=True)
    parser.add_argument('-o', '--output_file', help='Output file with images paths', required=True)

    args = parser.parse_args()
    image_paths = collect_images(args.input_dir)
    save_images_list(args.output_file, image_paths)
