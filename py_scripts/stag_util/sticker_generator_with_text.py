#! /usr/bin/env python3

import os
import io
import gc
import argparse

import glob

import cv2
import numpy as np

from PIL import Image
from tqdm import tqdm

import requests
import zipfile
import shutil

__LEN_TAG_DICT__ = 22309
__MODULE_PATH__ = os.path.dirname(os.path.abspath(__file__))

def create_tags_pdf(filename="",
                    tag_resolution=1000,
                    tag_size=20,
                    tag_interval=0,
                    paper_width=1190,
                    row_number=1,
                    tag_id_start=0,
                    dpi=600):
    """
        Create a pdf file of stickers with text labelling.
        There is a total of 22309 tags in total in the dictionnary,
        each row contains 47 tags => 474.66 rows.
        e.g.:
        The id tags goes as follow for a width of 1190 mm and a row_number of 150:
        - from tag_id_start 0: the last id is 7050
        - the next, from tag_id_start 7051: the first id is 7051, the last id is 14100
        - the next, from tag_id_start 14101: the first id is 14101, the last id is 21150
        - the last one, from tag_id_start 21151: the first id is 21151, the last id is 22308

        Args:
            tag_resolution (int): the resolution of the tag
            tag_size (int): the size of the tag in mm
            tag_interval (int): the interval between tags in mm
            paper_width (int): the width of the paper in mm
            row_number (int): the number of rows of stickers
            tag_id_start (int): the id of the first tag
            dpi (int): the dpi of the pdf
    """
    global TAG_FILENAMES_LISTS
    TAG_FILENAMES_LISTS = _download_extract()

    cutted_paper_width, imgs, tag_amount_per_row = _create_all_tags_rows(tag_resolution,
                                                                         tag_size,
                                                                         tag_interval,
                                                                         paper_width,
                                                                         row_number,
                                                                         tag_id_start)
    end_id_tag = _export_pdf(filename, tag_id_start, row_number, tag_amount_per_row, imgs, cutted_paper_width, dpi)

    return tag_amount_per_row, end_id_tag

def _create_tags_row(tag_resolution=1000,
                    tag_size=25,
                    tag_interval=0,
                    paper_width=1190,
                    tag_id_start=0):
    """
        Create a row of stickers with text labelling

        Args:
            tag_resolution (int): the resolution of the tag
            tag_size (int): the size of the tag in mm
            tag_interval (int): the interval between tags in mm
            paper_width (int): the width of the paper in mm
            tag_id_start (int): the id of the first tag

        Returns:
            tag_amount_per_row (int): the number of tags in the row
            cutted_paper_width (int): the width of the paper after trimming
            canvas_row (np.array): the canvas of the row
    """
    px_per_mm = tag_resolution / (tag_size)

    tag_amount_per_row = paper_width // (tag_size + tag_interval)

    # trim the width to fit the tags
    cutted_paper_width = (tag_amount_per_row * (tag_size + tag_interval) + tag_interval)
    # and the rest goes for the text
    text_canvas_width = int((paper_width - cutted_paper_width) * px_per_mm)

    # Placing the text
    text_canvas = np.ones((text_canvas_width, tag_resolution), dtype=np.uint8) * 255
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 7
    color = (0, 0, 0)
    thickness = 5
    cv2.putText(text_canvas, str(tag_id_start), (96, text_canvas_width // 2), font, font_scale, color, thickness, cv2.LINE_AA)
    cv2.putText(text_canvas, str(tag_id_start + tag_amount_per_row - 1), (96, text_canvas_width - 64), font, font_scale, color, thickness, cv2.LINE_AA)
    text_canvas = cv2.rotate(text_canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Placing the tags
    canvas_width = int(paper_width * px_per_mm)
    canvas_row = np.ones((tag_resolution, canvas_width), dtype=np.uint8) * 255
    
    canvas_row[:, 0:text_canvas_width] = text_canvas

    for i in range(tag_amount_per_row):
        tag_id = i + tag_id_start
        tag = cv2.imread(TAG_FILENAMES_LISTS[tag_id], cv2.IMREAD_GRAYSCALE)
        
        if tag.shape[0] != tag_resolution:
            tag = cv2.resize(tag, (tag_resolution, tag_resolution))

        tag_offset_x = text_canvas_width + int(px_per_mm * (i * (tag_size + tag_interval) + tag_interval))
        canvas_row[:, tag_offset_x: tag_offset_x + tag_resolution] = tag

    return tag_amount_per_row, paper_width, canvas_row

def _create_all_tags_rows(tag_resolution=1000,
                         tag_size=25,
                         tag_interval=0,
                         paper_width=1190,
                         row_number=1,
                         tag_id_start=0):
    """
        Create all the stripes of stickers rows

        Args:
            tag_resolution (int): the resolution of the tag
            tag_size (int): the size of the tag in mm
            tag_interval (int): the interval between tags in mm
            paper_width (int): the width of the paper in mm
            row_number (int): the number of rows of stickers
            tag_id_start (int): the id of the first tag

        Returns:
            cutted_paper_width (int): the width of the paper after trimming
            imgs_cv (list): a list of cv2 images
    """
    imgs_cv = []
    for i in tqdm(range(row_number), desc="Creating stickers"):
        tag_amount_per_row, cutted_paper_width, canvas_row = _create_tags_row(tag_resolution,
                                                                              tag_size,
                                                                              tag_interval,
                                                                              paper_width,
                                                                              tag_id_start)
        tag_id_start += tag_amount_per_row
        imgs_cv.append(canvas_row)
    
    return cutted_paper_width, imgs_cv, tag_amount_per_row

def _export_pdf(filename, tag_id_start, row_number, tag_amount_per_row, cv2_imgs, paper_width, dpi = 600):
    """
        Export the cv2 images to a pdf

        Args:
            filename (str): the name of the pdf file
            tag_id_start (int): the id of the first tag
            row_number (int): the number of rows of stickers
            tag_amount_per_row (int): the number of tags in the row
            cv2_imgs (list): a list of cv2 images
            paper_width (int): the width of the paper in mm
            dpi (int): the dpi of the pdf
    """
    # set filename with params and output folder
    if filename == "":
        filename = ("sticker_" + str(row_number) + "batch_"
                               + str(paper_width) + "mm_"
                               + str(dpi) + "dpi_"
                               + str(tag_id_start) + "_"  + str(tag_id_start+(row_number*tag_amount_per_row)-1) + "_"
                               + ".pdf")
    file_dir = os.path.join(__MODULE_PATH__, "pdf_out")
    if os.path.exists(file_dir) == False:
        os.mkdir(file_dir)
    file_path = os.path.join(file_dir, filename)

    # export to pdf
    imgs = []
    for i in tqdm(range(cv2_imgs.__len__()), desc="Exporting to pdf"):
        img = cv2_imgs[i]
        
        width = int(paper_width * 0.0393700787 * dpi) # 1mm = 0.0393700787 inch
        scale = width / img.shape[1]
        img = cv2.resize(img, (0, 0), fx=scale, fy=scale)

        img = Image.fromarray(img)
        imgs.append(img)

    imgs[0].save(file_path, "PDF" ,resolution=dpi, save_all=True, append_images=imgs[1:])
    imgs.clear()

    # return the last tag id of the pdf
    end_id_tag = tag_id_start+(row_number*tag_amount_per_row)-1
    return end_id_tag

def _download_extract() -> list:
    """
    Download and extract the tags from Zenodo
    """
    # make an HTTP request within a context manager
    with requests.get("https://zenodo.org/record/7733825/files/STagHD11.zip", stream=True) as r:
        if r.status_code != 200: # check if request was successful
            print("Error: could not download tags")
            exit()
        total_length = int(r.headers.get("Content-Length"))
        
        # create a new folder to store tags
        # __module_path__ = os.path.dirname(os.path.realpath(__file__))
        folder_tags_dir = os.path.join(__MODULE_PATH__, "STagHD11")
        folder_tags_png_dir = os.path.join(folder_tags_dir, "HD11")
        if not os.path.exists(folder_tags_dir):
            os.mkdir(folder_tags_dir)
        zip_name = os.path.basename(r.url)
        zip_path = os.path.join(folder_tags_dir, zip_name)

        # download and save the output to a file
        if not os.path.exists(zip_path):
            with open(zip_path, 'wb')as output:
                with tqdm.wrapattr(r.raw, "read",
                                   total=total_length,
                                   desc="Downloading tags")as raw:
                    shutil.copyfileobj(raw, output)
        
        # extract the zip file
        if not os.path.exists(folder_tags_png_dir):
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                print(f"Extracting files...")
                for file in tqdm(iterable=zip_ref.namelist(),
                                 total=len(zip_ref.namelist()),
                                 desc="Extracting tags"):
                    zip_ref.extract(file, folder_tags_dir)


        # sort and clean the tags/directory
        tag_filename_list = glob.glob(os.path.join(folder_tags_png_dir, "*.png"))
        tag_filename_list = [f for f in tag_filename_list if "(" not in f] # there're some duplicated files
        tag_filename_list.sort()
    return tag_filename_list


def main(_filename : str,
         _tag_resolution : int,
         _tag_size : int,
         _tag_interval : int,
         _paper_width : int,
         _row_number : int,
         _tag_id_start : int,
         _dpi : int) -> None:
    
    if _tag_id_start != 0:
        create_tags_pdf(filename=_filename,
                        tag_resolution=_tag_resolution,
                        tag_size=_tag_size,
                        tag_interval=_tag_interval,
                        paper_width=_paper_width,
                        row_number=_row_number,
                        tag_id_start=_tag_id_start,
                        dpi=_dpi)
    else:
        nbr_batches = __LEN_TAG_DICT__ // (_row_number * _paper_width // (_tag_size + _tag_interval))
        tags_per_row = 0;
        end_id_tag = 0;
        for i in range(nbr_batches):
            print(f"Batch {i+1}/{nbr_batches}")

            if i == 0:
                current_tag_id_start = 0
            else:
                current_tag_id_start = end_id_tag + 1

            new_filename = _filename + "_" + str(i*_row_number + 1)

            tags_per_row, end_id_tag = create_tags_pdf(filename=_filename,
                                                       tag_resolution=_tag_resolution,
                                                       tag_size=_tag_size,
                                                       tag_interval=_tag_interval,
                                                       paper_width=_paper_width,
                                                       row_number=_row_number,
                                                       tag_id_start=current_tag_id_start,
                                                       dpi=_dpi)

            # collect memeory to avoid overflow
            gc.collect()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                        formatter_class=argparse.RawDescriptionHelpFormatter,
                        epilog= "Example: python3 create_tags_pdf.py --filename=tags --tag_size=25 --tag_resolution=1000 --tag_interval=0 --paper_width=1180 --row_number=50 --tag_id_start=0 --dpi=600",
                        prog="STag pdf generator for print",
                        description="Create a pdf with stickers. The tags are downloaded from Zenodo. By default the app will print all the tags on 25mm and 1180mm -A0- width. If you want to print only a sequence set --tag_id_start with the first tag id to print. It is not possible to set the y-offset to the paper stripe boarder for now.")
    parser.add_argument('--filename', type=str, default="",
                        help='the name of the pdf file, it is not necessary')
    parser.add_argument('--tag_size', type=int, default=25,
                        help='the size of the entire stripe, the vertical border and the tag in mm')
    parser.add_argument('--tag_resolution', type=int, default=1000,
                        help='the resolution of the tag')
    parser.add_argument('--tag_interval', type=int, default=0,
                        help='the inter axis spacing between tags in mm. The distance tag-tag will be the double of the value (2*tag_interval). By if the value is 0 the distance is set to 5')
    parser.add_argument('--paper_width', type=int, default=1190,
                        help='the width of the paper in mm')
    parser.add_argument('--row_number', type=int, default=50,
                        help="the number of rows of stickers. If you experience memory issues try to reduce this value")
    parser.add_argument('--tag_id_start', type=int, default=0,
                        help='the id of the first tag')
    parser.add_argument('--dpi', type=int, default=600,
                        help='the dpi of the pdf')
    args = parser.parse_args()

    if "." in args.filename:
        print("Warning: the filename should not contain the extension")
        exit()
    if args.tag_id_start > 0:
        if args.tag_id_start > __LEN_TAG_DICT__:
            print("Error: the tag_id_start is greater than the number of tags")
            exit()
        if args.tag_id_start + args.row_number * 10 > __LEN_TAG_DICT__:
            print("Error: the tag_id_start is greater than the number of tags")
            exit()
        print("Printing only a tag selection")
    if args.tag_id_start == 0:
        print("Printing all the tags")

    main(_filename=args.filename,
            _tag_resolution=args.tag_resolution,
            _tag_size=args.tag_size,
            _tag_interval=args.tag_interval,
            _paper_width=args.paper_width,
            _row_number=args.row_number,
            _tag_id_start=args.tag_id_start,
            _dpi=args.dpi
    )