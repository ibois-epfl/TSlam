import os
import glob
from this import d
import cv2
import numpy as np

from PIL import Image
from tqdm import tqdm

TAG_DIR = "/home/tpp/Downloads/STagHD11/"

tag_filename_list = glob.glob(os.path.join(TAG_DIR, "*.png"))
tag_filename_list = [f for f in tag_filename_list if "(" not in f] # there're some duplicated files
tag_filename_list.sort()
# print(len(tag_filename_list))
# exit(0)

def create_row(tag_resolution=1000, tag_size=25, tag_interval=4, paper_width=1190, tag_id_start=0):
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
        tag = cv2.imread(tag_filename_list[tag_id], cv2.IMREAD_GRAYSCALE)
        
        if tag.shape[0] != tag_resolution:
            tag = cv2.resize(tag, (tag_resolution, tag_resolution))

        tag_offset_x = text_canvas_width + int(px_per_mm * (i * (tag_size + tag_interval) + tag_interval))
        canvas_row[:, tag_offset_x: tag_offset_x + tag_resolution] = tag

    return tag_amount_per_row, paper_width, canvas_row


def create_sticker(tag_resolution=1000, tag_size=25, tag_interval=0, paper_width=1190, row_number=1, tag_id_start=0):
    imgs = []

    for i in tqdm(range(row_number)):
        tag_amount_per_row, cutted_paper_width, canvas_row = create_row(tag_resolution, tag_size, tag_interval, paper_width, tag_id_start)
        tag_id_start += tag_amount_per_row

        imgs.append(canvas_row)
    
    return cutted_paper_width, imgs


def create_by_size(tag_resolution=1000, tag_size=25, tag_interval=4, interval_row=0, paper_height=297, paper_width=210, tag_id_start=0):
    """
    This function can be used to create, for example, a paper of tags in A4
    """
    px_per_mm = tag_resolution / (tag_size)

    canvas = np.ones((int(paper_height * px_per_mm), int(paper_width * px_per_mm)), dtype=np.uint8) * 255

    row_amount = paper_height // (tag_size + interval_row)

    for i in range(row_amount):
        tag_amount_per_row, canvas_row = create_row(tag_resolution, tag_size, tag_interval, paper_width, tag_id_start)
        tag_id_start += tag_amount_per_row

        offset_y = int(px_per_mm * (i * (tag_size + interval_row) + interval_row))
        canvas[offset_y: offset_y + tag_resolution, :] = canvas_row

    return canvas


def export_pdf(filename, cv2_imgs, paper_width, dpi = 600):
    print("Exporting PDF...")

    imgs = []
    for img in cv2_imgs:
        
        width = int(paper_width * 0.0393700787 * dpi) # 1mm = 0.0393700787 inch
        scale = width / img.shape[1]
        img = cv2.resize(img, (0, 0), fx=scale, fy=scale)

        img = Image.fromarray(img)
        imgs.append(img)

    imgs[0].save(filename, "PDF" ,resolution=dpi, save_all=True, append_images=imgs[1:])


if __name__ == '__main__':
    # 22309 tags in total, each row contains 47 tags => 474.66 rows
    cutted_paper_width, imgs = create_sticker(tag_resolution=1000, row_number=150, paper_width=1190, tag_id_start=0)
    export_pdf("sticker_1.pdf", imgs, paper_width=cutted_paper_width, dpi=600)
    cutted_paper_width, imgs = create_sticker(tag_resolution=1000, row_number=150, paper_width=1190, tag_id_start=7051)
    export_pdf("sticker_2.pdf", imgs, paper_width=cutted_paper_width, dpi=600)
    cutted_paper_width, imgs = create_sticker(tag_resolution=1000, row_number=150, paper_width=1190, tag_id_start=14101)
    export_pdf("sticker_3.pdf", imgs, paper_width=cutted_paper_width, dpi=600)
    cutted_paper_width, imgs = create_sticker(tag_resolution=1000, row_number=24, paper_width=1190, tag_id_start=21151)
    export_pdf("sticker_4.pdf", imgs, paper_width=cutted_paper_width, dpi=600)
    