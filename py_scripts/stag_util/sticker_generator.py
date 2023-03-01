import os
import glob
import cv2
import numpy as np

from PIL import Image
from tqdm import tqdm

TAG_DIR = "/home/tpp/Downloads/STagHD11/"

tag_filename_list = glob.glob(os.path.join(TAG_DIR, "*.png"))
tag_filename_list.sort()

def create_row(tag_resolution=1000, tag_size=25, tag_interval=0, paper_width=1190, tag_id_start=0, cut_end=True):
    px_per_mm = tag_resolution / (tag_size)

    tag_amount_per_row = paper_width // (tag_size + tag_interval)

    # Trim the white space in the end
    if cut_end:
        cutted_paper_width = (tag_amount_per_row * (tag_size + tag_interval) + tag_interval)
    else:
        cutted_paper_width = paper_width

    canvas_width = int(cutted_paper_width * px_per_mm)
    canvas_row = np.ones((tag_resolution, canvas_width), dtype=np.uint8) * 255

    # Placing the tags
    for i in range(tag_amount_per_row):
        tag_id = i + tag_id_start
        tag = cv2.imread(tag_filename_list[tag_id], cv2.IMREAD_GRAYSCALE)
        
        if tag.shape[0] != tag_resolution:
            tag = cv2.resize(tag, (tag_resolution, tag_resolution))

        tag_offset_x = int(px_per_mm * (i * (tag_size + tag_interval) + tag_interval))
        canvas_row[:, tag_offset_x: tag_offset_x + tag_resolution] = tag

    return tag_amount_per_row, cutted_paper_width, canvas_row


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


def export_pdf_single(filename, cv2_img):
    
    img = Image.fromarray(cv2_img)
    img.save(filename, "PDF" ,resolution=100.0, save_all=True)


if __name__ == '__main__':
    cutted_paper_width, imgs = create_sticker(tag_resolution=1000, row_number=150, paper_width=1190)
    export_pdf("sticker.pdf", imgs, paper_width=cutted_paper_width, dpi=600)

    