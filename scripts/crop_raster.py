import json
import os
import subprocess


def gdal_crop(input_file, output_file, corners, other_options=[]):
    proc_name = 'gdal_translate'
    # Using 'lanczos' instead of default 'nearest' because of bug (feature?)
    # in gdal 2.1 regarding subpixel unalignement http://www.gdal.org/gdal_translate.html
    options = ['-co', 'compress=lzw',
               '-r', 'lanczos',
               '-projwin', str(corners[0][0]), str(corners[0][1]), str(corners[1][0]),
               str(corners[1][1]),
               '-of', 'GTiff']
    options.extend(other_options)
    print(" ".join([proc_name] + options + [input_file, output_file]))
    result = subprocess.run([proc_name] + options + [input_file, output_file])

    if result.returncode != 0:
        raise RuntimeError("Error during execution! gdal_translate returned {}.".format(
            result.returncode))
    return result


def gdal_raster_extent(input_file):
    proc_name = 'gdalinfo'
    options = ['-json']

    result = subprocess.run([proc_name] + options + [input_file], stdout=subprocess.PIPE,
                            universal_newlines=True)

    if result.returncode != 0:
        raise RuntimeError("Error during execution! gdalinfo returned {}.".format(
            result.returncode))

    j = json.loads(str(result.stdout))
    extent = [j["cornerCoordinates"]["upperLeft"], j["cornerCoordinates"]["lowerRight"]]

    return extent


def crop_by_raster_mask(being_cropped_path, mask_raster_path, output_raster_path):
    extent = gdal_raster_extent(mask_raster_path)
    gdcrop = gdal_crop(being_cropped_path, output_raster_path, extent, other_options=["-tr", "5", "5"])
    if gdcrop.returncode != 0:
        raise RuntimeError("Error during execution! gdal_translate returned {}.".format(
            gdcrop.returncode))


if __name__ == "__main__":
    being_cropped = "/home/rbailonr/Documents/landcover/g100_clc06_V18_5_Lambert_compressed.tif"
    mask_dir = "/home/rbailonr/firers_data_5m/dem/RGEALTI_MNT_5M_ASC_LAMB93_IGN69/"
    dest_dir = "/home/rbailonr/firers_data_5m/landcover/RGEALTI_MNT_5M_ASC_LAMB93_IGN69/"
    for f in os.scandir(mask_dir):
        if f.is_file():
            if f.name.endswith(".aux.xml"):
                continue
            mask = f.path
            outfile = os.path.join(dest_dir, "landcover_" + f.name)
            crop_by_raster_mask(being_cropped, mask, outfile)
