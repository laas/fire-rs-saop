import subprocess

# TODO: Crop from boundaries of raster tiles in a folder

def gdal_crop(input_file, output_file, corners):
    proc_name = 'gdal_translate'
    # Using 'lanczos' instead of default 'nearest' because of bug (feature?)
    # in gdal 2.1 regarding subpixel unalignement http://www.gdal.org/gdal_translate.html
    options = ['-r', 'lanczos',
               '-projwin', str(corners[0][0]), str(corners[0][1]), str(corners[1][0]), str(corners[1][1]),
               '-of', 'GTiff']
    result = subprocess.run([proc_name]+options+[input_file, output_file])
    return result

if __name__ == "__main__":
    gdcrop = gdal_crop('/home/rbailonr/g100_clc06_V18_5_Lambert_compressed_25.tif',
              '/home/rbailonr/g100_clc06_V18_5_Lambert_compressed_25_0525_6250_yeeees.tif',
              ((524987.5, 6250012.5), (549987.5, 6225012.5)))
    if gdcrop.returncode != 0:
        raise RuntimeError("Error during execution! gdal_translate returned {}.".format(
            gdcrop.returncode))
