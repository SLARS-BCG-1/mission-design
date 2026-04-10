import rasterio
import numpy as np
import os

def calculate_ndwi(green_path, nir_path, output_path):
    print(f"\n--- SLARS-BCG-1: NDWI Flood Detection Mode ---")
    
    with rasterio.open(green_path) as green_src:
        green = green_src.read(1).astype('float32')
        profile = green_src.profile
            
    with rasterio.open(nir_path) as nir_src:
        nir = nir_src.read(1).astype('float32')

    # Step: Compute NDWI
    np.seterr(divide='ignore', invalid='ignore')
    ndwi = (green - nir) / (green + nir)
    ndwi = np.nan_to_num(ndwi, nan=-1.0)

    # Update metadata for float output
    profile.update(dtype=rasterio.float32, count=1, driver='GTiff')

    with rasterio.open(output_path, 'w', **profile) as dst:
        dst.write(ndwi.astype(rasterio.float32), 1)
    print(f"Success! Water index saved to: {os.path.basename(output_path)}")

if __name__ == "__main__":
    # This points exactly to the folder you provided
    base_path = r'D:\Programming and Web developing\SLARS-BCG-1\Data Pipeline\NDWI\Sample'
    
    # Verification: Ensure B03.tiff and B08.tiff are INSIDE the 'Sample' folder
    green_band = os.path.join(base_path, 'B03.tiff') 
    nir_band = os.path.join(base_path, 'B08.tiff')
    
    # The output will be created in that same folder
    output = os.path.join(base_path, 'SLARS_BCG1_NDWI_RESULT.tif')

    calculate_ndwi(green_band, nir_band, output)
