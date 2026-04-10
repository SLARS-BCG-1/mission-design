import rasterio
import numpy as np
import os

def calculate_ndvi(red_path, nir_path, output_path):
    """
    SLARS-BCG-1 Ground Segment Pipeline
    Converts raw S-band spectral data into Disaster Intelligence (NDVI).
    """
    print(f"\n--- SLARS-BCG-1 Pipeline: Processing Start ---")
    
    # Check if files exist
    if not os.path.exists(red_path):
        print(f"PIPELINE ERROR: {red_path} not found.")
        return
    if not os.path.exists(nir_path):
        print(f"PIPELINE ERROR: {nir_path} not found.")
        return

    try:
        print("Step 1: Ingesting Band 3 (Red) and Band 4 (NIR)...")
        with rasterio.open(red_path) as red_src:
            red = red_src.read(1).astype('float32')
            profile = red_src.profile
            
        with rasterio.open(nir_path) as nir_src:
            nir = nir_src.read(1).astype('float32')

        # Step 2: Disaster Intelligence Computation
        print("Step 2: Computing Normalized Difference Vegetation Index...")
        
        # Suppress warnings for division by zero (happens in deep water/shadows)
        np.seterr(divide='ignore', invalid='ignore')
        
        # NDVI Formula: (NIR - Red) / (NIR + Red)
        ndvi = (nir - red) / (nir + red)

        # Step 3: Data Formatting
        # We map NoData and Errors to -1.0 (Water/Background)
        ndvi = np.nan_to_num(ndvi, nan=-1.0, posinf=1.0, neginf=-1.0)

        # Update metadata for 32-bit Float GeoTIFF output
        profile.update(
            dtype=rasterio.float32,
            count=1,
            compress='lzw',
            driver='GTiff'
        )

        # Step 4: Final Science Product Export
        print(f"Step 3: Saving Science Product to: {os.path.basename(output_path)}")
        with rasterio.open(output_path, 'w', **profile) as dst:
            dst.write(ndvi.astype(rasterio.float32), 1)

        print("--- Pipeline Success! Result Ready for NDMSC Analysis ---")

    except Exception as e:
        print(f"CRITICAL SYSTEM FAILURE: {e}")

# ==========================================
# MISSION CONTROL EXECUTION
# ==========================================
if __name__ == "__main__":
    # Your specific mission folder
    base_path = r'D:\Programming and Web developing\SLARS-BCG-1\Data Pipeline'
    
    # Try .tiff first (since that's what your screenshot showed)
    # If the files are .tif, simply change these two lines:
    red_band_file = os.path.join(base_path, 'B04.tiff')
    nir_band_file = os.path.join(base_path, 'B08.tiff')
    
    # Final output name
    final_output = os.path.join(base_path, 'SLARS_BCG1_NDVI_FINAL.tif')

    calculate_ndvi(red_band_file, nir_band_file, final_output)
