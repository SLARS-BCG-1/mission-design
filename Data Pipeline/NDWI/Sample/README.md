# **🛰️ SLARS-BCG-1: NDWI Flood Detection Pipeline**

This guide covers the **Normalized Difference Water Index (NDWI)**, the dedicated "Flood Mode" for the SLARS-BCG-1 mission.

---

### **1. Python Processing Instructions**

#### **Required Bands**
Ensure the following files are in your `...\NDWI\Sample` folder:
* **B03.tiff** (Green Band)
* **B08.tiff** (Near-Infrared Band)

#### **The Script (`main.py`)**
```python
import rasterio
import numpy as np
import os

def calculate_ndwi(green_path, nir_path, output_path):
    print(f"--- SLARS-BCG-1: NDWI Processing Start ---")
    
    with rasterio.open(green_path) as green_src:
        green = green_src.read(1).astype('float32')
        profile = green_src.profile
            
    with rasterio.open(nir_path) as nir_src:
        nir = nir_src.read(1).astype('float32')

    # NDWI Formula: (Green - NIR) / (Green + NIR)
    np.seterr(divide='ignore', invalid='ignore')
    ndwi = (green - nir) / (green + nir)
    
    # Clean up NoData
    ndwi = np.nan_to_num(ndwi, nan=-1.0)

    profile.update(dtype=rasterio.float32, count=1, driver='GTiff')

    with rasterio.open(output_path, 'w', **profile) as dst:
        dst.write(ndwi.astype(rasterio.float32), 1)
    print(f"Success! Map saved as: {os.path.basename(output_path)}")

if __name__ == "__main__":
    base_path = r'D:\Programming and Web developing\SLARS-BCG-1\Data Pipeline\NDWI\Sample'
    green_band = os.path.join(base_path, 'B03.tiff') 
    nir_band = os.path.join(base_path, 'B08.tiff')
    output = os.path.join(base_path, 'SLARS_BCG1_NDWI_RESULT.tif')

    calculate_ndwi(green_band, nir_band, output)
```

---

### **2. QGIS Visualization Instructions**

To convert the raw data into a visual flood report for the NDMSC:

#### **Step A: Load Layer**
* Drag `SLARS_BCG1_NDWI_RESULT.tif` into QGIS.

#### **Step B: Symbology (The "Flood View")**
1.  **Right-click** the layer > **Properties** > **Symbology**.
2.  Set **Render type** to `Singleband pseudocolor`.
3.  **Color Ramp:** Choose **Blues**.
4.  **Min/Max Settings:**
    * Set **Min** to **`0.0`** (Hides dry land).
    * Set **Max** to **`0.5`** (Highlights deep water).
5.  Click **Classify** and then **OK**.

#### **Step C: Analysis Thresholds**
Use the **Identify Tool** (blue 'i' icon) to verify pixels:
* **Value > 0.3:** Open water (Rivers, Reservoirs).
* **Value 0.1 to 0.3:** Flooded land or saturated soil.
* **Value < 0.0:** Dry land / Vegetation (should appear white or transparent).

---

### **3. Troubleshooting**
* **`RasterioIOError`:** Check if your file is named `B03.tiff` or `B03.tif`. The code must match the filename exactly.
* **All Blue Map:** Adjust your **Min** value to `0.0`. If the Min is `-1`, the land will be colored light blue, making it hard to see the actual water.