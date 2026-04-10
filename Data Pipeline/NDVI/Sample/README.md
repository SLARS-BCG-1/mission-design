### **README: SLARS-BCG-1 NDVI Processing Pipeline**

#### **1. Mission Overview**
This pipeline is the core "Ground Segment" software for the **SLARS-BCG-1 CubeSat mission**. It is designed to ingest raw multispectral data and generate **NDVI (Normalized Difference Vegetation Index)** maps to monitor flood zones and agricultural health in Sri Lanka.

---

#### **2. Prerequisites**
* **Python 3.10+**
* **Required Libraries:** `pip install rasterio numpy`
* **GIS Software:** [QGIS Desktop](https://qgis.org/) (Long Term Release recommended).

---

#### **3. Data Preparation**
The script requires two specific "Raw" spectral bands:
1.  **B04.tiff (Red Band):** Detects chlorophyll absorption.
2.  **B08.tiff (NIR Band):** Detects plant leaf reflection.

**Setup:** Place `B04.tiff`, `B08.tiff`, and your `main.py` script in the same folder:  
`D:\Programming and Web developing\SLARS-BCG-1\Data Pipeline\`

---

#### **4. Running the Pipeline**
1.  Open `main.py` and ensure the `base_path` matches your folder.
2.  Run the script.
3.  **Expected Output:** A new file named **`SLARS_BCG1_NDVI_FINAL.tif`** will appear in your folder.

---

#### **5. QGIS Visualization Instructions (The "Disaster Map" Setup)**
To turn the raw numerical data into a visual intelligence product, follow these steps in QGIS:

**A. Load the Data**
* Drag and drop `SLARS_BCG1_NDVI_FINAL.tif` from your folder into the large **Map Canvas** in QGIS.
* The image will initially appear in **Greyscale** (Black = Water/Buildings, White = Forest).

**B. Apply Color Coding (Symbology)**
1.  In the **Layers Panel** (bottom-left), **Right-Click** your NDVI layer and select **Properties**.
2.  On the left sidebar, click the **Symbology** tab.
3.  Change **Render type** from "Singleband gray" to **Singleband pseudocolor**.
4.  **Color Ramp:** Click the dropdown and select **RdYlGn** (Red-Yellow-Green).
    * *Tip: If you don't see it, click "All Color Ramps" to find it.*
5.  **Classification:** Click the **Classify** button at the bottom to generate the color steps.
6.  Click **OK**.



**C. Analyzing the Science Product**
Use the **Identify Tool** (the blue 'i' icon in the top toolbar) to click on pixels. Compare the values to these mission standards:
* **Values < 0 (Red/Orange):** Water bodies, ocean, or flooded areas.
* **Values 0.1 to 0.3 (Yellow/Light Green):** Bare soil, urban areas, or stressed/drying crops.
* **Values 0.4 to 1.0 (Deep Green):** Dense tropical forest or healthy tea plantations.



---

#### **6. Troubleshooting**
* **"File Not Found":** Double-check if your files end in `.tif` or `.tiff` and update the code to match.
* **Red Error in Browser:** If downloading from Copernicus, ensure your "Crop" area is small enough to stay under the 2500px limit.
* **Image is all one color:** Ensure you clicked the **Classify** button in the Symbology tab before hitting OK.

---
**Mission Latency Goal:** Processing must be completed within 35 minutes of satellite downlink to provide real-time flood alerts to the NDMSC.