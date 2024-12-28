# Pallet Detection and Segmentation Using YOLOv11 and YOLOv11-seg

This repository is focused on **Pallet Detection and Segmentation** using **YOLOv11** and **YOLOv11-seg** models. The project involves detecting pallets and segmenting them from their background in images using a **custom dataset**. The models are fine-tuned on this dataset to achieve high accuracy in detecting and segmenting pallets.

### Key Highlights:
- **Custom Dataset:** A dataset of **519 images** was used, with **90 images manually annotated** using **Roboflow** for detection and segmentation tasks. 
- **YOLOv11 for Detection:** YOLOv11 was trained for pallet detection with a **mAP of around 60**.
- **YOLOv11-seg for Segmentation:** The segmentation model was used to extract pallet areas at the pixel level.
- **ROS Integration:** The system includes two ROS 2 packages:
  - One for receiving images from a topic and performing detection and segmentation.
  - Another for simulating camera input by publishing images from a folder to a ROS topic.

## Dataset Annotation and Model Training

### a. Dataset Annotation
- **Tool Used:** The dataset was annotated using **Roboflow**, where **90 images** were manually labeled for the detection task out of the total **519 images**.
- **Roboflow Links:**
  - **Pallet Segmentation Dataset:** [Segmentation Dataset](https://app.roboflow.com/assignment-zmesa/segment-pallets/)
  - **Semantic Segmentation Dataset:** [Semantic Segmentation Dataset](https://app.roboflow.com/assignment-zmesa/semantic-segment-pallets/)
  - **Detection Dataset:** [Pallet Dataset](https://app.roboflow.com/assignment-zmesa/pallet-2ntff/)
- **Model Deployment:** The dataset was uploaded to **Roboflow** for training the YOLOv11 detection model. The initial model was then used to further annotate the dataset.

### b. YOLOv11 for Detection
- **Detection Model:** The **YOLOv11** model was used for pallet detection, achieving a **mean Average Precision (mAP) of around 60** on the custom dataset.

### c. YOLOv11-seg for Segmentation
- **Segmentation Model:** **YOLOv11-seg** was used for **pixel-level segmentation** of the detected pallets.

### d. ROS Package for Detection and Segmentation
- A **ROS 2 package** was created to subscribe to an image topic, perform both detection and segmentation, and publish the results.

### e. ROS Package for Image Publishing
- Another **ROS 2 package** was developed to simulate camera input by publishing images from a folder to a ROS topic. This helps test the detection and segmentation pipeline in a simulated environment.

## Project File Structure
The project has the following structure:

```
/home/your_user/assignment/
    ├── images/                        # Folder containing images for testing
    │   ├── image_001.jpg              # Example image
    │   ├── image_002.jpg              # Example image
    │   └── ...                        # Other images in dataset
    ├── weights/                       # Folder containing model weights
    │   ├── detect_weight/             # Detection model weights
    │   │   └── best.pt                # YOLOv11 Detection model weights
    │   └── segment_weight/            # Segmentation model weights
    │       └── best.pt                # YOLOv11-seg model weights
    ├── ros_ws/                        # ROS workspace
    │   └── src/                       # Source folder for ROS packages
    │       ├── object_detection_pkg/  # Package for detection and segmentation
    │       └── image_publisher_pkg/   # Package for publishing images
    ├── requirements.txt               # Python dependencies for the project
    └── ...
```

## Running the Project on Your System

### a. Install the Required Dependencies
Ensure you have all necessary dependencies installed by running:
```bash
pip install -r requirements.txt
```

### b. Install ROS 2 Humble
Make sure **ROS 2 Humble** is installed on your system. You can follow the installation guide for ROS 2 Humble [here](https://docs.ros.org/en/humble/Installation.html).

### c. Organize Project Files
Save all the files in the `assignment` folder in your home directory. The directory should look like this:
```
/home/your_user/assignment/
    ├── images/              # Folder for images
    ├── weights/             # Folder for model weights
    ├── ros_ws/              # ROS 2 workspace
    ├── requirements.txt     # Project dependencies
    └── ...
```

### d. Download the Weights
- Download the **weights folder** from the provided [Google Drive link](https://drive.google.com/drive/folders/1aW8Ky6zVvp9q_QCgJUzxlyddnxsYijHu?usp=sharing) and place it in the `weights` directory under `assignment`.

### e. Build the ROS 2 Workspace
In the root of your ROS 2 workspace (`/home/your_user/assignment/ros_ws/`), run:
```bash
cd ~/assignment/ros_ws
colcon build
```

### f. Source the Workspace
After building the workspace, source it to make the packages available in the environment:
```bash
source ~/assignment/ros_ws/install/setup.bash
```

### g. Run the Object Detection and Segmentation Node
To run the object detection and segmentation node, use the following command:
```bash
ros2 run object_detection_pkg object_detection_node --ros-args -p weight_folder:="/home/your_user/assignment/weights"
```
Make sure to update the path to the `weights` folder.

### h. Run the Image Publishing Node (Optional)
If you want to simulate camera input by publishing images from a folder, use the following command:
```bash
ros2 run image_publisher_pkg image_publisher_node --ros-args -p image_folder:="/home/your_user/assignment/images" -p publish_rate:=1
```
Update the `image_folder` path and adjust the `publish_rate` as needed.

---

## Conclusion
This repository provides an end-to-end solution for **pallet detection and segmentation** using **YOLOv11** and **YOLOv11-seg**. With ROS 2 integration, it allows real-time detection and segmentation, making it suitable for automated pallet handling systems. The fine-tuned models and custom dataset ensure high accuracy in detecting and segmenting pallets.

---
