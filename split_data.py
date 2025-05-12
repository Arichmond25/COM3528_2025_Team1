import os
import random
import shutil

# Paths to your dataset
images_path = "dataset/images"  # Path to images folder
labels_path = "dataset/labels"  # Path to labels folder

# Output folders
train_images_path = "dataset/images/train"
val_images_path = "dataset/images/val"
train_labels_path = "dataset/labels/train"
val_labels_path = "dataset/labels/val"

# Create output directories if they don't exist
os.makedirs(train_images_path, exist_ok=True)
os.makedirs(val_images_path, exist_ok=True)
os.makedirs(train_labels_path, exist_ok=True)
os.makedirs(val_labels_path, exist_ok=True)

# Get all image files
image_files = [f for f in os.listdir(images_path) if f.endswith(('.jpg', '.png', '.jpeg'))]

# Shuffle and split the dataset
random.seed(42)  # For reproducibility
random.shuffle(image_files)
split_index = int(0.8 * len(image_files))  # 80% for training, 20% for validation
train_files = image_files[:split_index]
val_files = image_files[split_index:]

# Function to move files
def move_files(file_list, src_images, src_labels, dest_images, dest_labels):
    for file in file_list:
        # Move image
        shutil.move(os.path.join(src_images, file), os.path.join(dest_images, file))
        
        # Move corresponding label
        label_file = file.replace('.jpg', '.txt').replace('.png', '.txt').replace('.jpeg', '.txt')
        if os.path.exists(os.path.join(src_labels, label_file)):
            shutil.move(os.path.join(src_labels, label_file), os.path.join(dest_labels, label_file))

# Move training files
move_files(train_files, images_path, labels_path, train_images_path, train_labels_path)

# Move validation files
move_files(val_files, images_path, labels_path, val_images_path, val_labels_path)

print("Dataset split completed!")