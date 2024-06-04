def calculate_mean_iou(filename):
    original_iou = []
    segmented_iou = []
    segmented2_iou = []

    with open(filename, 'r') as file:
        lines = file.readlines()
        for line in lines:
            parts = line.split(',')
            original = float(parts[0].split(': ')[1])
            segmented = float(parts[1].split(': ')[1])
            segmented2 = float(parts[2].split(': ')[1])
            
            original_iou.append(original)
            segmented_iou.append(segmented)
            segmented2_iou.append(segmented2)
    
    mean_original_iou = sum(original_iou) / len(original_iou)
    mean_segmented_iou = sum(segmented_iou) / len(segmented_iou)
    mean_segmented2_iou = sum(segmented2_iou) / len(segmented2_iou)
    
    return mean_original_iou, mean_segmented_iou, mean_segmented2_iou

# Usage example
filename = 'all_iou.txt'
mean_original, mean_segmented, mean_segmented2 = calculate_mean_iou(filename)
print(f'Mean Original IOU: {mean_original}')
print(f'Mean Segmented IOU: {mean_segmented}')
print(f'Mean Segmented2 IOU: {mean_segmented2}')
