def calculate_metrics_from_file(filename):
    segmented_precision = []
    segmented_accuracy = []
    segmented_recall = []
    segmented_f1 = []
    
    segmented2_precision = []
    segmented2_accuracy = []
    segmented2_recall = []
    segmented2_f1 = []
    
    with open(filename, 'r') as file:
        lines = file.readlines()
        for line in lines:
            data = line.split('Segmented2: ')[1].replace('}', '').replace('{', '').split(',')
            segmented_data = line.split('Segmented: ')[1].split('},')[0].replace('}', '').replace('{', '').split(',')

            segmented_metrics = {item.split(': ')[0].replace("'", "").strip(): int(item.split(': ')[1].strip()) for item in segmented_data}
            segmented2_metrics = {item.split(': ')[0].replace("'", "").strip(): int(item.split(': ')[1].strip()) for item in data}

            # Segmented metrics
            TP = segmented_metrics["TP"]
            FP = segmented_metrics["FP"]
            TN = segmented_metrics["TN"]
            FN = segmented_metrics["FN"]
            
            precision = TP / (TP + FP) if (TP + FP) != 0 else 0
            accuracy = (TP + TN) / (TP + FP + TN + FN)
            recall = TP / (TP + FN) if (TP + FN) != 0 else 0
            f1 = 2 * precision * recall / (precision + recall) if (precision + recall) != 0 else 0
            
            segmented_precision.append(precision)
            segmented_accuracy.append(accuracy)
            segmented_recall.append(recall)
            segmented_f1.append(f1)
            
            # Segmented2 metrics
            TP = segmented2_metrics['TP']
            FP = segmented2_metrics['FP']
            TN = segmented2_metrics['TN']
            FN = segmented2_metrics['FN']
            
            precision = TP / (TP + FP) if (TP + FP) != 0 else 0
            accuracy = (TP + TN) / (TP + FP + TN + FN)
            recall = TP / (TP + FN) if (TP + FN) != 0 else 0
            f1 = 2 * precision * recall / (precision + recall) if (precision + recall) != 0 else 0
            
            segmented2_precision.append(precision)
            segmented2_accuracy.append(accuracy)
            segmented2_recall.append(recall)
            segmented2_f1.append(f1)
    
    mean_segmented_precision = sum(segmented_precision) / len(segmented_precision)
    mean_segmented_accuracy = sum(segmented_accuracy) / len(segmented_accuracy)
    mean_segmented_recall = sum(segmented_recall) / len(segmented_recall)
    mean_segmented_f1 = sum(segmented_f1) / len(segmented_f1)
    
    mean_segmented2_precision = sum(segmented2_precision) / len(segmented2_precision)
    mean_segmented2_accuracy = sum(segmented2_accuracy) / len(segmented2_accuracy)
    mean_segmented2_recall = sum(segmented2_recall) / len(segmented2_recall)
    mean_segmented2_f1 = sum(segmented2_f1) / len(segmented2_f1)
    
    return (mean_segmented_precision, mean_segmented_accuracy, mean_segmented_recall, mean_segmented_f1,
            mean_segmented2_precision, mean_segmented2_accuracy, mean_segmented2_recall, mean_segmented2_f1)

# Usage example
filename = 'all_confusion.txt'
(mean_segmented_precision, mean_segmented_accuracy, mean_segmented_recall, mean_segmented_f1,
 mean_segmented2_precision, mean_segmented2_accuracy, mean_segmented2_recall, mean_segmented2_f1) = calculate_metrics_from_file(filename)

print(f'Mean Segmented Precision: {mean_segmented_precision}')
print(f'Mean Segmented Accuracy: {mean_segmented_accuracy}')
print(f'Mean Segmented Recall: {mean_segmented_recall}')
print(f'Mean Segmented F1 Score: {mean_segmented_f1}')

print(f'Mean Segmented2 Precision: {mean_segmented2_precision}')
print(f'Mean Segmented2 Accuracy: {mean_segmented2_accuracy}')
print(f'Mean Segmented2 Recall: {mean_segmented2_recall}')
print(f'Mean Segmented2 F1 Score: {mean_segmented2_f1}')
