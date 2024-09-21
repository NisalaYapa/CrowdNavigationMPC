import numpy as np
import cv2

def extract_obstacles_from_slam_map(map_image, threshold=127):
    """
    Extracts static obstacles from a SLAM map and returns them as line segments.
    
    Parameters:
    - map_image: The input map in grayscale where obstacles are represented (e.g., black for obstacles, white for free space).
    - threshold: Pixel intensity threshold to differentiate between obstacles and free space.
    
    Returns:
    - line_segments: A list of tuples representing the line segments in the form [((start_x, start_y), (end_x, end_y)), ...]
    """
    
    # Apply threshold to extract obstacle areas (assumes binary or grayscale map)
    _, binary_map = cv2.threshold(map_image, threshold, 255, cv2.THRESH_BINARY_INV)
    
    # Detect contours which represent boundaries of obstacles
    contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    line_segments = []
    
    # Loop through each detected contour and approximate line segments
    for contour in contours:
        # Approximate the contour with a polyline (line segments)
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx_poly = cv2.approxPolyDP(contour, epsilon, True)
        
        # Convert polyline into start and end points of each line segment
        for i in range(len(approx_poly)):
            start_point = tuple(approx_poly[i][0])
            end_point = tuple(approx_poly[(i + 1) % len(approx_poly)][0])  # Next point in the polyline
            line_segments.append((start_point, end_point))
    
    return line_segments


# Example usage:
if __name__ == "__main__":
    # Load SLAM map (grayscale image with obstacles in black and free space in white)
    map_image = cv2.imread("slam_map.png", cv2.IMREAD_GRAYSCALE)
    
    # Extract static obstacles as line segments
    obstacles = extract_obstacles_from_slam_map(map_image)
    
    # Print the line segments
    for line in obstacles:
        print(f"Line segment: Start {line[0]}, End {line[1]}")
