import os
import cv2
import re
import numpy as np

def parse_filename(filename):
    """Разбор имени файла, чтобы получить количество автомобилей, временную метку и координаты."""
    match = re.match(r"(\d+)_(\d{2}):(\d{2}):(\d{2})\.(\d{3})_(.+)\.png", filename)
    if not match:
        raise ValueError(f"Filename '{filename}' does not match the expected format.")

    n = int(match.group(1))
    timestamp = f"{match.group(2)}:{match.group(3)}:{match.group(4)}.{match.group(5)}"
    coordinates = match.group(6).split("_")

    if len(coordinates) != n:
        raise ValueError(f"Filename '{filename}' indicates {n} vehicles, but only {len(coordinates)} coordinates found.")

    parsed_coordinates = [coord.strip("[]").split(":") for coord in coordinates]
    parsed_coordinates = [(float(lat), float(lon), float(angle)) for lat, lon, angle in parsed_coordinates]

    return n, timestamp, parsed_coordinates

def split_and_display_images(directory):
    """Считывает файлы из указанной директории, делит изображения и отображает их."""
    for filename in os.listdir(directory):
        if not filename.endswith(".png"):
            continue

        filepath = os.path.join(directory, filename)
        try:
            n, timestamp, coordinates = parse_filename(filename)
            print(f"Processing file: {filename}, Vehicles: {n}, Timestamp: {timestamp}")

            # Загрузка изображения
            image = cv2.imread(filepath)
            if image is None:
                raise ValueError(f"Unable to read image: {filepath}")

            # Определяем ширину каждой части изображения
            height, width, _ = image.shape
            part_width = width // n

            # Делим изображение и отображаем каждую часть
            for i in range(n):
                x_start = i * part_width
                x_end = (i + 1) * part_width
                vehicle_image = image[:, x_start:x_end]

                # Добавляем текст на изображение
                lat, lon, angle = coordinates[i]
                label = f"Time: {timestamp}\nLat: {lat:.6f}, Lon: {lon:.6f}, Angle: {angle:.6f}"
                for j, line in enumerate(label.split("\n")):
                    y = 30 + j * 30  # Позиция текста
                    cv2.putText(vehicle_image, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Отображение изображения
                cv2.imshow(f"Vehicle {i + 1}", vehicle_image)
                cv2.waitKey(0)

            # Ожидание клавиши для перехода к следующему изображению
            cv2.destroyAllWindows()

        except Exception as e:
            print(f"Error processing file '{filename}': {e}")

if __name__ == "__main__":
    directory = '/ulstu/repositories/webots_ros2_suv/data/'
    if os.path.isdir(directory):
        split_and_display_images(directory)
    else:
        print(f"Directory '{directory}' does not exist.")
