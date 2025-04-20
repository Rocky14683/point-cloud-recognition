import os

from dotenv import load_dotenv
from segments import SegmentsClient
import glob
from pathlib import Path


def create_dataset(dataset_name: str, client: SegmentsClient):
    description = "for pointcloud recognition pipeline"

    task_type = "pointcloud-cuboid"

    task_attributes = {
        "format_version": "0.1",
        "categories": [
            {
                "name": "UNKNOWN",
                "id": 0
            },
            {
                "name": "CAR",
                "id": 1
            },
            {
                "name": "PEDESTRIAN",
                "id": 2
            },
            {
                "name": "CYCLIST",
                "id": 3
            }
        ]
    }

    try:
        dataset = client.add_dataset(dataset_name, description, task_type, task_attributes)
        return dataset
    except Exception as e:
        print(e)
        return None


def upload_dataset(path: str, client: SegmentsClient) -> list[tuple[str, str]]:
    url_list: list[tuple[str, str]] = []  # [name, url]
    for filename in sorted(glob.glob(os.path.join(path, '*.bin'))):
        with open(filename, 'rb') as f:
            pure_filename = os.path.basename(filename)
            asset = client.upload_asset(f, pure_filename)
            url = asset.url
            print(f'adding {pure_filename}: {url}')
            url_list.append((pure_filename, url))

    return url_list


def load_sample_to_segments(url_list: list[tuple[str, str]], client: SegmentsClient) -> None:
    dataset_identifier = "Rocky/pointclouds"
    for filename, url in url_list:
        data_attribute = {
            "pcd": {
                "url": url,
                "type": "binary-xyzi"
            },
            "name": filename,
            "timestamp": Path(filename).stem,
            "ego_pose": {
                "position": {
                    "x": 0,
                    "y": 0,
                    "z": 0
                },
                "heading": {
                    "qx": 0,
                    "qy": 0,
                    "qz": 0,
                    "qw": 1
                }
            },
            "default_z": 0,
        }
        client.add_sample(dataset_identifier, filename, data_attribute)
        print(f'{filename} added to segments.ai')


def main():
    load_dotenv()
    client = SegmentsClient()

    create_dataset("pointclouds", client)
    url_list: list[tuple[str, str]] = upload_dataset("../../bin_datas/velodyne_points/data", client)
    load_sample_to_segments(url_list, client)


if __name__ == "__main__":
    main()
