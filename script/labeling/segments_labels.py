import os

from dotenv import load_dotenv
from segments import SegmentsClient

dataset_identifier = "Rocky/pointclouds"
def get_label_sets(client: SegmentsClient, name: str):
    label_set = client.get_labelset(dataset_identifier, name)
    print(label_set)

def get_release(client: SegmentsClient):
    name = 'v0.0'
    release = client.get_release(dataset_identifier, name)
    print(release)


def main():
    load_dotenv()
    client = SegmentsClient()
    get_release(client)



if __name__ == "__main__":
    main()