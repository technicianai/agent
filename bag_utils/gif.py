import argparse
import imageio
import cv2
import json
import numpy as np
import os
import pandas as pd
import sqlite3
import urllib.request

from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


parser = argparse.ArgumentParser()
parser.add_argument('uuid', type=str)
parser.add_argument('base_path', type=str)
parser.add_argument('urls', type=str)
args = parser.parse_args()

BRIDGE = CvBridge()

os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
DB_PATH = f'{args.base_path}/woeden/bags/{args.uuid}/{args.uuid}_0.db3'
GIF_BASE_PATH = f'{args.base_path}/woeden/gifs/{args.uuid}'

cnx = sqlite3.connect(DB_PATH)
df = pd.read_sql_query(
    '''
    SELECT name, type, timestamp, data 
    FROM messages AS m1
    INNER JOIN topics AS t ON m1.topic_id=t.id
    WHERE 
        (
            t.type='sensor_msgs/msg/Image' 
            OR t.type='sensor_msgs/msg/CompressedImage'
        )
        AND m1.id IN 
            (
                SELECT m2.id
                FROM (
                    SELECT ROW_NUMBER() OVER (ORDER BY timestamp) AS rownum, *
                    FROM messages
                ) m2 
                WHERE 
                    m1.topic_id = m2.topic_id
                    AND m2.rownum % 10 = 0
                ORDER BY m2.timestamp DESC LIMIT 10
            );
    ''',
    cnx
)

image_df = df[df['type'] == 'sensor_msgs/msg/Image']
compressed_df = df[df['type'] == 'sensor_msgs/msg/CompressedImage']

def ros_to_imageio(data, message_type):
    deserialized = deserialize_message(data, message_type)
    image = BRIDGE.imgmsg_to_cv2(deserialized, 'passthrough')

    info = np.iinfo(image.dtype)

    image = image / info.max
    image *= 255
    image = image.astype(np.uint8)

    png = cv2.imencode('.png', image)[1].tobytes()
    return imageio.v3.imread(png)

image_df['imageio'] = image_df['data'].apply(lambda data: ros_to_imageio(data, Image))
compressed_df['imageio'] = compressed_df['data'].apply(lambda data: ros_to_imageio(data, CompressedImage))
df = pd.concat([image_df, compressed_df])

urls_json = json.loads(args.urls)
urls_df = pd.DataFrame.from_dict(urls_json)

df = df[['name', 'imageio']].groupby('name').agg(list)
df = df.reset_index()

df = df.merge(urls_df, left_on='name', right_on='topic', how='inner')

def save_gif(row):
    topic = row['name']
    data = row['imageio']
    url = row['url']

    gif = imageio.v2.mimwrite('<bytes>', data, format='gif', duration=1)

    req = urllib.request.Request(url=url, data=gif, method='PUT')
    urllib.request.urlopen(req)

df.apply(save_gif, axis=1)
